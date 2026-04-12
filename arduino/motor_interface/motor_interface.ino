#include "motor_interface.h"
#include "encoder.h"

namespace {

constexpr uint8_t RAW_PWM_PACKET_SIZE = 4;
constexpr uint8_t VELOCITY_PACKET_SIZE = 7;
constexpr uint8_t VELOCITY_HEADER_0 = 0xA5;
constexpr uint8_t VELOCITY_HEADER_1 = 0x5A;
constexpr uint8_t TELEMETRY_HEADER_0 = 0x5A;
constexpr uint8_t TELEMETRY_HEADER_1 = 0xA5;

constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint32_t COMMAND_TIMEOUT_MS = 300;

constexpr int LEFT_MOTOR_SIGN = 1;
constexpr int RIGHT_MOTOR_SIGN = 1;
constexpr int LEFT_TPS_SIGN = 1;
constexpr int RIGHT_TPS_SIGN = 1;
constexpr float LEFT_TPS_TO_PWM = 0.80f;
constexpr float RIGHT_TPS_TO_PWM = 0.80f;
constexpr int LEFT_PWM_STATIC = 55;
constexpr int RIGHT_PWM_STATIC = 55;

enum CommandMode : uint8_t {
    COMMAND_NONE = 0,
    COMMAND_PWM = 1,
    COMMAND_WHEEL_VELOCITY = 2,
};

struct __attribute__((packed)) TelemetryPacket {
    uint8_t header0;
    uint8_t header1;
    int32_t left_speed;
    int32_t left_cnt;
    int32_t right_speed;
    int32_t right_cnt;
    uint8_t checksum;
};

CommandMode command_mode = COMMAND_NONE;
int raw_left_pwm = 0;
int raw_right_pwm = 0;
int16_t target_left_tps = 0;
int16_t target_right_tps = 0;

uint32_t last_command_millis = 0;
uint32_t last_control_millis = 0;
uint32_t last_telemetry_millis = 0;

uint8_t packet_checksum(const uint8_t* data, size_t size) {
    uint8_t sum = 0;
    for (size_t i = 0; i < size; ++i) {
        sum += data[i];
    }
    return sum;
}

int target_to_pwm(int16_t target_tps, int tps_sign, float tps_to_pwm, int pwm_static) {
    if (target_tps == 0) {
        return 0;
    }

    const int signed_target = tps_sign * static_cast<int>(target_tps);
    int pwm = static_cast<int>(abs(signed_target) * tps_to_pwm) + pwm_static;
    pwm = constrain(pwm, 0, 255);
    return (signed_target > 0) ? pwm : -pwm;
}

void stop_motion() {
    raw_left_pwm = 0;
    raw_right_pwm = 0;
    target_left_tps = 0;
    target_right_tps = 0;
    left_stop();
    right_stop();
}

bool read_velocity_packet() {
    if (Serial.available() < VELOCITY_PACKET_SIZE) {
        return false;
    }
    if (Serial.peek() != VELOCITY_HEADER_0) {
        return false;
    }

    uint8_t frame[VELOCITY_PACKET_SIZE];
    if (Serial.readBytes(reinterpret_cast<char*>(frame), VELOCITY_PACKET_SIZE) != VELOCITY_PACKET_SIZE) {
        return false;
    }
    if (frame[0] != VELOCITY_HEADER_0 || frame[1] != VELOCITY_HEADER_1) {
        return false;
    }
    if (packet_checksum(frame, VELOCITY_PACKET_SIZE - 1) != frame[VELOCITY_PACKET_SIZE - 1]) {
        return false;
    }

    target_left_tps = static_cast<int16_t>(frame[2] | (frame[3] << 8));
    target_right_tps = static_cast<int16_t>(frame[4] | (frame[5] << 8));
    command_mode = COMMAND_WHEEL_VELOCITY;
    last_command_millis = millis();
    return true;
}

void process_serial_commands() {
    while (Serial.available() > 0) {
        const int first = Serial.peek();
        if (first < 0) {
            return;
        }

        if (first == VELOCITY_HEADER_0) {
            if (Serial.available() < VELOCITY_PACKET_SIZE) {
                return;
            }
            if (!read_velocity_packet()) {
                Serial.read();
            }
            return;
        }

        if (Serial.available() >= RAW_PWM_PACKET_SIZE) {
            uint8_t raw[RAW_PWM_PACKET_SIZE];
            if (Serial.readBytes(reinterpret_cast<char*>(raw), RAW_PWM_PACKET_SIZE) != RAW_PWM_PACKET_SIZE) {
                return;
            }
            raw_left_pwm = static_cast<int>(raw[0]) - static_cast<int>(raw[1]);
            raw_right_pwm = static_cast<int>(raw[2]) - static_cast<int>(raw[3]);
            command_mode = COMMAND_PWM;
            last_command_millis = millis();
            return;
        }

        return;
    }
}

}  // namespace

void left_set_speed(int pwm) {
    pwm *= LEFT_MOTOR_SIGN;
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0) {
        analogWrite(LEFT_RPWM, pwm);
        analogWrite(LEFT_LPWM, 0);
    } else {
        analogWrite(LEFT_RPWM, 0);
        analogWrite(LEFT_LPWM, -pwm);
    }
}

void right_set_speed(int pwm) {
    pwm *= RIGHT_MOTOR_SIGN;
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0) {
        analogWrite(RIGHT_RPWM, pwm);
        analogWrite(RIGHT_LPWM, 0);
    } else {
        analogWrite(RIGHT_RPWM, 0);
        analogWrite(RIGHT_LPWM, -pwm);
    }
}

inline void left_stop() {
    left_set_speed(0);
}

inline void right_stop() {
    right_set_speed(0);
}

void setup() {
    pinMode(LEFT_RPWM, OUTPUT);
    pinMode(LEFT_LPWM, OUTPUT);
    pinMode(RIGHT_RPWM, OUTPUT);
    pinMode(RIGHT_LPWM, OUTPUT);
    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, 0);
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, 0);

    Serial.begin(115200);
    Serial.setTimeout(30);

    interrupts();
    enc_start();

    const uint32_t now = millis();
    last_command_millis = now;
    last_control_millis = now;
    last_telemetry_millis = now;
    stop_motion();
}

void loop() {
    process_serial_commands();

    left_enc.refresh();
    right_enc.refresh();

    const uint32_t now = millis();
    if ((uint32_t)(now - last_command_millis) > COMMAND_TIMEOUT_MS) {
        command_mode = COMMAND_NONE;
    }

    if ((uint32_t)(now - last_control_millis) >= CONTROL_PERIOD_MS) {
        last_control_millis = now;

        switch (command_mode) {
            case COMMAND_PWM:
                left_set_speed(raw_left_pwm);
                right_set_speed(raw_right_pwm);
                break;
            case COMMAND_WHEEL_VELOCITY:
                left_set_speed(target_to_pwm(target_left_tps, LEFT_TPS_SIGN, LEFT_TPS_TO_PWM, LEFT_PWM_STATIC));
                right_set_speed(target_to_pwm(target_right_tps, RIGHT_TPS_SIGN, RIGHT_TPS_TO_PWM, RIGHT_PWM_STATIC));
                break;
            case COMMAND_NONE:
            default:
                stop_motion();
                break;
        }
    }

    if ((uint32_t)(now - last_telemetry_millis) >= CONTROL_PERIOD_MS) {
        last_telemetry_millis = now;

        TelemetryPacket pkt{};
        pkt.header0 = TELEMETRY_HEADER_0;
        pkt.header1 = TELEMETRY_HEADER_1;
        pkt.left_speed = left_enc.speed();
        pkt.left_cnt = static_cast<int32_t>(left_enc.cnt());
        pkt.right_speed = right_enc.speed();
        pkt.right_cnt = static_cast<int32_t>(right_enc.cnt());
        pkt.checksum = packet_checksum(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - 1);
        Serial.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    }
}
