#include "motor_interface.h"
#include "encoder.h"

namespace {

constexpr uint8_t VELOCITY_PACKET_SIZE = 7;
constexpr uint8_t VELOCITY_HEADER_0 = 0xA5;
constexpr uint8_t VELOCITY_HEADER_1 = 0x5A;
constexpr uint8_t TELEMETRY_HEADER_0 = 0x5A;
constexpr uint8_t TELEMETRY_HEADER_1 = 0xA5;

constexpr uint32_t CONTROL_PERIOD_MS = 100;
constexpr uint32_t COMMAND_TIMEOUT_MS = 300;

constexpr int LEFT_MOTOR_SIGN = 1;
constexpr int RIGHT_MOTOR_SIGN = 1;
constexpr int LEFT_ENCODER_SIGN = -1;
constexpr int RIGHT_ENCODER_SIGN = -1;

// Conservative feedforward retune from floor logs.
constexpr float LEFT_TPS_TO_PWM = 0.068f;
constexpr float RIGHT_TPS_TO_PWM = 0.067f;
constexpr int LEFT_PWM_STATIC = 22;
constexpr int RIGHT_PWM_STATIC = 22;

// PI correction on top of feedforward.
constexpr float PI_KP = 0.04f;
constexpr float PI_KI = 0.06f;
constexpr float I_LIMIT = 250.0f;
constexpr float DT_S = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;

struct __attribute__((packed)) TelemetryPacket {
    uint8_t header0;
    uint8_t header1;
    int32_t left_speed;
    int32_t left_cnt;
    int32_t right_speed;
    int32_t right_cnt;
    uint8_t checksum;
};

int16_t target_left_tps = 0;
int16_t target_right_tps = 0;
float left_i = 0.0f;
float right_i = 0.0f;

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

int feedforward_pwm(int16_t target_tps, float gain, int pwm_static) {
    if (target_tps == 0) {
        return 0;
    }

    float pwm = static_cast<float>(target_tps) * gain;
    pwm += (target_tps > 0) ? static_cast<float>(pwm_static) : -static_cast<float>(pwm_static);
    return static_cast<int>(constrain(pwm, -255.0f, 255.0f));
}

int wheel_control_pwm(
    int16_t target_tps,
    int32_t measured_tps,
    float& integral_state,
    float ff_gain,
    int ff_static)
{
    if (target_tps == 0) {
        integral_state = 0.0f;
        return 0;
    }

    const float ff = static_cast<float>(feedforward_pwm(target_tps, ff_gain, ff_static));
    const float error = static_cast<float>(target_tps) - static_cast<float>(measured_tps);

    // Simple anti-windup: freeze integrator if control is saturated and
    // error pushes deeper into saturation.
    const float i_candidate = constrain(integral_state + error * DT_S, -I_LIMIT, I_LIMIT);
    const float u_candidate = ff + PI_KP * error + PI_KI * i_candidate;
    const bool sat_high = (u_candidate > 255.0f) && (error > 0.0f);
    const bool sat_low = (u_candidate < -255.0f) && (error < 0.0f);
    if (!sat_high && !sat_low) {
        integral_state = i_candidate;
    }

    const float u = ff + PI_KP * error + PI_KI * integral_state;
    return static_cast<int>(constrain(u, -255.0f, 255.0f));
}

void stop_motion() {
    target_left_tps = 0;
    target_right_tps = 0;
    left_stop();
    right_stop();
}

void process_serial_commands() {
    while (Serial.available() > 0) {
        if (Serial.peek() != VELOCITY_HEADER_0) {
            Serial.read();
            continue;
        }

        if (Serial.available() < VELOCITY_PACKET_SIZE) {
            return;
        }

        uint8_t frame[VELOCITY_PACKET_SIZE];
        if (Serial.readBytes(reinterpret_cast<char*>(frame), VELOCITY_PACKET_SIZE) != VELOCITY_PACKET_SIZE) {
            return;
        }

        if (frame[0] != VELOCITY_HEADER_0 || frame[1] != VELOCITY_HEADER_1) {
            continue;
        }

        if (packet_checksum(frame, VELOCITY_PACKET_SIZE - 1) != frame[VELOCITY_PACKET_SIZE - 1]) {
            continue;
        }

        target_left_tps = static_cast<int16_t>(frame[2] | (frame[3] << 8));
        target_right_tps = static_cast<int16_t>(frame[4] | (frame[5] << 8));
        last_command_millis = millis();
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
        target_left_tps = 0;
        target_right_tps = 0;
    }

    if ((uint32_t)(now - last_control_millis) >= CONTROL_PERIOD_MS) {
        last_control_millis = now;
        const int32_t left_meas = LEFT_ENCODER_SIGN * left_enc.speed();
        const int32_t right_meas = RIGHT_ENCODER_SIGN * right_enc.speed();
        left_set_speed(wheel_control_pwm(target_left_tps, left_meas, left_i, LEFT_TPS_TO_PWM, LEFT_PWM_STATIC));
        right_set_speed(wheel_control_pwm(target_right_tps, right_meas, right_i, RIGHT_TPS_TO_PWM, RIGHT_PWM_STATIC));
    }

    if ((uint32_t)(now - last_telemetry_millis) >= CONTROL_PERIOD_MS) {
        last_telemetry_millis = now;

        TelemetryPacket pkt{};
        pkt.header0 = TELEMETRY_HEADER_0;
        pkt.header1 = TELEMETRY_HEADER_1;
        pkt.left_speed = LEFT_ENCODER_SIGN * left_enc.speed();
        pkt.left_cnt = LEFT_ENCODER_SIGN * static_cast<int32_t>(left_enc.cnt());
        pkt.right_speed = RIGHT_ENCODER_SIGN * right_enc.speed();
        pkt.right_cnt = RIGHT_ENCODER_SIGN * static_cast<int32_t>(right_enc.cnt());
        pkt.checksum = packet_checksum(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt) - 1);
        Serial.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    }
}
