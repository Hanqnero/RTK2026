#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <GyverMotor2.h>

#include "control_protocol.h"
#include "encoder.h"
#include "frame.h"
#include "gains_storage.h"
#include "math_utils.h"
#include "motor_interface.h"
#include "profiling.h"
#include "sonar.h"
#include "wheel_controller.h"

namespace {

GyverMotor2<GM2::PWM_PWM_SPEED> left_motor(LEFT_PWM_A, LEFT_PWM_B);
GyverMotor2<GM2::PWM_PWM_SPEED> right_motor(RIGHT_PWM_A, RIGHT_PWM_B);

// Инверсия направления задаётся флагами из motor_interface.h и больше нигде.
EncoderCounter left_encoder(LEFT_ENC_CLK, LEFT_ENC_DT, kLeftEncoderReverse, INPUT_PULLUP);
EncoderCounter right_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT, kRightEncoderReverse, INPUT_PULLUP);

// Два независимых контура. Именно они делают осмысленной раздельную
// настройку моторов: у каждого колеса свои коэффициенты и своё состояние.
WheelController left_controller;
WheelController right_controller;
uint8_t gains_source = kGainsSourceCompiled;

TelemetryPayload telemetry = {};
FrameParser rx_parser;
LoopProfiler profiler;

// Самые длинные исходящие кадры - TelemetryPayload и PidDebugPayload,
// по 66 байт нагрузки плюс шесть служебных.
uint8_t tx_buffer[80];

uint16_t telemetry_seq = 0;
uint32_t tx_frames = 0;
uint16_t tx_dropped = 0;

// Действующая уставка. В режиме kControlModeWheelPwm вместо неё используется
// прямая команда PWM, а регуляторы удерживаются в сброшенном состоянии.
uint8_t control_mode = kControlModeVelocity;
float left_setpoint_rps = 0.0f;
float right_setpoint_rps = 0.0f;
int16_t direct_left_pwm = 0;
int16_t direct_right_pwm = 0;
bool pid_debug_requested = false;

uint32_t last_control_us = 0;  // плановое время следующего цикла
uint32_t last_cycle_us = 0;    // фактическое время предыдущего цикла
uint32_t last_command_ms = 0;  // время получения последней команды
uint32_t last_stats_ms = 0;    // время последней отправки статистики
bool command_timed_out = false;

float odom_x_m = 0.0f;
float odom_y_m = 0.0f;
float odom_heading_rad = 0.0f;
bool status_led_state = false;

void leftEncoderISR() {
    left_encoder.handleInterrupt();
}

void rightEncoderISR() {
    right_encoder.handleInterrupt();
}

// Дельты энкодеров передаются как int16. При 960 об/мин это 16 оборотов
// колеса в секунду, то есть около 3700 отсчётов в секунду и порядка 90
// за цикл - запас в сотни раз. Насыщение оставлено на случай выбросов
// при дребезге.
int16_t saturateToInt16(int32_t value) {
    if (value > 32767L) {
        return 32767;
    }
    if (value < -32768L) {
        return -32768;
    }
    return static_cast<int16_t>(value);
}

// Обороты колеса в секунду по фактическому интервалу.
// Берётся фактический интервал, а не номинальный период: по номинальному
// скорость была бы смещена ровно на величину джиттера цикла.
float countsToWheelRps(int32_t counts, uint32_t dt_us) {
    if (dt_us == 0) {
        return 0.0f;
    }
    const float dt_s = static_cast<float>(dt_us) * 1.0e-6f;
    return (static_cast<float>(counts) / kEncoderCountsPerWheelRev) / dt_s;
}

// Редуктор учтён в kEncoderCountsPerWheelRev, поэтому здесь уже обороты
// самого колеса и остаётся умножить на длину его окружности.
float wheelRpsToLinearMps(float wheel_rps) {
    return wheel_rps * kWheelCircumferenceM;
}

float linearMpsToWheelRps(float linear_mps) {
    return linear_mps / kWheelCircumferenceM;
}

// Нормализация угла в [-pi, pi].
// TODO: для обычной одометрии цикл выполняется максимум один раз, но при
// большом входном значении итераций может быть много. Альтернатива - fmodf.
float wrapAngleRad(float angle) {
    while (angle > kPi) {
        angle -= 2.0f * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0f * kPi;
    }
    return angle;
}

// Обратная кинематика дифференциального привода в соглашении ROS:
// положительная угловая скорость означает вращение против часовой стрелки,
// то есть правое колесо быстрее левого.
void bodyToWheelSetpoints(float linear_mps, float angular_rps) {
    const float half_track = 0.5f * kTrackWidthM;

    const float left_mps = linear_mps - angular_rps * half_track;
    const float right_mps = linear_mps + angular_rps * half_track;

    left_setpoint_rps = clampFloat(
        linearMpsToWheelRps(left_mps), -kMaxWheelSetpointRps, kMaxWheelSetpointRps);
    right_setpoint_rps = clampFloat(
        linearMpsToWheelRps(right_mps), -kMaxWheelSetpointRps, kMaxWheelSetpointRps);
}

void applyGains(const StoredGains& gains) {
    left_controller.setGains(gains.left);
    right_controller.setGains(gains.right);
}

StoredGains compiledGains() {
    StoredGains gains;

    gains.left = WheelGains{
        kLeftWheelKp, kLeftWheelKi, kLeftWheelKd,
        kLeftWheelKStatic, kLeftWheelKVelocity};
    gains.right = WheelGains{
        kRightWheelKp, kRightWheelKi, kRightWheelKd,
        kRightWheelKStatic, kRightWheelKVelocity};

    return gains;
}

StoredGains currentGains() {
    StoredGains gains;
    gains.left = left_controller.gains();
    gains.right = right_controller.gains();
    return gains;
}

void loadGainsAtStartup() {
    StoredGains gains = compiledGains();

    // loadGains меняет структуру только при успехе, поэтому скомпилированные
    // значения остаются как запасной вариант при пустой или битой EEPROM.
    gains_source = (loadGains(&gains) == GainsLoadResult::Ok)
                       ? kGainsSourceEeprom
                       : kGainsSourceCompiled;

    applyGains(gains);
}

void stopMotorsAndResetControllers() {
    left_motor.runSpeed(0);
    right_motor.runSpeed(0);

    left_controller.reset();
    right_controller.reset();
}

void configureHardware() {
    left_motor.setReverse(kLeftMotorReverse);
    right_motor.setReverse(kRightMotorReverse);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    configureSonar();

    left_encoder.begin();
    right_encoder.begin();

    // digitalPinToInterrupt переводит номер вывода в номер внешнего прерывания.
    // Оба канала каждого энкодера обрабатываются одним ISR по фронту и спаду.
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_CLK), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_DT), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_CLK), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_DT), rightEncoderISR, CHANGE);
}

void emitFrame(uint8_t message_id, const void* payload, uint8_t payload_length) {
    const uint8_t frame_length =
        buildFrame(tx_buffer, sizeof(tx_buffer), message_id, payload, payload_length);

    if (frame_length == 0) {
        return;
    }

    // Кадр отправляется только целиком. Частичная запись сдвинула бы поток,
    // а именно от этого протокол v2 и должен защищать, поэтому при нехватке
    // места в буфере пакет теряется полностью и попадает в счётчик.
    if (Serial.availableForWrite() < static_cast<int>(frame_length)) {
        ++tx_dropped;
        return;
    }

    Serial.write(tx_buffer, frame_length);
    ++tx_frames;
}

void fillWheelDebug(WheelDebugFields* fields, const WheelControllerTerms& terms) {
    fields->setpoint_rps = terms.setpoint_rps;
    fields->measured_rps = terms.measured_rps;
    fields->error_rps = terms.error_rps;
    fields->proportional = terms.proportional;
    fields->integral_term = terms.integral_term;
    fields->feedforward = terms.feedforward;
    fields->pid_output = terms.pid_output;
    fields->output_pwm = terms.output_pwm;
}

void sendPidDebug() {
    PidDebugPayload debug;

    debug.seq = telemetry.seq;
    fillWheelDebug(&debug.left, left_controller.terms());
    fillWheelDebug(&debug.right, right_controller.terms());

    emitFrame(kMsgPidDebug, &debug, sizeof(PidDebugPayload));
}

void sendTelemetry() {
    telemetry.seq = telemetry_seq;
    ++telemetry_seq;
    telemetry.mcu_time_ms = millis();

    emitFrame(kMsgTelemetry, &telemetry, sizeof(TelemetryPayload));

    if (pid_debug_requested) {
        sendPidDebug();
    }
}

void sendGainsReport(uint8_t wheel) {
    const WheelGains& gains =
        (wheel == kWheelRight) ? right_controller.gains() : left_controller.gains();

    GainsReportPayload report;

    report.wheel = wheel;
    report.kp = gains.kp;
    report.ki = gains.ki;
    report.kd = gains.kd;
    report.k_static = gains.k_static;
    report.k_velocity = gains.k_velocity;
    report.source = gains_source;

    emitFrame(kMsgGainsReport, &report, sizeof(GainsReportPayload));
}

void sendStats() {
    StatsPayload stats = {};

    stats.protocol_version = kProtocolVersion;
    stats.uptime_ms = millis();
    stats.control_cycles = profiler.cycleCount();

    stats.dt_min_us = profiler.windowMinDtUs();
    stats.dt_max_us = profiler.windowMaxDtUs();
    stats.dt_mean_us = profiler.windowMeanDtUs();
    stats.cycle_duration_max_us = profiler.windowMaxCycleDurationUs();
    stats.sonar_block_max_us = profiler.windowMaxSonarBlockUs();

    stats.tx_frames = tx_frames;
    stats.rx_frames = rx_parser.frameCount();

    stats.tx_dropped = tx_dropped;
    stats.rx_bad_crc = rx_parser.badCrcCount();
    stats.rx_resync = rx_parser.resyncCount();
    stats.rx_bad_length = rx_parser.badLengthCount();
    stats.overruns = profiler.overrunCount();

    stats.free_ram_bytes = freeRamBytes();

    emitFrame(kMsgStats, &stats, sizeof(StatsPayload));

    profiler.resetWindow();
}

void maybeSendStats() {
    const uint32_t now = millis();
    if (now - last_stats_ms < kStatsPeriodMs) {
        return;
    }
    last_stats_ms = now;
    sendStats();
}

// Общая часть всех трёх команд движения: сброс dead-man и флага отладки.
void acceptMotionCommand(uint8_t flags) {
    pid_debug_requested = (flags & kCommandFlagRequestPidDebug) != 0;
    last_command_ms = millis();
}

void handleVelocityCommand(const uint8_t* payload, uint8_t length) {
    if (length != sizeof(VelocityCommandPayload)) {
        return;
    }

    VelocityCommandPayload received;
    memcpy(&received, payload, sizeof(VelocityCommandPayload));

    // Кадр прошёл CRC, но содержимое всё равно проверяем: NaN в уставке
    // прошёл бы дальше и отравил и регулятор, и одометрию.
    if (isnan(received.target_linear_mps) || isnan(received.target_angular_rps)) {
        return;
    }

    const float linear_mps = clampFloat(
        received.target_linear_mps, -kMaxLinearCommandMps, kMaxLinearCommandMps);
    const float angular_rps = clampFloat(
        received.target_angular_rps, -kMaxAngularCommandRadS, kMaxAngularCommandRadS);

    control_mode = kControlModeVelocity;
    bodyToWheelSetpoints(linear_mps, angular_rps);
    acceptMotionCommand(received.flags);
}

void handleWheelSetpointCommand(const uint8_t* payload, uint8_t length) {
    if (length != sizeof(WheelSetpointCommandPayload)) {
        return;
    }

    WheelSetpointCommandPayload received;
    memcpy(&received, payload, sizeof(WheelSetpointCommandPayload));

    if (isnan(received.left_rps) || isnan(received.right_rps)) {
        return;
    }

    control_mode = kControlModeWheelSetpoint;
    left_setpoint_rps = clampFloat(
        received.left_rps, -kMaxWheelSetpointRps, kMaxWheelSetpointRps);
    right_setpoint_rps = clampFloat(
        received.right_rps, -kMaxWheelSetpointRps, kMaxWheelSetpointRps);
    acceptMotionCommand(received.flags);
}

void handleWheelPwmCommand(const uint8_t* payload, uint8_t length) {
    if (length != sizeof(WheelPwmCommandPayload)) {
        return;
    }

    WheelPwmCommandPayload received;
    memcpy(&received, payload, sizeof(WheelPwmCommandPayload));

    control_mode = kControlModeWheelPwm;
    direct_left_pwm = static_cast<int16_t>(clampFloat(
        static_cast<float>(received.left_pwm), -kMaxPwmCommand, kMaxPwmCommand));
    direct_right_pwm = static_cast<int16_t>(clampFloat(
        static_cast<float>(received.right_pwm), -kMaxPwmCommand, kMaxPwmCommand));

    // Уставки обнуляются, чтобы телеметрия не показывала цель, которой
    // в этом режиме никто не добивается.
    left_setpoint_rps = 0.0f;
    right_setpoint_rps = 0.0f;

    acceptMotionCommand(received.flags);
}

void handleSetGains(const uint8_t* payload, uint8_t length) {
    if (length != sizeof(SetGainsPayload)) {
        return;
    }

    SetGainsPayload received;
    memcpy(&received, payload, sizeof(SetGainsPayload));

    if (received.wheel != kWheelLeft && received.wheel != kWheelRight) {
        return;
    }

    // NaN в коэффициенте отравил бы регулятор необратимо: интеграл стал бы
    // NaN и остался таким до сброса.
    if (isnan(received.kp) || isnan(received.ki) || isnan(received.kd) ||
        isnan(received.k_static) || isnan(received.k_velocity)) {
        return;
    }

    const WheelGains gains{
        received.kp, received.ki, received.kd,
        received.k_static, received.k_velocity};

    if (received.wheel == kWheelLeft) {
        left_controller.setGains(gains);
    } else {
        right_controller.setGains(gains);
    }

    // Новые коэффициенты действуют с чистого состояния: интеграл, накопленный
    // при прежних коэффициентах, к новым отношения не имеет.
    if (received.wheel == kWheelLeft) {
        left_controller.reset();
    } else {
        right_controller.reset();
    }

    sendGainsReport(received.wheel);
}

void handleSaveGains() {
    if (saveGains(currentGains())) {
        gains_source = kGainsSourceEeprom;
    }

    // Отчёт уходит в любом случае: по полю source оператор видит,
    // удалась запись или коэффициенты остались только в оперативной памяти.
    sendGainsReport(kWheelLeft);
    sendGainsReport(kWheelRight);
}

void handleGetGains() {
    sendGainsReport(kWheelLeft);
    sendGainsReport(kWheelRight);
}

void handleResetCommand(const uint8_t* payload, uint8_t length) {
    if (length != sizeof(ResetCommandPayload)) {
        return;
    }

    const uint8_t mask = payload[0];

    if (mask & kResetMaskOdometry) {
        odom_x_m = 0.0f;
        odom_y_m = 0.0f;
        odom_heading_rad = 0.0f;
    }

    if (mask & kResetMaskPid) {
        left_controller.reset();
        right_controller.reset();
    }

    if (mask & kResetMaskStats) {
        profiler.resetAll();
        rx_parser.resetCounters();
        tx_frames = 0;
        tx_dropped = 0;
    }

    if (mask & kResetMaskGainsToDefault) {
        clearStoredGains();
        applyGains(compiledGains());
        gains_source = kGainsSourceCompiled;
        left_controller.reset();
        right_controller.reset();
        handleGetGains();
    }
}

void readIncomingFrames() {
    // Ограничение на итерацию не даёт потоку команд заблокировать loop(),
    // если хост по ошибке начнёт слать быстрее, чем мы успеваем разбирать.
    uint8_t budget = 128;

    while (Serial.available() > 0 && budget > 0) {
        --budget;

        if (!rx_parser.feed(static_cast<uint8_t>(Serial.read()))) {
            continue;
        }

        switch (rx_parser.messageId()) {
            case kMsgCmdVelocity:
                handleVelocityCommand(rx_parser.payload(), rx_parser.payloadLength());
                break;

            case kMsgCmdWheelSetpoint:
                handleWheelSetpointCommand(rx_parser.payload(), rx_parser.payloadLength());
                break;

            case kMsgCmdWheelPwm:
                handleWheelPwmCommand(rx_parser.payload(), rx_parser.payloadLength());
                break;

            case kMsgSetGains:
                handleSetGains(rx_parser.payload(), rx_parser.payloadLength());
                break;

            case kMsgSaveGains:
                handleSaveGains();
                break;

            case kMsgGetGains:
                handleGetGains();
                break;

            case kMsgCmdReset:
                handleResetCommand(rx_parser.payload(), rx_parser.payloadLength());
                break;

            default:
                // Кадр разобран корректно, но идентификатор неизвестен.
                // В счётчик ошибок это не попадает: так добавление новых
                // сообщений не выглядит как повреждение потока.
                break;
        }
    }
}

// После переноса аварийного стопа по сонару на Raspberry Pi этот таймаут
// останется единственной защитой на MCU, поэтому он обязан быть жёстким:
// обнуления уставки мало, регулятор продолжал бы тормозить плавно и на
// насыщенном интеграле.
void applyCommandTimeout() {
    const bool timed_out = (millis() - last_command_ms > kCommandTimeoutMs);

    if (timed_out && !command_timed_out) {
        left_setpoint_rps = 0.0f;
        right_setpoint_rps = 0.0f;
        direct_left_pwm = 0;
        direct_right_pwm = 0;
        stopMotorsAndResetControllers();
    }

    command_timed_out = timed_out;
}

void runControlCycle(uint32_t dt_us, bool overrun) {
    const uint32_t cycle_start_us = micros();

    // Дельты берутся как есть. Фильтрации между измерением и управлением нет:
    // подмена дельт средним скрывала именно ту разницу между колёсами,
    // ради которой всё это и измеряется.
    const int32_t left_delta = left_encoder.readAndResetDelta();
    const int32_t right_delta = right_encoder.readAndResetDelta();

    const float dt_s = static_cast<float>(dt_us) * 1.0e-6f;

    const float left_wheel_rps = countsToWheelRps(left_delta, dt_us);
    const float right_wheel_rps = countsToWheelRps(right_delta, dt_us);

    const float left_wheel_mps = wheelRpsToLinearMps(left_wheel_rps);
    const float right_wheel_mps = wheelRpsToLinearMps(right_wheel_rps);

    // Прямая кинематика в соглашении ROS: положительная угловая скорость -
    // вращение против часовой стрелки, то есть правое колесо быстрее.
    const float current_linear_mps = 0.5f * (left_wheel_mps + right_wheel_mps);
    const float current_angular_rps = (right_wheel_mps - left_wheel_mps) / kTrackWidthM;

    // Интегрирование по средней ориентации точнее чистого Эйлера на дуге.
    // TODO: проверять результат на NaN. Уставки от NaN защищены при приёме,
    // но одометрия считается из дельт и dt, и одно NaN здесь осталось бы
    // в накопленной позе навсегда.
    const float delta_heading = current_angular_rps * dt_s;
    const float heading_mid = odom_heading_rad + 0.5f * delta_heading;
    const float delta_s = current_linear_mps * dt_s;
    odom_x_m += delta_s * cosf(heading_mid);
    odom_y_m += delta_s * sinf(heading_mid);
    odom_heading_rad = wrapAngleRad(odom_heading_rad + delta_heading);

    uint8_t flags = 0;
    if (overrun) {
        flags |= kTelemetryFlagCycleOverrun;
    }
    if (command_timed_out) {
        flags |= kTelemetryFlagCommandTimeout;
    }

    // Сонар только измеряется. Решение об остановке принимает Raspberry Pi:
    // там же работает Nav2, который обязан видеть препятствие как часть общей
    // картины, а не узнавать о нём по внезапно остановившемуся роботу.
    profiler.recordSonarBlock(sonarTakeLastBlockUs());

    int16_t left_pwm_out = 0;
    int16_t right_pwm_out = 0;

    if (command_timed_out) {
        left_setpoint_rps = 0.0f;
        right_setpoint_rps = 0.0f;

        // Регуляторы сбрасываются вместе с остановкой: иначе интеграл,
        // накопленный до таймаута, дал бы рывок при возобновлении движения.
        stopMotorsAndResetControllers();
    } else if (control_mode == kControlModeWheelPwm) {
        // Прямой PWM в обход регуляторов. Режим существует ради идентификации
        // мотора: снять зависимость скорости от PWM можно, только разорвав
        // обратную связь.
        left_controller.reset();
        right_controller.reset();

        left_pwm_out = direct_left_pwm;
        right_pwm_out = direct_right_pwm;

        left_motor.runSpeed(left_pwm_out);
        right_motor.runSpeed(right_pwm_out);
    } else {
        const float left_output =
            left_controller.update(left_setpoint_rps, left_wheel_rps, dt_s);
        const float right_output =
            right_controller.update(right_setpoint_rps, right_wheel_rps, dt_s);

        // Насыщение фиксируется по выходу регулятора до округления: для
        // настройки важно отличать "ровно на пределе" от "запрошено втрое
        // больше, чем мотор способен выдать".
        if (fabsf(left_output) >= kMaxPwmCommand) {
            flags |= kTelemetryFlagPwmSaturatedLeft;
        }
        if (fabsf(right_output) >= kMaxPwmCommand) {
            flags |= kTelemetryFlagPwmSaturatedRight;
        }

        left_pwm_out = static_cast<int16_t>(left_output);
        right_pwm_out = static_cast<int16_t>(right_output);

        // TODO: убедиться, что диапазон PWM GyverMotor2 совпадает с [-255, 255].
        left_motor.runSpeed(left_pwm_out);
        right_motor.runSpeed(right_pwm_out);

        status_led_state = !status_led_state;
        digitalWrite(LED_BUILTIN, status_led_state ? HIGH : LOW);
    }

    telemetry.dt_us = dt_us;
    telemetry.left_encoder_delta = saturateToInt16(left_delta);
    telemetry.right_encoder_delta = saturateToInt16(right_delta);
    telemetry.left_encoder_total = left_encoder.readCount();
    telemetry.right_encoder_total = right_encoder.readCount();
    telemetry.left_wheel_rps = left_wheel_rps;
    telemetry.right_wheel_rps = right_wheel_rps;
    telemetry.left_setpoint_rps = left_setpoint_rps;
    telemetry.right_setpoint_rps = right_setpoint_rps;
    telemetry.left_pwm = left_pwm_out;
    telemetry.right_pwm = right_pwm_out;
    telemetry.odom_x_m = odom_x_m;
    telemetry.odom_y_m = odom_y_m;
    telemetry.odom_heading_rad = odom_heading_rad;
    telemetry.current_linear_mps = current_linear_mps;
    telemetry.current_angular_rps = current_angular_rps;
    telemetry.sonar_distance_cm =
        saturateToInt16(static_cast<int32_t>(lroundf(sonarLastDistanceCm())));
    telemetry.flags = flags;
    telemetry.mode = control_mode;

    // Длительность меряется до отправки: интерес представляет стоимость
    // расчёта, а не время выталкивания байт в UART.
    profiler.recordCycle(dt_us, micros() - cycle_start_us);
    if (overrun) {
        profiler.recordOverrun();
    }

    sendTelemetry();
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);

    configureHardware();
    loadGainsAtStartup();

    left_motor.runSpeed(0);
    right_motor.runSpeed(0);

    const uint32_t now_us = micros();
    last_control_us = now_us;
    last_cycle_us = now_us;
    last_command_ms = millis();
    last_stats_ms = last_command_ms;
}

void loop() {
    readIncomingFrames();
    applyCommandTimeout();

    // Автомат сонара продвигается на каждой итерации: между управляющими
    // циклами их тысячи, и этого хватает для разрешения в единицы сантиметров
    // без единой блокирующей паузы.
    updateSonar();

    const uint32_t now_us = micros();

    if (now_us - last_control_us < kControlPeriodUs) {
        maybeSendStats();
        return;
    }

    // Плановое время сдвигаем на номинальный период, чтобы не накапливать
    // дрейф фазы. Но если отстали больше чем на период, догонять пачкой
    // вызовов подряд бессмысленно: синхронизируемся с реальным временем
    // и отмечаем срыв.
    last_control_us += kControlPeriodUs;
    bool overrun = false;

    if (static_cast<int32_t>(now_us - last_control_us) >
        static_cast<int32_t>(kControlPeriodUs)) {
        last_control_us = now_us;
        overrun = true;
    }

    // Фактический интервал между считываниями энкодеров. Именно он идёт
    // в расчёт скоростей, в одометрию и в регуляторы.
    const uint32_t dt_us = now_us - last_cycle_us;
    last_cycle_us = now_us;

    if (dt_us > kCycleOverrunThresholdUs) {
        overrun = true;
    }

    runControlCycle(dt_us, overrun);
    maybeSendStats();
}
