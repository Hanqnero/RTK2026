#pragma once

#include <stdint.h>

// Протокол обмена Raspberry Pi <-> Arduino Mega, версия 2.
//
// Кадр одинаков в обе стороны:
//
//   0xAA 0x55 | msg_id u8 | len u8 | payload[len] | crc16_lo | crc16_hi
//
// CRC16-CCITT (полином 0x1021, начальное значение 0xFFFF) считается по
// msg_id, len и payload. Sync-байты в CRC не входят: они нужны только для
// поиска границы кадра.
//
// Все многобайтовые поля little-endian. Это совпадает и с ATmega2560,
// и с x86/ARM хостом, поэтому конвертация порядка байт нигде не нужна.
//
// Зачем кадрирование, а не поток фиксированных структур: без границы кадра
// один потерянный байт сдвигал бы поток навсегда, и хост молча выдавал бы
// правдоподобные, но неверные числа. Рассинхронизация обнаруживается по CRC
// и считается в StatsPayload.
//
// Границы ответственности: прошивка отдаёт наружу величины в соглашении ROS
// (x вперёд, y влево, z вверх, положительная angular.z против часовой).
// Все аппаратные инверсии моторов и энкодеров живут в motor_interface.h
// и за пределы прошивки не выходят.

constexpr uint8_t kProtocolVersion = 2U;

constexpr uint8_t kFrameSync1 = 0xAAU;
constexpr uint8_t kFrameSync2 = 0x55U;

// sync(2) + msg_id(1) + len(1) + crc(2)
constexpr uint8_t kFrameOverheadBytes = 6U;

// Самое длинное входящее сообщение - VelocityCommandPayload (9 байт).
// Запас оставлен под SET_GAINS шага 3. Всё, что длиннее, отбрасывается
// как повреждённое, чтобы одна битая длина не съела RAM.
constexpr uint8_t kMaxInboundPayloadBytes = 32U;

// Хост -> MCU
constexpr uint8_t kMsgCmdVelocity = 0x01U;
constexpr uint8_t kMsgCmdWheelPwm = 0x02U;
constexpr uint8_t kMsgCmdWheelSetpoint = 0x03U;
constexpr uint8_t kMsgSetGains = 0x04U;
constexpr uint8_t kMsgSetConfig = 0x05U;         // зарезервировано
constexpr uint8_t kMsgCmdReset = 0x06U;
constexpr uint8_t kMsgSaveGains = 0x07U;
constexpr uint8_t kMsgGetGains = 0x08U;
constexpr uint8_t kMsgCmdAutotune = 0x09U;

// MCU -> хост
constexpr uint8_t kMsgTelemetry = 0x81U;
constexpr uint8_t kMsgPidDebug = 0x82U;
constexpr uint8_t kMsgStats = 0x83U;
constexpr uint8_t kMsgGainsReport = 0x84U;
constexpr uint8_t kMsgAutotuneStatus = 0x85U;
constexpr uint8_t kMsgSonarSample = 0x86U;

// Флаги VelocityCommandPayload.flags
// Действует для всех трёх команд движения: пока флаг взведён, прошивка
// добавляет к телеметрии кадр PidDebugPayload.
constexpr uint8_t kCommandFlagRequestPidDebug = 0x01U;

// Флаги ResetCommandPayload.mask
constexpr uint8_t kResetMaskOdometry = 0x01U;
constexpr uint8_t kResetMaskPid = 0x02U;
constexpr uint8_t kResetMaskStats = 0x04U;
// Вернуть коэффициенты к скомпилированным значениям и стереть запись
// в EEPROM. Нужно, чтобы выйти из заведомо неудачной настройки, не
// перепрошивая плату.
constexpr uint8_t kResetMaskGainsToDefault = 0x08U;

// Индексы колёс в SetGainsPayload и GainsReportPayload.
constexpr uint8_t kWheelLeft = 0U;
constexpr uint8_t kWheelRight = 1U;

// Режим управления, отражённый в TelemetryPayload.mode.
// Режим переключается той командой движения, которая пришла последней.
constexpr uint8_t kControlModeVelocity = 0U;      // cmd_vel -> кинематика -> ПИД
constexpr uint8_t kControlModeWheelSetpoint = 1U; // уставка колеса -> ПИД
constexpr uint8_t kControlModeWheelPwm = 2U;      // прямой PWM, ПИД не работает
// Релейный автотюнер PIDtuner держит PWM сам, регуляторы отключены. Режим
// снимается автоматически по достижении точности либо по таймауту.
constexpr uint8_t kControlModeAutotune = 3U;

// Источник действующих коэффициентов в GainsReportPayload.source.
constexpr uint8_t kGainsSourceCompiled = 0U;
constexpr uint8_t kGainsSourceEeprom = 1U;

// Флаги TelemetryPayload.flags
// Бит 0x01 свободен. Решение об остановке по сонару принимает Raspberry Pi,
// прошивка сама не тормозит, поэтому такого флага у неё нет.
constexpr uint8_t kTelemetryFlagCommandTimeout = 0x02U;
constexpr uint8_t kTelemetryFlagPwmSaturatedLeft = 0x04U;
constexpr uint8_t kTelemetryFlagPwmSaturatedRight = 0x08U;
constexpr uint8_t kTelemetryFlagCycleOverrun = 0x10U;

struct __attribute__((packed)) VelocityCommandPayload {
    float target_linear_mps;
    float target_angular_rps;
    uint8_t flags;
};  // 9 байт

struct __attribute__((packed)) ResetCommandPayload {
    uint8_t mask;
};  // 1 байт

// Прямая команда PWM в обход регуляторов и кинематики.
// Нужна для идентификации мотора: снять зависимость установившейся скорости
// от PWM можно только тогда, когда между ними нет обратной связи.
struct __attribute__((packed)) WheelPwmCommandPayload {
    int16_t left_pwm;
    int16_t right_pwm;
    uint8_t flags;
};  // 5 байт

// Уставка скорости колёс в обход кинематики корпуса.
// Позволяет крутить одно колесо при неподвижном втором и настраивать
// регуляторы по отдельности.
struct __attribute__((packed)) WheelSetpointCommandPayload {
    float left_rps;
    float right_rps;
    uint8_t flags;
};  // 9 байт

// Запуск релейного автотюнера GyverPID/PIDtuner на одном колесе.
//
// Тюнер сам держит PWM: раскачивает колесо вокруг steady_pwm ступенькой
// step_pwm, меряет период и амплитуду автоколебаний и выводит из них
// коэффициенты. Регуляторы на время автотюна не работают, поэтому режим
// требует поднятого робота ровно так же, как прямой PWM.
//
// Нулевой step_pwm останавливает автотюн и возвращает прошивку в режим
// уставки скорости с нулями.
struct __attribute__((packed)) AutotuneCommandPayload {
    uint8_t wheel;

    // Базовый PWM, вокруг которого идёт раскачка. Обязан быть выше PWM
    // страгивания: на колесе, стоящем в мёртвой зоне, автоколебаний не
    // возникнет и тюнер зависнет на этапе стабилизации.
    int16_t steady_pwm;

    // Амплитуда релейной ступеньки в обе стороны от steady_pwm.
    int16_t step_pwm;

    // Сколько ждать установившейся скорости перед раскачкой, мс.
    uint16_t wait_ms;

    // Порог скорости изменения, ниже которого система считается устоявшейся.
    float window_rps;

    // Длительность первого импульса раскачки, мс.
    uint16_t pulse_ms;

    // Точность автоколебаний в процентах, при которой автотюн завершается.
    uint8_t target_accuracy;

    // Период итерации тюнера, мс.
    //
    // Не совпадает с периодом управляющего цикла намеренно. Скоростной
    // контур мотора почти лишён запаздывания, и на 25 мс реле переключается
    // раньше, чем скорость успевает уйти от средней линии: амплитуда
    // автоколебаний выходит близкой к нулю, а Ku = 4*step/((max-min)*pi)
    // улетает в тысячи. Более редкий опрос даёт реле различимую амплитуду
    // и коэффициенты того порядка, который привод способен отработать.
    uint16_t tuner_period_ms;
};  // 16 байт

// Ход автотюна. Отправляется раз в kAutotuneStatusPeriodMs, пока режим
// активен, и один раз по завершении с финальными коэффициентами.
struct __attribute__((packed)) AutotuneStatusPayload {
    uint8_t wheel;

    // Этап работы PIDtuner: 1 - стабилизация, 2 - первый импульс,
    // 3 - анализ автоколебаний.
    uint8_t state;

    // Совпадение соседних периодов автоколебаний в процентах. Чем ближе
    // к 100, тем достовернее коэффициенты.
    uint8_t accuracy;

    // Ненулевой, когда автотюн завершён и коэффициенты окончательные.
    uint8_t done;

    float kp;
    float ki;
    float kd;
};  // 16 байт

struct __attribute__((packed)) SetGainsPayload {
    uint8_t wheel;
    float kp;
    float ki;
    float kd;
    float k_static;
    float k_velocity;
};  // 21 байт

struct __attribute__((packed)) GainsReportPayload {
    uint8_t wheel;
    float kp;
    float ki;
    float kd;
    float k_static;
    float k_velocity;
    uint8_t source;
};  // 22 байта

struct __attribute__((packed)) TelemetryPayload {
    // Растёт на единицу с каждым отправленным пакетом и переполняется по
    // модулю 2^16. Разрыв последовательности на хосте означает именно потерю
    // пакета, а не задержку: задержка сдвигает время прибытия, но не номер.
    uint16_t seq;

    // Время MCU на момент формирования пакета. Позволяет отделить джиттер
    // прошивки от джиттера USB и планировщика Linux.
    uint32_t mcu_time_ms;

    // Фактический интервал между этим и предыдущим управляющим циклом.
    // Именно он используется в расчёте скоростей, а не номинальный период.
    uint32_t dt_us;

    // Дельты энкодеров за цикл, ровно те, что пришли из прерываний.
    // Никакой фильтрации между измерением и этим полем нет.
    int16_t left_encoder_delta;
    int16_t right_encoder_delta;

    // Накопленные отсчёты с момента запуска прошивки.
    //
    // Дублируют дельты намеренно: сумма дельт на хосте разъезжается при
    // потере пакета, а накопленный счётчик к потерям невосприимчив.
    // Калибровка энкодера и маршрутные тесты опираются именно на него.
    int32_t left_encoder_total;
    int32_t right_encoder_total;

    float left_wheel_rps;
    float right_wheel_rps;

    // Действующие уставки колёс. Без них график настройки построить нельзя:
    // измеренную скорость не с чем сравнивать, а уставка колеса не выводится
    // однозначно из команды корпуса, когда режим не kControlModeVelocity.
    float left_setpoint_rps;
    float right_setpoint_rps;

    int16_t left_pwm;
    int16_t right_pwm;

    float odom_x_m;
    float odom_y_m;
    float odom_heading_rad;

    float current_linear_mps;
    float current_angular_rps;

    // Дистанция сонара, -1 при отсутствии эха в пределах таймаута.
    //
    // Прошивка только измеряет. Порог, гистерезис и само решение об остановке
    // живут на Raspberry Pi, где их видит и Nav2.
    int16_t sonar_distance_cm;

    uint8_t flags;

    // Одна из констант kControlMode*.
    uint8_t mode;
};  // 66 байт

// Внутренности регуляторов обоих колёс за один цикл.
//
// Отправляется только по флагу kCommandFlagRequestPidDebug: в штатной работе
// этот кадр не нужен, а при настройке без него невозможно понять, что именно
// упёрлось - пропорциональная часть, насыщенный интеграл или недооценённый
// feedforward.
//
// Дифференциальная составляющая отдельным полем не передаётся: она
// однозначно восстанавливается как pid_output - proportional - integral_term.
struct __attribute__((packed)) WheelDebugFields {
    float setpoint_rps;
    float measured_rps;
    float error_rps;
    float proportional;
    float integral_term;
    float feedforward;
    float pid_output;
    float output_pwm;
};  // 32 байта

struct __attribute__((packed)) PidDebugPayload {
    // Совпадает с seq телеметрии того же цикла, чтобы связать записи.
    uint16_t seq;
    WheelDebugFields left;
    WheelDebugFields right;
};  // 66 байт

struct __attribute__((packed)) StatsPayload {
    uint8_t protocol_version;

    uint32_t uptime_ms;
    uint32_t control_cycles;

    // Окно: значения считаются заново после каждой отправки StatsPayload.
    uint32_t dt_min_us;
    uint32_t dt_max_us;
    uint32_t dt_mean_us;
    uint32_t cycle_duration_max_us;
    uint32_t sonar_block_max_us;

    // Накопительные счётчики от старта прошивки.
    uint32_t tx_frames;
    uint32_t rx_frames;

    // Накопительные счётчики ошибок. Переполняются по модулю 2^16,
    // хост считает приращения с учётом переполнения.
    uint16_t tx_dropped;
    uint16_t rx_bad_crc;
    uint16_t rx_resync;
    uint16_t rx_bad_length;
    uint16_t overruns;

    uint16_t free_ram_bytes;
};  // 49 байт

// Один завершённый замер одного сонара. Кадр отдельный от
// TelemetryPayload: добавление шести датчиков не ломает протокол v2,
// а хост получает замер сразу после завершения эха.
struct __attribute__((packed)) SonarSamplePayload {
    // Общий счётчик кадров сонаров по модулю 2^16.
    uint16_t seq;
    // Время MCU на момент завершения замера.
    uint32_t mcu_time_ms;
    // Индек в массивах sonar_topics / sonar_frames на хосте.
    uint8_t sensor_index;
    // Миллиметры; -1 означает отсутствие эха до предела датчика.
    int16_t distance_mm;
};  // 9 байт

// Размеры зафиксированы: они же захардкожены в host-кодеках
// (protocol/rtk_link.py для стенда и src/rtk2026_driver/rtk2026_driver/
// protocol.py для ROS). Молчаливое расхождение здесь означало бы неверную
// интерпретацию телеметрии на хосте, поэтому расхождение должно ломать
// сборку, а не поведение робота.
static_assert(sizeof(VelocityCommandPayload) == 9, "VelocityCommandPayload изменился");
static_assert(sizeof(ResetCommandPayload) == 1, "ResetCommandPayload изменился");
static_assert(sizeof(WheelPwmCommandPayload) == 5, "WheelPwmCommandPayload изменился");
static_assert(sizeof(WheelSetpointCommandPayload) == 9, "WheelSetpointCommandPayload изменился");
static_assert(sizeof(SetGainsPayload) == 21, "SetGainsPayload изменился");
static_assert(sizeof(GainsReportPayload) == 22, "GainsReportPayload изменился");
static_assert(sizeof(TelemetryPayload) == 66, "TelemetryPayload изменился");
static_assert(sizeof(PidDebugPayload) == 66, "PidDebugPayload изменился");
static_assert(sizeof(StatsPayload) == 49, "StatsPayload изменился");
static_assert(sizeof(SonarSamplePayload) == 9, "SonarSamplePayload изменился");

// Самое длинное входящее сообщение обязано помещаться в буфер разборщика.
static_assert(sizeof(SetGainsPayload) <= kMaxInboundPayloadBytes,
              "SetGainsPayload не помещается в буфер приёма");
