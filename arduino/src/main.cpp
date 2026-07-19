#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include <GyverMotor2.h>
#include <uPID.h>

#include "control_protocol.h"
#include "encoder.h"
#include "motor_interface.h"
#include "sonar.h"

// & сразу выпишу главное по синтаксису плюсов:
// & constexpr - вычисляем на этапе компиляции данную константу (сразу заметим, что константы имееют приписку k)
// & fabsf - абс значение float
namespace {

constexpr float kWheelCircumferenceM = 2.0F * kPi * kWheelRadiusM;  // вычисляем радиус окружности
constexpr float kControlPeriodS = static_cast<float>(kControlPeriodMs) / 1000.0f; // переводим период управления из мс в с

GyverMotor2<GM2::PWM_PWM_SPEED> left_motor(LEFT_PWM_A, LEFT_PWM_B); //создаем левый мотор, названия аргументов понятны
GyverMotor2<GM2::PWM_PWM_SPEED> right_motor(RIGHT_PWM_A, RIGHT_PWM_B); //создаем правый мотор, названия аргументов понятны

EncoderCounter left_encoder(LEFT_ENC_CLK, LEFT_ENC_DT, kLeftEncoderReverse, INPUT_PULLUP); //создаем левый энкодер (LEFT_ENC_CLK, LEFT_ENC_DT - первые и вторые каналы энкодера соотв, kLeftEncoderReverse - флаг инверсии направления,INPUT_PULLUP - режим входа со встроенным подтягивающим резистором)
EncoderCounter right_encoder(RIGHT_ENC_CLK, RIGHT_ENC_DT, kRightEncoderReverse, INPUT_PULLUP);//создаем левый энкодер (LEFT_ENC_CLK, LEFT_ENC_DT - первые и вторые каналы энкодера соотв, kLeftEncoderReverse - флаг инверсии направления,INPUT_PULLUP - режим входа со встроенным подтягивающим резистором)

uPID linear_pid( D_ERROR, kControlPeriodMs); // создаем пид регулятор линейного движения (D_ERROR - режим вычисления ошибки, kControlPeriodMs - период вызова регулятора в мс)
uPID angular_pid( D_ERROR | I_KI_INSIDE, kControlPeriodMs); // пид регулятор углового движения (? I_KI_INSIDE)

ControlPacket command_packet = {0.0F, 0.0F, 0U}; //
TelemetryPacket telemetry_packet = {}; 
bool debug_raw_encoder = false; 

uint32_t last_control_ms = 0; // время последнего упр цикла
uint32_t last_command_ms = 0; // время получения посл упр команды

float odom_x_m = 0.0f;  // одометрия по иск (в метрах)
float odom_y_m = 0.0f;  // одометрия по игрик
float odom_heading_rad = 0.0f; // одометрия по радианам
bool status_led_state = false; // состояние встроенного светодиода

void leftEncoderISR() {
    left_encoder.handleInterrupt();
} // прерывание лев энкодера

void rightEncoderISR() {
    right_encoder.handleInterrupt();
} // прерывание правого энкодера

float clampFloat(float value, float low, float high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
} // обработчик верхних и нижниц границ подаваемого на вход числа (от нижней к верх границе)
// TODO: могут возникнуть проблемы с NaN

struct EncoderDeltas {
    int32_t left;
    int32_t right;
};
// струткура для хранения дельты энкодеров

int32_t absInt32(int32_t value) {
    return value < 0 ? -value : value;
} // возвращеам модуль числа

EncoderDeltas filterEncoderDeltas(
    int32_t left_delta,
    int32_t right_delta,
    float target_linear_mps,
    float target_angular_rps) 
    /*
    ? input: дельты правого и левого моторов за цикл, заданная лин и угл скорости (в сек) 
    ? output: энкодер дельта
    */
    
    {
    const bool linear_stopped = fabsf(target_linear_mps) <= kLinearCommandDeadbandMps; // проверка в попадание зону периода действия
    const bool angular_stopped = fabsf(target_angular_rps) <= kAngularCommandDeadbandRadS; // проверка в попадание зону периода действия

    if (linear_stopped && angular_stopped &&
        absInt32(left_delta) <= kStoppedWheelDeltaDeadbandCounts &&
        absInt32(right_delta) <= kStoppedWheelDeltaDeadbandCounts) {
        return {0, 0};
    } // если команды малы по возмущениям, то возвращаем нули

    if (!linear_stopped && angular_stopped &&
        absInt32(right_delta - left_delta) <= kStraightDiffDeltaDeadbandCounts) {
        const int32_t average_delta = (left_delta + right_delta) / 2;
        return {average_delta, average_delta};
    } // если робот двигается линейно без угл движения

    return {left_delta, right_delta};
}
// ! смысл функции: фильтрация слишком маленьких возмущений энкодеров и фильтрация на линейное движение

float countsToMotorRps(int32_t counts_per_cycle) {
    const float counts_per_second = counts_per_cycle * (1000.0f / kControlPeriodMs); // сколько упр команд в 1 сек
    return counts_per_second / kEncoderCountsPerMotorRev; 
} // возвращает число оборотов мотора в секунду
// на вход подается число отсчетов энкодера в секу
// TODO: Формула предполагает, что фактический интервал между readAndResetDelta() всегда в точности равен kControlPeriodMs, а в реальности у нас может времени пройти больше и в энкодере накопятся вычисления за реально пройденный участок времени, а мы считаем щас за конкр меньший участок

float motorRpsToRpm(float motor_rps) {
    return motor_rps * 60.0f;
}
// переводим обороты/сек в минуты

float motorRpsToWheelLinearMps(float motor_rps) {
    const float wheel_rps = motor_rps; // ! так нужно?
    return wheel_rps * kWheelCircumferenceM;
}
// перевод оборотов в лин скорость колеса
// TODO: это корректно, когда энкодер измеряет именно обороты колеса без промежуточного редуктора

float linearMpsToWheelRpm(float linear_mps) {
    return (linear_mps / kWheelCircumferenceM) * 60.0f;
} // перевод лин скорости в обороты / мин

float angularRadSToWheelDeltaRpm(float angular_rad_s) {
    return (angular_rad_s * kTrackWidthM / kWheelCircumferenceM) * 60.0f;
} // перевод угл скорости в разность оборотов правого и лев колеса

float wrapAngleRad(float angle) {
    while (angle > kPi) {
        angle -= 2.0f * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0f * kPi;
    }
    return angle;
}
// нормализация угла [-pi,pi]
// TODO: Для обычной одометрии угол изменяется мало, поэтому цикл выполняется максимум один раз. Но для очень большого значения функция может выполнять много итераций. Альтернатива — fmodf

void configurePid(uPID& pid, float kp, float ki, float kd, float out_min, float out_max) {
    // ~ uPID& - ссылка на уже нами сделанный uPID, чтобы ничего не клонировать объект в памяти
    pid.setKp(kp);
    pid.setKi(ki);
    pid.setKd(kd);
    pid.outMin = out_min;
    pid.outMax = out_max;
}
// конфигурация пид-регулятора. Вроде все ясно

void configurePids() {
    configurePid(linear_pid, kLMotorKp, kLMotorKi, kLMotorKd, -kMaxPwmCommand, kMaxPwmCommand);
    configurePid(angular_pid, kAMotorKp, kAMotorKi, kAMotorKd, -kMaxPwmCommand, kMaxPwmCommand);
}
// настройка сразу обоих пидов под лин и угл скорости

void configureHardware() {
    left_motor.setReverse(kLeftMotorReverse); // зеркалим направление левого мотор
    right_motor.setReverse(kRightMotorReverse); // зеркалим направление правого мотора
    // ! главное учитывать инверсии и моторов и энкодеров
    pinMode(LED_BUILTIN, OUTPUT); // настраиваем светодиод
    digitalWrite(LED_BUILTIN, LOW); // выключаем светодиод
    configureSonar(); // инициализируем сонары

    left_encoder.begin(); // инициализируем энкодер левый
    right_encoder.begin(); // инициализируем энкодер правый
    // & attachInterrupt - 
    // & digitalPinToInterrupt - преобразует номер цифр вывода в номер внеш прерывания
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_CLK), leftEncoderISR, CHANGE); // 
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_DT), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_CLK), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_DT), rightEncoderISR, CHANGE);
}
// 
void readLatestCommand() {
    while (Serial.available() >= static_cast<int>(sizeof(ControlPacket))) {
        Serial.readBytes(reinterpret_cast<char*>(&command_packet), sizeof(ControlPacket));
        command_packet.target_linear_mps =
            clampFloat(command_packet.target_linear_mps, -kMaxLinearCommandMps, kMaxLinearCommandMps);
        command_packet.target_angular_rps =
            clampFloat(command_packet.target_angular_rps, -kMaxAngularCommandRadS, kMaxAngularCommandRadS);
        debug_raw_encoder = command_packet.debug_raw_encoder != 0;
        last_command_ms = millis();
    }
}
// считываем последнюю команду
// TODO: никаих синхронизаций и проверки на мусорность нет и если произойдет ошибочная передача, то все крашается сразу же

void applyCommandTimeout() {
    if (millis() - last_command_ms > kCommandTimeoutMs) {
        command_packet.target_linear_mps = 0.0f;
        command_packet.target_angular_rps = 0.0f;
    }
}
// проверяем насколько насколько давно получили команду, если давно, то обнуляем соотв переменные, но мы далее не применяем этот фалбек на моторы

void runControlCycle() {
    const int32_t left_delta = left_encoder.readAndResetDelta(); // считываем энкодер, обнуляем дельту и возвращаем искомое значение
    const int32_t right_delta = right_encoder.readAndResetDelta(); // считываем энкодер, обнуляем дельту и возвращаем искомое значение
    const EncoderDeltas filtered_deltas = filterEncoderDeltas( 
        left_delta,
        right_delta,
        command_packet.target_linear_mps,
        command_packet.target_angular_rps); // фильтрация значений энкодера

    const float left_motor_rps = countsToMotorRps(filtered_deltas.left); // скорости мотора
    const float right_motor_rps = countsToMotorRps(filtered_deltas.right); // скорости мотора
    const float current_left_wheel_rpm = motorRpsToRpm(left_motor_rps); // кол-во оборотов в мин
    const float current_right_wheel_rpm = motorRpsToRpm(right_motor_rps); // кол-во оборотов в мин

    const float current_left_wheel_mps = motorRpsToWheelLinearMps(left_motor_rps); // лин скорости моторов
    const float current_right_wheel_mps = motorRpsToWheelLinearMps(right_motor_rps); // лин скорости моторов

    const float current_linear_mps = 0.5f * (current_left_wheel_mps + current_right_wheel_mps); // лин скорость робота
    const float current_angular_rps = -(current_right_wheel_mps - current_left_wheel_mps) / kTrackWidthM; // угл скорость робота
    const float current_linear_rpm = 0.5f * (current_left_wheel_rpm + current_right_wheel_rpm); // средняя лин скорость моторов
    const float current_angular_rpm = -(current_right_wheel_rpm - current_left_wheel_rpm); // угловая разность моторов

    // Midpoint integration improves accuracy versus pure Euler for curved motion. одометрия по методу ср ориентации
    const float delta_heading = current_angular_rps * kControlPeriodS; // изменение угла
    const float heading_mid = odom_heading_rad + 0.5f * delta_heading; // ориент в сер интервала
    const float delta_s = current_linear_mps * kControlPeriodS; // пройденное расстояние центра робота
    odom_x_m += delta_s * cosf(heading_mid); // изменение икс коорды
    odom_y_m += delta_s * sinf(heading_mid); // изменение игрик коорды
    odom_heading_rad = wrapAngleRad(odom_heading_rad + delta_heading); // изменение угла робота
    // TODO: Энкодерные дельты являются интегральным измерением за интервал. Деление их на фиксированный период даёт среднюю скорость за этот интервал. Применение midpoint к средней линейной и угловой скорости является разумным приближением при условии примерно постоянных скоростей.

    if (sonarObstacleDetected()) {
        command_packet.target_linear_mps = 0.0f;
        command_packet.target_angular_rps = 0.0f;
        left_motor.runSpeed(0);
        right_motor.runSpeed(0);

        telemetry_packet.odom_x_m = odom_x_m;
        telemetry_packet.odom_y_m = odom_y_m;
        telemetry_packet.odom_heading_rad = odom_heading_rad;
        telemetry_packet.raw_left_encoder_delta = debug_raw_encoder ? left_delta : 0;
        telemetry_packet.raw_right_encoder_delta = debug_raw_encoder ? right_delta : 0;
        telemetry_packet.left_pwm = 0;
        telemetry_packet.right_pwm = 0;
        telemetry_packet.current_linear_mps = current_linear_mps;
        telemetry_packet.current_angular_rps = current_angular_rps;

        if (Serial.availableForWrite() >= static_cast<int>(sizeof(TelemetryPacket))) {
            Serial.write(reinterpret_cast<const uint8_t*>(&telemetry_packet), sizeof(TelemetryPacket));
        }
        return;
    }
    // TODO: у меня много вопрос к модулю обработки сонарных значений: по этому коду следует, что если мы близко находимся к препятствию, то срочно останавливаемся (и при этом интегралы пидовских регуляторов явно не занулются, то есть после нового начала движения у нас пид регулятор не ресетиться). Но точно ли стоит выносить логику остановки в ардуино составляющую, а не в малинку? Малинка как будто может скорректировать управление при слишком близком контакте с объектами.

    linear_pid.setpoint = linearMpsToWheelRpm(command_packet.target_linear_mps); // задаем лин пид
    angular_pid.setpoint = angularRadSToWheelDeltaRpm(command_packet.target_angular_rps); // задаем угл пид

    const float linear_pwm = linear_pid.compute(current_linear_rpm); // вычисляется выход пида для лин скорости
    const float angular_pwm = angular_pid.compute(current_angular_rpm); // вычисляется выход пида для угл скорости

    const int16_t left_pwm =
        static_cast<int16_t>(clampFloat(linear_pwm - angular_pwm, -kMaxPwmCommand, kMaxPwmCommand));
    const int16_t right_pwm =
        static_cast<int16_t>(clampFloat(linear_pwm + angular_pwm, -kMaxPwmCommand, kMaxPwmCommand));
    const int16_t left_pwm_limited =
        static_cast<int16_t>(clampFloat(static_cast<float>(left_pwm), -kMaxPwmCommand, kMaxPwmCommand));
    const int16_t right_pwm_limited =
        static_cast<int16_t>(clampFloat(static_cast<float>(right_pwm), -kMaxPwmCommand, kMaxPwmCommand));
// TODO: по моему тут возникают ошибки вычислений в зависимости от знаков лин и ангулар пвм и тут избыточные int16_t
    left_motor.runSpeed(left_pwm_limited); 
    right_motor.runSpeed(right_pwm_limited);
// TODO: диапозон гивер библиотеки пвм сигнала совпадает точно с нашим ? [-255, 255]
    status_led_state = !status_led_state; // переключаем лед
    digitalWrite(LED_BUILTIN, status_led_state ? HIGH : LOW); // меняется состояние на каждом упр цикле 

    telemetry_packet.odom_x_m = odom_x_m;
    telemetry_packet.odom_y_m = odom_y_m;
    telemetry_packet.odom_heading_rad = odom_heading_rad;
    telemetry_packet.raw_left_encoder_delta = debug_raw_encoder ? left_delta : 0;
    telemetry_packet.raw_right_encoder_delta = debug_raw_encoder ? right_delta : 0;
    telemetry_packet.left_pwm = left_pwm_limited;
    telemetry_packet.right_pwm = right_pwm_limited;
    telemetry_packet.current_linear_mps = current_linear_mps;
    telemetry_packet.current_angular_rps = current_angular_rps;
    // TODO: тут передаются кстати не фильтрованные дельты 

    if (Serial.availableForWrite() >= static_cast<int>(sizeof(TelemetryPacket))) {
        Serial.write(reinterpret_cast<const uint8_t*>(&telemetry_packet), sizeof(TelemetryPacket));
    }
    // передача телеметрии, но отстутсвует коунтер скипнутых команд
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate); // запуск UART, kSerialBaudRate - скорость передачи
    Serial.setTimeout(5); // таймаут в мс для потоковых операций
    // TODO: надо убедиться, что таймаут не слишком велик относительно периоду упр операций

    configureHardware();
    configurePids();

    left_motor.runSpeed(0);
    right_motor.runSpeed(0);

    last_control_ms = millis(); // время нач отсчета
    last_command_ms = last_control_ms; // время посл команды приравнивается к тек времени
}

void loop() {
    readLatestCommand(); // проверяем юарт
    applyCommandTimeout(); // проверка таймаута каждую итерацию

    const uint32_t now = millis();
    if (now - last_control_ms < kControlPeriodMs) {
        return;
    } // фиксируем время единное внутри одной итерации
    last_control_ms += kControlPeriodMs;
// TODO: мы обновляемся не по истинному времени, а по идельному периоду управления
    runControlCycle();
}

/*
! главные проблемы:
- проблемы со знаками у угл пида
- использование не истинного времени между упр воздействиями
- отсутствие проверки нанов
- не оч уверен, что хорошая идея обрабатывать не истинные значения энкодеров, а усреднять по фильтру лин движения
- остутсвие сброса пида при таймауте или срабатывании модуля с сонарами
- проверить код на атомарности операций
- убедиться что pulsein - неблокирующая операция
*/