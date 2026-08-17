#pragma once

#include <stdint.h>

// Распайка драйвера моторов с двумя входами PWM на канал.
//
// Мотор A (левый) и мотор B (правый): по два вывода PWM у каждого. Направление
// задаётся тем, на какой из двух выводов подан сигнал, поэтому отдельного
// вывода направления нет.
//
// Номера обязаны совпадать с фактической распайкой платы.

#define LEFT_PWM_A 7
#define LEFT_PWM_B 6
#define RIGHT_PWM_A 5
#define RIGHT_PWM_B 4

#define LEFT_ENC_CLK 20
#define LEFT_ENC_DT 2
#define RIGHT_ENC_CLK 21
#define RIGHT_ENC_DT 3

#define SONAR_VCC_PIN 37
#define SONAR_TRIG_PIN 39
#define SONAR_ECHO_PIN 41
#define SONAR_GND_PIN 43

constexpr float kPi = 3.14159265358979323846F;

// Единственное место, где живёт ориентация железа.
//
// Контракт после настройки этих четырёх флагов:
//   - положительный PWM крутит колесо так, что робот едет вперёд;
//   - положительная дельта энкодера означает вращение колеса вперёд.
//
// Как только контракт выполняется, кинематика, одометрия и телеметрия
// работают в соглашении ROS без единой дополнительной инверсии.
// Никаких компенсирующих минусов в main.cpp быть не должно, и Raspberry Pi
// про распайку моторов ничего не знает.
//
// Проверяется стендово через verify_signs.py.
constexpr bool kLeftMotorReverse = true;
constexpr bool kRightMotorReverse = true;
constexpr bool kLeftEncoderReverse = true;
constexpr bool kRightEncoderReverse = false;

constexpr uint32_t kSerialBaudRate = 115200;
// Период управляющего цикла. Величину задаёт разрешение энкодера, а не
// стоимость расчёта: скорость колеса считается по числу отсчётов за цикл,
// и ошибка квантования равна половине отсчёта. На рабочих 0.24 м/с колесо
// даёт около 364 отсчётов в секунду, то есть 9 отсчётов за 25 мс и ±5.5%
// шума измеренной скорости. Вдвое короче цикл - вдвое больше этот шум,
// и дифференциальная часть регулятора начинает дифференцировать квантование.
constexpr uint16_t kControlPeriodMs = 25;
constexpr uint32_t kControlPeriodUs = static_cast<uint32_t>(kControlPeriodMs) * 1000UL;
constexpr uint16_t kCommandTimeoutMs = 500;

// Цикл считается сорванным, если фактический интервал превысил номинальный
// более чем в полтора раза, то есть 37.5 мс. Порог мягкий намеренно:
// одиночная задержка в него укладывается, а систематический срыв - уже нет.
constexpr uint32_t kCycleOverrunThresholdUs = kControlPeriodUs + kControlPeriodUs / 2UL;

// Период отправки StatsPayload. Раз в секунду достаточно для профилирования
// и не мешает потоку телеметрии.
constexpr uint16_t kStatsPeriodMs = 1000;

constexpr float kWheelRadiusM = 0.024f;
constexpr float kTrackWidthM = 0.195f;
constexpr float kWheelCircumferenceM = 2.0f * kPi * kWheelRadiusM;

// Первичные параметры привода.
//
// Всё, что ниже, из них выводится, поэтому рассогласовать константы между
// собой невозможно: число отсчётов на оборот колеса и достижимая скорость
// опираются на одно и то же передаточное отношение.

// Импульсов на оборот ВАЛА МОТОРА на один канал. У холловского энкодера
// JGB37-520 обычно 11, реже 13. Точное значение снимается calibrate_encoder.py:
// вывести его из паспорта нельзя.
constexpr float kEncoderPulsesPerMotorRev = 11.0f;

// uEncoder настроен в Type::Step1, то есть щелчок на каждую четверть периода,
// а прерывания навешаны на оба канала по CHANGE. Значит считаются все четыре
// фронта квадратуры.
constexpr float kEncoderDecodeFactor = 4.0f;

// Передаточное отношение редуктора.
//
// Энкодер JGB37-520 стоит на валу мотора, ДО редуктора. Поэтому редуктор
// обязан входить в пересчёт отсчётов в обороты колеса, иначе одометрия
// ошибается ровно в это число раз. Для варианта 960 об/мин на выходе это
// примерно 1:5.2, но паспорт даёт только обороты, а не отношение, поэтому
// значение подлежит измерению.
constexpr float kGearRatio = 5.2f;

// Обороты ВЫХОДНОГО вала на холостом ходу, из паспорта мотора.
constexpr float kMaxWheelRpm = 960.0f;

// Производные величины.

// Отсчётов энкодера на один оборот КОЛЕСА. Именно эта величина участвует
// в пересчёте дельт в скорость: редуктор в неё уже сложен.
constexpr float kEncoderCountsPerWheelRev =
    kEncoderPulsesPerMotorRev * kEncoderDecodeFactor * kGearRatio;

constexpr float kMaxWheelSetpointRps = kMaxWheelRpm / 60.0f;

// Доля физического потолка, которую разрешено запрашивать.
//
// Паспортные обороты даны на холостом ходу. Под нагрузкой достижимая скорость
// ниже, и уставка у самого предела означала бы постоянное насыщение
// регулятора: интеграл упирался бы в ограничение, а ошибка не убывала.
constexpr float kCommandSpeedMargin = 0.70f;

constexpr float kMaxLinearCommandMps =
    kMaxWheelSetpointRps * kWheelCircumferenceM * kCommandSpeedMargin;

// Ограничение угловой скорости политическое, а не физическое: развернуться
// на месте робот способен куда быстрее, но Nav2 такие скорости не нужны.
constexpr float kMaxAngularCommandRadS = kPi/2.0f;

// Коэффициенты по умолчанию для регулятора каждого колеса.
//
// Это запасные значения на случай пустой или повреждённой EEPROM. Рабочие
// коэффициенты живут в EEPROM и задаются стендовым скриптом настройки,
// поэтому правка этих констант требует перепрошивки и нужна только для
// смены точки старта настройки.
//
// Уставка и измерение выражены в оборотах колеса в секунду, выход - в PWM.
//
// k_static и k_velocity не подбираются на глаз: они снимаются по ступенькам
// PWM и равны, соответственно, PWM страгивания и наклону зависимости
// установившейся скорости от PWM. До их измерения оба нуля означают работу
// без feedforward, то есть чистый ПИД.
constexpr float kLeftWheelKp = 0.0f;
constexpr float kLeftWheelKi = 0.0f;
constexpr float kLeftWheelKd = 0.0f;
constexpr float kLeftWheelKStatic = 0.0f;
constexpr float kLeftWheelKVelocity = 0.0f;

constexpr float kRightWheelKp = 0.0f;
constexpr float kRightWheelKi = 0.0f;
constexpr float kRightWheelKd = 0.0f;
constexpr float kRightWheelKStatic = 0.0f;
constexpr float kRightWheelKVelocity = 0.0f;


constexpr float kMaxPwmDuty = 0.90f;
constexpr float kMaxPwmCommand = 255.0f * kMaxPwmDuty;

// Дальше таймаута эхо не ждём: считаем, что препятствия в пределах дальности
// нет. Прошивка дистанцию только измеряет и отдаёт в телеметрии.
constexpr uint32_t kSonarEchoTimeoutUs = 25000UL;
constexpr uint16_t kSonarSamplePeriodMs = 60;
