#include <Arduino.h>

#include "MeArm.h"
#include "pca9685_servo_output.h"

namespace {

// =========================
// МЕНЯТЬ НАСТРОЙКИ ЗДЕСЬ
// =========================
//
// Это главный блок для настройки под твое железо.
// Сначала меняй только эти значения, остальной код трогать не надо.

// Скорость Serial Monitor. В мониторе порта поставь такое же значение: 115200.
constexpr uint32_t kSerialBaud = 115200;

// I2C-адрес PCA9685.
//
// Обычно он 0x40.
// Если I2C scanner покажет другой адрес, поменяй только эту строку.
constexpr uint8_t kPca9685Address = 0x40;

// Частота PWM для сервоприводов.
//
// Для обычных hobby servo почти всегда нужна частота 50 Hz.
constexpr uint16_t kServoFrequencyHz = 50;

// Номера каналов PCA9685, куда подключены сервоприводы.
//
// Важно: это НЕ пины Arduino.
// Это номера выходов на синей плате PCA9685: 0, 1, 2, ... 15.
//
// !!! Если рука двигает не тот сустав, поменяй номера каналов здесь!!!.
constexpr uint8_t kBaseChannel = 0;
constexpr uint8_t kShoulderChannel = 1;
constexpr uint8_t kElbowChannel = 2;
constexpr uint8_t kClawChannel = 3;

// =========================
// КОНЕЦ НАСТРОЕК
// =========================

Pca9685ServoOutput pwm_output;
MeArm arm;
bool ready = false;

void writeChannel(uint8_t channel, uint16_t pulse_us, uint16_t hold_ms) {
    // Эта функция отправляет один тестовый импульс на один канал.
    // Например: channel=0, pulse_us=1500 значит "канал 0 в центр".
    Serial.print(F("channel="));
    Serial.print(channel);
    Serial.print(F(" pulse_us="));
    Serial.println(pulse_us);
    pwm_output.writeMicroseconds(channel, pulse_us);
    delay(hold_ms);
}

void testSingleChannel(uint8_t channel) {
    // Первый тест делаем очень осторожным:
    // - 1500 us: центр;
    // - 1400 us: чуть в одну сторону;
    // - 1600 us: чуть в другую сторону.
    //
    // Здесь специально нет 1000..2000 us, чтобы на первом запуске не ударить механику.
    writeChannel(channel, Pca9685ServoOutput::kDefaultCenterPulseUs, 700);
    writeChannel(channel, 1400, 700);
    writeChannel(channel, 1600, 700);
    writeChannel(channel, Pca9685ServoOutput::kDefaultCenterPulseUs, 700);
}

void testRawChannels() {
    // Перед MeArm проверяем саму плату PCA9685.
    // Так проще понять проблему:
    // - если тут не двигается серво, проблема в питании/проводах/I2C;
    // - если тут работает, а MeArm нет, проблема выше, в калибровке руки.
    Serial.println(F("Raw PCA9685 channel test"));
    testSingleChannel(kBaseChannel);
    testSingleChannel(kShoulderChannel);
    testSingleChannel(kElbowChannel);
    testSingleChannel(kClawChannel);
}

void runMeArmDemo() {
    // Это уже тест всей цепочки:
    // MeArm -> адаптер -> I2C -> PCA9685 -> сервы.
    //
    // Движение специально короткое и спокойное.
    // Если механика упирается, останови питание серв и уменьши диапазоны.
    Serial.println(F("MeArm home"));
    arm.snapTo(0, 100, 50);
    delay(700);

    Serial.println(F("MeArm claw open"));
    arm.openClaw();
    delay(700);

    Serial.println(F("MeArm claw close"));
    arm.closeClaw();
    delay(700);

    Serial.println(F("MeArm short move"));
    arm.moveToXYZ(40, 120, 70);
    delay(700);

    Serial.println(F("MeArm return home"));
    arm.moveToXYZ(0, 100, 50);
    arm.openClaw();
    delay(1500);
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaud);
    Serial.setTimeout(5);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.println(F("MeArm PCA9685 I2C test"));
    Serial.println(F("Mega SDA=D20 SCL=D21 PCA9685 addr=0x40 freq=50Hz"));

    if (!pwm_output.begin(kPca9685Address, kServoFrequencyHz)) {
        Serial.println(F("PCA9685 begin failed"));
        return;
    }

    // Если какие-то сервы дергаются слишком далеко, можно сузить общий диапазон:
    // pwm_output.setPulseLimits(1100, 1900);
    //
    // После калибровки лучше сделать отдельные лимиты для каждого сустава,
    // но для первого запуска общий диапазон проще и безопаснее.

    testRawChannels();

    // Здесь MeArm получает 4 "виртуальных PWM-пина".
    // На самом деле это каналы PCA9685:
    // base -> kBaseChannel, shoulder -> kShoulderChannel, elbow -> kElbowChannel, claw -> kClawChannel.
    arm.begin(pwm_output, kBaseChannel, kShoulderChannel, kElbowChannel, kClawChannel);
    ready = true;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println(F("MeArm PCA9685 ready"));
}

void loop() {
    if (!ready) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(500);
        return;
    }

    runMeArmDemo();
}
