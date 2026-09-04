// Проверка ультразвукового дальномера HC-SR04.
// Монитор порта на 115200.

const uint8_t TRIG_PIN = 20;
const uint8_t ECHO_PIN = 21;

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);

  // Подтяжка отличает отсутствие датчика от его молчания. Без неё свободный
  // вход ловит наводку с соседней дорожки TRIG и выдаёт выдуманные
  // сантиметры. Подключённому датчику она не мешает.
  pinMode(ECHO_PIN, INPUT_PULLUP);
}

void loop() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Ждём эхо не дольше 25 мс: это около четырёх метров туда и обратно.
  const unsigned long echo_us = pulseIn(ECHO_PIN, HIGH, 25000);

  if (echo_us == 0) {
    Serial.println("нет эха");
  } else {
    // Звук проходит сантиметр туда и обратно за 58 микросекунд.
    Serial.print(echo_us / 58.0, 1);
    Serial.println(" см");
  }

  delay(100);
}
