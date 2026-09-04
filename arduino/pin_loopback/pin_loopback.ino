// Проверка самих выводов платы, без датчика.
//
// Датчик отключается, вместо него между PIN_OUT и PIN_IN ставится перемычка
// напрямую. Скетч дёргает один вывод и читает другой: если уровни повторяются,
// значит выводы, провод и код исправны, и виноват датчик или его подключение.
// Если не повторяются - дело в плате или в перемычке.
//
// Вывод смотрится в мониторе порта на скорости 115200.
//
// Перемычка безопасна, пока залит именно этот скетч: PIN_OUT работает
// выходом, PIN_IN входом, а вход высокоомный, и тока через него почти нет.
//
// Перед заливкой любой другой прошивки перемычку надо вынуть. Если оба
// вывода окажутся выходами и разойдутся по уровням, через кристалл пойдёт
// сквозной ток. В боевой прошивке выводы 7 и 6 - это как раз пара PWM
// правого мотора, то есть два выхода.

const uint8_t PIN_OUT = 7;
const uint8_t PIN_IN = 6;

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);

  Serial.println();
  Serial.print("перемычка между ");
  Serial.print(PIN_OUT);
  Serial.print(" и ");
  Serial.println(PIN_IN);
}

void loop() {
  digitalWrite(PIN_OUT, LOW);
  delay(5);
  const bool low_ok = digitalRead(PIN_IN) == LOW;

  digitalWrite(PIN_OUT, HIGH);
  delay(5);
  const bool high_ok = digitalRead(PIN_IN) == HIGH;

  if (low_ok && high_ok) {
    Serial.println("выводы исправны: сигнал проходит");
  } else if (!low_ok && !high_ok) {
    Serial.println("сигнал не проходит вовсе: нет перемычки или обрыв");
  } else {
    Serial.print("вывод залип: при LOW читается ");
    Serial.print(low_ok ? "LOW" : "HIGH");
    Serial.print(", при HIGH читается ");
    Serial.println(high_ok ? "HIGH" : "LOW");
  }

  delay(1000);
}
