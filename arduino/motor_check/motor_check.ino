// Проверка моторных каналов голым PWM, без протокола и энкодеров.
//
// Задача скетча - найти неработающий канал, поэтому выводы дёргаются
// ПО ОДНОМУ. Так видно не просто "правое не крутится", а какой именно
// вывод не даёт движения: это разделяет мёртвый вывод контроллера,
// мёртвый канал драйвера и оборванный провод мотора.
//
// Распайка обязана совпадать с arduino/include/motor_interface.h -
// иначе проверка отвечает не про тот канал, который потом поедет.
// Прежняя версия этого файла считала левым 7/4, а правым 6/5, то есть
// путала выводы 5 и 7 местами.
//
// РОБОТА ПОДНЯТЬ НАД СТОЛОМ.

// Мотор левого колеса. Направление задаёт то, на какой из двух выводов
// подан сигнал; второй при этом обязан быть в нуле.
const uint8_t LEFT_PWM_A = 5;
const uint8_t LEFT_PWM_B = 4;

// Мотор правого колеса.
const uint8_t RIGHT_PWM_A = 7;
const uint8_t RIGHT_PWM_B = 6;

const uint8_t PWM = 255;
const unsigned long STEP_MS = 4000;
const unsigned long PAUSE_MS = 1500;

struct Channel {
  const char* label;
  uint8_t pin;
  uint8_t idle_pin;  // Парный вывод того же мотора: обязан быть в нуле.
};

const Channel CHANNELS[] = {
  {"левый  вывод 5 (LEFT_PWM_A)",  LEFT_PWM_A,  LEFT_PWM_B},
  {"левый  вывод 4 (LEFT_PWM_B)",  LEFT_PWM_B,  LEFT_PWM_A},
  {"правый вывод 7 (RIGHT_PWM_A)", RIGHT_PWM_A, RIGHT_PWM_B},
  {"правый вывод 6 (RIGHT_PWM_B)", RIGHT_PWM_B, RIGHT_PWM_A},
};
const uint8_t CHANNEL_COUNT = sizeof(CHANNELS) / sizeof(CHANNELS[0]);

void allOff() {
  analogWrite(LEFT_PWM_A, 0);
  analogWrite(LEFT_PWM_B, 0);
  analogWrite(RIGHT_PWM_A, 0);
  analogWrite(RIGHT_PWM_B, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_PWM_A, OUTPUT);
  pinMode(LEFT_PWM_B, OUTPUT);
  pinMode(RIGHT_PWM_A, OUTPUT);
  pinMode(RIGHT_PWM_B, OUTPUT);

  allOff();

  Serial.println();
  Serial.println(F("Проверка моторных каналов, по одному выводу."));
  Serial.println(F("Смотрите, на каком шаге колесо ТРОГАЕТСЯ."));
  Serial.println();
}

void loop() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) {
    const Channel& c = CHANNELS[i];

    Serial.print(F("PWM 255 -> "));
    Serial.println(c.label);

    // Парный вывод глушим явно: на драйвере с двумя входами PWM сигнал
    // сразу на обоих означает торможение, а не вращение.
    analogWrite(c.idle_pin, 0);
    analogWrite(c.pin, PWM);
    delay(STEP_MS);

    allOff();
    Serial.println(F("   стоп"));
    delay(PAUSE_MS);
  }

  Serial.println(F("--- круг пройден, повтор ---"));
  Serial.println();
}
