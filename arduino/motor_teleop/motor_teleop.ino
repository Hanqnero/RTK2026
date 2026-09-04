// Ручное управление двумя моторами с клавиатуры + живой лог энкодеров.
//
// Монитор порта на 115200. Команды исполняются посимвольно, режим
// окончания строки в мониторе значения не имеет - лишние символы
// (перевод строки и т.п.) просто игнорируются.
//
//   w вперёд   s назад   a разворот влево   d разворот вправо   q стоп
//   + быстрее  - медленнее
//
// Пины ШИМ моторов совпадают с motor_interface.h: LEFT 5/4, RIGHT 7/6.
// Энкодеры - как physически разведено на этой плате прямо сейчас: 17/19
// у левого, 18/16 у правого. Это НЕ совпадает ни с motor_interface.h
// (20/2, 21/3), ни с arduino/pinout.md (18/19, 20/21) - в репозитории
// сейчас три разных источника правды про эти четыре вывода, разбираться
// с этим отдельно.
//
// Выходы энкодера JGB37-520 называются A и B (не CLK/DT - это чужая
// терминология от роторных кнопочных энкодеров). Какой физический
// провод считается A, а какой B, для правильности счёта не важно:
// это меняет знак, а не корректность, и в боевой прошивке для этого
// как раз есть kLeftEncoderReverse/kRightEncoderReverse. Здесь эти
// флаги не применяются - лог сырой, для проверки, что вообще крутится
// и считает.
//
// Квадратура читается ОПРОСОМ, без attachInterrupt: у 16 и 17 на Mega
// вообще нет аппаратного прерывания, а даже там, где оно есть, здесь
// в нём нет нужды. Энкодер сидит на валу мотора ДО редуктора, поэтому
// частота его фронтов не зависит от передаточного числа - только от
// оборотов самого моторчика.
//
// Потолок по паспортным константам из motor_interface.h: kMaxWheelRpm
// 333 об/мин колеса при kGearRatio 18.8 дают на валу мотора 6260 об/мин
// = 104 об/с; 44 фронта на оборот (kEncoderPulsesPerMotorRev 11 x
// kEncoderDecodeFactor 4) - итого около 4600 фронтов/с в пределе, один
// фронт раз в ~220 мкс. loop() без единой блокировки проходит этот
// промежуток десятки раз, так что опрос ничего не теряет. Прерывания
// в боевой прошивке нужны не из-за скорости энкодера, а потому что там
// loop() занят другой работой (ПИД, протокол, сонар) - здесь такой
// нагрузки нет.
//
// РОБОТА ПОДНЯТЬ НАД СТОЛОМ.

const uint8_t LEFT_PWM_A = 5;   // левый вперёд
const uint8_t LEFT_PWM_B = 4;   // левый назад
const uint8_t RIGHT_PWM_A = 7;  // правый вперёд
const uint8_t RIGHT_PWM_B = 6;  // правый назад

const uint8_t LEFT_ENC_A = 16;
const uint8_t LEFT_ENC_B = 18;
const uint8_t RIGHT_ENC_A = 17;
const uint8_t RIGHT_ENC_B = 19;

// Если долго не пришло ни одной команды, останавливаемся сами: иначе
// забытый на столе робот продолжит крутить колёса.
const unsigned long COMMAND_TIMEOUT_MS = 2000;
const unsigned long LOG_PERIOD_MS = 100;

long left_ticks = 0;
long right_ticks = 0;
uint8_t left_state = 0;
uint8_t right_state = 0;

// Квадратурное декодирование без внешних библиотек: индекс - это два
// старых бита и два новых, значение - на сколько сдвинулся счётчик.
// Недопустимые переходы (пропущенный фронт, если опрос всё же где-то
// не успел) дают 0, а не случайное число.
const int8_t QUAD_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

// Читает оба канала и продвигает счётчик, если состояние изменилось
// с прошлого опроса. Вызывается из loop() на каждой итерации.
void pollEncoder(uint8_t pin_a, uint8_t pin_b, uint8_t* state, long* ticks) {
  const uint8_t s = (digitalRead(pin_a) << 1) | digitalRead(pin_b);
  if (s != *state) {
    *ticks += QUAD_TABLE[(*state << 2) | s];
    *state = s;
  }
}

int16_t pwm_step = 120;
int16_t left_pwm = 0;
int16_t right_pwm = 0;
char command = 'q';

unsigned long last_command_ms = 0;
unsigned long last_log_ms = 0;
long left_last_ticks = 0;
long right_last_ticks = 0;

void drive(uint8_t fwd_pin, uint8_t back_pin, int16_t pwm) {
  if (pwm >= 0) {
    analogWrite(back_pin, 0);
    analogWrite(fwd_pin, pwm);
  } else {
    analogWrite(fwd_pin, 0);
    analogWrite(back_pin, -pwm);
  }
}

void applyCommand(char cmd) {
  switch (cmd) {
    case 'w': left_pwm = pwm_step;  right_pwm = pwm_step;  break;
    case 's': left_pwm = -pwm_step; right_pwm = -pwm_step; break;
    case 'a': left_pwm = -pwm_step; right_pwm = pwm_step;  break;
    case 'd': left_pwm = pwm_step;  right_pwm = -pwm_step; break;
    case 'q': left_pwm = 0;         right_pwm = 0;         break;
    default: return;
  }
  command = cmd;
  drive(LEFT_PWM_A, LEFT_PWM_B, left_pwm);
  drive(RIGHT_PWM_A, RIGHT_PWM_B, right_pwm);
  last_command_ms = millis();
}

// Что должно получиться на энкодерах при известной команде мотору.
// 'w'/'s': оба колеса крутятся в одну сторону - знаки дельт обязаны
// совпасть. 'a'/'d': разворот на месте, колёса крутятся навстречу друг
// другу - знаки обязаны разойтись. Ноль на одном канале при ненулевой
// команде - подозрение на МОЛЧАНИЕ, но сразу после смены команды это
// нормально: мотор ещё не успел толком раскрутиться за один тик лога.
const char* verdict(char cmd, long ld, long rd) {
  if (cmd == 'q') {
    return "-";
  }
  if (ld == 0 || rd == 0) {
    return "МОЛЧИТ?";
  }
  const bool same_sign = (ld > 0) == (rd > 0);
  if (cmd == 'w' || cmd == 's') {
    return same_sign ? "OK" : "НЕСОВПАДЕНИЕ";
  }
  // cmd == 'a' || cmd == 'd'
  return same_sign ? "НЕСОВПАДЕНИЕ" : "OK";
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_PWM_A, OUTPUT);
  pinMode(LEFT_PWM_B, OUTPUT);
  pinMode(RIGHT_PWM_A, OUTPUT);
  pinMode(RIGHT_PWM_B, OUTPUT);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Начальное состояние читается сразу, чтобы первый же вызов
  // pollEncoder() в loop() сравнивал с реальными уровнями, а не с
  // нулями по умолчанию.
  left_state = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);
  right_state = (digitalRead(RIGHT_ENC_A) << 1) | digitalRead(RIGHT_ENC_B);

  Serial.println();
  Serial.println("teleop: w s a d q, + - скорость");
  Serial.println("энкодеры опрашиваются в loop(), без прерываний");
  Serial.println("ms,cmd,pwm_step,left_pwm,right_pwm,left_delta,right_delta,left_total,right_total,verdict");

  last_command_ms = millis();
  last_log_ms = last_command_ms;
}

void loop() {
  pollEncoder(LEFT_ENC_A, LEFT_ENC_B, &left_state, &left_ticks);
  pollEncoder(RIGHT_ENC_A, RIGHT_ENC_B, &right_state, &right_ticks);

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '+') {
      pwm_step = min(255, pwm_step + 20);
      applyCommand(command);
    } else if (c == '-') {
      pwm_step = max(0, pwm_step - 20);
      applyCommand(command);
    } else if (c == 'w' || c == 's' || c == 'a' || c == 'd' || c == 'q') {
      applyCommand(c);
    }
  }

  if (command != 'q' && millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    Serial.println("# таймаут, стоп");
    applyCommand('q');
  }

  if (millis() - last_log_ms >= LOG_PERIOD_MS) {
    last_log_ms += LOG_PERIOD_MS;

    const long ld = left_ticks - left_last_ticks;
    const long rd = right_ticks - right_last_ticks;
    left_last_ticks = left_ticks;
    right_last_ticks = right_ticks;

    Serial.print(millis());
    Serial.print(',');
    Serial.print(command);
    Serial.print(',');
    Serial.print(pwm_step);
    Serial.print(',');
    Serial.print(left_pwm);
    Serial.print(',');
    Serial.print(right_pwm);
    Serial.print(',');
    Serial.print(ld);
    Serial.print(',');
    Serial.print(rd);
    Serial.print(',');
    Serial.print(left_ticks);
    Serial.print(',');
    Serial.print(right_ticks);
    Serial.print(',');
    Serial.println(verdict(command, ld, rd));
  }
}
