/*
  enc_test.ino — автоматическая диагностика энкодеров

  Скетч сам гоняет моторы вперёд/назад и печатает счётчики.
  Смотрим:
    1. Какой счётчик меняется при каком моторе (проверка перекрещенных проводов)
    2. Знак: вперёд → +, назад → −
*/

// ── пины моторов ─────────────────────────────────
#define LEFT_RPWM  11
#define LEFT_LPWM  10
#define RIGHT_RPWM  8
#define RIGHT_LPWM  9

// ── пины энкодеров ────────────────────────────────
// (текущее предположение: левый мотор → 18/19, правый → 2/3)
#define LEFT_ENC_A  18
#define LEFT_ENC_B  19
#define RIGHT_ENC_A  2
#define RIGHT_ENC_B  3

// ── ISR счётчики ──────────────────────────────────
volatile int32_t cnt_left  = 0;
volatile int32_t cnt_right = 0;

void isr_left() {
    bool a = digitalRead(LEFT_ENC_A);
    bool b = digitalRead(LEFT_ENC_B);
    if (a == b) cnt_left++; else cnt_left--;
}

void isr_right() {
    bool a = digitalRead(RIGHT_ENC_A);
    bool b = digitalRead(RIGHT_ENC_B);
    if (a == b) cnt_right++; else cnt_right--;
}

// ── управление моторами ───────────────────────────
void set_motors(int l, int r) {
    l = constrain(l, -255, 255);
    r = constrain(r, -255, 255);
    analogWrite(LEFT_RPWM,  l > 0 ?  l : 0);
    analogWrite(LEFT_LPWM,  l < 0 ? -l : 0);
    analogWrite(RIGHT_RPWM, r > 0 ?  r : 0);
    analogWrite(RIGHT_LPWM, r < 0 ? -r : 0);
}

void stop() { set_motors(0, 0); }

// ── тест ─────────────────────────────────────────
struct Phase {
    const char* label;
    int  l_pwm;
    int  r_pwm;
    int  duration_ms;
};

const Phase phases[] = {
    { "STOP",             0,    0,   500 },
    { "LEFT_FWD",       150,    0,  1500 },
    { "STOP",             0,    0,   500 },
    { "LEFT_BWD",      -150,    0,  1500 },
    { "STOP",             0,    0,   500 },
    { "RIGHT_FWD",        0,  150,  1500 },
    { "STOP",             0,    0,   500 },
    { "RIGHT_BWD",        0, -150,  1500 },
    { "STOP",             0,    0,   500 },
    { "BOTH_FWD",       150,  150,  1500 },
    { "STOP",             0,    0,   500 },
    { "BOTH_BWD",      -150, -150,  1500 },
    { "STOP",             0,    0,  1000 },
};
const int N_PHASES = sizeof(phases) / sizeof(phases[0]);

int   phase_idx  = 0;
bool  done       = false;
uint32_t phase_start = 0;

void setup() {
    Serial.begin(115200);

    pinMode(LEFT_RPWM,  OUTPUT); pinMode(LEFT_LPWM,  OUTPUT);
    pinMode(RIGHT_RPWM, OUTPUT); pinMode(RIGHT_LPWM, OUTPUT);
    stop();

    pinMode(LEFT_ENC_A,  INPUT_PULLUP); pinMode(LEFT_ENC_B,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  isr_left,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isr_right, CHANGE);

    // сброс счётчиков
    cnt_left = cnt_right = 0;

    Serial.println("phase            L_cnt   R_cnt");
    phase_start = millis();
    set_motors(phases[0].l_pwm, phases[0].r_pwm);
    Serial.print(phases[0].label); Serial.println(" START");
}

void loop() {
    if (done) return;

    uint32_t now = millis();

    // печатаем каждые 200 мс
    static uint32_t last_print = 0;
    if (now - last_print >= 200) {
        last_print = now;
        noInterrupts();
        int32_t l = cnt_left;
        int32_t r = cnt_right;
        interrupts();

        Serial.print(phases[phase_idx].label);
        Serial.print("\t");
        Serial.print(l);
        Serial.print("\t");
        Serial.println(r);
    }

    // переход к следующей фазе
    if (now - phase_start >= (uint32_t)phases[phase_idx].duration_ms) {
        phase_idx++;
        if (phase_idx >= N_PHASES) {
            stop();
            Serial.println("=== TEST DONE ===");
            done = true;
            return;
        }
        set_motors(phases[phase_idx].l_pwm, phases[phase_idx].r_pwm);
        phase_start = now;
        Serial.print(">>> "); Serial.println(phases[phase_idx].label);
        // сбрасываем счётчики в начале каждой фазы для удобства чтения
        noInterrupts();
        cnt_left = cnt_right = 0;
        interrupts();
    }
}
