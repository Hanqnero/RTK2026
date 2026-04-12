// motor_test.ino — standalone test: едем вперёд 2 с, стоп 1 с, назад 2 с, стоп.
// Логирует данные энкодеров каждые 100 мс через Serial (115200).
// Pins: LEFT_RPWM=11, LEFT_LPWM=10, RIGHT_RPWM=8, RIGHT_LPWM=9
//       LEFT_ENC_CLK=2, LEFT_ENC_DT=3, RIGHT_ENC_CLK=18, RIGHT_ENC_DT=19

// ── Пины ──────────────────────────────────────────────────────────────────
#define LEFT_RPWM  11
#define LEFT_LPWM  10
#define RIGHT_RPWM  8
#define RIGHT_LPWM  9
#define LEFT_ENC_A  2
#define LEFT_ENC_B  3
#define RIGHT_ENC_A 18
#define RIGHT_ENC_B 19

// ── Энкодеры ──────────────────────────────────────────────────────────────
volatile int32_t left_cnt  = 0;
volatile int32_t right_cnt = 0;

void left_isr() {
    left_cnt += (digitalRead(LEFT_ENC_B) == HIGH) ? 1 : -1;
}

void right_isr() {
    right_cnt += (digitalRead(RIGHT_ENC_B) == HIGH) ? 1 : -1;
}

// ── Мотор ─────────────────────────────────────────────────────────────────
void set_motors(int left_pwm, int right_pwm) {
    left_pwm  = constrain(left_pwm,  -255, 255);
    right_pwm = constrain(right_pwm, -255, 255);

    if (left_pwm >= 0) {
        analogWrite(LEFT_RPWM,  left_pwm);
        analogWrite(LEFT_LPWM,  0);
    } else {
        analogWrite(LEFT_RPWM,  0);
        analogWrite(LEFT_LPWM,  -left_pwm);
    }

    if (right_pwm >= 0) {
        analogWrite(RIGHT_RPWM, right_pwm);
        analogWrite(RIGHT_LPWM, 0);
    } else {
        analogWrite(RIGHT_RPWM, 0);
        analogWrite(RIGHT_LPWM, -right_pwm);
    }
}

// ── Лог энкодеров ─────────────────────────────────────────────────────────
uint32_t last_log_ms = 0;
int32_t  prev_left   = 0;
int32_t  prev_right  = 0;

void log_encoders(const char* phase) {
    int32_t l, r;
    noInterrupts();
    l = left_cnt;
    r = right_cnt;
    interrupts();

    int32_t dl = l - prev_left;
    int32_t dr = r - prev_right;
    prev_left  = l;
    prev_right = r;

    Serial.print("[");
    Serial.print(millis());
    Serial.print(" ms] ");
    Serial.print(phase);
    Serial.print("  L_cnt=");
    Serial.print(l);
    Serial.print("  R_cnt=");
    Serial.print(r);
    Serial.print("  dL=");
    Serial.print(dl);
    Serial.print("  dR=");
    Serial.println(dr);
}

// ── Сценарий ──────────────────────────────────────────────────────────────
struct Phase {
    const char* name;
    int  pwm;        // > 0 вперёд, < 0 назад, 0 стоп
    uint32_t dur_ms;
};

const Phase PHASES[] = {
    { "FORWARD",  100,  2000 },
    { "STOP",       0,  1000 },
    { "BACKWARD", -100, 2000 },
    { "STOP",       0,  1000 },
};
const uint8_t N_PHASES = sizeof(PHASES) / sizeof(PHASES[0]);

uint8_t  phase_idx    = 0;
uint32_t phase_start  = 0;
bool     test_done    = false;

// ── Setup / Loop ──────────────────────────────────────────────────────────
void setup() {
    pinMode(LEFT_RPWM,  OUTPUT);
    pinMode(LEFT_LPWM,  OUTPUT);
    pinMode(RIGHT_RPWM, OUTPUT);
    pinMode(RIGHT_LPWM, OUTPUT);
    set_motors(0, 0);

    pinMode(LEFT_ENC_A,  INPUT_PULLUP);
    pinMode(LEFT_ENC_B,  INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  left_isr,  RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), right_isr, RISING);

    Serial.begin(115200);
    delay(500);

    Serial.println("=== motor_test start ===");
    Serial.println("time_ms | phase | L_cnt | R_cnt | dL | dR");

    phase_start = millis();
    last_log_ms = millis();
    set_motors(PHASES[0].pwm, PHASES[0].pwm);
    Serial.print(">> Phase 0: ");
    Serial.println(PHASES[0].name);
}

void loop() {
    if (test_done) {
        return;
    }

    uint32_t now = millis();

    // Переключение фаз
    if ((uint32_t)(now - phase_start) >= PHASES[phase_idx].dur_ms) {
        set_motors(0, 0);
        phase_idx++;
        if (phase_idx >= N_PHASES) {
            Serial.println("=== motor_test done ===");
            log_encoders("FINAL");
            test_done = true;
            return;
        }
        phase_start = now;
        set_motors(PHASES[phase_idx].pwm, PHASES[phase_idx].pwm);
        Serial.print(">> Phase ");
        Serial.print(phase_idx);
        Serial.print(": ");
        Serial.println(PHASES[phase_idx].name);
    }

    // Периодический лог каждые 100 мс
    if ((uint32_t)(now - last_log_ms) >= 100) {
        last_log_ms = now;
        log_encoders(PHASES[phase_idx].name);
    }
}
