#define SPEED_PERIOD_MS 100
#define SPEED_MULT = (1000 / SPEED_PERIOD)

class Encoder {
    public:
        Encoder(int A, int B);
        bool direction() { return _direction};
        int64_t cnt() { return _cnt; };
        int32_t speed() {return _speed;};
    private:
        void ISR();
        int64_t _cnt;
        int64_t _cnt_old;
        int32_t last_time;
        bool _direction; // true for clock-wise and false for counter clock-wise
        int32_t _speed; // average speed over a SPEED_PERIOD_MS ms period
}

Encoder::Encoder(int A, int B) : _cnt{0}, _direction{false}, _speed{0}, last_time{0};
{
    pinmode(A, INPUT);
    pinmode(B, INPUT);

    attachInterrupt(digitalPinToInterrupt(A), ISR, RISING);
}

void Encoder::ISR() {
    if (digitalRead(B)) {
        // Clock-wise rotation
        ++_cnt;
        _direction = true;
    } else {
        // Counter Clock-wise rotation
        --_cnt;
        _direction = false;
    }

    // update speed
    if (millis() - last_time >= SPEED_PERIOD_MS) {
        _speed = (_cnt - cnt_old) * SPEED_MULT;
        _cnt_old = _cnt;
    }
}
