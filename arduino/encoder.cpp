class Encoder {
    public:
        Encoder(int A, int B);
        bool direction() { return _direction};
        int64_t cnt() { return _cnt; };
    private:
        void ISR();
        int64_t _cnt;    
        bool _direction; // true for clock-wise and false for counter clock-wise
}


Encoder::Encoder(int A, int B) : _cnt{0}, _direction{false};
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
}