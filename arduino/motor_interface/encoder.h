    #pragma once

class Encoder {
    public:
        Encoder(int, int);
        bool direction();
        int64_t cnt();
        int32_t speed();
        void refresh();
        void Handler();
        void start();
        const int A, B;
    private:
        volatile int64_t _cnt;
        volatile int64_t _cnt_old;
        volatile int32_t last_time;
        volatile bool _direction; // true for clock-wise and false for counter clock-wise
        volatile int32_t _speed; // average speed over a SPEED_PERIOD_MS ms period
};

extern Encoder left_enc;
extern Encoder right_enc;

void enc_start();
