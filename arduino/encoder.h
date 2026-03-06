class Encoder {
    public:
        Encoder(int A, int B);
        bool direction();
        int64_t cnt();
        int32_t speed() {return _speed;};
    private:
        void ISR();
        int64_t _cnt;
        int64_t _cnt_old;
        int32_t last_time;
        bool _direction; // true for clock-wise and false for counter clock-wise
        int32_t _speed; // average speed over a SPEED_PERIOD ms period
}
