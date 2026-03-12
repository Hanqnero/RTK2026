class Encoder {
    public:
        Encoder(int A, int B, int index = 0);
        bool direction() { return _direction; }
        int64_t cnt() { return _cnt; }
        int32_t speed() { return _speed; }
    private:
        void handleISR();
        static Encoder* _instances[2];
        static void _isr0();
        static void _isr1();
        int _pin_b;
        int64_t _cnt;
        int64_t _cnt_old;
        uint32_t _last_time;
        bool _direction;
        int32_t _speed;
};
