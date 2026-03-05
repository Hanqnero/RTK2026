class Encoder {
    public:
        Encoder(int A, int B);
        bool direction();
        int64_t cnt();
    private:
        void ISR();
        int64_t _cnt;    
        bool _direction; // true for clock-wise and false for counter clock-wise
}