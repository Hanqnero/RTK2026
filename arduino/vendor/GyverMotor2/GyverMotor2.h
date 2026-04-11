#pragma once
#include <Arduino.h>

enum class GM2 : uint8_t {
    ONLY_PWM,       // один ШИМ (PWM)
    DIR_DIR,        // без ШИМ (dir + dir)
    DIR_PWM,        // один ШИМ без инверсии (dir + PWM)
    DIR_PWM_INV,    // один ШИМ с инверсией (dir + ~PWM)
    PWM_PWM_SPEED,  // два ШИМ, режим скорости (PWM + PWM)
    PWM_PWM_POWER,  // два ШИМ, режим момента (~PWM + ~PWM)
    DIR_DIR_PWM,    // два направления и ШИМ (dir + dir + PWM)
};

template <GM2 driver, uint8_t res = 8>
class GyverMotor2 {
   public:
    // инициализация с указанием типа драйвера, разрешения ШИМ (бит) и пинов
    GyverMotor2(uint8_t pinA, uint8_t pinB = 0xff, uint8_t pinC = 0xff) : _pinA(pinA), _pinB(pinB), _pinC(pinC) {
        pinMode(_pinA, OUTPUT);
        if (driver != GM2::ONLY_PWM) pinMode(_pinB, OUTPUT);
        if (driver == GM2::DIR_DIR_PWM) pinMode(_pinC, OUTPUT);
        _setAll(0);
    }

    // установить deadtime (задержка при смене направления) в микросекундах (умолч. 0)
    void setDeadtime(uint8_t us) {
        _deadtime = us;
    }

    // получить deadtime в микросекундах
    uint8_t getDeadTime() const {
        return _deadtime;
    }

    // установить реверс направления (умолч. false)
    void setReverse(bool rev) {
        _rev = rev;
        _run(_speed);
    }

    // получить реверс направления
    bool getReverse() const {
        return _rev;
    }

    // установить минимальный ШИМ (умолч. 0)
    void setMinDuty(uint16_t duty) {
        _minDuty = (duty > _maxDuty) ? _maxDuty : duty;
        _run(_speed);
    }

    // установить минимальный ШИМ в % от максимального (умолч. 0)
    void setMinDutyPerc(uint8_t perc) {
        setMinDuty(_perc(perc));
    }

    // получить минимальный ШИМ
    uint16_t getMinDuty() const {
        return _minDuty;
    }

    // получить максимальный ШИМ
    uint16_t getMaxDuty() const {
        return _maxDuty;
    }

    // установить скорость ШИМ (-макс.. макс)
    void runSpeed(int16_t speed) {
        speed = constrain(speed, -_maxDuty, _maxDuty);
#ifndef GMOTOR2_NO_ACCEL
        _target = speed;
        if (!_ds) _run(speed);
#else
        _run(speed);
#endif
    }

    // установить скорость в % от максимального ШИМ (-100.. 100%)
    void runSpeedPerc(int8_t perc) {
        runSpeed(_perc(perc));
    }

    // получить текущую скорость мотора в ШИМ
    int16_t getSpeed() const {
        return _speed;
    }

    // получить направление: мотор крутится (1 и -1), мотор остановлен (0)
    int8_t getDir() const {
        return _speed ? (_speed > 0 ? 1 : -1) : 0;
    }

    // остановка. Если включено ускорение, то плавная. Отключает мотор
    void stop() {
        runSpeed(0);
    }

    // активное торможение (оставляет пины драйвера активными)
    void brake(bool active = true) {
        _setAll(active);
        _speed = 0;
#ifndef GMOTOR2_NO_ACCEL
        _target = 0;
#endif
    }

    // остановить и отключить мотор
    void disable() {
        brake(false);
    }

    // установить ускорение в ШИМ в секунду. Реальный минимум - 5. 0 чтобы отключить
    void setAccel(uint16_t accel) {
#ifndef GMOTOR2_NO_ACCEL
        if (!accel) {
            _ds = 0;
            _prd = 0;
        } else {
            uint16_t ms = 1000 / accel;
            _prd = constrain(ms, 30, 200);
            _ds = uint32_t(accel) * _prd / 1000;
            _ds = constrain(_ds, 1, _maxDuty);
        }
#endif
    }

    // установить ускорение в процентах в секунду. 0 чтобы отключить
    void setAccelPerc(uint8_t perc) {
        setAccel(_perc(perc));
    }

    // получить период, не реже которого нужно вызывать tick, мс
    uint8_t getDt() const {
#ifndef GMOTOR2_NO_ACCEL
        return _prd;
#else
        return 0;
#endif
    }

    // получить целевую скорость (режим ускорения)
    int16_t getTarget() const {
#ifndef GMOTOR2_NO_ACCEL
        return _target;
#else
        return 0;
#endif
    }

    // плавное изменение к указанной скорости с ускорением setAccel, вызывать в loop. Вернёт true при достижении скорости
    bool tick() {
#ifndef GMOTOR2_NO_ACCEL
        if (_ds && (_speed != _target) && (uint8_t(uint8_t(millis()) - _tmr) >= _prd)) {
            _tmr = millis();
            int16_t err = _target - _speed;
            _run(_speed + constrain(err, -_ds, _ds));
            return _speed == _target;
        }
#endif
        return false;
    }

   private:
    void _run(int16_t duty) {
        if (!duty) {
            disable();
            return;
        }

        if (_deadtime && _speed && (_speed > 0) != (duty > 0)) {
            _setAll(0);
            delayMicroseconds(_deadtime);
        }

        _speed = duty;

        bool dir = (duty > 0) ^ _rev;
        if (duty < 0) duty = -duty;
        if (_minDuty) duty = ((int32_t(duty) * (_maxDuty - _minDuty) + (1 << res) - 1) >> res) + _minDuty;

#ifdef __AVR__
        if (res > 8 && duty == 255) ++duty;  // защита от 255 при разрешении > 8 бит
#endif

        switch (driver) {
            case GM2::ONLY_PWM:
                analogWrite(_pinA, duty);
                break;

            case GM2::DIR_DIR:
                digitalWrite(_pinA, !dir);
                digitalWrite(_pinB, dir);
                break;

            case GM2::DIR_PWM:
                digitalWrite(_pinA, !dir);
                analogWrite(_pinB, duty);
                break;

            case GM2::DIR_PWM_INV:
                digitalWrite(_pinA, !dir);
                analogWrite(_pinB, dir ? duty : (_maxDuty - duty));
                break;

            case GM2::PWM_PWM_SPEED:
                analogWrite(_pinA, dir ? 0 : duty);
                analogWrite(_pinB, dir ? duty : 0);
                break;

            case GM2::PWM_PWM_POWER:
                analogWrite(_pinA, dir ? (_maxDuty - duty) : _maxDuty);
                analogWrite(_pinB, dir ? _maxDuty : (_maxDuty - duty));
                break;

            case GM2::DIR_DIR_PWM:
                digitalWrite(_pinA, !dir);
                digitalWrite(_pinB, dir);
                analogWrite(_pinC, duty);
                break;
        }
    }

    void _setAll(bool val) {
        switch (driver) {
            case GM2::ONLY_PWM:
                analogWrite(_pinA, 0);
                break;

            case GM2::DIR_DIR:
                digitalWrite(_pinA, val);
                digitalWrite(_pinB, val);
                break;

            case GM2::DIR_PWM:
            case GM2::DIR_PWM_INV:
                digitalWrite(_pinA, val);
                analogWrite(_pinB, val ? _maxDuty : 0);
                break;

            case GM2::PWM_PWM_SPEED:
            case GM2::PWM_PWM_POWER:
                analogWrite(_pinA, val ? _maxDuty : 0);
                analogWrite(_pinB, val ? _maxDuty : 0);
                break;

            case GM2::DIR_DIR_PWM:
                digitalWrite(_pinA, val);
                digitalWrite(_pinB, val);
                analogWrite(_pinC, val ? _maxDuty : 0);
                break;
        }
    }

    const uint8_t _pinA, _pinB, _pinC;
    uint8_t _deadtime = 0;
    int16_t _minDuty = 0, _speed = 0;
#ifndef GMOTOR2_NO_ACCEL
    int16_t _target = 0, _ds = 0;
    uint8_t _tmr = 0, _prd = 0;
#endif
    bool _rev = 0;

    static constexpr int16_t _maxDuty = (1 << res) - 1;
    static constexpr int16_t _perc(int8_t perc) { return int32_t(_maxDuty) * perc / 100; }
};