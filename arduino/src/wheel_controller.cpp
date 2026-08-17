#include "wheel_controller.h"

#include <math.h>

#include "math_utils.h"
#include "motor_interface.h"

namespace {

// Ниже этой уставки колесо считается остановленным и статическая часть
// feedforward не подаётся. Иначе при нулевой команде мотор получал бы
// k_static и дёргался на месте.
constexpr float kSetpointDeadbandRps = 0.02f;

}  // namespace

WheelController::WheelController()
    // D_ERROR - классическое дифференцирование ошибки.
    // I_SATURATE - conditional integration: при насыщении выхода интеграл
    // не накапливается. Пределы насыщения выставляются на каждом цикле
    // в update(), поэтому здесь важен только сам режим.
    : _pid(D_ERROR | I_SATURATE, kControlPeriodMs),
      _gains{0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      _previous_setpoint(0.0f),
      _configured_dt_ms(kControlPeriodMs),
      _terms{} {}

void WheelController::setGains(const WheelGains& gains) {
    _gains = gains;

    _pid.setKp(gains.kp);
    _pid.setKi(gains.ki);
    _pid.setKd(gains.kd);
}

void WheelController::reset() {
    // Предыдущая ошибка и флаг первого вызова в uPID приватные, публичного
    // сброса у библиотеки нет. Но объект тривиально копируем и ничего не
    // аллоцирует, поэтому присваивание свежего экземпляра честно обнуляет
    // всё внутреннее состояние, включая историю производной.
    //
    // Без этого первый расчёт после сброса дал бы выброс по D: разность
    // считалась бы относительно ошибки, оставшейся от прошлого движения.
    _pid = uPID(D_ERROR | I_SATURATE, _configured_dt_ms);

    setGains(_gains);

    _previous_setpoint = 0.0f;
}

float WheelController::update(float setpoint_rps, float measured_rps, float dt_s) {
    // Нулевой или отрицательный интервал означает сбой измерения времени.
    // Делить на него нельзя, а подставлять номинальный период - значит
    // вернуться ровно к той ошибке, которую убрал протокол v2.
    if (!(dt_s > 0.0f)) {
        return 0.0f;
    }

    // Смена направления - это новый режим движения. Интеграл, накопленный
    // при движении вперёд, при развороте работает против нового направления.
    if (setpoint_rps * _previous_setpoint < 0.0f) {
        _pid.integral = 0.0f;
    }
    _previous_setpoint = setpoint_rps;

    // uPID принимает период только в целых миллисекундах. Разрешения хватает,
    // пока период измеряется десятками миллисекунд: постоянная часть ошибки
    // поглощается подобранным Ki, а переменная равна джиттеру цикла, который
    // виден в StatsPayload и лечится не здесь.
    const uint16_t dt_ms = static_cast<uint16_t>(
        clampFloat(roundf(dt_s * 1000.0f), 1.0f, 65535.0f));

    if (dt_ms != _configured_dt_ms) {
        _pid.setDt(dt_ms);
        _configured_dt_ms = dt_ms;
    }

    float feedforward = 0.0f;
    if (fabsf(setpoint_rps) > kSetpointDeadbandRps) {
        feedforward =
            _gains.k_static * signOf(setpoint_rps) + _gains.k_velocity * setpoint_rps;
    }
    feedforward = clampFloat(feedforward, -kMaxPwmCommand, kMaxPwmCommand);

    // Ключевая деталь связки. I_SATURATE в uPID проверяет насыщение по своим
    // outMin/outMax и про feedforward ничего не знает. Если оставить пределы
    // равными полному диапазону PWM, регулятор считал бы себя ненасыщенным
    // тогда, когда сумма с feedforward уже упёрлась в предел, и продолжал бы
    // копить интеграл.
    //
    // Поэтому пределы задаются по остатку диапазона после feedforward.
    // Тогда сумма ff + pid всегда лежит в допустимом диапазоне, а анти-виндап
    // срабатывает ровно при насыщении суммы.
    _pid.outMax = kMaxPwmCommand - feedforward;
    _pid.outMin = -kMaxPwmCommand - feedforward;

    _pid.setpoint = setpoint_rps;
    const float pid_output = _pid.compute(measured_rps);

    const float output =
        clampFloat(feedforward + pid_output, -kMaxPwmCommand, kMaxPwmCommand);

    const float error = setpoint_rps - measured_rps;

    _terms.setpoint_rps = setpoint_rps;
    _terms.measured_rps = measured_rps;
    _terms.error_rps = error;
    _terms.proportional = _gains.kp * error;
    // Конфигурация I_KI_OUTSIDE: в выход идёт именно Ki * integral.
    _terms.integral_term = _gains.ki * _pid.integral;
    _terms.feedforward = feedforward;
    _terms.pid_output = pid_output;
    _terms.output_pwm = output;

    return output;
}
