#pragma once

#include <stdint.h>

#include <uPID.h>

// Регулятор скорости одного колеса: feedforward плюс uPID.
//
// Почему на каждое колесо свой контур:
//
// В прежней схеме были регуляторы линейной и угловой скорости, а их выходы
// смешивались в PWM обоих моторов. Отдельного мотора в контуре не существовало,
// поэтому фразе "донастроить левый мотор" не соответствовал ни один коэффициент.
// Здесь у каждого колеса собственный контур и собственные коэффициенты.
//
// Почему ПИД взят готовый:
//
// В uPID уже реализованы режимы ограничения интеграла - I_SATURATE
// (conditional integration) и I_BACK_CALC. Своя реализация повторяла бы их
// без выигрыша. Класс добавляет ровно то, чего в библиотеке нет.
//
// Почему feedforward обязателен:
//
// У мотора есть мёртвая зона: до некоторого PWM он не трогается вовсе.
// Чистый ПИД вынужден выбираться из неё интегралом, что даёт либо вялый
// старт, либо перерегулирование. Feedforward подаёт нужный PWM сразу:
//
//     pwm_ff = k_static * sign(setpoint) + k_velocity * setpoint
//
// где k_static компенсирует страгивание, а k_velocity - установившуюся
// зависимость скорости от PWM. После этого ПИД правит только остаток,
// и коэффициенты получаются малыми и устойчивыми.
//
// Обе константы измеряются стендово по ступенькам PWM, а не подбираются.

struct WheelGains {
    float kp;
    float ki;
    float kd;

    // PWM, необходимый для страгивания колеса с места.
    float k_static;

    // PWM на один оборот в секунду в установившемся режиме.
    float k_velocity;
};

// Промежуточные величины одного расчёта. Нужны для настройки: по одному
// лишь выходу нельзя понять, что именно упёрлось - пропорциональная часть,
// насыщенный интеграл или недооценённый feedforward.
//
// Дифференциальная составляющая отдельным полем не передаётся: uPID не
// выдаёт её наружу, но она однозначно восстанавливается на хосте как
// pid_output - proportional - integral_term.
struct WheelControllerTerms {
    float setpoint_rps;
    float measured_rps;
    float error_rps;
    float proportional;
    float integral_term;
    float feedforward;
    float pid_output;
    float output_pwm;
};

class WheelController {
public:
    WheelController();

    void setGains(const WheelGains& gains);
    const WheelGains& gains() const { return _gains; }

    // Обнулить интеграл и историю производной.
    //
    // Вызывается при таймауте команды, аварийной остановке и смене знака
    // уставки. Без этого накопленный интеграл выстреливает в момент
    // возобновления движения.
    void reset();

    // Рассчитать PWM для колеса.
    //
    // :param setpoint_rps: требуемая скорость колеса, оборотов в секунду.
    // :param measured_rps: измеренная скорость колеса, оборотов в секунду.
    // :param dt_s: фактический интервал с предыдущего вызова, секунды.
    // :returns: команда PWM, уже ограниченная допустимым диапазоном.
    float update(float setpoint_rps, float measured_rps, float dt_s);

    const WheelControllerTerms& terms() const { return _terms; }

private:
    uPID _pid;
    WheelGains _gains;

    float _previous_setpoint;
    uint16_t _configured_dt_ms;

    WheelControllerTerms _terms;
};
