#include "imitation.h"

void imitation::euler_step()
{
    // Не выполняем шаг, если конечное время уже достигнуто.
    if (t_curr >= t_max) return;

    // Фактический шаг:
    const long double remaining = t_max - t_curr;
    const long double dt = (remaining >= t_step) ? t_step : remaining;


    S_curr += V_curr * dt;
    V_curr += a * dt;
    t_curr += dt;

    if (remaining <= t_step) {
        t_curr = t_max;
    }
}

void imitation::RK4()
{
    //Не выполняем шаг, если конечное время уже достигнуто.
    //в main программа прогоняет сразу два объекта разными методами
    //из-за этого возможно, что один объект достигнет нужного результата
    //раньше другого, на этот случай и нужна проверка снизу.
    //Убрав её программа не сломается (данные будут высчитываться корректно),
    //Но с проверкой мы просто не делаем лишних расчётов и нагрузок на систему
    if (t_curr >= t_max) return;

    //Фактический шаг
    const long double remaining = t_max - t_curr;
    const long double dt = (remaining >= t_step) ? t_step : remaining;


    const long double k1 = V_curr * dt;
    const long double m1 = a * dt;

    const long double V_mid1 = V_curr + m1 / 2;
    const long double k2 = V_mid1 * dt;
    const long double m2 = a * dt;

    const long double V_mid2 = V_curr + m2 / 2;
    const long double k3 = V_mid2 * dt;
    const long double m3 = a * dt;

    const long double V_end = V_curr + m3;
    const long double k4 = V_end * dt;
    const long double m4 = a * dt;

    S_curr += (k1 + 2.0L * k2 + 2.0L * k3 + k4) / 6;
    V_curr += (m1 + 2.0L * m2 + 2.0L * m3 + m4) / 6;
    t_curr += dt;

    //По сути проверка ниже может быть излишняя,
    //Но убрав её, в какой-то момент, мы можем получить
    //на выходе не 10.000 (например), а 9.99999998
    if (remaining <= t_step) {
        t_curr = t_max;
    }
}

long double imitation::get_t() const{
    return t_curr;
}
long double imitation::get_tmax() const {
    return t_max;
}

long double imitation::get_S() const{
    return S_curr;
}

long double imitation::get_V() const{
    return V_curr;
}

long double imitation::get_S_exact() const {
    return S_0 + V_0 * t_curr + a*t_curr*t_curr/2.0L;
}
