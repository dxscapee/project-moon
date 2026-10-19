#include "imitation.h"

void imitation::euler_step()
{
    //избегаем лишних шагов
    if (t_curr >= t_max) return;

    if (t_max - t_curr >= t_step)
    {
        S_curr += V_curr*t_step;
        V_curr += a*t_step;
        ++step_count;
        t_curr = step_count * t_step;
    }
    else
    {
        S_curr += V_curr*(t_max - t_curr);
        V_curr += a*(t_max - t_curr);
        ++step_count;
        t_curr = t_max;
    }
}

void imitation::RK4()
{
    //избегаем лишних шагов
    if (t_curr >= t_max) return;

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

    ++step_count;

    if (remaining <= t_step)
        t_curr = t_max;
    else
        t_curr += dt;
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
