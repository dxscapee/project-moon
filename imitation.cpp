#include "imitation.h"

void imitation::euler_step()
{
    S_curr += V_curr*t_step;
    V_curr += a*t_step;
    t_curr += t_step;
}

void imitation::RK4()
{
    //Для первой итерации прмоежуточные S не нужны. но на второй
    //понадобятся поэтому не удаляем, а комменнтируем

    const long double k1 = V_curr * t_step;
    const long double m1 = a * t_step;

    const long double V_mid1 = V_curr + m1/2;
    //const long double S_mid1 = S_curr + k1/2;

    const long double k2 = V_mid1 * t_step;
    const long double m2 = a * t_step;

    const long double V_mid2 = V_curr + m2/2;
    //const long double S_mid2 = S_curr + k2/2;

    const long double k3 = V_mid2 * t_step;
    const long double m3 = a * t_step;

    const long double V_end = V_curr + m3;
    //const long double S_end = S_curr + k3;

    const long double k4 = V_end*t_step;
    const long double m4 = a * t_step;

    S_curr += (k1 + 2.0L*k2 + 2.0L*k3 + k4) / 6;
    V_curr += (m1 + 2.0L*m2 + 2.0L*m3 + m4) / 6;
    t_curr += t_step;
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

void imitation::get_type() const{
    std::cout << "time:" << (typeid(t_curr).name() == std::string("e") ? "long double" : typeid(time).name()) << " | ";
    std::cout << "curr_S:" << (typeid(S_curr).name() == std::string("e") ? "long double" : typeid(time).name()) << " | ";
    std::cout << "curr_V:" << (typeid(V_curr).name() == std::string("e") ? "long double" : typeid(time).name()) << std::endl;
}