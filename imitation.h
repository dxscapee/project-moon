#ifndef PROJECT_MOON_IMITATION_H
#define PROJECT_MOON_IMITATION_H

#include <cstddef>


enum class SimulationResult {
    Good,       //сел
    Crashed,    //разбился
    Stopped,    //остановился раньше нужного
    Over        //слишком много шагов
};

class imitation {
private:

    long double H_0;    //расстояние в нач. время
    long double H_curr; //расстояние в конкретное время
    long double V_0;    //скорость в нач. время
    long double V_curr; //скорость в конкретное время

    long double m; //масса (не учитываем вес топлива)
    long double g; //ускорение свободного падения (УСП)
    long double P_max; //тяга двигателя

    long double t_on; //время, при котором включается двигателя
    long double t_curr; //время в конкретный момент
    long double dt; //шаг по времени (dt)

    bool finished = false; //флаг, разрешающий моделирование


public:

    //params init
    imitation(const long double myH_0,const long double myV_0,
        const long double my_m, const long double myP_max, const long double my_g,
        const long double my_dt) :
    H_0(myH_0), H_curr(myH_0),
    V_0(myV_0), V_curr(myV_0),
    m(my_m), g(my_g), P_max(myP_max),
    t_on(0.0L), t_curr(0.0L), dt(my_dt){}

    //численные методы: Эйлер и Рунге-Кутта 4 порядка
    void euler_step();
    void RK4();

    void RK4(const long double step_dt);

    //getters
    long double get_t() const;
    long double get_H() const;
    long double get_V() const;
    long double get_H_exact(const long double t) const;
    long double get_t_on() const;
    long double get_a() const;

    //setters
    void set_t_on(long double my_t_on);

    //симуляция для нахождения t_on
    SimulationResult simulate_rk4(const std::size_t max_steps);

    //поиск нужного t_вкл (t_on)
    long double find_t_on();

    ~imitation() {}
};


#endif // PROJECT_MOON_IMITATION_H
