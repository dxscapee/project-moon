#include "imitation.h"
#include <iomanip>
#include <cmath>
#include <iostream>

//расчёт погрешности
long double get_error(const long double H, const long double H_exact)
{
    if (std::abs(H_exact) < 1e-12L) {
        return -1.0L;
    }

    return std::abs((H - H_exact) / H_exact) * 100.0L;
}



int main() {

    //ИНИЦИАЛИЗАЦИЯ
    //===================================================================
    const long double myH_0 = 200.0L;  // начальная высота
    const long double myV_0 = 0.0L;     // начальная скорость

    const long double my_m = 100.0L;    // масса аппарата
    const long double myP_max = 300.0L; // максимальная тяга
    const long double my_g = 1.62L;     // ускорение свободного падения

    const long double my_dt = 0.01L;    // шаг по времени

    //объект для подбора нужного времени включения (t_вкл)
    imitation test(myH_0, myV_0, my_m, myP_max, my_g, my_dt);

    //объект для моделирования при помощи RK4
    imitation IM1(myH_0, myV_0, my_m, myP_max, my_g, my_dt);

    //объект для моделирования при помощи Euler
    imitation IM2(myH_0, myV_0, my_m, myP_max, my_g, my_dt);
    //===================================================================


    std::cout << std::fixed << std::setprecision(3);
    //БЛОК ПОДБОРА ВРЕМЕНИ ВКЛЮЧЕНИЯ
    //===================================================================
    const long double t_on = test.find_t_on();

    //если невозможно приземлиться, то завершаем программу и не моделируем
    if (t_on < 0.0L) {
        std::cout << "t_on not found\n";
        return 0;
    }

    std::cout << "Optimal t_on: " << t_on << " s\n";

    const SimulationResult result = test.simulate_rk4(10000);
    std::cout << "Simulation result: ";
    switch (result) {
        case SimulationResult::Good:
            std::cout << "Good\n"; break;
        case SimulationResult::Crashed:
            std::cout << "Crashed\n"; break;
        case SimulationResult::Stopped:
            std::cout << "Stopped\n"; break;
        case SimulationResult::Over:
            std::cout << "Over\n"; break;
    }

    std::cout << "Final height: " << test.get_H() << " m\n";
    std::cout << "Final velocity: " << test.get_V() << " m/s\n";

    //остановка, чтобы зафиксировать t_on
    int stoper;
    std::cin >> stoper;

    //===================================================================



    // МОДЕЛИРОВАНИЕ
    //===================================================================
    IM1.set_t_on(t_on);
    IM2.set_t_on(t_on);

    bool rk4_active = true;
    bool euler_active = true;

    std::size_t step = 0;
    const std::size_t max_steps = 100'000;

    while ((rk4_active || euler_active) && step < max_steps) {
        std::cout << "t: " << IM1.get_t() << ';' << IM2.get_t()
                  << " | H: " << IM1.get_H() << ';' << IM2.get_H()
                  << " | V: " << IM1.get_V() << ';' << IM2.get_V()
                  << " | a: " << IM1.get_a() << ';' << IM2.get_a()
                  << '\n';

        if (rk4_active) {
            IM1.RK4();

            if (IM1.get_H() <= 0.0L) {
                rk4_active = false;
            }
        }

        if (euler_active) {
            IM2.euler_step();

            if (IM2.get_H() <= 0.0L) {
                euler_active = false;
            }
        }

        ++step;

    }

    if (rk4_active || euler_active) {
        std::cout << "Simulation stopped: maximum steps" << std::endl;
    }
    //===================================================================



    // РЕЗУЛЬТАТЫ
    //===================================================================
    std::cout << "========== INPUT DATA ==========" << std::endl;

    std::cout << "H_0: " << myH_0 << " m\n";
    std::cout << "V_0: " << myV_0 << " m/s\n";
    std::cout << "m: " << my_m << " kg\n";
    std::cout << "P_max: " << myP_max << " N\n";
    std::cout << "g: " << my_g << " m/s^2\n";
    std::cout << "dt: " << my_dt << " s\n";
    std::cout << "t_on: " << t_on << " s\n";


    std::cout << "========= OUTPUT  DATA =========" << std::endl;


    std::cout << "---------- RK4 RESULTS ----------" << std::endl;

    const long double H_exact_RK4 = IM1.get_H_exact(IM1.get_t());
    const long double abs_error_RK4 =
        std::abs(IM1.get_H() - H_exact_RK4);

    std::cout << "Time: " << IM1.get_t() << " s\n";
    std::cout << "Height: " << IM1.get_H() << " m\n";
    std::cout << "Absolute height difference: "
              << std::scientific << abs_error_RK4 << " m\n";
    std::cout << "Final velocity: " << IM1.get_V() << " m/s\n";




    std::cout << "---------- EULER RESULTS ----------" << std::endl;

    const long double H_exact_Euler = IM2.get_H_exact(IM2.get_t());
    const long double abs_error_Euler =
        std::abs(IM2.get_H() - H_exact_Euler);

    std::cout << "Time: " << IM2.get_t() << " s\n";
    std::cout << "Height: " << IM2.get_H() << " m\n";
    std::cout << "Absolute height difference: "
              << std::scientific << abs_error_Euler << " m\n";
    std::cout << "Final velocity: " << IM2.get_V() << " m/s\n";
    //===================================================================



    return 0;
}
