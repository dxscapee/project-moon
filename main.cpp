#include <iostream>
#include "imitation.h"
#include <cmath>
#include <iomanip>

//расчёт погрешности
long double get_error(const long double S,const long double S_exact) {
    return std::abs((S-S_exact)/S_exact) * 100.0L;
}

int main() {

    //В рамках первой итерация можно выставить
    //const-модификатор для переменных начального условия
    const long double myS_0 = 0.0L;
    const long double myV_0 = 0.0L;
    const long double mya = 2.0L;
    const long double myt_step = 0.0001L;
    const long double myt_max = 10.0L;


    //for Euler method
    imitation IM1(myS_0,myV_0,mya,myt_step,myt_max);

    //for Runge-Kutta method
    imitation IM2(myS_0,myV_0,mya,myt_step,myt_max);

    //прогон по времени, главный цикл программы
    while (IM1.get_t() <= IM1.get_tmax() && IM2.get_t() <= IM2.get_tmax())
    {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "t:" << IM1.get_t() << " | ";
        std::cout << "Euler:" << IM1.get_V() << " ";
        std::cout << IM1.get_S() << " | ";
        std::cout << "RK4:" << IM2.get_V() << " ";
        std::cout << IM2.get_S() << std::endl;

        if (IM1.get_t() <= IM1.get_tmax()) {
            IM1.euler_step();
        }
        if (IM2.get_t() <= IM2.get_tmax()) {
            IM2.RK4();
        }

    }
    std::cout << "t:" << IM1.get_t() << " | ";
    std::cout << "Euler:" << IM1.get_V() << " ";
    std::cout << IM1.get_S() << " | ";
    std::cout << "RK4:" << IM2.get_V() << " ";
    std::cout << IM2.get_S() << std::endl << std::endl;

    //вывод информации
    std::cout << "---/input data/---" << std::endl;
    std::cout << "S_0: " << myS_0 << std::endl;
    std::cout << "V_0: " << myV_0 << std::endl;
    std::cout << "a: " << mya << std::endl;
    std::cout << "t_delta: " << myt_step << std::endl;
    std::cout << "t_max: " << myt_max << std::endl;
    std::cout << "------------------" << std::endl << std::endl;

    std::cout << "--------------/output data/--------------" << std::endl;
    std::cout << "Analytics formula: " << "E:" << IM1.get_S_exact() << " RK4:" << IM2.get_S_exact() << std::endl;
    std::cout << "Error Euler method: " << get_error(IM1.get_S(),IM1.get_S_exact()) << '%'  << std::endl;
    std::cout << "Error Runge-Kutta method: " << get_error(IM2.get_S(),IM2.get_S_exact()) << '%' << std::endl;
    std::cout << "-----------------------------------------" << std::endl << std::endl ;
    std::cout << "t      " << "t_max  " << "S       " << "S_exact " << "V      " << std::endl;
    std::cout << IM1.get_t() << " " << IM1.get_tmax() << " "<< IM1.get_S() << " "<< IM1.get_S_exact() << " " << IM1.get_V() << std::endl;
    std::cout << IM2.get_t() << " " << IM2.get_tmax() << " "<< IM2.get_S() << " "<< IM2.get_S_exact() << " " << IM2.get_V() << std::endl;

    return 0;
}
