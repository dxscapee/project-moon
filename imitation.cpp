#include "imitation.h"
#include <cmath>
#include <iostream>


void imitation::euler_step()
{
    if (finished) {
        return;
    }

    if (H_curr <= 0.0L) {
        H_curr = 0.0L;
        V_curr = 0.0L;
        finished = true;
        return;
    }

    long double step_dt = dt;

    // Не допускаем перехода через момент включения двигателя.
    if (t_curr < t_on && t_curr + step_dt > t_on) {
        step_dt = t_on - t_curr;
    }

    const long double a = (t_curr >= t_on) ? (P_max / m - g) : -g;

    const long double H_start = H_curr;
    const long double V_start = V_curr;

    const long double H_next = H_start + V_start * step_dt;

    const long double V_next = V_start + a * step_dt;

    // Проверяем пересечение поверхности внутри шага.
    if (H_next <= 0.0L && V_start < 0.0L) {
        const long double tau = -H_start / V_start;

        t_curr += tau;
        H_curr = 0.0L;
        V_curr = V_start + a * tau;
        finished = true;
        return;
    }

    // Проверяем остановку аппарата во время движения вниз.
    if (V_start < 0.0L && V_next >= 0.0L && a > 0.0L) {
        const long double tau = -V_start / a;

        const long double H_stop = H_start + V_start * tau;

        t_curr += tau;
        V_curr = 0.0L;
        H_curr = (H_stop > 0.0L) ? H_stop : 0.0L;
        finished = true;
        return;
    }

    H_curr = H_next;
    V_curr = V_next;
    t_curr += step_dt;
}

void imitation::RK4()
{
    if (finished) {
        return;
    }

    if (H_curr <= 0.0L) {
        H_curr = 0.0L;
        V_curr = 0.0L;
        finished = true;
        return;
    }

    long double step_dt = dt;

    // Не допускаем перехода через момент включения двигателя.
    if (t_curr < t_on && t_curr + step_dt > t_on) {
        step_dt = t_on - t_curr;
    }

    if (step_dt <= 0.0L) {
        return;
    }

    // Ускорение на текущем участке движения.
    const long double a = (t_curr >= t_on) ? (P_max / m - g) : -g;


    const long double H_start = H_curr;
    const long double V_start = V_curr;

    const long double k1 = V_start * step_dt;
    const long double m1 = a * step_dt;

    const long double V_mid1 = V_start + m1 / 2.0L;
    const long double k2 = V_mid1 * step_dt;
    const long double m2 = a * step_dt;

    const long double V_mid2 = V_start + m2 / 2.0L;
    const long double k3 = V_mid2 * step_dt;
    const long double m3 = a * step_dt;

    const long double V_end = V_start + m3;
    const long double k4 = V_end * step_dt;
    const long double m4 = a * step_dt;

    const long double H_next = H_start + (k1 + 2.0L * k2 + 2.0L * k3 + k4) / 6.0L;

    const long double V_next = V_start + (m1 + 2.0L * m2 + 2.0L * m3 + m4) / 6.0L;

    // Проверяем остановку во время движения вниз.
    if (V_start < 0.0L && V_next >= 0.0L && a > 0.0L) {
        const long double tau = -V_start / a;

        const long double H_stop = H_start + V_start * tau + 0.5L * a * tau * tau;

        t_curr += tau;
        V_curr = 0.0L;
        H_curr = (H_stop > 0.0L) ? H_stop : 0.0L;
        finished = true;
        return;
    }

    // Проверяем пересечение поверхности внутри шага.
    if (H_next <= 0.0L && V_start < 0.0L) {
        const long double tau = -H_start / V_start;

        t_curr += tau;
        H_curr = 0.0L;
        V_curr = V_start + a * tau;
        finished = true;
        return;
    }

    H_curr = H_next;
    V_curr = V_next;
    t_curr += step_dt;
}



//==========================геттеры и сеттеры==========================
long double imitation::get_t() const{
    return t_curr;
}

long double imitation::get_H() const{
    return H_curr;
}

long double imitation::get_V() const{
    return V_curr;
}

long double imitation::get_t_on() const {
    return t_on;
}

long double imitation::get_a() const {
    return (t_curr >= t_on) ? (P_max/m - g) : -g;
}

long double imitation::get_H_exact(const long double t) const{

    if (t <= t_on) {
        return H_0 + V_0 * t - g * t * t / 2.0L;
    }

    const long double H_on =
        H_0 + V_0 * t_on - g * t_on * t_on / 2.0L;

    const long double V_on = V_0 - g * t_on;
    const long double tau = t - t_on;
    const long double a = P_max / m - g;

    return H_on + V_on * tau + a * tau * tau / 2.0L;
}

void imitation::set_t_on(long double my_t_on) {
    t_on = my_t_on;
}
//====================================================================



SimulationResult imitation::simulate_rk4(const std::size_t max_steps)
{
    finished = false;

    H_curr = H_0;
    V_curr = V_0;
    t_curr = 0.0L;


    //проверка корректности начальных условий
    if (!std::isfinite(H_0) || !std::isfinite(V_0) ||
        !std::isfinite(m) || !std::isfinite(P_max) ||
        !std::isfinite(g) || !std::isfinite(dt) ||
        !std::isfinite(t_on) ||
        H_0 <= 0.0L || m <= 0.0L || P_max < 0.0L ||
        g <= 0.0L || dt <= 0.0L || t_on < 0.0L ||
        max_steps == 0) {
        finished = true;
        return SimulationResult::Over;
    }


    const long double velocity_tolerance = 1e-3L;   //допустимая скорость
    const long double epsilon = 1e-12L;             //очень малое число

    for (std::size_t i = 0; i < max_steps; ++i) {
        const long double H_start = H_curr;
        const long double V_start = V_curr;
        const long double t_start = t_curr;

        long double step_dt = dt;

        // Не перескакиваем момент включения двигателя.
        if (t_curr < t_on && t_curr + step_dt > t_on) {
            step_dt = t_on - t_curr;
        }

        // Ускорение до и после включения двигателя.
        const long double a = (t_curr >= t_on) ? (P_max / m - g) : -g;

        const long double V_next = V_start + a * step_dt;
        const long double H_next = H_start + V_start * step_dt + 0.5L * a * step_dt * step_dt;

        // Определяем момент остановки при движении вниз.
        long double tau_stop = -1.0L;

        if (V_start < 0.0L && V_next >= 0.0L && a > 0.0L) {
            tau_stop = -V_start / a;
        }

        // Определяем момент пересечения поверхности.
        long double tau_contact = -1.0L;

        if (H_start > 0.0L && H_next <= 0.0L) {
            // Если ускорение нулевое
            if (std::abs(a) < epsilon) {
                if (V_start < 0.0L) {
                    tau_contact = -H_start / V_start;
                }
            }
            // Если ускорение ненулевое
            else {
                const long double D =
                    V_start * V_start - 2.0L * a * H_start;

                if (D >= 0.0L) {
                    const long double sqrt_D = std::sqrt(D);
                    const long double tau1 =
                        (-V_start - sqrt_D) / a;
                    const long double tau2 =
                        (-V_start + sqrt_D) / a;

                    // Важно выбрать наименьший положительный корень
                    if (tau1 >= 0.0L && tau1 <= step_dt) {
                        tau_contact = tau1;
                    }
                    if (tau2 >= 0.0L && tau2 <= step_dt &&
                        (tau_contact < 0.0L || tau2 < tau_contact)) {
                        tau_contact = tau2;
                    }
                }
            }
        }

        // Обрабатываем первое событие, если оно произошло в шаге.

        // Контакт
        if (tau_contact >= 0.0L && (tau_stop < 0.0L || tau_contact <= tau_stop)) {
            // Обработка времени контакта, высоты и скорости
            t_curr = t_start + tau_contact;
            H_curr = 0.0L;
            V_curr = V_start + a * tau_contact;
            finished = true;

            // Если скорость в момент контакта допустима
            if (std::abs(V_curr) <= velocity_tolerance) {
                V_curr = 0.0L;
                return SimulationResult::Good;
            }

            // Скорость во время контакта не допустима
            return SimulationResult::Crashed;
        }

        // Остановка
        if (tau_stop >= 0.0L && tau_stop <= step_dt) {
            t_curr = t_start + tau_stop;
            H_curr = H_start + V_start * tau_stop + 0.5L * a * tau_stop * tau_stop;
            V_curr = 0.0L;
            finished = true;

            // Если остановились слишком высоко (преждевременно)
            if (H_curr > 0.0L) {
                return SimulationResult::Stopped;
            }

            // Если остановились на допустимой высоте
            H_curr = 0.0L;
            return SimulationResult::Good;
        }

        // Событий в текущем шаге нет — завершаем обычный шаг.
        H_curr = H_next;
        V_curr = V_next;
        t_curr = t_start + step_dt;


        // Проверка: не возникли ли некорректные значения
        if (!std::isfinite(H_curr) || !std::isfinite(V_curr) ||
            !std::isfinite(t_curr)) {
            finished = true;
            return SimulationResult::Over;
        }
    }

    // Превышение допустимого количества итераций
    finished = true;
    return SimulationResult::Over;
}


long double imitation::find_t_on()
{
    const std::size_t steps = 10000;    //допустимое количество итерация для поиска
    long double left_t = 5.0L;          //левая граница поиска
    long double right_t = 100.0L;       //правая граница поиска

    // Проверяем границы поиска.
    set_t_on(left_t);
    const SimulationResult left_result = simulate_rk4(steps);

    set_t_on(right_t);
    const SimulationResult right_result = simulate_rk4(steps);

    /*
        По нашей логике важно, чтобы при значение левой граница аппарат
        останавливался раньше нужного, а при правой - разбивался.
        Если же будет так, что обе границы возвращают один исход,
        значит что мы ищем не в том диапазоне
     */

    if (left_result != SimulationResult::Stopped || right_result != SimulationResult::Crashed) {
        return -1.0L;
    }

    /* Ищем границу между остановкой над поверхностью и столкновением с поверхностью
       при помощи алгоритма бинарного поиска. Это позволит найти нужный результат за
       время O(logN), в отличии от обычного перебора с линейной O(n)
    */
    for (int i = 0; i < 60; ++i) {
        const long double middle_t = (left_t + right_t) / 2.0L;

        set_t_on(middle_t);
        const SimulationResult result = simulate_rk4(steps);

        // Проверка результата симуляции среднего значения t_on между границами
        if (result == SimulationResult::Good) {
            // Повторно проверяем найденное время.
            set_t_on(middle_t);
            const SimulationResult verification = simulate_rk4(steps);

            // Если значение оказалось подходящим
            if (verification == SimulationResult::Good) {
                return middle_t;
            }

            return -1.0L;
        }

        // Сдвиг границ
        if (result == SimulationResult::Stopped) {
            left_t = middle_t;
        } else if (result == SimulationResult::Crashed) {
            right_t = middle_t;
        } else {
            return -1.0L;
        }
    }

    return -1.0L;
}