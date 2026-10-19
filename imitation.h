#ifndef PROJECT_MOON
#define PROJECT_MOON
#include <cstddef>

class imitation {
private:
    long double S_0;
    long double S_curr;
    long double V_0;
    long double V_curr;
    long double a;

    std::size_t step_count = 0;
    long double t_curr;
    long double t_step;
    long double t_max;


public:

    //params init
    imitation(const long double _S_0,const long double _V_0,const long double _a,const long double _t_step,const long double _t_max) :
    S_0(_S_0), S_curr(_S_0),
    V_0(_V_0), V_curr(_V_0),
    a(_a),
    t_curr(0.0L), t_step(_t_step), t_max(_t_max){}

    //Euler Method
    void euler_step();

    //Runge-Kutta Method (RK4)
    void RK4();

    //getters
    long double get_t() const;
    long double get_tmax() const;
    long double get_S() const;
    long double get_V() const;
    long double get_S_exact() const;

    ~imitation() {}
};


#endif //PROJECT_MOON
