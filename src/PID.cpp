#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    p_error = 0;
    i_error = 0;
    d_error = 0;

}

void PID::UpdateError(double cte) {
    double prev_cte = p_error;
    p_error = cte;
    d_error = cte - prev_cte;
    i_error += cte;
}

double PID::TotalError() {
    return -Kp_ * p_error - Ki_ * i_error - Kd_ * d_error;
}

double PID::twiddle_error(vector<double> p){
    return -p[0] * p_error - p[1] * d_error - p[2] * i_error;
    
}

void PID::reset_errors() {
    p_error = 0;
    i_error = 0;
    d_error = 0;
}