#include "PID.h"
#include <algorithm>
#include <iostream>
#include <iomanip>

using std::cout;
using std::endl;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    saturation = 1000.0;
}

void PID::set_saturation(double satval) {
    saturation = satval;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    i_error += cte;
    i_error = std::max(-1.0*saturation, i_error);
    i_error = std::min(saturation, i_error);
    p_error = cte;
}

double PID::TotalError() {
    return Kp*p_error + Ki*i_error + Kd*d_error;
}

void PID::print_error() {
    cout << "errors, " << std::fixed << std::setprecision(3);
    cout << std::setw(8) << p_error << ", ";
    cout << std::setw(8) << i_error << ", ";
    cout << std::setw(8) << d_error << "| ";
    cout << std::setw(8) << Kp*p_error << ", ";
    cout << std::setw(8) << Ki*i_error << ", ";
    cout << std::setw(8) << Kd*d_error << endl;
}
