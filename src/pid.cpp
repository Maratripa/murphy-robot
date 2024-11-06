#include "pid.h"

PID::PID(double Kp, double Ki, double Kd, double Kw, double vmax) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _Kw = Kw;
    _vmax = vmax;

    _tolerance = 0;

    _u = 0;
    _u_sat = 0;

    _setPoint = 0;
    _integral = 0;
    _prevError = 0;
    _time = millis();
}

PID::PID(double Kp, double Ki, double Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
    _Kw = 0;
    _vmax = INFINITY;

    _tolerance = 0;

    _u = 0;
    _u_sat = 0;

    _setPoint = 0;
    _integral = 0;
    _prevError = 0;
    _time = millis();
}

void PID::setPoint(double reference) {
    _setPoint = reference;
    _integral = 0;
    _u = 0;
    _u_sat = 0;
    _prevError = 0;
    _time = millis();
}

void PID::setTolerance(double tolerance) {
    _tolerance = abs(tolerance);
}

double PID::update(double point) {
    double error = _setPoint - point;
    double dt = (millis() - _time) / 1000.0;

    if (abs(error) < _tolerance) {
        _prevError = error;
        _time = millis();
        return 0;
    }

    if (dt == 0) return _u_sat;

    _integral += _Ki * error * dt + _Kw * (_u_sat - _u) * dt;

    _u = (_Kp * error) + (_integral) + (_Kd * (error - _prevError) / dt);

    if (_u < 0) {
        _u_sat = max(-_vmax, _u);
    } else {
        _u_sat = min(_vmax, _u);
    }

    _prevError = error;
    _time = millis();

    return _u_sat;
}

double PID::getPrevError() {
    return _prevError;
}