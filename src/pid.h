#include <Arduino.h>

class PID {
    public:
        PID(double Kp, double Ki, double Kd, double Kw, double vmax);
        PID(double Kp, double Ki, double Kd);

        void setPoint(double reference);

        void setTolerance(double tolerance);

        double update(double point);

        double getPrevError();

    private:
        double _Kp;
        double _Ki;
        double _Kd;
        double _Kw;
        double _vmax;

        double _tolerance;

        double _u;
        double _u_sat;

        double _setPoint;
        double _integral;
        double _prevError;
        unsigned long _time;
};