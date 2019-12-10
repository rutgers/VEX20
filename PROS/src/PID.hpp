class PID
{
private:
    double kp;
    double ki;
    double kd;

    double last_err;
    double target;
    double error_sum;

public:
    PID(double p, double i, double d);
    void setCoeffs(double p, double i, double d);

    int get_kp();
    int get_ki();
    int get_kd();
    int get_target();

    void update_target(double new_target);
    double update(double measure, double dt);
};
