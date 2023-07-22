/// !!!!!! ONLY WORKS WITH NORMALIZED LOOP TIME
class PID {

  double p, i, d, max_error;
  double sum_i = 0, last_error = 0, T;

public:
  PID(double P, double I, double D, double Max_error, double loopTime = 1) {
    p = P;
    i = I;
    d = D;
    max_error = Max_error;
    T = loopTime;
  }

  double calculate(double value, double setPoint) {

    double error = setPoint - value;

    sum_i += error;
    sum_i = max(sum_i, -max_error);
    sum_i = min(sum_i, max_error);

    double delta_error = error - last_error;
    last_error = error;

    double ans = p * error + i * sum_i * T + d * delta_error / T;
    ans = max(ans, -max_error);
    ans = min(ans, max_error);

    return ans;

  }

};