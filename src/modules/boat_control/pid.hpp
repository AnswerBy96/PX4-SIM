#pragma once

class IncrementalPID {
public:
  IncrementalPID(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd),
      _prev_error(0.0), _prev_prev_error(0.0) {}
  IncrementalPID()=default;
  // 计算增量控制量
  float calculate(float target_value,float actual_value) {
	  float error = target_value - actual_value;
    float delta_u = _kp * (error - _prev_error)
                                    + _ki * error
                                    + _kd * (error - 2 * _prev_error + _prev_prev_error);

    // 更新历史误差
    _prev_prev_error = _prev_error;
    _prev_error = error;

    return delta_u;
  }

  void reset() {
    _prev_error = _prev_prev_error = 0.0;
  }

  void setParameters(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
  }

private:
  float _kp, _ki, _kd;
  float _prev_error{0.0};       // e(k-1)
  float _prev_prev_error{0.0};  // e(k-2)
};
