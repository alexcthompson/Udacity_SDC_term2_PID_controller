#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  std::vector<double> params;
  std::vector<double> last_params;

  // Twiddle?
  bool twiddle;

  // twiddle tracking
  int steps;
  int steps_to_run_straight;
  int steps_to_initialize;
  int steps_to_evaluate;
  std::vector<double> twiddle_amounts;
  double reduction_factor = 0.3;
  
  int index;
  double error_to_beat;
  bool error_initialized;
  double current_trun_error;

  // error for escalation
  double error_per_step_thresh;
  double per_step_improvement_factor;
  double graduating_error;
  double run_lenghtening_factor;

  bool new_run;
  bool was_a_param_updated;


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateErrorSimple(double cte);

  /*
  * Update the PID error and execute gnarly twiddle control flow
  */
  void UpdateErrorAndTwiddle(double cte);

  /*
  * Returns a steering ratio in [-1.0, 1.0] based on current error
  */
  double Steer();
};

#endif /* PID_H */
