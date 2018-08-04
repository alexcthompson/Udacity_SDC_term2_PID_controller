#include <iostream>
#include <math.h>

#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    params = {Kp, Ki, Kd};
    last_params = params;

    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    steps = 0;

    steps_to_run_straight = 30;
    steps_to_initialize = 10;
    steps_to_evaluate = 40;
    twiddle_amounts = {params[0] * 0.5, params[1] * 0.5, params[2] * 0.5};
    reduction_factor = 0.5;

    index = 0;
    error_to_beat = steps_to_evaluate * 1000.0;
    error_initialized = false;

    error_per_step_thresh = 0.2;
    per_step_improvement_factor = 0.9;
    graduating_error = steps_to_evaluate * error_per_step_thresh;
    run_lenghtening_factor = 1.7;

    new_run = false;

}

void PID::UpdateErrorSimple(double cte) {
    cout << "steps = " << steps << endl;
    if (steps != 0) {
        d_error = cte - p_error;
    } else {
        // reinitialization
        d_error = 0.0;
        i_error = 0.0;
    }

    p_error = cte;
    i_error += cte;
    cout << p_error << ", " << i_error << ", " << d_error << endl;

    steps++;
}

void PID::UpdateErrorAndTwiddle(double cte) {
    if (steps != 0) {
        d_error = cte - p_error;
    } else {
        d_error = 0.0;
        i_error = 0.0;
    }

    p_error = cte;
    i_error += cte;
    // cout << "pid error = " << p_error << ", " << i_error << ", " << d_error << endl;

    // cout << "\n== " << steps << " steps ==" << endl;
    // accrue current error toward twiddle
    if (steps == 0) {
        current_trun_error = 0;

        if (!error_initialized) {
            last_params = params;
        }

        // identify when a new run is starting
        if (new_run) {
            new_run = false;
            // check if a param was updated, if not, reset the best error, last_params = params
            if (!was_a_param_updated) {
                error_to_beat = 1000.0 * steps_to_evaluate;
                last_params = params;
            }

            was_a_param_updated = false;
        }
    }

    // add error if past initialization phase
    if (steps >= steps_to_run_straight + steps_to_initialize) {
        current_trun_error += cte * cte;
    }


    if (steps == steps_to_run_straight + steps_to_initialize + steps_to_evaluate - 1) {
        cout << "\n=== Run complete ===" << endl;
        cout << "\ncurrent best params = " << last_params[0] << ", " << last_params[1] << ", " << last_params[2] << endl;
        cout << "candidate params    = " << params[0] << ", " << params[1] << ", " << params[2] << endl;
        cout << "Steps evaluated     = " << steps_to_evaluate << endl;
        cout << "Error on new params = " << current_trun_error << endl;
        cout << "Error per step      = " << current_trun_error / steps_to_evaluate << endl;
        cout << "Graduating error    = " << graduating_error << endl;
        cout << "Error to beat       = " << error_to_beat << endl;

        bool do_param_update = true;

        if (current_trun_error < error_to_beat) {
            // check if these are different params, and if so, note that a param was updatde
            if (last_params != params) {
                was_a_param_updated = true;
            }

            error_to_beat = current_trun_error;
            last_params = params;

            // if this was not an initial run, increment the index
            if (error_initialized) {
                index = (index + 1) % 3;
            }
            else {
                error_initialized = true;
            }

            // this was a good enough run to graduate
            if (error_to_beat < graduating_error) {
                cout << "\n==UPGRADE!  TRY A LONGER RUN. ==\n";

                // increase initialization interval
                steps_to_initialize *= run_lenghtening_factor;
                cout << "steps_to_initialize   = " << steps_to_initialize + steps_to_run_straight << endl;

                // increase evaluation interval
                steps_to_evaluate *= run_lenghtening_factor;
                cout << "steps_to_evaluate     = " << steps_to_evaluate << endl;

                // decrease error tolerance per step
                error_per_step_thresh *= per_step_improvement_factor;
                cout << "error_per_step_thresh = " << error_per_step_thresh << endl;

                // define a new threshold for graduation
                graduating_error = steps_to_evaluate * error_per_step_thresh;
                cout << "graduating_error      = " << graduating_error << endl;

                // set this as the new error to beat
                error_to_beat = 1000.0 * steps_to_evaluate;
                cout << "error_to_beat         = " << error_to_beat << endl << endl;

                // don't update params
                do_param_update = false;

                // make all params positive as a reset of twiddle
                twiddle_amounts[0] = fabs(twiddle_amounts[0]);
                twiddle_amounts[1] = fabs(twiddle_amounts[1]);
                twiddle_amounts[2] = fabs(twiddle_amounts[2]);
            }
        }
        else {
            // restore the old params
            params = last_params;
            // cout << "\nRestored params = " << params[0] << ", " << params[1] << ", " << params[2] << endl;

            // failed positive increment
            if (twiddle_amounts[index] > 0) {
                twiddle_amounts[index] *= -1;
            } // failed negative increment
            else {
                twiddle_amounts[index] *= -1 * reduction_factor;
                index = (index + 1) % 3;

                if (index == 0) {
                    new_run = true;
                    if (!was_a_param_updated) do_param_update = false;
                }
            }
        }
        // update the params
        if (do_param_update) params[index] += twiddle_amounts[index];
        // cout << "\nNew params = " << params[0] << ", " << params[1] << ", " << params[2] << endl;
    }
    steps++;
}

double PID::Steer() {
    // steering is initially in radians
    // cout << "-Kp * p_error = " << -params[0] * p_error << endl;
    // cout << "-Ki * i_error = " << -params[1] * i_error << endl;
    // cout << "-Kd * d_error = " << -params[2] * d_error << endl;

    double raw_steer = -params[0] * p_error - params[1] * i_error - params[2] * d_error;
    // cout << "raw_steer = " << raw_steer << endl;

    // limit steering angle to +/- 0.4363323 radians, which is 25.0 degrees
    if (raw_steer > 0.4363323) raw_steer = 0.4363323;
    else if (raw_steer < -0.4363323) raw_steer = -0.4363323;

    return raw_steer / 0.4363323;
}
