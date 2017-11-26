#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    p_error_ = 0;
    i_error_ = 0;
    d_error_ = 0;

    // twiddler update settings
    counter_ = 0;
    twiddle_ = false;      // set to 'true'/'false' if you want to turn twiddler on/off
    threshold_ = 0.02;     // when to stop twiddling
    best_err_ = 100000;    // some big number for a start
    err_ = 0;
    num_steps_ = 200;
    num_steps_whole_track_ = 600;         // changes based on desired speed/throttle (for ex. if throttle 0.3 num = 1000)
    dp_ = {0.05*Kp_, 0.05*Ki_, 0.05*Kd_}; // how much to change each parameter during twiddling
    gains_ = {Kp_, Ki_, Kd_};             // save gains for convenience
    what_K_ = 0;                          // choose what parameter to tune
    best_init_ = false;                   // to set best error for the first time
    already_added_ = false;
    already_subtracted_ = false;

    std::cout << "Initial gains" << std::endl;
    std::cout << "Kp = " << Kp_ << " Ki = " << Ki_ << " Kd = " << Kd_ << std::endl;
}

void PID::UpdateError(double cte)
{
    d_error_ = cte - p_error_;
    p_error_ = cte;
    i_error_ += cte;

    if (counter_ % (num_steps_ + num_steps_whole_track_) > num_steps_)
        err_ += pow(cte, 2);
}

double PID::TotalError()
{
    return -Kp_ * p_error_ - Ki_ * i_error_ - Kd_ * d_error_;
}

void PID::Twiddler()
{
    if (!best_init_)
    {
        best_err_ = err_;
        std::cout << "best err = " << best_err_ << std::endl;
        best_init_ = true;
        err_ = 0;
        std::cout << "best error initiated" << std::endl;
        counter_--; // to start updating next parameter next time step
    }
    else
    {
        if (!already_added_)
        {
            err_ = 0;
            std::cout << "added dp " << dp_[what_K_] << std::endl;
            gains_[what_K_] += dp_[what_K_];
            already_added_ = true;

            // update gain parameter
            if (what_K_ == 0) Kp_ = gains_[what_K_];
            if (what_K_ == 1) Ki_ = gains_[what_K_];
            if (what_K_ == 2) Kd_ = gains_[what_K_];
        }
        else
        {
            if (err_ < best_err_)
            {
                best_err_ = err_;
                err_ = 0;
                std::cout << "best err = " << best_err_ << std::endl;
                
                std::cout << "increased dp" << std::endl;
                dp_[what_K_] *= 1.1;

                std::cout << "next parameter to tune"<< std::endl;  
                already_added_ = false;
                already_subtracted_ = false;
                
                // update gain parameter
                if (what_K_ == 0) Kp_ = gains_[what_K_];
                if (what_K_ == 1) Ki_ = gains_[what_K_];
                if (what_K_ == 2) Kd_ = gains_[what_K_];
                
                // next parameter index
                what_K_ = (what_K_ + 1) % 3;
                counter_--; // to start updating next parameter next time step
            }
            else
            {
                if (already_added_ and !already_subtracted_)
                {
                    err_ = 0;
                    gains_[what_K_] -= 2 * dp_[what_K_];
                    std::cout << "subtracted dp " << dp_[what_K_] << std::endl;
                    already_subtracted_ = true;

                    // update gain parameter
                    if (what_K_ == 0) Kp_ = gains_[what_K_];
                    if (what_K_ == 1) Ki_ = gains_[what_K_];
                    if (what_K_ == 2) Kd_ = gains_[what_K_];
                }
                else if (already_added_ and already_subtracted_)
                {
                    err_ = 0;
                    gains_[what_K_] += dp_[what_K_];

                    // update gain parameter
                    if (what_K_ == 0) Kp_ = gains_[what_K_];
                    if (what_K_ == 1) Ki_ = gains_[what_K_];
                    if (what_K_ == 2) Kd_ = gains_[what_K_];

                    std::cout << "returned to initial gain parameter and decreased dp" << std::endl;
                    dp_[what_K_] *= 0.9;
                    
                    std::cout << "next parameter to tune"<< std::endl;
                    already_added_ = false;
                    already_subtracted_ = false;
                    
                    // next parameter index
                    what_K_ = (what_K_ + 1) % 3;
                    counter_--; // to start updating next parameters next time step
                }
            }
        }
    }
}
