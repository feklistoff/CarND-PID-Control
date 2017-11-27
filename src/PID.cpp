#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

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
    num_steps_whole_track_ = 800;         // changes based on desired speed/throttle (for ex. if throttle 0.3 num = 1000)
    dp_ = {0.05*Kp_, 0.05*Ki_, 0.05*Kd_}; // how much to change each parameter during twiddling
    gains_ = {Kp_, Ki_, Kd_};             // save gains for convenience
    what_K_ = 0;                          // choose what parameter to tune

    std::cout << "Initial gains" << std::endl;
    std::cout << "Kp = " << Kp_ << " Ki = " << Ki_ << " Kd = " << Kd_ << std::endl;
}

void PID::UpdateError(double cte)
{
    // first timestep d_error should be 0
    if (counter_ == 1) p_error_ = cte;

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
    // twiddle every N timesteps
    if (counter_ % (num_steps_ + num_steps_whole_track_) == 0)
    {
        if (twiddle_)
        {
            if (dp_[0] + dp_[1] + dp_[2] > threshold_)
            {
                if (best_err_ == 100000)
                {
                    // need to init best_err
                    twiddle_state_ = 0;
                }
                else if (twiddle_state_ == 0) 
                {
                    // best initiated, need to add dp and tune next parameter
                    twiddle_state_ = 1;
                }
                else if (err_ < best_err_)
                {
                    // prepare for tuning next parameter
                    twiddle_state_ = 2;
                }
                else
                {
                    if (twiddle_state_ == 5) // added dp
                    {
                        // need to subtruct dp
                        twiddle_state_ = 3;
                    }
                    else
                    {
                        // already subtracted, need to decrease dp
                        twiddle_state_ = 4;
                    }
                }

                switch (twiddle_state_)
                {
                    case 0:
                        // init best error
                        best_err_ = err_;
                        std::cout << "best err = " << best_err_ << std::endl;                        
                        err_ = 0;
                        std::cout << "best error initiated" << std::endl;
                        counter_--; // to start updating next parameter next time step 
                        break;
                    
                    case 1:
                        // add dp
                        err_ = 0;
                        std::cout << "added dp " << dp_[what_K_] << std::endl;
                        gains_[what_K_] += dp_[what_K_];
                        
                        // update gain parameter
                        if (what_K_ == 0) Kp_ = gains_[what_K_];
                        if (what_K_ == 1) Ki_ = gains_[what_K_];
                        if (what_K_ == 2) Kd_ = gains_[what_K_];
                        
                        twiddle_state_ = 5; // added dp
                        break;
                    
                    case 2:
                        // err < best_err
                        best_err_ = err_;
                        err_ = 0;
                        std::cout << "best err = " << best_err_ << std::endl;
                        
                        std::cout << "increased dp" << std::endl;
                        dp_[what_K_] *= 1.1;

                        std::cout << "next parameter to tune"<< std::endl;  
                        
                        // update gain parameter
                        if (what_K_ == 0) Kp_ = gains_[what_K_];
                        if (what_K_ == 1) Ki_ = gains_[what_K_];
                        if (what_K_ == 2) Kd_ = gains_[what_K_];
                        
                        // next parameter index
                        what_K_ = (what_K_ + 1) % 3;
                        counter_--; // to start updating next parameter next time step
                        twiddle_state_ = 0; // need to start tuning next parameter
                        break;
                            
                    case 3:
                        // err >= best_err and need to subtruct
                        err_ = 0;
                        gains_[what_K_] -= 2 * dp_[what_K_];
                        std::cout << "subtracted dp " << dp_[what_K_] << std::endl;
                        
                        // update gain parameter
                        if (what_K_ == 0) Kp_ = gains_[what_K_];
                        if (what_K_ == 1) Ki_ = gains_[what_K_];
                        if (what_K_ == 2) Kd_ = gains_[what_K_];
                        
                        twiddle_state_ = 6; // subtracted
                        break;
                    
                    case 4:
                        // err >= best_err and need to decrease dp
                        err_ = 0;
                        gains_[what_K_] += dp_[what_K_]; // return to initial gain (we already subtracted)

                        // update gain parameter
                        if (what_K_ == 0) Kp_ = gains_[what_K_];
                        if (what_K_ == 1) Ki_ = gains_[what_K_];
                        if (what_K_ == 2) Kd_ = gains_[what_K_];

                        std::cout << "returned to initial gain parameter and decreased dp" << std::endl;
                        dp_[what_K_] *= 0.9;
                        
                        std::cout << "next parameter to tune"<< std::endl;                
                        // next parameter index
                        what_K_ = (what_K_ + 1) % 3;
                        counter_--; // to start updating next parameters next time step
                        twiddle_state_ = 0; // need to start tuning next parameter
                        break;
                }
                std::cout << "Kp = " << Kp_ << " Ki = " << Ki_ << " Kd = " << Kd_ << std::endl;
            }
        }
    }
}
