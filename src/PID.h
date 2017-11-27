#include <vector>

#ifndef PID_H
#define PID_H

class PID 
{
    public:
        /*
        * Errors
        */
        double p_error_;
        double i_error_;
        double d_error_;
        
        /*
        * Coefficients
        */ 
        double Kp_;
        double Ki_;
        double Kd_;

        // on-line update parameters
        int counter_;
        bool twiddle_;
        double threshold_;
        double best_err_;
        double err_;
        int num_steps_;
        int num_steps_whole_track_;
        std::vector<double> dp_;
        std::vector<double> gains_;
        int what_K_;
        int twiddle_state_;
        
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
        void UpdateError(double cte);

        /*
        * Calculate the total PID error.
        */
        double TotalError();

        /*
        * Update parameters Kx and find best
        */
        void Twiddler();

};

#endif /* PID_H */
