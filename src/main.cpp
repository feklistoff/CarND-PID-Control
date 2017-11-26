#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) 
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) 
    {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) 
    {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub h;

    PID pid;
    // Initialize the pid variable
    // pid.Init(0.2, 0.004, 3.0); // speed 30mph ! set throttle 0.3 | initial parameters that work well (were given in lessons)
    pid.Init(0.16, 0.00252, 2.1); // speed 50 mph ! set throttle 0.5

    // set throttle according chosen pid initial parameters!!!
    double throttle = 0.55;

    h.onMessage([&pid, &throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) 
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") 
            {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") 
                {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    // double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    double steer_value;
                
                    pid.UpdateError(cte);
                    steer_value = pid.TotalError();
                    
                    // steering value is [-1, 1]
                    if (steer_value < - 1) steer_value = -1;
                    if (steer_value > 1) steer_value = 1;

                    pid.counter_++;
                    
                    // twiddle every N timesteps
                    if (pid.counter_ % (pid.num_steps_ + pid.num_steps_whole_track_) == 0)
                    {
                        if (pid.twiddle_)
                        {
                            if (pid.dp_[0] + pid.dp_[1] + pid.dp_[2] > pid.threshold_)
                            {
                                pid.Twiddler();
                                std::cout << "Kp = " << pid.Kp_ << " Ki = " << pid.Ki_ << " Kd = " << pid.Kd_ << std::endl;                          
                            }
                        }
                    }
                                        
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;

                    // control throttle according to how much we turn (more we turn slower we drive)
                    if (deg2rad(angle) <= -0.4 and deg2rad(angle) >= 0.4) throttle *= (1 - abs(deg2rad(angle)));                    
                    
                    msgJson["throttle"] = throttle;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);                    
                }
            } 
            else 
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) 
    {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1)
        {
            res->end(s.data(), s.length());
        }
        else
        {
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) 
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
    {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    
    h.run();
}
