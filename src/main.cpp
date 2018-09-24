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

int iteration = 0;
int p_index = 0;
double total_cte = 0;
int twiddle_state = 0;
int twiddle_iteration = 0;
int n = 150;
std::vector<double> p, dp;
double best_error = 10000;
bool twiddle_on = false;
bool manual_throttle = false;
double dthrottle = 0.0; 

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char* argv[])
{
  uWS::Hub h;
  
  PID pid_steer_control;
  PID pid_speed_control;

  for(int i = 0;i <= 3; i++)
  {
    p.push_back(0);
    dp.push_back(0.1);
  }
  
  // TODO: Initialize the pid variable, from twiddle
  
  
  if (argc == 2)
  {
    std::cout << "twiddle for PID steer control is on..." << std::endl;
    twiddle_on = true;
  }
  else if (argc == 4)
  {
      std::cout << "Manual PID settings..." << std::endl;
      pid_steer_control.Init(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));
  }
  else if (argc == 5)
  {
      std::cout << "Manual PID settings and fixed throttle..." << std::endl;
      pid_steer_control.Init(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));
      dthrottle = std::stod(argv[4]);
      manual_throttle = true;
  }
  else
  {
      std::cout << "Standard values for steering and speed..." << std::endl;
      pid_steer_control.Init(0.174, 0.0006, 0.9097);
      pid_speed_control.Init(0.01, 0.0, 0.15);
  }
    
  h.onMessage([&pid_steer_control, &pid_speed_control](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, speed_reference = 50, speed_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          // update the CTE
          pid_steer_control.UpdateError(cte);
          
          if(!twiddle_on)
          {
            // get the total error and process it for steering
            steer_value = pid_steer_control.TotalError();
            steer_value = (steer_value < -1.0) ? -1 : steer_value;
            steer_value = (steer_value > 1.0) ? 1 : steer_value;
            
            // process speed
            pid_speed_control.UpdateError(speed - speed_reference);
            speed_value = pid_speed_control.TotalError();
        
          }
          
        if(twiddle_on)
        {
            speed_value = 0.3;
            // twiddle
            if(iteration == 0)
            {
                // reset index to zero, increment twiddle iteration
                if(p_index>2)
                {
                    p_index = 0;
                    twiddle_iteration++;
                }
                
                // if in state 0, add delta
                if(twiddle_state == 0)
                {
                    p[p_index] += dp[p_index];
                }
            }          
          
            steer_value = pid_steer_control.twiddle_error(p);
            steer_value = (steer_value < -1.0) ? -1 : steer_value;
            steer_value = (steer_value > 1.0) ? 1 : steer_value;
            /*
            pid_speed_control.UpdateError(speed - speed_reference);
            speed_value = pid_speed_control.twiddle_error(p);
            */
            iteration++;
 
            if(iteration > n)
            {
            total_cte += cte*cte;
            }
          
            if(iteration == 2*n)
            {
                if(twiddle_state == 0)
                {
                    // if new value caused lower error...
                    if(total_cte < best_error)
                    {
                        best_error = total_cte;
                        dp[p_index] *= 1.1;
                        // go to the next parameter
                        p_index++;
                        twiddle_state = 0;
                    }
                    // if it didn't...
                    else
                    {
                        // subtract two times the initial value and try it from the other side...
                        p[p_index] -= 2*dp[p_index];
                        // set next level...
                        twiddle_state++;
                    }
                    // reset error
                    total_cte = 0;
                    // reset iterations
                    iteration = 0;
                    // reset all errors in the controller, or I error gets huge
                    pid_steer_control.reset_errors();
                    
                    // restart the car
                    std::string msg = "42[\"reset\", {}]";
                    std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
                // state 1
                else
                {
                    // if the new setting caused better error...
                    if(total_cte < best_error)
                    {
                        best_error = total_cte;
                        dp[p_index] *= 1.1;
                        twiddle_state = 0;
                        p_index++;
                    }
                    // if it didn't, start over and reduce the delta
                    else
                    {
                        p[p_index] += dp[p_index];
                        dp[p_index] *= 0.9;
                        twiddle_state = 0;
                        p_index++;
                    }
                    total_cte = 0;
                    iteration = 0;
                    pid_speed_control.reset_errors();
                    pid_steer_control.reset_errors();
                  
                    std::string msg = "42[\"reset\", {}]";
                    std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
          
          
        std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
        std::cout << "Total CTE: " << total_cte << " Best Error: " << best_error <<  "Twiddle Iteration: " << twiddle_iteration << std::endl;
        std::cout << "p0: " << p[0] << " p1: " << p[1] << " p2: " << p[2] << std::endl;
        std::cout << "p_error: " << pid_steer_control.p_error << " i_error: " << pid_steer_control.i_error << " d_error: " << pid_steer_control.d_error << std::endl;
        }
          
        json msgJson;
        msgJson["steering_angle"] = steer_value;
        msgJson["throttle"] = manual_throttle ? dthrottle : speed_value;
        auto msg = "42[\"steer\"," + msgJson.dump() + "]";
        std::cout << msg << std::endl;
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
