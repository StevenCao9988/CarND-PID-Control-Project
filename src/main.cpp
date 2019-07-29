#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;
  /**
   * TODO: Initialize the pid variable.
   */
  PID pid_s;
  //pid_s.Init(0.3, 0.0001, 3);                              // 调参1、手动调出较优的参数 
  //pid_s.Init(0.274727, 0.000110003, 3.9361);               // 调参2、 0.3油门下 twiddle 修正的参数 
  // pid_s.Init(0.100415, 0.000110003, 5.00585);             // 调参5、手动调参
  pid_s.Init(0.100234, 0.000110018, 5.00585);                // 调参6、twiddle 修正的参数
  pid_s.twiddle_flg = 0;  // 进行参数调节  0:不调节   1：自动调节  一次只能调节一组

  PID pid_t; // 油门调节
  //pid_t.Init(1, 0, 3);                                    // 调参3、转向 手动调出较优的参数
  pid_t.Init(1.02551, 0, 3.32622);                          // 调参4、twiddle 修正的参数 
  pid_t.twiddle_flg = 0;  // 进行参数调节  0:不调节   1：自动调节

  h.onMessage([&pid_s, &pid_t](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
		  double throttle;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
		  pid_s.UpdateError(cte);
		  steer_value = pid_s.TotalError();

		  pid_t.UpdateError(fabs(cte));
		  throttle = pid_t.TotalError();
		  if (throttle > 0)
		  {
			  throttle = 0;
		  }

		  throttle = throttle + 1;
		  if (throttle < 0)
		  {
			  throttle = 0;
		  }

		  if (throttle > 1)
		  {
			  throttle = 1;
		  }

          // DEBUG
		  if ((pid_t.twiddle_flg == 0) && (pid_s.twiddle_flg == 0))
		  {
			  std::cout << "Steer value breakdown: " << std::endl;
			  std::cout << " P: " << -pid_s.Kp * pid_s.p_error;
			  std::cout << " I: " << -pid_s.Ki * pid_s.i_error;
			  std::cout << " D: " << -pid_s.Kd * pid_s.d_error << std::endl;
		  }


		  if ((pid_t.twiddle_flg == 0) && (pid_s.twiddle_flg == 0))
		  {
			  std::cout << "throttle: ";
			  std::cout << " P: " << -pid_t.Kp * pid_t.p_error;
			  std::cout << " I: " << -pid_t.Ki * pid_t.i_error;
			  std::cout << " D: " << -pid_t.Kd * pid_t.d_error << std::endl;
		  }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
		  if ((pid_t.twiddle_flg == 0) && (pid_s.twiddle_flg == 0))
		  {
			  std::cout << msg << std::endl;
		  }
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}