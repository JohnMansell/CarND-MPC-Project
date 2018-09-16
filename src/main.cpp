#include "json.hpp"
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"

using namespace std;
//---------------------
//    JSON
//---------------------
	using json = nlohmann::json;
	// Checks if the SocketIO event has JSON data.
	// If there is data the JSON object in string format will be returned,
	// else the empty string "" will be returned.
	string hasData(string s) {
		auto found_null = s.find("null");
		auto b1 = s.find_first_of("[");
		auto b2 = s.rfind("}]");
		if (found_null != string::npos) {
			return "";
		} else if (b1 != string::npos && b2 != string::npos) {
			return s.substr(b1, b2 - b1 + 2);
		}
		return "";
	}


//---------------------
//    Math
//---------------------
	constexpr double pi() { return M_PI; }
	double deg2rad(double x) { return x * pi() / 180; }
	double rad2deg(double x) { return x * 180 / pi(); }


//---------------------
//    Poly - Eval
//---------------------
	double polyeval(Eigen::VectorXd coeffs, double x) {
	  double result = 0.0;
	  for (int i = 0; i < coeffs.size(); i++) {
	    result += coeffs[i] * pow(x, i);
	  }
	  return result;
	}

//---------------------
//    Poly - Fit
//---------------------
	Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
	  assert(xvals.size() == yvals.size());
	  assert(order >= 1 && order <= xvals.size() - 1);
	  Eigen::MatrixXd A(xvals.size(), order + 1);

	  // Speed
		  for (int i = 0; i < xvals.size(); i++) {
		    A(i, 0) = 1.0;
		  }
	  // Way Points
		  for (int j = 0; j < xvals.size(); j++) {
		    for (int i = 0; i < order; i++) {
		      A(j, i + 1) = A(j, i) * xvals(j);
		    }
		  }

	  auto Q = A.householderQr();
	  auto result = Q.solve(yvals);
	  return result;
	}


//===========================
//      Main
//===========================

int main() {
  uWS::Hub h;

	//---------------------
	//    MPC - Init
	//---------------------
		MPC mpc;


	//---------------------
	//    Web Socket
	//---------------------
	    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
	                     uWS::OpCode opCode) {
	    // "42" at the start of the message means there's a websocket message event.
	    // The 4 signifies a websocket message
	    // The 2 signifies a websocket event
	    string sdata = string(data).substr(0, length);
	    //cout << sdata << endl;
	    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
	        string s = hasData(sdata);
	        if (s != "") {
	        auto j = json::parse(s);
	        string event = j[0].get<string>();
	        if (event == "telemetry") {
	          // j[1] is the data JSON object

        //---------------------
        //    Current State -- From Simulator
        //---------------------
            vector<double> ptsx = j[1]["ptsx"];
            vector<double> ptsy = j[1]["ptsy"];
            double px = j[1]["x"];
            double py = j[1]["y"];
            double psi = j[1]["psi"];
            double v = j[1]["speed"];

            // Transformation about origin
	            for (size_t i = 0; i < ptsx.size(); i++)
	            {
	                // Move (x,y) to origin
	                    double shift_x = ptsx[i] - px;
	                    double shift_y = ptsy[i] - py;

	                // Rotate by 90
	                    ptsx[i] = (shift_x * cos(0-psi) - shift_y * sin(0-psi));
	                    ptsy[i] = (shift_x * sin(0-psi) + shift_y * cos(0-psi));
	            }


            // Convert to Vector
                double* ptrx = &ptsx[0];
	            Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

	            double* ptry = &ptsy[0];
	            Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

            // Solve for Coefficients
	            auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

            // Calculate Error
	                double cte = polyeval(coeffs, 0);
	                double epsi = -atan(coeffs[1]);

                double steer_value = j[1]["steering_angle"];
                double throttle_value = j[1]["throttle"];

            // State Vector -- < x, y, theta, v, cte, epsi >
                Eigen::VectorXd state(6);
                state << 0, 0, 0, v, cte, epsi;

                auto vars = mpc.Solve(state, coeffs);


    //----------------------
    //      Controls
    //----------------------

        // Way Points - reference line
	        vector<double> next_x_vals;
	        vector<double> next_y_vals;

	        double poly_inc = 2.5;
	        int num_points = 25;

	        for (int i=1; i < num_points; i++)
	        {
		        next_x_vals.push_back(poly_inc * i);
		        next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
	        }

        // Predicted Path
            vector<double> mpc_x_vals;
            vector<double> mpc_y_vals;

            for (size_t i=2; i < vars.size(); i++)
            {
		        if (i%2 == 0) {
		        	mpc_x_vals.push_back(vars[i]);
		        }

		        else {
		        	mpc_y_vals.push_back(vars[i]);
		        }
            }

            double Lf = 2.67;




        //----------------------
        //      Send to Simulator
        //----------------------
            json msgJson;

            // Controls
		        msgJson["steering_angle"] = vars[0]/(deg2rad(25) * Lf);
		        msgJson["throttle"] = vars[1];

	        // Reference Path
		        msgJson["next_x"] = next_x_vals;
		        msgJson["next_y"] = next_y_vals;

	        // Predicted Path
	            msgJson["mpc_x"] = mpc_x_vals;
	            msgJson["mpc_y"] = mpc_y_vals;


            // Build JSON string
	            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
	           // std::cout << msg << std::endl;

            // Latency
	            // The purpose is to mimic real driving conditions where
	            // the car does actuate the commands instantly.
	            //
	            // Feel free to play around with this value but should be to drive
	            // around the track with 100ms latency.
	            //
	            // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
	            // SUBMITTING.

            // Send to Simulator
	            //this_thread::sleep_for(chrono::milliseconds(100));
	            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }

        } else {
	        // Manual driving
		        std::string msg = "42[\"manual\",{}]";
		        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

	//---------------------
	//    Don't Change
	//---------------------
	  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
	                     size_t, size_t) {
	    const std::string s = "<h1>Hello world!</h1>";
	    if (req.getUrl().valueLength == 1) {
	      res->end(s.data(), s.length());
	    } else {
	      // i guess this should be done more gracefully?
	      res->end(nullptr, 0);
	    }
	  });

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
