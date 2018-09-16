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
		Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
		{
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

	//---------------------
	//    Evaluate CTE
	//---------------------
		double evaluateCte (Eigen::VectorXd coeffs)
		{
			return polyeval(coeffs, 0);
		}

	//---------------------
	//    Evaluate EPSI
	//---------------------
		double evaluateEpsi (Eigen::VectorXd coeffs)
		{
			return -atan(coeffs[1]);
		}

	//---------------------
	//    Transform Points
	//      -- Global -> local
	//---------------------
		Eigen::MatrixXd transformGlobalToLocal(double x, double y, double psi, const vector<double> & ptsx, const vector<double> & ptsy)
		{
			assert(ptsx.size() == ptsy.size());
			unsigned len = ptsx.size();
			auto waypoints = Eigen::MatrixXd(2,len);

			for (auto i=0; i<len ; ++i){
				waypoints(0,i) =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
				waypoints(1,i) =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);
			}

			return waypoints;
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
		        Eigen::MatrixXd waypoints = transformGlobalToLocal(px,py,psi,ptsx,ptsy);
		        Eigen::VectorXd Ptsx = waypoints.row(0);
		        Eigen::VectorXd Ptsy = waypoints.row(1);

            // Solve for Coefficients
	            auto coeffs = polyfit(Ptsx, Ptsy, 3);

            // Calculate Error
	                double cte = evaluateCte(coeffs);
	                double epsi = evaluateEpsi(coeffs);

            // State Vector -- < x, y, theta, v, cte, epsi >
                Eigen::VectorXd state(6);
                state << 0, 0, 0, v, cte, epsi;

            // Solution
                auto vars = mpc.Solve(state, coeffs);

            // Controls
                double steer_value = vars.Delta.at(latency_ind);
                double throttle_value = vars.A.at(latency_ind);

            // Previous Values
                mpc.delta_prev = steer_value;
                mpc.a_prev = throttle_value;


        //----------------------
        //      Send to Simulator
        //----------------------
            json msgJson;

            // Controls
//		        msgJson["steering_angle"] = vars[0]/(deg2rad(25) * Lf);
//		        msgJson["throttle"] = vars[1];
		        msgJson["steering_angle"] = -steer_value/0.436332;
		        msgJson["throttle"] = throttle_value;

	        //Display the waypoints/reference line
		        vector<double> next_x_vals;
		        vector<double> next_y_vals;

		        for (unsigned i=0 ; i < ptsx.size(); ++i) {
			        next_x_vals.push_back(Ptsx(i));
			        next_y_vals.push_back(Ptsy(i));
		        }

	        // Reference Path
		        msgJson["next_x"] = next_x_vals;
		        msgJson["next_y"] = next_y_vals;

	        // Predicted Path
	            msgJson["mpc_x"] = vars.X;
	            msgJson["mpc_y"] = vars.Y;


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
	            this_thread::sleep_for(chrono::milliseconds(100));
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
