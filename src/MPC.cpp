#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/src/Core/util/DisableStupidWarnings.h"

using CppAD::AD;

//---------------------
//    Parameters
//---------------------
	// Time step
		const int N = 20;
		const double dt = 0.05;

	// Lf -- From Udacity
		const double Lf = 2.67;

	// Error
		double ref_cte = 0;
		double ref_epsi = 0;
		double ref_v = 80;

		const int cte_cost = 1;
		const int psi_cost = 1;
		const int v_cost = 1;

		const int steering_cost = 1;
		const int accl_cost = 10;

		const int d_angle_cost = 600;
		const int d_accl_cost = 1;


	// Index
		size_t x_start      = 0;
		size_t y_start      = x_start + N;
		size_t psi_start    = y_start + N;
		size_t v_start      = psi_start + N;
		size_t cte_start    = v_start + N;
		size_t epsi_start   = cte_start + N;
		size_t delta_start  = epsi_start + N;
		size_t a_start      = delta_start + N - 1;

//=====================
//    FG - Eval
//=====================
	class FG_eval {
	public:
		// Coeffs
			Eigen::VectorXd coeffs;

		// Previous Actuator State
			vector<double> prev_actuators;

		// Constructor
			FG_eval(Eigen::VectorXd coeffs, vector<double> prev_actuators) {
				this->coeffs = coeffs;
				this->prev_actuators = prev_actuators;
			}

		// Type Def
			typedef CPPAD_TESTVECTOR(AD<double>) ADvector;


	//============================
	//      Operator ()
	//============================
		void operator()(ADvector& fg, const ADvector& vars) {

			//-----------------------
			//      Cost Function
			//-----------------------
				fg[0] = 0;

			// Reference State Cost
				for (int i = 0; i < N; i++) {
					fg[0] += cte_cost * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
					fg[0] += psi_cost * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
					fg[0] += v_cost   * CppAD::pow(vars[v_start + i] - ref_v, 2);
				}

			// Minimize the use of actuators
				for (int i = 0; i < N - 1; i++) {
					fg[0] += steering_cost * CppAD::pow(vars[delta_start + i], 2);
					fg[0] += accl_cost * CppAD::pow(vars[a_start + i], 2);
				}

			// Smooth Steering
				for (int i = 0; i < N - 2; i++) {
					fg[0] += d_angle_cost * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
					fg[0] += d_accl_cost  * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
				}


			// Initial Constraints
				fg[1 + x_start]     = vars[x_start];
				fg[1 + y_start]     = vars[y_start];
				fg[1 + psi_start]   = vars[psi_start];
				fg[1 + v_start]     = vars[v_start];
				fg[1 + cte_start]   = vars[cte_start];
				fg[1 + epsi_start]  = vars[epsi_start];

		// Constraints
			for (int i = 0; i < N - 1; i++) {
				// State -- Time = t + 1
					AD<double> x1       = vars[x_start + i + 1];
					AD<double> y1       = vars[y_start + i + 1];
					AD<double> psi1     = vars[psi_start + i + 1];
					AD<double> v1       = vars[v_start + i + 1];
					AD<double> cte1     = vars[cte_start + i + 1];
					AD<double> epsi1    = vars[epsi_start + i + 1];

				// State -- Time = t
					AD<double> x0       = vars[x_start + i];
					AD<double> y0       = vars[y_start + i];
					AD<double> psi0     = vars[psi_start + i];
					AD<double> v0       = vars[v_start + i];
					AD<double> cte0     = vars[cte_start + i];
					AD<double> epsi0    = vars[epsi_start + i];

				// Initial State : actuators at T = 0
					AD<double> delta0 = vars[delta_start + i];
					AD<double> a0 = vars[a_start + i];

					AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
					AD<double> psides0 = CppAD::atan(coeffs[1]+2*coeffs[2]*x0 + 3 * coeffs[3]*x0*x0);


					fg[2 + x_start + i]     = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
					fg[2 + y_start + i]     = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
					fg[2 + psi_start + i]   = psi1 - (psi0 + v0 * delta0 / Lf * dt);
					fg[2 + v_start + i]     = v1 - (v0 + a0 * dt);
					fg[2 + cte_start + i]   = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
					fg[2 + epsi_start + i]  = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
			}
		}
	};
//---------------- FG Eval ----------------

//=====================
//    M P C
//=====================

	//----------------------
	//    Constructor
	//----------------------
		MPC::MPC() {}
		MPC::~MPC() {}

	//----------------------
	//    Solve
	//----------------------
		Solution MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {

		// Initialize Variables
			bool ok = true;
			typedef CPPAD_TESTVECTOR(double) Dvector;

		// State
			double x    = x0[0];
			double y    = x0[1];
			double psi  = x0[2];
			double v    = x0[3];
			double cte  = x0[4];
			double epsi = x0[5];

		// Num Vars
			int n_vars = N * 6 + (N - 1) * 2;

		// Constraints
			int n_constraints = N * 6;

		// Initialize
			Dvector vars(n_vars);
			for (int i = 0; i < n_vars; i++) {
				vars[i] = 0.0;
			}


		// Initial State Values
			vars[x_start]       = x;
			vars[y_start]       = y;
			vars[psi_start]     = psi;
			vars[v_start]       = v;
			vars[cte_start]     = cte;
			vars[epsi_start]    = epsi;

		// Bounds
			Dvector vars_lowerbound(n_vars);
			Dvector vars_upperbound(n_vars);

		// Non Actuator Limits
			for (size_t i = 0; i < delta_start; i++) {
				vars_lowerbound[i] = -1.0e19;
				vars_upperbound[i] = 1.0e19;
			}

		// Steering Limits
			for (size_t i = delta_start; i < a_start; i++) {
				vars_lowerbound[i] = -0.436332;
				vars_upperbound[i] = 0.436332;
			}

		// Steering Limits = state at time t
			for (size_t i = delta_start; i < delta_start + latency_ind; i++) {
				vars_lowerbound[i] = delta_prev;
				vars_upperbound[i] = delta_prev;
			}

		// Acceleration Limits
			for (size_t i = a_start; i < n_vars; i++) {
				vars_lowerbound[i] = -1.0;
				vars_upperbound[i] =  1.0;
			}

		// Actual acceleration at time t
			for (size_t i = a_start; i < a_start+latency_ind; i++) {
				vars_lowerbound[i] = a_prev;
				vars_upperbound[i] = a_prev;
			}

		// Constraints
			Dvector constraints_lowerbound(n_constraints);
			Dvector constraints_upperbound(n_constraints);
			for (int i = 0; i < n_constraints; i++) {
				constraints_lowerbound[i] = 0;
				constraints_upperbound[i] = 0;
			}

		// Initial State
			constraints_lowerbound[x_start]     = x;
			constraints_lowerbound[y_start]     = y;
			constraints_lowerbound[psi_start]   = psi;
			constraints_lowerbound[v_start]     = v;
			constraints_lowerbound[cte_start]   = cte;
			constraints_lowerbound[epsi_start]  = epsi;

			constraints_upperbound[x_start] = x;
			constraints_upperbound[y_start] = y;
			constraints_upperbound[psi_start] = psi;
			constraints_upperbound[v_start] = v;
			constraints_upperbound[cte_start] = cte;
			constraints_upperbound[epsi_start] = epsi;

		// Object that computes objective and constraints
			vector<double> prev_actuators = {delta_prev,a_prev};
			FG_eval fg_eval(coeffs,prev_actuators);

		//----------------------
		//    Ipopt Options
		//----------------------
			std::string options;
			options += "Integer print_level  0\n";
			options += "Sparse  true        forward\n";
			options += "Sparse  true        reverse\n";
			options += "Numeric max_cpu_time          0.5\n";

		//----------------------
		//    Result
		//----------------------

			// Solution Vector
				CppAD::ipopt::solve_result<Dvector> solution;

			// Solve the problem
				CppAD::ipopt::solve<Dvector, FG_eval>(
						options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
						constraints_upperbound, fg_eval, solution);

			// Check the solution values
				ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

			// Push bask the solution
				Solution result;
				for (auto i = 0; i < N-1 ; i++){
					result.X.push_back(solution.x[x_start+i]);
					result.Y.push_back(solution.x[y_start+i]);
					result.Delta.push_back(solution.x[delta_start+i]);
					result.A.push_back(solution.x[a_start+i]);
				}

			return result;

		}