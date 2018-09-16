#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

//---------------------
//    Parameters
//---------------------

    // Time Step
        const int N = 10;
        double dt = 0.1;

    // Lf -- from Udacity
        const double Lf = 2.67;

	// Error
		double ref_cte = 0.0;
		double ref_epsi = 0.0;
		double ref_v = 100.0;

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
		// Member
			Eigen::VectorXd coeffs;

		// Constructor
		    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

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
	                for (int i=0; i < N; i++)
	                {
	                	fg[0] += 2000 * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
	                	fg[0] += 2000 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
	                	fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
	                }

                //
                    for (int i=0; i < N - 1; i++)
                    {
                    	fg[0] += 5 * CppAD::pow(vars[delta_start + i], 2);
                    	fg[0] += 5 * CppAD::pow(vars[a_start + i], 2);
                    }

                // Smooth Steering
                    for (int i=0; i < N - 2; i++)
                    {
                    	fg[0] += 200 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
                    	fg[0] += 10  * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
                    }

		    //----------------------
		    //      Constraints
		    //----------------------

		        // Index + 1
		        // < x, y, psi, v, cte, espi >
		            fg[1 + x_start]     = vars[x_start];
				    fg[1 + y_start]     = vars[y_start];
				    fg[1 + psi_start]   = vars[psi_start];
				    fg[1 + v_start]     = vars[v_start];
				    fg[1 + cte_start]   = vars[cte_start];
				    fg[1 + epsi_start]  = vars[epsi_start];

			    // Constraints
			        for (int i=0; i < N - 1; i++)
			        {
			        	// Time t + 1
				            AD<double> x1       = vars[x_start + i + 1];
					        AD<double> y1       = vars[y_start + i + 1];
					        AD<double> psi1     = vars[psi_start + i + 1];
					        AD<double> v1       = vars[v_start + i + 1];
					        AD<double> cte1     = vars[cte_start + i + 1];
					        AD<double> epsi1    = vars[epsi_start + i + 1];

			            // Time t
					        AD<double> x0       = vars[x_start + i];
					        AD<double> y0       = vars[y_start + i];
					        AD<double> psi0     = vars[psi_start + i];
					        AD<double> v0       = vars[v_start + i];
					        AD<double> cte0     = vars[cte_start + i];
					        AD<double> epsi0    = vars[epsi_start + i];

					        AD<double> delta0 = vars[delta_start + i];
					        AD<double> a0 = vars[a_start + i];

			            // Polynomial
				            AD<double> f0 = coeffs[0] + ( coeffs[1] * x0 ) + ( coeffs[2] * x0 * x0 ) + ( coeffs[3] * x0 * x0 * x0 );

			            // Desired Psi
				            AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);

			            // Model Constraints
			                fg[2 + x_start + i]     = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
					        fg[2 + y_start + i]     = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
					        fg[2 + psi_start + i]   = psi1 - (psi0 - v0 * delta0 / Lf * dt);
					        fg[2 + v_start + i]     = v1 - (v0 + a0 * dt);
					        fg[2 + cte_start + i]   = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
					        fg[2 + epsi_start + i]  = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
			        }
	        }
	};
//---------------- FG Eval ----------------

//=====================
//    M P C
//=====================

	//---------------------
	//    Constructor
	//---------------------
		MPC::MPC() {}
		MPC::~MPC() {}

	//---------------------
	//    Solve
	//---------------------
		vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

		// Initialize Variables
			bool ok = true;
			size_t i;
			typedef CPPAD_TESTVECTOR(double) Dvector;

		// State
			double x    = state[0];
			double y    = state[1];
			double psi  = state[2];
			double v    = state[3];
			double cte  = state[4];
			double epsi = state[5];

		// Num Vars
			int n_vars = N * 6 + (N - 1) * 2;

		// Constraints
			int n_constraints = N * 6;

		// Initialize
			Dvector vars(n_vars);
			for (int i = 0; i < n_vars; i++) {
		        vars[i] = 0;
		    }

	    // Bounds
		    Dvector vars_lowerbound(n_vars);
			Dvector vars_upperbound(n_vars);

			// Non Actuator limits
				for (size_t i=0; i < delta_start; i++) {
					vars_lowerbound[i] = -1.0e19;
					vars_upperbound[i] =  1.0e19;
				}

			// Steering limits
				for (size_t i = delta_start; i < a_start; i++) {
					vars_lowerbound[i] = -0.436332 * Lf;
					vars_upperbound[i] =  0.436332 * Lf;
				}

			// Acceleration Limits
				for (int i = a_start; i < n_vars; i++) {
					vars_lowerbound[i] = -1.0;
					vars_upperbound[i] =  1.0;
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

			constraints_upperbound[x_start]     = x;
			constraints_upperbound[y_start]     = y;
			constraints_upperbound[psi_start]   = psi;
			constraints_upperbound[v_start]     = v;
			constraints_upperbound[cte_start]   = cte;
			constraints_upperbound[epsi_start]  = epsi;

	    // object that computes objective and constraints
		    FG_eval fg_eval(coeffs);

		  //
		  // NOTE: You don't have to worry about these options
		  //

	    //---------------------
		//    IPOPT Options
		//---------------------
			std::string options;

			// Uncomment this if you'd like more print information
				options += "Integer print_level  0\n";

			// NOTE: Setting sparse to true allows the solver to take advantage
			// of sparse routines, this makes the computation MUCH FASTER. If you
			// can uncomment 1 of these and see if it makes a difference or not but
			// if you uncomment both the computation time should go up in orders of
			// magnitude.
				options += "Sparse  true        forward\n";
				options += "Sparse  true        reverse\n";
			// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
			// Change this as you see fit.
				options += "Numeric max_cpu_time          0.5\n";
			// place to return solution
				CppAD::ipopt::solve_result<Dvector> solution;

			// solve the problem
				CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound,
				constraints_lowerbound, constraints_upperbound, fg_eval, solution);

			// Check some of the solution values
				ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

			// Cost
				//auto cost = solution.obj_value;
				//std::cout << "Cost " << cost << std::endl;

		//---------------------
		//    Result
		//---------------------
				vector <double> result;
				result.push_back(solution.x[delta_start]);
				result.push_back(solution.x[a_start]);

				for (int i=0; i < N-1; i++)
				{
					result.push_back(solution.x[x_start + i + 1]);
					result.push_back(solution.x[y_start + i + 1]);
				}

				return result;
		}
