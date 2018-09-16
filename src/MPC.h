#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
const int latency_ind = 2;

struct Solution {

		vector<double> X;
		vector<double> Y;
		vector<double> Psi;
		vector<double> V;
		vector<double> Cte;
		vector<double> EPsi;
		vector<double> Delta;
		vector<double> A;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  Solution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  double delta_prev {0};
  double a_prev {0.1};

};


#endif /* MPC_H */
