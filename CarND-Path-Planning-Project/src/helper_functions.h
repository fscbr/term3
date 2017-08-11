#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <iostream>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

Eigen::VectorXd convertMapToCar(const Eigen::VectorXd &state, const Eigen::VectorXd &pos);

Eigen::VectorXd convertCarToMap(const Eigen::VectorXd &state, const Eigen::VectorXd &pos);

void convertVectorMapToCar(const Eigen::VectorXd state,
		const std::vector<double> map_x,
		const std::vector<double> map_y,
		std::vector<double> &car_x,
		std::vector<double> &car_y,
		bool filterNegX);

void convertVectorCarToMap(const Eigen::VectorXd state,
		const std::vector<double> car_x,
		const std::vector<double> car_y,
		std::vector<double> &map_x,
		std::vector<double> &map_y);

Eigen::Map<Eigen::VectorXd> toVectorXd(std::vector<double> &vec);

void log_vector(const std::string name, std::vector<double> &vec);

double poly_eval_psi(Eigen::VectorXd &coeffs, double y0);

#endif /* HELPER_FUNCTIONS_H_ */
