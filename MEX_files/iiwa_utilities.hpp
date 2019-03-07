#ifndef IIWA_UTILITIES
#define IIWA_UTILITIES

#include </usr/local/include/eigen3/Eigen/Eigen>
#include <vector>
#include <string>

class iiwa
{
	public:
	static Eigen::MatrixXd compute_iiwa7_FK(Eigen::MatrixXd, Eigen::MatrixXd);
	static Eigen::MatrixXd compute_iiwa7_FK(std::vector<double>, Eigen::MatrixXd);
	static Eigen::MatrixXd compute_iiwa7_FK_all(Eigen::MatrixXd, Eigen::MatrixXd);
	static Eigen::MatrixXd compute_iiwa7_FK_all(std::vector<double>, Eigen::MatrixXd);
	static Eigen::MatrixXd compute_iiwa14_FK_all(Eigen::MatrixXd, Eigen::MatrixXd);
	static Eigen::MatrixXd compute_iiwa14_FK_all(std::vector<double>, Eigen::MatrixXd);
};

#endif