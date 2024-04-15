#pragma once

#include "Eigen/Dense"

namespace TDA {

template<int dim>
class PCA {
public:
	void Fit(const Eigen::MatrixXd& data);
	Eigen::Matrix<double, Eigen::Dynamic, dim> Transform(const Eigen::MatrixXd& data) const;

protected:
	Eigen::Matrix<double, Eigen::Dynamic, dim> _weights;
};

}

namespace TDA {

template<int dim>
void PCA<dim>::Fit(const Eigen::MatrixXd &data) {
	Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ComputeThinV> svd(data);
	svd.computeV();
	_weights = svd.matrixV().leftCols(dim);
}

template<int dim>
Eigen::Matrix<double, Eigen::Dynamic, dim> PCA<dim>::Transform(const Eigen::MatrixXd& data) const {
	return data * _weights;
}

}