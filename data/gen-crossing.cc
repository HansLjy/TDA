#include <fstream>
#include <filesystem>
#include <iostream>
#include "Eigen/Eigen"
#include "DataConfig.hpp"

namespace fs = std::filesystem;

template<class Matrix>
void write_binary(std::ofstream &out, const Matrix &matrix) {
    typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
    out.write((char *) (&rows), sizeof(typename Matrix::Index));
    out.write((char *) (&cols), sizeof(typename Matrix::Index));
    out.write((char *) matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));
}

int main() {
	int num_dim = 20;
	int num_points_on_edge = 20;
	double noise_level = 5e-3;

	Eigen::MatrixXd vertices = Eigen::MatrixXd::Zero(num_points_on_edge * num_dim, num_dim);
	int cur_id = 0;
	for (int dim = 0; dim < num_dim; dim++) {
		for (int point_id = 0; point_id < num_points_on_edge; point_id++) {
			vertices(cur_id, dim) = -1 + 2.0 / (num_points_on_edge - 1) * point_id;
			vertices.row(cur_id) += (noise_level * Eigen::VectorXd::Random(num_dim)).transpose();
			cur_id++;
		}
	}

	std::cerr << vertices.rows() << " " << vertices.cols() << std::endl;

	std::ofstream out(fs::path(HIGH_DIM_MODEL_DIR) / "cross.eig");
	write_binary(out, vertices);
}