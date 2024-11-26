#include "Eigen/Dense"
#include "ezc3d_all.h"
#include "DataConfig.hpp"
#include <set>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

template<class Matrix>
void write_binary(std::ofstream &out, const Matrix &matrix) {
    typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
    out.write((char *) (&rows), sizeof(typename Matrix::Index));
    out.write((char *) (&cols), sizeof(typename Matrix::Index));
    out.write((char *) matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));
}

template<class MatrixType>
void read_binary(std::ifstream &in, MatrixType &matrix) {
    typename MatrixType::Index rows = 0, cols = 0;
    in.read((char *) (&rows), sizeof(typename MatrixType::Index));
    in.read((char *) (&cols), sizeof(typename MatrixType::Index));
    matrix.resize(rows, cols);
    in.read((char *) matrix.data(), rows * cols * sizeof(typename MatrixType::Scalar));
}

int main(int argc, char* argv[]) {
    ezc3d::c3d data(fs::path(RAW_DATA_DIR) / argv[1]);

    const int num_frames = data.header().nbFrames();
    const int num_points = data.header().nb3dPoints();
	std::cerr << "number of frames: " << num_frames << std::endl
			  << "number of points: " << num_points << std::endl;


	std::set<int> unusable_point_ids;
    for (int frame_id = 0; frame_id < num_frames; frame_id++) {
        for (int point_id = 0; point_id < num_points; point_id++) {

            auto pt = data.data().frames()[frame_id].points().point(point_id);
			if (std::isnan(pt.x())) {
				unusable_point_ids.insert(point_id);
			}
        }
    }
	std::cerr << unusable_point_ids.size() << std::endl;

	const int num_usable_points = num_points - unusable_point_ids.size();

    Eigen::MatrixXd data_eigen(num_frames, num_usable_points * 3);
	for (int frame_id = 0; frame_id < num_frames; frame_id++) {
        for (int point_id = 0; point_id < num_points; point_id++) {

            auto pt = data.data().frames()[frame_id].points().point(point_id);
			if (!std::isnan(pt.x())) {
				auto diff_id = std::distance(unusable_point_ids.begin(), unusable_point_ids.lower_bound(point_id));
				data_eigen.block<1, 3>(frame_id, (point_id - diff_id) * 3)
					<< pt.x(), pt.y(), pt.z();
			}
        }
    }

	for (int frame_id = 0; frame_id < num_frames; frame_id++) {
		Eigen::Vector3d center = Eigen::Vector3d::Zero();
		for (int usable_point_id = 0; usable_point_id < num_usable_points; usable_point_id++) {
			center += data_eigen.block<1, 3>(frame_id, usable_point_id * 3);
		}
		center /= num_usable_points;
		for (int usable_point_id = 0; usable_point_id < num_usable_points; usable_point_id++) {
			data_eigen.block<1, 3>(frame_id, usable_point_id * 3) -= center.transpose();
		}
	}

    std::ofstream out_file(fs::path(HIGH_DIM_MODEL_DIR) / (std::string(argv[1]) + ".eig"));
    write_binary(out_file, data_eigen);
}