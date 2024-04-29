#include "HighDimVisualization.hpp"
#include "Utilities.hpp"
#include "TestConfig.hpp"
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

template<class MatrixType>
void read_binary(std::ifstream &in, MatrixType &matrix) {
    typename MatrixType::Index rows = 0, cols = 0;
    in.read((char *) (&rows), sizeof(typename MatrixType::Index));
    in.read((char *) (&cols), sizeof(typename MatrixType::Index));
    matrix.resize(rows, cols);
    in.read((char *) matrix.data(), rows * cols * sizeof(typename MatrixType::Scalar));
}

int main(int argc, char* argv[]) {
	std::ifstream high_dim_model_file(fs::path(HIGH_DIM_MODEL_DIR) / argv[1]);
	Eigen::MatrixXd vertices;
	read_binary(high_dim_model_file, vertices);

    Eigen::VectorXd coords_max = vertices.colwise().maxCoeff();
    Eigen::VectorXd coords_min = vertices.colwise().minCoeff();
    Eigen::VectorXd center = (coords_max + coords_min) / 2;
    float max_gap = (coords_max - coords_min).maxCoeff();

    vertices.rowwise() -= center.transpose();
    vertices *= float(2.0 / max_gap);

    auto gui = new TDA::HighDimGUI<20>();
    Renderer::GUI::RegisterGlobalGUI(gui);

	gui->Initialize();
	gui->CreateWindow(800, 600, "TDA High-dim");

	gui->SetVertices(vertices);

    gui->SetCamera(
        glm::vec3(2.0f, 0.0f, 0.0f),
        0.0f, 0.0f,
        0.1f, 100.0f
    );
    
    gui->PreCompute();
    gui->MainLoop();
}