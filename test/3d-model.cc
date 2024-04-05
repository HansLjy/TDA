#include "Visualization.hpp"
#include "Utilities.hpp"
#include "TestConfig.hpp"
#include <filesystem>

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
    Renderer::RowMajorMatrixX3f vertices;
    Renderer::RowMajorMatrixX2f uvs;
    Renderer::RowMajorMatrixX3u face_topo;
    Renderer::LoadObj(
        fs::path(TDA_MODEL_DIR) / argv[1], vertices, uvs, face_topo
    );
    const int num_vertics = vertices.rows();
    
    Eigen::Vector3f coords_max = vertices.colwise().maxCoeff();
    Eigen::Vector3f coords_min = vertices.colwise().minCoeff();
    Eigen::Vector3f center = (coords_max + coords_min) / 2;
    center = (coords_max + coords_min) / 2;
    float max_gap = (coords_max - coords_min).maxCoeff();

    vertices.rowwise() -= center.transpose();
    vertices *= float(2.0 / max_gap);

    Renderer::RowMajorMatrixX3f colors = vertices.array() * 0.5 + 1;

    auto gui = new TDA::TDAGUI();
    Renderer::GUI::RegisterGlobalGUI(gui);

	gui->Initialize();
	gui->CreateWindow(800, 600, "TDA");

    auto point_cloud = TDA::PointCloud::GetPointCloud(num_vertics, 0);
    point_cloud->SetCoordinates(vertices);
    point_cloud->SetColors(colors);

    gui->SetPointCloud(point_cloud);

    gui->SetCamera(
        glm::vec3(5.0f, 0.0f, 0.0f),
        0.0f, 0.0f,
        0.1f, 100.0f
    );
    
    gui->PreCompute();
    gui->MainLoop();
}