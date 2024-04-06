#include "CircularCoordinate.hpp"
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
    const int num_vertices = vertices.rows();
    
    Eigen::Vector3f coords_max = vertices.colwise().maxCoeff();
    Eigen::Vector3f coords_min = vertices.colwise().minCoeff();
    Eigen::Vector3f center = (coords_max + coords_min) / 2;
    center = (coords_max + coords_min) / 2;
    float max_gap = (coords_max - coords_min).maxCoeff();

    vertices.rowwise() -= center.transpose();
    vertices *= float(2.0 / max_gap);

    Renderer::RowMajorMatrixX3f colors(vertices.rows(), vertices.cols());

    auto gui = new TDA::TDAGUI();
    Renderer::GUI::RegisterGlobalGUI(gui);

	gui->Initialize();
	gui->CreateWindow(800, 600, "TDA");

    auto point_cloud = TDA::PointCloud::GetPointCloud(num_vertices, 0);
    point_cloud->SetCoordinates(vertices);

    double threshold = atof(argv[2]);

    auto circular_coords = TDA::CalculateGlobalCircularCoordinate<3>(vertices.cast<double>(), threshold);
    
    std::vector<std::pair<unsigned int, unsigned int>> edges;
    for (int i = 0; i < num_vertices; i++) {
        for (int j = i + 1; j < num_vertices; j++) {
            if ((vertices.row(i) - vertices.row(j)).norm() < threshold) {
                edges.push_back(std::make_pair(i, j));
            }
        }
    }
    const int num_edges = edges.size();
    Renderer::RowMajorMatrixX2u edges_eigen(num_edges, 2);
    for (int i = 0; i < num_edges; i++) {
        edges_eigen.row(i) << edges[i].first, edges[i].second;
    }

    point_cloud->SetEdges(edges_eigen);

    if (circular_coords.has_value()) {
        auto& hues = circular_coords.value()[0];
        assert(hues.size() == num_vertices);
        for (int i = 0; i < num_vertices; i++) {
            colors.row(i) = TDA::HSV2RGB(std::fmod(hues[i], 1.0), 1.0, 1.0);
        }
    }
    std::cerr << colors;

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