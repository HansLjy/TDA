#pragma once

#include "Eigen/Eigen"
#include "gudhi/Rips_complex.h"
#include "gudhi/Simplex_tree.h"
#include "gudhi/distance_functions.h"
#include <CGAL/Epeck_d.h>
#include "spdlog/spdlog.h"

namespace TDA {

struct Simplex {
    int _dim;
    std::vector<int> _faces;
    std::vector<int> _vertices;
};

struct Filtration {
	int _num_vertices = 0;
	int _num_edges = 0;
	int _num_faces = 0;

    std::vector<Simplex> _simplices;
    std::vector<double> _diameters;
};

std::ostream& operator<<(std::ostream& out, const Filtration& filtration);


template<int dim>
Filtration GenerateFiltration(
    const Eigen::Matrix<double, Eigen::Dynamic, dim>& point_cloud,
    const double threshold
);

}

// implementation
namespace TDA {

template<int dim>
using Kernel = CGAL::Epeck_d< CGAL::Dimension_tag<dim> >;

template<int dim>
Filtration GenerateFiltration(
    const Eigen::Matrix<double, Eigen::Dynamic, dim>& point_cloud,
    const double threshold
) {

    std::vector<std::vector<double>> points;

    int num_points = point_cloud.rows();
    for (int i = 0; i < num_points; i++) {
        std::vector<double> point(dim);
        for (int j = 0; j < dim; j++) {
            point[j] = point_cloud(i, j);
        }
        points.push_back(point);
    }

    Gudhi::rips_complex::Rips_complex<Gudhi::Simplex_tree<>::Filtration_value>
        rips_complex(points, threshold, Gudhi::Euclidean_distance());

    Gudhi::Simplex_tree<> simplex_tree;

    rips_complex.create_complex(simplex_tree, 2);   // get 2-skeleton(only vertics, edges, triangles)

    spdlog::info(
        "Rips complex has {} simplices and {} vertices",
        simplex_tree.num_simplices(),
        simplex_tree.num_vertices()
    );

    int simplex_id = 0;
    for (auto simplex_handle : simplex_tree.filtration_simplex_range()) {
        simplex_tree.assign_key(simplex_handle, simplex_id++);
    }

    Filtration filtration;
    for (auto simplex_handle : simplex_tree.filtration_simplex_range()) {
        Simplex simplex;
        simplex._dim = simplex_tree.dimension(simplex_handle);
        for (auto face_simplex_handle : simplex_tree.boundary_simplex_range(simplex_handle)) {
            simplex._faces.push_back(simplex_tree.key(face_simplex_handle));
        }
		std::reverse(simplex._faces.begin(), simplex._faces.end());
        for (auto vertex_id : simplex_tree.simplex_vertex_range(simplex_handle)) {
            simplex._vertices.push_back(vertex_id);
        }
		std::reverse(simplex._vertices.begin(), simplex._vertices.end());
        filtration._simplices.push_back(simplex);
        filtration._diameters.push_back(simplex_tree.filtration(simplex_handle));
    }

	filtration._num_vertices = 0;
	filtration._num_edges = 0;
	filtration._num_faces = 0;
	for (const auto& simplex : filtration._simplices) {
		switch (simplex._dim) {
			case 0:
				filtration._num_vertices++;
				break;
			case 1:
				filtration._num_edges++;
				break;
			case 2:
				filtration._num_faces++;
				break;
		}
	}

    return filtration;
}

}