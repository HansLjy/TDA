#pragma once

#include <vector>
#include "Filtration.hpp"
#include "PersistentCohomology.hpp"
#include "Lifting.hpp"

namespace TDA {

template<int dim>
std::optional<std::vector<std::vector<double>>> CalculateGlobalCircularCoordinate(
    const Eigen::Matrix<double, Eigen::Dynamic, dim>& point_cloud,
    const double threshold
);

template<int dim>
std::optional<std::pair<std::vector<int>, std::vector<std::vector<double>>>>
CalculateLocalCircularCoordinate(
    const Eigen::Matrix<double, Eigen::Dynamic, dim> &point_cloud,
    const Eigen::Vector<double, dim>& center,
    const double radius,
    const double threshold
);

}

namespace TDA {

namespace internal {

template<int p>
std::optional<std::vector<std::vector<double>>> TryWithPrime(
    const Filtration& filtration
) {
    spdlog::info("Begin persistent cohomology calculation with prime {}", p);
    auto pc = GeneratePersistentCohomology<p>(filtration);
    spdlog::info("Finish persistent cohomology calculation, order of cohomology group = {}", pc._I.size());

	if (pc._bettis[1] == 0) {
		spdlog::warn("H^1 is trivial for the threshold");
		return {};
	}

    spdlog::info("Begin lifting to real");
    bool success = true;
    std::vector<std::vector<double>> results;
    for (auto i_index : pc._I) {
		if (filtration._simplices[i_index]._dim != 1) {
			// only consider H^1
			continue;
		}
        auto coefs_Z = LiftingToZ(filtration, pc._coefs[i_index]);
        if (coefs_Z.has_value()) {
			const auto& coefs_Z_val = coefs_Z.value();
            results.push_back(
                LiftingToR(filtration, coefs_Z_val)
            );
        } else {
            success = false;
            break;
        }
    }
    spdlog::info("Finish lifting to real");

    if (success) {
        return results;
    } else {
        return {};
    }
}

inline int GetNewId(int id, const std::vector<int>& insert_places) {
    return id + (std::upper_bound(insert_places.begin(), insert_places.end(), id) - insert_places.begin());
}

}

template<int dim>
std::optional<std::vector<std::vector<double>>> CalculateGlobalCircularCoordinate(
    const Eigen::Matrix<double, Eigen::Dynamic, dim>& point_cloud,
    const double threshold
) {
    spdlog::info("Begin filtration generation");
    auto filtration = GenerateFiltration(point_cloud, threshold);
    spdlog::info(
        "Finished filtration generation. #vertices = {}, #edges = {}, #faces = {}, #simplicies = {}",
        filtration._num_vertices,
        filtration._num_edges,
        filtration._num_faces,
        filtration._simplices.size()
    );
    // std::cerr << "fuck: " << __FILE__ << ":" << __LINE__ << std::endl;
    // std::cerr << filtration << std::endl;
    auto result = internal::TryWithPrime<47>(filtration);
    if (result.has_value()) {
        spdlog::info("Success with prime 47");
        return result.value();
    }
    result = internal::TryWithPrime<53>(filtration);
    if (result.has_value()) {
        spdlog::info("Success with prime 53");
        return result.value();
    }
    result = internal::TryWithPrime<59>(filtration);
    if (result.has_value()) {
        spdlog::info("Success with prime 59");
        return result.value();
    }
    result = internal::TryWithPrime<997>(filtration);
    return result;
}


// <- the returned color[i] refers to color of local_vertex_indices[i - 1]
template<int dim>
std::optional<std::pair<std::vector<int>, std::vector<std::vector<double>>>>
CalculateLocalCircularCoordinate(
    const Eigen::Matrix<double, Eigen::Dynamic, dim> &point_cloud,
    const Eigen::Vector<double, dim>& center,
    const double radius,
    const double threshold
) {
    const int num_vertices = point_cloud.rows();
    Eigen::VectorXd distances = (point_cloud.rowwise() - center.transpose()).rowwise().norm();

    std::vector<int> local_vertex_ids;
    for (int i = 0; i < num_vertices; i++) {
        if (distances(i) < radius + threshold) {
            local_vertex_ids.push_back(i);
        }
    }
    const int num_local_vertices = local_vertex_ids.size();
    spdlog::info("{} vertices are considered in the local topology", num_local_vertices);

    Eigen::Matrix<double, Eigen::Dynamic, dim> point_cloud_local = point_cloud(local_vertex_ids, Eigen::all);

    std::vector<bool> inside_neighborhood;
    for (int i = 0; i < num_local_vertices; i++) {
        inside_neighborhood.push_back(distances(local_vertex_ids[i]) < radius);
    }

    spdlog::info("Begin filtration generation");
    Filtration filtration = GenerateFiltration(point_cloud_local, threshold);
    spdlog::info(
        "Finished filtration generation. #vertices = {}, #edges = {}, #faces = {}, #simplicies = {}",
        filtration._num_vertices,
        filtration._num_edges,
        filtration._num_faces,
        filtration._simplices.size()
    );
    // std::cerr << filtration << std::endl;

    spdlog::info("Begin filtration coning");
    std::vector<int> insert_places = {0};   // insert new simplex i before insert_places[i]
    int simplex_id = 0;
    for (const auto& simplex : filtration._simplices) {
        bool completely_outside = true;
        for (const auto vertex_id : simplex._vertices) {
            if (inside_neighborhood[vertex_id]) {
                completely_outside = false;
                break;
            }
        }
        
        if (completely_outside) {
            // connect it to the extra point
            insert_places.push_back(simplex_id + 1);
        }

        simplex_id++;
    }

    simplex_id = 0;
    Filtration coned_filtration;
    int insert_place_id = 1;

    coned_filtration._simplices.push_back(Simplex(0, {}, {0}));
    coned_filtration._diameters.push_back(0);

    for (const auto& simplex : filtration._simplices) {
        Simplex new_simplex;
        new_simplex._dim = simplex._dim;
        for (const auto vertex : simplex._vertices) {
            new_simplex._vertices.push_back(vertex + 1);
        }
        for (const auto face : simplex._faces) {
            new_simplex._faces.push_back(internal::GetNewId(face, insert_places));
        }
        coned_filtration._simplices.push_back(new_simplex);
        coned_filtration._diameters.push_back(filtration._diameters[simplex_id]);
        
        if (insert_place_id < insert_places.size() && insert_places[insert_place_id] == simplex_id + 1) {
            Simplex coned_simplex;
            coned_simplex._dim = simplex._dim + 1;
            coned_simplex._vertices.push_back(0);
            for (const auto vertex : simplex._vertices) {
                coned_simplex._vertices.push_back(vertex + 1);
            }

            coned_simplex._faces.push_back(coned_filtration._simplices.size() - 1);
            if (simplex._dim == 0) {
                coned_simplex._faces.push_back(0);
            } else {
                for (const auto& face : simplex._faces) {
                    // the simplex immediately after face is the added coning simplex
                    coned_simplex._faces.push_back(internal::GetNewId(face, insert_places) + 1);
                }
            }
            coned_filtration._simplices.push_back(coned_simplex);
            coned_filtration._diameters.push_back(filtration._diameters[simplex_id]);

            insert_place_id++;
        }
        simplex_id++;
    }

    coned_filtration.UpdateInfo();

    spdlog::info(
        "Finished filtration coning. #vertices = {}, #edges = {}, #faces = {}, #simplicies = {}",
        coned_filtration._num_vertices,
        coned_filtration._num_edges,
        coned_filtration._num_faces,
        coned_filtration._simplices.size()
    );
    // std::cerr << coned_filtration << std::endl;

    auto result = internal::TryWithPrime<47>(coned_filtration);
    if (result.has_value()) {
        return std::make_pair(local_vertex_ids, result.value());
    }
    result = internal::TryWithPrime<53>(coned_filtration);
    if (result.has_value()) {
        return std::make_pair(local_vertex_ids, result.value());
    }
    result = internal::TryWithPrime<59>(coned_filtration);
    if (result.has_value()) {
        return std::make_pair(local_vertex_ids, result.value());
    }
    result = internal::TryWithPrime<997>(coned_filtration);
    if (result.has_value()) {
        return std::make_pair(local_vertex_ids, result.value());
    }
    return {};
}

}