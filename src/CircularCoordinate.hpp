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
std::optional<std::vector<std::vector<double>>> CalculateLocalCircularCoordinate(
    const Eigen::Matrix<double, Eigen::Dynamic, dim>& point_cloud,
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
    auto pc = GeneratePersistentCohomology<p>(filtration);
    bool success = true;
    std::vector<std::vector<double>> results;
    for (auto i_index : pc._I) {
        auto coefs_Z = LiftingToZ(filtration, pc._alphas[i_index]);
        if (coefs_Z.has_value()) {
            results.push_back(
                LiftingToR(filtration, coefs_Z.value())
            );
        } else {
            success = false;
            break;
        }
    }
    if (success) {
        return results;
    } else {
        return {};
    }
}

}

template<int dim>
std::optional<std::vector<std::vector<double>>> CalculateGlobalCircularCoordinate(
    const Eigen::Matrix<double, Eigen::Dynamic, dim>& point_cloud,
    const double threshold
) {
    auto filtration = GenerateFiltration(point_cloud, threshold);
    constexpr int primes[] = {47, 53, 59, 997};
    auto result = internal::TryWithPrime<47>(filtration);
    if (result.has_value()) {
        return result.value();
    }
    result = internal::TryWithPrime<53>(filtration);
    if (result.has_value()) {
        return result.value();
    }
    result = internal::TryWithPrime<59>(filtration);
    if (result.has_value()) {
        return result.value();
    }
    result = internal::TryWithPrime<997>(filtration);
    return result;
}

template<int dim>
std::optional<std::vector<std::vector<double>>> CalculateLocalCircularCoordinate(
    const Eigen::Matrix<double, Eigen::Dynamic, dim> &point_cloud,
    const Eigen::Vector<double, dim>& center,
    const double radius,
    const double threshold
) {
    const int num_vertices = point_cloud.rows();
    std::vector<int> local_vertex_ids;
    for (int i = 0; i < num_vertices; i++) {
        if ((center - point_cloud.row(i)).norm() < radius + threshold) {
            local_vertex_ids.push_back(i);
        }
    }
    const int num_local_vertices = local_vertex_ids.size();
    spdlog::info("{} vertices are considered in the local topology", num_local_vertices);

    Eigen::Matrix<double, Eigen::Dynamic, dim> point_cloud_local = point_cloud[local_vertex_ids];

    std::vector<bool> inside_neighborhood;
    for (int i = 0; i < point_cloud_local.rows(); i++) {
        inside_neighborhood.push_back((point_cloud_local.row(i) - center).norm() < radius);
    }

    Filtration filtration = GenerateFiltration(point_cloud_local);

    std::vector<int> insert_places = {num_local_vertices - 1};
    int extra_vertex_id = num_local_vertices;
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
            insert_places.push_back(simplex_id);
        }

        simplex_id++;
    }

    simplex_id = 0;
    Filtration coned_filtration;
    int insert_place_id = 0;

    for (const auto& simplex : filtration._simplices) {
        Simplex new_simplex;
        new_simplex._dim = simplex._dim;
        new_simplex._vertices = simplex._vertices;
        for (const auto face : simplex._faces) {
            int num_simplices_before = std::lower_bound(insert_places.begin(), insert_places.end(), face)
                                     - insert_places.begin();
            new_simplex._faces.push_back(face + num_simplices_before);
        }
        coned_filtration._simplices.push_back(new_simplex);
        coned_filtration._diameters.push_back(filtration._diameters[simplex_id]);
        
        if (simplex_id == num_local_vertices) {
            Simplex extra_vertex;
            extra_vertex._dim = 0;
            extra_vertex._vertices.push_back(num_local_vertices);

            coned_filtration._simplices.push_back(extra_vertex);
            coned_filtration._diameters.push_back(0);
        }

        if (insert_place_id < insert_places.size() && insert_places[insert_place_id] == simplex_id) {
            Simplex coned_simplex;
            coned_simplex._dim = simplex._dim + 1;
            coned_simplex._vertices.push_back(extra_vertex_id);
            coned_simplex._vertices.insert(
                coned_simplex._vertices.end(),
                simplex._vertices.begin(), simplex._vertices.end()
            );
            coned_simplex._faces.push_back(coned_filtration._simplices.size() - 1);
            for (const auto& face : simplex._faces) {
                // the simplex immediately after face is the added coning simplex
                int num_simplices_before = std::lower_bound(insert_places.begin(), insert_places.end(), face)
                                         - insert_places.begin();
                coned_simplex._faces.push_back(face + num_simplices_before + 1);
            }
            coned_filtration._simplices.push_back(coned_simplex);
            coned_filtration._diameters.push_back(filtration._diameters[simplex_id]);

            insert_place_id++;
        }
        simplex_id++;
    }

    auto result = internal::TryWithPrime<47>(coned_filtration);
    if (result.has_value()) {
        return result.value();
    }
    result = internal::TryWithPrime<53>(coned_filtration);
    if (result.has_value()) {
        return result.value();
    }
    result = internal::TryWithPrime<59>(coned_filtration);
    if (result.has_value()) {
        return result.value();
    }
    result = internal::TryWithPrime<997>(coned_filtration);
    return result;
}

}