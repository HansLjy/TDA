#pragma once

#include "PersistentCohomology.hpp"
#include "Eigen/Sparse"

namespace TDA {

template<int p>
std::optional<std::vector<int>> LiftingToZ(
    const Filtration& filtration,
    const std::vector<Zp<p>>& coefs
);

inline std::vector<double> LiftingToR(
    const Filtration& filtration,
    const std::vector<int>& coefs
);

}

// implementation
namespace TDA {

namespace internal {

inline bool CheckPersistentCohomology(
    const Filtration& filtration,
    const std::vector<int>& coefs
) {
    for (const auto& simplex : filtration._simplices) {
        if (CoboundaryMap(coefs, simplex) != 0) {
            return false;
        }
    }
    return true;
}

}


template<int p>
std::optional<std::vector<int>> LiftingToZ(
    const Filtration& filtration,
    const std::vector<Zp<p>> &coefs
) {
    std::vector<int> lifted_coefs;
    for (const auto& coef : coefs) {
        lifted_coefs.push_back(coef.Cast2Int());
    }

    if (internal::CheckPersistentCohomology(filtration, lifted_coefs)) {
        return lifted_coefs;
    } else {
        return {};
    }
}

inline std::vector<double> LiftingToR(
    const Filtration& filtration,
    const std::vector<int>& coefs
) {
	const int num_vertices = filtration._num_vertices;
	const int num_edges = filtration._num_edges;

    Eigen::SparseMatrix<double> d0(num_edges, num_vertices);
    Eigen::VectorXd b(num_edges);

	int simplex_id = 0;
	int edge_id = 0;
    std::vector<Eigen::Triplet<double>> coo;
    for (const auto& simplex : filtration._simplices) {
        if (simplex._dim == 1) {
            coo.push_back(Eigen::Triplet<double>(
                edge_id, simplex._faces[0], 1
            ));
            coo.push_back(Eigen::Triplet<double>(
                edge_id, simplex._faces[1], -1
            ));
			b(edge_id) = coefs[simplex_id];
			edge_id++;
        }
		simplex_id++;
    }
	d0.setFromTriplets(coo.begin(), coo.end());

    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(d0);

    Eigen::VectorXd f = solver.solve(b);

    std::vector<double> result;
    result.reserve(num_vertices);
    for (int i = 0; i < num_vertices; i++) {
        result.push_back(f(i));
    }
    return result;
}

}