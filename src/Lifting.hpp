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
    Eigen::SparseMatrix<double> d0;
    std::vector<Eigen::Triplet<double>> coo;

    int simplex_id = 0;
    for (const auto& simplex : filtration._simplices) {
        if (simplex._dim == 1) {
            coo.push_back(Eigen::Triplet<double>(
                simplex_id, simplex._faces[1], 1
            ));
            coo.push_back(Eigen::Triplet<double>(
                simplex_id, simplex._faces[0], -1
            ));
        }
        simplex_id++;
    }

    Eigen::SparseMatrix<double> A = d0.transpose() * d0;
    Eigen::VectorXd coefs_eigen(coefs.size());
    for (int i = 0; i < coefs.size(); i++) {
        coefs_eigen(i) = coefs[i];
    }

    Eigen::VectorXd b = - d0.transpose() * coefs_eigen;
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(A);
    Eigen::VectorXd f = solver.solve(b);

    std::vector<double> result;
    result.reserve(coefs.size());
    for (int i = 0; i < coefs.size(); i++) {
        result.push_back(f(i));
    }
    return result;
}

inline void LiftingToR(
    Gudhi::Simplex_tree<>& simplex_tree,
    const Eigen::VectorXd& alpha,
    Eigen::VectorXd& f
) {
    Eigen::SparseMatrix<double> d0;
    std::vector<Eigen::Triplet<double>> coo;
    for (auto edge : simplex_tree.skeleton_simplex_range(1)) {
        int edge_index = simplex_tree.key(edge);
        // loop through all edges
        double coef = -1;
        for (auto vertex : simplex_tree.boundary_simplex_range(edge)) {
            int vertex_index = simplex_tree.key(vertex);
            coo.push_back(
                Eigen::Triplet<double>(
                    edge_index, vertex_index, coef
                )
            );
            coef = -coef;
        }
    }
    Eigen::SparseMatrix<double> A = d0.transpose() * d0;
    Eigen::VectorXd b = - d0.transpose() * alpha;
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    solver.compute(A);
    f = solver.solve(b);
}

}