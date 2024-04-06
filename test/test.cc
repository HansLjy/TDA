#include "gtest/gtest.h"
#include "PersistentCohomology.hpp"
#include "igl/readOBJ.h"
#include "TestConfig.hpp"
#include <filesystem>

namespace fs = std::filesystem;

TEST(PersistentCohomologyTest, CoboundaryTest) {
	double threshold = 1.45;
	Eigen::MatrixX3d V;
	Eigen::MatrixX3i F;
	igl::readOBJ(fs::path(TDA_MODEL_DIR) / "gen_circle_8.obj", V, F);

	auto filtration = TDA::GenerateFiltration(V, threshold);
	std::cerr << filtration << std::endl;
	auto pc = TDA::GeneratePersistentCohomology<53>(filtration);

	const int num_simplices = filtration._simplices.size();

	for (auto i_index : pc._I) {
		for (const auto& simplex : filtration._simplices) {
			EXPECT_EQ(TDA::internal::CoboundaryMap(pc._alphas[i_index], simplex), TDA::Zp<53>(0));
		}
	}

	for (auto [p_index, q_index] : pc._PQ_map) {
		for (int i = 0; i < num_simplices; i++) {
			EXPECT_EQ(
				TDA::internal::CoboundaryMap(
					pc._alphas[p_index],
					filtration._simplices[i]
				),
				pc._alphas[q_index][i]
			);
		}
	}
}