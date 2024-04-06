#include <fstream>
#include <cmath>
#include <filesystem>
#include "TestConfig.hpp"

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
	std::ofstream output(fs::path(TDA_MODEL_DIR) / (std::string("gen_") + argv[1]));

	int num_vertices = std::atoi(argv[2]);
	double delta_theta = 2 * std::acos(-1) / num_vertices;
	for (int i = 0; i < num_vertices; i++) {
		output << "v 0 " << std::cos(delta_theta * i) << " " << std::sin(delta_theta * i) << std::endl;
	}

	output.close();
	return 0;
}