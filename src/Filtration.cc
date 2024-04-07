#include "Filtration.hpp"

void TDA::Filtration::UpdateInfo() {
	_num_vertices = 0;
	_num_edges = 0;
	_num_faces = 0;
	for (const auto& simplex : _simplices) {
		switch (simplex._dim) {
			case 0:
				_num_vertices++;
				break;
			case 1:
				_num_edges++;
				break;
			case 2:
				_num_faces++;
				break;
		}
	}
}

std::ostream& TDA::operator<<(std::ostream& out, const Filtration& filtration) {
    const int num_simplices = filtration._simplices.size();
    for (int i = 0; i < num_simplices; i++) {
        if (i > 0) {
            out << std::endl;
        }
        const auto& simplex = filtration._simplices[i];
        out << "[ ";
        for (const auto face : simplex._faces) {
            out << face << " ";
        }
        out << "], [ ";
        for (const auto vertex : simplex._vertices) {
            out << vertex << " ";
        }
        out << "], ";
        out << "dim = " << simplex._dim << ", ";
        out << "diameter = " << filtration._diameters[i];
    }
    return out;
}
