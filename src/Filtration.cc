#include "Filtration.hpp"

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
