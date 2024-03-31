#pragma once

#include "Filtration.hpp"

namespace TDA {

template<int p>
struct Zp {
    static_assert(p != 2, "Must use a prime number larger than 2");

    int _val;

    Zp operator+(const Zp& rhs) const;
    Zp operator-() const;
    Zp operator-(const Zp& rhs) const;
    Zp operator*(const Zp& rhs) const;

    Zp Inv() const;
    Zp operator/(const Zp& rhs) const;

    Zp GetZp(int val);

    int Cast2Int() {
        return _val - (p - 1) / 2;
    }

private:
    Zp(int val) : _val(val) {}

    static bool _initialized;
    static std::array<int, p> _inverse;
    static void Initialize();
};

template <int p>
struct PersistentCohomology {
    PersistentCohomology(int num_simplices):
        _num_simplices(num_simplices), _alphas(num_simplices, std::vector<Zp<p>>(num_simplices)) {}

    int _num_simplices;
    std::set<int> _I;
    std::map<int, int> _PQ_map;
    std::vector<std::vector<Zp<p>>> _alphas;
};

template<int p>
PersistentCohomology<p> GeneratePersistentCohomology(
    const Filtration& filtraton
);

}

// Implementation
namespace TDA {

namespace internal {

template<typename T>
T CoboundaryMap(const std::vector<T>& coefs, const Simplex& simplex) {
    bool sign_positive = true;
    T val = 0;
    for (const auto face : simplex._faces) {
        if (sign_positive) {
            val += coefs[face];
        } else {
            val -= coefs[face];
        }
        sign_positive = sign_positive ? false : true;
    }
    return val;
}

}



template<int p>
PersistentCohomology<p> GeneratePersistentCohomology(
    const Filtration& filtraton
) {
    const int num_simplices = filtraton._simplices.size();
    PersistentCohomology<p> pc(num_simplices);

    int simplex_cnt = 0;
    for (const auto& simplex : filtraton._simplices) {
        std::vector<Zp<p>> c(simplex_cnt);
        for (int i = 0; i < simplex_cnt; i++) {
            c[i] = internal::CoboundaryMap(pc._alphas[i], simplex);
        }

        // Deal with P & Q
        for (auto [p_index, q_index] : pc._PQ_map) {
            pc._alphas[q_index][simplex_cnt] = c[p_index];
        }

        int max_non_zero_index = -1;
        for (auto itr =  pc._I.rbegin(); itr != pc._I.rend(); ++itr) {
            auto i_index = *itr;
            if (c[i_index] != 0) {
                max_non_zero_index = i_index;
                break;
            }
        }
        
        if (max_non_zero_index < 0) {
            // all zero
            pc._I.insert(simplex_cnt);
            pc._alphas[simplex_cnt][simplex_cnt] = 1;
        } else {
            pc._I.erase(max_non_zero_index);
            pc._PQ_map.insert(std::make_pair(max_non_zero_index, simplex_cnt));
            for (auto i_index : pc._I) {
                auto ratio = c[i_index] / c[max_non_zero_index];
                for (int j = max_non_zero_index; j < simplex_cnt; j++) {
                    pc._alphas[i_index][j] -= ratio * pc._alphas[max_non_zero_index][j] * ratio;
                }
                if (i_index > max_non_zero_index) {
                    break;
                }
            }
        }

        simplex_cnt++;
    }

    return pc;
}

}