#pragma once

#include "Filtration.hpp"

namespace TDA {

template<int p>
struct Zp {
    static_assert(p != 2, "Must use a prime number larger than 2");

    Zp(int val = 0) : _val(val) {
        assert(0 <= val && val < p);
    }

    int _val;

	bool operator==(const Zp& rhs) const {
		return _val == rhs._val;
	}

    bool operator!=(const Zp& rhs) const {
        return _val != rhs._val;
    }

    Zp<p> operator+(const Zp& rhs) const {
        Zp<p> result = *this;
        result += rhs;
        return result;
    }

    Zp<p>& operator+=(const Zp& rhs) {
        _val += rhs._val;
        if (_val >= p) {
            _val -= p;
        }
        return *this;
    }

    Zp<p> operator-() const {
        return Zp<p>(_val == 0 ? 0 : p - _val);
    }

    Zp<p> operator-(const Zp& rhs) const {
        Zp<p> result = *this;
        result -= rhs;
        return result;
    }

    Zp<p>& operator-=(const Zp& rhs) {
        _val -= rhs._val;
        if (_val < 0) {
            _val += p;
        }
        return *this;
    }

    Zp<p> operator*(const Zp& rhs) const {
        Zp<p> result = *this;
        result *= rhs;
        return result;
    }

    Zp<p>& operator*=(const Zp& rhs) {
        _val = _val * rhs._val % p;
        return *this;
    }

    Zp<p> Inv() const {
        return Pow(_val, p - 2);
    }

    Zp<p> operator/(const Zp& rhs) const {
        return *this * rhs.Inv();
    }

    int Cast2Int() const {
		return (_val > (p - 1) / 2) ? (_val - p) : _val;
    }

private:

    static int Pow(int x, int n) {
        int result = 1;
        while (n) {
            if (n & 1) {
                result *= x;
                result %= p;
            }
            x = x * x % p;
            n >>= 1;
        }
        return result;
    }
};

template<typename T>
struct SparseCoefs {
    struct Term {
        Term(int pos, const T& val) : _pos(pos), _val(val) {}
        int _pos;
        T _val;
    };

    std::vector<Term> _non_zeros;

    // *this -= rhs * ratio
    void Eliminate(const SparseCoefs<T>& rhs, const T& ratio) {
        std::vector<Term> result;
        result.reserve(_non_zeros.size() + rhs._non_zeros.size());

        int cur_index = 0;
        for (const auto& term : rhs._non_zeros) {
            while (cur_index < _non_zeros.size() && _non_zeros[cur_index]._pos < term._pos) {
                result.push_back(_non_zeros[cur_index++]);
            }
            if (cur_index >= _non_zeros.size() || _non_zeros[cur_index]._pos > term._pos) {
                result.push_back(Term(term._pos, - ratio * term._val));
            } else {
                T val = _non_zeros[cur_index]._val - ratio * term._val;
                if (val != 0) {
                    result.push_back(Term(term._pos, val));
                }
                cur_index++;
            }
        }
        while (cur_index < _non_zeros.size()) {
            result.push_back(_non_zeros[cur_index++]);
        }
        _non_zeros = result;
    }

    T operator[](int pos) const {
        auto itr = std::lower_bound(
            _non_zeros.begin(),
            _non_zeros.end(),
            Term(pos, 0),
            [](const Term& lhs, const Term& rhs) {
                return lhs._pos < rhs._pos;
            }
        );
        if (itr == _non_zeros.end() || itr->_pos > pos) {
            return 0;
        }
        return itr->_val;
    }
};

template <int p>
struct PersistentCohomology {

    PersistentCohomology(int num_simplices):
        _num_simplices(num_simplices),
        _coefs(num_simplices) {}

    int _num_simplices;
	std::array<int, 3> _bettis;
    std::set<int> _I;
    std::map<int, int> _PQ_map;
    std::vector<SparseCoefs<Zp<p>>> _coefs;
};

template<int p>
std::ostream& operator<<(std::ostream& out, const PersistentCohomology<p>& pc);

template<int p>
PersistentCohomology<p> GeneratePersistentCohomology(
    const Filtration& filtraton
);

}

// Implementation
namespace TDA {

namespace internal {

template<typename T>
T CoboundaryMap(const SparseCoefs<T>& coefs, const Simplex &simplex) {
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
std::ostream& operator<<(std::ostream& out, const PersistentCohomology<p>& pc) {
    out << "Live cocycles" << std::endl;
    for (const auto index : pc._I) {
        out << index << " ";
    }
    out << std::endl;

    out << "Dead cocycles" << std::endl;
    for (const auto pq : pc._PQ_map) {
        out << pq.first << " " << pq.second << std::endl;
    }
    out << std::endl;

    out << "Coefficients" << std::endl;
    for (int i = 0; i < pc._num_simplices; i++) {
        for (const auto c : pc._coefs[i]) {
            out << c._val << "\t";
        }
        out << std::endl;
    }
    return out;    
}


template<int p>
PersistentCohomology<p> GeneratePersistentCohomology(
    const Filtration& filtration
) {
    const int num_simplices = filtration._simplices.size();
    PersistentCohomology<p> pc(num_simplices);

    int simplex_cnt = 0;
    for (const auto& simplex : filtration._simplices) {
        std::vector<Zp<p>> c(simplex_cnt);
        for (int i = 0; i < simplex_cnt; i++) {
            c[i] = internal::CoboundaryMap(pc._coefs[i], simplex);
        }

        // Deal with P & Q
        for (auto [p_index, q_index] : pc._PQ_map) {
            pc._coefs[q_index]._non_zeros.push_back(
                typename SparseCoefs<Zp<p>>::Term(simplex_cnt, c[p_index])
            );
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
            pc._coefs[simplex_cnt]._non_zeros.push_back(
                typename SparseCoefs<Zp<p>>::Term(simplex_cnt, 1)
            );
        } else {
            pc._I.erase(max_non_zero_index);
            pc._PQ_map.insert(std::make_pair(max_non_zero_index, simplex_cnt));
            for (auto i_index : pc._I) {
                if (i_index >= max_non_zero_index) {
                    break;
                }

                auto ratio = c[i_index] / c[max_non_zero_index];
                if (ratio != 0) {
                    pc._coefs[i_index].Eliminate(pc._coefs[max_non_zero_index], ratio);
                }
            }
            pc._coefs[simplex_cnt]._non_zeros.push_back(
                typename SparseCoefs<Zp<p>>::Term(simplex_cnt, c[max_non_zero_index])
            );
        }
        simplex_cnt++;
    }

	pc._bettis = {0, 0, 0};
	for (const auto i_index : pc._I) {
		pc._bettis[filtration._simplices[i_index]._dim]++;
	}

    return pc;
}

}