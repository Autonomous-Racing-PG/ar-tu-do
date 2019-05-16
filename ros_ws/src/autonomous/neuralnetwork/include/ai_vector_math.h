#pragma once

#include <vector>
#include <cstdlib>
#include <algorithm>

inline void add(std::vector<fann_type> &a, std::vector<fann_type> &b) {
	for(unsigned int i = 0; i < a.size(); i++) {
		a[i] = a[i] + b[i];
	}
}

inline void add(std::vector<fann_type> &a, fann_type b) {
	for(unsigned int i = 0; i < a.size(); i++) {
		a[i] = a[i] + b;
	}
}

inline void mult(std::vector<fann_type> &a, std::vector<fann_type> &b) {
	for(unsigned int i = 0; i < a.size(); i++) {
		a[i] = a[i] * b[i];
	}
}

inline void mult(std::vector<fann_type> &a, fann_type b) {
	for(unsigned int i = 0; i < a.size(); i++) {
		a[i] = a[i] * b;
	}
}

inline void r_mult_all(std::vector<fann_type> &mult) {
	for(unsigned int i = 0; i < mult.size(); i++) {
		fann_type r = (fann_type)std::rand() / RAND_MAX;
		fann_type b = rand() % 2 == 0 ? 1.0f : -1.0f;
		mult[i] = r * b;
	}
}

inline void r_mult_one(std::vector<fann_type> &mult) {
	int i = std::rand() / (RAND_MAX / mult.size());
	fann_type r = (fann_type)std::rand() / RAND_MAX;
	fann_type b = rand() % 2 == 0 ? 1.0f : -1.0f;
	mult[i] = r * b;
}

inline std::vector<fann_type> random_vector(long size, fann_type rate, void (*f)(std::vector<fann_type>&)) {
	std::vector<fann_type> vec(size);
	srand(time(NULL));
	f(vec);			    // -1 < m[i] < 1
	mult(vec, rate);	// -scale < m[i] < scale
	add(vec, 1.0f);	    // 1-scale < m[i] < 1+scale
	return vec;
}