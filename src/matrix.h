#ifndef MATRIX_H
#define MATRIX_H

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>

// Simple matrix class for our SLAM implementation
class Matrix {
public:
    Matrix(int r, int c);

    Matrix(int r, int c, std::vector<std::vector<double>> d);

    Matrix set_data(const std::vector<std::vector<double>>& d);

    std::vector<std::vector<double>> get_data() const;
    
    Matrix operator*(const Matrix& other) const;
    
    Matrix operator+(const Matrix& other) const;
    
    Matrix operator-(const Matrix& other) const;
    
    Matrix transpose() const;
    
    Matrix inverse() const;
    
    void print() const;

    private:
    std::vector<std::vector<double>> data;
    int rows, cols;
};

#endif //MATRIX_H