#include "matrix.h"

// Constructor implementations
Matrix::Matrix(int r, int c) : rows(r), cols(c) {
    data.resize(rows, std::vector<double>(cols, 0.0));
}

Matrix::Matrix(int r, int c, std::vector<std::vector<double>> d) : rows(r), cols(c) {
    data.resize(rows, std::vector<double>(cols, 0.0));
    data = d;
}

Matrix Matrix::set_data(const std::vector<std::vector<double>>& d) {
    if (d.size() != rows || d[0].size() != cols) {
        throw std::runtime_error("Data dimensions do not match matrix dimensions");
    }
    data = d;
    return *this; // Return reference to self for method chaining
}

std::vector<std::vector<double>> Matrix::get_data() const { 
    return data;
}

Matrix Matrix::operator*(const Matrix& other) const {
    Matrix result(rows, other.cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < other.cols; j++) {
            for (int k = 0; k < cols; k++) {
                result.data[i][j] += data[i][k] * other.data[k][j];
            }
        }
    }
    return result;
}

Matrix Matrix::operator+(const Matrix& other) const {
    Matrix result(rows, cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result.data[i][j] = data[i][j] + other.data[i][j];
        }
    }
    return result;
}

Matrix Matrix::operator-(const Matrix& other) const {
    Matrix result(rows, cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result.data[i][j] = data[i][j] - other.data[i][j];
        }
    }
    return result;
}

Matrix Matrix::transpose() const {
    Matrix result(cols, rows);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result.data[j][i] = data[i][j];
        }
    }
    return result;
}

Matrix Matrix::inverse() const {
    // Simple 2x2 matrix inverse for our use case
    if (rows != 2 || cols != 2) {
        throw std::runtime_error("Only 2x2 matrix inverse implemented");
    }
    
    double det = data[0][0] * data[1][1] - data[0][1] * data[1][0];
    if (std::abs(det) < 1e-10) {
        throw std::runtime_error("Matrix is singular");
    }
    
    Matrix result(2, 2);
    result.data[0][0] = data[1][1] / det;
    result.data[0][1] = -data[0][1] / det;
    result.data[1][0] = -data[1][0] / det;
    result.data[1][1] = data[0][0] / det;
    
    return result;
}

void Matrix::print() const {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            std::cout << std::setw(8) << std::fixed << std::setprecision(3) 
                     << data[i][j] << " ";
        }
        std::cout << std::endl;
    }
}