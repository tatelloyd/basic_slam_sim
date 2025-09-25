#include "../src/matrix.h"
#include <iostream>
#include <vector>

void validate(std::vector<std::vector<double>> a, std::vector<std::vector<double>> b, const char* msg) {
    if (a != b) {
        std::cout << "Validation failed: " << msg << std::endl;
    } else {
        std::cout << "Validation passed: " << msg << std::endl;
    }
}

int main() {
    // Example usage
    Matrix A(2, 2);
    A.set_data({{1, 2}, {3, 4}});
    
    Matrix B(2, 2, {{5, 6}, {7, 8}});

    Matrix Z(3, 3);
    Z.set_data({{1,2,3},{4,5,6},{7,8,9}});
    
    Matrix Y(3, 3, {{10,11,12},{13, 14, 15},{16, 17, 18}});
    
    //2 x 2 tests
    Matrix C = A * B;
    C.print();

    validate(C.get_data(), {{19, 22}, {43, 50}}, "Matrix Multiplication");
    
    Matrix D = A + B;
    D.print();

    validate(D.get_data(), {{6, 8}, {10, 12}}, "Matrix Addition");
    
    Matrix E = A - B;
    E.print();

    validate(E.get_data(), {{-4, -4}, {-4, -4}}, "Matrix Subtraction");
    
    Matrix F = A.transpose();
    F.print();

    validate(F.get_data(), {{1, 3}, {2, 4}}, "Matrix Transpose");
    
    Matrix G = A.inverse();
    G.print();

    validate(G.get_data(), {{-2, 1}, {1.5, -0.5}}, "Matrix Inverse");

    //3 x 3 tests
    Matrix H = Z * Y;
    H.print();
    validate(H.get_data(), {{84, 90, 96}, {201, 216, 231}, {318, 342, 366}}, "3x3 Matrix Multiplication");

    Matrix I = Z + Y;
    I.print();
    validate(I.get_data(), {{11, 13, 15}, {17, 19, 21}, {23, 25, 27}}, "3x3 Matrix Addition");

    Matrix J = Z - Y;
    J.print();
    validate(J.get_data(), {{-9, -9, -9}, {-9, -9, -9}, {-9, -9, -9}}, "3x3 Matrix Subtraction");

    Matrix K = Z.transpose();
    K.print();
    validate(K.get_data(), {{1, 4, 7}, {2, 5, 8}, {3, 6, 9}}, "3x3 Matrix Transpose");
    
    return 0;
}