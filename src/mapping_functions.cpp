#include "mapping_functions.hpp"







int helloWorld() {
    Matrix2f matrise;
    matrise.row(0) << 1, 1;
    matrise.row(1) << 2, 2;
    Vector2f vektor = Vector2f(2,1);
    Vector2f sum;
    cout << matrise*vektor << endl;
    sum = matrise*vektor;
    return 1;

}