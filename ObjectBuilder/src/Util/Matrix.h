/*
 * Matrix.h
 *
 *  Created on: 06/07/2009
 *      Author: osushkov
 */

#ifndef MATRIX_H_
#define MATRIX_H_


#include <cstdlib>
#include <math.h>
#include <iostream>

static const double MM_LOW_VALUE = 1E-9;

template <class T, std::size_t Rows, std::size_t Cols>
class Matrix {
    T myData[Rows][Cols];

  public:

    typedef T value_type;
    typedef Matrix<T, Rows, Cols> this_type;

    // can't inline the constructors (don't know why)
    Matrix(const this_type &prev);
    Matrix(T val);

    Matrix() {
        reset();
    }

    inline this_type& identity(void){
        reset();

        if(Rows != Cols){
            std::cout << "Cannot make identity matrix" << std::endl;
        }
        else {
            for(size_t row = 0; row < Rows; row++){
                myData[row][row] = (T) 1;
            }
        }

        return *this;
    }

    inline void reset(void) {
        for (size_t row = 0 ; row < Rows ; row++) {
            for (size_t col = 0 ; col < Cols ; col++) {
                myData[row][col] = (T) 0;
            }
        }
    }

    inline bool equals(const this_type &other) const {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                if (myData[i][j] != other(i, j)){
                    return false;
                }
            }
        }

        return true;
    }

    value_type& operator()(std::size_t i, std::size_t j) {
        return myData[i][j];
    }

    value_type operator()(std::size_t i, std::size_t j) const {
        return myData[i][j];
    }

    // assignment
    inline this_type& operator=(const this_type &newVals) {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                myData[i][j] = newVals(i, j);
            }
        }

        return *this;
    }

    inline this_type& operator+=(const this_type &newVals) {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                myData[i][j] += newVals(i, j);
            }
        }

        return *this;
    }

    inline this_type& operator-=(const this_type &newVals) {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                myData[i][j] -= newVals(i, j);
            }
        }

        return *this;
    }


    template <std::size_t intSize>
    this_type& isMult(const Matrix<T, Rows, intSize> &A,
                      const Matrix<T, intSize, Cols> &B) {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                myData[i][j] = 0;
            }
        }

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < intSize; j++) {
                for (size_t k = 0; k < Cols; k++) {
                    myData[i][k] += A(i, j) * B(j, k);
                }
            }
        }

        return *this;
    }

    this_type& preMult( const Matrix<T, Rows, Rows> &M) {
        T temp[Rows][Cols];
        for (size_t r = 0 ; r < Rows ; r++) {
            for (size_t c = 0 ; c < Cols ; c++) {
                temp[r][c] = myData[r][c];
            }
        }

        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                myData[i][j] = 0;
            }
        }

        for (size_t r = 0 ; r < Rows ; r++) {
            for (size_t c = 0 ; c < Cols ; c++) {
                for (size_t i = 0 ; i < Rows ; i++) {
                    myData[r][c] += M(r, i) * temp[i][c];
                }
            }
        }

        return *this;
    }

    inline this_type& isTranspose(const Matrix<T, Cols, Rows> &M) {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                myData[i][j] = M(j, i);
            }
        }
        return *this;
    }

    inline this_type & operator+=(const value_type val) {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                myData[i][j] += val;
            }
        }

        return *this;
    }

    inline this_type & operator-=(const value_type val) {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                myData[i][j] -= val;
            }
        }

        return *this;
    }

    inline this_type & operator*=(const value_type val) {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                myData[i][j] *= val;
            }
        }

        return *this;
    }

    inline this_type & operator/=(const value_type val) {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                myData[i][j] /= val;
            }
        }

        return *this;
    }

    inline this_type & operator^=(int power) {
        this_type temp = *this;

        for (int i = 1 ; i < power ; i++) {
            this->preMult(temp);
        }

        return *this;
    }

    inline this_type & addMult(const value_type k, const this_type &m) {
        for (size_t i = 0; i < Rows; i++) {
            for (size_t j = 0; j < Cols; j++) {
                myData[i][j] += k*m(i,j);
            }
        }

        return *this;
    }

    // partial pivoting method
    int pivot(size_t row);

    void isInverse(const this_type &m);

    // calculate the determinant of a matrix
    inline value_type Det() {
        size_t i,j,k;
        T piv,detVal = T(1);

        if (Rows != Cols) {
            std::cout << "MatInv: Determinant of a non-square matrix" << std::endl;
        }

        this_type temp(*this);

        for (k=0; k < Rows; k++) {
            int indx = temp.pivot(k);
            if (indx == -1) return 0;
            if (indx != 0) detVal = - detVal;
            detVal = detVal * temp(k,k);

            for (i=k+1; i < Rows; i++) {
                piv = temp(i,k) / temp(k,k);

                for (j=k+1; j < Rows; j++)
                    temp(i,j) -= piv * temp(k,j);
            }
        }

        return detVal;
    }

    inline void const printOut() {
        for (size_t row = 0 ; row < Rows ; row++) {
            std::cout << "(";
            for (size_t col = 0 ; col < Cols ; col++) {
                printf("\t%1.4f", myData[row][col]);
                //std::cout << "\t" << myData[row][col];
            }
            std::cout << "\t)" << std::endl;
        }
    }

    inline this_type& dot(const this_type &newVals) {
        for (size_t row = 0 ; row < Rows ; row++) {
            for (size_t col = 0 ; col < Cols ; col++) {
                myData[row][col] *= newVals[row][col];
            }
        }

        return *this;
    }

};


typedef Matrix<float, 2, 1> MVec2;
typedef Matrix<float, 3, 1> MVec3;
typedef Matrix<float, 1, 2> MRowVec2;
typedef Matrix<float, 2, 2> Matrix2;
typedef Matrix<float, 3, 3> Matrix3;
typedef Matrix<float, 4, 4> Matrix4;


inline Matrix3 operator*(const Matrix3 &m1, const Matrix3 &m2){
    Matrix3 result;
    result.isMult(m1, m2);
    return result;
}

inline Matrix4 operator*(const Matrix4 &m1, const Matrix4 &m2){
    Matrix4 result;
    result.isMult(m1, m2);
    return result;
}


// Some complex functions can't be inlined so they are defined here to avoid
// compiler warnings.
template <class T, std::size_t Rows, std::size_t Cols>
Matrix<T, Rows, Cols>::Matrix(const this_type &prev) {
    for (size_t i = 0; i < Rows; i++) {
        for (size_t j = 0; j < Cols; j++) {
            myData[i][j] = prev(i, j);
        }
    }
}

template <class T, std::size_t Rows, std::size_t Cols>
Matrix<T, Rows, Cols>::Matrix(T val) {
    for (size_t i = 0 ; i < Rows; i++) {
        for (size_t j = 0; j < Cols; j++) {
            myData[i][j] = (i == j) ? val : (T) 0.0;
        }
    }
}

template <class T, std::size_t Rows, std::size_t Cols>
int Matrix<T, Rows, Cols>::pivot(size_t row) {
    int k = int(row);
    double amax, temp;

    if (myData[row][row] != T(0)) {
        return 0;
    }
    amax = -1;
    for (size_t i = row; i < Rows; i++) {
        if ((temp = fabs(myData[i][row])) > amax && temp != 0.0) {
            amax = temp;
            k = i;
        }
    }
    if (myData[k][row] == T(0)) {
        return -1;
    }

    if (k != int(row)) {
        T buf;
        for (size_t i = 0; i < Cols; i++) {
            buf = myData[k][i];
            myData[k][i] = myData[row][i];
            myData[row][i] = buf;
        }
        return k;
    }
    return 0;
}

template <class T, std::size_t Rows, std::size_t Cols>
void Matrix<T, Rows, Cols>::isInverse(const this_type &m) {
    if (Rows != Cols) {
        std::cout << "MatInv: Inversion of a non-square matrix" << std::endl;
    }

    if (Rows == 2) {
        // special formula case for 2x2 matrix
        T det = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
        if (fabs(det) < MM_LOW_VALUE) {
            std::cout << "MatInv(2): Inversion of singular matrix, det=" << det
                      << std::endl;
            det = (T)MM_LOW_VALUE;
        }
        // will assume matrix was non-singular
        T detInv = 1 / det;
        myData[0][0] = detInv * m(1, 1);
        myData[0][1] = -(detInv * m(0, 1));
        myData[1][0] = -(detInv * m(1, 0));
        myData[1][1] = detInv * m(0, 0);
        return;
    }
    else if (Rows == 3) {
        // special formula case for 3x3 matrix
        T det = m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) -
                m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
                m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));

        if (fabs(det) < MM_LOW_VALUE) {
            std::cout << "MatInv(3): Inversion of singular matrix, det=" << det
                      << std::endl;
            det = (T)MM_LOW_VALUE;
        }

        // will assume matrix was non-singular
        T detInv = 1 / det;
        myData[0][0] = detInv * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1));
        myData[0][1] = detInv * (m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2));
        myData[0][2] = detInv * (m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1));
        myData[1][0] = detInv * (m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2));
        myData[1][1] = detInv * (m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0));
        myData[1][2] = detInv * (m(0, 2) * m(1, 0) - m(0, 0) * m(1, 2));
        myData[2][0] = detInv * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
        myData[2][1] = detInv * (m(0, 1) * m(2, 0) - m(0, 0) * m(2, 1));
        myData[2][2] = detInv * (m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0));
        return;
    }
    else if (Rows == 4) {
        // partial determinants, missing one row and column
        T pdet00 = (m(1, 1) * (m(2, 2) * m(3, 3) - m(2, 3) * m(3, 2)))
            - (m(1, 2) * (m(2, 1) * m(3, 3) - m(2, 3) * m(3, 1)))
            + (m(1, 3) * (m(2, 1) * m(3, 2) - m(2, 2) * m(3, 1)));
        T pdet01 = (m(1, 0) * (m(2, 2) * m(3, 3) - m(2, 3) * m(3, 2)))
            - (m(1, 2) * (m(2, 0) * m(3, 3) - m(2, 3) * m(3, 0)))
            + (m(1, 3) * (m(2, 0) * m(3, 2) - m(2, 2) * m(3, 0)));
        T pdet02 = (m(1, 0) * (m(2, 1) * m(3, 3) - m(2, 3) * m(3, 1)))
            - (m(1, 1) * (m(2, 0) * m(3, 3) - m(2, 3) * m(3, 0)))
            + (m(1, 3) * (m(2, 0) * m(3, 1) - m(2, 1) * m(3, 0)));
        T pdet03 = (m(1, 0) * (m(2, 1) * m(3, 2) - m(2, 2) * m(3, 1)))
            - (m(1, 1) * (m(2, 0) * m(3, 2) - m(2, 2) * m(3, 0)))
            + (m(1, 2) * (m(2, 0) * m(3, 1) - m(2, 1) * m(3, 0)));
        T pdet10 = (m(0, 1) * (m(2, 2) * m(3, 3) - m(2, 3) * m(3, 2)))
            - (m(0, 2) * (m(2, 1) * m(3, 3) - m(2, 3) * m(3, 1)))
            + (m(0, 3) * (m(2, 1) * m(3, 2) - m(2, 2) * m(3, 1)));
        T pdet11 = (m(0, 0) * (m(2, 2) * m(3, 3) - m(2, 3) * m(3, 2)))
            - (m(0, 2) * (m(2, 0) * m(3, 3) - m(2, 3) * m(3, 0)))
            + (m(0, 3) * (m(2, 0) * m(3, 2) - m(2, 2) * m(3, 0)));
        T pdet12 = (m(0, 0) * (m(2, 1) * m(3, 3) - m(2, 3) * m(3, 1)))
            - (m(0, 1) * (m(2, 0) * m(3, 3) - m(2, 3) * m(3, 0)))
            + (m(0, 3) * (m(2, 0) * m(3, 1) - m(2, 1) * m(3, 0)));
        T pdet13 = (m(0, 0) * (m(2, 1) * m(3, 2) - m(2, 2) * m(3, 1)))
            - (m(0, 1) * (m(2, 0) * m(3, 2) - m(2, 2) * m(3, 0)))
            + (m(0, 2) * (m(2, 0) * m(3, 1) - m(2, 1) * m(3, 0)));
        T pdet20 = (m(0, 1) * (m(1, 2) * m(3, 3) - m(1, 3) * m(3, 2)))
            - (m(0, 2) * (m(1, 1) * m(3, 3) - m(1, 3) * m(3, 1)))
            + (m(0, 3) * (m(1, 1) * m(3, 2) - m(1, 2) * m(3, 1)));
        T pdet21 = (m(0, 0) * (m(1, 2) * m(3, 3) - m(1, 3) * m(3, 2)))
            - (m(0, 2) * (m(1, 0) * m(3, 3) - m(1, 3) * m(3, 0)))
            + (m(0, 3) * (m(1, 0) * m(3, 2) - m(1, 2) * m(3, 0)));
        T pdet22 = (m(0, 0) * (m(1, 1) * m(3, 3) - m(1, 3) * m(3, 1)))
            - (m(0, 1) * (m(1, 0) * m(3, 3) - m(1, 3) * m(3, 0)))
            + (m(0, 3) * (m(1, 0) * m(3, 1) - m(1, 1) * m(3, 0)));
        T pdet23 = (m(0, 0) * (m(1, 1) * m(3, 2) - m(1, 2) * m(3, 1)))
            - (m(0, 1) * (m(1, 0) * m(3, 2) - m(1, 2) * m(3, 0)))
            + (m(0, 2) * (m(1, 0) * m(3, 1) - m(1, 1) * m(3, 0)));
        T pdet30 = (m(0, 1) * (m(1, 2) * m(2, 3) - m(1, 3) * m(2, 2)))
            - (m(0, 2) * (m(1, 1) * m(2, 3) - m(1, 3) * m(2, 1)))
            + (m(0, 3) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)));
        T pdet31 = (m(0, 0) * (m(1, 2) * m(2, 3) - m(1, 3) * m(2, 2)))
            - (m(0, 2) * (m(1, 0) * m(2, 3) - m(1, 3) * m(2, 0)))
            + (m(0, 3) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)));
        T pdet32 = (m(0, 0) * (m(1, 1) * m(2, 3) - m(1, 3) * m(2, 1)))
            - (m(0, 1) * (m(1, 0) * m(2, 3) - m(1, 3) * m(2, 0)))
            + (m(0, 3) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0)));
        T pdet33 = (m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)))
            - (m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)))
            + (m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0)));

        T det = m(0, 0)*pdet00 - m(0, 1)*pdet01 + m(0, 2)*pdet02 - m(0, 3)*pdet03;

        if (fabs(det) < MM_LOW_VALUE) {
            std::cout << "MatInv: Inversion of singular matrix, det="
                      << det << std::endl;
            det = (T)MM_LOW_VALUE;
        }

        // will assume matrix was non-singular
        T detInv = 1 / det;
        myData[0][0] = detInv * pdet00;
        myData[0][1] = -detInv * pdet10;
        myData[0][2] = detInv * pdet20;
        myData[0][3] = -detInv * pdet30;
        myData[1][0] = -detInv * pdet01;
        myData[1][1] = detInv * pdet11;
        myData[1][2] = -detInv * pdet21;
        myData[1][3] = detInv * pdet31;
        myData[2][0] = detInv * pdet02;
        myData[2][1] = -detInv * pdet12;
        myData[2][2] = detInv * pdet22;
        myData[2][3] = -detInv * pdet32;
        myData[3][0] = -detInv * pdet03;
        myData[3][1] = detInv * pdet13;
        myData[3][2] = -detInv * pdet23;
        myData[3][3] = detInv * pdet33;
        return;
    }


    for (size_t i = 0 ; i < Rows; i++) {
        for (size_t j = 0; j < Cols; j++) {
            myData[i][j] = (i == j) ? (T) 1.0 : (T) 0.0;
        }
    }

    this_type m_copy(m);

    for (size_t k = 0; k < Rows; k++) {
        int indx = m_copy.pivot(k);
        if (indx == -1) {
            std::cout << "MatInv: Inversion of a non-square matrix" << std::endl;
        }

        if (indx != 0) {
            for (size_t i = 0; i < Cols; i++) {
                T tempElt = myData[k][i];
                myData[k][i] = myData[indx][i];
                myData[indx][i] = tempElt;
            }
        }

        T a1 = m_copy(k, k);
        for (size_t j = 0; j < Rows; j++) {
            m_copy(k, j) /= a1;
            myData[k][j] /= a1;
        }

        for (size_t i = 0; i < Rows; i++) {
            if (i != k) {
                T a2 = m_copy(i, k);
                for (size_t j = 0; j < Rows; j++) {
                    m_copy(i, j) -= a2 * m_copy(k, j);
                    myData[i][j] -= a2 * myData[k][j];
                }
            }
        }
    }
}

#endif
