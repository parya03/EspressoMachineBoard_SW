/**
 * File for matrix class and representations
 * 
 * Encoder, etc
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <esp_assert.h>
#include <cstring>
#include <string>
#include <esp_log.h>

#define SIZE_DIAGONAL 10 // Side of each side of the statically-allocated buffer
#define buffer_deref(buf, x, y) buf[(x * SIZE_DIAGONAL) + y]

/*
    Multiplies matrices A (rowsA × colsA) and B (colsA × colsB)
    Stores result in C (rowsA × colsB).
    All matrices are passed as 1D float arrays in row-major order.
*/
static void matmul(const float A[], int rowsA, int colsA,
    const float B[], int rowsB, int colsB,
    float C[]) {
    if (colsA != rowsB) {
        assert(0 && "Error: Incompatible dimensions for multiplication");
        return;
    }

    // Compute C = A × B
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {
            float sum = 0.0f;
                for (int k = 0; k < colsA; k++) {
                    sum += A[i * colsA + k] * B[k * colsB + j];
                }
            C[i * colsB + j] = sum;
        }
    }
}

static void transposeMatrix(const float* A, int rows, int cols, float* B) {
    // for (int i = 0; i < rows; i++) {
    //     for (int j = 0; j < cols; j++) {
    //         B[j * rows + i] = A[i * cols + j];
    //     }
    // }

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // printf("%d %d\n", i, j);
            buffer_deref(B, j, i) = buffer_deref(A, i, j);
        }
    }
}

class Matrix {
    public:
    const int &rows = _rows;
    const int &cols = _cols;

    Matrix(int rows, int cols) {
        // data = new float*[rows];
        // for(int i = 0; i < rows; i++) {
        //     data[i] = new float[cols];
        // }

        this->_rows = rows;
        this->_cols = cols;

        // 0 initialize
        for(int i = 0; i < rows; i++) {
            for(int o = 0; o < cols; o++) {
                buffer_deref(data, i, o) = 0;
            }
        }
    }

    Matrix(int rows, int cols, float data_in[100]) {
        // data = new float*[rows];
        // for(int i = 0; i < rows; i++) {
        //     data[i] = new float[cols];
        // }

        this->_rows = rows;
        this->_cols = cols;

        // 0 initialize
        for(int i = 0; i < rows; i++) {
            for(int o = 0; o < cols; o++) {
                buffer_deref(data, i, o) = data_in[(i * rows) + o];
            }
        }
    }

    Matrix(int rows, int cols, float data_in) {
        // data = new float*[rows];
        // for(int i = 0; i < rows; i++) {
        //     data[i] = new float[cols];
        // }

        this->_rows = rows;
        this->_cols = cols;

        // 0 initialize
        for(int i = 0; i < rows; i++) {
            for(int o = 0; o < cols; o++) {
                buffer_deref(data, i, o) = data_in;
            }
        }
    }

    ~Matrix() {
        // for(int i = 0; i < rows; i++) {
        //     delete[] data[i];
        // }

        // delete[] data;

        this->_rows = 0;
        this->_cols = 0;
    }

    float *operator[](int row_index) {
        return &(data[row_index * 10]);
    }

    Matrix &operator=(const Matrix& other) {
        // Copy other matrix into this one
        
        // Bounds check
        if(other.rows != rows || other.cols != cols) {
            assert(0 && "Matrix assignment with improper bounds!");
        }

        // // Deallocate if already has data
        // if(data) {
        //     for(int i = 0; i < rows; i++) {
        //         delete[] data[i];
        //     }
    
        //     delete[] data;
        // }

        // data = new float*[rows];
        // for(int i = 0; i < rows; i++) {
        //     data[i] = new float[cols];
        // }

        for(int i = 0; i < rows; i++) {
            for(int o = 0; o < cols; o++) {
                buffer_deref(data, i, o) = buffer_deref(other.data, i, o);
            }
        }

        return *this;
    }

    Matrix &operator=(const float* other) {
        // // Deallocate if already has data
        // if(data) {
        //     for(int i = 0; i < rows; i++) {
        //         delete[] data[i];
        //     }
    
        //     delete[] data;
        // }
        // data = new float*[rows];
        // for(int i = 0; i < rows; i++) {
        //     data[i] = new float[cols];
        // }

        // Assume the inputted matrix 2D array has same dimension. If it doesn't then womp womp.
        for(int i = 0; i < rows; i++) {
            for(int o = 0; o < cols; o++) {
                buffer_deref(data, i, o) = other[(i * rows) + o];
            }
        }

        return *this;
    }

    bool operator==(const float* other) {

        for(int i = 0; i < rows; i++) {
            for(int o = 0; o < cols; o++) {
                if(buffer_deref(data, i, o) != other[(i * rows) + o]) {
                    return false;
                }
            }
        }

        return true;
    }

    bool operator==(const Matrix& other) {
        
        if(!is_same_size(other)) {
            return false;
        }

        for(int i = 0; i < rows; i++) {
            for(int o = 0; o < cols; o++) {
                if(buffer_deref(data, i, o) != buffer_deref(other.data, i, o)) {
                    return false;
                }
            }
        }

        return true;
    }

    Matrix operator+(const Matrix& other) {
        if(this->is_same_size(other)) {
            float temp[100];
            memcpy(temp, data, 100);

            for(int i = 0; i < rows; i++) {
                for(int o = 0; o < cols; o++) {
                    buffer_deref(temp, i, o) += buffer_deref(other.data, i, o);
                }
            }

            return Matrix(rows, cols, temp);
        }
        else {
            assert(0 && "Matrix addition with different matrix sizes!");
        }
    }

    Matrix operator-(const Matrix& other) {
        if(this->is_same_size(other)) {
            float temp[100];
            memcpy(temp, data, 100);

            for(int i = 0; i < rows; i++) {
                for(int o = 0; o < cols; o++) {
                    buffer_deref(temp, i, o) -= buffer_deref(other.data, i, o);
                }
            }

            return Matrix(rows, cols, temp);
        }
        else {
            assert(0 && "Matrix addition with different matrix sizes!");
        }
    }

    Matrix operator*(const Matrix& other) {
        float temp[100];

        matmul((float *)data, rows, cols, (float *)other.data, other.rows, other.cols, (float *)temp);
        return Matrix(rows, other.cols, temp);
    }

    Matrix transpose() {
        float temp[100] = { 0 };
        transposeMatrix((const float *)data, rows, cols, (float *)temp);

        auto mat = Matrix(cols, rows, 0.0f);
        mat._copy_buf(temp);

        return mat;
    }

    void print() {
        printf("----------\n");
        for(int i = 0; i < rows; i++) {
            printf("[ ");
            for(int o = 0; o < cols; o++) {
                printf("%f, ", buffer_deref(data, i, o));
            }
            printf(" ]\n");
        }
        printf("----------\n");
    }

    bool is_same_size(const Matrix& other) {
        if(_rows == other._rows && _cols == other.cols) {
            return true;
        }

        return false;
    }

    private:
    int _rows = 0;
    int _cols = 0;

    void _copy_buf(float in[SIZE_DIAGONAL * SIZE_DIAGONAL]) {
        for(int i = 0; i < SIZE_DIAGONAL * SIZE_DIAGONAL; i++) {
            this->data[i] = in[i];
        }

        return;
    }

    // data[rows][cols]
    // float **data = 0;
    float data[SIZE_DIAGONAL * SIZE_DIAGONAL]; // Static size for now - 10x10
};

#endif