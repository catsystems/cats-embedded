/*
 * math_utils.c
 *
 *  Created on: Dec 15, 2020
 *      Author: Jonas
 */

#include "control/math_utils.h"

void matmul(int m, int n, int o, float A[m][n], float B[n][o], float C[m][o]) {
	for(int i = 0; i < m; i++){
		for(int j = 0; j < o; j++){
			for(int k = 0; k < n; k++){
				C[i][j] +=  A[i][k] * B[k][j];
			}
		}
	}
}

void matvecprod(int m, int n, float A[m][n], float b[n], float c[m]) {
	for(int i = 0; i < m; i++){
		for(int j = 0; j < n; j++){
			c[i] += A[i][j] * b[j];
		}
	}

}
void eye(int dim, float A[dim][dim]) {
	for(int i = 0; i < dim; i++){
		for(int j = 0; j < dim; j++){
			if (i == j){
				A[i][j] = 1;
			} else {
				A[i][j] = 0;
			}
		}
	}
}

void transpose(int m, int n, float A[m][n], float A_T[n][m]) {
	for(int i = 0; i < m; i++){
		for(int j = 0; j < n; j++){
			A_T[j][i] = A[i][j];
		}
	}
}

void vecadd(int n, float a[n], float b[n], float c[n]) {
	for(int i = 0; i < n; i++){
		c[i] = a[i] + b[i];
	}
}

void vecsub(int n, float a[n], float b[n], float c[n]) {
	for(int i = 0; i < n; i++){
		c[i] = a[i] - b[i];
	}
}

void matadd(int m, int n, float A[m][n], float B[m][n], float C[m][n]) {
	for(int i = 0; i < m; i++){
		for(int j = 0; j < n; j++){
			C[i][j] =  A[i][j] + B[i][j];
		}
	}
}

void matsub(int m, int n, float A[m][n], float B[m][n], float C[m][n]) {
	for(int i = 0; i < m; i++){
		for(int j = 0; j < n; j++){
			C[i][j] =  A[i][j] - B[i][j];
		}
	}
}

/* Function to get cofactor of A[p][q] in temp[][]. n is current dimension of A[][] */
/* https://www.geeksforgeeks.org/adjoint-inverse-matrix/ */
void cofactor(int dim, float A[dim][dim], float temp[dim][dim], int p, int q, int n)
{
    int i = 0, j = 0;

    // Looping for each element of the matrix
    for (int row = 0; row < n; row++)
    {
        for (int col = 0; col < n; col++)
        {
            //  Copying into temporary matrix only those element
            //  which are not in given row and column
            if (row != p && col != q)
            {
                temp[i][j++] = A[row][col];

                // Row is filled, so increase row index and
                // reset col index
                if (j == n - 1)
                {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

/* Recursive function for finding determinant of matrix. n is current dimension of A[][]. */
/* https://www.geeksforgeeks.org/adjoint-inverse-matrix/ */
float determinant(int dim, float A[dim][dim], int n)
{
    float D = 0; // Initialize result

    //  Base case : if matrix contains single element
    if (n == 1)
        return A[0][0];

    float temp[dim][dim]; // To store cofactors

    int sign = 1;  // To store sign multiplier

     // Iterate for each element of first row
    for (int f = 0; f < n; f++)
    {
        // Getting Cofactor of A[0][f]
        cofactor(dim, A, temp, 0, f, n);
        D += sign * A[0][f] * determinant(dim, temp, n - 1);

        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}

/* Function to get adjoint of A[dim][dim] in adj[dim][dim]. */
/* https://www.geeksforgeeks.org/adjoint-inverse-matrix/ */
void adjoint(int dim, float A[dim][dim], float adj[dim][dim])
{
    if (dim == 1)
    {
        adj[0][0] = 1;
        return;
    }

    // temp is used to store cofactors of A[][]
    int sign = 1;
	float temp[dim][dim];

    for (int i=0; i<dim; i++)
    {
        for (int j=0; j<dim; j++)
        {
            // Get cofactor of A[i][j]
            cofactor(dim, A, temp, i, j, dim);

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i+j)%2==0)? 1: -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign)*(determinant(dim, temp, dim-1));
        }
    }
}

/* Function to calculate and store inverse, returns false if matrix is singular */
/* https://www.geeksforgeeks.org/adjoint-inverse-matrix/ */
osStatus_t inverse(int dim, float A[dim][dim], float A_inv[dim][dim], float lambda)
{
	/* add damping factor to avoid singularities. */
	/* if no damping is required set lambda to 0.0 */
	float A_dash[dim][dim];
    memcpy(A_dash, A, dim * dim * sizeof(A[0][0]));
    for (int i=0; i<dim; i++) {
        A_dash[i][i] = A_dash[i][i] + lambda * lambda;
	}

    // Find determinant of A[][]
    float det = determinant(dim, A_dash, dim);

    if (det == 0)
    {
        return osError;
    }

    // Find adjoint
    float adj[dim][dim];
    adjoint(dim, A_dash, adj);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (int i=0; i<dim; i++) {
        for (int j=0; j<dim; j++) {
            A_inv[i][j] = adj[i][j] / ((float) det);
		}
	}

    return osOK;
}
