/*
 * math_utils.h
 *
 *  Created on: Dec 15, 2020
 *      Author: Jonas
 */

#ifndef INC_CONTROL_MATH_UTILS_H_
#define INC_CONTROL_MATH_UTILS_H_

#include "cmsis_os.h"
#include "string.h"



#endif /* INC_CONTROL_MATH_UTILS_H_ */


/* Calculates the transpose of a Matrix */
void transpose(int m, int n, float A[m][n], float A_T[n][m]);

/* Function to get cofactor of A[p][q] in temp[][]. n is current dimension of A[][] */
void cofactor(int dim, float A[dim][dim], float temp[dim][dim], int p, int q, int n);

/* Recursive function for finding determinant of matrix. n is current dimension of A[][]. */
float determinant(int dim, float A[dim][dim], int n);

/* Function to get adjoint of A[dim][dim] in adj[dim][dim]. */
void adjoint(int dim, float A[dim][dim], float adj[dim][dim]);

/* Function to calculate and store inverse, returns false if matrix is singular */
osStatus_t inverse(int dim, float A[dim][dim], float inverse[dim][dim], float lambda);
