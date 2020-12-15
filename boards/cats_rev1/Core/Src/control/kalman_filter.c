/*
 * kalman_filter.c
 *
 *  Created on: Dec 15, 2020
 *      Author: Jonas
 */


#include "control\kalman_filter.h"
#include <string.h>

void initialize_matrices(kalman_filter_t *filter){
	float A_dash[3][3] = {{1, filter->t_sampl, filter->t_sampl*filter->t_sampl/2}, {0, 1, filter->t_sampl}, {0, 0, 1}};
	float A_dash_T[3][3] = { 0 };
	transpose(3, 3, A_dash, A_dash_T);
	float G_dash[3] = {filter->t_sampl*filter->t_sampl/2, filter->t_sampl, 0};
	float B_dash[3] = {filter->t_sampl*filter->t_sampl/2, filter->t_sampl, 0};
	float Q_dash = 1;
	float H_full_dash[3][3] = {{1, 0, 0},{1, 0, 0},{1, 0, 0}};
	float H_full_dash_T[3][3] = { 0 };
	transpose(3, 3, H_full_dash, H_full_dash_T);
	float H_eliminated_dash[2][3] = {{1, 0, 0}, {1, 0, 0}};
	float H_eliminated_dash_T[3][3] = { 0 };
	transpose(2, 3, H_eliminated_dash, H_eliminated_dash_T);
	float R_full_dash[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
	float R_eliminated_dash[2][2] = {{1, 0}, {0, 1}};

	float GdQGd_T_dash[3][3] = { 0 };
	GdQGd_T_dash[0][0] = Q_dash*G_dash[0]*G_dash[0];
	GdQGd_T_dash[0][1] = Q_dash*G_dash[0]*G_dash[1];
	GdQGd_T_dash[0][2] = Q_dash*G_dash[0]*G_dash[2];
	GdQGd_T_dash[1][0] = Q_dash*G_dash[1]*G_dash[0];
	GdQGd_T_dash[1][1] = Q_dash*G_dash[1]*G_dash[1];
	GdQGd_T_dash[1][2] = Q_dash*G_dash[1]*G_dash[2];
	GdQGd_T_dash[2][0] = Q_dash*G_dash[2]*G_dash[0];
	GdQGd_T_dash[2][1] = Q_dash*G_dash[2]*G_dash[1];
	GdQGd_T_dash[2][2] = Q_dash*G_dash[2]*G_dash[2];

	memcpy(filter->Ad, A_dash, 9*sizeof(A_dash[0][0]));
	memcpy(filter->Ad_T, A_dash_T, 9*sizeof(A_dash_T[0][0]));
	memcpy(filter->Gd, G_dash, 3*sizeof(G_dash[0]));
	memcpy(filter->Bd, B_dash, 9*sizeof(B_dash[0]));
	filter->Q = Q_dash;
	memcpy(filter->H_full, H_full_dash, 9*sizeof(H_full_dash[0][0]));
	memcpy(filter->H_full_T, H_full_dash_T, 9*sizeof(H_full_dash_T[0][0]));
	memcpy(filter->H_eliminated, H_eliminated_dash, 6*sizeof(H_eliminated_dash[0][0]));
	memcpy(filter->H_eliminated_T, H_eliminated_dash_T, 6*sizeof(H_eliminated_dash_T[0][0]));
	memcpy(filter->R_full, R_full_dash, 9*sizeof(R_full_dash[0][0]));
	memcpy(filter->R_eliminated, R_eliminated_dash, 4*sizeof(R_eliminated_dash[0][0]));
	memcpy(filter->GdQGd_T, GdQGd_T_dash, 9*sizeof(GdQGd_T_dash[0][0]));
}

void kalman_step(kalman_filter_t *filter, state_estimation_data_t *data){
	float u = 0;
	float placeholder1[3] = { 0 };
	float placeholder2[3] = { 0 };

	/* Prediction Step */

	for(int i = 0; i < 3; i++){
		u = u + data->acceleration[i];
	}
	for(int i = 0; i < 3; i++){
		placeholder1[i] = u*filter->Bd[i];
	}

	matvecprod(3, 3, filter->Ad, filter->x_bar, placeholder2);
	vecadd(3, placeholder1, placeholder2, filter->x_hat);

	float placeholder_mat1[3][3] = { 0 };
	float placeholder_mat2[3][3] = { 0 };
	float placeholder_mat3[3][3] = { 0 };
	matmul(3, 3, 3, filter->Ad, filter->P_bar, placeholder_mat1);
	matmul(3, 3, 3, placeholder_mat1, filter->Ad_T, placeholder_mat2);


	matadd(3, 3, placeholder_mat2, filter->GdQGd_T, filter->P_hat);

	/* Prediction Step finished */
	/* Update Step */

	/* Calculate K = P_hat*H_T*(H*P_Hat*H_T+R)^-1 */
	memset(placeholder_mat1, 0, 9*sizeof(placeholder_mat1));
	memset(placeholder_mat2, 0, 9*sizeof(placeholder_mat2));

	matmul(3, 3, 3, filter->P_hat, filter->H_full_T, placeholder_mat1);

	matmul(3, 3, 3, filter->H_full, filter->P_hat, placeholder_mat2);
	matmul(3, 3, 3, placeholder_mat2, filter->H_full_T, placeholder_mat3);

	memset(placeholder_mat2, 0, 9*sizeof(placeholder_mat2));

	matadd(3, 3, placeholder_mat3, filter->R_full, placeholder_mat2);

	memset(placeholder_mat3, 0, 9*sizeof(placeholder_mat3));

	inverse(3, placeholder_mat2, placeholder_mat3, 0);

	matmul(3, 3, 3, placeholder_mat1, placeholder_mat3, filter->K_full);

	/* Finished Calculating K */
	memset(placeholder_mat1, 0, 9*sizeof(placeholder_mat1));
	memset(placeholder_mat2, 0, 9*sizeof(placeholder_mat2));
	memset(placeholder_mat3, 0, 9*sizeof(placeholder_mat3));
	memset(placeholder1, 0, 3*sizeof(placeholder1));
	memset(placeholder2, 0, 3*sizeof(placeholder2));


	/* Calculate x_bar = x_hat+K*(y-Hx_hat); */
	matvecprod(3, 3, filter->H_full, filter->x_hat, placeholder1);
	vecsub(3, data->calculated_AGL, placeholder1, placeholder2);

	memset(placeholder2, 0, 3*sizeof(placeholder1));

	matvecprod(3, 3, filter->K_full, placeholder2, placeholder1);

	vecadd(3, filter->x_hat, placeholder1, filter->x_bar);

	/* Finished Calculating x_bar */
	/* Calculate P_bar = (eye-K*H)*P_hat */

	eye(3, placeholder_mat1);
	matmul(3, 3, 3, filter->K_full, filter->H_full, placeholder_mat2);

	matsub(3, 3, placeholder_mat1, placeholder_mat2, placeholder_mat3);

	matmul(3, 3, 3, placeholder_mat3, filter->P_hat, filter->P_bar);
	/* Finished Calculating P_bar */

}
