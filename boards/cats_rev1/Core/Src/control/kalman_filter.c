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
	float Q_dash = 0.01f;
	float P_dash[3][3] = {{0.00001f, 0, 0},{0, 0.00001f, 0},{0, 0, 0.00001f}};
	float H_full_dash[3][3] = {{1, 0, 0},{1, 0, 0},{1, 0, 0}};
	float H_full_dash_T[3][3] = { 0 };
	transpose(3, 3, H_full_dash, H_full_dash_T);
	float H_eliminated_dash[2][3] = {{1, 0, 0}, {1, 0, 0}};
	float H_eliminated_dash_T[3][2] = { 0 };
	transpose(2, 3, H_eliminated_dash, H_eliminated_dash_T);
	float R_full_dash[3][3] = {{0.00001f, 0, 0}, {0, 0.00001f, 0}, {0, 0, 0.00001f}};
	float R_eliminated_dash[2][2] = {{0.00001f, 0}, {0, 0.00001f}};
	float x_dash[3] = {0, 0, 0};

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

	const size_t flt_3x3_size = 9*sizeof(float);
	const size_t flt_3_size = 3*sizeof(float);
	const size_t flt_2x3_size = 6*sizeof(float);
	const size_t flt_2x2_size = 4*sizeof(float);
	memcpy(filter->Ad, A_dash, flt_3x3_size);
	memcpy(filter->Ad_T, A_dash_T, flt_3x3_size);
	memcpy(filter->Gd, G_dash, flt_3_size);
	memcpy(filter->Bd, B_dash, flt_3_size);
	memcpy(filter->P_hat, P_dash, flt_3x3_size);
	memcpy(filter->P_bar, P_dash, flt_3x3_size);
	memcpy(filter->x_hat, x_dash, flt_3_size);
	memcpy(filter->x_bar, x_dash, flt_3_size);
	filter->Q = Q_dash;
	memcpy(filter->H_full, H_full_dash, flt_3x3_size);
	memcpy(filter->H_full_T, H_full_dash_T, flt_3x3_size);
	memcpy(filter->H_eliminated, H_eliminated_dash, flt_2x3_size);
	memcpy(filter->H_eliminated_T, H_eliminated_dash_T, flt_2x3_size);
	memcpy(filter->R_full, R_full_dash, flt_3x3_size);
	memcpy(filter->R_eliminated, R_eliminated_dash, flt_2x2_size);
	memcpy(filter->GdQGd_T, GdQGd_T_dash, flt_3x3_size);
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
		placeholder1[i] = ((float)(1)/3)*u*filter->Bd[i];
	}

	matvecprod(3, 3, filter->Ad, filter->x_bar, placeholder2);
	memset(filter->x_hat, 0, 3*sizeof(filter->x_hat[0]));
	vecadd(3, placeholder1, placeholder2, filter->x_hat);

	float placeholder_mat1[3][3] = { 0 };
	float placeholder_mat2[3][3] = { 0 };
	float placeholder_mat3[3][3] = { 0 };
	matmul(3, 3, 3, filter->Ad, filter->P_bar, placeholder_mat1);
	matmul(3, 3, 3, placeholder_mat1, filter->Ad_T, placeholder_mat2);

	memset(filter->P_hat, 0, sizeof(filter->P_hat));
	matadd(3, 3, placeholder_mat2, filter->GdQGd_T, filter->P_hat);

	/* Prediction Step finished */
	/* Update Step */

	/* Calculate K = P_hat*H_T*(H*P_Hat*H_T+R)^-1 */
	memset(placeholder_mat1, 0, sizeof(placeholder_mat1));
	memset(placeholder_mat2, 0, sizeof(placeholder_mat2));

	matmul(3, 3, 3, filter->P_hat, filter->H_full_T, placeholder_mat1);

	matmul(3, 3, 3, filter->H_full, filter->P_hat, placeholder_mat2);
	matmul(3, 3, 3, placeholder_mat2, filter->H_full_T, placeholder_mat3);

	memset(placeholder_mat2, 0, sizeof(placeholder_mat2));

	matadd(3, 3, placeholder_mat3, filter->R_full, placeholder_mat2);

	memset(placeholder_mat3, 0, sizeof(placeholder_mat3));

	inverse(3, placeholder_mat2, placeholder_mat3, 0);

	memset(filter->K_full, 0, sizeof(filter->K_full));
	matmul(3, 3, 3, placeholder_mat1, placeholder_mat3, filter->K_full);

	/* Finished Calculating K */
	memset(placeholder_mat1, 0, sizeof(placeholder_mat1));
	memset(placeholder_mat2, 0, sizeof(placeholder_mat2));
	memset(placeholder_mat3, 0, sizeof(placeholder_mat3));
	memset(placeholder1, 0, sizeof(placeholder1));
	memset(placeholder2, 0, sizeof(placeholder2));


	/* Calculate x_bar = x_hat+K*(y-Hx_hat); */
	matvecprod(3, 3, filter->H_full, filter->x_hat, placeholder1);
	vecsub(3, data->calculated_AGL, placeholder1, placeholder2);

	memset(placeholder1, 0, sizeof(placeholder1));

	matvecprod(3, 3, filter->K_full, placeholder2, placeholder1);

	memset(filter->x_bar, 0, sizeof(filter->x_bar));
	vecadd(3, filter->x_hat, placeholder1, filter->x_bar);

	/* Finished Calculating x_bar */
	/* Calculate P_bar = (eye-K*H)*P_hat */

	eye(3, placeholder_mat1);
	matmul(3, 3, 3, filter->K_full, filter->H_full, placeholder_mat2);

	matsub(3, 3, placeholder_mat1, placeholder_mat2, placeholder_mat3);

	memset(filter->P_bar, 0, sizeof(filter->P_bar));
	matmul(3, 3, 3, placeholder_mat3, filter->P_hat, filter->P_bar);
	/* Finished Calculating P_bar */

}
