#ifndef __EKF_H__
#define __EKF_H__


//#define ES_EKF_TEST


typedef struct {
	float Rw[3], Ra[3], Rad[3], hq[3], y[3];
	float q0[4], q1[4], X1[3], P0[9], P1[9];
	float Fq[16], F[9], H[9], Ht[9], S[9], invS[9], K[9], kd;
	float T0[9];
	
	
} EKF_Handle;


void EKF_Init(EKF_Handle *hd, float Rw[3], float Ra[3]);
int EKF_Predict(EKF_Handle *hd, float w[3], float dt);
int EKF_Update(EKF_Handle *hd, float a[3]);
int EKF_Plain_Update(EKF_Handle *hd);


#ifdef ES_EKF_TEST
int EKF_TEST();

#endif


#endif