#include "EKF.h"
#include "math.h"

#define pow2(x) ((x) * (x))

#define Acc_Norm_Max 10.3f
#define Acc_Norm_Min 9.2f
#define Alpha 0.5
#define Beta  100


static void mat_copy_3x3(const float src[9], float dst[9]) {
	dst[0]=src[0];	dst[1]=src[1];	dst[2]=src[2];
	dst[3]=src[3];	dst[4]=src[4];	dst[5]=src[5];
	dst[6]=src[6];	dst[7]=src[7];	dst[8]=src[8];
}


static void quat_mult(const float qA[4], const float qB[4], float q_dst[4]) {
	q_dst[0] = qA[0]*qB[0] - qA[1]*qB[1] - qA[2]*qB[2] - qA[3]*qB[3];
	q_dst[1] = qA[0]*qB[1] + qA[1]*qB[0] + qA[2]*qB[3] - qA[3]*qB[2];
	q_dst[2] = qA[0]*qB[2] - qA[1]*qB[3] + qA[2]*qB[0] + qA[3]*qB[1];
	q_dst[3] = qA[0]*qB[3] + qA[1]*qB[2] - qA[2]*qB[1] + qA[3]*qB[0];
}


static int mat_inverse_3x3(const float A[9], float inv[9]) {
    float a = A[0], b = A[1], c = A[2];
    float d = A[3], e = A[4], f = A[5];
    float g = A[6], h = A[7], i = A[8];

    float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
    if (fabsf(det) < 1e-8f) {
        return -1;
    }

    float inv_det = 1.0f / det;
    inv[0] =  (e*i - f*h) * inv_det;
    inv[1] = -(b*i - c*h) * inv_det;
    inv[2] =  (b*f - c*e) * inv_det;
    inv[3] = -(d*i - f*g) * inv_det;
    inv[4] =  (a*i - c*g) * inv_det;
    inv[5] = -(a*f - c*d) * inv_det;
    inv[6] =  (d*h - e*g) * inv_det;
    inv[7] = -(a*h - b*g) * inv_det;
    inv[8] =  (a*e - b*d) * inv_det;
    return 0;
}


static inline void mat_trans_3x3(const float A[9], float AT[9]) {
    AT[0] = A[0]; AT[1] = A[3]; AT[2] = A[6];
    AT[3] = A[1]; AT[4] = A[4]; AT[5] = A[7];
    AT[6] = A[2]; AT[7] = A[5]; AT[8] = A[8];
}


// 4x4 mat mult 4x1 vec
static inline void mat4x4_mult_vec4(const float M[16], const float V[4], float R[4]) {
    R[0] = M[0]*V[0] + M[1]*V[1] + M[2]*V[2] + M[3]*V[3];
    R[1] = M[4]*V[0] + M[5]*V[1] + M[6]*V[2] + M[7]*V[3];
    R[2] = M[8]*V[0] + M[9]*V[1] + M[10]*V[2] + M[11]*V[3];
    R[3] = M[12]*V[0] + M[13]*V[1] + M[14]*V[2] + M[15]*V[3];
}


// 3x3 mat mult 3x3 mat
static void mat_mult_3x3(const float A[9], const float B[9], float C[9]) {
    C[0] = A[0]*B[0] + A[1]*B[3] + A[2]*B[6];
    C[1] = A[0]*B[1] + A[1]*B[4] + A[2]*B[7];
    C[2] = A[0]*B[2] + A[1]*B[5] + A[2]*B[8];

    C[3] = A[3]*B[0] + A[4]*B[3] + A[5]*B[6];
    C[4] = A[3]*B[1] + A[4]*B[4] + A[5]*B[7];
    C[5] = A[3]*B[2] + A[4]*B[5] + A[5]*B[8];

    C[6] = A[6]*B[0] + A[7]*B[3] + A[8]*B[6];
    C[7] = A[6]*B[1] + A[7]*B[4] + A[8]*B[7];
    C[8] = A[6]*B[2] + A[7]*B[5] + A[8]*B[8];
}


// A, B is 3x3 mat, Dst = A*B*trans(A)
static void biLiner_mult_3x3(float A[9], float B[9], float Dst[9]) {
	float T0[9], tsA[9];
	mat_trans_3x3(A,tsA);
	mat_mult_3x3(B, tsA, T0);
	mat_mult_3x3(A, T0, Dst);
}


// generate Fq and F
static inline void getFq_F(float Fq_D[16], float F_D[9], float gyro[3], float dt) {
	
	float w[3]={gyro[0]*dt/2, gyro[1]*dt/2, gyro[2]*dt/2};
	Fq_D[0] = 1;	Fq_D[1] =-w[0];	Fq_D[2] =-w[1];	Fq_D[3] =-w[2];
	Fq_D[4] = w[0];	Fq_D[5] = 1;	Fq_D[6] = w[2];	Fq_D[7] =-w[1];
	Fq_D[8] = w[1];	Fq_D[9] =-w[2];	Fq_D[10]= 1;	Fq_D[11]= w[0];
	Fq_D[12]= w[2];	Fq_D[13]= w[1];	Fq_D[14]=-w[0];	Fq_D[15]= 1;
	
	w[0]=w[0]*2; w[1]=w[1]*2; w[2]=w[2]*2;
	F_D[0]= 1;		F_D[1]=-w[2];	F_D[2]= w[1];
	F_D[3]= w[2];	F_D[4]= 1;		F_D[5]=-w[0];
	F_D[6]=-w[1];	F_D[7]= w[0];	F_D[8]= 1;
}


// generate hq and H
static inline void get_hq_H(float hq[3], float H_D[9], float Ht_D[9], float q0[4]) {
	hq[0] = 2*(q0[1]*q0[3] - q0[0]*q0[2]);
	hq[1] = 2*(q0[2]*q0[3] + q0[0]*q0[1]);
	hq[2] = pow2(q0[0])-pow2(q0[1])-pow2(q0[2])+pow2(q0[3]);
	H_D[0] = 0;			H_D[1] =-hq[2];		H_D[2] = hq[1];
	H_D[3] = hq[2];		H_D[4] = 0;			H_D[5] =-hq[0];
	H_D[6] =-hq[1];		H_D[7] = hq[0];		H_D[8] = 0;
	Ht_D[0]=0;			Ht_D[1]=-H_D[1];	Ht_D[2]=-H_D[2];
	Ht_D[3]=-H_D[3];	Ht_D[4]=0;			Ht_D[5]=-H_D[5];
	Ht_D[6]=-H_D[6];	Ht_D[7]=-H_D[7];	Ht_D[8]=0;
}




void EKF_Init(EKF_Handle *hd, float Rw[3], float Ra[3]) {
	hd->Ra[0]=Ra[0]; hd->Ra[1]=Ra[1]; hd->Ra[2]=Ra[2];
	hd->Rw[0]=Rw[0]; hd->Rw[1]=Rw[1]; hd->Rw[2]=Rw[2];
	// P0 P1
	for(unsigned char i=0; i<9; i++) {
		hd->P0[i]=0;
		hd->P1[i]=0;
	}
	hd->P0[0]=1; hd->P0[4]=1; hd->P0[8]=1;
	hd->P1[0]=1; hd->P1[4]=1; hd->P1[8]=1;
	// X1 q0 q1
	hd->X1[0]=0; hd->X1[1]=0; hd->X1[2]=0;
	hd->q0[0]=1; hd->q0[1]=0; hd->q0[2]=0; hd->q0[3]=0;
	hd->q1[0]=1; hd->q1[1]=0; hd->q1[2]=0; hd->q1[3]=0;
}


int EKF_Predict(EKF_Handle *hd, float w[3], float dt) {
	
	float dt2 = dt*dt;
	int sta=0;
	getFq_F(hd->Fq, hd->F, w, dt);
	mat4x4_mult_vec4(hd->Fq, hd->q1, hd->q0);		// q0=Fq*q1
	biLiner_mult_3x3(hd->F, hd->P1, hd->P0);		// P0=F*P1*F'
	hd->P0[0] += hd->Rw[0]*dt2;						// P0=F*P1*F' + Rw*dt^2
	hd->P0[4] += hd->Rw[1]*dt2;
	hd->P0[8] += hd->Rw[2]*dt2;
	
	// norm
	float q0_norm = sqrtf(pow2(hd->q0[0])+pow2(hd->q0[1])+pow2(hd->q0[2])+pow2(hd->q0[3]));
	hd->q0[0]=hd->q0[0]/q0_norm;  hd->q0[1]=hd->q0[1]/q0_norm;
	hd->q0[2]=hd->q0[2]/q0_norm;  hd->q0[3]=hd->q0[3]/q0_norm;
	
	return 0;
}


int EKF_Update(EKF_Handle *hd, float a[3]) {
	float a_norm = sqrtf(pow2(a[0])+pow2(a[1])+pow2(a[2]));
	float a_unit[3] = {a[0]/a_norm, a[1]/a_norm, a[2]/a_norm};
	
	// 加速度门限初步筛选	130us
	if(a_norm > Acc_Norm_Max || a_norm < Acc_Norm_Min) {
		mat_copy_3x3(hd->P0, hd->P1);
		hd->q1[0]=hd->q0[0]; hd->q1[1]=hd->q0[1];
		hd->q1[2]=hd->q0[2]; hd->q1[3]=hd->q0[3];
		return 1;
	}
	
	get_hq_H(hd->hq, hd->H, hd->Ht, hd->q0);				// 获取Hq, H
	hd->y[0]=a_unit[0]-hd->hq[0];							// y=Z-hq (y = a_unit-hq_unit)
	hd->y[1]=a_unit[1]-hd->hq[1];
	hd->y[2]=a_unit[2]-hd->hq[2];
	biLiner_mult_3x3(hd->H, hd->P0, hd->S);					// S=H*P0*H'
	hd->S[0] += hd->Ra[0];									// S=H*P0*H' + Ra (初步观测误差)
	hd->S[4] += hd->Ra[1];
	hd->S[8] += hd->Ra[2];
	
	int sta=0;
	// 马氏距离筛选 136us
	sta=mat_inverse_3x3(hd->S, hd->invS); if(sta!=0) return sta;	// S^-1
	float D = hd->invS[0]*pow2(hd->y[0]) + hd->invS[4]*pow2(hd->y[1]) + hd->invS[8]*pow2(hd->y[2])
		+2*(hd->invS[1]*hd->y[0]*hd->y[1] + hd->invS[2]*hd->y[0]*hd->y[2] + hd->invS[5]*hd->y[1]*hd->y[2]);
	if(D > 7.81f) {
		mat_copy_3x3(hd->P0, hd->P1);
		hd->q1[0]=hd->q0[0]; hd->q1[1]=hd->q0[1];
		hd->q1[2]=hd->q0[2]; hd->q1[3]=hd->q0[3];
		return 1;
	}
	
	// 动态放大加速度协方差 22us
	float k1,k2;
	k1 = (1 + Alpha*D);
	k2 = (1 + Beta*pow2(9.8f-a_norm));
	hd->kd = k1*k2-1;												// 因为之前加过一次，所以需要减1
	hd->S[0] += hd->kd*hd->Ra[0];									// S=H*P0*H' + Rad (动态观测误差)
	hd->S[4] += hd->kd*hd->Ra[1];
	hd->S[8] += hd->kd*hd->Ra[2];

	// kalman gain 173us
	sta=mat_inverse_3x3(hd->S, hd->invS); if(sta!=0) return sta;	// S^-1
	mat_mult_3x3(hd->Ht, hd->invS, hd->T0); 						// Ht*S^-1
	mat_mult_3x3(hd->P0, hd->T0, hd->K);							// K=P*Ht*S^-1
	
	// update 130us
	hd->X1[0]=hd->K[0]*hd->y[0] + hd->K[1]*hd->y[1] + hd->K[2]*hd->y[2];		// X1=K*y
	hd->X1[1]=hd->K[3]*hd->y[0] + hd->K[4]*hd->y[1] + hd->K[5]*hd->y[2];
	hd->X1[2]=hd->K[6]*hd->y[0] + hd->K[7]*hd->y[1] + hd->K[8]*hd->y[2];
	mat_mult_3x3(hd->K, hd->H, hd->T0);											// K*H
	hd->T0[0]=1-hd->T0[0]; hd->T0[1]= -hd->T0[1]; hd->T0[2]= -hd->T0[2];		// I-K*H
	hd->T0[3]= -hd->T0[3]; hd->T0[4]=1-hd->T0[4]; hd->T0[5]= -hd->T0[5];
	hd->T0[6]= -hd->T0[6]; hd->T0[7]= -hd->T0[7]; hd->T0[8]=1-hd->T0[8];
	mat_mult_3x3(hd->T0, hd->P0, hd->P1);										// P1=(I-K*H)*P0
	
	// correction 133us
	float q_dq[4], dqe[4], q1[4];
	float dq[4] = {1, hd->X1[0]/2, hd->X1[1]/2, hd->X1[2]/2};					// dq = [1, dX/2]
	quat_mult(hd->q0, dq, q_dq);												// q0*dq
	float conj_q0[4] = {hd->q0[0], -hd->q0[1], -hd->q0[2], -hd->q0[3]};			// q0^-1
	quat_mult(q_dq, conj_q0, dqe);												// dqe = q0*dq*q0^-1
	dqe[3]=0;																	// z方向旋转置0，防止错误修正yaw
	quat_mult(dqe, hd->q0, q1);													// q1 = dqe*q0
	float q1_norm = sqrtf(pow2(q1[0])+pow2(q1[1])+pow2(q1[2])+pow2(q1[3]));
	hd->q1[0]=q1[0]/q1_norm;  hd->q1[1]=q1[1]/q1_norm;							// q1 = q1/norm(q1)
	hd->q1[2]=q1[2]/q1_norm;  hd->q1[3]=q1[3]/q1_norm;
	
	return 0;
}


// 直接更新，无观测修正
int EKF_Plain_Update(EKF_Handle *hd) {
	mat_copy_3x3(hd->P0, hd->P1);
	hd->q1[0]=hd->q0[0]; hd->q1[1]=hd->q0[1];
	hd->q1[2]=hd->q0[2]; hd->q1[3]=hd->q0[3];
	return 0;
}



#ifdef ES_EKF_TEST

#include "DWT.h"
DWT_TikTok tiktok;
EKF_Handle test_ekf;

int EKF_TEST() {
	float Ra[3]={0.1, 0.1, 0.1}, Rw[3]={0.001, 0.001, 0.001};
	EKF_Init(&test_ekf, Rw, Ra);
	
	float a[3]={0,0.1,9.83}, w[3]={1,0.5,0.3}, dt=0.01;
	int sta=0;
	for(int i=0;i<5;i++) {
		DWT_Tik(&tiktok);
		sta=EKF_Predict(&test_ekf, w, dt); if(sta<0) break;		// 190us@103-72Mhz
		DWT_Tok(&tiktok);
		DWT_Tik(&tiktok);
		sta=EKF_Update(&test_ekf, a);  if(sta<0) break;		// 593us@103-72Mhz
		DWT_Tok(&tiktok);
	}

	return 0;
}


#endif
