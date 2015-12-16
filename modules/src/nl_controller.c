/*
 * non_linear_controller.c
 *
 *  Created on: Nov 4, 2015
 *      Author: v
 */

#include "nl_controller.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "log.h"

float data[9];

void init_nl_controller()
{
	J[0][0] = 0.023951; //
	J[1][1] = 0.023951; // in gm . m^2
	J[2][2] = 0.032347; //

	Jin[0][0] = 1.0/J[0][0];
	Jin[1][1] = 1.0/J[1][1];
	Jin[2][2] = 1.0/J[2][2];

	m = 27.0; // in grams

	Binv_r = -8.33;
	Binv_p = -8.33;
	Binv_y = -6.7568;

	//d = 6cm, tc = 0.037gm . m^2 - Binv_rp - 8.33 , Binv_y - 6.756756756
	Binv[0][0] = 0.25;
	Binv[0][1] = 0.0;
	Binv[0][2] = Binv_p;
	Binv[0][3] = Binv_y;

	Binv[1][0] = 0.25;
	Binv[1][1] = -Binv_r;
	Binv[1][2] = 0.0;
	Binv[1][3] = -Binv_y;

	Binv[2][0] = 0.25;
	Binv[2][1] = 0.0;
	Binv[2][2] = -Binv_p;
	Binv[2][3] = Binv_y;

	Binv[3][0] = 0.25;
	Binv[3][1] = Binv_r;
	Binv[3][2] = 0.0;
	Binv[3][3] = -Binv_y;

	Kr = 0.5;//2.81
	Ko = 0.15;//1.54
	c1 = 3.6;
	c2 = 0.0001;
	epx = 0.04;
	epr = 0.04;
	dor = 2.0;

	//backstepping init

	len = 0.06; // 6cm

	BSa1 = (J[1][1] - J[2][2])/J[0][0];
	BSa3 = (J[2][2] - J[0][0])/J[1][1];
	BSa4 = (J[0][0] - J[1][1])/J[2][2];

	float Jr = 0.66*0.001*0.0225*0.0225; // Rad - 2.25cm, Weight - 1gm , 2/3 * M * r^2
	BSa2 = Jr/J[0][0];
	BSa4 = Jr/J[1][1];
	BSa6 = Jr/J[2][2];

	BSb1 = 0.08/J[0][0];
	BSb2 = 0.08/J[1][1];
	BSb3 = 0.08/J[2][2];

	int x = 0;
	BSc[x++] = 5.52;
	BSc[x++] = 3.40;
	BSc[x++] = 5.52;
	BSc[x++] = 3.40;
	BSc[x++] = 0.1;
	BSc[x++] = 0.2;

	int i;
	for(i=0 ; i<3 ; i++)
	{
		SDd[i] = 0.0;
		SDdd[i] = 0.0;
		oS[i] = 0.0;
		oSd[i] = 0.0;
	}
	commRPM = 0;
	cnt = 0;
}


void attitude_control_bs(uint16_t f, float rD, float pD, float yD, float rC, float pC, float yC, float rR, float pR, float yR, float dt, uint16_t op[4])
{
	data[0] = rD * 180 / M_PI;
	data[1] = -pD * 180 / M_PI;
	data[2] = yD * 180 / M_PI;

	data[3] = rC * 180 / M_PI;
	data[4] = pC * 180 / M_PI;
	data[5] = yC * 180 / M_PI;

	data[6] = rR * 180 / M_PI;
	data[7] = pR * 180 / M_PI;
	data[8] = yR * 180 / M_PI;

	/*data[9] = Oc[0] * 180 / M_PI;
	data[10] = Oc[1] * 180 / M_PI;
	data[11] = Oc[2] * 180 / M_PI;

	data[12] = eR[0] * 180 / M_PI;
	data[13] = eR[1] * 180 / M_PI;
	data[14] = eR[2] * 180 / M_PI;

	data[15] = eO[0] * 180 / M_PI;
	data[16] = eO[1] * 180 / M_PI;
	data[17] = eO[2] * 180 / M_PI;
*/
	float ang_def = 3*M_PI/4;

	float Rar = (cos(ang_def)*rR - sin(ang_def)*pR);
	float Rap = (sin(ang_def)*rR + cos(ang_def)*pR);

	rR = Rar;
	pR = Rap;

	//FOR ROLL
	float z2R;
	float z1R;

	//Finding z1
	z1R = rD - rC;

	//Finding State Desired dot for roll
	SDd[0] = (rD - oS[0]) / dt;
	oS[0] = rD;
	SDdd[0] = (SDd[0] - oSd[0]) / dt;
	oSd[0] = SDd[0];

	//Finding z2
	z2R = rR - SDd[0] - (BSc[0]*z1R);

	float UR = (-(BSc[1] * z2R) + z1R  - (pR * yR * BSa1) + (pR * commRPM * BSa2) + SDdd[0] + (BSc[0] * SDd[0]) - (BSc[0] * rR))/BSb1; //Todo - Find commRPM

	//FOR PITCH
	float z3P;
	float z4P;

	z3P = pD - pC;

	SDd[1] = (pD - oS[1]) / dt;
	oS[1] = pD;
	SDdd[1] = (SDd[1] - oSd[1]) / dt;
	oSd[1] = SDd[1];

	z4P = pR - SDd[1] - (BSc[3]*z3P);

	float UP = (-(BSc[3] * z4P) + z3P - (rR * yR * BSa3) + (rR * commRPM * BSa4) + SDdd[1] + (BSc[2] * SDd[1]) - (BSc[2] * pR))/BSb2;

	//For YAW
	float z5Y;
	float z6Y;

	z5Y = yD - yC;

	SDd[2] = (yD - oS[2]) / dt;
	oS[2] = yD;
	SDdd[2] = (SDd[2] - oSd[2]) / dt;
	oSd[2] = SDd[2];

	z6Y = yR - SDd[2] - (BSc[5]*z5Y);

	float UY = (-(BSc[5] * z6Y) + z5Y - (rR * pR * BSa5) + SDdd[2] + (BSc[4] * SDd[2]) - (BSc[4] * yR))/BSb3;

	float thrust_req = ((float)f / 65536.0f) * 60;
	A[0] = thrust_req;
	A[1] = UR * 0.08;
	A[2] = UP * 0.08;
	A[3] = UY;

	matrix_mult_44x41(Binv, A, C);

	int i = 0;

	for (i=0 ; i<4 ; i++)
	{
		if (C[i] > 15)
			C[i] = 15;
		if (C[i] < 0)
			C[i] = 0;
	}

	for (i=0 ; i<4 ; i++)
		op[i] = (C[i]/15.0f)*65500;
	cnt += 1;


}


void attitude_controller(uint16_t f, float rD, float pD, float yD, float rC, float pC, float yC, float rR, float pR, float yR, float dt, uint16_t op[4])
{
	//DEbug Data
	data[0] = rD * 180 / M_PI;
	data[1] = pD * 180 / M_PI;
	data[2] = yD * 180 / M_PI;

	data[3] = rC * 180 / M_PI;
	data[4] = pC * 180 / M_PI;
	data[5] = yC * 180 / M_PI;

	data[6] = rR * 180 / M_PI;
	data[7] = pR * 180 / M_PI;
	data[8] = yR * 180 / M_PI;

	Binv[0][0] = 0.25;
	Binv[0][1] = 0.0;
	Binv[0][2] = Binv_p;
	Binv[0][3] = Binv_y;

	Binv[1][0] = 0.25;
	Binv[1][1] = -Binv_r;
	Binv[1][2] = 0.0;
	Binv[1][3] = -Binv_y;

	Binv[2][0] = 0.25;
	Binv[2][1] = 0.0;
	Binv[2][2] = -Binv_p;
	Binv[2][3] = Binv_y;

	Binv[3][0] = 0.25;
	Binv[3][1] = Binv_r;
	Binv[3][2] = 0.0;
	Binv[3][3] = -Binv_y;

	float ang_def = 3*M_PI/4;

	//float rCa = (cos(ang_def)*rC - sin(ang_def)*pC);
	//float pCa = (sin(ang_def)*rC + cos(ang_def)*pC);

	//float rda = (cos(M_PI/4)*rD - sin(M_PI/4)*pD);
	//float pda = (sin(M_PI/4)*rD + cos(M_PI/4)*pD);

	//Calculate rot matrix for desired attitude
	rot_matrixXYZ(rD, pD, yD, Rc);
	//Calculate rot matrix for current attitude
	rot_matrixXYZ(rC, pC, yC, R);
	//Setting O matrix
	O[0] = (cos(ang_def)*rR - sin(ang_def)*pR);
	O[1] = (sin(ang_def)*rR + cos(ang_def)*pR);
	O[2] = yR;
	//Setting Omega Hat
	Oh[0][0] = 0.0;
	Oh[0][1] = -O[2];
	Oh[0][2] = O[1];

	Oh[1][0] = O[2];
	Oh[1][1] = 0.0;
	Oh[1][2] = -O[0];

	Oh[2][0] = -O[1];
	Oh[2][1] = O[0];
	Oh[2][2] = 0.0;

	//Finding Rc dot
	matrix_mult_33x33(Rc, Oh, RcD);

	/*for (i=0 ; i<3 ; i++)
		for (j=0 ; j<3 ; j++)
			RcD[i][j] = (Rc[i][j] - RcO[i][j])/dt;

	for (i=0 ; i<3 ; i++)
			for (j=0 ; j<3 ; j++)
				RcO[i][j] = Rc[i][j];
	*/
	//Get Rc Transpose
	matrix_trans_33(Rc, RcT);

	//Finding OmegaC Hat
	matrix_mult_33x33(RcT, RcD, OcH);

	//Finding OmegaC
	Oc[0] = OcH[2][1];
	Oc[1] = OcH[0][2];
	Oc[2] = OcH[1][0];

	//Finding eR
	float eRa[3][3];
	float eRb[3][3];
	float eRc[3][3];

	matrix_trans_33(R, Rt);
	matrix_mult_33x33(RcT, R, eRa);
	matrix_mult_33x33(Rt, Rc, eRb);

	int i;
	int j;
	for (i=0 ; i<3 ; i++)
		for (j=0 ; j<3 ; j++)
			eRc[i][j] = eRa[i][j] - eRb[i][j];

	eR[0] = eRc[2][1]/2.0;
	eR[1] = eRc[0][2]/2.0;
	eR[2] = eRc[1][0]/2.0;

	//Finding eO
	float eOa[3][3];
	float eOb[3];

	matrix_mult_33x33(Rt, Rc, eOa);
	matrix_mult_33x31(eOa, Oc, eOb);
	for (i=0 ; i<3 ; i++)
		eO[i] = O[i] - eOb[i];

	//Finding M - Part 3
	float Mp3[3];
	float Mp3a[3];
	matrix_mult_33x31(J, O, Mp3a);
	Mp3[0] = (O[1]*Mp3a[2]) - (O[2]*Mp3a[1]);//
	Mp3[1] = (O[2]*Mp3a[0]) - (O[0]*Mp3a[2]);// Omega x J.Omega
	Mp3[2] = (O[0]*Mp3a[1]) - (O[1]*Mp3a[0]);//

	//Finding OcD - Desired Angular Acceleration (Omega C Dot)
	for (i=0 ; i<3 ; i++)
	{
		Ocd[i] = (Oc[i] - Oco[i])/dt;
		Oco[i] = Oc[i];
	}


	//Finding M - Part 4
	float Mp4[3];
	float Mp4a[3][3];
	float Mp4b[3][3];
	float Mp4c[3];
	float Mp4d[3];

	matrix_mult_33x33(Oh, Rt, Mp4a);
	matrix_mult_33x33(Mp4a, Rc, Mp4b);
	matrix_mult_33x31(Mp4b, Oc, Mp4c);

	matrix_mult_33x33(Rt, Rc, Mp4a);
	matrix_mult_33x31(Mp4a, Ocd, Mp4d);

	for (i=0 ; i<3 ; i++)
		Mp4c[i] = Mp4c[i] - Mp4d[i];
	matrix_mult_33x31(J, Mp4c, Mp4);

	//Part 5 - Finding uR
	float Mp5[3];
	matrix_mult_33x31(Jin, eR, Mp5);
	for (i=0 ; i<3 ; i++)
		eA[i] = eO[i] + (c2 * Mp5[i]);
	float len_eA;
	len_eA = sqrt((eA[0]*eA[0]) + (eA[1]*eA[1]) + (eA[2]*eA[2]));

	for (i=0 ; i<3 ; i++)
		ur[i] = -((dor*dor) * eA[i])/((dor*len_eA) + epr);

	//Final M Calculation
	for (i=0 ; i<3 ; i++)
		M[i] = -(Kr * eR[i]) - (Ko * eO[i]) + Mp3[i] + Mp4[i] + ur[i];

	float thrust_req = ((float)f / 65536.0f) * 60;
	A[0] = thrust_req;
	A[1] = M[0];
	A[2] = M[1];
	A[3] = M[2];//M[2];

	matrix_mult_44x41(Binv, A, C);

	for (i=0 ; i<4 ; i++)
	{
		if (C[i] > 15)
			C[i] = 15;
		if (C[i] < 0)
			C[i] = 0;
	}

	for (i=0 ; i<4 ; i++)
		op[i] = (C[i]/15.0f)*65500;
}

void rot_matrixZYX(float r, float p, float y, float rm[][3])
{
    rm[0][0] = cos(p)*cos(y);
    rm[0][1] = -cos(p)*sin(y);
    rm[0][2] = sin(p);

    rm[1][0] = sin(r)*cos(p)*cos(y) + cos(r)*sin(y);
    rm[1][1] = -sin(r)*sin(p)*sin(y) + cos(r)*cos(y);
    rm[1][2] = -sin(r)*cos(p);

    rm[2][0] = -cos(r)*sin(p)*cos(y) + sin(r)*sin(y);
    rm[2][1] = cos(r)*sin(p)*sin(y) + sin(r)*cos(y);
    rm[2][2] = cos(r)*sin(p);
}

void rot_matrixXYZ(float r, float p, float y, float rm[][3])
{
    rm[0][0] = cos(p)*cos(y);
    rm[0][1] = sin(r)*sin(p)*cos(y);
    rm[0][2] = (cos(r)*sin(p)*cos(y)) + (sin(r)*sin(y));

    rm[1][0] = cos(p)*sin(y);
    rm[1][1] = (sin(r)*sin(p)*sin(y)) + cos(p)*cos(y);
    rm[1][2] = (cos(r)*sin(p)*sin(y)) - (sin(p)*cos(y));

    rm[2][0] = -sin(p);
    rm[2][1] = sin(r)*cos(p);
    rm[2][2] = cos(r)*cos(p);
}

void matrix_mult_33x33(float a[][3], float b[][3], float c[][3])
{
    float sum = 0.0;
    int aa = 3;
    int ab = 3;
    int bb = 3;
    int i = 0;
    int j = 0;
    int k = 0;
    for(i=0 ; i<aa ; i++)
    {
        for (j=0 ; j<bb ; j++)
        {
            for (k=0 ; k<ab ; k++)
            {
                sum += a[i][k] * b[k][j];
            }
            c[i][j] = sum;
            sum = 0;
        }
    }
}

void matrix_mult_33x31(float a[][3], float b[3], float c[3])
{
    float sum = 0.0;
    int aa = 3;
    int ab = 3;
    int bb = 1;
    int i = 0;
    int j = 0;
    int k = 0;
    for(i=0 ; i<aa ; i++)
    {
        for (j=0 ; j<bb ; j++)
        {
            for (k=0 ; k<ab ; k++)
            {
                sum += a[i][k] * b[k];
            }
            c[i] = sum;
            sum = 0;
        }
    }
}

void matrix_mult_44x41(float a[][4], float b[4], float c[4])
{
    float sum = 0.0;
    int aa = 4;
    int ab = 4;
    int bb = 1;
    int i = 0;
    int j;
    int k;
    for(i=0 ; i<aa ; i++)
    {
        for (j=0 ; j<bb ; j++)
        {
            for (k=0 ; k<ab ; k++)
            {
                sum += a[i][k] * b[k];
            }
            c[i] = sum;
            sum = 0;
        }
    }
}


void matrix_trans_33(float a[][3], float b[][3])
{
    b[0][0] = a[0][0];
    b[0][1] = a[1][0];
    b[0][2] = a[2][0];

    b[1][0] = a[0][1];
    b[1][1] = a[1][1];
    b[1][2] = b[2][1];

    b[2][0] = a[0][2];
    b[2][1] = a[1][2];
    b[2][2] = a[2][2];
}


LOG_GROUP_START(nlc)
LOG_ADD(LOG_FLOAT, rol_des, &data[0])
LOG_ADD(LOG_FLOAT, pit_des, &data[1])
LOG_ADD(LOG_FLOAT, yaw_des, &data[2])
LOG_ADD(LOG_FLOAT, rol_cur, &data[3])
LOG_ADD(LOG_FLOAT, pit_cur, &data[4])
LOG_ADD(LOG_FLOAT, yaw_cur, &data[5])
LOG_ADD(LOG_FLOAT, rol_rat, &data[6])
LOG_ADD(LOG_FLOAT, pit_rat, &data[7])
LOG_ADD(LOG_FLOAT, yaw_rat, &data[8])
LOG_ADD(LOG_FLOAT, pit_omc, &data[10])
LOG_ADD(LOG_FLOAT, pit_erc, &data[13])
LOG_ADD(LOG_FLOAT, pit_eoc, &data[16])
LOG_GROUP_STOP(nl_controller)

PARAM_GROUP_START(cont)
PARAM_ADD(PARAM_FLOAT, RotConstKr, &Kr)
PARAM_ADD(PARAM_FLOAT, OmgConstKo, &Ko)
PARAM_ADD(PARAM_FLOAT, ConstC1, &c1)
PARAM_ADD(PARAM_FLOAT, ConstC2, &c2)
PARAM_ADD(PARAM_FLOAT, EpsilonX, &epx)
PARAM_ADD(PARAM_FLOAT, EpsilonR, &epr)
PARAM_ADD(PARAM_FLOAT, BmatInvR, &Binv_r)
PARAM_ADD(PARAM_FLOAT, BmatInvP, &Binv_p)
PARAM_ADD(PARAM_FLOAT, BmatInvY, &Binv_y)
PARAM_ADD(PARAM_FLOAT, DoRot, &dor)
PARAM_ADD(PARAM_FLOAT, BSC1, &BSc[0])
PARAM_ADD(PARAM_FLOAT, BSC2, &BSc[1])
PARAM_ADD(PARAM_FLOAT, BSC3, &BSc[2])
PARAM_ADD(PARAM_FLOAT, BSC4, &BSc[3])
PARAM_ADD(PARAM_FLOAT, BSC5, &BSc[4])
PARAM_ADD(PARAM_FLOAT, BSC6, &BSc[5])
PARAM_GROUP_STOP(cont)
