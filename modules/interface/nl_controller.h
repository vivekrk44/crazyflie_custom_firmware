/*
 * nl_controller.h
 *
 *  Created on: Nov 4, 2015
 *      Author: v
 */

#ifndef MODULES_INTERFACE_NL_CONTROLLER_H_
#define MODULES_INTERFACE_NL_CONTROLLER_H_

#include "FreeRTOS.h"


float x[3]; //Current Position
float v[3]; //Current Velocity

float xd[3]; //Desired Position

float R[3][3]; //Rotation Matrix with current Attitude
float O[3]; //Omega, Angular Velocity
float Rt[3][3]; //Transpose of R
float Oh[3][3]; //Omega Hat

float Rc[3][3]; //Rotation Matrix with desired Attitude
float RcO[3][3]; //Previous Rotaion Matrix for finding Rc dot
float RcD[3][3]; // Rc dot
float RcT[3][3]; // Rc Transpose
float Oc[3]; //Desired Angular Velocity
float Ocd[3]; //Desired Angular Acceleration
float Oco[3];
float OcH[3][3]; //Omega Desired Hat

float Kx; //Proportional Constant for Ex
float Kv; //Proportional Constant for Ev
float Kr; //Proportional Constant for Er
float Ko; //Proportional Constant for EOmega

float J[3][3]; //Intertia Matrix of the Quadcotper
float Jin[3][3]; //Inverse of Intertia Matrix of the Quadcotper
float m; //Mass of the drone
float g; //Acceleration due to Gravity

float ux[3]; //Intermediate Value for x
float ur[3]; //Intermediate Value for R

float eB[3];
float eA[3];

float eR[3]; // Error of Rotation for attitude controller
float eO[3]; // Error of Angular Velocity for attitude controller

float eX[3];
float eV[3];
float xdd[3]; //float Differentiation of xd

float dox;
float dor;
float tau;
float epx;
float epr;

float c1;
float c2;

float Binv_r;
float Binv_p;
float Binv_y;

float e3[3];

float M[3];
float F;

float Binv[4][4];
float A[4];
float C[4];

void init_nl_controller();

void attitude_controller(uint16_t f, float rD, float pD, float yD, float rC, float pC, float yC, float rR, float pR, float yR, float dt, uint16_t op[4]);
void attitude_control_bs(uint16_t f, float rD, float pD, float yD, float rC, float pC, float yC, float rR, float pR, float yR, float dt, uint16_t op[4]);
int position_controller();

void rot_matrixXYZ(float r, float p, float y, float rm[][3]);
void rot_matrixZYX(float r, float p, float y, float rm[][3]);
void matrix_mult_33x33(float a[][3], float b[][3], float c[][3]);
void matrix_mult_33x31(float a[][3], float b[3], float c[3]);
void matrix_trans_33(float a[][3], float b[][3]);
void matrix_mult_44x41(float a[][4], float b[4], float c[4]);
//Attitude Controller

//For backsteping

float BSa1;
float BSa2;
float BSa3;
float BSa4;
float BSa5;
float BSa6;
float BSb1;
float BSb2;
float BSb3;

float oS[9];
float SDd[9];
float oSd[9];
float SDdd[9];

int cnt;
float len;

float BSc[6];

float commRPM;


#endif /* MODULES_INTERFACE_NL_CONTROLLER_H_ */
