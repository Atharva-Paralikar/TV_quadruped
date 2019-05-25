#ifndef TV_quadruped
#define TV_quadruped
#include "math.h"

//-------MATH----------
#define pi 3.141592
#define p180 0.01745329252	//pi/180
#define _180p 57.295779513	//180/pi
#define d2r(a) (a*p180)		//degrees to radians
#define r2d(a) (a*_180p)	//radians to degrees
//---------------------
class TVquad
{
private:
	int offset[8];		//offsets per motor for each leg to form perfect |_ position
	int signB[8];		//sign bits decide the direction the motors move
	int l1,l2;	 		//lengths of limb1 and limb2
	double trajectory_adjust;
	int elbow_position[4][2];
	float slope;
	int rotation;
	double height_mul[4];
	double length_mul[4];
	double points_of_trajectory[72][2];	//points in the trajectory 0->x 1->y
	int joint_angle[72][2];
	void (*uMotor)(int leg,int angle);
	void (*dMotor)(int leg,int angle);	
	void (*Delay)(int delayMicroseconds);	//the 0th angles for upper joint and 1st are for lower limbs
public:
	void setpoints(double[72][2]);	//takes 72 points of your choice
	void setUpperMotorf(void (*motor)(int leg,int angle));
	void setLowerMotorf(void (*motor)(int leg,int angle));	/*	give your function to control upper motors the first parameter takes	the leg which you want to move and second is used to provide your function with an angle which is percalculated for motion	*/
	void setDelayf(void (*system_Delay)(int microseconds));
	void setDirection(int[8]);
	void setOffsets(int[8]);
	void setPhaseDiff(int,int,int,int);
	void setHeightMul(int leg,double mul);
	void setLengthMul(int leg,double mul);
	void setLinkLength(int upper,int lower);
	void setDelay(int);
	void setConfiguration(int,int);
	void setSlope(int leg, int slope);
	void setRotation(int rot);

	void test();/*this function tests your quadruped with all angles set to 90 for proper caliberation*/
	void getAngles(int);
	void GoToPhase(int leg,int phase);
	void writeUpperA(int leg, int angle);
	void writeLowerA(int  leg,int angle);
	void initial();
	void alternateWalkMode();
	void staticGait();
	void RUN(int);
	writeUM(int leg,int angle);
	writeDM(int leg,int angle);
	void left();
	void right();
	void rightEx();
	void LeftEx();
	void straight();
	int currentPhase();

	TVquad();
	~TVquad();
};

const double pta[72][2] ={{0,0},{-4.3,0},{-8.6,0},{-12.9,0},{-21.5,0},{-25.8,0},
{-30.1,0},{-34.4,0},{-38.7,0},{-43,0},{-47.3,0},{-51.6,0},{-55.9,0},
{-60.2,0},{-64.5,0},{-68.8,0},{-73.1,0},{-77.4,0},{-88,1.8},{-96,6.4},
{-102.13,13},{-106,20},{-108,28},{-110,36},{-111,40},{-112,53},{-113,68},
{-112,77},{-111,84},{-108,102},{-106,109},{-100,119},{-91,123},{-87,128},
{-77.5,130},{-55.9,130},{55.6,130},{87,128},{91,123},{100,119},{104,113},
{106,109},{108,102},{111,84},{112,77},{113,68},{112,53},{111,40},
{110,36},{108,28},{106,20},{102.13,13},{102.13,13},{102.13,13},{77.4,0},
{73.1,0},{68.8,0},{64.5,0},{60.2,0},{55.9,0},{51.6,0},{47.3,0},{43,0},
{38.7,0},{34.4,0},{30.1,0},{25.8,0},{21.5,0},{17.2,0},{12.9,0},{8.6,0},
{4.3,0}};

const double pt1[72][2]={{0,0},{-2.87,0},{-5.74,0},{-8.61,0},
{-11.48,0},{-14.35,0},{-17.22,0},{-20.09,0},{-22.96,0},{-25.83,0},
{-28.7,0},{-31.57,0},{-34.44,0},{-37.31,0},{-40.18,0},{-43.05,0},
{-45.92,0},{-48.79,0},{-51.66,0},{-54.53,0},{-57.4,0},{-60.27,0},
{-63.14,0},{-66.01,0},{-68.88,0},{-71.75,0},{74.62,0},{-88,1.8},
{-96,6.4},{-102.13,13},{-106,20},{-111,40},{-113,68},{-100,119},
{-87,128},{-77.5,130},{55.6,130},{77.5,130},{87,128},{100,119},
{111,84},{113,68},{111,40},{108,28},{102.13,13},{77.5,0},{74.62,0},
{71.75,0},{68.88,0},{66.01,0},{63.14,0},{60.27,0},{57.4,0},
{54.53,0},{51.66,0},{48.79,0},{45.92,0},{43.05,0},{40.18,0},
{37.31,0},{34.44,0},{31.57,0},{28.7,0},{25.83,0},{22.96,0},
{20.09,0},{17.22,0},{14.35,0},{11.48,0},{8.61,0},{5.74,0},{2.87,0}};
#endif
