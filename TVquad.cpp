#include "TVquad.h"
TVquad::TVquad(){
	trajectory_multiplier = 1;
	elbow_position[0] = 0;
	elbow_position[1] = 440;
	current_phase = 00;
	config = -1;
	direction = 1;
	l1 = 230;
	l2 = 250;

	offset[0] = 0;
	offset[1] = 0;
	offset[2] = 0;
	offset[3] = 0;
	offset[4] = 0;
	offset[5] = 0;
	offset[6] = 0;
	offset[7] = 0;
	signB[0] = 1;
	signB[1] = 1;
	signB[2] = 1;
	signB[3] = 1;
	signB[4] = 1;
	signB[5] = 1;
	signB[6] = 1;
	signB[7] = 1;
	for(1nt i=0 ; i<72 ; i++){	
		points_of_trajectory[i][0]=pta[i][0]*trajectory_multiplier;
		points_of_trajectory[i][1]=pta[i][1]*trajectory_multiplier;
	}
	slope = 0;
	rotation = 0;
	phase_diff[0] = 0;
	phase_diff[1] = 180;
	phase_diff[2] = 0;
	phase_diff[3] = 180;
	
	height_mul[0] = 1;
	height_mul[1] = 1;
	height_mul[2] = 1;
	height_mul[3] = 1;
	length_mul[0] = 1;
	length_mul[1] = 1;
	length_mul[2] = 1;
	length_mul[3] = 1;
	delay_ms = 2000;
}
TVquad::~TVquad(){}

void TVquad::setpoints(double a[72][2]){
	for(int i=0;i<72;i++){
		points_of_trajectory[i][0]=a[i][0];
		points_of_trajectory[i][1]=a[i][1];
	}
}
void TVquad::setUpperMotorf(void (*motor)(int leg,int angle)){
	uMotor=motor;
}
void TVquad::setLowerMotorf(void (*motor)(int leg,int angle)){
	dMotor=motor;
}
void TVquad::setDelayf(void (*system_Delay)(int microseconds)){
	Delay=system_Delay;
}
void TVquad::setDirection(int a[8]){
	signB[0] = a[0];
	signB[1] = a[1];
	signB[2] = a[2];
	signB[3] = a[3];
	signB[4] = a[4];
	signB[5] = a[5];
	signB[6] = a[6];
	signB[7] = a[7];
}

void TVquad::setOffsets(int a[8]){
	offset[0] = a[0];
	offset[1] = a[1];
	offset[2] = a[2];
	offset[3] = a[3];
	offset[4] = a[4];
	offset[5] = a[5];
	offset[6] = a[6];
	offset[7] = a[7];
}
void TVquad::setPhaseDiff(int a[4]){
	phase_diff[0] = a[0];
	phase_diff[1] = a[1];
	phase_diff[2] = a[2];
	phase_diff[3] = a[3];
}
void TVquad::setHeightMul(int leg,double mul){
	if(leg<4 && leg>=0){
		height_mul[leg] = mul;
	}
}
void TVquad::setLengthMul(int leg,double mul){
	if(leg<4 && leg>=0){
		length_mul[leg] = mul;
	}
}
void TVquad::setLinkLength(int upper,int lower){
	l1 = upper;
	l2 = lower;
	trajectory_multiplier = (l1+l2)/49;
}
void TVquad::setDelay(int delayMicroseconds){
	delay_ms = delayMicroseconds;
}
void TVquad::setConfiguration(int c,int d){
	config = c;
	direction = d;
}
void TVquad::setSlope(int s){
	slope = s;
}
void TVquad::setRotation(int rot){
	rotation = rot;
}
void TVquad::setElbow(int leg, int pos[]){
	if(leg>3){
		setElbow(0,pos);
		setElbow(1,pos);
		setElbow(2,pos);
		setElbow(3,pos);
	}
	else{
		elbow_position[leg][0] = pos[0];
		elbow_position[leg][1] = pos[1];
	}
}
void TVquad::test(){
	writeUpperA(0,90);
	writeUpperA(1,90);
	writeUpperA(2,90);
	writeUpperA(3,90);
	writeLowerA(0,90);
	writeLowerA(1,90);
	writeLowerA(2,90);
	writeLowerA(3,90);
}
void TVquad::writeUpperA(int leg, int angle){
	if(leg>2)
		(*uMotor)(leg,((180+(angle+offset[leg]+rotation)*signB[leg])%180));
	else
		(*uMotor)(leg,((180+(angle+offset[leg]+(rotation*config))*signB[leg])%180));
}
void TVquad::writeLowerA(int leg, int angle){
	(*dMotor)(leg,((180+(angle+offset[leg+4])*signB[leg+4])%180));
}
void TVquad::getAngles(int leg){
	if(leg>3){
		getAngles(0);
		getAngles(1);
		getAngles(2);
		getAngles(3);
	}
	else{
		int al,th,m;
		int ex = elbow_position[leg][0];
		int ey = elbow_position[leg][1];
		double x,y,r,th1,th2,a2;
		double sin_slope = sin(slope);
		double cos_slope = cos(slope);
		for(int i=0 ; i < 72 ; i++){
			if(leg > 1)	m = config*direction;
			else 		m = direction;

			x = points_of_trajectory[i][0]*m*height_mul[i];
			y = points_of_trajectory[i][1]*length_mul[i];
			if(slope!=0){
				x = cos_slope*x + sin_slope*y;
				y = cos_slope*y - sin_slope*x;
			}

			r = sqrt(s(ex-x) + s(ey-y));

			th1 = acos((s(l1) + s(r) - s(l2)) / (2*l1*r) );
			th1 = r2d(th1);
			if(ex == x)	th2 = 90;
			else{
				th2 = atan((ey-y)/(ex-x));
				th2 = r2d(th2);
				if(th2 < 0)	th2+=180;
			}
			a2 = acos((s(l2) + s(r) - s(l1)) / (2*l2*r) );
			a2 = r2d(a2);
			al = (180 - th1 - a2);
			th = th1+th2;

			angles[leg][i][0] = th;
			angles[leg][i][1] = al;
		}
	}
}
void TVquad::goToPhase(int leg, int phase){
	phase = phase%360;
	phase = phase/5;
	phase = phase%72;
	writeUpperA(leg,angles[leg][phase][0]);
	writeLowerA(leg,angles[leg][phase][1]);
}
void TVquad::initial(){
	goToPhase(0,0);
	goToPhase(1,0);
	goToPhase(2,0);
	goToPhase(3,0);
}
void TVquad::alternateWalkMode(){
	for(int i=0 ; i<72 ; i++){	
		points_of_trajectory[i][0]=pta[i][0]*trajectory_multiplier;
		points_of_trajectory[i][1]=pta[i][1]*trajectory_multiplier;
	}
	slope = 0;
	rotation = 0;
	phase_diff[0] = 0;
	phase_diff[1] = 180;
	phase_diff[2] = 0;
	phase_diff[3] = 180;
	
	height_mul[0] = 1;
	height_mul[1] = 1;
	height_mul[2] = 1;
	height_mul[3] = 1;
	length_mul[0] = 1;
	length_mul[1] = 1;
	length_mul[2] = 1;
	length_mul[3] = 1;
	delay_ms = 2000;
}
void TVquad::staticGait(){
	for(int i=0 ; i<72 ; i++){	
		points_of_trajectory[i][0]=pt1[i][0]*trajectory_multiplier;
		points_of_trajectory[i][1]=pt1[i][1]*trajectory_multiplier;
	}
	slope = 0;
	rotation = 0;
	phase_diff[0] = 0;
	phase_diff[1] = 180;
	phase_diff[2] = 90;
	phase_diff[3] = 270;
	
	height_mul[0] = 1;
	height_mul[1] = 1;
	height_mul[2] = 1;
	height_mul[3] = 1;
	length_mul[0] = 1;
	length_mul[1] = 1;
	length_mul[2] = 1;
	length_mul[3] = 1;
	delay_ms = 7000;
}
void TVquad::RUN(int pause_play){
	current_phase+=(pause_play&1);
	goToPhase(0,current_phase+phase_diff[0]);
    goToPhase(1,current_phase+phase_diff[1]);
    goToPhase(2,current_phase+phase_diff[2]);
    goToPhase(3,current_phase+phase_diff[3]);
    (*Delay)(delay_ms);
}
void TVquad::left(){
	length_mul[0] = 0.6;
	length_mul[2] = 0.6;
	length_mul[1] = 1;
	length_mul[3] = 1;
}
void TVquad::right(){
	length_mul[0] = 1;
	length_mul[2] = 1;
	length_mul[1] = 0.6;
	length_mul[3] = 0.6;
}
void TVquad::leftEx(){
	length_mul[0] = -0.2;
	length_mul[2] = -0.2;
	length_mul[1] = 1;
	length_mul[3] = 1;
}
void TVquad::rightEx(){
	length_mul[0] = 1;
	length_mul[2] = 1;
	length_mul[1] = -0.2;
	length_mul[3] = -0.2;
}
void TVquad::straight(){
	height_mul[0] = 1;
	height_mul[1] = 1;
	height_mul[2] = 1;
	height_mul[3] = 1;
	length_mul[0] = 1;
	length_mul[1] = 1;
	length_mul[2] = 1;
	length_mul[3] = 1;
}
int TVquad::currentPhase(){
	return current_phase;
}