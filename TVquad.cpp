#include "TVquad.h"
int main(){	return 0;}

TVquad::TVquad(){}
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
void TVquad::test(){
	
}












