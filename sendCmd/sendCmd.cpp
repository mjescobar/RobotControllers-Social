#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

extern "C" {
    #include "extApi.h"
/*	#include "extApiCustom.h" if you wanna use custom remote API functions! */
}

int main(int argc,char* argv[])
{
	int portNb=19997;
	int leftMotorHandle;
	int rightMotorHandle;
	int sensorHandle;
	if(argc<2)
	{
		printf("not enough minerals!\n");
		return(0);
	}
	if(argc==3) 
		sleep(atoi(argv[2]));
	else 
		sleep(1);
	int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
	if (clientID!=-1)
	{
		if(strcmp(argv[1],"-start")==0)	simxStartSimulation(clientID,simx_opmode_oneshot_wait);
		if(strcmp(argv[1],"-stop")==0)	simxStopSimulation(clientID,simx_opmode_oneshot_wait);
		if(strcmp(argv[1],"-pause")==0)	simxPauseSimulation(clientID,simx_opmode_oneshot_wait);

		simxFinish(clientID);
	}
	return(0);
}

