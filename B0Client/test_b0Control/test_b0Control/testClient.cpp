#pragma once


#include "ErrCode.h"
#include "Common.hpp"
#include "HeadLink.h"

Head_Link * head = nullptr;
b0RemoteApi* cl = nullptr;
static float simTime = 0.0f;
static int sensorTrigger = 0;
static long lastTimeReceived = 0;

using namespace std;

/* PARAM: vJoint, hJoint, body, bodyS, prox */
int main(int argc, char* argv[])
{
	loguru::init(argc, argv);

	// Put every log message in "everything.log":
	loguru::add_file("everything.log", loguru::Append , loguru::Verbosity_MAX);

	int vJoint, hJoint, body, bodyS, prox, leftSensor, rightSensor, visionSensor;
	Sleep(2000);
	
	if (argc >= 2)
	{
		if (atoi(argv[1]) == LINK_TYPE::HEAD_LINK && argc == 10)
		{
			b0RemoteApi client("b0RemoteApi_Head_Client", "b0RemoteApi");
			cl = &client;

			vJoint = atoi(argv[2]);
			hJoint = atoi(argv[3]);
			body = atoi(argv[4]);
			bodyS = atoi(argv[5]);
			prox = atoi(argv[6]);
			leftSensor = atoi(argv[7]);
			rightSensor = atoi(argv[8]);
			visionSensor = atoi(argv[9]);

			LOG_F(INFO, "Successfully initialized!");

			head = new Head_Link(cl);
			head->setJointHandles({ vJoint, hJoint, body, bodyS });
			head->setSensorHandles({ prox, leftSensor, rightSensor, visionSensor });
			head->start();
			
		}
		else
		{
			/*
			string clientName = "b0RemoteApi_Respondable_Client_" + to_string(atoi(argv[2]));
			b0RemoteApi client(clientName.c_str(), "b0RemoteApi");
			cl = &client;
			*/
			LOG_F(ERROR, "Parameter error!");
			return(ERROR_CODE::PARAM_ERR);
		}
	}

	LOG_F(INFO, "Successfully ended!");

	return(ERROR_CODE::SUCCESS);
}