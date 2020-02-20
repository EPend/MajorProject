#include "b0RemoteApi.h"
#include "ErrCode.h"

b0RemoteApi* cl = nullptr;
static float simTime = 0.0f;
static int sensorTrigger = 0;
static long lastTimeReceived = 0;

/* PARAM: vJoint, hJoint, body, bodyS, prox */
int main(int argc, char* argv[])
{

	b0RemoteApi client("b0RemoteApi_c++Client", "b0RemoteApi");
	cl = &client;

	int vJoint, hJoint, body, bodyS, prox;

	if (argc >= 5) 
	{
		vJoint = atoi(argv[1]);
		hJoint = atoi(argv[2]);
		body = atoi(argv[3]);
		bodyS = atoi(argv[4]);
		prox = atoi(argv[5]);
		std::cout << "Valid handles: " << vJoint << ", " << hJoint << ", " << body << ", " << bodyS << ", " << prox << std::endl;
	}
	else
	{
		return PARAM_ERR;
	}


	std::cout << "Ended!" << std::endl;

	return(0);
}

