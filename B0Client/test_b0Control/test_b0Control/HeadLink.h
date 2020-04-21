#pragma once
#include "Common.hpp"

class Head_Link {
private:
	b0RemoteApi * m_Client;

	bool m_ShutDown = false;

	std::vector<int> m_JointHandles;
	std::vector<int> m_SensorHandles;
	std::thread m_SensingThread;
	std::thread m_MainThread;

	void main_function(void);
	void sensing_function(void);

	float m_LeftDistance = 0.0;
	float m_LeftDPX = 0.0;

	float m_RightDistance = 0.0;
	float m_RightDPX = 0.0;

	float m_FrontDistance = 0.0;

	enum m_OscPhase {
		LEFT = 0, 
		RIGHT = 1
	};

	double m_Oscillation = 0.0;
	double m_TurnSpeed = 0.0;
	int m_TurnCounter = 0;

	std::vector<double> m_OscRange = { 0.6, -0.6 };
	double m_DefaultOscStep = 0.1;

	double m_VelocityX = 0.0;
	double m_VelocityY = 0.0;

	bool m_Slow = true;

	CURRENT_WALL m_CurrentWall = CURRENT_WALL::NONE;

	double m_DWDistanceThreshold = 0.025;
	double m_DWMaxOscRate = 0.2;
	double m_DWPanicThreshold = 0.05;

	void FrontSensorCB(std::vector<msgpack::object>*);
	void LeftSensorCB(std::vector<msgpack::object>*);
	void RightSensorCB(std::vector<msgpack::object>*);
	void VelocityCB(std::vector<msgpack::object>*);

public:
	Head_Link(b0RemoteApi *);
	void setJointHandles(std::vector<int>);
	void setSensorHandles(std::vector<int>);
	void start(void);
	~Head_Link(void);
};