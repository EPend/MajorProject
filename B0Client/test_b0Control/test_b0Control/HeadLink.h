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

	enum m_eOscPhase {
		LEFT = 0, 
		RIGHT = 1
	};

	m_eOscPhase m_CurrentOscPhase = m_eOscPhase::LEFT;

	MOVE_MODE m_MoveMode = MOVE_MODE::REGULAR_MOVE;

	float m_vOscillation = 0.0;
	float m_vTurnSpeed = 0.0;
	float m_hOscillation = 0.0;
	float m_hTurnSpeed = 0.0;
	int m_TurnCounter = 0;

	std::vector<double> m_OscRange = { 0.6, -0.6 };
	float m_DefaultOscStep = 0.1F;

	float m_VelocityX = 0.0;
	float m_VelocityY = 0.0;

	CURRENT_WALL m_CurrentWall = CURRENT_WALL::NONE;

	float m_DWDistanceThreshold = 0.025F;
	float m_DWMaxOscRate = 0.2F;
	float m_DWPanicThreshold = 0.05F;

	void CalculateMotion();
	void ExecuteMoveAndPublish();

	void FrontSensorCB(std::vector<msgpack::object>*);
	void LeftSensorCB(std::vector<msgpack::object>*);
	void RightSensorCB(std::vector<msgpack::object>*);
	void VelocityCB(std::vector<msgpack::object>*);

	const char * m_ClientPublisher = nullptr;

public:
	Head_Link(b0RemoteApi *);
	void setJointHandles(std::vector<int>);
	void setSensorHandles(std::vector<int>);
	void start(void);
	~Head_Link(void);
};