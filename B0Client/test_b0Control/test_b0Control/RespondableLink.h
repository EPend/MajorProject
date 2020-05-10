#pragma once

#include "Common.hpp"

class Respondable_Link {
private:
	b0RemoteApi * m_Client;

	bool m_ShutDown = false;

	std::vector<int> m_JointHandles;
	std::vector<int> m_SensorHandles;
	std::thread m_SensingThread;
	std::thread m_MainThread;

	void main_function(void);
	void sensing_function(void);

	int m_ModulePos = 0;

	float m_LeftDistance = 0.0;

	float m_RightDistance = 0.0;

	MOVE_MODE m_MoveMode = MOVE_MODE::REGULAR_MOVE;

	float m_hOscillation = 0.0;
	float m_vOscillation = 0.0;

	void CalculateMotion();
	void ExecuteMove();

	void LeftSensorCB(std::vector<msgpack::object>*);
	void RightSensorCB(std::vector<msgpack::object>*);

	void MoveModeCB(std::vector<msgpack::object>*);
	void HOscCB(std::vector<msgpack::object>*);
	void VOscCB(std::vector<msgpack::object>*);

public:
	Respondable_Link(b0RemoteApi *, int);
	void setJointHandles(std::vector<int>);
	void setSensorHandles(std::vector<int>);
	void start(void);
	~Respondable_Link(void);
};