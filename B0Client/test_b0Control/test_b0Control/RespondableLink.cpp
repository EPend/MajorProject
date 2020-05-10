#include "RespondableLink.h"

void Respondable_Link::main_function(void)
{
	loguru::set_thread_name("HL_Actuation");

	while (!m_ShutDown)
	{
		CalculateMotion();
		ExecuteMove();

		m_Client->simxSpinOnce();
		m_Client->simxSleep(THREAD_POLLING_SPEED);
	}
}

void Respondable_Link::sensing_function(void)
{
	loguru::set_thread_name("HL_Sensing");

	LOG_F(INFO, "Registering MoveMode callback...");
	m_Client->simxGetIntSignal("MoveMode", m_Client->simxDefaultSubscriber(
		[this](std::vector<msgpack::object>* msg)
		{
			MoveModeCB(msg);
		}
	));
	LOG_F(INFO, "Registering vOscellation callback...");
	m_Client->simxGetFloatSignal("vOscellation", m_Client->simxDefaultSubscriber(
		[this](std::vector<msgpack::object>* msg)
		{
			VOscCB(msg);
		}
	));
	LOG_F(INFO, "Registering hOscellation callback...");
	m_Client->simxGetFloatSignal("hOscellation", m_Client->simxDefaultSubscriber(
		[this](std::vector<msgpack::object>* msg)
		{
			HOscCB(msg);
		}
	));

	/*
	LOG_F(INFO, "Registering left-sensor callback...");
	m_Client->simxReadProximitySensor(m_SensorHandles[1], m_Client->simxDefaultSubscriber(
		[this](std::vector<msgpack::object>* msg)
		{
			LeftSensorCB(msg);
		}
	));
	LOG_F(INFO, "Registering right-sensor callback...");
	m_Client->simxReadProximitySensor(m_SensorHandles[2], m_Client->simxDefaultSubscriber(
		[this](std::vector<msgpack::object>* msg)
		{
			RightSensorCB(msg);
		}
	));
	*/
	while (!m_ShutDown)
	{
		m_Client->simxSpinOnce();
		m_Client->simxSleep(THREAD_POLLING_SPEED);
	}
}

void Respondable_Link::CalculateMotion()
{
}

void Respondable_Link::ExecuteMove()
{
}

void Respondable_Link::LeftSensorCB(std::vector<msgpack::object>* msg)
{
	loguru::set_thread_name("HL_Sensing");
	if (b0RemoteApi::readBool(msg, 0))
	{
		if (b0RemoteApi::readInt(msg, 1))
		{

			m_LeftDistance = b0RemoteApi::readFloat(msg, 2);
			DLOG_F(INFO, "Updating left distance: %f", m_LeftDistance);
		}
		else
		{
			m_LeftDistance = 0.0;
		}
	}
	else
	{
		LOG_F(ERROR, "Left-sensor callback failed!");
	}
}

void Respondable_Link::RightSensorCB(std::vector<msgpack::object>* msg)
{
	loguru::set_thread_name("HL_Sensing");
	if (b0RemoteApi::readBool(msg, 0))
	{
		if (b0RemoteApi::readInt(msg, 1))
		{
			m_RightDistance = b0RemoteApi::readFloat(msg, 2);
			DLOG_F(INFO, "Updating right distance: %f", m_LeftDistance);
		}
		else
		{
			m_RightDistance = 0.0;
		}
	}
	else
	{
		LOG_F(ERROR, "Right-sensor callback failed!");
	}
}

void Respondable_Link::MoveModeCB(std::vector<msgpack::object>* msg)
{
	if (b0RemoteApi::readBool(msg, 0))
	{
		m_MoveMode = (MOVE_MODE)b0RemoteApi::readInt(msg, 1);

	}
	else
	{
		LOG_F(ERROR, "MoveMode callback failed!");
	}
}

void Respondable_Link::HOscCB(std::vector<msgpack::object>* msg)
{
	if (b0RemoteApi::readBool(msg, 0))
	{
		m_hOscillation = b0RemoteApi::readFloat(msg, 1);
	}
	else
	{
		LOG_F(ERROR, "HOsc callback failed!");
	}
}

void Respondable_Link::VOscCB(std::vector<msgpack::object>* msg)
{
	if (b0RemoteApi::readBool(msg, 0))
	{
		m_vOscillation = b0RemoteApi::readFloat(msg, 1);
	}
	else
	{
		LOG_F(ERROR, "VOsc callback failed!");
	}
}

Respondable_Link::Respondable_Link(b0RemoteApi * client, int modulePos)
{
	m_Client = client;
	m_ModulePos = modulePos;
}

void Respondable_Link::setJointHandles(std::vector<int> handles)
{
	m_JointHandles = handles;
}

void Respondable_Link::setSensorHandles(std::vector<int> handles)
{
	m_SensorHandles = handles;
}

void Respondable_Link::start(void)
{
	m_MainThread = std::thread(&Respondable_Link::main_function, this);
	m_SensingThread = std::thread(&Respondable_Link::sensing_function, this);

	m_MainThread.join();
	m_SensingThread.join();
}

Respondable_Link::~Respondable_Link(void)
{
}
