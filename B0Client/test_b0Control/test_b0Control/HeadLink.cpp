#include "HeadLink.h"


void Head_Link::main_function(void)
{
	loguru::set_thread_name("HL_Actuation");

	if (m_VelocityX<0.08 && m_VelocityX>-0.08 && m_VelocityY<0.08 && m_VelocityY>-0.08)
	{
		m_Slow = true;
		LOG_F(WARNING, "Too slow!");
	}
	else
	{
		m_Slow = false;
	}



	while (!m_ShutDown)
	{
		m_Client->simxSpinOnce();
		m_Client->simxSleep(THREAD_POLLING_SPEED);
	}
}

void Head_Link::FrontSensorCB(std::vector<msgpack::object>* msg)
{
	loguru::set_thread_name("HL_Sensing");
	if (b0RemoteApi::readBool(msg, 0))
	{
		if (b0RemoteApi::readInt(msg, 1))
		{
			m_FrontDistance = b0RemoteApi::readFloat(msg, 2);
			LOG_F(INFO, "Updating front distance: %f", m_FrontDistance);
		}
		else
		{
			m_FrontDistance = 0.0;
		}
		
	}
}

void Head_Link::LeftSensorCB(std::vector<msgpack::object>* msg)
{
	loguru::set_thread_name("HL_Sensing");
	if (b0RemoteApi::readBool(msg, 0))
	{
		if (b0RemoteApi::readInt(msg, 1))
		{
			
			m_LeftDistance = b0RemoteApi::readFloat(msg, 2);
			LOG_F(INFO, "Updating left distance: %f", m_LeftDistance);
			std::vector<float> dpArray;
			b0RemoteApi::readFloatArray(msg, dpArray, 3);
			m_LeftDPX = dpArray[0];
			LOG_F(INFO, "Updating left distance-point X: %f", m_LeftDPX);
		}
		else
		{
			m_LeftDistance = 0.0;
		}

	}
}

void Head_Link::RightSensorCB(std::vector<msgpack::object>* msg)
{
	loguru::set_thread_name("HL_Sensing");
	if (b0RemoteApi::readBool(msg, 0))
	{
		if (b0RemoteApi::readInt(msg, 1))
		{
			m_RightDistance = b0RemoteApi::readFloat(msg, 2);
			LOG_F(INFO, "Updating right distance: %f", m_LeftDistance);
			std::vector<float> dpArray;
			b0RemoteApi::readFloatArray(msg, dpArray, 3);
			m_RightDPX = dpArray[0];
			LOG_F(INFO, "Updating right distance-point X: %f", m_LeftDPX);
		}
		else
		{
			m_RightDistance = 0.0;
		}

	}
}

void Head_Link::VelocityCB(std::vector<msgpack::object>* msg)
{
	loguru::set_thread_name("HL_Sensing");
	if (b0RemoteApi::readBool(msg, 0))
	{
		std::vector<float> dpArray;
		b0RemoteApi::readFloatArray(msg, dpArray, 1);
		m_VelocityX = dpArray[0];
		m_VelocityY = dpArray[1];
		LOG_F(INFO, "Updating linear velocity X: %f Y: %f", m_VelocityX, m_VelocityY);
	}
}


void Head_Link::sensing_function()
{
	loguru::set_thread_name("HL_Sensing");
	LOG_F(INFO, "Registering object-velocity callback...");
	m_Client->simxGetObjectVelocity(m_SensorHandles[0], m_Client->simxDefaultSubscriber(
		[this](std::vector<msgpack::object>* msg)
		{
			VelocityCB(msg);
		}
	));

	LOG_F(INFO, "Registering front-sensor callback...");
	m_Client->simxReadProximitySensor(m_SensorHandles[0],m_Client->simxDefaultSubscriber(
		[this](std::vector<msgpack::object>* msg)
		{ 
			FrontSensorCB(msg); 
		}
	));
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
	while (!m_ShutDown)
	{
		m_Client->simxSpinOnce();
		m_Client->simxSleep(THREAD_POLLING_SPEED);
	}
}

void Head_Link::start()
{
	m_MainThread = std::thread(&Head_Link::main_function, this);
	m_SensingThread = std::thread(&Head_Link::sensing_function, this);
	
	m_MainThread.join();
	m_SensingThread.join();
}

Head_Link::Head_Link(b0RemoteApi * client)
{
	m_Client = client;
}

void Head_Link::setJointHandles(std::vector<int> handles)
{
	m_JointHandles = handles;
}

void Head_Link::setSensorHandles(std::vector<int> handles)
{
	m_SensorHandles = handles;
}

Head_Link::~Head_Link()
{
}
