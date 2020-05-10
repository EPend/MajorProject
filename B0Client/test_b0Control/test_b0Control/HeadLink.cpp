#include "HeadLink.h"


void Head_Link::main_function(void)
{
	loguru::set_thread_name("HL_Actuation");

	while (!m_ShutDown)
	{
		CalculateMotion();
		ExecuteMoveAndPublish();

		m_Client->simxSpinOnce();
		m_Client->simxSleep(THREAD_POLLING_SPEED);
	}
}

void Head_Link::CalculateMotion()
{
	loguru::set_thread_name("HL_Actuation");
	if (m_VelocityX<0.08 && m_VelocityX>-0.08 && m_VelocityY<0.08 && m_VelocityY>-0.08)
	{
		LOG_F(WARNING, "Too slow!");
		m_vTurnSpeed = 0.0;
		m_MoveMode = MOVE_MODE::REGULAR_MOVE;
	}
	else if (m_LeftDistance > 0.0 || m_RightDistance > 0.0)
	{
		m_TurnCounter = 0;
		if (m_LeftDistance > 0.0 && m_RightDistance > 0.0)
		{
			m_MoveMode = MOVE_MODE::WALL_FOLLOWING_BOTH_WALLS;
		}
		else
		{
			m_MoveMode = MOVE_MODE::WALL_FOLLOWING;
			if (m_LeftDistance > 0.0)
			{

			}
			else if (m_RightDistance > 0.0)
			{

			}
		}
	}
	else
	{
		if (m_CurrentWall == CURRENT_WALL::LEFT)
		{
			if (m_TurnCounter == 3)
			{
				m_MoveMode = MOVE_MODE::TURNING;
				LOG_F(INFO, "Turning left...");
				if (m_vTurnSpeed > 0) m_vTurnSpeed = 0;
				m_vTurnSpeed -= 0.1F;
				if (m_vTurnSpeed < -1.0) m_vTurnSpeed = -1.0;
			}
			else
			{
				m_TurnCounter += 1;
			}
		}
		else if (m_CurrentWall == CURRENT_WALL::RIGHT)
		{
			if (m_TurnCounter == 3)
			{
				m_MoveMode = MOVE_MODE::TURNING;
				LOG_F(INFO, "Turning right...");
				if (m_vTurnSpeed > 0) m_vTurnSpeed = 0;
				m_vTurnSpeed += 0.1F;
				if (m_vTurnSpeed < 1.0) m_vTurnSpeed = 1.0;
			}
			else
			{
				m_TurnCounter += 1;
			}
		}
	}

	if (m_MoveMode == MOVE_MODE::REGULAR_MOVE)
	{
		DLOG_F(INFO, "Moving forwards...");
		float fLeftRangePercent = 0.0;
		float fRightRangePercent = 0.0;

		if (m_LeftDistance > 0.0)
		{
			fLeftRangePercent = (float)(m_LeftDistance / (MAX_SENSOR_DISTANCE / 2));
		}
		else
		{
			fLeftRangePercent = 1.0;
		}

		if (m_RightDistance > 0.0)
		{
			fRightRangePercent = (float)(m_RightDistance / (MAX_SENSOR_DISTANCE / 2));
		}
		else
		{
			fRightRangePercent = 1.0;
		}

		if (fRightRangePercent > 1.0) fRightRangePercent = 1.0;
		if (fLeftRangePercent > 1.0) fLeftRangePercent = 1.0;

		m_vTurnSpeed = 0.0;

		if (m_CurrentOscPhase == m_eOscPhase::RIGHT)
		{
			if (m_vOscillation >= m_OscRange[0] * fRightRangePercent)
			{
				m_CurrentOscPhase = m_eOscPhase::LEFT;
			}
			else
			{
				m_vOscillation += m_DefaultOscStep;
			}
		}
		else
		{
			if (m_vOscillation <= m_OscRange[1] * fRightRangePercent)
			{
				m_CurrentOscPhase = m_eOscPhase::RIGHT;
			}
			else
			{
				m_vOscillation -= m_DefaultOscStep;
			}
		}
	}
	else
	{
		m_vOscillation = m_vTurnSpeed;
	}
}

void Head_Link::ExecuteMoveAndPublish()
{
	loguru::set_thread_name("HL_Actuation");
	//m_Client->simxSetIntSignal("MoveMode", (int)m_MoveMode, m_Client->simxDefaultPublisher());
	//m_Client->simxSetFloatSignal("vOscellation", m_vOscillation, m_Client->simxDefaultPublisher());
	//m_Client->simxSetFloatSignal("hOscellation", m_hOscillation, m_Client->simxDefaultPublisher());
}

void Head_Link::FrontSensorCB(std::vector<msgpack::object>* msg)
{
	loguru::set_thread_name("HL_Sensing");
	if (b0RemoteApi::readBool(msg, 0))
	{
		if (b0RemoteApi::readInt(msg, 1))
		{
			m_FrontDistance = b0RemoteApi::readFloat(msg, 2);
			DLOG_F(INFO, "Updating front distance: %f", m_FrontDistance);
		}
		else
		{
			m_FrontDistance = 0.0;
		}
	}
	else
	{
		LOG_F(ERROR, "Front-sensor callback failed!");
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
			DLOG_F(INFO, "Updating left distance: %f", m_LeftDistance);
			std::vector<float> dpArray;
			b0RemoteApi::readFloatArray(msg, dpArray, 3);
			m_LeftDPX = dpArray[0];
			DLOG_F(INFO, "Updating left distance-point X: %f", m_LeftDPX);
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

void Head_Link::RightSensorCB(std::vector<msgpack::object>* msg)
{
	loguru::set_thread_name("HL_Sensing");
	if (b0RemoteApi::readBool(msg, 0))
	{
		if (b0RemoteApi::readInt(msg, 1))
		{
			m_RightDistance = b0RemoteApi::readFloat(msg, 2);
			DLOG_F(INFO, "Updating right distance: %f", m_LeftDistance);
			std::vector<float> dpArray;
			b0RemoteApi::readFloatArray(msg, dpArray, 3);
			m_RightDPX = dpArray[0];
			DLOG_F(INFO, "Updating right distance-point X: %f", m_LeftDPX);
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

void Head_Link::VelocityCB(std::vector<msgpack::object>* msg)
{
	loguru::set_thread_name("HL_Sensing");
	if (b0RemoteApi::readBool(msg, 0))
	{
		std::vector<float> dpArray;
		b0RemoteApi::readFloatArray(msg, dpArray, 1);
		m_VelocityX = dpArray[0];
		m_VelocityY = dpArray[1];
		DLOG_F(INFO, "Updating linear velocity X: %f Y: %f", m_VelocityX, m_VelocityY);
	}
	else
	{
		LOG_F(ERROR, "Velocity callback failed!");
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
