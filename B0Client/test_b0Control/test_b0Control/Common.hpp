#pragma once

#define LOGURU_DEBUG_LOGGING 0

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include "loguru.hpp"
#include <thread>

#pragma warning ( push )
#pragma warning ( disable: 4100 )
#pragma warning ( disable: 4456 )
#include "b0RemoteApi.h"
#pragma warning ( pop )


enum LINK_TYPE {
	HEAD_LINK = 0,
	RESPONDABLE_LINK = 1
};

enum MOVE_MODE {
	REGULAR_MOVE = 0,
	WALL_FOLLOWING = 1,
	TURNING = 2,
	WALL_FOLLOWING_BOTH_WALLS = 3
};

enum CURRENT_WALL {
	NONE = 0,
	LEFT = 1,
	RIGHT = 2
};

#define THREAD_POLLING_SPEED 50

#define MAX_SENSOR_DISTANCE 0.43
#define MIN_SENSOR_DISTANCE 0

#define UPPER_SENSOR_DISTANCE_THRESHOLD 0.20
#define LOWER_SENSOR_DISTANCE_THRESHOLD 0.15

#define MAX_OSC_RATE 0.15

#define SNAKE_MASS 1.1
#define STR -20

