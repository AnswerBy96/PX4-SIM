#pragma once
#include <math.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/custom_commander.h>


struct MotorState
{
	/* data */
};

struct BmsState
{
	/* data */
};

class StateMachine
{
public:
	StateMachine(/* args */);
	~StateMachine();

	StateMachine::StateMachine(/* args */)
	{
	}

	StateMachine::~StateMachine()
	{
	}

	enum Mode{
		MANUAL = 0,				//手动驾驶模式
		REMOTE,						//遥控模式
		AUTO,						//自动驾驶模式，包括巡航模式、锚定模式、进出港模式等
		DRIVE_ASSISTENCE		//辅助驾驶模式，包括巡航摸索、进出港模式等

	};

	enum SubMode{
		CRUISE = 0,						//巡航模式
		ANCHORING,				//锚定模式
		PORT						//进出港模式
	};

	enum ErrorWarning{
		MOTOR_ERROR = 0,
		BMS_ERROR = 1
	};

	enum MoveState{
		FORWARD = 0,
		BACK,
		STOP
	};

	/*
		param1: input_mode : drive mode (MANUAL , REMOTE , AUTO , DRIVE_ASSISTENCE)
		param2: input_submode : CRUISE , ANCHORING , PORT
	*/
	void run_state_machine(u_int8_t input_mode , u_int8_t input_submode , custom_commander_s &custom_commander);

private:


};


