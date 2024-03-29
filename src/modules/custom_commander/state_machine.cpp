#include "state_machine.hpp"

void StateMachine::run_state_machine(u_int8_t input_mode , u_int8_t input_submode , custom_commander_s &custom_commander)
{
	switch (input_mode)
	{
	case MANUAL:
		/* code */
		break;

	case REMOTE:
		/* code */
		break;

	case AUTO:
		{
			switch (input_submode)
			{
			case CRUISE:
				/* code */
				break;
			case ANCHORING:
				/* code */
				break;
			case PORT:
				/* code */
				break;
			default:
				break;
			}
		}
		break;

	case DRIVE_ASSISTENCE:
		{
			switch (input_submode)
			{
			case CRUISE:
				/* code */
				break;
			case PORT:
				/* code */
				break;
			default:
				break;
			}

		}
		break;
	default:
		break;
	}
}
