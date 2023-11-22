/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-11-01 02:56:18
 * @LastEditors: rsj
 * @LastEditTime: 2023-11-20 22:22:13
 */
#ifndef EBOAT_MAVLINK_HPP
#define EBOAT_MAVLINK_HPP

#include<uORB/topics/px4_to_ui.h>
class MavlinkStreamEboatMavlink : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamEboatMavlink(mavlink); }

	static constexpr const char *get_name_static() { return "PX4_TO_UI"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_PX4_TO_UI; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return eboat_mavlink_sub.advertised() ? MAVLINK_MSG_ID_PX4_TO_UI_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamEboatMavlink(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription eboat_mavlink_sub{ORB_ID(px4_to_ui)};

	bool send() override
	{
		px4_to_ui_s px4_to_ui_;

		if (eboat_mavlink_sub.update(&px4_to_ui_)) {
			mavlink_px4_to_ui_t msg{};
			msg.timestamp = px4_to_ui_.timestamp;
			msg.motor1_speed = px4_to_ui_.motor1_speed;
			msg.motor2_speed = px4_to_ui_.motor2_speed;
			msg.motor1_direction = px4_to_ui_.motor1_direction;
			msg.motor2_direction = px4_to_ui_.motor2_direction;
			msg.eboat_speed = px4_to_ui_.eboat_speed;
			msg.eboat_heading = px4_to_ui_.eboat_heading;
			msg.gear = px4_to_ui_.gear;

			mavlink_msg_px4_to_ui_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // MOUNT_ORIENTATION
