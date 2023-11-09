/*
 * @Descripttion:
 * @version:
 * @Author: chenjw
 * @Date: 2023-11-01 02:56:18
 * @LastEditors: rsj
 * @LastEditTime: 2023-11-02 00:27:55
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
		return eboat_mavlink_sub.advertised() ? MAVLINK_MSG_ID_PX4_TO_UI + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamEboatMavlink(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription eboat_mavlink_sub{ORB_ID(px4_to_ui)};

	bool send() override
	{
		px4_to_ui_s px4_to_ui_;

		if (eboat_mavlink_sub.update(&px4_to_ui_)) {
			mavlink_px4_to_ui_t msg{};
			mavlink_msg_px4_to_ui_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // MOUNT_ORIENTATION
