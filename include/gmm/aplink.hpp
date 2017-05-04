#ifndef APLINK_H
#define APLINK_H 1

#include <stdint.h>

#define AP_DATALEN 68UL
#define AP_CMDLEN  8UL

namespace ap
{
	/// Data container for all Autopilot-generated feedback/timing signals
	typedef struct data_x
	{
		float time_stamp;
		float wheel_R;
		float wheel_L;
		float gyro[3UL];
		float acc1[3UL];
		float acc2[3UL];
		float magn[3UL];
		float velocity;
		float omega;
	} data_t;

	/// Packet support container for serial communication from AP
	typedef struct ret_x
	{
		union
		{
			data_t	val;
			uint8_t bytes[AP_DATALEN];
		} Data;

		uint8_t* header;

		uint32_t header_idx;
		uint32_t header_len;

		uint32_t data_idx;
		uint32_t data_len;

		uint8_t  checksum;
	} ret_t;

	/// Packet support contain for serial communication to AP
	typedef struct cmd_x
	{
		union
		{
			float   val[2];
			uint8_t bytes[AP_CMDLEN];
		} Data;
	} cmd_t;

	void init();
	void rxproc();
	void setCmd(const float& velocity, const float& omega);

}

#endif