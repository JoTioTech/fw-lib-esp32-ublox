# ESP32 GPS library for ublox library

Does not work as of now, I've just started writing the code

Basic library enabling message parsing and configuration with UBlox GPS devices.
Primarily designed for UBlox M9V and ESP32C6 however it should also work with other devices.
Basic parsers has been taken from [UBLOX_read](https://github.com/superjax/UBLOX_read)

```cpp
void got_ubx(const uint8_t cls, const uint8_t id, const ublox::UBX_message_t& msg)
{
		if (cls == ublox::CLASS_NAV && id == ublox::NAV_RELPOSNED)
				relposned_callback(cls, id, msg.NAV_RELPOSNED);
}

void relposned_callback(uint8_t cls, uint8_t type, const ublox::NAV_RELPOSNED_t& msg)
{
		int RTK_flag;
		if (msg.flags && 0b000000010)
		{
				printf("RTK");
				RTK_flag = 1;
		}
		else
		{
				printf("No RTK \n");
				RTK_flag = 0;
		}
		if (RTK_flag == 1)
		{
				if (msg.flags && 0b000100000)
						printf(", Moving Base");
				if (msg.flags && 0b000001000)
						printf(" , Floating \n");
				else if (msg.flags && 0b000010000)
						printf(" , Fixed \n");
				if (msg.flags && 0b000000100)
						printf("valid relative position components \n");
				printf("tow: %d relNED: %d, %d, %d, Distance: %d\n", msg.iTow / 1000, msg.relPosN,
							 msg.relPosE, msg.relPosD, msg.relPosLength);
		}
		fflush(stdout);  // Will now print everything in the stdout buffer
}
```
