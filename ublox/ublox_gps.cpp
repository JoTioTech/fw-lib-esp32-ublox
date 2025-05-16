/* Copyright (c) 2019 James Jackson, Matt Rydalch, Havránek Kryštof
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <cassert>
#include <chrono>
#include <cstring>
#include <sstream>
#include <stdio.h>

#include "ublox_gps.h"


using namespace std;


namespace ublox {
QueueHandle_t UbloxGPS::uart0_queue;

UbloxGPS::UbloxGPS()
{
	buffer_head_ = 0;
	parse_state_ = START;
	currMsgClass = 0;
	currMsgID = 0;
	currMsgLength = 0;
	checksumReferenceA = 0;
	checksumReferenceB = 0;
	prev_byte_ = 0;
	start_message_ = false;
	end_message_ = false;
}

bool UbloxGPS::parsing_message()
{
	return (start_message_ == true && end_message_ == false);
}

size_t UbloxGPS::num_messages_received()
{
	return num_messages_received_;
}

bool UbloxGPS::get_version()
{
	got_ver_ = false;
	send_message(CLASS_MON, MON_VER, out_message_, 0);
	auto start = esp_timer_get_time();
	int dt_ms = 0;
	while (!got_ver_ && dt_ms < TIMEOUT_MS) {
		dt_ms = esp_timer_get_time() - start;
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	if (got_ver_) {
		ESP_LOGI(TAG, "Got version\n");
		return true;
	} else {
		ESP_LOGI(TAG, "Did not get version\n");
		return false;
	}
}

bool UbloxGPS::wait_for_response()
{
	got_ack_ = got_nack_ = false;
	auto start = esp_timer_get_time();
	int dt_ms = 0;
	while (!got_ack_ && !got_nack_  && dt_ms < TIMEOUT_MS) {
		dt_ms = esp_timer_get_time() - start;
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	if (got_ack_) {
		ESP_LOGI(TAG, "Got Response\n");
		return true;
	} else if (got_nack_ || dt_ms >= TIMEOUT_MS) {
		ESP_LOGI(TAG, "No Response\n");
		return false;
	}
	return false;
}

bool UbloxGPS::send_message(uint8_t msg_class, uint8_t msg_id, UBX_message_t& message, uint16_t len)
{
	// First, calculate the checksum
	uint16_t checksum = calculateChecksum(msg_class, msg_id, len, message);

	uint8_t checksumA = checksum >> 8;
	uint8_t checksumB = checksum & 0xFF;

	// Send message
	write(START_BYTE_1);
	write(START_BYTE_2);
	write(msg_class);
	write(msg_id);
	write(len & 0xFF);
	write((len >> 8) & 0xFF);
	write(message.buffer, len);
	write(checksumA);
	write(checksumB);
	return true;
}

void UbloxGPS::start_survey_in()
{
	out_message_.CFG_TMODE3.flags = CFG_TMODE3_t::SURVEY_IN;
	out_message_.CFG_TMODE3.svinMinDur = 60;	// Wait at least 1 minute
	out_message_.CFG_TMODE3.svinAccLimit = 3; // At least within 3 centimeters
	send_message(CLASS_CFG, CFG_TMODE3, out_message_, sizeof(CFG_TMODE3_t));
}

void UbloxGPS::disableNMEA()
{
	ESP_LOGI(TAG, "Disabling NMEA messages ");
	// if (major_version_ <= 23) {
	// 	ESP_LOGI(TAG, "with old protocol\n");
	// 	using CF = CFG_PRT_t;
	// 	memset(&out_message_, 0, sizeof(CF));
	// 	out_message_.CFG_PRT.portID = CF::PORT_USB | CF::PORT_UART1;
	// 	out_message_.CFG_PRT.baudrate = 921600;
	// 	out_message_.CFG_PRT.outProtoMask = CF::OUT_UBX | CF::OUT_RTCM3;
	// 	out_message_.CFG_PRT.inProtoMask = CF::OUT_UBX | CF::OUT_RTCM3;
	// 	out_message_.CFG_PRT.flags = CF::CHARLEN_8BIT | CF::PARITY_NONE | CF::STOP_BITS_1;
	// 	send_message(CLASS_CFG, CFG_PRT, out_message_, sizeof(CF));
	// } else {
		ESP_LOGI(TAG, "with new protocol\n");
		using CV = CFG_VALSET_t;
		configure(CV::VERSION_0, CV::RAM, 0, CV::USB_INPROT_NMEA, 1);
		configure(CV::VERSION_0, CV::RAM, 0, CV::USB_OUTPROT_NMEA, 1);
	// }
}

void UbloxGPS::set_dynamic_mode()
{
	ESP_LOGI(TAG, "Setting dynamic mode ");
	// if (major_version_ <= 23) {
	// 	ESP_LOGI(TAG, "with old protocol\n");
	// 	memset(&out_message_, 0, sizeof(CFG_NAV5_t));
	// 	out_message_.CFG_NAV5.mask = CFG_NAV5_t::MASK_DYN;
	// 	out_message_.CFG_NAV5.dynModel = CFG_NAV5_t::DYNMODE_AIRBORNE_4G;
	// 	send_message(CLASS_CFG, CFG_NAV5, out_message_, sizeof(CFG_NAV5_t));
	// } else {
		ESP_LOGI(TAG, "with new protocol\n");
		using CV = CFG_VALSET_t;
		configure(CV::VERSION_0, CV::RAM, CV::DYNMODE_AIRBORNE_1G, CV::DYNMODEL, 1);
	// }
}

void UbloxGPS::set_nav_rate(uint8_t period_ms)
{
	ESP_LOGI(TAG, "Setting nav rate to %d\n", period_ms);
	// if (major_version_ <= 23) {
	// 	ESP_LOGI(TAG, "Using old protocol\n");
	// 	memset(&out_message_, 0, sizeof(CFG_RATE_t));
	// 	out_message_.CFG_RATE.measRate = period_ms;
	// 	out_message_.CFG_RATE.navRate = 1;
	// 	out_message_.CFG_RATE.timeRef = CFG_RATE_t::TIME_REF_UTC;
	// 	send_message(CLASS_CFG, CFG_RATE, out_message_, sizeof(CFG_RATE_t));
	// } else {
		ESP_LOGI(TAG, "Using new protocol\n");
		using CV = CFG_VALSET_t;
		configure(CV::VERSION_0, CV::RAM, period_ms, CV::RATE_MEAS, 1);
		configure(CV::VERSION_0, CV::RAM, 1, CV::RATE_NAV, 1);
		configure(CV::VERSION_0, CV::RAM, CV::TIME_REF_UTC, CV::RATE_TIMEREF, 1);
	// }
}
// void UbloxGPS::set_nav_rate(uint8_t period_ms) {

//   ESP_LOGI( TAG, "Setting nav rate to %d\n", period_ms);

//   configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, period_ms,
//             CFG_VALSET_t::RATE_MEAS, byte);
//   configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1,
//             CFG_VALSET_t::RATE_NAV, byte);
//   configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM,
//             CFG_VALSET_t::TIME_REF_UTC, CFG_VALSET_t::RATE_TIMEREF, byte);
// }

void UbloxGPS::enable_message(uint8_t msg_cls, uint8_t msg_id, uint8_t rate)
{
	ESP_LOGI(TAG, "Requesting %x:%x message with period=%d ", msg_cls, msg_id, rate);
	// if (major_version_ <= 23)
	ESP_LOGI(TAG, "Using old protocol\n");
	memset(&out_message_, 0, sizeof(CFG_MSG_t));
	out_message_.CFG_MSG.msgClass = msg_cls;
	out_message_.CFG_MSG.msgID = msg_id;
	out_message_.CFG_MSG.rate = rate;
	send_message(CLASS_CFG, CFG_MSG, out_message_, sizeof(CFG_MSG_t));
	// }
	// else
	// {
	//     ESP_LOGI( TAG, "Using new protocol\n");
	//     using CV = CFG_VALSET_t;
	//     configure(CV::VERSION_0, CV::RAM, 1, CV::MSGOUT_PVT, 1);
	// }
}

bool UbloxGPS::processNewByte(uint8_t byte)
{
	// ESP_LOGI(TAG, "processNewByte: %02x\n", byte);
	switch (parse_state_) {
	case START:
		// printf("%02X%02X,", prev_byte_, byte);
		if (byte == START_BYTE_2 && prev_byte_ == START_BYTE_1) {
#ifdef CONFIG_GPS_DEBUG
			ESP_LOGI(TAG, "Got start byte\n");
#endif
			buffer_head_ = 0;
			parse_state_ = GOT_START_FRAME;
			currMsgClass = 0;
			currMsgID = 0;
			currMsgLength = 0;
			checksumReferenceA = 0;
			checksumReferenceB = 0;
			start_message_ = true;
			end_message_ = false;
		}
		break;
	case GOT_START_FRAME:
#ifdef CONFIG_GPS_DEBUG
		ESP_LOGI(TAG, "Setting class byte %02X", byte);
#endif
		currMsgClass = byte;
		parse_state_ = GOT_CLASS;
		break;
	case GOT_CLASS:
#ifdef CONFIG_GPS_DEBUG
		ESP_LOGI(TAG, "Setting id byte %02X", byte);
#endif
		currMsgID = byte;
		parse_state_ = GOT_MSG_ID;
		break;
	case GOT_MSG_ID:
#ifdef CONFIG_GPS_DEBUG
		ESP_LOGI(TAG, "Setting length 1 %02X", byte);
#endif
		currMsgLength = byte;
		parse_state_ = GOT_LENGTH1;
		break;
	case GOT_LENGTH1:
#ifdef CONFIG_GPS_DEBUG
		ESP_LOGI(TAG, "Setting length 2 %02X", byte);
#endif
		currMsgLength |= (uint16_t)byte << 8;
		parse_state_ = GOT_LENGTH2;
		if (currMsgLength > BUFFER_SIZE) {
			ESP_LOGI(TAG, "Message 0x%x-0x%x is too long (%d > %d)\n", currMsgClass, currMsgID,
					currMsgLength, BUFFER_SIZE);
			num_errors_++;
			prev_byte_ = byte;
			restart();
			return false;
		}
		break;
	case GOT_LENGTH2:

		if (buffer_head_ < currMsgLength) {
			// push the byte onto the data buffer
			incomingMessage.buffer[buffer_head_] = byte;
			if (buffer_head_ == currMsgLength - 1) {
				parse_state_ = GOT_PAYLOAD;
			}
			buffer_head_++;
		}
		break;
	case GOT_PAYLOAD:
		checksumReferenceA = byte;
		parse_state_ = GOT_CK_A;
		break;
	case GOT_CK_A:
		checksumReferenceB = byte;
		parse_state_ = GOT_CK_B;
		break;
	default:
		num_errors_++;
		parse_state_ = START;
		end_message_ = false;
		start_message_ = false;
		break;
	}

	// If we have a complete packet, then try to parse it
	if (parse_state_ == GOT_CK_B) {
		ESP_LOGI(TAG, "Got checksum bytes %02X %02X", checksumReferenceA, checksumReferenceB);
		if (decodeMessage()) {
			ESP_LOGI(TAG, "Finished message, restating parser");
			parse_state_ = START;
			end_message_ = true;
			start_message_ = false;
			prev_byte_ = byte;
			return true;
		} else {
			ESP_LOGI(TAG, "Failed to decode message");
			// indicate error if it didn't work
			num_errors_++;
			parse_state_ = START;
			start_message_ = false;
			end_message_ = false;
		}
	}
	prev_byte_ = byte;
	return false;
}

void UbloxGPS::restart()
{
	parse_state_ = START;
	end_message_ = false;
	start_message_ = false;
}

bool UbloxGPS::decodeMessage() {
	// ESP_LOGI(TAG, "decodeMessage | class:%d | type:%d | length:%d", currMsgClass, currMsgID, currMsgLength);
	// return true;
	// First, check the checksum
	uint8_t checksumA = 0;
	uint8_t checksumB = 0;

	// Add in class
	checksumA += currMsgClass;
	checksumB += checksumA;

	// Id
	checksumA += currMsgID;
	checksumB += checksumA;

	// Length
	checksumA += currMsgLength & 0xFF;
	checksumB += checksumA;
	checksumA += (currMsgLength >> 8) & 0xFF;
	checksumB += checksumA;

	// Payload
	for (int i = 0; i < currMsgLength; i++) {
		checksumA += incomingMessage.buffer[i];
		checksumB += checksumA;
	}


	// uint16_t checksum = calculateChecksum(currMsgClass, currMsgID, currMsgLength, incomingMessage); // NOTE crashes here
	// uint8_t checksumA = checksum >> 8;
	// uint8_t checksumB = checksum & 0xFF;

	ESP_LOGI(TAG, "Checksum calculated: %02X-%02X vs %02X-%02X", checksumA, checksumB, checksumReferenceA, checksumReferenceB);
	if (checksumA != checksumReferenceA || checksumB != checksumReferenceB)
		return false;

	ESP_LOGI(TAG, "Checksum OK\n");

	uint8_t version; // 0 poll request, 1 poll (receiver to return config data key
									 // and value pairs)
	uint8_t layer;
	uint8_t reserved1[2];
	uint32_t cfgDataKey;
	uint64_t cfgData;
	num_messages_received_++;

	// Parse the payload
	switch (currMsgClass) {
	case CLASS_ACK:
		switch (currMsgID) {
		case ACK_ACK:
			got_ack_ = true;
			ESP_LOGI(TAG, "decodeMessage | class:ACK | type:ACK");
			break;
		case ACK_NACK:
			got_nack_ = true;
			ESP_LOGI(TAG, "decodeMessage | class:ACK | type:NACK");
			break;
		default:
			ESP_LOGI(TAG, "decodeMessage | class:ACK | type:%d", currMsgID);
			break;
		}
		break;
	case CLASS_MON:
		switch (currMsgID) {
		case MON_VER:
			ESP_LOGI(TAG, "decodeMessage | class:MON | type:VER");
			version_cb();
			break;
		case MON_COMMS:
			ESP_LOGI(TAG, "decodeMessage | class:MON | type:COMMS");
			break;
		case MON_TXBUF:
			ESP_LOGI(TAG, "decodeMessage | class:MON | type:TXMON_TXBUF");
			break;
		}
		break;
	case CLASS_RXM:
		switch (currMsgID) {
		case RXM_RAWX:
			ESP_LOGI(TAG, "decodeMessage | class:RXM | type:RAWX");
			break;
		case RXM_SFRBX:
			ESP_LOGI(TAG, "decodeMessage | class:RXM | type:SFRBX");
			break;
		}
		break;
	case CLASS_NAV:
		switch (currMsgID) {
		case NAV_PVT:
			ESP_LOGI(TAG, "decodeMessage | class:NAV | type:PVT");
			break;
		case NAV_RELPOSNED:
			ESP_LOGI(TAG, "decodeMessage | class:NAV | type:RELPOSNED");
			break;
		case NAV_POSECEF:
			ESP_LOGI(TAG, "decodeMessage | class:NAV | type:POSECEF");
			break;
		case NAV_VELECEF:
			ESP_LOGI(TAG, "decodeMessage | class:NAV | type:VELECEF");
			break;
		default:
			ESP_LOGI(TAG, "decodeMessage | class:NAV | type:%d", currMsgID);
		}
		break;
	case CLASS_CFG: // only needed for getting data
		ESP_LOGI(TAG, "CFG_");
		switch (currMsgID) {
		case CFG_VALGET: {
			ESP_LOGI(TAG, "decodeMessage | class:CFG | type:VALGET=%lld", incomingMessage.CFG_VALGET.cfgData);
			break;
		}
		default:
			ESP_LOGI(TAG, "decodeMessage | class:CFG | type:%x\n", currMsgID);
		}
		break;

	default:
		//        ESP_LOGI( TAG, "Unknown (%d-%d)\n", currMsgClass, currMsgID);
		break;
	}

	// call callbacks TODO
	// for (auto &l : listeners_)
	//    {
	//        if (l->subscribed(currMsgClass, currMsgID))
	//            l->got_ubx(currMsgClass, currMsgID, incomingMessage);
	//    }

	if (currMsgClass == ublox::CLASS_NAV && currMsgID == ublox::NAV_RELPOSNED) {
		ublox::NAV_RELPOSNED_t msg = incomingMessage.NAV_RELPOSNED;
		int RTK_flag;
		if (msg.flags && 0b000000010) {
			printf("RTK");
			RTK_flag = 1;
		} else {
			printf("No RTK \n");
			RTK_flag = 0;
		}
		if (RTK_flag == 1) {
			if (msg.flags && 0b000100000)
				printf(", Moving Base");
			if (msg.flags && 0b000001000)
				printf(" , Floating \n");
			else if (msg.flags && 0b000010000)
				printf(" , Fixed \n");
			if (msg.flags && 0b000000100)
				printf("valid relative position components \n");
			printf("tow: %ld relNED: %ld, %ld, %ld, Distance: %ld\n", msg.iTow / 1000, msg.relPosN,
					msg.relPosE, msg.relPosD, msg.relPosLength);
		}
	}

	return true;
}

uint16_t UbloxGPS::calculateChecksum(const uint8_t msg_cls, const uint8_t msg_id, const uint16_t len, const UBX_message_t payload) {
	uint8_t checksumA = 0;
	uint8_t checksumB = 0;

	// Add in class
	checksumA += msg_cls;
	checksumB += checksumA;

	// Id
	checksumA += msg_id;
	checksumB += checksumA;

	// Length
	checksumA += len & 0xFF;
	checksumB += checksumA;
	checksumA += (len >> 8) & 0xFF;
	checksumB += checksumA;

	// Payload
	for (int i = 0; i < len; i++) {
		checksumA += payload.buffer[i];
		checksumB += checksumA;
	}
	return ((uint16_t) checksumA << 8) | checksumB;
}

void UbloxGPS::configure(uint8_t version,
		uint8_t layer,
		uint64_t cfgData,
		uint32_t cfgDataKey,
		uint8_t size)
{
	memset(&out_message_, 0, sizeof(CFG_VALSET_t));
	out_message_.CFG_VALSET.version = version;
	out_message_.CFG_VALSET.layer = layer;
	if (size == 1) {
		out_message_.CFG_VALSET.cfgData.bytes[0] = cfgData;
	}
	if (size == 2) {
		out_message_.CFG_VALSET.cfgData.word = cfgData;
	}
	out_message_.CFG_VALSET.cfgDataKey = cfgDataKey;
	send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
}

void UbloxGPS::get_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey)
{
	memset(&out_message_, 0, sizeof(CFG_VALGET_t));
	out_message_.CFG_VALGET.version = version;
	out_message_.CFG_VALGET.layer = layer;
	out_message_.CFG_VALGET.cfgDataKey = cfgDataKey;
	send_message(CLASS_CFG, CFG_VALGET, out_message_, sizeof(CFG_VALGET_t));
}

void UbloxGPS::write(const uint8_t byte)
{
	uart_write_bytes(EX_UART_NUM, &byte, 1);
}
void UbloxGPS::write(const uint8_t* byte, const size_t size)
{
	uart_write_bytes(EX_UART_NUM, byte, size);
}

void UbloxGPS::version_cb()
{
	int protocol_version;
	std::string module_type;
	int firmware_version;
	for (int i = 0; i < 20; ++i) {
		if (strncmp(incomingMessage.MON_VER.extension[i], "PROTVER=", 8) == 0) {
			extract_version_string(incomingMessage.MON_VER.extension[i]);
		}
		if (strncmp(incomingMessage.MON_VER.extension[i], "MOD=", 4) == 0) {
			extract_module_name(incomingMessage.MON_VER.extension[i]);
		}
	}

	ESP_LOGI(TAG, "Connected:\n\tModule Type: %s\n\tFirmware Version: %d.%d\n\tProtocol Version: %d.%d\n",
			module_name_, major_version_, minor_version_, major_version_, minor_version_);
	got_ver_ = true;
}

void UbloxGPS::extract_version_string(const char* str)
{
	// Get major version
	char tmp[5];
	const char* s = str + 8;
	for (int j = 0; j < 5; j++) {
		if (*s == '.') {
			tmp[j] = 0;
			++s;
			break;
		}
		tmp[j] = *s;
		++s;
	}
	major_version_ = atoi(tmp);

	// Get minor version
	memset(tmp, 0, sizeof(tmp));
	for (int j = 0; j < 5; j++) {
		if (*s == 0) {
			tmp[j] = 0;
			break;
		}
		tmp[j] = *s;
		++s;
	}
	minor_version_ = atoi(tmp);
}

void UbloxGPS::extract_module_name(const char* str)
{
	const char* s = str + 4;
	for (int i = 0; i < sizeof(module_name_); i++) {
		if (*s == 0) {
			module_name_[i] = 0;
			break;
		}

		module_name_[i] = *s;
		++s;
	}
}

void UbloxGPS::uartEvent(void* pvParameters)
{
	UbloxGPS* gps = static_cast<UbloxGPS*>(pvParameters);
	auto startIndex = [](const char* str) { // skips first 4 characters and all spaces and tabs
		int i = 4;
		while (str[i] == ' ' || str[i] == '\t')
			i++;
		return i;
	};

	static uart_event_t event;
	static uint8_t* dtmp = (uint8_t*)malloc(SERIAL_BUFFER_SIZE);

	int32_t i = 0;
	int32_t j = 0;
	int32_t j_prev = 0;
	int32_t pointer = 0;
	ESP_LOGI(TAG, "uartEvent | uartEvent started");
	while (1) {
		if (xQueueReceive(uart0_queue, (void*)&event, (TickType_t)portMAX_DELAY)) {
			memset(dtmp, 0, SERIAL_BUFFER_SIZE);
			if (event.type == UART_DATA) {
				printf("\n");
				uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
				ESP_LOGI(TAG, "uartEvent | uartEvent received %d bytes", event.size);
				for (j = 0; j < event.size; j++) {
					gps->processNewByte(dtmp[j]);
				}
			}
		}
	}
	free(dtmp);
	dtmp = NULL;
	vTaskDelete(NULL);
}

void UbloxGPS::setupSerial()
{
	ESP_LOGI(TAG, "setupSerial | Setting up serial");
	uart_config_t uart_config = {
		.baud_rate = 38400,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	// Install UART driver, and get the queue.
	uart_driver_install(EX_UART_NUM, BUFFER_SIZE << 1, BUFFER_SIZE << 1, 20, &uart0_queue, 0);
	uart_param_config(EX_UART_NUM, &uart_config);
	uart_set_pin(EX_UART_NUM, CONFIG_GPS_TX_PIN, CONFIG_GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	// Create a task to handler UART event from ISR
	xTaskCreate(uartEvent, "uartEvent", 2048, this, 12, NULL);
}

} // namespace ublox
