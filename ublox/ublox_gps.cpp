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

bool UbloxGPS::addObserver(UbloxEventObserver* v, uint8_t msgClass, uint8_t msgID)
{
	std::list<std::pair<UbloxEventObserver*, uint16_t>>::iterator it = observers.begin();
	uint16_t key = ((uint16_t)msgClass) << 8 | msgID;
	while (it != observers.end()) {
		if (it->first == v && it->second == key) {
			return false;
		}
		++it;
	}
	observers.push_back(std::pair<UbloxEventObserver*, uint16_t>(v, key));
	return true;
}

bool UbloxGPS::removeObserver(UbloxEventObserver* v, uint8_t msgClass, uint8_t msgID)
{
	std::list<std::pair<UbloxEventObserver*, uint16_t>>::iterator it = observers.begin();
	uint16_t key = ((uint16_t)msgClass) << 8 | msgID;
	while (it != observers.end()) {
		if (it->first == v && it->second == key) {
			it = observers.erase(it);
			return true;
		}
		++it;
	}
	return false;
}

void UbloxGPS::notifyObservers(const uint8_t msgClass, const uint8_t msgID, const UBX_message_t* message, const uint16_t msgLen)
{
	std::list<std::pair<UbloxEventObserver*, uint16_t>>::iterator it = observers.begin();
	uint16_t key = ((uint16_t)msgClass) << 8 | msgID;
	while (it != observers.end()) {
		if (it->second == key)
			it->first->onUbloxEvent(msgClass, msgID, message, msgLen);
		++it;
	}
}

bool UbloxGPS::requestVersion()
{
	got_ver_ = false;
	sendMessage(CLASS_MON, MON_VER, out_message_, 0);
	auto start = (esp_timer_get_time() >> 10);
	int dt_ms = 0;
	while (!got_ver_ && dt_ms < TIMEOUT_MS) {
		dt_ms = (esp_timer_get_time() >> 10) - start;
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	if (got_ver_) {
		ESP_LOGI(TAG, "Got version");
		return true;
	} else {
		ESP_LOGI(TAG, "Did not get version");
		return false;
	}
}

bool UbloxGPS::waitForResponse()
{
	got_ack_ = got_nack_ = false;
	auto start = (esp_timer_get_time() >> 10);
	int dt_ms = 0;
	while (!got_ack_ && !got_nack_ && dt_ms < TIMEOUT_MS) {
		dt_ms = (esp_timer_get_time() >> 10) - start;
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	if (got_ack_) {
		ESP_LOGI(TAG, "Got Response");
		return true;
	} else if (got_nack_ || dt_ms >= TIMEOUT_MS) {
		ESP_LOGI(TAG, "No Response");
		return false;
	}
	return false;
}

bool UbloxGPS::sendMessage(uint8_t msgClass, uint8_t msgID, UBX_message_t& message, uint16_t len)
{
	// First, calculate the checksum
	uint16_t checksum = calculateChecksum(msgClass, msgID, len, message);

	uint8_t checksumA = checksum >> 8;
	uint8_t checksumB = checksum & 0xFF;

	// Send message
	uint8_t byte = START_BYTE_1;
	uart_write_bytes(EX_UART_NUM, &byte, 1);
	byte = START_BYTE_2;
	uart_write_bytes(EX_UART_NUM, &byte, 1);
	uart_write_bytes(EX_UART_NUM, &msgClass, 1);
	uart_write_bytes(EX_UART_NUM, &msgID, 1);
	uart_write_bytes(EX_UART_NUM, &len, 2); // send length as two bytes
	uart_write_bytes(EX_UART_NUM, message.buffer, len);
	uart_write_bytes(EX_UART_NUM, &checksumA, 1);
	uart_write_bytes(EX_UART_NUM, &checksumB, 1);
	return true;
}

void UbloxGPS::disableNMEA()
{
	ESP_LOGI(TAG, "Disabling NMEA messages");
	if (major_version_ <= 23) {
		ESP_LOGI(TAG, "with old protocol");
		using CF = CFG_PRT_t;
		memset(&out_message_, 0, sizeof(CF));
		out_message_.CFG_PRT.portID = CF::PORT_USB | CF::PORT_UART1;
		out_message_.CFG_PRT.baudrate = 921600;
		out_message_.CFG_PRT.outProtoMask = CF::OUT_UBX | CF::OUT_RTCM3;
		out_message_.CFG_PRT.inProtoMask = CF::OUT_UBX | CF::OUT_RTCM3;
		out_message_.CFG_PRT.flags = CF::CHARLEN_8BIT | CF::PARITY_NONE | CF::STOP_BITS_1;
		sendMessage(CLASS_CFG, CFG_PRT, out_message_, sizeof(CF));
	} else {
		ESP_LOGI(TAG, "with new protocol");
		using CV = CFG_VALSET_t;
		configure(CV::VERSION_0, CV::RAM, 0, CV::USB_INPROT_NMEA, 1);
		configure(CV::VERSION_0, CV::RAM, 0, CV::USB_OUTPROT_NMEA, 1);
	}
}

bool UbloxGPS::setDynamicMode(uint8_t mode)
{
	ESP_LOGI(TAG, "Setting dynamic mode");
	if (major_version_ <= 23) {
		ESP_LOGI(TAG, "with old protocol");
		if (mode > CFG_NAV5_t::DYNMODE_AIRBORNE_4G) {
			ESP_LOGE(TAG, "Invalid dynamic mode %d", mode);
			return false;
		}
		memset(&out_message_, 0, sizeof(CFG_NAV5_t));
		out_message_.CFG_NAV5.mask = CFG_NAV5_t::MASK_DYN;
		out_message_.CFG_NAV5.dynModel = mode;
		sendMessage(CLASS_CFG, CFG_NAV5, out_message_, sizeof(CFG_NAV5_t));
	} else {
		ESP_LOGI(TAG, "with new protocol");
		using CV = CFG_VALSET_t;
		configure(CV::VERSION_0, CV::RAM, mode, CV::DYNMODEL, 1);
	}
	return true;
}


void UbloxGPS::setNavRate(uint16_t period)
{
	ESP_LOGI(TAG, "Setting nav rate to %d", period);
	if (true) {
	// if (major_version_ <= 23) {
		ESP_LOGI(TAG, "Using old protocol");
		memset(&out_message_, 0, sizeof(CFG_RATE_t));
		out_message_.CFG_RATE.measRate = period;
		out_message_.CFG_RATE.navRate = 1;
		out_message_.CFG_RATE.timeRef = CFG_RATE_t::TIME_REF_UTC;
		sendMessage(CLASS_CFG, CFG_RATE, out_message_, sizeof(CFG_RATE_t));
	} else {
		ESP_LOGI(TAG, "Using new protocol");
		using CV = CFG_VALSET_t;
		configure(CV::VERSION_0, CV::RAM, period, CV::RATE_MEAS, 2);
		configure(CV::VERSION_0, CV::RAM, 1, CV::RATE_NAV, 1);
		configure(CV::VERSION_0, CV::RAM, CV::TIME_REF_UTC, CV::RATE_TIMEREF, 1);
	}
}

bool UbloxGPS::setMessageRate(uint8_t msgClass, uint8_t msgID, uint8_t rate)
{
	memset(&out_message_, 0, sizeof(CFG_MSG_t));
	out_message_.CFG_MSG.msgClass = msgClass;
	out_message_.CFG_MSG.msgID = msgID;
	out_message_.CFG_MSG.rate = rate;
	sendMessage(CLASS_CFG, CFG_MSG, out_message_, sizeof(CFG_MSG_t));
	return true;
}

bool UbloxGPS::toggleGNSS(uint32_t system, bool toggle){
	if (major_version_ <= 23)
		return false;

	using CV = CFG_VALSET_t;
	configure(CV::VERSION_0, CV::RAM, toggle, system, 1);
	return true;
}

bool UbloxGPS::toggleIMU(bool toggle){
	if (major_version_ <= 23)
		return false;
	// Enable IMU messages
	using CV = CFG_VALSET_t;
	ESP_LOGI(TAG, "Enabling IMU messages");
	configure(CV::VERSION_0, CV::RAM, 1, 0x1006001d, 1);
	return true;
}

bool UbloxGPS::setMessageRate(uint32_t cfgDataKey, uint8_t rate)
{
	if (major_version_ <= 23)
		return false;
	using CV = CFG_VALSET_t;
	configure(CV::VERSION_0, CV::RAM, rate, cfgDataKey, 1);
	return true;
}

bool UbloxGPS::processNewByte(uint8_t byte)
{
	// ESP_LOGI(TAG, "processNewByte: %02x\n", byte);
	switch (parse_state_) {
	case START:
		if (byte == START_BYTE_2 && prev_byte_ == START_BYTE_1) {
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
		currMsgClass = byte;
		parse_state_ = GOT_CLASS;
		break;
	case GOT_CLASS:
		currMsgID = byte;
		parse_state_ = GOT_MSG_ID;
		break;
	case GOT_MSG_ID:
		currMsgLength = byte;
		parse_state_ = GOT_LENGTH1;
		break;
	case GOT_LENGTH1:
		currMsgLength |= (uint16_t)byte << 8;
		parse_state_ = GOT_LENGTH2;
		if (currMsgLength > BUFFER_SIZE) {
			ESP_LOGE(TAG, "Message 0x%x-0x%x is too long (%d > %d)", currMsgClass, currMsgID,
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
		if (decodeMessage()) {
			parse_state_ = START;
			end_message_ = true;
			start_message_ = false;
			prev_byte_ = byte;
			ESP_LOGI(TAG, "Finished message, restarted parser");
			return true;
		} else {
			// indicate error if it didn't work
			num_errors_++;
			parse_state_ = START;
			start_message_ = false;
			end_message_ = false;
			ESP_LOGI(TAG, "Failed to decode message");
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

bool UbloxGPS::decodeMessage()
{

	// NOTE: there's some RTOS fuckery that results in a crash if I call checksum function from here. No idea why since things like version callback work just fine.
	// uint16_t checksum = calculateChecksum(currMsgClass, currMsgID, currMsgLength, incomingMessage);
	// uint8_t checksumA = checksum >> 8;
	// uint8_t checksumB = checksum & 0xFF;

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

	if (checksumA != checksumReferenceA || checksumB != checksumReferenceB)
		return false;

	// ESP_LOGI(TAG, "decodeMessage | processing message with length=%d", currMsgLength);

	uint8_t version; // 0 poll request, 1 poll (receiver to return config data key
									 // and value pairs)
	uint8_t layer;
	uint8_t reserved1[2];
	uint32_t cfgDataKey;
	uint64_t cfgData;
	num_messages_received_++;

	if(currMsgClass == CLASS_MON && currMsgID == MON_VER){
		ESP_LOGI(TAG, "decodeMessage | class:MON | type:VER");
		versionCallback();
	}


#ifdef CONFIG_GPS_DEBUG
	Parse the payload
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
			ESP_LOGI(TAG, "decodeMessage | class:CFG | type:%x", currMsgID);
		}
		break;

	case CLASS_ESF:
		switch (currMsgID) {
		case ESF_ALG:
			ESP_LOGI(TAG, "decodeMessage | class:ESF | type:ALG");
			break;
		case ESF_CAL:
			ESP_LOGI(TAG, "decodeMessage | class:ESF | type:CAL");
			break;
		case ESF_INS:
			ESP_LOGI(TAG, "decodeMessage | class:ESF | type:INS");
			break;
		case ESF_STATUS:
			ESP_LOGI(TAG, "decodeMessage | class:ESF | type:STATUS");
			break;
		case ESF_RAW:
			ESP_LOGI(TAG, "decodeMessage | class:ESF | type:RAW");
			break;
		case ESF_MEAS:
			ESP_LOGI(TAG, "decodeMessage | class:ESF | type:MEAS");
			break;
		default:
			ESP_LOGI(TAG, "decodeMessage | class:ESF | type:%d", currMsgID);
			break;
		}
		break;
	default:
		ESP_LOGE(TAG, "Unknown (%02X-%02X)", currMsgClass, currMsgID);
		break;
	}
#endif /* CONFIG_GPS_DEBUG */

	notifyObservers(currMsgClass, currMsgID, &incomingMessage, currMsgLength);

	return true;
}

uint16_t UbloxGPS::calculateChecksum(const uint8_t msgClass, const uint8_t msgID, const uint16_t len, const UBX_message_t payload)
{
	uint8_t checksumA = 0;
	uint8_t checksumB = 0;

	// Add in class
	checksumA += msgClass;
	checksumB += checksumA;

	// Id
	checksumA += msgID;
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
	return ((uint16_t)checksumA << 8) | checksumB;
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
		ESP_LOGI(TAG, "configure | cfgDataKey: %ld, cfgData: %d", cfgDataKey, out_message_.CFG_VALSET.cfgData.bytes[0]);
	}
	if (size == 2) {
		out_message_.CFG_VALSET.cfgData.word = cfgData;
		ESP_LOGI(TAG, "configure | cfgDataKey: %ld, cfgData: %ld", cfgDataKey, out_message_.CFG_VALSET.cfgData.word);
	}
	out_message_.CFG_VALSET.cfgDataKey = cfgDataKey;
	sendMessage(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
}

void UbloxGPS::requestConfiguration(uint8_t version, uint8_t layer, uint32_t cfgDataKey)
{
	memset(&out_message_, 0, sizeof(CFG_VALGET_t));
	out_message_.CFG_VALGET.version = version;
	out_message_.CFG_VALGET.layer = layer;
	out_message_.CFG_VALGET.cfgDataKey = cfgDataKey;
	sendMessage(CLASS_CFG, CFG_VALGET, out_message_, sizeof(CFG_VALGET_t));
}

void UbloxGPS::versionCallback()
{
	int protocol_version;
	std::string module_type;
	int firmware_version;
	for (int i = 0; i < 20; ++i) {
		if (strncmp(incomingMessage.MON_VER.extension[i], "PROTVER=", 8) == 0) {
			extractVersionString(incomingMessage.MON_VER.extension[i]);
		}
		if (strncmp(incomingMessage.MON_VER.extension[i], "MOD=", 4) == 0) {
			extractModuleName(incomingMessage.MON_VER.extension[i]);
		}
	}

	ESP_LOGI(TAG, "Connected:\n\tModule Type: %s\n\tFirmware Version: %d.%d\n\tProtocol Version: %d.%d",
			module_name_, major_version_, minor_version_, major_version_, minor_version_);
	got_ver_ = true;
}

void UbloxGPS::extractVersionString(const char* str)
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

void UbloxGPS::extractModuleName(const char* str)
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
				uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
				for (j = 0; j < event.size; j++)
					gps->processNewByte(dtmp[j]);
			}
		}
	}
	free(dtmp);
	dtmp = NULL;
	vTaskDelete(NULL);
}

void UbloxGPS::setupSerial()
{
	ESP_LOGI(TAG, "setupSerial | Setting up serial, using UART %d", EX_UART_NUM);

	uart_config_t uart_config = {
		.baud_rate = 38400,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

#if CONFIG_IDF_TARGET_ESP32C6
	if(EX_UART_NUM == LP_UART_NUM_0) {
		ESP_LOGI(TAG, "setupSerial | Using LP UART %d", EX_UART_NUM);
		uart_config.rx_flow_ctrl_thresh = 0;
		uart_config.source_clk = (uart_sclk_t) LP_UART_SCLK_LP_FAST; //
	}
#endif /* if CONFIG_IDF_TARGET_ESP32C6 */

	// Install UART driver, and get the queue.
	uart_driver_install(EX_UART_NUM, BUFFER_SIZE << 1, BUFFER_SIZE << 1, 20, &uart0_queue, 0);
	// ESP_LOGI(TAG, "setupSerial | UART driver installed");
	// vTaskDelay(100 / portTICK_PERIOD_MS);
	uart_param_config(EX_UART_NUM, &uart_config);
	// ESP_LOGI(TAG, "setupSerial | UART parameters configured");
	// vTaskDelay(100 / portTICK_PERIOD_MS);
	// uart_set_pin(EX_UART_NUM, CONFIG_GPS_TX_PIN, CONFIG_GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_set_pin(EX_UART_NUM, CONFIG_GPS_TX_PIN, CONFIG_GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	// ESP_LOGI(TAG, "setupSerial | UART pins set");
	// vTaskDelay(100 / portTICK_PERIOD_MS);

	// Create a task to handler UART event from ISR
	xTaskCreate(uartEvent, "uartEvent", 2048, this, 12, NULL);
}

} // namespace ublox
