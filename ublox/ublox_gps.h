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

#ifndef UBLOX_GPS_H
#define UBLOX_GPS_H

#include <cstdint>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <functional>
#include <iostream>
#include <esp_timer.h>
#include <sdkconfig.h>
#include <set>
#include <stdio.h>
#include <string>
#include <vector>

#define INPUT_REG_SIZE 128
#define HOLDING_REG_SIZE 128
#define SERIAL_BUFFER_SIZE 256
#define EX_UART_NUM (uart_port_t) CONFIG_GPS_UART_NUM

#define LENGTH_TO_COPY(x, y) (x + y >= BUFFER_SIZE ? BUFFER_SIZE - x : y)

#include "ublox_definitions.h"
// #include "serial_interface.h"
constexpr static char TAG[] = "OSCore";

namespace ublox {

class UbloxGPS {
	static constexpr int TIMEOUT_MS = 1000;
	constexpr static char TAG[] = "UbloxGPS";

	private:
	static void uartEvent(void* pvParameters);
	static QueueHandle_t uart0_queue;

	uint8_t prev_byte_;
	uint16_t buffer_head_ = 0;
	bool start_message_ = false;
	bool end_message_ = false;
	bool got_ack_ = false;
	bool got_ver_ = false;
	bool got_nack_ = false;
	parse_state_t parse_state_;
	uint8_t currMsgClass;
	uint8_t currMsgID;
	uint16_t currMsgLength;
	uint8_t checksumReferenceA;
	uint8_t checksumReferenceB;
	uint32_t num_errors_ = 0;
	uint32_t num_messages_received_ = 0;
	uint8_t version; // 0 poll request, 1 poll (receiver to return config data key
									 // and value pairs)
	uint8_t layer;
	uint32_t cfgDataKey;
	uint64_t cfgData;
	uint8_t size;
	// uint8_t byte = 1;
	// uint8_t word = 2;
	// local storage
	volatile bool new_data_;

	int major_version_;
	int minor_version_;
	char module_name_[10];

	// Main buffers for communication
	UBX_message_t out_message_;
	UBX_message_t incomingMessage;

	/*
	 * @brief revert message parsing state to default
	 */
	void restart(); // DONE

	public:
	UbloxGPS();


	// This function returns true when a new message has been parsed
	/**
	 * @brief processes each new byte read from the serial port
	 *
	 * @param byte the byte to process
	 * @return true if a byte was processed completely
	 */
	bool processNewByte(uint8_t byte); // DONE

	// low-level parsing functions
	/**
	 * @brief check if the message is valid, notify observers
	 *
	 * @return true if the message is valid
	 */
	bool decodeMessage();

	/**
	 * @brief calculate the checksum for a message
	 *
	 * @param msg_cls message class
	 * @param msg_id message type
	 * @param len length of the message
	 * @param payload the message payload
	 * @param checksumA checksum byte a
	 * @param checksumB checksum byte b
	 */
	uint16_t calculateChecksum(const uint8_t msg_cls, const uint8_t msg_id, const uint16_t len, const UBX_message_t payload);

  /**
	 * @brief configure serial connection to the module
	 */
	void setupSerial();

	void configure(uint8_t version,
			uint8_t layer,
			uint64_t cfgData,
			uint32_t cfgDataKey,
			uint8_t size);

	void get_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey);

	void set_dynamic_mode();

	void enable_message(uint8_t msg_cls, uint8_t msg_id, uint8_t rate);

	void config_rover();

	void config_base();

	void config_base_stationary();

	void config_base_mobile();

	void poll_value();

	void disableNMEA();


	void start_survey_in();

	bool get_version();

	bool wait_for_response();

	void version_cb();

	// listener handling
	// void registerListener(UBXListener* listener);
	//
	// std::vector<UBXListener*> listeners_;

	bool parsing_message();

	size_t num_messages_received();

	void set_nav_rate(uint8_t period_ms);

	// Send the supplied message
	bool send_message(uint8_t msg_class, uint8_t msg_id, UBX_message_t& message, uint16_t len);




	void extract_version_string(const char* str);

	void extract_module_name(const char* str);

	// Parsing State Working Memory

	int major_version() const { return major_version_; }

	int minor_version() const { return minor_version_; }

	const char* module_name() const { return module_name_; }

	// Serial Port
	void write(const uint8_t byte);											// TODO replace

	void write(const uint8_t* byte, const size_t size); // TODO replace
};

} // namespace ublox
#endif // UBLOX_GPS_H
