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
#include <esp_timer.h>
#include <functional>
#include <iostream>
#include <sdkconfig.h>
#include <set>
#include <list>
#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>

#define INPUT_REG_SIZE 128
#define HOLDING_REG_SIZE 128
#define SERIAL_BUFFER_SIZE 256
#define EX_UART_NUM (uart_port_t) CONFIG_GPS_UART_NUM
// #define EX_UART_NUM (uart_port_t) LP_UART_NUM_0

#define LENGTH_TO_COPY(x, y) (x + y >= BUFFER_SIZE ? BUFFER_SIZE - x : y)

#include "ublox_definitions.h"
#include "ublox_event_observer.h"
constexpr static char TAG[] = "OSCore";

namespace ublox {

class UbloxGPS {
	static constexpr int TIMEOUT_MS = 2000;
	constexpr static char TAG[] = "UbloxGPS";

	private:
	static void uartEvent(void* pvParameters);
	static QueueHandle_t uart0_queue;
	std::list<std::pair<UbloxEventObserver*, uint16_t>> observers;

	uint8_t prev_byte_;
	uint16_t buffer_head_ = 0;
	bool start_message_ = false;
	bool end_message_ = false;
	bool got_ack_ = false;
	bool got_ver_ = false;
	bool got_backup_ = false;
	bool got_nack_ = false;
	parse_state_t parse_state_;
	uint8_t currMsgClass;
	uint8_t currMsgID;
	uint16_t currMsgLength;
	uint8_t checksumReferenceA;
	uint8_t internalUpdatePeriod = 200;
	uint8_t checksumReferenceB;
	uint32_t num_errors_ = 0;
	uint32_t num_messages_received_ = 0;
	uint8_t version; // 0 poll request, 1 poll (receiver to return config data key
									 // and value pairs)
	uint8_t layer;
	uint32_t cfgDataKey;
	uint64_t cfgData;
	// uint8_t size;
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

	/**
	 * @brief configure the module with the given parameters
	 *
	 * @param version message version
	 * @param layer layer of configuration (RAM, BBR, FLASH, DEFAULT)
	 * @param cfgData configuration data to set
	 * @param cfgDataKey key for the configuration data
	 * @param size size of the configuration data (1 byte or 2 bytes)
	 */
	void configure(uint8_t version,
			uint8_t layer,
			uint64_t cfgData,
			uint32_t cfgDataKey,
			uint8_t size);

	/**
	 * @brief callback for version message
	 *
	 * This function is called when a version message is received.
	 */
	void versionCallback();

	/**
	 * @brief extract version string from the given string, sets major_version_ and minor_version_
	 * called by versionCallback()
	 *
	 * @param str the string to extract version from
	 */
	void extractVersionString(const char* str);

	/**
	 * @brief extract module name from the given string, sets module_name_
	 * called by versionCallback()
	 *
	 * @param str the string to extract module name from
	 */
	void extractModuleName(const char* str);

	/**
	 * @brief processes each new byte read from the serial port
	 *
	 * @param byte the byte to process
	 * @return true if a byte was processed completely
	 */
	bool processNewByte(uint8_t byte);

	/**
	 * @brief check if the message is valid, notify observers
	 *
	 * @return true if the message is valid
	 */
	bool decodeMessage();

	// Send the supplied message
	bool sendMessage(uint8_t msgClass, uint8_t msgID, UBX_message_t& message, uint16_t len);

	void notifyObservers(const uint8_t msgClass, const uint8_t msgID, const UBX_message_t* message, const uint16_t msgLen);

	public:
	UbloxGPS();

	/**
	 * @brief calculate the checksum for a message
	 *
	 * @param msgClass message class
	 * @param msgID message type
	 * @param len length of the message
	 * @param payload the message payload
	 * @param checksumA checksum byte a
	 * @param checksumB checksum byte b
	 */
	uint16_t calculateChecksum(const uint8_t msgClass, const uint8_t msgID, const uint16_t len, const UBX_message_t payload);

	/**
	 * @brief configure serial connection to the module
	 */
	void setupSerial();

	/**
	 * @brief configure rate at which navigation solutions are generated by the receive
	 *
	 * the period also determines timing of all measurements/messages that depend on them.
	 *
	 * @param period the period in milliseconds
	 */
	void setNavRate(uint16_t period);

	/**
	 * @brief reqest a value for given key
	 * I guess only applicable for new protocol (version > 23)
	 *
	 * @param version message version
	 * @param layer layer of configuration (RAM, BBR, FLASH, DEFAULT)
	 * @param cfgDataKey key for the configuration data
	 */
	void requestConfiguration(uint8_t version, uint8_t layer, uint32_t cfgDataKey);

	/**
	 * @brief set the dynamic mode of the module
	 * this affects settings of internal kalman filters that affect as data are processed
	 * on older divce WRIST and BIKE modes aren't available
	 *
	 * @param dynamicMode the dynamic mode to set
	 * @return true if the dynamic mode was set successfully
	 */
	bool setDynamicMode(uint8_t dynamicMode);


	/**
	 * @brief request a reset of the module
	 * uses exclusively old protocol (version <= 23)
	 *
	 * @param maskBRR bitmask of the BRR to reset
	 *       0x0000 - Hot start (default)
	 *       0x0001 - Warm start
	 *       0xFFFF - Cold start
	 * @param resetMode the reset mode to use (e.g. RESET_MODE_HOT, RESET_MODE_WARM, RESET_MODE_COLD)
	 * @return true if the reset was requested successfully
	 */
	bool resetDevice(uint16_t maskBRR, uint8_t resetMode);

	/**
	 * @brief request a backup of the module
	 * uses exclusively old protocol (version <= 23)
	 * this function is blocking and will wait for a TIMEOUT_MS for a response
	 *
	 * @return true if the backup was requested successfully
	 */
	bool requestBackup();

	/**
	 * @brief enable a message with the given class and id
	 * uses exclusively old protocol (version <= 23)
	 *
	 * @param msgClass message class
	 * @param msgID message id
	 * @param rate number of periods (configured with setNavRate) between messages
	 * @return true
	 */
	bool setMessageRate(uint8_t msgClass, uint8_t msgID, uint8_t rate);


	/**
	 * @brief toggle onboard IMU  on and off
	 * uses new protocol (version > 23)
	 *
	 * @param toggle true to enable IMU
	 * @return true if device supports new protocols and message was sent
	 */
	bool toggleIMU(bool toggle);


	/**
	 * @brief toggle GNSS system on and off
	 * uses new protocol (version > 23)
	 *
	 * @param system the system to toggle (e.g. SIGNAL_GPS, SIGNAL_GAL, SIGNAL_BDS)
	 * @param toggle true to enable the system
	 * @return true if device supports new protocol and message was sent
	 */
	bool toggleGNSS(uint32_t system, bool toggle);

	/**
	 * @brief enable a message with the given class and id
	 * uses new protocol (version > 23)
	 *
	 * @param msgClass message class
	 * @param msgID message id
	 * @param rate number of periods (configured with setNavRate) between messages
	 * @return true if device supports new protocol  and message was sent
	 */
	bool setMessageRate(uint32_t cfgDataKey, uint8_t rate);

	/*
	 * @brief disable NMEA messages switch to UBX
	 */
	void disableNMEA();

	/*
	 * @brief get the current version of the module
	 *
	 * this function will wait for a TIMEOUT_MS for a message to be received.
	 * if message doesn't arrive, it will return false. But version still might
	 * be set eventually when response arrives.
	 *
	 * @return true if the version was successfully retrieved
	 */
	bool requestVersion();

	/**
	 * @brief request a database dump from the module
	 * this function is blocking and will wait for a TIMEOUT_MS for a response
	 *
	 * @return true if the request was sent successfully
	 */
	bool requestDatabaseDump();

	/**
	 * @brief upload a database dump to the module
	 * (reserved 12 bytes at the beginning are added automatically)
	 *
	 * @param data pointer to the data to upload
	 * @param len length of the data to upload
	 * @return true if the upload was successful
	 */
	void uploadDatabaseDump(uint8_t* data, size_t len);

	/**
	 * @brief blocks execution until message is received or timeout occurs
	 * as it isn't called from the library itself it's up to the user to wait if they want to
	 *
	 * @return true if a message was received, false if timeout occurred
	 */
	bool waitForResponse();

	/**
	 * adds new observer (if not already present) to the list of observers, ownership is not transferred
	 *
	 * @param *v - observer to add
	 * @param msgClass - class of the message to observe
	 * @param msgID - id of the message to observe
	 * @return  true if observer was added, false if observer was already present
	 */
	bool addObserver(UbloxEventObserver* v, uint8_t msgClass, uint8_t msgID);

	/**
	 * removes observer from the list of observers, delete is not called on
	 *
	 * @param *v - observer to remove
	 * @param msgClass - class of the message to stop observing
	 * @param msgID - id of the message to stop observing
	 * @return  true if observer was removed, false if observer was not present
	 */
	bool removeObserver(UbloxEventObserver* v, uint8_t msgClass, uint8_t msgID);

	bool currentlyReceivingMessage()
	{
		return (start_message_ == true && end_message_ == false);
	}

	size_t sizeOfMessageReceived()
	{
		return num_messages_received_;
	}

	// Parsing State Working Memory

	/*
	 * @brief get major device version
	 *
	 * @return major version
	 */
	int getMajorVersion() const { return major_version_; }

	/*
	 * @brief get minor device version
	 *
	 * @return minor version
	 */
	int getMinorVersion() const { return minor_version_; }

	/*
	 * @brief get module name
	 *
	 * @return module name
	 */
	const char* getModuleName() const { return module_name_; }


	void interpretBufferAsMEAS();
};

} // namespace ublox
#endif // UBLOX_GPS_H
