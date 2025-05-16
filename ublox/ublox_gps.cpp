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

#include <stdio.h>
#include <cassert>
#include <chrono>
#include <cstring>
#include <sstream>

#include "ublox_gps.h"

QueueHandle_t UBloxGPS::uart0_queue;

using namespace std::chrono;
using namespace std;

//#ifndef NDEBUG
// #define ESP_LOG( TAG, ...) fprintf(stderr, __VA_ARGS__)
//#else
#define ESP_LOG( TAG, ...)
//#endif

namespace ublox
{
UBloxGPS::UBloxGPS()
{
    buffer_head_ = 0;
    parse_state_ = START;
    message_class_ = 0;
    message_type_ = 0;
    length_ = 0;
    ck_a_ = 0;
    ck_b_ = 0;
    prev_byte_ = 0;
    start_message_ = false;
    end_message_ = false;
}

bool UBloxGPS::parsing_message()
{
    return (start_message_ == true && end_message_ == false);
}

size_t UBloxGPS::num_messages_received()
{
    return num_messages_received_;
}

bool UBloxGPS::get_version()
{
    using namespace std::chrono;

    got_ver_ = false;
    send_message(CLASS_MON, MON_VER, out_message_, 0);

    auto start = high_resolution_clock::now();
    int dt_ms = 0;
    while (!got_ver_ && dt_ms < TIMEOUT_MS)
    {
        dt_ms = duration_cast<milliseconds>(high_resolution_clock::now() - start).count();
    }

    if (got_ver_)
    {
        ESP_LOG( TAG, "Got version\n");
        return true;
    }
    else
    {
        ESP_LOG( TAG, "Did not get version\n");
        return false;
    }
}

bool UBloxGPS::wait_for_response()
{
    using namespace std::chrono;
    got_ack_ = got_nack_ = false;
    auto start = high_resolution_clock::now();
    int dt_ms = 0;
    while (!got_ack_ && !got_nack_ && dt_ms < TIMEOUT_MS)
    {
        dt_ms = duration_cast<milliseconds>(high_resolution_clock::now() - start).count();
    }
    if (got_ack_)
    {
        ESP_LOG( TAG, "Got Response\n");
        return true;
    }
    else if (got_nack_ || dt_ms >= TIMEOUT_MS)
    {
        ESP_LOG( TAG, "No Response\n");
        return false;
    }
    return false;
}

bool UBloxGPS::send_message(uint8_t msg_class, uint8_t msg_id, UBloxGPS_message_t &message, uint16_t len)
{
    // First, calculate the checksum
    uint8_t ck_a, ck_b;
    calculate_checksum(msg_class, msg_id, len, message, ck_a, ck_b);

    // Send message
    write(START_BYTE_1);
    write(START_BYTE_2);
    write(msg_class);
    write(msg_id);
    write(len & 0xFF);
    write((len >> 8) & 0xFF);
    write(message.buffer, len);
    write(ck_a);
    write(ck_b);
    return true;
}

void UBloxGPS::start_survey_in()
{
    out_message_.CFG_TMODE3.flags = CFG_TMODE3_t::SURVEY_IN;
    out_message_.CFG_TMODE3.svinMinDur = 60;   // Wait at least 1 minute
    out_message_.CFG_TMODE3.svinAccLimit = 3;  // At least within 3 centimeters
    send_message(CLASS_CFG, CFG_TMODE3, out_message_, sizeof(CFG_TMODE3_t));
}

void UBloxGPS::disable_nmea()
{
    ESP_LOG( TAG, "Disabling NMEA messages ");
    if (major_version_ <= 23)
    {
        ESP_LOG( TAG, "with old protocol\n");
        using CF = CFG_PRT_t;
        memset(&out_message_, 0, sizeof(CF));
        out_message_.CFG_PRT.portID = CF::PORT_USB | CF::PORT_UART1;
        out_message_.CFG_PRT.baudrate = 921600;
        out_message_.CFG_PRT.outProtoMask = CF::OUT_UBloxGPS | CF::OUT_RTCM3;
        out_message_.CFG_PRT.inProtoMask = CF::OUT_UBloxGPS | CF::OUT_RTCM3;
        out_message_.CFG_PRT.flags = CF::CHARLEN_8BIT | CF::PARITY_NONE | CF::STOP_BITS_1;
        send_message(CLASS_CFG, CFG_PRT, out_message_, sizeof(CF));
    }
    else
    {
        ESP_LOG( TAG, "with new protocol\n");
        using CV = CFG_VALSET_t;
        configure(CV::VERSION_0, CV::RAM, 0, CV::USB_INPROT_NMEA, 1);
        configure(CV::VERSION_0, CV::RAM, 0, CV::USB_OUTPROT_NMEA, 1);
    }
}

void UBloxGPS::set_dynamic_mode()
{
    ESP_LOG( TAG, "Setting dynamic mode ");
    if (major_version_ <= 23)
    {
        ESP_LOG( TAG, "with old protocol\n");
        memset(&out_message_, 0, sizeof(CFG_NAV5_t));
        out_message_.CFG_NAV5.mask = CFG_NAV5_t::MASK_DYN;
        out_message_.CFG_NAV5.dynModel = CFG_NAV5_t::DYNMODE_AIRBORNE_4G;
        send_message(CLASS_CFG, CFG_NAV5, out_message_, sizeof(CFG_NAV5_t));
    }
    else
    {
        ESP_LOG( TAG, "with new protocol\n");
        using CV = CFG_VALSET_t;
        configure(CV::VERSION_0, CV::RAM, CV::DYNMODE_AIRBORNE_1G, CV::DYNMODEL, 1);
    }
}

void UBloxGPS::set_nav_rate(uint8_t period_ms)
{
    ESP_LOG( TAG, "Setting nav rate to %d\n", period_ms);
    if (major_version_ <= 23)
    {
        ESP_LOG( TAG, "Using old protocol\n");
        memset(&out_message_, 0, sizeof(CFG_RATE_t));
        out_message_.CFG_RATE.measRate = period_ms;
        out_message_.CFG_RATE.navRate = 1;
        out_message_.CFG_RATE.timeRef = CFG_RATE_t::TIME_REF_UTC;
        send_message(CLASS_CFG, CFG_RATE, out_message_, sizeof(CFG_RATE_t));
    }
    else
    {
        ESP_LOG( TAG, "Using new protocol\n");
        using CV = CFG_VALSET_t;
        configure(CV::VERSION_0, CV::RAM, period_ms, CV::RATE_MEAS, 1);
        configure(CV::VERSION_0, CV::RAM, 1, CV::RATE_NAV, 1);
        configure(CV::VERSION_0, CV::RAM, CV::TIME_REF_UTC, CV::RATE_TIMEREF, 1);
    }
}
// void UBloxGPS::set_nav_rate(uint8_t period_ms) {

//   ESP_LOG( TAG, "Setting nav rate to %d\n", period_ms);

//   configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, period_ms,
//             CFG_VALSET_t::RATE_MEAS, byte);
//   configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM, 1,
//             CFG_VALSET_t::RATE_NAV, byte);
//   configure(CFG_VALSET_t::VERSION_0, CFG_VALSET_t::RAM,
//             CFG_VALSET_t::TIME_REF_UTC, CFG_VALSET_t::RATE_TIMEREF, byte);
// }

void UBloxGPS::enable_message(uint8_t msg_cls, uint8_t msg_id, uint8_t rate)
{
    ESP_LOG( TAG, "Requesting %x:%x message with period=%d ", msg_cls, msg_id, rate);
    // if (major_version_ <= 23)
    ESP_LOG( TAG, "Using old protocol\n");
    memset(&out_message_, 0, sizeof(CFG_MSG_t));
    out_message_.CFG_MSG.msgClass = msg_cls;
    out_message_.CFG_MSG.msgID = msg_id;
    out_message_.CFG_MSG.rate = rate;
    send_message(CLASS_CFG, CFG_MSG, out_message_, sizeof(CFG_MSG_t));
    // }
    // else
    // {
    //     ESP_LOG( TAG, "Using new protocol\n");
    //     using CV = CFG_VALSET_t;
    //     configure(CV::VERSION_0, CV::RAM, 1, CV::MSGOUT_PVT, 1);
    // }
}

bool UBloxGPS::read_cb(uint8_t byte)
{
    switch (parse_state_)
    {
    case START:
        if (byte == START_BYTE_2 && prev_byte_ == START_BYTE_1)
        {
            buffer_head_ = 0;
            parse_state_ = GOT_START_FRAME;
            message_class_ = 0;
            message_type_ = 0;
            length_ = 0;
            ck_a_ = 0;
            ck_b_ = 0;
            start_message_ = true;
            end_message_ = false;
        }
        break;
    case GOT_START_FRAME:
        message_class_ = byte;
        parse_state_ = GOT_CLASS;
        break;
    case GOT_CLASS:
        message_type_ = byte;
        parse_state_ = GOT_MSG_ID;
        break;
    case GOT_MSG_ID:
        length_ = byte;
        parse_state_ = GOT_LENGTH1;
        break;
    case GOT_LENGTH1:
        length_ |= (uint16_t)byte << 8;
        parse_state_ = GOT_LENGTH2;
        if (length_ > BUFFER_SIZE)
        {
            ESP_LOG( TAG, "Message 0x%x-0x%x is too long (%d > %ld)\n", message_class_, message_type_,
                length_, BUFFER_SIZE);
            num_errors_++;
            prev_byte_ = byte;
            restart();
            return false;
        }
        break;
    case GOT_LENGTH2:
        if (buffer_head_ < length_)
        {
            // push the byte onto the data buffer
            in_message_.buffer[buffer_head_] = byte;
            if (buffer_head_ == length_ - 1)
            {
                parse_state_ = GOT_PAYLOAD;
            }
            buffer_head_++;
        }
        break;
    case GOT_PAYLOAD:
        ck_a_ = byte;
        parse_state_ = GOT_CK_A;
        break;
    case GOT_CK_A:
        ck_b_ = byte;
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
    if (parse_state_ == GOT_CK_B)
    {
        if (decode_message())
        {
            parse_state_ = START;
            end_message_ = true;
            start_message_ = false;
            prev_byte_ = byte;
            return true;
        }
        else
        {
            // indicate error if it didn't work
            ESP_LOG( TAG, "\n failed to parse message\n");
            num_errors_++;
            parse_state_ = START;
            start_message_ = false;
            end_message_ = false;
        }
    }
    prev_byte_ = byte;
    return false;
}

void UBloxGPS::restart()
{
    parse_state_ = START;
    end_message_ = false;
    start_message_ = false;
}

bool UBloxGPS::decode_message()
{
    // First, check the checksum
    uint8_t ck_a, ck_b;
    calculate_checksum(message_class_, message_type_, length_, in_message_, ck_a, ck_b);
    if (ck_a != ck_a_ || ck_b != ck_b_)
        return false;
    uint8_t version;  // 0 poll request, 1 poll (receiver to return config data key
                      // and value pairs)
    uint8_t layer;
    uint8_t reserved1[2];
    uint32_t cfgDataKey;
    uint64_t cfgData;
    num_messages_received_++;

    // Parse the payload
    switch (message_class_)
    {
    case CLASS_ACK:
        ESP_LOG( TAG, "ACK_");
        switch (message_type_)
        {
        case ACK_ACK:
            got_ack_ = true;
            ESP_LOG( TAG, "ACK\n");
            break;
        case ACK_NACK:
            got_nack_ = true;
            ESP_LOG( TAG, "NACK\n");
            break;
        default:
            ESP_LOG( TAG, "%d\n", message_type_);
            break;
        }
        break;
    case CLASS_MON:
        ESP_LOG( TAG, "MON_");
        switch (message_type_)
        {
        case MON_VER:
            ESP_LOG( TAG, "VER\n");
            version_cb();
            break;
        case MON_COMMS:

            ESP_LOG( TAG, "COMMS\n");
            break;
        case MON_TXBUF:

            ESP_LOG( TAG, "TXMON_TXBUF\n");
            break;
        }
        break;
    case CLASS_RXM:
        ESP_LOG( TAG, "RXM_");
        switch (message_type_)
        {
        case RXM_RAWX:
            ESP_LOG( TAG, "RAWX\n");
            break;
        case RXM_SFRBX:
            ESP_LOG( TAG, "SFRBX\n");
            break;
        }
        break;
    case CLASS_NAV:
        ESP_LOG( TAG, "NAV_");
        switch (message_type_)
        {
        case NAV_PVT:
            ESP_LOG( TAG, "PVT \n");
            break;
        case NAV_RELPOSNED:
            ESP_LOG( TAG, "RELPOSNED \n");
            break;
        case NAV_POSECEF:
            ESP_LOG( TAG, "POSECEF\n");
            break;
        case NAV_VELECEF:
            ESP_LOG( TAG, "VELECEF\n");
            break;
        default:
            ESP_LOG( TAG, " unknown (%d) \n", message_type_);
        }
        break;
    case CLASS_CFG:  // only needed for getting data
        ESP_LOG( TAG, "CFG_");
        switch (message_type_)
        {
        case CFG_VALGET:
        {
            ESP_LOG( TAG, "VALGET = ");
            int value = in_message_.CFG_VALGET.cfgData;
            ESP_LOG( TAG, "%d \n", value);
            break;
        }
        default:
            ESP_LOG( TAG, "unknown: %x\n", message_type_);
        }
        break;

    default:
        //        ESP_LOG( TAG, "Unknown (%d-%d)\n", message_class_, message_type_);
        break;
    }

    // call callbacks

    return true;
}

void UBloxGPS::calculate_checksum(const uint8_t msg_cls,
                             const uint8_t msg_id,
                             const uint16_t len,
                             const UBloxGPS_message_t payload,
                             uint8_t &ck_a,
                             uint8_t &ck_b) const
{
    if (msg_cls == 5)
        volatile int debug = 1;
    ck_a = ck_b = 0;

    // Add in class
    ck_a += msg_cls;
    ck_b += ck_a;

    // Id
    ck_a += msg_id;
    ck_b += ck_a;

    // Length
    ck_a += len & 0xFF;
    ck_b += ck_a;
    ck_a += (len >> 8) & 0xFF;
    ck_b += ck_a;

    // Payload
    for (int i = 0; i < len; i++)
    {
        ck_a += payload.buffer[i];
        ck_b += ck_a;
    }
}

void UBloxGPS::configure(uint8_t version,
                    uint8_t layer,
                    uint64_t cfgData,
                    uint32_t cfgDataKey,
                    uint8_t size)
{
    memset(&out_message_, 0, sizeof(CFG_VALSET_t));
    out_message_.CFG_VALSET.version = version;
    out_message_.CFG_VALSET.layer = layer;
    if (size == 1)
    {
        out_message_.CFG_VALSET.cfgData.bytes[0] = cfgData;
    }
    if (size == 2)
    {
        out_message_.CFG_VALSET.cfgData.word = cfgData;
    }
    out_message_.CFG_VALSET.cfgDataKey = cfgDataKey;
    send_message(CLASS_CFG, CFG_VALSET, out_message_, sizeof(CFG_VALSET_t));
}

void UBloxGPS::get_configuration(uint8_t version, uint8_t layer, uint32_t cfgDataKey)
{
    memset(&out_message_, 0, sizeof(CFG_VALGET_t));
    out_message_.CFG_VALGET.version = version;
    out_message_.CFG_VALGET.layer = layer;
    out_message_.CFG_VALGET.cfgDataKey = cfgDataKey;
    send_message(CLASS_CFG, CFG_VALGET, out_message_, sizeof(CFG_VALGET_t));
}

void UBloxGPS::write(const uint8_t byte)
{
    ser_.write(&byte, 1);
}
void UBloxGPS::write(const uint8_t *byte, const size_t size)
{
    ser_.write(byte, size);
}

void UBloxGPS::version_cb()
{
    int protocol_version;
    std::string module_type;
    int firmware_version;
    for (int i = 0; i < 20; ++i)
    {
        if (strncmp(in_message_.MON_VER.extension[i], "PROTVER=", 8) == 0)
        {
            extract_version_string(in_message_.MON_VER.extension[i]);
        }
        if (strncmp(in_message_.MON_VER.extension[i], "MOD=", 4) == 0)
        {
            extract_module_name(in_message_.MON_VER.extension[i]);
        }
    }

    printf("Connected:\n");
    printf("Module: %s\n", module_name_);
    printf("Protocol Version: %d.%d\n\n", major_version_, minor_version_);
    got_ver_ = true;
}

void UBloxGPS::extract_version_string(const char *str)
{
    // Get major version
    char tmp[5];
    const char *s = str + 8;
    for (int j = 0; j < 5; j++)
    {
        if (*s == '.')
        {
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
    for (int j = 0; j < 5; j++)
    {
        if (*s == 0)
        {
            tmp[j] = 0;
            break;
        }
        tmp[j] = *s;
        ++s;
    }
    minor_version_ = atoi(tmp);
}

void UBloxGPS::extract_module_name(const char *str)
{
    const char *s = str + 4;
    for (int i = 0; i < sizeof(module_name_); i++)
    {
        if (*s == 0)
        {
            module_name_[i] = 0;
            break;
        }

        module_name_[i] = *s;
        ++s;
    }
}

void UBloxGPS::uartEvent(void* pvParameters)
{
	auto startIndex = [](const char* str) { // skips first 4 characters and all spaces and tabs
		int i = 4;
		while (str[i] == ' ' || str[i] == '\t')
			i++;
		return i;
	};

	static uart_event_t event;
	static uint8_t* dtmp = (uint8_t*)malloc(BUFFER_SIZE);

	static char buffer[BUFFER_SIZE + 2];
	static int i = 0;
	static int j = 0;
	static int j_prev = 0;
	static char* pointer = 0;
	i = 0;
	j = 0;
	j_prev = 0;
	pointer = 0;
	ESP_LOGI(TAG, "uartEvent | uartEvent started");
	while (1) {
		if (xQueueReceive(uart0_queue, (void*)&event, (TickType_t)portMAX_DELAY)) {
			memset(dtmp, 0, BUFFER_SIZE);
			if (event.type == UART_DATA) {
				uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);

				for (j = 0, j_prev = 0; j < event.size; j++) {
					if ((int)dtmp[j] == 13) {
						memcpy(buffer + i, dtmp + j_prev, LENGTH_TO_COPY(i, j - j_prev));
						i += LENGTH_TO_COPY(i, j - j_prev);
						buffer[i] = '\0';

						ESP_LOGI(TAG, "uartEvent | Received data: %s", buffer);

						i = 0;
						j_prev = j + 1;
					}
				}
				memcpy(buffer + i, dtmp + j_prev, LENGTH_TO_COPY(i, event.size - j_prev)); // copy rest of data
				i += LENGTH_TO_COPY(i, event.size - j_prev);
			}
		}
	}
	free(dtmp);
	dtmp = NULL;
	vTaskDelete(NULL);
}

void UBloxGPS::setupSerial()
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
	uart_driver_install(EX_UART_NUM, BUFFER_SIZE<<1, BUFFER_SIZE<<1, 20, &uart0_queue, 0);
	uart_param_config(EX_UART_NUM, &uart_config);
	uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	// Create a task to handler UART event from ISR
	xTaskCreate(uartEvent, "uartEvent", 2048, NULL, 12, NULL);
}



}  // namespace ublox
