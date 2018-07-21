#include <Arduino.h>
#include <algorithm>

#include "driver/spi_slave.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_wifi_internal.h"
#include "esp_heap_caps.h"
#include "wifi_raw.h"
#include "fec_codec.h"
#include "esp_task_wdt.h"
#include "bt.h"

#include "structures.h"
#include "spi_comms.h"

static int s_stats_last_tp = 0;

static constexpr uint8_t s_rate_mapping[31] = 
{ 
    0,  //0  - B 1M   DSSS
    1,  //1  - B 2M   DSSS
    5,  //2  - B 2M   DSSS Short Preamble
    2,  //3  - B 5.5M DSSS
    6,  //4  - B 5.5M DSSS Short Preamble
    3,  //5  - B 11M  DSSS
    7,  //6  - B 11M  DSSS Short Preamble

    11, //7  - G 6M   ODFM
    15, //8  - G 9M   ODFM
    10, //9  - G 12M  ODFM 
    14, //10 - G 18M  ODFM
    9,  //11 - G 24M  ODFM
    13, //12 - G 36M  ODFM
    8,  //13 - G 48M  ODFM
    12, //14 - G 54M  ODFM

    16, //15 - N 6.5M  MCS0
    24, //16 - N 7.2M  MCS0 Short Guard Interval
    17, //17 - N 13M   MCS1
    25, //18 - N 14.4M MCS1 Short Guard Interval
    18, //19 - N 19.5M MCS2
    26, //20 - N 21.7M MCS2 Short Guard Interval
    19, //21 - N 26M   MCS3
    27, //22 - N 28.9M MCS3 Short Guard Interval
    20, //23 - N 39M   MCS4
    28, //24 - N 43.3M MCS4 Short Guard Interval
    21, //25 - N 52M   MCS5
    29, //26 - N 57.8M MCS5 Short Guard Interval
    22, //27 - N 58M   MCS6
    30, //28 - N 65M   MCS6 Short Guard Interval
    23, //29 - N 65M   MCS7
    31, //30 - N 72M   MCS7 Short Guard Interval
};

static constexpr gpio_num_t GPIO_MOSI = gpio_num_t(13);
static constexpr gpio_num_t GPIO_MISO = gpio_num_t(12);
static constexpr gpio_num_t GPIO_SCLK = gpio_num_t(14);
static constexpr gpio_num_t GPIO_CS = gpio_num_t(15);

/////////////////////////////////////////////////////////////////////////

static int s_uart_verbose = 0;
static char s_uart_command = 0;
static int s_uart_error_count = 0;

#define LOG(...) if (s_uart_verbose > 0) Serial.printf(__VA_ARGS__)

/////////////////////////////////////////////////////////////////////////

static constexpr uint8_t STATUS_LED_PIN = 2;
static constexpr uint8_t STATUS_LED_ON = HIGH;
static constexpr uint8_t STATUS_LED_OFF = LOW;
static constexpr uint32_t STATUS_LED_ON_DURATION = 30; //ms
static uint32_t s_status_led_tp = 0;
static uint32_t s_status_led_state = 0;

void initialize_status_led()
{
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, STATUS_LED_OFF);
    s_status_led_tp = 0;
    s_status_led_state = 0;
}

void set_status_led_on()
{
    uint32_t now = millis();
    if (now - s_status_led_tp < STATUS_LED_ON_DURATION)
    {
        return;
    }
    
    if (s_status_led_state == 0)
    {
        digitalWrite(STATUS_LED_PIN, STATUS_LED_ON);
    }
    else
    {
        digitalWrite(STATUS_LED_PIN, STATUS_LED_OFF);
    }
    s_status_led_state = !s_status_led_state;
    s_status_led_tp = now;
}

IRAM_ATTR void update_status_led()
{
    if (s_status_led_tp == 0)
    {
        return;
    }
    if (millis() - s_status_led_tp >= STATUS_LED_ON_DURATION)
    {
        digitalWrite(STATUS_LED_PIN, STATUS_LED_OFF);
        s_status_led_tp = 0;
    }
}

/////////////////////////////////////////////////////////////////////////

static uint8_t s_crc8_table[256];     /* 8-bit table */
static void init_crc8_table()
{
    static constexpr uint8_t DI = 0x07;
    for (uint16_t i = 0; i < 256; i++)
    {
        uint8_t crc = (uint8_t)i;
        for (uint8_t j = 0; j < 8; j++)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
        }
        s_crc8_table[i] = crc & 0xFF;
    }
}

IRAM_ATTR static uint8_t crc8(uint8_t crc, const void *c_ptr, size_t len)
{
    const uint8_t* c = reinterpret_cast<const uint8_t*>(c_ptr);
    size_t n = (len + 7) >> 3;
    switch (len & 7)
    {
    case 0: do { crc = s_crc8_table[crc ^ (*c++)];
    case 7:      crc = s_crc8_table[crc ^ (*c++)];
            case 6:      crc = s_crc8_table[crc ^ (*c++)];
            case 5:      crc = s_crc8_table[crc ^ (*c++)];
            case 4:      crc = s_crc8_table[crc ^ (*c++)];
            case 3:      crc = s_crc8_table[crc ^ (*c++)];
            case 2:      crc = s_crc8_table[crc ^ (*c++)];
            case 1:      crc = s_crc8_table[crc ^ (*c++)];
        } while (--n > 0);
    }
    return crc;
}

/////////////////////////////////////////////////////////////////////////

portMUX_TYPE s_fec_codec_mux = portMUX_INITIALIZER_UNLOCKED;
Fec_Codec s_fec_codec;

/////////////////////////////////////////////////////////////////////////

float s_wlan_power_dBm = 0;

esp_err_t set_wlan_power_dBm(float dBm)
{
    constexpr float k_min = 14.f;
    constexpr float k_max = 20.5f;

    dBm = std::max(std::min(dBm, k_max), k_min);
    s_wlan_power_dBm = dBm;
    int8_t power = static_cast<int8_t>(((dBm - k_min) / (k_max - k_min)) * 80) + 7;
    return esp_wifi_set_max_tx_power(power);
}

float get_wlan_power_dBm()
{
    return s_wlan_power_dBm;
}

uint8_t s_wlan_rate = 0;
esp_err_t set_wifi_fixed_rate(uint8_t value)
{
    //https://github.com/espressif/esp-idf/issues/833
    uint8_t clamped_value = value <= 30 ? value : 30;
    wifi_internal_rate_t rate;
    rate.fix_rate = s_rate_mapping[clamped_value];
    esp_err_t err = esp_wifi_internal_set_rate(100, 1, 4, &rate);
    if (err == ESP_OK)
    {
        s_wlan_rate = clamped_value;
    }
    return err;
}

uint8_t get_wifi_fixed_rate()
{
    return s_wlan_rate;
}

IRAM_ATTR esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

int16_t s_wlan_incoming_rssi = 0; //this is protected by the s_wlan_incoming_mux

///////////////////////////////////////////////////////////////////////////////////////////

IRAM_ATTR void add_to_wlan_outgoing_queue(const Wlan_Packet_Header& packet_header, const void* data, size_t size, bool isr)
{
    Wlan_Outgoing_Packet packet;

    if (isr)
    {
      portENTER_CRITICAL_ISR(&s_wlan_outgoing_mux);
      start_writing_wlan_outgoing_packet(packet, size + sizeof(Wlan_Packet_Header));
      portEXIT_CRITICAL_ISR(&s_wlan_outgoing_mux);
    }
    else
    {
      portENTER_CRITICAL(&s_wlan_outgoing_mux);
      start_writing_wlan_outgoing_packet(packet, size + sizeof(Wlan_Packet_Header));
      portEXIT_CRITICAL(&s_wlan_outgoing_mux);
    }
    
    if (!packet.ptr)
    {
        //LOG("Sending failed: previous packet still in flight\n");
        return;
    }
    //LOG("Sending %d\n", size);

    *((Wlan_Packet_Header*)packet.payload_ptr) = packet_header;

    memcpy(packet.payload_ptr + sizeof(Wlan_Packet_Header), data, size);
    
    //LOG("Sending packet of size %d\n", packet.size);

    if (isr)
    {
      portENTER_CRITICAL_ISR(&s_wlan_outgoing_mux);
      end_writing_wlan_outgoing_packet(packet);
      portEXIT_CRITICAL_ISR(&s_wlan_outgoing_mux);  
    }
    else
    {
      portENTER_CRITICAL(&s_wlan_outgoing_mux);
      end_writing_wlan_outgoing_packet(packet);
      portEXIT_CRITICAL(&s_wlan_outgoing_mux);  
    }
}

IRAM_ATTR void add_to_wlan_incoming_queue(const void* data, size_t size, int16_t rssi, bool isr)
{
    Wlan_Incoming_Packet packet;

    if (isr)
    {
      portENTER_CRITICAL_ISR(&s_wlan_incoming_mux);
      start_writing_wlan_incoming_packet(packet, size);
      portEXIT_CRITICAL_ISR(&s_wlan_incoming_mux);
    }
    else
    {
      portENTER_CRITICAL(&s_wlan_incoming_mux);
      start_writing_wlan_incoming_packet(packet, size);
      portEXIT_CRITICAL(&s_wlan_incoming_mux);
    }
    
    if (!packet.ptr)
    {
        //LOG("Sending failed: previous packet still in flight\n");
        return;
    }
    //LOG("decoded %d\n", size);
    
    memcpy(packet.ptr, data, size);
    
    //LOG("Sending packet of size %d\n", packet.size);

    packet.rssi = rssi;

    if (isr)
    {
      portENTER_CRITICAL_ISR(&s_wlan_incoming_mux);
      end_writing_wlan_incoming_packet(packet);
      portEXIT_CRITICAL_ISR(&s_wlan_incoming_mux);
    }
    else
    {
      portENTER_CRITICAL(&s_wlan_incoming_mux);
      end_writing_wlan_incoming_packet(packet);
      portEXIT_CRITICAL(&s_wlan_incoming_mux);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////

IRAM_ATTR void packet_received_cb(void* buf, wifi_promiscuous_pkt_type_t type)
{
    if (type == WIFI_PKT_MGMT)
    {
        //Serial.printf("management packet\n");
        return;
    }
    else if (type == WIFI_PKT_DATA)
    {
        //Serial.printf("data packet\n");
    }
    else if (type == WIFI_PKT_MISC)
    {
        //Serial.printf("misc packet\n");
        return;
    }

    wifi_promiscuous_pkt_t* pkt = reinterpret_cast<wifi_promiscuous_pkt_t*>(buf);

    uint16_t len = pkt->rx_ctrl.sig_len;
    //s_stats.wlan_data_received += len;
    //s_stats.wlan_data_sent += 1;

    if (len <= WLAN_IEEE_HEADER_SIZE + sizeof(Wlan_Packet_Header))
    {
        //LOG("WLAN receive header error");
        s_stats.wlan_error_count++;
        return;
    }

    //Serial.printf("Recv callback #%d: %d bytes\n", counter++, len);
    //Serial.printf("Channel: %d PHY: %d\n", pkt->rx_ctl.channel, wifi_get_phy_mode());

    //uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    //Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    uint8_t* data = pkt->payload;
    if (memcmp(data + 10, s_wlan_ieee_header + 10, 6) != 0)
    {
        return;
    }

    data += WLAN_IEEE_HEADER_SIZE;
    len -= WLAN_IEEE_HEADER_SIZE; //skip the 802.11 header

    len -= 4;//the received length has 4 more bytes at the end for some reason.

    int16_t rssi = pkt->rx_ctrl.rssi;
    /*  if (s_uart_verbose >= 1)
  {
    Serial.printf("RSSI: %d, CH: %d, SZ: %d\n", rssi, pkt->rx_ctrl.channel, len);
    if (s_uart_verbose >= 2)
    {
      Serial.printf("---->\n");
      Serial.write(data, len);
      Serial.printf("\n<----\n");
    }
  }
*/


    size_t size = std::min<size_t>(len, WLAN_MAX_PAYLOAD_SIZE);

    Wlan_Packet_Header& packet_header = *((Wlan_Packet_Header*)data);
    data += sizeof(Wlan_Packet_Header);
    size -= sizeof(Wlan_Packet_Header);
    
    if (packet_header.uses_fec)
    {
      portENTER_CRITICAL_ISR(&s_wlan_incoming_mux);
      s_wlan_incoming_rssi = rssi;
      portEXIT_CRITICAL_ISR(&s_wlan_incoming_mux);
  
      portENTER_CRITICAL_ISR(&s_fec_codec_mux);
      if (!s_fec_codec.decode_data(data, size, true, false))
      {
          s_stats.wlan_received_packets_dropped++;
      }
      portEXIT_CRITICAL_ISR(&s_fec_codec_mux);
    }
    else
    {
      add_to_wlan_incoming_queue(data, size, rssi, true);
    }
    
    s_stats.wlan_data_received += len;
}

/////////////////////////////////////////////////////////////////////////

uint8_t s_uart_buffer[WLAN_MAX_PAYLOAD_SIZE];
size_t s_uart_offset = 0;

void parse_command()
{
    if (s_uart_command == 0)
    {
        char ch = Serial.read();
        if (ch <= 0 || ch == '\n')
        {
            return;
        }
        s_uart_command = ch;
    }

    int available = Serial.available();
    if (available <= 0)
    {
        return;
    }

    if (s_uart_command == 'V')
    {
        s_uart_command = 0;
        s_uart_verbose++;
        if (s_uart_verbose > 2)
        {
            s_uart_verbose = 0;
        }
        Serial.printf("Verbose: %d\n", s_uart_verbose);
    }
    else if (s_uart_command == 'S')
    {
        while (available-- > 0)
        {
            char ch = Serial.read();
            if (ch == '\n' && s_uart_offset > 0)
            {
                Wlan_Outgoing_Packet packet;
                
                portENTER_CRITICAL(&s_wlan_outgoing_mux);
                start_writing_wlan_outgoing_packet(packet, s_uart_offset);
                portEXIT_CRITICAL(&s_wlan_outgoing_mux);
                
                if (!packet.ptr)
                {
                    LOG("Sending failed: previous packet still in flight\n");
                    s_uart_command = 0;
                    s_uart_offset = 0;
                    s_uart_error_count++;
                    return;
                }

                memcpy(packet.payload_ptr, s_uart_buffer, s_uart_offset);

                LOG("Sending packet of size %d\n", packet.size);
                
                portENTER_CRITICAL(&s_wlan_outgoing_mux);
                end_writing_wlan_outgoing_packet(packet);
                portEXIT_CRITICAL(&s_wlan_outgoing_mux);
                
                s_uart_command = 0;
                s_uart_offset = 0;
            }
            else
            {
                if (s_uart_offset >= WLAN_MAX_PAYLOAD_SIZE)
                {
                    while (available-- > 0) Serial.read();
                    LOG("Packet too big: %d > %d\n", s_uart_offset + 1, WLAN_MAX_PAYLOAD_SIZE);

                    s_uart_command = 0;
                    s_uart_offset = 0;
                    s_uart_error_count++;
                    return;
                }
                s_uart_buffer[s_uart_offset++] = ch;
            }
        }
    }
    else if (s_uart_command == 'T')
    {
        s_uart_command = 0;
        //Serial.printf("Received: %d bytes, errors: %d\n", s_received, s_receive_error_count);
        //Serial.printf("Sent: %d bytes, errors: %d\n", s_sent, s_send_error_count);
        //Serial.printf("Command errors: %d\n", s_uart_error_count);
    }
    else if (s_uart_command == 'C')
    {
        if (available == 0)
        {
            return;
        }
        s_uart_command = 0;

        uint8_t channel = 0;
        char ch = Serial.read();
        if (ch >= '1' && ch <= '9') channel = ch - '0';
        else if (ch >= 'A' && ch <= 'F') channel = ch - 'A' + 10;
        else if (ch >= 'a' && ch <= 'f') channel = ch - 'a' + 10;
        else
        {
            s_uart_error_count++;
            LOG("Command error: Illegal channel %c\n", ch);
            return;
        }
        if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) == ESP_OK)
        {
          LOG("Channel set to %d\n", channel);
        }
        else
        {
          LOG("Command error: call to wifi_set_channel failed\n");
        }
    }
    else if (s_uart_command == 'P')
    {
        if (available == 0)
        {
            return;
        }
        s_uart_command = 0;

        float power = 0.f;
        char ch = Serial.read();
        if (ch >= '0' && ch <= '9') power = ((ch - '0') / 9.f) * 20.5f;
        else
        {
            s_uart_error_count++;
            LOG("Command error: Illegal power %c\n", ch);
            return;
        }

        set_wlan_power_dBm(power);
        LOG("Power set to %.2f\n", power);
    }
    else if (s_uart_command == 'R')
    {
        if (available == 0)
        {
            return;
        }
        s_uart_command = 0;

        uint8_t rate = 0;
        do
        {
            char ch = Serial.read();
            if (ch >= '0' && ch <= '9') rate = rate * 10 + (ch - '0');
            else
            {
                break;
            }
        } while (true);
        
        if (set_wifi_fixed_rate(rate) == ESP_OK)
        {
            LOG("Rate set to %d\n", rate);
        }
        else
        {
            LOG("Command error: call to wifi_set_user_fixed_rate failed\n");
        }
    }
    else
    {
        s_uart_error_count++;
        LOG("Command error: %c\n", s_uart_command);
        s_uart_command = 0;
    }
}

/////////////////////////////////////////////////////////////////////////

IRAM_ATTR void fec_encoded_cb(void* data, size_t size)
{
    Wlan_Packet_Header packet_header;
    packet_header.uses_fec = 1;
    add_to_wlan_outgoing_queue(packet_header, data, size, false);
}

IRAM_ATTR void fec_decoded_cb(void* data, size_t size)
{
    portENTER_CRITICAL(&s_wlan_incoming_mux);
    int16_t rssi = s_wlan_incoming_rssi;
    portEXIT_CRITICAL(&s_wlan_incoming_mux);
    
    add_to_wlan_incoming_queue(data, size, rssi, false);
}

/////////////////////////////////////////////////////////////////////////

uint8_t* s_spi_tx_buffer = nullptr;
uint8_t* s_spi_rx_buffer = nullptr;
uint8_t* s_spi_rx_payload_ptr = nullptr;
Wlan_Incoming_Packet s_spi_last_packet;
uint8_t s_spi_packet_id = 0;
int s_spi_transaction_id = 0;

IRAM_ATTR void spi_post_setup_cb(spi_slave_transaction_t* trans)
{
    //    LOG("SPI armed %d: %d\n", (int)trans->user, trans->length / 8);
}

IRAM_ATTR void spi_post_trans_cb(spi_slave_transaction_t* trans)
{
    //    LOG("SPI done %d: %d / %d\n", (int)trans->user, trans->length / 8, trans->trans_len / 8);
}

static spi_slave_transaction_t s_spi_transaction;

esp_err_t init_spi()
{
    s_spi_tx_buffer = (uint8_t*)heap_caps_malloc(MAX_SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
    if (!s_spi_tx_buffer)
    {
        return ESP_ERR_NO_MEM;
    }
    s_spi_rx_buffer = (uint8_t*)heap_caps_malloc(MAX_SPI_BUFFER_SIZE, MALLOC_CAP_DMA);
    if (!s_spi_rx_buffer)
    {
        return ESP_ERR_NO_MEM;
    }

    memset(&s_spi_transaction, 0, sizeof(s_spi_transaction));

    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);
    
    
    //Configuration for the SPI bus
    spi_bus_config_t bus_config;
    memset(&bus_config, 0, sizeof(bus_config));
    bus_config.mosi_io_num = GPIO_MOSI;
    bus_config.miso_io_num = GPIO_MISO;
    bus_config.sclk_io_num = GPIO_SCLK;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = MAX_SPI_BUFFER_SIZE * 8;
    
    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slave_config;
    slave_config.mode = 0;
    slave_config.spics_io_num = GPIO_CS;
    slave_config.queue_size = 2;
    slave_config.flags = 0;
    slave_config.post_setup_cb = spi_post_setup_cb;
    slave_config.post_trans_cb = spi_post_trans_cb;
    
    return spi_slave_initialize(VSPI_HOST, &bus_config, &slave_config, 1);
}

IRAM_ATTR void setup_spi_packet_response(size_t transfer_size, uint8_t seq)
{
    SPI_Res_Packet_Header& header = *reinterpret_cast<SPI_Res_Packet_Header*>(s_spi_tx_buffer);
    header.res = static_cast<uint8_t>(SPI_Res::PACKET);
    header.seq = seq & 0x7F;

    ///////////////////////////////////////////////////////
    portENTER_CRITICAL_ISR(&s_wlan_incoming_mux);
    //did the transfer push the last packet out? finish it
    if (s_spi_last_packet.ptr != nullptr && transfer_size >= s_spi_last_packet.size + sizeof(SPI_Res_Packet_Header))
    {
        //LOG("Ending packet\n");
        end_reading_wlan_incoming_packet(s_spi_last_packet);
        s_spi_packet_id++;
    }

    //no packet, get a new one
    if (!s_spi_last_packet.ptr)
    {
        //LOG("Advance packet\n");
        start_reading_wlan_incoming_packet(s_spi_last_packet);
    }
    else
    {
        //LOG("Same packet\n");
    }
    header.pending_packets = s_wlan_incoming_queue.count();
    portEXIT_CRITICAL_ISR(&s_wlan_incoming_mux);

    ///////////////////////////////////////////////////////

    if (s_spi_last_packet.ptr)
    {
        //LOG("Yes packet\n");
        header.rssi = s_spi_last_packet.rssi;
        header.next_packet_size = s_spi_last_packet.size;
        header.packet_size = s_spi_last_packet.size;
        memcpy(s_spi_tx_buffer + sizeof(header), s_spi_last_packet.ptr, s_spi_last_packet.size);
    }
    else
    {
        //LOG("No packet\n");
        header.next_packet_size = 0;
        header.packet_size = 0;
    }

    header.packet_id = s_spi_packet_id;
    header.crc = 0;
    header.crc = crc8(0, s_spi_tx_buffer, sizeof(header));

    s_spi_transaction.length = MAX_SPI_BUFFER_SIZE * 8;//(size + payload_size) * 8;
    s_spi_transaction.tx_buffer = s_spi_tx_buffer;
    s_spi_transaction.rx_buffer = s_spi_rx_buffer;
    esp_err_t err = spi_slave_queue_trans(VSPI_HOST, &s_spi_transaction, 0);
    ESP_ERROR_CHECK(err);
}

IRAM_ATTR void setup_spi_base_response(size_t header_size)
{
    SPI_Res_Base_Header& header = *reinterpret_cast<SPI_Res_Base_Header*>(s_spi_tx_buffer);

    portENTER_CRITICAL_ISR(&s_wlan_incoming_mux);
    header.pending_packets = s_wlan_incoming_queue.count();
    header.next_packet_size = s_wlan_incoming_queue.size();
    portEXIT_CRITICAL_ISR(&s_wlan_incoming_mux);

    header.crc = 0;
    header.crc = crc8(0, s_spi_tx_buffer, header_size);

    s_spi_transaction.length = MAX_SPI_BUFFER_SIZE * 8;//(size + payload_size) * 8;
    s_spi_transaction.tx_buffer = s_spi_tx_buffer;
    s_spi_transaction.rx_buffer = s_spi_rx_buffer;
    esp_err_t err = spi_slave_queue_trans(VSPI_HOST, &s_spi_transaction, 0);
    ESP_ERROR_CHECK(err);
}

IRAM_ATTR void setup_spi_initial_response()
{
    SPI_Res_Packet_Header& header = *reinterpret_cast<SPI_Res_Packet_Header*>(s_spi_tx_buffer);
    memset(&header, 0, sizeof(header));
    header.res = static_cast<uint8_t>(SPI_Res::PACKET);
    header.crc = crc8(0, s_spi_tx_buffer, sizeof(header));

    s_spi_transaction.length = MAX_SPI_BUFFER_SIZE * 8;//(size + payload_size) * 8;
    s_spi_transaction.tx_buffer = s_spi_tx_buffer;
    s_spi_transaction.rx_buffer = s_spi_rx_buffer;
    esp_err_t err = spi_slave_queue_trans(VSPI_HOST, &s_spi_transaction, 0);
    ESP_ERROR_CHECK(err);
}

IRAM_ATTR size_t get_header_size(const uint8_t* ptr)
{
    const SPI_Req_Base_Header& base_header = *reinterpret_cast<const SPI_Req_Base_Header*>(ptr);
    SPI_Req req = static_cast<SPI_Req>(base_header.req);
    switch (req)
    {
    case SPI_Req::PACKET: return sizeof(SPI_Req_Packet_Header);
    case SPI_Req::SETUP_FEC_CODEC: return sizeof(SPI_Req_Setup_Fec_Codec_Header);
    case SPI_Req::SET_RATE: return sizeof(SPI_Req_Set_Rate_Header);
    case SPI_Req::GET_RATE: return sizeof(SPI_Res_Get_Rate_Header);
    case SPI_Req::SET_CHANNEL: return sizeof(SPI_Req_Set_Channel_Header);
    case SPI_Req::GET_CHANNEL: return sizeof(SPI_Res_Get_Channel_Header);
    case SPI_Req::SET_POWER: return sizeof(SPI_Req_Set_Power_Header);
    case SPI_Req::GET_POWER: return sizeof(SPI_Res_Get_Power_Header);
    }  
    return 0;
}

IRAM_ATTR void process_spi_transaction(spi_slave_transaction_t* trans)
{
    //LOG("SPI done %d: %d / %d\n", (int)trans->user, trans->length / 8, trans->trans_len / 8);

    size_t transfer_size = trans->trans_len >> 3;
    if (transfer_size < sizeof(SPI_Req_Base_Header))
    {
        LOG("SPI error: transfer too small: %d\n", transfer_size);
        s_stats.spi_error_count++;
        setup_spi_packet_response(0, 0);
        return;
    }

    s_stats.spi_data_received += transfer_size;

    size_t header_size = get_header_size(s_spi_rx_buffer);
    if (header_size == 0)
    {
        LOG("SPI error: unknown header\n");
        s_stats.spi_error_count++;
        setup_spi_packet_response(0, 0);
        return;
    }

    SPI_Req_Base_Header& base_header = *reinterpret_cast<SPI_Req_Base_Header*>(s_spi_rx_buffer);
    SPI_Req req = static_cast<SPI_Req>(base_header.req);
    uint8_t crc = base_header.crc;

    base_header.crc = 0;
    uint8_t computed_crc = crc8(0, s_spi_rx_buffer, header_size);
    if (crc != computed_crc)
    {
        LOG("Crc error: %d != %d\n", crc, computed_crc);
        s_stats.spi_error_count++;
        setup_spi_packet_response(0, 0);
        return;
    }

    //LOG("spi header received: %d\n", transfer_size);
    if (req == SPI_Req::PACKET)
    {
        //LOG("PACKET\n");
        SPI_Req_Packet_Header& req_header = *reinterpret_cast<SPI_Req_Packet_Header*>(s_spi_rx_buffer);
        if (req_header.packet_size > 0)
        {
            //LOG("packet %d", req_header.packet_size);
            if (transfer_size < req_header.packet_size + sizeof(req_header))
            {
                LOG("Not enough data: %d < %d\n", transfer_size, req_header.packet_size + sizeof(req_header));
                s_stats.spi_error_count++;
            }
            else if (req_header.packet_size > WLAN_MAX_PAYLOAD_SIZE)
            {
                LOG("Too much data: %d, %d\n", req_header.packet_size, WLAN_MAX_PAYLOAD_SIZE);
                s_stats.spi_error_count++;
            }
            else
            {
                if (req_header.use_fec)
                {
                    if (!s_fec_codec.is_initialized())
                    {
                        LOG("Uninitialized fec codec\n");
                        s_stats.spi_error_count++;
                    }
                    else 
                    {
                        portENTER_CRITICAL_ISR(&s_fec_codec_mux);
                        if (!s_fec_codec.encode_data(s_spi_rx_buffer + sizeof(req_header), req_header.packet_size, true, false))
                        {
                            LOG("Fec codec busy\n");
                            s_stats.spi_error_count++;
                        }
                        portEXIT_CRITICAL_ISR(&s_fec_codec_mux);
                    }
                }
                else
                {
                    Wlan_Packet_Header packet_header;
                    packet_header.uses_fec = 0;
                    add_to_wlan_outgoing_queue(packet_header, s_spi_rx_buffer + sizeof(req_header), req_header.packet_size, true);
                }
            }
        }
        setup_spi_packet_response(transfer_size, req_header.seq);
        return;
    }

    if (req == SPI_Req::SETUP_FEC_CODEC)
    {
        LOG("SETUP_FEC_CODEC\n");

        SPI_Req_Setup_Fec_Codec_Header& req_header = *reinterpret_cast<SPI_Req_Setup_Fec_Codec_Header*>(s_spi_rx_buffer);

        Fec_Codec::Descriptor descriptor;
        descriptor.coding_k = req_header.fec_coding_k;
        descriptor.coding_n = req_header.fec_coding_n;
        descriptor.mtu = req_header.fec_mtu;
        descriptor.encoder_core = Fec_Codec::Core::Core_0;
        descriptor.decoder_core = Fec_Codec::Core::Core_0;
        descriptor.encoder_priority = 1;
        descriptor.decoder_priority = 1;

        if (descriptor.coding_k > descriptor.coding_n || descriptor.mtu < 32)
        {
            LOG("Bad fec params");
            s_stats.spi_error_count++;
        }
        else 
        {
            portENTER_CRITICAL_ISR(&s_fec_codec_mux);
            if (!s_fec_codec.init(descriptor))
            {
                LOG("Failed to init fec codec");
                s_stats.spi_error_count++;
            }
            else
            {
                s_fec_codec.set_data_encoded_cb(&fec_encoded_cb);
                s_fec_codec.set_data_decoded_cb(&fec_decoded_cb);
            }
            portEXIT_CRITICAL_ISR(&s_fec_codec_mux);
        }

        SPI_Res_Setup_Fec_Codec_Header& res_header = *reinterpret_cast<SPI_Res_Setup_Fec_Codec_Header*>(s_spi_tx_buffer);
        res_header.res = static_cast<uint8_t>(SPI_Res::SETUP_FEC_CODEC);
        res_header.seq = req_header.seq & 0x7F;
        descriptor = s_fec_codec.get_descriptor();
        res_header.fec_coding_k = descriptor.coding_k;
        res_header.fec_coding_n = descriptor.coding_n;
        res_header.fec_mtu = descriptor.mtu;

        setup_spi_base_response(sizeof(SPI_Res_Setup_Fec_Codec_Header));
        return;
    }
    if (req == SPI_Req::SET_RATE)
    {
        LOG("SET_RATE\n");

        SPI_Req_Set_Rate_Header& req_header = *reinterpret_cast<SPI_Req_Set_Rate_Header*>(s_spi_rx_buffer);

        LOG("Setting rate: %d\n", (int)req_header.rate);
        if (set_wifi_fixed_rate(req_header.rate) != ESP_OK)
        {
            LOG("Failed to set rate %d", (int)req_header.rate);
            s_stats.spi_error_count++;
        }

        SPI_Res_Set_Rate_Header& res_header = *reinterpret_cast<SPI_Res_Set_Rate_Header*>(s_spi_tx_buffer);
        res_header.res = static_cast<uint8_t>(SPI_Res::SET_RATE);
        res_header.seq = req_header.seq & 0x7F;
        res_header.rate = get_wifi_fixed_rate();

        setup_spi_base_response(sizeof(SPI_Res_Set_Rate_Header));
        return;
    }
    if (req == SPI_Req::GET_RATE)
    {
        LOG("GET_RATE\n");

        SPI_Res_Get_Rate_Header& res_header = *reinterpret_cast<SPI_Res_Get_Rate_Header*>(s_spi_tx_buffer);
        res_header.res = static_cast<uint8_t>(SPI_Res::GET_RATE);
        res_header.seq = base_header.seq & 0x7F;
        res_header.rate = get_wifi_fixed_rate();

        setup_spi_base_response(sizeof(SPI_Res_Get_Rate_Header));
        return;
    }
    if (req == SPI_Req::SET_CHANNEL)
    {
        LOG("SET_CHANNEL\n");

        SPI_Req_Set_Channel_Header& req_header = *reinterpret_cast<SPI_Req_Set_Channel_Header*>(s_spi_rx_buffer);

        SPI_Res_Set_Channel_Header& res_header = *reinterpret_cast<SPI_Res_Set_Channel_Header*>(s_spi_tx_buffer);
        res_header.res = static_cast<uint8_t>(SPI_Res::SET_CHANNEL);
        res_header.seq = req_header.seq & 0x7F;

        LOG("Setting channel: %d\n", (int)req_header.channel);
        if (esp_wifi_set_channel(req_header.channel, WIFI_SECOND_CHAN_NONE) != ESP_OK)
        {
            LOG("Failed to set channel %d", (int)req_header.channel);
            res_header.channel = 0;
            s_stats.spi_error_count++;
        }
        else
        {
            res_header.channel = req_header.channel;
        }

        setup_spi_base_response(sizeof(SPI_Res_Set_Channel_Header));
        return;
    }
    if (req == SPI_Req::GET_CHANNEL)
    {
        LOG("GET_RATE\n");

        SPI_Res_Get_Channel_Header& res_header = *reinterpret_cast<SPI_Res_Get_Channel_Header*>(s_spi_tx_buffer);
        res_header.res = static_cast<uint8_t>(SPI_Res::GET_CHANNEL);
        res_header.seq = base_header.seq & 0x7F;
        res_header.channel = 0;
        setup_spi_base_response(sizeof(SPI_Res_Set_Channel_Header));
        return;
    }
    if (req == SPI_Req::SET_POWER)
    {
        LOG("SET_POWER\n");

        SPI_Req_Set_Power_Header& req_header = *reinterpret_cast<SPI_Req_Set_Power_Header*>(s_spi_rx_buffer);

        float power = static_cast<float>(req_header.power) / 10.f;
        if (set_wlan_power_dBm(power) != ESP_OK)
        {
            LOG("Failed to set power %f", power);
            s_stats.spi_error_count++;
        }
        else
        {
            LOG("Setting power: %f (real %f)\n", power, get_wlan_power_dBm());
        }

        SPI_Res_Set_Power_Header& res_header = *reinterpret_cast<SPI_Res_Set_Power_Header*>(s_spi_tx_buffer);
        res_header.res = static_cast<uint8_t>(SPI_Res::SET_POWER);
        res_header.seq = req_header.seq & 0x7F;
        res_header.power = static_cast<int16_t>(get_wlan_power_dBm() * 10.f);

        setup_spi_base_response(sizeof(SPI_Res_Set_Power_Header));
        return;
    }
    if (req == SPI_Req::GET_POWER)
    {
        LOG("GET_POWER\n");

        SPI_Res_Get_Power_Header& res_header = *reinterpret_cast<SPI_Res_Get_Power_Header*>(s_spi_tx_buffer);
        res_header.res = static_cast<uint8_t>(SPI_Res::GET_POWER);
        res_header.seq = base_header.seq & 0x7F;
        res_header.power = static_cast<int16_t>(get_wlan_power_dBm() * 10.f);

        setup_spi_base_response(sizeof(SPI_Res_Get_Power_Header));
        return;
    }
    LOG("Unknown req: %d\n", (int)req);
}

/////////////////////////////////////////////////////////////////////////

#if 0
#define ESP_WIFI_MODE WIFI_MODE_STA
#define ESP_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESP_WIFI_MODE WIFI_MODE_AP
#define ESP_WIFI_IF   ESP_IF_WIFI_AP
#endif

void setup() 
{
    init_crc8_table();

    Serial.begin(115200);
    Serial.setTimeout(999999);

    srand(millis());

    Serial.printf("Initializing...\n");

    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) 
    {
        // app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    init_fec();
    initialize_status_led();

    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    /*    Fec_Codec::Descriptor descriptor;
    descriptor.mtu = 1400;
    //descriptor.encoder_core = Fec_Codec::Core::Core_1;
    //descriptor.decoder_core = Fec_Codec::Core::Core_1;

    descriptor.coding_k = 2;
    descriptor.coding_n = 3;
    if (!s_fec_codec.init(descriptor))
    {
        Serial.printf("Failed to initialize fec\n");
    }
    test_fec_encoding();
    test_fec_decoding();
    descriptor.coding_k = 4;
    descriptor.coding_n = 6;
    if (!s_fec_codec.init(descriptor))
    {
        Serial.printf("Failed to initialize fec\n");
    }
    test_fec_encoding();
    test_fec_decoding();
    descriptor.coding_k = 6;
    descriptor.coding_n = 9;
    if (!s_fec_codec.init(descriptor))
    {
        Serial.printf("Failed to initialize fec\n");
    }
    test_fec_encoding();
    test_fec_decoding();
    descriptor.coding_k = 8;
    descriptor.coding_n = 12;
    if (!s_fec_codec.init(descriptor))
    {
        Serial.printf("Failed to initialize fec\n");
    }
    test_fec_encoding();
    test_fec_decoding();
*/
    

    //bool ok = false;

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init_internal(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESP_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(init_spi());

    setup_spi_initial_response();

    ESP_ERROR_CHECK( esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE) );

    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(packet_received_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    //set_wlan_power_dBm(20.5f);

    esp_log_level_set("*", ESP_LOG_NONE);

    set_wifi_fixed_rate(30);

    //S012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789cacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacaca

    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    Serial.printf("Initialized\n");
}

Wlan_Outgoing_Packet s_outgoing_wlan_packet;
IRAM_ATTR void loop() 
{
  /*
    static bool xxx = false;
    if (xxx)
    {
      digitalWrite(STATUS_LED_PIN, STATUS_LED_ON);
    }
    else
    {
      digitalWrite(STATUS_LED_PIN, STATUS_LED_OFF);
    }
    xxx = !xxx;
    delay(1000);
    */
    
    parse_command();

    //send pending wlan packets
    if (!s_outgoing_wlan_packet.ptr)
    {
        portENTER_CRITICAL(&s_wlan_outgoing_mux);
        start_reading_wlan_outgoing_packet(s_outgoing_wlan_packet);
        portEXIT_CRITICAL(&s_wlan_outgoing_mux);
        
        if (s_outgoing_wlan_packet.ptr)
        {
            memcpy(s_outgoing_wlan_packet.ptr, s_wlan_ieee_header, WLAN_IEEE_HEADER_SIZE);
        }
    }

    if (s_outgoing_wlan_packet.ptr)
    {
        //LOG("packet %d", p.size);
        esp_err_t res = esp_wifi_80211_tx(ESP_WIFI_IF, s_outgoing_wlan_packet.ptr, WLAN_IEEE_HEADER_SIZE + s_outgoing_wlan_packet.size, false);
        if (res == ESP_OK)
        {
            //delay(15);
            s_stats.wlan_data_sent += s_outgoing_wlan_packet.size;
            set_status_led_on();
            //LOG("WLAN inject OKKK\n");
            portENTER_CRITICAL(&s_wlan_outgoing_mux);
            end_reading_wlan_outgoing_packet(s_outgoing_wlan_packet);
            portEXIT_CRITICAL(&s_wlan_outgoing_mux);
        }
        else
        {
            //LOG("WLAN inject error: %d\n", res);
            s_stats.wlan_error_count++;
        }
    }

    update_status_led();

    {
        spi_slave_transaction_t* qt = nullptr;
        spi_slave_get_trans_result(VSPI_HOST, &qt, 0);
        if (qt)
        {
            process_spi_transaction(qt);

            //char xxx[128];
            //memcpy(xxx, s_spi_rx_buf, qt->length / 8);
            //xxx[qt->length / 8] = 0;
            //static int xxx = 0;
            //xxx++;
            //s_stats.spi_data_received += qt->trans_len / 8;
            //LOG("SPI confirmed %d: %d: %s\n", (int)qt->user, qt->length / 8, xxx);
        }
    }

    /*if (s_outgoing_wlan_packet.ptr)
  {
    esp_err_t res = esp_wifi_80211_tx(ESP_WIFI_IF, s_outgoing_wlan_packet.ptr, HEADER_SIZE + s_outgoing_wlan_packet.size, false);
    if (res == ESP_OK)
    {
      s_stats.wlan_data_sent += s_outgoing_wlan_packet.size;
      digitalWrite(LED_BUILTIN, LED_ON);
      //LOG("WLAN inject OKKK\n");
    }
    else
    {
      //LOG("WLAN inject error: %d\n", res);
      s_stats.wlan_error_count++;
    }
    //delayMicroseconds(300);
  }
*/

    if (s_uart_verbose > 0 && millis() - s_stats_last_tp >= 1000)
    {
        s_stats_last_tp = millis();
        //    Serial.printf("Sent: %d bytes ec:%d, Received: %d bytes, SPI SS: %d, SPI SR: %d, SPI DS: %d, SPI DR: %d, SPI ERR: %d, SPI PD: %d\n", s_sent, s_send_error_count, s_received, s_spi_status_sent, s_spi_status_received, s_spi_data_sent, s_spi_data_received, s_spi_error_count, s_spi_packets_dropped);
        Serial.printf("WLAN S: %d, R: %d, E: %d, D: %d, %%: %d  SPI S: %d, R: %d, E: %d, D: %d, %%: %d\n",
                      s_stats.wlan_data_sent, s_stats.wlan_data_received, s_stats.wlan_error_count, s_stats.wlan_received_packets_dropped, s_wlan_outgoing_queue.size() * 100 / s_wlan_outgoing_queue.capacity(),
                      s_stats.spi_data_sent, s_stats.spi_data_received, s_stats.spi_error_count, s_stats.spi_received_packets_dropped, s_wlan_incoming_queue.size() * 100 / s_wlan_incoming_queue.capacity());

        s_stats = Stats();

        //Serial.printf("Sent: %d bytes, min %dms, max %dms, ec: %d\n", s_sent, s_send_min_time, s_send_max_time, s_send_error_count);
        //s_sent = 0;
        //s_send_max_time = -999999;
        //s_send_min_time = 999999;
        //s_send_error_count = 0;
    }
    //*/

    taskYIELD();
    esp_task_wdt_feed();

    //Serial.printf("Call to wifi_set_channel failed\n");
}
