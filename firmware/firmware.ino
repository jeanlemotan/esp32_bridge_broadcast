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

static int s_uart_verbose = 1;
static char s_uart_command = 0;
static int s_uart_error_count = 0;

#define LOG(...) if (s_uart_verbose > 0) Serial.printf(__VA_ARGS__)

/////////////////////////////////////////////////////////////////////////

static constexpr uint8_t STATUS_LED_PIN = 2;
static constexpr uint8_t STATUS_LED_ON = HIGH;
static constexpr uint8_t STATUS_LED_OFF = LOW;
static constexpr uint32_t STATUS_LED_ON_DURATION = 30; //ms
static uint32_t s_status_led_tp = 0;

void initialize_status_led()
{
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, STATUS_LED_OFF);
    s_status_led_tp = 0;
}

void set_status_led_on()
{
    digitalWrite(STATUS_LED_PIN, STATUS_LED_ON);
    s_status_led_tp = millis();
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

Fec_Codec s_fec_codec;

/////////////////////////////////////////////////////////////////////////

Wlan_Outgoing_Packet s_outgoing_wlan_packet;
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
    wifi_internal_rate_t rate;
    s_wlan_rate = value <= 30 ? value : 30;
    rate.fix_rate = s_rate_mapping[s_wlan_rate];
    return esp_wifi_internal_set_rate(100, 1, 4, &rate);
}

uint8_t get_wifi_fixed_rate()
{
    return s_wlan_rate;
}

IRAM_ATTR esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

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

  if (len <= WLAN_HEADER_SIZE)
  {
    LOG("WLAN receive header error");
    s_stats.wlan_error_count++;
    return;
  }

  //Serial.printf("Recv callback #%d: %d bytes\n", counter++, len);
  //Serial.printf("Channel: %d PHY: %d\n", pkt->rx_ctl.channel, wifi_get_phy_mode());

  //uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  //Serial.printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  uint8_t* data = pkt->payload;
  if (memcmp(data + 10, s_wlan_packet_header + 10, 6) != 0)
  {
    return;
  }

  data += WLAN_HEADER_SIZE;
  len -= WLAN_HEADER_SIZE; //skip the 802.11 header

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
/*
  uint8_t* buffer = nullptr;
  {
    lock_guard lg;
    buffer = s_wlan_incoming_queue.start_writing(size);
    //xxx p = s_spi_free_queue.pop();
  }
  
  if (buffer)
  {
    memcpy(buffer, data, size);
    //p->rssi = rssi;
    
    {
      lock_guard lg;
      s_wlan_incoming_queue.end_writing();
      //xxx s_spi_to_send_queue.push(p);
    }
  }
  else
  {
    s_stats.wlan_received_packets_dropped++;
  }
*/  
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

    lock_guard lg;

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
                start_writing_wlan_outgoing_packet(packet, s_uart_offset);
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
                end_writing_wlan_outgoing_packet(packet);
                s_uart_command = 0;
                s_uart_offset = 0;

                if (s_outgoing_wlan_packet.ptr)
                {
                    end_reading_wlan_outgoing_packet(s_outgoing_wlan_packet);
                }
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
        /*    if (wifi_set_channel(channel))
    {
      LOG("Channel set to %d\n", channel);
    }
    else
    {
      LOG("Command error: call to wifi_set_channel failed\n");
    }
*/  }
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

uint8_t* s_spi_tx_buffer = nullptr;
uint8_t* s_spi_rx_buffer = nullptr;
uint8_t* s_spi_rx_payload_ptr = nullptr;
int s_spi_transaction_id = 0;

IRAM_ATTR void spi_post_setup_cb(spi_slave_transaction_t* trans)
{
//    LOG("SPI armed %d: %d\n", (int)trans->user, trans->length / 8);
}

IRAM_ATTR void spi_post_trans_cb(spi_slave_transaction_t* trans)
{
//    LOG("SPI done %d: %d / %d\n", (int)trans->user, trans->length / 8, trans->trans_len / 8);
}

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

IRAM_ATTR void setup_spi_response(SPI_Base_Response* response, size_t size,const void* payload_ptr, size_t payload_size, bool last_command_ok)
{
    assert(size >= sizeof(SPI_Base_Response) && size <= MAX_SPI_BUFFER_SIZE);
    
    static spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));

    response->size = size + payload_size;
    response->last_command_ok = last_command_ok ? 1 : 0;
    response->pending_packets = s_wlan_incoming_queue.count();
    response->next_packet_size = s_wlan_incoming_queue.next_reading_size();
    response->crc = 0;
    response->crc = crc8(0, reinterpret_cast<const uint8_t*>(response), size);
    memcpy(s_spi_tx_buffer, response, size);

    if (payload_size > 0)
    {
        memcpy(s_spi_tx_buffer + size, payload_ptr, payload_size);
    }

    t.length = MAX_SPI_BUFFER_SIZE * 8;//(size + payload_size) * 8;
    t.tx_buffer = s_spi_tx_buffer;
    t.rx_buffer = s_spi_rx_buffer;
    esp_err_t err = spi_slave_queue_trans(VSPI_HOST, &t, 0);
    ESP_ERROR_CHECK(err);
}


IRAM_ATTR void setup_spi_base_response(bool last_command_ok)
{
    SPI_Base_Response response;
    setup_spi_response(&response, sizeof(response), nullptr, 0, last_command_ok);
}

IRAM_ATTR void process_spi_transaction(spi_slave_transaction_t* trans)
{
    //LOG("SPI done %d: %d / %d\n", (int)trans->user, trans->length / 8, trans->trans_len / 8);

    {
        size_t transfer_size = trans->trans_len >> 3;
        if (transfer_size < sizeof(SPI_Base_Header))
        {
          LOG("SPI error: transfer too small: %d\n", transfer_size);
          goto error;
        }
    
        s_stats.spi_data_received += transfer_size;
    
        SPI_Base_Header& base_header = *reinterpret_cast<SPI_Base_Header*>(s_spi_rx_buffer);
        SPI_Command command = static_cast<SPI_Command>(base_header.command);
        uint8_t crc = base_header.crc;
    
        //LOG("spi header received: %d\n", transfer_size);
        if (command == SPI_Command::SPI_CMD_SEND_PACKET)
        {
            SPI_Send_Packet_Header& header = *reinterpret_cast<SPI_Send_Packet_Header*>(s_spi_rx_buffer);
            header.crc = 0;
            uint8_t computed_crc = crc8(0, reinterpret_cast<const uint8_t*>(&header), sizeof(header));
            if (crc != computed_crc)
            {
                LOG("Crc error: %d != %d\n", crc, computed_crc);
                goto error;
            }
    
            if (transfer_size < header.size + sizeof(header))
            {
                LOG("Not enough data: %d < %d\n", transfer_size, header.size + sizeof(header));
                goto error;
            }
            if (header.size > WLAN_MAX_PAYLOAD_SIZE)
            {
                LOG("Too much data: %d, %d\n", header.size, WLAN_MAX_PAYLOAD_SIZE);
                goto error;
            }    
            Fec_Codec& codec = s_fec_codec;
            if (!codec.is_initialized())
            {
                LOG("Uninitialized fec codec\n");
                goto error;
            }
            if (!codec.encode_data(s_spi_rx_buffer + sizeof(header), header.size, false))
            {
                LOG("Fec codec busy\n");
                goto error;
            }
            setup_spi_base_response(true);
            goto done;
        }
    
        if (command == SPI_Command::SPI_CMD_SETUP_FEC_CODEC)
        {
            LOG("SPI_CMD_SETUP_FEC_CODEC\n");
          
            SPI_Setup_Fec_Codec_Header& header = *reinterpret_cast<SPI_Setup_Fec_Codec_Header*>(s_spi_rx_buffer);
            header.crc = 0;
            uint8_t computed_crc = crc8(0, reinterpret_cast<const uint8_t*>(&header), sizeof(header));
            if (crc != computed_crc)
            {
                LOG("Crc error: %d != %d\n", crc, computed_crc);
                goto error;
            }
    
            Fec_Codec::Descriptor descriptor;
            descriptor.coding_k = header.fec_coding_k;
            descriptor.coding_n = header.fec_coding_n;
            descriptor.mtu = header.fec_mtu;
            if (descriptor.coding_k > descriptor.coding_n || descriptor.mtu < 32)
            {
                LOG("Bad fec params");
                goto error;
            }
    
            if (!s_fec_codec.init(descriptor))
            {
                LOG("Failed to init fec codec");
                goto error;
            }
            
            setup_spi_base_response(true);
            goto done;
        }
    
        if (command == SPI_Command::SPI_CMD_GET_PACKET)
        {
            LOG("SPI_CMD_GET_PACKET\n");
          
            SPI_Get_Packet_Header& header = *reinterpret_cast<SPI_Get_Packet_Header*>(s_spi_rx_buffer);
            header.crc = 0;
            uint8_t computed_crc = crc8(0, reinterpret_cast<const uint8_t*>(&header), sizeof(header));
            if (crc != computed_crc)
            {
                LOG("Crc error: %d != %d\n", crc, computed_crc);
                goto error;
            }

            Wlan_Incoming_Packet packet;
            if (!start_reading_wlan_incoming_packet(packet))
            {
                LOG("No incoming packet\n");
                goto error;
            }

            SPI_Get_Packet_Response_Header response;
            response.rssi = packet.rssi;
            setup_spi_response(&response, sizeof(SPI_Base_Response), packet.ptr, packet.size, true);
            goto done;
        }
    
        if (command == SPI_Command::SPI_CMD_SET_RATE)
        {
            LOG("SPI_CMD_SET_RATE\n");
          
            SPI_Set_Rate_Header& header = *reinterpret_cast<SPI_Set_Rate_Header*>(s_spi_rx_buffer);
            header.crc = 0;
            uint8_t computed_crc = crc8(0, reinterpret_cast<const uint8_t*>(&header), sizeof(header));
            if (crc != computed_crc)
            {
                LOG("Crc error: %d != %d\n", crc, computed_crc);
                goto error;
            }
          
            LOG("Setting rate: %d\n", (int)header.rate);
            if (set_wifi_fixed_rate(header.rate) != ESP_OK)
            {
                LOG("Failed to set rate %d", (int)header.rate);
                goto error;
            }
            setup_spi_base_response(true);
            goto done;
        }
        else if (command == SPI_Command::SPI_CMD_GET_RATE)
        {
            LOG("SPI_CMD_GET_RATE\n");
          
            base_header.crc = 0;
            uint8_t computed_crc = crc8(0, reinterpret_cast<const uint8_t*>(&base_header), sizeof(base_header));
            if (crc != computed_crc)
            {
                LOG("Crc error: %d != %d\n", crc, computed_crc);
                goto error;
            }

            SPI_Get_Rate_Response_Header response;
            response.rate = get_wifi_fixed_rate();
            setup_spi_response(&response, sizeof(response), nullptr, 0, true);
            goto done;
        }
        else if (command == SPI_Command::SPI_CMD_SET_CHANNEL)
        {
            LOG("SPI_CMD_SET_CHANNEL\n");
          
            SPI_Set_Channel_Header& header = *reinterpret_cast<SPI_Set_Channel_Header*>(s_spi_rx_buffer);
            header.crc = 0;
            uint8_t computed_crc = crc8(0, reinterpret_cast<const uint8_t*>(&header), sizeof(header));
            if (crc != computed_crc)
            {
                LOG("Crc error: %d != %d\n", crc, computed_crc);
                goto error;
            }
          
            LOG("Setting channel: %d\n", (int)header.channel);
            if (esp_wifi_set_channel(header.channel, WIFI_SECOND_CHAN_NONE) != ESP_OK)
            {
                LOG("Failed to set channel %d", (int)header.channel);
                goto error;
            }
            setup_spi_base_response(true);
            goto done;
        }
        else if (command == SPI_Command::SPI_CMD_GET_CHANNEL)
        {
            LOG("SPI_CMD_GET_RATE\n");
          
            base_header.crc = 0;
            uint8_t computed_crc = crc8(0, reinterpret_cast<const uint8_t*>(&base_header), sizeof(base_header));
            if (crc != computed_crc)
            {
                LOG("Crc error: %d != %d\n", crc, computed_crc);
                goto error;
            }
            SPI_Get_Channel_Response_Header response;
            response.channel = 0;
            setup_spi_response(&response, sizeof(response), nullptr, 0, true);
            goto done;
        }
        else if (command == SPI_Command::SPI_CMD_SET_POWER)
        {
            LOG("SPI_CMD_SET_POWER\n");
          
            SPI_Set_Power_Header& header = *reinterpret_cast<SPI_Set_Power_Header*>(s_spi_rx_buffer);
            header.crc = 0;
            uint8_t computed_crc = crc8(0, reinterpret_cast<const uint8_t*>(&header), sizeof(header));
            if (crc != computed_crc)
            {
                LOG("Crc error: %d != %d\n", crc, computed_crc);
                goto error;
            }
    
            float power = static_cast<float>(header.power) / 10.f - 100.f;
            if (set_wlan_power_dBm(power) != ESP_OK)
            {
                LOG("Failed to set power %f", power);
                goto error;
            }
            LOG("Setting power: %f (real %f)\n", power, get_wlan_power_dBm());
            setup_spi_base_response(true);
            goto done;
        }
        else if (command == SPI_Command::SPI_CMD_GET_POWER)
        {
            LOG("SPI_CMD_GET_POWER\n");
          
            base_header.crc = 0;
            uint8_t computed_crc = crc8(0, reinterpret_cast<const uint8_t*>(&base_header), sizeof(base_header));
            if (crc != computed_crc)
            {
                LOG("Crc error: %d != %d\n", crc, computed_crc);
                goto error;
            }
            SPI_Get_Power_Response_Header response;
            response.power = static_cast<uint16_t>((get_wlan_power_dBm() + 100.f) * 10.f);
            setup_spi_response(&response, sizeof(response), nullptr, 0, true);
            goto done;
        }
        else if (command == SPI_Command::SPI_CMD_GET_STATS)
        {
            LOG("Get stats\n");
            uint32_t data[8] = { 0 };
            memcpy(data, &s_stats, sizeof(Stats));
            //    spi_slave_set_data(data);
            goto error;
        }
        else
        {
            LOG("Unknown command: %d\n", command);
        }
    }
    
error:
    s_stats.spi_error_count++;
    setup_spi_base_response(false);

done:
  ;
}

/////////////////////////////////////////////////////////////////////////

IRAM_ATTR void fec_encoded_cb(void* data, size_t size)
{
}

IRAM_ATTR void fec_decoded_cb(void* data, size_t size)
{
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

    s_fec_codec.set_data_encoded_cb(&fec_encoded_cb);
    s_fec_codec.set_data_decoded_cb(&fec_decoded_cb);
    
/*    Fec_Codec::Descriptor descriptor;
    descriptor.mtu = 1400;
    //descriptor.encoder_core = Fec_Codec::Core::Core_1;
    //descriptor.decoder_core = Fec_Codec::Core::Core_1;

    descriptor.coding_k = 2;
    descriptor.coding_n = 3;
    initialize_status_led();
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
    

    bool ok = false;

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init_internal(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESP_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(init_spi());

    setup_spi_base_response(true);

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
    

    Serial.printf("Initialized\n");
}

IRAM_ATTR void loop() 
{
    parse_command();

    

    //send pending wlan packets
    Wlan_Outgoing_Packet p;
    if (!s_outgoing_wlan_packet.ptr)
    {
        lock_guard lg;
        if (!s_outgoing_wlan_packet.ptr)
        {
            start_reading_wlan_outgoing_packet(s_outgoing_wlan_packet);
            p = s_outgoing_wlan_packet;
            if (p.ptr)
            {
              memcpy(p.ptr, s_wlan_packet_header, WLAN_HEADER_SIZE);
            }
        }
    }
    else
    {
      p = s_outgoing_wlan_packet;
    }

    if (p.ptr)
    {
        esp_err_t res = esp_wifi_80211_tx(ESP_WIFI_IF, p.ptr, WLAN_HEADER_SIZE + p.size, false);
        if (res == ESP_OK)
        {
            //delay(15);
            s_stats.wlan_data_sent += s_outgoing_wlan_packet.size;
            set_status_led_on();
            //LOG("WLAN inject OKKK\n");
            lock_guard lg;
            end_reading_wlan_outgoing_packet(s_outgoing_wlan_packet);
        }
        else
        {
            //LOG("WLAN inject error: %d\n", res);
            s_stats.wlan_error_count++;
            lock_guard lg;
            cancel_reading_wlan_outgoing_packet(s_outgoing_wlan_packet);
        }
    }

    update_status_led();

    {
        spi_slave_transaction_t* qt = nullptr;
        esp_err_t err = spi_slave_get_trans_result(VSPI_HOST, &qt, 0);
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

    if (/*s_uart_verbose > 0 && */millis() - s_stats_last_tp >= 1000)
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

    //Serial.printf("Call to wifi_set_channel failed\n");
}
