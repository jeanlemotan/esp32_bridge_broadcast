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

static constexpr uint8_t s_filtered_mac[] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, };

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

void update_status_led()
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

Fec_Codec s_fec_codec;
size_t s_fec_encoded_data_size = 0;
size_t s_fec_decoded_data_size = 0;

void fec_encoded_cb(void* data, size_t size)
{
    s_fec_encoded_data_size += size;
}
void fec_decoded_cb(void* data, size_t size)
{
    s_fec_decoded_data_size += size;
}

void test_fec_encoding()
{
    s_fec_encoded_data_size = 0;
    s_fec_decoded_data_size = 0;
    s_fec_codec.set_data_encoded_cb(&fec_encoded_cb);
    s_fec_codec.set_data_decoded_cb(&fec_decoded_cb);

    LOG("starting test\n");

    uint8_t data[128] = { 0 };

    uint32_t start_tp = millis();

    size_t iteration = 0;
    size_t fec_data_in = 0;
    //encode
    while (millis() - start_tp < 1000)
    {
//        Serial.printf("Encoding %d\n", iteration);

        if (!s_fec_codec.encode_data(data, sizeof(data), true))
        {
            Serial.printf("Failed to encode\n");
            return;
        }
        fec_data_in += sizeof(data);
        //LOG("Pass %d %dms\n", i, millis() - start_pass_tp);

        iteration++;
    }
    float ds = 1.f;//d / 1000.f;
    float total_data_in = fec_data_in / 1024.f;
    float total_data_out = s_fec_encoded_data_size / 1024.f;
    LOG("Total IN: %.2fKB, %.2fKB/s, OUT: %.2fKB, %.2fKB/s\n", total_data_in, total_data_in / ds, total_data_out, total_data_out / ds);
}

volatile size_t xxx = 0;
void fec_encoded2_cb(void* data, size_t size)
{
    s_fec_encoded_data_size += size;

    //if (rand() > RAND_MAX / 4)
    xxx++;

    size_t n = s_fec_codec.get_descriptor().coding_n;
    if ((xxx % n) < n * 75 / 100)
    {
        s_fec_codec.decode_data(data, size, true);
    }
    else
    {
        //LOG("Skipped packet %d\n", xxx);
    }
}

void test_fec_decoding()
{
    s_fec_encoded_data_size = 0;
    s_fec_decoded_data_size = 0;
    s_fec_codec.set_data_encoded_cb(&fec_encoded2_cb);
    s_fec_codec.set_data_decoded_cb(&fec_decoded_cb);

    LOG("starting test\n");

    uint8_t data[128] = { 0 };

    uint32_t start_tp = millis();

    size_t iteration = 0;
    size_t fec_data_in = 0;
    //encode
    while (millis() - start_tp < 1000)
    {
        //Serial.printf("Encoding %d\n", iteration);

        if (!s_fec_codec.encode_data(data, sizeof(data), true))
        {
            Serial.printf("Failed to encode\n");
            return;
        }
        fec_data_in += sizeof(data);
        //LOG("Pass %d %dms\n", i, millis() - start_pass_tp);

        iteration++;
    }
    float ds = 1.f;//d / 1000.f;
    float total_data_in = fec_data_in / 1024.f;
    float total_data_encoded = s_fec_encoded_data_size / 1024.f;
    float total_data_decoded = s_fec_decoded_data_size / 1024.f;
    LOG("Total IN: %.2fKB, %.2fKB/s, ENCODED: %.2fKB, %.2fKB/s, DECODED: %.2fKB, %.2fKB/s\n", total_data_in, total_data_in / ds, total_data_encoded, total_data_encoded / ds, total_data_decoded, total_data_decoded / ds);
}

/////////////////////////////////////////////////////////////////////////

S2W_Packet s_wlan_packet;
float s_wlan_power_dBm = 0;

void set_wlan_power_dBm(float dBm)
{
    dBm = std::max(std::min(dBm, 20.5f), 0.f);
    s_wlan_power_dBm = dBm;
    //system_phy_set_max_tpw(static_cast<uint8_t>(dBm * 4.f));
}

float get_wlan_power_dBm()
{
    return s_wlan_power_dBm;
}

esp_err_t set_wifi_fixed_rate(uint8_t value)
{
    //https://github.com/espressif/esp-idf/issues/833
    wifi_internal_rate_t rate;
    rate.fix_rate = value;
    return esp_wifi_internal_set_rate(100, 1, 4, &rate);
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void packet_sent_cb(uint8_t status)
{
    {
        lock_guard lg;

        if (s_wlan_packet.ptr)
        {
            if (status == 0)
            {
                //int dt = micros() - s_send_start_time;
                //s_send_max_time = std::max(s_send_max_time, dt);
                //s_send_min_time = std::min(s_send_min_time, dt);
                s_stats.wlan_data_sent += s_wlan_packet.size + WLAN_HEADER_SIZE;
            }
            else
            {
                LOG("WLAN send error");
                s_stats.wlan_error_count++;
            }
            end_reading_s2w_packet(s_wlan_packet);
            //xxx s_wlan_free_queue.push_and_clear(s_wlan_packet);
        }
        else
        {
            LOG("WLAN send missing packet");
            s_stats.wlan_error_count++;
        }
    }
}

void packet_received_cb(void* buf, wifi_promiscuous_pkt_type_t type)
{
    /*  if (type == WIFI_PKT_MGMT)
  {
    Serial.printf("management packet\n");
  }
  else if (type == WIFI_PKT_DATA)
  {
    Serial.printf("data packet\n");
  }
  else if (type == WIFI_PKT_MISC)
  {
    Serial.printf("misc packet\n");
  }

  wifi_promiscuous_pkt_t* pkt = reinterpret_cast<wifi_promiscuous_pkt_t*>(buf);
  
  uint16_t len = pkt->rx_ctrl.sig_len;
  if (len <= HEADER_SIZE)
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
//  if (memcmp(data + 10, s_filtered_mac, 6) != 0)
//  {
//    return;
//  }

  data += HEADER_SIZE;
  len -= HEADER_SIZE; //skip the 802.11 header

  len -= 4;//the received length has 4 more bytes at the end for some reason.

  int16_t rssi = pkt->rx_ctrl.rssi;
  if (s_uart_verbose >= 1)
  {
    Serial.printf("RSSI: %d, CH: %d, SZ: %d\n", rssi, pkt->rx_ctrl.channel, len);
    if (s_uart_verbose >= 2)
    {
      Serial.printf("---->\n");
      Serial.write(data, len);
      Serial.printf("\n<----\n");
    }
  }

  size_t size = std::min<size_t>(len, MAX_PAYLOAD_SIZE);

  uint8_t* buffer = nullptr;
  {
    lock_guard lg;
    buffer = s_w2s_queue.start_writing(size);
    //xxx p = s_spi_free_queue.pop();
  }
  
  if (buffer)
  {
    memcpy(buffer, data, size);
    //p->rssi = rssi;
    
    {
      lock_guard lg;
      s_w2s_queue.end_writing();
      //xxx s_spi_to_send_queue.push(p);
    }
  }
  else
  {
    s_stats.wlan_received_packets_dropped++;
  }
  s_stats.wlan_data_received += len;

  //*/
}

/////////////////////////////////////////////////////////////////////////

uint8_t s_uart_buffer[MAX_WLAN_PAYLOAD_SIZE];
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
                S2W_Packet packet;
                start_writing_s2w_packet(packet, s_uart_offset);
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
                end_writing_s2w_packet(packet);
                s_uart_command = 0;
                s_uart_offset = 0;

                if (s_wlan_packet.ptr)
                {
                    end_reading_s2w_packet(s_wlan_packet);
                }
            }
            else
            {
                if (s_uart_offset >= MAX_WLAN_PAYLOAD_SIZE)
                {
                    while (available-- > 0) Serial.read();
                    LOG("Packet too big: %d > %d\n", s_uart_offset + 1, MAX_WLAN_PAYLOAD_SIZE);

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
        char ch = Serial.read();
        if (ch >= '0' && ch <= '9') rate = ch - '0';
        else if (ch >= 'A' && ch <= 'Z') rate = ch - 'A' + 10;
        else if (ch >= 'a' && ch <= 'z') rate = ch - 'a' + 10;
        else
        {
            s_uart_error_count++;
            LOG("Command error: Illegal rate %c\n", ch);
            return;
        }
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

S2W_Packet s_spi_incoming_packet;
W2S_Packet s_spi_outgoing_packet;

enum SPI_Command : uint16_t
{
    SPI_CMD_SEND_PACKET = 1,
    SPI_CMD_GET_PACKET = 2,
    SPI_CMD_SET_RATE = 3,
    SPI_CMD_GET_RATE = 4,
    SPI_CMD_SET_CHANNEL = 5,
    SPI_CMD_GET_CHANNEL = 6,
    SPI_CMD_SET_POWER = 7,
    SPI_CMD_GET_POWER = 8,
    SPI_CMD_GET_STATS = 9,
};

uint32_t s_spi_temp_buffer[16];

void spi_on_data_received()
{
    /*  lock_guard lg;

  if (s_spi_incoming_packet.ptr)
  {
    uint32_t poffset = s_spi_incoming_packet.offset;
    uint32_t psize = s_spi_incoming_packet.size;

    uint32_t size = psize - poffset;
    if (size > CHUNK_SIZE)
    {
      size = CHUNK_SIZE;
    }
    
    if (size == CHUNK_SIZE && ((size_t)(s_spi_incoming_packet.payload_ptr + poffset) & 3) == 0)
    {
      spi_slave_get_data((uint32_t*)(s_spi_incoming_packet.payload_ptr + poffset));
    }
    else
    {
      spi_slave_get_data(s_spi_temp_buffer);
      memcpy(s_spi_incoming_packet.payload_ptr + poffset, s_spi_temp_buffer, size);
    }
    
    s_spi_incoming_packet.offset += size;
    if (s_spi_incoming_packet.offset >= psize)
    {
      end_writing_s2w_packet(s_spi_incoming_packet);
      //s_stats.spi_packets_received++;
    }
    s_stats.spi_data_received += size;
  }
  else
  {
    s_stats.spi_error_count++;
    //Serial.printf("Unexpected data\n");
  }
*/}

void spi_on_data_sent()
{
    /*  lock_guard lg;

  if (s_spi_outgoing_packet.ptr)
  {
    uint32_t psize = s_spi_outgoing_packet.size;
    uint32_t size = psize - s_spi_outgoing_packet.offset;
    if (size > CHUNK_SIZE)
    {
      size = CHUNK_SIZE;
    }
    s_spi_outgoing_packet.offset += size;
    if (s_spi_outgoing_packet.offset >= psize)
    {
      end_reading_w2s_packet(s_spi_outgoing_packet);
      //xxx s_spi_free_queue.push_and_clear(s_spi_outgoing_packet);
      //s_spi_packets_sent++;
    }
    else
    {
      size_t poffset = s_spi_outgoing_packet.offset;
      //prepare next transfer
      if (size == CHUNK_SIZE && ((size_t)(s_spi_outgoing_packet.ptr + poffset) & 3) == 0)
      {
        spi_slave_set_data((uint32_t*)(s_spi_outgoing_packet.ptr + poffset));
      }
      else
      {
        memcpy(s_spi_temp_buffer, s_spi_outgoing_packet.ptr + poffset, size);
        spi_slave_set_data(s_spi_temp_buffer);
      }
    }

    s_stats.spi_data_sent += size;
  }
  else
  {
    LOG("SPI send missing packet\n");
    s_stats.spi_error_count++;
  }
*/}


/////////////////////////////////////////////////////////////////////////

constexpr size_t MAX_SPI_BUFFER_SIZE = MAX_WLAN_PACKET_SIZE + 8;
uint8_t* s_spi_tx_buffer = nullptr;
uint8_t* s_spi_rx_buffer = nullptr;
uint8_t* s_spi_rx_payload_ptr = nullptr;
int s_spi_transaction_id = 0;

void spi_post_setup_cb(spi_slave_transaction_t* trans)
{
//    LOG("SPI armed %d: %d\n", (int)trans->user, trans->length / 8);
}

void spi_post_trans_cb(spi_slave_transaction_t* trans)
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


void setup_spi_idle_transfer()
{
    //LOG("ARM SPI idle transfer\n");

    static spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    *reinterpret_cast<uint32_t*>(s_spi_tx_buffer) = 0xABCD;
    t.length = MAX_SPI_BUFFER_SIZE * 8;
    t.tx_buffer = s_spi_tx_buffer;
    t.rx_buffer = s_spi_rx_buffer;
    //t.user = (void*)s_spi_transaction_id++;
    esp_err_t err = spi_slave_queue_trans(VSPI_HOST, &t, 0);
    ESP_ERROR_CHECK(err);
}

void process_spi_transaction(spi_slave_transaction_t* trans)
{
    //LOG("SPI done %d: %d / %d\n", (int)trans->user, trans->length / 8, trans->trans_len / 8);

    size_t transfer_size = trans->trans_len / 8;
    if (transfer_size < 4)
    {
      LOG("SPI error: transfer too small: %d\n", transfer_size);
      setup_spi_idle_transfer();
      return;
    }

    size_t payload_size = transfer_size - 4;

    //lock_guard lg;

    uint32_t status = *reinterpret_cast<uint32_t*>(s_spi_rx_buffer);

    //  Serial.printf("spi status received sent: %d\n", status);
    SPI_Command command = (SPI_Command)(status >> 24);
    if (command == SPI_Command::SPI_CMD_SEND_PACKET)
    {
        uint32_t size = status & 0xFFFF;
        if (size > MAX_WLAN_PAYLOAD_SIZE || size != payload_size)
        {
            s_stats.spi_error_count++;
            //LOG("Packet too big: %d\n", size);
        }
        else
        {
            LOG("Received spi packet: %d\n", size);
        }
        setup_spi_idle_transfer();
        return;
    }

    if (command == SPI_Command::SPI_CMD_GET_PACKET)
    {
        LOG("Get SPI packet\n");

/*        if (!s_spi_outgoing_packet.ptr)
        {
            start_reading_w2s_packet(s_spi_outgoing_packet);
            //xxx s_spi_outgoing_packet = s_spi_to_send_queue.pop();
        }

        if (s_spi_outgoing_packet.ptr)
        {
            uint32_t size = s_spi_outgoing_packet.size & 0xFFFF;
            uint32_t rssi = 0;//*reinterpret_cast<uint8_t*>(&s_spi_outgoing_packet->rssi) & 0xFF;
            uint32_t status = (uint32_t(SPI_Command::SPI_CMD_GET_PACKET) << 24) | (rssi << 16) | size;
            //      spi_slave_set_status(status);

            if (size > CHUNK_SIZE)
            {
                size = CHUNK_SIZE;
            }
            if (size == CHUNK_SIZE && ((size_t)(s_spi_outgoing_packet.ptr) & 3) == 0)
            {
                //        spi_slave_set_data((uint32_t*)(s_spi_outgoing_packet.ptr));
            }
            else
            {
                memcpy(s_spi_temp_buffer, s_spi_outgoing_packet.ptr, size);
                //        spi_slave_set_data(s_spi_temp_buffer);
            }
        }
        else
        {
            //      spi_slave_set_status(0);
        }
*/
        setup_spi_idle_transfer();
        return;
    }

    if (command == SPI_Command::SPI_CMD_SET_RATE)
    {
        uint32_t rate = status & 0xFFFF;
        LOG("Setting rate: %d\n", rate);
        /*    if (rate >= sizeof(s_rate_mapping) || wifi_set_user_fixed_rate(FIXED_RATE_MASK_ALL, s_rate_mapping[rate]) != 0)
    {
      LOG("Failed to set rate %d", rate);
      s_stats.spi_error_count++;
    }
*/  }
    else if (command == SPI_Command::SPI_CMD_GET_RATE)
    {
        LOG("Get rate\n");
        uint8_t enable_mask = 0;
        uint8_t rate = 0;
        /*    if (wifi_get_user_fixed_rate(&enable_mask, &rate) == 0)
    {
      uint16_t mapped_rate = 0xFFFF;
      for (uint16_t i = 0; i < sizeof(s_rate_mapping); i++)
      {
        if (rate == s_rate_mapping[i])
        {
          mapped_rate = i;
          break;
        }
      }
      if (mapped_rate == 0xFFFF)
      {
        LOG("Cannot map hardware rate %d", rate);
      }
      spi_slave_set_status((SPI_Command::SPI_CMD_GET_RATE << 24) | mapped_rate);
    }
    else
    {
      LOG("Cannot get rate");
      s_stats.spi_error_count++;
    }
*/  }
    else if (command == SPI_Command::SPI_CMD_SET_CHANNEL)
    {
        uint32_t channel = status & 0xFFFF;
        LOG("Setting channel: %d\n", channel);
        /*    if (channel == 0 || channel > 11 || !wifi_set_channel(channel))
    {
      LOG("Cannot set channel %d", channel);
      s_stats.spi_error_count++;
    }
*/  }
    else if (command == SPI_Command::SPI_CMD_GET_CHANNEL)
    {
      LOG("Get channel\n");
        //    uint8_t channel = wifi_get_channel();
        //    spi_slave_set_status((SPI_Command::SPI_CMD_GET_CHANNEL << 24) | channel);
    }
    else if (command == SPI_Command::SPI_CMD_SET_POWER)
    {
        uint16_t power = status & 0xFFFF;
        float dBm = (static_cast<float>(power) - 32767.f) / 100.f;
        LOG("Setting power: %f\n", dBm);
        set_wlan_power_dBm (dBm);
    }
    else if (command == SPI_Command::SPI_CMD_GET_POWER)
    {
        LOG("Get power\n");
        float dBm = get_wlan_power_dBm();
        uint16_t power = static_cast<uint16_t>(std::max(std::min((dBm * 100.f), 32767.f), -32767.f) + 32767.f);
        //    spi_slave_set_status((SPI_Command::SPI_CMD_GET_POWER << 24) | power);
    }
    else if (command == SPI_Command::SPI_CMD_GET_STATS)
    {
        LOG("Get stats\n");
        uint32_t data[8] = { 0 };
        memcpy(data, &s_stats, sizeof(Stats));
        //    spi_slave_set_data(data);
    }
    else
    {
        LOG("Unknown command: %d\n", command);
    }

      setup_spi_idle_transfer();
    
    //s_stats.spi_status_received++;
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
    Serial.begin(115200);
    Serial.setTimeout(999999);

    srand(millis());

    Serial.printf("Initializing...\n");

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

    setup_spi_idle_transfer();

    //ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0) );

    //  ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(packet_received_cb));
    //  ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    set_wlan_power_dBm(20.5f);

    //esp_log_level_set("*", ESP_LOG_NONE);

    set_wifi_fixed_rate(31);

    //S012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789

    Serial.printf("Initialized\n");
}

void loop() 
{
    parse_command();

    //send pending wlan packets
    if (!s_wlan_packet.ptr)
    {
        S2W_Packet p;
        {
            lock_guard lg;
            if (!s_wlan_packet.ptr)
            {
                start_reading_s2w_packet(s_wlan_packet);
                p = s_wlan_packet;
            }
        }

        if (p.ptr)
        {
            memcpy(p.ptr, s_wlan_packet_header, WLAN_HEADER_SIZE);
            esp_err_t res = esp_wifi_80211_tx(ESP_WIFI_IF, p.ptr, WLAN_HEADER_SIZE + p.size, false);
            if (res == ESP_OK)
            {
                s_stats.wlan_data_sent += s_wlan_packet.size;
                set_status_led_on();
                LOG("WLAN inject OKKK\n");
            }
            else
            {
                LOG("WLAN inject error: %d\n", res);
                s_stats.wlan_error_count++;
            }
            {
                lock_guard lg;
                end_reading_s2w_packet(s_wlan_packet);
            }
        }
    }

    update_status_led();

    {
        spi_slave_transaction_t* qt = nullptr;
        esp_err_t err = spi_slave_get_trans_result(VSPI_HOST, &qt, portMAX_DELAY);
        ESP_ERROR_CHECK(err);

        if (qt)
        {
            process_spi_transaction(qt);

            //char xxx[128];
            //memcpy(xxx, s_spi_rx_buf, qt->length / 8);
            //xxx[qt->length / 8] = 0;
            //static int xxx = 0;
            //xxx++;
            s_stats.spi_data_received += qt->length / 8;
            //LOG("SPI confirmed %d: %d: %s\n", (int)qt->user, qt->length / 8, xxx);
        }
    }

    /*  if (s_wlan_packet.ptr)
  {
    esp_err_t res = esp_wifi_80211_tx(ESP_WIFI_IF, s_wlan_packet.ptr, HEADER_SIZE + s_wlan_packet.size, false);
    if (res == ESP_OK)
    {
      s_stats.wlan_data_sent += s_wlan_packet.size;
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
                      s_stats.wlan_data_sent, s_stats.wlan_data_received, s_stats.wlan_error_count, s_stats.wlan_received_packets_dropped, s_s2w_queue.size() * 100 / s_s2w_queue.capacity(),
                      s_stats.spi_data_sent, s_stats.spi_data_received, s_stats.spi_error_count, s_stats.spi_received_packets_dropped, s_w2s_queue.size() * 100 / s_w2s_queue.capacity());

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
