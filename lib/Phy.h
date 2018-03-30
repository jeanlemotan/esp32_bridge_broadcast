#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include <array>
#include <mutex>
#include <linux/spi/spidev.h>

class Phy
{
public:
    Phy();

    enum class Init_Result
    {
        OK,
        ALREADY_INITIALIZED,
        BAD_PARAMS,
        HW_FAILURE
    };

    Init_Result init_pigpio(size_t port, size_t channel, size_t speed = 8000000, size_t comms_delay = 25);
    Init_Result init_dev(const char* device, size_t speed = 8000000, size_t comms_delay = 20);

    void process();

    static const size_t MAX_PAYLOAD_SIZE = 1374;

    bool send_data(size_t fec_channel, void const* data, size_t size);
    bool receive_data(size_t fec_channel, void* data, size_t& size, int& rssi);

    bool setup_fec_channel(size_t fec_channel, size_t coding_k, size_t coding_n, size_t mtu);

    enum class Rate
    {
        RATE_B_1M_CCK,
        RATE_B_2M_CCK,
        RATE_B_2M_CCK_SHOST_PREAMBLE,
        RATE_B_5_5M_CCK,
        RATE_B_5_5M_CCK_SHOST_PREAMBLE,
        RATE_B_11M_CCK,
        RATE_B_11M_CCK_SHOST_PREAMBLE,

        RATE_G_6M_ODFM,
        RATE_G_9M_ODFM,
        RATE_G_12M_ODFM,
        RATE_G_18M_ODFM,
        RATE_G_24M_ODFM,
        RATE_G_36M_ODFM,
        RATE_G_48M_ODFM,
        RATE_G_54M_ODFM,

        RATE_N_6_5M_MCS0,
        RATE_N_7_2M_MCS0_SHORT_GI,
        RATE_N_13M_MCS1,
        RATE_N_14_4M_MCS1_SHORT_GI,
        RATE_N_19_5M_MCS2,
        RATE_N_21_7M_MCS2_SHORT_GI,
        RATE_N_26M_MCS3,
        RATE_N_28_9M_MCS3_SHORT_GI,
        RATE_N_39M_MCS4,
        RATE_N_43_3M_MCS4_SHORT_GI,
        RATE_N_52M_MCS5,
        RATE_N_57_8M_MCS5_SHORT_GI,
        RATE_N_58M_MCS6,
        RATE_N_65M_MCS6_SHORT_GI,
        RATE_N_65M_MCS7,
        RATE_N_72M_MCS7_SHORT_GI,

        COUNT
    };

    bool set_rate(Rate rate);
    bool get_rate(Rate& rate);

    bool set_channel(uint8_t channel);
    bool get_channel(uint8_t& channel);

    bool set_power(float power_dBm);
    bool get_power(float& power_dBm);

    struct Stats
    {
        uint32_t wlan_data_sent = 0;
        uint32_t wlan_data_received = 0;
        uint16_t wlan_error_count = 0;
        uint16_t wlan_received_packets_dropped = 0;
        uint32_t spi_data_sent = 0;
        uint32_t spi_data_received = 0;
        uint16_t spi_error_count = 0;
        uint16_t spi_received_packets_dropped = 0;
    };

    bool get_stats(Stats& stats);

private:
    bool transfer(void const* tx_data, void* rx_data, size_t size);
    uint32_t get_status();
    bool send_command(uint32_t command);
    bool get_data();

    std::mutex m_mutex;

    size_t m_speed = 0;
    size_t m_comms_delay = 0;

    int m_pigpio_fd = -1;
    int m_dev_fd = -1;

    static const size_t MAX_TRANSFERS = 64;

    std::array<std::vector<uint8_t>, MAX_TRANSFERS> m_spi_transfers_data;
    std::array<spi_ioc_transfer, MAX_TRANSFERS> m_spi_transfers;
};

