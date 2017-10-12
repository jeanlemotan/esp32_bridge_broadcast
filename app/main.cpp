#include "Fec_Encoder.h"
#include "Phy.h"
#include "utils/pigpio.h"
#include <iostream>
#include <string>
#include <cstdio>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

bool s_verbose = false;
bool s_flush = false;

bool s_fec_benchmark = false;
bool s_phy_benchmark = false;
bool s_use_fec = false;
uint32_t s_fec_coding_k = 0;
uint32_t s_fec_coding_n = 0;

const size_t MAX_MTU = Phy::MAX_PAYLOAD_SIZE - Fec_Encoder::PAYLOAD_OVERHEAD;
size_t s_mtu = MAX_MTU;

bool s_use_spi_dev = true;
std::string s_spi_dev = "/dev/spidev0.0";
size_t s_pigpio_spi_port = 0;
size_t s_pigpio_spi_channel = 0;
size_t s_spi_speed = 8000000;
size_t s_spi_delay = 20;
Phy::Rate s_phy_rate = Phy::Rate::RATE_B_5_5M_CCK;
float s_phy_power = 20.5f;
uint8_t s_phy_channel = 1;

/* This prints an "Assertion failed" message and aborts.  */
void __assert_fail(const char *__assertion, const char *__file, unsigned int __line, const char *__function)
{
    printf("assert: %s:%d: %s: %s", __file, __line, __function, __assertion);
    fflush(stdout);
//    abort();
}

void show_help()
{
    std::cout << "Esp8266 Broadcast with FEC support\n";
    std::cout << "Usage:\n";
    std::cout << "\t--hrlp\tShows this help message\n";
    std::cout << "\t--fec-benchmark\tRuns a FEC benchmark\n";
    std::cout << "\t--phy-benchmark\tRuns a PHY bandwidth benchmark\n";
    std::cout << "\t--verbose\tPrint out the settings\n";
    std::cout << "\t--flush\tFlush stdout when writing to it. This can reduce latency\n";
    std::cout << "\t--fec K N\tUse FEC (Forward Error Correction) for transmission and reception\n";
    std::cout << "\t\tK and N are the coding constants. Every K packets, N are produced (N > K)\n";
    std::cout << "\t--mtu " << std::to_string(s_mtu) << "\tUse the specified packet size. Max is " << std::to_string(MAX_MTU) << "\n";
    std::cout << "\t--spi-dev \"/dev/spidev0.0\"\tUse the specified device for SPI\n";
    std::cout << "\t--spi-pigpio PORT CHANNEL\tUse PIGPIO on the specified port & channel for SPI\n";
    std::cout << "\t--spi-speed 8000000 \tUse the specified SPI speed (Hz)\n";
    std::cout << "\t--spi-delay 20\tUse the specified delay in microseconds for SPI transactions\n";
    std::cout << "\t--phy-rate X\tThe PHY rate index, out of these values:\n";
    std::cout << "\t\t0:  802.11b 1Mbps, CCK modulation\n";
    std::cout << "\t\t1:  802.11b 2Mbps, CCK modulation\n";
    std::cout << "\t\t2:  802.11b 2Mbps, Short Preamble, CCK modulation\n";
    std::cout << "\t\t3:  802.11b 5.5Mbps, CCK modulation\n";
    std::cout << "\t\t4:  802.11b 5.5Mbps, Short Preamble, CCK modulation\n";
    std::cout << "\t\t5:  802.11b 11Mbps, CCK modulation\n";
    std::cout << "\t\t6:  802.11b 11Mbps, Short Preamble, CCK modulation\n";
    std::cout << "\t\t7:  802.11g 6Mbps, ODFM modulation\n";
    std::cout << "\t\t8:  802.11g 9Mbps, ODFM modulation\n";
    std::cout << "\t\t9:  802.11g 12Mbps, ODFM modulation\n";
    std::cout << "\t\t10: 802.11g 18Mbps, ODFM modulation\n";
    std::cout << "\t\t11: 802.11g 24Mbps, ODFM modulation\n";
    std::cout << "\t\t12: 802.11g 36Mbps, ODFM modulation\n";
    std::cout << "\t\t13: 802.11g 48Mbps, ODFM modulation\n";
    std::cout << "\t\t14: 802.11g 56Mbps, ODFM modulation\n";
    std::cout << "\t--phy-power X\tThe PHY power in dBm between 0dBm to 20.5dBm\n";
    std::cout << "\t--phy-channel X\tThe PHY channel between 1 and 11\n";
}

int parse_arguments(int argc, const char* argv[])
{
    for (int i = 1; i < argc; i++)
    {
        int remanining = argc - i - 1;

        std::string arg(argv[i]);
        if (arg == "--help")
        {
            show_help();
            return 0;
        }
        else if (arg == "--fec-benchmark")
        {
            s_fec_benchmark = true;
        }
        else if (arg == "--phy-benchmark")
        {
            s_phy_benchmark = true;
        }
        else if (arg == "--verbose")
        {
            s_verbose = true;
        }
        else if (arg == "--flush")
        {
            s_flush = true;
        }
        else if (arg == "--fec")
        {
            if (remanining < 2)
            {
                std::cerr << arg << " has to be followed by the K and N constants\n";
                return -1;
            }
            s_fec_coding_k = std::stoul(argv[i + 1]);
            s_fec_coding_n = std::stoul(argv[i + 2]);
            if (s_fec_coding_k > s_fec_coding_n || s_fec_coding_k > Fec_Encoder::MAX_CODING_K || s_fec_coding_n > Fec_Encoder::MAX_CODING_N)
            {
                std::cerr << "FEC coding K has to be smaller than N. K has to be <= than " << std::to_string(Fec_Encoder::MAX_CODING_K) <<
                             " and N has to be  <= than " << std::to_string(Fec_Encoder::MAX_CODING_N) << "\n";
                return -1;
            }
            s_use_fec = true;
            i += 2;
        }
        else if (arg == "--mtu")
        {
            if (remanining == 0)
            {
                std::cerr << arg << " has to be followed by a numeric value > 0 && < " << std::to_string(Phy::MAX_PAYLOAD_SIZE) << "\n";
                return -1;
            }
            s_mtu = std::stoul(argv[i + 1]);
            if (s_mtu == 0 || s_mtu > MAX_MTU)
            {
                std::cerr << arg << "Invalid mtu: " << std::to_string(s_mtu) << "\n";
                return -1;
            }
            i++;
        }
        else if (arg == "--spi-dev")
        {
            if (remanining == 0)
            {
                std::cerr << arg << " has to be followed by a device name\n";
                return -1;
            }
            s_use_spi_dev = true;
            s_spi_dev = argv[i + 1];
            i++;
        }
        else if (arg == "--spi-pigpio")
        {
            if (remanining < 2)
            {
                std::cerr << arg << " has to be followed by a numeric port and channel\n";
                return -1;
            }
            s_use_spi_dev = false;
            s_pigpio_spi_port = std::stoul(argv[i + 1]);
            s_pigpio_spi_channel = std::stoul(argv[i + 2]);
            i += 2;
        }
        else if (arg == "--spi-speed")
        {
            if (remanining == 0)
            {
                std::cerr << arg << " has to be followed by a numeric value\n";
                return -1;
            }
            s_spi_speed = std::stoul(argv[i + 1]);
            i++;
        }
        else if (arg == "--spi-delay")
        {
            if (remanining == 0)
            {
                std::cerr << arg << " has to be followed by a numeric value\n";
                return -1;
            }
            s_spi_delay = std::stoul(argv[i + 1]);
            i++;
        }
        else if (arg == "--phy-rate")
        {
            if (remanining == 0)
            {
                std::cerr << arg << " has to be followed by a numeric value >= 0 && <= " << std::to_string(size_t(Phy::Rate::COUNT) - 1) << "\n";
                return -1;
            }
            size_t rate = std::stoul(argv[i + 1]);
            if (rate >= size_t(Phy::Rate::COUNT))
            {
                std::cerr << "Invalid rate: " << std::to_string(rate) << "\n";
                return -1;
            }
            s_phy_rate = static_cast<Phy::Rate>(rate);
            i++;
        }
        else if (arg == "--phy-power")
        {
            if (remanining == 0)
            {
                std::cerr << arg << " has to be followed by a numeric value\n";
                return -1;
            }
            s_phy_power = std::stof(argv[i + 1]);
            i++;
        }
        else if (arg == "--phy-channel")
        {
            if (remanining == 0)
            {
                std::cerr << arg << " has to be followed by a numeric value\n";
                return -1;
            }
            s_phy_channel = std::stoul(argv[i + 1]);
            i++;
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << "\n";
            return -1;
        }
    }

    return 0;
}


int run_fec_benchmark()
{
    Fec_Encoder tx;
    Fec_Encoder rx;

    Fec_Encoder::TX_Descriptor tx_descriptor;
    tx_descriptor.coding_k = s_fec_coding_k;
    tx_descriptor.coding_n = s_fec_coding_n;
    tx_descriptor.mtu = s_mtu;
    if (!tx.init_tx(tx_descriptor))
    {
        return -1;
    }

    Fec_Encoder::RX_Descriptor rx_descriptor;
    rx_descriptor.coding_k = s_fec_coding_k;
    rx_descriptor.coding_n = s_fec_coding_n;
    rx_descriptor.mtu = s_mtu;
    if (!rx.init_rx(rx_descriptor))
    {
        return -1;
    }

    typedef Fec_Encoder::Clock Clock;

    std::string reference = "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine."
            "This is a test sentence. There are many like it, but this one is mine.";

    size_t total_data_size = 0;
    size_t total_fec_data_size = 0;

    size_t encoded_packets = 0;
    size_t encoded_size = 0;
    size_t decoded_packets = 0;
    size_t decoded_size = 0;

    tx.on_tx_data_encoded = [&rx, &encoded_size, &encoded_packets, &total_fec_data_size](void const* data, size_t size)
    {
        total_fec_data_size += size;
        encoded_size += size;
        encoded_packets++;
        rx.add_rx_packet(data, size, true);
    };

    rx.on_rx_data_decoded = [&reference, &decoded_size, &decoded_packets](void const* data, size_t size)
    {
        decoded_size += size;
        decoded_packets++;
    };

    float seconds = 5.f;
    Clock::time_point start_tp = Clock::now();
    while (Clock::now() - start_tp < std::chrono::duration<float>(seconds))
    {
        tx.add_tx_packet(reference.data(), reference.size(), true);
        total_data_size += reference.size();
    }

    while (decoded_size + s_mtu * 2 < total_data_size)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    float total_data_size_mb = static_cast<float>(total_data_size) / (1024.f * 1024.f);
    float total_fec_data_size_mb = static_cast<float>(total_fec_data_size) / (1024.f * 1024.f);
    std::cout << "Data:\t" << std::to_string(total_data_size_mb) << " MB, " << std::to_string(total_data_size_mb / seconds) << " MBps\n";
    std::cout << "FEC Data:\t" << std::to_string(total_fec_data_size_mb) << " MB, " << std::to_string(total_fec_data_size_mb / seconds) << " MBps\n";

    std::cout << "Encoded:\n";
    std::cout << "\t" << std::to_string(static_cast<float>(encoded_size)  / (seconds * 1024.f * 1024.f)) << " MBps\n";
    std::cout << "\t" << std::to_string(encoded_packets) << " packets\n";
    std::cout << "Decoded:\n";
    std::cout << "\t" << std::to_string(static_cast<float>(decoded_size) / (seconds * 1024.f * 1024.f)) << " MBps\n";
    std::cout << "\t" << std::to_string(decoded_packets) << " packets\n";

    return 0;
}


int run_fec(Phy& phy)
{
    if (s_verbose)
    {
        std::cout << "FEC\n\tK " << std::to_string(s_fec_coding_k) <<
                     "\n\tN " << std::to_string(s_fec_coding_n) << "\n";
    }

    typedef Fec_Encoder::Clock Clock;

    Fec_Encoder tx;
    Fec_Encoder rx;

    Fec_Encoder::TX_Descriptor tx_descriptor;
    tx_descriptor.coding_k = s_fec_coding_k;
    tx_descriptor.coding_n = s_fec_coding_n;
    tx_descriptor.mtu = s_mtu;
    if (!tx.init_tx(tx_descriptor))
    {
        return -1;
    }

    Fec_Encoder::RX_Descriptor rx_descriptor;
    rx_descriptor.coding_k = s_fec_coding_k;
    rx_descriptor.coding_n = s_fec_coding_n;
    rx_descriptor.mtu = s_mtu;
    if (!rx.init_rx(rx_descriptor))
    {
        return -1;
    }

    tx.on_tx_data_encoded = [&phy](void const* data, size_t size)
    {
//        std::cout << "sending fec data " << std::to_string(size) << "\n";
        phy.send_data(data, size);
    };

    rx.on_rx_data_decoded = [](void const* data, size_t size)
    {
        std::cout.write(reinterpret_cast<const char*>(data), size);
        if (s_flush)
        {
            std::flush(std::cout);
        }
    };

    std::array<uint8_t, Phy::MAX_PAYLOAD_SIZE> rx_data;
    size_t rx_data_size = 0;
    int rx_rssi = 0;

    std::array<uint8_t, Phy::MAX_PAYLOAD_SIZE> tx_data;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    Clock::time_point last_receive_tp = Clock::now();
    while (true)
    {
        if (Clock::now() - last_receive_tp >= std::chrono::microseconds(500))
        {
            last_receive_tp = Clock::now();
            if (phy.receive_data(rx_data.data(), rx_data_size, rx_rssi))
            {
                //std::cout << "received packet " << std::to_string(rx_data_size) << "\n";
                rx.add_rx_packet(rx_data.data(), rx_data_size, true);
            }
        }

        {
            int res = read(STDIN_FILENO, tx_data.data(), tx_data.size());
            if (res > 0)
            {
                tx.add_tx_packet(tx_data.data(), static_cast<size_t>(res), true);
            }
        }
    }

    return 0;
}

int run_no_fec(Phy& phy)
{
    typedef Fec_Encoder::Clock Clock;


    std::array<uint8_t, Phy::MAX_PAYLOAD_SIZE> rx_data;
    size_t rx_data_size = 0;
    int rx_rssi = 0;

    std::array<uint8_t, Phy::MAX_PAYLOAD_SIZE> tx_data;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    Clock::time_point last_receive_tp = Clock::now();
    while (true)
    {
        if (Clock::now() - last_receive_tp >= std::chrono::microseconds(500))
        {
            last_receive_tp = Clock::now();
            if (phy.receive_data(rx_data.data(), rx_data_size, rx_rssi))
            {
                std::cout.write(reinterpret_cast<const char*>(rx_data.data()), rx_data_size);
                if (s_flush)
                {
                    std::flush(std::cout);
                }
            }
        }

        {
            int res = read(STDIN_FILENO, tx_data.data(), s_mtu);
            if (res > 0)
            {
                phy.send_data(tx_data.data(), res);
            }
        }
    }

    return 0;
}


int main(int argc, const char* argv[])
{
    int result = parse_arguments(argc, argv);
    if (result < 0)
    {
        show_help();
        return result;
    }

    if (s_fec_coding_k == 0)
    {
        s_fec_coding_k = 12;
    }
    if (s_fec_coding_n == 0)
    {
        s_fec_coding_n = 20;
    }

    if (gpioCfgClock(5, PI_CLOCK_PCM, 0) < 0 || gpioCfgPermissions(static_cast<uint64_t>(-1)))
    {
        std::cerr << "Cannot configure pigpio\n";
        return -1;
    }
    if (gpioInitialise() < 0)
    {
        std::cerr << "Cannot initialize pigpio\n";
        return -1;
    }

    if (s_fec_benchmark)
    {
        return run_fec_benchmark();
    }
    if (s_phy_benchmark)
    {
        //return run_phy_benchmark();
    }

    Phy phy;
    if (s_use_spi_dev)
    {
        if (s_verbose)
        {
            std::cout << "SPI\n\tdev " << s_spi_dev <<
                         "\n\tspeed " << std::to_string(s_spi_speed) <<
                         " Hz\n\tdelay " << std::to_string(s_spi_delay) << "us\n";
        }
        if (phy.init_dev(s_spi_dev.c_str(), s_spi_speed, s_spi_delay) != Phy::Init_Result::OK)
        {
            return -1;
        }
    }
    else
    {
        if (s_verbose)
        {
            std::cout << "SPI\n\tpigpio, port " << std::to_string(s_pigpio_spi_port) << ", channel " << std::to_string(s_pigpio_spi_channel) <<
                         "\n\tspeed " << std::to_string(s_spi_speed) <<
                         " Hz\n\tdelay " << std::to_string(s_spi_delay) << "us\n";
        }
        if (phy.init_pigpio(s_pigpio_spi_port, s_pigpio_spi_channel, s_spi_speed, s_spi_delay) != Phy::Init_Result::OK)
        {
            return -1;
        }
    }

    phy.set_rate(s_phy_rate);
    phy.set_power(s_phy_power);
    phy.set_channel(s_phy_channel);
    int actual_rate = -1;
    float actual_power = -1;
    int actual_channel = -1;
    {
        Phy::Rate rate;
        actual_rate = phy.get_rate(rate) ? static_cast<int>(rate) : -1;
        if (!phy.get_power(actual_power))
        {
            actual_power = -1;
        }
        uint8_t ch;
        actual_channel = phy.get_channel(ch) ? ch : -1;
    }
    if (s_verbose)
    {
        std::cout << "PHY\n\trequested rate " << std::to_string(static_cast<int>(s_phy_rate)) << ", actual rate " << std::to_string(actual_rate)
                  << "\n\trequested power " << std::to_string(s_phy_power) << ", actual power " << std::to_string(actual_power)
                  << "\n\trequested channel " << std::to_string(s_phy_channel) << ", actual channel " << std::to_string(actual_channel)
                  << "\n";
    }

    result = s_use_fec ? run_fec(phy) : run_no_fec(phy);

    gpioTerminate();

    return result;
}
