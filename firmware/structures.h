#pragma once

#include <cassert>

constexpr uint8_t s_wlan_ieee_header[] =
{
    0x08, 0x01, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
    0x10, 0x86
};

constexpr size_t WLAN_IEEE_HEADER_SIZE = sizeof(s_wlan_ieee_header);
constexpr size_t WLAN_MAX_PACKET_SIZE = 1500;
constexpr size_t WLAN_MAX_PAYLOAD_SIZE = WLAN_MAX_PACKET_SIZE - WLAN_IEEE_HEADER_SIZE;

static_assert(WLAN_IEEE_HEADER_SIZE == 24, "");

struct Wlan_Outgoing_Packet
{
  uint8_t* ptr = nullptr;
  uint8_t* payload_ptr = nullptr;
  uint16_t size = 0;
  uint16_t offset = 0;
};

struct Wlan_Incoming_Packet
{
  uint8_t* ptr = nullptr;
  uint16_t size = 0;
  uint16_t offset = 0;
  int8_t rssi = 0;
};

struct Wlan_Packet_Header
{
  uint8_t uses_fec : 1;
};

/////////////////////////////////////////////////////////////////////////

constexpr size_t WLAN_INCOMING_BUFFER_SIZE = 32000;
alignas(uint32_t) uint8_t s_wlan_incoming_buffer[WLAN_INCOMING_BUFFER_SIZE];

constexpr size_t WLAN_OUTGOING_BUFFER_SIZE = 32000;
alignas(uint32_t) uint8_t s_wlan_outgoing_buffer[WLAN_OUTGOING_BUFFER_SIZE];


template<size_t N>
struct Queue
{
  Queue(uint8_t* buffer)
    : m_buffer(buffer)
  {
    //memset(m_buffer, 0, N);
  }

  IRAM_ATTR inline size_t count() const
  {
    return m_count;
  }

  IRAM_ATTR inline size_t next_reading_size()
  {
    if (m_read_start != m_read_end)
    {
      return 0;
    }
    if (m_read_start == m_write_start)
    {
      return 0;
    }
    size_t size;
    memcpy(&size, m_buffer + m_read_start, sizeof(uint32_t)); //read the size
    return size;
  }

  IRAM_ATTR inline size_t size() const
  {
    if (m_read_start == m_write_start)
    {
      return 0;
    }
    if (m_write_start > m_read_start) //no wrap
    {
      return m_write_end - m_read_start;
    }
    if (m_write_start < m_read_start) //wrap
    {
      return (N - m_read_start) + m_write_end;
    }
    return 0;
  }

  IRAM_ATTR inline size_t capacity() const
  {
    return N;
  }

  IRAM_ATTR inline uint8_t* start_writing(size_t size) __attribute__((always_inline))
  {
    if (m_write_start != m_write_end)
    {
      return nullptr;
    }
    
    //size_t aligned_size = (size + (CHUNK_SIZE - 1)) & ~CHUNK_MASK; //align the size
    size_t end = m_write_start + sizeof(uint32_t) + size;
    if (end <= N) //no wrap
    {
      //check read collisions
      if (m_write_start < m_read_start && end >= m_read_start)
      {
//        Serial.printf("\tf1: %d < %d && %d >= %d\n", start, m_read_start, end, m_read_start);
        return nullptr;
      }

//      Serial.printf("\tw1: %d, %d, %d, %d\n", start, end, m_read_start, m_read_end);
      memcpy(m_buffer + m_write_start, &size, sizeof(uint32_t)); //write the size before wrapping
      m_write_end = end;
      return m_buffer + m_write_start + sizeof(uint32_t);
    }
    else //wrap
    {
      //check read collisions
      if (m_read_start > m_write_start) //if the read offset is between start and the end of the buffer
      {
//        Serial.printf("\tf2: %d > %d\n", m_read_start, start);
        return nullptr;
      }
      end = size;
      //check read collisions
      if (end >= m_read_start)
      {
//        Serial.printf("\tf3: %d >= %d\n", end, m_read_start);
        return nullptr;
      }

//      Serial.printf("\tw2: %d, %d, %d, %d\n", start, end, m_read_start, m_read_end);
      memcpy(m_buffer + m_write_start, &size, sizeof(uint32_t)); //write the size before wrapping
      m_write_end = end;
      return m_buffer;
    }
  }

  IRAM_ATTR inline void end_writing() __attribute__((always_inline))
  {
    m_write_start = m_write_end;
    m_count++;
  }
  IRAM_ATTR inline void cancel_writing() __attribute__((always_inline))
  {
    m_write_end = m_write_start;
  }

  IRAM_ATTR inline uint8_t* start_reading(size_t& size) __attribute__((always_inline))
  {
    if (m_read_start != m_read_end)
    {
      return nullptr;
    }
    if (m_read_start == m_write_start)
    {
//      Serial.printf("\tf4: %d == %d\n", m_read_start, m_write_start);
      size = 0;
      return nullptr;
    }
    size_t start = m_read_start;
    memcpy(&size, m_buffer + start, sizeof(uint32_t)); //read the size
    //size_t aligned_size = (size + (CHUNK_SIZE - 1)) & ~CHUNK_MASK; //align the size
    size_t end = start + sizeof(uint32_t) + size;
    if (end <= N)
    {
      m_read_end = end;
      return m_buffer + start + sizeof(uint32_t);
    }
    else
    {
      //m_read_start = 0;
      m_read_end = size;
      return m_buffer;
    }
  }

  IRAM_ATTR inline void end_reading() __attribute__((always_inline))
  {
    m_read_start = m_read_end;
    assert(m_count > 0);
    m_count--;
  }
  IRAM_ATTR inline void cancel_reading()  __attribute__((always_inline))
  {
    m_read_end = m_read_start;
  }
  
private:
  uint8_t* m_buffer = nullptr;
  size_t m_write_start = 0;
  size_t m_write_end = 0;
  size_t m_read_start = 0;
  size_t m_read_end = 0;
  size_t m_count = 0;
};

////////////////////////////////////////////////////////////////////////////////////

Queue<WLAN_INCOMING_BUFFER_SIZE> s_wlan_incoming_queue(s_wlan_incoming_buffer);
portMUX_TYPE s_wlan_incoming_mux = portMUX_INITIALIZER_UNLOCKED;

Queue<WLAN_OUTGOING_BUFFER_SIZE> s_wlan_outgoing_queue(s_wlan_outgoing_buffer);
portMUX_TYPE s_wlan_outgoing_mux = portMUX_INITIALIZER_UNLOCKED;

////////////////////////////////////////////////////////////////////////////////////

IRAM_ATTR bool start_writing_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet, size_t size)
{
  size_t real_size = WLAN_IEEE_HEADER_SIZE + size;
  uint8_t* buffer = s_wlan_outgoing_queue.start_writing(real_size);
  if (!buffer)
  {
    packet.ptr = nullptr;
    return false;
  }
  packet.offset = 0;
  packet.size = size;
  packet.ptr = buffer;
  packet.payload_ptr = buffer + WLAN_IEEE_HEADER_SIZE;
  return true;
}
IRAM_ATTR void end_writing_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet)
{
  s_wlan_outgoing_queue.end_writing();
  packet.ptr = nullptr;
}
IRAM_ATTR void cancel_writing_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet)
{
  s_wlan_outgoing_queue.cancel_writing();
  packet.ptr = nullptr;
}

IRAM_ATTR bool start_reading_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet)
{
  size_t real_size = 0;
  uint8_t* buffer = s_wlan_outgoing_queue.start_reading(real_size);
  if (!buffer)
  {
    packet.ptr = nullptr;
    return false;
  }
  packet.offset = 0;
  packet.size = real_size - WLAN_IEEE_HEADER_SIZE;
  packet.ptr = buffer;
  packet.payload_ptr = buffer + WLAN_IEEE_HEADER_SIZE;
  return true;
}
IRAM_ATTR void end_reading_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet)
{
  s_wlan_outgoing_queue.end_reading();
  packet.ptr = nullptr;
}
IRAM_ATTR void cancel_reading_wlan_outgoing_packet(Wlan_Outgoing_Packet& packet)
{
  s_wlan_outgoing_queue.cancel_reading();
  packet.ptr = nullptr;
}

////////////////////////////////////////////////////////////////////////////////////

IRAM_ATTR bool start_writing_wlan_incoming_packet(Wlan_Incoming_Packet& packet, size_t size)
{
  uint8_t* buffer = s_wlan_incoming_queue.start_writing(size);
  if (!buffer)
  {
    packet.ptr = nullptr;
    return false;
  }
  packet.offset = 0;
  packet.size = size;
  packet.ptr = buffer;
  return true;
}
IRAM_ATTR void end_writing_wlan_incoming_packet(Wlan_Incoming_Packet& packet)
{
  s_wlan_incoming_queue.end_writing();
  packet.ptr = nullptr;
}
IRAM_ATTR void cancel_writing_wlan_incoming_packet(Wlan_Incoming_Packet& packet)
{
  s_wlan_incoming_queue.cancel_writing();
  packet.ptr = nullptr;
}

IRAM_ATTR bool start_reading_wlan_incoming_packet(Wlan_Incoming_Packet& packet)
{
  size_t size = 0;
  uint8_t* buffer = s_wlan_incoming_queue.start_reading(size);
  if (!buffer)
  {
    packet.ptr = nullptr;
    return false;
  }
  packet.offset = 0;
  packet.size = size;
  packet.ptr = buffer;
  return true;
}
IRAM_ATTR void end_reading_wlan_incoming_packet(Wlan_Incoming_Packet& packet)
{
  s_wlan_incoming_queue.end_reading();
  packet.ptr = nullptr;
}
IRAM_ATTR void cancel_reading_wlan_incoming_packet(Wlan_Incoming_Packet& packet)
{
  s_wlan_incoming_queue.cancel_reading();
  packet.ptr = nullptr;
}

////////////////////////////////////////////////////////////////////////////////////

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
static_assert(sizeof(Stats) == 6 * 4, "Stats too big");

Stats s_stats;
