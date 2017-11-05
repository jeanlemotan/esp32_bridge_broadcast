#pragma once

#include <cassert>

constexpr uint8_t s_wlan_packet_header[] =
{
    0x08, 0x01, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
    0x10, 0x86
};

constexpr size_t WLAN_HEADER_SIZE = sizeof(s_wlan_packet_header);
constexpr size_t MAX_WLAN_PACKET_SIZE = 1400;
constexpr size_t MAX_WLAN_PAYLOAD_SIZE = MAX_WLAN_PACKET_SIZE - WLAN_HEADER_SIZE;

static_assert(WLAN_HEADER_SIZE == 24, "");

struct S2W_Packet
{
  uint8_t* ptr = nullptr;
  uint8_t* payload_ptr = nullptr;
  uint16_t size = 0;
  uint16_t offset = 0;
};

struct W2S_Packet
{
  uint8_t* ptr = nullptr;
  uint16_t size = 0;
  uint16_t offset = 0;
  int8_t rssi = 0;
};

/////////////////////////////////////////////////////////////////////////

constexpr size_t W2S_BUFFER_SIZE = 130;
alignas(uint32_t) uint8_t s_w2s_buffer[W2S_BUFFER_SIZE];

constexpr size_t S2W_BUFFER_SIZE = 130;
alignas(uint32_t) uint8_t s_s2w_buffer[S2W_BUFFER_SIZE];


template<size_t N>
struct Queue
{
  Queue(uint8_t* buffer)
    : m_buffer(buffer)
  {
    //memset(m_buffer, 0, N);
  }

  inline size_t size() const
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
  }

  inline size_t capacity() const
  {
    return N;
  }

  inline uint8_t* start_writing(size_t size) __attribute__((always_inline))
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

  inline void end_writing() __attribute__((always_inline))
  {
    m_write_start = m_write_end;
  }
  inline void cancel_writing() __attribute__((always_inline))
  {
    m_write_end = m_write_start;
  }

  inline uint8_t* start_reading(size_t& size) __attribute__((always_inline))
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
      m_read_start = 0;
      m_read_end = size;
      return m_buffer;
    }
  }

  inline void end_reading() __attribute__((always_inline))
  {
    m_read_start = m_read_end;
  }
  inline void cancel_reading()  __attribute__((always_inline))
  {
    m_read_end = m_read_start;
  }
  
private:
  uint8_t* m_buffer = nullptr;
  size_t m_write_start = 0;
  size_t m_write_end = 0;
  size_t m_read_start = 0;
  size_t m_read_end = 0;
};

////////////////////////////////////////////////////////////////////////////////////

Queue<W2S_BUFFER_SIZE> s_w2s_queue(s_w2s_buffer);
Queue<S2W_BUFFER_SIZE> s_s2w_queue(s_s2w_buffer);

////////////////////////////////////////////////////////////////////////////////////

bool start_writing_s2w_packet(S2W_Packet& packet, size_t size)
{
  size_t real_size = WLAN_HEADER_SIZE + size;
  uint8_t* buffer = s_s2w_queue.start_writing(real_size);
  if (!buffer)
  {
    packet.ptr = nullptr;
    return false;
  }
  packet.offset = 0;
  packet.size = size;
  packet.ptr = buffer;
  packet.payload_ptr = buffer + WLAN_HEADER_SIZE;
  return true;
}
void end_writing_s2w_packet(S2W_Packet& packet)
{
  s_s2w_queue.end_writing();
  packet.ptr = nullptr;
}
void cancel_writing_s2w_packet(S2W_Packet& packet)
{
  s_s2w_queue.cancel_writing();
  packet.ptr = nullptr;
}

bool start_reading_s2w_packet(S2W_Packet& packet)
{
  size_t real_size = 0;
  uint8_t* buffer = s_s2w_queue.start_reading(real_size);
  if (!buffer)
  {
    packet.ptr = nullptr;
    return false;
  }
  packet.offset = 0;
  packet.size = real_size - WLAN_HEADER_SIZE;
  packet.ptr = buffer;
  packet.payload_ptr = buffer + WLAN_HEADER_SIZE;
  return true;
}
void end_reading_s2w_packet(S2W_Packet& packet)
{
  s_s2w_queue.end_reading();
  packet.ptr = nullptr;
}
void cancel_reading_s2w_packet(S2W_Packet& packet)
{
  s_s2w_queue.cancel_reading();
  packet.ptr = nullptr;
}

////////////////////////////////////////////////////////////////////////////////////

bool start_writing_w2s_packet(W2S_Packet& packet, size_t size)
{
  uint8_t* buffer = s_w2s_queue.start_writing(size);
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
void end_writing_w2s_packet(W2S_Packet& packet)
{
  s_w2s_queue.end_writing();
  packet.ptr = nullptr;
}
void cancel_writing_w2s_packet(W2S_Packet& packet)
{
  s_w2s_queue.cancel_writing();
  packet.ptr = nullptr;
}

bool start_reading_w2s_packet(W2S_Packet& packet)
{
  size_t size = 0;
  uint8_t* buffer = s_w2s_queue.start_reading(size);
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
void end_reading_w2s_packet(W2S_Packet& packet)
{
  s_w2s_queue.end_reading();
  packet.ptr = nullptr;
}
void cancel_reading_w2s_packet(W2S_Packet& packet)
{
  s_w2s_queue.cancel_reading();
  packet.ptr = nullptr;
}

////////////////////////////////////////////////////////////////////////////////////

struct lock_guard
{
  inline lock_guard() __attribute__((always_inline))
  {
    noInterrupts();
  }
  inline ~lock_guard() __attribute__((always_inline))
  {
    interrupts();
  }
  lock_guard(const lock_guard&) = delete;
  lock_guard& operator=(const lock_guard&) = delete;
};


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
