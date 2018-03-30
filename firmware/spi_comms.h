#pragma once

enum SPI_Command : uint8_t
{
    SPI_CMD_SEND_PACKET = 1,
    SPI_CMD_GET_PACKET = 2,
    SPI_CMD_SET_RATE = 3,
    SPI_CMD_GET_RATE = 4,
    SPI_CMD_SETUP_FEC_CODEC = 5,
    SPI_CMD_SET_CHANNEL = 6,
    SPI_CMD_GET_CHANNEL = 7,
    SPI_CMD_SET_POWER = 8,
    SPI_CMD_GET_POWER = 9,
    SPI_CMD_GET_STATS = 10,
};

#pragma pack(push, 1) // exact fit - no padding

///////////////////////////////////////////////////////////////////////////////////////

struct SPI_Base_Response
{
  uint32_t crc : 8; //crc of the entire header
  uint32_t last_command_ok : 1;
  uint32_t pending_packets : 10;
  uint32_t next_packet_size : 11;
  uint32_t channel : 5;
  uint32_t rate : 5;
  uint32_t power : 8; //(power + 100)/10 dbm
  uint32_t rssi : 9; //rssi + 256
};

///////////////////////////////////////////////////////////////////////////////////////

struct SPI_Base_Header
{
  uint32_t crc : 8; //crc of the entire header
  uint32_t command : 5;
};

///////////////////////////////////////////////////////////////////////////////////////

struct SPI_Send_Packet_Header
{
  uint32_t crc : 8; //crc of the entire header
  uint32_t command : 5;
  uint32_t flush : 1;
  uint32_t size : 11;
};
static_assert(sizeof(SPI_Send_Packet_Header) <= 4, "");

struct SPI_Get_Packet_Header
{
  uint32_t crc : 8; //crc of the entire header
  uint32_t command : 5;
  uint32_t size : 11;
};
static_assert(sizeof(SPI_Get_Packet_Header) <= 4, "");

///////////////////////////////////////////////////////////////////////////////////////

struct SPI_Setup_Fec_Codec_Header
{
  uint32_t crc : 8; //crc of the entire header
  uint32_t command : 5;
  uint32_t fec_coding_k : 5;
  uint32_t fec_coding_n : 5;
  uint32_t fec_mtu : 11;
};
static_assert(sizeof(SPI_Setup_Fec_Codec_Header) <= 8, "");

///////////////////////////////////////////////////////////////////////////////////////

struct SPI_Set_Channel_Header
{
  uint32_t crc : 8; //crc of the entire header
  uint32_t command : 5;
  uint32_t channel : 5;
};
static_assert(sizeof(SPI_Set_Channel_Header) <= 4, "");

struct SPI_Get_Channel_Header
{
  uint32_t crc : 8; //crc of the entire header
  uint32_t command : 5;
  uint32_t channel : 5;
};
static_assert(sizeof(SPI_Get_Channel_Header) <= 4, "");

struct SPI_Get_Channel_Response_Header : public SPI_Base_Response
{
  uint8_t channel;
};

///////////////////////////////////////////////////////////////////////////////////////

struct SPI_Set_Power_Header
{
  uint32_t crc : 8; //crc of the entire header
  uint32_t command : 5;
  uint32_t power : 16; // (power + 100)/10 dbm
};
static_assert(sizeof(SPI_Set_Power_Header) <= 4, "");

struct SPI_Get_Power_Header
{
  uint32_t crc : 8; //crc of the entire header
  uint32_t command : 5;
  uint32_t power : 16; // (power + 100)/10 dbm
};
static_assert(sizeof(SPI_Get_Power_Header) <= 4, "");

struct SPI_Get_Power_Response_Header : public SPI_Base_Response
{
  uint8_t power; // power/10 dbm
};

///////////////////////////////////////////////////////////////////////////////////////

struct SPI_Set_Rate_Header
{
  uint32_t crc : 8; //crc of the entire header
  uint32_t command : 5;
  uint32_t rate : 5;
};
static_assert(sizeof(SPI_Set_Rate_Header) <= 4, "");

struct SPI_Get_Rate_Header
{
  uint32_t crc : 8; //crc of the entire header
  uint32_t command : 5;
};
static_assert(sizeof(SPI_Get_Rate_Header) <= 4, "");

struct SPI_Get_Rate_Response_Header : public SPI_Base_Response
{
  uint8_t rate;
};

///////////////////////////////////////////////////////////////////////////////////////

#pragma pack(pop)

