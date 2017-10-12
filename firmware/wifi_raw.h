/*#ifndef WIFI_RAW
#define WIFI_RAW

#include "c_types.h"

struct RxControl {
	signed rssi:8;
	unsigned rate:4;
	unsigned is_group:1;
	unsigned:1;
	unsigned sig_mode:2;
	unsigned legacy_length:12;
	unsigned damatch0:1;
	unsigned damatch1:1;
	unsigned bssidmatch0:1;
	unsigned bssidmatch1:1;
	unsigned MCS:7;
	unsigned CWB:1;
	unsigned HT_length:16;
	unsigned Smoothing:1;
	unsigned Not_Sounding:1;
	unsigned:1;
	unsigned Aggregation:1;
	unsigned STBC:2;
	unsigned FEC_CODING:1;
	unsigned SGI:1;
	unsigned rxend_state:8;
	unsigned ampdu_cnt:8;
	unsigned channel:4;
	unsigned:12;
};

struct RxPacket {
	struct RxControl rx_ctl;
	uint8 data[];
};

typedef void (*wifi_raw_recv_cb_fn)(struct RxPacket *);

void ICACHE_FLASH_ATTR wifi_set_raw_recv_cb(wifi_raw_recv_cb_fn rx_fn);

#endif
*/

extern "C"
{
//esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len);
esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);

typedef union {
     uint8_t fix_rate;
     uint8_t b5;
     uint8_t b4;
 
     struct {
         uint8_t b3;
         uint8_t b2;
     } b1;
 
     struct {
         uint32_t a1;
         uint8_t  a2;
         uint8_t  a3;
         uint8_t  a4;
         uint8_t  a5;
         struct {
             uint8_t a6;
             uint8_t a7;
         } a8[4];
         uint8_t a9;
         uint8_t a10;
         uint8_t a11;
         uint8_t a12;
     } a13;
 
 } wifi_internal_rate_t;
 
 /*
 wifi_internal_rate_t rate;
 rate.fix_rate = rate;
 esp_wifi_internal_set_rate(100, 1, 4, &rate);
 */
 esp_err_t esp_wifi_internal_set_rate(int a, int b, int c, wifi_internal_rate_t *d);
 }

