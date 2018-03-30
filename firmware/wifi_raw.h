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

