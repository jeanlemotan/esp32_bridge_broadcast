/*#include "lwip/pbuf.h"

#include "wifi_raw.h"

#define STATION_IF	0x00
#define STATION_MODE	0x01

#define SOFTAP_IF	0x01
#define SOFTAP_MODE	0x02


static wifi_raw_recv_cb_fn rx_func = NULL;

void ICACHE_RAM_ATTR __wrap_ppEnqueueRxq(void *a)
{
	// int i;
	// for (i = 0; i < 30; i++){
	// 	ets_uart_printf("%p ", ((void **)a)[i]);
	// 	if((uint32)((void**)a)[i]>0x30000000){
	// 		ets_uart_printf("Pointer greater than 0x30000000:\n");
	// 		int j;
	// 		for (j = 0; j < 100; j++){
	// 			ets_uart_printf("%02x ", ((uint8 **)a)[i][j]);
	// 		}
	// 		ets_uart_printf("\n\n");
	// 	}
	// }
//	if (rx_func == NULL) {
//		ets_uart_printf("Rx func is null\n");
//	}

	// 4 is the only spot that contained the packets
	// Discovered by trial and error printing the data
	if (rx_func)
		rx_func((struct RxPacket *)(((void **)a)[4]));

	__real_ppEnqueueRxq(a);
}

void ICACHE_FLASH_ATTR wifi_set_raw_recv_cb(wifi_raw_recv_cb_fn rx_fn)
{
	rx_func = rx_fn;
}

*/
