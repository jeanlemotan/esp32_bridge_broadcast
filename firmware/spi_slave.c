/*
#include "spi_slave.h"
#include "esp8266_peri.h"
#include "ets_sys.h"

#define INTERRUPTS_EN (SPISTRIE | SPISWSIE | SPISRSIE | SPISWBIE | SPISRBIE)

static void (*_spi_slave_rx_data_cb)() = NULL;
static void (*_spi_slave_tx_data_cb)() = NULL;
static void (*_spi_slave_rx_status_cb)(uint32_t data) = NULL;
static void (*_spi_slave_tx_status_cb)(uint32_t data) = NULL;
static uint32_t _spi_slave_buffer[8];

void ICACHE_RAM_ATTR _spi_slave_isr_handler(void *arg)
{
    uint32_t status;
    uint32_t istatus;

    istatus = SPIIR;

    if(istatus & (1 << SPII1)) { //SPI1 ISR
        status = SPI1S;
        SPI1S &= ~(INTERRUPTS_EN);//disable interrupts
        SPI1S |= SPISSRES;//reset

        if((status & SPISRBIS) != 0) {
            _spi_slave_tx_data_cb();
        }
        if((status & SPISRSIS) != 0) {
            uint32_t s = SPI1WS;
            _spi_slave_tx_status_cb(s);
        }
        if((status & SPISWSIS) != 0) {
            uint32_t s = SPI1WS;
            _spi_slave_rx_status_cb(s);
        }
        if((status & SPISWBIS) != 0) {
            _spi_slave_rx_data_cb();
        }

        SPI1S |= SPISSRES;//reset
        SPI1S &= ~(0x1F);//clear interrupts
        SPI1S |= (INTERRUPTS_EN);//enable interrupts
    } else if(istatus & (1 << SPII0)) { //SPI0 ISR
        SPI0S &= ~(0x3ff);//clear SPI ISR
    } else if(istatus & (1 << SPII2)) {} //I2S ISR
}

void spi_slave_begin(uint8_t status_len, void * arg)
{
    status_len &= 7;
    if(status_len > 4) {
        status_len = 4;    //max 32 bits
    }
    if(status_len == 0) {
        status_len = 1;    //min 8 bits
    }

    pinMode(SS, SPECIAL);
    pinMode(SCK, SPECIAL);
    pinMode(MISO, SPECIAL);
    pinMode(MOSI, SPECIAL);

    SPI1S = SPISE | SPISBE | INTERRUPTS_EN;
//    SPI1U = SPIUMISOH | SPIUCOMMAND | SPIUSSE;
    SPI1U = SPIUCOMMAND | SPIUSSE;
    SPI1CLK = 0;
    SPI1U2 = (7 << SPILCOMMAND);
//    SPI1S1 = (((status_len * 8) - 1) << SPIS1LSTA) | (0xff << SPIS1LBUF) | (7 << SPIS1LWBA) | (7 << SPIS1LRBA) | SPIS1RSTA;
    SPI1S1 = (((status_len * 8) - 1) << SPIS1LSTA) | (0x1ff << SPIS1LBUF) | (7 << SPIS1LWBA) | (7 << SPIS1LRBA) | SPIS1RSTA;
    SPI1P = (1 << 19);
    SPI1CMD = SPIBUSY;

    ETS_SPI_INTR_ATTACH(_spi_slave_isr_handler, arg);
    ETS_SPI_INTR_ENABLE();
}

void spi_slave_set_data8(uint8_t* data)
{
    uint8_t i;
    uint32_t out = 0;
    uint8_t bi = 0;
    uint8_t wi = 8;

    for(i=0; i<32; i++) 
    {
        out |= data[i] << (bi * 8);
        bi++;
        bi &= 3;
        if(!bi) {
            SPI1W(wi) = out;
            out = 0;
            wi++;
        }
    }
}


void spi_slave_on_data(void (*rxd_cb)())
{
    _spi_slave_rx_data_cb = rxd_cb;
}

void spi_slave_on_data_sent(void (*txd_cb)())
{
    _spi_slave_tx_data_cb = txd_cb;
}

void spi_slave_on_status(void (*rxs_cb)(uint32_t))
{
    _spi_slave_rx_status_cb = rxs_cb;
}

void spi_slave_on_status_sent(void (*txs_cb)(uint32_t data))
{
    _spi_slave_tx_status_cb = txs_cb;
}
*/
