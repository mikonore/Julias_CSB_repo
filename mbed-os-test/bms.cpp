#include "mbed.h"
#include "bms.h"
#include <cstdio>
//SPI device(SPI_MOSI, SPI_MISO, SPI_SCK);
//SPI spi(PC_12,PC_11,PC_10);
SPI spi_2(PA_7,PA_6,PA_5); //SPI spi_2(PB_15,PB_14,PB_13);
DigitalOut cs(PD_14);
//DigitalOut cs(PC_1);
void cs_low(void)
{
    cs=0;
}

void cs_high(void)
{
    cs=1;
}

void delay_u(uint16_t micro)
{
    wait_us(micro);
}

void delay_m(uint16_t milli)
{
    thread_sleep_for(milli);
}

void spi_enable(void) // Configures SCK frequency. Use constant defined in header file.
{
    cs = 1;                     //high as init for disable SPI
    spi_2.format(8, 3);               //byte width, spi mode
    spi_2.frequency(1000000);         //1MHz
}
    

/*
Writes an array of bytes out of the SPI port
*/
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    )
{
   //cs=0;
    for (uint8_t i = 0; i < len; i++) {
        //spi_2.write(0xFF);
        spi_2.write((int8_t)data[i]);
        //SPI.transfer((int8_t)data[i]); Setup the transfer function from SPI library
        //spi_2.transfer((int8_t)data[i]);
       // spi_transfer((int8_t)data[i]);
        //wait_us(100);
    }
    //cs=1;
}

/*
 Writes and read a set number of bytes using the SPI port.

*/

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
    //cs=0;
    for (uint8_t i = 0; i < tx_len; i++) {
        spi_2.write(tx_Data[i]);
    }
    //cs=1;

    //cs=0;
    for (uint8_t i = 0; i < rx_len; i++) {

        rx_data[i] = (uint8_t)spi_2.write(0xFF);
    }
    //cs=1;

}


uint8_t spi_read_byte(uint8_t tx_dat)
{
    uint8_t data;
    //cs=0;
    data = (uint8_t)spi_2.write(0xFF);
    //cs=1;
    return(data);
}
