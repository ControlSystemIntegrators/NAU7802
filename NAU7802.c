#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0

// I2C address
static const uint8_t NAU7802_ADDR = 0x2A;  // Address of NAU7802  42, per datasheet, in hex.

// Registers
static const uint8_t PU_CTRL = 0x00;   //DEFAULT = 0x00
static const uint8_t CTRL1 = 0x01;     //DEFAULT = 0x00
static const uint8_t CTRL2 = 0x02;     //DEFAULT = 0x00
static const uint8_t OCAL1_B2 = 0x03;  //DEFAULT = 0x00
static const uint8_t OCAL1_B1 = 0x04;  //DEFAULT = 0x00
static const uint8_t OCAL1_B0 = 0x05;  //DEFAULT = 0x00
static const uint8_t GCAL1_B3 = 0x06;  //DEFAULT = 0x00
static const uint8_t GCAL1_B2 = 0x07;  //DEFAULT = 0x80 (128 decimal)
static const uint8_t GCAL1_B1 = 0x08;  //DEFAULT = 0x00
static const uint8_t GCAL1_B0 = 0x09;  //DEFAULT = 0x00
static const uint8_t OCAL2_B2 = 0x0A;  //DEFAULT = 0x00
static const uint8_t OCAL2_B1 = 0x0B;  //DEFAULT = 0x00
static const uint8_t OCAL2_B0 = 0x0C;  //DEFAULT = 0x00
static const uint8_t GCAL2_B3 = 0x0D;  //DEFAULT = 0x00
static const uint8_t GCAL2_B2 = 0x0E;  //DEFAULT = 0x80  (128 decimal)
static const uint8_t GCAL2_B1 = 0x0F;  //DEFAULT = 0x00
static const uint8_t GCAL2_B0 = 0x10;  //DEFAULT = 0x00
static const uint8_t I2C_CTRL = 0x11;  //DEFAULT = 0x00
static const uint8_t ADC0_B2 = 0x12;   //Read only, ADC output byte 2.
static const uint8_t ADC0_B1 = 0x13;   //Read only, ADC output byte 1.
static const uint8_t ADC0_B0 = 0x14;   //Read only, ADC output byte 0.
static const uint8_t OTP_B1 = 0x15;    //Read only.
static const uint8_t OTP_B0 = 0x16;    //Read only.
static const uint8_t REVISION = 0x1F;  //Read Only Device revision code.

// Verify that the device revision of the chip equals this.  This is used to troubleshoot and verify 
//connectivity.
//static const uint8_t KNOWN_REVISION = 0x62;

void a2d_init(void){

    sleep_ms(100);    // Allow time for A2D to get used to being powered.
    
    uint8_t DATA[20];  // Declare an array to read in the registers.
    i2c_write_blocking(I2C_PORT, NAU7802_ADDR, &PU_CTRL, 1, true);  //in this case, PU_CTRL is just being used for its address, 0x00.
    i2c_read_blocking(I2C_PORT, NAU7802_ADDR, DATA, 20, false);  //This functions to read 20 bytes.  The length must be less than or equal
    // to the size of the DATA array created previously.

    //Set up NAU7802.
    uint8_t send[2];
    send[0] = PU_CTRL;
    send[1] = 1;     //SET RESET on NAU7802.
    i2c_write_blocking(I2C_PORT, NAU7802_ADDR, send, 2, false);
    
    sleep_ms(10);

    send[0] = PU_CTRL;
    send[1] = (4 + 2);  //power up the digital and analog circuit.
    i2c_write_blocking(I2C_PORT, NAU7802_ADDR, send, 2, false);

    sleep_ms(10);

    send[0] = CTRL1;  
    send[1] = (64 + 16 + 4 + 2 + 1);  //Set PGA gain to 128 And also, set ANALOG SENBSE VOLTAGE to 3 V.
    i2c_write_blocking(I2C_PORT, NAU7802_ADDR, send, 2, false);

    sleep_ms(10);

    send[0] = PU_CTRL;
    send[1] = (128 + 4 + 2);  //power up the digital and analog circuit, Set AVDD source to internal LDO..
    i2c_write_blocking(I2C_PORT, NAU7802_ADDR, send, 2, false);

    sleep_ms(10);

    send[0] = OTP_B1;  
    send[1] = (32 + 16);
    i2c_write_blocking(I2C_PORT, NAU7802_ADDR, send, 2, false);

    sleep_ms(10);


}

int main(void){
    stdio_init_all();  //Initalize STDIO io for printing over serial.

    //Configure port for i2c communications.
    i2c_init(I2C_PORT, 100000);  // Set up port to operate at 1kHz.

    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    //Call above initalization function.  This sets up the A2D converter.
    a2d_init();


    // Next need to repeatedly read, then byte shift.
    uint8_t data[3];  // DATA is used to store the 24 bit result connected from the ADC.
    uint8_t send[2];  // SEND is used to store a byte which will be sent to the ADC.
    uint8_t troubleshoot[20];  // TROUBLESHOOT is a 20 byte array, which can store 20 bytes read from the ADC.
    uint32_t result = 0;  // constructed from DATA.  The number of counts recieved from the ADC.
    double weight = 0;  // Calculated from result knowing the ZERO OFFSET, SPAN for the load cell.

    while(true){
        send[0] = 0x00;  
        send[1] = 150;  //Turn on sampling.
        sleep_ms(20);
        i2c_write_blocking(I2C_PORT, NAU7802_ADDR, send, 2, false);  // turn on bit 4 to trigger a sample from the ADC.

        sleep_ms(110);  // wait for a conversion result to becom available.

        i2c_write_blocking(I2C_PORT, NAU7802_ADDR, &ADC0_B2, 1, true);  //Read from the ADC.  To read, first write the starting byte.
        i2c_read_blocking(I2C_PORT, NAU7802_ADDR, data, 3, false);  // hold open bus control and read in the 3 bytes on the bus.

        //Use for troubleshooting to see the three bytes coming back.
        //printf("%d  %d  %d \n",data[0], data[1], data[2]);

        //Shift the three bytes into a single 32-bit integer.
        result = (data[0]<<16) | (data[1]<<8) | (data[2]);
        //printf("The Result is: %d ", result);

        // 368000 is my load cell output with nothing but the platform.
        // 4550880- is my platform scale loaded at max capacity with
        // calibration weights.
        weight = (double) (((result - 368000) / (double) (4550880)) * 5000);
        printf("The Weight is: %4.1lf  \n",weight);

        //With this, weight looks good, however, there is noise.
        //The scale I'm using has a 5kg capacity, 2mV/V excitation.
        //We are using here, 4 million counts of the range.


    }
    



}