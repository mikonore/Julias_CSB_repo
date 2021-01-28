
#include "mbed.h"
#include "bms.h"
#include "LTC681x.h"
#include "LTC6811.h"
#include "stdio.h"
#include <cstdio>

//#include <Terminal.h>

#define UI_BUFFER_SIZE 64
#define SERIAL_TERMINATOR '\n'

#define ENABLED 1
#define DISABLED 0

#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0


#define RED     (0xFF0000)
#define GREEN   (0x00FF00)
#define YELLOW  (0xFFFF00)
#define WHITE   (0xFFFFFF)

DigitalOut led1(LED1);
BufferedSerial pc(USBTX, USBRX);
FileHandle *mbed_override_console(int){return &pc;}
//Terminal pc_term(USBTX, USBRX);

uint8_t run_command(uint32_t cmd);
void measurement_loop(uint8_t datalog_en);
void print_menu();
void print_cells(uint8_t datalog_en);
uint8_t check_cells_4V(uint8_t datalog_en, float low_limit, float high_limit);
uint8_t check_memory_read_write(uint8_t datalog_en);
uint8_t check_cells_discharge(uint8_t datalog_en, uint8_t cell_number);
void print_open();
void print_aux(uint8_t datalog_en);
uint8_t check_aux_voltage(uint8_t datalog_en);
void print_stat();
void print_config();
void print_rxconfig();
void print_pec(void);
uint8_t compute_pec(void);
void serial_print_hex(uint8_t data);
void check_error(int error);
int8_t select_s_pin(void);
void print_rxcomm(void);
uint8_t get_rxcomm_write(void);
uint8_t get_rxcomm_read(void);
uint16_t get_rxcomm_read_temp(void);
void print_wrcomm(void);
//char get_char();
//void read_config_data(uint8_t cfg_data[][6], uint8_t nIC);

/**********************************************************
  Setup Variables
  The following variables can be modified to
  configure the software.

***********************************************************/
const uint8_t TOTAL_IC = 2;//!<number of ICs in the daisy chain
char ui_buffer[UI_BUFFER_SIZE];


//ADC Command Configurations
//const uint8_t ADC_OPT = ADC_OPT_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;//MD_7KHZ_3KHZ; //MD_26HZ_2KHZ;//MD_7KHZ_3KHZ; // See LTC6811_daisy.h for Options
//const uint8_t ADC_DCP = DCP_DISABLED; // See LTC6811_daisy.h for Options
const uint8_t ADC_DCP = DCP_ENABLED; // See LTC6811_daisy.h for Options
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; // See LTC6811_daisy.h for Options
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; // See LTC6811_daisy.h for Options
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; // See LTC6811_daisy.h for Options

const uint16_t MEASUREMENT_LOOP_TIME = 500;//milliseconds(mS)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; // Over voltage threshold ADC Code. LSB = 0.0001
const uint16_t UV_THRESHOLD = 30000; // Under voltage threshold ADC Code. LSB = 0.0001

//Loop Measurement Setup These Variables are ENABLED or DISABLED Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t READ_CONFIG = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_CELL = ENABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_AUX = DISABLED; // This is ENABLED or DISABLED
const uint8_t MEASURE_STAT = DISABLED; //This is ENABLED or DISABLED
const uint8_t PRINT_PEC = DISABLED; //This is ENABLED or DISABLED

float lower_limit = 3.57;
float higher_limit = 3.60;

// Read data from the serial interface into the ui_buffer buffer
uint8_t read_data();

// Read a float value from the serial interface
float read_float();

// Read an integer from the serial interface.
// The routine can recognize Hex, Decimal, Octal, or Binary
// Example:
// Hex:     0x11 (0x prefix)
// Decimal: 17
// Octal:   O21 (leading letter O prefix)
// Binary:  B10001 (leading letter B prefix)
int32_t read_int();

// Read a string from the serial interface.  Returns a pointer to the ui_buffer.
char *read_string();

// Read a character from the serial interface
int8_t read_char();

/************************************
  END SETUP
*************************************/

/******************************************************
 *** Global Battery Variables received from 681x commands
 These variables store the results from the LTC6811
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/

cell_asic bms_ic[TOTAL_IC];

/*!*********************************************************************
  \brief main loop
***********************************************************************/
int main(void)
{
    pc.set_baud(115200);
    pc.set_format(8,BufferedSerial::None,1); 
    uint32_t user_command;
    //pc_term.cls();
    //pc_term.locate(0,0);  
    //pc_term.foreground(WHITE);    
    printf("Press return to start...\r\n");
    run_command(34);                               
                                            
    while(1){
        run_command(34);
        while(!pc.readable()) {
            delay_m(300);
        }         // Check for user input
        user_command = read_int();      // Read the user command
        print_menu();
        printf("\r\n");
        //pc_term.cls();
        //pc_term.locate(0,0);
        //pc.baud(115200);
        spi_enable();
        LTC681x_init_cfg(TOTAL_IC, bms_ic);
        LTC6811_reset_crc_count(TOTAL_IC,bms_ic);
        LTC6811_init_reg_limits(TOTAL_IC,bms_ic);
        run_command(user_command);
        //clear the screen
    }
}

/*!*****************************************
  \brief executes the user command
*******************************************/

uint8_t run_command(uint32_t cmd)
{
    uint8_t result = 0; // final result of the run_command
    uint8_t tmp = 0;    // place holder result value
                        // for recursive intermidiary function calls
    uint8_t total_pec=0;
    uint8_t written_data = 0;
    uint8_t read_data = 0;
    uint8_t ST_manufacturor_code =0;
    uint8_t temperature_resolution = 0;
    uint16_t read_temp = 0;
    float real_temp = 0;

    int8_t error = 0;
    uint32_t conv_time = 0;
//    uint32_t user_command;
    int8_t readIC=0;
    char input = 0;
    int8_t s_pin_read=0;
    
    
    switch (cmd){
        case 1: // Write Configuration Register
            wakeup_sleep(TOTAL_IC);
            LTC6811_wrcfg(TOTAL_IC,bms_ic);
            //print_config();
            break;          

        case 2: // Read Configuration Register
            wakeup_sleep(TOTAL_IC);
            error = LTC6811_rdcfg(TOTAL_IC,bms_ic);
            check_error(error);
            //print_rxconfig();
            break;

         /* Start Cell ADC Measurement then Cell Voltage Registers reading*/
        case 34:{
            wakeup_sleep(TOTAL_IC);
            LTC681x_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
            uint32_t error = LTC681x_pollAdc();     // Wait for end of ADC conversion
            printf("Counter from spi_read_byte is: %d \r\n", error);
            wakeup_idle(TOTAL_IC);
            LTC681x_rdcv(0, TOTAL_IC,bms_ic);       // Read back all cell voltage registers

            
            printf("\r\n [case %d]",cmd);
            check_error(error);
            printf("Cell conversion completed in %d us \r\n",error);
            print_cells(DATALOG_DISABLED);
            
            break;
        }
            
        case 3: // Start Cell ADC Measurement
            wakeup_sleep(TOTAL_IC);
            LTC6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
            conv_time = LTC6811_pollAdc();
            printf("cell conversion completed in:");
            printf("%.1f",((float)conv_time/1000));
            printf("mS\r\n");
            //result not changed, will return 0 (that's okay because we don't
            //care about the return of this particular case)
            break;

        case 4: // Read Cell Voltage Registers
            wakeup_sleep(TOTAL_IC);
            error = LTC6811_rdcv(0, TOTAL_IC,bms_ic); // Set to read back all cell voltage registers
            check_error(error);
            print_cells(DATALOG_DISABLED);
            
            break;
    
        case 56:
            // 5 then (adapted) 6
            tmp = run_command(5);
            tmp = run_command(6);
            result = check_aux_voltage(DATALOG_DISABLED);
            printf("                 Test completed\r\n");
            break;
            
        case 5: // Start GPIO ADC Measurement
            wakeup_sleep(TOTAL_IC);
            LTC6811_adax(ADC_CONVERSION_MODE , AUX_CH_TO_CONVERT);
            LTC6811_pollAdc();
            printf("aux conversion completed\r\n");
            printf("\r\n");
            break;

        case 6: // Read AUX Voltage Registers
            wakeup_sleep(TOTAL_IC);
            error = LTC6811_rdaux(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
            check_error(error);
            printf("GPIO and VREF voltages: \r\n");
            print_aux(DATALOG_DISABLED);
            break;

        case 7: // Start Status ADC Measurement
            wakeup_sleep(TOTAL_IC);
            LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
            LTC6811_pollAdc();
            printf("stat conversion completed\r\n");
            printf("\r\n");
            break;

        case 8: // Read Status registers
            wakeup_sleep(TOTAL_IC);
            error = LTC6811_rdstat(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
            check_error(error);
            print_stat();
            break;

        case 9: // Loop Measurements
            printf("transmit 'm' to quit\r\n");
            wakeup_sleep(TOTAL_IC);
            LTC6811_wrcfg(TOTAL_IC,bms_ic);
            while (input != 'm') {
                //if (pc.readable()) {
                    input = read_char();
                //}

                measurement_loop(DATALOG_DISABLED);

                delay_m(MEASUREMENT_LOOP_TIME);
            }
            //print_menu();
            break;

        case 10: // Run open wire self test
            // the open wire self test is passed if there are 0 PEC errors
            
            tmp = run_command(3);
            tmp = run_command(4);
            
            total_pec = compute_pec();            
            if(total_pec){  // if there are errors, we return 0 for FAILURE
                printf("\r\nThere were PEC errors\r\n");
                result = 0;
            }else{
                result = 1; // no errors, success
            }
            break;
                   
        case 11: // Read in raw configuration data
            LTC6811_reset_crc_count(TOTAL_IC,bms_ic);
            break;

        case 12:  // Run the ADC/Memory Self Test
            wakeup_sleep(TOTAL_IC);
            error = LTC6811_run_cell_adc_st(CELL,ADC_CONVERSION_MODE,bms_ic);
            printf("%d", error);
            printf(" : errors detected in Digital Filter and CELL Memory\r\n");

            wakeup_sleep(TOTAL_IC);
            error = LTC6811_run_cell_adc_st(AUX,ADC_CONVERSION_MODE, bms_ic);
            printf("%d",error);
            printf(" : errors detected in Digital Filter and AUX Memory\r\n");

            wakeup_sleep(TOTAL_IC);
            error = LTC6811_run_cell_adc_st(STAT,ADC_CONVERSION_MODE, bms_ic);
            printf("%d",error);
            printf(" : errors detected in Digital Filter and STAT Memory\r\n");
            print_menu();
            break;
            
        case 98: // Enable a discharge transistor
            printf("Please enter the Spin number\r\n");
            readIC = read_int();
            printf("pin number-> %d \r\n", readIC);
            LTC6811_set_discharge(readIC,TOTAL_IC,bms_ic);
            wakeup_sleep(TOTAL_IC);
            LTC6811_wrcfg(TOTAL_IC,bms_ic);
            print_config();
            break;
            
        case 134:
            //13 then 14; looping on all the cell thingies
            // check if this 12 is not somewhere (so not hardcoded)
            //!! pin numbers go from 1 to 12 and not
            // 0 to 11 because of a hack in LTC6811_set_discharge
            // -> bms_ic[0].ic_reg.cell_channels. Something something check the value
            // is it 11 or 12, should be good
            
            tmp = run_command(14);
            // array of results
            // DO NOT PUT THE bms_ic[0].ic_reg.cell_channels INSTEAD OF 12
            // OR YOU WILL GET A STRANGE ERROR WITH THE SWITCH AT SOME POINT,
            // FOR SOME REASON
            uint8_t cells_discharged[12]; //12 cells

            for(uint8_t cell_number = 1; cell_number < (bms_ic[0].ic_reg.cell_channels+1); cell_number++){
                //printf("---loop---\r\n");
                //printf("%d",cell_number);
                //case 13
                //printf("pin number-> %d \r\n", cell_number);
                LTC6811_set_discharge(cell_number,TOTAL_IC,bms_ic);
                wakeup_sleep(TOTAL_IC);//removing
                LTC6811_wrcfg(TOTAL_IC,bms_ic);
                //print_config();
                wakeup_idle(TOTAL_IC);//adding
                error = LTC6811_rdcfg(TOTAL_IC,bms_ic);
                check_error(error);
                //print_rxconfig();
                //case 1
                tmp = run_command(1);
                //case 3
                tmp = run_command(3);
                //case 4
                tmp = run_command(4);
                cells_discharged[cell_number-1] = check_cells_discharge(DATALOG_DISABLED, cell_number);
                tmp = run_command(14);
            }
            // start by putting to true and then put to false if we have at least
            // one error
            result = 1;
            for(uint8_t i = 0; i < (bms_ic[0].ic_reg.cell_channels); i++){
                if (cells_discharged[i] == 0){
                    //printf("During discharge of pin %d voltage was not correctly reduced\r\n",(i+1));
                    result = 0;    
                }
            }
            break;
            
        case 13: // Enable a discharge transistor
            printf("Please enter the Spin number\r\n");
            readIC = read_int();
            printf("pin number-> %d \r\n", readIC);
            //wakeup_sleep(TOTAL_IC);//adding
            LTC6811_set_discharge(readIC,TOTAL_IC,bms_ic);
            //wait_ms(2000);
            wakeup_sleep(TOTAL_IC);//removing
            LTC6811_wrcfg(TOTAL_IC,bms_ic);
            //print_config();
            wakeup_idle(TOTAL_IC);//adding
            error = LTC6811_rdcfg(TOTAL_IC,bms_ic);
            check_error(error);
            //print_rxconfig();
            //printf("-- end case 13 -- \r\n");
            break;

        case 14: // Clear all discharge transistors
            clear_discharge(TOTAL_IC,bms_ic);
            wakeup_sleep(TOTAL_IC);
            LTC6811_wrcfg(TOTAL_IC,bms_ic);
            //print_config();
            break;

        case 15: // Clear all ADC measurement registers
            wakeup_sleep(TOTAL_IC);
            LTC6811_clrcell();
            LTC6811_clraux();
            LTC6811_clrstat();
            printf("All Registers Cleared\r\n");
            break;

        case 16: // Run the Mux Decoder Self Test
            wakeup_sleep(TOTAL_IC);
            LTC6811_diagn();
            delay_m(5);
            error = LTC6811_rdstat(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
            check_error(error);
            error = 0;
            for (int ic = 0;
                    ic<TOTAL_IC;
                    ic++) {
                if (bms_ic[ic].stat.mux_fail[0] != 0) error++;
            }
            if (error==0) printf("Mux Test: PASS\r\n");
            else printf("Mux Test: FAIL\r\n");

            break;

        case 17: // Run ADC Overlap self test
            wakeup_sleep(TOTAL_IC);
            error = (int8_t)LTC6811_run_adc_overlap(TOTAL_IC,bms_ic);
            if (error==0) printf("Overlap Test: PASS\r\n");
            else printf("Overlap Test: FAIL\r\n");
            break;

        case 18: // Run ADC Redundancy self test
            wakeup_sleep(TOTAL_IC);
            error = LTC6811_run_adc_redundancy_st(ADC_CONVERSION_MODE,AUX,TOTAL_IC, bms_ic);
            printf("%d",error);
            printf(" : errors detected in AUX Measurement\r\n");

            wakeup_sleep(TOTAL_IC);
            error = LTC6811_run_adc_redundancy_st(ADC_CONVERSION_MODE,STAT,TOTAL_IC, bms_ic);
            printf("%d",error);
            printf(" : errors detected in STAT Measurement\r\n");
            break;

        case 19:
            LTC6811_run_openwire(TOTAL_IC, bms_ic);
            print_open();
            break;

        case 20: //Datalog print option Loop Measurements
            printf("transmit 'm' to quit\r\n");
            wakeup_sleep(TOTAL_IC);
            LTC6811_wrcfg(TOTAL_IC,bms_ic);
            while (input != 'm') {
                //if (pc.readable()) {
                    input = read_char();
                //}

                measurement_loop(DATALOG_ENABLED);

                delay_m(MEASUREMENT_LOOP_TIME);
            }
            print_menu();
            break;
            
         case 29: // write byte I2C Communication on the GPIO Ports(using I2C eeprom 24LC025)
            /************************************************************
             Ensure to set the GPIO bits to 1 in the CFG register group. 
            *************************************************************/   
            for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
            {
                bms_ic[current_ic].com.tx_data[0]= 0x6A; // Icom Start(6) + I2C_address D0 (0xA0)
                bms_ic[current_ic].com.tx_data[1]= 0x00;// Fcom master ACK(0)  
                bms_ic[current_ic].com.tx_data[2]= 0x00; // Icom Blank (0) + eeprom address D1 high(0x00)
                //bms_ic[current_ic].com.tx_data[3]= 0x08; // Fcom master NACK(8)   
                bms_ic[current_ic].com.tx_data[3]= 0x00; // eeprom address D1 low(0x00) + Fcom master ACK(0b0000)   
                bms_ic[current_ic].com.tx_data[4]= 0x07; // Icom Blank (0) + data D2 high (0b0001)
                bms_ic[current_ic].com.tx_data[5]= 0x39; // data D2 low (0xA)Fcom master NACK + Stop(9)
            }
            wakeup_sleep(TOTAL_IC);       
            LTC6811_wrcomm(TOTAL_IC,bms_ic); // write to comm register    
            //print_wrcomm(); // print transmitted data from the comm register
            
            wakeup_idle(TOTAL_IC);
            LTC6811_stcomm(); // data length=3 // initiates communication between master and the I2C slave
            
            wakeup_idle(TOTAL_IC);
            error = LTC6811_rdcomm(TOTAL_IC,bms_ic); // read from comm register                       
            check_error(error);
            //print_rxcomm(); // print received data into the comm register  
            break;    

        case 30: // Read byte data I2C Communication on the GPIO Ports(using I2C eeprom 24LC025)
            /************************************************************
             Ensure to set the GPIO bits to 1 in the CFG register group.  
            *************************************************************/     
            for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
            {
            
            // STEP1.1: WRCOMM: choose the I2C configuration values to write from the uC to the I2C master
            
            //Reading the ST manufacturer code from the Identification page:
            //address in IDENTIFICATION page: 0x00
            //value EXPECTED TO BE READ address 0x00: 0x20
            //Device identifier: 0b1011
            //operation: Read -> 1
            //device address: 0b000 (since all 3 pins are to ground this is the only value we can
            //use here) 
            //data D2: 0x01 (we want to read 1 byte) (not sure this is the meaning of data D2
            
            bms_ic[current_ic].com.tx_data[0]= 0x6B; // Icom Start(6) + identification code/device identifier = 0b1011 for ID page 
            bms_ic[current_ic].com.tx_data[1]= 0x00; // WRITE to set the word address (that we want to read). This is for
                                                     // resetting a hardware pointer. + FCOM Master ACK (0b0000) 
            bms_ic[current_ic].com.tx_data[2]= 0x00; // Icom Blank (0b0000) + address of ST manufacturer code (high 4 bits)
            bms_ic[current_ic].com.tx_data[3]= 0x00; // address of ST manufacturer code (low 4 bits) + FCOM Master ACK (0b0000)   
            bms_ic[current_ic].com.tx_data[4]= 0x6B; // Icom Start (6) + device select again (B1 for READ the identification page)
            //bms_ic[current_ic].com.tx_data[5]= 0x19; // data D2(low 4 bits) + Fcom master (NACK and Stop =0x9) 
            bms_ic[current_ic].com.tx_data[5]= 0x10; // data D2(low 4 bits) + FCOM Master ACK (0b0000) 
            }
            // STEP1.2: WRCOMM: write the 6 bytes of COMM values in the COMM registers of the MASTER
            wakeup_sleep(TOTAL_IC);         
            LTC6811_wrcomm(TOTAL_IC,bms_ic); // write to comm register 
            
            // STEP 2: STCOMM: start the I2C communication with the configuration defined in SETP 1 during WRCOMM operation
            wakeup_idle(TOTAL_IC);
            LTC6811_stcomm(); // data length=3 // initiates communication between master and the I2C slave
            //print_rxcomm(); 
            
            //wait 72 cock cycles -> done in stcomm function? not sure what the for loop is doing.
            
            wakeup_idle(TOTAL_IC);
            error = LTC6811_rdcomm(TOTAL_IC,bms_ic); // read from comm register                
            check_error(error);
            //print_rxcomm(); // print received data from the comm register    
      
            //SETP 3: RDCOMM: read back the values of COMM registers from the I2C master to uC
            for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
            { 
                // Communication control bits and communication data bytes. Refer to the data sheet.       
                //bms_ic[current_ic].com.tx_data[0]= 0x0F; // Icom Blank (0) + data D0 (FF)
                //bms_ic[current_ic].com.tx_data[1]= 0xF9; // Fcom master NACK + Stop(9) 
                //bms_ic[current_ic].com.tx_data[2]= 0x7F; // Icom No Transmit (7) + data D1 (FF)
                //bms_ic[current_ic].com.tx_data[3]= 0xF9; // Fcom master NACK + Stop(9)
                //bms_ic[current_ic].com.tx_data[4]= 0x7F; // Icom No Transmit (7) + data D2 (FF)
                //bms_ic[current_ic].com.tx_data[5]= 0xF9; // Fcom master NACK + Stop(9) 
                // Communication control bits and communication data bytes. Refer to the data sheet.       
                bms_ic[current_ic].com.tx_data[0]= 0x0F; // Icom Blank (0) + data D0 (FF)
                bms_ic[current_ic].com.tx_data[1]= 0xF9; // Fcom master NACK + Stop(9) 
                bms_ic[current_ic].com.tx_data[2]= 0x7F; // Icom No Transmit (7) + data D1 (FF)
                bms_ic[current_ic].com.tx_data[3]= 0xF9; // Fcom master NACK + Stop(9)
                bms_ic[current_ic].com.tx_data[4]= 0x7F; // Icom No Transmit (7) + data D2 (FF)
                bms_ic[current_ic].com.tx_data[5]= 0xF9; // Fcom master NACK + Stop(9) 
            }
            wakeup_idle(TOTAL_IC);
            LTC6811_wrcomm(TOTAL_IC,bms_ic); // write to comm register
            
            wakeup_idle(TOTAL_IC);
            LTC6811_stcomm(); // data length=1 // initiates communication between master and the I2C slave  
            
            wakeup_idle(TOTAL_IC);
            error = LTC6811_rdcomm(TOTAL_IC,bms_ic); // read from comm register                
            check_error(error);
            //print_rxcomm(); // print received data from the comm register    
            break;
                
        case 90:
            // executes 29,30 and 99
            // first executes the commands normally for the prints then check the result
            run_command(29);
            // get the relevant data from command 29
            written_data = get_rxcomm_write();
            run_command(99);
            read_data = get_rxcomm_read();
            run_command(30);
            printf("Reading device ID, should be 0x20: ");
            ST_manufacturor_code = get_rxcomm_read();
            if (written_data == read_data && written_data == 0x73 && ST_manufacturor_code == 0x20){
                result = 1;
            }
            break;
        case 38:
            // executes 29,30 and 99
            // first executes the commands normally for the prints then check the result
            //run_command(29);
            // get the relevant data from command 29
            //uint8_t written_data = get_rxcomm_write();
            //run_command(99);
            
            // step 1: write the configuration register (register 0x01) with desired resolution
            // see datasheet 7.5.3.5 for values meaning
            temperature_resolution = (0b11) << 5;
            // step 2: read the temperature (register 0x00)
            run_command(40);
            read_temp = get_rxcomm_read_temp();
            real_temp = read_temp*0.0625;
            printf("real temperature: %.2f\r\n",real_temp);
            if (real_temp  > 20 && real_temp < 30){
                result = 1;
            }
            break;
         case 40: // Read byte data I2C Communication on the GPIO Ports(using I2C eeprom 24LC025)
            /************************************************************
             Ensure to set the GPIO bits to 1 in the CFG register group.  
            *************************************************************/     
            for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
            {
            
            // STEP1.1: WRCOMM: choose the I2C configuration values to write from the uC to the I2C master
            
            //Reading the ST manufacturer code from the Identification page:
            //address in IDENTIFICATION page: 0x00
            //value EXPECTED TO BE READ address 0x00: 0x20
            //Device identifier: 0b1011
            //operation: Read -> 1
            //device address: 0b000 (since all 3 pins are to ground this is the only value we can
            //use here) 
            //data D2: 0x01 (we want to read 1 byte) (not sure this is the meaning of data D2
            
            bms_ic[current_ic].com.tx_data[0]= 0x69; // Icom Start(6) + identification code/device identifier = 0b1011 for ID page 
            bms_ic[current_ic].com.tx_data[1]= 0x00; // WRITE to set the word address (that we want to read). This is for
                                                     // resetting a hardware pointer. + FCOM Master ACK (0b0000) 
            bms_ic[current_ic].com.tx_data[2]= 0x00; // Icom Blank (0b0000) + address of ST manufacturer code (high 4 bits)
            bms_ic[current_ic].com.tx_data[3]= 0x00; // address of ST manufacturer code (low 4 bits) + FCOM Master ACK (0b0000)   
            bms_ic[current_ic].com.tx_data[4]= 0x69; // Icom Start (6) + device select again (B1 for READ the identification page)
            bms_ic[current_ic].com.tx_data[5]= 0x10; // data D2(low 4 bits) + FCOM Master ACK (0b0000) 
            }
            // STEP1.2: WRCOMM: write the 6 bytes of COMM values in the COMM registers of the MASTER
            wakeup_sleep(TOTAL_IC);         
            LTC6811_wrcomm(TOTAL_IC,bms_ic); // write to comm register 
            
            // STEP 2: STCOMM: start the I2C communication with the configuration defined in SETP 1 during WRCOMM operation
            wakeup_idle(TOTAL_IC);
            LTC6811_stcomm(); // data length=3 // initiates communication between master and the I2C slave
            //print_rxcomm(); 
            
            //wait 72 cock cycles -> done in stcomm function? not sure what the for loop is doing.
            
            wakeup_idle(TOTAL_IC);
            error = LTC6811_rdcomm(TOTAL_IC,bms_ic); // read from comm register                
            check_error(error);
            //print_rxcomm(); // print received data from the comm register    
      
            //SETP 3: RDCOMM: read back the values of COMM registers from the I2C master to uC
            for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
            { 
                // Communication control bits and communication data bytes. Refer to the data sheet.       
                bms_ic[current_ic].com.tx_data[0]= 0x0F; // Icom Blank (0) + data D0 (F)
                bms_ic[current_ic].com.tx_data[1]= 0xF0; // data D0 (F) + master ACK
                bms_ic[current_ic].com.tx_data[2]= 0x0F; // Blank + data D1 (FF)
                bms_ic[current_ic].com.tx_data[3]= 0xF9; // Fcom master NACK + Stop(9)
                bms_ic[current_ic].com.tx_data[4]= 0x7F; // Icom No Transmit (7) + data D2 (FF)
                bms_ic[current_ic].com.tx_data[5]= 0xF9; // Fcom master NACK + Stop(9) 
            }
            wakeup_idle(TOTAL_IC);
            LTC6811_wrcomm(TOTAL_IC,bms_ic); // write to comm register
            
            wakeup_idle(TOTAL_IC);
            LTC6811_stcomm(); // data length=1 // initiates communication between master and the I2C slave  
            
            wakeup_idle(TOTAL_IC);
            error = LTC6811_rdcomm(TOTAL_IC,bms_ic); // read from comm register                
            check_error(error);
            //print_rxcomm(); // print received data from the comm register    
            break;  
              
        case 99:
            // do this after the write to address 0x00
            // COMMENTS ARE NOT UP TO DATE (copy pasted from case 30)
            for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
            {
                // STEP1.1: WRCOMM: choose the I2C configuration values to write from the uC to the I2C master
                
                //Reading the ST manufacturer code from the Identification page:
                //address in IDENTIFICATION page: 0x00
                //value EXPECTED TO BE READ address 0x00: 0x20
                //Device identifier: 0b1011
                //operation: Read -> 1
                //device address: 0b000 (since all 3 pins are to ground this is the only value we can
                //use here) 
                //data D2: 0x01 (we want to read 1 byte) (not sure this is the meaning of data D2
                
                bms_ic[current_ic].com.tx_data[0]= 0x6A; // Icom Start(6) + identification code/device identifier = 0b1011 for ID page 
                bms_ic[current_ic].com.tx_data[1]= 0x00; // WRITE to set the word address (that we want to read). This is for
                                                         // resetting a hardware pointer. + FCOM Master ACK (0b0000) 
                bms_ic[current_ic].com.tx_data[2]= 0x00; // Icom Blank (0b0000) + address of ST manufacturer code (high 4 bits)
                bms_ic[current_ic].com.tx_data[3]= 0x00; // address of ST manufacturer code (low 4 bits) + FCOM Master ACK (0b0000)   
                bms_ic[current_ic].com.tx_data[4]= 0x6A; // Icom Start (6) + device select again (B1 for READ the identification page)
                //bms_ic[current_ic].com.tx_data[5]= 0x19; // data D2(low 4 bits) + Fcom master (NACK and Stop =0x9) 
                bms_ic[current_ic].com.tx_data[5]= 0x10; // data D2(low 4 bits) + FCOM Master ACK (0b0000) 
            }
            // STEP1.2: WRCOMM: write the 6 bytes of COMM values in the COMM registers of the MASTER
            wakeup_sleep(TOTAL_IC);         
            LTC6811_wrcomm(TOTAL_IC,bms_ic); // write to comm register 
            
            // STEP 2: STCOMM: start the I2C communication with the configuration defined in SETP 1 during WRCOMM operation
            wakeup_idle(TOTAL_IC);
            LTC6811_stcomm(); // data length=3 // initiates communication between master and the I2C slave
            
            //wait 72 cock cycles -> done in stcomm function? not sure what the for loop is doing.
            
            wakeup_idle(TOTAL_IC);
            error = LTC6811_rdcomm(TOTAL_IC,bms_ic); // read from comm register                
            check_error(error);
            //print_rxcomm(); // print received data from the comm register    
            
            //SETP 3: RDCOMM: read back the values of COMM registers from the I2C master to uC
            for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++) 
            { 
                // Communication control bits and communication data bytes. Refer to the data sheet.       
                bms_ic[current_ic].com.tx_data[0]= 0x0F; // Icom Blank (0) + data D0 (FF)
                bms_ic[current_ic].com.tx_data[1]= 0xF9; // Fcom master NACK + Stop(9) 
                bms_ic[current_ic].com.tx_data[2]= 0x7F; // Icom No Transmit (7) + data D1 (FF)
                bms_ic[current_ic].com.tx_data[3]= 0xF9; // Fcom master NACK + Stop(9)
                bms_ic[current_ic].com.tx_data[4]= 0x7F; // Icom No Transmit (7) + data D2 (FF)
                bms_ic[current_ic].com.tx_data[5]= 0xF9; // Fcom master NACK + Stop(9) 
            }  
            
            wakeup_idle(TOTAL_IC);
            LTC6811_wrcomm(TOTAL_IC,bms_ic); // write to comm register
            
            wakeup_idle(TOTAL_IC);
            LTC6811_stcomm(); // data length=1 // initiates communication between master and the I2C slave  
            
            wakeup_idle(TOTAL_IC);
            error = LTC6811_rdcomm(TOTAL_IC,bms_ic); // read from comm register                
            check_error(error);
            //print_rxcomm(); // print received data from the comm register    
            break;

        case 'm': //prints menu
            print_menu();
            break;

        default:
            printf("Incorrect Option\r\n");
            break;
    }
    return result;
}

void measurement_loop(uint8_t datalog_en)
{
    int8_t error = 0;
    if (WRITE_CONFIG == ENABLED) {
        wakeup_sleep(TOTAL_IC);
        LTC6811_wrcfg(TOTAL_IC,bms_ic);
        print_config();
    }

    if (READ_CONFIG == ENABLED) {
        wakeup_sleep(TOTAL_IC);
        error = LTC6811_rdcfg(TOTAL_IC,bms_ic);
        check_error(error);
        print_rxconfig();
    }

    if (MEASURE_CELL == ENABLED) {
        wakeup_idle(TOTAL_IC);
        LTC6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
        LTC6811_pollAdc();
        wakeup_idle(TOTAL_IC);
        error = LTC6811_rdcv(0, TOTAL_IC,bms_ic);
        check_error(error);
        print_cells(datalog_en);

    }

    if (MEASURE_AUX == ENABLED) {
        wakeup_idle(TOTAL_IC);
        LTC6811_adax(ADC_CONVERSION_MODE , AUX_CH_ALL);
        LTC6811_pollAdc();
        wakeup_idle(TOTAL_IC);
        error = LTC6811_rdaux(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
        check_error(error);
        print_aux(datalog_en);
    }

    if (MEASURE_STAT == ENABLED) {
        wakeup_idle(TOTAL_IC);
        LTC6811_adstat(ADC_CONVERSION_MODE, STAT_CH_ALL);
        LTC6811_pollAdc();
        wakeup_idle(TOTAL_IC);
        error = LTC6811_rdstat(0,TOTAL_IC,bms_ic); // Set to read back all aux registers
        check_error(error);
        print_stat();
    }

    if (PRINT_PEC == ENABLED) {
        print_pec();
    }

}


/*!*********************************
  \brief Prints the main menu
***********************************/
void print_menu()
{
    printf("Please enter LTC6811 Command\r\n");
    printf("Write Configuration: 1            | Reset PEC Counter: 11\r\n");
    printf("Read Configuration: 2             | Run ADC Self Test: 12\r\n");
    printf("Start Cell Voltage Conversion: 3  | Set Discharge: 13\r\n");
    printf("Read Cell Voltages: 4             | Clear Discharge: 14\r\n");
    printf("Start Aux Voltage Conversion: 5   | Clear Registers: 15\r\n");
    printf("Read Aux Voltages: 6              | Run Mux Self Test: 16\r\n");
    printf("Start Stat Voltage Conversion: 7  | Run ADC overlap Test: 17\r\n");
    printf("Read Stat Voltages: 8             | Run Digital Redundancy Test: 18\r\n");
    printf("loop Measurements: 9              | Run Open Wire Test: 19\r\n");
    printf("Read PEC Errors: 10               |  Loop measurements with datalog output: 20\r\n");
    printf("\r\n");
    printf("Please enter command:\r\n");
    printf("\r\n");
}

/*!************************************************************
  \brief Prints cell voltage codes to the serial port
 *************************************************************/
void print_cells(uint8_t datalog_en)
{
    for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {

        if (datalog_en == 0) {
            printf("IC%d, ", current_ic+1);
            for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++) {
                printf("C%d:", i+1);
                //printf("%.4f, ", bms_ic[current_ic].cells.c_codes[i]*0.0001);
                printf("%d,
                 ", bms_ic[current_ic].cells.c_codes[i]/**0.0001*/);
            }
            printf("\r\n");
        } else {
            printf("Cells, ");
            for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++) {
                printf("%.4f, ",bms_ic[current_ic].cells.c_codes[i]*0.0001);
            }
        }

    }
    printf("\r\n");
}

uint8_t check_cells_4V(uint8_t datalog_en,float low_limit, float high_limit)
{

    uint8_t result = 1;
    // if a cell is out of bounds, we will return an error (0)
    for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {
        if (datalog_en == 0) {
            for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++) {
                if(bms_ic[current_ic].cells.c_codes[i]*0.0001 < low_limit || bms_ic[current_ic].cells.c_codes[i]*0.0001 > high_limit){
                    // the test fails if the voltage is out of bounds
                    printf("Cell %d out of acceptable range (%.2fV - %.2fV) -- ",(i+1)+(current_ic*12),(current_ic+1),low_limit,high_limit);
                    printf("measured Voltage: %.2fV\r\n",bms_ic[current_ic].cells.c_codes[i]*0.0001);
                    result = 0; 
                    } 
                else{
                    printf("Cell %d measured Voltage: %.2fV\r\n",(i+1)+(current_ic*12),bms_ic[current_ic].cells.c_codes[i]*0.0001);
                    }
                
                
            }
        } else {
            for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++) {
                if(bms_ic[current_ic].cells.c_codes[i]*0.0001 < 3.3 ||
                bms_ic[current_ic].cells.c_codes[i]*0.0001 > 4.1){
                    printf("Cell %d of IC %d out of acceptable range (3.3V - 4.1V) -- ",(i+1),(current_ic+1));
                    printf("Voltage was: %.2fV\r\n",bms_ic[current_ic].cells.c_codes[i]*0.0001);
                    result = 0;    
                }  
            }
        }

    }
    return result;
}

uint8_t check_cells_discharge(uint8_t datalog_en, uint8_t cell_number)
{
    // cell_number from 1 to 12 here as input.
    // if a cell is out of bounds, we will return an error (0)
    uint8_t result = 1;
    for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {
        if (datalog_en == 0) {
            for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++) {
                if (i == cell_number-1){
                    if(bms_ic[current_ic].cells.c_codes[i]*0.0001 < 1.9 ||
                    bms_ic[current_ic].cells.c_codes[i]*0.0001 > 2.3){
                        printf("During discharge of cell %d of IC %d, cell %d out of acceptable range (1.9V - 2.3V) -- ",cell_number,(current_ic+1),(i+1));
                        printf("Voltage was: %.2fV\r\n",bms_ic[current_ic].cells.c_codes[i]*0.0001);
                        result = 0;
                    }
                }else if(bms_ic[current_ic].cells.c_codes[i]*0.0001 < 3.9 ||
                bms_ic[current_ic].cells.c_codes[i]*0.0001 > 4.23){
                    // the test fails if the voltage is out of bounds
                    printf("During discharge of cell %d of IC %d, cell %d out of acceptable range (3.9V - 4.23V) -- ",cell_number,(current_ic+1),(i+1));
                    printf("Voltage was: %.2fV\r\n",bms_ic[current_ic].cells.c_codes[i]*0.0001);
                    result = 0;    
                }
            }
        } else {
            //This case is not pretty printed
            for (int i=0; i<bms_ic[0].ic_reg.cell_channels; i++) {
                if (i == cell_number-1){
                    if(bms_ic[current_ic].cells.c_codes[i]*0.0001 < 1.9 ||
                    bms_ic[current_ic].cells.c_codes[i]*0.0001 > 2.3){
                        printf("During discharge of cell %d of IC %d, cell %d out of acceptable range (1.9V - 2.3V) -- ",cell_number,(current_ic+1),(i+1));
                        printf("Voltage was: %.2fV\r\n",bms_ic[current_ic].cells.c_codes[i]*0.0001);
                        result = 0;
                    } 
                }else if(bms_ic[current_ic].cells.c_codes[i]*0.0001 < 3.9 ||
                bms_ic[current_ic].cells.c_codes[i]*0.0001 > 4.23){
                    // the test fails if the voltage is out of bounds
                    printf("During discharge of cell %d of IC %d, cell %d out of acceptable range (3.9V - 4.23V) -- ",cell_number,(current_ic+1),(i+1));
                    printf("Voltage was: %.2fV\r\n",bms_ic[current_ic].cells.c_codes[i]*0.0001);
                    result = 0;
                }  
            }
        }
    }
    return result;
}
/*!****************************************************************************
  \brief Prints Open wire test results to the serial port
 *****************************************************************************/
void print_open()
{
    for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++) {
        if (bms_ic[current_ic].system_open_wire == 0) {
            printf("No Opens Detected on IC%d\r\n", current_ic+1);
        } else {
            for (int cell=0; cell<bms_ic[0].ic_reg.cell_channels+1; cell++) {
                if ((bms_ic[current_ic].system_open_wire &(1<<cell))>0) {
                    printf("There is an open wire on IC%d Channel: %d\r\n", current_ic + 1, cell);
                }
            }
        }
    }
}

/*!****************************************************************************
  \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_aux(uint8_t datalog_en)
{
    for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++) {
        if (datalog_en == 0) {
            printf(" IC%d", current_ic+1);
            for (int i=0; i < 5; i++) {
                printf(" GPIO%d: %.4f,", i+1, bms_ic[current_ic].aux.a_codes[i]*0.0001);
            }
            printf("Vref: %.4f\r\n", bms_ic[current_ic].aux.a_codes[5]*0.0001);
        } else {
            printf("AUX, ");
            for (int i=0; i < 6; i++) {
                printf("%.4f,", bms_ic[current_ic].aux.a_codes[i]*0.0001);
            }
        }
    }
    printf("\r\n");
}
uint8_t check_aux_voltage(uint8_t datalog_en)
{

    uint8_t result = 1;
    for (int current_ic = 0 ; current_ic < TOTAL_IC; current_ic++) {
        if (datalog_en == 0) {
            //printf(" IC%d", current_ic+1);
            for (int i=0; i < 5; i++){
                if (current_ic == 0){//IC 1
                    if(i < 3){//GPIO 1,2,3 corresponding to i=0,1,2
                        if(bms_ic[current_ic].aux.a_codes[i]*0.0001 < 1.4||
                        bms_ic[current_ic].aux.a_codes[i]*0.0001 > 1.6){
                            printf("Thermistor %d out of acceptable range (1.4V - 1.6V) -- ",(i+1)+(current_ic*3));
                            printf("Voltage was: %.2fV\r\n",bms_ic[current_ic].aux.a_codes[i]*0.0001);
                            result = 0;
                            
                        }
                    else{
                        printf("Thermistor %d measured Voltage : %.2fV\r\n",(i+1),bms_ic[current_ic].aux.a_codes[i]*0.0001);
                        }
                
                    }
                }else if(current_ic == 1){//IC 2
                    if(bms_ic[current_ic].aux.a_codes[i]*0.0001 < 1.4||
                    bms_ic[current_ic].aux.a_codes[i]*0.0001 > 1.6){
                        printf("Thermistor %d out of acceptable range (1.4V - 1.6V) -- ",(i+1)+(current_ic*3));
                        printf("Voltage was: %.2fV\r\n",bms_ic[current_ic].aux.a_codes[i]*0.0001);
                        result = 0;          
                
                    }
                    else{
                        printf("Thermistor %d measured Voltage : %.2fV\r\n",(i+1)+3,bms_ic[current_ic].aux.a_codes[i]*0.0001);
                        }                   
                }else{
                    printf("This test is only made for 2 IC's!\r\n");    
                    printf("Check the schematic and change this code accordingly!\r\n");
                    result = 0; 
                }
            }
            /*
            if(bms_ic[current_ic].aux.a_codes[5]*0.0001 < 2.99
            || bms_ic[current_ic].aux.a_codes[5]*0.0001 > 3.09){
                printf("VREF of IC %d out of acceptable range (2.99V - 3.09V) -- ",(current_ic+1));
                printf("Voltage was: %.2fV\r\n",bms_ic[current_ic].aux.a_codes[5]*0.0001);
                result = 0;               
            }
            */
        }else {
            printf("Case not supported in the test!!");
            result = 0;
        }
    }
    return result;
}

/*!****************************************************************************
  \brief Prints Status voltage codes and Vref2 voltage code onto the serial port
 *****************************************************************************/
void print_stat()
{

    for (int current_ic =0 ; current_ic < TOTAL_IC; current_ic++) {
        printf("IC%d", current_ic+1);
        printf(" SOC:%.4f,", bms_ic[current_ic].stat.stat_codes[0]*0.0001*20);
        printf(" Itemp:%.4f,", bms_ic[current_ic].stat.stat_codes[1]*0.0001);
        printf(" VregA:%.4f,", bms_ic[current_ic].stat.stat_codes[2]*0.0001);
        printf(" VregD:%.4f\r\n", bms_ic[current_ic].stat.stat_codes[3]*0.0001);
    }

    printf("\r\n");
}

/*!******************************************************************************
 \brief Prints the configuration data that is going to be written to the LTC6811
 to the serial port.
 ********************************************************************************/
void print_config()
{
    int cfg_pec;

    printf("Written Configuration: \r\n");
    for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++) {
        printf(" IC ");
        printf("%d", current_ic+1);
        printf(": ");
        printf("");
        serial_print_hex(bms_ic[current_ic].config.tx_data[0]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].config.tx_data[1]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].config.tx_data[2]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].config.tx_data[3]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].config.tx_data[4]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].config.tx_data[5]);
        printf(", Calculated PEC: ");
        cfg_pec = pec15_calc(6,&bms_ic[current_ic].config.tx_data[0]);
        serial_print_hex((uint8_t)(cfg_pec>>8));
        printf(", ");
        serial_print_hex((uint8_t)(cfg_pec));
        printf("\r\n");
    }
    printf("\r\n");
}

/*!*****************************************************************
 \brief Prints the configuration data that was read back from the
 LTC6811 to the serial port.
 *******************************************************************/
void print_rxconfig()
{
    printf("Received Configuration\r\n");
    for (int current_ic=0; current_ic<TOTAL_IC; current_ic++) {
        printf("IC ");
        printf("%d", current_ic+1);
        printf(": ");
        serial_print_hex(bms_ic[current_ic].config.rx_data[0]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].config.rx_data[1]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].config.rx_data[2]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].config.rx_data[3]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].config.rx_data[4]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].config.rx_data[5]);
        printf(", Received PEC: ");
        serial_print_hex(bms_ic[current_ic].config.rx_data[6]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].config.rx_data[7]);
        printf("\r\n");
    }
    printf("\r\n");
}

void print_pec()
{
    for (int current_ic=0; current_ic<TOTAL_IC; current_ic++) {
        printf("\r\n%d", bms_ic[current_ic].crc_count.pec_count);
        printf(" : PEC Errors Detected on IC");
        printf("%d\r\n", current_ic+1);
    }
}

uint8_t compute_pec()
{
    uint8_t total_pec = 0;
    for (int current_ic=0; current_ic<TOTAL_IC; current_ic++) {
        printf("\r\n%d", bms_ic[current_ic].crc_count.pec_count);
        printf(" PEC Errors Detected on IC");
        printf("%d\r\n", current_ic+1);
        total_pec += bms_ic[current_ic].crc_count.pec_count;
    }

    return total_pec;
}


void serial_print_hex(uint8_t data)
{
    if (data < 16) {
        printf("0x0%X", data);
    } else
        printf("0x%X", data);
}

//Function to check error flag and print PEC error message
void check_error(int error)
{
    if (error == -1) {
        printf("A PEC error was detected in the received data");
    }
}


// hex conversion constants
char hex_digits[16]= {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'
};

// global variables

char hex_to_byte_buffer[5]= {
    '0', 'x', '0', '0', '\0'
};               // buffer for ASCII hex to byte conversion
char byte_to_hex_buffer[3]= {
    '\0','\0','\0'
};

// Read data from the serial interface into the ui_buffer
uint8_t read_data()
{
    uint8_t index = 0; //index to hold current location in ui_buffer
    int c,d; // single character used to store incoming keystrokes
    //printf("check 1\r\n");
    while (index < UI_BUFFER_SIZE-1) {
        //printf("check 2\r\n");
        pc.read(&c,1);
        //return c;
        //printf("check 3\r\n");
        
        if (((char) c == '\r') || ((char) c == '\r\n')) break; // if carriage return or linefeed, stop and return data
        if (((char) c == '\x7F') || ((char) c == '\x08')) { // remove previous character (decrement index) if Backspace/Delete key pressed      index--;
            if (index > 0) index--;
        } else if (c >= 0) {
            ui_buffer[index++]=(char) c; // put character into ui_buffer
        }
        //printf("check 4\r\n");
        
    }
    ui_buffer[index]='\0';  // terminate string with NULL

    if ((char) c == '\r') {  // if the "last" character was a carriage return, also clear linefeed if it is next character
        //wait_ms(5);
        delay_m(10);
        //printf("check 5\r\n");
        
        if (pc.readable()==1) {
            //printf("check 6\r\n");
             pc.read(&d,1); // if linefeed appears, read it and throw it away
            //wait_ms(5);
        }
        //printf("check 7\r\n");
        
    }
    //printf("check 8\r\n");
        
    return index; // return number of characters, not including null terminator
}

// Read a float value from the serial interface
float read_float()
{
    float data;
    read_data();
    data = atof(ui_buffer);
    return(data);
}

// Read an integer from the serial interface.
// The routine can recognize Hex, Decimal, Octal, or Binary
// Example:
// Hex:     0x11 (0x prefix)
// Decimal: 17
// Octal:   021 (leading zero prefix)
// Binary:  B10001 (leading B prefix)
int32_t read_int()
{
    //printf("entering function read int");
    int32_t data;
    read_data();
    if (ui_buffer[0] == 'm')
        return('m');
    if ((ui_buffer[0] == 'B') || (ui_buffer[0] == 'b')) {
        data = strtol(ui_buffer+1, NULL, 2);
    } else
        data = strtol(ui_buffer, NULL, 0);
    return(data);
}

// Read a string from the serial interface.  Returns a pointer to the ui_buffer.
char *read_string()
{
    read_data();
    return(ui_buffer);
}

// Read a character from the serial interface
int8_t read_char()
{
    read_data();
    return(ui_buffer[0]);
}
//*!****************************************************
 // \brief Function to select the S pin for discharge
  //@return void
 //******************************************************/
int8_t select_s_pin(void)
{
    int8_t read_s_pin=0;
    
    printf("Please enter the Spin number:/n");
    read_s_pin = (int8_t)read_int();
    printf("Pin",read_s_pin);
    return(read_s_pin);
}

void print_rxcomm(void)
{
    printf("Received Data in COMM register:\r\n");
    
    for (int current_ic=0; current_ic<TOTAL_IC; current_ic++)
    {
        printf(" IC- ");
        printf(" %d",current_ic+1);
        
        for(int i = 0; i < 6; i++)
        {
            printf(", ");
            serial_print_hex(bms_ic[current_ic].com.rx_data[i]);
        }
        printf(", Received PEC: ");
        serial_print_hex(bms_ic[current_ic].com.rx_data[6]);
        printf(", ");
        serial_print_hex(bms_ic[current_ic].com.rx_data[7]);
        printf("\r\n");
    }
}

uint8_t get_rxcomm_write(void)
{    
    // The actual data that we want to write in the EPROM is in D3 from the 
    // COMM registers, so we want to read the low nibble (lowest 4 bits of a
    // byte) from element 4 of the array, and the high nibble from element 5
    // -> see the data sheet for a pretty visual representation

    //serial_print_hex(bms_ic[0].com.rx_data[4]);
    //printf("\r\n");
    //serial_print_hex(bms_ic[0].com.rx_data[5]);
    //printf("\r\n");
    
    uint8_t data_high = ((bms_ic[0].com.rx_data[4]) & 0x0F) << 4;
    uint8_t data_low  = ((bms_ic[0].com.rx_data[5]) & 0xF0) >> 4;
    //printf("data_high: %d\r\n", data_high);
    //printf("data_low: %d\r\n", data_low);
    //since everything is already shifted correctly, we can just sum
    uint8_t written_data = data_high + data_low;
    printf("written data to memory: ");
    serial_print_hex(written_data);
    printf("\r\n");
    return written_data;
}

uint8_t get_rxcomm_read(void)
{    
    // The actual data that we want to write in the EPROM is in D3 from the 
    // COMM registers, so we want to read the low nibble (lowest 4 bits of a
    // byte) from element 4 of the array, and the high nibble from element 5
    // -> see the data sheet for a pretty visual representation
    
    //serial_print_hex(bms_ic[0].com.rx_data[0]);
    //printf("\r\n");
    //serial_print_hex(bms_ic[0].com.rx_data[1]);
    //printf("\r\n");

    uint8_t data_high = ((bms_ic[0].com.rx_data[0]) & 0x0F) << 4;
    uint8_t data_low  = ((bms_ic[0].com.rx_data[1]) & 0xF0) >> 4;
    //printf("data_high: %d\r\n", data_high);
    //printf("data_low: %d\r\n", data_low);
    //since everything is already shifted correctly, we can just sum
    uint8_t read_data = data_high + data_low;
    printf("read data from memory : ");
    serial_print_hex(read_data);
    printf("\r\n");
    return read_data;
}

uint16_t get_rxcomm_read_temp(void)
{    
    // The actual data that we want to write in the EPROM is in D3 from the 
    // COMM registers, so we want to read the low nibble (lowest 4 bits of a
    // byte) from element 4 of the array, and the high nibble from element 5
    // -> see the data sheet for a pretty visual representation
    
    //serial_print_hex(bms_ic[0].com.rx_data[0]);
    //printf("\r\n");
    //serial_print_hex(bms_ic[0].com.rx_data[1]);
    //printf("\r\n");
    //serial_print_hex(bms_ic[0].com.rx_data[2]);
    //printf("\r\n");
    uint16_t data_high_high = ((bms_ic[0].com.rx_data[0]) & 0x0F) << 8;
    uint16_t data_high      = ((bms_ic[0].com.rx_data[1]) & 0xF0);
    uint16_t data_low       = ((bms_ic[0].com.rx_data[2]) & 0x0F);

    
    //printf("data_high_high: %d\r\n", data_high_high);
    //printf("data_high: %d\r\n", data_high);
    //printf("data_low: %d\r\n", data_low);
    //since everything is already shifted correctly, we can just sum
    uint16_t read_data = data_high_high + data_high + data_low;
    //printf("read_data: %d\r\n", read_data);
    return read_data;
}

void print_wrcomm(void)
{
    int comm_pec;
    
    printf("Written Data in COMM Register:\r\n");
    for (int current_ic = 0; current_ic<TOTAL_IC; current_ic++)
    {
    printf(" IC- ");
    printf(" %d",(current_ic+1));
    
    for(int i = 0; i < 6; i++)
    {
      printf(", ");
      serial_print_hex(bms_ic[current_ic].com.tx_data[i]);
    }
    printf(", Calculated PEC: ");
    comm_pec = pec15_calc(6,&bms_ic[current_ic].com.tx_data[0]);
    serial_print_hex((uint8_t)(comm_pec>>8));
    printf(", ");
    serial_print_hex((uint8_t)(comm_pec));
    printf("\r\n");
    }
}


//uint8_t check_memory_read_write(uint8_t datalog_en)
//{
  //  typedef unsigned char lower_byte;
    // typedef unsigned char higher_byte;
     
     //if( bms_ic[current_ic].com.rx_data[1]>>8==bms_ic[current_ic].com.rx_data[2]8)
     //{
         
     //}
     
//}   