#ifndef __UTILS_H__
#define __UTILS_H__

#include "fsl_debug_console.h"
#include "board.h"
//#include "fsl_adc16.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "math.h"
#include "fsl_mma.h"
#include "fsl_tpm.h"

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_i2c.h"




extern const int RED_LED_PIN;
extern const int SWITCH_1_PIN;
extern const int SWITCH_0_PIN;
extern const int SWITCH_3_PIN;// Change the pin number for SW3
extern SIM_Type *global_SIM;
extern PORT_Type *global_PORTE;
extern GPIO_Type *global_PTE;
extern PORT_Type *global_PORTC;
extern GPIO_Type *global_PTC;


//accelerometer
extern const uint8_t g_accel_address[];


void LED_Initialize(void);
void Right_Button_Initialize(void);
void Left_Button_Initialize(void);
void Light_Initialize();
void Capactive_Touch_Initialize(void);
void init_uart(void);
void uart_putc(char ch);
void BOARD_I2C_ReleaseBus(void);
void BOARD_I2C_ReleaseBus(void);
void Timer_Init(void);
//static void Board_UpdatePwm(uint16_t x, uint16_t y);


#endif
