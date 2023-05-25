#include <MKL46Z4.h>
#include <pin_mux.h>
#include <clock_config.h>
#include <stdio.h>
//#include <board.h>
#include <fsl_debug_console.h>
#include "utils.h"
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

// Accelerometer constants
#define BOARD_TIMER_BASEADDR TPM0
#define BOARD_FIRST_TIMER_CHANNEL 5U
#define BOARD_SECOND_TIMER_CHANNEL 2U
/* Get source clock for TPM driver */
#define BOARD_TIMER_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#define TIMER_CLOCK_MODE 1U
/* I2C source clock */
#define I2C_BAUDRATE 100000U

#define I2C_RELEASE_SDA_PORT PORTE
#define I2C_RELEASE_SCL_PORT PORTE
#define I2C_RELEASE_SDA_GPIO GPIOE
#define I2C_RELEASE_SDA_PIN 25U
#define I2C_RELEASE_SCL_GPIO GPIOE
#define I2C_RELEASE_SCL_PIN 24U
#define I2C_RELEASE_BUS_COUNT 100U



const int RED_LED_PIN = 29;
 const int SWITCH_1_PIN = 3;
const int SWITCH_0_PIN = 12;
const int SWITCH_3_PIN = 12;// Change the pin number for SW3
SIM_Type *global_SIM = SIM;
PORT_Type *global_PORTE = PORTE;
GPIO_Type *global_PTE = PTE;
PORT_Type *global_PORTC = PORTC;
GPIO_Type *global_PTC = PTC;
// Accelerometer Initialization Functions and data
void BOARD_I2C_ReleaseBus(void);
//const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

static void i2c_release_bus_delay(void)
{
  uint32_t i = 0;
  for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
  {
    __NOP();
  }
}

void BOARD_I2C_ReleaseBus(void)
{
  uint8_t i = 0;
  gpio_pin_config_t pin_config;
  port_pin_config_t i2c_pin_config = {0};

  /* Config pin mux as gpio */
  i2c_pin_config.pullSelect = kPORT_PullUp;
  i2c_pin_config.mux = kPORT_MuxAsGpio;

  pin_config.pinDirection = kGPIO_DigitalOutput;
  pin_config.outputLogic = 1U;
  CLOCK_EnableClock(kCLOCK_PortE);
  PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
  PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

  GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
  GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

  /* Drive SDA low first to simulate a start */
  GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
  i2c_release_bus_delay();

  /* Send 9 pulses on SCL and keep SDA high */
  for (i = 0; i < 9; i++)
  {
    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();
    i2c_release_bus_delay();
  }

  /* Send stop */
  GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
  i2c_release_bus_delay();

  GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
  i2c_release_bus_delay();

  GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
  i2c_release_bus_delay();

  GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
  i2c_release_bus_delay();
}
/* Initialize timer module */
void Timer_Init(void)
{
  /* convert to match type of data */
  tpm_config_t tpmInfo;
  tpm_chnl_pwm_signal_param_t tpmParam[2];

  /* Configure tpm params with frequency 24kHZ */
  tpmParam[0].chnlNumber = (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL;
  tpmParam[0].level = kTPM_LowTrue;
  tpmParam[0].dutyCyclePercent = 0U;

  tpmParam[1].chnlNumber = (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL;
  tpmParam[1].level = kTPM_LowTrue;
  tpmParam[1].dutyCyclePercent = 0U;

  /* Initialize TPM module */
  TPM_GetDefaultConfig(&tpmInfo);
  TPM_Init(BOARD_TIMER_BASEADDR, &tpmInfo);

  CLOCK_SetTpmClock(1U);

  TPM_SetupPwm(BOARD_TIMER_BASEADDR, tpmParam, 2U, kTPM_EdgeAlignedPwm, 24000U, BOARD_TIMER_SOURCE_CLOCK);
  TPM_StartTimer(BOARD_TIMER_BASEADDR, kTPM_SystemClock);
}

/* Update the duty cycle of an active pwm signal */
static void Board_UpdatePwm(uint16_t x, uint16_t y)
{
  /* Updated duty cycle */
  TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, x);
  TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, y);
}

void Right_Button_Initialize(void)
{
  // setup switch 1
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;                 // Enable the clock to port C
  PORTC->PCR[SWITCH_1_PIN] &= ~PORT_PCR_MUX(0b111);   // Clear PCR Mux bits for PTC3
  PORTC->PCR[SWITCH_1_PIN] |= PORT_PCR_MUX(0b001);    // Set up PTC3 as GPIO
  PTC->PDDR &= ~GPIO_PDDR_PDD(1 << SWITCH_1_PIN);     // make it input
  PORTC->PCR[SWITCH_1_PIN] |= PORT_PCR_PE(1);         // Turn on the pull enable
  PORTC->PCR[SWITCH_1_PIN] |= PORT_PCR_PS(1);         // Enable the pullup resistor
  PORTC->PCR[SWITCH_1_PIN] &= ~PORT_PCR_IRQC(0b1111); // Clear IRQC bits for PTC3
  PORTC->PCR[SWITCH_1_PIN] |= PORT_PCR_IRQC(0b1011);  // Set up the IRQC to interrupt on either edge (i.e. from high to low or low to high)
}

//void Left_Button_Initialize(void) // not guaranteed to be right!!!!!!
//{
//  // setup switch 0
//  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;                 // Enable the clock to port C
//  PORTC->PCR[SWITCH_0_PIN] &= ~PORT_PCR_MUX(0b111);   // Clear PCR Mux bits for PTC3
//  PORTC->PCR[SWITCH_0_PIN] |= PORT_PCR_MUX(0b001);    // Set up PTC3 as GPIO
//  PTC->PDDR &= ~GPIO_PDDR_PDD(1 << SWITCH_0_PIN);     // make it input
//  PORTC->PCR[SWITCH_0_PIN] |= PORT_PCR_PE(1);         // Turn on the pull enable
//  PORTC->PCR[SWITCH_0_PIN] |= PORT_PCR_PS(1);         // Enable the pullup resistor
//  PORTC->PCR[SWITCH_0_PIN] &= ~PORT_PCR_IRQC(0b1111); // Clear IRQC bits for PTC3
//  PORTC->PCR[SWITCH_0_PIN] |= PORT_PCR_IRQC(0b1011);  // Set up the IRQC to interrupt on either edge (i.e. from high to low or low to high)
//}

void init_uart(void)
{
  // Use UART port through debug interface
  //  Connect to UART with TX (115200, 8N1)

  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
}

void uart_putc(char ch)
{
  /* Wait until space is available in the FIFO */
  while (!(UART0->S1 & UART_S1_TDRE_MASK)) {

  }

  /* Send the character */
  UART0->D = (uint8_t)ch;
}
