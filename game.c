
#include <MKL46Z4.h>
#include <stdlib.h>
#include <stdint.h>
#include "lcd.h"
#include "utils.h"
#include "game.h"
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
#include "fsl_adc16.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U											// pin20 port c
#define DEMO_ADC16_USER_CHANNEL 3 /* PTE20, ADC0_SE0 */ //

/* Upper bound and lower bound angle values */
#define ANGLE_UPPER_BOUND 85U
#define ANGLE_LOWER_BOUND 5U

volatile int current_time; // time in ms
int score = 0;
bool playing;


void LCD_displayScore(int number)
{
	clearDisplay();

	int digits[4] = {0}; // initialize array with 4 elements and 0s
	int i = 3;

	// extract digits from the number and store them in an array
	while (number > 0)
	{
		digits[i] = number % 10;
		number /= 10;
		i--;
	}

	// print the digits in reverse order with leading zeros if necessary
	//printf("Digits: ");
	for (int j = 0; j < 4; j++)
	{
		displayDigit(j + 1, digits[j]);
	}
	//printf("\n");
}




// gives an instruction to do. If it is done correctly in time return true;
// if the wrong thing is done or the deadline passes return false;
bool instruction(int itype, int deadline)
{
	bool completed = false;
	// big switch statement between button press, slide ect. ect.
	//  this could be super long so we might want to break it up
	switch (itype)
	{
	case 0:
		// left button
		completed = operate_switch_polling(deadline, 0);
		break;

	case 3:
		completed = operate_switch_polling(deadline, 1);
		break;
//
	case 1:
		completed = operate_light_sensor_polling(deadline);
		break;
//
//	case 3:
//		// touch slider
//		break;
//
	case 2:
		 //accelerometer
		completed = accelerometer_test(deadline);
		break;

	default:
		break;
	}
	return completed;
}
// this is definitely right
bool operate_switch_polling(int deadline, int button)
{
	int pin;
	if(button == 0) {
		pin = 3;
	} else {
		pin = 12;
	}

	  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;                 // Enable the clock to port C
	  PORTC->PCR[pin] &= ~PORT_PCR_MUX(0b111);   // Clear PCR Mux bits for PTC3
	  PORTC->PCR[pin] |= PORT_PCR_MUX(0b001);    // Set up PTC3 as GPIO
	  PTC->PDDR &= ~GPIO_PDDR_PDD(1 << pin);     // make it input
	  PORTC->PCR[pin] |= PORT_PCR_PE(1);         // Turn on the pull enable
	  PORTC->PCR[pin] |= PORT_PCR_PS(1);         // Enable the pullup resistor
	  PORTC->PCR[pin] &= ~PORT_PCR_IRQC(0b1111); // Clear IRQC bits for PTC3
	  PORTC->PCR[pin] |= PORT_PCR_IRQC(0b1011);  // Set up the IRQC

	while (current_time < deadline)
	{
		if ((PORTC->PCR[pin] & PORT_PCR_ISF(1)) != 0) {
			PORTC->PCR[pin] |= PORT_PCR_ISF(1);
			while(current_time < deadline) {
				if ((PORTC->PCR[pin] & PORT_PCR_ISF(1)) != 0){
					PORTC->PCR[pin] |= PORT_PCR_ISF(1);
					return true;
				}
			}



		}

	}
	return false;
}
// this is probably wrong
bool operate_left_switch_polling(int deadline)
{
	while (current_time < deadline)
	{
		if((PORTC->PCR[SWITCH_1_PIN] & PORT_PCR_ISF(1)) != 0)
		{
			return true;
		}
	}
	return false;
}
// i'm imagining a but of poll operators running concurrently

// Updates the current time every millisecond
void PIT_IRQHandler()
{
	current_time++;

	PIT->CHANNEL[0].TFLG = 1;
	PIT->CHANNEL[0].LDVAL = 0x4E20;
}

void start_time(void)
{

	// enable interrupt timer
	current_time = 0;
	SIM->SCGC6 = SIM_SCGC6_PIT_MASK;
	PIT->MCR = 0;
	PIT->CHANNEL[0].TFLG = 1;
	PIT->CHANNEL[0].LDVAL = 0x4E20; // ~Should be 1 ms this is wrong rn
	PIT->CHANNEL[0].TCTRL |= 3;
	NVIC_EnableIRQ(PIT_IRQn);
}

int main(void)
{
	// initialize all peripherals
	init_uart();
	//Right_Button_Initialize();
	start_time();
	init_lcd();
	// print to serial "Press right button to start game mode 1 left to start game mode 2"
	// python says this out loud
	bool playing = true;


	int count = 0;
	while (1)
	{
		// display to monitor: press button to begin
		score = 0;

		// LCD resetting and disp laying high score
		if(count == 0){
			uart_putc((char)8);
		}
		playing = true;
		operate_switch_polling(300000, 0);

		//playing = true;

				while (playing)
		{
			// LCD resetting and updating current score
			LCD_displayScore(score);



			int instruction_type = gen_instruction();
			//PRINTF("instruction %d\n", instruction_type);
			// send instruction over serial
			uart_putc((char)instruction_type);
			int deadline = deadline_calc(score);
			//deadline = 100000000;

			playing = instruction(instruction_type, deadline);


			if (playing)
			{
				score++;
				//PRINTF("Score: %d\r\n", score);
			}
			else
			{
				count++;
				uart_putc((char)9);
				//PRINTF("GAME OVER");


				break;
			}
		}
	}

	return 0;
}

int deadline_calc(int score)
{
	// starting round time
	int initial_time = 10000;

	// next round time relative to next round start time
	int time_amount = initial_time - 200 * score;

	return current_time + time_amount;
}

int gen_instruction()
{

	// generates a number from 0 to 4 inclusive.
	int random_number = rand() % 4;
	//random_number = 0;

	return random_number;
}

bool operate_light_sensor_polling(int deadline)
{
	uint32_t result = 0;
	adc16_config_t adc16ConfigStruct;
	adc16_channel_config_t adc16ChannelConfigStruct;

	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();

	//PRINTF("\r\nADC16 polling Example.\r\n");

	/*
	 * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	 * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
	 * adc16ConfigStruct.enableAsynchronousClock = true;
	 * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
	 * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
	 * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
	 * adc16ConfigStruct.enableHighSpeed = false;
	 * adc16ConfigStruct.enableLowPower = false;
	 * adc16ConfigStruct.enableContinuousConversion = false;
	 */
	ADC16_GetDefaultConfig(&adc16ConfigStruct);
#ifdef BOARD_ADC_USE_ALT_VREF
	adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif
	ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
	{
		//PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
	}
	else
	{
		//PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
	}
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */
	//PRINTF("Press any key to get user channel's ADC value ...\r\n");

	adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

	bool round = false;
	while (current_time < deadline)
	{
		/*
		 When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
		 function, which works like writing a conversion command and executing it. For another channel's conversion,
		 just to change the "channelNumber" field in channel's configuration structure, and call the
		 "ADC16_ChannelConfigure() again.
		*/
		ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
		while (0U == (kADC16_ChannelConversionDoneFlag &
									ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
		{
		}

		result = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP); /*Read ADC conversion result*/

		if (result > 3900)
		{																										 /*If sensor is completely covered, ADC result is maximum (0xFFFF)*/
			//PRINTF("Light sensor is completely covered.\r\n"); /*Print message to serial console*/
			PRINTF("LIGHT VALUE: %d \n", result);
			round = true;
			//return true;
		}
		else
		{
			if(round) {
				return true;
			}
			//PRINTF("Light sensor is not completely covered.\r\n"); /*Print message to serial console*/
		}

		//PRINTF("ADC Value: %d\r\n", ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP));
	}
	return false;
}

bool accelerometer_test(int deadline)
{

	const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

	mma_handle_t mmaHandle = {0};
	mma_data_t sensorData = {0};
	mma_config_t config = {0};
	status_t result;
	uint8_t sensorRange = 0;
	uint8_t dataScale = 0;
	int16_t xData = 0;
	int16_t yData = 0;
	int16_t xAngle = 0;
	int16_t yAngle = 0;
	int16_t xDuty = 0;
	int16_t yDuty = 0;
	uint8_t i = 0;
	uint8_t array_addr_size = 0;

	/* Board pin, clock, debug console init */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_I2C_ReleaseBus();
	BOARD_I2C_ConfigurePins();
	BOARD_InitDebugConsole();

	/* I2C initialize */
	BOARD_Accel_I2C_Init();
	/* Configure the I2C function */
	config.I2C_SendFunc = BOARD_Accel_I2C_Send;
	config.I2C_ReceiveFunc = BOARD_Accel_I2C_Receive;

	/* Initialize sensor devices */
	array_addr_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);
	for (i = 0; i < array_addr_size; i++)
	{
		config.slaveAddress = g_accel_address[i];
		/* Initialize accelerometer sensor */
		result = MMA_Init(&mmaHandle, &config);
		if (result == kStatus_Success)
		{
			break;
		}
	}

	//if (result != kStatus_Success)
	//{
		//PRINTF("\r\nSensor device initialize failed!\r\n");
		//return -1;
	//}
	/* Get sensor range */
	//if (MMA_ReadReg(&mmaHandle, kMMA8451_XYZ_DATA_CFG, &sensorRange) != kStatus_Success)
	//{
		//return -1;
	//}
	if (sensorRange == 0x00)
	{
		dataScale = 2U;
	}
	else if (sensorRange == 0x01)
	{
		dataScale = 4U;
	}
	else if (sensorRange == 0x10)
	{
		dataScale = 8U;
	}
	else
	{
	}
	/* Init timer */
	Timer_Init();

	bool round = false;
	// main loop
	while (current_time < deadline)
	{
		if (MMA_ReadSensorData(&mmaHandle, &sensorData) != kStatus_Success)
		{
			return false;
		}

		/* Get the X and Y data from the sensor data structure in 14 bit left format data*/
		xData = (int16_t)((uint16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB) / 4U;
		yData = (int16_t)((uint16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB) / 4U;

		/* Convert raw data to angle (normalize to 0-90 degrees). No negative angles. */
		xAngle = (int16_t)floor((double)xData * (double)dataScale * 90 / 8192);
		if (xAngle < 0)
		{
			xAngle *= -1;
		}
		yAngle = (int16_t)floor((double)yData * (double)dataScale * 90 / 8192);
		if (yAngle < 0)
		{
			yAngle *= -1;
		}

		//PRINTF("xangle %d\n", xAngle);
		//PRINTF("yangle %d\n", yAngle);
		/* Update duty cycle to turn on LEDs when angles ~ 90 */
		if (xAngle > ANGLE_UPPER_BOUND)
		{
			xDuty = 100;
			round = true;


		}
		if (yAngle > ANGLE_UPPER_BOUND)
		{
			yDuty = 100;

			round = true;
		}
		if(xAngle < ANGLE_LOWER_BOUND && round == true) {
			return true;
		}
		if(yAngle < ANGLE_LOWER_BOUND && round == true) {
				return true;
		}
		// Board_UpdatePwm(xDuty, yDuty);
	}

	return false;
}

