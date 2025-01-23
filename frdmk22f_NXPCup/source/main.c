/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "pin_mux.h"
#if defined(FSL_FEATURE_SOC_PORT_COUNT) && (FSL_FEATURE_SOC_PORT_COUNT)
#include "fsl_port.h"
#endif
#include "clock_config.h"
#include "board.h"
#include "fsl_ftm.h"

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK22F51212.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "fsl_adc16.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* The Flextimer base address/channel used for board */
#define BOARD_FTM_BASEADDR FTM0
#define BOARD_FTM_CHANNEL_LEFT  kFTM_Chnl_6 //D9
#define BOARD_FTM_CHANNEL_RIGHT kFTM_Chnl_4 //D10

#define BOARD_FTM2_BASEADDR FTM2
#define BOARD_FTM2_CHANNEL_TURN  kFTM_Chnl_0 //D5

//timer
#define BOARD_FTM3_BASEADDR FTM3
#define BOARD_FTM3_CHANNEL_TIMER  kFTM_Chnl_0

/* Interrupt number and interrupt handler for the FTM base address used */
#define FTM0_INTERRUPT_NUMBER   FTM0_IRQn
#define FTM0_STARTUP_HANDLER    FTM0_IRQHandler

#define FTM2_INTERRUPT_NUMBER   FTM2_IRQn
#define FTM2_STARTUP_HANDLER    FTM2_IRQHandler

//timer
#define FTM3_INTERRUPT_NUMBER   FTM3_IRQn
#define TIMER_STARTUP_HANDLER   FTM3_IRQHandler

/* Interrupt to enable and flag to read */
#define FTM_CHANNEL_INTERRUPT_ENABLE_LEFT kFTM_Chnl6InterruptEnable
#define FTM_CHANNEL_FLAG_LEFT             kFTM_Chnl6Flag

#define FTM_CHANNEL_INTERRUPT_ENABLE_RIGHT kFTM_Chnl4InterruptEnable
#define FTM_CHANNEL_FLAG_RIGHT             kFTM_Chnl4Flag

#define FTM2_CHANNEL_INTERRUPT_ENABLE_TURN kFTM_Chnl0InterruptEnable
#define FTM2_CHANNEL_FLAG_TURN             kFTM_Chnl0Flag


/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#ifndef FTM_PWM_ON_LEVEL
#define FTM_PWM_ON_LEVEL kFTM_HighTrue
#endif

#define MOTOR_PWM_FREQUENCY (32000U)
#define SERVO_PWM_FREQUENCY (50U)

#define SystemFrequency 120 000 000
#define US_TIME    SystemFrequency/100000    /* 10 uS */*

//GPIO
#define GpioC GPIOC
#define PinTriger  2U //A3
#define PinEcho  1U //A2

#define BOARD_SW_GPIO        BOARD_SW2_GPIO
#define BOARD_SW_PORT        BOARD_SW2_PORT
#define BOARD_SW_GPIO_PIN    BOARD_SW2_GPIO_PIN
#define Echo_IRQ             BOARD_SW2_IRQ
#define ECHO_IRQ_HANDLER  	 BOARD_SW2_IRQ_HANDLER
#define BOARD_SW_NAME        BOARD_SW2_NAME

//Button black
#define GpioB GPIOB
#define PinButtonBlack  19U //D8



//camera

//Clock pin
#define BOARD_LED_GPIO GPIOC
#define BOARD_LED_GPIO_PIN 3 //D6

//SI pin
#define BOARD__SI_GPIO_PIN 6 //D7

//Pin analog reception
#define BOARD_DAC_BASEADDR ADC0 //A0
#define DEMO_ADC16_USER_CHANNEL  8U

// Seuils ajustés pour correspondre à la nouvelle logique
#define THRESHOLD_BLACK 1000 // Seuil pour la détection du noir
#define THRESHOLD_WHITE 1000 // Seuil pour la détection du blanc
#define SEQUENCE_LENGTH 8
#define PIXEL_COUNT 128

#define ALPHA 0.8 //Facteur filtrage

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
void motorBreak(void);
void turn();
void UpdatePwmDutycycleTurn(void);
void UpdatePwmDutycycleSpeed(void);
void startUpSpeed(void);
void delay_us(int);
void sendTrigger(void);
void clockPulse();
void delay(uint32_t duration);
void initADC();
int readADC();
void detectTrack(int *pixels, int length, int *leftPosition, int *rightPosition);
int readFilteredADC();
int adjustDirection(int position);
void setupCamera();
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool ftmIsrFlag         = true;
volatile bool ftm2IsrFlag        = true;

volatile bool turnLeft           = false;
volatile bool turnRight          = false;
volatile bool middle             = false;

volatile bool accelerate         = false;
volatile bool decelerate         = false;
volatile bool stop         		 = false;

volatile uint8_t updatedDutycycleLeft  = 47U;
volatile uint8_t updatedDutycycleRight = 47U;
volatile uint8_t updatedDutycycleTurn  = 11U;

volatile int updatedDutycycleTurnMaxRight = 13U;
volatile int updatedDutycycleTurnMaxLeft = 8U;
volatile int updatedDutycycleTurnMiddle  = 11U;

volatile int updatedDutycycleSpeedMax = 65;
volatile int updatedDutycycleSpeedMin = 30;

volatile unsigned int usTicks = 0U;
volatile unsigned int usTicksDelay = 0U;

volatile int distance = 0;
volatile bool sendNewTrigger = false;
volatile bool waitForEcho = false;
volatile bool increaseTimer = false;


float kp = 2;//0.1/1/10/100
float kr = 1;//0.1/1/10/100
float krMin = 1;//0.1/1/10/100
int size = 10;
int erreurs [10] = {0,0,0,0,0,0,0,0,0,0};
int cpt = 0;
float Ti = 100;//0.1/1/10/100/200


adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;
int pixelArray[128];


/*******************************************************************************
 * Code
 ******************************************************************************/

//time.c
void SysTick_Handler(void) {
	usTicks++;
	usTicksDelay++;
}

void init_delay(void) {
    SysTick_Config(SystemCoreClock / 873813);// 1us par tick
}

void delay_us(int us) {
    usTicksDelay = 0;
    while(usTicksDelay < us);
}
//

//moteur.h
void turn(){
	int lastUpdatedDutycycleTurn = updatedDutycycleTurn;
	if(turnLeft){
		if(updatedDutycycleTurn > updatedDutycycleTurnMaxLeft){
			updatedDutycycleTurn = updatedDutycycleTurnMaxLeft;
		}
		ftm2IsrFlag = true;
	}else if(turnRight){
		if(updatedDutycycleTurn < updatedDutycycleTurnMaxRight){
			updatedDutycycleTurn = updatedDutycycleTurnMaxRight;
		}
		ftm2IsrFlag = true;
	}else if(middle){
		updatedDutycycleTurn = 11;
		ftm2IsrFlag = true;
	}
	turnRight = false;
	turnLeft = false;
	middle = false;
	if(lastUpdatedDutycycleTurn != updatedDutycycleTurn){
		UpdatePwmDutycycleTurn();
	}

}

void motorBreak(void){
	updatedDutycycleLeft = 47U;
	updatedDutycycleRight = 47U;
	ftmIsrFlag = true;
	UpdatePwmDutycycleSpeed();

}

void startUpSpeed(){
	for (int i = 47;i<=49;i++){
		delay_us(1000000);
		updatedDutycycleLeft = i;
		updatedDutycycleRight = i;
		PRINTF("%i\r\n",i);
		ftmIsrFlag = true;
		UpdatePwmDutycycleSpeed();
	}
	PRINTF("StartUp motor end \r\n");
}

void UpdatePwmDutycycleTurn(){
	if (ftm2IsrFlag)
	{
		ftm_pwm_level_select_t pwmLevel = FTM_PWM_ON_LEVEL;
		ftm2IsrFlag = false;

		FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM2_BASEADDR, BOARD_FTM2_CHANNEL_TURN, 0U);

		if (kStatus_Success !=
			FTM_UpdatePwmDutycycle(BOARD_FTM2_BASEADDR, BOARD_FTM2_CHANNEL_TURN, kFTM_CenterAlignedPwm, updatedDutycycleTurn))
		{
			PRINTF("Update duty cycle fail, the target duty cycle may out of range!\r\n");
		}

		FTM_SetSoftwareTrigger(BOARD_FTM2_BASEADDR, true);

		FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM2_BASEADDR, BOARD_FTM2_CHANNEL_TURN, pwmLevel);
	}
}

void UpdatePwmDutycycleSpeed(){
	if (ftmIsrFlag)
	{
		ftm_pwm_level_select_t pwmLevel = FTM_PWM_ON_LEVEL;
		//FTM_DisableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE_LEFT);

		ftmIsrFlag = false;

		FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL_LEFT, 0U);
		FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL_RIGHT, 0U);

		if (kStatus_Success !=
			FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL_LEFT, kFTM_CenterAlignedPwm, updatedDutycycleLeft))
		{
			PRINTF("Update duty cycle fail, the target duty cycle may out of range!\r\n");
		}
		if (kStatus_Success !=
			FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL_RIGHT, kFTM_CenterAlignedPwm, updatedDutycycleRight))
		{
			PRINTF("Update duty cycle fail, the target duty cycle may out of range!\r\n");
		}
		FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR, true);

		FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL_LEFT, pwmLevel);
		FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL_RIGHT, pwmLevel);


		//FTM_EnableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE_LEFT);
	}
}


//

//interrupt.h
void FTM2_STARTUP_HANDLER(void)
{
	FTM_ClearStatusFlags(BOARD_FTM2_BASEADDR, FTM_GetStatusFlags(BOARD_FTM2_BASEADDR));
	SDK_ISR_EXIT_BARRIER;
}

void FTM0_STARTUP_HANDLER(void)
{
	FTM_ClearStatusFlags(BOARD_FTM_BASEADDR, FTM_GetStatusFlags(BOARD_FTM_BASEADDR));
	SDK_ISR_EXIT_BARRIER;
}

void ECHO_IRQ_HANDLER(void)
{
	bool test = false;
	if(GPIO_PinRead(GpioC, PinEcho)){
		usTicks = 0;
	}else{
		distance = (usTicks * 350 * 100 / 2/ 1000000); //cm
		usTicks = 0;
	}

	#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT) || \
		(!defined(FSL_FEATURE_SOC_PORT_COUNT))
		/* Clear external interrupt flag. */
		GPIO_GpioClearInterruptFlags(BOARD_SW_GPIO, 1U << BOARD_SW_GPIO_PIN);
	#else
		/* Clear external interrupt flag. */
		GPIO_PortClearInterruptFlags(BOARD_SW_GPIO, 1U << BOARD_SW_GPIO_PIN);
	#endif
	//SDK_ISR_EXIT_BARRIER;

}
//

//ultrason.h
void sendTrigger(){
	GPIO_PinWrite(GpioC, PinTriger, 1);
	delay_us(10);
	GPIO_PinWrite(GpioC, PinTriger, 0);
}
//

//camera.h
int readFilteredADC() {
    static int previousValue = 0;
    int rawValue = readADC();
    int filteredValue = (int)(ALPHA * rawValue + (1 - ALPHA) * previousValue);
    previousValue = filteredValue;
    return filteredValue;
}

void clockPulse() {
	GPIO_PinWrite(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, 1);
	delay_us(1);
	GPIO_PinWrite(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, 0);
}

void initADC() {
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    ADC16_Init(BOARD_DAC_BASEADDR, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(BOARD_DAC_BASEADDR, false);
    if (ADC16_DoAutoCalibration(BOARD_DAC_BASEADDR) != kStatus_Success) {
        PRINTF("ADC calibration failed!\n");
    } else {
        PRINTF("ADC calibration successful!\n");
    }
    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
}

int readADC() {
    ADC16_SetChannelConfig(BOARD_DAC_BASEADDR, 0, &adc16ChannelConfigStruct);
    while (!(ADC16_GetChannelStatusFlags(BOARD_DAC_BASEADDR, 0) & kADC16_ChannelConversionDoneFlag)) {
    }
    return ADC16_GetChannelConversionValue(BOARD_DAC_BASEADDR, 0);
}

/*
 * Parcourt les pixels pour trouver deux séquences de pixels noirs
 * vérifie la présence d'une piste blanche entre les deux lignes noires en comptant le nombre de pixels blancs
 */
int sum;
void detectTrack(int *pixels, int length, int *leftPosition, int *rightPosition) {
    int firstBlackLineStart = -1;
    int secondBlackLineStart = -1;
    int count = 0;
    const int MIN_DISTANCE_BETWEEN_LINES = 40; // Minimum distance between two detected black lines
    const int LEFT_THRESHOLD = 50;// Threshold to determine if the robot is more to the left
    const int RIGHT_THRESHOLD = 70;
    sum=0;
    for (int i = 0; i < length; i++) {
    	sum+=pixels[i];
    	//delay_us(100);
        if (pixels[i] < THRESHOLD_BLACK) { // Détection du noir
            count++;
            if (count >= SEQUENCE_LENGTH) {
                if (firstBlackLineStart == -1) {
                    firstBlackLineStart = i - SEQUENCE_LENGTH + 1;
                } else if (secondBlackLineStart == -1 && (i - SEQUENCE_LENGTH + 1 - firstBlackLineStart) >= MIN_DISTANCE_BETWEEN_LINES) {
                    secondBlackLineStart = i - SEQUENCE_LENGTH + 1;
                }
           }
        } else {
            count = 0;
        }

    }
    sum=sum/length;
    PRINTF("moyenne valeur : %i\r\n",sum);
    if (firstBlackLineStart != -1 && secondBlackLineStart != -1 ) {
        *leftPosition = firstBlackLineStart;
        *rightPosition = secondBlackLineStart;

    } else {
        // Si on ne trouve qu'une seule ligne noire valide ou si les lignes ne sont pas suffisamment éloignées
        *leftPosition = -1;
        *rightPosition = -1;

        // Si le robot est inférieur à 60 pixels, on considère qu'il est plus à gauche
        if (firstBlackLineStart != -1 && firstBlackLineStart < LEFT_THRESHOLD) {
            *leftPosition = firstBlackLineStart;
        }
        else if (firstBlackLineStart != -1 && firstBlackLineStart > RIGHT_THRESHOLD) {
           *rightPosition = firstBlackLineStart;
        }
        else{
            *leftPosition = -1;
            *rightPosition = -1;

        }
    }
}
/*
 * Calcul la deviation de la position du centre de la piste par rapp au centre de l image
 * print la direction
 */
int adjustDirection(int position) {
    int center = PIXEL_COUNT / 2;
    int deviation = position - center;
    return deviation;
}

int getErrorFromCamera(){
	delay_us(10000);
	GPIO_PinWrite(BOARD_LED_GPIO, BOARD__SI_GPIO_PIN, 1);
	clockPulse();
	GPIO_PinWrite(BOARD_LED_GPIO, BOARD__SI_GPIO_PIN, 0);
	delay_us(1);
	for(int i=0; i<128; i++) {
		pixelArray[i] = readADC();
	    clockPulse();
	}

	int leftPosition = -1, rightPosition = -1;
	detectTrack(pixelArray, PIXEL_COUNT, &leftPosition, &rightPosition);

	if (leftPosition != -1 && rightPosition != -1) {
        int trackCenter = (leftPosition + rightPosition) / 2;
        return adjustDirection(trackCenter);
    } else if (leftPosition !=-1 && rightPosition == -1  ) {
    	if(leftPosition == 0){
    		return 20;
    	}
        rightPosition = PIXEL_COUNT;
        int trackCenter = (leftPosition + rightPosition) / 2;
        return adjustDirection(trackCenter);
    } else if (rightPosition !=-1 && leftPosition == -1  ) {
        leftPosition = 0;
        int trackCenter = (leftPosition + rightPosition) / 2;
        return adjustDirection(trackCenter);
    }


	return 0;
}

void setupCamera(){
	for(int i = 0;i<128;i++){
		clockPulse();
	}
	GPIO_PinWrite(BOARD_LED_GPIO, BOARD__SI_GPIO_PIN, 1);
	clockPulse();
	GPIO_PinWrite(BOARD_LED_GPIO, BOARD__SI_GPIO_PIN, 0);

	for(int i = 0;i<128;i++){
		clockPulse();
	}

}
//
/*!
 * @brief Main function
 */
int main(void)
{
	//----------------------setup--------------------------//
    ftm_config_t ftm0Info;
    ftm_config_t ftm2Info;
    ftm_config_t ftm3Info;
    ftm_chnl_pwm_signal_param_t ftmParamLeft;
    ftm_chnl_pwm_signal_param_t ftmParamRight;
    ftm_chnl_pwm_signal_param_t ftmParamTurn;
    ftm_chnl_pwm_signal_param_t ftmParamTimer;
    ftm_pwm_level_select_t pwmLevel = FTM_PWM_ON_LEVEL;

    /* Board pin, clock, debug console init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    init_delay();


    /*  ultrason sensor setup  */
    gpio_pin_config_t triger_config = {
		kGPIO_DigitalOutput,
		0,
	};
    gpio_pin_config_t echo_config = {
		kGPIO_DigitalInput
	};
    /* Init input switch GPIO. */
    GPIO_PinInit(GpioC, PinTriger, &triger_config);
    GPIO_PinInit(GpioC, PinEcho, &echo_config);

	#if (defined(FSL_FEATURE_PORT_HAS_NO_INTERRUPT) && FSL_FEATURE_PORT_HAS_NO_INTERRUPT) || \
		(!defined(FSL_FEATURE_SOC_PORT_COUNT))
		GPIO_SetPinInterruptConfig(BOARD_SW_GPIO, BOARD_SW_GPIO_PIN, kGPIO_InterruptEitherEdge);
	#else
		PORT_SetPinInterruptConfig(BOARD_SW_PORT, BOARD_SW_GPIO_PIN, kPORT_InterruptEitherEdge);
	#endif
	EnableIRQ(Echo_IRQ);

	/* Init Button GPIO*/
	gpio_pin_config_t Button_config = {
		kGPIO_DigitalInput
	};
	GPIO_PinInit(GpioB, PinButtonBlack, &Button_config);

    /* Print a note to terminal */
    PRINTF("\r\nFTM Start up\r\n");

    /* Fill in the FTM config struct with the default settings */
    FTM_GetDefaultConfig(&ftm0Info);
    FTM_GetDefaultConfig(&ftm2Info);
    FTM_GetDefaultConfig(&ftm3Info);

    /* Calculate the clock division based on the PWM frequency to be obtained */
    ftm0Info.prescale = FTM_CalculateCounterClkDiv(BOARD_FTM_BASEADDR, MOTOR_PWM_FREQUENCY, FTM_SOURCE_CLOCK);
    ftm2Info.prescale = FTM_CalculateCounterClkDiv(BOARD_FTM2_BASEADDR, SERVO_PWM_FREQUENCY, FTM_SOURCE_CLOCK);

    /* Initialize FTM module */
    FTM_Init(BOARD_FTM_BASEADDR, &ftm0Info);
    FTM_Init(BOARD_FTM2_BASEADDR, &ftm2Info);
    FTM_Init(BOARD_FTM3_BASEADDR, &ftm3Info);

    //motor left
    ftmParamLeft.chnlNumber            = BOARD_FTM_CHANNEL_LEFT;
    ftmParamLeft.level                 = pwmLevel;
    ftmParamLeft.dutyCyclePercent      = updatedDutycycleLeft;
    ftmParamLeft.firstEdgeDelayPercent = 0U;
    ftmParamLeft.enableComplementary   = false;
    ftmParamLeft.enableDeadtime        = false;
    if (kStatus_Success !=
        FTM_SetupPwm(BOARD_FTM_BASEADDR, &ftmParamLeft, 1U , kFTM_CenterAlignedPwm, MOTOR_PWM_FREQUENCY, FTM_SOURCE_CLOCK))
    {
        PRINTF("\r\nSetup PWM fail, please check the configuration parameters!\r\n");
        return -1;
    }

    //motor right
    ftmParamRight.chnlNumber            = BOARD_FTM_CHANNEL_RIGHT;
    ftmParamRight.level                 = pwmLevel;
    ftmParamRight.dutyCyclePercent      = updatedDutycycleRight;
    ftmParamRight.firstEdgeDelayPercent = 0U;
    ftmParamRight.enableComplementary   = false;
    ftmParamRight.enableDeadtime        = false;
    if (kStatus_Success !=
       FTM_SetupPwm(BOARD_FTM_BASEADDR, &ftmParamRight, 1U , kFTM_CenterAlignedPwm, MOTOR_PWM_FREQUENCY, FTM_SOURCE_CLOCK))
    {
       PRINTF("\r\nSetup PWM fail, please check the configuration parameters!\r\n");
       return -1;
    }

    //Servo motor turn
    ftmParamTurn.chnlNumber            = BOARD_FTM2_CHANNEL_TURN;
    ftmParamTurn.level                 = pwmLevel;
    ftmParamTurn.dutyCyclePercent      = updatedDutycycleTurn;
    ftmParamTurn.firstEdgeDelayPercent = 0U;
    ftmParamTurn.enableComplementary   = false;
    ftmParamTurn.enableDeadtime        = false;
    if (kStatus_Success !=
    		FTM_SetupPwm(BOARD_FTM2_BASEADDR, &ftmParamTurn, 1U , kFTM_CenterAlignedPwm, SERVO_PWM_FREQUENCY, FTM_SOURCE_CLOCK))
    {
    	PRINTF("\r\nSetup PWM fail, please check the configuration parameters!\r\n");
    	return -1;
    }

    /* Enable channel interrupt flag.*/
    //FTM_EnableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE_LEFT);
    //FTM_EnableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE_RIGHT);
    //FTM_EnableInterrupts(BOARD_FTM2_BASEADDR, FTM2_CHANNEL_INTERRUPT_ENABLE_TURN);

    /* Enable at the NVIC */
    //EnableIRQ(FTM0_INTERRUPT_NUMBER);
    //EnableIRQ(FTM2_INTERRUPT_NUMBER);

    FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
    FTM_StartTimer(BOARD_FTM2_BASEADDR, kFTM_SystemClock);
    PRINTF("\r\nEnd FTM Startup\r\n");

    //camera

    gpio_pin_config_t gpioConfigclk;
    gpioConfigclk.pinDirection = kGPIO_DigitalOutput;
    gpioConfigclk.outputLogic = 1;

    gpio_pin_config_t gpioConfigsi;
    gpioConfigsi.pinDirection = kGPIO_DigitalOutput;
    gpioConfigsi.outputLogic = 1;


    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &gpioConfigclk);
    GPIO_PinInit(BOARD_LED_GPIO, BOARD__SI_GPIO_PIN, &gpioConfigsi);
    initADC();
    setupCamera();

    //------------------end setup--------------------------//

    //-------------------startup---------------------------//
    startUpSpeed();
    //-----------------end startup-------------------------//
    int iNext = 1;
    int cptEchoToInfinite = 0;
    while (1)
    {
    	if(sendNewTrigger){
    		sendTrigger();
    		sendNewTrigger = false;
    	}
    	if(usTicks > 200000){
    		sendNewTrigger = true;
    		usTicks = 0;
    	}

    	if(distance < 20 && distance != 0){
    		motorBreak();
    		while(distance < 20 && distance != 0){
    			PRINTF("distance : %i\r\n",distance);
    			PRINTF("arrêt urgence\r\n");
    			if(sendNewTrigger){
					sendTrigger();
					sendNewTrigger = false;
				}
				if(usTicks > 200000){
					sendNewTrigger = true;
					usTicks = 0;
				}

    			if(GPIO_PinRead(GpioB, PinButtonBlack)==0){
    				PRINTF("Button press : restart\r\n");
    				break;
    			}
    		}
    		startUpSpeed();
			usTicks = 0;
			sendNewTrigger = false;
			distance = 1000;
    	}

        // detection de l'erreur avec camera
    	int erreurActuel = getErrorFromCamera();
    	//PRINTF("erreur : %i\r\n",erreurActuel);
    	//PRINTF("-----\r\n");
    	bool newError = true;
		//vitesse
        if(newError){
			//le turn rate doit varier selon l'erreur et la valeur de théta
			if(erreurActuel > 2 || erreurActuel < -2){  //si erreur pas acceptable
				if(erreurActuel >= 0){
					turnRight = true;
					updatedDutycycleLeft = 52;
					updatedDutycycleRight = 45;
				}else{
					turnLeft = true;
					updatedDutycycleLeft = 45;
					updatedDutycycleRight = 52;
				}
				turn();
				ftmIsrFlag = true;
				UpdatePwmDutycycleSpeed();
			}else{
				middle = true;
				turn();
				//on fait rien on accelère
	        	updatedDutycycleLeft = 54;
	        	updatedDutycycleRight = 54;
	        	ftmIsrFlag = true;
	        	UpdatePwmDutycycleSpeed();

			}
        }
    }
}
