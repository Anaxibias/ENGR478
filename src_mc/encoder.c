#include <stdint.h>
#include <stdbool.h>

#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "inc/hw_adc.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "wav.h"

//#include "sdcard.h"



#define BUFFERSIZE		512
#define LED_MASK			GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
#define PERIOD				1512 // 66.67 MHz clock speed, sampling at 44.1kHz, needs ~1512 clock cycles between samples
#define OFFSET				1551

// Enum used as status flags when checking if one of the buffers is full
enum BUFFERSTATUS
								{	EMPTY,
									FILLING,
									FULL
								};

#if defined(ewarm)
#pragma data_alignment=1024
uint8_t ui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ui8ControlTable, 1024)
uint8_t ui8ControlTable[1024];
#else
uint8_t ui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

static uint32_t ADCBuffer1[BUFFERSIZE];
static uint32_t ADCBuffer2[BUFFERSIZE];
static uint16_t SPIBufferA[BUFFERSIZE];
static uint16_t SPIBufferB[BUFFERSIZE];
static enum BUFFERSTATUS BufferStatus[2];
							
uint32_t count1 = 0;
uint32_t count2 = 0;
uint32_t sum;
								
uint16_t ui16Sample;
uint16_t avg;

uint8_t numChannels = 0;
uint8_t countGaps = 0;

//struct Header fileHeader;



// this variable should be set to false, but is set to true to streamline testing
// also the gpio edge triggered interrupt and button debouncing was never implemented
bool start = true;
bool channelsSet = false;

// this is a variable that verifies that an SD card is detected
bool cardDetection = false;
	
// GPIO initialization
void GPIO_Init(void){
	  //
    // Enable Peripheral Clocks 
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable pin PE3 for ADC AIN0
    //
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
	
		// Enable pins PF1, PF2, and PF3 for LED output
		//
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0x08 | 0x04 | 0x02);
		GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, 0x01 | 0x10);
}

void ADC0_Init(void){
	
		SysCtlClockSet(SYSCTL_SYSDIV_3|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ); // configure the system clock to be 40MHz
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	//activate the clock of ADC0
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //activate the clock of Timer0
		SysCtlDelay(2);	//insert a few cycles after enabling the peripheral to allow the clock to be fully activated.
		
		TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); // configure Timer0 as periodic
		TimerLoadSet(TIMER0_BASE, TIMER_A, PERIOD);
	
		ADCSequenceDisable(ADC0_BASE, 0); //disable ADC0 before the configuration is complete
		ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
		ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0); //ADC0 SS1 Step 0, sample from internal temperature sensor
		ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH0); //ADC0 SS1 Step 1, sample from internal temperature sensor, completion of this step will set RIS
		ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH0); //ADC0 SS1 Step 2, sample from internal temperature sensor
		ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH0 | ADC_CTL_IE);
		ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH0);
		ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH0);
		ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH0);
		ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
		//ADC0 SS1 Step 0, sample from internal temperature sensor, completion of this step will set RIS, last sample of the sequence
		//ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_TS|ADC_CTL_IE|ADC_CTL_END); 
		
		ADCSequenceDMAEnable(ADC0_BASE, 0);
		ADCIntEnable(ADC0_BASE, 0);
		IntEnable(INT_ADC0SS0);
		
		TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
		ADCSequenceEnable(ADC0_BASE, 0); //enable ADC0
		
		TimerEnable(TIMER0_BASE, TIMER_A);
}

void DMA_Init(void){
	
		//
    // Enable the uDMA clock
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
		
    uDMAEnable();

    //
    // Set the control table
		//
    uDMAControlBaseSet(ui8ControlTable);
	
		uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC0, UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY | UDMA_ATTR_REQMASK);

    uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC0, UDMA_ATTR_USEBURST);
    // Only allow burst transfers

    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);

    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADCBuffer1, BUFFERSIZE);
    uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADCBuffer2, BUFFERSIZE);

    uDMAChannelEnable(UDMA_CHANNEL_ADC0);												 
}

void ADC0_Handler(void){
	
	if((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT) == UDMA_MODE_STOP) && (BufferStatus[0] == FILLING)){
		count1++;
		BufferStatus[0] = FULL;
		BufferStatus[1] = FILLING;
		
		uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADCBuffer1, BUFFERSIZE);
		uDMAChannelEnable(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT);
	}
	else if((uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT) == UDMA_MODE_STOP) && (BufferStatus[1] == FILLING)){
		count2++;
		BufferStatus[0] = FILLING;
		BufferStatus[1] = FULL;
		
		uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADCBuffer2, BUFFERSIZE);
		uDMAChannelEnable(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT);
	}
}

int main(void){
	
	GPIO_Init();
	IntMasterEnable();
	
	while(!start){
		while(!channelsSet){
			// turn the led yellow to show that the system has been set to record
			// 1 channel of audio
			if(numChannels == 1)
				GPIOPinWrite(GPIO_PORTF_BASE, LED_MASK, 0x0b);
			// turn the led cyan to show that the system has been set to record
			// 2 channels of audio
			else if(numChannels == 2)
				GPIOPinWrite(GPIO_PORTF_BASE, LED_MASK, 0x0d);
			// turn the led red to show that the number of channels is invalid
			else
				GPIOPinWrite(GPIO_PORTF_BASE, LED_MASK, 0x02);
		}
		// turn on the green led on exit from the channelSet loop
		// to show that the channels have been set and system is ready
		// to start recording
		GPIOPinWrite(GPIO_PORTF_BASE, LED_MASK, 0x08);
	}
	
	// disable interrupts for adc and udma initialization
	IntMasterDisable();
	
	// turn on blue led on exit from start loop to show that
	// the system is now recording
	GPIOPinWrite(GPIO_PORTF_BASE, LED_MASK, 0x04);
	
	// initialize status flags for ADCBuffer1 and ADCBuffer2
	BufferStatus[0] = FILLING;
	BufferStatus[1] = EMPTY;
	
	// initialize the DMA and ADC for operation
	// SPI initialization should be here as well
	DMA_Init();
	ADC0_Init();
	IntMasterEnable();
	
	while(1){
		if(BufferStatus[0] == FULL){
			sum = 0;
			for(int i = 0; i < BUFFERSIZE; i++){
				sum += ADCBuffer1[i];
				SPIBufferA[i] = ADCBuffer1[i] - OFFSET;
				ADCBuffer1[i] = 0;
			}
			BufferStatus[0] = EMPTY;
//			uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADCBuffer1, BUFFERSIZE);
//			uDMAChannelEnable(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT);
		}
		else if(BufferStatus[1] == FULL){
			sum = 0;
			for(int i = 0; i < BUFFERSIZE; i++){
				sum += ADCBuffer2[i];
				SPIBufferB[i] = ADCBuffer2[i] - OFFSET;
				ADCBuffer2[i] = 0;
			}
			BufferStatus[1] = EMPTY;
//			uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG, (void *)(ADC0_BASE + ADC_O_SSFIFO0), &ADCBuffer2, BUFFERSIZE); 
//			uDMAChannelEnable(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT);
		}
		avg = sum << 9;
		
		// if the average minus the offset voltage is below threshold, count
		// it as a silence and increment countGaps
		if(avg - OFFSET < 20 || avg - OFFSET < -20)
			countGaps++;
		else
			countGaps = 0;
		
		// if silences are detected, turn the led purple
		// else, turn on the blue led alone
		// in future this function will also call getNewHeader() from wav.c
		// to construct a new header, since the silences mark the beginning
		// of a new track
		if(countGaps == 255){
			countGaps = 0;
			GPIOPinWrite(GPIO_PORTF_BASE, LED_MASK, 0x06);
		}
		else
			GPIOPinWrite(GPIO_PORTF_BASE, LED_MASK, 0x04);
	}
	
}
