/*Code run on code composer to run computation for calculating the position of the ball with respect to the obstacles. This also includes the custom
code to read from the external accelerometer using I2C 16 bits at a time. The base code was from FreeRTOS imu for TI's lm'*/
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
//#include "inc/common.h"
#include "driverlib/i2c.h"
#include "inc/i2c_IMU.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "cpu_usage.h"
#include "driverlib/systick.h"
# define GPIO_PA0_U0RX 0x00000001
# define GPIO_PA1_U0TX 0x00000401
#define PI 3.14159265
#define TIME 0.004
#define I2C_PORT I2C1_MASTER_BASE
#define I2C1_MASTER_BASE        0x40021000  // I2C1 Master
float t = 0;//timer to measure every two seconds updated in timer 0 and used in timer 1
float BALL_SIZE =20.00;// Radius of ball
int COMPUATION_TIME=10;/*Time for generation of ball positions after the given time*/
int MODE =1; /*Visualization or computaion mode*/
unsigned int g_ulCPUUsage;
float compute_distance_1(float X,float Y,float m, float out);
float compute_distance_2(float X,float Y,float m);
//static int percent1;
void uarttogui(int index, float Px, float Py,float output1,int percent1);
float gAXC, gAYC, gAZC;
float fraction;
float d1,d2;
static uint32_t ui32Value, ui32Usage,g_ui32CPUUsagePrevious;;
//static uint32_t g_ui32CPUUsagePrevious;
static uint32_t g_ui32CPUUsage;
static uint32_t g_ui32CPUUsageTicks;
//static int percent;
uint32_t g_ui32SysClock;
static uint32_t g_pui32CPUUsageTimerBase[6] =
{
    TIMER0_BASE, TIMER1_BASE, TIMER2_BASE, TIMER3_BASE, TIMER4_BASE,
    TIMER5_BASE
};
static uint32_t g_ui32CPUUsageTimer;
//uint32_t ui32Value, ui32Usage;
short gAXRaw, gAYRaw, gAZRaw;
struct ball{
float Vx;
float Vy;
float Px;
float Py;
int ball_index;
int flag1;
int flag2;
};
uint32_t output=0;
float percent;
uint32_t final;
void collision_detected(struct ball *ball1,float m);

float Init_pos_x =900,Init_pos_y=900;
float Line1_X1=500,Line1_Y1=300,Line1_X2=1000,Line1_Y2=800;
float Line2_X1=20,Line2_Y1=750,Line2_X2=500,Line2_Y2=100;// lines positions. these values needs to be updaed by java

//int No_of_collision =0;
int NO_OF_BALLS=0 ;
struct ball ball1[500];
static uint32_t g_pui32CPUUsageTimerPeriph[6] =
{
    SYSCTL_PERIPH_TIMER0, SYSCTL_PERIPH_TIMER1, SYSCTL_PERIPH_TIMER2,
    SYSCTL_PERIPH_TIMER3, SYSCTL_PERIPH_TIMER4, SYSCTL_PERIPH_TIMER5
};
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    unsigned long ulStatus;

    //
    // Get the interrrupt status.
    //
    ulStatus = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART0_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE,
                                   ROM_UARTCharGetNonBlocking(UART0_BASE));

        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
    }
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************

void collision_detected(struct ball *ball1,float m){

float E =sqrt(((ball1->Vx)*(ball1->Vx))+ ((ball1->Vy)*(ball1->Vy)));
float cosaplpha = 1/sqrt(m*m+1);
float sinalpha = m/sqrt(m*m+1);
float cosgamma = (ball1->Vx/sqrt((ball1->Vx*ball1->Vx)+ (ball1->Vy*ball1->Vy)));
float singamma = (ball1->Vy/sqrt((ball1->Vx*ball1->Vx)+ (ball1->Vy*ball1->Vy)));
ball1->Vx=(float) E*((2*cosaplpha*cosaplpha-1)*cosgamma+(2*sinalpha*cosaplpha)*singamma);
ball1->Vy=(float) E*((2*sinalpha*cosaplpha)*cosgamma-(2*cosaplpha*cosaplpha-1)*singamma);

}

float compute_distance_1(float X,float Y,float m, float out)
{
float A = m;//m
float B = -1;
float C = (Line1_Y1)-m*(Line1_X1+out);//800--m*(out+100)

float distance = (fabs(A*X+B*Y+C))/(sqrt(A*A+B*B));
return distance;
}

float compute_distance_2(float X,float Y,float m)
{

	float A = m;
	float B = -1;
	float C = Line2_Y1-m*Line2_X1;

float distance = (fabsf(A*X+B*Y+C))/(sqrt(pow(A,2)+pow(B,2)));
return distance;
}

int
main(void)
{

		unsigned long ulPeriod,ulPeriod2;
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

		TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC);
		TimerConfigure(TIMER1_BASE, TIMER_CFG_A_PERIODIC);

		ulPeriod = (SysCtlClockGet()*2) ;
		ulPeriod2= (SysCtlClockGet()/250);

		TimerLoadSet(TIMER0_BASE, TIMER_A, ulPeriod -1);
		TimerLoadSet(TIMER1_BASE, TIMER_A, ulPeriod2 -1);
		//CPUUsageInit(ulPeriod2,10000, 1);

		IntEnable(INT_TIMER0A);
		IntEnable(INT_TIMER1A);

		TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

		IntMasterEnable();

		TimerEnable(TIMER0_BASE, TIMER_A);
		TimerEnable(TIMER1_BASE, TIMER_A);
		/*g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
		                                             SYSCTL_OSC_MAIN |
		                                             SYSCTL_USE_PLL |
		                                             SYSCTL_CFG_VCO_480), 120000000);
		ROM_SysTickPeriodSet(g_ui32SysClock / 100);
		    ROM_SysTickIntEnable();
		    ROM_SysTickEnable();
		    CPUUsageInit(g_ui32SysClock, 100, 2);*/

/*

	    //
	    // Determine the number of system clocks per measurement period.
	    //
	    g_ui32CPUUsageTicks = SysCtlClockGet() / (SysCtlClockGet()/250);

	    //
	    // Set the previous value of the timer to the initial timer value.
	    //
	    g_ui32CPUUsagePrevious = 0xffffffff;

	    //
	    // Enable peripheral clock gating.
	    //
	    MAP_SysCtlPeripheralClockGating(true);

	    //
	    // Enable the third timer while the processor is in run mode, but disable
	    // it in sleep mode.  It will therefore count system clocks when the
	    // processor is running but not when it is sleeping.
	    //
	    MAP_SysCtlPeripheralEnable(g_pui32CPUUsageTimerPeriph[1]);
	    MAP_SysCtlPeripheralSleepDisable(g_pui32CPUUsageTimerPeriph[1]);

	    //
	    // Configure the third timer for 32-bit periodic operation.
	    //
	    MAP_TimerConfigure(g_pui32CPUUsageTimerBase[1],
	                       TIMER_CFG_PERIODIC);

	    //
	    // Set the load value for the third timer to the maximum value.
	    //
	    MAP_TimerLoadSet(g_pui32CPUUsageTimerBase[1], TIMER_A, 0xffffffff);

	    //
	    // Enable the third timer.  It will now count the number of system clocks
	    // during which the processor is executing code.
	    //
	    MAP_TimerEnable(g_pui32CPUUsageTimerBase[1], TIMER_A);

*/

		//
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    ROM_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    ROM_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    ROM_GPIOPinConfigure(GPIO_PA7_I2C1SDA);



    			ROM_I2CMasterInitExpClk(I2C_PORT, SysCtlClockGet(), 1); // 1 : 400Khz, 0 : 100Khz

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    UARTStdioConfig(0, 115200, 16000000);
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));


    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    ROM_I2CMasterInitExpClk(I2C_PORT, SysCtlClockGet(), 1); // 1 : 400Khz, 0 : 100Khz
    //
    // Prompt for text to be entered.
    //



    imu_Init();


    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    {//cpuusage=CPUUsageTick();

    }
}
void uarttogui(int index, float Px, float Py, float output1,int percent1){

	/*char buffer[21];
	memset(buffer,0,21);//%0.3d %0.3d %0.3d ,index,(int)Px,(int)Py,,(int)output1
	sprintf(buffer,"%0.3d %0.3d %0.3d %0.3d %0.3d",index,(int)Px,(int)Py,(int)output1,percent1);
	//UARTSend((unsigned char *)buffer, 21);*/
	UARTprintf("%03d %03d %03d %03d %03d\n",index,(int)Px,(int)Py,(int)output1,percent1);
}

void Timer0IntHandler(void)
{


/*adding the new ball every two seconds*/
	t = t+2;
	int i=NO_OF_BALLS;
/*Initialization to zero initially*/
	ball1[i].Vx=0;
	ball1[i].Vy=0;
	ball1[i].Py=Init_pos_y;
	ball1[i].Px=Init_pos_x;
	ball1[i].flag1=0;
	ball1[i].flag2=0;

	NO_OF_BALLS =NO_OF_BALLS+1;

	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2))
	{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	}
	else
	{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 4);
	}

}

void Timer1IntHandler(void)

{
	//g_ulCPUUsage=CPUUsageTick();

	adxl345_ReadXYZRawData(&gAXRaw, &gAYRaw, &gAZRaw);
    adxl345_GetCorrectedData(gAXRaw, gAYRaw, gAZRaw, &gAXC, &gAYC, &gAZC);    	//update with calibration Data
			if(gAXRaw<0)
			{	fraction=(float)gAXRaw/256;
				output=250-abs(fraction*250);
			}
			/*else if(gAXRaw==0)
			{
				gAXRaw=450;

			}*/
			else if(gAXRaw>0)
			{	fraction=(float)gAXRaw/256;
				output=(fraction*250)+250;
			}
	/*float m1 = (800-800)/((output+100)-output);*/
    float m1 = ((Line1_Y2)-(Line1_Y1))/((Line1_X1+output+300)-(Line1_X1+output));
	float m2 = (Line2_Y2-Line2_Y1)/(Line2_X2-Line2_X1);
	int i;
	for( i=0;i<NO_OF_BALLS;i++)// add this part in other timer
		{

			ball1[i].Vy=ball1[i].Vy-9800*TIME;
			ball1[i].Px=ball1[i].Px+ball1[i].Vx*TIME;
			ball1[i].Py=ball1[i].Py+ball1[i].Vy*TIME-0.5*9800*TIME*TIME;
			d1=compute_distance_1(ball1[i].Px,ball1[i].Py,m1,output);
			d2=compute_distance_2(ball1[i].Px,ball1[i].Py,m2);
			//int has=(int) d1;
			if((d1< (BALL_SIZE) )&& (ball1[i].flag1==0)) //Flag is set to prevent multiple detecion
			{

				collision_detected(&ball1[i],m1);
				ball1[i].flag1=1;

			}
			/*if (ball1[i].Py>798 && (ball1[i].flag1==0))
						{
							if(ball1[i].Px>output && ball1[i].Px<output+100)
							{

							ball1[i].Py=797;
							ball1[i].Vy=-ball1[i].Vy;
							ball1[i].flag1=1;
							//ball1[i].Vx=-ball1[i].Vx;
							}

							else if(ball1[i].Px>(output/2)+50 && ball1[i].Px<output+100)
							{

							ball1[i].Py=797;
							ball1[i].Vy=-ball1[i].Vy;
							ball1[i].Vx=10*ball1[i].Vx;
							}




							else
											{

								ball1[i].Py=-10000;
								ball1[i].flag1=1;


											}
						}

						else if (ball1[i].Py<10 && (ball1[i].flag1==0))
									{





										ball1[i].Vy=ball1[i].Vy;
										ball1[i].flag1=1;
									}
						else
						{


														ball1[i].Vy=ball1[i].Vy;
														ball1[i].flag1=1;
						}


						if (ball1[i].Px>798 && (ball1[i].flag1==0))
						{
							ball1[i].Px=797;
							ball1[i].Vx=-ball1[i].Vx;
							ball1[i].flag1=1;
						}

						else if (ball1[i].Px<10 && (ball1[i].flag1==0))
									{
										ball1[i].Px=11;

										ball1[i].Vx=-ball1[i].Vx;
										ball1[i].flag1=1;
									}*/
			if((d2<(BALL_SIZE)) && (ball1[i].flag2==0))
			{

				collision_detected(&ball1[i],m2);
				ball1[i].flag2=1;


			}

			else if((ball1[i].flag1==1) && (d1>20)){
			ball1[i].flag1=0;
				}

			else if((ball1[i].flag2==1) && (d2>20)){
			ball1[i].flag2=0;
				}
			else if((ball1[i].Px>0&&ball1[i].Px<1000)&&((ball1[i].Py>0)&& (ball1[i].Py<1000)))
			{
				uarttogui(i,ball1[i].Px,ball1[i].Py,output,percent);
			}

			else if((ball1[i].Px<0 && ball1[i].Px>1000)&&((ball1[i].Py<0) && (ball1[i].Py>1000)))
			{

				i++;
			}





		}
	//g_ui32CPUUsagePrevious = ui32Value;
	ui32Value =
	        MAP_TimerValueGet(g_pui32CPUUsageTimerBase[1],
	                          TIMER_A);

	    //
	    // Based on the number of clock ticks accumulated by the timer during the
	    // previous timing period, compute the CPU usage as a 16.16 fixed-point
	    // value.
	    //

		ui32Usage = ((((g_ui32CPUUsagePrevious - ui32Value) * 6400) /
	                 4000000) * 1024);

	    //
	    // Save the previous value of the timer.
	    //

			g_ui32CPUUsagePrevious = ui32Value;


		percent= ui32Usage >> 16;

	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3))
	{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	}
	else
	{
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 8);
	}

}


