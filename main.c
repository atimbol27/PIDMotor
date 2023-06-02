/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/******************************************************************************
 * This project provides two demo applications.  A simple blinky style project,
 * and a more comprehensive test and demo application.  The
 * configCREATE_SIMPLE_TICKLESS_DEMO setting (defined in FreeRTOSConfig.h) is
 * used to select between the two.  The simply blinky demo is implemented and
 * described in main_blinky.c.  The more comprehensive test and demo application
 * is implemented and described in main_full.c.
 *
 * The comprehensive demo uses FreeRTOS+CLI to create a simple command line
 * interface through a UART.
 *
 * The blinky demo uses FreeRTOS's tickless idle mode to reduce power
 * consumption.  See the notes on the web page below regarding the difference
 * in power saving that can be achieved between using the generic tickless
 * implementation (as used by the blinky demo) and a tickless implementation
 * that is tailored specifically to the MSP432.
 *
 * This file implements the code that is not demo specific.
 *
 * See http://www.FreeRTOS.org/TI_MSP432_Free_RTOS_Demo.html for instructions.
 *
 */

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/*-----------------------------------------------------------*/

/* NOTE: If an IAR build results in an undefined "reference to __write" linker
error then set the printf formatter project option to "tiny" and the scanf
formatter project option to "small". */

/*
 * Set up the hardware ready to run this demo.
 */
static void prvSetupHardware( void );

/*
 * main_blinky() is used when configCREATE_SIMPLE_TICKLESS_DEMO is set to 1.
 * main_full() is used when configCREATE_SIMPLE_TICKLESS_DEMO is set to 0.
 */
extern void main_blinky( void );
extern void main_full( void );

/*-----------------------------------------------------------*/
void config_keypad_display(void); //done
void config_TIMER_A0(void); //done
void config_NVIC(void); //done
void config_encoder(void); //done
void prvConfigureClocks(void);
void show(int numb);
int input_key(int key_press);
int decode(int r, int c);
void convert_float_to_array(int temp_result);
void vTask_display(void *pvParameters);
void vTask_PID(void *pvParameters);

float Kp_s = 0.005;
float Ki_s = 0.003;
float Kd_s = 0.00001;

int digit_array[] = {0b11000000, //0
                     0b11111001, //1
                     0b10100100, //2
                     0b10110000, //3
                     0b10011001, //4
                     0b10010010, //5
                     0b10000010, //6
                     0b11111000, //7
                     0b10000000, //8
                     0b10010000};//9

int port8[]={0b11011111, 0b11101111, 0b11110111, 0b11111011};

int keypad_table[4][4]={{1,2,3,10},
                        {4,5,6,11},
                        {7,8,9,12},
                        {14,0,15,13}};

uint8_t display[4]={0,0,0,0};

int target_RPM;

TaskHandle_t xPIDHandle;
int pcPIDgain;

float x[2];
float y[2];
int prevError_s = 0;
float error_s = 0;
float integral_s = 0;
float diff_s = 0;
int speed;
int speed_filtered;
float PID_s;

int main( void )
{
    config_keypad_display(); //done
    config_TIMER_A0(); //done
    config_NVIC(); //done
    config_encoder(); //done
    prvConfigureClocks();

    xTaskCreate(vTask_display, "display_task", 512, NULL, 3, NULL);
    xTaskCreate(vTask_PID, "PID_task", 512, (void *)&pcPIDgain, 4, &xPIDHandle);

    vTaskSuspend(xPIDHandle);

    vTaskStartScheduler();

}


/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
extern void FPU_enableModule( void );

	/* The clocks are not configured here, but inside main_full() and
	main_blinky() as the full demo uses a fast clock and the blinky demo uses
	a slow clock. */

	/* Stop the watchdog timer. */
	MAP_WDT_A_holdTimer();

	/* Ensure the FPU is enabled. */
	FPU_enableModule();

	/* Selecting P1.2 and P1.3 in UART mode and P1.0 as output (LED) */
	MAP_GPIO_setAsPeripheralModuleFunctionInputPin( GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION );
	MAP_GPIO_setOutputLowOnPin( GPIO_PORT_P1, GPIO_PIN0 );
	MAP_GPIO_setAsOutputPin( GPIO_PORT_P1, GPIO_PIN0 );
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void *malloc( size_t xSize )
{
	/* There should not be a heap defined, so trap any attempts to call
	malloc. */
	Interrupt_disableMaster();
	for( ;; );
}
/*-----------------------------------------------------------*/

void config_TIMER_A0(void){
    //Set P2 as TA0.1 and TA0.2
    P2->SEL1 &= ~(BIT5 | BIT4); //0
    P2->SEL0 |= (BIT5 | BIT4); //1

    //Set P2.4 and P2.5 as OUTPUT
    P2->DIR |= BIT5 | BIT4;

    TIMER_A0->CTL &= ~(BIT8 | BIT9); //Reset bits
    //Select Clock Source to SMCLK
    //Bit 8-9 is timer clock source select: 10b = SMCLK
    TIMER_A0->CTL |= BIT9;
    TIMER_A0->CTL &= ~BIT8;

    //bit 6 to 7: input divider. 00b = /1
    TIMER_A0->CTL &= ~(BIT6 | BIT7);

    TIMER_A0->CTL &= ~(BIT4 | BIT5);//reset bits

    //bit 4 to 5: mode control. 01b = up mode: counter counts up to TAxCCR0 then down
    TIMER_A0->CTL |= BIT4;

    //Bit 8: capture mode -0b = Compare Mode
    TIMER_A0->CCTL[1] &= ~BIT8; //Select COMPARE Mode
    TIMER_A0->CCTL[2] &= ~BIT8; //Select COMPARE Mode

    //bit 7-5: output mode -011b = Set/Reset Mode
    TIMER_A0->CCTL[1] |= BIT7 | BIT6 | BIT5; //Reset/Set Mode
    TIMER_A0->CCTL[2] |= BIT7 | BIT6 | BIT5; //Reset/Set Mode

    //Setting up capture/compare registers
    //CCR0 is the MAX count before it resets to 0
    TIMER_A0->CCR[0] = 150; //TA1CCR0 is the period of the timer
}

int enc_counter = 0;

void PORT3_IRQHandler (void){
   // printf("Interrupt from Phase A\n");
    if(P3->IV == 0x0E){
        if(P5->IN & BIT3){
            //printf("Count DOWN\n");
            enc_counter--;
        }
        else{
            //printf("Count UP\n");
            enc_counter++;
        }
    }
}

void PORT5_IRQHandler (void){
    //printf("Interrupt from Phase B\n");
    if(P5->IV == 0x08){
            if(P3->IN & BIT6){
                //printf("Count UP\n");
                enc_counter++;
            }
            else{
                //printf("Count DOWN\n");
                enc_counter--;
            }
        }
}

void config_keypad_display(void){
    /************************
       P8.x and P4.x OUTPUT
     ************************/
        P8->SEL0 &= ~0x3C;
        P8->SEL1 &= ~0x3C;
        P8->DIR = 0b00111100; //P8.2 to P8.5 is Digit D to A set to OUTPUT

        P4->SEL0 &= ~0xFF;
        P4->SEL1 &= ~0xFF;
        P4->DIR = 0b11111111;

     /***********************
             P9.x INPUT
      ***********************/
        P9->SEL0 &= ~0x0F;
        P9->SEL1 &= ~0x0F;
        P9->DIR = 0b11110000;

        P9->OUT = 0b00001111;
}

void config_encoder(void){
    P3->SEL0 = 0x00;
    P3->SEL1 = 0x00;

    P3->DIR = 0b10111111; //Set P3.6(Phase A) to Input

    P5->SEL0 = 0x00;
    P5->SEL1 = 0x00;

    P5->DIR = 0b00000001; //Set P5.3(Phase B) to Input & P5.0(LED) to Output

    //Phase A = P3.6
    //Enable Interrupt for P3.6
    P3->IES |= BIT6;  //-1b falling edge
    P3->IE |= BIT6;


    //Phase B = P5.3
    //Enable Interrupt for P5.3
    P5->IES |= BIT3; //-1b falling edge
    P5->IE |= BIT3;
}

void config_NVIC(void){
    NVIC->ISER[1] = 1 << (PORT3_IRQn & 0x001F); //Enable P3 (37) to Interrupt
    NVIC->ISER[1] = 1 << (PORT5_IRQn & 0x001F); //Enable P5 (39) to Interrupt
    NVIC->IP[37] |= BIT5;
    NVIC->IP[39] |= BIT5;
}

void prvConfigureClocks(void){
    //Set Power Level for 48Mhz (VCORE1 = 1.4V)
    uint32_t key_bits = 0x695A0000; //Unlock Key
    uint32_t AM_LDO_VCORE_bits = 0x00000001; //(AMR) Active Mode Request - 01b = AM_LDO_VCORE1
    while(PCM->CTL1 & 0x00000100) //wait for PCM to not be busy
    PCM->CTL0 = key_bits | AM_LDO_VCORE_bits; //Unlock PCM register and set the power mode
    while(PCM->CTL1 & 0x00000100) //wait for PCM to not be busy again
    PCM->CTL0 = 0x0000FFFF; //Lock PCM Register

    //Flash Read wait state number change
    FLCTL->BANK0_RDCTL &= ~(BIT(12) | BIT(13) | BIT(14) | BIT(15));// RESET bits
    FLCTL->BANK0_RDCTL |= BIT(12); //wait state number selection  - 0001b: 1 wait states
    FLCTL->BANK1_RDCTL &= ~(BIT(12) | BIT(13) | BIT(14) | BIT(15));// RESET bits
    FLCTL->BANK1_RDCTL |= BIT(12); //wait state number selection  - 0001b: 1 wait states

    //Set the Frequency to 48Mhz
    //Set the Source of MCLK to DCOCLK
    CS->KEY = 0x695A; //UNLOCK CS REGISTERS
    CS->CTL0 |= BIT(18) | BIT(16); //DCORSEL FREQUENCY RANGE = 48MHz
    CS->CTL0 |= BIT(23); //Enable DCOEN, enables oscillator
    CS->CTL1 |= BIT1 | BIT0; //MCLK source: DCOCLK (011b)
    CS->CTL1 |= BIT5 | BIT4; //SMCLK source: DCOCLK (011b)

    CS->CTL1 |= BIT(30) | BIT(29) | BIT(28); //SMCLK Divider: -111b /128
    CS->CLKEN |= BIT1 | BIT3; //Enable MCLK
    CS->KEY = 0x0; //LOCK CS REGISTERS

    while(!(CS->STAT & BIT(26))){} //while HSM clock isnt steady check
}

void vTask_display(void *pvParameters){
    TickType_t xLastWakeTime;

        //set the frequency of the task to 10 or 100Hz
    const TickType_t xPeriod = 5;

            xLastWakeTime = xTaskGetTickCount();

            int x = 0; // referring to the ROW ACTIVE which is P8.5-P8.2
            int digit = 0; //counting which display or Digit in FSM
            int N = 5;
            int row;
            int col = 0;
            int state = 0;
            int counter = 0;
            uint8_t key;

    for(;;){
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        /****************************
         Enabling xth-row and digit
        *****************************/
         P4->OUT = 0xFF; //BLANK DISPLAY
         P8->OUT = port8[x];
         key = input_key(P9->IN);

         /*****************************
                Setting P4->OUT
          ****************************/
         show(display[x]);


            /***********************************************
                              STATE MACHINE
            ************************************************/
                    switch(state){
                                   /***************************
                                               IDLE
                                    ***************************/

                               case 0:
                                   if(key != 0){
                                                 col = key;
                                                 row = x;
                                                 counter = 0;
                                                 state = 1;
                               }
                                   break;

                                   /***************************
                                           PRESS DEBOUNCE
                                    ***************************/

                               case 1:
                                   if(counter > N){
                                       //printf("%d", key);
                                       state=2;
                                   }
                                   if(x == row && key != col){
                                       counter = 0;
                                       state = 0;
                                   }
                                   if(x == row && key == col){
                                       counter++;
                                   }
                                   break;

                                   /***************************
                                           PROCESS KEY
                                    ***************************/

                               case 2:
                                    display[digit] = decode(row, col);
                                    digit++;
                                    if(digit == 4){
                                        digit = 0;
                                    }
                                    counter = 0;
                                    state = 3;
                                    break;

                                     /***************************
                                           RELEASE DEBOUNCE
                                     ***************************/
                               case 3:
                                   if(counter > N){
                                       //key = 0;
                                       counter = 0;
                                       state = 0;
                                       //printf("%d", key);
                                   }
                                   if(x == row && key != 0){
                                       counter = 0;

                                   }
                                   if(x == row && key == 0){
                                       counter++;
                                   }
                                   break;
                    }

                       /****************************
                               FOREVER LOOP
                       ****************************/
                        x++;
                        if(x == 4){
                           x = 0;
                        }
        }
        //display and keypad fsm *done
            //if D is pressed, suspend vTaskPID *done inside decode function
            //if A is pressed, resume  vTaskPID *done inside decode function
        //call vTaskDelayUntil to block this display task? *done
}

void vTask_PID(void *pvParameters){
    TickType_t xLastWakeTime;

    //set the frequency of the task to 4 or 250Hz
    const TickType_t xPeriod = 4;

        xLastWakeTime = xTaskGetTickCount();

        pcPIDgain = * (int *)pvParameters;


    for(;;){
        vTaskDelayUntil(&xLastWakeTime, xPeriod);


        speed = enc_counter * 18.75f;
        enc_counter = 0;
        x[0] = speed;
        y[0] = 0.9922f*y[1] + 0.0078f*x[1];

        speed_filtered = y[0];
        y[1] = y[0];
        x[1] = x[0];

        convert_float_to_array(speed_filtered);

        /* target_RPM is the value entered from the keypad
         * read_display will return this value
         */

        error_s = target_RPM - speed;

        integral_s += (error_s * 0.004f);
        diff_s = (error_s - prevError_s)/0.004f;

        PID_s = Kp_s * error_s + Ki_s * integral_s + Kd_s * diff_s;

        if(PID_s >= 75)
                PID_s = 74;
        else if(PID_s < -75 )
            PID_s = -74;

        //filter the speed to smooth out noise *done
        //pid control calculation: PID_s = Kp * error + Ki * integral + Kd * diff
        //control the new PWM values CCR2 and CCR1 to change the revolution speed
        //call taskdelayuntil to block this PID task to allow the display task to run

        TIMER_A0->CCR[2] = 75 + PID_s; //
        TIMER_A0->CCR[1] = 75 - PID_s; //

        prevError_s = error_s;
    };
}

void convert_float_to_array(int temp_result){
        int num0 = temp_result;
        int num1 = temp_result % 1000;
        int num2 = temp_result % 100;

        //first digit
        if(num0 <= 999){
            num0 = 0;
        }
        while(num0 >= 1000){
            num0 /= 1000;
        }

        //second digit
        if(num1 <= 99){
                num1 = 0;
            }
        while(num1 >= 100)
        {
            num1 /= 100;
        }

        //third digit
        if(num2 <= 9){
                num2 = 0;
            }
        while(num2 >= 10){
            num2 /= 10;
        }

        //last digit
        int num3 = temp_result % 10;

        display[0] = num0;
        display[1] = num1;
        display[2] = num2;
        display[3] = num3;
}

void read_display(void){
    target_RPM = display[3] + display[2] * 10 + display[1] * 100 + display[0] * 1000;
}


int decode(int r, int c){
    int output;
            if(r == 0){
                    if(c ==  4){
                        output = keypad_table[0][0]; //1
                    }
                    if( c == 3){
                        output = keypad_table[0][1]; //2
                    }
                    if( c == 2){
                        output = keypad_table[0][2]; //3
                    }
                    if( c == 1){
                        output = keypad_table[0][3]; //10
                    }

                }

            if(r == 1){
                    if(c == 4){
                        output = keypad_table[1][0]; //4
                    }
                    if( c == 3){
                        output = keypad_table[1][1]; //5
                    }
                    if( c == 2){
                        output = keypad_table[1][2]; //6
                    }
                    if( c == 1){
                        output = keypad_table[1][3]; //11
                    }

                }

            if(r == 2){
                    if(c == 4){
                        output = keypad_table[2][0]; //7
                    }
                    if( c == 3){
                        output = keypad_table[2][1]; //8
                    }
                    if( c == 2){
                        output = keypad_table[2][2]; //9
                    }
                    if( c == 1){
                        output = keypad_table[2][3]; //12
                    }
                }

            if(r == 3){
                    if(c == 4){
                        output = keypad_table[3][0]; //14
                    }
                    if( c == 3){
                        output = keypad_table[3][1]; //0
                    }
                    if( c == 2){
                        output = keypad_table[3][2]; //15
                    }
                    if( c == 1){
                        output = keypad_table[3][3]; //13
                    }
                }

    if(output == keypad_table[3][3]){
        TIMER_A0->CCR[1] = 75;
        TIMER_A0->CCR[2] = 75;
        error_s = 0;
        PID_s = 0;
        speed = 0;
        display[0] = 0;
        display[1] = 0;
        display[2] = 0;
        display[3] = 0;
        enc_counter = 0;
        //suspendPIDtask
        vTaskSuspend(xPIDHandle);
    }
    if(output == keypad_table[0][3]){
        read_display();
        vTaskResume(xPIDHandle);
        //resumePIDtask
    }

    return output;
}

void show(int numb){
    P4->OUT = digit_array[numb];
}

int input_key(int key_press){
    int ret = 0;

    if(key_press == 8){
        ret = 4;
    }
    if(key_press == 4){
        ret = 3;
    }
    if(key_press == 2){
        ret = 2;
    }
    if(key_press == 1){
        ret = 1;
    }
    return ret;
}
