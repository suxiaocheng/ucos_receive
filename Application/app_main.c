/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2012; Micrium, Inc.; Weston, FL
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                         STM32F103C8T6-EVAL
*                                         Evaluation Board
*
* Filename      : app_main.c
* Version       : V1.00
* Programmer(s) : George
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <includes.h>

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
static OS_STK AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
#ifdef NOR_FLASH_OPERATION_ENABLE
static OS_STK AppTaskNorFlashOperationStk[APP_CFG_TASK_START_STK_SIZE];
#endif
static OS_STK AppTaskNorNRF24L01Stk[APP_CFG_TASK_START_STK_SIZE];

#ifdef NOR_FLASH_OPERATION_ENABLE
/* Used for nor flash test */
INT8U spi_ram_buf[512];
INT8U spi_ram_cache[512];
#endif

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static void AppTaskStart(void *p_arg);
#ifdef NOR_FLASH_OPERATION_ENABLE
static void AppTaskNorFlashOperation(void *p_arg);
#endif
static void AppTaskNRF24L01(void *p_arg);

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/

int main(void)
{
	INT8U os_err;

	BSP_Init();

	stm_printf("System Startup\n");

	OSInit();		/* Initialize "uC/OS-II, The Real-Time Kernel"          */

#if OS_TASK_CREATE_EXT_EN > 0u
	OSTaskCreateExt((void (*)(void *))AppTaskStart,	/* Create the start task                                */
			(void *)0,
			(OS_STK *) & AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE
						     - 1],
			(INT8U) APP_CFG_TASK_START_PRIO,
			(INT16U) APP_CFG_TASK_START_PRIO,
			(OS_STK *) & AppTaskStartStk[0],
			(INT32U) APP_CFG_TASK_START_STK_SIZE, (void *)0,
			(INT16U) (OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
#else
	(void)OSTaskCreate(AppTaskStart,
			   (void *)0,
			   &AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE - 1],
			   APP_CFG_TASK_START_PRIO);
#endif

#if (OS_TASK_NAME_EN > 0)
	OSTaskNameSet((INT8U) APP_CFG_TASK_START_PRIO,
		      (INT8U *) "Start_Task", (INT8U *) & os_err);
#endif

	OSStart();		/* Start multitasking (i.e. give control to uC/OS-II)   */

	return (1);
}

/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static void AppTaskStart(void *p_arg)
{
	INT32U cnts;
	INT32U current_status = FALSE;
	RCC_ClocksTypeDef RCC_Clocks;

	BSP_Init();

	(void)p_arg;		/* Note #1                                             */

	//OS_CPU_SysTickInit(OS_TICKS_PER_SEC);                       /* Initialize tick counter                              */

	RCC_GetClocksFreq(&RCC_Clocks);	/* Determine SysTick reference freq.                    */
	cnts = RCC_Clocks.SYSCLK_Frequency / (INT32U) OS_TICKS_PER_SEC;
	OS_CPU_SysTickInit(cnts);	/* Init uC/OS periodic time src (SysTick).              */
#if NOR_FLASH_OPERATION_ENABLE
#if OS_TASK_CREATE_EXT_EN > 0u
	OSTaskCreateExt((void (*)(void *))AppTaskNorFlashOperation,	/* Create the start task                                */
			(void *)0,
			(OS_STK *) &
			AppTaskNorFlashOperationStk[APP_CFG_TASK1_STK_SIZE - 1],
			(INT8U) APP_CFG_TASK_NOR_FLASH_OP_PRIO,
			(INT16U) APP_CFG_TASK_NOR_FLASH_OP_PRIO,
			(OS_STK *) & AppTaskNorFlashOperationStk[0],
			(INT32U) APP_CFG_TASK1_STK_SIZE, (void *)0,
			(INT16U) (OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
#else
	(void)OSTaskCreate(AppTaskNorFlashOperation,
			   (void *)0,
			   &AppTaskNorFlashOperationStk[APP_CFG_TASK1_STK_SIZE -
							1],
			   APP_CFG_TASK_NOR_FLASH_OP_PRIO);
#endif
#endif

#if OS_TASK_CREATE_EXT_EN > 0u
	OSTaskCreateExt((void (*)(void *))AppTaskNRF24L01,	/* Create the start task                                */
			(void *)0,
			(OS_STK *) &
			AppTaskNorNRF24L01Stk[APP_CFG_TASK2_STK_SIZE - 1],
			(INT8U) APP_CFG_TASK_NRF24L01_PRIO,
			(INT16U) APP_CFG_TASK_NRF24L01_PRIO,
			(OS_STK *) & AppTaskNorNRF24L01Stk[0],
			(INT32U) APP_CFG_TASK2_STK_SIZE, (void *)0,
			(INT16U) (OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
#else
	(void)OSTaskCreate(AppTaskNorFlashOperation,
			   (void *)0,
			   &AppTaskNorNRF24L01Stk[APP_CFG_TASK2_STK_SIZE - 1],
			   APP_CFG_TASK_NRF24L01_PRIO);
#endif

#if (OS_TASK_STAT_EN > 0)
	OSStatInit();		/* Determine CPU capacity                               */
#endif

	while (1) {		/* Task body, always written as an infinite loop.       */
		OSTimeDlyHMSM(0, 0, 0, 500);
		lcd_backlight(current_status);
		current_status = !current_status;
		stm_printf("Tick\n");
	}
}
#ifdef NOR_FLASH_OPERATION_ENABLE
static void AppTaskNorFlashOperation(void *p_arg)
{
	INT32U count;
	INT32U err = 0;
	INT32U current_stage;	//0: program, 1: read, 2:compare

	for (count = 0; count < INIT_RETRY_TIMES; count++) {
		if (nor_flash_init() == 0) {
			break;
		}
		OSTimeDlyHMSM(0, 0, 0, INIT_RETRY_DELAY);
	}
	while (1) {
		OSTimeDlyHMSM(0, 0, 0, 500);
		stm_printf("Task1\n");
	}
}
#endif

//#define NRF24L01_TEST

void AppTaskNRF24L01(void *p_arg)
{
	uint32_t status = 0;
	uint8_t receive_dat[4];
	uint32_t ret;
	
	#ifndef NRF24L01_TEST
	nrf24l01_init();

	nrf24l01_set_rx_mode();

	while (1) {
		do{
			ret = nrf24l01_receive_data(receive_dat);
			if(ret){
				stm_printf("Receive data, %x-%x-%x-%x\n", 
					receive_dat[0], receive_dat[1], 
					receive_dat[2], receive_dat[3]);
			}
		}while(ret);
		OSTimeDlyHMSM(0, 0, 0, 20);
		//stm_printf("Task2\n");
	}
	
	#else

	{
		GPIO_InitTypeDef GPIO_InitStructure;
		/* Init the gpio */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	
	while (1) {
		OSTimeDlyHMSM(0, 0, 0, 500);
		stm_printf("Task2\n");
		status = !status;
		if(status){
			GPIO_SetBits(GPIOA, GPIO_Pin_8);
			GPIO_SetBits(GPIOB, GPIO_Pin_12);
			GPIO_SetBits(GPIOB, GPIO_Pin_13);
			GPIO_SetBits(GPIOB, GPIO_Pin_14);
			GPIO_SetBits(GPIOB, GPIO_Pin_15);
		}else{
			GPIO_ResetBits(GPIOA, GPIO_Pin_8);
			GPIO_ResetBits(GPIOB, GPIO_Pin_12);
			GPIO_ResetBits(GPIOB, GPIO_Pin_13);
			GPIO_ResetBits(GPIOB, GPIO_Pin_14);
			GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		}		
	}
	#endif
}
