/* FreeRTOS-libopencm3 test to echo characters on USART1 and blink a LED

The first LED is blinked regularly. Characters from the source on USART1 are
echoed. The second LED is toggled with the reception of a character, and the
third is toggled on the transmission of a character.

The board used is the ET-STM32F103 with LEDs on port B pins 8-15,
but the test should work on the majority of STM32 based boards.
*/

/*
    FreeRTOS V8.2.3 - Copyright (C) 2011 Real Time Engineers Ltd.
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pars_const.h"
#include "periph.h"

/* Libopencm3 includes. */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scs.h>

#include <libopencm3/stm32/f1/exti.h>
#include <libopencm3/stm32/f1/timer.h>

#define BUFFER_SIZE 64

#ifdef ENABLE_SEMIHOSTING
extern void initialise_monitor_handles(void);
#endif

static uint8_t iBleBuf = 0;
static char cBleInBuf[BLE_PACK_SIZE + 4];
static char cReceivedPack[BLE_PACK_SIZE + 4];

uint16_t ADCbuffer[12];
uint16_t echo_count = 0;
uint8_t flag_power_off = 0;
uint8_t RxOverflow = 0;

uint8_t cnt_status = CNT_ON_MS;
uint32_t ir_count = 0;
echo_sensor esensor = { 0, 0, 0/*, false*/};
char dbg_buff[16];

static uint16_t remainCmdQueue = cmd_QUEUE_LEN;
static uint16_t remainCmdMotQueue = cmd_QUEUE_LEN;
static uint16_t remainCmdSt1Queue = cmd_QUEUE_LEN;
static uint16_t remainCmdSt2Queue = cmd_QUEUE_LEN;
static uint16_t remainCmdServoQueue = cmd_QUEUE_LEN;
static uint16_t remainCmdLedsQueue = cmd_QUEUE_LEN;
static uint16_t remainCmdEchoQueue = cmd_QUEUE_LEN;

static uint8_t init_process = 0;  // идет инициализация железа
static uint16_t adcval[5] = {0, 0, 0, 0, 0}; // 3.3V, 6V, P5, P6, delta
static char jparams[3][8];  // параметры из пакета

static TickType_t leds_delay = pdMS_TO_TICKS(1000);
static TickType_t board_led_delay = pdMS_TO_TICKS(500);

extern motor_ctrl motors;
extern mob_leds leds[];
extern stepper stepp[2];
extern servo_drive servo;
extern uint8_t axle_stepper;
extern uint8_t axle_search;
extern mob_leds leds[];

/* for FreeRTOS */
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
extern void vPortSVCHandler( void );

/* Prototypes */
static void clock_setup(void);
static void systickSetup();
static void usart_setup(void);
static void gpio_setup(void);
static void tim1_setup(void);
static void tim2_setup(void);
static void tim3_setup(void);
static void tim4_setup(void);

static void adc_setup(void);
static void dma_adc_init(void);

static void dma_write(char *data, int size);

char * itoa(int val, int base);

void ADC_calc(void);
uint8_t get_fields(char *fld);
void running_delay(uint32_t lasting, uint8_t c_status);
void procSteppCmd(uint8_t stnum, ncommand_item cmd);
TickType_t echo_one_shot(void);
void calc_echo_dist(uint16_t count);
void echo_pulse(void);
uint8_t sendPackToBLE(char * blepack);
void check_states(void);
bool push_state(uint8_t mstate, uint8_t num);
void put_to_cmd_queue(uint16_t cmd, int16_t iparam);
void put_motors_cmd(char command, int16_t iparam);
void put_stepp_cmd(uint8_t step_num, char command, int16_t iparam);
void put_servo_cmd(char command, int16_t iparam);
void put_echo_cmd(char command, int16_t iparam);
void put_leds_cmd(char command, uint16_t iparam);

/* Task priorities. */
#define mainBLINK_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

static void prvSetupHardware( void );

//static void prvUsartTask( void *pvParameters );
static void prvBlinkTask( void *pvParameters );
static void prvMainTask(void *pvParameters);
static void prvBLETask(void *pvParameters);
static void prvADCTask( void *pvParameters );
static void prvLedsBlinkTask( void *pvParameters );
static void prvCmdTask(void *pvParameters);
static void prvCmdMotTask(void *pvParameters);
static void prvCmdStTask(void *pvParameters);
static void prvCmdServoTask(void *pvParameters);
static void prvCmdEchoTask(void *pvParameters);
static void prvCmdLedsTask(void *pvParameters);

// События
EventGroupHandle_t xEventGroupBLE;      // события, связанные с BLE
EventGroupHandle_t xEventGroupAlrm;     // события от внешних сигналов и АЦП
EventGroupHandle_t xEventGroupDev;      // необходимость передачи статуса устройств

// Семафоры
SemaphoreHandle_t xSemaphHandleEcho;    // есть результат от эхо-локатора
SemaphoreHandle_t xSemaphVoltage;       // завершено сканирование каналов
SemaphoreHandle_t xSemaphSent;          // пакет в BLE отправлен
SemaphoreHandle_t xSemaphSendErr;       // ошибка отправки
// Мьютексы
SemaphoreHandle_t xSteppMutex[2];       // для управления ШД
SemaphoreHandle_t xIRCountMutex;        // модификация счетчика пути
SemaphoreHandle_t xSendPackMutex;       // для передачи пакета пульту

// очереди
QueueHandle_t xRxedChars;               // очередь принятых символов из BLE
QueueHandle_t xCmdQueue;                // общая очередь команд для исполнения
// новые очереди
QueueHandle_t xCmdMotQueue;             // очередь команд ДПТ
QueueHandle_t xCmdSt1Queue;             // очередь команд ШД1
QueueHandle_t xCmdSt2Queue;             // очередь команд ШД2
QueueHandle_t xCmdServoQueue;           // очередь команд СЕРВО
QueueHandle_t xCmdEchoQueue;            // очередь команд эхо сенсора
QueueHandle_t xCmdLedsQueue;            // очередь команд светодиодов
QueueHandle_t xStateQueue;              // очередь состояний для передачи в пульт

/*-----------------------------------------------------------*/

int main( void )
{
#ifdef ENABLE_SEMIHOSTING
    if(SCS_DHCSR & SCS_DHCSR_C_DEBUGEN) {   // if(CoreDebug->DHCSR & 1)
        initialise_monitor_handles();
    }
#endif

	prvSetupHardware();

#ifdef ENABLE_SEMIHOSTING
	printf("start!!!\n");
#endif

    xEventGroupBLE = xEventGroupCreate();
    xEventGroupAlrm = xEventGroupCreate();
	xEventGroupDev = xEventGroupCreate();

        // создание семафоров
    xSemaphHandleEcho = xSemaphoreCreateBinary();
    xSemaphVoltage = xSemaphoreCreateBinary();
    xSemaphSent = xSemaphoreCreateBinary();
    xSemaphSendErr = xSemaphoreCreateBinary();

    // создание мьютексов
    xSteppMutex[0] = xSemaphoreCreateMutex();
    xSteppMutex[1] = xSemaphoreCreateMutex();
    xIRCountMutex = xSemaphoreCreateMutex();
    xSendPackMutex = xSemaphoreCreateMutex();

    // создание очередей
    xRxedChars = xQueueCreate(serRX_QUEUE_LEN, sizeof(uint8_t));
    xCmdMotQueue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdSt1Queue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdSt2Queue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdServoQueue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdEchoQueue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdLedsQueue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdQueue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xStateQueue = xQueueCreate(state_QUEUE_LEN, sizeof(uint8_t) * (BLE_PACK_SIZE + 1));



	/* Start the blink task. */
	xTaskCreate( prvBlinkTask, "Flash", configMINIMAL_STACK_SIZE, NULL, mainBLINK_TASK_PRIORITY, NULL );
	
	/* Start the usart task. */
	//xTaskCreate( prvUsartTask, "USART", configMINIMAL_STACK_SIZE, NULL, mainBLINK_TASK_PRIORITY, NULL );

	xTaskCreate( prvMainTask, "MainTask", configMINIMAL_STACK_SIZE * 2, NULL, mainBLINK_TASK_PRIORITY, NULL );

	xTaskCreate( prvBLETask, "BLE", configMINIMAL_STACK_SIZE, NULL, mainBLINK_TASK_PRIORITY, NULL );

	xTaskCreate( prvADCTask, "ADC", configMINIMAL_STACK_SIZE, NULL, mainBLINK_TASK_PRIORITY, NULL );

	xTaskCreate( prvLedsBlinkTask, "LedsBlink", configMINIMAL_STACK_SIZE, NULL, mainBLINK_TASK_PRIORITY, NULL );

	xTaskCreate( prvCmdTask, "CommonCmd", configMINIMAL_STACK_SIZE, NULL, mainBLINK_TASK_PRIORITY, NULL );

	xTaskCreate( prvCmdMotTask, "MotorCmd", configMINIMAL_STACK_SIZE, NULL, mainBLINK_TASK_PRIORITY, NULL );

	xTaskCreate( prvCmdStTask, "SteppCmd", configMINIMAL_STACK_SIZE, NULL, mainBLINK_TASK_PRIORITY, NULL );

	xTaskCreate( prvCmdServoTask, "ServoCmd", configMINIMAL_STACK_SIZE, NULL, mainBLINK_TASK_PRIORITY, NULL );

	xTaskCreate( prvCmdEchoTask, "EchoCmd", configMINIMAL_STACK_SIZE, NULL, mainBLINK_TASK_PRIORITY, NULL );

	xTaskCreate( prvCmdLedsTask, "LedsCmd", configMINIMAL_STACK_SIZE, NULL, mainBLINK_TASK_PRIORITY, NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();
	
	/* Will only get here if there was not enough heap space to create the
	idle task. */
	return -1;
}
/*-----------------------------------------------------------*/

static void prvBlinkTask( void *pvParameters )
{
    ( void ) pvParameters;

	portTickType pt0 = xTaskGetTickCount();
	portTickType pt1 = pt0;
	//portTickType xLastExecutionTime;

	// Initialise the xLastExecutionTime variable on task entry. 
	//xLastExecutionTime = xTaskGetTickCount();

    for( ;; )
	{
		// Simply toggle the LED periodically.  This just provides some timing
		//verification./

		//vTaskDelayUntil( &xLastExecutionTime, mainBLINK_DELAY );
		pt1 = xTaskGetTickCount();
		if ((pt1 - pt0) > board_led_delay)    // mainBLINK_DELAY
		{
			//gpio_toggle(GPIOB, GPIO8);	// LED on/off
			gpio_toggle(GPIOC, GPIO13);	// LED on/off
			pt0 = pt1;
		}
		taskYIELD();
	}
}

static void prvMainTask(void *pvParameters)
{
    ( void ) pvParameters;

    board_led_delay = 250;  // init process

    // не готов к обмену во время инициализации
    usart_disable_rx_interrupt(USART1);
    sendPackToBLE((char *)js_at);

    motors.state = MOTOR_OFF;
    esensor.state = ECHO_ONE;
    stepp[0].mode = stepp[1].mode = STEP_MAN;
    set_servo_angle(90);

    vTaskDelay(pdMS_TO_TICKS(1000));

    // готов к обмену
    usart_enable_rx_interrupt(USART1);

    xEventGroupClearBits(xEventGroupAlrm, (const EventBits_t)0xFF);
    // в начале надо передать состояние всех устройств
    xEventGroupSetBits(xEventGroupDev, (const EventBits_t)0x1FFF);

    // Включаем 6V
    gpio_set(POWER_HOLD_PORT, POWER_6V_PIN);
    init_process = 0;

    board_led_delay = 1000;  // normal process

    EventBits_t uxBits;

    TickType_t xLastWakeTime = xTaskGetTickCount();

    // для парсинга пакета
    static char tag[8];
    static char fields[20];
    static char tmpbuff[32];

	// Основной цикл
    for (;;)
    {
		// проверка нажатия кнопки питания, разряда батареи,
        // и команды на выключение
        uxBits = xEventGroupWaitBits(
        	xEventGroupAlrm,
            alarm_POWER_BUTTON_BIT ||
            alarm_CONTRL_OFF_VOLTAGE_BIT ||
            alarm_POWER_COMMAND_BIT,
            pdTRUE,             // очищаем флаг
            pdFALSE,    //       Don't wait for all bits.
            0 );                // ticks to wait

        if ( (uxBits & alarm_POWER_BUTTON_BIT) != 0 )       // нажатие кнопки питания
		{   
        	push_state(STATE_PACK_PWROFF_KEY, 0);
            flag_power_off = 1;
            vTaskDelay(pdMS_TO_TICKS(100));
            power_off();
		}
        else if ( (uxBits & alarm_POWER_COMMAND_BIT) != 0 ) // команда выключения
        {
        	push_state(STATE_PACK_PWROFF_CMD, 0);
            flag_power_off = 1;
            vTaskDelay(pdMS_TO_TICKS(100));
            power_off();
		}
        else if ( (uxBits & alarm_CONTRL_OFF_VOLTAGE_BIT) != 0 )    // разряд батареи
		{
        	push_state(STATE_PACK_PWROFF_BAT, 0);
            flag_power_off = 1;
            vTaskDelay(pdMS_TO_TICKS(100));
			power_off();
		}
                        
        // проверка на недопустимый ток нагрузки
        uxBits = xEventGroupWaitBits(
        	xEventGroupAlrm,
            alarm_OVERLOAD_BIT ||
            alarm_CONTRL_LOW_VOLTAGE_BIT,
            pdTRUE,             // очищаем флаг
            pdFALSE,            // Don't wait for all bits.
            0 );                // ticks to wait
		
		if ( (uxBits & alarm_OVERLOAD_BIT) != 0 )
        {   
        	// Слишком большой ток - очищаем очереди и выключаем моторы
			xQueueReset(xCmdMotQueue);
            xQueueReset(xCmdSt1Queue);
            xQueueReset(xCmdSt2Queue);
            xQueueReset(xCmdServoQueue);
            stop_all();
            push_state(STATE_PACK_OVERLOAD, 0);
		}

        if ( (uxBits & alarm_CONTRL_LOW_VOLTAGE_BIT) != 0 )
        {   
            // низкий заряд батареи
            push_state(STATE_PACK_LOWBAT, 0);
        }

        // проверка наличия входящего пакета и его обработка
        uxBits = xEventGroupWaitBits(
        	xEventGroupBLE, ble_RECIEVED_JSON,
            pdTRUE,             // очищаем флаг
            pdFALSE,    // Don't wait for all bits.
            0 );                // ticks to wait

		if ( (uxBits & ble_RECIEVED_JSON) != 0 )
        {
        	// пришел пакет от ДУ, обрабатываем cReceivedPack[]
            bool parsed = false;
            // парсим входящий пакет, в пакете может быть только 1 команда
            char *pb = strchr(cReceivedPack, '{');
            char *pe = strchr(cReceivedPack, '}');
            int16_t ll = (int16_t)(pe - pb);
            if ( (pb != NULL) && (pe != NULL) && (ll > 0) ) // похож на JSON
            {
                strncpy(tmpbuff, (pb + 1), ll - 1); // без {}

                bool fl_js = false;
                char *token = strtok(tmpbuff, ":");
                if (token != NULL)
                {
                    strcpy(tag, token);
                    token = strtok(NULL, ":");
                    if (token != NULL)
                    {
                        strcpy(fields, token);
                        fl_js = true;
                    }
                    else
                    {
                        fields[0] = '\0';
                    }
                }
                else
                {
                    token[0] = '\0';
                }

                uint8_t n_params = get_fields(fields); // извлекаем параметры

                if (fl_js && (n_params > 0))  // есть команда и параметры
                {
                    parsed = false;
                    // проверка наличия и обработка команд

                    // CMD_PAUSE
                    if (strstr(tag, jsp_pause))
                    {
                        put_to_cmd_queue(CMD_PAUSE, (uint16_t)atoi(jparams[0]));
                        parsed = true;
                    }
                    // CMD_POWER_OFF
                    if ( !parsed && strstr(tag, jsp_pwroff))
                    {
                        parsed = true;
                        if (strstr(jparams[0], js_true))
                        {
                            // выключить без очереди
                            push_state(STATE_PACK_PWROFF_CMD, 0);
                            flag_power_off = 1;
                            vTaskDelay(pdMS_TO_TICKS(100));
                            power_off();
                        }
                        else // в очередь
                        {
                            put_to_cmd_queue(CMD_POWER_OFF, 0);
                        }
                    }
                    // CMD_STOP_ALL
                    if ( !parsed && strstr(tag, jsp_stop))
                    {
                        parsed = true;
                        if (strstr(jparams[0], js_true))
                        {
                            // немедленно
                            xQueueReset(xCmdMotQueue);
                            xQueueReset(xCmdSt1Queue);
                            xQueueReset(xCmdSt2Queue);
                            xQueueReset(xCmdServoQueue);
                            stop_all();                            
                        }
                        else
                        {
                            put_to_cmd_queue(CMD_STOP_ALL, 0);
                        }
                    }
                    //  MOT_STOP, MOT_UP_DOWN, MOT_RIGHT, MOT_LEFT, MOT_STRAIGHT, MOT_PAUSE
                    if ( !parsed && strstr(tag, jsp_mot))
                    {
                        parsed = true;
                        if (n_params == 2)  // 2 параметра - f, b, p
                        {
                            put_motors_cmd(jparams[0][1], (int16_t)atoi(jparams[1]));
                        }
                        else if (n_params == 1)  // 1 параметр - l,r,n,s
                        {
                            put_motors_cmd(jparams[0][1], 0);
                        }

                    }
                    //  ST_UP_DOWN, ST_NULL, ST_NULL_FAST, ST_STOP, ST_CONT_LEFT,
                    //  ST_CONT_RIGHT, ST_CONT_STOP, ST_RET, ST_AXLE, ST_PAUSE
                    if ( !parsed && strstr(tag, jsp_st))
                    {
                        parsed = true;
                        if (n_params == 2)  // 2 параметра - h,n,s,l,r
                        {
                            put_stepp_cmd((uint8_t)atoi(jparams[1]), jparams[0][1], 0);                            
                        }
                        else if (n_params == 3)  // 3 параметра - a,p
                        {
                            put_stepp_cmd((uint8_t)atoi(jparams[1]), jparams[0][1], 
                                (int16_t)atoi(jparams[2]));                            
                        }
                    }
                    //  LED0_MOD, LED1_MOD, LED2_MOD, LED3_MOD, LEDS_PAUSE
                    if ( !parsed && strstr(tag, jsp_led))
                    {
                        parsed = true;
                        put_leds_cmd(jparams[0][1], (uint16_t)atoi(jparams[1]));
                    }
                    // // SERVO_SET, SERVO_PAUSE
                    if ( !parsed && strstr(tag, jsp_servo))
                    {
                        parsed = true;
                        put_servo_cmd(jparams[0][1], (uint16_t)atoi(jparams[1]));
                    }
                    // CMD_CNT_ON_MS, CMD_CNT_ON_STEPS                    
                    if ( !parsed && strstr(tag, jsp_dist))
                    {
                        parsed = true;
                        uint16_t tmp = CMD_NO;
                        switch (jparams[0][1])
                        {
                            case 's':       // в импульсах датчика
                                tmp = CMD_CNT_SET_ON_STEPS;
                                break;
                            case 'c':       // сбросить
                                tmp = CMD_CNT_RESET;
                                break;
                            default:        // m - в милисекундах
                                tmp = CMD_CNT_SET_ON_MS;
                        }
                        put_to_cmd_queue(tmp, 0);
                    }
                    //  ECHO_SHOT, ECHO_PAUSE
                    if ( !parsed && strstr(tag, jsp_echo)) // ECHO_ONE, PAUSE
                    {
                        parsed = true;
                        put_echo_cmd(jparams[0][1], (int16_t)atoi(jparams[1]));
                    }
				}
			}
			// был пакет от пульта
	    	xLastWakeTime = xTaskGetTickCount();
		} // end JSON
		else // не было JSON пакета
        {
			TickType_t xLastWakeTimeNow = xTaskGetTickCount();        	
            if ((xLastWakeTimeNow - xLastWakeTime) > WAIT_FOR_PACK)
            {
                // долго не было пакетов от пульта - останавливаем все
				xLastWakeTime = xLastWakeTimeNow;
                put_to_cmd_queue(CMD_STOP_ALL, 0);
			}
		}

		check_states();

    	taskYIELD();
	}
}

// Прием команд из BLE и отправка статусов из очереди xStateQueue
static void prvBLETask(void *pvParameters)
{
    ( void ) pvParameters;

    static TickType_t xTime1;
    signed char cChar;
    static char sbuf[20];

    xEventGroupClearBits(xEventGroupBLE, (const EventBits_t)0xFF);
    xTime1 = xTaskGetTickCount();
    
    for (;;)
    {
        if (flag_power_off)
        {
            taskYIELD();
        }

        bool pack_end = false;
        // проверяем наличие символа в очереди приема
        if ( xQueueReceive( xRxedChars, &cChar, 0 ) == pdPASS )
        {
            if (cChar != 0x0A)      // не LF
            {
                cBleInBuf[iBleBuf] = cChar;
                iBleBuf++;
                if ( iBleBuf >= BLE_PACK_SIZE )
                {
                    pack_end = true;
                }
            }
            else
            {
                pack_end = true;
            }

            if (pack_end)
            {   
                cBleInBuf[iBleBuf] = '\0'; // завершаем строку
                strcpy(cReceivedPack, cBleInBuf);       // копируем пакет
                iBleBuf = 0;
                // сбрасываем буфер приема
                cBleInBuf[iBleBuf] = '\0';
                // устанавливаем флаг приема пакета
                xEventGroupSetBits(xEventGroupBLE, ble_RECIEVED_JSON);
			}
		}
                        
        TickType_t xTime2 = xTaskGetTickCount();
        // если прошло 10 mS после отправки статуса
        // и есть статус в очереди
        if ( ((xTime2 - xTime1) >= BLE_SEND_DELAY) &&
           (xQueueReceive(xStateQueue, sbuf, 0) == pdPASS ) )
        {
            // отправляем пакет статуса
            sendPackToBLE(sbuf);
            xTime1 = xTime2;
        }

        taskYIELD(); 
    }
}

// Управление АЦП и обработка результатов измерений
static void prvADCTask(void *pvParameters)
{
    ( void ) pvParameters;
                
	vTaskDelay(pdMS_TO_TICKS(20));

    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
    
    while (adc_is_calibrating(ADC1))
    {
		vTaskDelay(pdMS_TO_TICKS(10));    	
    }

    adc_start_conversion_direct(ADC1);
    
    for (;;) {

    	if (flag_power_off)
        {
        	taskYIELD();
		}

		if ( xSemaphoreTake(xSemaphVoltage, 0) == pdTRUE )
        {
			// есть новые результаты измерения
            ADC_calc();
            if ( (adcval[2] >= SENS_LEVEL_4) &&
            	(adcval[2] < SENS_LEVEL_5) )//P5
			{
            	xEventGroupSetBits(xEventGroupAlrm, alarm_HALL1_BIT);
			}
			else if ( (adcval[2] >= SENS_LEVEL_8) &&
            	(adcval[2] < SENS_LEVEL_9) )
			{
            	xEventGroupSetBits(xEventGroupAlrm, alarm_HALL2_BIT);
			}

			// запускаем следующий цикл измерений
            vTaskDelay(pdMS_TO_TICKS(20));

            dma_adc_init();
            adc_start_conversion_direct(ADC1);
		}

		taskYIELD();
	}
}

// обеспечение мигания светодиодов в режиме LED_BLINK
static void prvLedsBlinkTask(void *pvParameters)
{
    ( void ) pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    for (;;)
    {
        if (flag_power_off)
        {
            taskYIELD();
        }

        for (uint8_t i = 0; i < 4; i++)
        {   
        	if ( leds[i].mode == LED_BLINK )
            {
            	gpio_set(leds[i].port, leds[i].pin);
            }
		}
         
        vTaskDelayUntil( &xLastWakeTime, leds_delay );
         
        for (uint8_t i = 0; i < 4; i++)
        {
        	if ( leds[i].mode == LED_BLINK )
            {
                gpio_clear(leds[i].port, leds[i].pin);
            }
        }
         
        vTaskDelayUntil( &xLastWakeTime, leds_delay );
    }                       
}

// Обработка команд общей очереди
static void prvCmdTask(void *pvParameters)
{
    ( void ) pvParameters;

    ncommand_item item;

    for (;;)
    {
        if (flag_power_off)
        {
			taskYIELD();
		}
        // есть ли очередная команда?
        if ( xQueueReceive( xCmdQueue, &item, 0 ) == pdPASS )
        {
			switch (item.cmd)
            {
            case CMD_PAUSE:     // просто пауза
            	running_delay(item.param, cnt_status);
                break;
			case CMD_POWER_OFF: // выключить
            	flag_power_off = 1;
                xEventGroupSetBits(xEventGroupAlrm, alarm_POWER_COMMAND_BIT);
				break;
			case CMD_STOP_ALL:
				// останавливаем ШД
            	for (uint8_t i = 0; i < 2; i++)
            	{
            		stepp[i].mode = STEP_MAN;  // перевод вручной режим
                	if ( xSemaphoreTake(xSteppMutex[i], pdMS_TO_TICKS(50)) == pdTRUE )
					{
                		if ( stepp[i].steps > 1 )
                    	{
                    		// даем корректно завершить последние такты
                        	stepp[i].steps = stepp[i].steps & 7;
						}
                    	xSemaphoreGive(xSteppMutex[i]);
					}
                	else
                	{
                		// не смогли получить мьютекс - останавливаем аварийно
						stepp_stop(0);
					}
				}
            	// останов моторов
            	set_motor_value(3, GEAR_0, GEAR_0, GEAR_0);
            	// серво в центр
            	set_servo_angle(90);
            	// гасим внешние светодиоды
            	for (uint8_t i = 0; i < 4; i++)
            	{
            		leds[i].mode = LED_OFF;
                	gpio_clear(leds[i].port, leds[i].pin);
				}
                break;
			case CMD_CNT_SET_ON_MS:
            	cnt_status = CNT_ON_MS;
                break;
			case CMD_CNT_SET_ON_STEPS:
            	cnt_status = CNT_ON_STEPS;
                break;
			case CMD_CNT_RESET:
            	ir_count = 0;
                break;
			}
		}

        taskYIELD();
    }
}


static void prvCmdMotTask(void *pvParameters)
{
    ( void ) pvParameters;

    ncommand_item item;

    for (;;)
    {
        if (flag_power_off)
        {
    		taskYIELD();
		}
                                
        // есть ли очередная команда?
        if ( xQueueReceive( xCmdMotQueue, &item, 0 ) == pdPASS )
        {
            switch (item.cmd)
            {
            case MOT_PAUSE: // просто пауза
            	running_delay(item.param, cnt_status);
                break;
			case MOT_STOP:  // останов моторов
                set_motor_value(3, GEAR_0, GEAR_0, GEAR_0);
                motors.state = MOTOR_STOPPED;
                break;
			case MOT_UP_DOWN:
				// если поворот - статус не меняется, а скоростьможет меняться
                if (motors.state == MOTOR_LEFT)
                {
                	set_motor_value(2, (int8_t)item.param, GEAR_0, (int8_t)item.param);
				}
                else if (motors.state == MOTOR_RIGHT)
                {
                    set_motor_value(1, (int8_t)item.param, (int8_t)item.param, GEAR_0);
				}
                else
                {
                    int8_t tg = GEAR_2;
                    if (item.param < 0)
                    {
                        motors.state = MOTOR_DOWN;
                        tg = -tg;
                    }
                    else
                    {
                        motors.state = MOTOR_UP;
                    }

                	// при старте с 1-й скорости, сначала включаем 2-ю скорость
					if ((motors.state == MOTOR_STOPPED) && (abs(item.param) ==  GEAR_1))
                    {
						set_motor_value(3, tg, tg, tg);
                        vTaskDelay(pdMS_TO_TICKS(MOTOR_START_TIME));
					}
                    // теперь заданную скорость
                    set_motor_value(3, (int8_t)item.param, (int8_t)item.param, (int8_t)item.param);
				}
                break;
			case MOT_LEFT:
            	switch (motors.state)
                {
                case MOTOR_LEFT:
                	break;
				case MOTOR_STOPPED:
                    motors.state = MOTOR_LEFT;
                    set_motor_value(3, GEAR_NOP, -GEAR_2, GEAR_2); // старт со 2-й скорости
                    vTaskDelay(pdMS_TO_TICKS(MOTOR_START_TIME));
                    set_motor_value(3, GEAR_NOP, -GEAR_1, GEAR_1);
					break;
                case MOTOR_RIGHT:
                    motors.state = MOTOR_STOPPED;
                	set_motor_value(3, GEAR_NOP, GEAR_0, GEAR_0); // стоп оба
                    vTaskDelay(pdMS_TO_TICKS(MOTOR_START_TIME));
					motors.state = MOTOR_LEFT;
                    set_motor_value(3, GEAR_NOP, -GEAR_2, GEAR_2); // старт со 2-й скорости
                    vTaskDelay(pdMS_TO_TICKS(MOTOR_START_TIME));
                    set_motor_value(3, GEAR_NOP, -GEAR_1, GEAR_1);
                    break;
				}
                break;
			case MOT_RIGHT:
            	switch (motors.state)
            	{
            	case MOTOR_RIGHT:
                	break;
				case MOTOR_STOPPED:
                    motors.state = MOTOR_RIGHT;
                    set_motor_value(3, GEAR_NOP, GEAR_2, -GEAR_2); // старт со 2-й скорости
                    vTaskDelay(pdMS_TO_TICKS(MOTOR_START_TIME));
					set_motor_value(3, GEAR_NOP, GEAR_1, -GEAR_1);
					break;
				case MOTOR_LEFT:
                    motors.state = MOTOR_STOPPED;
                    set_motor_value(3, GEAR_NOP, GEAR_0, GEAR_0); // стоп оба
                    vTaskDelay(pdMS_TO_TICKS(MOTOR_START_TIME));
                    motors.state = MOTOR_RIGHT;
                    set_motor_value(3, GEAR_NOP, GEAR_2, -GEAR_2); // старт со 2-й скорости
                    vTaskDelay(pdMS_TO_TICKS(MOTOR_START_TIME));
                    set_motor_value(3, GEAR_NOP, GEAR_1, -GEAR_1);
                    break;
                }
                break;
			case MOT_STRAIGHT:      // отмена поворота
            	if (motors.curr_gear == GEAR_0)
                {
                	motors.state = MOTOR_STOPPED;
                    set_motor_value(3, GEAR_NOP, GEAR_0, GEAR_0); // стоп оба
				}
                else
                {
                	set_motor_value(3, GEAR_NOP, motors.curr_gear, motors.curr_gear);
                    if (motors.curr_gear > 0)
                    {
                    	motors.state = MOTOR_UP;
                    }
                    else
                    {
                    	motors.state = MOTOR_DOWN;
					}
				}
                break;
			}
		}

        taskYIELD();
    }        
}

static void prvCmdStTask(void *pvParameters)
{
    ( void ) pvParameters;

    ncommand_item item;     // = {NO_CMD, 0, 0, CNT_OFF, 0};

    for (;;)
    {
        if (flag_power_off)
        {
			taskYIELD();
		}

        // есть ли очередная команда?
        if ( xQueueReceive( xCmdSt1Queue, &item, 0 ) == pdPASS )
        {
            procSteppCmd(0, item);
        }

        if ( xQueueReceive( xCmdSt2Queue, &item, 0 ) == pdPASS )
        {
            procSteppCmd(1, item);
        }

        taskYIELD();
    }
}

static void prvCmdServoTask(void *pvParameters)
{
    ( void ) pvParameters;

    ncommand_item item;

    for (;;)
    {
        if (flag_power_off)
        {
			taskYIELD();
		}
               // есть ли очередная команда?
        if ( xQueueReceive( xCmdServoQueue, &item, 0 ) == pdPASS )
        {
			if (item.cmd == SERVO_SET)
            {
                set_servo_angle((uint8_t)item.param);
            }
            else if (item.cmd == SERVO_PAUSE)
            {
                running_delay(item.param, cnt_status);
            }
		}

        taskYIELD();
	}
}

static void prvCmdEchoTask(void *pvParameters)
{
    ( void ) pvParameters;

    ncommand_item item;

    for (;;)
    {
        if (flag_power_off)
        {
			taskYIELD();
		}
		// есть ли очередная команда?
        if ( xQueueReceive( xCmdEchoQueue, &item, 0 ) == pdPASS )
        {
        	if (item.cmd == ECHO_SHOT)
            {
                uint16_t vv = echo_one_shot();
                if (vv > 38)
                {
            		esensor.value = 2048;
                }
                else
                {
					calc_echo_dist(echo_count);
				}
                xEventGroupSetBits(xEventGroupDev, dev_ECHO_BIT);
			}
            else if (item.cmd == ECHO_PAUSE)
            {
                running_delay(item.param, cnt_status);
            }
		}

        taskYIELD();
	}
}

static void prvCmdLedsTask(void *pvParameters)
{
    ( void ) pvParameters;

    ncommand_item item;

    for (;;)
    {
        if (flag_power_off)
        {
			taskYIELD();
		}
        // есть ли очередная команда?
        if ( xQueueReceive( xCmdLedsQueue, &item, 0 ) == pdPASS )
        {
        	switch (item.param)
            {
            case LED0_MOD:
            case LED1_MOD:
            case LED2_MOD:
            case LED3_MOD:
            	set_led((uint8_t)item.param, item.cmd);
                break;
			case LEDS_PAUSE:
            	running_delay(item.param, cnt_status);
				break;
        	}
		}

        taskYIELD();
	}
}


void procSteppCmd(uint8_t stnum, ncommand_item cmd)
{
	switch (cmd.cmd)
    {
    case ST_PAUSE:  // просто пауза
    	running_delay(cmd.param, cnt_status);
        break;
	case ST_CONT_STOP:      // перевод из STEP_MAN_CONT в STEP_MAN
    	stepp[stnum].mode = STEP_MAN;
        stepp[stnum].steps = stepp[stnum].steps & 7;
        // ждем завершения движения
        while ( (stepp[stnum].steps > 0) || stepp[stnum].last_step )
        {
			vTaskDelay(pdMS_TO_TICKS(10));
		}
        break;    
	case ST_SET_ANGLE:        // поворот ШД в ручном режиме, переводит в STEP_MAN
		stepp[stnum].mode = STEP_MAN;
        stepp[stnum].steps = stepp[stnum].steps & 7;
        // ждем завершения движения
        while ( (stepp[stnum].steps > 0) || stepp[stnum].last_step )
        {
        	vTaskDelay(pdMS_TO_TICKS(10));
		}
        // <угол поворота> = <новое значение> - <текущее значение>
        uint16_t a = cmd.param - stepp[stnum].angle_curr;
        if ( xSemaphoreTake(xSteppMutex[stnum], pdMS_TO_TICKS(200)))
        {
        	stepp_clk_cclk(stnum, a);
            xSemaphoreGive( xSteppMutex[stnum] );
		}
        break;
	case ST_CONT_CCLK:
    case ST_CONT_CLK:
        stepp[stnum].mode = STEP_MAN;
        stepp_start_cont(stnum, cmd.cmd);
        break;
    case ST_NULL:           // возврат в 0 и перевод в ручной режим
	case ST_NULL_FAST:      //   со сбросом оборотов
    	stepp[stnum].mode = STEP_MAN;
        stepp[stnum].steps = stepp[stnum].steps & 7;
        // ждем завершения движения
        while ( (stepp[stnum].steps > 0) || stepp[stnum].last_step )
        {
        	vTaskDelay(pdMS_TO_TICKS(10));
		}
        if (stepp[stnum].angle_curr != 0)
        {
        	uint8_t fast = 0;
            if (cmd.cmd == ST_NULL_FAST)
            {
            	fast = 1;
			}
            stepp_to_null(stnum, fast);
		}
        break;
	}
}

uint8_t sendPackToBLE(char * blepack)
{
	if ( xSemaphoreTake(xSendPackMutex, pdMS_TO_TICKS(500)))
    {
    	//send_usart_dma(blepack);
    	dma_write(blepack, strlen(blepack));
        xSemaphoreGive(xSendPackMutex);
        return 1;
	}
    else
    {
    	return 0;
	}
}

// проверка достаточности места в очереди статусов - если нет - возврат
#define IF_STQ_AVAIL(v)     { \
                                if(uxQueueSpacesAvailable(xStateQueue) < v) \
                                    return; \
                            }
// проверка состояний очередей и устройств, помещение статусов в очередь
// через вызов push_state()                            
void check_states(void)
{
    IF_STQ_AVAIL(1);
	// проверка изменения очередей, отправка количества ожидающих
	//  исполнения команд в очередях при их изменении uxQueueSpacesAvailable(xStateQueue)
    uint16_t tmp = uxQueueSpacesAvailable(xCmdQueue);
    if (tmp != remainCmdQueue)
    {
    	push_state(STATE_PACK_QUEUE_CMD, (uint8_t)tmp);
       	remainCmdQueue = tmp;
	}
    IF_STQ_AVAIL(1);
	tmp = uxQueueSpacesAvailable(xCmdMotQueue);
    if (tmp != remainCmdMotQueue)
    {
		push_state(STATE_PACK_QUEUE_MOT, (uint8_t)tmp);
       	remainCmdMotQueue = tmp;
	}
    IF_STQ_AVAIL(1);
	tmp = uxQueueSpacesAvailable(xCmdSt1Queue);
	if (tmp != remainCmdSt1Queue)
	{
    	push_state(STATE_PACK_QUEUE_ST1, (uint8_t)tmp);
       	remainCmdSt1Queue = tmp;
	}
    IF_STQ_AVAIL(1);
	tmp = uxQueueSpacesAvailable(xCmdSt2Queue);
    if (tmp != remainCmdSt2Queue)
    {
    	push_state(STATE_PACK_QUEUE_ST2, (uint8_t)tmp);
		remainCmdSt2Queue = tmp;
	}
    IF_STQ_AVAIL(1);
	tmp = uxQueueSpacesAvailable(xCmdServoQueue);
    if (tmp != remainCmdServoQueue)
    {
    	push_state(STATE_PACK_QUEUE_SERVO, (uint8_t)tmp);
       	remainCmdServoQueue = tmp;
	}
    IF_STQ_AVAIL(1);
	tmp = uxQueueSpacesAvailable(xCmdEchoQueue);
    if (tmp != remainCmdEchoQueue)
    {
    	push_state(STATE_PACK_QUEUE_ECHO, (uint8_t)tmp);
       	remainCmdEchoQueue = tmp;
	}
    IF_STQ_AVAIL(1);
	tmp = uxQueueSpacesAvailable(xCmdLedsQueue);
	if (tmp != remainCmdLedsQueue)
	{
    	push_state(STATE_PACK_QUEUE_LEDS, (uint8_t)tmp);
       	remainCmdLedsQueue = tmp;
	}

    IF_STQ_AVAIL(1);

   	// проверка изменения состояния устройств, 
   	//  при наличии изменений - отправка их состояния
	EventBits_t uxBits = xEventGroupWaitBits(
		xEventGroupDev, 0x1FFF,	// all
		pdFALSE,             	// не очищаем флаг
        pdFALSE,    			// Don't wait for all bits.
        0 );                	// ticks to wait

    if ( (uxBits & dev_MOTORS_BIT) != 0 )
    {
    	push_state(STATE_PACK_MOTORS, 0);
   		xEventGroupClearBits(xEventGroupDev, dev_MOTORS_BIT);
	}

    if ( (uxBits & dev_STEPP1_BIT) != 0 )
	{
        IF_STQ_AVAIL(2);
		push_state(STATE_PACK_STEPP, 0);
   		xEventGroupClearBits(xEventGroupDev, dev_STEPP1_BIT);
	}
    if ( (uxBits & dev_STEPP2_BIT) != 0 )
	{
        IF_STQ_AVAIL(2);
		push_state(STATE_PACK_STEPP, 1);
   		xEventGroupClearBits(xEventGroupDev, dev_STEPP2_BIT);
	}

	EventBits_t evTmp = dev_LED0_BIT;
	for (int8_t i = 0; i < 4; i++)
	{
    	if ( (uxBits & evTmp) != 0 )
		{
            IF_STQ_AVAIL(1);
			push_state(STATE_PACK_LED, i);
			xEventGroupClearBits(xEventGroupDev, evTmp);
		}
        evTmp = evTmp << 1; // dev_LED1_BIT .. dev_LED3_BIT
	}
	if ( (uxBits & dev_SERVO_BIT) != 0 )
	{
        IF_STQ_AVAIL(1);
		push_state(STATE_PACK_SERVO, 0);
   		xEventGroupClearBits(xEventGroupDev, dev_SERVO_BIT);
	}
	if ( (uxBits & dev_ECHO_BIT) != 0 )
	{
        IF_STQ_AVAIL(1);
		push_state(STATE_PACK_ECHO, 0);
   		xEventGroupClearBits(xEventGroupDev, dev_ECHO_BIT);
	}
	
	evTmp = dev_ADC0_BIT;
	for (uint8_t i = 0; i < 4; i++)
	{
    	if ( (uxBits & evTmp) != 0 )
		{
            IF_STQ_AVAIL(1);
			push_state(STATE_PACK_ADC, i);
   	    	xEventGroupClearBits(xEventGroupDev, evTmp);
		}
        evTmp = evTmp << 1; // dev_ADC1_BIT .. dev_ADC3_BIT
	}
}

// формирование JSON-пакетов статусов, 
// помещение их в очередь xStateQueue для отправки на пульт
bool push_state(uint8_t mstate, uint8_t num)
{
    static char pack[20];

    bool ret = true;
    strcpy(pack, js_begin);
    
    if ( (mstate == STATE_PACK_READY) || (mstate == STATE_PACK_BUSY) )
    {
    	strcat(pack, js_ready);
        strcat(pack, js_delim);
        if (mstate == STATE_PACK_READY)
        {
        	strcat(pack, js_true);
		}
        else
        {
        	strcat(pack, js_false);
		}
	}
    else if (mstate == STATE_PACK_DBG)
    {
    	strcat(pack, js_dbg);
        strcat(pack, js_delim);
        strcat(pack, "\"");
        strcat(pack, dbg_buff);
        strcat(pack, "\"");
	}
    else if ( (mstate == STATE_PACK_PWROFF_BAT) ||
    		(mstate == STATE_PACK_PWROFF_KEY) ||
			(mstate == STATE_PACK_PWROFF_CMD) )
	{
    	strcat(pack, js_pwroff);
        strcat(pack, js_delim);
        if (mstate == STATE_PACK_PWROFF_BAT)
        {
        	strcat(pack, "\"b\"");  // по разряду батареи
		}
        else if (mstate == STATE_PACK_PWROFF_CMD)
        {
        	strcat(pack, "\"c\"");  // по команде от пульта
		}
        else
        {
        	strcat(pack, "\"k\"");  // по нажатию кнопки
		}
	}
    else if ( (mstate == STATE_PACK_LOWBAT) || (mstate == STATE_PACK_OVERLOAD) )
	{
    	strcat(pack, js_batt);
        strcat(pack, js_delim);
        if (mstate == STATE_PACK_OVERLOAD)
        {
        	strcat(pack, js_l);
		}
        else
		{
        	strcat(pack, js_b);
		}
	}
    else if ( mstate == STATE_PACK_MOTORS )
    {
    	strcat(pack, js_ms);
        strcat(pack, js_delim);
        strcat(pack, js_lbr);                   // [
        switch (motors.state)                   // статус
        {
        case MOTOR_STOPPED:
        	strcat(pack, js_s);
            break;
		case MOTOR_UP:
        	strcat(pack, js_f);
            break;
		case MOTOR_DOWN:
        	strcat(pack, js_b);
            break;
		case MOTOR_LEFT:
        	strcat(pack, js_l);
            break;
		case MOTOR_RIGHT:
        	strcat(pack, js_r);
            break;
		default:
        	strcat(pack, js_a); // MOTOR_ALARM
		}
        strcat(pack, js_coma);
        // отрицательные значения скоростей передаются 
        //  положительными значениями: abs(v) + 4
        int8_t tv = motors.curr_gear;
        if (tv < 0)
        {
            tv = abs(tv) + 4;
        }
        strcat(pack, itoa(tv, 10));  // заданная скорость
        strcat(pack, js_coma);      // ,
        tv = motors.gear1;
        if (tv < 0)
        {
            tv = abs(tv) + 4;
        }
        strcat(pack, itoa(tv, 10));     // скорость ДПТ1
        strcat(pack, js_coma);                     // ,
        tv = motors.gear2;
        if (tv < 0)
        {
            tv = abs(tv) + 4;
        }
        strcat(pack, itoa(tv, 10));     // скорость ДПТ2
        strcat(pack, js_rbr);                      // ]
	}
    else if ( mstate == STATE_PACK_STEPP)
    {
    	if ( uxQueueSpacesAvailable(xStateQueue) < 2 )
        {
        	ret = false;    // статус ШД - 2 пакета
		}
        else
        {
        	char stn[2];
            if (num == 0)
            {
            	stn[0] = '0';                   // ШД0
			}
            else
            {
            	stn[0] = '1';                   // ШД1
			}
            stn[1] = '\0';
            strcat(pack, js_ss);
			strcat(pack, js_delim);
            strcat(pack, js_lbr);                   // [
            strcat(pack, stn);                              // 0/1
            strcat(pack, js_coma);                  // ,
            if (stepp[num].moving == 0)                     // остановлен/в движении
			{
            	strcat(pack, js_s);
			}
            else
			{
            	strcat(pack, js_v);
			}
            strcat(pack, js_coma);                  // ,
            switch (stepp[num].mode)
            {
            case STEP_MAN:
            	strcat(pack, js_m);
                break;
			case STEP_MAN_CONT:
            	strcat(pack, js_c);
                break;
			default:
            	strcat(pack, js_n);
			}
            strcat(pack, js_rbr);                   // ]
            strcat(pack, js_end);
            xQueueSend(xStateQueue, pack, pdMS_TO_TICKS(20));
                        
            strcpy(pack, js_begin);
            strcat(pack, js_sv);
            strcat(pack, js_delim);
            strcat(pack, js_lbr);                   // [
            strcat(pack, stn);                              // 0/1
            strcat(pack, js_coma);                  // ,
            strcat(pack, itoa((int16_t)stepp[num].turns, 10));
            strcat(pack, js_coma);                  // ,
            strcat(pack, itoa((int16_t)stepp[num].angle_curr, 10));
            strcat(pack, js_rbr);                   // ]
		}
	}
    else if ( mstate == STATE_PACK_LED)
    {
    	strcat(pack, js_led);
        strcat(pack, js_delim);
        strcat(pack, js_lbr);                   // [
        //itoa(num, st, 10);
        strcat(pack, itoa(num, 10));                            // 0..3
        strcat(pack, js_coma);                  // ,
        switch (leds[num].mode)
        {
        case LED_ON:
        	strcat(pack, js_u);
            break;
		case LED_BLINK:
        	strcat(pack, js_b);
            break;
		default:
        	strcat(pack, js_d);
		}
        strcat(pack, js_rbr);                   // ]
	}
	else if ( mstate == STATE_PACK_ADC)
    {       
    	strcat(pack, js_adc);
        strcat(pack, js_delim);
        strcat(pack, js_lbr);                   // [
        strcat(pack, itoa(num, 10));                            // 0..3
        strcat(pack, js_coma);                  // ,
        strcat(pack, itoa(adcval[num], 10));
        // adc value
        strcat(pack, js_rbr);                   // ]
	}
    else if ( mstate == STATE_PACK_ECHO)
    {
    	strcat(pack, js_echo);
        strcat(pack, js_delim);
        strcat(pack, itoa(esensor.value, 10));		// echo value
    }
    else if ( mstate == STATE_PACK_SERVO)
	{
    	strcat(pack, js_servo);
        strcat(pack, js_delim);
		strcat(pack, itoa(servo.angle, 10));
	}
    else if ( mstate == STATE_PACK_QUEUE_CMD)
    {
    	strcat(pack, js_queue);
        strcat(pack, js_delim);
        strcat(pack, itoa(num, 10));                            // наличие команд в очереди
	}
    else if ( mstate == STATE_PACK_QUEUE_MOT)
    {
    	strcat(pack, js_qmot);
        strcat(pack, js_delim);
        strcat(pack, itoa(num, 10));                            // наличие команд в очереди
	}
    else if ( mstate == STATE_PACK_QUEUE_ST1)
    {
    	strcat(pack, js_qst1);
        strcat(pack, js_delim);
        strcat(pack, itoa(num, 10));                            // наличие команд в очереди
	}
    else if ( mstate == STATE_PACK_QUEUE_ST2)
    {
    	strcat(pack, js_qst2);
        strcat(pack, js_delim);
        strcat(pack, itoa(num, 10));                            // наличие команд в очереди
	}
    else if ( mstate == STATE_PACK_QUEUE_SERVO)
    {
    	strcat(pack, js_qservo);
        strcat(pack, js_delim);
        strcat(pack, itoa(num, 10));                            // наличие команд в очереди
	}
    else if ( mstate == STATE_PACK_QUEUE_ECHO)
    {
    	strcat(pack, js_qecho);
        strcat(pack, js_delim);
        strcat(pack, itoa(num, 10));                            // наличие команд в очереди
	}
    else if ( mstate == STATE_PACK_QUEUE_LEDS)
    {
    	strcat(pack, js_qleds);
        strcat(pack, js_delim);
        strcat(pack, itoa(num, 10));                            // наличие команд в очереди
	}
    else
    {
    	ret = false;
	}
                
    if (ret)
    {       
    	strcat(pack, js_end);
        if (xQueueSend(xStateQueue, pack, 0) != pdPASS)
        {
            ret = false;
        }
	}
    return ret;
}

void put_to_cmd_queue(uint16_t cmd, int16_t iparam)
{
	ncommand_item item;
             
    item.cmd = cmd;
    item.param = iparam;
  
    uint8_t cc = 5;
    while ( (uxQueueSpacesAvailable(xCmdQueue) == 0) && (cc-- > 0) ) // ждем до 100mS
    {
    	vTaskDelay(pdMS_TO_TICKS(20));
	}
    if (cc > 0)
    {
    	xQueueSend(xCmdQueue, &item, pdMS_TO_TICKS(50));
	}
}

void put_motors_cmd(char command, int16_t iparam)
{
	ncommand_item item;

    item.param = iparam;

    switch (command)
    {       
    case 'f':
    	item.cmd = MOT_UP_DOWN;
        break;
	case 'b':
    	item.cmd = MOT_UP_DOWN;
        item.param = -item.param;
        break;
	case 'r':
    	item.cmd = MOT_RIGHT;
        break;
	case 'l':
    	item.cmd = MOT_LEFT;
        break;
	case 'n':
    	item.cmd = MOT_STRAIGHT;
        break;
	case 'p':
    	item.cmd = MOT_PAUSE;
        break;
	default:        //case 's':
    	item.cmd = MOT_STOP;
	}
                
	uint8_t cc = 5;
    while ( (uxQueueSpacesAvailable(xCmdMotQueue) == 0) && (cc-- > 0) ) // ждем до 100mS
    {
    	vTaskDelay(pdMS_TO_TICKS(20));
	}
    if (cc > 0)
    {
    	xQueueSend(xCmdMotQueue, &item, pdMS_TO_TICKS(50));
	}
}

void put_stepp_cmd(uint8_t step_num, char command, int16_t iparam)
{
	ncommand_item item;

    item.param = iparam;
    switch (command)
    {
    case 'a': // новое значение угла
    	if (iparam == 0)
        {
        	item.cmd = ST_NULL;
		}
        else
        {
        	item.cmd = ST_SET_ANGLE;
		}
        break;
	case 'h':       // в исх. положение без сброса оборотов
        item.cmd = ST_NULL;
        break;
	case 'n':       // в исх. положение со сбросом оборотов
        item.cmd = ST_NULL_FAST;
        break;
	case 'r':       // вращать по часовой стрелке
        item.cmd = ST_CONT_CLK;
        break;
    case 'l':       // вращать против часовой стрелки
        item.cmd = ST_CONT_CCLK;
        break;
    case 'p':
    	item.cmd = ST_PAUSE;
        break;
	default:        // 's' - stop
        item.cmd = ST_CONT_STOP;
	}
        
    QueueHandle_t xQueue;
    if (step_num == 0)
    {
    	xQueue = xCmdSt1Queue;
	}
    else
    {
    	xQueue = xCmdSt1Queue;
	}
                
    uint8_t cc = 5;
    while ( (uxQueueSpacesAvailable(xQueue) == 0) && (cc-- > 0) ) // ждем до 100mS
    {
    	vTaskDelay(pdMS_TO_TICKS(20));
	}
    if (cc > 0)
    {
    	xQueueSend(xQueue, &item, pdMS_TO_TICKS(50));
	}
}

void put_leds_cmd(char command, uint16_t iparam)
{
	ncommand_item item;

    if (command == 'p')     // PAUSE
    {
    	item.param = iparam;
	}
    else
    {
    	item.param = iparam & 0x03; // LED0_MOD..LED3_MOD
	}
                
    switch (command)
    {
    case 'p':
    	item.cmd = LEDS_PAUSE;
        break;
	case 'u':
    	item.cmd = LED_ON;
        break;
	case 'b':
    	item.cmd = LED_BLINK;
        break;
	default:        //case 'd':
    	item.cmd = LED_OFF;
	}
                
    uint8_t cc = 5;
    while ( (uxQueueSpacesAvailable(xCmdLedsQueue) == 0) && (cc-- > 0) ) //ждем до 100mS
    {
    	vTaskDelay(pdMS_TO_TICKS(20));
	}
    if (cc > 0)
    {
    	xQueueSend(xCmdLedsQueue, &item, pdMS_TO_TICKS(50));
	}
}

void put_servo_cmd(char command, int16_t iparam)
{
	ncommand_item item;

    if (command == 'p')     // PAUSE
    {
    	item.cmd = SERVO_PAUSE;
        item.param = iparam;
	}
    else
    {
    	item.cmd = SERVO_SET;
        if ( (iparam < SERVO_0GRAD) || (iparam < SERVO_180GRAD) )
        {
        	item.param = SERVO_90GRAD;
		}
        else
        {
        	item.param = abs(iparam); // SET
		}
	}
    uint8_t cc = 5;
    while ( (uxQueueSpacesAvailable(xCmdServoQueue) == 0) && (cc-- > 0) ) // ждем до 100mS
	{
    	vTaskDelay(pdMS_TO_TICKS(20));
	}
    if (cc > 0)
    {
    	xQueueSend(xCmdServoQueue, &item, pdMS_TO_TICKS(50));
	}
}

void put_echo_cmd(char command, int16_t iparam)
{
    ncommand_item item;

    if (command == 'd')     // измерение и пауза (если есть)
    {
    	item.cmd = ECHO_SHOT;
	}
    else // 'p'
    {
    	item.cmd = ECHO_PAUSE;
	}

    item.param = iparam;

    uint8_t cc = 5;
    while ( (uxQueueSpacesAvailable(xCmdEchoQueue) == 0) && (cc-- > 0) ) //ждем до 100mS
	{
    	vTaskDelay(pdMS_TO_TICKS(20));
	}
    if (cc > 0)
    {
    	xQueueSend(xCmdEchoQueue, &item, pdMS_TO_TICKS(50));
	}
}

void calc_echo_dist(uint16_t count)
{
    esensor.value = count / 58;     // расстояние в см
    if (esensor.value > 400)        // больше 4-х метров - далеко
    {
        esensor.value = 2048;
    }
    xEventGroupSetBits(xEventGroupDev, dev_ECHO_BIT);
}

void echo_pulse(void)
{
    timer_set_period(TIM4, 11);
    timer_set_oc_value(TIM4, TIM_OC4, 2);
    timer_enable_counter(TIM4);
}

TickType_t echo_one_shot(void)
{
    echo_pulse();
                        
    // засекаем время
    TickType_t xLastWakeTime1 = xTaskGetTickCount();
    // ждем эхо до 38 мС + 2
    xSemaphoreTake(xSemaphHandleEcho, pdMS_TO_TICKS(40)); // 38
    // засекаем время
    TickType_t xLastWakeTime2 = xTaskGetTickCount();
                        
    // на случай, если нет импульса от эхо-сенсора
    timer_disable_counter(TIM1);
                
    // за какое время получили отклик?
    return (xLastWakeTime2 - xLastWakeTime1);
}

// извлечение полей пакета в jparams
uint8_t get_fields(char *fld)
{
    uint8_t cnt = 0;
    uint8_t multi = 1;

    char *pb = strchr(fld, '[');
    char *pe = strchr(fld, ']');
    if ( (pb == NULL) || (pe == NULL) )
    {
        multi = 0;
    }

    if (multi) // > 1 параметра
    {
        *pe = '\0'; // укорачиваем строку - убираем "]"
        char *token = strtok((pb + 1), ",");
        if (token != NULL)
        {
            cnt = 1;
            strcpy(jparams[0], token);
            token = strtok(NULL, ",");
            if (token != NULL)
            {
                cnt = 2;
                strcpy(jparams[1], token);
                token = strtok(NULL, ",");
                if (token != NULL)
                {
                    cnt = 3;
                    strcpy(jparams[2], token);
                }
                else
                {
                    jparams[2][0] = '\0';   
                }
            }
            else
            {
                jparams[1][0] = '\0';   
            }
        }
        else
        {
            jparams[0][0] = '\0';
        }
    }
    else    // single
    {
        strcpy(jparams[0], fld);
        jparams[1][0] = '\0';
        jparams[2][0] = '\0';
        cnt = 1;
    }

    return cnt; // количество извлеченных  параметров
}

// вычисление основных параметров по результатам цикла АЦП
void ADC_calc(void)
{
    xEventGroupClearBits(xEventGroupDev, (const EventBits_t)0x0F); // dev_ADC0..3_BIT

    // 1-е измерение по каждому каналу отбрасываем, 2-е и 3-е усредняются
    uint16_t v = (ADCbuffer[1] + ADCbuffer[2]) >> 1;    // 3v3
    if (abs(adcval[0] - v) > 7)
    {
    	xEventGroupSetBits(xEventGroupDev, dev_ADC0_BIT);
        adcval[0] = v;
	}

    v = (ADCbuffer[10] + ADCbuffer[11]) >> 1; // 6v
    if (abs(adcval[1] - v) > 7)
    {
    	xEventGroupSetBits(xEventGroupDev, dev_ADC1_BIT);
        adcval[1] = v;
	}

    if (adcval[1] < adcval[0])  // load
    {
    	adcval[4] = adcval[1] - adcval[0];
	}
    else
    {
    	adcval[4] = 0;
	}

    v = (ADCbuffer[7] + ADCbuffer[8]) >> 1; // P5
    if (abs(adcval[2] - v) > 7)
    {
    	xEventGroupSetBits(xEventGroupDev, dev_ADC2_BIT);
        adcval[2] = v;
	}

    v = (ADCbuffer[4] + ADCbuffer[5]) >> 1; // P6
    if (abs(adcval[2] - v) > 7)
    {
    	xEventGroupSetBits(xEventGroupDev, dev_ADC3_BIT);
        adcval[3] = v;
	}
        
    xEventGroupClearBits(xEventGroupAlrm, (const EventBits_t)0xFF);

    if (!init_process)
    {
    	if ( adcval[0] < BATT_LOW_LEVEL )
    	{
        	// батарея контроллера разряжена
            if ( adcval[0] < BATT_OFF_LEVEL )
            {
            	// требуется аварийное отключение
                xEventGroupSetBits( xEventGroupAlrm, alarm_CONTRL_OFF_VOLTAGE_BIT);
			}
            xEventGroupSetBits( xEventGroupAlrm, alarm_CONTRL_LOW_VOLTAGE_BIT);
		}

        if ( adcval[4] > LOAD_MAX )
        {
            // слишком большой ток
            xEventGroupSetBits( xEventGroupAlrm, alarm_OVERLOAD_BIT);
        }
    }	
}	

// обеспечение паузы в mS или импульсах датчика оборотов колеса
void running_delay(uint32_t lasting, uint8_t c_status)
{
	uint32_t st_new;
    int32_t t_max;
                
    if ( c_status == CNT_ON_STEPS )
    {
    	st_new = ir_count + lasting;
        t_max = (int32_t)(lasting * MS_PER_STEP); // максимальное время в mS - чтобы не зависнуть
        // цикл до получеиия нужного количества шагов или до истечения времени
        while ((ir_count < st_new) && (t_max > 0))
        {
        	vTaskDelay(pdMS_TO_TICKS(20));
            t_max -= 20;
		}
	}
    else    // CNT_OFF или CNT_ON_MS
    {
    	vTaskDelay(pdMS_TO_TICKS(lasting));
	}
}

// в стандарте C99 этой функции нет, а sprinf слишком тяжелая
char * itoa(int val, int base) {
    static char buf[32] = {0};

    if (val == 0) {
        buf[0] = '0';
        buf[1] = '\0';
        return &buf[0];
    }
    else {
        int i = 30;

        for(; val && i ; --i, val /= base)
        {
            buf[i] = "0123456789abcdef"[val % base];
        }
        return &buf[i+1];
    }
}

/* ======================== SETUP ========================= */

static void prvSetupHardware( void )
{
    clock_setup();
    systickSetup();
    gpio_setup();
    usart_setup();
    tim1_setup();
    tim2_setup();
    tim3_setup();
    tim4_setup();
    adc_setup();
}

static void clock_setup(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_DMA1);
}

static void systickSetup()
{
    /* 72MHz / 8 => 9,000,000 counts per second */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

    /* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
    /* SysTick interrupt every N clock pulses: set reload to N-1 */
    systick_set_reload(8999);

    systick_interrupt_enable();

    /* Start counting. */
    systick_counter_enable();
}

/*----------------------------- GPIO ------------------------*/

static void gpio_setup(void)
{    
    /* Освобождаем ножки PA15, PB3, PB4. Оставляем для отладки
       только SWDIO и SWCLK - для ST-Link V2 вполне достаточно. 
      JTAG-DP Disabled and SW-DP Enabled: */
    AFIO_MAPR &= ~AFIO_MAPR_SWJ_MASK;               // 0 to 24..26
    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;  // (0x2 << 24) - 1 to 25

    // BOARD LED
    gpio_clear(GPIOC, GPIO13);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    
    // пины GPIOB
    gpio_clear(GPIOB, GPIO10|GPIO11|GPIO12|GPIO13|GPIO14|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7|GPIO8);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
        GPIO10|GPIO11|GPIO12|GPIO13|GPIO14|GPIO3|GPIO4|GPIO5|GPIO6|GPIO7|GPIO8);

    // заранее обнуляем A11, A15
    gpio_clear(GPIOA, GPIO11|GPIO15);
    // заранее ставим высокий уровень на PA8 - при 0 - выключается питание
    // PA8 должен быть GPIO_CNF_OUTPUT_OPENDRAIN, иначе управление питанием
    // не будет работать корректно
    gpio_set(POWER_HOLD_PORT, POWER_HOLD_PIN);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, GPIO8);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO11|GPIO15);
    // дублируем на всякий случай
    gpio_set(POWER_HOLD_PORT, POWER_HOLD_PIN);

    // входы АЦП
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO3);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO4);

    // от ножки 25(PIO2) HM-10 - индикация установленного соединения
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO12);

    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
    nvic_enable_irq(NVIC_EXTI15_10_IRQ);

    // от эхо-сенсора
    exti_select_source(EXTI5, GPIOA);
    exti_set_trigger(EXTI5, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI5);

    // от датчика пути
    exti_select_source(EXTI15, GPIOB);
    exti_set_trigger(EXTI15, EXTI_TRIGGER_BOTH);
    exti_enable_request(EXTI15);

    /* по нажатию кнопки питания - эту функцию теперь выполняет EXTI8
    exti_select_source(EXTI12, GPIOA);
    exti_set_trigger(EXTI12, EXTI_TRIGGER_BOTH);
    exti_enable_request(EXTI12);
    */

    // эксперимент: получилось организовать прерывание по просадке выхода PA8
    // при нажатии кнопки питания и исключить EXTI12 (снять перемычку)
    exti_select_source(EXTI8, GPIOA);
    exti_set_trigger(EXTI8, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI8);
}

void exti9_5_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // эксперимент: попытаться получить прерывание по выходу PA8
    // по нажатию кнопки питания
    if (exti_get_flag_status(EXTI8) != 0)
    {
        exti_reset_request(EXTI8);
        // нажата кнопка питания
        xEventGroupSetBits( xEventGroupAlrm, alarm_POWER_BUTTON_BIT);
    }
        
    // прерывание по счетчику пути
    if (exti_get_flag_status(EXTI5) != 0)
    {
        exti_reset_request(EXTI5);
        if ( xSemaphoreTakeFromISR(xIRCountMutex, &xHigherPriorityTaskWoken) ==pdTRUE )
        {
            // если не удастся захватить счетчик - не страшно,  т.к.  это может
            //  быть только для сброса - в момент сброса 1 импульс может быть потерян
            ir_count++;
            xSemaphoreGiveFromISR(xIRCountMutex, &xHigherPriorityTaskWoken );
        }
    }
        
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}       

void exti15_10_isr(void)
{   
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    /* эту функцию теперь выполняет EXTI8
    if (exti_get_flag_status(EXTI12) != 0)
    {
        exti_reset_request(EXTI12);
        if ( gpio_get(POWEROFF_PORT, POWEROFF_PIN) == 0 )
        {
            // нажата кнопка питания
            xEventGroupSetBits( xEventGroupAlrm, alarm_POWER_BUTTON_BIT);
        }
    }
    */    
    if (exti_get_flag_status(EXTI15) != 0)
    {
        exti_reset_request(EXTI15);
        if (gpio_get(ECHO_PORT, ECHO_ECHO_PIN) != 0)
        {
            // начинаем отсчет появления эхо-сигнала от эхо-сенсора
            timer_set_counter(TIM1, 0);
            timer_enable_counter(TIM1);
            echo_count = 0;
        }
        else
        {
            // пришел эхо-сигнал
            echo_count = timer_get_counter(TIM1);
            timer_disable_counter(TIM1);
            xSemaphoreGiveFromISR(xSemaphHandleEcho, &xHigherPriorityTaskWoken);
        }
    }
        
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}       

/*---------------------------- ADC ------------------------*/

static void dma_adc_init(void)
{
    dma_channel_reset(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC1_DR);
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)ADCbuffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 12);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_MEDIUM);
    dma_enable_channel(DMA1, DMA_CHANNEL1);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL1);	
}

uint8_t channel_array[] = { 0, 0, 0, 1, 1, 1, 3, 3, 3, 4, 4, 4};
static void adc_setup(void)
{
    nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    adc_power_off(ADC1);

    dma_adc_init();

    //rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
    //rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6); // 12MHz
    adc_set_dual_mode(ADC_CR1_DUALMOD_IND);
    adc_set_right_aligned(ADC1);
    adc_enable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);

    adc_disable_external_trigger_regular(ADC1);
    adc_enable_dma(ADC1);
    
    adc_set_regular_sequence(ADC1, 12, channel_array);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);
    
    adc_power_on(ADC1);	// пауза на включение - при старте prvADCTask
}

// ADC scan comlete
void dma1_channel1_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((DMA1_ISR & DMA_IFCR_CTCIF1) != 0) // Tranfer complete flag
    {
        DMA1_IFCR |= DMA_IFCR_CTCIF1;

        // сканирование каналов закончено, устанавливаем семафор
        xSemaphoreGiveFromISR(xSemaphVoltage, &xHigherPriorityTaskWoken);
    }
    
    if ((DMA1_ISR & DMA_IFCR_CTEIF1) != 0) // Tranfer error flag
    {
        DMA1_IFCR |= DMA_IFCR_CTEIF1;
    }
    
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*---------------------------- USART1 ------------------------*/

// USART1 - 115200, no flow control, rx - interrupt, tx - dma
static void usart_setup(void)
{
    nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
    nvic_enable_irq(NVIC_USART1_IRQ);
    
	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_enable_rx_interrupt(USART1);
	usart_disable_tx_interrupt(USART1);
	/* Finally enable the USART. */
	usart_enable(USART1);
}

// USART1 RX interrupt
void usart1_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint8_t data = 'A';

    /* Check if we were called because of RXNE. */
    if (((USART_CR1(USART1) & USART_FLAG_RXNE) != 0) &&
            ((USART_SR(USART1) & USART_FLAG_RXNE) != 0))
    {
        /* Retrieve the data from the peripheral. */
        data = usart_recv(USART1);
        if ( xQueueSendFromISR( xRxedChars, &data, &xHigherPriorityTaskWoken ) != pdTRUE )
        {
            RxOverflow = 1;
        }
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

// Using channel 4 for USART1 TX
static void dma_write(char *data, int size)
{    
    //Reset DMA channel
    dma_channel_reset(DMA1, DMA_CHANNEL4);

    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART1_DR);
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)data);
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, size);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_HIGH);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

    dma_enable_channel(DMA1, DMA_CHANNEL4);

    usart_enable_tx_dma(USART1);
}

// USART1 tranfer complete
void dma1_channel4_isr(void)
{
    if ((DMA1_ISR &DMA_ISR_TCIF4) != 0)
    {
        DMA1_IFCR |= DMA_IFCR_CTCIF4;
    }
    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    usart_disable_tx_dma(USART1);
    dma_disable_channel(DMA1, DMA_CHANNEL4);
}

/*---------------------------- TIMERS ------------------------*/

/* Таймер 1 - для измерения эхо-сигнала от локатора.
  * запускается по переднему
  *  фронту внешнего прерывания,
  * останавливается по заднему фронту
 */
static void tim1_setup(void) {
    rcc_periph_clock_enable(RCC_TIM1);

    /* Reset TIM1 peripheral to defaults. */
    rcc_periph_reset_pulse(RST_TIM1);
        /* Timer global mode:
         * - No divider
         * - Alignment edge
         * - Direction up
         * (These are actually default values after reset above, so this call
         * is strictly unnecessary, but demos the api for alternative settings)
         */
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, (rcc_apb1_frequency / 1000000 - 1));
    timer_disable_preload(TIM1);
    timer_set_period(TIM1, 60000);  // ARR
}

/* Таймер 2 - PWM для управления серво-приводом
  * период - 20 мС
  * длительность импульса - от 1 до 2-х мС, 1.5 - среднее положение
*/
static void tim2_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_reset_pulse(RST_TIM2);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM2_CH3);

    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2, (4000 - 1));
    timer_set_period(TIM2, 360 -1);  // ARR
    timer_disable_preload(TIM2);
    timer_enable_oc_preload(TIM2, TIM_OC3);
    timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);

    timer_set_oc_polarity_high(TIM2, TIM_OC3);
    timer_set_oc_idle_state_set(TIM2, TIM_OC3);
    timer_set_oc_slow_mode(TIM2, TIM_OC3);
    timer_set_oc_value(TIM2, TIM_OC3, SERVO_90GRAD);

    timer_enable_oc_output(TIM2, TIM_OC3);
    timer_enable_counter(TIM2);
}

/* Таймер 3 - ШИМ - моторы 1,2 */
static void tim3_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_reset_pulse(RST_TIM3);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM3_CH1 | GPIO_TIM3_CH2); // PA6, PA7
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM3_CH3 | GPIO_TIM3_CH4); // PB0, PB1

    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM3, (3 - 1));
    timer_set_period(TIM3, 512 - 1);    // ARR
    timer_enable_preload(TIM3);         // ширина импульса изменится со след. периода

    timer_enable_oc_preload(TIM3, TIM_OC1);
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM3, TIM_OC1);
    timer_set_oc_idle_state_set(TIM3, TIM_OC1);
    timer_set_oc_value(TIM3, TIM_OC1, 0);
    timer_set_oc_slow_mode(TIM3, TIM_OC1);
    timer_enable_oc_output(TIM3, TIM_OC1);

    timer_enable_oc_preload(TIM3, TIM_OC2);
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM3, TIM_OC2);
    timer_set_oc_idle_state_set(TIM3, TIM_OC2);
    timer_set_oc_value(TIM3, TIM_OC2, 0);
    timer_set_oc_slow_mode(TIM3, TIM_OC2);
    timer_enable_oc_output(TIM3, TIM_OC2);

    timer_enable_oc_preload(TIM3, TIM_OC3);
    timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM3, TIM_OC3);
    timer_set_oc_idle_state_set(TIM3, TIM_OC3);
    timer_set_oc_value(TIM3, TIM_OC3, 0);
    timer_set_oc_slow_mode(TIM3, TIM_OC3);
    timer_enable_oc_output(TIM3, TIM_OC3);

    timer_enable_oc_preload(TIM3, TIM_OC4);
    timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM3, TIM_OC4);
    timer_set_oc_idle_state_set(TIM3, TIM_OC4);
    timer_set_oc_value(TIM3, TIM_OC4, 0);
    timer_set_oc_slow_mode(TIM3, TIM_OC4);
    timer_enable_oc_output(TIM3, TIM_OC4);

    timer_enable_counter(TIM3);
}

/* Таймер 4 - для формирования импульса запуска эхо-локатора.
  * Работает в режиме ШИМ2 в One Pulse Mode
  * пауза 2 uS, импульс 10uS
  */
static void tim4_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM4);
    rcc_periph_reset_pulse(RST_TIM4);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM4_CH4);

    timer_set_mode(TIM4, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM4, (72 - 1));
    timer_set_period(TIM4, 11);  // ARR
    timer_disable_preload(TIM4);
    timer_one_shot_mode(TIM4);

    timer_enable_oc_preload(TIM4, TIM_OC4);
    timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM2);
    timer_set_oc_polarity_high(TIM4, TIM_OC4);
    timer_set_oc_idle_state_set(TIM4, TIM_OC4);
    timer_set_oc_value(TIM4, TIM_OC4, 2);
    timer_set_oc_slow_mode(TIM4, TIM_OC4);
    timer_enable_oc_output(TIM4, TIM_OC4);
}

/*-----------------------------------------------------------*/
/*----       ISR Overrides in libopencm3     ----------------*/
/*-----------------------------------------------------------*/

void sv_call_handler(void)
{
  	vPortSVCHandler();
}

/*-----------------------------------------------------------*/

void pend_sv_handler(void)
{
  	xPortPendSVHandler();
}

/*-----------------------------------------------------------*/

void sys_tick_handler(void)
{
  	xPortSysTickHandler();
}
