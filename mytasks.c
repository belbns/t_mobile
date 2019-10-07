#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/adc.h>

#include "mytasks.h"
#include "periph.h"
#include "jfes.h"

bool push_state(uint8_t mstate, uint8_t num);

void put_to_cmd_queue(uint16_t cmd, int16_t iparam);
void put_motors_cmd(char command, int16_t iparam);
void put_stepp_cmd(uint8_t step_num, char command, int16_t iparam);
void put_servo_cmd(char command, int16_t iparam);
void put_dist_cmd(char command, int16_t iparam);
void put_leds_cmd(char command, int16_t iparam);
void procSteppCmd(uint8_t stnum, ncommand_item cmd);
void send_usart_dma(const char * pack);

void service_blink_on(TickType_t ticks);
void service_blink_off(void);

char * itoa(int val, int base);

void echo_pulse(void);
TickType_t echo_one_shot(void);
void calc_dist(uint16_t count);
void ADC_calc(void);
void running_delay(uint32_t lasting, uint8_t c_status);

// константы для формирования пакета к пульту
const char js_at[] = "\n\rAT\n\r";
const char js_delim[] = ":";
const char js_coma[] = ",";
const char js_lbr[] = "[";
const char js_rbr[] = "]";
const char js_begin[] = "{";
const char js_end[] = "}\n";
const char js_true[] = "true";
const char js_false[] = "false";
const char js_ready[] = "\"ready\"";
const char js_queue[] = "\"queue\"";
const char js_qmot[] = "\"qmot\"";
const char js_qst1[] = "\"qst1\"";
const char js_qst2[] = "\"qst2\"";
const char js_qservo[] = "\"qservo\"";
const char js_qdist[] = "\"qdist\"";
const char js_qleds[] = "\"qleds\"";
const char js_batt[] = "\"batt\"";
const char js_pwroff[] = "\"pwroff\"";
const char js_ms[] = "\"ms\"";
const char js_mq[] = "\"mq\"";
const char js_ss[] = "\"ss\"";
const char js_sv[] = "\"sv\"";
const char js_sq[] = "\"sq\"";
const char js_led[] = "\"led\"";
const char js_servo[] = "\"servo\"";
const char js_dist[] = "\"dist\"";
const char js_adc[] = "\"adc\"";
const char js_a[] = "\"a\"";
const char js_b[] = "\"b\"";
const char js_c[] = "\"c\"";
const char js_d[] = "\"d\"";
const char js_f[] = "\"f\"";
const char js_l[] = "\"l\"";
const char js_m[] = "\"m\"";
const char js_n[] = "\"n\"";
const char js_r[] = "\"r\"";
const char js_s[] = "\"s\"";
const char js_u[] = "\"u\"";
const char js_v[] = "\"v\"";

// константы для парсинга пакета от пульта
const char jsp_check[] = "check";
const char jsp_pwroff[] = "pwroff";
const char jsp_stop[] = "stop";
const char jsp_mot[] = "mot";
const char jsp_st[] = "st";
const char jsp_led[] = "le";
const char jsp_servo[] = "servo";
const char jsp_echo[] = "echo";
const char jsp_dist[] = "dist";
const char jsp_pause[] = "pause";


extern QueueHandle_t xRxedChars;
extern QueueHandle_t xCmdQueue;
extern QueueHandle_t xCmdMotQueue;
extern QueueHandle_t xCmdSt1Queue;
extern QueueHandle_t xCmdSt2Queue;
extern QueueHandle_t xCmdServoQueue;
extern QueueHandle_t xCmdLedsQueue;
extern QueueHandle_t xCmdDistQueue;
extern QueueHandle_t xStateQueue;

extern EventGroupHandle_t xEventGroupBLE;
extern EventGroupHandle_t xEventGroupADC;

extern SemaphoreHandle_t xSemaphHandleEcho;
extern SemaphoreHandle_t xSemaphVoltage;
extern SemaphoreHandle_t xSemaphSent;
extern SemaphoreHandle_t xSemaphSendErr;

extern SemaphoreHandle_t xSteppMutex[2];
extern SemaphoreHandle_t xSendPackMutex;

extern motor_ctrl motors;
extern stepper stepp[2];
extern servo_drive servo;
extern uint8_t axle_stepper;
extern uint8_t axle_search;
extern mob_leds leds[];

static uint16_t lenCmdQueue = 0;
static uint16_t lenCmdMotQueue = 0;
static uint16_t lenCmdSt1Queue = 0;
static uint16_t lenCmdSt2Queue = 0;
static uint16_t lenCmdServoQueue = 0;
static uint16_t lenCmdLedsQueue = 0;
static uint16_t lenCmdDistQueue = 0;

jfes_config_t config;
jfes_parser_t parser;
jfes_token_t tokens[JFES_MAX_TOKENS_COUNT];
jfes_size_t tokens_count = JFES_MAX_TOKENS_COUNT;

static uint8_t init_process = 1;  // идет инициализация железа
static adcvalue adcval[4] = { {0, false}, {0, false}, {0, false}, {0, false} };
static queuestat q_cmd = {0, false};

static char cBleInBuf[BLE_PACK_SIZE + 4];
static uint8_t iBleBuf = 0;					// текущий индекс буфера cBLEInBuf
static char cReceivedPack[BLE_PACK_SIZE + 4];
static char stat_pack[BLE_PACK_SIZE + 4];
static uint8_t flag_power_off = 0;

static TickType_t leds_delay = pdMS_TO_TICKS(1000);

char TxBuffer[BLE_PACK_SIZE + 4];

uint8_t cnt_status = CNT_ON_MS;
// счетчик пройденного пути
uint32_t ir_count = 0;

echo_sensor esensor = { 0, 0, 0, false};

uint16_t ADCbuffer[12];
uint16_t echo_count = 0;

static uint32_t nocmd_count = 0;


void vTaskMain(void *pvParameters)
{
    ( void ) pvParameters;

	EventBits_t uxBits;

	jfes_value_t value;
	jfes_array_t *jarr;
	jfes_value_t *newvalue;

	//printf("tMain\n");

	// init jfes
	config.jfes_malloc = (jfes_malloc_t)pvPortMalloc;
	config.jfes_free = vPortFree;
	jfes_init_parser(&parser, &config);

	//printf("tM1\n");

	//service_blink_on(pdMS_TO_TICKS(500));

	// не готов к обмену во время инициализации
	usart_disable_rx_interrupt(USART1);
	sendPackToBLE(js_at);

	//push_state(STATE_PACK_BUSY, 0);
	printf("tM2\n");

	motors.state = MOTOR_OFF;
	esensor.state = ECHO_ONE;
	stepp[0].mode = stepp[1].mode = STEP_MAN;
	set_servo_angle(90);
	printf("tM3\n");
	vTaskDelay(pdMS_TO_TICKS(500));
	printf("tM4\n");
	sendPackToBLE("tM4\n");
	//push_state(STATE_PACK_READY, 0);
	//printf("tM5\n");

	// готов к обмену
	usart_enable_rx_interrupt(USART1);

	xEventGroupClearBits(xEventGroupADC, (const EventBits_t)0xFF);

	// Включаем 6V
	gpio_set(POWER_HOLD_PORT, POWER_6V_PIN);
	init_process = 0;

	//service_blink_on(pdMS_TO_TICKS(1000));
	//service_blink_off();

	printf("tM6\n");
	sendPackToBLE("tM6\n");

	int16_t ipar16 = 0;
	int16_t num = 0;
	uint16_t tmp = 0;
	char st[4];

	// Основной цикл
	for (;;)
	{
		//printf("_tMain\n");
		//printf("vTaskMain stack: %d\n", uxTaskGetStackHighWaterMark( NULL ));
		// проверка нажатия кнопки питания, разряда батареи,
		// и команды на выключение
		uxBits = xEventGroupWaitBits(
			xEventGroupADC,
			alarm_POWER_BUTTON_BIT ||
			alarm_MOTORS_OFF_VOLTAGE_BIT  ||
			alarm_CONTRL_OFF_VOLTAGE_BIT ||
			alarm_POWER_COMMAND_BIT,
            pdTRUE,		// очищаем флаг
		    pdFALSE,	//	 Don't wait for all bits.
		    0 );		// ticks to wait

		if ( uxBits & alarm_POWER_BUTTON_BIT) // нажатие кнопки питания
		{
			push_state(STATE_PACK_PWROFF_KEY, 0);
			flag_power_off = 1;
			vTaskDelay(pdMS_TO_TICKS(100));
			power_off();
		}
		else if (uxBits & alarm_POWER_COMMAND_BIT) // команда выключения
		{
			push_state(STATE_PACK_PWROFF_CMD, 0);
			flag_power_off = 1;
			vTaskDelay(pdMS_TO_TICKS(100));
			power_off();
		}
		else if (uxBits & (alarm_MOTORS_OFF_VOLTAGE_BIT ||
				alarm_CONTRL_OFF_VOLTAGE_BIT)) // разряд батареи
		{
			push_state(STATE_PACK_PWROFF_BAT, 0);
			flag_power_off = 1;
			vTaskDelay(pdMS_TO_TICKS(100));
			power_off();
		}

		// проверка на недопустимый ток нагрузки
		uxBits = xEventGroupWaitBits(
			xEventGroupADC,
			alarm_OVERLOAD_BIT,
            pdTRUE,		// очищаем флаг
		    pdFALSE,	//	 Don't wait for all bits.
		    0 );		// ticks to wait

		if ( uxBits & alarm_OVERLOAD_BIT)
		{
			// Слишком большой ток - очищаем очереди и выключаем моторы
			//trace_puts("*** too big current");
			xQueueReset(xCmdMotQueue);
			xQueueReset(xCmdSt1Queue);
			xQueueReset(xCmdSt2Queue);
			xQueueReset(xCmdServoQueue);
			stop_all();
			push_state(STATE_PACK_OVERLOAD, 0);
		}

		// проверка наличия входящего пакета и его обработка
		uxBits = xEventGroupWaitBits(
			xEventGroupBLE, ble_RECIEVED_JSON,
            pdTRUE,		// очищаем флаг
		    pdFALSE,	// Don't wait for all bits.
		    0 );		// ticks to wait

		if (uxBits & ble_RECIEVED_JSON)
		{
			// пришел пакет от ДУ, обрабатываем cReceivedPack[]
			//puts(cReceivedPack);
			st[0] = '\0';
			bool parsed = false;
			// парсим входящий пакет, в пакете может быть только 1 команда
			jfes_status_t status = jfes_parse_to_value(&config, cReceivedPack,
					strlen(cReceivedPack), &value);
			if (status == jfes_success)
			{
				// PAUSE
				newvalue = jfes_get_child(&value, jsp_pause, 0);
				if (newvalue)
				{
					put_to_cmd_queue(PAUSE, newvalue->data.int_val);
					parsed = true;
				}
				// POWER_OFF
				if (! parsed)
				{
					newvalue = jfes_get_child(&value, jsp_pwroff, 0);
					if (newvalue)
					{
						bool now = newvalue->data.bool_val;
						if (now)		// выключить немедленно
						{
							push_state(STATE_PACK_PWROFF_CMD, 0);
							flag_power_off = 1;
							vTaskDelay(pdMS_TO_TICKS(100));
							power_off();
						}
						else
						{
							put_to_cmd_queue(POWER_OFF, 0);
						}
					}
				}
				// STOP_ALL
				if (! parsed)
				{
					newvalue = jfes_get_child(&value, jsp_stop, 0);
					if (newvalue)
					{
						bool now = newvalue->data.bool_val;
						if (now)		// остановить немедленно
						{
							xQueueReset(xCmdMotQueue);
							xQueueReset(xCmdSt1Queue);
							xQueueReset(xCmdSt2Queue);
							xQueueReset(xCmdServoQueue);
							stop_all();
						}
						else
						{
							put_to_cmd_queue(STOP_ALL, 0);
						}
					}
				}
				/*
				MOT_STOP,
				MOT_UP_DOWN,
				MOT_RIGHT,
				MOT_LEFT,
				MOT_STRAIGHT
				*/
				if (! parsed)
				{
					newvalue = jfes_get_child(&value, jsp_mot, 0);
					if (newvalue)
					{
			        	if (newvalue->type == jfes_type_array)  // 2 параметра - f, b, p
			        	{
			            	jarr = newvalue->data.array_val;
			            	if (jarr->items[0]->type == jfes_type_string)
			            	{
			            		memcpy(st, jarr->items[0]->data.string_val.data,
			            			jarr->items[0]->data.string_val.size);
				            	if (jarr->items[1]->type == jfes_type_integer)
				            	{
				            		put_motors_cmd(st[0], jarr->items[1]->data.int_val);
				            	}
				            	else
				            	{
				            		printf("*** error in token motors (1)!\n");
				            	}
			            	}
			            	else
			            	{
			            		printf("*** error in token motors (0)!\n");
			            	}
			        	}
			        	else if (newvalue->type == jfes_type_string) // 1 параметр - l, n, r, s
			        	{
		            		memcpy(st, newvalue->data.string_val.data,
		            			newvalue->data.string_val.size);
		            		put_motors_cmd(st[0], 0);
			        	}
					}
				}
				/*
				ST_NULL,
				ST_STOP,
				ST_CONT,
				ST_CONT_STOP,
				ST_RET,
				 */
				if (! parsed)
				{
					newvalue = jfes_get_child(&value, jsp_st, 0);
					if (newvalue)
					{
						if (newvalue->type == jfes_type_array) {
							jarr = newvalue->data.array_val;
							if (jarr->items[0]->type == jfes_type_string)
							{
								memcpy(st, jarr->items[0]->data.string_val.data,
			            			jarr->items[0]->data.string_val.size);
								if (jarr->items[1]->type == jfes_type_integer)
								{
									num = jarr->items[1]->data.int_val;
									if ( (jarr->count > 2) &&
										(jarr->items[2]->type == jfes_type_integer) )
									{
										ipar16 = (uint8_t)jarr->items[2]->data.int_val;
									}
									else
									{
										ipar16 = 0;
									}
									put_stepp_cmd(num, st[0], ipar16);
								}
								else
								{
									printf("*** error in token stepp (1)!\n");
								}
							}
							else
							{
								printf("*** error in token stepp (0)!\n");
							}
						}
					}
				}
				// LEDS_MOD
				if (! parsed)
				{
					newvalue = jfes_get_child(&value, jsp_led, 0);
					if (newvalue)
					{
						if (newvalue->type == jfes_type_array) {
							jarr = newvalue->data.array_val;
							if (jarr->items[0]->type == jfes_type_string)
							{
								memcpy(st, jarr->items[0]->data.string_val.data,
			            			jarr->items[0]->data.string_val.size);
								if (jarr->items[1]->type == jfes_type_integer)
								{
									ipar16 = jarr->items[1]->data.int_val;
									put_leds_cmd(st[0], ipar16);
								}
								else
								{
									printf("*** error in token leds (1)!\n");
								}
							}
							else
							{
								printf("*** error in token leds (0)!\n");
							}
						}
					}
				}
				// SERVO_SET
				// SERVO_PAUSE
				if (! parsed)
				{
					newvalue = jfes_get_child(&value, jsp_servo, 0);
					if (newvalue)
					{
						if (newvalue->type == jfes_type_array) {
							jarr = newvalue->data.array_val;
							if (jarr->items[0]->type == jfes_type_string)
							{
								memcpy(st, jarr->items[0]->data.string_val.data,
			            			jarr->items[0]->data.string_val.size);
								if (jarr->items[1]->type == jfes_type_integer)
								{
									ipar16 = jarr->items[1]->data.int_val;
									put_servo_cmd(st[0], ipar16);
								}
								else
								{
									printf("*** error in token servo (1)!\n");
								}
							}
							else
							{
								printf("*** error in token servo (0)!\n");
							}
						}
					}
				}
				// DIST_SHOT
				// DIST_PAUSE
				if (! parsed)
				{
					newvalue = jfes_get_child(&value, jsp_echo, 0);
					if (newvalue)
					{
						if (newvalue->type == jfes_type_array)
						{
							jarr = newvalue->data.array_val;
							if (jarr->items[0]->type == jfes_type_string)
							{
								memcpy(st, jarr->items[0]->data.string_val.data,
			            			jarr->items[0]->data.string_val.size);
								if (jarr->items[1]->type == jfes_type_integer)
								{
									ipar16 = jarr->items[1]->data.int_val;
									put_dist_cmd(st[0], ipar16);
								}
								else
								{
									printf("*** error in token dist (1)!\n");
								}
							}
							else
							{
								printf("*** error in token dist (0)!\n");
							}
						}
					}
				}
				/*
				CNT_SET_OFF,
				CNT_SET_ON_MS,
				CNT_SET_ON_STEPS,
				CNT_RESET
				*/
				if (! parsed)
				{
					newvalue = jfes_get_child(&value, jsp_dist, 0);
					if (newvalue)
					{
						tmp = NO_COMMAND;
						memcpy(st, newvalue->data.string_val.data, newvalue->data.string_val.size);
						switch (st[0])
						{
						case 'm':	// в милисекундах
							tmp = CNT_SET_ON_MS;
							break;
						case 's':	// в импульсах датчика
							tmp = CNT_SET_ON_STEPS;
							break;
						case 'c':	// сбросить
							tmp = CNT_RESET;
							break;
						default:	// 'n' - не использовать
							tmp = CNT_SET_OFF;
						}
						put_to_cmd_queue(tmp, 0);
					}
				}
			}
			nocmd_count = 0;					// был пакет от пульта
		}	// end JSON
		else
		{
			nocmd_count++;
			if (nocmd_count > WAIT_FOR_PACK)	// долго не было пакетов
			{
				nocmd_count = 0;
				put_to_cmd_queue(STOP_ALL, 0);	// останавливаем все
			}
		}

		// проверка изменения очередей, отправка изменений
		tmp = uxQueueMessagesWaiting(xCmdQueue);
		if (tmp != lenCmdQueue)
		{
			if ( push_state(STATE_PACK_QUEUE_CMD, (uint8_t)tmp) )
			{
				lenCmdQueue = tmp;
			}
		}
		tmp = uxQueueMessagesWaiting(xCmdMotQueue);
		if (tmp != lenCmdMotQueue)
		{
			if ( push_state(STATE_PACK_QUEUE_MOT, (uint8_t)tmp) )
			{
				lenCmdMotQueue = tmp;
			}
		}
		tmp = uxQueueMessagesWaiting(xCmdSt1Queue);
		if (tmp != lenCmdSt1Queue)
		{
			if ( push_state(STATE_PACK_QUEUE_ST1, (uint8_t)tmp) )
			{
				lenCmdSt1Queue = tmp;
			}
		}
		tmp = uxQueueMessagesWaiting(xCmdSt2Queue);
		if (tmp != lenCmdSt2Queue)
		{
			if ( push_state(STATE_PACK_QUEUE_ST2, (uint8_t)tmp) )
			{
				lenCmdSt2Queue = tmp;
			}
		}
		tmp = uxQueueMessagesWaiting(xCmdServoQueue);
		if (tmp != lenCmdServoQueue)
		{
			if ( push_state(STATE_PACK_QUEUE_SERVO, (uint8_t)tmp) )
			{
				lenCmdServoQueue = tmp;
			}
		}
		tmp = uxQueueMessagesWaiting(xCmdDistQueue);
		if (tmp != lenCmdDistQueue)
		{
			if ( push_state(STATE_PACK_QUEUE_DIST, (uint8_t)tmp) )
			{
				lenCmdDistQueue = tmp;
			}
		}
		tmp = uxQueueMessagesWaiting(xCmdLedsQueue);
		if (tmp != lenCmdLedsQueue)
		{
			if ( push_state(STATE_PACK_QUEUE_LEDS, (uint8_t)tmp) )
			{
				lenCmdLedsQueue = tmp;
			}
		}

		// проверка изменения состояния устройств, отправка изменений
		if (!motors.checked && push_state(STATE_PACK_MOTORS, 0))
		{
			motors.checked = true;
		}
		if (!stepp[0].checked && push_state(STATE_PACK_STEPP, 0))
		{
			stepp[0].checked = true;
		}
		if (!stepp[1].checked && push_state(STATE_PACK_STEPP, 1))
		{
			stepp[1].checked = true;
		}
		for (int8_t i = 0; i < 4; i++)
		{
			if (!leds[i].checked && push_state(STATE_PACK_LED, i))
			{
				leds[i].checked = true;
			}
		}
		if (!servo.checked && push_state(STATE_PACK_SERVO, 0))
		{
			servo.checked = true;
		}
		if (!esensor.checked && push_state(STATE_PACK_SERVO, 0))
		{
			esensor.checked = true;
		}
		//adc
		for (uint8_t i = 0; i < 4; i++)
		{
			if (!adcval[i].checked && push_state(STATE_PACK_ADC, i))
			{
				adcval[i].checked = true;
			}
		}
		if (!q_cmd.checked && push_state(STATE_PACK_QUEUE_CMD, 0))
		{
			q_cmd.checked = true;
		}

		taskYIELD();
	} 
}

void vTaskADC(void *pvParameters)
{
    ( void ) pvParameters;

	vTaskDelay(pdMS_TO_TICKS(20));
	adc_start_conversion_direct(ADC1);

	printf( "tADC" );

	for (;;) {
		//printf("_ADC\n");
		//printf("vTaskADC stack: %d\n", uxTaskGetStackHighWaterMark( NULL ));

		if (flag_power_off)
		{
			taskYIELD();
		}
		//puts("tADC1");
		if ( xSemaphoreTake(xSemaphVoltage, 0) == pdTRUE )
		{
			//puts("vTaskADC 3");
			// есть новые результаты измерения
			ADC_calc();
			if ( (adcval[2].value >= SENS_LEVEL_4) &&
					(adcval[2].value < SENS_LEVEL_5) )	//P5
			{
				xEventGroupSetBits( xEventGroupADC,	alarm_HALL1_BIT);
			}
			else if ( (adcval[2].value >= SENS_LEVEL_8) &&
					(adcval[2].value < SENS_LEVEL_9) )
			{
				xEventGroupSetBits( xEventGroupADC,	alarm_HALL2_BIT);
			}

			// запускаем следующий цикл измерений
			vTaskDelay(pdMS_TO_TICKS(20));
			adc_start_conversion_direct(ADC1);
		}
		//printf("tADC2");
		taskYIELD();
	}


}

/*
 *  Управляем миганием светодиодов с режимом LED_BLINK
 *  5-й светодиод - на плате STM32, всегда в режиме LED_BLINK
 */
void vTaskLedsBlink(void *pvParameters)
{
    ( void ) pvParameters;

    //trace_printf("vTaskLedsBlink start stack: %d\n", uxTaskGetStackHighWaterMark( NULL ));
    //printf("tBlnk\n");
    TickType_t xLastWakeTime = xTaskGetTickCount();
        
    for (;;) {
        //trace_printf("vTaskLedsBlink stack: %d\n", uxTaskGetStackHighWaterMark( NULL ));
        //printf("_tBlnk\n");
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

void vTaskBLE(void *pvParameters)
{
    ( void ) pvParameters;
        
    static TickType_t xTime1;
                
    signed char cChar;
    static char sbuf[20];
                
    //puts( "start vTaskBLE" );
    //printf("tBLE\n");
                
    xEventGroupClearBits(xEventGroupBLE, (const EventBits_t)0xFF);
    xTime1 = xTaskGetTickCount();
                
    for (;;) {
        //trace_printf("vTaskBLE stack: %d\n", uxTaskGetStackHighWaterMark( NULL ));
        //printf("_tBLE\n");

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
                //printf("%c\n", cChar);
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
                //printf(cBleInBuf);
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
            //lSerialPutString( sbuf, strlen(sbuf) ); 
            // отправляем пакет статуса
            sendPackToBLE(sbuf);
            xTime1 = xTime2;
        }

        taskYIELD();
    }
}

/*
 * Исполнение команд из очереди xCmdQueue
 */
void vTaskCmd(void *pvParameters)
{
	ncommand_item item;     // = {NO_CMD, 0, 0, CNT_OFF, 0};

	( void ) pvParameters;

    //printf("tCmd\n");
    //puts( "start vTaskCmd" );
    //printf("vTaskCmd start stack: %d\n", uxTaskGetStackHighWaterMark( NULL));

    for (;;) {
    	//printf("_tCmd\n");

        //trace_printf("vTaskCmd stack: %d\n", uxTaskGetStackHighWaterMark( NULL ));
        if (flag_power_off)
        {
        	taskYIELD();
		}

        // есть ли очередная команда?
        if ( xQueueReceive( xCmdQueue, &item, 0 ) == pdPASS )
        {
        	//printf("vTaskCMD: q_space=%d cmd=%d dev=%d uparam=%d\n",
			//              uxQueueSpacesAvailable(xCmdQueue),
            //              item.cmd, item.dev, item.uparam);
            //printf("cmd=%d dev=%d sparam1=%d sparam2=%d\n",
            //              item.cmd, item.dev, item.sparam1, item.sparam2);

			switch (item.cmd)
            {
            	case PAUSE:     // просто пауза
                	running_delay(item.param, cnt_status);
                    break;
                case POWER_OFF: // выключить
                	flag_power_off = 1;
                    xEventGroupSetBits( xEventGroupADC, alarm_POWER_COMMAND_BIT);
					break;
				case STOP_ALL:
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
                    set_motor_value(3, MOTOR_GEAR0, MOTOR_GEAR0);
                    motors.gear = GEAR_0;
					// серво в центр
                    set_servo_angle(90);
                    // гасим внешние светодиоды
				    for (uint8_t i = 0; i < 4; i++)
					{
                    	leds[i].mode = LED_OFF;
                        gpio_clear(leds[i].port, leds[i].pin);
                    }
                    break;
                case CNT_SET_OFF:
					cnt_status = CNT_OFF;
                    break;
			    case CNT_SET_ON_MS:
            		cnt_status = CNT_ON_MS;
                    break;
			    case CNT_SET_ON_STEPS:
            		cnt_status = CNT_ON_STEPS;
                    break;
    			case CNT_RESET:
                	ir_count = 0;
                    break;
		    }
		}
		taskYIELD();
    }
}

/*
 * Исполнение команд из очереди xCmdQueue
 */
void vTaskCmdMot(void *pvParameters)
{
    ncommand_item item;
                                
    ( void ) pvParameters;
                        
    //printf("tCmdMot\n");
    for (;;) {
    	//printf("_tCmdMot\n");
                        
        //trace_printf("vTaskCmdMot stack: %d\n", uxTaskGetStackHighWaterMark( NULL ));
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
                	set_motor_value(3, MOTOR_GEAR0, MOTOR_GEAR0);
                    motors.gear = GEAR_0;
                    break;
			    case MOT_UP_DOWN:
                    // если поворот - статус не меняется, а скоростьможет меняться
					if (motors.state == MOTOR_LEFT)
                    {
                        set_motor_value(2, 0, item.param);
                    }
                    else if (motors.state == MOTOR_RIGHT)
                    {
                        set_motor_value(1, item.param, 0);
                    }
                    else
                    {
                        // при старте, сначала включаем 2-ю скорость
						if ((motors.state == MOTOR_STOPPED) && (abs(item.param) ==  GEAR_1))
						{
                        	if (item.param > 0)
                            {
                            	set_motor_value(3, MOTOR_GEAR2, MOTOR_GEAR2);
							}
                            else
                            {
                            	set_motor_value(3, -MOTOR_GEAR2, -MOTOR_GEAR2);
							}
                                                vTaskDelay(pdMS_TO_TICKS(MOTOR_START_TIME));
						}
                        set_motor_value(3, item.param, item.param);
					}
                    motors.pre_value = item.param;
                    motors.gear = item.param;
                    break;
                case MOT_LEFT:
                	switch (motors.state)
                    {
                    	case MOTOR_LEFT:
                        	break;
                        case MOTOR_STOPPED:
                        	motors.pre_value = MOTOR_GEAR0;
                            motors.state = MOTOR_LEFT;
                            set_motor_value(3, -MOTOR_GEAR2, MOTOR_GEAR2);
                            vTaskDelay(pdMS_TO_TICKS(MOTOR_START_TIME));
							set_motor_value(3, -MOTOR_GEAR1, MOTOR_GEAR1);
							break;
					    case MOTOR_RIGHT:
                    		set_motor_value(3, 0, 0);       // стопоба
							vTaskDelay(pdMS_TO_TICKS(20));
                            motors.state = MOTOR_LEFT;
                            set_motor_value(3, motors.pre_value, 0);
                            break;
                        default:
                        	motors.state = MOTOR_LEFT;
                            set_motor_value(1, 0, 0);       // стоплевый
					}
                    break;
                case MOT_RIGHT:
                	switch (motors.state)
                    {
                    	case MOTOR_RIGHT:
                        break;
                        case MOTOR_STOPPED:
                        	motors.pre_value = MOTOR_GEAR0;
                            motors.state = MOTOR_RIGHT;
                            set_motor_value(3, MOTOR_GEAR2, -MOTOR_GEAR2);
							vTaskDelay(pdMS_TO_TICKS(MOTOR_START_TIME));
							set_motor_value(3, MOTOR_GEAR1, -MOTOR_GEAR1);
							break;
						case MOTOR_LEFT:
                        	set_motor_value(3, 0, 0);       // стопоба
							vTaskDelay(pdMS_TO_TICKS(20));
                            motors.state = MOTOR_RIGHT;
                            set_motor_value(3, 0, motors.pre_value);
                            break;
                        default:
                        	motors.state = MOTOR_RIGHT;
                            set_motor_value(2, 0, 0);       // стоп правый
					}
                    break;
                case MOT_STRAIGHT:      // отмена поворота
                	if (motors.pre_value == 0)
                    {
                    	motors.state = MOTOR_STOPPED;
                        set_motor_value(3, 0, 0);
					}
                    else
                    {
                    	set_motor_value(3, motors.pre_value, motors.pre_value);
						if (motors.pre_value > 0)
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

/*
 * Исполнение команд из очередей xCmdSt1Queue, xCmdSt2Queue
 */
void vTaskCmdSt(void *pvParameters)
{
    ncommand_item item;
    ( void ) pvParameters;

    //printf("tCmdS1\n");
    for (;;) {
        //printf("_tCmdS1\n");
        //printf("vTaskCmdSt1 stack: %d\n", uxTaskGetStackHighWaterMark(NULL ));
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

/*
 * Исполнение команд из очереди xCmdServoQueue
 */
void vTaskCmdServo(void *pvParameters)
{
	ncommand_item item;
                                                
    ( void ) pvParameters;
                                        
    //printf("tCmdSrv\n");
    for (;;) {
    	//printf("_tCmdSrv\n");
        
        //printf("vTaskCmdServo stack: %d\n", uxTaskGetStackHighWaterMark( NULL ));
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

/*
 * Исполнение команд из очереди xCmdDistQueue
 */
void vTaskCmdDist(void *pvParameters)
{
	ncommand_item item;

    ( void ) pvParameters;

    //printf("tCmdD\n");
	for (;;) {
    	//printf("_tCmdD\n");

        //printf("vTaskCmdDist stack: %d\n", uxTaskGetStackHighWaterMark( NULL ));
		if (flag_power_off)
        {
			taskYIELD();
		}

		// есть ли очередная команда?
        if ( xQueueReceive( xCmdDistQueue, &item, 0 ) == pdPASS )
        {
        	if (item.cmd == DIST_SHOT)
            {
            	uint16_t vv = echo_one_shot();
                if (vv > 38)
                {
                	esensor.value = 2048;
                }
                else
                {
                	calc_dist(echo_count);
				}
                esensor.checked = false;
			}
            else if (item.cmd == DIST_PAUSE)
            {
            	running_delay(item.param, cnt_status);
			}
		}
		taskYIELD();
	}
}

/*
 * Исполнение команд из очереди xCmdLedsQueue
 */
void vTaskCmdLeds(void *pvParameters)
{
	ncommand_item item;

    ( void ) pvParameters;

    //printf("tCmdL\n");
    for (;;) {
    	//printf("_tCmdL\n");
        //printf("vTaskCmdLeds stack: %d\n", uxTaskGetStackHighWaterMark( NULL ));
		if (flag_power_off)
        {
        	taskYIELD();
		}
        
        // есть ли очередная команда?
		if ( xQueueReceive( xCmdLedsQueue, &item, 0 ) == pdPASS )
        {
        	switch (item.cmd)
            {
            	case LED0_MOD:
                case LED1_MOD:
                case LED2_MOD:
                case LED3_MOD:
                	set_led((uint8_t)item.cmd, item.param);
                    break;
				case LEDS_PAUSE:
                	running_delay(item.param, cnt_status);
                    break;
			}
		}
        taskYIELD();
	}
}

// ==============================================================
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
			item.cmd = ST_UP_DOWN;
		}
		break;
	case 'h':	// в исх. положение без сброса оборотов
       	item.cmd = ST_NULL;
   		break;
	case 'n':	// в исх. положение со сбросом оборотов
       	item.cmd = ST_NULL_FAST;
       	break;
	case 'r':	// вращать по часовой стрелке
       	item.cmd = ST_CONT_RIGHT;
       	break;
	case 'l':	// вращать против часовой стрелки
       	item.cmd = ST_CONT_LEFT;
       	break;
	case 'p':
		item.cmd = ST_PAUSE;
			break;
	default:	// 's' - stop
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


void put_motors_cmd(char command, int16_t iparam)
{
	ncommand_item item;

	switch (iparam)
	{
	case 1:
		item.param = MOTOR_GEAR1;
		break;
	case 2:
		item.param = MOTOR_GEAR2;
		break;
	case 3:
		item.param = MOTOR_GEAR3;
		break;
	default:
		item.param = MOTOR_GEAR0;
	}

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
	default:	//case 's':
		item.cmd = MOT_STOP;
		break;
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

void put_leds_cmd(char command, int16_t iparam)
{
	ncommand_item item;

	if (command == 'p')	// PAUSE
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
	default:	//case 'd':
		item.cmd = LED_OFF;
		break;
	}

	uint8_t cc = 5;
	while ( (uxQueueSpacesAvailable(xCmdLedsQueue) == 0) && (cc-- > 0) ) // ждем до 100mS
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

	if (command == 'p')	// PAUSE
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

void put_dist_cmd(char command, int16_t iparam)
{
	ncommand_item item;

	if (command == 'p')	// PAUSE
	{
		item.cmd = DIST_PAUSE;
		item.param = iparam;
	}
	else
	{
		item.cmd = DIST_SHOT;
		item.param = 0;
	}

	uint8_t cc = 5;
	while ( (uxQueueSpacesAvailable(xCmdDistQueue) == 0) && (cc-- > 0) ) // ждем до 100mS
	{
		vTaskDelay(pdMS_TO_TICKS(20));
	}
	if (cc > 0)
	{
		xQueueSend(xCmdDistQueue, &item, pdMS_TO_TICKS(50));
	}
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

void procSteppCmd(uint8_t stnum, ncommand_item cmd)
{
	switch (cmd.cmd)
	{

	case ST_PAUSE:	// просто пауза
		running_delay(cmd.param, cnt_status);
		break;

	case ST_CONT_STOP:	// перевод из STEP_MAN_CONT в STEP_MAN
		stepp[stnum].mode = STEP_MAN;
		stepp[stnum].steps = stepp[stnum].steps & 7;
		// ждем завершения движения
		while ( (stepp[stnum].steps > 0) || stepp[stnum].last_step )
		{
			vTaskDelay(pdMS_TO_TICKS(10));
		}
		break;

	case ST_UP_DOWN:	// поворот ШД в ручном режиме, переводит в STEP_MAN
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
			stepp_up_down(stnum, a);
			xSemaphoreGive( xSteppMutex[stnum] );
		}
		else
		{
			//puts("stepper error - cannot take the xSteppMutex[ii]");
		}
		break;

	case ST_CONT_LEFT:
	case ST_CONT_RIGHT:
		if (stepp[stnum].mode == STEP_MAN)
		{
			stepp[stnum].steps = stepp[stnum].steps & 7;
			// ждем окончания движения
			while ( (stepp[stnum].steps > 0) || stepp[stnum].last_step )
			{
				vTaskDelay(pdMS_TO_TICKS(10));
			}
			stepp[stnum].mode = STEP_MAN_CONT;
			uint32_t steps = 0;
			// Вычисляем кол-во шагов до точки 0/512
			if (stepp[stnum].angle_curr != 0)
			{
				if (stepp[stnum].angle_curr > 0)
				{
					if (cmd.cmd == ST_CONT_LEFT)
					{
						steps = (STEPPER_ANGLE_TURN - stepp[stnum].angle_curr) << 3;
					}
					else
					{
						steps = stepp[stnum].angle_curr << 3;
					}
				}
				else
				{
					if (cmd.cmd == ST_CONT_LEFT)
					{
						steps = stepp[stnum].angle_curr << 3;								steps = (STEPPER_ANGLE_TURN - stepp[0].angle_curr) << 3;
					}
					else
					{
						steps = (STEPPER_ANGLE_TURN - stepp[stnum].angle_curr) << 3;
					}
				}
			}
			else
			{
				steps = STEPPER_ANGLE_TURN;
			}
			if (cmd.cmd == ST_CONT_LEFT)
			{
				stepp[stnum].clockw = 1;
			}
			else
			{
				stepp[stnum].clockw = 1;
			}
			stepp[stnum].steps = steps;
		}
		break;

	case ST_NULL:		// возврат в 0 и перевод в ручной режим
	case ST_NULL_FAST:	//   со сбросом оборотов
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

bool push_state(uint8_t mstate, uint8_t num)
{
	static char pack[20];

	if ( uxQueueSpacesAvailable(xStateQueue) == 0 )
	{
		return false;
	}

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
	else if ( (mstate == STATE_PACK_PWROFF_BAT) ||
			(mstate == STATE_PACK_PWROFF_KEY) ||
			(mstate == STATE_PACK_PWROFF_CMD) )
	{
		strcat(pack, js_pwroff);
		strcat(pack, js_delim);
		if (mstate == STATE_PACK_PWROFF_BAT)
		{
			strcat(pack, "\"b\"");	// по разряду батареи
		}
		else if (mstate == STATE_PACK_PWROFF_CMD)
		{
			strcat(pack, "\"c\"");	// по команде от пульта
		}
		else
		{
			strcat(pack, "\"k\"");	// по нажатию кнопки
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
		strcat(pack, js_lbr);	// [
		switch (motors.state)	// статус
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
		strcat(pack, js_coma);			// ,
		strcat(pack, itoa(motors.pre_value, 10));				// заданная скорость
		strcat(pack, js_coma);			// ,
		strcat(pack, itoa(motors.value1, 10));				// скорость ДПТ1
		strcat(pack, js_coma);			// ,
		strcat(pack, itoa(motors.value2, 10));				// скорость ДПТ2
		strcat(pack, js_rbr);			// ]
	}
	else if ( mstate == STATE_PACK_STEPP)
	{
		if ( uxQueueSpacesAvailable(xStateQueue) < 2 )
		{
			ret = false;	// статус ШД - 2 пакета
		}
		else
		{
			char stn[2];
			if (num == 0)
			{
				stn[0] = '0';			// ШД0
			}
			else
			{
				stn[0] = '1';			// ШД1
			}
			stn[1] = '\0';

			strcat(pack, js_ss);
			strcat(pack, js_delim);
			strcat(pack, js_lbr);			// [
			strcat(pack, stn);				// 0/1
			strcat(pack, js_coma);			// ,
			if (stepp[num].moving == 0)			// остановлен/в движении
			{
				strcat(pack, js_s);
			}
			else
			{
				strcat(pack, js_v);
			}
			strcat(pack, js_coma);			// ,
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
			strcat(pack, js_rbr);			// ]
			strcat(pack, js_end);
			xQueueSend(xStateQueue, pack, pdMS_TO_TICKS(20));

			strcpy(pack, js_begin);
			strcat(pack, js_sv);
			strcat(pack, js_delim);
			strcat(pack, js_lbr);			// [
			strcat(pack, stn);				// 0/1
			strcat(pack, js_coma);			// ,
			strcat(pack, itoa((int16_t)stepp[num].turns, 10));
			strcat(pack, js_coma);			// ,
			strcat(pack, itoa((int16_t)stepp[num].angle_curr, 10));
			strcat(pack, js_rbr);			// ]
		}
	}
	else if ( mstate == STATE_PACK_LED)
	{
		strcat(pack, js_led);
		strcat(pack, js_delim);
		strcat(pack, js_lbr);			// [
		//itoa(num, st, 10);
		strcat(pack, itoa(num, 10));				// 0..3
		strcat(pack, js_coma);			// ,
		switch (leds[num].mode)
		{
		case LED_ON:
			strcat(stat_pack, js_u);
			break;
		case LED_BLINK:
			strcat(stat_pack, js_b);
			break;
		default:
			strcat(stat_pack, js_d);
		}
		strcat(pack, js_rbr);			// ]
	}

	else if ( mstate == STATE_PACK_ADC)
	{
		strcat(pack, js_adc);
		strcat(pack, js_delim);
		strcat(pack, js_lbr);			// [
		strcat(pack, itoa(num, 10));				// 0..3
		strcat(pack, js_coma);			// ,
		strcat(pack, itoa(adcval[num].value, 10));				// adc value
		strcat(pack, js_rbr);			// ]
	}
	else if ( mstate == STATE_PACK_DIST)
	{
		strcat(pack, js_dist);
		strcat(pack, js_delim);
		strcat(pack, itoa(esensor.value, 10));				// dist value
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
		strcat(pack, itoa(num, 10));				// наличие команд в очереди
	}
	else if ( mstate == STATE_PACK_QUEUE_MOT)
	{
		strcat(pack, js_qmot);
		strcat(pack, js_delim);
		strcat(pack, itoa(num, 10));				// наличие команд в очереди
	}
	else if ( mstate == STATE_PACK_QUEUE_ST1)
	{
		strcat(pack, js_qst1);
		strcat(pack, js_delim);
		strcat(pack, itoa(num, 10));				// наличие команд в очереди
	}
	else if ( mstate == STATE_PACK_QUEUE_ST2)
	{
		strcat(pack, js_qst2);
		strcat(pack, js_delim);
		strcat(pack, itoa(num, 10));				// наличие команд в очереди
	}
	else if ( mstate == STATE_PACK_QUEUE_SERVO)
	{
		strcat(pack, js_qservo);
		strcat(pack, js_delim);
		strcat(pack, itoa(num, 10));				// наличие команд в очереди
	}
	else if ( mstate == STATE_PACK_QUEUE_DIST)
	{
		strcat(pack, js_qdist);
		strcat(pack, js_delim);
		strcat(pack, itoa(num, 10));				// наличие команд в очереди
	}
	else if ( mstate == STATE_PACK_QUEUE_LEDS)
	{
		strcat(pack, js_qleds);
		strcat(pack, js_delim);
		strcat(pack, itoa(num, 10));				// наличие команд в очереди
	}
	else
	{
		ret = false;
	}

	if (ret)
	{
		strcat(pack, js_end);
		xQueueSend(xStateQueue, pack, pdMS_TO_TICKS(50));
	}
	return ret;
}

void calc_dist(uint16_t count)
{
	esensor.value = count / 58;	// расстояние в см
	if (esensor.value > 400)	// больше 4-х метров - далеко
	{
		esensor.value = 2048;
	}
	esensor.checked =  false;
}

uint8_t sendPackToBLE(const char * blepack)
{
	if ( xSemaphoreTake(xSendPackMutex, pdMS_TO_TICKS(500)))
	{
		send_usart_dma(blepack);
		xSemaphoreGive(xSendPackMutex);
		return 1;
	}
	else
	{
		return 0;
	}
}

// ==============================================================
// формирование одиночного импульса 10мкС для запуска эхо-сенсора
void echo_pulse(void)
{
	//TIM4_ARR = 11;
	timer_set_period(TIM4, 11);
	//TIM4_CCR4 = 2;
	timer_set_oc_value(TIM4, TIM_OC4, 2);
	//TIM4_CR1 |= TIM_CR1_CEN;
	timer_enable_counter(TIM4);
	//printf("echo pulse");
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

	//TIM1_CR1 &= ~TIM_CR1_CEN;
	timer_enable_counter(TIM1);

	// за какое время получили отклик?
	return (xLastWakeTime2 - xLastWakeTime1);
}

// вычисление основных параметров по результатам цикла АЦП
void ADC_calc(void)
{
	uint16_t delta;
	//printf("ADC_calc");
	// 1-е измерение по каждому каналу отбрасываем, 2-е и 3-е усредняются
	uint16_t v = (ADCbuffer[10] + ADCbuffer[11]) >> 1;
	if (abs(adcval[0].value - v) > 7)
	{
		adcval[0].checked = false;
	}
	adcval[0].value = v;
	v = (ADCbuffer[1] + ADCbuffer[2]) >> 1;
	if (abs(adcval[1].value - v) > 7)
	{
		adcval[1].checked = false;
	}
	adcval[1].value = v;

	if (adcval[1].value < adcval[0].value)
	{
		delta = adcval[1].value - adcval[0].value;
	}
	else
	{
		delta = 0;
	}

	v = (ADCbuffer[7] + ADCbuffer[8]) >> 1;
	if (abs(adcval[2].value - v) > 7)
	{
		adcval[2].checked = false;
	}
	adcval[2].value = v;
	v = (ADCbuffer[4] + ADCbuffer[5]) >> 1;
	if (abs(adcval[2].value - v) > 7)
	{
		adcval[3].checked = false;
	}
	adcval[3].value = v;

	xEventGroupClearBits(xEventGroupADC, (const EventBits_t)0xFF);

	if (!init_process)
	{
		if ( adcval[0].value < BATT_LOW_LEVEL )
		{
			// батарея контроллера разряжена
			if ( adcval[0].value < BATT_OFF_LEVEL )
			{
				// требуется аварийное отключение
				//flag_power_off = 1;
				//xEventGroupSetBits( xEventGroupADC, alarm_CONTRL_OFF_VOLTAGE_BIT);
			}
			xEventGroupSetBits( xEventGroupADC, alarm_CONTRL_LOW_VOLTAGE_BIT);
		}

		if ( adcval[1].value < BATT_LOW_LEVEL )
		{
			// батарея моторов разряжена
			if ( adcval[1].value < BATT_OFF_LEVEL )
			{
				// требуется аварийное отключение
				//flag_power_off = 1;
				//xEventGroupSetBits( xEventGroupADC, alarm_MOTORS_OFF_VOLTAGE_BIT);
			}
			xEventGroupSetBits( xEventGroupADC, alarm_MOTORS_LOW_VOLTAGE_BIT);
		}

		if ( delta > LOAD_MAX )
		{
			// слишком большой ток
			xEventGroupSetBits( xEventGroupADC, alarm_OVERLOAD_BIT);
		}
	}
}

void running_delay(uint32_t lasting, uint8_t c_status)
{
	uint32_t st_new;
	int32_t t_max;

	/* если глобальный статус CNT_OFF, то датчик пути отсутствует
	if (cnt_status == CNT_OFF)
	{
		c_status = CNT_OFF;
	}
	*/
	if ( c_status == CNT_ON_STEPS )
	{
		st_new = ir_count + lasting;
		t_max = (int32_t)(lasting * MS_PER_STEP);	// максимальное время для прохода
		// цикл до получения нужного количества шагов или до истечения времени
		while ((ir_count < st_new) && (t_max > 0))
		{
			vTaskDelay(pdMS_TO_TICKS(20));
			t_max -= 20;
		}
	}
	else	// CNT_OFF или CNT_ON_MS
	{
		vTaskDelay(pdMS_TO_TICKS(lasting));
	}
}

void send_usart_dma(const char * pack)
{
	uint8_t len = strlen(pack);
	if (len > BLE_PACK_SIZE)
	{
		len = BLE_PACK_SIZE;
	}
	strncpy(TxBuffer, pack, len);

    dma_channel_reset(DMA1, DMA_CHANNEL4); 
    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&USART1_DR);
    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)TxBuffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL4, len);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL4, DMA_CCR_PL_HIGH);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

    dma_enable_channel(DMA1, DMA_CHANNEL4);
    usart_enable_tx_dma(USART1);
 
 	xSemaphoreTake(xSemaphSent, pdMS_TO_TICKS(100));
	xSemaphoreTake(xSemaphSendErr, 0);
	dma_disable_channel(DMA1, DMA_CHANNEL4);
}

char* itoa(int val, int base) {
	
	static char buf[32] = {0};
	
	int i = 30;
	
	for(; val && i ; --i, val /= base)
	
		buf[i] = "0123456789abcdef"[val % base];
	
	return &buf[i+1];
	
}
