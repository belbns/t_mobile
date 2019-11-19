#ifndef PERIPH_H
#define PERIPH_H

#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/stm32/gpio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "timers.h"


// выключение питания по падающему фронту
#define POWEROFF_PORT	    GPIOA
#define POWEROFF_PIN	    GPIO12	// вход/прерывание
#define POWER_HOLD_PIN		GPIO8	// PORTA - удержание питания
#define POWER_6V_PIN		GPIO11	// PORTA - питание 6V
#define POWER_HOLD_PORT		GPIOA

// эхо-сенсор - разъем P4
#define ECHO_TRIG_PIN		GPIO9	// выход
#define ECHO_ECHO_PIN       GPIO15	// вход/прерывание
#define ECHO_PORT		    GPIOB

// счетчик пути - разъем P3
#define IR_COUNT_PIN		GPIO5	// вход/прерывание
#define IR_COUNT_PORT		GPIOA

// серво-привод - разъем P3
#define SERVO_PIN			GPIO2		// выход таймера TIM2, CH3
#define SERVO_PORT			GPIOA
// для серво 360 - правое положение, 180 - левое, т.е. зеркально от пульта
#define SERVO_0GRAD			(180 - 1)
#define SERVO_90GRAD		(270 - 1)
#define SERVO_180GRAD 		(360 - 1)
#define SERVO_SHIFT			0			// смещение - по результатам калибровки

// внешние светодиоды
#define LED0_PIN        	GPIO3
#define LED0_PORT       	GPIOB
#define LED1_PIN        	GPIO4
#define LED1_PORT       	GPIOB
#define LED2_PIN        	GPIO15
#define LED2_PORT       	GPIOA
#define LED3_PIN        	GPIO14
#define LED3_PORT       	GPIOB

#define BOARD_LED_PIN   	GPIO13
#define BOARD_LED_PORT  	GPIOC

#define STEPP_PORT			GPIOB

#define STEP1_PIN1      	GPIO10
#define STEP1_PIN2      	GPIO11
#define STEP1_PIN3      	GPIO12
#define STEP1_PIN4      	GPIO13

#define STEP2_PIN1      	GPIO5
#define STEP2_PIN2      	GPIO6
#define STEP2_PIN3      	GPIO7
#define STEP2_PIN4      	GPIO8

#define MOTOR1_PIN1			GPIO6
#define MOTOR1_PIN2			GPIO7
#define MOTOR1_PORT			GPIOA

#define MOTOR2_PIN1			GPIO0
#define MOTOR2_PIN2			GPIO1
#define MOTOR2_PORT			GPIOB

#define serRX_QUEUE_LEN		( 32 )	// длина очереди приема из UART
#define cmd_QUEUE_LEN		( 10 )	// длина очереди команд
#define state_QUEUE_LEN		( 40 )	// длина очереди команд

#define CHAN_P6				ADC_CHANNEL_1	// сенсоры на разъеме P6
#define CHAN_3V3		    ADC_CHANNEL_0	// напряжение на ключе 3.3V / 4.64 (12.29v -> 3826)
#define CHAN_P5				ADC_CHANNEL_3	// сенсоры на разъеме P5
#define CHAN_6V				ADC_CHANNEL_4	// напряжение на ключе 6V / 4.64

#define BLE_PACK_SIZE		20
#define BLE_SEND_DELAY		10		// мин. интервал между передачами пакетов, mS
//#define JFES_MAX_TOKENS_COUNT		16	-- определено в jfes.h

// биты для xEventGroupBLE - события, связанные с BLE
#define ble_RECIEVED_JSON		(1UL << 0UL)
#define ble_DISCONNECTED		(1UL << 1UL)

// биты для xEventGroupAlrm - события от внешних сигналов и измерений
#define alarm_POWER_BUTTON_BIT			(1UL << 0UL)
#define alarm_CONTRL_LOW_VOLTAGE_BIT	(1UL << 1UL)
#define alarm_CONTRL_OFF_VOLTAGE_BIT	(1UL << 2UL)
#define alarm_MOTORS_LOW_VOLTAGE_BIT	(1UL << 3UL)
#define alarm_MOTORS_OFF_VOLTAGE_BIT	(1UL << 4UL)
#define alarm_OVERLOAD_BIT				(1UL << 5UL)
#define alarm_HALL1_BIT					(1UL << 6UL)
#define alarm_HALL2_BIT					(1UL << 7UL)
#define alarm_HALL3_BIT					(1UL << 8UL)
#define alarm_HALL4_BIT					(1UL << 9UL)
#define alarm_POWER_COMMAND_BIT			(1UL << 10UL)

// биты для xEventGroupDev - необходимость передачи статуса устройств
#define dev_ADC0_BIT			(1UL << 0UL)
#define dev_ADC1_BIT			(1UL << 1UL)
#define dev_ADC2_BIT			(1UL << 2UL)
#define dev_ADC3_BIT			(1UL << 3UL)
#define dev_LED0_BIT			(1UL << 4UL)
#define dev_LED1_BIT			(1UL << 5UL)
#define dev_LED2_BIT			(1UL << 6UL)
#define dev_LED3_BIT			(1UL << 7UL)
#define dev_MOTORS_BIT			(1UL << 8UL)
#define dev_STEPP1_BIT			(1UL << 9UL)
#define dev_STEPP2_BIT			(1UL << 10UL)
#define dev_SERVO_BIT			(1UL << 11UL)
#define dev_ECHO_BIT			(1UL << 12UL)


// АЦП val = U * 1000 / 4.64
#define BATT_LOW_LEVEL		2390	// порог предупреждения о разряде батареи - 11.1V
#define BATT_OFF_LEVEL		2200	// порог аварийного отключения по разряду батареи 10.2V

#define SENS_LEVEL_4		2000	// ?
#define SENS_LEVEL_5		2171	// ?

#define SENS_LEVEL_8		2700	// ?
#define SENS_LEVEL_9		2986	// ?

// корректировка положения оси - пока не используется
#define AXLE_CORR			80		// ?

// порог обнаружения нагрузки
#define LOAD_DETECT		30			// ?

// значение motors_delta, при котором определяется недопустимая нагрузка
#define LOAD_MAX		50	// ???????????????????
// максимальное отклонение от оси
#define STEPPER_ANGLE 		32		// для эхо-локатора
#define STEPPER_ANGLE_TURN	512 	// угол 1 оборота = 512 ( * 8 = 4096 )
#define STEPPER_TURNS_DIV	9		// степень двойки в 512
#define STEPPER_TURN_MASK	0x1FF	// остаток от делния на 512
#define AXLE_STEPPER		0		// ШД эхо-локатора

// для управления ДПТ значение в команде может быть от 0 до +/- MOTOR_MAX_VALUE
// в таймере значение умножается на 16 (0..PWM_MAX_VALUE-1)
#define MOTOR_MAX_VALUE	32
#define PWM_MAX_VALUE	511

#define MOTOR_START_TIME	50	// время на трогание мотора, mS

// значения коэффициентов скоростей ДПТ
// - при записи TIM_OCX таймера будут умножаться на 16
#define K_GEAR0			0
#define K_GEAR1			16
#define K_GEAR2			24
#define K_GEAR3			32

// Максимальное время отутствия пакетов от пульта в mS 
// - при достижениии формируется STOP_ALL
#define WAIT_FOR_PACK		600000	// 10 мин. - уменьшить после отладки

#define MIN_STEPS_PER_SEC	1	// кол-во импульсов в секунду датчика движения
								//  при самом медленном вращении колеса
// интервал между импульсами при самом медленном вращении колеса
#define MS_PER_STEP			( 1000 / MIN_STEPS_PER_SEC )


enum {
	STATE_PACK_READY = 0,
	STATE_PACK_BUSY,
	STATE_PACK_PWROFF_BAT,
	STATE_PACK_PWROFF_KEY,
	STATE_PACK_PWROFF_CMD,
	STATE_PACK_LOWBAT,
	STATE_PACK_OVERLOAD,
	STATE_PACK_MOTORS,
	STATE_PACK_STEPP,
	STATE_PACK_LED,
	STATE_PACK_SERVO,
	STATE_PACK_ECHO,
	STATE_PACK_ADC,
	STATE_PACK_QUEUE_CMD,
	STATE_PACK_QUEUE_MOT,
	STATE_PACK_QUEUE_ST1,
	STATE_PACK_QUEUE_ST2,
	STATE_PACK_QUEUE_SERVO,
	STATE_PACK_QUEUE_ECHO,
	STATE_PACK_QUEUE_LEDS,
	STATE_PACK_DBG
};

enum {
	CNT_OFF = 0,	// счетчик пути выключен
	CNT_ON_MS,		// включен, интервалы в mS
	CNT_ON_STEPS	// включен, интервалы в шагах счетчика
};

// команды очереди ДПТ
enum {
	MOT_STOP = 0,
	MOT_UP_DOWN,
	MOT_RIGHT,
	MOT_LEFT,
	MOT_STRAIGHT,
	MOT_PAUSE
};

// команды очереди ШД
enum {
	ST_SET_ANGLE = 0,
	ST_NULL,
	ST_NULL_FAST,
	ST_STOP,
	ST_CONT_CCLK,		// counterclockwise
	ST_CONT_CLK,		// clockwise
	ST_CONT_STOP,
	ST_RET,
	ST_AXLE,
	ST_PAUSE
};

// команды очереди LEDS
enum {
	LED0_MOD = 0,
	LED1_MOD,
	LED2_MOD,
	LED3_MOD,
	LEDS_PAUSE
};
// состояния светодиодов
enum {
	LED_OFF = 0,
    LED_ON,
    LED_BLINK
};

// команды очереди SERVO
enum {
	SERVO_SET = 0,
	SERVO_PAUSE
};

// команды очереди ECHO
enum {
	ECHO_SHOT = 0,
	ECHO_PAUSE
};

// команды общей очереди
enum {
	CMD_NO = 0,
	CMD_PAUSE,		// пауза отдельной командой
	CMD_POWER_OFF,
	CMD_STOP_ALL,
	CMD_CNT_SET_ON_MS,
	CMD_CNT_SET_ON_STEPS,
	CMD_CNT_RESET
};

typedef struct _ncommand_item {
	uint16_t cmd;
	int16_t param;
} ncommand_item;

enum {
	GEAR_0 = 0,
	GEAR_1,
	GEAR_2,
	GEAR_3,
	GEAR_NOP
};

enum {
	MOTOR_STOPPED = 0,
    MOTOR_UP,
    MOTOR_DOWN,
	MOTOR_LEFT,
    MOTOR_RIGHT,
    MOTOR_ALARM
};

typedef struct _motor_ctrl {
        uint8_t state;			// enum
        int8_t curr_gear;
        int8_t gear1;
        int8_t gear2;
//        uint8_t need_update1;
//        uint8_t need_update2;
        //bool checked;
} motor_ctrl;

enum {
        ECHO_OFF = 0,
        ECHO_AUTO,
		ECHO_ONE
};

typedef struct _echo_sensor {
        uint16_t value;			// есть обновленная инфо
        uint8_t state;           // ON-OFF
        uint8_t shot;
        //bool checked;
} echo_sensor;

enum {
        STEP_MAN = 0,
        STEP_MAN_CONT,
        STEP_OFF
};

typedef struct _stepper {
        uint8_t mode;
        int32_t angle_curr;		// текущий угол
        int32_t turns;			// количество оборотов
    	uint32_t steps;			// оставшееся количество шагов (*8) для выполнения
    	uint8_t clockw;			// направление поворота 1 - по часовой стрелке
    	uint8_t last_step;		// выполняется последний шаг
    	uint8_t moving;
    	//bool checked;
} stepper;

typedef struct _mob_leds {
	uint32_t port;
	uint16_t pin;
	uint8_t on;
	uint8_t mode;
	//bool checked;
} mob_leds;

enum {
	SERVO_OFF = 0,
	SERVO_ON
};

typedef struct _servo_drive {
	uint8_t stat;
	uint8_t angle;
	uint8_t angle_prev;
} servo_drive;

void vApplicationTickHook( void );

void set_motor_value(uint8_t mmask, int8_t sgear, int8_t sgear1, int8_t sgear2);
void stepp_stop(uint8_t stnum);
void set_servo_angle(uint8_t angle);
void servo_stop(void);
void stepp_to_null(uint8_t stnum, uint8_t fast);
void stepp_start_cont(uint8_t stnum, uint8_t cmd);
void stepp_clk_cclk(uint8_t stnum, int16_t sparam);
void set_led(uint8_t num, uint8_t value);
void stop_all(void);
void power_off(void);

#endif
