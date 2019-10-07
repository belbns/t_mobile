#ifndef PERIPH_H
#define PERIPH_H

#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/stm32/gpio.h>

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
#define SERVO_PIN			GPIO2	// выход таймера TIM2, CH3
#define SERVO_PORT			GPIOA
#define SERVO_0GRAD			(18 - 1)
#define SERVO_90GRAD		(27 - 1)
#define SERVO_180GRAD		(36 - 1)

// внешние светодиоды
#define LED0_PIN        	GPIO13	// на плате запаян PB3
#define LED0_PORT       	GPIOC
#define LED1_PIN        	GPIO4
#define LED1_PORT       	GPIOB
#define LED2_PIN        	GPIO14
#define LED2_PORT       	GPIOB
#define LED3_PIN        	GPIO15
#define LED3_PORT       	GPIOA

//#define BOARD_LED_PIN   	GPIO13
//#define BOARD_LED_PORT  	GPIOC

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
#define state_QUEUE_LEN		( 10 )	// длина очереди команд

#define CHAN_P6				ADC_CHANNEL_1	// сенсоры на разъеме P6
#define CHAN_3V3		    ADC_CHANNEL_0	// напряжение на ключе 3.3V
#define CHAN_P5				ADC_CHANNEL_3	// сенсоры на разъеме P5
#define CHAN_6V				ADC_CHANNEL_4	// напряжение на ключе 6V

#define BLE_PACK_SIZE		20
#define BLE_SEND_DELAY		10		// мин. интервал между передачами пакетов, mS
//#define JFES_MAX_TOKENS_COUNT		16	-- определено в jfes.h

// биты для xEventGroupBLE - события, связанные с BLE
#define ble_RECIEVED_JSON		(1UL << 0UL)
#define ble_DISCONNECTED		(1UL << 1UL)

// биты для xEventGroupADC - события, определяемые по результатам АЦП
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

// АЦП выдает по питанию приблизительно 617 единиц на 1 вольт
#define BATT_LOW_LEVEL		1720	// порог предупреждения о разряде батареи - 11.1V
#define BATT_OFF_LEVEL		1580	// порог аварийного отключения по разряду батареи 10.2V

#define SENS_LEVEL_4		2000
#define SENS_LEVEL_5		2171

#define SENS_LEVEL_8		2700
#define SENS_LEVEL_9		2986

// корректировка положения оси - пока не используется
#define AXLE_CORR			80

// порог обнаружения нагрузки
#define LOAD_DETECT		30

// значение motors_delta, при котором определяется недопустимая нагрузка
#define LOAD_MAX		1000	// ???????????????????
// максимальное отклонение от оси
#define STEPPER_ANGLE 		32
#define STEPPER_ANGLE_TURN	512 	// угол 1 оборота = 512 ( * 8 = 4096 )
#define STEPPER_TURNS_DIV	9		// степень двойки в 512
#define STEPPER_TURN_MASK	0x1FF	// остаток от делния на 512
#define AXLE_STEPPER		0		// ШД эхо-локатора

// для управления ДПТ значение в команде может быть от 0 до +/- MOTOR_MAX_VALUE
// в таймере значение умножается на 16 (0..PWM_MAX_VALUE-1)
#define MOTOR_MAX_VALUE	32
#define PWM_MAX_VALUE	511

// байт ошибок
#define ERR_BATT_CTRL_LOW	0x80
#define ERR_BATT_MOTOR_LOW	0x40
#define ERR_OVERLOAD		0x20
#define ERR_ESENSOR_OFF		0x10
#define ERR_STEP0_OFF		0x08
#define ERR_STEP1_OFF		0x04
//#define ERR_MOT1_OFF		0x02
#define ERR_MOT_OFF			0x01

// байт статусов
#define ESENSOR_QUEUE		0x20
#define MOTORS_QUEUE		0x10
#define CMD_BUFF_IS_FULL	0x08
#define AXLE_SEARCHING		0x04
#define STEPP1_QUEUE		0x02
#define STEPP0_QUEUE		0x01

#define MOTOR_START_TIME	50	// время на трогание мотора, mS
#define MOTOR_GEAR0			0
#define MOTOR_GEAR1			16
#define MOTOR_GEAR2			24
#define MOTOR_GEAR3			32

// кол-во циклов vTaskMain для отключения при потере связи
#define WAIT_FOR_PACK		500000

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
	STATE_PACK_DIST,
	STATE_PACK_ADC,
	STATE_PACK_QUEUE_CMD,
	STATE_PACK_QUEUE_MOT,
	STATE_PACK_QUEUE_ST1,
	STATE_PACK_QUEUE_ST2,
	STATE_PACK_QUEUE_SERVO,
	STATE_PACK_QUEUE_DIST,
	STATE_PACK_QUEUE_LEDS
};

enum {
	GEAR_0 = 0,
	GEAR_1,
	GEAR_2,
	GEAR_3
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
	ST_UP_DOWN = 0,
	ST_NULL,
	ST_NULL_FAST,
	ST_STOP,
	ST_CONT_LEFT,
	ST_CONT_RIGHT,
	ST_CONT_STOP,
	ST_RET,
	ST_AXLE,
	ST_PAUSE,
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

// команды очереди DIST
enum {
	DIST_SHOT = 0,
	DIST_PAUSE
};

// команды общей очереди
enum {
	NO_COMMAND = 0,
	PAUSE,		// пауза отдельной командой
	POWER_OFF,
	STOP_ALL,
	CNT_SET_OFF,
	CNT_SET_ON_MS,
	CNT_SET_ON_STEPS,
	CNT_RESET
};

typedef struct _ncommand_item {
	uint16_t cmd;
	int16_t param;
} ncommand_item;


enum {
        MOTOR_STOPPED = 0,
        MOTOR_UP,
        MOTOR_DOWN,
		MOTOR_LEFT,
        MOTOR_RIGHT,
        MOTOR_ALARM,
        MOTOR_OFF
};

typedef struct _motor_ctrl {
        uint8_t state;			// enum
        int8_t gear;
        int8_t value1;
        int8_t value2;
        int8_t pre_value;
        uint8_t need_update1;
        uint8_t need_update2;
        bool checked;
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
        bool checked;
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
    	bool checked;
} stepper;

typedef struct _mob_leds {
	uint32_t port;
	uint16_t pin;
	uint8_t on;
	uint8_t mode;
	bool checked;
} mob_leds;


typedef struct _voltage {
	uint16_t motors;			// напряжение на батарее моторов
	uint16_t delta;				// разность напряжений на входах БП на 3.3 и 6 вольт
	uint16_t ctrl;				// напряжение на батарее контроллера
	uint16_t P5;				// напряжение от датчиков P5
	uint16_t P6;				// напряжение от датчиков P6
	bool checked;
} voltage;

typedef struct _adcvalue {
	uint16_t value;
	bool checked;
} adcvalue;

typedef struct _queuestat {
	uint16_t value;
	bool checked;
} queuestat;

enum {
	SERVO_OFF = 0,
	SERVO_ON
};

typedef struct _servo_drive {
	uint8_t stat;
	uint8_t angle;
	uint8_t angle_prev;
	bool checked;
} servo_drive;

void vApplicationTickHook( void );

void set_motor_value(uint8_t mmask, int8_t value0, int8_t value1);
void stepp_stop(uint8_t stnum);
void set_servo_angle(uint8_t angle);
void servo_stop(void);
void stepp_to_null(uint8_t stnum, uint8_t fast);
void stepp_up_down(uint8_t stnum, int16_t sparam);
void set_led(uint8_t num, uint8_t value);
void stop_all(void);
void power_off(void);

#endif
