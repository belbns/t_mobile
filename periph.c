/*

 * periph.c
 *
 *  Created on: 1 мар. 2017 г.
 *      Author: nb
 */

#include <stdlib.h>

#include <libopencm3/stm32/timer.h>

#include "periph.h"
/*
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
*/
/* длительность полупериода мигания светодиодов (в mS)
 * полупериод показывает состояние устройства:
 * 1000 mS - устройство готово к приему команд
 * 100 mS  - инициализация
 * 500 mS - ошибка ...
 */

void oneStep(uint8_t pos, uint8_t stpr);
void angle_update(uint8_t stnum);

extern EventGroupHandle_t xEventGroupADC;
extern EventGroupHandle_t xEventGroupDev;
extern SemaphoreHandle_t xSteppMutex[2]; 

const uint8_t motor_gear[4] = {K_GEAR0, K_GEAR1, K_GEAR2, K_GEAR3};

motor_ctrl motors = {
    .state = MOTOR_STOPPED, .curr_gear = GEAR_0, .gear1 = GEAR_0, .gear2 = GEAR_0
};

stepper stepp[2] = {
    { STEP_OFF, 0, 0, 0, 0, 0, 0},
	{ STEP_OFF, 0, 0, 0, 0, 0, 0}
};


mob_leds leds[4] = {
    { .port = LED0_PORT, .pin = LED0_PIN, .on = 0, .mode = LED_OFF},
	{ .port = LED1_PORT, .pin = LED1_PIN, .on = 0, .mode = LED_OFF},
	{ .port = LED2_PORT, .pin = LED2_PIN, .on = 0, .mode = LED_OFF},
	{ .port = LED3_PORT, .pin = LED3_PIN, .on = 0, .mode = LED_OFF},
};

servo_drive servo = {SERVO_ON, 90, 90};

uint8_t axle_stepper = AXLE_STEPPER;	// номер ШД эхо-локатора

uint8_t axle_search = 0;

const uint16_t step_pins[2][4] = {
    { STEP1_PIN1, STEP1_PIN2, STEP1_PIN3, STEP1_PIN4 },
	{ STEP2_PIN1, STEP2_PIN2, STEP2_PIN3, STEP2_PIN4 }
};

static uint8_t st_fast_ret = 0;

/*
 * Callback функция системного таймера.
 * Используется для тактирования шаговых двигателей
 *
*/
void vApplicationTickHook( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint8_t stepp_count = 0;

    // тактирование ШД0 - по четным тактам, ШД1 - по нечетным.
    uint8_t i = stepp_count & 1;
    EventBits_t evStep = dev_STEPP1_BIT << i;   // если 1 -> dev_STEPP2_BIT
    // захватываем ШД
    if ( xSemaphoreTakeFromISR( xSteppMutex[i], &xHigherPriorityTaskWoken )== pdPASS )
    {
        stepp_count++;

        if ( stepp[i].last_step )  // отработан последний шаг
        {
            if (stepp[i].mode != STEP_MAN_CONT)
            {
                if ((stepp[i].turns != 0) && (st_fast_ret != 0))
                {
                    stepp[i].steps = STEPPER_ANGLE_TURN << 3; // 512 * 8
                }
                else
                {
                    stepp_stop(i);
                    stepp[i].moving = 0;
                    st_fast_ret = 0;
                }
                xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
            }
            else    // непрерывное движение
            {
                stepp[i].steps = STEPPER_ANGLE_TURN << 3; // 512 * 8
            }
            stepp[i].last_step = 0;
                
            // приращиваем текущий угол - последние 8 тактов учитываются здесь
            angle_update(i);
            xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
        }
        else if ( stepp[i].steps > 0 )  // еще есть куда шагать
        {
            uint8_t pos = (uint8_t)(stepp[i].steps & 0x07); // младшие 3 бита определяют комбинацию

            // при переходе pos через 0 (отработано 8 тактов) выполняется приращение
            // текущего угиа ШД. Для исключения исходного нуля stepp[i].moving переводится
            // в 1 после обработки 1 такта.
            if ((pos == 0) && stepp[i].moving)
            {
                angle_update(i);
                xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
            }

            if ( stepp[i].clockw )
            {
                pos = 7 - pos;
            }

            oneStep(pos, i);
                        
            stepp[i].steps--;
            if ( stepp[i].steps == 0 )
            {
                stepp[i].last_step = 1;
            }
                        
            // чтобы не было приращения на начальном нуле
            // признак движения устанавливаем после обработки 1-го шага
            stepp[i].moving = 1;
            xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
        }
        xSemaphoreGiveFromISR( xSteppMutex[i], &xHigherPriorityTaskWoken );
    }
}

// инкремент/декремент угла ШД
void angle_update(uint8_t stnum)
{
    if ( stepp[stnum].clockw )  // по ч.с. 0, 511, 510, ..., 1, 0, 511
    {
        if (stepp[stnum].angle == 0)
        {
            stepp[stnum].angle = STEPPER_ANGLE_TURN - 1;
            stepp[stnum].turns--;
        }
        else
        {
            stepp[stnum].angle--;
        }
    }
    else    // против ч.с. 0, 1, 2, ..., 511, 0
    {
        if (stepp[stnum].angle == (STEPPER_ANGLE_TURN - 1))
        {
            stepp[stnum].angle = 0;   
            stepp[stnum].turns++;
        }
        else
        {
            stepp[stnum].angle++;
        }                
    }
}

/*
    Установка режима ДПТ, таймер TIM3 в режиме PWM1 управляет моторами через L293D.
    Для ДПТ1 используются каналы TIM_OC1 и TIM_OC2,
    для ДПТ2 - TIM_OC3 и TIM_OC4.
*/
void set_motor_value(uint8_t mmask, int8_t sgear, int8_t sgear1, int8_t sgear2)
{
	uint16_t pulse1 = 0;
	uint16_t pulse2 = 0;
    uint8_t need_update1 = 0;
    uint8_t need_update2 = 0;

    if (abs(sgear) != GEAR_NOP) // GEAR_NOP - не менять
    {
        motors.curr_gear = sgear;
    }
    
    if ( mmask & 1 )
    {
        need_update1 = 1;
        motors.gear1 = sgear1;
    }
    if ( mmask & 2 )
    {
        need_update2 = 1;
        motors.gear2 = sgear2;
    }
    
    if (need_update1)
    {
        // в 0 для исключения сквозных токов, а также при 0-й скорости 
        timer_set_oc_value(TIM3, TIM_OC1, 0);
        timer_set_oc_value(TIM3, TIM_OC2, 0);

        if (motors.gear1 != 0)
        {
            pulse1 = motor_gear[abs(motors.gear1) & 3] << 4;   // K_GEARx * 16
            if (pulse1 > PWM_MAX_VALUE)
            {
                pulse1 = PWM_MAX_VALUE;
            }

            if (motors.gear1 > 0)
            {
                // ждем 0 на TIM3->CCR2
                while (gpio_get(MOTOR1_PORT, MOTOR1_PIN2) != 0) ;
                timer_set_oc_value(TIM3, TIM_OC1, pulse1);
            }
            else // < 0
            {
                // ждем 0 на TIM3->CCR1
                while (gpio_get(MOTOR1_PORT, MOTOR1_PIN1) != 0) ;
                timer_set_oc_value(TIM3, TIM_OC2, pulse1);
            }
        }
    }

    if (need_update2)
    {
        // в 0 для исключения сквозных токов, а также при 0-й скорости 
        timer_set_oc_value(TIM3, TIM_OC3, 0);
        timer_set_oc_value(TIM3, TIM_OC4, 0);

        if (motors.gear2 != 0)
        {
            pulse2 = motor_gear[abs(motors.gear2) & 3] << 4;   // K_GEARx * 16
            if (pulse2 > PWM_MAX_VALUE)
            {
                pulse2 = PWM_MAX_VALUE;
            }
            if (motors.gear2 > 0)
            {
        	   // ждем 0 на TIM3->CCR4
                while (gpio_get(MOTOR2_PORT, MOTOR2_PIN2) != 0) ;
                timer_set_oc_value(TIM3, TIM_OC3, pulse2);
            }
            else // < 0
            {
        	   // ждем 0 на TIM3->CCR3
                while (gpio_get(MOTOR2_PORT, MOTOR2_PIN1) != 0) ;
                timer_set_oc_value(TIM3, TIM_OC4, pulse2);
            }
        }
    }

    xEventGroupSetBits(xEventGroupDev, dev_MOTORS_BIT);
}

void set_servo_angle(uint8_t angle)
{
	servo.angle_prev = servo.angle;
	servo.angle = angle;
    // для серво 360 - правое положение, 180 - левое, т.е. зеркально от пульта,
    // поэтому в таймер пишем 360 - angle
    timer_set_oc_value(TIM2, TIM_OC3, (uint16_t)(SERVO_180GRAD - angle  - 1 + SERVO_SHIFT));

    xEventGroupSetBits(xEventGroupDev, dev_SERVO_BIT);        
}

void servo_stop(void)
{
	servo.angle_prev = servo.angle;
	servo.angle = 90;
    timer_set_oc_value(TIM2, TIM_OC3, (uint16_t)servo.angle);
    xEventGroupSetBits(xEventGroupDev, dev_SERVO_BIT);        
}

/*
 * Поворот ШД на один шаг
 */
void oneStep(uint8_t pos, uint8_t stpr) {

    switch(pos)
    {
    case 0:
        gpio_clear(STEPP_PORT, step_pins[stpr][0]);
        gpio_clear(STEPP_PORT, step_pins[stpr][1]);
        gpio_clear(STEPP_PORT, step_pins[stpr][2]);
        gpio_set(STEPP_PORT, step_pins[stpr][3]);
        break;
    case 1:
        gpio_clear(STEPP_PORT, step_pins[stpr][0]);
        gpio_clear(STEPP_PORT, step_pins[stpr][1]);
        gpio_set(STEPP_PORT, step_pins[stpr][2]);
        gpio_set(STEPP_PORT, step_pins[stpr][3]);
        break;
    case 2:
        gpio_clear(STEPP_PORT, step_pins[stpr][0]);
        gpio_clear(STEPP_PORT, step_pins[stpr][1]);
        gpio_clear(STEPP_PORT, step_pins[stpr][3]);
        gpio_set(STEPP_PORT, step_pins[stpr][2]);
        break;
    case 3:
        gpio_clear(STEPP_PORT, step_pins[stpr][0]);
        gpio_clear(STEPP_PORT, step_pins[stpr][3]);
        gpio_set(STEPP_PORT, step_pins[stpr][1]);
        gpio_set(STEPP_PORT, step_pins[stpr][2]);
        break;
    case 4:
        gpio_clear(STEPP_PORT, step_pins[stpr][0]);
        gpio_clear(STEPP_PORT, step_pins[stpr][2]);
        gpio_clear(STEPP_PORT, step_pins[stpr][3]);
        gpio_set(STEPP_PORT, step_pins[stpr][1]);
        break;
    case 5:
        gpio_clear(STEPP_PORT, step_pins[stpr][2]);
        gpio_clear(STEPP_PORT, step_pins[stpr][3]);
        gpio_set(STEPP_PORT, step_pins[stpr][0]);
        gpio_set(STEPP_PORT, step_pins[stpr][1]);
        break;
    case 6:
        gpio_clear(STEPP_PORT, step_pins[stpr][1]);
        gpio_clear(STEPP_PORT, step_pins[stpr][2]);
        gpio_clear(STEPP_PORT, step_pins[stpr][3]);
        gpio_set(STEPP_PORT, step_pins[stpr][0]);
        break;
    case 7:
        gpio_clear(STEPP_PORT, step_pins[stpr][1]);
        gpio_clear(STEPP_PORT, step_pins[stpr][2]);
        gpio_set(STEPP_PORT, step_pins[stpr][0]);
        gpio_set(STEPP_PORT, step_pins[stpr][3]);
        break;
    }
}

void stepp_stop(uint8_t stnum)
{
    EventBits_t evStep = dev_STEPP1_BIT << stnum;   // если 1 -> dev_STEPP2_BIT
	if ( stnum < 2 )
	{
		gpio_clear(STEPP_PORT, step_pins[stnum][0]);	// MS0_PIN
		gpio_clear(STEPP_PORT, step_pins[stnum][1]);	// MS1_PIN
		gpio_clear(STEPP_PORT, step_pins[stnum][2]);	// MS2_PIN
		gpio_clear(STEPP_PORT, step_pins[stnum][3]);	// MS3_PIN

        stepp[stnum].last_step = 0;
        stepp[stnum].steps = 0;
        xEventGroupSetBits(xEventGroupDev, evStep); 
	}
}

// доводит положение ШД до точки 0/512, 
// последующее движение возобновляется в vApplicationTickHook
void stepp_start_cont(uint8_t stnum, uint8_t cmd)
{
    EventBits_t evStep = dev_STEPP1_BIT << stnum;   // если 1 -> dev_STEPP2_BIT

    stepp[stnum].steps = stepp[stnum].steps & 7;
    // ждем окончания движения
    while ( (stepp[stnum].steps > 0) || stepp[stnum].last_step )
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    stepp[stnum].mode = STEP_MAN_CONT;

    // Вычисляем кол-во шагов до точки 0/512
    if (cmd == ST_CONT_CCLK)
    {
        stepp[stnum].clockw = 0;
        stepp[stnum].steps = (STEPPER_ANGLE_TURN - stepp[stnum].angle - 1) << 3;
    }
    else
    {
        stepp[stnum].clockw = 1;
        if (stepp[stnum].angle > 0)
        {
            stepp[stnum].steps = stepp[stnum].angle << 3;    
        }
        else
        {
            stepp[stnum].steps = STEPPER_ANGLE_TURN << 3;
        }
    }

    xEventGroupSetBits(xEventGroupDev, evStep);
}


// процедура должна вызываться при захваченом xSteppMutex[stnum]
void stepp_clk_cclk(uint8_t stnum, int16_t sparam)
{
    EventBits_t evStep = dev_STEPP1_BIT << stnum;   // если 1 -> dev_STEPP2_BIT

	// ждем окончания движения
	while ( (stepp[stnum].steps > 0) || stepp[stnum].last_step )
	{
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	uint32_t steps = ((uint32_t)abs(sparam));

	if (sparam < 0)
	{
		stepp[stnum].clockw = 1;
	}
	else
	{
		stepp[stnum].clockw = 0;	// clockwise
	}
	stepp[stnum].steps = steps << 3;	// *8

    xEventGroupSetBits(xEventGroupDev, evStep);        
}

// возврат ШД в нулевое положение
void stepp_to_null(uint8_t stnum, uint8_t fast)
{
    EventBits_t evStep = dev_STEPP1_BIT << stnum;   // если 1 -> dev_STEPP2_BIT

	if ( xSemaphoreTake( xSteppMutex[stnum], pdMS_TO_TICKS(100)))
	{
		stepp[stnum].mode = STEP_MAN;

		// ждем окончания движения
		while ( (stepp[stnum].steps > 0) || stepp[stnum].last_step )
		{
			vTaskDelay(pdMS_TO_TICKS(10));
		}

        // определяем возврат направление и кол-во шагов для возврата в 0.
        // возврат оборотов определяется флагом fast и выполняется в vApplicationTickHook.
        st_fast_ret = fast;
        if ((stepp[stnum].turns > 0) || (stepp[stnum].clockw == 0))
        { // есть обороты против ч.с. или последнее движение было против ч.с
            stepp[stnum].clockw = 1; // возврат в 0 по ч.с.
            stepp[stnum].steps = stepp[stnum].angle << 3; // до нуля
        }
        else if ((stepp[stnum].turns < 0) || (stepp[stnum].clockw == 1))
        { // есть обороты по ч.с. или последнее движение было по ч.с
            stepp[stnum].clockw = 0; // возврат в 0 против ч.с.
            stepp[stnum].steps = (STEPPER_ANGLE_TURN - stepp[stnum].angle) << 3;
        }
        xEventGroupSetBits(xEventGroupDev, evStep);        
        xSemaphoreGive( xSteppMutex[stnum] );
	}
}

void set_led(uint8_t num, uint8_t value)
{
	uint8_t n = num & 0x03;
    EventBits_t evLed = dev_LED0_BIT << n;   // LED0..LED3

	switch (value)
	{
		case 0:
	    		gpio_clear(leds[n].port, leds[n].pin);
	    		leds[n].on = 0;
	            leds[n].mode = LED_OFF;
	            break;
		case 1:
	    		gpio_set(leds[n].port, leds[n].pin);
	    		leds[n].on = 1;
	            leds[n].mode = LED_ON;
	            break;
		case 2:
	    		leds[n].mode = LED_BLINK;
	    		leds[n].on = 1;
	            break;
		default:
	    		leds[n].mode = LED_OFF;
	    		leds[n].on = 0;
	            gpio_clear(leds[n].port, leds[n].pin);
	}
	//leds[n].checked = false;
    xEventGroupSetBits(xEventGroupDev, evLed);        
}

void stop_all(void)
{
	set_motor_value(3, GEAR_0, GEAR_0, GEAR_0);
	stepp_stop(0);
	stepp_stop(1);
	servo_stop();
}

void power_off(void)
{
	stop_all();

	// отключаем 6V
	gpio_clear(POWER_HOLD_PORT, POWER_6V_PIN);

	// отключаем 3.3V
	gpio_clear(POWER_HOLD_PORT, POWER_HOLD_PIN);

	for (;;) { }
}
