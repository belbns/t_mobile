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

//extern SemaphoreHandle_t xSteppMutex[2];
extern EventGroupHandle_t xEventGroupADC;
extern EventGroupHandle_t xEventGroupDev;
extern SemaphoreHandle_t xSteppMutex[2]; 

motor_ctrl motors = {
    .state = MOTOR_OFF, .gear = GEAR_0, .value1 = 0, .value2 = 0,
	.pre_value = 0, .need_update1 = 0, .need_update2 = 0   //, .checked = false
};

stepper stepp[2] = {
    { STEP_OFF, 0, 0, 0, 0, 0, 0/*, false*/ },
	{ STEP_OFF, 0, 0, 0, 0, 0, 0/*, false*/ }
};


mob_leds leds[4] = {
    { .port = LED0_PORT, .pin = LED0_PIN, .on = 0, .mode = LED_OFF/*, .checked = false*/},
	{ .port = LED1_PORT, .pin = LED1_PIN, .on = 0, .mode = LED_OFF/*, .checked = false*/},
	{ .port = LED2_PORT, .pin = LED2_PIN, .on = 0, .mode = LED_OFF/*, .checked = false*/},
	{ .port = LED3_PORT, .pin = LED3_PIN, .on = 0, .mode = LED_OFF/*, .checked = false*/},
};

servo_drive servo = {SERVO_ON, 90, 90/*, false*/};

uint8_t axle_stepper = AXLE_STEPPER;	// номер ШД эхо-локатора

uint8_t axle_search = 0;

const uint16_t step_pins[2][4] = {
    { STEP1_PIN1, STEP1_PIN2, STEP1_PIN3, STEP1_PIN4 },
	{ STEP2_PIN1, STEP2_PIN2, STEP2_PIN3, STEP2_PIN4 }
};

/*
void vApplicationMallocFailedHook( void )
{

}
*/
/*
 * Callback функция системного таймера.
 * Используется для тактирования шаговых двигателей
 *
*/
void vApplicationTickHook( void )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint8_t stepp_count = 0;

//xEventGroupSetBits(xEventGroupDev, (const EventBits_t)0x1FFF);

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
                stepp_stop(i);
                stepp[i].moving = 0;
                //stepp[i].checked = false;
                xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
            }
            else    // непрерывное движение
            {
                stepp[i].steps = (uint32_t)(STEPPER_ANGLE_TURN << 3); // 512 * 8
            }
            stepp[i].last_step = 0;
                
            // приращиваем текущий угол - последние 8 тактов учитываются здесь
            if ( stepp[i].clockw )
            {
                stepp[i].angle_curr++;
                //stepp[i].checked = false;
                xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
            }
            else
            {
                stepp[i].angle_curr--;
                //stepp[i].checked = false;
                xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
            }
        }
        else if ( stepp[i].steps > 0 )  // еще есть куда шагать
        {
            uint8_t pos = (uint8_t)(stepp[i].steps & 0x07); // младшие 3 бита определяют комбинацию

            // при переходе pos через 0 (отработано 8 тактов) выполняется приращение
            // текущего угиа ШД. Для исключения исходного нуля stepp[i].moving переводится
            // в 1 после обработки 1 такта.
            if ( stepp[i].clockw )
            {
                // при переходе через 0 (исключая начальный ноль) - приращение текущего угла
                if ((pos == 0) && stepp[i].moving)
                {
                    stepp[i].angle_curr++;
                    //stepp[i].checked = false;
                    xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
                }
                pos = 7 - pos;
            }
            else
            {
                // приращение текущего угла
                if ((pos == 0) && stepp[i].moving)
                {
                    stepp[i].angle_curr--;
                    //stepp[i].checked = false;
                    xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
                }
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
            //stepp[i].checked = false;
            xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
        }
        // контроль полного оборота
        if (stepp[i].angle_curr == STEPPER_ANGLE_TURN)
        {
            stepp[i].angle_curr = 0;
            stepp[i].turns++;
            //stepp[i].checked = false;
            xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
        }
        else if (stepp[i].angle_curr == -STEPPER_ANGLE_TURN)
        {
            stepp[i].angle_curr = 0;
            stepp[i].turns--;
            //stepp[i].checked = false;
            xEventGroupSetBitsFromISR(xEventGroupDev, evStep, &xHigherPriorityTaskWoken);
        }
        xSemaphoreGiveFromISR( xSteppMutex[i], &xHigherPriorityTaskWoken );
    }
}

/*
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
        // This function will get called if a task overflows its stack.   If the
        // parameters are corrupt then inspect pxCurrentTCB to find which was the
        // offending task.

        ( void ) pxTask;
        ( void ) pcTaskName;

        //trace_printf("Stack overflow, Task Name: %s", pcTaskName);
        gpio_set(BOARD_LED_PORT, BOARD_LED_PIN);
        for(uint32_t i = 0; i < 720000000; i++)	// 10c
        {
        	asm("nop");
        }
        gpio_clear(POWER_HOLD_PORT, POWER_HOLD_PIN);	// выключаемся
        for( ;; );
}
*/

/*
    Установка режима ДПТ, таймер TIM3 в режиме PWM1 управляет моторами через L293D.
    Для ДПТ1 используются каналы TIM_OC1 и TIM_OC2,
    для ДПТ2 - TIM_OC3 и TIM_OC4.
*/
void set_motor_value(uint8_t mmask, int8_t value0, int8_t value1)
{
	uint16_t pulse0 = 0;
	uint16_t pulse1 = 0;

    if ( mmask & 1 )
    {
    	// в 0 для исключения сквозных токов 
        timer_set_oc_value(TIM3, TIM_OC1, 0);
        timer_set_oc_value(TIM3, TIM_OC2, 0);

    	if (value0 != 0)
        {
        	pulse0 = abs(value0) & 0x3F;	// каждая 1 от пульта = 16 ступеней
        	pulse0 = pulse0 << 4;			// * 16
            if (pulse0 > PWM_MAX_VALUE)
            	pulse0 = PWM_MAX_VALUE;
        }
    	else
    	{
        	pulse0 = 0;
    	}
    	motors.need_update1 = 1;
        motors.value1 = value0;
    }
    else
    {
    	motors.need_update1 = 0;
    }

    if ( mmask & 2 )
    {
    	// в 0 для исключения сквозных токов
        timer_set_oc_value(TIM3, TIM_OC3, 0);
        timer_set_oc_value(TIM3, TIM_OC4, 0);

    	if (value1 != 0)
        {
        	pulse1 = abs(value1) & 0x3F;	// каждая 1 от пульта = 16 ступеней
        	pulse1 = pulse1 << 4;			// * 16
            if (pulse1 > PWM_MAX_VALUE)
            	pulse1 = PWM_MAX_VALUE;
        }
    	else
    	{
        	pulse1 = 0;
    	}
    	motors.need_update2 = 1;
        motors.value2 = value1;
    }
    else
    {
    	motors.need_update2 = 0;
    }

    if (motors.need_update1)
    {
        if (value0 > 0)
        {
        	// ждем 0 на TIM3->CCR2
            while (gpio_get(MOTOR1_PORT, MOTOR1_PIN2) != 0) ;
            //TIM3_CCR1 = pulse0;
            timer_set_oc_value(TIM3, TIM_OC1, pulse0);
        }
        else if (value0 < 0)
        {
        	// ждем 0 на TIM3->CCR1
            while (gpio_get(MOTOR1_PORT, MOTOR1_PIN1) != ) ;
            //TIM3_CCR2 = pulse0;
            timer_set_oc_value(TIM3, TIM_OC2, pulse0);
        }
        motors.need_update1 = 0;
    }

    if (motors.need_update2)
    {
        if (value1 > 0)
        {
        	// ждем 0 на TIM3->CCR3
            while (gpio_get(MOTOR2_PORT, MOTOR2_PIN2) != 0) ;
            //TIM3_CCR3 = pulse1;
            timer_set_oc_value(TIM3, TIM_OC3, pulse1);
        }
        else if (value1 < 0)
        {
        	// ждем 0 на TIM3->CCR4
            while (gpio_get(MOTOR2_PORT, MOTOR2_PIN1) != 0) ;
            //TIM3_CCR4 = pulse1;
            timer_set_oc_value(TIM3, TIM_OC4, pulse1);
        }
        motors.need_update2 = 0;
    }

    //motors.checked = false;
    xEventGroupSetBits(xEventGroupDev, dev_MOTORS_BIT);

}

void set_servo_angle(uint8_t angle)
{
	if (angle <= 180)
	{
		servo.angle_prev = servo.angle;
		servo.angle = angle;
        //TIM2_CCR3 = (uint16_t)(angle + SERVO_0GRAD);
        timer_set_oc_value(TIM2, TIM_OC3, (uint16_t)(angle + SERVO_0GRAD));
		//servo.checked = false;
        xEventGroupSetBits(xEventGroupDev, dev_SERVO_BIT);        
	}
}

void servo_stop(void)
{
	servo.angle_prev = servo.angle;
	servo.angle = SERVO_90GRAD;
    //TIM2_CCR3 = 0; ???
    timer_set_oc_value(TIM2, TIM_OC3, (uint16_t)servo.angle);
	//servo.checked = false;
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
        //stepp[stnum].checked = false;
        xEventGroupSetBits(xEventGroupDev, evStep); 
	}
}

// процедура должна вызываться при захваченом xSteppMutex[stnum]
void stepp_up_down(uint8_t stnum, int16_t sparam)
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
		stepp[stnum].clockw = 0;
	}
	else
	{
		stepp[stnum].clockw = 1;	// clockwise
	}
	stepp[stnum].steps = steps << 3;	// *8

	//stepp[stnum].checked = false;
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

		if (stepp[stnum].angle_curr != 0)
		{
			int32_t steps = 0;
			if (!fast)
			{
				steps = stepp[stnum].angle_curr;
			}
			else
			{
				steps = stepp[stnum].angle_curr % STEPPER_ANGLE_TURN;
				stepp[stnum].turns = 0;	// обнуляем обороты
			}
			if (steps > 0)
			{
				stepp[stnum].clockw = 0;
			}
			else
			{
				stepp[stnum].clockw = 1;
			}

			stepp[stnum].steps = ((uint32_t)abs(steps)) << 3;
			xSemaphoreGive( xSteppMutex[stnum] );
		}
		//stepp[stnum].checked = false;
        xEventGroupSetBits(xEventGroupDev, evStep);        
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
	set_motor_value(3, 0, 0);
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
