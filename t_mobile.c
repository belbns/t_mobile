#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/exti.h>
#include <libopencm3/stm32/f1/timer.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f1/dma.h>
#include <libopencm3/stm32/f1/usart.h>
#include <libopencm3/stm32/f1/adc.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/nvic.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"

#include "mytasks.h"
#include "periph.h"

static void clock_setup(void);
static void systick_setup(void);
static void gpio_setup(void);
static void usart_setup(void);
static void adc_setup(void);
static void tim1_setup(void);
static void tim2_setup(void);
static void tim3_setup(void);
static void tim4_setup(void);

uint64_t millis(void);
void delay(uint64_t duration);

// События
EventGroupHandle_t xEventGroupBLE;      // события, связанные с BLE
EventGroupHandle_t xEventGroupADC;      // события по результатам работы АЦП

// Семафоры
SemaphoreHandle_t xSemaphHandleEcho;    // есть результат от эхо-локатора
SemaphoreHandle_t xSemaphVoltage;       // завершено сканирование каналов
SemaphoreHandle_t xSemaphSent;          // пакет в BLE отправлен
SemaphoreHandle_t xSemaphSendErr;       // ошибка отправки
// Мьютексы
SemaphoreHandle_t xSteppMutex[2];       // для управления ШД
SemaphoreHandle_t xIRCountMutex;        // модификация счетчика пути
SemaphoreHandle_t xSendPackMutex;       // для передачи пакета пульту

// задачи
TaskHandle_t xTaskADC;                  // Опрос АЦП и формирование флагов событ
TaskHandle_t xTaskLedsBlink;            // управление светодиодами
TaskHandle_t xTaskBLE;                  // обмен с BLE
TaskHandle_t xTaskMain;                 // Главная задача
TaskHandle_t xTaskCmd;                  // Обработка общей очереди команд
// новые задачи
TaskHandle_t xTaskCmdMot;               // Обработка очереди команд ДПТ
TaskHandle_t xTaskCmdSt;                // Обработка очереди команд ШД
TaskHandle_t xTaskCmdServo;             // Обработка очереди команд Серво
TaskHandle_t xTaskCmdDist;              // Обработка очереди команд эхо-сенсора
TaskHandle_t xTaskCmdLeds;              // Обработка очереди команд LEDs

// очереди
QueueHandle_t xRxedChars;               // очередь принятых символов из BLE
QueueHandle_t xCmdQueue;                // общая очередь команд для исполнения
// новые очереди
QueueHandle_t xCmdMotQueue;             // очередь команд ДПТ
QueueHandle_t xCmdSt1Queue;             // очередь команд ШД1
QueueHandle_t xCmdSt2Queue;             // очередь команд ШД2
QueueHandle_t xCmdServoQueue;           // очередь команд СЕРВО
QueueHandle_t xCmdDistQueue;            // очередь команд эхо сенсора
QueueHandle_t xCmdLedsQueue;            // очередь команд светодиодов
QueueHandle_t xStateQueue;              // очередь состояний для передачи в пульт

extern uint16_t ADCbuffer[12];
extern uint32_t ir_count;
extern uint16_t echo_count;

uint8_t rxIndex = 0;
uint8_t txIndex = 0;
uint8_t RxOverflow;


int main(void) {
    clock_setup();
    systick_setup();
    gpio_setup();
    usart_setup();
    tim1_setup();
    tim2_setup();
    tim3_setup();
    tim4_setup();
    adc_setup();

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

    // создание групп событий
    xEventGroupBLE = xEventGroupCreate();
    xEventGroupADC = xEventGroupCreate();

    xRxedChars = xQueueCreate(serRX_QUEUE_LEN, sizeof(uint8_t));
    //configASSERT( xRxedChars );
    xCmdMotQueue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdSt1Queue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdSt2Queue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdServoQueue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdDistQueue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdLedsQueue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xCmdQueue = xQueueCreate(cmd_QUEUE_LEN, sizeof(ncommand_item));
    xStateQueue = xQueueCreate(state_QUEUE_LEN, BLE_PACK_SIZE);

    BaseType_t xReturned;
    xReturned = xTaskCreate( vTaskMain, "TaskMain", configMINIMAL_STACK_SIZE * 2,
                ( void * ) 1, tskIDLE_PRIORITY + 1, &xTaskMain);
    xReturned = xTaskCreate( vTaskADC, "TaskADC", configMINIMAL_STACK_SIZE,
                ( void * ) 1, tskIDLE_PRIORITY + 1, &xTaskADC);
    xReturned = xTaskCreate( vTaskLedsBlink, "LedsBlink", configMINIMAL_STACK_SIZE,
                ( void * ) 1, tskIDLE_PRIORITY + 1, &xTaskLedsBlink);
    xReturned = xTaskCreate( vTaskBLE, "BLE", configMINIMAL_STACK_SIZE,
                ( void * ) 1, tskIDLE_PRIORITY + 1, &xTaskBLE);
    xReturned = xTaskCreate( vTaskCmd, "TaskCmd", configMINIMAL_STACK_SIZE,
                ( void * ) 1, tskIDLE_PRIORITY + 1, &xTaskCmd);
    xReturned = xTaskCreate( vTaskCmdMot, "TaskCmdMot", configMINIMAL_STACK_SIZE,
                ( void * ) 1, tskIDLE_PRIORITY + 1, &xTaskCmdMot);
    xReturned = xTaskCreate( vTaskCmdSt, "TaskCmdSt", configMINIMAL_STACK_SIZE,
                ( void * ) 1, tskIDLE_PRIORITY + 1, &xTaskCmdSt);
    xReturned = xTaskCreate( vTaskCmdServo, "TaskCmdServo", configMINIMAL_STACK_SIZE,
                ( void * ) 1, tskIDLE_PRIORITY + 1, &xTaskCmdServo);
    xReturned = xTaskCreate( vTaskCmdDist, "TaskCmdDist", configMINIMAL_STACK_SIZE,
                ( void * ) 1, tskIDLE_PRIORITY + 1, &xTaskCmdDist);
    xReturned = xTaskCreate( vTaskCmdLeds, "TaskCmdLeds", configMINIMAL_STACK_SIZE,
                ( void * ) 1, tskIDLE_PRIORITY + 1, &xTaskCmdLeds);
  //printf( "Create vTaskCmdLeds: %ld\n", xReturned );
  
    while (1)
    {
    }

    return 0;
}

// =====================================================

static void clock_setup(void) {

    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);

    // JTAG-DP Disabled and SW-DP Enabled:
    AFIO_MAPR &= ~AFIO_MAPR_SWJ_MASK;               // 0 to 24..26
    AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;  // (0x2 << 24) - 1 to 25

    // In order to use our UART, we must enable the clock to it as well.
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_DMA1);
}

static void systick_setup(void) {
    // Set the systick clock source to our main clock
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    // Clear the Current Value Register so that we start at 0
    STK_CVR = 0;
    // In order to trigger an interrupt every millisecond, we can set the reload
    // value to be the speed of the processor / 1000 -1
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    // Enable interrupts from the system tick clock
    systick_interrupt_enable();
    // Enable the system tick counter
    systick_counter_enable();
}

static void gpio_setup(void) {
    // Our test LED is connected to Port C pin 13, so let's set it as output
    gpio_clear(GPIOC, GPIO13);
    gpio_clear(GPIOB, GPIO10|GPIO11|GPIO12|GPIO13|GPIO14|GPIO4|GPIO5|GPIO6|GPIO7|GPIO8);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
        GPIO10|GPIO11|GPIO12|GPIO13|GPIO14|GPIO4|GPIO5|GPIO6|GPIO7|GPIO8);

    // заранее обнуляем A11, A15
    gpio_clear(GPIOA, GPIO11|GPIO15);
    // заранее ставим высокий уровень на PA8
    gpio_set(POWER_HOLD_PORT, POWER_HOLD_PIN);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8|GPIO11|GPIO15);

    // дублируем
    gpio_set(POWER_HOLD_PORT, POWER_HOLD_PIN);

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO3);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO4);

    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
    nvic_enable_irq(NVIC_EXTI15_10_IRQ);

    exti_select_source(EXTI5, GPIOA);
    exti_set_trigger(EXTI5, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI5);

    exti_select_source(EXTI15, GPIOB);
    exti_set_trigger(EXTI15, EXTI_TRIGGER_BOTH);
    exti_enable_request(EXTI15);

    exti_select_source(EXTI12, GPIOA);
    exti_set_trigger(EXTI12, EXTI_TRIGGER_BOTH);
    exti_enable_request(EXTI12);
}

static void usart_setup(void) {

    nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
    nvic_enable_irq(NVIC_USART1_IRQ);

    /* Setup GPIO pins for USART1 transmit. */
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                    GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    /* Setup GPIO pins for USART1 receive */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

        /* Setup UART parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    /* Enable USART1 Receive interrupt. */
    USART_CR1(USART1) |= USART_CR1_RXNEIE;
    //usart_enable_rx_interrupt(USART1);
    /* Finally enable the USART. */
    usart_enable(USART1);
}

uint8_t channel_array[] = { 0, 0, 0, 1, 1, 1, 3, 3, 3, 4, 4, 4};
static void adc_setup(void)
{
    adc_power_off(ADC1);

    dma_channel_reset(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC1_DR);
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)ADCbuffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 12);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_MEDIUM);
    // прерывания по окончанию и ошибке
    //DMA1_CCR1 |= (DMA_CCR_TCIE | DMA_CCR_TEIE);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL1);
    // dma_enable_channel(DMA1, DMA_CHANNEL1);


    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
    adc_power_off(ADC1);
    rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
    rcc_peripheral_clear_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6); // 12MHz
 
    adc_set_dual_mode(ADC_CR1_DUALMOD_IND);   
    adc_disable_external_trigger_regular(ADC1);
    adc_enable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_regular_sequence(ADC1, 12, channel_array);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);
    adc_set_right_aligned(ADC1);

    adc_power_on(ADC1);
    /* Wait for ADC starting up. */
    for (int i = 0; i < 800000; i++) /* Wait a bit. */
    {
        __asm__("nop");
    }
 
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);

    adc_enable_dma(ADC1);
}

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
                      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM1_CH3);

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


// Storage for our monotonic system clock.
// Note that it needs to be volatile since we're modifying it from an interrupt.
static volatile uint64_t _millis = 0;

uint64_t millis(void) {
    return _millis;
}

// This is our interrupt handler for the systick reload interrupt.
// The full list of interrupt services routines that can be implemented is
// listed in libopencm3/include/libopencm3/stm32/f0/nvic.h
void sys_tick_handler(void) {
    // Increment our monotonic clock
    _millis++;
}

/**
 * Delay for a real number of milliseconds
 */
void delay(uint64_t duration) {
    const uint64_t until = millis() + duration;
    while (millis() < until);
}

void dma1_channel4_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((DMA1_ISR & DMA_IFCR_CTCIF4) != 0)
    {
        DMA1_IFCR |= DMA_IFCR_CTCIF4;
        //transfered = 1;
    }

        if ((DMA1_ISR & DMA_IFCR_CTEIF4) != 0) // Tranfer error flag
    {
        DMA1_IFCR |= DMA_IFCR_CTEIF4;
    }

    dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    usart_disable_tx_dma(USART1);
    dma_disable_channel(DMA1, DMA_CHANNEL4);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

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

void usart1_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Check if we were called because of RXNE. */
    if (((USART_CR1(USART1) & USART_FLAG_RXNE) != 0) &&
            ((USART_SR(USART1) & USART_FLAG_RXNE) != 0))
    {
        uint8_t r = USART_DR(USART1);
        // FLAG_RXNE сбрасывается при чтении
        if ( xQueueSendFromISR( xRxedChars, &r, &xHigherPriorityTaskWoken ) != pdTRUE )
        {
                RxOverflow = 1;
        }
    }        

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
            ((USART_SR(USART1) & USART_FLAG_TXE) != 0))   //USART_ISR_TXE
    {

        USART_CR1(USART1) &= ~USART_CR1_TXEIE;
    }

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
* @brief This function handles EXTI line[9:5] interrupts.
* Прерывание от датчика пути
*/
void exti9_5_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    //exti_get_flag_status(uint32_t exti)
    if (exti_get_flag_status(EXTI5) != 0)
    {
        exti_reset_request(EXTI5);
        if ( xSemaphoreTakeFromISR(xIRCountMutex, &xHigherPriorityTaskWoken) == pdTRUE )
        {
            // если не удастся захватить счетчик - не страшно,  т.к.
            //  быть только для сброса - в момент сброса 1 импульс м
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
     
    if (exti_get_flag_status(EXTI12) != 0)
    {
        exti_reset_request(EXTI12);
        if ( gpio_get(POWEROFF_PORT, POWEROFF_PIN) == 0 )
        {
            // нажата кнопка питания
            xEventGroupSetBits( xEventGroupADC, alarm_POWER_BUTTON_BIT);
        }
    }
     
    if (exti_get_flag_status(EXTI15) != 0)
    {   
        exti_reset_request(EXTI15);
        if (gpio_get(ECHO_PORT, ECHO_ECHO_PIN) != 0)
        {
            // начинаем отсчет появления эхо-сигнала от эхо-сенсора
            //LL_TIM_SetCounter(TIM1, 0);
            //TIM1_CNT = 0;
            timer_set_counter(TIM1, 0);
            //TIM1_CR1 |= TIM_CR1_CEN;
            timer_enable_counter(TIM1);

            echo_count = 0;
        }
        else
        {
            // пришел эхо-сигнал
            //echo_count = TIM1_CNT;
            echo_count = timer_get_counter(TIM1);
            //TIM1_CR1 &= ~TIM_CR1_CEN;
            timer_disable_counter(TIM1);
            xSemaphoreGiveFromISR(xSemaphHandleEcho, &xHigherPriorityTaskWoken);
        }
    }
        
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}              
 