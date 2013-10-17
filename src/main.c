#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#include "systime.h"
#include "macros.h"
#include "scheduler.h"

#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
#define CYCLE_COUNTS        2000        // 5 Hz at 10 kS/s

extern uint8_t protocol_rx_buffer[64];
static Task *pid_task;
static uint16_t counter = 0;

__IO uint16_t ADC3ConvertedValue;

static void ADC3_CH12_DMA_Config(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

    DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream0, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;     /* ADC 3 on PC2 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;  
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC3, &ADC_InitStructure);
    
    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
    ADC_DMACmd(ADC3, ENABLE);
    ADC_Cmd(ADC3, ENABLE);
}

uint16_t adc_read(void)
{
    uint16_t val1;
    val1 = ADC3ConvertedValue;
    while(ADC3ConvertedValue == val1);
    return (val1 + ADC3ConvertedValue) / 2;
}

/* Task Callbacks */
void pid_callback(Task *task)
{
    if (counter > CYCLE_COUNTS) {
        Scheduler_SetTaskEnabled(task, FALSE);
        counter = 0;
    }
}

int main(void)
{
    ADC3_CH12_DMA_Config();
    ADC_SoftwareStartConv(ADC3);

    SysTime_Init();

    pid_task = Scheduler_AddTask(systime(), 1, &pid_callback, FALSE);
    while (1) {
        Scheduler_Run(systime());
    }

    return 0;
}
