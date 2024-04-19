#include "bsp_adc.h"

/**********************************************************
 * �� �� �� �ƣ�adc_config
 * �� �� �� �ܣ�ADC��ʼ��
 * �� �� �� ������
 * �� �� �� �أ���
 * ��       �ߣ�LCKFB
 * ��       ע����
**********************************************************/
void adc_config(void)
{
	//ʹ������ʱ��
    rcu_periph_clock_enable(RCU_GPIOC);      
    //ʹ��ADCʱ��
    rcu_periph_clock_enable(RCU_ADC0);
    //����ADCʱ��
    adc_clock_config(ADC_ADCCK_PCLK2_DIV4);
	
	//��������Ϊģ������ģʽ
    gpio_mode_set(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_1); 
	
	//����ADCΪ����ģʽ      
	adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
		
	//ʹ��ɨ��ģʽ
	adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);

	//�����Ҷ���     
	adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
	   
	//ADC0����Ϊ12λ�ֱ���               
	adc_resolution_config(ADC0, ADC_RESOLUTION_12B);
	
	//ADC0����Ϊ������  һ��ʹ�� 1 ��ͨ��
	adc_channel_length_config(ADC0,ADC_ROUTINE_CHANNEL, 1);

	//ADC�ⲿ��������, ��ֻ��ʹ���������
	adc_external_trigger_config(ADC0, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);

	//ADC0ʹ��
	adc_enable(ADC0);

	//����ADC��У׼
	adc_calibration_enable(ADC0); 
}

/**********************************************************
 * �� �� �� �ƣ�Get_ADC_Value
 * �� �� �� �ܣ���ȡADCֵ
 * �� �� �� ����ADC_CHANNEL_x=Ҫ�ɼ���ͨ��
 * �� �� �� �أ���������ֵ
 * ��       �ߣ�LC
 * ��       ע����
**********************************************************/
unsigned int Get_ADC_Value(uint8_t  ADC_CHANNEL_x)
{
    unsigned int adc_value = 0;
    //���òɼ�ͨ��
    adc_routine_channel_config(ADC0, 0, ADC_CHANNEL_x, ADC_SAMPLETIME_15);
    //��ʼ���ת��
    adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
    //�ȴ� ADC0 ������� 
    while ( adc_flag_get(ADC0, ADC_FLAG_EOC) == RESET ) 
    {
        ;
    }
    //��ȡ����ֵ
    adc_value = adc_routine_data_read(ADC0);
    //���ز���ֵ
    return adc_value;
}
