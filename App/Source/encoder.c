#include "encoder.h"

//电机编码器结构体
encoder_counter motor1_encoder = {0, 0, 0, 0, 0, 0, 0};
encoder_counter motor2_encoder = {0, 0, 0, 0, 0, 0, 0};

static void Encoder_GPIO_Config(void)
{
    /* 使能时钟 */
	rcu_periph_clock_enable(MOTOR1_encoderA_RCU);
    rcu_periph_clock_enable(MOTOR1_encoderB_RCU);
	rcu_periph_clock_enable(MOTOR2_encoderA_RCU);
    rcu_periph_clock_enable(MOTOR2_encoderB_RCU);

	/* 配置GPIO的模式 */
	gpio_mode_set(MOTOR1_encoderA_PORT,GPIO_MODE_AF,GPIO_PUPD_NONE,MOTOR1_encoderA_PIN);
    gpio_mode_set(MOTOR1_encoderB_PORT,GPIO_MODE_AF,GPIO_PUPD_NONE,MOTOR1_encoderB_PIN);
	gpio_mode_set(MOTOR2_encoderA_PORT,GPIO_MODE_AF,GPIO_PUPD_NONE,MOTOR2_encoderA_PIN);
    gpio_mode_set(MOTOR2_encoderB_PORT,GPIO_MODE_AF,GPIO_PUPD_NONE,MOTOR2_encoderB_PIN);

	/* 配置GPIO的复用 */
	gpio_af_set(MOTOR1_encoderA_PORT,MOTOR1_encoderA_AF,MOTOR1_encoderA_PIN);
    gpio_af_set(MOTOR1_encoderB_PORT,MOTOR1_encoderB_AF,MOTOR1_encoderB_PIN);
	gpio_af_set(MOTOR2_encoderA_PORT,MOTOR2_encoderA_AF,MOTOR2_encoderA_PIN);
    gpio_af_set(MOTOR2_encoderB_PORT,MOTOR2_encoderB_AF,MOTOR2_encoderB_PIN);
}

static void Motor1_Encoder_Timer_Init(void)
{
    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icintpara;

    // 开启定时器时钟
	rcu_periph_clock_enable(MOTOR1_encoder_TIMER_RCU);
	rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);	// 配置定时器时钟

    // 复位定时器
    timer_deinit(MOTOR1_encoder_TIMER);
    timer_initpara.period 				= (100-1)*4;//设定计数器自动重装值
	timer_initpara.prescaler 			= 0; // 预分频器 
	timer_initpara.clockdivision 		= TIMER_CKDIV_DIV1; 		// 不分频
	timer_initpara.alignedmode 			= TIMER_COUNTER_EDGE; 	// 边缘对齐
	timer_initpara.counterdirection     = TIMER_COUNTER_UP;			// 向上计数
	timer_initpara.repetitioncounter    = 0; // 重复计数器 0-255
	timer_init(MOTOR1_encoder_TIMER,&timer_initpara); 	// 初始化定时器

    // timer1编码器模式2
    timer_quadrature_decoder_mode_config(MOTOR1_encoder_TIMER, TIMER_QUAD_DECODER_MODE2, TIMER_IC_POLARITY_RISING, TIMER_IC_POLARITY_RISING);

    timer_channel_input_struct_para_init(&timer_icintpara);
    //更新通道输入捕获过滤器
    timer_icintpara.icfilter = 10;
    timer_input_capture_config(MOTOR1_encoder_TIMER, MOTOR1_encoderA_CHANNEL, &timer_icintpara);
    timer_input_capture_config(MOTOR1_encoder_TIMER, MOTOR1_encoderB_CHANNEL, &timer_icintpara);

    //开启中断
    nvic_irq_enable(MOTOR1_encoder_IRQ, 2, 2);
    timer_interrupt_enable(MOTOR1_encoder_TIMER, TIMER_INT_UP);
    //清除定时器更新标志
    timer_interrupt_flag_clear(MOTOR1_encoder_TIMER, TIMER_INT_FLAG_UP);
    //定时器计数清零
    timer_counter_value_config(MOTOR1_encoder_TIMER, 0);
    //启用自动加载阴影功能
	timer_auto_reload_shadow_enable(MOTOR1_encoder_TIMER);
    //开启定时器
	timer_enable(MOTOR1_encoder_TIMER);
}

static void Motor2_Encoder_Timer_Init(void)
{
    timer_parameter_struct timer_initpara;
    timer_ic_parameter_struct timer_icintpara;

    // 开启定时器时钟
	rcu_periph_clock_enable(MOTOR2_encoder_TIMER_RCU);
	rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);	// 配置定时器时钟

    // 复位定时器
    timer_deinit(MOTOR2_encoder_TIMER);
    timer_initpara.period 				= (100-1)*4;//设定计数器自动重装值
	timer_initpara.prescaler 			= 0; // 预分频器 
	timer_initpara.clockdivision 		= TIMER_CKDIV_DIV1; 		// 不分频
	timer_initpara.alignedmode 			= TIMER_COUNTER_EDGE; 	// 边缘对齐
	timer_initpara.counterdirection     = TIMER_COUNTER_UP;			// 向上计数
	timer_initpara.repetitioncounter    = 0; // 重复计数器 0-255
	timer_init(MOTOR2_encoder_TIMER,&timer_initpara); 	// 初始化定时器

    // timer1编码器模式2
    timer_quadrature_decoder_mode_config(MOTOR2_encoder_TIMER, TIMER_QUAD_DECODER_MODE2, TIMER_IC_POLARITY_RISING, TIMER_IC_POLARITY_RISING);

    timer_channel_input_struct_para_init(&timer_icintpara);
    //更新通道输入捕获过滤器
    timer_icintpara.icfilter = 10;
    timer_input_capture_config(MOTOR2_encoder_TIMER, MOTOR2_encoderA_CHANNEL, &timer_icintpara);
    timer_input_capture_config(MOTOR2_encoder_TIMER, MOTOR2_encoderB_CHANNEL, &timer_icintpara);

    //开启中断
    nvic_irq_enable(MOTOR2_encoder_IRQ, 2, 2);
    timer_interrupt_enable(MOTOR2_encoder_TIMER, TIMER_INT_UP);
    //清除定时器更新标志
    timer_interrupt_flag_clear(MOTOR2_encoder_TIMER, TIMER_INT_FLAG_UP);
    //定时器计数清零
    timer_counter_value_config(MOTOR2_encoder_TIMER, 0);
    //启用自动加载阴影功能
	timer_auto_reload_shadow_enable(MOTOR2_encoder_TIMER);
    //开启定时器
	timer_enable(MOTOR2_encoder_TIMER);
}

void Motor_Encoder_Init(void)
{
    Encoder_GPIO_Config();
    Motor1_Encoder_Timer_Init();
    Motor2_Encoder_Timer_Init();
}

uint32_t Motor1_Encoder_Value(void)
{
    uint32_t encoder_value=0;
    encoder_value = timer_counter_read(MOTOR1_encoder_TIMER); //读取计数值，电机正转，cnt：0->396,反转：396->0
    // timer_counter_value_config(MOTOR1_encoder_TIMER, 0);
    return encoder_value;
}

uint32_t Motor2_Encoder_Value(void)
{
    uint32_t encoder_value=0;
    encoder_value = timer_counter_read(MOTOR2_encoder_TIMER);
    timer_counter_value_config(MOTOR2_encoder_TIMER, 0);
    return encoder_value;
}

void Motor1_Encoder_IRQHandler(void)
{
    if(timer_interrupt_flag_get(MOTOR1_encoder_TIMER,TIMER_INT_UP) == SET)
	{
        uint8_t dir = (TIMER_CTL0(MOTOR1_encoder_TIMER) & 0x10) >> 4;
        if(dir) 
            motor1_encoder.circle_count--;
        else
            motor1_encoder.circle_count++;
        timer_interrupt_flag_clear(MOTOR1_encoder_TIMER,TIMER_INT_UP);
    }
}

void Motor2_Encoder_IRQHandler(void)
{
    if(timer_interrupt_flag_get(MOTOR2_encoder_TIMER,TIMER_INT_UP) == SET)
	{
        uint8_t dir = (TIMER_CTL0(MOTOR2_encoder_TIMER) & 0x10) >> 4;
        if(dir) 
            motor2_encoder.circle_count--;
        else
            motor2_encoder.circle_count++;
        timer_interrupt_flag_clear(MOTOR2_encoder_TIMER,TIMER_INT_UP);
    }
}