本项目的电机驱动电源为31V，锂电池供电。

【资源使用说明】

【电机速度控制】
PA5     ------> TIM2_CH1:左轮PWM
PC0     ------> ENABLE_L
PC1     ------> DIRECTION_L

PA1     ------> TIM2_CH2:右轮PWM
PC2     ------> ENABLE_R
PC3     ------> DIRECTION_R

【电机速度采集】
PA6     ------> TIM3_CH1:左轮方向(HA)
PA7     ------> TIM3_CH2:左轮方向(HB)
PB0     ------> TIM3_CH3:左轮速度采集*
PB10     ------> USART3_TX:其他信息采集串口
PB11     ------> USART3_RX 

PD12     ------> TIM4_CH1:右轮方向(HA)
PD13     ------> TIM4_CH2:右轮方向(HB)
PD14     ------> TIM4_CH3:右轮速度采集*
PC6     ------> USART6_TX：其他信息采集串口
PC7     ------> USART6_RX 



