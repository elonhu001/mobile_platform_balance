本项目的电机驱动电源为31V，锂电池供电。

【资源使用说明】

【电机速度控制】
PE5     ------> TIM9_CH1:左轮PWM
PC0     ------> ENABLE_L
PC1     ------> DIRECTION_L

PE6     ------> TIM9_CH2:右轮PWM
PC2     ------> ENABLE_R
PC3     ------> DIRECTION_R

PA5     ------> TIM2_CH1 备用
PB3     ------> TIM2_CH2 备用
【电机速度采集】
PA6     ------> TIM3_CH1:左轮方向(HA)
PA7     ------> TIM3_CH2:左轮方向(HB)
PB0     ------> TIM3_CH3:左轮速度采集*
PA2     ------> USART2_TX
PA3     ------> USART2_RX 

PD12     ------> TIM4_CH1:右轮方向(HA)
PD13     ------> TIM4_CH2:右轮方向(HB)
PD14     ------> TIM4_CH3:右轮速度采集*
PB10     ------> USART3_TX
PB11     ------> USART3_RX 



