# SMBED SW4STM32
compiles fine but fails at runtime as soon as wait statement is reached, step debugging tells the probelm is related to systick irq.

It fails in us_ticker_32b.c at __HAL_TIM_SET_COMPARE(&TimMasterHandle, TIM_CHANNEL_1, (uint32_t)timestamp); 

