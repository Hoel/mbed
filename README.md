# MBED SW4STM32 STM32F303CC
compiles fine but fails at runtime as soon as wait statement is reached, step debugging tells the probelm is related to systick irq.

It falls in <UsageFault_Handler> when it reach __HAL_TIM_SET_COMPARE(&TimMasterHandle, TIM_CHANNEL_1, (uint32_t)timestamp); 
 this function is situated in the file : us_ticker_32b.c 

