# ADC Injection by TRGO

TIM1 enable output compare OC4REF as TRGO (Trigger output), ADC2 injection group triggered by TIM1 TRGO falling edge.

Check the code under the [User](./User/) folder.

- GPIO
  - ADC2_IN3: PA6
  - TIM_CH1: PA8
  - TIM_CH1N: PA7
- UART: 115200-8-n-1  

> STM32CubeIDE `v1.14.1`  
> Test on Nucleo-G431KB
