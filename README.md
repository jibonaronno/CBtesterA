# Timer Triggered ADC-DMA at 10 - 20 Kilo Sample Per Second for audio
BlackPill. STM32F401CCU6 Development Board.
<img alt="NO IMAGE" src="blackpill.png"><br><br>
### Setup the ADC in CubeMX as follows. <br>
<img alt="NO IMAGE" src="adcA.png"><br>
### Setup DMA<br>
<img alt="NO IMAGE" src="adcB.png"><br><br>
### Setup ADC Interrupt (For checking Convertion Rate)
<img alt="NO IMAGE" src="adcC.png"><br><br>
### Now setup the TIM2 as follows. Note: Trigger Event Selection Must be set to Update Event.
<img alt="NO IMAGE" src="timA.png"><br><br>

Reference : <br>
[Set up multiple ADCs on STM32 microcontrollers using DMA](https://www.youtube.com/watch?v=AloHXBk6Bfk)<br>
[STM32CubeIDE basics - 10 ADC DMA TIM HAL lab](https://www.youtube.com/watch?v=pLsAhJ8umJk)<br>
[create audio signals with PWM STM32F103C8](https://marcelmg.github.io/pwm_dac_sound/)<br>
