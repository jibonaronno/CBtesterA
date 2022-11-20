# Timer Triggered ADC-DMA at 10 - 20 Kilo Sample Per Second for audio
BlackPill. STM32F401CCU6 Development Board.
<img alt="NO IMAGE" src="blackpill.png"><br><br>
Here we use Timer2 EVENT TRIGGER output to trigger the ADC start conversion at a regular interval. 
Now ADC conversion rate is 9 Kilo Sample per second. Check the Timer settings below. We alwyas have 
more options to increase conversion rate. From the document [HERE](https://marcelmg.github.io/pwm_dac_sound/) we can calculate maximum audio frequency
from the conversion rate. 
### Setup the ADC in CubeMX as follows. <br>
<img alt="NO IMAGE" src="adcA.png"><br>
### Setup DMA<br>
<img alt="NO IMAGE" src="adcB.png"><br><br>
### Setup ADC Interrupt (For checking Convertion Rate)
<img alt="NO IMAGE" src="adcC.png"><br><br>
### Now setup the TIM2 as follows. Note: Trigger Event Selection Must be set to Update Event.
<img alt="NO IMAGE" src="timA.png"><br><br>

___

### We setup the PWM DAC as described in<br>
[STM32 TIMERS #1. PWM Output || DMA](https://www.youtube.com/watch?v=OwlfFp8fPN0) and <br>
[STM32CubeIDE basics - 05 TIM PWM HAL lab](https://www.youtube.com/watch?v=-AFCcfzK9xc)<br>

Configure TIM1 as below. <br><br>
<img alt="NO IMAGE" src="TIM1_PWM.png"><br>
<img alt="NO IMAGE" src="TIM1_PWM0.png"><br><br>
<br><br>
### As referenced from  Reference : <br>[Set up multiple ADCs on STM32 microcontrollers using DMA At Time 1:53](https://youtu.be/AloHXBk6Bfk?t=113)<br>
it shows multiple ADC input setup in Cube IDE. <br >
1. Set Number of Conversion = 2 Press Enter<br>
2. Scan Conversion Mode = Enabled<br>
3. DMA Settings should be configured as [DMA setup AT 3:48](https://youtu.be/AloHXBk6Bfk?t=228)

[Set up multiple ADCs on STM32 microcontrollers using DMA](https://www.youtube.com/watch?v=AloHXBk6Bfk)<br>
[STM32CubeIDE basics - 10 ADC DMA TIM HAL lab](https://www.youtube.com/watch?v=pLsAhJ8umJk)<br>
[create audio signals with PWM STM32F103C8](https://marcelmg.github.io/pwm_dac_sound/)<br>
[STM32 TIMERS #1. PWM Output || DMA](https://www.youtube.com/watch?v=OwlfFp8fPN0)<br>
[EmbeddedExpertIO](https://embeddedexpert.io/?p=858&fbclid=IwAR2bhHOcd__Dlb0FOMDrgP93dbdVjOlu2wdS3iJBXY4OYVTGU9i-guuQK3w)<br>
