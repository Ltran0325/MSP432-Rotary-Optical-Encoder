# MSP432-Rotary-Optical-Encoder

### Demo:

https://youtu.be/wDcu66NudCA

### Encoder Operation:

![image](https://user-images.githubusercontent.com/62213019/112936832-67e17b00-90db-11eb-9b15-487cd2b0b1af.png)

Source: Operating principle of the optical incremental encoder - Sang-Hoon Kim.

Operation:

- When a rising edge of Phase A is detected while phase B is high, negative angle rotation.

- When a rising edge of Phase A is detected while phase B is low, positive angle rotation.

- Phase A and B are connected to MSP432 as GPIO input.

### Setup:

![image](https://user-images.githubusercontent.com/62213019/112938993-906b7400-90df-11eb-85b7-43f4ec052a5a.png)

Red LED on left of 7-segment display for encoder angle polarity (ON for negative).
- CW is positive count direction.
- CCW is negative count direction.
