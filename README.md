# GMK-Lite
 GMK Lite 

# Joystick control
The main files you should consider looking into are located under Src and Inc and are both labelled joystick.c and joystick.h respectively.

# The Rest
The rest is device specific, in setting up the device's peripherals. The ADC, TIM2, RCC, and USB are all included. The device is currently setup as a custom HID compliant device with a polling rate of 1000Hz. Each packet sent is 5 bytes. Two (2) bytes for each axis on the joystick and a single byte for the button. This configuration allows for a 
