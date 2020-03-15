# STM32-USBHID-MultimediaPilot
STM32 based USB device with a joystick to control your media player on PC.
It's a simple USB keyboard with only multimedia keys, and joystick with encoder instead of keys.

Main components are:
- Stm32F103 aka 'blue pill' development board
- ALPS RKJXT1F42001 joystick
- 3D printed enclosure
- firmware with custom USB descriptor based on ST's USB_DEVICE HAL library

Encoder is acting as volume up/down, pushbutton is play/pause key, left/right is next/prev track.
![alt text](https://i.imgur.com/bkhTU0X.png)
