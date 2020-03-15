# STM32 firmware

If you're lazy I provide ready to flash usbhid_firmware.bin file. Flash with your ST-LINK and enjoy.

Source written in STM32CubeIDE 1.3.0.

If you regenerate code using Cube **make a backup** of files in /Middlewares/ST/STM32_USB_Device_Library/Class/HID/
because **Cube will overwrite** them with example report descriptor for USB mouse. 
