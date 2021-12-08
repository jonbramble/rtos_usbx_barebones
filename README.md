## README

Following this: https://github.com/STMicroelectronics/x-cube-azrtos-f4/tree/main/Projects/STM32469I-Discovery/Applications/USBX/Ux_Device_CDC_ACM 

## Status - not working

Problems: 

Do we need dynamic memory allocation? The readme mentions it but the code suggest static memory allocation. 
Fails with insufficent memory unless I make UX memory pool size to 74K.  