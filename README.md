# stm32-ublox-ubx-parser
uBlox UBX Message parser and uBlox GNSS Driver Library for STM32

## How to use?

Import library to main.c
```
#include <GNSS.h>
```

Create instance
```
GNSS_StateHandle GNSS_Handle;
```

Start to configure GNSS module. Set baudrate to 115200
```
GNSS_Init(&GNSS_Handle, &huart1);
```

Because we changed GNSS baud rate to 115200, we need to change also UART1 baud rate and init it again.
```
HAL_UART_Abort_IT(&huart1);
HAL_UART_DeInit(&huart1);
huart1.Init.BaudRate = 115200;
if (HAL_UART_Init(&huart1) != HAL_OK) {
Error_Handler();
}
HAL_Delay(1000);
```

Timer values for GNSS requester and Hz counter
```
uint32_t Timer = HAL_GetTick();
uint32_t TimeHz = HAL_GetTick();
int printFlag = 0;
```

Set GNSS mode to desired aplication case
```
GNSS_SetMode(&GNSS_Handle, Stationary);

HAL_Delay(250);
```

In while loop;

Request GNSS data whenever timer hits 100 ms.
As HAL_GetTick returns SysTime as milliseconds, 100 ms means we are getting GNSS values at 5 Hz rate

Request "Navigation Position Velocity Time Solution"
Refeer to "32.17.17 UBX-NAV-PVT (0x01 0x07)" at M8N Interface manual
```
if ((HAL_GetTick() - Timer) > 200) {


GNSS_GetPVTData(&GNSS_Handle);
GNSS_ParseBuffer(&GNSS_Handle);

Timer = HAL_GetTick();

printFlag = 1;
count ++;
}
```
