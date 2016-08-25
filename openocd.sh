#!/bin/bash
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "stm32f4x.cpu configure -rtos auto"