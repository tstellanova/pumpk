# pumpk

This is a simple embedded rust application that utilizes the
[freertos-rust](https://crates.io/crates/freertos-rust) crate 
to build a hybrid application where FreeRTOS creates and drives tasks.

This app was built for and tested on the 
[Robotdyn stm32f303 mini Cortex-M4 dev board](https://robotdyn.com/stm32f303cct6-256-kb-flash-stm32-arm-cortexr-m4-mini-system-dev-board-3326a9dd-3c19-11e9-910a-901b0ebb3621.html). 

and uses J-Link/RTT for debugging by default. 