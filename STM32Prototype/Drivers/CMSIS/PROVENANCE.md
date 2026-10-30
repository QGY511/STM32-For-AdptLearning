Only the STM32G474 device headers, GCC Cortex-M4 core headers and system source are vendored. No HAL/LL driver runtime is linked.

- STMicroelectronics/cmsis-device-g4, commit `626ee412334a5ed2e5b320af5a8d77d69f03a558`, fetched 2026-09-07: `Device/ST/STM32G4xx/Include/{stm32g4xx.h,stm32g474xx.h,system_stm32g4xx.h}` and project `Src/system_stm32g4xx.c`. License retained at `Device/ST/STM32G4xx/LICENSE.md`.
- ARM-software/CMSIS_5, commit `55b19837f5703e418ca37894d5745b1dc05e4c91`, fetched 2026-09-07: `Include/{core_cm4.h,cmsis_version.h,cmsis_compiler.h,cmsis_gcc.h,mpu_armv7.h}`. Apache-2.0 license retained at `LICENSE.txt`.

Repository sources: [ST device CMSIS](https://github.com/STMicroelectronics/cmsis-device-g4/tree/626ee412334a5ed2e5b320af5a8d77d69f03a558), [Arm CMSIS](https://github.com/ARM-software/CMSIS_5/tree/55b19837f5703e418ca37894d5745b1dc05e4c91).
