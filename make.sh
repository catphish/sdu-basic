set -e
rm -f *.o
REPOROOT="/home/charlie/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.0"
CCOPTS="-Wall -mcpu=cortex-m3 -mthumb -I$REPOROOT/Drivers/CMSIS/Device/ST/STM32F1xx/Include -I$REPOROOT/Drivers/CMSIS/Include -DSTM32F1xx -O2 -ffast-math -gdwarf-2 -g3"
arm-none-eabi-gcc $CCOPTS -c startup_stm32f103xb.s -o startup_stm32f103xb.o
arm-none-eabi-gcc $CCOPTS -c main.c -o main.o
arm-none-eabi-gcc $CCOPTS -c system.c -o system.o
arm-none-eabi-gcc $CCOPTS -T STM32F103RBTx_FLASH.ld -Wl,--gc-sections *.o -o main.elf -lm
arm-none-eabi-objcopy -O binary main.elf main.bin
#st-flash write main.bin 0x8000000
