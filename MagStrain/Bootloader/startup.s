/* STM32F103 bare-metal bootloader: no application HAL dependency. */
.syntax unified
.cpu cortex-m3
.thumb

.section .isr_vector,"a",%progbits
.global g_pfnVectors
g_pfnVectors:
.word _estack
.word Reset_Handler
.rept 80
.word Default_Handler
.endr

.section .text.Reset_Handler,"ax",%progbits
.thumb_func
.global Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
1:  cmp r1, r2
    bcs 2f
    ldr r3, [r0], #4
    str r3, [r1], #4
    b 1b
2:  ldr r1, =_sbss
    ldr r2, =_ebss
    movs r3, #0
3:  cmp r1, r2
    bcs 4f
    str r3, [r1], #4
    b 3b
4:  bl boot_main
5:  b 5b

.thumb_func
Default_Handler:
    b Default_Handler
