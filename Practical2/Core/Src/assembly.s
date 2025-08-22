/*
 * assembly.s
 *
 */
 
 @ DO NOT EDIT
	.syntax unified
    .text
    .global ASM_Main
    .thumb_func

@ DO NOT EDIT
vectors:
	.word 0x20002000
	.word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

	@ Some code is given below for you to start with
	LDR R0, RCC_BASE  		@ Enable clock for GPIOA and B by setting bit 17 and 18 in RCC_AHBENR
	LDR R1, [R0, #0x14]
	LDR R2, AHBENR_GPIOAB	@ AHBENR_GPIOAB is defined under LITERALS at the end of the code
	ORRS R1, R1, R2
	STR R1, [R0, #0x14]

	LDR R0, GPIOA_BASE		@ Enable pull-up resistors for pushbuttons
	MOVS R1, #0b01010101
	STR R1, [R0, #0x0C]
	LDR R1, GPIOB_BASE  	@ Set pins connected to LEDs to outputs
	LDR R2, MODER_OUTPUT
	STR R2, [R1, #0]
	MOVS R2, #0         	@ NOTE: R2 will be dedicated to holding the value on the LEDs

@ TODO: Add code, labels and logic for button checks and LED patterns

main_loop:
	@start of student's code
	LDR R3, GPIOA_BASE
	LDR R3, [R3, #0x10] @get the state of GPIOA->IDR

	@check if BTN3 is pressed
    MOVS R4, #8
	ANDS R4, R4, R3
	BEQ main_loop @if BTN3 is pressed, continue to next iteration

	@check if BTN2 is pressed
	MOVS R4, #4
	ANDS R4, R4, R3
	BNE skip_btn2 @if BTN2 is not pressed, skip the next instruction
	MOVS R2, #0xAA @set the LED pattern to 0xAA
	B write_leds
	skip_btn2:

	@check if BTN1 is pressed
	@change delay from 0.7s to 0.3s
	@otherwise change delay to 0.7s

	@check if BTN0 is pressed
	MOVS R4, #2
	ANDS R4, R4, R3
	BNE skip_btn0 @skip the BTN0 special special function
	MOVS R4, #2
	ADD R2, R2, R4 @increment LEDs by 2
	B write_leds
	skip_btn0: @normal increment
	MOVS R4, #1
	ADD R2, R2, R4 @increment LEDs by 1

	@end of student's code

write_leds:
	STR R2, [R1, #0x14]
	B main_loop

@ LITERALS; DO NOT EDIT
	.align
RCC_BASE: 			.word 0x40021000
AHBENR_GPIOAB: 		.word 0b1100000000000000000
GPIOA_BASE:  		.word 0x48000000
GPIOB_BASE:  		.word 0x48000400
MODER_OUTPUT: 		.word 0x5555

@ TODO: Add your own values for these delays
LONG_DELAY_CNT: 	.word 0
SHORT_DELAY_CNT: 	.word 0
