/* FEL escape: arm the H3 watchdog (WDT0 at 0x01c20ca0, CFG +0x14 = whole-system reset, MODE +0x18 = enable, shortest interval) and spin */
	.arm
	.text
	.global _start
_start:
	ldr	r0, =0x01c20ca0
	mov	r1, #1
	str	r1, [r0, #0x14]
	str	r1, [r0, #0x18]
1:	b	1b
