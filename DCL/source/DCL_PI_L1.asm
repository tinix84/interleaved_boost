; DCL_PI_L1.asm - Series PI controller
;
; Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
; ALL RIGHTS RESERVED 

		.if __TI_EABI__
		.asg	DCL_runPI_L1, _DCL_runPI_L1
		.endif

		.global _DCL_runPI_L1
		.def	__cla_DCL_runPI_L1_sp

SIZEOF_LFRAME	.set	5
LFRAME_MR3		.set	0

__cla_DCL_runPI_L1_sp	.usect ".scratchpad:Cla1Prog:_DCL_runPI_L1", SIZEOF_LFRAME, 0, 1
		.asg	 __cla_DCL_runPI_L1_sp, LFRAME

		.sect 	"Cla1Prog:_DCL_runPI_L1"

		.align 	2

; C prototype:
; float DCL_runPI_L1(PI_CTL *p, float rk, float yk)
; argument 1 = *p : 32-bit PI structure address [MAR0]
; argument 2 = rk : 32-bit floating-point reference [MR0]
; argument 3 = yk : 32-bit floating-point feedback [MR1]
; return = uk : 32-bit floating-point [MR0]

_DCL_runPI_L1:
;		MDEBUGSTOP
		MMOV32		@LFRAME + LFRAME_MR3, MR3 	; save MR3

;*** servo error ***
		MSUBF32		MR2, MR0, MR1				; MR2 = v1

;*** proportional path ***
		MNOP									; MAR0 load delay slot
		MMOV32		MR1, *MAR0[2]++				; MR1 = Kp
		MMPYF32		MR2, MR1, MR2				; MR2 = v2

;*** integral path ***
||		MMOV32		MR1, *MAR0[8]++				; MR1 = Ki
		MMPYF32		MR0, MR2, MR1				; MR0 = v3
||		MMOV32		MR1, *MAR0[-6]++			; MR1 = i6
		MMPYF32		MR0, MR0, MR1				; MR0 = v8
||		MMOV32		MR1, *MAR0					; MR1 = i10
		MADDF32		MR0, MR0, MR1				; MR0 = v4
		MMOV32		*MAR0[2]++, MR0				; save i10
		MADDF32		MR0, MR0, MR2				; MR0 = v5

;*** output saturation ***
		MMOVF32		MR2, #0.0f					; MR2 = 0.0f
		MMOVF32		MR3, #1.0f					; MR3 = 1.0f
		MMOV32		MR1, *MAR0[2]++				; MR1 = Umax
		MMINF32		MR0, MR1					; MR0 = sat+
		MMOV32		MR3, MR2, GT				; MR3 = v6
		MMOV32		MR1, *MAR0[2]++				; MR1 = Umin
		MMAXF32		MR0, MR1					; MR0 = sat-
		MRCNDD		UNC							; return call
		MMOV32		MR3, MR2, LT				; MR3 = v6
		MMOV32		*MAR0, MR3					; save i6
		MMOV32		MR3, @LFRAME + LFRAME_MR3	; restore MR3

		.unasg	LFRAME

; end of file
