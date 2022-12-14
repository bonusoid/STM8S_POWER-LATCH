;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.5.0 #9253 (Apr  3 2018) (Linux)
; This file was generated Sat Sep  3 15:01:27 2022
;--------------------------------------------------------
	.module main
	.optsdcc -mstm8
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _font_arr
	.globl _main
	.globl _read_pkey
	.globl _chgst
	.globl _vbat
	.globl _scf
	.globl _ktime
	.globl _chg_disp
	.globl _page_id
	.globl _pkeylock
	.globl _pkeyp
	.globl _readreg
	.globl _delay_init
	.globl _delay_us
	.globl _delay_ms
	.globl _delay_timer
	.globl _clock_init
	.globl _i2c_init
	.globl _i2c_set_start
	.globl _i2c_set_address
	.globl _i2c_set_stop
	.globl _i2c_clear_ack
	.globl _i2c_set_ack
	.globl _i2c_ack_pos_current
	.globl _i2c_ack_pos_next
	.globl _i2c_poll_SB
	.globl _i2c_poll_ADDR
	.globl _i2c_poll_BTF
	.globl _i2c_poll_TXE
	.globl _i2c_poll_RXNE
	.globl _i2c_clear_bits
	.globl _i2c_clear_ADDR
	.globl _i2c_enable_interrupts
	.globl _i2c_disable_interrupts
	.globl _adc_init
	.globl _read_adc
	.globl _uart1_init
	.globl _uart1_send
	.globl _uart1_recv
	.globl _uart1_recv_i
	.globl _pwm1_init
	.globl _pwm2_init
	.globl _pwm1ch1_enable
	.globl _pwm1ch1_disable
	.globl _pwm2ch1_enable
	.globl _pwm2ch1_disable
	.globl _pwm1_update
	.globl _pwm2_update
	.globl _lcdn1202_gpio_init
	.globl _lcdn1202_9bsend
	.globl _lcdn1202_clock1
	.globl _lcdn1202_blon
	.globl _lcdn1202_bloff
	.globl _lcdn1202_init
	.globl _lcdn1202_sendcom
	.globl _lcdn1202_senddat
	.globl _lcdn1202_setpos
	.globl _lcdn1202_clear
	.globl _LCD_setpos
	.globl _LCD_drawbyte
	.globl _LCD_drawchar
	.globl _LCD_drawtext
	.globl _LCD_drawint
	.globl _LCD_clear
	.globl _LCD_clearblock
	.globl _LCD_normal
	.globl _LCD_reverse
	.globl _LCD_BL_ON
	.globl _LCD_BL_OFF
	.globl _powerman_init
	.globl _Power_Latch
	.globl _Power_Unlatch
	.globl _read_pkey
	.globl _vbat_mon
	.globl _chgst_mon
	.globl _loop
	.globl _Page_Charging
	.globl _Page_Main
	.globl _update_pkey
	.globl _on_single_click
	.globl _chg_single_click
	.globl _disp_bat_status
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area DATA
_readreg::
	.ds 1
_pkeyp::
	.ds 1
_pkeylock::
	.ds 1
_page_id::
	.ds 1
_chg_disp::
	.ds 1
_ktime::
	.ds 1
_scf::
	.ds 1
_vbat::
	.ds 2
_chgst::
	.ds 1
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area INITIALIZED
;--------------------------------------------------------
; Stack segment in internal ram 
;--------------------------------------------------------
	.area	SSEG
__start__stack:
	.ds	1

;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area DABS (ABS)
;--------------------------------------------------------
; interrupt vector 
;--------------------------------------------------------
	.area HOME
__interrupt_vect:
	int s_GSINIT ;reset
	int 0x0000 ;trap
	int 0x0000 ;int0
	int 0x0000 ;int1
	int 0x0000 ;int2
	int 0x0000 ;int3
	int 0x0000 ;int4
	int 0x0000 ;int5
	int 0x0000 ;int6
	int 0x0000 ;int7
	int 0x0000 ;int8
	int 0x0000 ;int9
	int 0x0000 ;int10
	int 0x0000 ;int11
	int 0x0000 ;int12
	int 0x0000 ;int13
	int 0x0000 ;int14
	int 0x0000 ;int15
	int 0x0000 ;int16
	int 0x0000 ;int17
	int 0x0000 ;int18
	int 0x0000 ;int19
	int 0x0000 ;int20
	int 0x0000 ;int21
	int 0x0000 ;int22
	int 0x0000 ;int23
	int 0x0000 ;int24
	int 0x0000 ;int25
	int 0x0000 ;int26
	int 0x0000 ;int27
	int 0x0000 ;int28
	int 0x0000 ;int29
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME
	.area GSINIT
	.area GSFINAL
	.area GSINIT
__sdcc_gs_init_startup:
__sdcc_init_data:
; stm8_genXINIT() start
	ldw x, #l_DATA
	jreq	00002$
00001$:
	clr (s_DATA - 1, x)
	decw x
	jrne	00001$
00002$:
	ldw	x, #l_INITIALIZER
	jreq	00004$
00003$:
	ld	a, (s_INITIALIZER - 1, x)
	ld	(s_INITIALIZED - 1, x), a
	decw	x
	jrne	00003$
00004$:
; stm8_genXINIT() end
	.area GSFINAL
	jp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME
	.area HOME
__sdcc_program_startup:
	jp	_main
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CODE
;	delay.c: 7: void delay_init()
;	-----------------------------------------
;	 function delay_init
;	-----------------------------------------
_delay_init:
;	delay.c: 9: TIM4_PSCR = 4; // CLK/16
	mov	0x5347+0, #0x04
	ret
;	delay.c: 12: void delay_us(unsigned long delus)
;	-----------------------------------------
;	 function delay_us
;	-----------------------------------------
_delay_us:
	sub	sp, #6
;	delay.c: 16: for(du=0;du<(delus/10);du++)
	push	#0x0a
	clrw	x
	pushw	x
	push	#0x00
	ldw	x, (0x0f, sp)
	pushw	x
	ldw	x, (0x0f, sp)
	pushw	x
	call	__divulong
	addw	sp, #8
	ldw	(0x05, sp), x
	ldw	(0x03, sp), y
	clrw	x
	ldw	(0x01, sp), x
00103$:
	ldw	x, (0x01, sp)
	clrw	y
	cpw	x, (0x05, sp)
	ld	a, yl
	sbc	a, (0x04, sp)
	ld	a, yh
	sbc	a, (0x03, sp)
	jrnc	00101$
;	delay.c: 18: delay_timer(100);
	push	#0x64
	call	_delay_timer
	pop	a
;	delay.c: 16: for(du=0;du<(delus/10);du++)
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x01, sp), x
	jra	00103$
00101$:
;	delay.c: 20: delay_timer(delus%10);
	push	#0x0a
	clrw	x
	pushw	x
	push	#0x00
	ldw	x, (0x0f, sp)
	pushw	x
	ldw	x, (0x0f, sp)
	pushw	x
	call	__modulong
	addw	sp, #8
	ld	a, xl
	push	a
	call	_delay_timer
	addw	sp, #7
	ret
;	delay.c: 23: void delay_ms(unsigned long delms)
;	-----------------------------------------
;	 function delay_ms
;	-----------------------------------------
_delay_ms:
	sub	sp, #8
;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
	ldw	x, (0x0d, sp)
	pushw	x
	ldw	x, (0x0d, sp)
	pushw	x
	push	#0x64
	clrw	x
	pushw	x
	push	#0x00
	call	__mullong
	addw	sp, #8
	ldw	(0x07, sp), x
	ldw	(0x05, sp), y
	clrw	x
	clr	a
	clr	(0x01, sp)
00103$:
	push	a
	cpw	x, (0x08, sp)
	ld	a, (1, sp)
	sbc	a, (0x07, sp)
	ld	a, (0x02, sp)
	sbc	a, (0x06, sp)
	pop	a
	jrnc	00105$
;	delay.c: 29: delay_timer(100);
	push	a
	pushw	x
	push	#0x64
	call	_delay_timer
	pop	a
	popw	x
	pop	a
;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
	addw	x, #0x0001
	adc	a, #0x00
	push	a
	ld	a, (0x02, sp)
	adc	a, #0x00
	ld	(0x02, sp), a
	pop	a
	jra	00103$
00105$:
	addw	sp, #8
	ret
;	delay.c: 33: void delay_timer(unsigned char deltim)
;	-----------------------------------------
;	 function delay_timer
;	-----------------------------------------
_delay_timer:
;	delay.c: 35: TIM4_CR1 = (1<<TIM4_CR1_CEN);
	mov	0x5340+0, #0x01
;	delay.c: 36: while(TIM4_CNTR<deltim);
00101$:
	ldw	x, #0x5346
	ld	a, (x)
	cp	a, (0x03, sp)
	jrc	00101$
;	delay.c: 37: TIM4_CR1 = (0<<TIM4_CR1_CEN);
	mov	0x5340+0, #0x00
;	delay.c: 38: TIM4_CNTR = 0; //reset timer	
	mov	0x5346+0, #0x00
	ret
;	periph_stm8s.c: 16: void clock_init()
;	-----------------------------------------
;	 function clock_init
;	-----------------------------------------
_clock_init:
;	periph_stm8s.c: 18: CLK_CKDIVR = 0x00; //fMASTER = fCPU = fHSI = 16MHz
	mov	0x50c6+0, #0x00
;	periph_stm8s.c: 19: CLK_ICKR = (1<<CLK_ICKR_HSIEN);	//High Speed Internal RC Oscillator enabled
	mov	0x50c0+0, #0x01
	ret
;	periph_stm8s.c: 24: void i2c_init()
;	-----------------------------------------
;	 function i2c_init
;	-----------------------------------------
_i2c_init:
;	periph_stm8s.c: 26: I2C_CR1 = (0<<I2C_CR1_PE); //Disable I2C before configuration starts
	mov	0x5210+0, #0x00
;	periph_stm8s.c: 27: I2C_FREQR = 16;	//fCLK = 16 MHz
	mov	0x5212+0, #0x10
;	periph_stm8s.c: 28: I2C_CCRH = (0<<I2C_CCRH_FS)|(0<<I2C_CCRH_CCR11)|(0<<I2C_CCRH_CCR10)|(0<<I2C_CCRH_CCR9)|(0<<I2C_CCRH_CCR8); //Standard Mode
	mov	0x521c+0, #0x00
;	periph_stm8s.c: 29: I2C_CCRL = 0x80;  //Clock Speed = 100 kHz
	mov	0x521b+0, #0x80
;	periph_stm8s.c: 31: I2C_OARH = (0<<I2C_OARH_ADDMODE)|(1<<I2C_OARH_ADDCONF); //7-bit address mode, ADDCONF always must be 1
	mov	0x5214+0, #0x40
;	periph_stm8s.c: 32: I2C_TRISER = 17;  //Setup Bus Characteristic
	mov	0x521d+0, #0x11
;	periph_stm8s.c: 37: I2C_CR1 = (1<<I2C_CR1_PE);  //Enable I2C after configuration complete
	mov	0x5210+0, #0x01
	ret
;	periph_stm8s.c: 40: void i2c_set_start()
;	-----------------------------------------
;	 function i2c_set_start
;	-----------------------------------------
_i2c_set_start:
;	periph_stm8s.c: 42: I2C_CR2 |= (1<<I2C_CR2_START);
	bset	0x5211, #0
	ret
;	periph_stm8s.c: 45: void i2c_set_address(unsigned char addr, unsigned char dir)
;	-----------------------------------------
;	 function i2c_set_address
;	-----------------------------------------
_i2c_set_address:
;	periph_stm8s.c: 47: if(dir==I2C_READ) I2C_DR = (addr<<1)|dir;
	ld	a, (0x03, sp)
	ld	xl, a
	sllw	x
	ld	a, (0x04, sp)
	cp	a, #0x01
	jrne	00104$
	ld	a, xl
	or	a, (0x04, sp)
	ldw	x, #0x5216
	ld	(x), a
	jra	00106$
00104$:
;	periph_stm8s.c: 48: else if(dir==I2C_WRITE) I2C_DR = (addr<<1)&dir;
	ld	a, (0x04, sp)
	cp	a, #0xfe
	jrne	00106$
	ld	a, xl
	and	a, (0x04, sp)
	ldw	x, #0x5216
	ld	(x), a
00106$:
	ret
;	periph_stm8s.c: 52: void i2c_set_stop()
;	-----------------------------------------
;	 function i2c_set_stop
;	-----------------------------------------
_i2c_set_stop:
;	periph_stm8s.c: 54: I2C_CR2 |= (1<<I2C_CR2_STOP);
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
	ret
;	periph_stm8s.c: 57: void i2c_clear_ack()
;	-----------------------------------------
;	 function i2c_clear_ack
;	-----------------------------------------
_i2c_clear_ack:
;	periph_stm8s.c: 59: I2C_CR2 &= ~(1<<I2C_CR2_ACK); //Disable Acknowledge
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
	ret
;	periph_stm8s.c: 62: void i2c_set_ack()
;	-----------------------------------------
;	 function i2c_set_ack
;	-----------------------------------------
_i2c_set_ack:
;	periph_stm8s.c: 64: I2C_CR2 |= (1<<I2C_CR2_ACK); //Enable Acknowledge
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
	ret
;	periph_stm8s.c: 67: void i2c_ack_pos_current()
;	-----------------------------------------
;	 function i2c_ack_pos_current
;	-----------------------------------------
_i2c_ack_pos_current:
;	periph_stm8s.c: 69: I2C_CR2 &= ~(1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
	ldw	x, #0x5211
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
	ret
;	periph_stm8s.c: 72: void i2c_ack_pos_next()
;	-----------------------------------------
;	 function i2c_ack_pos_next
;	-----------------------------------------
_i2c_ack_pos_next:
;	periph_stm8s.c: 74: I2C_CR2 |= (1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the next byte which will be received in the shift register
	ldw	x, #0x5211
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
	ret
;	periph_stm8s.c: 77: void i2c_poll_SB()
;	-----------------------------------------
;	 function i2c_poll_SB
;	-----------------------------------------
_i2c_poll_SB:
;	periph_stm8s.c: 79: while((I2C_SR1&(1<<I2C_SR1_SB))!=(1<<I2C_SR1_SB)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x01
	cp	a, #0x01
	jrne	00101$
	ret
;	periph_stm8s.c: 82: void i2c_poll_ADDR()
;	-----------------------------------------
;	 function i2c_poll_ADDR
;	-----------------------------------------
_i2c_poll_ADDR:
;	periph_stm8s.c: 84: while((I2C_SR1&(1<<I2C_SR1_ADDR))!=(1<<I2C_SR1_ADDR)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x02
	cp	a, #0x02
	jrne	00101$
	ret
;	periph_stm8s.c: 87: void i2c_poll_BTF()
;	-----------------------------------------
;	 function i2c_poll_BTF
;	-----------------------------------------
_i2c_poll_BTF:
;	periph_stm8s.c: 89: while((I2C_SR1&(1<<I2C_SR1_BTF))!=(1<<I2C_SR1_BTF)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x04
	cp	a, #0x04
	jrne	00101$
	ret
;	periph_stm8s.c: 92: void i2c_poll_TXE()
;	-----------------------------------------
;	 function i2c_poll_TXE
;	-----------------------------------------
_i2c_poll_TXE:
;	periph_stm8s.c: 94: while((I2C_SR1&(1<<I2C_SR1_TXE))!=(1<<I2C_SR1_TXE)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x80
	cp	a, #0x80
	jrne	00101$
	ret
;	periph_stm8s.c: 97: void i2c_poll_RXNE()
;	-----------------------------------------
;	 function i2c_poll_RXNE
;	-----------------------------------------
_i2c_poll_RXNE:
;	periph_stm8s.c: 99: while((I2C_SR1&(1<<I2C_SR1_RXNE))!=(1<<I2C_SR1_RXNE)){}
00101$:
	ldw	x, #0x5217
	ld	a, (x)
	and	a, #0x40
	cp	a, #0x40
	jrne	00101$
	ret
;	periph_stm8s.c: 102: void i2c_clear_bits()
;	-----------------------------------------
;	 function i2c_clear_bits
;	-----------------------------------------
_i2c_clear_bits:
;	periph_stm8s.c: 104: readreg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
	ld	_readreg+0, a
	ret
;	periph_stm8s.c: 107: void i2c_clear_ADDR()
;	-----------------------------------------
;	 function i2c_clear_ADDR
;	-----------------------------------------
_i2c_clear_ADDR:
;	periph_stm8s.c: 109: readreg = I2C_SR1;
	ldw	x, #0x5217
	ld	a, (x)
;	periph_stm8s.c: 110: readreg = I2C_SR3;
	ldw	x, #0x5219
	ld	a, (x)
	ld	_readreg+0, a
	ret
;	periph_stm8s.c: 113: void i2c_enable_interrupts()
;	-----------------------------------------
;	 function i2c_enable_interrupts
;	-----------------------------------------
_i2c_enable_interrupts:
;	periph_stm8s.c: 115: I2C_ITR = 0x07;
	mov	0x521a+0, #0x07
	ret
;	periph_stm8s.c: 117: void i2c_disable_interrupts()
;	-----------------------------------------
;	 function i2c_disable_interrupts
;	-----------------------------------------
_i2c_disable_interrupts:
;	periph_stm8s.c: 119: I2C_ITR = 0x00;
	mov	0x521a+0, #0x00
	ret
;	periph_stm8s.c: 124: void adc_init()
;	-----------------------------------------
;	 function adc_init
;	-----------------------------------------
_adc_init:
;	periph_stm8s.c: 126: ADC1_CR1 = fADC_fMASTER_8<<ADC1_CR1_SPSEL; // ADCCLK = MCLK/8
	mov	0x5401+0, #0x40
;	periph_stm8s.c: 127: ADC1_CR2 = (1<<ADC1_CR2_ALIGN);  // right alignment adc data
	mov	0x5402+0, #0x08
;	periph_stm8s.c: 129: ADC1_CR1 |= (1<<ADC1_CR1_ADON);  // turn on ADC
	bset	0x5401, #0
	ret
;	periph_stm8s.c: 133: unsigned int read_adc(unsigned char adcch)
;	-----------------------------------------
;	 function read_adc
;	-----------------------------------------
_read_adc:
	sub	sp, #4
;	periph_stm8s.c: 137: ADC1_CSR &= 0xF0;  // select
	ldw	x, #0x5400
	ld	a, (x)
	and	a, #0xf0
	ld	(x), a
;	periph_stm8s.c: 138: ADC1_CSR |= adcch; // channel
	ldw	x, #0x5400
	ld	a, (x)
	or	a, (0x07, sp)
	ldw	x, #0x5400
	ld	(x), a
;	periph_stm8s.c: 141: ADC1_CR1 |= (1<<ADC1_CR1_ADON); // start conversion
	bset	0x5401, #0
;	periph_stm8s.c: 142: while(!((ADC1_CSR)&(1<<ADC1_CSR_EOC)));; // conversion is in progress
00101$:
	ldw	x, #0x5400
	ld	a, (x)
	tnz	a
	jrpl	00101$
;	periph_stm8s.c: 143: adcval = (ADC1_DRH<<8) + ADC1_DRL;
	ldw	x, #0x5404
	ld	a, (x)
	clr	(0x03, sp)
	ld	(0x01, sp), a
	clr	(0x02, sp)
	ldw	x, #0x5405
	ld	a, (x)
	clrw	x
	ld	xl, a
	addw	x, (0x01, sp)
;	periph_stm8s.c: 144: ADC1_CSR |= (0<<ADC1_CSR_EOC); // reset EOC
	ldw	y, #0x5400
	ld	a, (y)
	ldw	y, #0x5400
	ld	(y), a
;	periph_stm8s.c: 146: return adcval;
	addw	sp, #4
	ret
;	periph_stm8s.c: 151: void uart1_init(unsigned char rxien) //UART Initialization
;	-----------------------------------------
;	 function uart1_init
;	-----------------------------------------
_uart1_init:
;	periph_stm8s.c: 155: UART1_BRR1 = 0x68;
	mov	0x5232+0, #0x68
;	periph_stm8s.c: 156: UART1_BRR2 = 0x03;
	mov	0x5233+0, #0x03
;	periph_stm8s.c: 158: UART1_CR1 |= (0<<UART1_CR1_M)|(0<<UART1_CR1_PCEN); //8 bit Data; No Parity
	ldw	x, #0x5234
	ld	a, (x)
	ldw	x, #0x5234
	ld	(x), a
;	periph_stm8s.c: 159: UART1_CR3 |= (0<<UART1_CR3_STOP); //Stop Bit = 1
	ldw	x, #0x5236
	ld	a, (x)
	ldw	x, #0x5236
	ld	(x), a
;	periph_stm8s.c: 161: if(rxien==1) 
	ld	a, (0x03, sp)
	cp	a, #0x01
	jrne	00102$
;	periph_stm8s.c: 163: UART1_CR2 |= (1<<UART1_CR2_RIEN); //Enable Interrupt on Receiver Mode
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x20
	ld	(x), a
;	periph_stm8s.c: 164: ITC_SPR5 = (level_2<<ITC_SPR5_VECT18); //UART Interrupt Setting
	mov	0x7f74+0, #0x00
00102$:
;	periph_stm8s.c: 167: UART1_CR2 |= (1<<UART1_CR2_TEN); //Enable Transmitter Mode
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
;	periph_stm8s.c: 168: UART1_CR2 |= (1<<UART1_CR2_REN); //Enable Receiver Mode
	ldw	x, #0x5235
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
	ret
;	periph_stm8s.c: 171: void uart1_send(unsigned char usend) //UART Transmit a Byte
;	-----------------------------------------
;	 function uart1_send
;	-----------------------------------------
_uart1_send:
;	periph_stm8s.c: 173: UART1_DR = usend; //Write to UART Data Register
	ldw	x, #0x5231
	ld	a, (0x03, sp)
	ld	(x), a
;	periph_stm8s.c: 174: while((UART1_SR&(1<<UART1_SR_TXE))!=(1<<UART1_SR_TXE)); //Wait until Transmission complete
00101$:
	ldw	x, #0x5230
	ld	a, (x)
	and	a, #0x80
	cp	a, #0x80
	jrne	00101$
	ret
;	periph_stm8s.c: 177: unsigned char uart1_recv() //UART Receive a Byte (using Polling)
;	-----------------------------------------
;	 function uart1_recv
;	-----------------------------------------
_uart1_recv:
;	periph_stm8s.c: 180: if((UART1_SR&(1<<UART1_SR_RXNE))==(1<<UART1_SR_RXNE)) //Check if any data in Data Register
	ldw	x, #0x5230
	ld	a, (x)
	and	a, #0x20
	cp	a, #0x20
	jrne	00102$
;	periph_stm8s.c: 182: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
	ldw	x, #0x5231
	ld	a, (x)
;	periph_stm8s.c: 184: else urecv=0;
	.byte 0x21
00102$:
	clr	a
00103$:
;	periph_stm8s.c: 185: return urecv;
	ret
;	periph_stm8s.c: 188: unsigned char uart1_recv_i() //UART Receive a Byte (using Interrupt)
;	-----------------------------------------
;	 function uart1_recv_i
;	-----------------------------------------
_uart1_recv_i:
;	periph_stm8s.c: 191: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
	ldw	x, #0x5231
	ld	a, (x)
;	periph_stm8s.c: 192: return urecv;
	ret
;	periph_stm8s.c: 198: void pwm1_init(unsigned int timval)
;	-----------------------------------------
;	 function pwm1_init
;	-----------------------------------------
_pwm1_init:
	sub	sp, #2
;	periph_stm8s.c: 200: TIM1_PSCRH = 0x00; //TIM_CLK = CLK
	mov	0x5260+0, #0x00
;	periph_stm8s.c: 201: TIM1_PSCRL = 0x00; //TIM_CLK = CLK
	mov	0x5261+0, #0x00
;	periph_stm8s.c: 202: TIM1_ARRH = (timval >> 8); //TIM RELOAD
	ld	a, (0x05, sp)
	clr	(0x01, sp)
	ldw	x, #0x5262
	ld	(x), a
;	periph_stm8s.c: 203: TIM1_ARRL = (timval & 0x00FF); //TIM RELOAD
	ld	a, (0x06, sp)
	ld	xh, a
	clr	a
	ld	a, xh
	ldw	x, #0x5263
	ld	(x), a
;	periph_stm8s.c: 204: pwm1ch1_enable();
	call	_pwm1ch1_enable
;	periph_stm8s.c: 205: TIM1_CCER1 |= (0<<TIM1_CCER1_CC1P); //Output active high
	ldw	x, #0x525c
	ld	a, (x)
	ldw	x, #0x525c
	ld	(x), a
;	periph_stm8s.c: 206: TIM1_CCMR1 = (TIM1_OCxREF_PWM_mode1<<TIM1_CCMR1_OC1M); //PWM MODE 1 for Channel 1
	mov	0x5258+0, #0x60
;	periph_stm8s.c: 207: pwm1_update(0x0000); //Start Value
	clrw	x
	pushw	x
	call	_pwm1_update
	addw	sp, #2
;	periph_stm8s.c: 208: TIM1_BKR = (1<<TIM1_BKR_MOE); //ENABLE MAIN OUTPUT 
	mov	0x526d+0, #0x80
;	periph_stm8s.c: 209: TIM1_CR1 |= (1<<TIM1_CR1_CEN); //ENABLE TIM
	ldw	x, #0x5250
	ld	a, (x)
	or	a, #0x01
	ld	(x), a
	addw	sp, #2
	ret
;	periph_stm8s.c: 212: void pwm2_init(unsigned int timval)
;	-----------------------------------------
;	 function pwm2_init
;	-----------------------------------------
_pwm2_init:
	sub	sp, #2
;	periph_stm8s.c: 214: TIM2_PSCR = 0x00; //TIM_CLK = CLK
	mov	0x530e+0, #0x00
;	periph_stm8s.c: 215: TIM2_ARRH = (timval >> 8); //TIM RELOAD
	ld	a, (0x05, sp)
	clr	(0x01, sp)
	ldw	x, #0x530f
	ld	(x), a
;	periph_stm8s.c: 216: TIM2_ARRL = (timval & 0x00FF); //TIM RELOAD
	ld	a, (0x06, sp)
	ld	xh, a
	clr	a
	ld	a, xh
	ldw	x, #0x5310
	ld	(x), a
;	periph_stm8s.c: 217: pwm2ch1_enable();
	call	_pwm2ch1_enable
;	periph_stm8s.c: 218: TIM2_CCER1 |= (0<<TIM2_CCER1_CC1P); //Output active high
	ldw	x, #0x530a
	ld	a, (x)
	ldw	x, #0x530a
	ld	(x), a
;	periph_stm8s.c: 219: TIM2_CCMR1 = (TIM2_OCxREF_PWM_mode1<<TIM2_CCMR1_OC1M); //PWM MODE 1 for Channel 1 
	mov	0x5307+0, #0x60
;	periph_stm8s.c: 220: pwm2_update(0x0000); //Start Value
	clrw	x
	pushw	x
	call	_pwm2_update
	addw	sp, #2
;	periph_stm8s.c: 221: TIM2_CR1 |= (1<<TIM2_CR1_CEN); //ENABLE TIM
	ldw	x, #0x5300
	ld	a, (x)
	or	a, #0x01
	ld	(x), a
	addw	sp, #2
	ret
;	periph_stm8s.c: 224: void pwm1ch1_enable()
;	-----------------------------------------
;	 function pwm1ch1_enable
;	-----------------------------------------
_pwm1ch1_enable:
;	periph_stm8s.c: 226: TIM1_CCER1 |= (1<<TIM1_CCER1_CC1E);
	bset	0x525c, #0
	ret
;	periph_stm8s.c: 229: void pwm1ch1_disable()
;	-----------------------------------------
;	 function pwm1ch1_disable
;	-----------------------------------------
_pwm1ch1_disable:
;	periph_stm8s.c: 231: TIM1_CCER1 &= ~(1<<TIM1_CCER1_CC1E);
	bres	0x525c, #0
	ret
;	periph_stm8s.c: 234: void pwm2ch1_enable()
;	-----------------------------------------
;	 function pwm2ch1_enable
;	-----------------------------------------
_pwm2ch1_enable:
;	periph_stm8s.c: 236: TIM2_CCER1 |= (1<<TIM2_CCER1_CC1E);
	bset	0x530a, #0
	ret
;	periph_stm8s.c: 239: void pwm2ch1_disable()
;	-----------------------------------------
;	 function pwm2ch1_disable
;	-----------------------------------------
_pwm2ch1_disable:
;	periph_stm8s.c: 241: TIM2_CCER1 &= ~(1<<TIM2_CCER1_CC1E);
	bres	0x530a, #0
	ret
;	periph_stm8s.c: 244: void pwm1_update(unsigned int pwmval)
;	-----------------------------------------
;	 function pwm1_update
;	-----------------------------------------
_pwm1_update:
	sub	sp, #2
;	periph_stm8s.c: 246: TIM1_CCR1L = (pwmval & 0x00FF);
	ld	a, (0x06, sp)
	ld	xh, a
	clr	a
	ld	a, xh
	ldw	x, #0x5266
	ld	(x), a
;	periph_stm8s.c: 247: TIM1_CCR1H = (pwmval >> 8);
	ld	a, (0x05, sp)
	clr	(0x01, sp)
	ldw	x, #0x5265
	ld	(x), a
	addw	sp, #2
	ret
;	periph_stm8s.c: 250: void pwm2_update(unsigned int pwmval)
;	-----------------------------------------
;	 function pwm2_update
;	-----------------------------------------
_pwm2_update:
	sub	sp, #2
;	periph_stm8s.c: 252: TIM2_CCR1L = (pwmval & 0x00FF);
	ld	a, (0x06, sp)
	ld	xh, a
	clr	a
	ld	a, xh
	ldw	x, #0x5312
	ld	(x), a
;	periph_stm8s.c: 253: TIM2_CCR1H = (pwmval >> 8);
	ld	a, (0x05, sp)
	clr	(0x01, sp)
	ldw	x, #0x5311
	ld	(x), a
	addw	sp, #2
	ret
;	lcd_n1202_stm8s.c: 7: void lcdn1202_gpio_init()
;	-----------------------------------------
;	 function lcdn1202_gpio_init
;	-----------------------------------------
_lcdn1202_gpio_init:
;	lcd_n1202_stm8s.c: 9: LCDDDR |= (OUTPUT<<LCDDAT)|(OUTPUT<<LCDCLK)|(OUTPUT<<LCDBL);	//Configure GPIO as Output
	ldw	x, #0x5002
	ld	a, (x)
	or	a, #0x0e
	ld	(x), a
;	lcd_n1202_stm8s.c: 10: LCDCR1 |= (pushpull<<LCDDAT)|(pushpull<<LCDCLK)|(pushpull<<LCDBL); //Configure Output Type
	ldw	x, #0x5003
	ld	a, (x)
	or	a, #0x0e
	ld	(x), a
;	lcd_n1202_stm8s.c: 11: LCDCR2 |= (speed_10MHz<<LCDDAT)|(speed_10MHz<<LCDCLK)|(speed_10MHz<<LCDBL); //Configure GPIO speed
	ldw	x, #0x5004
	ld	a, (x)
	or	a, #0x0e
	ld	(x), a
;	lcd_n1202_stm8s.c: 12: LCDODR = 0x00; //Starting value
	mov	0x5000+0, #0x00
	ret
;	lcd_n1202_stm8s.c: 15: void lcdn1202_9bsend(unsigned char cdsign, unsigned char comdat)
;	-----------------------------------------
;	 function lcdn1202_9bsend
;	-----------------------------------------
_lcdn1202_9bsend:
	push	a
;	lcd_n1202_stm8s.c: 19: if(cdsign==0) LCDODR &= LCDDAT_MASKL; //1st bit is 0 for Command
	tnz	(0x04, sp)
	jrne	00102$
	ldw	x, #0x5000
	ld	a, (x)
	and	a, #0xfd
	ld	(x), a
	jra	00103$
00102$:
;	lcd_n1202_stm8s.c: 20: else LCDODR |= LCDDAT_MASKH; //1st bit is 1 for Data
	ldw	x, #0x5000
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
00103$:
;	lcd_n1202_stm8s.c: 21: lcdn1202_clock1();
	call	_lcdn1202_clock1
;	lcd_n1202_stm8s.c: 23: for(cdi=0;cdi<8;cdi++) //Send 2nd-9th bit
	clr	(0x01, sp)
00108$:
;	lcd_n1202_stm8s.c: 25: if(comdat & 0x80) LCDODR |= LCDDAT_MASKH; //LCDDAT = '1'
	tnz	(0x05, sp)
	jrpl	00105$
	ldw	x, #0x5000
	ld	a, (x)
	or	a, #0x02
	ld	(x), a
	jra	00106$
00105$:
;	lcd_n1202_stm8s.c: 26: else LCDODR &= LCDDAT_MASKL;		  //LCDDAT = '0'
	ldw	x, #0x5000
	ld	a, (x)
	and	a, #0xfd
	ld	(x), a
00106$:
;	lcd_n1202_stm8s.c: 27: lcdn1202_clock1();
	call	_lcdn1202_clock1
;	lcd_n1202_stm8s.c: 28: comdat <<= 1; //Shift to next bit
	sll	(0x05, sp)
;	lcd_n1202_stm8s.c: 23: for(cdi=0;cdi<8;cdi++) //Send 2nd-9th bit
	inc	(0x01, sp)
	ld	a, (0x01, sp)
	cp	a, #0x08
	jrc	00108$
;	lcd_n1202_stm8s.c: 30: LCDODR &= LCDDAT_MASKL;
	ldw	x, #0x5000
	ld	a, (x)
	and	a, #0xfd
	ld	(x), a
	pop	a
	ret
;	lcd_n1202_stm8s.c: 33: void lcdn1202_clock1()
;	-----------------------------------------
;	 function lcdn1202_clock1
;	-----------------------------------------
_lcdn1202_clock1:
;	lcd_n1202_stm8s.c: 35: LCDODR |= LCDCLK_MASKH; //Send 1 pulse to LCDCLK
	ldw	x, #0x5000
	ld	a, (x)
	or	a, #0x04
	ld	(x), a
;	lcd_n1202_stm8s.c: 36: delay_us(1); //Short delay
	push	#0x01
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_us
	addw	sp, #4
;	lcd_n1202_stm8s.c: 37: LCDODR &= LCDCLK_MASKL;
	ldw	x, #0x5000
	ld	a, (x)
	and	a, #0xfb
	ld	(x), a
	ret
;	lcd_n1202_stm8s.c: 40: void lcdn1202_blon()
;	-----------------------------------------
;	 function lcdn1202_blon
;	-----------------------------------------
_lcdn1202_blon:
;	lcd_n1202_stm8s.c: 42: LCDODR |= LCDBL_MASKH; //LCDBL = '1'
	ldw	x, #0x5000
	ld	a, (x)
	or	a, #0x08
	ld	(x), a
	ret
;	lcd_n1202_stm8s.c: 45: void lcdn1202_bloff()
;	-----------------------------------------
;	 function lcdn1202_bloff
;	-----------------------------------------
_lcdn1202_bloff:
;	lcd_n1202_stm8s.c: 47: LCDODR &= LCDBL_MASKL; //LCDBL = '0'
	ldw	x, #0x5000
	ld	a, (x)
	and	a, #0xf7
	ld	(x), a
	ret
;	lcd_n1202.c: 9: void lcdn1202_init()
;	-----------------------------------------
;	 function lcdn1202_init
;	-----------------------------------------
_lcdn1202_init:
;	lcd_n1202.c: 11: lcdn1202_gpio_init();
	call	_lcdn1202_gpio_init
;	lcd_n1202.c: 15: delay_ms(10);
	push	#0x0a
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_ms
	addw	sp, #4
;	lcd_n1202.c: 17: lcdn1202_sendcom(0xE2);	//Soft Reset
	push	#0xe2
	call	_lcdn1202_sendcom
	pop	a
;	lcd_n1202.c: 18: delay_ms(1);
	push	#0x01
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_ms
	addw	sp, #4
;	lcd_n1202.c: 19: lcdn1202_sendcom(0xA4); //Normal Display Mode
	push	#0xa4
	call	_lcdn1202_sendcom
	pop	a
;	lcd_n1202.c: 20: lcdn1202_sendcom(0x2F);	//Power Control = Max (Booster On, VReg On, VFol On)
	push	#0x2f
	call	_lcdn1202_sendcom
	pop	a
;	lcd_n1202.c: 22: lcdn1202_sendcom(0xA0); //Segment Driver Direction = Normal (lines start at left)
	push	#0xa0
	call	_lcdn1202_sendcom
	pop	a
;	lcd_n1202.c: 23: lcdn1202_sendcom(0xC0); //Common Driver Direction = Normal
	push	#0xc0
	call	_lcdn1202_sendcom
	pop	a
;	lcd_n1202.c: 24: lcdn1202_sendcom(0x80|16); //Set Contrast to default
	push	#0x90
	call	_lcdn1202_sendcom
	pop	a
;	lcd_n1202.c: 26: lcdn1202_sendcom(0xAF);	//Display On
	push	#0xaf
	call	_lcdn1202_sendcom
	pop	a
;	lcd_n1202.c: 28: LCD_BL_OFF(); //Backlight off
	call	_LCD_BL_OFF
;	lcd_n1202.c: 29: LCD_clear();  //Clear pixel memory
	call	_LCD_clear
;	lcd_n1202.c: 30: LCD_BL_ON();  //Backlight on
	jp	_LCD_BL_ON
;	lcd_n1202.c: 33: void lcdn1202_sendcom(unsigned char ssd1306com)
;	-----------------------------------------
;	 function lcdn1202_sendcom
;	-----------------------------------------
_lcdn1202_sendcom:
;	lcd_n1202.c: 35: lcdn1202_9bsend(0,ssd1306com); //Send Command
	ld	a, (0x03, sp)
	push	a
	push	#0x00
	call	_lcdn1202_9bsend
	addw	sp, #2
	ret
;	lcd_n1202.c: 38: void lcdn1202_senddat(unsigned char ssd1306dat)
;	-----------------------------------------
;	 function lcdn1202_senddat
;	-----------------------------------------
_lcdn1202_senddat:
;	lcd_n1202.c: 40: lcdn1202_9bsend(1,ssd1306dat); //Send Data
	ld	a, (0x03, sp)
	push	a
	push	#0x01
	call	_lcdn1202_9bsend
	addw	sp, #2
	ret
;	lcd_n1202.c: 43: void lcdn1202_setpos(unsigned char row, unsigned char col)
;	-----------------------------------------
;	 function lcdn1202_setpos
;	-----------------------------------------
_lcdn1202_setpos:
;	lcd_n1202.c: 45: lcdn1202_sendcom(0xB0|(row&0x0F)); //Set page of row
	ld	a, (0x03, sp)
	and	a, #0x0f
	or	a, #0xb0
	push	a
	call	_lcdn1202_sendcom
	pop	a
;	lcd_n1202.c: 46: lcdn1202_sendcom(0x00|(col&0x0F)); //Set lower nibble of Column
	ld	a, (0x04, sp)
	and	a, #0x0f
	push	a
	call	_lcdn1202_sendcom
	pop	a
;	lcd_n1202.c: 47: lcdn1202_sendcom(0x10|((col>>4)&0x0F)); //Set upper nibble of Column
	ld	a, (0x04, sp)
	swap	a
	and	a, #0x0f
	and	a, #0x0f
	or	a, #0x10
	push	a
	call	_lcdn1202_sendcom
	pop	a
	ret
;	lcd_n1202.c: 50: void lcdn1202_clear()
;	-----------------------------------------
;	 function lcdn1202_clear
;	-----------------------------------------
_lcdn1202_clear:
	push	a
;	lcd_n1202.c: 53: lcdn1202_setpos(0,0);
	push	#0x00
	push	#0x00
	call	_lcdn1202_setpos
	addw	sp, #2
;	lcd_n1202.c: 54: for(row=0;row<LCDN1202_ROW;row++)	//Scan rows (pages)
	clr	(0x01, sp)
;	lcd_n1202.c: 56: for(col=0;col<LCDN1202_COL;col++)	//Scan columns
00109$:
	clr	a
00103$:
;	lcd_n1202.c: 58: lcdn1202_senddat(0);	//Send 0 to every pixel
	push	a
	push	#0x00
	call	_lcdn1202_senddat
	pop	a
	pop	a
;	lcd_n1202.c: 56: for(col=0;col<LCDN1202_COL;col++)	//Scan columns
	inc	a
	cp	a, #0x60
	jrc	00103$
;	lcd_n1202.c: 54: for(row=0;row<LCDN1202_ROW;row++)	//Scan rows (pages)
	inc	(0x01, sp)
	ld	a, (0x01, sp)
	cp	a, #0x09
	jrc	00109$
	pop	a
	ret
;	lcd_n1202.c: 63: void LCD_setpos(unsigned char row, unsigned char col)
;	-----------------------------------------
;	 function LCD_setpos
;	-----------------------------------------
_LCD_setpos:
;	lcd_n1202.c: 65: lcdn1202_setpos(row,col); //Set coordinate (for LCD_drawbyte)
	ld	a, (0x04, sp)
	push	a
	ld	a, (0x04, sp)
	push	a
	call	_lcdn1202_setpos
	addw	sp, #2
	ret
;	lcd_n1202.c: 68: void LCD_drawbyte(unsigned char dbyte)
;	-----------------------------------------
;	 function LCD_drawbyte
;	-----------------------------------------
_LCD_drawbyte:
;	lcd_n1202.c: 70: lcdn1202_senddat(dbyte); //Send 1 byte data only
	ld	a, (0x03, sp)
	push	a
	call	_lcdn1202_senddat
	pop	a
	ret
;	lcd_n1202.c: 73: void LCD_drawchar(unsigned char chr, unsigned char chrrow, unsigned char chrcol)
;	-----------------------------------------
;	 function LCD_drawchar
;	-----------------------------------------
_LCD_drawchar:
	sub	sp, #11
;	lcd_n1202.c: 78: lcdn1202_setpos(chrrow,chrcol);
	ld	a, (0x10, sp)
	push	a
	ld	a, (0x10, sp)
	push	a
	call	_lcdn1202_setpos
	addw	sp, #2
;	lcd_n1202.c: 83: chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
	ld	a, (0x0e, sp)
	ld	(0x0b, sp), a
	clr	(0x0a, sp)
;	lcd_n1202.c: 80: if((chr>31)&&(chr<128))	//Alphanumeric & Punctuation Area
	ld	a, (0x0e, sp)
	cp	a, #0x1f
	jrule	00107$
	ld	a, (0x0e, sp)
	cp	a, #0x80
	jrnc	00107$
;	lcd_n1202.c: 82: lcdn1202_senddat(0x00);
	push	#0x00
	call	_lcdn1202_senddat
	pop	a
;	lcd_n1202.c: 83: chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
	ldw	x, (0x0a, sp)
	subw	x, #0x0020
	pushw	x
	push	#0x05
	push	#0x00
	call	__mulint
	addw	sp, #4
	ldw	(0x08, sp), x
;	lcd_n1202.c: 84: for(ci=0;ci<5;ci++)
	ldw	x, #_font_arr+0
	ldw	(0x06, sp), x
	clr	a
00110$:
;	lcd_n1202.c: 86: fchar = font_arr[chridx+ci]; //Get character pattern from Font Array
	clrw	x
	ld	xl, a
	addw	x, (0x08, sp)
	addw	x, (0x06, sp)
	push	a
	ld	a, (x)
	ld	xl, a
	pop	a
;	lcd_n1202.c: 87: lcdn1202_senddat(fchar); //Send pattern 1 byte at a time
	push	a
	pushw	x
	addw	sp, #1
	call	_lcdn1202_senddat
	pop	a
	pop	a
;	lcd_n1202.c: 84: for(ci=0;ci<5;ci++)
	inc	a
	cp	a, #0x05
	jrc	00110$
	jra	00114$
00107$:
;	lcd_n1202.c: 90: else if((chr>127)&&(chr<148))	//Frame & Arrow Area
	ld	a, (0x0e, sp)
	cp	a, #0x7f
	jrule	00114$
	ld	a, (0x0e, sp)
	cp	a, #0x94
	jrnc	00114$
;	lcd_n1202.c: 92: chridx=(chr-128)*8; //Start at index 128. 5 columns for each symbol
	ldw	x, (0x0a, sp)
	subw	x, #0x0080
	sllw	x
	sllw	x
	sllw	x
;	lcd_n1202.c: 93: for(ci=0;ci<8;ci++)
	ldw	y, #_font_arr+0
	ldw	(0x04, sp), y
	addw	x, #0x01e0
	ldw	(0x02, sp), x
	clr	(0x01, sp)
00112$:
;	lcd_n1202.c: 95: fchar = font_arr[chridx+480+ci]; //Get symbol pattern from Font Array
	clrw	x
	ld	a, (0x01, sp)
	ld	xl, a
	addw	x, (0x02, sp)
	addw	x, (0x04, sp)
	ld	a, (x)
;	lcd_n1202.c: 96: lcdn1202_senddat(fchar); //Send pattern 1 byte at a time
	push	a
	call	_lcdn1202_senddat
	pop	a
;	lcd_n1202.c: 93: for(ci=0;ci<8;ci++)
	inc	(0x01, sp)
	ld	a, (0x01, sp)
	cp	a, #0x08
	jrc	00112$
00114$:
	addw	sp, #11
	ret
;	lcd_n1202.c: 102: void LCD_drawtext(char *text, unsigned char txtrow, unsigned char txtcol)
;	-----------------------------------------
;	 function LCD_drawtext
;	-----------------------------------------
_LCD_drawtext:
	sub	sp, #2
;	lcd_n1202.c: 106: while(text[stridx] != 0) //Scan characters in string
	clrw	x
	ldw	(0x01, sp), x
00101$:
	ldw	x, (0x05, sp)
	addw	x, (0x01, sp)
	ld	a, (x)
	ld	xl, a
	tnz	a
	jreq	00104$
;	lcd_n1202.c: 108: LCD_drawchar(text[stridx],txtrow,txtcol+(8*stridx)); //Display each character
	ld	a, (0x02, sp)
	sll	a
	sll	a
	sll	a
	add	a, (0x08, sp)
	push	a
	ld	a, (0x08, sp)
	push	a
	ld	a, xl
	push	a
	call	_LCD_drawchar
	addw	sp, #3
;	lcd_n1202.c: 109: stridx++;
	ldw	x, (0x01, sp)
	incw	x
	ldw	(0x01, sp), x
	jra	00101$
00104$:
	addw	sp, #2
	ret
;	lcd_n1202.c: 113: void LCD_drawint(unsigned int num, unsigned char numrow, unsigned char numcol)
;	-----------------------------------------
;	 function LCD_drawint
;	-----------------------------------------
_LCD_drawint:
	sub	sp, #12
;	lcd_n1202.c: 121: numb = num;
	ldw	x, (0x0f, sp)
;	lcd_n1202.c: 122: while(numb!=0) //Counting digit
	clr	a
00101$:
	tnzw	x
	jreq	00114$
;	lcd_n1202.c: 124: ndigit++;
	inc	a
;	lcd_n1202.c: 125: numb /= 10; 
	ldw	y, #0x000a
	divw	x, y
	jra	00101$
00114$:
	ld	(0x0a, sp), a
;	lcd_n1202.c: 127: for(nd=0;nd<ndigit;nd++) //Converting each digit
	clr	a
	ldw	x, sp
	addw	x, #3
	ldw	(0x0b, sp), x
00106$:
	cp	a, (0x0a, sp)
	jrnc	00104$
;	lcd_n1202.c: 129: numb = num%10;
	ldw	x, (0x0f, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	(0x01, sp), y
;	lcd_n1202.c: 130: num = num/10;
	ldw	x, (0x0f, sp)
	ldw	y, #0x000a
	divw	x, y
	ldw	(0x0f, sp), x
;	lcd_n1202.c: 131: ibuff[ndigit-(nd+1)] = numb + '0'; //Start from last_index-1
	inc	a
	ld	(0x09, sp), a
	ld	a, (0x0a, sp)
	sub	a, (0x09, sp)
	clrw	x
	ld	xl, a
	addw	x, (0x0b, sp)
	ld	a, (0x02, sp)
	add	a, #0x30
	ld	(x), a
;	lcd_n1202.c: 127: for(nd=0;nd<ndigit;nd++) //Converting each digit
	ld	a, (0x09, sp)
	jra	00106$
00104$:
;	lcd_n1202.c: 133: ibuff[ndigit] = '\0'; //Last character is null
	clrw	x
	ld	a, (0x0a, sp)
	ld	xl, a
	addw	x, (0x0b, sp)
	clr	(x)
;	lcd_n1202.c: 135: LCD_drawtext(ibuff,numrow,numcol); //Display number as text
	ldw	x, (0x0b, sp)
	ld	a, (0x12, sp)
	push	a
	ld	a, (0x12, sp)
	push	a
	pushw	x
	call	_LCD_drawtext
	addw	sp, #16
	ret
;	lcd_n1202.c: 138: void LCD_clear()
;	-----------------------------------------
;	 function LCD_clear
;	-----------------------------------------
_LCD_clear:
;	lcd_n1202.c: 140: lcdn1202_sendcom(0xAE);  //Set Display off
	push	#0xae
	call	_lcdn1202_sendcom
	pop	a
;	lcd_n1202.c: 141: lcdn1202_clear(); //Clear display
	call	_lcdn1202_clear
;	lcd_n1202.c: 142: lcdn1202_sendcom(0xAF); //Set Display on
	push	#0xaf
	call	_lcdn1202_sendcom
	pop	a
	ret
;	lcd_n1202.c: 145: void LCD_clearblock(unsigned char row, unsigned char col_start, unsigned char col_fin)
;	-----------------------------------------
;	 function LCD_clearblock
;	-----------------------------------------
_LCD_clearblock:
;	lcd_n1202.c: 149: lcdn1202_setpos(row,col_start); //Set start position
	ld	a, (0x04, sp)
	push	a
	ld	a, (0x04, sp)
	push	a
	call	_lcdn1202_setpos
	addw	sp, #2
;	lcd_n1202.c: 150: for(col=col_start;col<=col_fin;col++) //Scan columns
	ld	a, (0x04, sp)
00103$:
	cp	a, (0x05, sp)
	jrugt	00105$
;	lcd_n1202.c: 152: lcdn1202_senddat(0);	//Send 0 to every pixel in a column
	push	a
	push	#0x00
	call	_lcdn1202_senddat
	pop	a
	pop	a
;	lcd_n1202.c: 150: for(col=col_start;col<=col_fin;col++) //Scan columns
	inc	a
	jra	00103$
00105$:
	ret
;	lcd_n1202.c: 156: void LCD_normal()
;	-----------------------------------------
;	 function LCD_normal
;	-----------------------------------------
_LCD_normal:
;	lcd_n1202.c: 158: lcdn1202_sendcom(0xA6);	//Black Pixel in White Background
	push	#0xa6
	call	_lcdn1202_sendcom
	pop	a
	ret
;	lcd_n1202.c: 161: void LCD_reverse()
;	-----------------------------------------
;	 function LCD_reverse
;	-----------------------------------------
_LCD_reverse:
;	lcd_n1202.c: 163: lcdn1202_sendcom(0xA7);	//White Pixel in Black Background
	push	#0xa7
	call	_lcdn1202_sendcom
	pop	a
	ret
;	lcd_n1202.c: 166: void LCD_BL_ON()
;	-----------------------------------------
;	 function LCD_BL_ON
;	-----------------------------------------
_LCD_BL_ON:
;	lcd_n1202.c: 168: lcdn1202_blon(); //Backlight on
	jp	_lcdn1202_blon
;	lcd_n1202.c: 171: void LCD_BL_OFF()
;	-----------------------------------------
;	 function LCD_BL_OFF
;	-----------------------------------------
_LCD_BL_OFF:
;	lcd_n1202.c: 173: lcdn1202_bloff(); //Backlight off
	jp	_lcdn1202_bloff
;	powerman.c: 6: void powerman_init() //GPIOs Initialization 
;	-----------------------------------------
;	 function powerman_init
;	-----------------------------------------
_powerman_init:
;	powerman.c: 8: POWLATCHDDR |= (OUTPUT<<POW_LATCH);
	ldw	x, #0x5016
	ld	a, (x)
	or	a, #0x20
	ld	(x), a
;	powerman.c: 9: POWLATCHCR1 |= (pushpull<<POW_LATCH);
	ldw	x, #0x5017
	ld	a, (x)
	or	a, #0x20
	ld	(x), a
;	powerman.c: 10: POWLATCHCR2 |= (speed_2MHz<<POW_LATCH);
	ldw	x, #0x5018
	ld	a, (x)
	ldw	x, #0x5018
	ld	(x), a
;	powerman.c: 12: POWDETDDR |= (INPUT<<POW_DET);
	ldw	x, #0x501b
	ld	a, (x)
	ldw	x, #0x501b
	ld	(x), a
;	powerman.c: 13: POWDETCR1 |= (floating<<POW_DET);
	ldw	x, #0x501c
	ld	a, (x)
	ldw	x, #0x501c
	ld	(x), a
;	powerman.c: 14: POWDETCR2 |= (exti_disabled<<POW_DET);
	ldw	x, #0x501d
	ld	a, (x)
	ldw	x, #0x501d
	ld	(x), a
;	powerman.c: 16: CHGDDR |= (INPUT<<CHG_PR) | (INPUT<<CHG_FL);
	ldw	x, #0x5007
	ld	a, (x)
	ldw	x, #0x5007
	ld	(x), a
;	powerman.c: 17: CHGCR1 |= (pullup<<CHG_PR) | (pullup<<CHG_FL);
	ldw	x, #0x5008
	ld	a, (x)
	or	a, #0xc0
	ld	(x), a
;	powerman.c: 18: CHGCR2 |= (exti_disabled<<CHG_PR) | (exti_disabled<<CHG_FL);
	ldw	x, #0x5009
	ld	a, (x)
	ldw	x, #0x5009
	ld	(x), a
	ret
;	powerman.c: 21: void Power_Latch() //Activate Power-Latch : latch Power-Switch transistor
;	-----------------------------------------
;	 function Power_Latch
;	-----------------------------------------
_Power_Latch:
;	powerman.c: 23: POWLATCHODR |= POWLATCH_MASKH; //POWER-LATCH = 1
	ldw	x, #0x5014
	ld	a, (x)
	or	a, #0x20
	ld	(x), a
	ret
;	powerman.c: 26: void Power_Unlatch() //Release Power-Latch : unlatch Power-Switch transistor
;	-----------------------------------------
;	 function Power_Unlatch
;	-----------------------------------------
_Power_Unlatch:
;	powerman.c: 28: POWLATCHODR &= POWLATCH_MASKL; //POWER-LATCH = 0
	ldw	x, #0x5014
	ld	a, (x)
	and	a, #0xdf
	ld	(x), a
	ret
;	powerman.c: 31: unsigned char read_pkey() //Check Powerkey State
;	-----------------------------------------
;	 function read_pkey
;	-----------------------------------------
_read_pkey:
;	powerman.c: 35: pkeyval = POWDETIDR & POWDET_MASKH; //Read POWER-DETECT state
	ldw	x, #0x501a
	ld	a, (x)
	and	a, #0x10
;	powerman.c: 37: return pkeyval;
	ret
;	powerman.c: 40: unsigned int vbat_mon() //Battery Voltage reading
;	-----------------------------------------
;	 function vbat_mon
;	-----------------------------------------
_vbat_mon:
;	powerman.c: 47: vbat = read_adc(BAT_LVL)*66/40*3; //VADC_MAX=3333mV,VBAT_MAX=5000mV
	push	#0x03
	call	_read_adc
	pop	a
	pushw	x
	push	#0x42
	push	#0x00
	call	__mulint
	addw	sp, #4
	ldw	y, #0x0028
	divw	x, y
	pushw	x
	push	#0x03
	push	#0x00
	call	__mulint
	addw	sp, #4
;	powerman.c: 49: return vbat;
	ret
;	powerman.c: 52: unsigned char chgst_mon() //Charging Status reading
;	-----------------------------------------
;	 function chgst_mon
;	-----------------------------------------
_chgst_mon:
;	powerman.c: 57: chgst = CHGIDR & CHGST_MASKH; //Check CHG-PR & CHG-FL
	ldw	x, #0x5006
	ld	a, (x)
	and	a, #0xc0
;	powerman.c: 59: return chgst;
	ret
;	main.c: 40: int main()
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
;	main.c: 43: clock_init();
	call	_clock_init
;	main.c: 44: delay_init();
	call	_delay_init
;	main.c: 45: powerman_init();
	call	_powerman_init
;	main.c: 46: adc_init();
	call	_adc_init
;	main.c: 47: lcdn1202_init();
	call	_lcdn1202_init
;	main.c: 48: LCD_clear();
	call	_LCD_clear
;	main.c: 51: pkeylock = 1; //lock powerkey for next press/click
	mov	_pkeylock+0, #0x01
;	main.c: 52: chg_disp = 0; //Charging Status & Battery Voltage is not displayed
	clr	_chg_disp+0
;	main.c: 55: pkeyp = read_pkey(); //check if Powerkey is pressed
	call	_read_pkey
;	main.c: 56: if(pkeyp==POWKEY_PRESSED) Page_Main(); //if pressed then Power-On Mode, go to Main Page
	ld	_pkeyp+0, a
	cp	a, #0x10
	jrne	00102$
	call	_Page_Main
	jra	00103$
00102$:
;	main.c: 57: else Page_Charging(); //if not pressed then Charging Mode, go to Charging Page
	call	_Page_Charging
00103$:
;	main.c: 59: loop();
	call	_loop
;	main.c: 60: return 0;
	clrw	x
	ret
;	main.c: 66: void loop()
;	-----------------------------------------
;	 function loop
;	-----------------------------------------
_loop:
;	main.c: 69: ktime = 0; //start Long-Press counter with 0
	clr	_ktime+0
;	main.c: 71: while(1)
00111$:
;	main.c: 73: if((read_pkey()==0)&&(pkeylock!=0)) pkeylock = 0; //unlock Powerkey from Long-Press
	call	_read_pkey
	tnz	a
	jrne	00107$
	tnz	_pkeylock+0
	jreq	00107$
	clr	_pkeylock+0
;	main.c: 76: while((read_pkey()!=POWKEY_UNPRESSED)&&(pkeylock==0)) //check if Powerkey is pressed and unlocked
00107$:
	call	_read_pkey
	tnz	a
	jreq	00109$
	tnz	_pkeylock+0
	jrne	00109$
;	main.c: 78: pkeyp = read_pkey();
	call	_read_pkey
	ld	_pkeyp+0, a
;	main.c: 79: delay_ms(KDELAY); //Anti-Bouncing delay
	push	#0x14
	clrw	x
	pushw	x
	push	#0x00
	call	_delay_ms
	addw	sp, #4
;	main.c: 80: ktime++; //increment Long-Press counter
	ld	a, _ktime+0
	inc	a
;	main.c: 81: if(ktime>KLONGP) break; //detect Long-Press
	ld	_ktime+0, a
	cp	a, #0x64
	jrule	00107$
00109$:
;	main.c: 83: update_pkey(); //Execute Action for Powerkey
	call	_update_pkey
;	main.c: 85: disp_bat_status(); //Display Battery Status
	call	_disp_bat_status
	jra	00111$
	ret
;	main.c: 94: void Page_Charging() //Charging Mode
;	-----------------------------------------
;	 function Page_Charging
;	-----------------------------------------
_Page_Charging:
;	main.c: 96: page_id = 0; //Charging Page ID
	clr	_page_id+0
;	main.c: 97: LCD_clear();
	call	_LCD_clear
;	main.c: 99: chg_disp = 1; //Charging Status displayed
	mov	_chg_disp+0, #0x01
;	main.c: 100: LCD_drawtext("CHARGING",2,16);
	ldw	x, #___str_0+0
	push	#0x10
	push	#0x02
	pushw	x
	call	_LCD_drawtext
	addw	sp, #4
;	main.c: 101: delay_ms(2000); //disable Powerkey for a while
	push	#0xd0
	push	#0x07
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 102: pkeyp = 0; //reset Powerkey flag
	clr	_pkeyp+0
;	main.c: 103: scf = 1; //default toggle state of Single Click in Charging Mode
	mov	_scf+0, #0x01
	ret
;	main.c: 106: void Page_Main() //Power-On Mode
;	-----------------------------------------
;	 function Page_Main
;	-----------------------------------------
_Page_Main:
;	main.c: 108: page_id = 1; //Power-On Page ID
	mov	_page_id+0, #0x01
;	main.c: 109: Power_Latch(); //activate Power-Latch
	call	_Power_Latch
;	main.c: 110: LCD_clear();
	call	_LCD_clear
;	main.c: 112: chg_disp = 0; //Charging Status not displayed
	clr	_chg_disp+0
;	main.c: 113: LCD_drawtext("POWER ON",2,16);
	ldw	x, #___str_1+0
	push	#0x10
	push	#0x02
	pushw	x
	call	_LCD_drawtext
	addw	sp, #4
;	main.c: 114: delay_ms(2000); //disable Powerkey for a while
	push	#0xd0
	push	#0x07
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 115: LCD_clearblock(2,16,79);
	push	#0x4f
	push	#0x10
	push	#0x02
	call	_LCD_clearblock
	addw	sp, #3
;	main.c: 116: pkeyp = 0; //reset Powerkey flag
	clr	_pkeyp+0
;	main.c: 117: scf = 1; //default toggle state of Single Click in Power-On Mode
	mov	_scf+0, #0x01
	ret
;	main.c: 123: void update_pkey() //Action options for Powerkey
;	-----------------------------------------
;	 function update_pkey
;	-----------------------------------------
_update_pkey:
	sub	sp, #2
;	main.c: 125: if((pkeyp==POWKEY_PRESSED)&&(page_id==1)&&(ktime>KLONGP)) //Long-Press in Power-On Mode
	ld	a, _pkeyp+0
	cp	a, #0x10
	jrne	00171$
	ld	a, #0x01
	ld	(0x02, sp), a
	jra	00172$
00171$:
	clr	(0x02, sp)
00172$:
	ld	a, _page_id+0
	cp	a, #0x01
	jrne	00174$
	ld	a, #0x01
	ld	(0x01, sp), a
	jra	00175$
00174$:
	clr	(0x01, sp)
00175$:
	ld	a, _ktime+0
	cp	a, #0x64
	jrugt	00176$
	clr	a
	jra	00177$
00176$:
	ld	a, #0x01
00177$:
	tnz	(0x02, sp)
	jreq	00116$
	tnz	(0x01, sp)
	jreq	00116$
	tnz	a
	jreq	00116$
;	main.c: 127: pkeyp = 0; //reset Powerkey value
	clr	_pkeyp+0
;	main.c: 128: pkeylock = 1; //lock Powerkey for next press/click
	mov	_pkeylock+0, #0x01
;	main.c: 129: Power_Unlatch(); //release Power-Latch
	call	_Power_Unlatch
;	main.c: 130: LCD_clear();
	call	_LCD_clear
;	main.c: 131: LCD_drawtext("POWR OFF",2,16);
	ldw	x, #___str_2+0
	push	#0x10
	push	#0x02
	pushw	x
	call	_LCD_drawtext
	addw	sp, #4
;	main.c: 132: delay_ms(2000); //make sure Page_Charging is not pop-up after Power-Off
	push	#0xd0
	push	#0x07
	clrw	x
	pushw	x
	call	_delay_ms
	addw	sp, #4
;	main.c: 133: Page_Charging(); //change to Charging Mode
	call	_Page_Charging
	jra	00117$
00116$:
;	main.c: 135: else if((pkeyp==POWKEY_PRESSED)&&(page_id==1)&&(ktime<=KLONGP)) //Single-Click in Power-On Mode
	tnz	(0x02, sp)
	jreq	00111$
	tnz	(0x01, sp)
	jreq	00111$
	tnz	a
	jrne	00111$
;	main.c: 137: pkeyp = 0; //reset Powerkey value
	clr	_pkeyp+0
;	main.c: 138: on_single_click();
	call	_on_single_click
	jra	00117$
00111$:
;	main.c: 141: else if((pkeyp==POWKEY_PRESSED)&&(page_id==0)&&(ktime>KLONGP)) //Long-Press in Charging Mode
	tnz	(0x02, sp)
	jreq	00106$
	tnz	_page_id+0
	jrne	00106$
	tnz	a
	jreq	00106$
;	main.c: 143: pkeyp = 0; //reset Powerkey value
	clr	_pkeyp+0
;	main.c: 144: pkeylock = 1; //lock Powerkey for next press/click
	mov	_pkeylock+0, #0x01
;	main.c: 145: Page_Main(); //change to Power-On Mode
	call	_Page_Main
	jra	00117$
00106$:
;	main.c: 147: else if((pkeyp==POWKEY_PRESSED)&&(page_id==0)&&(ktime<=KLONGP)) //Single-Click in Charging Mode
	tnz	(0x02, sp)
	jreq	00117$
	tnz	_page_id+0
	jrne	00117$
	tnz	a
	jrne	00117$
;	main.c: 149: pkeyp = 0; //reset Powerkey value
	clr	_pkeyp+0
;	main.c: 150: chg_single_click();
	call	_chg_single_click
00117$:
;	main.c: 153: ktime = 0;
	clr	_ktime+0
	addw	sp, #2
	ret
;	main.c: 159: void on_single_click() //Single-Click action on Power-On Mode
;	-----------------------------------------
;	 function on_single_click
;	-----------------------------------------
_on_single_click:
;	main.c: 161: if(scf==0)
	tnz	_scf+0
	jrne	00104$
;	main.c: 163: LCD_BL_ON(); //Backlight On
	call	_LCD_BL_ON
;	main.c: 164: scf = 1;
	mov	_scf+0, #0x01
	jra	00106$
00104$:
;	main.c: 166: else if(scf==1)
	ld	a, _scf+0
	cp	a, #0x01
	jrne	00106$
;	main.c: 168: LCD_BL_OFF(); //Backlight Off
	call	_LCD_BL_OFF
;	main.c: 169: scf = 0;
	clr	_scf+0
00106$:
	ret
;	main.c: 174: void chg_single_click() //Single-Click action on Charging Mode
;	-----------------------------------------
;	 function chg_single_click
;	-----------------------------------------
_chg_single_click:
;	main.c: 176: if(scf==0)
	tnz	_scf+0
	jrne	00104$
;	main.c: 178: LCD_drawtext("        ",4,16); //Clear text
	ldw	x, #___str_3+0
	push	#0x10
	push	#0x04
	pushw	x
	call	_LCD_drawtext
	addw	sp, #4
;	main.c: 179: scf = 1;
	mov	_scf+0, #0x01
	jra	00106$
00104$:
;	main.c: 181: else if(scf==1)
	ld	a, _scf+0
	cp	a, #0x01
	jrne	00106$
;	main.c: 183: LCD_drawtext("bonusoid",4,16); //Display text
	ldw	x, #___str_4+0
	push	#0x10
	push	#0x04
	pushw	x
	call	_LCD_drawtext
	addw	sp, #4
;	main.c: 184: scf = 0;
	clr	_scf+0
00106$:
	ret
;	main.c: 192: void disp_bat_status()
;	-----------------------------------------
;	 function disp_bat_status
;	-----------------------------------------
_disp_bat_status:
;	main.c: 195: chgst = chgst_mon();
	call	_chgst_mon
;	main.c: 197: if(chgst==CHG_PROGRESS) LCD_drawtext("BCHRG",0,0);
	ld	_chgst+0, a
	cp	a, #0x80
	jrne	00111$
	ldw	x, #___str_5+0
	push	#0x00
	push	#0x00
	pushw	x
	call	_LCD_drawtext
	addw	sp, #4
	jra	00112$
00111$:
;	main.c: 198: else if(chgst==CHG_FULL) LCD_drawtext("BFULL",0,0);
	ld	a, _chgst+0
	cp	a, #0x40
	jrne	00108$
	ldw	x, #___str_6+0
	push	#0x00
	push	#0x00
	pushw	x
	call	_LCD_drawtext
	addw	sp, #4
	jra	00112$
00108$:
;	main.c: 199: else if(chgst==CHG_NOCHG) LCD_drawtext("NOCHG",0,0);
	ld	a, _chgst+0
	cp	a, #0xc0
	jrne	00105$
	ldw	x, #___str_7+0
	push	#0x00
	push	#0x00
	pushw	x
	call	_LCD_drawtext
	addw	sp, #4
	jra	00112$
00105$:
;	main.c: 200: else if(chgst==CHG_NOBAT) LCD_drawtext("NOBAT",0,0);
	tnz	_chgst+0
	jrne	00102$
	ldw	x, #___str_8+0
	push	#0x00
	push	#0x00
	pushw	x
	call	_LCD_drawtext
	addw	sp, #4
	jra	00112$
00102$:
;	main.c: 201: else LCD_drawtext("UNKWN",0,0);
	ldw	x, #___str_9+0
	push	#0x00
	push	#0x00
	pushw	x
	call	_LCD_drawtext
	addw	sp, #4
00112$:
;	main.c: 204: LCD_drawtext("mV",0,80);
	ldw	x, #___str_10+0
	push	#0x50
	push	#0x00
	pushw	x
	call	_LCD_drawtext
	addw	sp, #4
;	main.c: 205: vbat = vbat_mon();
	call	_vbat_mon
	ldw	_vbat+0, x
;	main.c: 206: LCD_drawint(vbat,0,48);
	push	#0x30
	push	#0x00
	push	_vbat+1
	push	_vbat+0
	call	_LCD_drawint
	addw	sp, #4
	ret
	.area CODE
_font_arr:
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x5F	; 95
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x05	; 5
	.db #0x03	; 3
	.db #0x00	; 0
	.db #0x05	; 5
	.db #0x03	; 3
	.db #0x14	; 20
	.db #0x7F	; 127
	.db #0x14	; 20
	.db #0x7F	; 127
	.db #0x14	; 20
	.db #0x24	; 36
	.db #0x2A	; 42
	.db #0x7F	; 127
	.db #0x2A	; 42
	.db #0x12	; 18
	.db #0x23	; 35
	.db #0x13	; 19
	.db #0x08	; 8
	.db #0x64	; 100	'd'
	.db #0x62	; 98	'b'
	.db #0x36	; 54	'6'
	.db #0x49	; 73	'I'
	.db #0x55	; 85	'U'
	.db #0x22	; 34
	.db #0x50	; 80	'P'
	.db #0x00	; 0
	.db #0x05	; 5
	.db #0x03	; 3
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x1C	; 28
	.db #0x22	; 34
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x22	; 34
	.db #0x1C	; 28
	.db #0x00	; 0
	.db #0x0A	; 10
	.db #0x04	; 4
	.db #0x1F	; 31
	.db #0x04	; 4
	.db #0x0A	; 10
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x3E	; 62
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x50	; 80	'P'
	.db #0x30	; 48	'0'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x60	; 96
	.db #0x60	; 96
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x20	; 32
	.db #0x10	; 16
	.db #0x08	; 8
	.db #0x04	; 4
	.db #0x02	; 2
	.db #0x3E	; 62
	.db #0x51	; 81	'Q'
	.db #0x49	; 73	'I'
	.db #0x45	; 69	'E'
	.db #0x3E	; 62
	.db #0x00	; 0
	.db #0x42	; 66	'B'
	.db #0x7F	; 127
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x42	; 66	'B'
	.db #0x61	; 97	'a'
	.db #0x51	; 81	'Q'
	.db #0x49	; 73	'I'
	.db #0x46	; 70	'F'
	.db #0x22	; 34
	.db #0x41	; 65	'A'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x36	; 54	'6'
	.db #0x18	; 24
	.db #0x14	; 20
	.db #0x12	; 18
	.db #0x7F	; 127
	.db #0x10	; 16
	.db #0x27	; 39
	.db #0x45	; 69	'E'
	.db #0x45	; 69	'E'
	.db #0x45	; 69	'E'
	.db #0x39	; 57	'9'
	.db #0x3E	; 62
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x32	; 50	'2'
	.db #0x61	; 97	'a'
	.db #0x11	; 17
	.db #0x09	; 9
	.db #0x05	; 5
	.db #0x03	; 3
	.db #0x36	; 54	'6'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x36	; 54	'6'
	.db #0x26	; 38
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x3E	; 62
	.db #0x00	; 0
	.db #0x36	; 54	'6'
	.db #0x36	; 54	'6'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x56	; 86	'V'
	.db #0x36	; 54	'6'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x22	; 34
	.db #0x00	; 0
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x00	; 0
	.db #0x22	; 34
	.db #0x14	; 20
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x02	; 2
	.db #0x01	; 1
	.db #0x51	; 81	'Q'
	.db #0x09	; 9
	.db #0x06	; 6
	.db #0x32	; 50	'2'
	.db #0x49	; 73	'I'
	.db #0x79	; 121	'y'
	.db #0x41	; 65	'A'
	.db #0x3E	; 62
	.db #0x7C	; 124
	.db #0x12	; 18
	.db #0x11	; 17
	.db #0x12	; 18
	.db #0x7C	; 124
	.db #0x7F	; 127
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x36	; 54	'6'
	.db #0x3E	; 62
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x22	; 34
	.db #0x7F	; 127
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x22	; 34
	.db #0x1C	; 28
	.db #0x7F	; 127
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x7F	; 127
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x3E	; 62
	.db #0x41	; 65	'A'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x3A	; 58
	.db #0x7F	; 127
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x7F	; 127
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x7F	; 127
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x41	; 65	'A'
	.db #0x3F	; 63
	.db #0x01	; 1
	.db #0x7F	; 127
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x22	; 34
	.db #0x41	; 65	'A'
	.db #0x7F	; 127
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x7F	; 127
	.db #0x02	; 2
	.db #0x0C	; 12
	.db #0x02	; 2
	.db #0x7F	; 127
	.db #0x7F	; 127
	.db #0x04	; 4
	.db #0x08	; 8
	.db #0x10	; 16
	.db #0x7F	; 127
	.db #0x3E	; 62
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x3E	; 62
	.db #0x7F	; 127
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x06	; 6
	.db #0x3E	; 62
	.db #0x41	; 65	'A'
	.db #0x51	; 81	'Q'
	.db #0x21	; 33
	.db #0x5E	; 94
	.db #0x7F	; 127
	.db #0x09	; 9
	.db #0x19	; 25
	.db #0x29	; 41
	.db #0x46	; 70	'F'
	.db #0x26	; 38
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x49	; 73	'I'
	.db #0x32	; 50	'2'
	.db #0x01	; 1
	.db #0x01	; 1
	.db #0x7F	; 127
	.db #0x01	; 1
	.db #0x01	; 1
	.db #0x3F	; 63
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x3F	; 63
	.db #0x1F	; 31
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x20	; 32
	.db #0x1F	; 31
	.db #0x3F	; 63
	.db #0x40	; 64
	.db #0x38	; 56	'8'
	.db #0x40	; 64
	.db #0x3F	; 63
	.db #0x63	; 99	'c'
	.db #0x14	; 20
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x63	; 99	'c'
	.db #0x07	; 7
	.db #0x08	; 8
	.db #0x70	; 112	'p'
	.db #0x08	; 8
	.db #0x07	; 7
	.db #0x61	; 97	'a'
	.db #0x51	; 81	'Q'
	.db #0x49	; 73	'I'
	.db #0x45	; 69	'E'
	.db #0x43	; 67	'C'
	.db #0x00	; 0
	.db #0x7F	; 127
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x02	; 2
	.db #0x04	; 4
	.db #0x08	; 8
	.db #0x10	; 16
	.db #0x20	; 32
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x41	; 65	'A'
	.db #0x7F	; 127
	.db #0x00	; 0
	.db #0x04	; 4
	.db #0x02	; 2
	.db #0x01	; 1
	.db #0x02	; 2
	.db #0x04	; 4
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x01	; 1
	.db #0x02	; 2
	.db #0x04	; 4
	.db #0x00	; 0
	.db #0x20	; 32
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x78	; 120	'x'
	.db #0x7F	; 127
	.db #0x50	; 80	'P'
	.db #0x48	; 72	'H'
	.db #0x48	; 72	'H'
	.db #0x30	; 48	'0'
	.db #0x38	; 56	'8'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x28	; 40
	.db #0x30	; 48	'0'
	.db #0x48	; 72	'H'
	.db #0x48	; 72	'H'
	.db #0x50	; 80	'P'
	.db #0x7F	; 127
	.db #0x38	; 56	'8'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x18	; 24
	.db #0x08	; 8
	.db #0x7E	; 126
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x02	; 2
	.db #0x08	; 8
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x3C	; 60
	.db #0x7F	; 127
	.db #0x10	; 16
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x70	; 112	'p'
	.db #0x00	; 0
	.db #0x48	; 72	'H'
	.db #0x7A	; 122	'z'
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x48	; 72	'H'
	.db #0x3A	; 58
	.db #0x00	; 0
	.db #0x7F	; 127
	.db #0x10	; 16
	.db #0x28	; 40
	.db #0x44	; 68	'D'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x7F	; 127
	.db #0x40	; 64
	.db #0x00	; 0
	.db #0x7C	; 124
	.db #0x04	; 4
	.db #0x7C	; 124
	.db #0x04	; 4
	.db #0x78	; 120	'x'
	.db #0x7C	; 124
	.db #0x08	; 8
	.db #0x04	; 4
	.db #0x04	; 4
	.db #0x78	; 120	'x'
	.db #0x38	; 56	'8'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x38	; 56	'8'
	.db #0x7C	; 124
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x14	; 20
	.db #0x14	; 20
	.db #0x18	; 24
	.db #0x7C	; 124
	.db #0x7C	; 124
	.db #0x08	; 8
	.db #0x04	; 4
	.db #0x04	; 4
	.db #0x08	; 8
	.db #0x48	; 72	'H'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x54	; 84	'T'
	.db #0x20	; 32
	.db #0x04	; 4
	.db #0x3F	; 63
	.db #0x44	; 68	'D'
	.db #0x44	; 68	'D'
	.db #0x20	; 32
	.db #0x3C	; 60
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0x20	; 32
	.db #0x7C	; 124
	.db #0x1C	; 28
	.db #0x20	; 32
	.db #0x40	; 64
	.db #0x20	; 32
	.db #0x1C	; 28
	.db #0x3C	; 60
	.db #0x40	; 64
	.db #0x38	; 56	'8'
	.db #0x40	; 64
	.db #0x3C	; 60
	.db #0x44	; 68	'D'
	.db #0x28	; 40
	.db #0x10	; 16
	.db #0x28	; 40
	.db #0x44	; 68	'D'
	.db #0x0C	; 12
	.db #0x50	; 80	'P'
	.db #0x50	; 80	'P'
	.db #0x50	; 80	'P'
	.db #0x3C	; 60
	.db #0x44	; 68	'D'
	.db #0x64	; 100	'd'
	.db #0x54	; 84	'T'
	.db #0x4C	; 76	'L'
	.db #0x44	; 68	'D'
	.db #0x00	; 0
	.db #0x08	; 8
	.db #0x36	; 54	'6'
	.db #0x41	; 65	'A'
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x7F	; 127
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x41	; 65	'A'
	.db #0x36	; 54	'6'
	.db #0x08	; 8
	.db #0x00	; 0
	.db #0x10	; 16
	.db #0x08	; 8
	.db #0x08	; 8
	.db #0x10	; 16
	.db #0x08	; 8
	.db #0x06	; 6
	.db #0x09	; 9
	.db #0x09	; 9
	.db #0x06	; 6
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0xF8	; 248
	.db #0xF8	; 248
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0xF8	; 248
	.db #0xF8	; 248
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0xF8	; 248
	.db #0xF8	; 248
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x1F	; 31
	.db #0x1F	; 31
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x1F	; 31
	.db #0x1F	; 31
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x1F	; 31
	.db #0x1F	; 31
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x18	; 24
	.db #0x0C	; 12
	.db #0x06	; 6
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x06	; 6
	.db #0x0C	; 12
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x30	; 48	'0'
	.db #0x60	; 96
	.db #0xFF	; 255
	.db #0xFF	; 255
	.db #0x60	; 96
	.db #0x30	; 48	'0'
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x3C	; 60
	.db #0x7E	; 126
	.db #0xDB	; 219
	.db #0x99	; 153
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x18	; 24
	.db #0x99	; 153
	.db #0xDB	; 219
	.db #0x7E	; 126
	.db #0x3C	; 60
	.db #0x18	; 24
	.db #0x7F	; 127
	.db #0x7F	; 127
	.db #0x0F	; 15
	.db #0x1F	; 31
	.db #0x3B	; 59
	.db #0x73	; 115	's'
	.db #0xE3	; 227
	.db #0x40	; 64
	.db #0x40	; 64
	.db #0xE3	; 227
	.db #0x73	; 115	's'
	.db #0x3B	; 59
	.db #0x1F	; 31
	.db #0x0F	; 15
	.db #0x7F	; 127
	.db #0x7F	; 127
	.db #0xFE	; 254
	.db #0xFE	; 254
	.db #0xF0	; 240
	.db #0xF8	; 248
	.db #0xDC	; 220
	.db #0xCE	; 206
	.db #0xC7	; 199
	.db #0x02	; 2
	.db #0x02	; 2
	.db #0xC7	; 199
	.db #0xCE	; 206
	.db #0xDC	; 220
	.db #0xF8	; 248
	.db #0xF0	; 240
	.db #0xFE	; 254
	.db #0xFE	; 254
	.db #0x3C	; 60
	.db #0x42	; 66	'B'
	.db #0x81	; 129
	.db #0x99	; 153
	.db #0x99	; 153
	.db #0x81	; 129
	.db #0x42	; 66	'B'
	.db #0x3C	; 60
___str_0:
	.ascii "CHARGING"
	.db 0x00
___str_1:
	.ascii "POWER ON"
	.db 0x00
___str_2:
	.ascii "POWR OFF"
	.db 0x00
___str_3:
	.ascii "        "
	.db 0x00
___str_4:
	.ascii "bonusoid"
	.db 0x00
___str_5:
	.ascii "BCHRG"
	.db 0x00
___str_6:
	.ascii "BFULL"
	.db 0x00
___str_7:
	.ascii "NOCHG"
	.db 0x00
___str_8:
	.ascii "NOBAT"
	.db 0x00
___str_9:
	.ascii "UNKWN"
	.db 0x00
___str_10:
	.ascii "mV"
	.db 0x00
	.area INITIALIZER
	.area CABS (ABS)
