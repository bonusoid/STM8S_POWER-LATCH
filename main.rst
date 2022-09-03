                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 3.5.0 #9253 (Apr  3 2018) (Linux)
                                      4 ; This file was generated Sat Sep  3 15:01:27 2022
                                      5 ;--------------------------------------------------------
                                      6 	.module main
                                      7 	.optsdcc -mstm8
                                      8 	
                                      9 ;--------------------------------------------------------
                                     10 ; Public variables in this module
                                     11 ;--------------------------------------------------------
                                     12 	.globl _font_arr
                                     13 	.globl _main
                                     14 	.globl _read_pkey
                                     15 	.globl _chgst
                                     16 	.globl _vbat
                                     17 	.globl _scf
                                     18 	.globl _ktime
                                     19 	.globl _chg_disp
                                     20 	.globl _page_id
                                     21 	.globl _pkeylock
                                     22 	.globl _pkeyp
                                     23 	.globl _readreg
                                     24 	.globl _delay_init
                                     25 	.globl _delay_us
                                     26 	.globl _delay_ms
                                     27 	.globl _delay_timer
                                     28 	.globl _clock_init
                                     29 	.globl _i2c_init
                                     30 	.globl _i2c_set_start
                                     31 	.globl _i2c_set_address
                                     32 	.globl _i2c_set_stop
                                     33 	.globl _i2c_clear_ack
                                     34 	.globl _i2c_set_ack
                                     35 	.globl _i2c_ack_pos_current
                                     36 	.globl _i2c_ack_pos_next
                                     37 	.globl _i2c_poll_SB
                                     38 	.globl _i2c_poll_ADDR
                                     39 	.globl _i2c_poll_BTF
                                     40 	.globl _i2c_poll_TXE
                                     41 	.globl _i2c_poll_RXNE
                                     42 	.globl _i2c_clear_bits
                                     43 	.globl _i2c_clear_ADDR
                                     44 	.globl _i2c_enable_interrupts
                                     45 	.globl _i2c_disable_interrupts
                                     46 	.globl _adc_init
                                     47 	.globl _read_adc
                                     48 	.globl _uart1_init
                                     49 	.globl _uart1_send
                                     50 	.globl _uart1_recv
                                     51 	.globl _uart1_recv_i
                                     52 	.globl _pwm1_init
                                     53 	.globl _pwm2_init
                                     54 	.globl _pwm1ch1_enable
                                     55 	.globl _pwm1ch1_disable
                                     56 	.globl _pwm2ch1_enable
                                     57 	.globl _pwm2ch1_disable
                                     58 	.globl _pwm1_update
                                     59 	.globl _pwm2_update
                                     60 	.globl _lcdn1202_gpio_init
                                     61 	.globl _lcdn1202_9bsend
                                     62 	.globl _lcdn1202_clock1
                                     63 	.globl _lcdn1202_blon
                                     64 	.globl _lcdn1202_bloff
                                     65 	.globl _lcdn1202_init
                                     66 	.globl _lcdn1202_sendcom
                                     67 	.globl _lcdn1202_senddat
                                     68 	.globl _lcdn1202_setpos
                                     69 	.globl _lcdn1202_clear
                                     70 	.globl _LCD_setpos
                                     71 	.globl _LCD_drawbyte
                                     72 	.globl _LCD_drawchar
                                     73 	.globl _LCD_drawtext
                                     74 	.globl _LCD_drawint
                                     75 	.globl _LCD_clear
                                     76 	.globl _LCD_clearblock
                                     77 	.globl _LCD_normal
                                     78 	.globl _LCD_reverse
                                     79 	.globl _LCD_BL_ON
                                     80 	.globl _LCD_BL_OFF
                                     81 	.globl _powerman_init
                                     82 	.globl _Power_Latch
                                     83 	.globl _Power_Unlatch
                                     84 	.globl _read_pkey
                                     85 	.globl _vbat_mon
                                     86 	.globl _chgst_mon
                                     87 	.globl _loop
                                     88 	.globl _Page_Charging
                                     89 	.globl _Page_Main
                                     90 	.globl _update_pkey
                                     91 	.globl _on_single_click
                                     92 	.globl _chg_single_click
                                     93 	.globl _disp_bat_status
                                     94 ;--------------------------------------------------------
                                     95 ; ram data
                                     96 ;--------------------------------------------------------
                                     97 	.area DATA
      000001                         98 _readreg::
      000001                         99 	.ds 1
      000002                        100 _pkeyp::
      000002                        101 	.ds 1
      000003                        102 _pkeylock::
      000003                        103 	.ds 1
      000004                        104 _page_id::
      000004                        105 	.ds 1
      000005                        106 _chg_disp::
      000005                        107 	.ds 1
      000006                        108 _ktime::
      000006                        109 	.ds 1
      000007                        110 _scf::
      000007                        111 	.ds 1
      000008                        112 _vbat::
      000008                        113 	.ds 2
      00000A                        114 _chgst::
      00000A                        115 	.ds 1
                                    116 ;--------------------------------------------------------
                                    117 ; ram data
                                    118 ;--------------------------------------------------------
                                    119 	.area INITIALIZED
                                    120 ;--------------------------------------------------------
                                    121 ; Stack segment in internal ram 
                                    122 ;--------------------------------------------------------
                                    123 	.area	SSEG
      00000B                        124 __start__stack:
      00000B                        125 	.ds	1
                                    126 
                                    127 ;--------------------------------------------------------
                                    128 ; absolute external ram data
                                    129 ;--------------------------------------------------------
                                    130 	.area DABS (ABS)
                                    131 ;--------------------------------------------------------
                                    132 ; interrupt vector 
                                    133 ;--------------------------------------------------------
                                    134 	.area HOME
      008000                        135 __interrupt_vect:
      008000 82 00 80 83            136 	int s_GSINIT ;reset
      008004 82 00 00 00            137 	int 0x0000 ;trap
      008008 82 00 00 00            138 	int 0x0000 ;int0
      00800C 82 00 00 00            139 	int 0x0000 ;int1
      008010 82 00 00 00            140 	int 0x0000 ;int2
      008014 82 00 00 00            141 	int 0x0000 ;int3
      008018 82 00 00 00            142 	int 0x0000 ;int4
      00801C 82 00 00 00            143 	int 0x0000 ;int5
      008020 82 00 00 00            144 	int 0x0000 ;int6
      008024 82 00 00 00            145 	int 0x0000 ;int7
      008028 82 00 00 00            146 	int 0x0000 ;int8
      00802C 82 00 00 00            147 	int 0x0000 ;int9
      008030 82 00 00 00            148 	int 0x0000 ;int10
      008034 82 00 00 00            149 	int 0x0000 ;int11
      008038 82 00 00 00            150 	int 0x0000 ;int12
      00803C 82 00 00 00            151 	int 0x0000 ;int13
      008040 82 00 00 00            152 	int 0x0000 ;int14
      008044 82 00 00 00            153 	int 0x0000 ;int15
      008048 82 00 00 00            154 	int 0x0000 ;int16
      00804C 82 00 00 00            155 	int 0x0000 ;int17
      008050 82 00 00 00            156 	int 0x0000 ;int18
      008054 82 00 00 00            157 	int 0x0000 ;int19
      008058 82 00 00 00            158 	int 0x0000 ;int20
      00805C 82 00 00 00            159 	int 0x0000 ;int21
      008060 82 00 00 00            160 	int 0x0000 ;int22
      008064 82 00 00 00            161 	int 0x0000 ;int23
      008068 82 00 00 00            162 	int 0x0000 ;int24
      00806C 82 00 00 00            163 	int 0x0000 ;int25
      008070 82 00 00 00            164 	int 0x0000 ;int26
      008074 82 00 00 00            165 	int 0x0000 ;int27
      008078 82 00 00 00            166 	int 0x0000 ;int28
      00807C 82 00 00 00            167 	int 0x0000 ;int29
                                    168 ;--------------------------------------------------------
                                    169 ; global & static initialisations
                                    170 ;--------------------------------------------------------
                                    171 	.area HOME
                                    172 	.area GSINIT
                                    173 	.area GSFINAL
                                    174 	.area GSINIT
      008083                        175 __sdcc_gs_init_startup:
      008083                        176 __sdcc_init_data:
                                    177 ; stm8_genXINIT() start
      008083 AE 00 0A         [ 2]  178 	ldw x, #l_DATA
      008086 27 07            [ 1]  179 	jreq	00002$
      008088                        180 00001$:
      008088 72 4F 00 00      [ 1]  181 	clr (s_DATA - 1, x)
      00808C 5A               [ 2]  182 	decw x
      00808D 26 F9            [ 1]  183 	jrne	00001$
      00808F                        184 00002$:
      00808F AE 00 00         [ 2]  185 	ldw	x, #l_INITIALIZER
      008092 27 09            [ 1]  186 	jreq	00004$
      008094                        187 00003$:
      008094 D6 8D 33         [ 1]  188 	ld	a, (s_INITIALIZER - 1, x)
      008097 D7 00 0A         [ 1]  189 	ld	(s_INITIALIZED - 1, x), a
      00809A 5A               [ 2]  190 	decw	x
      00809B 26 F7            [ 1]  191 	jrne	00003$
      00809D                        192 00004$:
                                    193 ; stm8_genXINIT() end
                                    194 	.area GSFINAL
      00809D CC 80 80         [ 2]  195 	jp	__sdcc_program_startup
                                    196 ;--------------------------------------------------------
                                    197 ; Home
                                    198 ;--------------------------------------------------------
                                    199 	.area HOME
                                    200 	.area HOME
      008080                        201 __sdcc_program_startup:
      008080 CC 86 9E         [ 2]  202 	jp	_main
                                    203 ;	return from main will return to caller
                                    204 ;--------------------------------------------------------
                                    205 ; code
                                    206 ;--------------------------------------------------------
                                    207 	.area CODE
                                    208 ;	delay.c: 7: void delay_init()
                                    209 ;	-----------------------------------------
                                    210 ;	 function delay_init
                                    211 ;	-----------------------------------------
      0080A0                        212 _delay_init:
                                    213 ;	delay.c: 9: TIM4_PSCR = 4; // CLK/16
      0080A0 35 04 53 47      [ 1]  214 	mov	0x5347+0, #0x04
      0080A4 81               [ 4]  215 	ret
                                    216 ;	delay.c: 12: void delay_us(unsigned long delus)
                                    217 ;	-----------------------------------------
                                    218 ;	 function delay_us
                                    219 ;	-----------------------------------------
      0080A5                        220 _delay_us:
      0080A5 52 06            [ 2]  221 	sub	sp, #6
                                    222 ;	delay.c: 16: for(du=0;du<(delus/10);du++)
      0080A7 4B 0A            [ 1]  223 	push	#0x0a
      0080A9 5F               [ 1]  224 	clrw	x
      0080AA 89               [ 2]  225 	pushw	x
      0080AB 4B 00            [ 1]  226 	push	#0x00
      0080AD 1E 0F            [ 2]  227 	ldw	x, (0x0f, sp)
      0080AF 89               [ 2]  228 	pushw	x
      0080B0 1E 0F            [ 2]  229 	ldw	x, (0x0f, sp)
      0080B2 89               [ 2]  230 	pushw	x
      0080B3 CD 8C 5E         [ 4]  231 	call	__divulong
      0080B6 5B 08            [ 2]  232 	addw	sp, #8
      0080B8 1F 05            [ 2]  233 	ldw	(0x05, sp), x
      0080BA 17 03            [ 2]  234 	ldw	(0x03, sp), y
      0080BC 5F               [ 1]  235 	clrw	x
      0080BD 1F 01            [ 2]  236 	ldw	(0x01, sp), x
      0080BF                        237 00103$:
      0080BF 1E 01            [ 2]  238 	ldw	x, (0x01, sp)
      0080C1 90 5F            [ 1]  239 	clrw	y
      0080C3 13 05            [ 2]  240 	cpw	x, (0x05, sp)
      0080C5 90 9F            [ 1]  241 	ld	a, yl
      0080C7 12 04            [ 1]  242 	sbc	a, (0x04, sp)
      0080C9 90 9E            [ 1]  243 	ld	a, yh
      0080CB 12 03            [ 1]  244 	sbc	a, (0x03, sp)
      0080CD 24 0D            [ 1]  245 	jrnc	00101$
                                    246 ;	delay.c: 18: delay_timer(100);
      0080CF 4B 64            [ 1]  247 	push	#0x64
      0080D1 CD 81 3A         [ 4]  248 	call	_delay_timer
      0080D4 84               [ 1]  249 	pop	a
                                    250 ;	delay.c: 16: for(du=0;du<(delus/10);du++)
      0080D5 1E 01            [ 2]  251 	ldw	x, (0x01, sp)
      0080D7 5C               [ 2]  252 	incw	x
      0080D8 1F 01            [ 2]  253 	ldw	(0x01, sp), x
      0080DA 20 E3            [ 2]  254 	jra	00103$
      0080DC                        255 00101$:
                                    256 ;	delay.c: 20: delay_timer(delus%10);
      0080DC 4B 0A            [ 1]  257 	push	#0x0a
      0080DE 5F               [ 1]  258 	clrw	x
      0080DF 89               [ 2]  259 	pushw	x
      0080E0 4B 00            [ 1]  260 	push	#0x00
      0080E2 1E 0F            [ 2]  261 	ldw	x, (0x0f, sp)
      0080E4 89               [ 2]  262 	pushw	x
      0080E5 1E 0F            [ 2]  263 	ldw	x, (0x0f, sp)
      0080E7 89               [ 2]  264 	pushw	x
      0080E8 CD 8B EE         [ 4]  265 	call	__modulong
      0080EB 5B 08            [ 2]  266 	addw	sp, #8
      0080ED 9F               [ 1]  267 	ld	a, xl
      0080EE 88               [ 1]  268 	push	a
      0080EF CD 81 3A         [ 4]  269 	call	_delay_timer
      0080F2 5B 07            [ 2]  270 	addw	sp, #7
      0080F4 81               [ 4]  271 	ret
                                    272 ;	delay.c: 23: void delay_ms(unsigned long delms)
                                    273 ;	-----------------------------------------
                                    274 ;	 function delay_ms
                                    275 ;	-----------------------------------------
      0080F5                        276 _delay_ms:
      0080F5 52 08            [ 2]  277 	sub	sp, #8
                                    278 ;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
      0080F7 1E 0D            [ 2]  279 	ldw	x, (0x0d, sp)
      0080F9 89               [ 2]  280 	pushw	x
      0080FA 1E 0D            [ 2]  281 	ldw	x, (0x0d, sp)
      0080FC 89               [ 2]  282 	pushw	x
      0080FD 4B 64            [ 1]  283 	push	#0x64
      0080FF 5F               [ 1]  284 	clrw	x
      008100 89               [ 2]  285 	pushw	x
      008101 4B 00            [ 1]  286 	push	#0x00
      008103 CD 8C B8         [ 4]  287 	call	__mullong
      008106 5B 08            [ 2]  288 	addw	sp, #8
      008108 1F 07            [ 2]  289 	ldw	(0x07, sp), x
      00810A 17 05            [ 2]  290 	ldw	(0x05, sp), y
      00810C 5F               [ 1]  291 	clrw	x
      00810D 4F               [ 1]  292 	clr	a
      00810E 0F 01            [ 1]  293 	clr	(0x01, sp)
      008110                        294 00103$:
      008110 88               [ 1]  295 	push	a
      008111 13 08            [ 2]  296 	cpw	x, (0x08, sp)
      008113 7B 01            [ 1]  297 	ld	a, (1, sp)
      008115 12 07            [ 1]  298 	sbc	a, (0x07, sp)
      008117 7B 02            [ 1]  299 	ld	a, (0x02, sp)
      008119 12 06            [ 1]  300 	sbc	a, (0x06, sp)
      00811B 84               [ 1]  301 	pop	a
      00811C 24 19            [ 1]  302 	jrnc	00105$
                                    303 ;	delay.c: 29: delay_timer(100);
      00811E 88               [ 1]  304 	push	a
      00811F 89               [ 2]  305 	pushw	x
      008120 4B 64            [ 1]  306 	push	#0x64
      008122 CD 81 3A         [ 4]  307 	call	_delay_timer
      008125 84               [ 1]  308 	pop	a
      008126 85               [ 2]  309 	popw	x
      008127 84               [ 1]  310 	pop	a
                                    311 ;	delay.c: 27: for(dm=0;dm<(delms*100);dm++)
      008128 1C 00 01         [ 2]  312 	addw	x, #0x0001
      00812B A9 00            [ 1]  313 	adc	a, #0x00
      00812D 88               [ 1]  314 	push	a
      00812E 7B 02            [ 1]  315 	ld	a, (0x02, sp)
      008130 A9 00            [ 1]  316 	adc	a, #0x00
      008132 6B 02            [ 1]  317 	ld	(0x02, sp), a
      008134 84               [ 1]  318 	pop	a
      008135 20 D9            [ 2]  319 	jra	00103$
      008137                        320 00105$:
      008137 5B 08            [ 2]  321 	addw	sp, #8
      008139 81               [ 4]  322 	ret
                                    323 ;	delay.c: 33: void delay_timer(unsigned char deltim)
                                    324 ;	-----------------------------------------
                                    325 ;	 function delay_timer
                                    326 ;	-----------------------------------------
      00813A                        327 _delay_timer:
                                    328 ;	delay.c: 35: TIM4_CR1 = (1<<TIM4_CR1_CEN);
      00813A 35 01 53 40      [ 1]  329 	mov	0x5340+0, #0x01
                                    330 ;	delay.c: 36: while(TIM4_CNTR<deltim);
      00813E                        331 00101$:
      00813E AE 53 46         [ 2]  332 	ldw	x, #0x5346
      008141 F6               [ 1]  333 	ld	a, (x)
      008142 11 03            [ 1]  334 	cp	a, (0x03, sp)
      008144 25 F8            [ 1]  335 	jrc	00101$
                                    336 ;	delay.c: 37: TIM4_CR1 = (0<<TIM4_CR1_CEN);
      008146 35 00 53 40      [ 1]  337 	mov	0x5340+0, #0x00
                                    338 ;	delay.c: 38: TIM4_CNTR = 0; //reset timer	
      00814A 35 00 53 46      [ 1]  339 	mov	0x5346+0, #0x00
      00814E 81               [ 4]  340 	ret
                                    341 ;	periph_stm8s.c: 16: void clock_init()
                                    342 ;	-----------------------------------------
                                    343 ;	 function clock_init
                                    344 ;	-----------------------------------------
      00814F                        345 _clock_init:
                                    346 ;	periph_stm8s.c: 18: CLK_CKDIVR = 0x00; //fMASTER = fCPU = fHSI = 16MHz
      00814F 35 00 50 C6      [ 1]  347 	mov	0x50c6+0, #0x00
                                    348 ;	periph_stm8s.c: 19: CLK_ICKR = (1<<CLK_ICKR_HSIEN);	//High Speed Internal RC Oscillator enabled
      008153 35 01 50 C0      [ 1]  349 	mov	0x50c0+0, #0x01
      008157 81               [ 4]  350 	ret
                                    351 ;	periph_stm8s.c: 24: void i2c_init()
                                    352 ;	-----------------------------------------
                                    353 ;	 function i2c_init
                                    354 ;	-----------------------------------------
      008158                        355 _i2c_init:
                                    356 ;	periph_stm8s.c: 26: I2C_CR1 = (0<<I2C_CR1_PE); //Disable I2C before configuration starts
      008158 35 00 52 10      [ 1]  357 	mov	0x5210+0, #0x00
                                    358 ;	periph_stm8s.c: 27: I2C_FREQR = 16;	//fCLK = 16 MHz
      00815C 35 10 52 12      [ 1]  359 	mov	0x5212+0, #0x10
                                    360 ;	periph_stm8s.c: 28: I2C_CCRH = (0<<I2C_CCRH_FS)|(0<<I2C_CCRH_CCR11)|(0<<I2C_CCRH_CCR10)|(0<<I2C_CCRH_CCR9)|(0<<I2C_CCRH_CCR8); //Standard Mode
      008160 35 00 52 1C      [ 1]  361 	mov	0x521c+0, #0x00
                                    362 ;	periph_stm8s.c: 29: I2C_CCRL = 0x80;  //Clock Speed = 100 kHz
      008164 35 80 52 1B      [ 1]  363 	mov	0x521b+0, #0x80
                                    364 ;	periph_stm8s.c: 31: I2C_OARH = (0<<I2C_OARH_ADDMODE)|(1<<I2C_OARH_ADDCONF); //7-bit address mode, ADDCONF always must be 1
      008168 35 40 52 14      [ 1]  365 	mov	0x5214+0, #0x40
                                    366 ;	periph_stm8s.c: 32: I2C_TRISER = 17;  //Setup Bus Characteristic
      00816C 35 11 52 1D      [ 1]  367 	mov	0x521d+0, #0x11
                                    368 ;	periph_stm8s.c: 37: I2C_CR1 = (1<<I2C_CR1_PE);  //Enable I2C after configuration complete
      008170 35 01 52 10      [ 1]  369 	mov	0x5210+0, #0x01
      008174 81               [ 4]  370 	ret
                                    371 ;	periph_stm8s.c: 40: void i2c_set_start()
                                    372 ;	-----------------------------------------
                                    373 ;	 function i2c_set_start
                                    374 ;	-----------------------------------------
      008175                        375 _i2c_set_start:
                                    376 ;	periph_stm8s.c: 42: I2C_CR2 |= (1<<I2C_CR2_START);
      008175 72 10 52 11      [ 1]  377 	bset	0x5211, #0
      008179 81               [ 4]  378 	ret
                                    379 ;	periph_stm8s.c: 45: void i2c_set_address(unsigned char addr, unsigned char dir)
                                    380 ;	-----------------------------------------
                                    381 ;	 function i2c_set_address
                                    382 ;	-----------------------------------------
      00817A                        383 _i2c_set_address:
                                    384 ;	periph_stm8s.c: 47: if(dir==I2C_READ) I2C_DR = (addr<<1)|dir;
      00817A 7B 03            [ 1]  385 	ld	a, (0x03, sp)
      00817C 97               [ 1]  386 	ld	xl, a
      00817D 58               [ 2]  387 	sllw	x
      00817E 7B 04            [ 1]  388 	ld	a, (0x04, sp)
      008180 A1 01            [ 1]  389 	cp	a, #0x01
      008182 26 09            [ 1]  390 	jrne	00104$
      008184 9F               [ 1]  391 	ld	a, xl
      008185 1A 04            [ 1]  392 	or	a, (0x04, sp)
      008187 AE 52 16         [ 2]  393 	ldw	x, #0x5216
      00818A F7               [ 1]  394 	ld	(x), a
      00818B 20 0D            [ 2]  395 	jra	00106$
      00818D                        396 00104$:
                                    397 ;	periph_stm8s.c: 48: else if(dir==I2C_WRITE) I2C_DR = (addr<<1)&dir;
      00818D 7B 04            [ 1]  398 	ld	a, (0x04, sp)
      00818F A1 FE            [ 1]  399 	cp	a, #0xfe
      008191 26 07            [ 1]  400 	jrne	00106$
      008193 9F               [ 1]  401 	ld	a, xl
      008194 14 04            [ 1]  402 	and	a, (0x04, sp)
      008196 AE 52 16         [ 2]  403 	ldw	x, #0x5216
      008199 F7               [ 1]  404 	ld	(x), a
      00819A                        405 00106$:
      00819A 81               [ 4]  406 	ret
                                    407 ;	periph_stm8s.c: 52: void i2c_set_stop()
                                    408 ;	-----------------------------------------
                                    409 ;	 function i2c_set_stop
                                    410 ;	-----------------------------------------
      00819B                        411 _i2c_set_stop:
                                    412 ;	periph_stm8s.c: 54: I2C_CR2 |= (1<<I2C_CR2_STOP);
      00819B AE 52 11         [ 2]  413 	ldw	x, #0x5211
      00819E F6               [ 1]  414 	ld	a, (x)
      00819F AA 02            [ 1]  415 	or	a, #0x02
      0081A1 F7               [ 1]  416 	ld	(x), a
      0081A2 81               [ 4]  417 	ret
                                    418 ;	periph_stm8s.c: 57: void i2c_clear_ack()
                                    419 ;	-----------------------------------------
                                    420 ;	 function i2c_clear_ack
                                    421 ;	-----------------------------------------
      0081A3                        422 _i2c_clear_ack:
                                    423 ;	periph_stm8s.c: 59: I2C_CR2 &= ~(1<<I2C_CR2_ACK); //Disable Acknowledge
      0081A3 AE 52 11         [ 2]  424 	ldw	x, #0x5211
      0081A6 F6               [ 1]  425 	ld	a, (x)
      0081A7 A4 FB            [ 1]  426 	and	a, #0xfb
      0081A9 F7               [ 1]  427 	ld	(x), a
      0081AA 81               [ 4]  428 	ret
                                    429 ;	periph_stm8s.c: 62: void i2c_set_ack()
                                    430 ;	-----------------------------------------
                                    431 ;	 function i2c_set_ack
                                    432 ;	-----------------------------------------
      0081AB                        433 _i2c_set_ack:
                                    434 ;	periph_stm8s.c: 64: I2C_CR2 |= (1<<I2C_CR2_ACK); //Enable Acknowledge
      0081AB AE 52 11         [ 2]  435 	ldw	x, #0x5211
      0081AE F6               [ 1]  436 	ld	a, (x)
      0081AF AA 04            [ 1]  437 	or	a, #0x04
      0081B1 F7               [ 1]  438 	ld	(x), a
      0081B2 81               [ 4]  439 	ret
                                    440 ;	periph_stm8s.c: 67: void i2c_ack_pos_current()
                                    441 ;	-----------------------------------------
                                    442 ;	 function i2c_ack_pos_current
                                    443 ;	-----------------------------------------
      0081B3                        444 _i2c_ack_pos_current:
                                    445 ;	periph_stm8s.c: 69: I2C_CR2 &= ~(1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the current byte being received in the shift register
      0081B3 AE 52 11         [ 2]  446 	ldw	x, #0x5211
      0081B6 F6               [ 1]  447 	ld	a, (x)
      0081B7 A4 F7            [ 1]  448 	and	a, #0xf7
      0081B9 F7               [ 1]  449 	ld	(x), a
      0081BA 81               [ 4]  450 	ret
                                    451 ;	periph_stm8s.c: 72: void i2c_ack_pos_next()
                                    452 ;	-----------------------------------------
                                    453 ;	 function i2c_ack_pos_next
                                    454 ;	-----------------------------------------
      0081BB                        455 _i2c_ack_pos_next:
                                    456 ;	periph_stm8s.c: 74: I2C_CR2 |= (1<<I2C_CR2_POS); //ACK bit controls the (N)ACK of the next byte which will be received in the shift register
      0081BB AE 52 11         [ 2]  457 	ldw	x, #0x5211
      0081BE F6               [ 1]  458 	ld	a, (x)
      0081BF AA 08            [ 1]  459 	or	a, #0x08
      0081C1 F7               [ 1]  460 	ld	(x), a
      0081C2 81               [ 4]  461 	ret
                                    462 ;	periph_stm8s.c: 77: void i2c_poll_SB()
                                    463 ;	-----------------------------------------
                                    464 ;	 function i2c_poll_SB
                                    465 ;	-----------------------------------------
      0081C3                        466 _i2c_poll_SB:
                                    467 ;	periph_stm8s.c: 79: while((I2C_SR1&(1<<I2C_SR1_SB))!=(1<<I2C_SR1_SB)){}
      0081C3                        468 00101$:
      0081C3 AE 52 17         [ 2]  469 	ldw	x, #0x5217
      0081C6 F6               [ 1]  470 	ld	a, (x)
      0081C7 A4 01            [ 1]  471 	and	a, #0x01
      0081C9 A1 01            [ 1]  472 	cp	a, #0x01
      0081CB 26 F6            [ 1]  473 	jrne	00101$
      0081CD 81               [ 4]  474 	ret
                                    475 ;	periph_stm8s.c: 82: void i2c_poll_ADDR()
                                    476 ;	-----------------------------------------
                                    477 ;	 function i2c_poll_ADDR
                                    478 ;	-----------------------------------------
      0081CE                        479 _i2c_poll_ADDR:
                                    480 ;	periph_stm8s.c: 84: while((I2C_SR1&(1<<I2C_SR1_ADDR))!=(1<<I2C_SR1_ADDR)){}
      0081CE                        481 00101$:
      0081CE AE 52 17         [ 2]  482 	ldw	x, #0x5217
      0081D1 F6               [ 1]  483 	ld	a, (x)
      0081D2 A4 02            [ 1]  484 	and	a, #0x02
      0081D4 A1 02            [ 1]  485 	cp	a, #0x02
      0081D6 26 F6            [ 1]  486 	jrne	00101$
      0081D8 81               [ 4]  487 	ret
                                    488 ;	periph_stm8s.c: 87: void i2c_poll_BTF()
                                    489 ;	-----------------------------------------
                                    490 ;	 function i2c_poll_BTF
                                    491 ;	-----------------------------------------
      0081D9                        492 _i2c_poll_BTF:
                                    493 ;	periph_stm8s.c: 89: while((I2C_SR1&(1<<I2C_SR1_BTF))!=(1<<I2C_SR1_BTF)){}
      0081D9                        494 00101$:
      0081D9 AE 52 17         [ 2]  495 	ldw	x, #0x5217
      0081DC F6               [ 1]  496 	ld	a, (x)
      0081DD A4 04            [ 1]  497 	and	a, #0x04
      0081DF A1 04            [ 1]  498 	cp	a, #0x04
      0081E1 26 F6            [ 1]  499 	jrne	00101$
      0081E3 81               [ 4]  500 	ret
                                    501 ;	periph_stm8s.c: 92: void i2c_poll_TXE()
                                    502 ;	-----------------------------------------
                                    503 ;	 function i2c_poll_TXE
                                    504 ;	-----------------------------------------
      0081E4                        505 _i2c_poll_TXE:
                                    506 ;	periph_stm8s.c: 94: while((I2C_SR1&(1<<I2C_SR1_TXE))!=(1<<I2C_SR1_TXE)){}
      0081E4                        507 00101$:
      0081E4 AE 52 17         [ 2]  508 	ldw	x, #0x5217
      0081E7 F6               [ 1]  509 	ld	a, (x)
      0081E8 A4 80            [ 1]  510 	and	a, #0x80
      0081EA A1 80            [ 1]  511 	cp	a, #0x80
      0081EC 26 F6            [ 1]  512 	jrne	00101$
      0081EE 81               [ 4]  513 	ret
                                    514 ;	periph_stm8s.c: 97: void i2c_poll_RXNE()
                                    515 ;	-----------------------------------------
                                    516 ;	 function i2c_poll_RXNE
                                    517 ;	-----------------------------------------
      0081EF                        518 _i2c_poll_RXNE:
                                    519 ;	periph_stm8s.c: 99: while((I2C_SR1&(1<<I2C_SR1_RXNE))!=(1<<I2C_SR1_RXNE)){}
      0081EF                        520 00101$:
      0081EF AE 52 17         [ 2]  521 	ldw	x, #0x5217
      0081F2 F6               [ 1]  522 	ld	a, (x)
      0081F3 A4 40            [ 1]  523 	and	a, #0x40
      0081F5 A1 40            [ 1]  524 	cp	a, #0x40
      0081F7 26 F6            [ 1]  525 	jrne	00101$
      0081F9 81               [ 4]  526 	ret
                                    527 ;	periph_stm8s.c: 102: void i2c_clear_bits()
                                    528 ;	-----------------------------------------
                                    529 ;	 function i2c_clear_bits
                                    530 ;	-----------------------------------------
      0081FA                        531 _i2c_clear_bits:
                                    532 ;	periph_stm8s.c: 104: readreg = I2C_SR1;
      0081FA AE 52 17         [ 2]  533 	ldw	x, #0x5217
      0081FD F6               [ 1]  534 	ld	a, (x)
      0081FE C7 00 01         [ 1]  535 	ld	_readreg+0, a
      008201 81               [ 4]  536 	ret
                                    537 ;	periph_stm8s.c: 107: void i2c_clear_ADDR()
                                    538 ;	-----------------------------------------
                                    539 ;	 function i2c_clear_ADDR
                                    540 ;	-----------------------------------------
      008202                        541 _i2c_clear_ADDR:
                                    542 ;	periph_stm8s.c: 109: readreg = I2C_SR1;
      008202 AE 52 17         [ 2]  543 	ldw	x, #0x5217
      008205 F6               [ 1]  544 	ld	a, (x)
                                    545 ;	periph_stm8s.c: 110: readreg = I2C_SR3;
      008206 AE 52 19         [ 2]  546 	ldw	x, #0x5219
      008209 F6               [ 1]  547 	ld	a, (x)
      00820A C7 00 01         [ 1]  548 	ld	_readreg+0, a
      00820D 81               [ 4]  549 	ret
                                    550 ;	periph_stm8s.c: 113: void i2c_enable_interrupts()
                                    551 ;	-----------------------------------------
                                    552 ;	 function i2c_enable_interrupts
                                    553 ;	-----------------------------------------
      00820E                        554 _i2c_enable_interrupts:
                                    555 ;	periph_stm8s.c: 115: I2C_ITR = 0x07;
      00820E 35 07 52 1A      [ 1]  556 	mov	0x521a+0, #0x07
      008212 81               [ 4]  557 	ret
                                    558 ;	periph_stm8s.c: 117: void i2c_disable_interrupts()
                                    559 ;	-----------------------------------------
                                    560 ;	 function i2c_disable_interrupts
                                    561 ;	-----------------------------------------
      008213                        562 _i2c_disable_interrupts:
                                    563 ;	periph_stm8s.c: 119: I2C_ITR = 0x00;
      008213 35 00 52 1A      [ 1]  564 	mov	0x521a+0, #0x00
      008217 81               [ 4]  565 	ret
                                    566 ;	periph_stm8s.c: 124: void adc_init()
                                    567 ;	-----------------------------------------
                                    568 ;	 function adc_init
                                    569 ;	-----------------------------------------
      008218                        570 _adc_init:
                                    571 ;	periph_stm8s.c: 126: ADC1_CR1 = fADC_fMASTER_8<<ADC1_CR1_SPSEL; // ADCCLK = MCLK/8
      008218 35 40 54 01      [ 1]  572 	mov	0x5401+0, #0x40
                                    573 ;	periph_stm8s.c: 127: ADC1_CR2 = (1<<ADC1_CR2_ALIGN);  // right alignment adc data
      00821C 35 08 54 02      [ 1]  574 	mov	0x5402+0, #0x08
                                    575 ;	periph_stm8s.c: 129: ADC1_CR1 |= (1<<ADC1_CR1_ADON);  // turn on ADC
      008220 72 10 54 01      [ 1]  576 	bset	0x5401, #0
      008224 81               [ 4]  577 	ret
                                    578 ;	periph_stm8s.c: 133: unsigned int read_adc(unsigned char adcch)
                                    579 ;	-----------------------------------------
                                    580 ;	 function read_adc
                                    581 ;	-----------------------------------------
      008225                        582 _read_adc:
      008225 52 04            [ 2]  583 	sub	sp, #4
                                    584 ;	periph_stm8s.c: 137: ADC1_CSR &= 0xF0;  // select
      008227 AE 54 00         [ 2]  585 	ldw	x, #0x5400
      00822A F6               [ 1]  586 	ld	a, (x)
      00822B A4 F0            [ 1]  587 	and	a, #0xf0
      00822D F7               [ 1]  588 	ld	(x), a
                                    589 ;	periph_stm8s.c: 138: ADC1_CSR |= adcch; // channel
      00822E AE 54 00         [ 2]  590 	ldw	x, #0x5400
      008231 F6               [ 1]  591 	ld	a, (x)
      008232 1A 07            [ 1]  592 	or	a, (0x07, sp)
      008234 AE 54 00         [ 2]  593 	ldw	x, #0x5400
      008237 F7               [ 1]  594 	ld	(x), a
                                    595 ;	periph_stm8s.c: 141: ADC1_CR1 |= (1<<ADC1_CR1_ADON); // start conversion
      008238 72 10 54 01      [ 1]  596 	bset	0x5401, #0
                                    597 ;	periph_stm8s.c: 142: while(!((ADC1_CSR)&(1<<ADC1_CSR_EOC)));; // conversion is in progress
      00823C                        598 00101$:
      00823C AE 54 00         [ 2]  599 	ldw	x, #0x5400
      00823F F6               [ 1]  600 	ld	a, (x)
      008240 4D               [ 1]  601 	tnz	a
      008241 2A F9            [ 1]  602 	jrpl	00101$
                                    603 ;	periph_stm8s.c: 143: adcval = (ADC1_DRH<<8) + ADC1_DRL;
      008243 AE 54 04         [ 2]  604 	ldw	x, #0x5404
      008246 F6               [ 1]  605 	ld	a, (x)
      008247 0F 03            [ 1]  606 	clr	(0x03, sp)
      008249 6B 01            [ 1]  607 	ld	(0x01, sp), a
      00824B 0F 02            [ 1]  608 	clr	(0x02, sp)
      00824D AE 54 05         [ 2]  609 	ldw	x, #0x5405
      008250 F6               [ 1]  610 	ld	a, (x)
      008251 5F               [ 1]  611 	clrw	x
      008252 97               [ 1]  612 	ld	xl, a
      008253 72 FB 01         [ 2]  613 	addw	x, (0x01, sp)
                                    614 ;	periph_stm8s.c: 144: ADC1_CSR |= (0<<ADC1_CSR_EOC); // reset EOC
      008256 90 AE 54 00      [ 2]  615 	ldw	y, #0x5400
      00825A 90 F6            [ 1]  616 	ld	a, (y)
      00825C 90 AE 54 00      [ 2]  617 	ldw	y, #0x5400
      008260 90 F7            [ 1]  618 	ld	(y), a
                                    619 ;	periph_stm8s.c: 146: return adcval;
      008262 5B 04            [ 2]  620 	addw	sp, #4
      008264 81               [ 4]  621 	ret
                                    622 ;	periph_stm8s.c: 151: void uart1_init(unsigned char rxien) //UART Initialization
                                    623 ;	-----------------------------------------
                                    624 ;	 function uart1_init
                                    625 ;	-----------------------------------------
      008265                        626 _uart1_init:
                                    627 ;	periph_stm8s.c: 155: UART1_BRR1 = 0x68;
      008265 35 68 52 32      [ 1]  628 	mov	0x5232+0, #0x68
                                    629 ;	periph_stm8s.c: 156: UART1_BRR2 = 0x03;
      008269 35 03 52 33      [ 1]  630 	mov	0x5233+0, #0x03
                                    631 ;	periph_stm8s.c: 158: UART1_CR1 |= (0<<UART1_CR1_M)|(0<<UART1_CR1_PCEN); //8 bit Data; No Parity
      00826D AE 52 34         [ 2]  632 	ldw	x, #0x5234
      008270 F6               [ 1]  633 	ld	a, (x)
      008271 AE 52 34         [ 2]  634 	ldw	x, #0x5234
      008274 F7               [ 1]  635 	ld	(x), a
                                    636 ;	periph_stm8s.c: 159: UART1_CR3 |= (0<<UART1_CR3_STOP); //Stop Bit = 1
      008275 AE 52 36         [ 2]  637 	ldw	x, #0x5236
      008278 F6               [ 1]  638 	ld	a, (x)
      008279 AE 52 36         [ 2]  639 	ldw	x, #0x5236
      00827C F7               [ 1]  640 	ld	(x), a
                                    641 ;	periph_stm8s.c: 161: if(rxien==1) 
      00827D 7B 03            [ 1]  642 	ld	a, (0x03, sp)
      00827F A1 01            [ 1]  643 	cp	a, #0x01
      008281 26 0B            [ 1]  644 	jrne	00102$
                                    645 ;	periph_stm8s.c: 163: UART1_CR2 |= (1<<UART1_CR2_RIEN); //Enable Interrupt on Receiver Mode
      008283 AE 52 35         [ 2]  646 	ldw	x, #0x5235
      008286 F6               [ 1]  647 	ld	a, (x)
      008287 AA 20            [ 1]  648 	or	a, #0x20
      008289 F7               [ 1]  649 	ld	(x), a
                                    650 ;	periph_stm8s.c: 164: ITC_SPR5 = (level_2<<ITC_SPR5_VECT18); //UART Interrupt Setting
      00828A 35 00 7F 74      [ 1]  651 	mov	0x7f74+0, #0x00
      00828E                        652 00102$:
                                    653 ;	periph_stm8s.c: 167: UART1_CR2 |= (1<<UART1_CR2_TEN); //Enable Transmitter Mode
      00828E AE 52 35         [ 2]  654 	ldw	x, #0x5235
      008291 F6               [ 1]  655 	ld	a, (x)
      008292 AA 08            [ 1]  656 	or	a, #0x08
      008294 F7               [ 1]  657 	ld	(x), a
                                    658 ;	periph_stm8s.c: 168: UART1_CR2 |= (1<<UART1_CR2_REN); //Enable Receiver Mode
      008295 AE 52 35         [ 2]  659 	ldw	x, #0x5235
      008298 F6               [ 1]  660 	ld	a, (x)
      008299 AA 04            [ 1]  661 	or	a, #0x04
      00829B F7               [ 1]  662 	ld	(x), a
      00829C 81               [ 4]  663 	ret
                                    664 ;	periph_stm8s.c: 171: void uart1_send(unsigned char usend) //UART Transmit a Byte
                                    665 ;	-----------------------------------------
                                    666 ;	 function uart1_send
                                    667 ;	-----------------------------------------
      00829D                        668 _uart1_send:
                                    669 ;	periph_stm8s.c: 173: UART1_DR = usend; //Write to UART Data Register
      00829D AE 52 31         [ 2]  670 	ldw	x, #0x5231
      0082A0 7B 03            [ 1]  671 	ld	a, (0x03, sp)
      0082A2 F7               [ 1]  672 	ld	(x), a
                                    673 ;	periph_stm8s.c: 174: while((UART1_SR&(1<<UART1_SR_TXE))!=(1<<UART1_SR_TXE)); //Wait until Transmission complete
      0082A3                        674 00101$:
      0082A3 AE 52 30         [ 2]  675 	ldw	x, #0x5230
      0082A6 F6               [ 1]  676 	ld	a, (x)
      0082A7 A4 80            [ 1]  677 	and	a, #0x80
      0082A9 A1 80            [ 1]  678 	cp	a, #0x80
      0082AB 26 F6            [ 1]  679 	jrne	00101$
      0082AD 81               [ 4]  680 	ret
                                    681 ;	periph_stm8s.c: 177: unsigned char uart1_recv() //UART Receive a Byte (using Polling)
                                    682 ;	-----------------------------------------
                                    683 ;	 function uart1_recv
                                    684 ;	-----------------------------------------
      0082AE                        685 _uart1_recv:
                                    686 ;	periph_stm8s.c: 180: if((UART1_SR&(1<<UART1_SR_RXNE))==(1<<UART1_SR_RXNE)) //Check if any data in Data Register
      0082AE AE 52 30         [ 2]  687 	ldw	x, #0x5230
      0082B1 F6               [ 1]  688 	ld	a, (x)
      0082B2 A4 20            [ 1]  689 	and	a, #0x20
      0082B4 A1 20            [ 1]  690 	cp	a, #0x20
      0082B6 26 05            [ 1]  691 	jrne	00102$
                                    692 ;	periph_stm8s.c: 182: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
      0082B8 AE 52 31         [ 2]  693 	ldw	x, #0x5231
      0082BB F6               [ 1]  694 	ld	a, (x)
                                    695 ;	periph_stm8s.c: 184: else urecv=0;
      0082BC 21                     696 	.byte 0x21
      0082BD                        697 00102$:
      0082BD 4F               [ 1]  698 	clr	a
      0082BE                        699 00103$:
                                    700 ;	periph_stm8s.c: 185: return urecv;
      0082BE 81               [ 4]  701 	ret
                                    702 ;	periph_stm8s.c: 188: unsigned char uart1_recv_i() //UART Receive a Byte (using Interrupt)
                                    703 ;	-----------------------------------------
                                    704 ;	 function uart1_recv_i
                                    705 ;	-----------------------------------------
      0082BF                        706 _uart1_recv_i:
                                    707 ;	periph_stm8s.c: 191: urecv = UART1_DR; //Read Data Register (RXNE cleared automatically)
      0082BF AE 52 31         [ 2]  708 	ldw	x, #0x5231
      0082C2 F6               [ 1]  709 	ld	a, (x)
                                    710 ;	periph_stm8s.c: 192: return urecv;
      0082C3 81               [ 4]  711 	ret
                                    712 ;	periph_stm8s.c: 198: void pwm1_init(unsigned int timval)
                                    713 ;	-----------------------------------------
                                    714 ;	 function pwm1_init
                                    715 ;	-----------------------------------------
      0082C4                        716 _pwm1_init:
      0082C4 52 02            [ 2]  717 	sub	sp, #2
                                    718 ;	periph_stm8s.c: 200: TIM1_PSCRH = 0x00; //TIM_CLK = CLK
      0082C6 35 00 52 60      [ 1]  719 	mov	0x5260+0, #0x00
                                    720 ;	periph_stm8s.c: 201: TIM1_PSCRL = 0x00; //TIM_CLK = CLK
      0082CA 35 00 52 61      [ 1]  721 	mov	0x5261+0, #0x00
                                    722 ;	periph_stm8s.c: 202: TIM1_ARRH = (timval >> 8); //TIM RELOAD
      0082CE 7B 05            [ 1]  723 	ld	a, (0x05, sp)
      0082D0 0F 01            [ 1]  724 	clr	(0x01, sp)
      0082D2 AE 52 62         [ 2]  725 	ldw	x, #0x5262
      0082D5 F7               [ 1]  726 	ld	(x), a
                                    727 ;	periph_stm8s.c: 203: TIM1_ARRL = (timval & 0x00FF); //TIM RELOAD
      0082D6 7B 06            [ 1]  728 	ld	a, (0x06, sp)
      0082D8 95               [ 1]  729 	ld	xh, a
      0082D9 4F               [ 1]  730 	clr	a
      0082DA 9E               [ 1]  731 	ld	a, xh
      0082DB AE 52 63         [ 2]  732 	ldw	x, #0x5263
      0082DE F7               [ 1]  733 	ld	(x), a
                                    734 ;	periph_stm8s.c: 204: pwm1ch1_enable();
      0082DF CD 83 3A         [ 4]  735 	call	_pwm1ch1_enable
                                    736 ;	periph_stm8s.c: 205: TIM1_CCER1 |= (0<<TIM1_CCER1_CC1P); //Output active high
      0082E2 AE 52 5C         [ 2]  737 	ldw	x, #0x525c
      0082E5 F6               [ 1]  738 	ld	a, (x)
      0082E6 AE 52 5C         [ 2]  739 	ldw	x, #0x525c
      0082E9 F7               [ 1]  740 	ld	(x), a
                                    741 ;	periph_stm8s.c: 206: TIM1_CCMR1 = (TIM1_OCxREF_PWM_mode1<<TIM1_CCMR1_OC1M); //PWM MODE 1 for Channel 1
      0082EA 35 60 52 58      [ 1]  742 	mov	0x5258+0, #0x60
                                    743 ;	periph_stm8s.c: 207: pwm1_update(0x0000); //Start Value
      0082EE 5F               [ 1]  744 	clrw	x
      0082EF 89               [ 2]  745 	pushw	x
      0082F0 CD 83 4E         [ 4]  746 	call	_pwm1_update
      0082F3 5B 02            [ 2]  747 	addw	sp, #2
                                    748 ;	periph_stm8s.c: 208: TIM1_BKR = (1<<TIM1_BKR_MOE); //ENABLE MAIN OUTPUT 
      0082F5 35 80 52 6D      [ 1]  749 	mov	0x526d+0, #0x80
                                    750 ;	periph_stm8s.c: 209: TIM1_CR1 |= (1<<TIM1_CR1_CEN); //ENABLE TIM
      0082F9 AE 52 50         [ 2]  751 	ldw	x, #0x5250
      0082FC F6               [ 1]  752 	ld	a, (x)
      0082FD AA 01            [ 1]  753 	or	a, #0x01
      0082FF F7               [ 1]  754 	ld	(x), a
      008300 5B 02            [ 2]  755 	addw	sp, #2
      008302 81               [ 4]  756 	ret
                                    757 ;	periph_stm8s.c: 212: void pwm2_init(unsigned int timval)
                                    758 ;	-----------------------------------------
                                    759 ;	 function pwm2_init
                                    760 ;	-----------------------------------------
      008303                        761 _pwm2_init:
      008303 52 02            [ 2]  762 	sub	sp, #2
                                    763 ;	periph_stm8s.c: 214: TIM2_PSCR = 0x00; //TIM_CLK = CLK
      008305 35 00 53 0E      [ 1]  764 	mov	0x530e+0, #0x00
                                    765 ;	periph_stm8s.c: 215: TIM2_ARRH = (timval >> 8); //TIM RELOAD
      008309 7B 05            [ 1]  766 	ld	a, (0x05, sp)
      00830B 0F 01            [ 1]  767 	clr	(0x01, sp)
      00830D AE 53 0F         [ 2]  768 	ldw	x, #0x530f
      008310 F7               [ 1]  769 	ld	(x), a
                                    770 ;	periph_stm8s.c: 216: TIM2_ARRL = (timval & 0x00FF); //TIM RELOAD
      008311 7B 06            [ 1]  771 	ld	a, (0x06, sp)
      008313 95               [ 1]  772 	ld	xh, a
      008314 4F               [ 1]  773 	clr	a
      008315 9E               [ 1]  774 	ld	a, xh
      008316 AE 53 10         [ 2]  775 	ldw	x, #0x5310
      008319 F7               [ 1]  776 	ld	(x), a
                                    777 ;	periph_stm8s.c: 217: pwm2ch1_enable();
      00831A CD 83 44         [ 4]  778 	call	_pwm2ch1_enable
                                    779 ;	periph_stm8s.c: 218: TIM2_CCER1 |= (0<<TIM2_CCER1_CC1P); //Output active high
      00831D AE 53 0A         [ 2]  780 	ldw	x, #0x530a
      008320 F6               [ 1]  781 	ld	a, (x)
      008321 AE 53 0A         [ 2]  782 	ldw	x, #0x530a
      008324 F7               [ 1]  783 	ld	(x), a
                                    784 ;	periph_stm8s.c: 219: TIM2_CCMR1 = (TIM2_OCxREF_PWM_mode1<<TIM2_CCMR1_OC1M); //PWM MODE 1 for Channel 1 
      008325 35 60 53 07      [ 1]  785 	mov	0x5307+0, #0x60
                                    786 ;	periph_stm8s.c: 220: pwm2_update(0x0000); //Start Value
      008329 5F               [ 1]  787 	clrw	x
      00832A 89               [ 2]  788 	pushw	x
      00832B CD 83 64         [ 4]  789 	call	_pwm2_update
      00832E 5B 02            [ 2]  790 	addw	sp, #2
                                    791 ;	periph_stm8s.c: 221: TIM2_CR1 |= (1<<TIM2_CR1_CEN); //ENABLE TIM
      008330 AE 53 00         [ 2]  792 	ldw	x, #0x5300
      008333 F6               [ 1]  793 	ld	a, (x)
      008334 AA 01            [ 1]  794 	or	a, #0x01
      008336 F7               [ 1]  795 	ld	(x), a
      008337 5B 02            [ 2]  796 	addw	sp, #2
      008339 81               [ 4]  797 	ret
                                    798 ;	periph_stm8s.c: 224: void pwm1ch1_enable()
                                    799 ;	-----------------------------------------
                                    800 ;	 function pwm1ch1_enable
                                    801 ;	-----------------------------------------
      00833A                        802 _pwm1ch1_enable:
                                    803 ;	periph_stm8s.c: 226: TIM1_CCER1 |= (1<<TIM1_CCER1_CC1E);
      00833A 72 10 52 5C      [ 1]  804 	bset	0x525c, #0
      00833E 81               [ 4]  805 	ret
                                    806 ;	periph_stm8s.c: 229: void pwm1ch1_disable()
                                    807 ;	-----------------------------------------
                                    808 ;	 function pwm1ch1_disable
                                    809 ;	-----------------------------------------
      00833F                        810 _pwm1ch1_disable:
                                    811 ;	periph_stm8s.c: 231: TIM1_CCER1 &= ~(1<<TIM1_CCER1_CC1E);
      00833F 72 11 52 5C      [ 1]  812 	bres	0x525c, #0
      008343 81               [ 4]  813 	ret
                                    814 ;	periph_stm8s.c: 234: void pwm2ch1_enable()
                                    815 ;	-----------------------------------------
                                    816 ;	 function pwm2ch1_enable
                                    817 ;	-----------------------------------------
      008344                        818 _pwm2ch1_enable:
                                    819 ;	periph_stm8s.c: 236: TIM2_CCER1 |= (1<<TIM2_CCER1_CC1E);
      008344 72 10 53 0A      [ 1]  820 	bset	0x530a, #0
      008348 81               [ 4]  821 	ret
                                    822 ;	periph_stm8s.c: 239: void pwm2ch1_disable()
                                    823 ;	-----------------------------------------
                                    824 ;	 function pwm2ch1_disable
                                    825 ;	-----------------------------------------
      008349                        826 _pwm2ch1_disable:
                                    827 ;	periph_stm8s.c: 241: TIM2_CCER1 &= ~(1<<TIM2_CCER1_CC1E);
      008349 72 11 53 0A      [ 1]  828 	bres	0x530a, #0
      00834D 81               [ 4]  829 	ret
                                    830 ;	periph_stm8s.c: 244: void pwm1_update(unsigned int pwmval)
                                    831 ;	-----------------------------------------
                                    832 ;	 function pwm1_update
                                    833 ;	-----------------------------------------
      00834E                        834 _pwm1_update:
      00834E 52 02            [ 2]  835 	sub	sp, #2
                                    836 ;	periph_stm8s.c: 246: TIM1_CCR1L = (pwmval & 0x00FF);
      008350 7B 06            [ 1]  837 	ld	a, (0x06, sp)
      008352 95               [ 1]  838 	ld	xh, a
      008353 4F               [ 1]  839 	clr	a
      008354 9E               [ 1]  840 	ld	a, xh
      008355 AE 52 66         [ 2]  841 	ldw	x, #0x5266
      008358 F7               [ 1]  842 	ld	(x), a
                                    843 ;	periph_stm8s.c: 247: TIM1_CCR1H = (pwmval >> 8);
      008359 7B 05            [ 1]  844 	ld	a, (0x05, sp)
      00835B 0F 01            [ 1]  845 	clr	(0x01, sp)
      00835D AE 52 65         [ 2]  846 	ldw	x, #0x5265
      008360 F7               [ 1]  847 	ld	(x), a
      008361 5B 02            [ 2]  848 	addw	sp, #2
      008363 81               [ 4]  849 	ret
                                    850 ;	periph_stm8s.c: 250: void pwm2_update(unsigned int pwmval)
                                    851 ;	-----------------------------------------
                                    852 ;	 function pwm2_update
                                    853 ;	-----------------------------------------
      008364                        854 _pwm2_update:
      008364 52 02            [ 2]  855 	sub	sp, #2
                                    856 ;	periph_stm8s.c: 252: TIM2_CCR1L = (pwmval & 0x00FF);
      008366 7B 06            [ 1]  857 	ld	a, (0x06, sp)
      008368 95               [ 1]  858 	ld	xh, a
      008369 4F               [ 1]  859 	clr	a
      00836A 9E               [ 1]  860 	ld	a, xh
      00836B AE 53 12         [ 2]  861 	ldw	x, #0x5312
      00836E F7               [ 1]  862 	ld	(x), a
                                    863 ;	periph_stm8s.c: 253: TIM2_CCR1H = (pwmval >> 8);
      00836F 7B 05            [ 1]  864 	ld	a, (0x05, sp)
      008371 0F 01            [ 1]  865 	clr	(0x01, sp)
      008373 AE 53 11         [ 2]  866 	ldw	x, #0x5311
      008376 F7               [ 1]  867 	ld	(x), a
      008377 5B 02            [ 2]  868 	addw	sp, #2
      008379 81               [ 4]  869 	ret
                                    870 ;	lcd_n1202_stm8s.c: 7: void lcdn1202_gpio_init()
                                    871 ;	-----------------------------------------
                                    872 ;	 function lcdn1202_gpio_init
                                    873 ;	-----------------------------------------
      00837A                        874 _lcdn1202_gpio_init:
                                    875 ;	lcd_n1202_stm8s.c: 9: LCDDDR |= (OUTPUT<<LCDDAT)|(OUTPUT<<LCDCLK)|(OUTPUT<<LCDBL);	//Configure GPIO as Output
      00837A AE 50 02         [ 2]  876 	ldw	x, #0x5002
      00837D F6               [ 1]  877 	ld	a, (x)
      00837E AA 0E            [ 1]  878 	or	a, #0x0e
      008380 F7               [ 1]  879 	ld	(x), a
                                    880 ;	lcd_n1202_stm8s.c: 10: LCDCR1 |= (pushpull<<LCDDAT)|(pushpull<<LCDCLK)|(pushpull<<LCDBL); //Configure Output Type
      008381 AE 50 03         [ 2]  881 	ldw	x, #0x5003
      008384 F6               [ 1]  882 	ld	a, (x)
      008385 AA 0E            [ 1]  883 	or	a, #0x0e
      008387 F7               [ 1]  884 	ld	(x), a
                                    885 ;	lcd_n1202_stm8s.c: 11: LCDCR2 |= (speed_10MHz<<LCDDAT)|(speed_10MHz<<LCDCLK)|(speed_10MHz<<LCDBL); //Configure GPIO speed
      008388 AE 50 04         [ 2]  886 	ldw	x, #0x5004
      00838B F6               [ 1]  887 	ld	a, (x)
      00838C AA 0E            [ 1]  888 	or	a, #0x0e
      00838E F7               [ 1]  889 	ld	(x), a
                                    890 ;	lcd_n1202_stm8s.c: 12: LCDODR = 0x00; //Starting value
      00838F 35 00 50 00      [ 1]  891 	mov	0x5000+0, #0x00
      008393 81               [ 4]  892 	ret
                                    893 ;	lcd_n1202_stm8s.c: 15: void lcdn1202_9bsend(unsigned char cdsign, unsigned char comdat)
                                    894 ;	-----------------------------------------
                                    895 ;	 function lcdn1202_9bsend
                                    896 ;	-----------------------------------------
      008394                        897 _lcdn1202_9bsend:
      008394 88               [ 1]  898 	push	a
                                    899 ;	lcd_n1202_stm8s.c: 19: if(cdsign==0) LCDODR &= LCDDAT_MASKL; //1st bit is 0 for Command
      008395 0D 04            [ 1]  900 	tnz	(0x04, sp)
      008397 26 09            [ 1]  901 	jrne	00102$
      008399 AE 50 00         [ 2]  902 	ldw	x, #0x5000
      00839C F6               [ 1]  903 	ld	a, (x)
      00839D A4 FD            [ 1]  904 	and	a, #0xfd
      00839F F7               [ 1]  905 	ld	(x), a
      0083A0 20 07            [ 2]  906 	jra	00103$
      0083A2                        907 00102$:
                                    908 ;	lcd_n1202_stm8s.c: 20: else LCDODR |= LCDDAT_MASKH; //1st bit is 1 for Data
      0083A2 AE 50 00         [ 2]  909 	ldw	x, #0x5000
      0083A5 F6               [ 1]  910 	ld	a, (x)
      0083A6 AA 02            [ 1]  911 	or	a, #0x02
      0083A8 F7               [ 1]  912 	ld	(x), a
      0083A9                        913 00103$:
                                    914 ;	lcd_n1202_stm8s.c: 21: lcdn1202_clock1();
      0083A9 CD 83 D8         [ 4]  915 	call	_lcdn1202_clock1
                                    916 ;	lcd_n1202_stm8s.c: 23: for(cdi=0;cdi<8;cdi++) //Send 2nd-9th bit
      0083AC 0F 01            [ 1]  917 	clr	(0x01, sp)
      0083AE                        918 00108$:
                                    919 ;	lcd_n1202_stm8s.c: 25: if(comdat & 0x80) LCDODR |= LCDDAT_MASKH; //LCDDAT = '1'
      0083AE 0D 05            [ 1]  920 	tnz	(0x05, sp)
      0083B0 2A 09            [ 1]  921 	jrpl	00105$
      0083B2 AE 50 00         [ 2]  922 	ldw	x, #0x5000
      0083B5 F6               [ 1]  923 	ld	a, (x)
      0083B6 AA 02            [ 1]  924 	or	a, #0x02
      0083B8 F7               [ 1]  925 	ld	(x), a
      0083B9 20 07            [ 2]  926 	jra	00106$
      0083BB                        927 00105$:
                                    928 ;	lcd_n1202_stm8s.c: 26: else LCDODR &= LCDDAT_MASKL;		  //LCDDAT = '0'
      0083BB AE 50 00         [ 2]  929 	ldw	x, #0x5000
      0083BE F6               [ 1]  930 	ld	a, (x)
      0083BF A4 FD            [ 1]  931 	and	a, #0xfd
      0083C1 F7               [ 1]  932 	ld	(x), a
      0083C2                        933 00106$:
                                    934 ;	lcd_n1202_stm8s.c: 27: lcdn1202_clock1();
      0083C2 CD 83 D8         [ 4]  935 	call	_lcdn1202_clock1
                                    936 ;	lcd_n1202_stm8s.c: 28: comdat <<= 1; //Shift to next bit
      0083C5 08 05            [ 1]  937 	sll	(0x05, sp)
                                    938 ;	lcd_n1202_stm8s.c: 23: for(cdi=0;cdi<8;cdi++) //Send 2nd-9th bit
      0083C7 0C 01            [ 1]  939 	inc	(0x01, sp)
      0083C9 7B 01            [ 1]  940 	ld	a, (0x01, sp)
      0083CB A1 08            [ 1]  941 	cp	a, #0x08
      0083CD 25 DF            [ 1]  942 	jrc	00108$
                                    943 ;	lcd_n1202_stm8s.c: 30: LCDODR &= LCDDAT_MASKL;
      0083CF AE 50 00         [ 2]  944 	ldw	x, #0x5000
      0083D2 F6               [ 1]  945 	ld	a, (x)
      0083D3 A4 FD            [ 1]  946 	and	a, #0xfd
      0083D5 F7               [ 1]  947 	ld	(x), a
      0083D6 84               [ 1]  948 	pop	a
      0083D7 81               [ 4]  949 	ret
                                    950 ;	lcd_n1202_stm8s.c: 33: void lcdn1202_clock1()
                                    951 ;	-----------------------------------------
                                    952 ;	 function lcdn1202_clock1
                                    953 ;	-----------------------------------------
      0083D8                        954 _lcdn1202_clock1:
                                    955 ;	lcd_n1202_stm8s.c: 35: LCDODR |= LCDCLK_MASKH; //Send 1 pulse to LCDCLK
      0083D8 AE 50 00         [ 2]  956 	ldw	x, #0x5000
      0083DB F6               [ 1]  957 	ld	a, (x)
      0083DC AA 04            [ 1]  958 	or	a, #0x04
      0083DE F7               [ 1]  959 	ld	(x), a
                                    960 ;	lcd_n1202_stm8s.c: 36: delay_us(1); //Short delay
      0083DF 4B 01            [ 1]  961 	push	#0x01
      0083E1 5F               [ 1]  962 	clrw	x
      0083E2 89               [ 2]  963 	pushw	x
      0083E3 4B 00            [ 1]  964 	push	#0x00
      0083E5 CD 80 A5         [ 4]  965 	call	_delay_us
      0083E8 5B 04            [ 2]  966 	addw	sp, #4
                                    967 ;	lcd_n1202_stm8s.c: 37: LCDODR &= LCDCLK_MASKL;
      0083EA AE 50 00         [ 2]  968 	ldw	x, #0x5000
      0083ED F6               [ 1]  969 	ld	a, (x)
      0083EE A4 FB            [ 1]  970 	and	a, #0xfb
      0083F0 F7               [ 1]  971 	ld	(x), a
      0083F1 81               [ 4]  972 	ret
                                    973 ;	lcd_n1202_stm8s.c: 40: void lcdn1202_blon()
                                    974 ;	-----------------------------------------
                                    975 ;	 function lcdn1202_blon
                                    976 ;	-----------------------------------------
      0083F2                        977 _lcdn1202_blon:
                                    978 ;	lcd_n1202_stm8s.c: 42: LCDODR |= LCDBL_MASKH; //LCDBL = '1'
      0083F2 AE 50 00         [ 2]  979 	ldw	x, #0x5000
      0083F5 F6               [ 1]  980 	ld	a, (x)
      0083F6 AA 08            [ 1]  981 	or	a, #0x08
      0083F8 F7               [ 1]  982 	ld	(x), a
      0083F9 81               [ 4]  983 	ret
                                    984 ;	lcd_n1202_stm8s.c: 45: void lcdn1202_bloff()
                                    985 ;	-----------------------------------------
                                    986 ;	 function lcdn1202_bloff
                                    987 ;	-----------------------------------------
      0083FA                        988 _lcdn1202_bloff:
                                    989 ;	lcd_n1202_stm8s.c: 47: LCDODR &= LCDBL_MASKL; //LCDBL = '0'
      0083FA AE 50 00         [ 2]  990 	ldw	x, #0x5000
      0083FD F6               [ 1]  991 	ld	a, (x)
      0083FE A4 F7            [ 1]  992 	and	a, #0xf7
      008400 F7               [ 1]  993 	ld	(x), a
      008401 81               [ 4]  994 	ret
                                    995 ;	lcd_n1202.c: 9: void lcdn1202_init()
                                    996 ;	-----------------------------------------
                                    997 ;	 function lcdn1202_init
                                    998 ;	-----------------------------------------
      008402                        999 _lcdn1202_init:
                                   1000 ;	lcd_n1202.c: 11: lcdn1202_gpio_init();
      008402 CD 83 7A         [ 4] 1001 	call	_lcdn1202_gpio_init
                                   1002 ;	lcd_n1202.c: 15: delay_ms(10);
      008405 4B 0A            [ 1] 1003 	push	#0x0a
      008407 5F               [ 1] 1004 	clrw	x
      008408 89               [ 2] 1005 	pushw	x
      008409 4B 00            [ 1] 1006 	push	#0x00
      00840B CD 80 F5         [ 4] 1007 	call	_delay_ms
      00840E 5B 04            [ 2] 1008 	addw	sp, #4
                                   1009 ;	lcd_n1202.c: 17: lcdn1202_sendcom(0xE2);	//Soft Reset
      008410 4B E2            [ 1] 1010 	push	#0xe2
      008412 CD 84 4E         [ 4] 1011 	call	_lcdn1202_sendcom
      008415 84               [ 1] 1012 	pop	a
                                   1013 ;	lcd_n1202.c: 18: delay_ms(1);
      008416 4B 01            [ 1] 1014 	push	#0x01
      008418 5F               [ 1] 1015 	clrw	x
      008419 89               [ 2] 1016 	pushw	x
      00841A 4B 00            [ 1] 1017 	push	#0x00
      00841C CD 80 F5         [ 4] 1018 	call	_delay_ms
      00841F 5B 04            [ 2] 1019 	addw	sp, #4
                                   1020 ;	lcd_n1202.c: 19: lcdn1202_sendcom(0xA4); //Normal Display Mode
      008421 4B A4            [ 1] 1021 	push	#0xa4
      008423 CD 84 4E         [ 4] 1022 	call	_lcdn1202_sendcom
      008426 84               [ 1] 1023 	pop	a
                                   1024 ;	lcd_n1202.c: 20: lcdn1202_sendcom(0x2F);	//Power Control = Max (Booster On, VReg On, VFol On)
      008427 4B 2F            [ 1] 1025 	push	#0x2f
      008429 CD 84 4E         [ 4] 1026 	call	_lcdn1202_sendcom
      00842C 84               [ 1] 1027 	pop	a
                                   1028 ;	lcd_n1202.c: 22: lcdn1202_sendcom(0xA0); //Segment Driver Direction = Normal (lines start at left)
      00842D 4B A0            [ 1] 1029 	push	#0xa0
      00842F CD 84 4E         [ 4] 1030 	call	_lcdn1202_sendcom
      008432 84               [ 1] 1031 	pop	a
                                   1032 ;	lcd_n1202.c: 23: lcdn1202_sendcom(0xC0); //Common Driver Direction = Normal
      008433 4B C0            [ 1] 1033 	push	#0xc0
      008435 CD 84 4E         [ 4] 1034 	call	_lcdn1202_sendcom
      008438 84               [ 1] 1035 	pop	a
                                   1036 ;	lcd_n1202.c: 24: lcdn1202_sendcom(0x80|16); //Set Contrast to default
      008439 4B 90            [ 1] 1037 	push	#0x90
      00843B CD 84 4E         [ 4] 1038 	call	_lcdn1202_sendcom
      00843E 84               [ 1] 1039 	pop	a
                                   1040 ;	lcd_n1202.c: 26: lcdn1202_sendcom(0xAF);	//Display On
      00843F 4B AF            [ 1] 1041 	push	#0xaf
      008441 CD 84 4E         [ 4] 1042 	call	_lcdn1202_sendcom
      008444 84               [ 1] 1043 	pop	a
                                   1044 ;	lcd_n1202.c: 28: LCD_BL_OFF(); //Backlight off
      008445 CD 86 17         [ 4] 1045 	call	_LCD_BL_OFF
                                   1046 ;	lcd_n1202.c: 29: LCD_clear();  //Clear pixel memory
      008448 CD 85 D9         [ 4] 1047 	call	_LCD_clear
                                   1048 ;	lcd_n1202.c: 30: LCD_BL_ON();  //Backlight on
      00844B CC 86 14         [ 2] 1049 	jp	_LCD_BL_ON
                                   1050 ;	lcd_n1202.c: 33: void lcdn1202_sendcom(unsigned char ssd1306com)
                                   1051 ;	-----------------------------------------
                                   1052 ;	 function lcdn1202_sendcom
                                   1053 ;	-----------------------------------------
      00844E                       1054 _lcdn1202_sendcom:
                                   1055 ;	lcd_n1202.c: 35: lcdn1202_9bsend(0,ssd1306com); //Send Command
      00844E 7B 03            [ 1] 1056 	ld	a, (0x03, sp)
      008450 88               [ 1] 1057 	push	a
      008451 4B 00            [ 1] 1058 	push	#0x00
      008453 CD 83 94         [ 4] 1059 	call	_lcdn1202_9bsend
      008456 5B 02            [ 2] 1060 	addw	sp, #2
      008458 81               [ 4] 1061 	ret
                                   1062 ;	lcd_n1202.c: 38: void lcdn1202_senddat(unsigned char ssd1306dat)
                                   1063 ;	-----------------------------------------
                                   1064 ;	 function lcdn1202_senddat
                                   1065 ;	-----------------------------------------
      008459                       1066 _lcdn1202_senddat:
                                   1067 ;	lcd_n1202.c: 40: lcdn1202_9bsend(1,ssd1306dat); //Send Data
      008459 7B 03            [ 1] 1068 	ld	a, (0x03, sp)
      00845B 88               [ 1] 1069 	push	a
      00845C 4B 01            [ 1] 1070 	push	#0x01
      00845E CD 83 94         [ 4] 1071 	call	_lcdn1202_9bsend
      008461 5B 02            [ 2] 1072 	addw	sp, #2
      008463 81               [ 4] 1073 	ret
                                   1074 ;	lcd_n1202.c: 43: void lcdn1202_setpos(unsigned char row, unsigned char col)
                                   1075 ;	-----------------------------------------
                                   1076 ;	 function lcdn1202_setpos
                                   1077 ;	-----------------------------------------
      008464                       1078 _lcdn1202_setpos:
                                   1079 ;	lcd_n1202.c: 45: lcdn1202_sendcom(0xB0|(row&0x0F)); //Set page of row
      008464 7B 03            [ 1] 1080 	ld	a, (0x03, sp)
      008466 A4 0F            [ 1] 1081 	and	a, #0x0f
      008468 AA B0            [ 1] 1082 	or	a, #0xb0
      00846A 88               [ 1] 1083 	push	a
      00846B CD 84 4E         [ 4] 1084 	call	_lcdn1202_sendcom
      00846E 84               [ 1] 1085 	pop	a
                                   1086 ;	lcd_n1202.c: 46: lcdn1202_sendcom(0x00|(col&0x0F)); //Set lower nibble of Column
      00846F 7B 04            [ 1] 1087 	ld	a, (0x04, sp)
      008471 A4 0F            [ 1] 1088 	and	a, #0x0f
      008473 88               [ 1] 1089 	push	a
      008474 CD 84 4E         [ 4] 1090 	call	_lcdn1202_sendcom
      008477 84               [ 1] 1091 	pop	a
                                   1092 ;	lcd_n1202.c: 47: lcdn1202_sendcom(0x10|((col>>4)&0x0F)); //Set upper nibble of Column
      008478 7B 04            [ 1] 1093 	ld	a, (0x04, sp)
      00847A 4E               [ 1] 1094 	swap	a
      00847B A4 0F            [ 1] 1095 	and	a, #0x0f
      00847D A4 0F            [ 1] 1096 	and	a, #0x0f
      00847F AA 10            [ 1] 1097 	or	a, #0x10
      008481 88               [ 1] 1098 	push	a
      008482 CD 84 4E         [ 4] 1099 	call	_lcdn1202_sendcom
      008485 84               [ 1] 1100 	pop	a
      008486 81               [ 4] 1101 	ret
                                   1102 ;	lcd_n1202.c: 50: void lcdn1202_clear()
                                   1103 ;	-----------------------------------------
                                   1104 ;	 function lcdn1202_clear
                                   1105 ;	-----------------------------------------
      008487                       1106 _lcdn1202_clear:
      008487 88               [ 1] 1107 	push	a
                                   1108 ;	lcd_n1202.c: 53: lcdn1202_setpos(0,0);
      008488 4B 00            [ 1] 1109 	push	#0x00
      00848A 4B 00            [ 1] 1110 	push	#0x00
      00848C CD 84 64         [ 4] 1111 	call	_lcdn1202_setpos
      00848F 5B 02            [ 2] 1112 	addw	sp, #2
                                   1113 ;	lcd_n1202.c: 54: for(row=0;row<LCDN1202_ROW;row++)	//Scan rows (pages)
      008491 0F 01            [ 1] 1114 	clr	(0x01, sp)
                                   1115 ;	lcd_n1202.c: 56: for(col=0;col<LCDN1202_COL;col++)	//Scan columns
      008493                       1116 00109$:
      008493 4F               [ 1] 1117 	clr	a
      008494                       1118 00103$:
                                   1119 ;	lcd_n1202.c: 58: lcdn1202_senddat(0);	//Send 0 to every pixel
      008494 88               [ 1] 1120 	push	a
      008495 4B 00            [ 1] 1121 	push	#0x00
      008497 CD 84 59         [ 4] 1122 	call	_lcdn1202_senddat
      00849A 84               [ 1] 1123 	pop	a
      00849B 84               [ 1] 1124 	pop	a
                                   1125 ;	lcd_n1202.c: 56: for(col=0;col<LCDN1202_COL;col++)	//Scan columns
      00849C 4C               [ 1] 1126 	inc	a
      00849D A1 60            [ 1] 1127 	cp	a, #0x60
      00849F 25 F3            [ 1] 1128 	jrc	00103$
                                   1129 ;	lcd_n1202.c: 54: for(row=0;row<LCDN1202_ROW;row++)	//Scan rows (pages)
      0084A1 0C 01            [ 1] 1130 	inc	(0x01, sp)
      0084A3 7B 01            [ 1] 1131 	ld	a, (0x01, sp)
      0084A5 A1 09            [ 1] 1132 	cp	a, #0x09
      0084A7 25 EA            [ 1] 1133 	jrc	00109$
      0084A9 84               [ 1] 1134 	pop	a
      0084AA 81               [ 4] 1135 	ret
                                   1136 ;	lcd_n1202.c: 63: void LCD_setpos(unsigned char row, unsigned char col)
                                   1137 ;	-----------------------------------------
                                   1138 ;	 function LCD_setpos
                                   1139 ;	-----------------------------------------
      0084AB                       1140 _LCD_setpos:
                                   1141 ;	lcd_n1202.c: 65: lcdn1202_setpos(row,col); //Set coordinate (for LCD_drawbyte)
      0084AB 7B 04            [ 1] 1142 	ld	a, (0x04, sp)
      0084AD 88               [ 1] 1143 	push	a
      0084AE 7B 04            [ 1] 1144 	ld	a, (0x04, sp)
      0084B0 88               [ 1] 1145 	push	a
      0084B1 CD 84 64         [ 4] 1146 	call	_lcdn1202_setpos
      0084B4 5B 02            [ 2] 1147 	addw	sp, #2
      0084B6 81               [ 4] 1148 	ret
                                   1149 ;	lcd_n1202.c: 68: void LCD_drawbyte(unsigned char dbyte)
                                   1150 ;	-----------------------------------------
                                   1151 ;	 function LCD_drawbyte
                                   1152 ;	-----------------------------------------
      0084B7                       1153 _LCD_drawbyte:
                                   1154 ;	lcd_n1202.c: 70: lcdn1202_senddat(dbyte); //Send 1 byte data only
      0084B7 7B 03            [ 1] 1155 	ld	a, (0x03, sp)
      0084B9 88               [ 1] 1156 	push	a
      0084BA CD 84 59         [ 4] 1157 	call	_lcdn1202_senddat
      0084BD 84               [ 1] 1158 	pop	a
      0084BE 81               [ 4] 1159 	ret
                                   1160 ;	lcd_n1202.c: 73: void LCD_drawchar(unsigned char chr, unsigned char chrrow, unsigned char chrcol)
                                   1161 ;	-----------------------------------------
                                   1162 ;	 function LCD_drawchar
                                   1163 ;	-----------------------------------------
      0084BF                       1164 _LCD_drawchar:
      0084BF 52 0B            [ 2] 1165 	sub	sp, #11
                                   1166 ;	lcd_n1202.c: 78: lcdn1202_setpos(chrrow,chrcol);
      0084C1 7B 10            [ 1] 1167 	ld	a, (0x10, sp)
      0084C3 88               [ 1] 1168 	push	a
      0084C4 7B 10            [ 1] 1169 	ld	a, (0x10, sp)
      0084C6 88               [ 1] 1170 	push	a
      0084C7 CD 84 64         [ 4] 1171 	call	_lcdn1202_setpos
      0084CA 5B 02            [ 2] 1172 	addw	sp, #2
                                   1173 ;	lcd_n1202.c: 83: chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
      0084CC 7B 0E            [ 1] 1174 	ld	a, (0x0e, sp)
      0084CE 6B 0B            [ 1] 1175 	ld	(0x0b, sp), a
      0084D0 0F 0A            [ 1] 1176 	clr	(0x0a, sp)
                                   1177 ;	lcd_n1202.c: 80: if((chr>31)&&(chr<128))	//Alphanumeric & Punctuation Area
      0084D2 7B 0E            [ 1] 1178 	ld	a, (0x0e, sp)
      0084D4 A1 1F            [ 1] 1179 	cp	a, #0x1f
      0084D6 23 3F            [ 2] 1180 	jrule	00107$
      0084D8 7B 0E            [ 1] 1181 	ld	a, (0x0e, sp)
      0084DA A1 80            [ 1] 1182 	cp	a, #0x80
      0084DC 24 39            [ 1] 1183 	jrnc	00107$
                                   1184 ;	lcd_n1202.c: 82: lcdn1202_senddat(0x00);
      0084DE 4B 00            [ 1] 1185 	push	#0x00
      0084E0 CD 84 59         [ 4] 1186 	call	_lcdn1202_senddat
      0084E3 84               [ 1] 1187 	pop	a
                                   1188 ;	lcd_n1202.c: 83: chridx=(chr-32)*5; //Start at character 32 (Space). 5 columns for each character
      0084E4 1E 0A            [ 2] 1189 	ldw	x, (0x0a, sp)
      0084E6 1D 00 20         [ 2] 1190 	subw	x, #0x0020
      0084E9 89               [ 2] 1191 	pushw	x
      0084EA 4B 05            [ 1] 1192 	push	#0x05
      0084EC 4B 00            [ 1] 1193 	push	#0x00
      0084EE CD 8B CD         [ 4] 1194 	call	__mulint
      0084F1 5B 04            [ 2] 1195 	addw	sp, #4
      0084F3 1F 08            [ 2] 1196 	ldw	(0x08, sp), x
                                   1197 ;	lcd_n1202.c: 84: for(ci=0;ci<5;ci++)
      0084F5 AE 88 FF         [ 2] 1198 	ldw	x, #_font_arr+0
      0084F8 1F 06            [ 2] 1199 	ldw	(0x06, sp), x
      0084FA 4F               [ 1] 1200 	clr	a
      0084FB                       1201 00110$:
                                   1202 ;	lcd_n1202.c: 86: fchar = font_arr[chridx+ci]; //Get character pattern from Font Array
      0084FB 5F               [ 1] 1203 	clrw	x
      0084FC 97               [ 1] 1204 	ld	xl, a
      0084FD 72 FB 08         [ 2] 1205 	addw	x, (0x08, sp)
      008500 72 FB 06         [ 2] 1206 	addw	x, (0x06, sp)
      008503 88               [ 1] 1207 	push	a
      008504 F6               [ 1] 1208 	ld	a, (x)
      008505 97               [ 1] 1209 	ld	xl, a
      008506 84               [ 1] 1210 	pop	a
                                   1211 ;	lcd_n1202.c: 87: lcdn1202_senddat(fchar); //Send pattern 1 byte at a time
      008507 88               [ 1] 1212 	push	a
      008508 89               [ 2] 1213 	pushw	x
      008509 5B 01            [ 2] 1214 	addw	sp, #1
      00850B CD 84 59         [ 4] 1215 	call	_lcdn1202_senddat
      00850E 84               [ 1] 1216 	pop	a
      00850F 84               [ 1] 1217 	pop	a
                                   1218 ;	lcd_n1202.c: 84: for(ci=0;ci<5;ci++)
      008510 4C               [ 1] 1219 	inc	a
      008511 A1 05            [ 1] 1220 	cp	a, #0x05
      008513 25 E6            [ 1] 1221 	jrc	00110$
      008515 20 39            [ 2] 1222 	jra	00114$
      008517                       1223 00107$:
                                   1224 ;	lcd_n1202.c: 90: else if((chr>127)&&(chr<148))	//Frame & Arrow Area
      008517 7B 0E            [ 1] 1225 	ld	a, (0x0e, sp)
      008519 A1 7F            [ 1] 1226 	cp	a, #0x7f
      00851B 23 33            [ 2] 1227 	jrule	00114$
      00851D 7B 0E            [ 1] 1228 	ld	a, (0x0e, sp)
      00851F A1 94            [ 1] 1229 	cp	a, #0x94
      008521 24 2D            [ 1] 1230 	jrnc	00114$
                                   1231 ;	lcd_n1202.c: 92: chridx=(chr-128)*8; //Start at index 128. 5 columns for each symbol
      008523 1E 0A            [ 2] 1232 	ldw	x, (0x0a, sp)
      008525 1D 00 80         [ 2] 1233 	subw	x, #0x0080
      008528 58               [ 2] 1234 	sllw	x
      008529 58               [ 2] 1235 	sllw	x
      00852A 58               [ 2] 1236 	sllw	x
                                   1237 ;	lcd_n1202.c: 93: for(ci=0;ci<8;ci++)
      00852B 90 AE 88 FF      [ 2] 1238 	ldw	y, #_font_arr+0
      00852F 17 04            [ 2] 1239 	ldw	(0x04, sp), y
      008531 1C 01 E0         [ 2] 1240 	addw	x, #0x01e0
      008534 1F 02            [ 2] 1241 	ldw	(0x02, sp), x
      008536 0F 01            [ 1] 1242 	clr	(0x01, sp)
      008538                       1243 00112$:
                                   1244 ;	lcd_n1202.c: 95: fchar = font_arr[chridx+480+ci]; //Get symbol pattern from Font Array
      008538 5F               [ 1] 1245 	clrw	x
      008539 7B 01            [ 1] 1246 	ld	a, (0x01, sp)
      00853B 97               [ 1] 1247 	ld	xl, a
      00853C 72 FB 02         [ 2] 1248 	addw	x, (0x02, sp)
      00853F 72 FB 04         [ 2] 1249 	addw	x, (0x04, sp)
      008542 F6               [ 1] 1250 	ld	a, (x)
                                   1251 ;	lcd_n1202.c: 96: lcdn1202_senddat(fchar); //Send pattern 1 byte at a time
      008543 88               [ 1] 1252 	push	a
      008544 CD 84 59         [ 4] 1253 	call	_lcdn1202_senddat
      008547 84               [ 1] 1254 	pop	a
                                   1255 ;	lcd_n1202.c: 93: for(ci=0;ci<8;ci++)
      008548 0C 01            [ 1] 1256 	inc	(0x01, sp)
      00854A 7B 01            [ 1] 1257 	ld	a, (0x01, sp)
      00854C A1 08            [ 1] 1258 	cp	a, #0x08
      00854E 25 E8            [ 1] 1259 	jrc	00112$
      008550                       1260 00114$:
      008550 5B 0B            [ 2] 1261 	addw	sp, #11
      008552 81               [ 4] 1262 	ret
                                   1263 ;	lcd_n1202.c: 102: void LCD_drawtext(char *text, unsigned char txtrow, unsigned char txtcol)
                                   1264 ;	-----------------------------------------
                                   1265 ;	 function LCD_drawtext
                                   1266 ;	-----------------------------------------
      008553                       1267 _LCD_drawtext:
      008553 52 02            [ 2] 1268 	sub	sp, #2
                                   1269 ;	lcd_n1202.c: 106: while(text[stridx] != 0) //Scan characters in string
      008555 5F               [ 1] 1270 	clrw	x
      008556 1F 01            [ 2] 1271 	ldw	(0x01, sp), x
      008558                       1272 00101$:
      008558 1E 05            [ 2] 1273 	ldw	x, (0x05, sp)
      00855A 72 FB 01         [ 2] 1274 	addw	x, (0x01, sp)
      00855D F6               [ 1] 1275 	ld	a, (x)
      00855E 97               [ 1] 1276 	ld	xl, a
      00855F 4D               [ 1] 1277 	tnz	a
      008560 27 19            [ 1] 1278 	jreq	00104$
                                   1279 ;	lcd_n1202.c: 108: LCD_drawchar(text[stridx],txtrow,txtcol+(8*stridx)); //Display each character
      008562 7B 02            [ 1] 1280 	ld	a, (0x02, sp)
      008564 48               [ 1] 1281 	sll	a
      008565 48               [ 1] 1282 	sll	a
      008566 48               [ 1] 1283 	sll	a
      008567 1B 08            [ 1] 1284 	add	a, (0x08, sp)
      008569 88               [ 1] 1285 	push	a
      00856A 7B 08            [ 1] 1286 	ld	a, (0x08, sp)
      00856C 88               [ 1] 1287 	push	a
      00856D 9F               [ 1] 1288 	ld	a, xl
      00856E 88               [ 1] 1289 	push	a
      00856F CD 84 BF         [ 4] 1290 	call	_LCD_drawchar
      008572 5B 03            [ 2] 1291 	addw	sp, #3
                                   1292 ;	lcd_n1202.c: 109: stridx++;
      008574 1E 01            [ 2] 1293 	ldw	x, (0x01, sp)
      008576 5C               [ 2] 1294 	incw	x
      008577 1F 01            [ 2] 1295 	ldw	(0x01, sp), x
      008579 20 DD            [ 2] 1296 	jra	00101$
      00857B                       1297 00104$:
      00857B 5B 02            [ 2] 1298 	addw	sp, #2
      00857D 81               [ 4] 1299 	ret
                                   1300 ;	lcd_n1202.c: 113: void LCD_drawint(unsigned int num, unsigned char numrow, unsigned char numcol)
                                   1301 ;	-----------------------------------------
                                   1302 ;	 function LCD_drawint
                                   1303 ;	-----------------------------------------
      00857E                       1304 _LCD_drawint:
      00857E 52 0C            [ 2] 1305 	sub	sp, #12
                                   1306 ;	lcd_n1202.c: 121: numb = num;
      008580 1E 0F            [ 2] 1307 	ldw	x, (0x0f, sp)
                                   1308 ;	lcd_n1202.c: 122: while(numb!=0) //Counting digit
      008582 4F               [ 1] 1309 	clr	a
      008583                       1310 00101$:
      008583 5D               [ 2] 1311 	tnzw	x
      008584 27 08            [ 1] 1312 	jreq	00114$
                                   1313 ;	lcd_n1202.c: 124: ndigit++;
      008586 4C               [ 1] 1314 	inc	a
                                   1315 ;	lcd_n1202.c: 125: numb /= 10; 
      008587 90 AE 00 0A      [ 2] 1316 	ldw	y, #0x000a
      00858B 65               [ 2] 1317 	divw	x, y
      00858C 20 F5            [ 2] 1318 	jra	00101$
      00858E                       1319 00114$:
      00858E 6B 0A            [ 1] 1320 	ld	(0x0a, sp), a
                                   1321 ;	lcd_n1202.c: 127: for(nd=0;nd<ndigit;nd++) //Converting each digit
      008590 4F               [ 1] 1322 	clr	a
      008591 96               [ 1] 1323 	ldw	x, sp
      008592 1C 00 03         [ 2] 1324 	addw	x, #3
      008595 1F 0B            [ 2] 1325 	ldw	(0x0b, sp), x
      008597                       1326 00106$:
      008597 11 0A            [ 1] 1327 	cp	a, (0x0a, sp)
      008599 24 27            [ 1] 1328 	jrnc	00104$
                                   1329 ;	lcd_n1202.c: 129: numb = num%10;
      00859B 1E 0F            [ 2] 1330 	ldw	x, (0x0f, sp)
      00859D 90 AE 00 0A      [ 2] 1331 	ldw	y, #0x000a
      0085A1 65               [ 2] 1332 	divw	x, y
      0085A2 17 01            [ 2] 1333 	ldw	(0x01, sp), y
                                   1334 ;	lcd_n1202.c: 130: num = num/10;
      0085A4 1E 0F            [ 2] 1335 	ldw	x, (0x0f, sp)
      0085A6 90 AE 00 0A      [ 2] 1336 	ldw	y, #0x000a
      0085AA 65               [ 2] 1337 	divw	x, y
      0085AB 1F 0F            [ 2] 1338 	ldw	(0x0f, sp), x
                                   1339 ;	lcd_n1202.c: 131: ibuff[ndigit-(nd+1)] = numb + '0'; //Start from last_index-1
      0085AD 4C               [ 1] 1340 	inc	a
      0085AE 6B 09            [ 1] 1341 	ld	(0x09, sp), a
      0085B0 7B 0A            [ 1] 1342 	ld	a, (0x0a, sp)
      0085B2 10 09            [ 1] 1343 	sub	a, (0x09, sp)
      0085B4 5F               [ 1] 1344 	clrw	x
      0085B5 97               [ 1] 1345 	ld	xl, a
      0085B6 72 FB 0B         [ 2] 1346 	addw	x, (0x0b, sp)
      0085B9 7B 02            [ 1] 1347 	ld	a, (0x02, sp)
      0085BB AB 30            [ 1] 1348 	add	a, #0x30
      0085BD F7               [ 1] 1349 	ld	(x), a
                                   1350 ;	lcd_n1202.c: 127: for(nd=0;nd<ndigit;nd++) //Converting each digit
      0085BE 7B 09            [ 1] 1351 	ld	a, (0x09, sp)
      0085C0 20 D5            [ 2] 1352 	jra	00106$
      0085C2                       1353 00104$:
                                   1354 ;	lcd_n1202.c: 133: ibuff[ndigit] = '\0'; //Last character is null
      0085C2 5F               [ 1] 1355 	clrw	x
      0085C3 7B 0A            [ 1] 1356 	ld	a, (0x0a, sp)
      0085C5 97               [ 1] 1357 	ld	xl, a
      0085C6 72 FB 0B         [ 2] 1358 	addw	x, (0x0b, sp)
      0085C9 7F               [ 1] 1359 	clr	(x)
                                   1360 ;	lcd_n1202.c: 135: LCD_drawtext(ibuff,numrow,numcol); //Display number as text
      0085CA 1E 0B            [ 2] 1361 	ldw	x, (0x0b, sp)
      0085CC 7B 12            [ 1] 1362 	ld	a, (0x12, sp)
      0085CE 88               [ 1] 1363 	push	a
      0085CF 7B 12            [ 1] 1364 	ld	a, (0x12, sp)
      0085D1 88               [ 1] 1365 	push	a
      0085D2 89               [ 2] 1366 	pushw	x
      0085D3 CD 85 53         [ 4] 1367 	call	_LCD_drawtext
      0085D6 5B 10            [ 2] 1368 	addw	sp, #16
      0085D8 81               [ 4] 1369 	ret
                                   1370 ;	lcd_n1202.c: 138: void LCD_clear()
                                   1371 ;	-----------------------------------------
                                   1372 ;	 function LCD_clear
                                   1373 ;	-----------------------------------------
      0085D9                       1374 _LCD_clear:
                                   1375 ;	lcd_n1202.c: 140: lcdn1202_sendcom(0xAE);  //Set Display off
      0085D9 4B AE            [ 1] 1376 	push	#0xae
      0085DB CD 84 4E         [ 4] 1377 	call	_lcdn1202_sendcom
      0085DE 84               [ 1] 1378 	pop	a
                                   1379 ;	lcd_n1202.c: 141: lcdn1202_clear(); //Clear display
      0085DF CD 84 87         [ 4] 1380 	call	_lcdn1202_clear
                                   1381 ;	lcd_n1202.c: 142: lcdn1202_sendcom(0xAF); //Set Display on
      0085E2 4B AF            [ 1] 1382 	push	#0xaf
      0085E4 CD 84 4E         [ 4] 1383 	call	_lcdn1202_sendcom
      0085E7 84               [ 1] 1384 	pop	a
      0085E8 81               [ 4] 1385 	ret
                                   1386 ;	lcd_n1202.c: 145: void LCD_clearblock(unsigned char row, unsigned char col_start, unsigned char col_fin)
                                   1387 ;	-----------------------------------------
                                   1388 ;	 function LCD_clearblock
                                   1389 ;	-----------------------------------------
      0085E9                       1390 _LCD_clearblock:
                                   1391 ;	lcd_n1202.c: 149: lcdn1202_setpos(row,col_start); //Set start position
      0085E9 7B 04            [ 1] 1392 	ld	a, (0x04, sp)
      0085EB 88               [ 1] 1393 	push	a
      0085EC 7B 04            [ 1] 1394 	ld	a, (0x04, sp)
      0085EE 88               [ 1] 1395 	push	a
      0085EF CD 84 64         [ 4] 1396 	call	_lcdn1202_setpos
      0085F2 5B 02            [ 2] 1397 	addw	sp, #2
                                   1398 ;	lcd_n1202.c: 150: for(col=col_start;col<=col_fin;col++) //Scan columns
      0085F4 7B 04            [ 1] 1399 	ld	a, (0x04, sp)
      0085F6                       1400 00103$:
      0085F6 11 05            [ 1] 1401 	cp	a, (0x05, sp)
      0085F8 22 0B            [ 1] 1402 	jrugt	00105$
                                   1403 ;	lcd_n1202.c: 152: lcdn1202_senddat(0);	//Send 0 to every pixel in a column
      0085FA 88               [ 1] 1404 	push	a
      0085FB 4B 00            [ 1] 1405 	push	#0x00
      0085FD CD 84 59         [ 4] 1406 	call	_lcdn1202_senddat
      008600 84               [ 1] 1407 	pop	a
      008601 84               [ 1] 1408 	pop	a
                                   1409 ;	lcd_n1202.c: 150: for(col=col_start;col<=col_fin;col++) //Scan columns
      008602 4C               [ 1] 1410 	inc	a
      008603 20 F1            [ 2] 1411 	jra	00103$
      008605                       1412 00105$:
      008605 81               [ 4] 1413 	ret
                                   1414 ;	lcd_n1202.c: 156: void LCD_normal()
                                   1415 ;	-----------------------------------------
                                   1416 ;	 function LCD_normal
                                   1417 ;	-----------------------------------------
      008606                       1418 _LCD_normal:
                                   1419 ;	lcd_n1202.c: 158: lcdn1202_sendcom(0xA6);	//Black Pixel in White Background
      008606 4B A6            [ 1] 1420 	push	#0xa6
      008608 CD 84 4E         [ 4] 1421 	call	_lcdn1202_sendcom
      00860B 84               [ 1] 1422 	pop	a
      00860C 81               [ 4] 1423 	ret
                                   1424 ;	lcd_n1202.c: 161: void LCD_reverse()
                                   1425 ;	-----------------------------------------
                                   1426 ;	 function LCD_reverse
                                   1427 ;	-----------------------------------------
      00860D                       1428 _LCD_reverse:
                                   1429 ;	lcd_n1202.c: 163: lcdn1202_sendcom(0xA7);	//White Pixel in Black Background
      00860D 4B A7            [ 1] 1430 	push	#0xa7
      00860F CD 84 4E         [ 4] 1431 	call	_lcdn1202_sendcom
      008612 84               [ 1] 1432 	pop	a
      008613 81               [ 4] 1433 	ret
                                   1434 ;	lcd_n1202.c: 166: void LCD_BL_ON()
                                   1435 ;	-----------------------------------------
                                   1436 ;	 function LCD_BL_ON
                                   1437 ;	-----------------------------------------
      008614                       1438 _LCD_BL_ON:
                                   1439 ;	lcd_n1202.c: 168: lcdn1202_blon(); //Backlight on
      008614 CC 83 F2         [ 2] 1440 	jp	_lcdn1202_blon
                                   1441 ;	lcd_n1202.c: 171: void LCD_BL_OFF()
                                   1442 ;	-----------------------------------------
                                   1443 ;	 function LCD_BL_OFF
                                   1444 ;	-----------------------------------------
      008617                       1445 _LCD_BL_OFF:
                                   1446 ;	lcd_n1202.c: 173: lcdn1202_bloff(); //Backlight off
      008617 CC 83 FA         [ 2] 1447 	jp	_lcdn1202_bloff
                                   1448 ;	powerman.c: 6: void powerman_init() //GPIOs Initialization 
                                   1449 ;	-----------------------------------------
                                   1450 ;	 function powerman_init
                                   1451 ;	-----------------------------------------
      00861A                       1452 _powerman_init:
                                   1453 ;	powerman.c: 8: POWLATCHDDR |= (OUTPUT<<POW_LATCH);
      00861A AE 50 16         [ 2] 1454 	ldw	x, #0x5016
      00861D F6               [ 1] 1455 	ld	a, (x)
      00861E AA 20            [ 1] 1456 	or	a, #0x20
      008620 F7               [ 1] 1457 	ld	(x), a
                                   1458 ;	powerman.c: 9: POWLATCHCR1 |= (pushpull<<POW_LATCH);
      008621 AE 50 17         [ 2] 1459 	ldw	x, #0x5017
      008624 F6               [ 1] 1460 	ld	a, (x)
      008625 AA 20            [ 1] 1461 	or	a, #0x20
      008627 F7               [ 1] 1462 	ld	(x), a
                                   1463 ;	powerman.c: 10: POWLATCHCR2 |= (speed_2MHz<<POW_LATCH);
      008628 AE 50 18         [ 2] 1464 	ldw	x, #0x5018
      00862B F6               [ 1] 1465 	ld	a, (x)
      00862C AE 50 18         [ 2] 1466 	ldw	x, #0x5018
      00862F F7               [ 1] 1467 	ld	(x), a
                                   1468 ;	powerman.c: 12: POWDETDDR |= (INPUT<<POW_DET);
      008630 AE 50 1B         [ 2] 1469 	ldw	x, #0x501b
      008633 F6               [ 1] 1470 	ld	a, (x)
      008634 AE 50 1B         [ 2] 1471 	ldw	x, #0x501b
      008637 F7               [ 1] 1472 	ld	(x), a
                                   1473 ;	powerman.c: 13: POWDETCR1 |= (floating<<POW_DET);
      008638 AE 50 1C         [ 2] 1474 	ldw	x, #0x501c
      00863B F6               [ 1] 1475 	ld	a, (x)
      00863C AE 50 1C         [ 2] 1476 	ldw	x, #0x501c
      00863F F7               [ 1] 1477 	ld	(x), a
                                   1478 ;	powerman.c: 14: POWDETCR2 |= (exti_disabled<<POW_DET);
      008640 AE 50 1D         [ 2] 1479 	ldw	x, #0x501d
      008643 F6               [ 1] 1480 	ld	a, (x)
      008644 AE 50 1D         [ 2] 1481 	ldw	x, #0x501d
      008647 F7               [ 1] 1482 	ld	(x), a
                                   1483 ;	powerman.c: 16: CHGDDR |= (INPUT<<CHG_PR) | (INPUT<<CHG_FL);
      008648 AE 50 07         [ 2] 1484 	ldw	x, #0x5007
      00864B F6               [ 1] 1485 	ld	a, (x)
      00864C AE 50 07         [ 2] 1486 	ldw	x, #0x5007
      00864F F7               [ 1] 1487 	ld	(x), a
                                   1488 ;	powerman.c: 17: CHGCR1 |= (pullup<<CHG_PR) | (pullup<<CHG_FL);
      008650 AE 50 08         [ 2] 1489 	ldw	x, #0x5008
      008653 F6               [ 1] 1490 	ld	a, (x)
      008654 AA C0            [ 1] 1491 	or	a, #0xc0
      008656 F7               [ 1] 1492 	ld	(x), a
                                   1493 ;	powerman.c: 18: CHGCR2 |= (exti_disabled<<CHG_PR) | (exti_disabled<<CHG_FL);
      008657 AE 50 09         [ 2] 1494 	ldw	x, #0x5009
      00865A F6               [ 1] 1495 	ld	a, (x)
      00865B AE 50 09         [ 2] 1496 	ldw	x, #0x5009
      00865E F7               [ 1] 1497 	ld	(x), a
      00865F 81               [ 4] 1498 	ret
                                   1499 ;	powerman.c: 21: void Power_Latch() //Activate Power-Latch : latch Power-Switch transistor
                                   1500 ;	-----------------------------------------
                                   1501 ;	 function Power_Latch
                                   1502 ;	-----------------------------------------
      008660                       1503 _Power_Latch:
                                   1504 ;	powerman.c: 23: POWLATCHODR |= POWLATCH_MASKH; //POWER-LATCH = 1
      008660 AE 50 14         [ 2] 1505 	ldw	x, #0x5014
      008663 F6               [ 1] 1506 	ld	a, (x)
      008664 AA 20            [ 1] 1507 	or	a, #0x20
      008666 F7               [ 1] 1508 	ld	(x), a
      008667 81               [ 4] 1509 	ret
                                   1510 ;	powerman.c: 26: void Power_Unlatch() //Release Power-Latch : unlatch Power-Switch transistor
                                   1511 ;	-----------------------------------------
                                   1512 ;	 function Power_Unlatch
                                   1513 ;	-----------------------------------------
      008668                       1514 _Power_Unlatch:
                                   1515 ;	powerman.c: 28: POWLATCHODR &= POWLATCH_MASKL; //POWER-LATCH = 0
      008668 AE 50 14         [ 2] 1516 	ldw	x, #0x5014
      00866B F6               [ 1] 1517 	ld	a, (x)
      00866C A4 DF            [ 1] 1518 	and	a, #0xdf
      00866E F7               [ 1] 1519 	ld	(x), a
      00866F 81               [ 4] 1520 	ret
                                   1521 ;	powerman.c: 31: unsigned char read_pkey() //Check Powerkey State
                                   1522 ;	-----------------------------------------
                                   1523 ;	 function read_pkey
                                   1524 ;	-----------------------------------------
      008670                       1525 _read_pkey:
                                   1526 ;	powerman.c: 35: pkeyval = POWDETIDR & POWDET_MASKH; //Read POWER-DETECT state
      008670 AE 50 1A         [ 2] 1527 	ldw	x, #0x501a
      008673 F6               [ 1] 1528 	ld	a, (x)
      008674 A4 10            [ 1] 1529 	and	a, #0x10
                                   1530 ;	powerman.c: 37: return pkeyval;
      008676 81               [ 4] 1531 	ret
                                   1532 ;	powerman.c: 40: unsigned int vbat_mon() //Battery Voltage reading
                                   1533 ;	-----------------------------------------
                                   1534 ;	 function vbat_mon
                                   1535 ;	-----------------------------------------
      008677                       1536 _vbat_mon:
                                   1537 ;	powerman.c: 47: vbat = read_adc(BAT_LVL)*66/40*3; //VADC_MAX=3333mV,VBAT_MAX=5000mV
      008677 4B 03            [ 1] 1538 	push	#0x03
      008679 CD 82 25         [ 4] 1539 	call	_read_adc
      00867C 84               [ 1] 1540 	pop	a
      00867D 89               [ 2] 1541 	pushw	x
      00867E 4B 42            [ 1] 1542 	push	#0x42
      008680 4B 00            [ 1] 1543 	push	#0x00
      008682 CD 8B CD         [ 4] 1544 	call	__mulint
      008685 5B 04            [ 2] 1545 	addw	sp, #4
      008687 90 AE 00 28      [ 2] 1546 	ldw	y, #0x0028
      00868B 65               [ 2] 1547 	divw	x, y
      00868C 89               [ 2] 1548 	pushw	x
      00868D 4B 03            [ 1] 1549 	push	#0x03
      00868F 4B 00            [ 1] 1550 	push	#0x00
      008691 CD 8B CD         [ 4] 1551 	call	__mulint
      008694 5B 04            [ 2] 1552 	addw	sp, #4
                                   1553 ;	powerman.c: 49: return vbat;
      008696 81               [ 4] 1554 	ret
                                   1555 ;	powerman.c: 52: unsigned char chgst_mon() //Charging Status reading
                                   1556 ;	-----------------------------------------
                                   1557 ;	 function chgst_mon
                                   1558 ;	-----------------------------------------
      008697                       1559 _chgst_mon:
                                   1560 ;	powerman.c: 57: chgst = CHGIDR & CHGST_MASKH; //Check CHG-PR & CHG-FL
      008697 AE 50 06         [ 2] 1561 	ldw	x, #0x5006
      00869A F6               [ 1] 1562 	ld	a, (x)
      00869B A4 C0            [ 1] 1563 	and	a, #0xc0
                                   1564 ;	powerman.c: 59: return chgst;
      00869D 81               [ 4] 1565 	ret
                                   1566 ;	main.c: 40: int main()
                                   1567 ;	-----------------------------------------
                                   1568 ;	 function main
                                   1569 ;	-----------------------------------------
      00869E                       1570 _main:
                                   1571 ;	main.c: 43: clock_init();
      00869E CD 81 4F         [ 4] 1572 	call	_clock_init
                                   1573 ;	main.c: 44: delay_init();
      0086A1 CD 80 A0         [ 4] 1574 	call	_delay_init
                                   1575 ;	main.c: 45: powerman_init();
      0086A4 CD 86 1A         [ 4] 1576 	call	_powerman_init
                                   1577 ;	main.c: 46: adc_init();
      0086A7 CD 82 18         [ 4] 1578 	call	_adc_init
                                   1579 ;	main.c: 47: lcdn1202_init();
      0086AA CD 84 02         [ 4] 1580 	call	_lcdn1202_init
                                   1581 ;	main.c: 48: LCD_clear();
      0086AD CD 85 D9         [ 4] 1582 	call	_LCD_clear
                                   1583 ;	main.c: 51: pkeylock = 1; //lock powerkey for next press/click
      0086B0 35 01 00 03      [ 1] 1584 	mov	_pkeylock+0, #0x01
                                   1585 ;	main.c: 52: chg_disp = 0; //Charging Status & Battery Voltage is not displayed
      0086B4 72 5F 00 05      [ 1] 1586 	clr	_chg_disp+0
                                   1587 ;	main.c: 55: pkeyp = read_pkey(); //check if Powerkey is pressed
      0086B8 CD 86 70         [ 4] 1588 	call	_read_pkey
                                   1589 ;	main.c: 56: if(pkeyp==POWKEY_PRESSED) Page_Main(); //if pressed then Power-On Mode, go to Main Page
      0086BB C7 00 02         [ 1] 1590 	ld	_pkeyp+0, a
      0086BE A1 10            [ 1] 1591 	cp	a, #0x10
      0086C0 26 05            [ 1] 1592 	jrne	00102$
      0086C2 CD 87 40         [ 4] 1593 	call	_Page_Main
      0086C5 20 03            [ 2] 1594 	jra	00103$
      0086C7                       1595 00102$:
                                   1596 ;	main.c: 57: else Page_Charging(); //if not pressed then Charging Mode, go to Charging Page
      0086C7 CD 87 14         [ 4] 1597 	call	_Page_Charging
      0086CA                       1598 00103$:
                                   1599 ;	main.c: 59: loop();
      0086CA CD 86 CF         [ 4] 1600 	call	_loop
                                   1601 ;	main.c: 60: return 0;
      0086CD 5F               [ 1] 1602 	clrw	x
      0086CE 81               [ 4] 1603 	ret
                                   1604 ;	main.c: 66: void loop()
                                   1605 ;	-----------------------------------------
                                   1606 ;	 function loop
                                   1607 ;	-----------------------------------------
      0086CF                       1608 _loop:
                                   1609 ;	main.c: 69: ktime = 0; //start Long-Press counter with 0
      0086CF 72 5F 00 06      [ 1] 1610 	clr	_ktime+0
                                   1611 ;	main.c: 71: while(1)
      0086D3                       1612 00111$:
                                   1613 ;	main.c: 73: if((read_pkey()==0)&&(pkeylock!=0)) pkeylock = 0; //unlock Powerkey from Long-Press
      0086D3 CD 86 70         [ 4] 1614 	call	_read_pkey
      0086D6 4D               [ 1] 1615 	tnz	a
      0086D7 26 0A            [ 1] 1616 	jrne	00107$
      0086D9 72 5D 00 03      [ 1] 1617 	tnz	_pkeylock+0
      0086DD 27 04            [ 1] 1618 	jreq	00107$
      0086DF 72 5F 00 03      [ 1] 1619 	clr	_pkeylock+0
                                   1620 ;	main.c: 76: while((read_pkey()!=POWKEY_UNPRESSED)&&(pkeylock==0)) //check if Powerkey is pressed and unlocked
      0086E3                       1621 00107$:
      0086E3 CD 86 70         [ 4] 1622 	call	_read_pkey
      0086E6 4D               [ 1] 1623 	tnz	a
      0086E7 27 22            [ 1] 1624 	jreq	00109$
      0086E9 72 5D 00 03      [ 1] 1625 	tnz	_pkeylock+0
      0086ED 26 1C            [ 1] 1626 	jrne	00109$
                                   1627 ;	main.c: 78: pkeyp = read_pkey();
      0086EF CD 86 70         [ 4] 1628 	call	_read_pkey
      0086F2 C7 00 02         [ 1] 1629 	ld	_pkeyp+0, a
                                   1630 ;	main.c: 79: delay_ms(KDELAY); //Anti-Bouncing delay
      0086F5 4B 14            [ 1] 1631 	push	#0x14
      0086F7 5F               [ 1] 1632 	clrw	x
      0086F8 89               [ 2] 1633 	pushw	x
      0086F9 4B 00            [ 1] 1634 	push	#0x00
      0086FB CD 80 F5         [ 4] 1635 	call	_delay_ms
      0086FE 5B 04            [ 2] 1636 	addw	sp, #4
                                   1637 ;	main.c: 80: ktime++; //increment Long-Press counter
      008700 C6 00 06         [ 1] 1638 	ld	a, _ktime+0
      008703 4C               [ 1] 1639 	inc	a
                                   1640 ;	main.c: 81: if(ktime>KLONGP) break; //detect Long-Press
      008704 C7 00 06         [ 1] 1641 	ld	_ktime+0, a
      008707 A1 64            [ 1] 1642 	cp	a, #0x64
      008709 23 D8            [ 2] 1643 	jrule	00107$
      00870B                       1644 00109$:
                                   1645 ;	main.c: 83: update_pkey(); //Execute Action for Powerkey
      00870B CD 87 7A         [ 4] 1646 	call	_update_pkey
                                   1647 ;	main.c: 85: disp_bat_status(); //Display Battery Status
      00870E CD 88 75         [ 4] 1648 	call	_disp_bat_status
      008711 20 C0            [ 2] 1649 	jra	00111$
      008713 81               [ 4] 1650 	ret
                                   1651 ;	main.c: 94: void Page_Charging() //Charging Mode
                                   1652 ;	-----------------------------------------
                                   1653 ;	 function Page_Charging
                                   1654 ;	-----------------------------------------
      008714                       1655 _Page_Charging:
                                   1656 ;	main.c: 96: page_id = 0; //Charging Page ID
      008714 72 5F 00 04      [ 1] 1657 	clr	_page_id+0
                                   1658 ;	main.c: 97: LCD_clear();
      008718 CD 85 D9         [ 4] 1659 	call	_LCD_clear
                                   1660 ;	main.c: 99: chg_disp = 1; //Charging Status displayed
      00871B 35 01 00 05      [ 1] 1661 	mov	_chg_disp+0, #0x01
                                   1662 ;	main.c: 100: LCD_drawtext("CHARGING",2,16);
      00871F AE 8B 7F         [ 2] 1663 	ldw	x, #___str_0+0
      008722 4B 10            [ 1] 1664 	push	#0x10
      008724 4B 02            [ 1] 1665 	push	#0x02
      008726 89               [ 2] 1666 	pushw	x
      008727 CD 85 53         [ 4] 1667 	call	_LCD_drawtext
      00872A 5B 04            [ 2] 1668 	addw	sp, #4
                                   1669 ;	main.c: 101: delay_ms(2000); //disable Powerkey for a while
      00872C 4B D0            [ 1] 1670 	push	#0xd0
      00872E 4B 07            [ 1] 1671 	push	#0x07
      008730 5F               [ 1] 1672 	clrw	x
      008731 89               [ 2] 1673 	pushw	x
      008732 CD 80 F5         [ 4] 1674 	call	_delay_ms
      008735 5B 04            [ 2] 1675 	addw	sp, #4
                                   1676 ;	main.c: 102: pkeyp = 0; //reset Powerkey flag
      008737 72 5F 00 02      [ 1] 1677 	clr	_pkeyp+0
                                   1678 ;	main.c: 103: scf = 1; //default toggle state of Single Click in Charging Mode
      00873B 35 01 00 07      [ 1] 1679 	mov	_scf+0, #0x01
      00873F 81               [ 4] 1680 	ret
                                   1681 ;	main.c: 106: void Page_Main() //Power-On Mode
                                   1682 ;	-----------------------------------------
                                   1683 ;	 function Page_Main
                                   1684 ;	-----------------------------------------
      008740                       1685 _Page_Main:
                                   1686 ;	main.c: 108: page_id = 1; //Power-On Page ID
      008740 35 01 00 04      [ 1] 1687 	mov	_page_id+0, #0x01
                                   1688 ;	main.c: 109: Power_Latch(); //activate Power-Latch
      008744 CD 86 60         [ 4] 1689 	call	_Power_Latch
                                   1690 ;	main.c: 110: LCD_clear();
      008747 CD 85 D9         [ 4] 1691 	call	_LCD_clear
                                   1692 ;	main.c: 112: chg_disp = 0; //Charging Status not displayed
      00874A 72 5F 00 05      [ 1] 1693 	clr	_chg_disp+0
                                   1694 ;	main.c: 113: LCD_drawtext("POWER ON",2,16);
      00874E AE 8B 88         [ 2] 1695 	ldw	x, #___str_1+0
      008751 4B 10            [ 1] 1696 	push	#0x10
      008753 4B 02            [ 1] 1697 	push	#0x02
      008755 89               [ 2] 1698 	pushw	x
      008756 CD 85 53         [ 4] 1699 	call	_LCD_drawtext
      008759 5B 04            [ 2] 1700 	addw	sp, #4
                                   1701 ;	main.c: 114: delay_ms(2000); //disable Powerkey for a while
      00875B 4B D0            [ 1] 1702 	push	#0xd0
      00875D 4B 07            [ 1] 1703 	push	#0x07
      00875F 5F               [ 1] 1704 	clrw	x
      008760 89               [ 2] 1705 	pushw	x
      008761 CD 80 F5         [ 4] 1706 	call	_delay_ms
      008764 5B 04            [ 2] 1707 	addw	sp, #4
                                   1708 ;	main.c: 115: LCD_clearblock(2,16,79);
      008766 4B 4F            [ 1] 1709 	push	#0x4f
      008768 4B 10            [ 1] 1710 	push	#0x10
      00876A 4B 02            [ 1] 1711 	push	#0x02
      00876C CD 85 E9         [ 4] 1712 	call	_LCD_clearblock
      00876F 5B 03            [ 2] 1713 	addw	sp, #3
                                   1714 ;	main.c: 116: pkeyp = 0; //reset Powerkey flag
      008771 72 5F 00 02      [ 1] 1715 	clr	_pkeyp+0
                                   1716 ;	main.c: 117: scf = 1; //default toggle state of Single Click in Power-On Mode
      008775 35 01 00 07      [ 1] 1717 	mov	_scf+0, #0x01
      008779 81               [ 4] 1718 	ret
                                   1719 ;	main.c: 123: void update_pkey() //Action options for Powerkey
                                   1720 ;	-----------------------------------------
                                   1721 ;	 function update_pkey
                                   1722 ;	-----------------------------------------
      00877A                       1723 _update_pkey:
      00877A 52 02            [ 2] 1724 	sub	sp, #2
                                   1725 ;	main.c: 125: if((pkeyp==POWKEY_PRESSED)&&(page_id==1)&&(ktime>KLONGP)) //Long-Press in Power-On Mode
      00877C C6 00 02         [ 1] 1726 	ld	a, _pkeyp+0
      00877F A1 10            [ 1] 1727 	cp	a, #0x10
      008781 26 06            [ 1] 1728 	jrne	00171$
      008783 A6 01            [ 1] 1729 	ld	a, #0x01
      008785 6B 02            [ 1] 1730 	ld	(0x02, sp), a
      008787 20 02            [ 2] 1731 	jra	00172$
      008789                       1732 00171$:
      008789 0F 02            [ 1] 1733 	clr	(0x02, sp)
      00878B                       1734 00172$:
      00878B C6 00 04         [ 1] 1735 	ld	a, _page_id+0
      00878E A1 01            [ 1] 1736 	cp	a, #0x01
      008790 26 06            [ 1] 1737 	jrne	00174$
      008792 A6 01            [ 1] 1738 	ld	a, #0x01
      008794 6B 01            [ 1] 1739 	ld	(0x01, sp), a
      008796 20 02            [ 2] 1740 	jra	00175$
      008798                       1741 00174$:
      008798 0F 01            [ 1] 1742 	clr	(0x01, sp)
      00879A                       1743 00175$:
      00879A C6 00 06         [ 1] 1744 	ld	a, _ktime+0
      00879D A1 64            [ 1] 1745 	cp	a, #0x64
      00879F 22 03            [ 1] 1746 	jrugt	00176$
      0087A1 4F               [ 1] 1747 	clr	a
      0087A2 20 02            [ 2] 1748 	jra	00177$
      0087A4                       1749 00176$:
      0087A4 A6 01            [ 1] 1750 	ld	a, #0x01
      0087A6                       1751 00177$:
      0087A6 0D 02            [ 1] 1752 	tnz	(0x02, sp)
      0087A8 27 32            [ 1] 1753 	jreq	00116$
      0087AA 0D 01            [ 1] 1754 	tnz	(0x01, sp)
      0087AC 27 2E            [ 1] 1755 	jreq	00116$
      0087AE 4D               [ 1] 1756 	tnz	a
      0087AF 27 2B            [ 1] 1757 	jreq	00116$
                                   1758 ;	main.c: 127: pkeyp = 0; //reset Powerkey value
      0087B1 72 5F 00 02      [ 1] 1759 	clr	_pkeyp+0
                                   1760 ;	main.c: 128: pkeylock = 1; //lock Powerkey for next press/click
      0087B5 35 01 00 03      [ 1] 1761 	mov	_pkeylock+0, #0x01
                                   1762 ;	main.c: 129: Power_Unlatch(); //release Power-Latch
      0087B9 CD 86 68         [ 4] 1763 	call	_Power_Unlatch
                                   1764 ;	main.c: 130: LCD_clear();
      0087BC CD 85 D9         [ 4] 1765 	call	_LCD_clear
                                   1766 ;	main.c: 131: LCD_drawtext("POWR OFF",2,16);
      0087BF AE 8B 91         [ 2] 1767 	ldw	x, #___str_2+0
      0087C2 4B 10            [ 1] 1768 	push	#0x10
      0087C4 4B 02            [ 1] 1769 	push	#0x02
      0087C6 89               [ 2] 1770 	pushw	x
      0087C7 CD 85 53         [ 4] 1771 	call	_LCD_drawtext
      0087CA 5B 04            [ 2] 1772 	addw	sp, #4
                                   1773 ;	main.c: 132: delay_ms(2000); //make sure Page_Charging is not pop-up after Power-Off
      0087CC 4B D0            [ 1] 1774 	push	#0xd0
      0087CE 4B 07            [ 1] 1775 	push	#0x07
      0087D0 5F               [ 1] 1776 	clrw	x
      0087D1 89               [ 2] 1777 	pushw	x
      0087D2 CD 80 F5         [ 4] 1778 	call	_delay_ms
      0087D5 5B 04            [ 2] 1779 	addw	sp, #4
                                   1780 ;	main.c: 133: Page_Charging(); //change to Charging Mode
      0087D7 CD 87 14         [ 4] 1781 	call	_Page_Charging
      0087DA 20 42            [ 2] 1782 	jra	00117$
      0087DC                       1783 00116$:
                                   1784 ;	main.c: 135: else if((pkeyp==POWKEY_PRESSED)&&(page_id==1)&&(ktime<=KLONGP)) //Single-Click in Power-On Mode
      0087DC 0D 02            [ 1] 1785 	tnz	(0x02, sp)
      0087DE 27 10            [ 1] 1786 	jreq	00111$
      0087E0 0D 01            [ 1] 1787 	tnz	(0x01, sp)
      0087E2 27 0C            [ 1] 1788 	jreq	00111$
      0087E4 4D               [ 1] 1789 	tnz	a
      0087E5 26 09            [ 1] 1790 	jrne	00111$
                                   1791 ;	main.c: 137: pkeyp = 0; //reset Powerkey value
      0087E7 72 5F 00 02      [ 1] 1792 	clr	_pkeyp+0
                                   1793 ;	main.c: 138: on_single_click();
      0087EB CD 88 25         [ 4] 1794 	call	_on_single_click
      0087EE 20 2E            [ 2] 1795 	jra	00117$
      0087F0                       1796 00111$:
                                   1797 ;	main.c: 141: else if((pkeyp==POWKEY_PRESSED)&&(page_id==0)&&(ktime>KLONGP)) //Long-Press in Charging Mode
      0087F0 0D 02            [ 1] 1798 	tnz	(0x02, sp)
      0087F2 27 16            [ 1] 1799 	jreq	00106$
      0087F4 72 5D 00 04      [ 1] 1800 	tnz	_page_id+0
      0087F8 26 10            [ 1] 1801 	jrne	00106$
      0087FA 4D               [ 1] 1802 	tnz	a
      0087FB 27 0D            [ 1] 1803 	jreq	00106$
                                   1804 ;	main.c: 143: pkeyp = 0; //reset Powerkey value
      0087FD 72 5F 00 02      [ 1] 1805 	clr	_pkeyp+0
                                   1806 ;	main.c: 144: pkeylock = 1; //lock Powerkey for next press/click
      008801 35 01 00 03      [ 1] 1807 	mov	_pkeylock+0, #0x01
                                   1808 ;	main.c: 145: Page_Main(); //change to Power-On Mode
      008805 CD 87 40         [ 4] 1809 	call	_Page_Main
      008808 20 14            [ 2] 1810 	jra	00117$
      00880A                       1811 00106$:
                                   1812 ;	main.c: 147: else if((pkeyp==POWKEY_PRESSED)&&(page_id==0)&&(ktime<=KLONGP)) //Single-Click in Charging Mode
      00880A 0D 02            [ 1] 1813 	tnz	(0x02, sp)
      00880C 27 10            [ 1] 1814 	jreq	00117$
      00880E 72 5D 00 04      [ 1] 1815 	tnz	_page_id+0
      008812 26 0A            [ 1] 1816 	jrne	00117$
      008814 4D               [ 1] 1817 	tnz	a
      008815 26 07            [ 1] 1818 	jrne	00117$
                                   1819 ;	main.c: 149: pkeyp = 0; //reset Powerkey value
      008817 72 5F 00 02      [ 1] 1820 	clr	_pkeyp+0
                                   1821 ;	main.c: 150: chg_single_click();
      00881B CD 88 43         [ 4] 1822 	call	_chg_single_click
      00881E                       1823 00117$:
                                   1824 ;	main.c: 153: ktime = 0;
      00881E 72 5F 00 06      [ 1] 1825 	clr	_ktime+0
      008822 5B 02            [ 2] 1826 	addw	sp, #2
      008824 81               [ 4] 1827 	ret
                                   1828 ;	main.c: 159: void on_single_click() //Single-Click action on Power-On Mode
                                   1829 ;	-----------------------------------------
                                   1830 ;	 function on_single_click
                                   1831 ;	-----------------------------------------
      008825                       1832 _on_single_click:
                                   1833 ;	main.c: 161: if(scf==0)
      008825 72 5D 00 07      [ 1] 1834 	tnz	_scf+0
      008829 26 09            [ 1] 1835 	jrne	00104$
                                   1836 ;	main.c: 163: LCD_BL_ON(); //Backlight On
      00882B CD 86 14         [ 4] 1837 	call	_LCD_BL_ON
                                   1838 ;	main.c: 164: scf = 1;
      00882E 35 01 00 07      [ 1] 1839 	mov	_scf+0, #0x01
      008832 20 0E            [ 2] 1840 	jra	00106$
      008834                       1841 00104$:
                                   1842 ;	main.c: 166: else if(scf==1)
      008834 C6 00 07         [ 1] 1843 	ld	a, _scf+0
      008837 A1 01            [ 1] 1844 	cp	a, #0x01
      008839 26 07            [ 1] 1845 	jrne	00106$
                                   1846 ;	main.c: 168: LCD_BL_OFF(); //Backlight Off
      00883B CD 86 17         [ 4] 1847 	call	_LCD_BL_OFF
                                   1848 ;	main.c: 169: scf = 0;
      00883E 72 5F 00 07      [ 1] 1849 	clr	_scf+0
      008842                       1850 00106$:
      008842 81               [ 4] 1851 	ret
                                   1852 ;	main.c: 174: void chg_single_click() //Single-Click action on Charging Mode
                                   1853 ;	-----------------------------------------
                                   1854 ;	 function chg_single_click
                                   1855 ;	-----------------------------------------
      008843                       1856 _chg_single_click:
                                   1857 ;	main.c: 176: if(scf==0)
      008843 72 5D 00 07      [ 1] 1858 	tnz	_scf+0
      008847 26 13            [ 1] 1859 	jrne	00104$
                                   1860 ;	main.c: 178: LCD_drawtext("        ",4,16); //Clear text
      008849 AE 8B 9A         [ 2] 1861 	ldw	x, #___str_3+0
      00884C 4B 10            [ 1] 1862 	push	#0x10
      00884E 4B 04            [ 1] 1863 	push	#0x04
      008850 89               [ 2] 1864 	pushw	x
      008851 CD 85 53         [ 4] 1865 	call	_LCD_drawtext
      008854 5B 04            [ 2] 1866 	addw	sp, #4
                                   1867 ;	main.c: 179: scf = 1;
      008856 35 01 00 07      [ 1] 1868 	mov	_scf+0, #0x01
      00885A 20 18            [ 2] 1869 	jra	00106$
      00885C                       1870 00104$:
                                   1871 ;	main.c: 181: else if(scf==1)
      00885C C6 00 07         [ 1] 1872 	ld	a, _scf+0
      00885F A1 01            [ 1] 1873 	cp	a, #0x01
      008861 26 11            [ 1] 1874 	jrne	00106$
                                   1875 ;	main.c: 183: LCD_drawtext("bonusoid",4,16); //Display text
      008863 AE 8B A3         [ 2] 1876 	ldw	x, #___str_4+0
      008866 4B 10            [ 1] 1877 	push	#0x10
      008868 4B 04            [ 1] 1878 	push	#0x04
      00886A 89               [ 2] 1879 	pushw	x
      00886B CD 85 53         [ 4] 1880 	call	_LCD_drawtext
      00886E 5B 04            [ 2] 1881 	addw	sp, #4
                                   1882 ;	main.c: 184: scf = 0;
      008870 72 5F 00 07      [ 1] 1883 	clr	_scf+0
      008874                       1884 00106$:
      008874 81               [ 4] 1885 	ret
                                   1886 ;	main.c: 192: void disp_bat_status()
                                   1887 ;	-----------------------------------------
                                   1888 ;	 function disp_bat_status
                                   1889 ;	-----------------------------------------
      008875                       1890 _disp_bat_status:
                                   1891 ;	main.c: 195: chgst = chgst_mon();
      008875 CD 86 97         [ 4] 1892 	call	_chgst_mon
                                   1893 ;	main.c: 197: if(chgst==CHG_PROGRESS) LCD_drawtext("BCHRG",0,0);
      008878 C7 00 0A         [ 1] 1894 	ld	_chgst+0, a
      00887B A1 80            [ 1] 1895 	cp	a, #0x80
      00887D 26 0F            [ 1] 1896 	jrne	00111$
      00887F AE 8B AC         [ 2] 1897 	ldw	x, #___str_5+0
      008882 4B 00            [ 1] 1898 	push	#0x00
      008884 4B 00            [ 1] 1899 	push	#0x00
      008886 89               [ 2] 1900 	pushw	x
      008887 CD 85 53         [ 4] 1901 	call	_LCD_drawtext
      00888A 5B 04            [ 2] 1902 	addw	sp, #4
      00888C 20 4E            [ 2] 1903 	jra	00112$
      00888E                       1904 00111$:
                                   1905 ;	main.c: 198: else if(chgst==CHG_FULL) LCD_drawtext("BFULL",0,0);
      00888E C6 00 0A         [ 1] 1906 	ld	a, _chgst+0
      008891 A1 40            [ 1] 1907 	cp	a, #0x40
      008893 26 0F            [ 1] 1908 	jrne	00108$
      008895 AE 8B B2         [ 2] 1909 	ldw	x, #___str_6+0
      008898 4B 00            [ 1] 1910 	push	#0x00
      00889A 4B 00            [ 1] 1911 	push	#0x00
      00889C 89               [ 2] 1912 	pushw	x
      00889D CD 85 53         [ 4] 1913 	call	_LCD_drawtext
      0088A0 5B 04            [ 2] 1914 	addw	sp, #4
      0088A2 20 38            [ 2] 1915 	jra	00112$
      0088A4                       1916 00108$:
                                   1917 ;	main.c: 199: else if(chgst==CHG_NOCHG) LCD_drawtext("NOCHG",0,0);
      0088A4 C6 00 0A         [ 1] 1918 	ld	a, _chgst+0
      0088A7 A1 C0            [ 1] 1919 	cp	a, #0xc0
      0088A9 26 0F            [ 1] 1920 	jrne	00105$
      0088AB AE 8B B8         [ 2] 1921 	ldw	x, #___str_7+0
      0088AE 4B 00            [ 1] 1922 	push	#0x00
      0088B0 4B 00            [ 1] 1923 	push	#0x00
      0088B2 89               [ 2] 1924 	pushw	x
      0088B3 CD 85 53         [ 4] 1925 	call	_LCD_drawtext
      0088B6 5B 04            [ 2] 1926 	addw	sp, #4
      0088B8 20 22            [ 2] 1927 	jra	00112$
      0088BA                       1928 00105$:
                                   1929 ;	main.c: 200: else if(chgst==CHG_NOBAT) LCD_drawtext("NOBAT",0,0);
      0088BA 72 5D 00 0A      [ 1] 1930 	tnz	_chgst+0
      0088BE 26 0F            [ 1] 1931 	jrne	00102$
      0088C0 AE 8B BE         [ 2] 1932 	ldw	x, #___str_8+0
      0088C3 4B 00            [ 1] 1933 	push	#0x00
      0088C5 4B 00            [ 1] 1934 	push	#0x00
      0088C7 89               [ 2] 1935 	pushw	x
      0088C8 CD 85 53         [ 4] 1936 	call	_LCD_drawtext
      0088CB 5B 04            [ 2] 1937 	addw	sp, #4
      0088CD 20 0D            [ 2] 1938 	jra	00112$
      0088CF                       1939 00102$:
                                   1940 ;	main.c: 201: else LCD_drawtext("UNKWN",0,0);
      0088CF AE 8B C4         [ 2] 1941 	ldw	x, #___str_9+0
      0088D2 4B 00            [ 1] 1942 	push	#0x00
      0088D4 4B 00            [ 1] 1943 	push	#0x00
      0088D6 89               [ 2] 1944 	pushw	x
      0088D7 CD 85 53         [ 4] 1945 	call	_LCD_drawtext
      0088DA 5B 04            [ 2] 1946 	addw	sp, #4
      0088DC                       1947 00112$:
                                   1948 ;	main.c: 204: LCD_drawtext("mV",0,80);
      0088DC AE 8B CA         [ 2] 1949 	ldw	x, #___str_10+0
      0088DF 4B 50            [ 1] 1950 	push	#0x50
      0088E1 4B 00            [ 1] 1951 	push	#0x00
      0088E3 89               [ 2] 1952 	pushw	x
      0088E4 CD 85 53         [ 4] 1953 	call	_LCD_drawtext
      0088E7 5B 04            [ 2] 1954 	addw	sp, #4
                                   1955 ;	main.c: 205: vbat = vbat_mon();
      0088E9 CD 86 77         [ 4] 1956 	call	_vbat_mon
      0088EC CF 00 08         [ 2] 1957 	ldw	_vbat+0, x
                                   1958 ;	main.c: 206: LCD_drawint(vbat,0,48);
      0088EF 4B 30            [ 1] 1959 	push	#0x30
      0088F1 4B 00            [ 1] 1960 	push	#0x00
      0088F3 3B 00 09         [ 1] 1961 	push	_vbat+1
      0088F6 3B 00 08         [ 1] 1962 	push	_vbat+0
      0088F9 CD 85 7E         [ 4] 1963 	call	_LCD_drawint
      0088FC 5B 04            [ 2] 1964 	addw	sp, #4
      0088FE 81               [ 4] 1965 	ret
                                   1966 	.area CODE
      0088FF                       1967 _font_arr:
      0088FF 00                    1968 	.db #0x00	; 0
      008900 00                    1969 	.db #0x00	; 0
      008901 00                    1970 	.db #0x00	; 0
      008902 00                    1971 	.db #0x00	; 0
      008903 00                    1972 	.db #0x00	; 0
      008904 00                    1973 	.db #0x00	; 0
      008905 00                    1974 	.db #0x00	; 0
      008906 5F                    1975 	.db #0x5F	; 95
      008907 00                    1976 	.db #0x00	; 0
      008908 00                    1977 	.db #0x00	; 0
      008909 05                    1978 	.db #0x05	; 5
      00890A 03                    1979 	.db #0x03	; 3
      00890B 00                    1980 	.db #0x00	; 0
      00890C 05                    1981 	.db #0x05	; 5
      00890D 03                    1982 	.db #0x03	; 3
      00890E 14                    1983 	.db #0x14	; 20
      00890F 7F                    1984 	.db #0x7F	; 127
      008910 14                    1985 	.db #0x14	; 20
      008911 7F                    1986 	.db #0x7F	; 127
      008912 14                    1987 	.db #0x14	; 20
      008913 24                    1988 	.db #0x24	; 36
      008914 2A                    1989 	.db #0x2A	; 42
      008915 7F                    1990 	.db #0x7F	; 127
      008916 2A                    1991 	.db #0x2A	; 42
      008917 12                    1992 	.db #0x12	; 18
      008918 23                    1993 	.db #0x23	; 35
      008919 13                    1994 	.db #0x13	; 19
      00891A 08                    1995 	.db #0x08	; 8
      00891B 64                    1996 	.db #0x64	; 100	'd'
      00891C 62                    1997 	.db #0x62	; 98	'b'
      00891D 36                    1998 	.db #0x36	; 54	'6'
      00891E 49                    1999 	.db #0x49	; 73	'I'
      00891F 55                    2000 	.db #0x55	; 85	'U'
      008920 22                    2001 	.db #0x22	; 34
      008921 50                    2002 	.db #0x50	; 80	'P'
      008922 00                    2003 	.db #0x00	; 0
      008923 05                    2004 	.db #0x05	; 5
      008924 03                    2005 	.db #0x03	; 3
      008925 00                    2006 	.db #0x00	; 0
      008926 00                    2007 	.db #0x00	; 0
      008927 00                    2008 	.db #0x00	; 0
      008928 1C                    2009 	.db #0x1C	; 28
      008929 22                    2010 	.db #0x22	; 34
      00892A 41                    2011 	.db #0x41	; 65	'A'
      00892B 00                    2012 	.db #0x00	; 0
      00892C 00                    2013 	.db #0x00	; 0
      00892D 41                    2014 	.db #0x41	; 65	'A'
      00892E 22                    2015 	.db #0x22	; 34
      00892F 1C                    2016 	.db #0x1C	; 28
      008930 00                    2017 	.db #0x00	; 0
      008931 0A                    2018 	.db #0x0A	; 10
      008932 04                    2019 	.db #0x04	; 4
      008933 1F                    2020 	.db #0x1F	; 31
      008934 04                    2021 	.db #0x04	; 4
      008935 0A                    2022 	.db #0x0A	; 10
      008936 08                    2023 	.db #0x08	; 8
      008937 08                    2024 	.db #0x08	; 8
      008938 3E                    2025 	.db #0x3E	; 62
      008939 08                    2026 	.db #0x08	; 8
      00893A 08                    2027 	.db #0x08	; 8
      00893B 00                    2028 	.db #0x00	; 0
      00893C 50                    2029 	.db #0x50	; 80	'P'
      00893D 30                    2030 	.db #0x30	; 48	'0'
      00893E 00                    2031 	.db #0x00	; 0
      00893F 00                    2032 	.db #0x00	; 0
      008940 08                    2033 	.db #0x08	; 8
      008941 08                    2034 	.db #0x08	; 8
      008942 08                    2035 	.db #0x08	; 8
      008943 08                    2036 	.db #0x08	; 8
      008944 08                    2037 	.db #0x08	; 8
      008945 00                    2038 	.db #0x00	; 0
      008946 60                    2039 	.db #0x60	; 96
      008947 60                    2040 	.db #0x60	; 96
      008948 00                    2041 	.db #0x00	; 0
      008949 00                    2042 	.db #0x00	; 0
      00894A 20                    2043 	.db #0x20	; 32
      00894B 10                    2044 	.db #0x10	; 16
      00894C 08                    2045 	.db #0x08	; 8
      00894D 04                    2046 	.db #0x04	; 4
      00894E 02                    2047 	.db #0x02	; 2
      00894F 3E                    2048 	.db #0x3E	; 62
      008950 51                    2049 	.db #0x51	; 81	'Q'
      008951 49                    2050 	.db #0x49	; 73	'I'
      008952 45                    2051 	.db #0x45	; 69	'E'
      008953 3E                    2052 	.db #0x3E	; 62
      008954 00                    2053 	.db #0x00	; 0
      008955 42                    2054 	.db #0x42	; 66	'B'
      008956 7F                    2055 	.db #0x7F	; 127
      008957 40                    2056 	.db #0x40	; 64
      008958 00                    2057 	.db #0x00	; 0
      008959 42                    2058 	.db #0x42	; 66	'B'
      00895A 61                    2059 	.db #0x61	; 97	'a'
      00895B 51                    2060 	.db #0x51	; 81	'Q'
      00895C 49                    2061 	.db #0x49	; 73	'I'
      00895D 46                    2062 	.db #0x46	; 70	'F'
      00895E 22                    2063 	.db #0x22	; 34
      00895F 41                    2064 	.db #0x41	; 65	'A'
      008960 49                    2065 	.db #0x49	; 73	'I'
      008961 49                    2066 	.db #0x49	; 73	'I'
      008962 36                    2067 	.db #0x36	; 54	'6'
      008963 18                    2068 	.db #0x18	; 24
      008964 14                    2069 	.db #0x14	; 20
      008965 12                    2070 	.db #0x12	; 18
      008966 7F                    2071 	.db #0x7F	; 127
      008967 10                    2072 	.db #0x10	; 16
      008968 27                    2073 	.db #0x27	; 39
      008969 45                    2074 	.db #0x45	; 69	'E'
      00896A 45                    2075 	.db #0x45	; 69	'E'
      00896B 45                    2076 	.db #0x45	; 69	'E'
      00896C 39                    2077 	.db #0x39	; 57	'9'
      00896D 3E                    2078 	.db #0x3E	; 62
      00896E 49                    2079 	.db #0x49	; 73	'I'
      00896F 49                    2080 	.db #0x49	; 73	'I'
      008970 49                    2081 	.db #0x49	; 73	'I'
      008971 32                    2082 	.db #0x32	; 50	'2'
      008972 61                    2083 	.db #0x61	; 97	'a'
      008973 11                    2084 	.db #0x11	; 17
      008974 09                    2085 	.db #0x09	; 9
      008975 05                    2086 	.db #0x05	; 5
      008976 03                    2087 	.db #0x03	; 3
      008977 36                    2088 	.db #0x36	; 54	'6'
      008978 49                    2089 	.db #0x49	; 73	'I'
      008979 49                    2090 	.db #0x49	; 73	'I'
      00897A 49                    2091 	.db #0x49	; 73	'I'
      00897B 36                    2092 	.db #0x36	; 54	'6'
      00897C 26                    2093 	.db #0x26	; 38
      00897D 49                    2094 	.db #0x49	; 73	'I'
      00897E 49                    2095 	.db #0x49	; 73	'I'
      00897F 49                    2096 	.db #0x49	; 73	'I'
      008980 3E                    2097 	.db #0x3E	; 62
      008981 00                    2098 	.db #0x00	; 0
      008982 36                    2099 	.db #0x36	; 54	'6'
      008983 36                    2100 	.db #0x36	; 54	'6'
      008984 00                    2101 	.db #0x00	; 0
      008985 00                    2102 	.db #0x00	; 0
      008986 00                    2103 	.db #0x00	; 0
      008987 56                    2104 	.db #0x56	; 86	'V'
      008988 36                    2105 	.db #0x36	; 54	'6'
      008989 00                    2106 	.db #0x00	; 0
      00898A 00                    2107 	.db #0x00	; 0
      00898B 00                    2108 	.db #0x00	; 0
      00898C 08                    2109 	.db #0x08	; 8
      00898D 14                    2110 	.db #0x14	; 20
      00898E 22                    2111 	.db #0x22	; 34
      00898F 00                    2112 	.db #0x00	; 0
      008990 14                    2113 	.db #0x14	; 20
      008991 14                    2114 	.db #0x14	; 20
      008992 14                    2115 	.db #0x14	; 20
      008993 14                    2116 	.db #0x14	; 20
      008994 14                    2117 	.db #0x14	; 20
      008995 00                    2118 	.db #0x00	; 0
      008996 22                    2119 	.db #0x22	; 34
      008997 14                    2120 	.db #0x14	; 20
      008998 08                    2121 	.db #0x08	; 8
      008999 00                    2122 	.db #0x00	; 0
      00899A 02                    2123 	.db #0x02	; 2
      00899B 01                    2124 	.db #0x01	; 1
      00899C 51                    2125 	.db #0x51	; 81	'Q'
      00899D 09                    2126 	.db #0x09	; 9
      00899E 06                    2127 	.db #0x06	; 6
      00899F 32                    2128 	.db #0x32	; 50	'2'
      0089A0 49                    2129 	.db #0x49	; 73	'I'
      0089A1 79                    2130 	.db #0x79	; 121	'y'
      0089A2 41                    2131 	.db #0x41	; 65	'A'
      0089A3 3E                    2132 	.db #0x3E	; 62
      0089A4 7C                    2133 	.db #0x7C	; 124
      0089A5 12                    2134 	.db #0x12	; 18
      0089A6 11                    2135 	.db #0x11	; 17
      0089A7 12                    2136 	.db #0x12	; 18
      0089A8 7C                    2137 	.db #0x7C	; 124
      0089A9 7F                    2138 	.db #0x7F	; 127
      0089AA 49                    2139 	.db #0x49	; 73	'I'
      0089AB 49                    2140 	.db #0x49	; 73	'I'
      0089AC 49                    2141 	.db #0x49	; 73	'I'
      0089AD 36                    2142 	.db #0x36	; 54	'6'
      0089AE 3E                    2143 	.db #0x3E	; 62
      0089AF 41                    2144 	.db #0x41	; 65	'A'
      0089B0 41                    2145 	.db #0x41	; 65	'A'
      0089B1 41                    2146 	.db #0x41	; 65	'A'
      0089B2 22                    2147 	.db #0x22	; 34
      0089B3 7F                    2148 	.db #0x7F	; 127
      0089B4 41                    2149 	.db #0x41	; 65	'A'
      0089B5 41                    2150 	.db #0x41	; 65	'A'
      0089B6 22                    2151 	.db #0x22	; 34
      0089B7 1C                    2152 	.db #0x1C	; 28
      0089B8 7F                    2153 	.db #0x7F	; 127
      0089B9 49                    2154 	.db #0x49	; 73	'I'
      0089BA 49                    2155 	.db #0x49	; 73	'I'
      0089BB 49                    2156 	.db #0x49	; 73	'I'
      0089BC 49                    2157 	.db #0x49	; 73	'I'
      0089BD 7F                    2158 	.db #0x7F	; 127
      0089BE 09                    2159 	.db #0x09	; 9
      0089BF 09                    2160 	.db #0x09	; 9
      0089C0 09                    2161 	.db #0x09	; 9
      0089C1 09                    2162 	.db #0x09	; 9
      0089C2 3E                    2163 	.db #0x3E	; 62
      0089C3 41                    2164 	.db #0x41	; 65	'A'
      0089C4 49                    2165 	.db #0x49	; 73	'I'
      0089C5 49                    2166 	.db #0x49	; 73	'I'
      0089C6 3A                    2167 	.db #0x3A	; 58
      0089C7 7F                    2168 	.db #0x7F	; 127
      0089C8 08                    2169 	.db #0x08	; 8
      0089C9 08                    2170 	.db #0x08	; 8
      0089CA 08                    2171 	.db #0x08	; 8
      0089CB 7F                    2172 	.db #0x7F	; 127
      0089CC 00                    2173 	.db #0x00	; 0
      0089CD 41                    2174 	.db #0x41	; 65	'A'
      0089CE 7F                    2175 	.db #0x7F	; 127
      0089CF 41                    2176 	.db #0x41	; 65	'A'
      0089D0 00                    2177 	.db #0x00	; 0
      0089D1 20                    2178 	.db #0x20	; 32
      0089D2 40                    2179 	.db #0x40	; 64
      0089D3 41                    2180 	.db #0x41	; 65	'A'
      0089D4 3F                    2181 	.db #0x3F	; 63
      0089D5 01                    2182 	.db #0x01	; 1
      0089D6 7F                    2183 	.db #0x7F	; 127
      0089D7 08                    2184 	.db #0x08	; 8
      0089D8 14                    2185 	.db #0x14	; 20
      0089D9 22                    2186 	.db #0x22	; 34
      0089DA 41                    2187 	.db #0x41	; 65	'A'
      0089DB 7F                    2188 	.db #0x7F	; 127
      0089DC 40                    2189 	.db #0x40	; 64
      0089DD 40                    2190 	.db #0x40	; 64
      0089DE 40                    2191 	.db #0x40	; 64
      0089DF 40                    2192 	.db #0x40	; 64
      0089E0 7F                    2193 	.db #0x7F	; 127
      0089E1 02                    2194 	.db #0x02	; 2
      0089E2 0C                    2195 	.db #0x0C	; 12
      0089E3 02                    2196 	.db #0x02	; 2
      0089E4 7F                    2197 	.db #0x7F	; 127
      0089E5 7F                    2198 	.db #0x7F	; 127
      0089E6 04                    2199 	.db #0x04	; 4
      0089E7 08                    2200 	.db #0x08	; 8
      0089E8 10                    2201 	.db #0x10	; 16
      0089E9 7F                    2202 	.db #0x7F	; 127
      0089EA 3E                    2203 	.db #0x3E	; 62
      0089EB 41                    2204 	.db #0x41	; 65	'A'
      0089EC 41                    2205 	.db #0x41	; 65	'A'
      0089ED 41                    2206 	.db #0x41	; 65	'A'
      0089EE 3E                    2207 	.db #0x3E	; 62
      0089EF 7F                    2208 	.db #0x7F	; 127
      0089F0 09                    2209 	.db #0x09	; 9
      0089F1 09                    2210 	.db #0x09	; 9
      0089F2 09                    2211 	.db #0x09	; 9
      0089F3 06                    2212 	.db #0x06	; 6
      0089F4 3E                    2213 	.db #0x3E	; 62
      0089F5 41                    2214 	.db #0x41	; 65	'A'
      0089F6 51                    2215 	.db #0x51	; 81	'Q'
      0089F7 21                    2216 	.db #0x21	; 33
      0089F8 5E                    2217 	.db #0x5E	; 94
      0089F9 7F                    2218 	.db #0x7F	; 127
      0089FA 09                    2219 	.db #0x09	; 9
      0089FB 19                    2220 	.db #0x19	; 25
      0089FC 29                    2221 	.db #0x29	; 41
      0089FD 46                    2222 	.db #0x46	; 70	'F'
      0089FE 26                    2223 	.db #0x26	; 38
      0089FF 49                    2224 	.db #0x49	; 73	'I'
      008A00 49                    2225 	.db #0x49	; 73	'I'
      008A01 49                    2226 	.db #0x49	; 73	'I'
      008A02 32                    2227 	.db #0x32	; 50	'2'
      008A03 01                    2228 	.db #0x01	; 1
      008A04 01                    2229 	.db #0x01	; 1
      008A05 7F                    2230 	.db #0x7F	; 127
      008A06 01                    2231 	.db #0x01	; 1
      008A07 01                    2232 	.db #0x01	; 1
      008A08 3F                    2233 	.db #0x3F	; 63
      008A09 40                    2234 	.db #0x40	; 64
      008A0A 40                    2235 	.db #0x40	; 64
      008A0B 40                    2236 	.db #0x40	; 64
      008A0C 3F                    2237 	.db #0x3F	; 63
      008A0D 1F                    2238 	.db #0x1F	; 31
      008A0E 20                    2239 	.db #0x20	; 32
      008A0F 40                    2240 	.db #0x40	; 64
      008A10 20                    2241 	.db #0x20	; 32
      008A11 1F                    2242 	.db #0x1F	; 31
      008A12 3F                    2243 	.db #0x3F	; 63
      008A13 40                    2244 	.db #0x40	; 64
      008A14 38                    2245 	.db #0x38	; 56	'8'
      008A15 40                    2246 	.db #0x40	; 64
      008A16 3F                    2247 	.db #0x3F	; 63
      008A17 63                    2248 	.db #0x63	; 99	'c'
      008A18 14                    2249 	.db #0x14	; 20
      008A19 08                    2250 	.db #0x08	; 8
      008A1A 14                    2251 	.db #0x14	; 20
      008A1B 63                    2252 	.db #0x63	; 99	'c'
      008A1C 07                    2253 	.db #0x07	; 7
      008A1D 08                    2254 	.db #0x08	; 8
      008A1E 70                    2255 	.db #0x70	; 112	'p'
      008A1F 08                    2256 	.db #0x08	; 8
      008A20 07                    2257 	.db #0x07	; 7
      008A21 61                    2258 	.db #0x61	; 97	'a'
      008A22 51                    2259 	.db #0x51	; 81	'Q'
      008A23 49                    2260 	.db #0x49	; 73	'I'
      008A24 45                    2261 	.db #0x45	; 69	'E'
      008A25 43                    2262 	.db #0x43	; 67	'C'
      008A26 00                    2263 	.db #0x00	; 0
      008A27 7F                    2264 	.db #0x7F	; 127
      008A28 41                    2265 	.db #0x41	; 65	'A'
      008A29 41                    2266 	.db #0x41	; 65	'A'
      008A2A 00                    2267 	.db #0x00	; 0
      008A2B 02                    2268 	.db #0x02	; 2
      008A2C 04                    2269 	.db #0x04	; 4
      008A2D 08                    2270 	.db #0x08	; 8
      008A2E 10                    2271 	.db #0x10	; 16
      008A2F 20                    2272 	.db #0x20	; 32
      008A30 00                    2273 	.db #0x00	; 0
      008A31 41                    2274 	.db #0x41	; 65	'A'
      008A32 41                    2275 	.db #0x41	; 65	'A'
      008A33 7F                    2276 	.db #0x7F	; 127
      008A34 00                    2277 	.db #0x00	; 0
      008A35 04                    2278 	.db #0x04	; 4
      008A36 02                    2279 	.db #0x02	; 2
      008A37 01                    2280 	.db #0x01	; 1
      008A38 02                    2281 	.db #0x02	; 2
      008A39 04                    2282 	.db #0x04	; 4
      008A3A 40                    2283 	.db #0x40	; 64
      008A3B 40                    2284 	.db #0x40	; 64
      008A3C 40                    2285 	.db #0x40	; 64
      008A3D 40                    2286 	.db #0x40	; 64
      008A3E 40                    2287 	.db #0x40	; 64
      008A3F 00                    2288 	.db #0x00	; 0
      008A40 01                    2289 	.db #0x01	; 1
      008A41 02                    2290 	.db #0x02	; 2
      008A42 04                    2291 	.db #0x04	; 4
      008A43 00                    2292 	.db #0x00	; 0
      008A44 20                    2293 	.db #0x20	; 32
      008A45 54                    2294 	.db #0x54	; 84	'T'
      008A46 54                    2295 	.db #0x54	; 84	'T'
      008A47 54                    2296 	.db #0x54	; 84	'T'
      008A48 78                    2297 	.db #0x78	; 120	'x'
      008A49 7F                    2298 	.db #0x7F	; 127
      008A4A 50                    2299 	.db #0x50	; 80	'P'
      008A4B 48                    2300 	.db #0x48	; 72	'H'
      008A4C 48                    2301 	.db #0x48	; 72	'H'
      008A4D 30                    2302 	.db #0x30	; 48	'0'
      008A4E 38                    2303 	.db #0x38	; 56	'8'
      008A4F 44                    2304 	.db #0x44	; 68	'D'
      008A50 44                    2305 	.db #0x44	; 68	'D'
      008A51 44                    2306 	.db #0x44	; 68	'D'
      008A52 28                    2307 	.db #0x28	; 40
      008A53 30                    2308 	.db #0x30	; 48	'0'
      008A54 48                    2309 	.db #0x48	; 72	'H'
      008A55 48                    2310 	.db #0x48	; 72	'H'
      008A56 50                    2311 	.db #0x50	; 80	'P'
      008A57 7F                    2312 	.db #0x7F	; 127
      008A58 38                    2313 	.db #0x38	; 56	'8'
      008A59 54                    2314 	.db #0x54	; 84	'T'
      008A5A 54                    2315 	.db #0x54	; 84	'T'
      008A5B 54                    2316 	.db #0x54	; 84	'T'
      008A5C 18                    2317 	.db #0x18	; 24
      008A5D 08                    2318 	.db #0x08	; 8
      008A5E 7E                    2319 	.db #0x7E	; 126
      008A5F 09                    2320 	.db #0x09	; 9
      008A60 09                    2321 	.db #0x09	; 9
      008A61 02                    2322 	.db #0x02	; 2
      008A62 08                    2323 	.db #0x08	; 8
      008A63 54                    2324 	.db #0x54	; 84	'T'
      008A64 54                    2325 	.db #0x54	; 84	'T'
      008A65 54                    2326 	.db #0x54	; 84	'T'
      008A66 3C                    2327 	.db #0x3C	; 60
      008A67 7F                    2328 	.db #0x7F	; 127
      008A68 10                    2329 	.db #0x10	; 16
      008A69 08                    2330 	.db #0x08	; 8
      008A6A 08                    2331 	.db #0x08	; 8
      008A6B 70                    2332 	.db #0x70	; 112	'p'
      008A6C 00                    2333 	.db #0x00	; 0
      008A6D 48                    2334 	.db #0x48	; 72	'H'
      008A6E 7A                    2335 	.db #0x7A	; 122	'z'
      008A6F 40                    2336 	.db #0x40	; 64
      008A70 00                    2337 	.db #0x00	; 0
      008A71 20                    2338 	.db #0x20	; 32
      008A72 40                    2339 	.db #0x40	; 64
      008A73 48                    2340 	.db #0x48	; 72	'H'
      008A74 3A                    2341 	.db #0x3A	; 58
      008A75 00                    2342 	.db #0x00	; 0
      008A76 7F                    2343 	.db #0x7F	; 127
      008A77 10                    2344 	.db #0x10	; 16
      008A78 28                    2345 	.db #0x28	; 40
      008A79 44                    2346 	.db #0x44	; 68	'D'
      008A7A 00                    2347 	.db #0x00	; 0
      008A7B 00                    2348 	.db #0x00	; 0
      008A7C 41                    2349 	.db #0x41	; 65	'A'
      008A7D 7F                    2350 	.db #0x7F	; 127
      008A7E 40                    2351 	.db #0x40	; 64
      008A7F 00                    2352 	.db #0x00	; 0
      008A80 7C                    2353 	.db #0x7C	; 124
      008A81 04                    2354 	.db #0x04	; 4
      008A82 7C                    2355 	.db #0x7C	; 124
      008A83 04                    2356 	.db #0x04	; 4
      008A84 78                    2357 	.db #0x78	; 120	'x'
      008A85 7C                    2358 	.db #0x7C	; 124
      008A86 08                    2359 	.db #0x08	; 8
      008A87 04                    2360 	.db #0x04	; 4
      008A88 04                    2361 	.db #0x04	; 4
      008A89 78                    2362 	.db #0x78	; 120	'x'
      008A8A 38                    2363 	.db #0x38	; 56	'8'
      008A8B 44                    2364 	.db #0x44	; 68	'D'
      008A8C 44                    2365 	.db #0x44	; 68	'D'
      008A8D 44                    2366 	.db #0x44	; 68	'D'
      008A8E 38                    2367 	.db #0x38	; 56	'8'
      008A8F 7C                    2368 	.db #0x7C	; 124
      008A90 14                    2369 	.db #0x14	; 20
      008A91 14                    2370 	.db #0x14	; 20
      008A92 14                    2371 	.db #0x14	; 20
      008A93 08                    2372 	.db #0x08	; 8
      008A94 08                    2373 	.db #0x08	; 8
      008A95 14                    2374 	.db #0x14	; 20
      008A96 14                    2375 	.db #0x14	; 20
      008A97 18                    2376 	.db #0x18	; 24
      008A98 7C                    2377 	.db #0x7C	; 124
      008A99 7C                    2378 	.db #0x7C	; 124
      008A9A 08                    2379 	.db #0x08	; 8
      008A9B 04                    2380 	.db #0x04	; 4
      008A9C 04                    2381 	.db #0x04	; 4
      008A9D 08                    2382 	.db #0x08	; 8
      008A9E 48                    2383 	.db #0x48	; 72	'H'
      008A9F 54                    2384 	.db #0x54	; 84	'T'
      008AA0 54                    2385 	.db #0x54	; 84	'T'
      008AA1 54                    2386 	.db #0x54	; 84	'T'
      008AA2 20                    2387 	.db #0x20	; 32
      008AA3 04                    2388 	.db #0x04	; 4
      008AA4 3F                    2389 	.db #0x3F	; 63
      008AA5 44                    2390 	.db #0x44	; 68	'D'
      008AA6 44                    2391 	.db #0x44	; 68	'D'
      008AA7 20                    2392 	.db #0x20	; 32
      008AA8 3C                    2393 	.db #0x3C	; 60
      008AA9 40                    2394 	.db #0x40	; 64
      008AAA 40                    2395 	.db #0x40	; 64
      008AAB 20                    2396 	.db #0x20	; 32
      008AAC 7C                    2397 	.db #0x7C	; 124
      008AAD 1C                    2398 	.db #0x1C	; 28
      008AAE 20                    2399 	.db #0x20	; 32
      008AAF 40                    2400 	.db #0x40	; 64
      008AB0 20                    2401 	.db #0x20	; 32
      008AB1 1C                    2402 	.db #0x1C	; 28
      008AB2 3C                    2403 	.db #0x3C	; 60
      008AB3 40                    2404 	.db #0x40	; 64
      008AB4 38                    2405 	.db #0x38	; 56	'8'
      008AB5 40                    2406 	.db #0x40	; 64
      008AB6 3C                    2407 	.db #0x3C	; 60
      008AB7 44                    2408 	.db #0x44	; 68	'D'
      008AB8 28                    2409 	.db #0x28	; 40
      008AB9 10                    2410 	.db #0x10	; 16
      008ABA 28                    2411 	.db #0x28	; 40
      008ABB 44                    2412 	.db #0x44	; 68	'D'
      008ABC 0C                    2413 	.db #0x0C	; 12
      008ABD 50                    2414 	.db #0x50	; 80	'P'
      008ABE 50                    2415 	.db #0x50	; 80	'P'
      008ABF 50                    2416 	.db #0x50	; 80	'P'
      008AC0 3C                    2417 	.db #0x3C	; 60
      008AC1 44                    2418 	.db #0x44	; 68	'D'
      008AC2 64                    2419 	.db #0x64	; 100	'd'
      008AC3 54                    2420 	.db #0x54	; 84	'T'
      008AC4 4C                    2421 	.db #0x4C	; 76	'L'
      008AC5 44                    2422 	.db #0x44	; 68	'D'
      008AC6 00                    2423 	.db #0x00	; 0
      008AC7 08                    2424 	.db #0x08	; 8
      008AC8 36                    2425 	.db #0x36	; 54	'6'
      008AC9 41                    2426 	.db #0x41	; 65	'A'
      008ACA 00                    2427 	.db #0x00	; 0
      008ACB 00                    2428 	.db #0x00	; 0
      008ACC 00                    2429 	.db #0x00	; 0
      008ACD 7F                    2430 	.db #0x7F	; 127
      008ACE 00                    2431 	.db #0x00	; 0
      008ACF 00                    2432 	.db #0x00	; 0
      008AD0 00                    2433 	.db #0x00	; 0
      008AD1 41                    2434 	.db #0x41	; 65	'A'
      008AD2 36                    2435 	.db #0x36	; 54	'6'
      008AD3 08                    2436 	.db #0x08	; 8
      008AD4 00                    2437 	.db #0x00	; 0
      008AD5 10                    2438 	.db #0x10	; 16
      008AD6 08                    2439 	.db #0x08	; 8
      008AD7 08                    2440 	.db #0x08	; 8
      008AD8 10                    2441 	.db #0x10	; 16
      008AD9 08                    2442 	.db #0x08	; 8
      008ADA 06                    2443 	.db #0x06	; 6
      008ADB 09                    2444 	.db #0x09	; 9
      008ADC 09                    2445 	.db #0x09	; 9
      008ADD 06                    2446 	.db #0x06	; 6
      008ADE 00                    2447 	.db #0x00	; 0
      008ADF 00                    2448 	.db #0x00	; 0
      008AE0 00                    2449 	.db #0x00	; 0
      008AE1 00                    2450 	.db #0x00	; 0
      008AE2 F8                    2451 	.db #0xF8	; 248
      008AE3 F8                    2452 	.db #0xF8	; 248
      008AE4 18                    2453 	.db #0x18	; 24
      008AE5 18                    2454 	.db #0x18	; 24
      008AE6 18                    2455 	.db #0x18	; 24
      008AE7 18                    2456 	.db #0x18	; 24
      008AE8 18                    2457 	.db #0x18	; 24
      008AE9 18                    2458 	.db #0x18	; 24
      008AEA F8                    2459 	.db #0xF8	; 248
      008AEB F8                    2460 	.db #0xF8	; 248
      008AEC 18                    2461 	.db #0x18	; 24
      008AED 18                    2462 	.db #0x18	; 24
      008AEE 18                    2463 	.db #0x18	; 24
      008AEF 18                    2464 	.db #0x18	; 24
      008AF0 18                    2465 	.db #0x18	; 24
      008AF1 18                    2466 	.db #0x18	; 24
      008AF2 F8                    2467 	.db #0xF8	; 248
      008AF3 F8                    2468 	.db #0xF8	; 248
      008AF4 00                    2469 	.db #0x00	; 0
      008AF5 00                    2470 	.db #0x00	; 0
      008AF6 00                    2471 	.db #0x00	; 0
      008AF7 00                    2472 	.db #0x00	; 0
      008AF8 00                    2473 	.db #0x00	; 0
      008AF9 00                    2474 	.db #0x00	; 0
      008AFA FF                    2475 	.db #0xFF	; 255
      008AFB FF                    2476 	.db #0xFF	; 255
      008AFC 18                    2477 	.db #0x18	; 24
      008AFD 18                    2478 	.db #0x18	; 24
      008AFE 18                    2479 	.db #0x18	; 24
      008AFF 18                    2480 	.db #0x18	; 24
      008B00 18                    2481 	.db #0x18	; 24
      008B01 18                    2482 	.db #0x18	; 24
      008B02 FF                    2483 	.db #0xFF	; 255
      008B03 FF                    2484 	.db #0xFF	; 255
      008B04 18                    2485 	.db #0x18	; 24
      008B05 18                    2486 	.db #0x18	; 24
      008B06 18                    2487 	.db #0x18	; 24
      008B07 18                    2488 	.db #0x18	; 24
      008B08 18                    2489 	.db #0x18	; 24
      008B09 18                    2490 	.db #0x18	; 24
      008B0A FF                    2491 	.db #0xFF	; 255
      008B0B FF                    2492 	.db #0xFF	; 255
      008B0C 00                    2493 	.db #0x00	; 0
      008B0D 00                    2494 	.db #0x00	; 0
      008B0E 00                    2495 	.db #0x00	; 0
      008B0F 00                    2496 	.db #0x00	; 0
      008B10 00                    2497 	.db #0x00	; 0
      008B11 00                    2498 	.db #0x00	; 0
      008B12 1F                    2499 	.db #0x1F	; 31
      008B13 1F                    2500 	.db #0x1F	; 31
      008B14 18                    2501 	.db #0x18	; 24
      008B15 18                    2502 	.db #0x18	; 24
      008B16 18                    2503 	.db #0x18	; 24
      008B17 18                    2504 	.db #0x18	; 24
      008B18 18                    2505 	.db #0x18	; 24
      008B19 18                    2506 	.db #0x18	; 24
      008B1A 1F                    2507 	.db #0x1F	; 31
      008B1B 1F                    2508 	.db #0x1F	; 31
      008B1C 18                    2509 	.db #0x18	; 24
      008B1D 18                    2510 	.db #0x18	; 24
      008B1E 18                    2511 	.db #0x18	; 24
      008B1F 18                    2512 	.db #0x18	; 24
      008B20 18                    2513 	.db #0x18	; 24
      008B21 18                    2514 	.db #0x18	; 24
      008B22 1F                    2515 	.db #0x1F	; 31
      008B23 1F                    2516 	.db #0x1F	; 31
      008B24 00                    2517 	.db #0x00	; 0
      008B25 00                    2518 	.db #0x00	; 0
      008B26 00                    2519 	.db #0x00	; 0
      008B27 18                    2520 	.db #0x18	; 24
      008B28 18                    2521 	.db #0x18	; 24
      008B29 18                    2522 	.db #0x18	; 24
      008B2A 18                    2523 	.db #0x18	; 24
      008B2B 18                    2524 	.db #0x18	; 24
      008B2C 18                    2525 	.db #0x18	; 24
      008B2D 18                    2526 	.db #0x18	; 24
      008B2E 18                    2527 	.db #0x18	; 24
      008B2F 00                    2528 	.db #0x00	; 0
      008B30 00                    2529 	.db #0x00	; 0
      008B31 00                    2530 	.db #0x00	; 0
      008B32 FF                    2531 	.db #0xFF	; 255
      008B33 FF                    2532 	.db #0xFF	; 255
      008B34 00                    2533 	.db #0x00	; 0
      008B35 00                    2534 	.db #0x00	; 0
      008B36 00                    2535 	.db #0x00	; 0
      008B37 18                    2536 	.db #0x18	; 24
      008B38 0C                    2537 	.db #0x0C	; 12
      008B39 06                    2538 	.db #0x06	; 6
      008B3A FF                    2539 	.db #0xFF	; 255
      008B3B FF                    2540 	.db #0xFF	; 255
      008B3C 06                    2541 	.db #0x06	; 6
      008B3D 0C                    2542 	.db #0x0C	; 12
      008B3E 18                    2543 	.db #0x18	; 24
      008B3F 18                    2544 	.db #0x18	; 24
      008B40 30                    2545 	.db #0x30	; 48	'0'
      008B41 60                    2546 	.db #0x60	; 96
      008B42 FF                    2547 	.db #0xFF	; 255
      008B43 FF                    2548 	.db #0xFF	; 255
      008B44 60                    2549 	.db #0x60	; 96
      008B45 30                    2550 	.db #0x30	; 48	'0'
      008B46 18                    2551 	.db #0x18	; 24
      008B47 18                    2552 	.db #0x18	; 24
      008B48 3C                    2553 	.db #0x3C	; 60
      008B49 7E                    2554 	.db #0x7E	; 126
      008B4A DB                    2555 	.db #0xDB	; 219
      008B4B 99                    2556 	.db #0x99	; 153
      008B4C 18                    2557 	.db #0x18	; 24
      008B4D 18                    2558 	.db #0x18	; 24
      008B4E 18                    2559 	.db #0x18	; 24
      008B4F 18                    2560 	.db #0x18	; 24
      008B50 18                    2561 	.db #0x18	; 24
      008B51 18                    2562 	.db #0x18	; 24
      008B52 99                    2563 	.db #0x99	; 153
      008B53 DB                    2564 	.db #0xDB	; 219
      008B54 7E                    2565 	.db #0x7E	; 126
      008B55 3C                    2566 	.db #0x3C	; 60
      008B56 18                    2567 	.db #0x18	; 24
      008B57 7F                    2568 	.db #0x7F	; 127
      008B58 7F                    2569 	.db #0x7F	; 127
      008B59 0F                    2570 	.db #0x0F	; 15
      008B5A 1F                    2571 	.db #0x1F	; 31
      008B5B 3B                    2572 	.db #0x3B	; 59
      008B5C 73                    2573 	.db #0x73	; 115	's'
      008B5D E3                    2574 	.db #0xE3	; 227
      008B5E 40                    2575 	.db #0x40	; 64
      008B5F 40                    2576 	.db #0x40	; 64
      008B60 E3                    2577 	.db #0xE3	; 227
      008B61 73                    2578 	.db #0x73	; 115	's'
      008B62 3B                    2579 	.db #0x3B	; 59
      008B63 1F                    2580 	.db #0x1F	; 31
      008B64 0F                    2581 	.db #0x0F	; 15
      008B65 7F                    2582 	.db #0x7F	; 127
      008B66 7F                    2583 	.db #0x7F	; 127
      008B67 FE                    2584 	.db #0xFE	; 254
      008B68 FE                    2585 	.db #0xFE	; 254
      008B69 F0                    2586 	.db #0xF0	; 240
      008B6A F8                    2587 	.db #0xF8	; 248
      008B6B DC                    2588 	.db #0xDC	; 220
      008B6C CE                    2589 	.db #0xCE	; 206
      008B6D C7                    2590 	.db #0xC7	; 199
      008B6E 02                    2591 	.db #0x02	; 2
      008B6F 02                    2592 	.db #0x02	; 2
      008B70 C7                    2593 	.db #0xC7	; 199
      008B71 CE                    2594 	.db #0xCE	; 206
      008B72 DC                    2595 	.db #0xDC	; 220
      008B73 F8                    2596 	.db #0xF8	; 248
      008B74 F0                    2597 	.db #0xF0	; 240
      008B75 FE                    2598 	.db #0xFE	; 254
      008B76 FE                    2599 	.db #0xFE	; 254
      008B77 3C                    2600 	.db #0x3C	; 60
      008B78 42                    2601 	.db #0x42	; 66	'B'
      008B79 81                    2602 	.db #0x81	; 129
      008B7A 99                    2603 	.db #0x99	; 153
      008B7B 99                    2604 	.db #0x99	; 153
      008B7C 81                    2605 	.db #0x81	; 129
      008B7D 42                    2606 	.db #0x42	; 66	'B'
      008B7E 3C                    2607 	.db #0x3C	; 60
      008B7F                       2608 ___str_0:
      008B7F 43 48 41 52 47 49 4E  2609 	.ascii "CHARGING"
             47
      008B87 00                    2610 	.db 0x00
      008B88                       2611 ___str_1:
      008B88 50 4F 57 45 52 20 4F  2612 	.ascii "POWER ON"
             4E
      008B90 00                    2613 	.db 0x00
      008B91                       2614 ___str_2:
      008B91 50 4F 57 52 20 4F 46  2615 	.ascii "POWR OFF"
             46
      008B99 00                    2616 	.db 0x00
      008B9A                       2617 ___str_3:
      008B9A 20 20 20 20 20 20 20  2618 	.ascii "        "
             20
      008BA2 00                    2619 	.db 0x00
      008BA3                       2620 ___str_4:
      008BA3 62 6F 6E 75 73 6F 69  2621 	.ascii "bonusoid"
             64
      008BAB 00                    2622 	.db 0x00
      008BAC                       2623 ___str_5:
      008BAC 42 43 48 52 47        2624 	.ascii "BCHRG"
      008BB1 00                    2625 	.db 0x00
      008BB2                       2626 ___str_6:
      008BB2 42 46 55 4C 4C        2627 	.ascii "BFULL"
      008BB7 00                    2628 	.db 0x00
      008BB8                       2629 ___str_7:
      008BB8 4E 4F 43 48 47        2630 	.ascii "NOCHG"
      008BBD 00                    2631 	.db 0x00
      008BBE                       2632 ___str_8:
      008BBE 4E 4F 42 41 54        2633 	.ascii "NOBAT"
      008BC3 00                    2634 	.db 0x00
      008BC4                       2635 ___str_9:
      008BC4 55 4E 4B 57 4E        2636 	.ascii "UNKWN"
      008BC9 00                    2637 	.db 0x00
      008BCA                       2638 ___str_10:
      008BCA 6D 56                 2639 	.ascii "mV"
      008BCC 00                    2640 	.db 0x00
                                   2641 	.area INITIALIZER
                                   2642 	.area CABS (ABS)
