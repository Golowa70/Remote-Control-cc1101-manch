
_fn_cc1101_strob:

;test_cc1101_TX.c,122 :: 		void fn_cc1101_strob(unsigned char  strob)
;test_cc1101_TX.c,124 :: 		Chip_Select = 0;
	BCF        RC5_bit+0, BitPos(RC5_bit+0)
;test_cc1101_TX.c,126 :: 		Soft_SPI_Write( strob);
	MOVF       FARG_fn_cc1101_strob_strob+0, 0
	MOVWF      FARG_Soft_SPI_Write_sdata+0
	CALL       _Soft_SPI_Write+0
;test_cc1101_TX.c,127 :: 		delay_ms(1);
	MOVLW      6
	MOVWF      R12
	MOVLW      48
	MOVWF      R13
L_fn_cc1101_strob0:
	DECFSZ     R13, 1
	GOTO       L_fn_cc1101_strob0
	DECFSZ     R12, 1
	GOTO       L_fn_cc1101_strob0
	NOP
;test_cc1101_TX.c,128 :: 		Chip_Select = 1;
	BSF        RC5_bit+0, BitPos(RC5_bit+0)
;test_cc1101_TX.c,130 :: 		}
L_end_fn_cc1101_strob:
	RETURN
; end of _fn_cc1101_strob

_fn_cc1101_init:

;test_cc1101_TX.c,135 :: 		void fn_cc1101_init()
;test_cc1101_TX.c,138 :: 		Chip_Select = 0;                // активируем порт, выбираем чип.
	BCF        RC5_bit+0, BitPos(RC5_bit+0)
;test_cc1101_TX.c,142 :: 		for(i=0;i!=58;i++)  // запись 34 регистров из таблицы
	CLRF       fn_cc1101_init_i_L0+0
L_fn_cc1101_init1:
	MOVF       fn_cc1101_init_i_L0+0, 0
	XORLW      58
	BTFSC      STATUS+0, 2
	GOTO       L_fn_cc1101_init2
;test_cc1101_TX.c,144 :: 		Soft_SPI_Write(rf_settings[i]);       //
	MOVLW      _rf_settings+0
	MOVWF      FSR0L
	MOVLW      hi_addr(_rf_settings+0)
	MOVWF      FSR0H
	MOVF       fn_cc1101_init_i_L0+0, 0
	ADDWF      FSR0L, 1
	BTFSC      STATUS+0, 0
	INCF       FSR0H, 1
	MOVF       INDF0+0, 0
	MOVWF      FARG_Soft_SPI_Write_sdata+0
	CALL       _Soft_SPI_Write+0
;test_cc1101_TX.c,142 :: 		for(i=0;i!=58;i++)  // запись 34 регистров из таблицы
	INCF       fn_cc1101_init_i_L0+0, 1
;test_cc1101_TX.c,146 :: 		}
	GOTO       L_fn_cc1101_init1
L_fn_cc1101_init2:
;test_cc1101_TX.c,150 :: 		Chip_Select = 1;          // отпускаем порт
	BSF        RC5_bit+0, BitPos(RC5_bit+0)
;test_cc1101_TX.c,155 :: 		delay_ms(5);
	MOVLW      26
	MOVWF      R12
	MOVLW      248
	MOVWF      R13
L_fn_cc1101_init4:
	DECFSZ     R13, 1
	GOTO       L_fn_cc1101_init4
	DECFSZ     R12, 1
	GOTO       L_fn_cc1101_init4
	NOP
;test_cc1101_TX.c,156 :: 		fn_cc1101_strob (SIDLE);
	MOVLW      54
	MOVWF      FARG_fn_cc1101_strob_strob+0
	CALL       _fn_cc1101_strob+0
;test_cc1101_TX.c,157 :: 		return;
;test_cc1101_TX.c,158 :: 		}
L_end_fn_cc1101_init:
	RETURN
; end of _fn_cc1101_init

_ManInit:

;test_cc1101_TX.c,206 :: 		void ManInit (void)
;test_cc1101_TX.c,208 :: 		TRANSMIT_TRIS &= ~TRANSMIT_LINE;                        //лини€ на вывод
	BCF        TRISC+0, 0
;test_cc1101_TX.c,209 :: 		}
L_end_ManInit:
	RETURN
; end of _ManInit

_ManBufAddByte:

;test_cc1101_TX.c,214 :: 		void ManBufAddByte (unsigned char place, unsigned char byte)
;test_cc1101_TX.c,216 :: 		if (place >= MAN_BUF_LENGTH)        return;
	MOVLW      16
	SUBWF      FARG_ManBufAddByte_place+0, 0
	BTFSS      STATUS+0, 0
	GOTO       L_ManBufAddByte5
	GOTO       L_end_ManBufAddByte
L_ManBufAddByte5:
;test_cc1101_TX.c,217 :: 		ManTransmitDataBuf [place] = byte;
	MOVLW      _ManTransmitDataBuf+0
	MOVWF      FSR1L
	MOVLW      hi_addr(_ManTransmitDataBuf+0)
	MOVWF      FSR1H
	MOVF       FARG_ManBufAddByte_place+0, 0
	ADDWF      FSR1L, 1
	BTFSC      STATUS+0, 0
	INCF       FSR1H, 1
	MOVF       FARG_ManBufAddByte_byte+0, 0
	MOVWF      INDF1+0
;test_cc1101_TX.c,218 :: 		}
L_end_ManBufAddByte:
	RETURN
; end of _ManBufAddByte

_ManTransmitData:

;test_cc1101_TX.c,223 :: 		void ManTransmitData (unsigned char BufLen){
;test_cc1101_TX.c,224 :: 		unsigned char  i=0;
	CLRF       ManTransmitData_i_L0+0
	CLRF       ManTransmitData_u_L0+0
	CLRF       ManTransmitData_a_L0+0
	CLRF       ManTransmitData_byte_L0+0
;test_cc1101_TX.c,230 :: 		INTCON.GIE =0;                                         //запрет всех прерываний
	BCF        INTCON+0, 7
;test_cc1101_TX.c,232 :: 		for ( i=0; i< MAN_PILOT_LEN; i++) {                     //передача пилотного сигнала
	CLRF       ManTransmitData_i_L0+0
L_ManTransmitData6:
	MOVLW      8
	SUBWF      ManTransmitData_i_L0+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_ManTransmitData7
;test_cc1101_TX.c,233 :: 		ManTransmitBit (1);
	MOVLW      1
	MOVWF      FARG_ManTransmitBit_bit_t+0
	CALL       _ManTransmitBit+0
;test_cc1101_TX.c,232 :: 		for ( i=0; i< MAN_PILOT_LEN; i++) {                     //передача пилотного сигнала
	INCF       ManTransmitData_i_L0+0, 1
;test_cc1101_TX.c,234 :: 		}
	GOTO       L_ManTransmitData6
L_ManTransmitData7:
;test_cc1101_TX.c,240 :: 		while (1)   {
L_ManTransmitData9:
;test_cc1101_TX.c,242 :: 		byte = ManIdentifier [a];
	MOVLW      _ManIdentifier+0
	MOVWF      FSR0L
	MOVLW      hi_addr(_ManIdentifier+0)
	MOVWF      FSR0H
	MOVF       ManTransmitData_a_L0+0, 0
	ADDWF      FSR0L, 1
	BTFSC      STATUS+0, 0
	INCF       FSR0H, 1
	MOVF       INDF0+0, 0
	MOVWF      R0
	MOVF       R0, 0
	MOVWF      ManTransmitData_byte_L0+0
;test_cc1101_TX.c,243 :: 		a++;
	INCF       ManTransmitData_a_L0+0, 1
;test_cc1101_TX.c,244 :: 		if (byte)ManTransmitByte (byte);
	MOVF       R0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_ManTransmitData11
	MOVF       ManTransmitData_byte_L0+0, 0
	MOVWF      FARG_ManTransmitByte_byte+0
	CALL       _ManTransmitByte+0
	GOTO       L_ManTransmitData12
L_ManTransmitData11:
;test_cc1101_TX.c,246 :: 		break;
	GOTO       L_ManTransmitData10
L_ManTransmitData12:
;test_cc1101_TX.c,247 :: 		}
	GOTO       L_ManTransmitData9
L_ManTransmitData10:
;test_cc1101_TX.c,251 :: 		CheckSummByte = 0;                                   //обнулить контрольку
	CLRF       _CheckSummByte+0
;test_cc1101_TX.c,252 :: 		ManTransmitByte (BufLen);
	MOVF       FARG_ManTransmitData_BufLen+0, 0
	MOVWF      FARG_ManTransmitByte_byte+0
	CALL       _ManTransmitByte+0
;test_cc1101_TX.c,255 :: 		for (  u=0; u<(BufLen); u++) {
	CLRF       ManTransmitData_u_L0+0
L_ManTransmitData13:
	MOVF       FARG_ManTransmitData_BufLen+0, 0
	SUBWF      ManTransmitData_u_L0+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_ManTransmitData14
;test_cc1101_TX.c,257 :: 		ManTransmitByte (ManTransmitDataBuf [u]);
	MOVLW      _ManTransmitDataBuf+0
	MOVWF      FSR0L
	MOVLW      hi_addr(_ManTransmitDataBuf+0)
	MOVWF      FSR0H
	MOVF       ManTransmitData_u_L0+0, 0
	ADDWF      FSR0L, 1
	BTFSC      STATUS+0, 0
	INCF       FSR0H, 1
	MOVF       INDF0+0, 0
	MOVWF      FARG_ManTransmitByte_byte+0
	CALL       _ManTransmitByte+0
;test_cc1101_TX.c,255 :: 		for (  u=0; u<(BufLen); u++) {
	INCF       ManTransmitData_u_L0+0, 1
;test_cc1101_TX.c,258 :: 		}
	GOTO       L_ManTransmitData13
L_ManTransmitData14:
;test_cc1101_TX.c,262 :: 		ManTransmitByte (CheckSummByte);
	MOVF       _CheckSummByte+0, 0
	MOVWF      FARG_ManTransmitByte_byte+0
	CALL       _ManTransmitByte+0
;test_cc1101_TX.c,265 :: 		LATC.B0 = 0;                                       //гасим порт
	BCF        LATC+0, 0
;test_cc1101_TX.c,267 :: 		INTCON.GIE =1;                                       //разрешение прерываний
	BSF        INTCON+0, 7
;test_cc1101_TX.c,269 :: 		}
L_end_ManTransmitData:
	RETURN
; end of _ManTransmitData

_ManTransmitByte:

;test_cc1101_TX.c,274 :: 		void ManTransmitByte (unsigned char byte)
;test_cc1101_TX.c,275 :: 		{       unsigned char i=0;
	CLRF       ManTransmitByte_i_L0+0
;test_cc1101_TX.c,276 :: 		ManCheckSumm (byte);
	MOVF       FARG_ManTransmitByte_byte+0, 0
	MOVWF      FARG_ManCheckSumm_data_t+0
	CALL       _ManCheckSumm+0
;test_cc1101_TX.c,278 :: 		for ( i=0; i<8; i++) {
	CLRF       ManTransmitByte_i_L0+0
L_ManTransmitByte16:
	MOVLW      8
	SUBWF      ManTransmitByte_i_L0+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_ManTransmitByte17
;test_cc1101_TX.c,280 :: 		if (byte & 0x80)        ManTransmitBit (1);
	BTFSS      FARG_ManTransmitByte_byte+0, 7
	GOTO       L_ManTransmitByte19
	MOVLW      1
	MOVWF      FARG_ManTransmitBit_bit_t+0
	CALL       _ManTransmitBit+0
	GOTO       L_ManTransmitByte20
L_ManTransmitByte19:
;test_cc1101_TX.c,281 :: 		else                        ManTransmitBit (0);
	CLRF       FARG_ManTransmitBit_bit_t+0
	CALL       _ManTransmitBit+0
L_ManTransmitByte20:
;test_cc1101_TX.c,282 :: 		byte <<= 1;
	LSLF       FARG_ManTransmitByte_byte+0, 1
;test_cc1101_TX.c,278 :: 		for ( i=0; i<8; i++) {
	INCF       ManTransmitByte_i_L0+0, 1
;test_cc1101_TX.c,283 :: 		}
	GOTO       L_ManTransmitByte16
L_ManTransmitByte17:
;test_cc1101_TX.c,284 :: 		}
L_end_ManTransmitByte:
	RETURN
; end of _ManTransmitByte

_ManTransmitBit:

;test_cc1101_TX.c,288 :: 		void ManTransmitBit (unsigned char bit_t)
;test_cc1101_TX.c,290 :: 		if (bit_t) {
	MOVF       FARG_ManTransmitBit_bit_t+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_ManTransmitBit21
;test_cc1101_TX.c,292 :: 		TRANSMIT_PORT &= ~(TRANSMIT_LINE);        ManPause ();
	BCF        PORTC+0, 0
	CALL       _ManPause+0
;test_cc1101_TX.c,293 :: 		TRANSMIT_PORT |= TRANSMIT_LINE;                ManPause ();                                                                                   ///
	BSF        PORTC+0, 0
	CALL       _ManPause+0
;test_cc1101_TX.c,294 :: 		}
	GOTO       L_ManTransmitBit22
L_ManTransmitBit21:
;test_cc1101_TX.c,297 :: 		TRANSMIT_PORT |= TRANSMIT_LINE;                ManPause ();
	BSF        PORTC+0, 0
	CALL       _ManPause+0
;test_cc1101_TX.c,298 :: 		TRANSMIT_PORT &= ~TRANSMIT_LINE;        ManPause ();
	BCF        PORTC+0, 0
	CALL       _ManPause+0
;test_cc1101_TX.c,299 :: 		}
L_ManTransmitBit22:
;test_cc1101_TX.c,300 :: 		}
L_end_ManTransmitBit:
	RETURN
; end of _ManTransmitBit

_ManPause:

;test_cc1101_TX.c,303 :: 		void ManPause (void)
;test_cc1101_TX.c,305 :: 		delay_us (500000 / MAN_SPEED);
	MOVLW      3
	MOVWF      R12
	MOVLW      151
	MOVWF      R13
L_ManPause23:
	DECFSZ     R13, 1
	GOTO       L_ManPause23
	DECFSZ     R12, 1
	GOTO       L_ManPause23
	NOP
	NOP
;test_cc1101_TX.c,306 :: 		}
L_end_ManPause:
	RETURN
; end of _ManPause

_ManCheckSumm:

;test_cc1101_TX.c,315 :: 		void ManCheckSumm(unsigned char data_t)
;test_cc1101_TX.c,316 :: 		{           unsigned char i=0;
	CLRF       ManCheckSumm_i_L0+0
;test_cc1101_TX.c,318 :: 		for ( i=0; i<8; i++) {
	CLRF       ManCheckSumm_i_L0+0
L_ManCheckSumm24:
	MOVLW      8
	SUBWF      ManCheckSumm_i_L0+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_ManCheckSumm25
;test_cc1101_TX.c,320 :: 		unsigned char temp = data_t;
	MOVF       FARG_ManCheckSumm_data_t+0, 0
	MOVWF      R2+0
;test_cc1101_TX.c,321 :: 		temp ^= CheckSummByte;
	MOVF       _CheckSummByte+0, 0
	XORWF      FARG_ManCheckSumm_data_t+0, 0
	MOVWF      R1
	MOVF       R1, 0
	MOVWF      R2+0
;test_cc1101_TX.c,323 :: 		if (temp & 0x01) {
	BTFSS      R1, 0
	GOTO       L_ManCheckSumm27
;test_cc1101_TX.c,325 :: 		CheckSummByte ^= 0x18;
	MOVLW      24
	XORWF      _CheckSummByte+0, 1
;test_cc1101_TX.c,326 :: 		temp = 0x80;
	MOVLW      128
	MOVWF      R2+0
;test_cc1101_TX.c,327 :: 		}
	GOTO       L_ManCheckSumm28
L_ManCheckSumm27:
;test_cc1101_TX.c,329 :: 		else        temp = 0;
	CLRF       R2+0
L_ManCheckSumm28:
;test_cc1101_TX.c,331 :: 		CheckSummByte >>= 1;
	LSRF       _CheckSummByte+0, 1
;test_cc1101_TX.c,332 :: 		CheckSummByte |= temp;
	MOVF       R2+0, 0
	IORWF       _CheckSummByte+0, 1
;test_cc1101_TX.c,333 :: 		data_t >>= 1;
	LSRF       FARG_ManCheckSumm_data_t+0, 1
;test_cc1101_TX.c,318 :: 		for ( i=0; i<8; i++) {
	INCF       ManCheckSumm_i_L0+0, 1
;test_cc1101_TX.c,334 :: 		}
	GOTO       L_ManCheckSumm24
L_ManCheckSumm25:
;test_cc1101_TX.c,335 :: 		}
L_end_ManCheckSumm:
	RETURN
; end of _ManCheckSumm

_interrupt:

;test_cc1101_TX.c,338 :: 		void interrupt (void){
;test_cc1101_TX.c,339 :: 		if(INTCON.IOCIE & INTCON.IOCIF){                      //если прерывание по изменению...просыпаемс€
	BTFSS      INTCON+0, 3
	GOTO       L__interrupt60
	BTFSS      INTCON+0, 0
	GOTO       L__interrupt60
	BSF        3, 0
	GOTO       L__interrupt61
L__interrupt60:
	BCF        3, 0
L__interrupt61:
	BTFSS      3, 0
	GOTO       L_interrupt29
;test_cc1101_TX.c,340 :: 		INTCON.IOCIF=0;                                    //сбросим флаги
	BCF        INTCON+0, 0
;test_cc1101_TX.c,341 :: 		IOCAF.IOCAF0=0;
	BCF        IOCAF+0, 0
;test_cc1101_TX.c,342 :: 		IOCAF.IOCAF1=0;
	BCF        IOCAF+0, 1
;test_cc1101_TX.c,343 :: 		IOCAF.IOCAF2=0;
	BCF        IOCAF+0, 2
;test_cc1101_TX.c,344 :: 		IOCAF.IOCAF4=0;
	BCF        IOCAF+0, 4
;test_cc1101_TX.c,345 :: 		IOCAF.IOCAF5=0;
	BCF        IOCAF+0, 5
;test_cc1101_TX.c,348 :: 		INTCON=0;                                       //глобальный запрет прерываний
	CLRF       INTCON+0
;test_cc1101_TX.c,349 :: 		IOCAN=0;                                        //запрет прерываний по изменению порта
	CLRF       IOCAN+0
;test_cc1101_TX.c,351 :: 		}
L_interrupt29:
;test_cc1101_TX.c,354 :: 		}
L_end_interrupt:
L__interrupt59:
	RETFIE     %s
; end of _interrupt

_main:

;test_cc1101_TX.c,358 :: 		void main (void) {
;test_cc1101_TX.c,359 :: 		OSCCON=0b11111111;                                       //тактова€ частота
	MOVLW      255
	MOVWF      OSCCON+0
;test_cc1101_TX.c,360 :: 		TRISA=0b11111111;
	MOVLW      255
	MOVWF      TRISA+0
;test_cc1101_TX.c,361 :: 		ANSELA=0;
	CLRF       ANSELA+0
;test_cc1101_TX.c,362 :: 		ANSELC=0;                                               //отключение ј÷ѕ
	CLRF       ANSELC+0
;test_cc1101_TX.c,363 :: 		PORTA=0b000000;
	CLRF       PORTA+0
;test_cc1101_TX.c,364 :: 		WPUA=0b111111;                                           //подт€гивающие резист.
	MOVLW      63
	MOVWF      WPUA+0
;test_cc1101_TX.c,365 :: 		TRISC=0b00000000;
	CLRF       TRISC+0
;test_cc1101_TX.c,366 :: 		PORTC=0b00000000;
	CLRF       PORTC+0
;test_cc1101_TX.c,367 :: 		OPTION_REG=0b00000000;
	CLRF       OPTION_REG+0
;test_cc1101_TX.c,368 :: 		INTCON=0b00000000;                                       //настройки прерываний
	CLRF       INTCON+0
;test_cc1101_TX.c,369 :: 		WDTCON=0b00010000;                                       //делитель собаки ???
	MOVLW      16
	MOVWF      WDTCON+0
;test_cc1101_TX.c,370 :: 		IOCAN=0b00000000;                                        //прерывание по изменению порта
	CLRF       IOCAN+0
;test_cc1101_TX.c,373 :: 		Soft_SPI_Init();                                       //инициализаци€ программного SPI
	CALL       _Soft_SPI_Init+0
;test_cc1101_TX.c,374 :: 		fn_cc1101_init();                                      // инициализаци€ трансивера cc1101
	CALL       _fn_cc1101_init+0
;test_cc1101_TX.c,375 :: 		ManInit ();                                           //инициализаци€  передачи манчестер сигнала
	CALL       _ManInit+0
;test_cc1101_TX.c,378 :: 		while(1){
L_main30:
;test_cc1101_TX.c,379 :: 		asm{clrwdt};                                       //сброс собаки
	CLRWDT
;test_cc1101_TX.c,381 :: 		if( !flag_status_cc1101_tx ) {                           // проверка флага включени€ передатчика
	MOVF       _flag_status_cc1101_tx+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L_main32
;test_cc1101_TX.c,382 :: 		PWR_TRANSEIVER=1;                                  // включаем питание передатчика
	BSF        LATC+0, 1
;test_cc1101_TX.c,383 :: 		fn_cc1101_strob (STX);                             //включаем трансивер на передачу
	MOVLW      53
	MOVWF      FARG_fn_cc1101_strob_strob+0
	CALL       _fn_cc1101_strob+0
;test_cc1101_TX.c,384 :: 		delay_ms(5);
	MOVLW      26
	MOVWF      R12
	MOVLW      248
	MOVWF      R13
L_main33:
	DECFSZ     R13, 1
	GOTO       L_main33
	DECFSZ     R12, 1
	GOTO       L_main33
	NOP
;test_cc1101_TX.c,385 :: 		flag_status_cc1101_tx=1;                          //устанавливаем флаг
	MOVLW      1
	MOVWF      _flag_status_cc1101_tx+0
;test_cc1101_TX.c,386 :: 		}
L_main32:
;test_cc1101_TX.c,389 :: 		while( flag_status_cc1101_tx){
L_main34:
	MOVF       _flag_status_cc1101_tx+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main35
;test_cc1101_TX.c,393 :: 		asm{clrwdt};                                       //сброс собаки
	CLRWDT
;test_cc1101_TX.c,395 :: 		if( Button(FORWARD))  dataButtons |=(1<<0);        //опрос кнопок
	MOVLW      PORTA+0
	MOVWF      FARG_Button_port+0
	MOVLW      hi_addr(PORTA+0)
	MOVWF      FARG_Button_port+1
	CLRF       FARG_Button_pin+0
	MOVLW      20
	MOVWF      FARG_Button_time_ms+0
	CLRF       FARG_Button_active_state+0
	CALL       _Button+0
	MOVF       R0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main36
	BSF        _dataButtons+0, 0
L_main36:
;test_cc1101_TX.c,396 :: 		if( Button(REVERSE))  dataButtons |=(1<<1);
	MOVLW      PORTA+0
	MOVWF      FARG_Button_port+0
	MOVLW      hi_addr(PORTA+0)
	MOVWF      FARG_Button_port+1
	MOVLW      1
	MOVWF      FARG_Button_pin+0
	MOVLW      20
	MOVWF      FARG_Button_time_ms+0
	CLRF       FARG_Button_active_state+0
	CALL       _Button+0
	MOVF       R0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main37
	BSF        _dataButtons+0, 1
L_main37:
;test_cc1101_TX.c,397 :: 		if( Button(LEFT ))    dataButtons |=(1<<2);
	MOVLW      PORTA+0
	MOVWF      FARG_Button_port+0
	MOVLW      hi_addr(PORTA+0)
	MOVWF      FARG_Button_port+1
	MOVLW      2
	MOVWF      FARG_Button_pin+0
	MOVLW      20
	MOVWF      FARG_Button_time_ms+0
	CLRF       FARG_Button_active_state+0
	CALL       _Button+0
	MOVF       R0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main38
	BSF        _dataButtons+0, 2
L_main38:
;test_cc1101_TX.c,398 :: 		if( Button(RIGHT))    dataButtons |=(1<<3);
	MOVLW      PORTA+0
	MOVWF      FARG_Button_port+0
	MOVLW      hi_addr(PORTA+0)
	MOVWF      FARG_Button_port+1
	MOVLW      4
	MOVWF      FARG_Button_pin+0
	MOVLW      20
	MOVWF      FARG_Button_time_ms+0
	CLRF       FARG_Button_active_state+0
	CALL       _Button+0
	MOVF       R0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main39
	BSF        _dataButtons+0, 3
L_main39:
;test_cc1101_TX.c,400 :: 		if( Button(TRIGGER_PASS))  {
	MOVLW      PORTA+0
	MOVWF      FARG_Button_port+0
	MOVLW      hi_addr(PORTA+0)
	MOVWF      FARG_Button_port+1
	MOVLW      5
	MOVWF      FARG_Button_pin+0
	MOVLW      20
	MOVWF      FARG_Button_time_ms+0
	MOVLW      1
	MOVWF      FARG_Button_active_state+0
	CALL       _Button+0
	MOVF       R0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main40
;test_cc1101_TX.c,401 :: 		flagOldstate=1;
	MOVLW      1
	MOVWF      _flagOldstate+0
;test_cc1101_TX.c,402 :: 		}
L_main40:
;test_cc1101_TX.c,405 :: 		if( Button (TRIGGER_ACT) && flagOldstate ) {        //управление светом фар
	MOVLW      PORTA+0
	MOVWF      FARG_Button_port+0
	MOVLW      hi_addr(PORTA+0)
	MOVWF      FARG_Button_port+1
	MOVLW      5
	MOVWF      FARG_Button_pin+0
	MOVLW      20
	MOVWF      FARG_Button_time_ms+0
	CLRF       FARG_Button_active_state+0
	CALL       _Button+0
	MOVF       R0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main43
	MOVF       _flagOldstate+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main43
L__main48:
;test_cc1101_TX.c,406 :: 		flagOldstate=0;
	CLRF       _flagOldstate+0
;test_cc1101_TX.c,407 :: 		flagTrigger = ~flagTrigger;
	COMF       _flagTrigger+0, 0
	MOVWF      R0
	MOVF       R0, 0
	MOVWF      _flagTrigger+0
;test_cc1101_TX.c,408 :: 		if(flagTrigger){
	MOVF       R0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main44
;test_cc1101_TX.c,409 :: 		dataButtons |=(1<<4);                      //дл€ триггера(свет фар)используем два бита,один(4)вкл.
	BSF        _dataButtons+0, 4
;test_cc1101_TX.c,410 :: 		dataButtons &=~(1<<5);                     //второй(5) выкл.
	BCF        _dataButtons+0, 5
;test_cc1101_TX.c,411 :: 		}
	GOTO       L_main45
L_main44:
;test_cc1101_TX.c,413 :: 		dataButtons &=~(1<<4);                   //очистить бит включени€ света
	BCF        _dataButtons+0, 4
;test_cc1101_TX.c,414 :: 		dataButtons |=(1<<5);                    //команда выключить свет
	BSF        _dataButtons+0, 5
;test_cc1101_TX.c,415 :: 		}
L_main45:
;test_cc1101_TX.c,416 :: 		}
L_main43:
;test_cc1101_TX.c,419 :: 		if(dataButtons){                                  // если кнопка нажата...
	MOVF       _dataButtons+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main46
;test_cc1101_TX.c,421 :: 		ManBufAddByte(0,dataButtons );                //поместить в 0 €чейку буфера байт данных кнопок
	CLRF       FARG_ManBufAddByte_place+0
	MOVF       _dataButtons+0, 0
	MOVWF      FARG_ManBufAddByte_byte+0
	CALL       _ManBufAddByte+0
;test_cc1101_TX.c,422 :: 		ManBufAddByte(1,speedLevel);                  //поместить в 1 €чейку буфера байт данных скорости
	MOVLW      1
	MOVWF      FARG_ManBufAddByte_place+0
	MOVF       _speedLevel+0, 0
	MOVWF      FARG_ManBufAddByte_byte+0
	CALL       _ManBufAddByte+0
;test_cc1101_TX.c,424 :: 		ManTransmitData (2);                         //передать два байта данных из буфера
	MOVLW      2
	MOVWF      FARG_ManTransmitData_BufLen+0
	CALL       _ManTransmitData+0
;test_cc1101_TX.c,425 :: 		dataButtons=0;                               //обнулить переменную опроса кнопок
	CLRF       _dataButtons+0
;test_cc1101_TX.c,427 :: 		}
	GOTO       L_main47
L_main46:
;test_cc1101_TX.c,429 :: 		flagOldstate=1;                           //выставим флаг триггера
	MOVLW      1
	MOVWF      _flagOldstate+0
;test_cc1101_TX.c,430 :: 		PWR_TRANSEIVER=0;                         //выключаем питание передатчика
	BCF        LATC+0, 1
;test_cc1101_TX.c,431 :: 		fn_cc1101_strob (SIDLE);                  //трансивер в ждущий режим
	MOVLW      54
	MOVWF      FARG_fn_cc1101_strob_strob+0
	CALL       _fn_cc1101_strob+0
;test_cc1101_TX.c,432 :: 		flag_status_cc1101_tx=0;                  //сброс флага включени€ передатчика
	CLRF       _flag_status_cc1101_tx+0
;test_cc1101_TX.c,433 :: 		INTCON=0b10001000;                        //настройки прерываний
	MOVLW      136
	MOVWF      INTCON+0
;test_cc1101_TX.c,434 :: 		IOCAN=0b00110111;                        //прерывание по изменению порта
	MOVLW      55
	MOVWF      IOCAN+0
;test_cc1101_TX.c,435 :: 		asm{sleep};                              //идем спать
	SLEEP
;test_cc1101_TX.c,437 :: 		}
L_main47:
;test_cc1101_TX.c,439 :: 		}
	GOTO       L_main34
L_main35:
;test_cc1101_TX.c,442 :: 		}
	GOTO       L_main30
;test_cc1101_TX.c,445 :: 		}
L_end_main:
	GOTO       $+0
; end of _main
