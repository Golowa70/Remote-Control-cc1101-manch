
_fn_cc1101_strob:

;test_cc1101_RX.c,122 :: 		void fn_cc1101_strob(unsigned char  strob)
;test_cc1101_RX.c,124 :: 		Chip_Select = 0;
	BCF        RC4_bit+0, BitPos(RC4_bit+0)
;test_cc1101_RX.c,126 :: 		Soft_SPI_Write( strob);
	MOVF       FARG_fn_cc1101_strob_strob+0, 0
	MOVWF      FARG_Soft_SPI_Write_sdata+0
	CALL       _Soft_SPI_Write+0
;test_cc1101_RX.c,127 :: 		delay_ms(1);
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
;test_cc1101_RX.c,128 :: 		Chip_Select = 1;
	BSF        RC4_bit+0, BitPos(RC4_bit+0)
;test_cc1101_RX.c,130 :: 		}
L_end_fn_cc1101_strob:
	RETURN
; end of _fn_cc1101_strob

_fn_cc1101_init:

;test_cc1101_RX.c,135 :: 		void fn_cc1101_init()
;test_cc1101_RX.c,138 :: 		Chip_Select = 0;                // активируем порт, выбираем чип.
	BCF        RC4_bit+0, BitPos(RC4_bit+0)
;test_cc1101_RX.c,142 :: 		for(i=0;i!=58;i++)  // запись 34 регистров из таблицы
	CLRF       fn_cc1101_init_i_L0+0
L_fn_cc1101_init1:
	MOVF       fn_cc1101_init_i_L0+0, 0
	XORLW      58
	BTFSC      STATUS+0, 2
	GOTO       L_fn_cc1101_init2
;test_cc1101_RX.c,144 :: 		Soft_SPI_Write(rf_settings[i]);       //
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
;test_cc1101_RX.c,142 :: 		for(i=0;i!=58;i++)  // запись 34 регистров из таблицы
	INCF       fn_cc1101_init_i_L0+0, 1
;test_cc1101_RX.c,146 :: 		}
	GOTO       L_fn_cc1101_init1
L_fn_cc1101_init2:
;test_cc1101_RX.c,150 :: 		Chip_Select = 1;          // отпускаем порт
	BSF        RC4_bit+0, BitPos(RC4_bit+0)
;test_cc1101_RX.c,155 :: 		delay_ms(5);
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
;test_cc1101_RX.c,156 :: 		fn_cc1101_strob (SIDLE);
	MOVLW      54
	MOVWF      FARG_fn_cc1101_strob_strob+0
	CALL       _fn_cc1101_strob+0
;test_cc1101_RX.c,157 :: 		return;
;test_cc1101_RX.c,158 :: 		}
L_end_fn_cc1101_init:
	RETURN
; end of _fn_cc1101_init

_ManReceiveStart:

;test_cc1101_RX.c,226 :: 		void ManReceiveStart (void)
;test_cc1101_RX.c,229 :: 		INTCON.GIE=0;                                       //запретить все прерывания
	BCF        INTCON+0, 7
;test_cc1101_RX.c,231 :: 		OPTION_REG=0b10000101;                             //предделитель на 64 (частота счета 16000000/4 / 32 = 125000 Hz = 8 uS)  //64?????????
	MOVLW      133
	MOVWF      OPTION_REG+0
;test_cc1101_RX.c,232 :: 		INTCON.TMR0IE=1;                                   //прерывание при переполнении
	BSF        INTCON+0, 5
;test_cc1101_RX.c,235 :: 		INTCON.IOCIE=1;                                   //разрешение прерываний по изменению уровня
	BSF        INTCON+0, 3
;test_cc1101_RX.c,236 :: 		IOCAP.IOCAP4=1;                                   //включить прерывание  по фронту от RA5
	BSF        IOCAP+0, 4
;test_cc1101_RX.c,237 :: 		IOCAN.IOCAN4=1;                                   //включить прерывание  по спаду от RA5
	BSF        IOCAN+0, 4
;test_cc1101_RX.c,242 :: 		ManFlags &= ~(bTIM0_OVF| bDATA_ENBL);             //очистить флаг наличия данных и флаг переполнения
	MOVLW      252
	ANDWF      _ManFlags+0, 1
;test_cc1101_RX.c,243 :: 		ManFlags |= bHEADER_RCV;                          //включить режим приема заголовка
	BSF        _ManFlags+0, 3
;test_cc1101_RX.c,244 :: 		ByteCounter = 0;                                  //начать прием с начала
	CLRF       _ByteCounter+0
;test_cc1101_RX.c,245 :: 		ByteIn = 0x00;                                    //очистить байт приемник
	CLRF       _ByteIn+0
;test_cc1101_RX.c,247 :: 		INTCON.GIE=1;                                     //разрешить все прерывания
	BSF        INTCON+0, 7
;test_cc1101_RX.c,248 :: 		}
L_end_ManReceiveStart:
	RETURN
; end of _ManReceiveStart

_ManReceiveStop:

;test_cc1101_RX.c,253 :: 		void ManReceiveStop (void)
;test_cc1101_RX.c,255 :: 		INTCON.GIE=0;
	BCF        INTCON+0, 7
;test_cc1101_RX.c,256 :: 		INTCON.TMR0IE=0;                                  //выключить "прерывание при переполнении Т0"
	BCF        INTCON+0, 5
;test_cc1101_RX.c,257 :: 		INTCON.IOCIE=0;                                   //выключить "внешнее прерывание от IOC"
	BCF        INTCON+0, 3
;test_cc1101_RX.c,258 :: 		INTCON.GIE=1;
	BSF        INTCON+0, 7
;test_cc1101_RX.c,259 :: 		}
L_end_ManReceiveStop:
	RETURN
; end of _ManReceiveStop

_ManRcvDataCheck:

;test_cc1101_RX.c,265 :: 		unsigned char* ManRcvDataCheck (void)
;test_cc1101_RX.c,267 :: 		if (ManFlags & bDATA_ENBL)                             //проверка наличия принятых данных
	BTFSS      _ManFlags+0, 0
	GOTO       L_ManRcvDataCheck5
;test_cc1101_RX.c,269 :: 		ManFlags &= ~bDATA_ENBL;                       //очистить флаг наличия данных
	BCF        _ManFlags+0, 0
;test_cc1101_RX.c,270 :: 		return ManBuffer;                              //при наличии данных - возвращаем указатель на буффер
	MOVLW      _ManBuffer+0
	MOVWF      R0
	MOVLW      hi_addr(_ManBuffer+0)
	MOVWF      R1
	GOTO       L_end_ManRcvDataCheck
;test_cc1101_RX.c,271 :: 		}
L_ManRcvDataCheck5:
;test_cc1101_RX.c,272 :: 		return 0;                                              //при отсутствии данных - возвращаем 0
	CLRF       R0
	CLRF       R1
;test_cc1101_RX.c,273 :: 		}
L_end_ManRcvDataCheck:
	RETURN
; end of _ManRcvDataCheck

_interrupt:

;test_cc1101_RX.c,279 :: 		void interrupt (void)  {
;test_cc1101_RX.c,280 :: 		if( PIE3.TMR6IE && PIR3.TMR6IF ) {
	BTFSS      PIE3+0, 3
	GOTO       L_interrupt8
	BTFSS      PIR3+0, 3
	GOTO       L_interrupt8
L__interrupt69:
;test_cc1101_RX.c,281 :: 		PIR3.TMR6IF=0;
	BCF        PIR3+0, 3
;test_cc1101_RX.c,282 :: 		if( timeOffOut_counter <0)
	MOVLW      128
	XORWF      _timeOffOut_counter+1, 0
	MOVWF      R0
	MOVLW      128
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__interrupt77
	MOVLW      0
	SUBWF      _timeOffOut_counter+0, 0
L__interrupt77:
	BTFSC      STATUS+0, 0
	GOTO       L_interrupt9
;test_cc1101_RX.c,283 :: 		timeOffOut_counter++;
	INCF       _timeOffOut_counter+0, 1
	BTFSC      STATUS+0, 2
	INCF       _timeOffOut_counter+1, 1
L_interrupt9:
;test_cc1101_RX.c,284 :: 		if(timeOffDevice_counter <0)
	MOVLW      128
	XORWF      _timeOffDevice_counter+1, 0
	MOVWF      R0
	MOVLW      128
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__interrupt78
	MOVLW      0
	SUBWF      _timeOffDevice_counter+0, 0
L__interrupt78:
	BTFSC      STATUS+0, 0
	GOTO       L_interrupt10
;test_cc1101_RX.c,285 :: 		timeOffDevice_counter++;
	INCF       _timeOffDevice_counter+0, 1
	BTFSC      STATUS+0, 2
	INCF       _timeOffDevice_counter+1, 1
L_interrupt10:
;test_cc1101_RX.c,287 :: 		asm {clrwdt};                                    //сброс собаки
	CLRWDT
;test_cc1101_RX.c,288 :: 		}
L_interrupt8:
;test_cc1101_RX.c,293 :: 		if (INTCON.T0IF && INTCON.T0IE)
	BTFSS      INTCON+0, 2
	GOTO       L_interrupt13
	BTFSS      INTCON+0, 5
	GOTO       L_interrupt13
L__interrupt68:
;test_cc1101_RX.c,296 :: 		INTCON.T0IF=0;                                              //сбросить флаг переполнения таймера
	BCF        INTCON+0, 2
;test_cc1101_RX.c,297 :: 		if (INTCON.IOCIE==1)                                        //если ожидали внеш прерывания от IOC -
	BTFSS      INTCON+0, 3
	GOTO       L_interrupt14
;test_cc1101_RX.c,298 :: 		ManFlags |= bTIM0_OVF;                              // - отметить переполнение
	BSF        _ManFlags+0, 1
	GOTO       L_interrupt15
L_interrupt14:
;test_cc1101_RX.c,302 :: 		if (MAN_IN_PIN & MAN_IN_LINE)    {
	BTFSS      PORTA+0, 4
	GOTO       L_interrupt16
;test_cc1101_RX.c,303 :: 		ManFlags |= bLINE_VAL;
	BSF        _ManFlags+0, 2
;test_cc1101_RX.c,304 :: 		}
	GOTO       L_interrupt17
L_interrupt16:
;test_cc1101_RX.c,307 :: 		ManFlags &= ~bLINE_VAL;
	BCF        _ManFlags+0, 2
;test_cc1101_RX.c,308 :: 		}
L_interrupt17:
;test_cc1101_RX.c,310 :: 		INTCON.IOCIE=1;                                        //включить внешние прерывания от IOC
	BSF        INTCON+0, 3
;test_cc1101_RX.c,311 :: 		INTCON.IOCIF=0;                                        //сбросить возможно проскочившее прерывание
	BCF        INTCON+0, 0
;test_cc1101_RX.c,312 :: 		IOCAF.IOCAF4=0;                                        //--''--
	BCF        IOCAF+0, 4
;test_cc1101_RX.c,314 :: 		asm {clrwdt};                                    //сброс собаки
	CLRWDT
;test_cc1101_RX.c,315 :: 		}
L_interrupt15:
;test_cc1101_RX.c,317 :: 		}
L_interrupt13:
;test_cc1101_RX.c,321 :: 		if (INTCON.IOCIE && INTCON.IOCIF && IOCAF.IOCAF4)
	BTFSS      INTCON+0, 3
	GOTO       L_interrupt20
	BTFSS      INTCON+0, 0
	GOTO       L_interrupt20
	BTFSS      IOCAF+0, 4
	GOTO       L_interrupt20
L__interrupt67:
;test_cc1101_RX.c,325 :: 		TimerVal = TMR0;
	MOVF       TMR0+0, 0
	MOVWF      _TimerVal+0
;test_cc1101_RX.c,326 :: 		TMR0 = 255 - ((MAN_PERIOD_LEN )* 3 / 4);               //счетчик таймера настроить на 3/4 длины периода MANCHESTER бита данных
	MOVLW      209
	MOVWF      TMR0+0
;test_cc1101_RX.c,327 :: 		INTCON.IOCIE=0;                                        //выключить внешнее прерывание
	BCF        INTCON+0, 3
;test_cc1101_RX.c,328 :: 		INTCON.IOCIF=0;                                        //на случай ВЧ сигнала сбросить возможно проскочившее повторное прерывание
	BCF        INTCON+0, 0
;test_cc1101_RX.c,329 :: 		IOCAF.IOCAF4=0;
	BCF        IOCAF+0, 4
;test_cc1101_RX.c,332 :: 		if ( (TimerVal > (MAN_PERIOD_LEN/2)) || (ManFlags & bTIM0_OVF))
	MOVF       _TimerVal+0, 0
	SUBLW      31
	BTFSS      STATUS+0, 0
	GOTO       L__interrupt66
	BTFSC      _ManFlags+0, 1
	GOTO       L__interrupt66
	GOTO       L_interrupt23
L__interrupt66:
;test_cc1101_RX.c,335 :: 		Ini :
___interrupt_Ini:
;test_cc1101_RX.c,336 :: 		asm {clrwdt};                                    //сброс собаки
	CLRWDT
;test_cc1101_RX.c,338 :: 		ManFlags &= ~(bTIM0_OVF);                       //сбросить флаг переполнения
	BCF        _ManFlags+0, 1
;test_cc1101_RX.c,339 :: 		ManFlags |= bHEADER_RCV;                        //ожидать прием заголовка
	BSF        _ManFlags+0, 3
;test_cc1101_RX.c,340 :: 		ByteCounter = 0;                                //начать прием с начала
	CLRF       _ByteCounter+0
;test_cc1101_RX.c,341 :: 		ByteIn = 0x00;                                  //очистить байт приемник
	CLRF       _ByteIn+0
;test_cc1101_RX.c,343 :: 		}
L_interrupt23:
;test_cc1101_RX.c,346 :: 		ByteIn <<= 1;                                           //сдвигаем байт перед записью бита
	LSLF       _ByteIn+0, 1
;test_cc1101_RX.c,348 :: 		if (! (ManFlags & bLINE_VAL))   {
	BTFSC      _ManFlags+0, 2
	GOTO       L_interrupt24
;test_cc1101_RX.c,349 :: 		ByteIn |= 1;
	BSF        _ByteIn+0, 0
;test_cc1101_RX.c,350 :: 		}
L_interrupt24:
;test_cc1101_RX.c,354 :: 		if (ManFlags & bHEADER_RCV)
	BTFSS      _ManFlags+0, 3
	GOTO       L_interrupt25
;test_cc1101_RX.c,357 :: 		if (ByteCounter == 0)
	MOVF       _ByteCounter+0, 0
	XORLW      0
	BTFSS      STATUS+0, 2
	GOTO       L_interrupt26
;test_cc1101_RX.c,359 :: 		Invert = ~ManIdentifier [0];
	COMF       _ManIdentifier+0, 0
	MOVWF      _Invert+0
;test_cc1101_RX.c,361 :: 		if (ByteIn != ManIdentifier [0]) {                            //?????? я добавил скобки
	MOVF       _ByteIn+0, 0
	XORWF      _ManIdentifier+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_interrupt27
;test_cc1101_RX.c,363 :: 		if (ByteIn != Invert)
	MOVF       _ByteIn+0, 0
	XORWF      _Invert+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_interrupt28
;test_cc1101_RX.c,365 :: 		return;                                        //пока нет совпадения - выход
	GOTO       L__interrupt76
L_interrupt28:
;test_cc1101_RX.c,366 :: 		}
L_interrupt27:
;test_cc1101_RX.c,367 :: 		if (ByteIn == ManIdentifier [0]) {
	MOVF       _ByteIn+0, 0
	XORWF      _ManIdentifier+0, 0
	BTFSS      STATUS+0, 2
	GOTO       L_interrupt29
;test_cc1101_RX.c,368 :: 		ManFlags &= ~bLINE_INV;                //прямое совпадение
	BCF        _ManFlags+0, 4
;test_cc1101_RX.c,370 :: 		}
	GOTO       L_interrupt30
L_interrupt29:
;test_cc1101_RX.c,373 :: 		ManFlags |= bLINE_INV;                //инверсное совпадение
	BSF        _ManFlags+0, 4
;test_cc1101_RX.c,374 :: 		}
L_interrupt30:
;test_cc1101_RX.c,375 :: 		BitCounter = 0;                                        //готовимся к приему следующих байтов хедера
	CLRF       _BitCounter+0
;test_cc1101_RX.c,376 :: 		ByteCounter++;
	INCF       _ByteCounter+0, 1
;test_cc1101_RX.c,377 :: 		return;
	GOTO       L__interrupt76
;test_cc1101_RX.c,378 :: 		}
L_interrupt26:
;test_cc1101_RX.c,380 :: 		asm {clrwdt};                                    //сброс собаки
	CLRWDT
;test_cc1101_RX.c,382 :: 		if (++BitCounter < 8)                                //ждем заполнения байта
	INCF       _BitCounter+0, 1
	MOVLW      8
	SUBWF      _BitCounter+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_interrupt31
;test_cc1101_RX.c,383 :: 		return;
	GOTO       L__interrupt76
L_interrupt31:
;test_cc1101_RX.c,385 :: 		if (ManFlags & bLINE_INV)                        //если сигнал инверсный
	BTFSS      _ManFlags+0, 4
	GOTO       L_interrupt32
;test_cc1101_RX.c,386 :: 		ByteIn = ~ByteIn;
	COMF       _ByteIn+0, 1
L_interrupt32:
;test_cc1101_RX.c,388 :: 		if (ManIdentifier [ByteCounter])        //если хедер еще не закончен
	MOVLW      _ManIdentifier+0
	MOVWF      FSR0L
	MOVLW      hi_addr(_ManIdentifier+0)
	MOVWF      FSR0H
	MOVF       _ByteCounter+0, 0
	ADDWF      FSR0L, 1
	BTFSC      STATUS+0, 0
	INCF       FSR0H, 1
	MOVF       INDF0+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_interrupt33
;test_cc1101_RX.c,390 :: 		if (ByteIn != ManIdentifier [ByteCounter]){             //проверяем идентичность хедера
	MOVLW      _ManIdentifier+0
	MOVWF      FSR0L
	MOVLW      hi_addr(_ManIdentifier+0)
	MOVWF      FSR0H
	MOVF       _ByteCounter+0, 0
	ADDWF      FSR0L, 1
	BTFSC      STATUS+0, 0
	INCF       FSR0H, 1
	MOVF       _ByteIn+0, 0
	XORWF      INDF0+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_interrupt34
;test_cc1101_RX.c,391 :: 		goto Ini;                                        //байт не соответствует хедеру - рестарт
	GOTO       ___interrupt_Ini
;test_cc1101_RX.c,392 :: 		}
L_interrupt34:
;test_cc1101_RX.c,393 :: 		BitCounter = 0;
	CLRF       _BitCounter+0
;test_cc1101_RX.c,394 :: 		ByteCounter++;                                         //ожидаем следующий байт хедера
	INCF       _ByteCounter+0, 1
;test_cc1101_RX.c,395 :: 		return;
	GOTO       L__interrupt76
;test_cc1101_RX.c,396 :: 		}
L_interrupt33:
;test_cc1101_RX.c,399 :: 		if (ByteIn > MAN_BUF_LENGTH)  {
	MOVF       _ByteIn+0, 0
	SUBLW      16
	BTFSC      STATUS+0, 0
	GOTO       L_interrupt35
;test_cc1101_RX.c,401 :: 		goto Ini;                                             //размер блока данных превышает допустимый - рестарт
	GOTO       ___interrupt_Ini
;test_cc1101_RX.c,403 :: 		}
L_interrupt35:
;test_cc1101_RX.c,405 :: 		DataLength = ByteIn;                                        //запомним длину пакета
	MOVF       _ByteIn+0, 0
	MOVWF      _DataLength+0
;test_cc1101_RX.c,407 :: 		CheckSummByte = 0;                                         //очистить байт контрольной суммы
	CLRF       _CheckSummByte+0
;test_cc1101_RX.c,408 :: 		CheckSumm (ByteIn);                                        //подсчет контрольки, начиная с байта длины пакета
	MOVF       _ByteIn+0, 0
	MOVWF      FARG_CheckSumm_dataa+0
	CALL       _CheckSumm+0
;test_cc1101_RX.c,410 :: 		ManFlags &= ~bHEADER_RCV;                                 //переходим к приему основного файла
	BCF        _ManFlags+0, 3
;test_cc1101_RX.c,411 :: 		BitCounter = 0;
	CLRF       _BitCounter+0
;test_cc1101_RX.c,412 :: 		ByteCounter = 0;
	CLRF       _ByteCounter+0
;test_cc1101_RX.c,413 :: 		return;
	GOTO       L__interrupt76
;test_cc1101_RX.c,414 :: 		}
L_interrupt25:
;test_cc1101_RX.c,415 :: 		asm {clrwdt};                                    //сброс собаки
	CLRWDT
;test_cc1101_RX.c,417 :: 		if (++BitCounter < 8)                                        //ждем накопления байта
	INCF       _BitCounter+0, 1
	MOVLW      8
	SUBWF      _BitCounter+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_interrupt36
;test_cc1101_RX.c,418 :: 		return;
	GOTO       L__interrupt76
L_interrupt36:
;test_cc1101_RX.c,419 :: 		BitCounter = 0;
	CLRF       _BitCounter+0
;test_cc1101_RX.c,421 :: 		if (ManFlags & bLINE_INV)                                   //необходима ли инверсия
	BTFSS      _ManFlags+0, 4
	GOTO       L_interrupt37
;test_cc1101_RX.c,422 :: 		ByteIn = ~ByteIn;
	COMF       _ByteIn+0, 1
L_interrupt37:
;test_cc1101_RX.c,424 :: 		CheckSumm (ByteIn);                                         //подсчет контрольки
	MOVF       _ByteIn+0, 0
	MOVWF      FARG_CheckSumm_dataa+0
	CALL       _CheckSumm+0
;test_cc1101_RX.c,426 :: 		if (DataLength--) {                                         //если это еще байты пакета -
	MOVF       _DataLength+0, 0
	MOVWF      R0
	DECF       _DataLength+0, 1
	MOVF       R0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_interrupt38
;test_cc1101_RX.c,427 :: 		ManBuffer [ByteCounter++] = ByteIn;                   // - сохраняем принятый байт
	MOVLW      _ManBuffer+0
	MOVWF      FSR1L
	MOVLW      hi_addr(_ManBuffer+0)
	MOVWF      FSR1H
	MOVF       _ByteCounter+0, 0
	ADDWF      FSR1L, 1
	BTFSC      STATUS+0, 0
	INCF       FSR1H, 1
	MOVF       _ByteIn+0, 0
	MOVWF      INDF1+0
	INCF       _ByteCounter+0, 1
;test_cc1101_RX.c,428 :: 		}
	GOTO       L_interrupt39
L_interrupt38:
;test_cc1101_RX.c,432 :: 		if (CheckSummByte) {                              //если контролька не верна (не 0) -
	MOVF       _CheckSummByte+0, 0
	BTFSC      STATUS+0, 2
	GOTO       L_interrupt40
;test_cc1101_RX.c,433 :: 		goto Ini;                                   // - рестарт
	GOTO       ___interrupt_Ini
;test_cc1101_RX.c,434 :: 		}
L_interrupt40:
;test_cc1101_RX.c,437 :: 		ManFlags |= bDATA_ENBL;                          //установить флаг наличия данных
	BSF        _ManFlags+0, 0
;test_cc1101_RX.c,438 :: 		ManReceiveStop ();                               //тормозим дальнейший прием
	CALL       _ManReceiveStop+0
;test_cc1101_RX.c,439 :: 		}
L_interrupt39:
;test_cc1101_RX.c,442 :: 		asm {clrwdt};                                    //сброс собаки
	CLRWDT
;test_cc1101_RX.c,443 :: 		}
L_interrupt20:
;test_cc1101_RX.c,444 :: 		}
L_end_interrupt:
L__interrupt76:
	RETFIE     %s
; end of _interrupt

_CheckSumm:

;test_cc1101_RX.c,452 :: 		void CheckSumm(unsigned char dataa)
;test_cc1101_RX.c,453 :: 		{              unsigned char i=0;
	CLRF       CheckSumm_i_L0+0
;test_cc1101_RX.c,454 :: 		for ( i=0; i<8; i++)
	CLRF       CheckSumm_i_L0+0
L_CheckSumm41:
	MOVLW      8
	SUBWF      CheckSumm_i_L0+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_CheckSumm42
;test_cc1101_RX.c,456 :: 		unsigned char temp = dataa;
	MOVF       FARG_CheckSumm_dataa+0, 0
	MOVWF      R2+0
;test_cc1101_RX.c,457 :: 		temp ^= CheckSummByte;
	MOVF       _CheckSummByte+0, 0
	XORWF      FARG_CheckSumm_dataa+0, 0
	MOVWF      R1
	MOVF       R1, 0
	MOVWF      R2+0
;test_cc1101_RX.c,459 :: 		if (temp & 0x01)        {CheckSummByte ^= 0x18; temp = 0x80;}
	BTFSS      R1, 0
	GOTO       L_CheckSumm44
	MOVLW      24
	XORWF      _CheckSummByte+0, 1
	MOVLW      128
	MOVWF      R2+0
	GOTO       L_CheckSumm45
L_CheckSumm44:
;test_cc1101_RX.c,460 :: 		else                                temp = 0;
	CLRF       R2+0
L_CheckSumm45:
;test_cc1101_RX.c,462 :: 		CheckSummByte >>= 1;
	LSRF       _CheckSummByte+0, 1
;test_cc1101_RX.c,463 :: 		CheckSummByte |= temp;
	MOVF       R2+0, 0
	IORWF       _CheckSummByte+0, 1
;test_cc1101_RX.c,464 :: 		dataa >>= 1;
	LSRF       FARG_CheckSumm_dataa+0, 1
;test_cc1101_RX.c,454 :: 		for ( i=0; i<8; i++)
	INCF       CheckSumm_i_L0+0, 1
;test_cc1101_RX.c,465 :: 		}
	GOTO       L_CheckSumm41
L_CheckSumm42:
;test_cc1101_RX.c,466 :: 		}
L_end_CheckSumm:
	RETURN
; end of _CheckSumm

_main:

;test_cc1101_RX.c,469 :: 		void main (void){
;test_cc1101_RX.c,471 :: 		OSCCON=0b11111111;                                  //  16 MHz HF
	MOVLW      255
	MOVWF      OSCCON+0
;test_cc1101_RX.c,472 :: 		TRISA=0b00010000;
	MOVLW      16
	MOVWF      TRISA+0
;test_cc1101_RX.c,473 :: 		PORTA=0b00000000;
	CLRF       PORTA+0
;test_cc1101_RX.c,474 :: 		TRISC=0b00000000;
	CLRF       TRISC+0
;test_cc1101_RX.c,475 :: 		PORTC=0b00000000;
	CLRF       PORTC+0
;test_cc1101_RX.c,476 :: 		ANSELA  = 0;                                       //выключение АЦП
	CLRF       ANSELA+0
;test_cc1101_RX.c,477 :: 		ANSELC  = 0;
	CLRF       ANSELC+0
;test_cc1101_RX.c,478 :: 		CM1CON0=0;                                         //выключение компаратора
	CLRF       CM1CON0+0
;test_cc1101_RX.c,479 :: 		CM2CON0=0;
	CLRF       CM2CON0+0
;test_cc1101_RX.c,481 :: 		T6CON=0b11111111;                              //настройка TMR6.Прескалер 64,постскалер 16.
	MOVLW      255
	MOVWF      T6CON+0
;test_cc1101_RX.c,482 :: 		PIE3.TMR6IE=1;                                 //разрешение прерываний от TMR6
	BSF        PIE3+0, 3
;test_cc1101_RX.c,484 :: 		asm {clrwdt};                                    //сброс собаки
	CLRWDT
;test_cc1101_RX.c,486 :: 		PWM1_Init(1500);                                 //частота ШИМ
	BSF        T2CON+0, 0
	BSF        T2CON+0, 1
	MOVLW      166
	MOVWF      PR2+0
	CALL       _PWM1_Init+0
;test_cc1101_RX.c,487 :: 		PWM1_Start();
	CALL       _PWM1_Start+0
;test_cc1101_RX.c,488 :: 		PWM1_Set_Duty(0);
	CLRF       FARG_PWM1_Set_Duty_new_duty+0
	CALL       _PWM1_Set_Duty+0
;test_cc1101_RX.c,489 :: 		PWM2_Init(1500);
	BSF        T2CON+0, 0
	BSF        T2CON+0, 1
	MOVLW      166
	MOVWF      PR2+0
	CALL       _PWM2_Init+0
;test_cc1101_RX.c,490 :: 		PWM2_Start();
	CALL       _PWM2_Start+0
;test_cc1101_RX.c,491 :: 		PWM2_Set_Duty(0);
	CLRF       FARG_PWM2_Set_Duty_new_duty+0
	CALL       _PWM2_Set_Duty+0
;test_cc1101_RX.c,493 :: 		PWR_RECEIVER =1;                                // включить питание приемника
	BSF        LATC+0, 4
;test_cc1101_RX.c,495 :: 		Soft_SPI_Init();
	CALL       _Soft_SPI_Init+0
;test_cc1101_RX.c,496 :: 		fn_cc1101_init();
	CALL       _fn_cc1101_init+0
;test_cc1101_RX.c,497 :: 		fn_cc1101_strob (SRX);
	MOVLW      52
	MOVWF      FARG_fn_cc1101_strob_strob+0
	CALL       _fn_cc1101_strob+0
;test_cc1101_RX.c,498 :: 		ManReceiveStart ();                             // старт приема данных
	CALL       _ManReceiveStart+0
;test_cc1101_RX.c,499 :: 		timeOffDevice_counter = - 9000;                //таймер полного выключения 9000-10 мин
	MOVLW      216
	MOVWF      _timeOffDevice_counter+0
	MOVLW      220
	MOVWF      _timeOffDevice_counter+1
;test_cc1101_RX.c,501 :: 		while (1)
L_main46:
;test_cc1101_RX.c,505 :: 		unsigned char *pBuf = ManRcvDataCheck();                    //проверка наличия данных
	CALL       _ManRcvDataCheck+0
	MOVF       R0, 0
	MOVWF      main_pBuf_L1+0
	MOVF       R1, 0
	MOVWF      main_pBuf_L1+1
;test_cc1101_RX.c,506 :: 		asm {clrwdt};                                           //сброс собаки
	CLRWDT
;test_cc1101_RX.c,507 :: 		if (pBuf)                                                   //если указатель не нулевой, значит данные поступили
	MOVF       main_pBuf_L1+0, 0
	IORWF       main_pBuf_L1+1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main48
;test_cc1101_RX.c,509 :: 		timeOffDevice_counter = -9000;                      //обновляем таймер выключения
	MOVLW      216
	MOVWF      _timeOffDevice_counter+0
	MOVLW      220
	MOVWF      _timeOffDevice_counter+1
;test_cc1101_RX.c,510 :: 		codeButtons = *pBuf;                                //копируем первый байт буфера
	MOVF       main_pBuf_L1+0, 0
	MOVWF      FSR0L
	MOVF       main_pBuf_L1+1, 0
	MOVWF      FSR0H
	MOVF       INDF0+0, 0
	MOVWF      R1
	MOVF       R1, 0
	MOVWF      _codeButtons+0
;test_cc1101_RX.c,511 :: 		*pBuf++;
	INCF       main_pBuf_L1+0, 1
	BTFSC      STATUS+0, 2
	INCF       main_pBuf_L1+1, 1
;test_cc1101_RX.c,512 :: 		speedLevel = *pBuf;
	MOVF       main_pBuf_L1+0, 0
	MOVWF      FSR0L
	MOVF       main_pBuf_L1+1, 0
	MOVWF      FSR0H
	MOVF       INDF0+0, 0
	MOVWF      _speedLevel+0
;test_cc1101_RX.c,514 :: 		if(codeButtons & (1<<0))                          //проверяем какая кнопка нажата
	BTFSS      R1, 0
	GOTO       L_main49
;test_cc1101_RX.c,516 :: 		if( ++ timerStartForward_counter < 5 )  FORWARD(255); //если вперед,при старте ШИМ на 100
	INCF       _timerStartForward_counter+0, 1
	MOVLW      5
	SUBWF      _timerStartForward_counter+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_main50
	MOVLW      255
	MOVWF      FARG_PWM1_Set_Duty_new_duty+0
	CALL       _PWM1_Set_Duty+0
	GOTO       L_main51
L_main50:
;test_cc1101_RX.c,518 :: 		FORWARD(speedLevel);                          //передаем в ШИМ значение скорости
	MOVF       _speedLevel+0, 0
	MOVWF      FARG_PWM1_Set_Duty_new_duty+0
	CALL       _PWM1_Set_Duty+0
L_main51:
;test_cc1101_RX.c,519 :: 		}
	GOTO       L_main52
L_main49:
;test_cc1101_RX.c,522 :: 		timerStartForward_counter=0;
	CLRF       _timerStartForward_counter+0
;test_cc1101_RX.c,523 :: 		FORWARD(0);
	CLRF       FARG_PWM1_Set_Duty_new_duty+0
	CALL       _PWM1_Set_Duty+0
;test_cc1101_RX.c,524 :: 		}
L_main52:
;test_cc1101_RX.c,526 :: 		if(codeButtons & (1<<1))                              //если назад,передаем в ШИМ значение скорости
	BTFSS      _codeButtons+0, 1
	GOTO       L_main53
;test_cc1101_RX.c,528 :: 		if(++ timerStartReverse_counter<5) REVERSE(255);
	INCF       _timerStartReverse_counter+0, 1
	MOVLW      5
	SUBWF      _timerStartReverse_counter+0, 0
	BTFSC      STATUS+0, 0
	GOTO       L_main54
	MOVLW      255
	MOVWF      FARG_PWM2_Set_Duty_new_duty+0
	CALL       _PWM2_Set_Duty+0
	GOTO       L_main55
L_main54:
;test_cc1101_RX.c,530 :: 		REVERSE (speedlevel);
	MOVF       _speedLevel+0, 0
	MOVWF      FARG_PWM2_Set_Duty_new_duty+0
	CALL       _PWM2_Set_Duty+0
L_main55:
;test_cc1101_RX.c,531 :: 		}
	GOTO       L_main56
L_main53:
;test_cc1101_RX.c,534 :: 		timerStartReverse_counter=0;
	CLRF       _timerStartReverse_counter+0
;test_cc1101_RX.c,535 :: 		REVERSE(0);
	CLRF       FARG_PWM2_Set_Duty_new_duty+0
	CALL       _PWM2_Set_Duty+0
;test_cc1101_RX.c,536 :: 		}
L_main56:
;test_cc1101_RX.c,539 :: 		if(codeButtons & (1<<2)) LEFT =1;                    //в лево
	BTFSS      _codeButtons+0, 2
	GOTO       L_main57
	BSF        LATC+0, 2
	GOTO       L_main58
L_main57:
;test_cc1101_RX.c,540 :: 		else  LEFT=0;
	BCF        LATC+0, 2
L_main58:
;test_cc1101_RX.c,542 :: 		if(codeButtons & (1<<3)) RIGHT =1;                   //в право
	BTFSS      _codeButtons+0, 3
	GOTO       L_main59
	BSF        LATC+0, 1
	GOTO       L_main60
L_main59:
;test_cc1101_RX.c,543 :: 		else  RIGHT=0;
	BCF        LATC+0, 1
L_main60:
;test_cc1101_RX.c,545 :: 		if(codeButtons & (1<<4)) TRIGGER =1;                 //включаем свет
	BTFSS      _codeButtons+0, 4
	GOTO       L_main61
	BSF        LATC+0, 0
	GOTO       L_main62
L_main61:
;test_cc1101_RX.c,547 :: 		if(codeButtons & (1<<5))
	BTFSS      _codeButtons+0, 5
	GOTO       L_main63
;test_cc1101_RX.c,548 :: 		TRIGGER=0;                                      //выключаем свет
	BCF        LATC+0, 0
L_main63:
;test_cc1101_RX.c,549 :: 		}
L_main62:
;test_cc1101_RX.c,552 :: 		ManBuffer[1]=0;                                     // обнуляем первый байт буфера данных
	CLRF       _ManBuffer+1
;test_cc1101_RX.c,553 :: 		ManBuffer[2]=0;                                     // обнуляем второй байт буфера данных
	CLRF       _ManBuffer+2
;test_cc1101_RX.c,554 :: 		codeButtons=0;                                      // обнуляем переменную кнопок
	CLRF       _codeButtons+0
;test_cc1101_RX.c,555 :: 		speedLevel=0;                                      // обнуляем переменную скорости
	CLRF       _speedLevel+0
;test_cc1101_RX.c,557 :: 		timeOffOut_counter = -3;                          //запуск таймера выключения выходов 2-160мс,3-240мсек.
	MOVLW      253
	MOVWF      _timeOffOut_counter+0
	MOVLW      255
	MOVWF      _timeOffOut_counter+1
;test_cc1101_RX.c,559 :: 		ManReceiveStart ();                                //перезапуск процесса чтения MANCHESTER данных
	CALL       _ManReceiveStart+0
;test_cc1101_RX.c,562 :: 		}
L_main48:
;test_cc1101_RX.c,565 :: 		asm {clrwdt};                                    //сброс собаки
	CLRWDT
;test_cc1101_RX.c,567 :: 		if(timeOffOut_counter>=0)                       //таймер отсчитал
	MOVLW      128
	XORWF      _timeOffOut_counter+1, 0
	MOVWF      R0
	MOVLW      128
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main81
	MOVLW      0
	SUBWF      _timeOffOut_counter+0, 0
L__main81:
	BTFSS      STATUS+0, 0
	GOTO       L_main64
;test_cc1101_RX.c,569 :: 		FORWARD(0);                              //выключаем выходы
	CLRF       FARG_PWM1_Set_Duty_new_duty+0
	CALL       _PWM1_Set_Duty+0
;test_cc1101_RX.c,570 :: 		REVERSE(0);
	CLRF       FARG_PWM2_Set_Duty_new_duty+0
	CALL       _PWM2_Set_Duty+0
;test_cc1101_RX.c,571 :: 		LEFT=0;
	BCF        LATC+0, 2
;test_cc1101_RX.c,572 :: 		RIGHT=0;
	BCF        LATC+0, 1
;test_cc1101_RX.c,573 :: 		timerStartForward_counter=0;
	CLRF       _timerStartForward_counter+0
;test_cc1101_RX.c,574 :: 		timerStartReverse_counter=0;
	CLRF       _timerStartReverse_counter+0
;test_cc1101_RX.c,575 :: 		}
L_main64:
;test_cc1101_RX.c,577 :: 		if(timeOffDevice_counter >=0 )
	MOVLW      128
	XORWF      _timeOffDevice_counter+1, 0
	MOVWF      R0
	MOVLW      128
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main82
	MOVLW      0
	SUBWF      _timeOffDevice_counter+0, 0
L__main82:
	BTFSS      STATUS+0, 0
	GOTO       L_main65
;test_cc1101_RX.c,579 :: 		TRIGGER=0;                                //выключаем свет
	BCF        LATC+0, 0
;test_cc1101_RX.c,580 :: 		FORWARD(0);                              //выключаем выходы
	CLRF       FARG_PWM1_Set_Duty_new_duty+0
	CALL       _PWM1_Set_Duty+0
;test_cc1101_RX.c,581 :: 		REVERSE(0);
	CLRF       FARG_PWM2_Set_Duty_new_duty+0
	CALL       _PWM2_Set_Duty+0
;test_cc1101_RX.c,582 :: 		LEFT=0;
	BCF        LATC+0, 2
;test_cc1101_RX.c,583 :: 		RIGHT=0;
	BCF        LATC+0, 1
;test_cc1101_RX.c,584 :: 		PWR_RECEIVER =0;                         //выключить питание приемника
	BCF        LATC+0, 4
;test_cc1101_RX.c,585 :: 		asm {sleep};                            //всем спать ))
	SLEEP
;test_cc1101_RX.c,586 :: 		}
L_main65:
;test_cc1101_RX.c,590 :: 		}
	GOTO       L_main46
;test_cc1101_RX.c,595 :: 		}
L_end_main:
	GOTO       $+0
; end of _main
