#line 1 "C:/Users/Public/Documents/Mikroelektronika/mikroC PRO for PIC/Projects/test_cc1101_TX/test_cc1101_TX.c"
#line 21 "C:/Users/Public/Documents/Mikroelektronika/mikroC PRO for PIC/Projects/test_cc1101_TX/test_cc1101_TX.c"
sbit Chip_Select at RC5_bit;
sbit SoftSpi_CLK at RC4_bit;
sbit SoftSpi_SDI at RC3_bit;
sbit SoftSpi_SDO at RC2_bit;

sbit Chip_Select_Direction at TRISC5_bit;
sbit SoftSpi_CLK_Direction at TRISC4_bit;
sbit SoftSpi_SDI_Direction at TRISC3_bit;
sbit SoftSpi_SDO_Direction at TRISC2_bit;
#line 89 "C:/Users/Public/Documents/Mikroelektronika/mikroC PRO for PIC/Projects/test_cc1101_TX/test_cc1101_TX.c"
unsigned char rf_settings []={
  0x02 , 0x00,
  0x00 , 0x0D,
  0x03 , 0x47,
  0x08 ,0x32,
  0x0B , 0x06,
  0x0C , 0x00,
  0x0D , 0x10,
  0x0E , 0xB0,
  0x0F , 0x71,
  0x10 , 0xA7,
  0x11 , 0x32,
  0x12 , 0x30,
  0x13 , 0x22,
  0x14 , 0xF8,
  0x16 , 0x07,
  0x17 , 0x30,
  0x18 , 0x18,
  0x1B , 0x04,
  0x1C , 0x00,
  0x1D , 0x92,
  0x21 , 0xB6,
  0x22 , 0x11,
  0x23 , 0xE9,
  0x24 , 0x2A,
  0x25 , 0x00,
  0x26 , 0x1F,
  0x2C , 0x81,
  0x2D , 0x35,
  0x2E , 0x09
} ;


void fn_cc1101_strob(unsigned char strob)
{
 Chip_Select = 0;

 Soft_SPI_Write( strob);
 delay_ms(1);
 Chip_Select = 1;

}




void fn_cc1101_init()
{
 char i ;
 Chip_Select = 0;



 for(i=0;i!=58;i++)
 {
 Soft_SPI_Write(rf_settings[i]);

 }



 Chip_Select = 1;




 delay_ms(5);
 fn_cc1101_strob ( 0x36 );
return;
}
#line 187 "C:/Users/Public/Documents/Mikroelektronika/mikroC PRO for PIC/Projects/test_cc1101_TX/test_cc1101_TX.c"
void ManInit (void);
void ManBufAddByte (unsigned char place, unsigned char byte);
void ManTransmitData (unsigned char BufLen);
void ManTransmitByte (unsigned char byte);
void ManTransmitBit (unsigned char bit_t);
void ManPause (void);
void ManCheckSumm(unsigned char data_t);


unsigned char ManTransmitDataBuf [ 16 ];
unsigned char ManIdentifier [] = { "s" };
unsigned char CheckSummByte;
unsigned char dataButtons=0;
unsigned char speedLevel= 128;
unsigned char flagOldstate=0;
unsigned char flagTrigger=0;
volatile unsigned char flag_status_cc1101_tx=0;


void ManInit (void)
{
  TRISC  &= ~ (1<<0) ;
}




void ManBufAddByte (unsigned char place, unsigned char byte)
{
 if (place >=  16 ) return;
 ManTransmitDataBuf [place] = byte;
}




void ManTransmitData (unsigned char BufLen){
 unsigned char i=0;
 unsigned char u=0;
 unsigned char a=0;
 unsigned char byte =0 ;


 INTCON.GIE =0;

 for ( i=0; i<  8 ; i++) {
 ManTransmitBit (1);
 }





 while (1) {

 byte = ManIdentifier [a];
 a++;
 if (byte)ManTransmitByte (byte);
 else
 break;
 }



 CheckSummByte = 0;
 ManTransmitByte (BufLen);


 for ( u=0; u<(BufLen); u++) {

 ManTransmitByte (ManTransmitDataBuf [u]);
 }



 ManTransmitByte (CheckSummByte);


 LATC.B0 = 0;

 INTCON.GIE =1;

}




void ManTransmitByte (unsigned char byte)
{ unsigned char i=0;
 ManCheckSumm (byte);

 for ( i=0; i<8; i++) {

 if (byte & 0x80) ManTransmitBit (1);
 else ManTransmitBit (0);
 byte <<= 1;
 }
}



void ManTransmitBit (unsigned char bit_t)
{
 if (bit_t) {

  PORTC  &= ~( (1<<0) ); ManPause ();
  PORTC  |=  (1<<0) ; ManPause ();
 }

 else {
  PORTC  |=  (1<<0) ; ManPause ();
  PORTC  &= ~ (1<<0) ; ManPause ();
 }
}


 void ManPause (void)
{
 delay_us (500000 /  1000 );
}








void ManCheckSumm(unsigned char data_t)
{ unsigned char i=0;

 for ( i=0; i<8; i++) {

 unsigned char temp = data_t;
 temp ^= CheckSummByte;

 if (temp & 0x01) {

 CheckSummByte ^= 0x18;
 temp = 0x80;
 }

 else temp = 0;

 CheckSummByte >>= 1;
 CheckSummByte |= temp;
 data_t >>= 1;
 }
}


void interrupt (void){
 if(INTCON.IOCIE & INTCON.IOCIF){
 INTCON.IOCIF=0;
 IOCAF.IOCAF0=0;
 IOCAF.IOCAF1=0;
 IOCAF.IOCAF2=0;
 IOCAF.IOCAF4=0;
 IOCAF.IOCAF5=0;

 PORTA;
 INTCON=0;
 IOCAN=0;

 }


}



void main (void) {
 OSCCON=0b11111111;
 TRISA=0b11111111;
 ANSELA=0;
 ANSELC=0;
 PORTA=0b000000;
 WPUA=0b111111;
 TRISC=0b00000000;
 PORTC=0b00000000;
 OPTION_REG=0b00000000;
 INTCON=0b00000000;
 WDTCON=0b00010000;
 IOCAN=0b00000000;


 Soft_SPI_Init();
 fn_cc1101_init();
 ManInit ();


 while(1){
 asm{clrwdt};

 if( !flag_status_cc1101_tx ) {
  LATC.B1 =1;
 fn_cc1101_strob ( 0x35 );
 delay_ms(5);
 flag_status_cc1101_tx=1;
 }


 while( flag_status_cc1101_tx){



 asm{clrwdt};

 if( Button( &PORTA,0,20,0 )) dataButtons |=(1<<0);
 if( Button( &PORTA,1,20,0 )) dataButtons |=(1<<1);
 if( Button( &PORTA,2,20,0  )) dataButtons |=(1<<2);
 if( Button( &PORTA,4,20,0 )) dataButtons |=(1<<3);

 if( Button( &PORTA,5,20,1 )) {
 flagOldstate=1;
 }


 if( Button ( &PORTA,5,20,0 ) && flagOldstate ) {
 flagOldstate=0;
 flagTrigger = ~flagTrigger;
 if(flagTrigger){
 dataButtons |=(1<<4);
 dataButtons &=~(1<<5);
 }
 else {
 dataButtons &=~(1<<4);
 dataButtons |=(1<<5);
 }
 }


 if(dataButtons){

 ManBufAddByte(0,dataButtons );
 ManBufAddByte(1,speedLevel);

 ManTransmitData (2);
 dataButtons=0;

 }
 else {
 flagOldstate=1;
  LATC.B1 =0;
 fn_cc1101_strob ( 0x36 );
 flag_status_cc1101_tx=0;
 INTCON=0b10001000;
 IOCAN=0b00110111;
 asm{sleep};

 }

 }


 }


}
