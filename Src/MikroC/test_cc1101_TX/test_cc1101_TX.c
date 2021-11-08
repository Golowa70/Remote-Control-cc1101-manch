  /* ***��������� ����������� ������� ��������������� � �������������� ���� ����������***
  ���� ������ ������(������),���������� ���������� ������ � ����� ������� ������(������)
  ,����� ������ ��������,��� ���������� ������ ���� �����.���� ������ �� ������,
  ���������� ��������� � ����� SLEEP.
*/

// ***����� ���������-����***
//  � ������ ���������� ��������� ��������� (�������� ������) ����� ��������� �������� ,
//  ����� ��������� - �������������, �������� 's'  (���� ����),
//  ����� ���� ����� ������, ����� �������� ����� ������ � ���� CRC
// (�.�. ����������� ����� � 1 ������ ������, ������ 4 �����)
//  ����0 = 's' - ������������� (����� ���� �� 1 �� 16 ��������� ���������� �����)
//  ����1 = 0x01 - ���� ����� ����� ������ (����� ���� �� 1 ����� �� ������������� �������� ������� ������)
//  ����2 = 0xXX - ����(�) ����� ������ (���������� ���� ������, ����� ����������� ��������)
//  ����3 = 0xXX - ���� CRC

//**********************************************************************************************************

 //-----------------------------------------------------------------------------
   // CC1101 module connections
sbit Chip_Select at RC5_bit;
sbit SoftSpi_CLK at RC4_bit;
sbit SoftSpi_SDI at RC3_bit;
sbit SoftSpi_SDO at RC2_bit;

sbit Chip_Select_Direction at TRISC5_bit;
sbit SoftSpi_CLK_Direction at TRISC4_bit;
sbit SoftSpi_SDI_Direction at TRISC3_bit;
sbit SoftSpi_SDO_Direction at TRISC2_bit;
// End CC1101 module connections
//#define RX_DATA     PORTA.B4      // ����� ������ ���������

#define SIDLE   0x36              // ������ �����
#define SRX     0x34              // ����
#define STX     0x35              // ��������
#define SRES    0x30              // �����
#define SNOP    0x3D              // �������� ������
//-----------------------------------------------------------------------------

#define IOCFG2     0x00
#define IOCFG1     0x01
#define IOCFG0     0x02
#define FIFOTHR    0x03
#define SYNC1      0x04
#define SYNC0      0x05
#define PKTLEN     0x06
#define PKTCTRL1   0x07
#define PKTCTRL0   0x08
#define ADDR       0x09
#define CHANNR     0x0A
#define FSCTRL1    0x0B
#define FSCTRL0    0x0C
#define FREQ2      0x0D
#define FREQ1      0x0E
#define FREQ0      0x0F
#define MDMCFG4    0x10
#define MDMCFG3    0x11
#define MDMCFG2    0x12
#define MDMCFG1    0x13
#define MDMCFG0    0x14
#define DEVIATN    0x15
#define MCSM2            0x16
#define MCSM1            0x17
#define MCSM0            0x18
#define FOCCFG            0x19
#define BSCFG            0x1A
#define AGCTRL2    0x1B
#define AGCTRL1    0x1C
#define AGCTRL0    0x1D
#define WOREVT1    0x1E
#define WOREVT0    0x1F
#define WORCTRL    0x20
#define FREND1            0x21
#define FREND0            0x22
#define FSCAL3            0x23
#define FSCAL2            0x24
#define FSCAL1            0x25
#define FSCAL0            0x26
#define RCCTRL1           0x27
#define RCCTRL0           0x28
#define FSTEST            0x29
#define PTEST            0x2A
#define AGCTEST          0x2B
#define TEST2            0x2C
#define TEST1            0x2D
#define TEST0            0x2E


unsigned char rf_settings []={
  IOCFG0,  0x00,                 //   ���� GDO0
  IOCFG2,  0x0D,                 //   GDO2
  FIFOTHR, 0x47,
  PKTCTRL0,0x32,                 //����������� ����� ������ ����������
  FSCTRL1, 0x06,
  FSCTRL0, 0x00,
  FREQ2,   0x10,
  FREQ1,   0xB0,
  FREQ0,   0x71,
  MDMCFG4, 0xA7,
  MDMCFG3, 0x32,
  MDMCFG2, 0x30,
  MDMCFG1, 0x22,
  MDMCFG0, 0xF8,
  MCSM2,   0x07,
  MCSM1,   0x30,
  MCSM0,   0x18,
  AGCTRL2, 0x04,
  AGCTRL1, 0x00,
  AGCTRL0, 0x92,
  FREND1,  0xB6,
  FREND0,  0x11,
  FSCAL3,  0xE9,
  FSCAL2,  0x2A,
  FSCAL1,  0x00,
  FSCAL0,  0x1F,
  TEST2,   0x81,
  TEST1,   0x35,
  TEST0,   0x09
} ;

 //----------------------------------------------------------------------------
void fn_cc1101_strob(unsigned char  strob)
{
     Chip_Select = 0;
    // strobe
    Soft_SPI_Write( strob);
    delay_ms(1);
    Chip_Select = 1;

}

 ///////////////////////////////


void fn_cc1101_init()
{
    char i ;
    Chip_Select = 0;                // ���������� ����, �������� ���.
    //while(spi_miso);        // ��� ����� ��� ��� �������, ������� miso
    // rf_settings

    for(i=0;i!=58;i++)  // ������ 34 ��������� �� �������
    {
        Soft_SPI_Write(rf_settings[i]);       //

    }

    // for( i=0;i<sizeof(rf_settings);i++) Soft_SPI_Write(rf_settings[i]);

    Chip_Select = 1;          // ��������� ����
    //---------------------------------------------------
    // ��� ����� ���� �� ���������������� paTable (��. ������)
    // �� ��� ���� ����� ��� �� �������������.
    // ������� ����� ����� ����� ������� � ������� � ����� IDLE.
    delay_ms(5);
    fn_cc1101_strob (SIDLE);
return;
}


//------------------------------------------------------------------------------





#define F_CPU 16000000                                             //������� ����������
#define MAN_SPEED                    1000                          /*1000-4000 ���\��� - ������� MANCHESTER �������*/
#define MAN_IDENTIFIER                "s"                         /*1-16 ���. ����. ��������� - ������������� ������.*/
#define MAN_PILOT_LEN                  8                           /*1-16 ���. ����� ��������� �������*/
#define MAN_BUF_LENGTH                16                           /*1-255 ����. ������ ������ ������*/

#define TRANSMIT_TRIS                 TRISC                       //���� ������ ������� MANCHESTER
#define TRANSMIT_PORT                 PORTC
#define TRANSMIT_LINE                 (1<<0)                      //����� ������ ������� MANCHESTER
//------------------------------------------------------------------------------
#define   FORWARD                     &PORTA,0,20,0               //������
#define   REVERSE                     &PORTA,1,20,0
#define   LEFT                        &PORTA,2,20,0
#define   RIGHT                       &PORTA,4,20,0
#define   TRIGGER_ACT                 &PORTA,5,20,0
#define   TRIGGER_PASS                &PORTA,5,20,1

#define   PWR_TRANSEIVER              LATC.B1                    //����� ������� �����������
//------------------------------------------------------------------------------
//��������� �������
void ManInit (void);                                             //������������� ����������
void ManBufAddByte (unsigned char place, unsigned char byte);    //������� ��������� ����� ������ � ����� ��������
void ManTransmitData (unsigned char BufLen);                     //������� �������� ������ � ��������� manchester
void ManTransmitByte (unsigned char byte);                       //������� �������� �����
void ManTransmitBit (unsigned char bit_t);                       //������� �������� ����
void ManPause (void);                                            //������� �������� ��� ��������� ������� manchester
void ManCheckSumm(unsigned char data_t);                         //������� �������� ����������� �����

//���������� ���������� --------------------------------------------------------
unsigned char ManTransmitDataBuf [MAN_BUF_LENGTH];            //����� ������������ ������
unsigned char ManIdentifier [] = {MAN_IDENTIFIER};            //��������� - ������������� ������.
unsigned char CheckSummByte;                                  //���� �������� CRC
unsigned char dataButtons=0;                                  //���������� ������ ������
unsigned char speedLevel= 128;                                 //������� ��������
unsigned char flagOldstate=0;                                 //���� ��������� ������ ��������
unsigned char flagTrigger=0;                                  //���� ��������
volatile unsigned char flag_status_cc1101_tx=0;                        //

//��������� ���������� ---------------------------------------------------------
void ManInit (void)
{
        TRANSMIT_TRIS &= ~TRANSMIT_LINE;                        //����� �� �����
}
//------------------------------------------------------------------------------
//������� ��������� ����� � ����� ������
//��������1 - ����� ������ ������, ���� ��������� ���� ������
//��������2 - ���� ������ ���������� � �����
void ManBufAddByte (unsigned char place, unsigned char byte)
{
        if (place >= MAN_BUF_LENGTH)        return;
        ManTransmitDataBuf [place] = byte;
}
//------------------------------------------------------------------------------
//������� �������� ������ � ��������� MANCHESTER
//�������� - ���������� ���� ������ ManTransmitDataBuf[], ������� ���������� ��������
//�� ����� ������ �������, ���������� �����������
void ManTransmitData (unsigned char BufLen){
         unsigned char  i=0;
         unsigned char  u=0;
         unsigned char  a=0;
         unsigned char  byte =0 ;

       // unsigned char srbuf = STATUS;                        //��������� ��������� ���������� ?
        INTCON.GIE =0;                                         //������ ���� ����������

       for ( i=0; i< MAN_PILOT_LEN; i++) {                     //�������� ��������� �������
                ManTransmitBit (1);
              }



        //�������� �������������

           while (1)   {

                    byte = ManIdentifier [a];
                       a++;
                         if (byte)ManTransmitByte (byte);
                    else
                    break;
               }


        //�������� ������ ����� ������
        CheckSummByte = 0;                                   //�������� ����������
        ManTransmitByte (BufLen);

        //�������� ������ ������
        for (  u=0; u<(BufLen); u++) {

                ManTransmitByte (ManTransmitDataBuf [u]);
              }


        //�������� ����������� �����
          ManTransmitByte (CheckSummByte);

//        ManTransmitBit (0);
          LATC.B0 = 0;                                       //����� ����
       // STATUS = srbuf;                                    //��������������� ��������� ���������� ?
        INTCON.GIE =1;                                       //���������� ����������

}

//------------------------------------------------------------------------------
//�������� �����
//�������� - ���� ������������ ������
void ManTransmitByte (unsigned char byte)
{       unsigned char i=0;
        ManCheckSumm (byte);

        for ( i=0; i<8; i++) {

                if (byte & 0x80)        ManTransmitBit (1);
                else                        ManTransmitBit (0);
                byte <<= 1;
             }
}
//------------------------------------------------------------------------------
//�������� ����
//�������� - ���� �� ��������� 0 ��� 1 (�������� ������������� ����)
void ManTransmitBit (unsigned char bit_t)
{
        if (bit_t) {

                TRANSMIT_PORT &= ~(TRANSMIT_LINE);        ManPause ();
                TRANSMIT_PORT |= TRANSMIT_LINE;                ManPause ();                                                                                   ///
              }

        else {
                TRANSMIT_PORT |= TRANSMIT_LINE;                ManPause ();
                TRANSMIT_PORT &= ~TRANSMIT_LINE;        ManPause ();
               }
}
//-----------------------------------------------------------------------------
//������� �����
 void ManPause (void)
{
        delay_us (500000 / MAN_SPEED);
}
//-----------------------------------------------------------------------------
//������� �������� ����������� �����
//�������� - ���� ����������� � ������������ ����������� �����
//CheckSummByte - ���������� ���������� ����������� �����
//� ������ ������ ���������� �������� CheckSummByte
//�� ���������� ������ (���������� ����� �-��� ��� ����� ������
// � ���� ����������� �����) � CheckSummByte ������ ���� 0

void ManCheckSumm(unsigned char data_t)
{           unsigned char i=0;

        for ( i=0; i<8; i++) {

                unsigned char temp = data_t;
                temp ^= CheckSummByte;

                if (temp & 0x01) {

                        CheckSummByte ^= 0x18;
                        temp = 0x80;
                     }

                else        temp = 0;

                CheckSummByte >>= 1;
                CheckSummByte |= temp;
                data_t >>= 1;
            }
}

//-----------------------------------------------------------------------------
void interrupt (void){
  if(INTCON.IOCIE & INTCON.IOCIF){                      //���� ���������� �� ���������...�����������
     INTCON.IOCIF=0;                                    //������� �����
     IOCAF.IOCAF0=0;
     IOCAF.IOCAF1=0;
     IOCAF.IOCAF2=0;
     IOCAF.IOCAF4=0;
     IOCAF.IOCAF5=0;
    // flag_status_cc1101_tx=0;
     PORTA;                                           //��������� ���� ?
     INTCON=0;                                       //���������� ������ ����������
     IOCAN=0;                                        //������ ���������� �� ��������� �����

    }


}

//-----------------------------------------------------------------------------

void main (void) {
     OSCCON=0b11111111;                                       //�������� �������
     TRISA=0b11111111;
     ANSELA=0; 
     ANSELC=0;                                               //���������� ���
     PORTA=0b000000;
     WPUA=0b111111;                                           //������������� ������.
     TRISC=0b00000000;
     PORTC=0b00000000;
     OPTION_REG=0b00000000;
     INTCON=0b00000000;                                       //��������� ����������
     WDTCON=0b00010000;                                       //�������� ������ ???
     IOCAN=0b00000000;                                        //���������� �� ��������� �����


      Soft_SPI_Init();                                       //������������� ������������ SPI
      fn_cc1101_init();                                      // ������������� ���������� cc1101
      ManInit ();                                           //�������������  �������� ��������� �������


     while(1){
          asm{clrwdt};                                       //����� ������
          
         if( !flag_status_cc1101_tx ) {                           // �������� ����� ��������� �����������
              PWR_TRANSEIVER=1;                                  // �������� ������� �����������
              fn_cc1101_strob (STX);                             //�������� ��������� �� ��������
              delay_ms(5);
              flag_status_cc1101_tx=1;                          //������������� ����
           }
           
           
         while( flag_status_cc1101_tx){

            

           asm{clrwdt};                                       //����� ������

           if( Button(FORWARD))  dataButtons |=(1<<0);        //����� ������
           if( Button(REVERSE))  dataButtons |=(1<<1);
           if( Button(LEFT ))    dataButtons |=(1<<2);
           if( Button(RIGHT))    dataButtons |=(1<<3);

           if( Button(TRIGGER_PASS))  {
               flagOldstate=1;
              }


           if( Button (TRIGGER_ACT) && flagOldstate ) {        //���������� ������ ���
                 flagOldstate=0;
                 flagTrigger = ~flagTrigger;
                 if(flagTrigger){
                    dataButtons |=(1<<4);                      //��� ��������(���� ���)���������� ��� ����,����(4)���.
                    dataButtons &=~(1<<5);                     //������(5) ����.
                  }
                 else {
                      dataButtons &=~(1<<4);                   //�������� ��� ��������� �����
                      dataButtons |=(1<<5);                    //������� ��������� ����
                      }
              }


           if(dataButtons){                                  // ���� ������ ������...

               ManBufAddByte(0,dataButtons );                //��������� � 0 ������ ������ ���� ������ ������
               ManBufAddByte(1,speedLevel);                  //��������� � 1 ������ ������ ���� ������ ��������

               ManTransmitData (2);                         //�������� ��� ����� ������ �� ������
               dataButtons=0;                               //�������� ���������� ������ ������

            }
          else  {                                           // ���� ������ �� ������
                  flagOldstate=1;                           //�������� ���� ��������
                  PWR_TRANSEIVER=0;                         //��������� ������� �����������
                  fn_cc1101_strob (SIDLE);                  //��������� � ������ �����
                  flag_status_cc1101_tx=0;                  //����� ����� ��������� �����������
                  INTCON=0b10001000;                        //��������� ����������
                  IOCAN=0b00110111;                        //���������� �� ��������� �����
                  asm{sleep};                              //���� �����

                }

         }
            
            
     }


}

//------------------------------------------------------------------------------