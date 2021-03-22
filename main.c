/*  16F883 LEDでPWM練習プログラム
 * 作成者：坂本題
 * 日付：2021年3月15日
 * 仕様：200HZスタートで3秒毎に20HZ減らしていき、40HZになったら最初に戻るプログラム。段階的に周期とON時間を減らしていく
 * 目的：PICでのPWM操作に慣れる。
 */
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#define _XTAL_FREQ 500000 //動作クロック周波数0.5MHz

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF        // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown Out Reset Selection bits (BOR enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is enabled)
#pragma config LVP = OFF         // Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)


 
#define BAUDRATE 2400       // 9.6kbps
#define TX9_RX9_BIT 0       // 0: 8bit, 1: 9bit
#define BRGH_BIT 1          // 0: 低速サンプル指定, 1: 高速サンプル指定
 
#if TX9_RX9_BIT == 1
#define TX9_RX9_DATA 0x40 // 9bit
#else
#define TX9_RX9_DATA 0x00 // 8bit
#endif
 
#if BRGH_BIT == 1
#define BRGH_DATA 0x04
#define SPBRG_DATA ((unsigned char)(((_XTAL_FREQ / 16) / BAUDRATE) - 1))
#else
#define BRGH_DATA 0x00
#define SPBRG_DATA ((unsigned char)(((_XTAL_FREQ / 64) / BAUDRATE) - 1))
#endif

static void set_port(void);

void out_pwm(long dutylatio);
void init_pwm(void);
void change_ON(long period, long freq);

void initUART();
void putch(char byte);
char getch();
char getche();

/*
 *
 */
char BUZ=0x00;
char work;
void main(void) {
    set_port();
    init_pwm();
 
    char LEDS=0xff;
    char LED=0x00;
    char LED1=0x04;
    char LED2=0x08;
    char LED3=0x10;
    char LED4=0x20;
    PORTA = LEDS & (~LED1);//点灯
  
    initUART();             // 調歩同期式シリアル通信設定
    __delay_ms(500);
    printf("\n"); //改行
    printf("Hello World!\n\n");
    PORTA = PORTA & (~LED3);//点灯
    /*
    while(1){
        work = getch(); // 1文字受信
        if(!(work==0x00)){
            printf("Echo: %c\n", work); // 受信した文字を送信
            PORTA ^= 0b01000001;    // RA0(LED1)とRA6(LED2)をビット反転
        }
    }
    */

    while(1){
            init_pwm();
            double PWMPeriod;
            unsigned char bits;
            double priscaler; 
            double CLKPeriod;
        for(int freq=200; freq>=20; freq-=20){
            //周期PR2を設定
            PWMPeriod = 1000000.0f/freq;
            bits = OSCCON & 0b1110000;

            switch(bits){
                case 0b1110000://8MHz
                    CLKPeriod=1000000/8000000;
                    break;
                case 0b1100000://4MHz
                    CLKPeriod=1000000/4000000;
                    break;
                case 0b1010000://2MHz
                    CLKPeriod=1000000/2000000;
                    break;
                case 0b1000000://1MHz
                    CLKPeriod=1000000/1000000;
                    break;
                case 0b0110000://500kHz
                    CLKPeriod=1000000/500000;//2us
                    break;
                case 0b0100000://250kHz
                    CLKPeriod=1000000/250000;
                    break;
                case 0b0010000://125kHz
                    CLKPeriod=1000000/125000;
                    break;
                case 0b0000000://31kHz
                    CLKPeriod=1000000/31000;
                    break;
                default://error
                    break;
            }

            if(T2CONbits.T2CKPS==0b00){ 
               priscaler = 1.0f;
            }else if(T2CONbits.T2CKPS==0b01){
                priscaler = 4.0f;
            }else if(T2CONbits.T2CKPS&0b10){
                priscaler =16.0f;
            }else{
                //error
            }
            unsigned char uctp;     
            double temp = PWMPeriod / (4*CLKPeriod* priscaler);
            while(temp>0xFF || temp <0x00){
                //error
                if(priscaler == 1.0f){
                    T2CONbits.T2CKPS=0b01;
                    priscaler=4.0f;
                }
                else if(priscaler == 4.0f){
                    T2CONbits.T2CKPS=0b10;
                    priscaler=16.0f;
                }
                else if(priscaler == 16.0f){
                    //クロック周期変更
                    switch(bits){
                        case 0b1110000://8MHz
                            //4Mhzに変更

                            break;
                        case 0b1100000://4MHz
                            //2Mhzに変更
                            break;
                        case 0b1010000://2MHz
                            //1Mhzに変更
                            break;
                        case 0b1000000://1MHz
                            //0.5Mhzに変更
                            break;
                        case 0b0110000://500kHz
                            //0.25Mhzに変更
                            OSCCON = 0b0100000; //内部発振 500kHz使用に設定
                            CLKPeriod=1000000/250000;
                            break;
                        case 0b0100000://250kHz
                            //0.125Mhzに変更
                            OSCCON = 0b0010000; //内部発振 500kHz使用に設定
                            CLKPeriod=1000000/125000;
                            break;
                        case 0b0010000://125kHz
                            break;
                        case 0b0000000://31kHz
                            //SYS_ERR
                            return;
                        default://error
                            break;
                    }
                    bits = OSCCON & 0b1110000;
                }else{

                }
                temp = PWMPeriod / (4*CLKPeriod* priscaler);

            }
            uctp =(unsigned char)temp;
            PR2 = uctp;//周期決定
            
            double onTime = PWMPeriod / 2;
            double tmp = onTime / (priscaler * CLKPeriod);
            if(tmp> 1023 || tmp<0){
                //error
            }
            
            unsigned char tp[2];
            tp[0]=((unsigned short)tmp)/4;
            tp[1]=((unsigned short)tmp)&0b11;
            //ON時間変更
            CCPR1L=tp[0];//2.5u
            CCP1CONbits.DC1B=tp[1]&0b11;
            T2CONbits.TMR2ON = 1;//PWMスタート
            printf("PR2=%d \t CCPR1L=%x \t DC1B=%x\n",(unsigned char)uctp,tp[0],tp[1]);
            if(bits==0b0110000){
                //500kHz
                __delay_ms(3000);

            }else if(bits==0b0100000){
                //250kHz
                __delay_ms(1500);

            }

        }
        
    }
    return;
}

static void set_port(){
    ANSEL = 0b00000;//AN0~4をデジタルピンとして使用。アナログピンとして使用する場合1をたてる。
    TRISA = 0x00;  //RA0~7を出力として使用。入力として使用する場合1を立てる
    PORTA = 0xff;  //RA0~7をHIGH出力//LED群で、吸い込みなので全OFF
    TRISB = 0x00;  //RB0~7を出力として使用。
    PORTB = 0x00;  //RB0~7をLOW出力
    TRISC = 0x00;  //RC0~7を出力として使用。
    return;
}

void init_pwm(){
    OSCCON = 0b0110000; //内部発振 500kHz使用に設定
//    OSCCON = 0b1110000; //内部発振 8MHz使用に設定
    CCP1CONbits.CCP1M=0b1100;//CCP1(RC2)をPWMモードで使用。P1A~DをactiveHIFHで使用。
    CCP1CONbits.P1M0=0;
    CCP1CONbits.P1M1=0;
    T2CONbits.T2CKPS=0b01;//プリスケーラx4
}

void initUART()
{
    SPBRG = SPBRG_DATA; // ボーレート設定
    TXSTA = (TX9_RX9_DATA | BRGH_DATA | 0x20); // 8or9bit, 低速or高速サンプル指定
    RCSTA = (TX9_RX9_DATA | 0x90); // 低速or高速サンプル指定
    TXSTAbits.TXEN = 1;
    RCSTAbits.SPEN = 1;
}

void putch(char byte) // 1byte送信
{
    while(!TXIF){
    }
    TXREG = byte;
}
 
 
char getch() // 1byte受信
{
    while(!RCIF){
    }
 
    return RCREG;
}
 
char getche() // getchしてecho
{
    unsigned char c;
 
    c = getch();
    putch(c);
 
    return c;
}

/*
// ON時間を変更する関数
void changeON(long period, long freq){
    long ontime = period / freq * 
    //ON時間を設定
    CCPR1L=3/4;//最初は200Hzのため、5us。
    CCP1CONbits.DC1B=3&0b11;
}
*/