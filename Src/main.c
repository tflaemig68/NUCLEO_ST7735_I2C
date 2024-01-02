/**
 * ST7735 TFT Display
 * ===========================
 *
 * Ansteuerung eines TFT Display ueber SPI.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <stm32f4xx.h>

#include <mcalSysTick.h>
#include <mcalGPIO.h>
#include <mcalSPI.h>
#include <mcalI2C.h>

#include "hw_config.h"
#include "ST7735.h"
#include "I2C.h"
#include "tux_50_ad.h"


bool timerTrigger = false;



/* uncommend the next line if Graphictestroutine not used */
//#define GrafikTests 21

#ifdef GrafikTests
	unsigned int testcount = GrafikTests;
#endif

// Declaration  Timer1 = Main Prog
// 				ST7725_Timer delay Counter
uint32_t	Timer1 = 0UL;
uint32_t    ST7735_Timer = 0UL;
uint32_t    I2C_Timer = 0UL;
//#define I2CTaskTime 20


/* Private function prototypes -----------------------------------------------*/
void test_ascii_screen(void);
void test_graphics(void);

uint8_t I2C_SCAN(uint8_t scanAddr);



int main(void)
{
/*  I2C Variables  */

	uint8_t        scanAddr = 0x7F;  //7Bit Adresse
	I2C_TypeDef   *i2c  = I2C1;

	uint32_t   I2CTaskTime = 20UL;

/*  End I2C Variables  */

	//char strOut[] =      ". . . . . . . .\0";
	char strFirmware[] = ". . .          \0";  // dummyString with NULL
	char strX[8],strY[8],strZ[8], strT[8];
	int16_t Temp;
	int16_t XYZ[3];
	float X,Y,Z;

	static uint8_t testmode = 1;


	//int testmode = 1;
   	//unsigned int r = 0;

       // Dies ist das Array, das die Adressen aller Timer-Variablen enthaelt.
       // Auch die Groesse des Arrays wird berechnet.
       uint32_t *timerList[] = { &Timer1, &ST7735_Timer, &I2C_Timer /*, weitere Timer */ };
       size_t    arraySize = sizeof(timerList)/sizeof(timerList[0]);


    // Initialisiert den Systick-Timer
    systickInit(SYSTICK_1MS);

    systickSetMillis(&Timer1, 100);
    systickSetMillis(&I2C_Timer, I2CTaskTime);
    //lcd7735_initR(0);
    lcd7735_setup();
    LED_red_on;
    lcd7735_initR(INITR_REDTAB);
    lcd7735_setRotation(LANDSCAPE);
    //lcd7735_init_screen((uint8_t *)&SmallFont[0],ST7735_GREEN,ST7735_BLACK,LANDSCAPE); // not OK
    lcd7735_setFont((uint8_t *)&SmallFont[0]);
    LED_red_off;
    lcd7735_fillScreen(ST7735_BLACK);
    //lcd7735_cursor_set(0,0);
    //i2cResetDevice(i2c);
    i2c_activate_pb89(i2c);
    i2cSetRiseTime(i2c, 17);
    lcd7735_print((char *)"I2C Scanner running \0",0,0,0);


    while (1)
    {
	   if (true == timerTrigger)
	   {
			systickUpdateTimerList((uint32_t *) timerList, arraySize);
	   }

	   if (isSystickExpired(I2C_Timer))
	   {
		   systickSetTicktime(&I2C_Timer, I2CTaskTime);
		   LED_green_off;


		   switch (testmode)
		   {
		   	   case 1:  //I2C Scan
		   	   {
		   		   LED_green_on;
		   		   if ( I2C_SCAN(scanAddr) != 0)
				   {
					   LED_green_off;
					   switch (scanAddr)
					   {
						   case i2cAddr_RFID:
						   {
							   enableRFID = true;
							   lcd7735_print((char *)"RFID connected \0",0,28,0);
							   RFID_LED(i2c,true);
							   break;
						   }
						   case i2cAddr_LIDAR:
						   {
							   enableLIDAR = true;
							   //lcd7735_print((char *)"TOF/LIADR connected \0",0,28,0);
							   break;
						   }
						   case i2cAddr_LIS3DH:
						   {
							   enableLIS3DH = true;
							   lcd7735_print((char *)"LIS3DH connected \0",0,28,0);
							   lcd7735_print((char *)"Temp:\0",0,40,0);
							   lcd7735_print((char *)"X:\0",0,50,0);
							   lcd7735_print((char *)"Y:\0",0,60,0);
							   lcd7735_print((char *)"Z:\0",0,70,0);
							   LED_blue_on;
							   break;
						   }
					   }
				   }

				   if ((scanAddr == 0) && (enableRFID))
				   {
					   scanAddr = 0x7F;
					   I2CTaskTime = 100UL;
					   testmode = 2;
				   }
				   if ((scanAddr == 0) && (enableLIS3DH))
				   {
					   scanAddr = 0x7F;
					   testmode = 3;
					   I2CTaskTime = 200UL;
					   LIS3DH_init(i2c);
				   }
				   if ((scanAddr == 0))
				   {
					   scanAddr = 0x7F;
				       testmode = 4;
				   }
				   else
				   {
					   scanAddr -=1;
				   }

				   break;
				}
		   	   	case 2:  // read RFID
				{
					// RFID_readCard(i2c, CardID);

					if (RFID_readFWVersion(i2c, strFirmware) >= 0)
					{
						lcd7735_print((char *)"FW: \0",0,48,0);
						lcd7735_print((char *)strFirmware,24,48,0);
						testmode = 3;
						lcd7735_print((char *)"ID:\0",0,70,0);
					}
					else
					{
						;
					}
					break;
				}
		   		case 3:  // read LIS3DH Data
		   		{
		   			//lcd7735_print((char *)"ID:\0",0,70,0);
		   			LED_blue_on;

		   			Temp = LIS3DH_Temp(i2c);
		   			sprintf(strT, "%6i", Temp);
		   			lcd7735_print((char *)strT,40,40,0);

		   			Temp = LIS3DH_XYZ(i2c, XYZ);
  					X = (float) XYZ[0]/0x3FFF;  //skalierung 1mg/digit at +-2g
		   			Y = (float) XYZ[1]/0x3FFF;
		   			Z = (float) XYZ[2]/0x3FFF;
		   			sprintf(strX, "%+6.3f", X);
		   			lcd7735_print((char *)strX,20,50,0);
		   			sprintf(strY, "%+6.3f", Y);
		   			lcd7735_print((char *)strY,20,60,0);
		   			sprintf(strZ, "%+6.3f", Z);
		   			lcd7735_print((char *)strZ,20,70,0);
						//testmode = 2;
		   			LED_blue_off;


#ifdef GrafikTests
					testmode = 4;
#endif
				    break;
				}
#ifdef GrafikTests
		   	    case 4:
				{
					test_graphics();
				break;
				}
		   	    case 5:
				{
				    if (r++ > 360)
					{
					   r = 0;
					}
					lcd7735_print("Rotation",80,40,r);
				break;
				}
#endif
		   	   default:
				{
					testmode = 1;
				}
		   }  //end switch (testmode)
	   } // end if systickexp
    } //end while
    return 0;
}

#ifdef GrafikTests
    void test_graphics(void)
    {
   	switch (testcount--)
    	{
   		case 21:
   		{
   			lcd7735_setRotation(LANDSCAPE);
   			lcd7735_setFont((uint8_t *)&SmallFont[0]);
   			lcd7735_print("Hi the 1st output",0,0,0);
   		break;
   		}
   		case 20:
		{
			lcd7735_setFont((uint8_t *)&BigFont[0]);
			lcd7735_print("BigFont",0,20,0);
		break;
		}
   		case 19:
   		{
   			lcd7735_setFont((uint8_t *)&SevenSegNumFont[0]);
   			lcd7735_print("01234",0,60,0);
   		break;
   		}
   		case 18:
   		{
   			lcd7735_setFont((uint8_t *)&BigFont[0]);
   			lcd7735_fillScreen(ST7735_MAGENTA);
   			lcd7735_print("Hello!",20,10,0);
   		break;
   		}
    	case 17:	lcd7735_print("37deg Hello!",10,5,37);break;
    	case 16:	lcd7735_drawBitmap(0,0,50,52,(bitmapdatatype)tux_50_ad,1);break;
    	case 15:	lcd7735_drawBitmap(55,0,50,52,(bitmapdatatype)tux_50_ad,2);break;
    	case 14:
    	{
    		lcd7735_setRotation(PORTRAIT);
    		lcd7735_drawBitmap(0,0,50,52,(bitmapdatatype)tux_50_ad,1);
    	break;
    	}
    	case 13:	lcd7735_invertDisplay(INVERT_ON);break;
    	case 12:	lcd7735_invertDisplay(INVERT_OFF);break;
    	case 11:	lcd7735_fillScreen(ST7735_RED);break;
    	case 10:	lcd7735_fillScreen(ST7735_GREEN);break;
    	case  9:	lcd7735_fillScreen(ST7735_BLUE);break;
    	case  8:	lcd7735_fillScreen(ST7735_BLACK);break;
    	case  7:	lcd7735_fillRect(20,15,40,35,ST7735_BLUE);break;
    	case  6:	lcd7735_fillCircle(70,70,30,ST7735_YELLOW);break;
    	case  5:	lcd7735_drawRect(10,20,90,100,ST7735_MAGENTA);break;
    	case  4:	lcd7735_drawCircle(60,120,35,ST7735_CYAN);break;
    	case  3:	lcd7735_drawFastLine(10,5,110,120,ST7735_WHITE);break;
    	case  2:	lcd7735_invertDisplay(INVERT_ON);break;
        case  1:	lcd7735_invertDisplay(INVERT_OFF);break;
        	default:
        	{
        		testcount = GrafikTests;
        		lcd7735_fillScreen(ST7735_BLACK);
        		break;
        	}
    	}
    }

/* does not work
 *
 *
 */
    void test_ascii_screen(void)
    {
    	unsigned char x;
    	int i;

    	//lcd7735_init_screen((void *)&SmallFont[0],ST7735_GREEN,ST7735_BLACK,PORTRAIT);
      	lcd7735_fillScreen(ST7735_BLACK);
      	//cursor_init();
    	//printf("zz=%03.4f\n",34.678);
    	//while(1) {
    		x = 0x20;
    		for(i=0;i<95;i++)
    		{
    			lcd7735_putc(x+i);
    			delay_ms(50);
    		}
    		//if( UserButtonPressed == 0x01 )	return;
    }

#endif

uint8_t I2C_SCAN(uint8_t scanAddr)
{
	I2C_TypeDef   *i2c  = I2C1;
	//uint8_t 	*outString;
	uint8_t 	*outString2 = (uint8_t *) "Addr at: \0";
	uint8_t     *result;

	uint8_t foundAddr = 0;
	static int xPos = 0;
	//static int testrun = 0;



	// ! Scan on read/odd Adress keep SDA at Low -
	// I2C Interface will stopped
	// use only write/even adresse
	foundAddr = i2cFindSlaveAddr(i2c, scanAddr);
	if (xPos == 0)
	{
		lcd7735_print((char *)outString2,0,14,0);
		xPos = 66;
	}
	result = convDecByteToHex(scanAddr);
	if (foundAddr != 0)
	{
		//outString = outString2;
		lcd7735_print((char *)result,xPos,14,0);
		xPos = (int) 20 + xPos;
		if (xPos > 140)
		{
			xPos = 66;
		}
	}
	else
	{
	//	lcd7735_print((char *)result,xPos,14,0);
	}
	return foundAddr;

}





