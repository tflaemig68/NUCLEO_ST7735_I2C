/*
 * I2C.c
 *
 *  Created on: 18.12.2023
 *      Author: tobia
 */

#include <mcalGPIO.h>
#include <mcalI2C.h>
#include <mcalSysTick.h>
#include "I2C.h"




    //RFID SL018

	uint8_t 	*strFirmware = (uint8_t *) " ....        \0";
	uint8_t  RFIDcmd_LEDon[3] = {0x02, 0x40, 0xFF};
	uint8_t  RFIDcmd_LEDoff[3] = {0x02, 0x40, 0x00};
	uint8_t  RFIDcmd_getFirmwareVersion[2] = {0x01, 0xF0};
	uint8_t  RFIDcmd_getMifareUID[2] = {0x01, 0x01};

	bool enableRFID = false;

	// LIS3DH
	bool enableLIS3DH = false;
	uint8_t  LIS3DHcmd_TempOn[2] = {0x1F, 0x40};
	#define ADCConfReg 0x1F
	#define StatusReg 0xA7
	#define AcclReg 0x28
	#define ADC1Reg 0x08
	#define ADC3Temp 0x0C
	#define OUTxyz 0x28
	#define repeat 0x80
	#define CtrlReg1 0x20
	#define CtrlReg4 0x23
	#define CtrlReg5 0x24
	// LIDAR
	bool enableLIDAR = false;


void i2c_activate_pb89(I2C_TypeDef *i2c)
{

    GPIO_TypeDef  *portB = GPIOB;

    // GPIOB-Bustakt aktivieren wegen der Verwendung von PB8/PB9 (I2C).
    i2cSelectI2C(i2c);                           // I2C1: Bustakt aktivieren
    //i2cDisableDevice(i2c);
    gpioInitPort(portB);
    //gpioSetPin(portB, PIN8);// falls der Takt-Ausgang auf Low hängen geblieben ist
    //gpioSetPin(portB, PIN9);
    gpioSelectPinMode(portB, PIN8, ALTFUNC);
    gpioSelectAltFunc(portB, PIN8, AF4);         // PB8 : I2C1 SCL
    gpioSelectPinMode(portB, PIN9, ALTFUNC);
    gpioSelectAltFunc(portB, PIN9, AF4);         // PB9 : I2C1 SDA

    /**
     * Verwenden Sie auf keinen Fall die MCU-internen Pull-up-Widerstaende!
     * Widerstandswerte: jeweils 4k7 fuer SDA und SCL!
     */
    gpioSetOutputType(portB, PIN8, OPENDRAIN);   // Immer externe Pull-up-
    gpioSetOutputType(portB, PIN9, OPENDRAIN);   // Widerstaende verwenden!!!
    // Initialisierung des I2C-Controllers

    i2cInitI2C(i2c, I2C_DUTY_CYCLE_2, 17, I2C_CLOCK_400);
    /*
    i2cSetPeripheralClockFreq(i2c, pclock);      // I2C1: Periph. Clk in MHz
    i2cSetDutyCycle(i2c, I2C_DUTY_CYCLE_2);      // I2C1: Duty-cycle einstellen
    i2cSetRiseTime(i2c, 17);                     // I2C1: 17 ist ein bewaehrter Wert
    i2c->CCR |= 0x50;                            // I2C1: Keine MCAL-Funktion
    */
    i2cEnableDevice(i2c);                        // I2C1: Aktivieren


}


uint8_t *convDecByteToHex(uint8_t byte)
{
    static  uint8_t hex[2] = { 0 };

    uint8_t temp;

    temp = byte % 16;
    if (temp < 10)
    {
        temp += '0';
    }
    else
    {
        temp += '7';
    }
    hex[1] = temp;

    temp = byte / 16;
    if (temp < 10)
    {
        temp += '0';
    }
    else
    {
        temp += '7';
    }
    hex[0] = temp;

    return hex;
}

int8_t LIS3DH_init(I2C_TypeDef *i2c)
{
	//i2cBurstWrite(i2c, i2cAddr_LIS3DH, LIS3DHcmd_TempOn, 2);
	i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH,CtrlReg5,  0b11000000); // reboot default memory content
	i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH,CtrlReg4,  0b10001010); // BDU BLE FS1 FS0 HR ST1 ST0 SIM
	i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH,CtrlReg4,  0b10001100); // BDU BLE FS1 FS0 HR ST1 ST0 SIM
	i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH,CtrlReg1,  0b10010111); //
	i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH,CtrlReg4,  0b10001000); // BDU BLE FS1 FS0 HR ST1 ST0 SIM
	i2cSendByteToSlaveReg(i2c, i2cAddr_LIS3DH,ADCConfReg,0b11000000); // ADC Temp 0 0 0 0 0 0
	return 0;
}


int16_t LIS3DH_XYZ(I2C_TypeDef *i2c, int16_t *xyz)
{
	uint8_t readBuffer[6];
	int16_t Temp = 0;
	i2cBurstRegRead(i2c, i2cAddr_LIS3DH, (OUTxyz|repeat),readBuffer, 6);
	*xyz = (readBuffer[1]<<8) + readBuffer[0];
	xyz++;
	*xyz = (readBuffer[3]<<8) + readBuffer[2];
	xyz++;
	*xyz = (readBuffer[5]<<8) + readBuffer[4];
	return Temp;
}


/* This function register represent the deviation of the current Sensor Temp versus the Kalibration Refenze Temp approx 20dergC
 *
 *
 */
int16_t LIS3DH_Temp(I2C_TypeDef *i2c)
{
	uint8_t readBuffer[6];
	int16_t Temp;
	i2cBurstRegRead(i2c, i2cAddr_LIS3DH, (ADC3Temp|repeat), readBuffer, 2);
	Temp = (readBuffer[1]<<8) + readBuffer[0];
	return (Temp);					// makes an approx absolut Temp Value
}


void RFID_LED(I2C_TypeDef *i2c, bool LEDon)
{
	if (LEDon == 1)
	{
		i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_LEDon, 3);
	}
	else
	{
		i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_LEDoff, 3);
	}
}

int8_t RFID_readCard(I2C_TypeDef *i2c, uint8_t* CardID)
{
	static  uint8_t step = 1;
	uint8_t readBuffer[12];
	uint8_t len, i, j = 0;
	int8_t typeCard = -1;
	static int8_t RFID_Status;
	char *p_out;

	switch (step)
	{
		case 1:
		{
			i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_getMifareUID, 2);
			step = 2;
			break;
		}
		case 2:
		{
			i2cBurstRead(i2c, i2cAddr_RFID, readBuffer, 0xC);
			len = readBuffer[0]-2;
			step = 1;
			if (RFID_Status != readBuffer[2])
			{
				typeCard = readBuffer[len];
				j = 0;
				for (i = 0; i< 8; i++ )
				{
					if (i < len)
					{
						if (i == len-1) { CardID[j++]='-'; }
						p_out =	(char *) convDecByteToHex(readBuffer[i+3]);
						CardID[j++]  = (char)*(p_out++);
						CardID[j++] =(char)*p_out;
					}
					else
					{CardID[j++]='.'; CardID[j++]='.';}

				}
				CardID[j]='.';
				RFID_Status = readBuffer[2];
			}
			break;
		}
		default:
		{
			step = 1;
		}
	}
	return typeCard;
}

int8_t RFID_readFWVersion(I2C_TypeDef *i2c, uint8_t* strFirmware)
{
	static  uint8_t step = 1;
		uint8_t readBuffer[16];
		uint8_t i, len;
		int8_t status = -1;

		switch (step)
		{
			case 1:
			{
				RFID_LED(i2c,0);
				step = 2;
				break;
			}
			case 2:
			{
				i2cBurstWrite(i2c, i2cAddr_RFID, RFIDcmd_getFirmwareVersion, 2);
				step = 3;
				break;
			}
			case 3:
			{
				i2cBurstRead(i2c, i2cAddr_RFID, readBuffer, 0xF);
				len = readBuffer[0];
				status = readBuffer[2];
				step = 4;
				for (i = 0; i<= len; i++ )
				{
				 strFirmware[i] = readBuffer[i+3];
				}

				break;
			}
			case 4:
			{
				RFID_LED(i2c,0);
				step = 1;
				break;
			}

			default:
			{
				step = 1;
			}
		}
		return status;

}

