/*
*---------------基于"HAL底层库"的工程(CAUC_Flight)--------------
*@file driver_power.c
*@version V1.0
*@date  2018/5/24
*@brief 激光驱动文件
*@design 无人机研究小组
	     谈政政   朱通    赵志华   邓丽萍  张硕
*
*@all copyright reserved 中国民航大学 机器人研究所
*@hardware platform  STM32F40X
*@remark   	We are a team.
*@motto		I just want to know why the BeiJing MTR Line 4 are so many people every day.
*/
#include "stm32f4xx.h"
#include "driver_lidarlite.h"
#include "device_iic.h"



void LIDARLite_begin(u8 configuration, u8 fasti2c, u8 showErrorReporting, u8 LidarLiteI2cAddress)
{
	//首先要配置IIC接口
	if(fasti2c)
	{
		//快速IIC模式置1
	}
	LIDARLite_configure(configuration, LidarLiteI2cAddress);
}


void LIDARLite_configure(u8 configuration, u8 LidarLiteI2cAddress)
{
	switch (configuration)
	{
		case 0: //  Default configuration
		  IIC_Write_1Byte(LidarLiteI2cAddress,0x00,0x00);
		break;
		case 1: //  Set aquisition count to 1/3 default value, faster reads, slightly
				//  noisier values
		  IIC_Write_1Byte(LidarLiteI2cAddress,0x04,0x00);
		break;
		case 2: //  Low noise, low sensitivity: Pulls decision criteria higher
				//  above the noise, allows fewer false detections, reduces
				//  sensitivity
		  IIC_Write_1Byte(LidarLiteI2cAddress,0x1c,0x20);
		break;
		case 3: //  High noise, high sensitivity: Pulls decision criteria into the
				//  noise, allows more false detections, increses sensitivity
		  IIC_Write_1Byte(LidarLiteI2cAddress,0x1c,0x60);
		break;
		default:
			break;
  }
}

void LIDARLite_beginContinuous(u8 modePinLow, u8 interval, u8 numberOfReadings,u8 LidarLiteI2cAddress)
{
	//  Register 0x45 sets the time between measurements. 0xc8 corresponds to 10Hz
	//  while 0x13 corresponds to 100Hz. Minimum value is 0x02 for proper
	//  operation.
	IIC_Write_1Byte(LidarLiteI2cAddress,0x45,interval);
	//  Set register 0x04 to 0x20 to look at "NON-default" value of velocity scale
	//  If you set bit 0 of 0x04 to "1" then the mode pin will be low when done
	if(modePinLow)
	{
		IIC_Write_1Byte(LidarLiteI2cAddress,0x04,0x21);
	}
	else
	{
		IIC_Write_1Byte(LidarLiteI2cAddress,0x04,0x20);
	}
	//  Set the number of readings, 0xfe = 254 readings, 0x01 = 1 reading and
	//  0xff = continuous readings
	IIC_Write_1Byte(LidarLiteI2cAddress,0x11,numberOfReadings);
	//  Initiate reading distance
	IIC_Write_1Byte(LidarLiteI2cAddress,0x00,0x04);
}

u16 LIDARLite_distance(u8 stablizePreampFlag, u8 takeReference, u8 LidarLiteI2cAddress)
{
	// Array to store high and low bytes of distance
	u8 distanceArray[2];
	u16 distance = 0;
	
	if(stablizePreampFlag)
	{
		// Take acquisition & correlation processing with DC correction
		IIC_Write_1Byte(LidarLiteI2cAddress,0x00,0x04);
	}else
	{
		// Take acquisition & correlation processing without DC correction
		IIC_Write_1Byte(LidarLiteI2cAddress,0x00,0x03);
	}

	// Read two bytes from register 0x8f. (See autoincrement note above)
	IIC_Read_nByte(LidarLiteI2cAddress,0x8f,2,distanceArray);
	// Shift high byte and add to low byte
	distance = (distanceArray[0] << 8) + distanceArray[1];
	return(distance);
}

u16 LIDARLite_distanceContinuous(u8 LidarLiteI2cAddress)
{
	u8 distanceArray[2]; // Array to store high and low bytes of distance
	u16 distance = 0;
	
	IIC_Read_nByte(0x62,0x8f,2,distanceArray); // Read two bytes from register 0x8f. (See autoincrement note above)
	distance = (distanceArray[0] << 8) + distanceArray[1]; // Shift high byte and add to low byte
	return(distance);
}

void LIDARLite_scale(u8 velocityScalingValue, u8 LidarLiteI2cAddress)
{
	//  Array of velocity scaling values
	u8 scale[] = {0xc8, 0x50, 0x28, 0x14};
	//  Write scaling value to register 0x45 to set
	IIC_Write_1Byte(LidarLiteI2cAddress,0x45,scale[velocityScalingValue]);
}

u8 LIDARLite_velocity(u8 LidarLiteI2cAddress)
{
	u8 velocityArray;
	
	//  Write 0xa0 to 0x04 to switch on velocity mode
	IIC_Write_1Byte(LidarLiteI2cAddress,0x04,0xa0);
	//  Write 0x04 to register 0x00 to start getting distance readings
	IIC_Write_1Byte(LidarLiteI2cAddress,0x00,0x04);
	//  Array to store bytes from read function
	
	//  Read 1 byte from register 0x09 to get velocity measurement
	IIC_Read_1Byte(LidarLiteI2cAddress,0x09,&velocityArray);
	//  Convert 1 byte to char and then to int to get signed int value for velo-
	//  city measurement
	return(velocityArray);
}


u8 LIDARLite_signalStrength(u8 LidarLiteI2cAddress)
{
	//  Array to store read value
	u8 signalStrengthArray;
	
	//  Read one byte from 0x0e
	IIC_Read_1Byte(LidarLiteI2cAddress,0x0e,&signalStrengthArray);
	return(signalStrengthArray);
}

void LIDARLite_correlationRecordToArray(int *arrayToSave, u8 numberOfReadings, u8 LidarLiteI2cAddress)
{
	// Array to store read values
    u8 correlationArray[2];
    // Var to store value of correlation record
    int correlationValue = 0;
	
    //  Selects memory bank
    IIC_Write_1Byte(LidarLiteI2cAddress,0x5d,0xc0);
    // Sets test mode select
    IIC_Write_1Byte(LidarLiteI2cAddress,0x40, 0x07);
    for(u8 i = 0; i<numberOfReadings; i++)
	{
      // Select single byte
      IIC_Read_nByte(LidarLiteI2cAddress,0xd2,2,correlationArray);
      //  Low byte is the value of the correlation record
      correlationValue = correlationArray[0];
      // if upper byte lsb is set, the value is negative
      if(correlationArray[1] == 1)
	  {
        correlationValue |= 0xff00;
      }
      arrayToSave[i] = correlationValue;
    }
    // Send null command to control register
    IIC_Write_1Byte(LidarLiteI2cAddress,0x40,0x00);
}

void LIDARLite_correlationRecordToSerial(u8 separator, u8 numberOfReadings, u8 LidarLiteI2cAddress)
{
	// Array to store read values
	u8 correlationArray[2];
	// Var to store value of correlation record
	int correlationValue = 0;
	
	//  Selects memory bank
	IIC_Write_1Byte(LidarLiteI2cAddress,0x5d,0xc0);
	// Sets test mode select
	IIC_Write_1Byte(LidarLiteI2cAddress,0x40, 0x07);
	for(int i = 0; i<numberOfReadings; i++)
	{
		// Select single byte
		IIC_Read_nByte(LidarLiteI2cAddress,0xd2,2,correlationArray);
		//  Low byte is the value of the correlation record
		correlationValue = correlationArray[0];
		// if upper byte lsb is set, the value is negative
		if(correlationArray[1] == 1)
		{
		  correlationValue |= 0xff00;
		}
	}
	// Send null command to control register
	IIC_Write_1Byte(0x40,0x00,LidarLiteI2cAddress);
}

u8 LIDARLite_changeAddress(u8 newI2cAddress,  u8 disablePrimaryAddress, u8 currentLidarLiteAddress)
{
	//  Array to save the serial number
	u8 serialNumber[2];
	u8 newI2cAddressArray[1];

	//  Read two bytes from 0x96 to get the serial number
	IIC_Read_nByte(currentLidarLiteAddress,0x96,2,serialNumber);
	//  Write the low byte of the serial number to 0x18
	IIC_Write_1Byte(currentLidarLiteAddress,0x18,serialNumber[0]);
	//  Write the high byte of the serial number of 0x19
	IIC_Write_1Byte(currentLidarLiteAddress,0x19,serialNumber[1]);
	//  Write the new address to 0x1a
	IIC_Write_1Byte(currentLidarLiteAddress,0x1a,newI2cAddress);


	while(newI2cAddress != newI2cAddressArray[0]){
		IIC_Read_1Byte(currentLidarLiteAddress,0x1a,newI2cAddressArray);
	}
	//  Choose whether or not to use the default address of 0x62
	if(disablePrimaryAddress){
		IIC_Write_1Byte(currentLidarLiteAddress,0x1e,0x08);
	}else{
		IIC_Write_1Byte(currentLidarLiteAddress,0x1e,0x00);
	}

	return newI2cAddress;
}


