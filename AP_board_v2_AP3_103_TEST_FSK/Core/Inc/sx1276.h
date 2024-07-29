/*
 * USER.h
 * Created on: 21-Mar-2024
 * Author: Le Huu An
 */

#ifndef SX1276_H_
#define SX1276_H_

#include<main.h>
/*Include the type of stm32*/
#include "stm32f1xx_hal.h"

#include "stdbool.h"


/*DEFINE*/

#define SX1276_OK	   0
#define SX1276_ERROR   1

// 2s timeout (in ms)
#define TIMEOUT 2000
#define TRANSMIT_TIMEOUT		2000
#define RECEIVE_TIMEOUT			2000

/*!
 * SX1276 definitions
 */
#define XTAL_FREQ				32000000
#define FREQ_STEP				61.03515625f

/*!
 * FSK bandwidth definition
 */
typedef struct
{
	uint32_t bandwidth;
	uint8_t	 RegValue;
}FskBandwidth_t;

//typedef struct
//{
//    unsigned long CSPort;
//    unsigned long CSPin;
//    unsigned long RESETPort;
//    unsigned long RESETPin;
//    unsigned long DIO0Port;
//    unsigned long DIO0Pin;
//    void(*callback)(  );

//} SX1276Pins;

typedef struct LoRa_setting{
	
	// Hardware setings:
	GPIO_TypeDef*		CS_port;
	uint16_t		CS_pin;
	GPIO_TypeDef*		reset_port;
	uint16_t		reset_pin;
	GPIO_TypeDef*		DIO0_port;
	uint16_t		DIO0_pin;
	SPI_HandleTypeDef*	hSPIx;
	
	// Module settings:
	int			current_mode;
	int 			frequency;
	uint8_t			spredingFactor;
	uint8_t			bandWidth;
	uint8_t			crcRate;
	uint16_t		preamble;
	uint8_t			power;
	uint8_t			overCurrentProtection;
	
} FSK_t;
			

/************************************************************************************/
/*DECLARE STRUCT*/
		
/*DECLARE FUNCTION*/
//void setFrequency(unsigned long);
//unsigned long getFrequency();
//signed short getFrequencyError();

//unsigned char setIdleMode(bool);
//void setTxConfig(TxConfig_t*);


//writeRegister(FSK_t* _FSK, uint8_t address, uint8_t *value, uint8_t length);


//bool sendFSK(unsigned char *buffer, unsigned char size);

/************************************************************************************/
#endif /* DATA_PROCESS_H */

