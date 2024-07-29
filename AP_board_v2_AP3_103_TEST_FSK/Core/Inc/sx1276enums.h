/*
 * USER.h
 * Created on: 21-Mar-2024
 * Author: Le Huu An
 */

#ifndef SX1276ENUMS_H_
#define SX1276ENUMS_H_

#include<main.h>
/*Include the type of stm32*/
#include "stm32f1xx_hal.h"

#include "stdbool.h"


/*DEFINE*/




			

/************************************************************************************/
/*DECLARE STRUCT*/

/*!
 * Radio driver internal state machine states definition
 */
typedef enum RadioState
{
    RF_IDLE = 0,
    RF_RX_RUNNING,
    RF_TX_RUNNING,
    RF_CAD,
}RadioState_t;

/*!
 * Shaping filter values
 */
typedef enum ShapingFilter
{
    NONE = 0,
    BT_1_0,
    BT_0_5,
    BT_0_3
}ShapingFilter_t;

/*!
 *    Type of the modem. [LORA / FSK]
 */
typedef enum ModemType
{
    MODEM_FSK = 0,
    MODEM_LORA
}RadioModems_t;

typedef struct
{
    RadioModems_t modem;
    unsigned char power;
    ShapingFilter_t filtertype;
    unsigned int fdev;
    unsigned int bandwidth;
    unsigned int datarate;
    unsigned char coderate;
    unsigned short preambleLen;
    unsigned short payloadLen;
    bool whitening;
    bool fixLen;
    bool crcOn;
    bool freqHopOn;
    unsigned char hopPeriod;
    bool iqInverted;
    unsigned int timeout;
}TxConfig_t;

typedef struct
{
    RadioModems_t modem;
    unsigned char power;
    ShapingFilter_t filtertype;
    unsigned int fdev;
    unsigned int bandwidth;
    unsigned int bandwidthAfc;
    unsigned int datarate;
    unsigned char coderate;
    unsigned short preambleLen;
    unsigned short payloadLen;
    bool whitening;
    bool fixLen;
    bool crcOn;
    bool freqHopOn;
    unsigned char hopPeriod;
    bool iqInverted;
    unsigned int timeout;
}RxConfig_t;
		
/*DECLARE FUNCTION*/



/************************************************************************************/
#endif /* SX1276ENUMS_H_ */

