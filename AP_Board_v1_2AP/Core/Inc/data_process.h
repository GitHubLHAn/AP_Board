/*
 * USER.h
 * Created on: 21-Mar-2024
 * Author: Le Huu An
 */

#ifndef DATA_PROCESS_H_
#define DATA_PROCESS_H_

#include<main.h>
/*Include the type of stm32*/
#include "stm32f1xx_hal.h"


/*DEFINE*/
#define Wrong_Header 0xE0
#define Wrong_CS 0xE1

#define MESS_OK 0xBB



			

/************************************************************************************/
/*DECLARE STRUCT*/
		
/*DECLARE FUNCTION*/

uint8_t handle_RX_fromServer(uint8_t *rx_fromServer, uint8_t *tx_buff);
uint8_t check_RX(uint8_t *rx_fromServer, uint8_t *tx_toRobot);

/************************************************************************************/
#endif /* DATA_PROCESS_H */

//uint8_t sr = 0x00;
//		
//		do
//		{
//			sr = getSn_SR(1);
//		}
//		while(sr!=0x17 && sr!=0x00);
//		
//		if(sr == 0)
//		{
//			my_printf("Some error occurred on server socket. Please restart!\r\n");
//			while(1);
//		}
//		
//		if(sr == 0x17)
//		{
//			my_printf("A client connected!\r\n");
//			my_printf("Waiting for client data...!\r\n");
//			
//			while(1)
//			{
//				int len = recv(1, recive_buff, RECIVE_BUF_SIZE);
//				if(len == SOCKERR_SOCKSTATUS)
//				{
//					my_printf("Client has disconnected\r\n");
//					my_printf("*** SESSION ONVER!!!****\r\n ");
//					break;
//				}
//				recive_buff[len] = '\0';
//				my_printf("Received %d bytes from client \r\n", len);
//				my_printf("Data received: %s\r\n",recive_buff);
//				
//				send(1,(uint8_t*)"[",1);
//				send(1,recive_buff,len);
//				send(1,(uint8_t*)"]",1);
//				
//				my_printf("\r\nECHO sent back to client\r\n");
//				
//				if(strcmp((char*)recive_buff,"QUIT") == 0)
//				{
//					my_printf("Received QUIT command for client\r\n");
//					my_printf("Disconnecting...\r\n");
//					my_printf("*** SESSION ONVER!!!****\r\n ");
//					disconnect(1);
//					break;
//				}
//			}
//		}
