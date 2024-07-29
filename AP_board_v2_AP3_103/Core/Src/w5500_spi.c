#include<w5500_spi.h>
#include "wizchip_conf.h"
#include "string.h"
#include "socket.h"
#include "w5500.h"



///*
//NOTE: 
//	
//*/
///**********************************************************************************************************************************/

/*Extern*/
extern SPI_HandleTypeDef hspi2;

extern void my_printf(const char* format, ...);



/**********************************************************************************************************************************/
/*Declare variable*/
	wiz_NetInfo gwIZNETINFO = {
		.mac 	= {0xC0, 0xA8, 0x01, 0x67, 0x04, 0x57},	
		.ip 	= {192, 168, 1, 103},				
		.sn 	= {255, 255, 225 , 0},					
		.gw 	= {192, 168, 1, 1},
		.dns  = {8, 8, 8, 8},
		.dhcp = NETINFO_STATIC };	

	uint8_t destination_ip[] = {192,168,1,100};
	uint16_t destination_port = 1111;
	
	uint8_t rx_tx_buff_sizes[] = {2,2,2,2,2,2,2,2};
	uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2}, {2,2,2,2,2,2,2,2} };
		
		
//		.mac 	= {0xC0, 0xA8, 0x01, 0x67, 0x04, 0x57},	
//		.ip 	= {192, 168, 1, 103},	
		
//		.mac 	= {0xC0, 0xA8, 0x01, 0x66, 0x04, 0x57},	
//		.ip 	= {192, 168, 1, 102},

/**********************************************************************************************************************************/
/*FUNCTION*/
/*____________________________________________________________________________________*/
	static void PHYStatusCheck(void){
		uint8_t tmp;

		my_printf("\r\nChecking Ethernet Cable Presence...");
		ctlwizchip(CW_GET_PHYCONF, (void*) &tmp);
		
		if(tmp == PHY_LINK_OFF)
		{		
			my_printf("NO Cable Connected!");
			HAL_Delay(1500);
		}
	
	 // while(tmp == PHY_LINK_OFF);
		
		my_printf("Good! Cable got connected!");
	}
/*____________________________________________________________________________________*/
	static void PrintPHYConf(void){
		wiz_PhyConf phyconf;
		
		ctlwizchip(CW_GET_PHYCONF, (void*)&phyconf);
		if(phyconf.by == PHY_CONFBY_HW)
		{
			my_printf("\n\rPHY COnfigured by Hardware Pins");
		}
		else
		{
			my_printf("\n\rPHY COnfigured by Registers");
		}
		if(phyconf.mode == PHY_MODE_AUTONEGO)
		{
			my_printf("\n\rAutonegotiation Enable");
		}
		else
		{
			my_printf("\n\rAutonegotiation NOT Enable");
		}
		if(phyconf.duplex == PHY_DUPLEX_FULL)
		{
			my_printf("\n\rDuplex Mode: Full");
		}
		else
		{
			my_printf("\n\rDuplex Mode: Hafl");
		}
		if(phyconf.speed == PHY_SPEED_10)
		{
			my_printf("\n\rSpeed: 10Mbps");
		}
		else
		{
			my_printf("\n\rSpeed: 100Mbps");
		}
	}
/*____________________________________________________________________________________*/
	void W5500_Select(void) {
			HAL_GPIO_WritePin(SCS_ETH_GPIO_Port, SCS_ETH_Pin, GPIO_PIN_RESET);
	}
/*____________________________________________________________________________________*/
	void W5500_Deselect(void) {
			HAL_GPIO_WritePin(SCS_ETH_GPIO_Port, SCS_ETH_Pin, GPIO_PIN_SET);
	}
/*____________________________________________________________________________________*/
	void W5500_ReadBuff(uint8_t *buff, uint16_t len) 
	{
			HAL_SPI_Receive(&hspi2, buff, len, HAL_MAX_DELAY);
	}
/*____________________________________________________________________________________*/
	void W5500_WriteBuff(uint8_t *buff, uint16_t len) {
			HAL_SPI_Transmit(&hspi2, buff, len, HAL_MAX_DELAY);
	}
/*____________________________________________________________________________________*/
	uint8_t W5500_ReadByte(void)
	{
		uint8_t byte;
		W5500_ReadBuff(&byte,sizeof(byte));
		return byte;
	}
/*____________________________________________________________________________________*/
	void W5500_WriteByte(uint8_t byte)
	{
		W5500_WriteBuff(&byte,sizeof(byte));
	}
/*____________________________________________________________________________________*/
	void w5500_Init(void){
		my_printf("\r\nStart New Session!\r\n");
		HAL_GPIO_WritePin(RESETETH_GPIO_Port,RESETETH_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(RESETETH_GPIO_Port,RESETETH_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);

		reg_wizchip_cs_cbfunc(W5500_Select, W5500_Deselect);
		reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
		reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);
					
		wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);
		wizchip_setnetinfo(&gwIZNETINFO);
 }
/*____________________________________________________________________________________*/
	uint8_t w5500_Config(void){
		uint8_t result = 0;
		ctlnetwork(CN_SET_NETINFO, (void*) &gwIZNETINFO);
		HAL_Delay(1000);
		if(ctlwizchip(CW_INIT_WIZCHIP, (void*)memsize) == -1)
		{
			my_printf("WIZCHIP Initialized Failed! \r\n");
			//while(1);
			result = 0;
		}
		else{
			my_printf("WIZCHIP Initialized Success! \r\n");
		result = 1;
		}
		PHYStatusCheck();
		PrintPHYConf();
		return (result) ? (SUCCESS) : (FAIL);
	}
/*____________________________________________________________________________________*/
	uint8_t creat_a_socket(void){
		if(socket(1,Sn_MR_TCP, 0, 0) == 1){
			my_printf("\r\nSocket Created Successfully!");
			return SUCCESS;
		}
		else{
			my_printf("\r\n Cannot create socket!");
			//while(1);
			return FAIL;
		}
	}
/*____________________________________________________________________________________*/
	uint8_t connect_toServer(void){
		my_printf("\r\n Connecting to server: %d.%d.%d.%d @ TCP Port: 1111");
		if(connect(1,destination_ip,destination_port) == SOCK_OK){
			my_printf("\r\nConnected with server!");
			return SUCCESS;
		}
		else{
			my_printf("\r\n Cannot connected with server!");
			return FAIL;
		}
	}
/*____________________________________________________________________________________*/


	
	
	
	
/*____________________________________________________________________________________*/





/*____________________________________________________________________________________*/




/*____________________________________________________________________________________*/

//				int len = recv(1, recive_buff, RECIVE_BUF_SIZE);
//				
//				if(len == SOCKERR_SOCKSTATUS)
//				{
//					my_printf("Client has disconnected\r\n");
//					my_printf("*** SESSION ONVER!!!****\r\n ");
//					break;
//				}
//				
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


//char msg[] = "Hello\r\n";

//if(send(1,(uint8_t *)msg, sizeof(msg))<=SOCK_ERROR){
//	my_printf("\r\nSending Failed!");
//}
//else{
//	my_printf("\r\nSending Success!");
//}

