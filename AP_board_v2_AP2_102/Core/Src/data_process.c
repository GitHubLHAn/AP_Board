#include<data_process.h>
#include <stdlib.h>


/*
NOTE: 
	
*/
/**********************************************************************************************************************************/

/*Extern*/



/**********************************************************************************************************************************/
/*Declare variable*/
		

/**********************************************************************************************************************************/
/*FUNCTION*/

/*____________________________________________________________________________________*/
	unsigned char Check_CRC8(unsigned char *data, uint16_t data_length) {
			uint16_t length = data_length - 1;
			unsigned char crc = 0;
			uint16_t i, j;
		
			for (i = 0; i < length; i++){
				crc ^= data[i];
				for (j = 0; j < 8; j++){
					if (crc & 0x80)
							crc = (crc << 1) ^ 0x07;
					else 
							crc <<= 1;
					crc &= 0xFF;
				}
			}
			return crc;
	}
/*____________________________________________________________________________________*/
	uint8_t handle_RX_fromServer(uint8_t *rx_fromServer, uint8_t *tx_buff){
		if(rx_fromServer[0] == 0xFF){
			tx_buff[0] = 0xFF;
			tx_buff[1] = rx_fromServer[1];
			tx_buff[2] = rx_fromServer[2];
			tx_buff[3] = rx_fromServer[3];
			tx_buff[4] = rand() % 100;
			
			uint8_t huong = rand() % 4;
			if(huong == 0)
				tx_buff[5] = 0x0A;
			else if(huong == 1)
				tx_buff[5] = 0x0B;
			else if(huong == 2)
				tx_buff[5] = 0x0C;
			else 
				tx_buff[5] = 0x0D;
			
			// x position
			tx_buff[6] = 0x00;
			tx_buff[7] = rand() % 0xFF;
			tx_buff[8] = rand() % 0xFF;
			
			// y position
			tx_buff[9] = 0x00;
			tx_buff[10] = rand() % 0xFF;
			tx_buff[11] = rand() % 0xFF;
			
			//RDID
			for(uint8_t i = 12; i< 19; i++)
				tx_buff[i] = rand() % 0xFF;
			
			//Checksum
				tx_buff[19] = Check_CRC8(tx_buff, 20);
			return MESS_OK;
		}
		else return Wrong_Header;
	
	}
	
/*____________________________________________________________________________________*/
		uint8_t check_RX(uint8_t *rx_from, uint8_t *tx_to){
			//check header
			if(rx_from[0] == 0xFF){
				//Checksum
				uint8_t check_byte = 0x00;
				check_byte = Check_CRC8(rx_from, 20);
				if(check_byte == rx_from[20-1]){
					for(uint8_t i=0; i<20; i++)
						tx_to[i] = rx_from[i];
					return MESS_OK;
				}
				else return Wrong_CS;		
			}
			else return Wrong_Header;	
		}
	
	
	
	
	
			// SENDING DATA - - - - - - - - - - - - - - - - - - - - - - - - -
//		send_data[0] = 0x3B; // MY ADDRESS
//		for(int i=0; i<20; i++)
//			send_data[i+1] = rand()%0xFF;
//		LoRa_transmit(&myLoRa, send_data, 20, 100);
//		HAL_Delay(100);
