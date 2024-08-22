/*
 * ATCommands.h
 *
 *  Created on: Aug 9, 2024
 *      Author: Owner
 */

#ifndef INC_ATCOMMANDS_H_
#define INC_ATCOMMANDS_H_

#define AT_RFS                "AT+RFS\r\n"
/*
#define AT_SET_DEVEUI         "AT+DEUI=00:00:00:00:00:02:63:22\r\n"
#define AT_SET_APPEUI         "AT+APPEUI=62:00:34:9E:58:AF:A4:6B\r\n"
#define AT_SET_APPKEY         "AT+APPKEY=5E:31:39:AF:3A:6C:27:E6:80:05:EA:71:CE:34:02:57\r\n"
#define AT_SET_NWKKEY         "AT+NWKKEY=5E:31:39:AF:3A:6C:27:E6:80:05:EA:71:CE:34:02:57\r\n"
#define AT_SET_APPSKEY         "AT+APPSKEY=5E:31:39:AF:3A:6C:27:E6:80:05:EA:71:CE:34:02:57\r\n"
#define AT_SET_NWKSKEY         "AT+NWKSKEY=5E:31:39:AF:3A:6C:27:E6:80:05:EA:71:CE:34:02:57\r\n"
*/
#define AT_SET_DEVEUI         "AT+DEUI=00:00:00:00:00:02:63:23\r\n"
#define AT_SET_APPEUI         "AT+APPEUI=34:B2:9A:5C:BE:ED:E9:38\r\n"
#define AT_SET_APPKEY         "AT+APPKEY=DE:79:A2:AC:39:31:A6:58:F8:08:26:47:6B:8E:A3:E6\r\n"
#define AT_SET_NWKKEY         "AT+NWKKEY=DE:79:A2:AC:39:31:A6:58:F8:08:26:47:6B:8E:A3:E6\r\n"
#define AT_SET_APPSKEY         "AT+APPSKEY=DE:79:A2:AC:39:31:A6:58:F8:08:26:47:6B:8E:A3:E6\r\n"
#define AT_SET_NWKSKEY         "AT+NWKSKEY=DE:79:A2:AC:39:31:A6:58:F8:08:26:47:6B:8E:A3:E6\r\n"

#define AT_SET_ADR       	"AT+ADR=0\r\n"
#define AT_SAVE_TO_FLASH 	"AT+CS\r\n"
#define AT_RST_MCU 		    "ATZ\r\n"

#define AT_JOIN_OTAA   		"AT+JOIN=1\r\n"
#define AT_ADR_DISABLE   	"AT+ADR=0\r\n"
#define AT_ADR_ENABLE   	"AT+ADR=1\r\n"
//#define AT_SEND_            "AT+SEND=1:0:"

typedef enum{
	UART_IDLE = 0,
	AT_RESPONSE_CAPTURE_OK,
	AT_RESPONSE_CAPTURE_NVM_STORED,
	AT_RESPONSE_CAPTURE_RESET,
	AT_RESPONSE_CAPTURE_JOIN,
	AT_RESPONSE_CAPTURE_SEND_OK,

}LPUARTState;



#endif /* INC_ATCOMMANDS_H_ */
