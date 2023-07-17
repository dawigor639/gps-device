/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> //sscanf
#include <time.h> //time struct
#include <stdint.h> //uint
#include <string.h>
#include <stdbool.h> //bool;
#include <stdlib.h> //atoi
#include <math.h> //haversine

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * Structure that contains information about the subscriber
 */
struct Subscriber {
    /** Character array representing phone number of a subscriber */
    char address[17]; 
    /** Determines whether the zone was exceeded  */
    int state;
    /** Determines the safe zone, includes information about its latitude, longitude and radius */
    float circle[3];
};

/**
 * Structure that contains current device properties
 */
struct Device {
    /** Array of two floating-point numbers representing the latitude and longitude of the device current position */
    float position[2];
    /** Time related to the current position of the device, corresponds to the Unix timestamp */
    time_t unixTime;
    /** Period of time between device cycles*/
    int interval;
    /** Array of subscribers assigned to the device */
    struct Subscriber subscribers[10];
    /** Represents the number of subscribers assigned to the device */
    int count;
    /** Unique identification number assigned to the SIM card */
    char iccid[23];
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** Maximum number of subscribers allowed */
#define MAX_PHONES 10
/** Maximum size of an SMS message */
#define MAX_SMS_SIZE 160
/** Maximum length of an ICCID (Integrated Circuit Card Identifier) */
#define MAX_ICCID_LEN 22
/** Maximum length of a phone number */
#define MAX_PHONE_LEN 16
/** Maximum number of settings */
#define MAX_SETTINGS 3
/** Maximum length of time */
#define MAX_TIME_LEN 14
/** Maximum length of a circle */
#define MAX_CIRCLE_LEN 29
/** Length of a formatted time string */
#define FORMATTED_TIME_LEN 20
/** Length of a UNIX timestamp */
#define UNIX_TIME_LEN 10
/** Maximum length of a file name */
#define MAX_FILE_LEN 539
/** Maximum size of a GPS response */
#define MAX_GPS_RESPONSE 120
/** Maximum length of an index */
#define MAX_INDEX_LEN 5
/** Maximum size of a CMGL (command response) record */
#define MAX_CMGL_RECORD_SIZE 260
/** Maximum size of an AT command response */
#define MAX_AT_RESPONSE 100
/** Minimum value for latitude */
#define LATITUDE_MIN -90
/** Maximum value for latitude */
#define LATITUDE_MAX 90
/** Minimum value for longitude */
#define LONGITUDE_MIN -180
/** Maximum value for longitude */
#define LONGITUDE_MAX 180
/** Minimum value for radius */
#define RADIUS_MIN 50
/** Maximum value for radius */
#define RADIUS_MAX 6371000
/** Minimum value for UNIX timestamp */
#define UNIX_TIME_MIN 1678970022
/** Maximum value for UNIX timestamp */
#define UNIX_TIME_MAX 2147483647
/** Length of a request ID */
#define REQUEST_ID_LEN 14
/** Default latitude value */
#define LATITUDE_DEFAULT 52
/** Default longitude value */
#define LONGITUDE_DEFAULT 19

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/** Configuring and controlling a real-time clock (RTC) peripheral in a microcontroller */
RTC_HandleTypeDef hrtc;
/** Controlling UART communication */
UART_HandleTypeDef huart1;
/** Configuring and managing data transfers using DMA (Direct Memory Access). */
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/** Used to indicate whether the UART is ready to send or receive data */
volatile bool uartReady = 0;
/** Used to indicate UART communication errors */
volatile uint8_t uartError = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/**
 * This is a callback function, when UART receive event occurs it sets a flag indicating that the
 * UART is ready.
 * @param huart UART handle structure.
 * @param Size Amount of data to be received.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	uartReady=1;
}

/**
 * This callback function sets a flag indicating an error occurred during UART communication.
 * @param huart UART handle structure.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uartError=1;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * The function reads data from UART using DMA and stores it in a buffer.
 * 
 * @param data Buffer where the received data will be stored.
 * @param size Number of bytes to be read.
 */
void uartRead(uint8_t* data, const uint16_t size) {
	uartError=0;
	uartReady=0;
	HAL_UART_DMAStop(&huart1); //stop possible previous transfer
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, data, size);
}

/**
 * The function sends data through UART using the HAL library.
 * 
 * @param data Buffer that contains the data to be transmitted over UART.
 * @param size Number of bytes of data to be transmitted.
 */
void uartSend(uint8_t* data, const uint16_t size) {
	HAL_UART_Transmit(&huart1, data, size,500); //send atCommand
}

/**
 * The function checks if UART is available and returns a boolean value accordingly.
 * 
 * @return UART readiness status.
 */
bool uartAvailable() {
	if(uartReady) {
		uartReady=0;
		return 1;
	}
	else {
		return 0;
	}
}

/**
 * The function checks if there are any UART errors and returns a boolean value indicating the presence
 * of errors.
 * 
 * @return Status of UART errors.
 */
bool uartErrors() {
	if(uartError) {
		uartError=0;
		return 1;
	}
	else {
		return 0;
	}
}

/**
 * This function sends an AT command over UART and waits for a response within a specified timeout,
 * returning true if the expected answer is found in the response and false otherwise.
 * 
 * @param atCommand AT command to be sent to a device via UART communication.
 * @param timeout Maximum time in milliseconds that the function will wait for a response from the
 * module before timimeout.
 * @param expectedAnswer Text expected in the response.
 * @return Value indicating whether the desired answer was found in the
 * response of the module within the specified timeout period.
 */
bool sendAtCommand(const char* atCommand, const uint32_t timeout, const char* expectedAnswer) {
    uint32_t startTime = 0;
    // Buffer for response
    char receiveBuffer[MAX_AT_RESPONSE]={0}; 
    // Send atCommand
    uartSend((uint8_t *)atCommand, strlen(atCommand)); 
    // Start reading
    uartRead((uint8_t*)receiveBuffer, MAX_AT_RESPONSE); 
    startTime = HAL_GetTick();
    // Wait for the answer
    while( (HAL_GetTick()-startTime) < timeout ){
        if(uartAvailable()){
                // Check if the desired answer is in the response
                if (strstr(receiveBuffer, expectedAnswer) != NULL) {
                    return 1;
                }
				else {
					return 0;
				}
    	}
    }
    return 0;
}

/**
 * This function sends an AT command over UART and waits for a response within a specified timeout,
 * checking if the response contains an expected answer. Function also handles communication errors.
 * 
 * @param atCommand AT command to be sent to a device via UART communication.
 * @param timeout Maximum time in milliseconds that the function will wait for a response from the
 * module before timimeout.
 * @param expectedAnswer Text expected in the response.
 * @return Value indicating whether the desired answer was found in the
 * response of the module within the specified timeout period.
 */
bool sendAtCommand2(const char* atCommand, const uint32_t timeout, const char* expectedAnswer) {
    uint32_t startTime = 0;
    // Buffer for response
    char receiveBuffer[MAX_AT_RESPONSE]={0}; 
    // Send atCommand
    uartSend((uint8_t *)atCommand, strlen(atCommand));
    // Start reading
    uartRead((uint8_t*)receiveBuffer, MAX_AT_RESPONSE);
    startTime = HAL_GetTick();
    // Wait for the answer
    while( (HAL_GetTick()-startTime) < timeout ){

    	if(uartErrors()) {
    		 memset(receiveBuffer,'\0',strlen(receiveBuffer));
             // Start reading
    		 uartRead((uint8_t*)receiveBuffer, MAX_AT_RESPONSE);
    	}
    	else {
			if(uartAvailable()){
					// Check if the desired answer is in the response
					if (strstr(receiveBuffer, expectedAnswer) != NULL) {
						return 1;
					}
					else {
						return 0;
					}
			}
    	}
    }
    return 0;
}

/**
 * This function sends an AT command over UART and waits for a response that matches an expected answer
 * within a specified timeout period. Function writes response to the receiveBuffer. 
 * 
 * @param atCommand AT command to be sent to a device via UART communication.
 * @param timeout Maximum time in milliseconds that the function will wait for a response from the
 * module before timimeout.
 * @param expectedAnswer Text expected in the response.
 * @param receiveBuffer Buffer in which received data will be stored.
 * @param size Number of characters that can be stored in the receiveBuffer.
 * 
 * @return Value indicating whether the desired answer was found in the
 * response of the module within the specified timeout period.
 */
bool getSerialResponse(const char* atCommand, const uint32_t timeout, const char* expectedAnswer, char* receiveBuffer , const uint16_t size) {
    uint32_t startTime = 0;
    // Send atCommand
    uartSend((uint8_t *)atCommand, strlen(atCommand));
    // Start reading
    uartRead((uint8_t*)receiveBuffer, size);
    startTime = HAL_GetTick();
    // Wait for the answer
    while( (HAL_GetTick()-startTime) < timeout ){
        if(uartAvailable()){
				// Check if the desired answer is in the response
                if (strstr(receiveBuffer, expectedAnswer) != NULL)
                {
                    return 1;
                }
				else {
					return 0;
				}
    	}
    }
    return 0;
}
/**
 * This function reads data from UART and waits for a response within a specified timeout
 * period. Function writes response to the receiveBuffer. 
 * 
 * @param timeout Maximum time in milliseconds that the function will wait for a response from the
 * @param receiveBuffer Buffer in which received data will be stored.
 * @param size Number of characters that can be stored in the receiveBuffer.
 * 
 * @return Value indicating whether the response of the module was received within the specified timeout period.
 */
bool getSerialResponse2(const uint32_t timeout, char* receiveBuffer , const uint16_t size) {
    uint32_t startTime = 0;
    // Start reading
    uartRead((uint8_t*)receiveBuffer, size); 
    startTime = HAL_GetTick();
    // Wait for the answer
    while( (HAL_GetTick()-startTime) < timeout ){
        if(uartAvailable()){
        	return 1;
    	}
    }
    return 0;
}

/**
 * The function checks connection with module by sending an AT command specified number of
 * times.
 * 
 * @param maxAttempts The maximum number of attempts to send the "AT" command.
 * 
 * @return Indicates whether a positive response has been received.
 */
bool waitConnect(const uint8_t maxAttempts) {
	for(int i=0; i<maxAttempts;i++) {
		if (sendAtCommand("AT\r",2000,"OK"))
			return 1;
	}
	return 0;
}


/**
 * The function extracts a substring from a given input string using two delimiters and stores it in an
 * output string.
 * 
 * @param output A pointer to a character array where the extracted text will be stored.
 * @param input A pointer to a character array containing the input string to be searched for the
 * delimiters.
 * @param outputSize The maximum size of the output buffer
 * @param delimiter1 The first delimiter used to locate the beginning of the desired text in the input
 * string.
 * @param delimiter2 The second delimiter used to extract a substring from the input string. If an
 * empty string is passed as delimiter2, the function will extract the substring from the first
 * delimiter until the end of the input string.
 * 
 * @return Value indicating whether the operation was successful or not.
 */
bool getText(char* output, const char* input, const uint32_t outputSize, const char* delimiter1, const char* delimiter2) {
    int delimLen = strlen(delimiter1);
    uint32_t charNum = 0;
    char* p1, * p2;
    p1 = strstr(input, delimiter1);
    if (p1) {

        if (strcmp(delimiter2, "") == 0)
        {
            charNum = (uintptr_t)(&input[strlen(input)] - p1 - delimLen);
            if (charNum <= outputSize - 1) { //data is not longer than outputSize-1
                strncpy(output, p1 + delimLen, charNum);
                output[charNum] = '\0';
                return 1;
            }
            else {
                return 0;
            }
        }

        p2 = strstr(p1 + delimLen, delimiter2);
        if (p2) {
            charNum = (uintptr_t)(p2 - p1 - delimLen);
            if (charNum <= outputSize - 1) { //data is not longer than outputSize-1
                strncpy(output, p1 + delimLen, charNum);
                output[charNum] = '\0';
                return 1;
            }
            else {
                return 0;
            }
        }
    }
    return 0;
}

/**
 * This function reads unread SMS messages and stores them in a provided buffer.
 * 
 * @param smsList A pointer to a character array where the SMS list will be stored.
 * @param smsListSize The maximum size of the character array `smsList` that will store the SMS list.
 * 
 * @return A boolean value indicating whether the function call was successful or not.
 */
bool readSmsList(char * smsList, const uint16_t smsListSize){
	return getSerialResponse("AT+CMGL=\"REC UNREAD\",1\r",20000,"+CMGL:",smsList,smsListSize);
}

/**
 * The function deletes an SMS message with a specified index using an AT command.
 * 
 * @param index Index of the SMS message to be deleted.
 * 
 * @return a boolean value, which indicates whether the SMS deletion was successful or not.
 */
bool deleteSms(const char* index) {
	char command[16]={0};
    snprintf(command, sizeof(command), "AT+CMGD=%s\r", index);
	return sendAtCommand(command,5000,"OK");
}


/**
 * The function stores an SMS message with a given address using AT commands.
 * 
 * @param address The phone number or address of the recipient of the SMS message.
 * @param message The message to be stored in the SMS memory of the device.
 * 
 * @return Indicates whether the SMS message was successfully stored or not.
 */
bool storeSms(const char* address,const char* message) {
    char command[MAX_SMS_SIZE]={0};
    // AT+CMGW=<da><CR>text<ctrl-Z/ESC>
    sprintf(command, "AT+CMGW=\"%s\"\r", address);
    sendAtCommand(command,1000,">"); //init sms
	memset(command,'\0',strlen(command));
	sprintf(command, "%s%c",message,26); //store sms
	if ( sendAtCommand(command,5000,"OK") ) {
		HAL_Delay(100);
		return 1;
	}
	else {
		HAL_Delay(100);
		return 0;
	}
}

/**
 * The function reads the number of messages stored in memory storage 1 and returns a boolean value
 * indicating success or failure.
 * 
 * @param count Number of stored messages.
 * 
 * @return Indicates whether the stored count was successfully read or not.
 */
bool readStoredCount1(uint16_t* count) {

	char response[100]={0};
	uint16_t used=0;

	if (!getSerialResponse("AT+CPMS?\r",20000,"OK",response,sizeof(response)))
		return 0;

	if (sscanf(response, "\r\n+CPMS: %*[^,],%d,%*d,%*[^,],%*d,%*d,%*[^,],%*d,%*d", &used) == 1) { //EDIT \r\n
		*count = used;
		return 1;
	} else
		return 0;

}

/**
 * The function reads the number of messages stored in memory storage 2 and returns a boolean value
 * indicating success or failure.
 * 
 * @param count Number of stored messages.
 * 
 * @return Indicates whether the stored count was successfully read or not.
 */
bool readStoredCount2(uint16_t* count) {

	char response[100]={0};
	uint16_t used=0;

	if (!getSerialResponse("AT+CPMS?\r",20000,"OK",response,sizeof(response)))
		return 0;

	if (sscanf(response, "\r\n+CPMS: %*[^,],%*d,%*d,%*[^,],%d,%*d,%*[^,],%*d,%*d", &used) == 1) { //EDIT \r\n
		*count = used;
		return 1;
	} else
		return 0;

}

/**
 * The function sends stored SMS messages and deletes all sent and read messages.
 * 
 * @return Indicates whether the SMS sending process was successful or not.
 */
bool sendStoredSms() {

	uint16_t count=0;
	char command[16]={0};

	if (!readStoredCount2(&count))
		return 0;

    for (uint16_t i = 0; i < count; i++) {
        snprintf(command, sizeof(command), "AT+CMSS=%d\r", i);
        if (!sendAtCommand2(command,60000,"+CMSS:")) //edit
			break;

    }

    if (!sendAtCommand("AT+CPMS=\"ME\",\"ME\",\"SM\"\r",5000,"OK")) {
    	return 0;
    }

	snprintf(command, sizeof(command), "AT+CMGD=1,2\r");
    sendAtCommand(command,50000,"OK"); //delete all sent and read messages
	return 1;
}

/**
 * The function writes a file named "settings.txt" with the contents of the input string to a specific
 * location using AT commands.
 * 
 * @param settings Data that will be written to the file.
 * 
 * @return Indicates whether the write operation was successful or not.
 */
bool writeFile(const char* settings) {

	bool success=0;
	//AT+CFSWFILE=<index>,<filename>,<mode>,<file size>,<inputtime>
	//AT+CFSWFILE=0,"settings.txt",0,strlen(settings),9999
	char command[60]={0};
	snprintf(command, sizeof(command), "AT+CFSWFILE=0,\"settings.txt\",0,%d,10000\r\n", strlen(settings));

	if(!sendAtCommand("AT+CFSINIT\r",2000,"OK")) //Init
		return 0;
	uartSend((uint8_t *)command, strlen(command)); //send atCommand

	char confirm[14];
	  if (!getSerialResponse2(5000, confirm , sizeof(confirm))) {
	  	sendAtCommand("AT+CFSTERM\r",2000,"OK"); //Free
	    return 0;
	  }

	//uartSend((uint8_t *)settings, strlen(settings)); //send atCommand
	success=sendAtCommand(settings,2000,"OK");
	//HAL_Delay(100);
	sendAtCommand("AT+CFSTERM\r",2000,"OK"); //Free
	return success;
}

/**
 * This function reads the size of a file named "settings.txt" using AT commands and returns a boolean
 * value indicating success or failure.
 * 
 * @param fileSize A pointer to a variable where the file size will be stored
 * 
 * @return Indicates whether the file size was successfully read or not.
 */
bool readFileSize(uint16_t* fileSize) {

	char response[30]={0};
	uint16_t fileSizeCopy=0;

	if(!sendAtCommand("AT+CFSINIT\r",2000,"OK")) //Init1
		return 0;

	//AT+CFSGFIS=<index>,<filename>
	//AT+CFSGFIS=0,"settings.txt"
	if (!getSerialResponse("AT+CFSGFIS=0,\"settings.txt\"\r",2000,"+CFSGFIS:",response,sizeof(response))) {
		sendAtCommand("AT+CFSTERM\r",2000,"OK"); //Free
		return 0;
	}

	if (sscanf(response, "\r\n+CFSGFIS: %d", &fileSizeCopy) == 1) { // EDIT \r\n
		*fileSize=fileSizeCopy;
		sendAtCommand("AT+CFSTERM\r",2000,"OK"); //Free
		return 1;
	} else {
		sendAtCommand("AT+CFSTERM\r",2000,"OK"); //Free
		return 0;
	}

}

/**
 * The function reads a file and returns its contents in text format.
 * 
 * @param settings a pointer to a character array where the contents of the file will be stored
 * 
 * @return Indicates whether the file was successfully read or not.
 */
bool readFile(char* settings) {

	char command[60]={0};
	uint16_t fileSize=0;
	char response[MAX_FILE_LEN]={0};

	if(!readFileSize(&fileSize)) {
		char initialSettings[21]="52,19,1672527600,120";
		if (writeFile(initialSettings)) { //if no file write file
			strncpy(settings,initialSettings,strlen(initialSettings));
			return 1;
		}
		else
			return 0;
	}

	//AT+CFSRFILE=<index>,<filename>,<mode>,<file size>,<position>
	//AT+CFSRFILE=0,"settings.txt",0,fileSize,0
	snprintf(command, sizeof(command), "AT+CFSRFILE=0,\"settings.txt\",0,%d,0\r", fileSize);

	if(!sendAtCommand("AT+CFSINIT\r",2000,"OK")) //Init
		return 0;

	  if (!sendAtCommand(command,10000,"+CFSRFILE:")) {
	  	sendAtCommand("AT+CFSTERM\r",2000,"OK"); //Free
	  	return 0;
	  }

	  if (!getSerialResponse2(10000, response , sizeof(response))) {
	  	sendAtCommand("AT+CFSTERM\r",2000,"OK"); //Free
	    return 0;
	  }
	  char confirm[8];
	  if (!getSerialResponse2(5000, confirm , sizeof(confirm))) {
	  	sendAtCommand("AT+CFSTERM\r",2000,"OK"); //Free
	    return 0;
	  }

	strncpy(settings,response,strlen(response));
	sendAtCommand("AT+CFSTERM\r",2000,"OK"); //Free
	return 1;

}

//setup commands

/**
 * The function sets the ICCID and returns a boolean value indicating success or failure.
 * 
 * @param iccid pointer to character array where ICCID value will be stored.
 * 
 * @return Value indicating success or failure
 */
bool setIccid(char* iccid) {

	char response[50]={0};
	char iccidCopy[MAX_ICCID_LEN+1]={0};

	if (getSerialResponse("AT+CCID\r",2000,"ERROR",response,sizeof(response)))
		return 0;

	if (sscanf(response, "\r\n%s", iccidCopy) == 1) { //EDIT %s
		iccidCopy[strlen(iccidCopy)-1] = '\0'; //cut last char
		strncpy(iccid,iccidCopy,strlen(iccidCopy));
		return 1;
	}
	else
		return 0;

}


/**
 * The function copies a device struct and its members to a new memory location.
 * 
 * @param device A pointer to a struct Device.
 * 
 * @return Returns a pointer to a newly allocated `struct Device` that is a
 * copy of the input `device` struct. If memory allocation fails, it returns `NULL`.
 */
struct Device* copyDevice(const struct Device* device) {

    struct Device* newDevice = malloc(sizeof(struct Device)); // Allocate memory for the new device struct
    if (newDevice == NULL) {
        return NULL; // Return NULL if memory allocation fails
    }
    memcpy(newDevice, device, sizeof(struct Device)); // Copy the device struct members to the new memory location
    newDevice->count = device->count; // Copy the subscribers array to the new memory location
    for (int i = 0; i < device->count; i++) {
        strcpy(newDevice->subscribers[i].address, device->subscribers[i].address);
        memcpy(newDevice->subscribers[i].circle, device->subscribers[i].circle, sizeof(float) * 3);
    }
    strcpy(newDevice->iccid, device->iccid);     // Copy the iccid string to the new memory location

    return newDevice;
}

/**
 * The function converts given struct to a string format.
 * 
 * @param str A pointer to a character array where the resulting string will be stored.
 * @param device A pointer to a struct Device
 */
void structToString(char* str, const struct Device* device) {
    for (int i = 0; i < device->count; i++) {
        sprintf(str, "%s,%d,%.6f,%.6f,%.0f\r\n", device->subscribers[i].address, device->subscribers[i].state,
            device->subscribers[i].circle[0], device->subscribers[i].circle[1], device->subscribers[i].circle[2]);
        str += strlen(str);
    }
    char deviceData[40] = { 0 };
    sprintf(deviceData, "%.6f,%.6f,%d,%d", device->position[0], device->position[1], (int)(device->unixTime), device->interval);
    strcat(str, deviceData);

}

/**
 * The function reads the information from the string and writes the data into a given structure.
 * 
 * @param str A pointer to a character array containing the string to be parsed.
 * @param device a pointer to a struct Device.
 */
void stringToStruct(const char* str, struct Device* device) {
    char* endLine = strstr(str, "\r\n");
    int i = 0;
    int readCount = 0;

    while (i < MAX_PHONES && endLine) {
        sscanf(str, "%[^,],%d,%f,%f,%f\r\n", device->subscribers[i].address, &(device->subscribers[i].state),
            &(device->subscribers[i].circle[0]), &(device->subscribers[i].circle[1]), &(device->subscribers[i].circle[2]));
        readCount++;
        i++;
        str = endLine + 2;
        endLine = strstr(str, "\r\n");
    }
    (device->count) = readCount;

    sscanf(str, "%f,%f,%d,%d", &(device->position[0]), &(device->position[1]), (int*)&(device->unixTime), &(device->interval) );

}

/**
 * The function reads a file and converts its contents to a struct
 * 
 * @param device Pointer to a struct of type Device.
 * 
 * @return Indicates success or failure.
 */
bool handleReadFile(struct Device* device) {

    char settings[MAX_FILE_LEN + 1] = { 0 };

    if (!readFile(settings))
        return 0;

    stringToStruct(settings, device);
    return 1;

}

/**
 * The function handles writing data to a file.
 * 
 * @param device Pointer to a struct of type Device.
 * 
 * @return Indicates success or failure.
 */
bool handleWriteFile(const struct Device* device) {

    char settings[MAX_FILE_LEN + 1] = { 0 };
    structToString(settings, device);
    //printf("structToString:%s", settings);

    return writeFile(settings);

}

/**
 * The function searches for a subscriber in an array of subscribers based on their address and returns
 * their index if found.
 * 
 * @param index pointer to an integer variable that will store the index of the subscriber.
 * @param subscribersArr A pointer to an array of subscribers.
 * @param count Pointer to an integer variable that holds the number of elements in the
 * subscribersArr array.
 * @param address The address parameter is a pointer to a character array that represents the address
 * of a subscriber.
 * 
 * @return Indicates success or failure.
 */
bool findSubscriber(int * index, const struct Subscriber* subscribersArr, const int* count, const char* address) {
    int i = 0;
    for (i = 0; i < *count; i++) {
        if (strcmp(subscribersArr[i].address, address) == 0) {
            *index = i;
            return 1;
        }
    }
    *index = i-1;
    return 0;
}

/**
 * Function handles subscription and updates the device's subscribers list.
 * 
 * @param device Pointer to a struct Device.
 * @param newSubscriber Pointer to a struct Subscriber
 * 
 * @return Indicates whether the subscribe operation was successful or not.
 */
bool subscribe(struct Device * device, struct Subscriber* newSubscriber) {

    int i = 0;
    bool exist = 0;
    struct Device* deviceCopy = copyDevice(device); //deepCopy as temporary device struct

    exist = findSubscriber(&i, deviceCopy->subscribers, &(deviceCopy->count), newSubscriber->address);
    if (exist) {
        deviceCopy->subscribers[i] = *newSubscriber; //edit existing subscriber
    }
    else if (i < MAX_PHONES-1) {
        deviceCopy->subscribers[i+1] = *newSubscriber; //add new subscriber
        (deviceCopy->count)++; //add 1 to count
    }
    else {
       free(deviceCopy);
       return 0;
    }

    if (handleWriteFile(deviceCopy)) {
        memcpy(device, deviceCopy, sizeof(struct Device));
        free(deviceCopy);
        return 1;
    }
    else {
        free(deviceCopy);
        return 0;
    }

}

/**
 * The function unsubscribes a subscriber from a device and updates the device's subscribers list
 * 
 * @param device Pointer to a struct Device, which contains information about a device and its
 * subscribers.
 * @param address The address of the subscriber that needs to be unsubscribed from the device's list of
 * subscribers.
 * 
 * @return Indicates whether the unsubscribe operation was successful or not.
 */
bool unsubscribe(struct Device* device, const char* address) {
    int i = 0;
    int j = 0;
    bool exist = 0;
    struct Device* deviceCopy = copyDevice(device); //deepCopy as temporary device struct

    exist = findSubscriber(&i, deviceCopy->subscribers, &(deviceCopy->count), address);

    if (exist) {
        for (j = i; j < deviceCopy->count - 1; j++) {
            memcpy(&(deviceCopy->subscribers[j]), &(deviceCopy->subscribers[j+1]), sizeof(struct Subscriber)); // Shift all elements after index i by 1 pos
        }
        (deviceCopy->count)--; // Decrement the count
    }
    else {
        free(deviceCopy);
        return 0;
    }

    if (handleWriteFile(deviceCopy)) {
        memcpy(device, deviceCopy, sizeof(struct Device));
        free(deviceCopy);
        return 1;
    }
    else {
        free(deviceCopy);
        return 0;
    }

}

/**
 * Writes time and date parsed from text to Unix time variable.
 * 
 * @param unixTime A pointer to a time_t variable that will be filled with the Unix timestamp
 * corresponding to the input string.
 * @param str A string representing a date and time in the format "YYYYMMDDHHMMSS".
 */
void fillUnixTime(time_t* unixTime, const char* str) {
    struct tm tm_time = { 0 };
    int year, month, day, hour, minute, second;

    sscanf(str, "%4d%2d%2d%2d%2d%2d", &year, &month, &day, &hour, &minute, &second);

    tm_time.tm_year = year - 1900;
    tm_time.tm_mon = month - 1;
    tm_time.tm_mday = day;
    tm_time.tm_hour = hour;
    tm_time.tm_min = minute;
    tm_time.tm_sec = second;

    *unixTime = mktime(&tm_time);
}

/**
 * The function converts a Unix timestamp to a formatted string representing the date and time.
 * 
 * @param str A pointer to a character array where the formatted time string will be stored.
 * @param unixTime A pointer to a time_t variable that represents the Unix timestamp.
 */
void unixTimeToString(char* str, const time_t* unixTime) {
    struct tm timeinfo = *localtime(unixTime);
    strftime(str, FORMATTED_TIME_LEN, "%d.%m.%Y %H:%M:%S", &timeinfo);
}

/**
 * The function extracts GPS position and time information from a given buffer and saves them using given parameters.
 * 
 * @param position A pointer to a float array that will hold the latitude and longitude values
 * extracted from the GPS buffer.
 * @param unixTime A pointer to a time_t variable.
 * @param gpsBuffer A character array containing GPS data.
 * 
 * @return Indicates success or failure.
 */
bool setPositionAndTime(float* position,time_t* unixTime, const char* gpsBuffer) {

    char timeString[MAX_TIME_LEN + 1] = { 0 };
    float lat, lng;
    if (sscanf(gpsBuffer, "\r\n+CGNSINF: %*d,%*d,%[^.].%*[0-9],%f,%f", timeString, &lat, &lng) == 3) {
        fillUnixTime(unixTime, timeString);
        position[0] = lat;
        position[1] = lng;
        //printf("Time: %s, Latitude: %.6f, Longitude: %.6f\n", timeString, lat, lng);
        return 1;
    }
    else {
        return 0;
    }

}

/**
 * The function attempts to retrieve and store navigation information parsed from NMEA Sentences 
 * within a specified timeout period.
 * 
 * @param device Pointer to a struct representing the device.
 * @param timeout The maximum time in milliseconds that the function will wait for valid GPS data before
 * timing out and returning false.
 * 
 * @return Value indicating whether the GPS positioning was successful or not.
 */
bool gpsPositioning(struct Device* device, const uint32_t timeout) {
	uint32_t startTime = 0;
    bool answer = 0;
	char response[MAX_GPS_RESPONSE]={0};
    // Turn on GNSS power supply 
    if(!sendAtCommand("AT+CGNSPWR=1\r",2000,"OK"))    
		return 0;
    startTime = HAL_GetTick();
    // Wait for answer
    while( answer == 0 && ((HAL_GetTick()-startTime) < timeout) ) {
        if(getSerialResponse("AT+CGNSINF\r",2000,"OK",response,sizeof(response))) {
			if(strstr(response,",,,,,") == NULL) {
                // Valid data
            	answer = 1;
            }
            else {
            	memset(response,'\0',strlen(response));
            	HAL_Delay(1000);
            }
        }
        else {
            memset(response,'\0',strlen(response));
            HAL_Delay(1000);
            }
        }
    // Turn off the GNSS power supply
	sendAtCommand("AT+CGNSPWR=0\r",2000,"OK"); 
	if (answer) {
        // Sets the device's position and Unix time 
        return setPositionAndTime(device->position,&(device->unixTime),response);
	}
	else {
		return 0;
	}
}

/**
 * The function checks if a given string has a certain prefix.
 * 
 * @param str A pointer to a character array representing the string to be checked for the prefix.
 * @param pre The prefix string that we want to check if it is present at the beginning of the str
 * string.
 * 
 * @return Indicates whether string has a certain prefix or not.
 */
bool prefix(const char* str, const char* pre)
{
    return strncmp(str, pre, strlen(pre)) == 0;
}


/**
 * Finds the length written as string and returns it as an integer.
 * 
 * @param input A pointer to a character array (string) containing a number at the end.
 * 
 * @return an integer value which is the reverse of the last 3 digits of the input string.
 */
int findLength(const char* input) {
    char temp[4] = { 0 };
    char reve[4] = { 0 };
    char ch = 0;
    for (int i = 1;i < 4; ++i) {
        ch = *(input - i);
        if (ch < '0' || ch > '9')
            break;
        temp[i - 1] = ch;

    }
    int j = strlen(temp) - 1;
    for (int i = 0; i < strlen(temp); ++i)
    {
        reve[i] = temp[j];
        --j;

    }
    return atoi(reve);
}

/**
 * The function splits a given input string into multiple substrings based on the delimiter "\r\n" and
 * returns the count of substrings.
 * 
 * @param input A pointer to a character array that contains the input text to be split.
 * 
 * @return Value which represents the number of times the delimiter "\r\n" was found in the
 * input string.
 */
uint16_t splitText(char* input) {
    char* p, * m;
    uint8_t delimLen = 2; //\r\n
    int messageLen = 0;
    uint16_t count = 0;

    p = strstr(input, "\r\n");
    messageLen = findLength(p);
    m = input;

    while (p && messageLen) {
        m = p + delimLen + messageLen;
        *m = '\0';
        m += delimLen;
        //printf("%d\n",count);
        p = strstr(m, "\r\n");
        if (p) {
            messageLen = findLength(p);
            //printf("%d\r\n", messageLen);
        }
        ++count;
        //printf("%d\r\n", messageLen);
        //printf("%d",count);
    }
    return count;
}

/**
 * The function calculates the distance between two points on Earth using the Haversine formula.
 * 
 * @param lat1 Latitude of the first location in degrees
 * @param lng1 Longitude of the first location in degrees.
 * @param lat2 Latitude of the second point in degrees.
 * @param lng2 Longitude of the second point in degrees.
 * 
 * @return the distance between two points on the Earth's surface, calculated using the Haversine
 * formula. The distance is returned in meters.
 */
double haversine(double lat1, double lng1, double lat2, double lng2) {
    double dx, dy, dz;
    lng1 -= lng2;
    lng1 *= (M_PI / 180);
    lat1 *= (M_PI / 180);
    lat2 *= (M_PI / 180);
    dz = sin(lat1) - sin(lat2);
    dx = cos(lng1) * cos(lat1) - cos(lat2);
    dy = sin(lng1) * cos(lat1);
    return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * 6371000;
}

/**
 * Function converts the data contained in the structure into text
 * 
 * @param str A pointer to a character array where the function will store the string representation of
 * the circle coordinates.
 * @param subscribersArr A pointer to an array of subscribers.
 * @param count A pointer to an integer variable that holds the number of elements in the
 * subscribersArr array.
 * @param address A pointer to a character string representing the address of a subscriber.
 * 
* @return Indicates success or failure.
 */
bool circleToString(char * str, struct Subscriber* subscribersArr, int* count, const char* address) {

    for (int i = 0; i < *count; i++) {
        if (strcmp(subscribersArr[i].address, address) == 0) {
            sprintf(str, "%.6f,%.6f,%.0f", subscribersArr[i].circle[0], subscribersArr[i].circle[1], subscribersArr[i].circle[2]);
            return 1;
        }
    }
    return 0;
}


/**
 * The function checks whether the device's position exceeds any of the circles set by subscribers, 
 * handles SMS notification writing and zone exceedance status changing
 * 
 * @param device a pointer to a struct representing the device.
 * 
 * @return Indicates whether any zone has been exceeded or not
 */
bool checkCircles(struct Device* device) {
    bool change = 0;
    double distance = 0;
    for (int i = 0; i < device->count; i++) {
        distance = haversine((device->position[0]), (device->position[1]),
            (device->subscribers[i].circle[0]), (device->subscribers[i].circle[1]) );
        if (distance > device->subscribers[i].circle[2] && !device->subscribers[i].state) {
            char response[MAX_SMS_SIZE] = { 0 }; // Sms response
            char time[FORMATTED_TIME_LEN + 1] = { 0 };
            unixTimeToString(time, &(device->unixTime) );
            sprintf(response, "Zone exceeded by %.1fm\r\nPosition:\r\n%.6f,%.6f\r\nTime:\r\n%s\r\n", distance - device->subscribers[i].circle[2],
                device->position[0], device->position[1], time);
            storeSms(device->subscribers[i].address, response);
            device->subscribers[i].state = 1;    // Set state and send notification
            change = 1;
        }
        else if (distance < device->subscribers[i].circle[2] && device->subscribers[i].state) {
            device->subscribers[i].state = 0; // Reset state
            change = 1;
        }
    }
    return change; // Return change 0 if state did not change and 1 if any state changed
}

/**
 * Function handles notifications in case of zone exceedance
 * 
 * 
 * @param device A pointer to a struct Device.
 */
void handleNotifications(struct Device* device) {

    struct Device* deviceCopy = copyDevice(device); //deepCopy as temporary device struct

    if (checkCircles(deviceCopy)) {
        if (handleWriteFile(deviceCopy)) {
            memcpy(device, deviceCopy, sizeof(struct Device));
            free(deviceCopy); // Deallocate
        }
        else
            free(deviceCopy); // Deallocate
    }
    else
        free(deviceCopy); // Deallocate
}

/**
 * The function checks if a given number is within a specified range.
 * 
 * @param number A float number that needs to be checked if it falls within the range of min and max.
 * @param min The minimum value that the input number can have to be considered valid.
 * @param max The maximum value that the input number can have to be considered valid.
 * 
 * @return Indicates whether the input `number` is within the range specified by `min` and `max`.
 */
bool validNum(float number, float min, float max) {
    if ((number >= min) && (number <= max))  
        return 1;
    else
        return 0;
}

/**
 * The function generates an SMS response with information about a device's position and sends it to a
 * specified address (phone number).
 * 
 * @param device a pointer to a struct Device.
 * @param address The phone number of the recipient of the SMS response.
 * @param requestId A string representing the unique identifier of the request.
 * @param responseCode An integer representing the response code to be sent in the SMS response.
 * 
 * @return a boolean value true.
 */
bool appResponse(struct Device* device, const char* address, const char* requestId, const int responseCode) {

    char response[MAX_SMS_SIZE] = { 0 }; //sms response
    char circle[MAX_CIRCLE_LEN + 1] = { 0 };
    bool hasCircle = circleToString(circle, (device->subscribers), &(device->count), address);

    if ( responseCode>=0 && responseCode<10 ) {
    	sprintf(response, "resp=%s,%d,%.6f,%.6f,%d,%s,%d", requestId, responseCode, device->position[0], device->position[1],
    		(int)(device->unixTime), hasCircle ? circle : "52,19,0", device->interval);
    } else {
	sprintf(response, "resp=%s,%d", requestId, responseCode);
    }

	storeSms(address, response);

	return 1;
}

/**
 * This function process a message based on a command, the commands are different for SMS and app (SMS handled by the application).
 * 
 * @param message Pointer to a character array representing the message received by the device.
 * @param address Pointer to a string representing the phone number
 * @param device Pointer to a struct Device.
 * @param command Pointer to a character array representing the command to be processed.
 * 
 * @return Indicates success or failure.
 */
bool processMessageByCommand(const char* message, const char* address, struct Device* device, const char* command) {
    switch (*command) {
    case 'a': // Subscribe SMS
    {
        float circle[3] = { 0,0,0 }; // parameters included in sms message

        if (sscanf(message, "%f,%f,%f", &circle[0], &circle[1], &circle[2]) != 3) {
            return 0;
        }

        if (!(validNum(circle[0], LATITUDE_MIN, LATITUDE_MAX) && validNum(circle[1], LONGITUDE_MIN, LONGITUDE_MAX) && validNum(circle[2], RADIUS_MIN, RADIUS_MAX)))
            return 0;

        struct Subscriber newSubscriber = { "", 0, { circle[0], circle[1], circle[2] } }; // Create element based on message
        strncpy(newSubscriber.address, address, strlen(address)); // Fill element based on message
        bool success = subscribe(device, &newSubscriber);


		char response[MAX_SMS_SIZE] = { 0 }; // Sms response
        if (success) {
            sprintf(response, "Subscribed:\r\n%.6f,%.6f,%.0f", circle[0], circle[1], circle[2]);
        }
        else {
            sprintf(response, "Subscription failed:\r\n%.6f,%.6f,%.0f", circle[0], circle[1], circle[2]);
        }

        storeSms(address, response);

        return 1;
    }
    case 'p': // Get position SMS
    {
        char response[MAX_SMS_SIZE] = { 0 }; // Sms response
        char circle[MAX_CIRCLE_LEN + 1] = { 0 };
        bool hasCircle = circleToString(circle, (device->subscribers), &(device->count), address);
        char time[FORMATTED_TIME_LEN + 1] = { 0 };

        unixTimeToString(time, &(device->unixTime));
        sprintf(response, "Position:\r\n%.6f,%.6f\r\nTime:\r\n%s\r\nZone:\r\n%s\r\nInterval:\r\n%d", device->position[0], device->position[1],
            time, hasCircle ? circle : "not subscribed", device->interval);

        storeSms(address, response);

        return 1;
    }
    case 'd': // Unsubscribe SMS
    {
        bool success = unsubscribe(device, address);
        char response[MAX_SMS_SIZE] = { 0 }; // Sms response
        if (success) {
            sprintf(response, "Unsubscribed");
        }
        else {
            sprintf(response, "Unsubscription failed");
        }

        storeSms(address, response);

        return 1;
    }

	case '0': // Get position app
    {
		char requestId[REQUEST_ID_LEN+1] = {0};
		int responseCode = 0;

		if (sscanf(message, "%14s", requestId) != 1)
			return 0;

		responseCode = 1;
		return appResponse(device,address,requestId,responseCode);
    }

		case '2': // Unsubscribe app
    {
		char requestId[REQUEST_ID_LEN+1] = {0};
		int responseCode = 0;
		bool unsubscribeSuccess = 0;

		if (sscanf(message, "%14s", requestId) != 1)
			return 0;

		unsubscribeSuccess = unsubscribe(device, address);

		responseCode = unsubscribeSuccess ? 21 : 20;
		return appResponse(device,address,requestId,responseCode);
    }

	case '1': // Subscribe app
	{
		char requestId[REQUEST_ID_LEN+1] = {0};
		int responseCode = 0;
        float circle[3] = { 0,0,0 };
		int subscribeSuccess = 0;

		if (sscanf(message, "%14[^,],%f,%f,%f", requestId, &circle[0], &circle[1], &circle[2]) != 4)
			return 0;
		if (!( validNum(circle[0], LATITUDE_MIN, LATITUDE_MAX) && validNum(circle[1], LONGITUDE_MIN, LONGITUDE_MAX) && validNum(circle[2], RADIUS_MIN, RADIUS_MAX)))
			return 0;

        struct Subscriber newSubscriber = { "", 0, { circle[0], circle[1], circle[2] } }; // Create element based on message
        strncpy(newSubscriber.address, address, strlen(address)); // Fill element based on message
        subscribeSuccess = subscribe(device, &newSubscriber);

        responseCode =  subscribeSuccess ? 11 : (device->count<MAX_PHONES ? 10 : 12);
        return appResponse(device,address,requestId,responseCode);
	}

    default:
        return 0;
    }
}

/**
 * Function reads the prefix and the command contained in the message, 
 * if the message has a valid prefix then a function is called to handle the message based on the command
 * 
 * @param message A pointer to a character array containing the message to be processed.
 * @param address Pointer to a string representing the phone number
 * @param device A pointer to a struct Device.
 * 
 * @return Indicates success or failure.
 */
bool processMessage(const char* message, const char* address, struct Device* device) {

    char prefixStr[MAX_ICCID_LEN + 2] = ""; // +2 for ",\0"
    sprintf(prefixStr, "%s,", device->iccid);

    if (!prefix(message, prefixStr)) {
        return 0;
    }
    message += strlen(prefixStr); 

    char command[3] = { 0 };
    strncpy(command, message, 2);
    if (!((command[0] != 0 && command[1] == 0) || (command[0] != 0 && command[1] == ',')))
    {
        return 0;
    }

    return processMessageByCommand(message + 2, address, device, &command[0]);
}

/**
 * The function handles SMS messages by reading and processing them, and then deleting them from the memory.
 * 
 * @param device a pointer to a struct Device.
 * 
 * @return The function does not have a return type specified, so it does not explicitly return
 * anything. However, it may exit early and return nothing if the `readStoredCount1()` or
 * `readSmsList()` functions return false.
 */
void handleMessages(struct Device* device) {

    char body[MAX_SMS_SIZE] = { 0 }; // Sms body
    char address[MAX_PHONE_LEN + 1] = { 0 }; // Sms sender
    char index[MAX_INDEX_LEN + 1] = { 0 }; // Sms index

    uint16_t storedCount = 0;
    if (!readStoredCount1(&storedCount))
       return;

    if ( storedCount < 1 ) // Exit if no messages
    	return;

    if (storedCount > 128) // Ram limits
    	storedCount=128;

    char* smsList = (char*)malloc(storedCount * MAX_CMGL_RECORD_SIZE);
    if (!readSmsList(smsList, storedCount * MAX_CMGL_RECORD_SIZE)) {
    	free(smsList);
    	return;
    }

    uint16_t unreadCount = splitText(smsList+2);

    char* msg = smsList+2;
    for (uint16_t i = 0; i < unreadCount; i++) {
        getText(body, msg, MAX_SMS_SIZE, "\r\n", "");
        getText(address, msg, MAX_PHONE_LEN + 1, "UNREAD\",\"", "\"");
        getText(index, msg, MAX_INDEX_LEN + 1, "+CMGL: ", ",");
        processMessage(body, address, device);
        deleteSms(index);  
        msg += strlen(msg) + 2;
    }

    free(smsList);
}

/**
 * The function performs a service cycle for a device, including sending AT commands, GPS positioning, handling files, SMS messages and notifications.
 * 
 * @param device a pointer to a struct representing a device
 * 
 * @return The function may return at several points depending on the outcome of the if statements. If
 * any of the if statements evaluate to false, the function will return without executing the remaining
 * code.
 */
void serviceCycle(struct Device* device) {
    // Check connection with module by sending an AT command
	if (!waitConnect(5))
		return;
    // Disable echo mode
	sendAtCommand("ATE0\r",2000,"OK");
    // Set SMS text mode
    if (!sendAtCommand("AT+CMGF=1\r",2000,"OK"))
        return;
    // Show SMS Text Mode Parameters
    if (!sendAtCommand("AT+CSDH=1\r",2000,"OK"))
        return;
    // Configure SMS message indications
    if (!sendAtCommand("AT+CNMI=2,0,0,0,0\r",5000,"OK"))
    	return;
    // Select message storage
    if (!sendAtCommand("AT+CPMS=\"SM\",\"ME\",\"SM\"\r",5000,"OK"))
       	return;
    // Read data from a file
    if (!handleReadFile(device))
        return;
    // Perform GPS positioning
    if (gpsPositioning(device,90000)) {
        // Write data to a file
        handleWriteFile(device);
        // Handle notifications
        handleNotifications(device);
    }
    // Set ICCID (Integrated Circuit Card Identifier)
    if (!setIccid(device->iccid))
    	return;
    // Handle received messages
    handleMessages(device);
    // Send stored SMS messages
    sendStoredSms();
}

/**
 * The function enters STOP/LOWPOWER mode for a specified interval using RTC wake-up timer and resumes
 * normal operation after the interval.
 * 
 * @param interval A pointer to an integer variable that specifies the time interval (in milliseconds)
 * after which the microcontroller should wake up from sleep mode.
 */
void enterSleep(const int* interval) {
	/* Set STOP/LOWPOWER mode */
    HAL_SuspendTick();
    HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, *interval , RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
    /* Enter STOP/LOWPOWER mode */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    SystemClock_Config();
    HAL_ResumeTick();
}

/**
 * Sets the DTR level to a high level so that the module enters sleep mode.
 */
void enterSleepModule() {
	// Enable slow clock controlled by DTR.
	sendAtCommand("AT+CSCLK=1\r",2000,"OK");
	// Set DTR high in order to enter sleep mode
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
}

/**
 * Sets the DTR pin to a low level so that the module quits sleep mode.
 */
void quitSleepModule() {
	//set DTR low in order to quit sleep mode
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

/**
 * The function initializes the device structure, calls the function "serviceCycle" and enables sleep mode on module and microcontroller. 
 */
void workCycle() {
	// Module will exit sleep mode
    quitSleepModule();
    // Structure that contains current device properties
	struct Device device = {0};
    // Factory interval in seconds
	device.interval = 120;
    // Call serviceCycle
	serviceCycle(&device);
    // Testing only!
	// device.interval = 1;
    // Module will enter sleep mode
	enterSleepModule();
    // The microcontroller will enter sleep mode
	enterSleep(&(device.interval));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  //setup

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Loop
	  workCycle();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 300, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA8 PA9 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB12 PB13 PB14 PB15
                           PB4 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
