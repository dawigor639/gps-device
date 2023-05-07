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
#include <stdio.h>
#include <time.h> //time struct
#include <stdint.h> //uint
#include <string.h>
#include <stdbool.h> //bool;
#include <stdlib.h> //atoi
#include <math.h> //haversine

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct Subscriber {
    char address[17];
    int state;
    float circle[3];
};

struct Device {
    float position[2];
    time_t unixTime;
    int interval;
    struct Subscriber subscribers[10];
    int count;
    char iccid[23];
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PHONES  10
#define MAX_SMS_SIZE 160
#define MAX_ICCID_LEN  22
#define MAX_PHONE_LEN  16
#define MAX_SETTINGS  3
#define MAX_TIME_LEN  14
#define MAX_CIRCLE_LEN  29
#define FORMATTED_TIME_LEN 20
#define UNIX_TIME_LEN 10
#define MAX_FILE_LEN  539
#define MAX_GPS_RESPONSE 120
#define MAX_INDEX_LEN 5
#define MAX_CMGL_RECORD_SIZE 260
// Uart short responses
#define MAX_AT_RESPONSE 100
// latitude range
#define LATITUDE_MIN -90
#define LATITUDE_MAX 90
// longitude range
#define LONGITUDE_MIN -180
#define LONGITUDE_MAX 180
// radius range
#define RADIUS_MIN 50
#define RADIUS_MAX 6371000
// Unix time range
#define UNIX_TIME_MIN 1678970022
#define UNIX_TIME_MAX 2147483647
// requestID
#define REQUEST_ID_LEN 14
// Default latitude and longitude values
#define LATITUDE_DEFAULT 52
#define LONGITUDE_DEFAULT 19
// Constants for interval range
//#define INTERVAL_MIN 120 // May be used in future
//#define INTERVAL_MAX 86400 // May be used in future

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
volatile bool uartReady = 0;
volatile uint8_t uartError = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	uartReady=1;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uartError=1;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///////////////////////////////////

void uartRead(uint8_t* data, const uint16_t size) {
	uartError=0;
	uartReady=0;
	HAL_UART_DMAStop(&huart1); //stop possible previous transfer
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, data, size);
}

//void uartReadBlocking(uint8_t* data, const uint16_t size,const uint32_t timeout) {
//	HAL_UART_Receive(&huart1, data, size, timeout);
//}

void uartSend(uint8_t* data, const uint16_t size) {
	HAL_UART_Transmit(&huart1, data, size,500); //send atCommand
}

bool uartAvailable() {
	if(uartReady) {
		uartReady=0;
		return 1;
	}
	else {
		return 0;
	}
}

bool uartErrors() {
	if(uartError) {
		uartError=0;
		return 1;
	}
	else {
		return 0;
	}
}

bool sendAtCommand(const char* atCommand, const uint32_t timeout, const char* expectedAnswer) {

    uint32_t startTime = 0;
    char receiveBuffer[MAX_AT_RESPONSE]={0}; // Receive from UART Buffer

    uartSend((uint8_t *)atCommand, strlen(atCommand)); //send atCommand

    uartRead((uint8_t*)receiveBuffer, MAX_AT_RESPONSE); //start reading

    startTime = HAL_GetTick();

    // this loop waits for the answer
    while( (HAL_GetTick()-startTime) < timeout ){
        if(uartAvailable()){
                // check if the desired answer is in the response of the module
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

bool sendAtCommand2(const char* atCommand, const uint32_t timeout, const char* expectedAnswer) {

    uint32_t startTime = 0;
    char receiveBuffer[MAX_AT_RESPONSE]={0}; // Receive from UART Buffer

    uartSend((uint8_t *)atCommand, strlen(atCommand)); //send atCommand

    uartRead((uint8_t*)receiveBuffer, MAX_AT_RESPONSE); //start reading

    startTime = HAL_GetTick();

    // this loop waits for the answer
    while( (HAL_GetTick()-startTime) < timeout ){

    	if(uartErrors()) {
    		 memset(receiveBuffer,'\0',strlen(receiveBuffer));
    		 uartRead((uint8_t*)receiveBuffer, MAX_AT_RESPONSE); //start reading
    	}
    	else {
			if(uartAvailable()){
					// check if the desired answer is in the response of the module
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

bool getSerialResponse(const char* atCommand, const uint32_t timeout, const char* expectedAnswer, char* receiveBuffer , const uint16_t size) {

    uint32_t startTime = 0;

    uartSend((uint8_t *)atCommand, strlen(atCommand)); //send atCommand

    uartRead((uint8_t*)receiveBuffer, size); //start reading

    startTime = HAL_GetTick();

    // this loop waits for the answer
    while( (HAL_GetTick()-startTime) < timeout ){
        if(uartAvailable()){
                // check if the desired answer is in the response of the module
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

bool getSerialResponse2(const uint32_t timeout, char* receiveBuffer , const uint16_t size) {

    uint32_t startTime = 0;

    uartRead((uint8_t*)receiveBuffer, size); //start reading

    startTime = HAL_GetTick();

    // this loop waits for the answer
    while( (HAL_GetTick()-startTime) < timeout ){
        if(uartAvailable()){
        	return 1;
    	}
    }
    return 0;
}

///////////////////////////////////

bool waitConnect(const uint8_t maxAttempts) {
	for(int i=0; i<maxAttempts;i++) {
		if (sendAtCommand("AT\r",2000,"OK"))
			return 1;
	}
	return 0;
}


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

bool readSmsList(char * smsList, const uint16_t smsListSize){
	return getSerialResponse("AT+CMGL=\"REC UNREAD\",1\r",20000,"+CMGL:",smsList,smsListSize);
}

bool deleteSms(const char* index) {
	char command[16]={0};
    snprintf(command, sizeof(command), "AT+CMGD=%s\r", index);
	return sendAtCommand(command,5000,"OK");
}


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

//setup commands//

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

///////////////////////////////////

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
bool handleReadFile(struct Device* device) {

    //char settings[MAX_FILE_LEN + 1] = "+852348467,0,-50.446896,-18.838770,6371000\r\n+48346765889,0,-50.446896,-18.838770,6371000\r\n+9946765889,0,-50.446896,-18.838770,6371000\r\n21,37,1234567890,144";
    //char settings[MAX_FILE_LEN + 1] = "21,37,1234567890,144";
    char settings[MAX_FILE_LEN + 1] = { 0 };

    if (!readFile(settings))
        return 0;

    stringToStruct(settings, device);
    return 1;

}

//EditFile pracuje na kopii struktury aby zachować poprzednie dane w przypadku wystąpienia problemu z zapisem do pliku
bool handleWriteFile(const struct Device* device) {

    char settings[MAX_FILE_LEN + 1] = { 0 };
    structToString(settings, device);
    //printf("structToString:%s", settings);

    return writeFile(settings);

}

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

void unixTimeToString(char* str, const time_t* unixTime) {
    struct tm timeinfo = *localtime(unixTime);
    strftime(str, FORMATTED_TIME_LEN, "%d.%m.%Y %H:%M:%S", &timeinfo);
}

bool setPositionAndTime(float* position,time_t* unixTime, const char* gpsBuffer) {

    char timeString[MAX_TIME_LEN + 1] = { 0 };
    float lat, lng;
    if (sscanf(gpsBuffer, "\r\n+CGNSINF: %*d,%*d,%[^.].%*[0-9],%f,%f", timeString, &lat, &lng) == 3) { //Edit \r\n
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

bool gpsPositioning(struct Device* device, const uint32_t timeout){

	uint32_t startTime = 0;
    bool answer = 0;
	char response[MAX_GPS_RESPONSE]={0};


    if(!sendAtCommand("AT+CGNSPWR=1\r",2000,"OK"))    //Turn on the GNSS power.
		return 0;

    startTime = HAL_GetTick();

    // this loop waits for the answer
    while( answer == 0 && ((HAL_GetTick()-startTime) < timeout) ) {
        if(getSerialResponse("AT+CGNSINF\r",2000,"OK",response,sizeof(response))) {
			if(strstr(response,",,,,,") == NULL) {
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

	sendAtCommand("AT+CGNSPWR=0\r",2000,"OK"); //Turn off the GNSS power

	if (answer) {
		return setPositionAndTime(device->position,&(device->unixTime),response);
	}
	else {
		return 0;
	}

}

bool prefix(const char* str, const char* pre)
{
    return strncmp(str, pre, strlen(pre)) == 0;
}

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

double haversine(double lat1,double lng1,double lat2,double lng2)
{
    //printf("%f,%f,%f,%f\n", lat1, lng1, lat2, lng2);

    double dx, dy, dz;
    lng1 -= lng2;
    lng1 *= (3.14159265358979323846 / 180);
    lat1 *= (3.14159265358979323846 / 180);
    lat2 *= (3.14159265358979323846 / 180);

    dz = sin(lat1) - sin(lat2);
    dx = cos(lng1) * cos(lat1) - cos(lat2);
    dy = sin(lng1) * cos(lat1);

    return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * 6371000;
}


bool circleToString(char * str, struct Subscriber* subscribersArr, int* count, const char* address) {

    for (int i = 0; i < *count; i++) {
        if (strcmp(subscribersArr[i].address, address) == 0) {
            sprintf(str, "%.6f,%.6f,%.0f", subscribersArr[i].circle[0], subscribersArr[i].circle[1], subscribersArr[i].circle[2]);
            return 1;
        }
    }
    return 0;
}

//return true if any state changed
bool checkCircles(struct Device* device) {
    bool change = 0;
    double distance = 0;
    for (int i = 0; i < device->count; i++) {

        distance = haversine((device->position[0]), (device->position[1]),
            (device->subscribers[i].circle[0]), (device->subscribers[i].circle[1]) );
        //printf("dist: %f m subscriber: [%d]\n", distance , i);

        if (distance > device->subscribers[i].circle[2] && !device->subscribers[i].state) {

            char response[MAX_SMS_SIZE] = { 0 }; //sms response
            char time[FORMATTED_TIME_LEN + 1] = { 0 };
            unixTimeToString(time, &(device->unixTime) );
            sprintf(response, "Zone exceeded by %.1fm\r\nPosition:\r\n%.6f,%.6f\r\nTime:\r\n%s\r\n", distance - device->subscribers[i].circle[2],
                device->position[0], device->position[1], time);

            //printf("Response:\r\n%s", response);
            storeSms(device->subscribers[i].address, response);

            device->subscribers[i].state = 1;    //setState and send notification
            change = 1;
        }
        else if (distance < device->subscribers[i].circle[2] && device->subscribers[i].state) {
            device->subscribers[i].state = 0; //Reset state
            change = 1;
        }
    }
    return change; //return change 0 if state did not change and 1 if any state changed
}

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

bool validNum(float number, float min, float max) {
    if ((number >= min) && (number <= max))  //if ((min ? number >= min : true) && max ? (number <= max) : true)
        return 1;
    else
        return 0;
}

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
	//printf("Response:\r\n%s", response);
	storeSms(address, response);

	return 1;
}

bool processMessageByCommand(const char* message, const char* address, struct Device* device, const char* command) {
    switch (*command) {
    case 'a': //Subscribe -> setCircle / editCircle
    {
        float circle[3] = { 0,0,0 }; //parameters included in  sms message [rad/interval,lat,lng]

        if (sscanf(message, "%f,%f,%f", &circle[0], &circle[1], &circle[2]) != 3) {
            return 0;
        }

        if (!(validNum(circle[0], LATITUDE_MIN, LATITUDE_MAX) && validNum(circle[1], LONGITUDE_MIN, LONGITUDE_MAX) && validNum(circle[2], RADIUS_MIN, RADIUS_MAX)))
            return 0;

        struct Subscriber newSubscriber = { "", 0, { circle[0], circle[1], circle[2] } }; //Create element based on message
        strncpy(newSubscriber.address, address, strlen(address)); //Fill element based on message
        bool success = subscribe(device, &newSubscriber);


		char response[MAX_SMS_SIZE] = { 0 }; //sms response
        if (success) {
            sprintf(response, "Subscribed:\r\n%.6f,%.6f,%.0f", circle[0], circle[1], circle[2]);
        }
        else {
            sprintf(response, "Subscription failed:\r\n%.6f,%.6f,%.0f", circle[0], circle[1], circle[2]);
        }

        //printf("Response:\r\n%s", response);
        storeSms(address, response);

        return 1;
    }
    case 'p': //getPosition sms
    {
        //resp=50.446896,18.838770,1678729041,50.446896,18.838770,50000
        char response[MAX_SMS_SIZE] = { 0 }; //sms response
        char circle[MAX_CIRCLE_LEN + 1] = { 0 };
        bool hasCircle = circleToString(circle, (device->subscribers), &(device->count), address);
        char time[FORMATTED_TIME_LEN + 1] = { 0 };

        unixTimeToString(time, &(device->unixTime));
        sprintf(response, "Position:\r\n%.6f,%.6f\r\nTime:\r\n%s\r\nZone:\r\n%s\r\nInterval:\r\n%d", device->position[0], device->position[1],
            time, hasCircle ? circle : "not subscribed", device->interval);

        //printf("Response:\r\n%s", response);
        storeSms(address, response);

        return 1;
    }
    case 'd': //Unsubscribe
    {
        bool success = unsubscribe(device, address);
        char response[MAX_SMS_SIZE] = { 0 }; //sms response
        if (success) {
            sprintf(response, "Unsubscribed");
        }
        else {
            sprintf(response, "Unsubscription failed");
        }

        //printf("Response:\r\n%s", response);
        storeSms(address, response);

        return 1;
    }
    /*
    case 'i': //setInterval
    {
        int interval = 0;
        if (sscanf(message, "%d", &interval) != 1) {
            return 0;
        }


        if (!validNum(interval, INTERVAL_MIN, INTERVAL_MAX))
            return 0;

        int intervalBck = device->interval; //Copy current interval
        device->interval = interval; //Update with newInterval

        bool success = handleWriteFile(device);
        if (!success)
            device->interval = intervalBck; //Undo changes

        char response[MAX_SMS_SIZE] = { 0 }; //sms response
        if (success) {
            sprintf(response, "Interval set:\r\n%d", interval);
        }
        else {
            sprintf(response, "Setting interval failed:\r\n%d", interval);
        }

        //printf("Response:\r\n%s", response);
        storeSms(address, response);

        return 1;
    }
    */
	case '0': //Position
    {
		char requestId[REQUEST_ID_LEN+1] = {0};
		int responseCode = 0;

		if (sscanf(message, "%14s", requestId) != 1)
			return 0;

		responseCode = 1;
		return appResponse(device,address,requestId,responseCode);
    }

		case '2': //Unsubscribe
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

	case '1': //Subscribe
	{
		char requestId[REQUEST_ID_LEN+1] = {0};
		int responseCode = 0;
        float circle[3] = { 0,0,0 };
		int subscribeSuccess = 0;

		if (sscanf(message, "%14[^,],%f,%f,%f", requestId, &circle[0], &circle[1], &circle[2]) != 4)
			return 0;
		if (!( validNum(circle[0], LATITUDE_MIN, LATITUDE_MAX) && validNum(circle[1], LONGITUDE_MIN, LONGITUDE_MAX) && validNum(circle[2], RADIUS_MIN, RADIUS_MAX)))
			return 0;

        struct Subscriber newSubscriber = { "", 0, { circle[0], circle[1], circle[2] } }; //Create element based on message
        strncpy(newSubscriber.address, address, strlen(address)); //Fill element based on message
        subscribeSuccess = subscribe(device, &newSubscriber);

        responseCode =  subscribeSuccess ? 11 : (device->count<MAX_PHONES ? 10 : 12);
        return appResponse(device,address,requestId,responseCode);
	}
	/*
	case '3': //Interval
    {
		int requestTime = 0;
		int responseCode = 0;
		int interval = 0;
		bool intervalSuccess = 0;

		if (sscanf(message, "%d,%d", &requestTime, &interval) != 2)
			return 0;
		if (!(validNum(requestTime, UNIX_TIME_MIN, UNIX_TIME_MAX) && validNum(interval, INTERVAL_MIN, INTERVAL_MAX)))
			return 0;

        int intervalBck = device->interval; //Copy current interval
        device->interval = interval; //Update with newInterval
        intervalSuccess = handleWriteFile(device);
        if (!intervalSuccess)
            device->interval = intervalBck; //Undo changes

        responseCode = intervalSuccess ? 31 : 30;
		return appResponse(device,address,requestTime,responseCode);

    }
    */
    default:
        return 0;
    }
}

bool processMessage(const char* message, const char* address, struct Device* device) {

    char prefixStr[MAX_ICCID_LEN + 2] = ""; //+2 for ",\0"
    sprintf(prefixStr, "%s,", device->iccid);

    if (!prefix(message, prefixStr)) {
        return 0;
    }
    message += strlen(prefixStr); //printf("%s\n", message);

    char command[3] = { 0 };
    strncpy(command, message, 2);
    if (!((command[0] != 0 && command[1] == 0) || (command[0] != 0 && command[1] == ',')))
    {
        return 0;
    }

    return processMessageByCommand(message + 2, address, device, &command[0]);
}

void handleMessages(struct Device* device) {

    char body[MAX_SMS_SIZE] = { 0 }; //sms body
    char address[MAX_PHONE_LEN + 1] = { 0 }; //sms sender
    char index[MAX_INDEX_LEN + 1] = { 0 }; //sms index

    uint16_t storedCount = 0;
    if (!readStoredCount1(&storedCount))
       return;

    if ( storedCount < 1 ) //exit if no messages
    	return;

    if (storedCount > 128) //ram limits
    	storedCount=128;

    char* smsList = (char*)malloc(storedCount * MAX_CMGL_RECORD_SIZE);
    //strcpy(smsList,"+CMGL: 0,\"REC UNREAD\",\"+27832729407\",,\"12/03/17,21:32:05+08\",145,46\r\n891004234814455936,a,50.446896,18.838770,50000\r\n+CMGL: 1,\"REC UNREAD\",\"+27832729407\",,\"12/03/17,21:32:30+08\",145,20\r\n891004234814455936,0\r\n+CMGL: 2,\"REC UNREAD\",\"+27832729407\",,\"12/03/17,21:32:58+08\",145,7\r\nTEXT123\r\n+CMGL: 3,\"REC UNREAD\",\"+27832729407\",,\"12/03/17,21:33:19+08\",145,8\r\nTEXT1234\r\n+CMGL: 4,\"REC UNREAD\",\"+27832729407\",,\"12/03/17,21:34:03+08\",145,9\r\nTEXT12345\r\n\r\nOK\r\n");
    if (!readSmsList(smsList, storedCount * MAX_CMGL_RECORD_SIZE)) {
    	free(smsList);
    	return;
    }

    uint16_t unreadCount = splitText(smsList+2);

    char* msg = smsList+2;
    for (uint16_t i = 0; i < unreadCount; i++) {

        //printf("[%d]%s", i, msg);
        getText(body, msg, MAX_SMS_SIZE, "\r\n", "");
        getText(address, msg, MAX_PHONE_LEN + 1, "UNREAD\",\"", "\"");
        getText(index, msg, MAX_INDEX_LEN + 1, "+CMGL: ", ",");
        processMessage(body, address, device);
        deleteSms(index);  //

        msg += strlen(msg) + 2;
    }

    free(smsList);
}

void serviceCycle(struct Device* device) {

	if (!waitConnect(5))
		return;

	sendAtCommand("ATE0\r",2000,"OK");

    if (!sendAtCommand("AT+CMGF=1\r",2000,"OK"))
        return;

    if (!sendAtCommand("AT+CSDH=1\r",2000,"OK"))
        return;

    if (!sendAtCommand("AT+CNMI=2,0,0,0,0\r",2000,"OK"))
    	return;

    if (!sendAtCommand("AT+CPMS=\"SM\",\"ME\",\"SM\"\r",5000,"OK"))
    	return;

    if (!handleReadFile(device))
        return;

    if (gpsPositioning(device,90000)) {
        handleWriteFile(device);
        handleNotifications(device);
   }

    if (!setIccid(device->iccid))
    	return;

    handleMessages(device);

    sendStoredSms();

}

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

void workCycle() {

	struct Device device = {0}; //struct Device device = { {52,19}, 1672527600, 120, { 0 } , 0 , {0} };

	device.interval = 120; //factory interval in s

	serviceCycle(&device);

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
	  //loop
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
                           PB4 PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
