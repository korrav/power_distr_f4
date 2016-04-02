/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 *
 * COPYRIGHT(c) 2016 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "spi.h"
#include "i2c.h"

/* USER CODE BEGIN Includes */     
#include "configure.h"
#include <string.h>
#include "lwip/api.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId mTaskHandle;
osThreadId iTaskHandle;
osThreadId wcTaskHandle;
osThreadId rcTaskHandle;

/* USER CODE BEGIN Variables */
osPoolId poolUInt32_Id;	//пул, содержащий int32_t элементы
osPoolId poolSecConfigure_Id;	//пул, содержащий Configure элементы
osMailQId qInputTransI2c1_Id;	//входная очередь сообщений, предназначенных для передачи по i2c1 интерфейсу
osMessageQId qMainTaskComHandle;	//входная очередь основной задачи
osMailQId qMainTaskInputConf_Id;	//входная очередь конфигурации основной задачи
static uint16_t bufData[NUM_ELEM_BUF_DATA];	//буфер данных, заполняемый показаниями мониторинга питания МАД
static uint16_t* addrBegBufData = bufData;	//адрес начала буфера данных
static uint16_t* addrMeanBufData = bufData + NUM_ELEM_BUF_DATA/2;	//адрес середины буфера данных
static uint16_t* addrEndBufData = bufData + NUM_ELEM_BUF_DATA;	//адрес последнего элемента буфера данных
static struct Sample* addrStopPoint;	//адрес следующей точки остановки мониторинговой программы
static struct Sample* addrCurPoint;	//адрес текущего положения мониторинговой программы
static uint32_t countOfLead;	//счётчик опережения. Используется для определния насколько сегментов мониторинг данных отстал от их заполнения
static struct Configure configureEEPROM;	//общая конфигурация приложения
static uint32_t i2c1RecMes;
static struct Status status;
static struct netconn *conn;
static struct ip_addr ipSc;
static int8_t bufDataMonitPower[10 * MAX_SAMPL_IN_PACK_DATA_MONIT_POWER + sizeof(struct HeadPack) + sizeof(int32_t)];	//заполняемый буфер данных мониторинга питания перед отправкой по ethernet
static int8_t bufInfo[sizeof(struct HeadPack) + sizeof(int32_t) + 5 * sizeof(int32_t)];	//заполняемый буфер для передачи информационных сообщений
static struct StatusFillEthBufOfDataMonitPower stFillBufMonitPower;
uint32_t t_count1 = 0;
uint32_t t_count2 = 0;
uint32_t t_count3 = 0;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void taskMain(void const * argument);
void taskI2cTrans(void const * argument);
void taskWriteConf(void const * argument);
void taskReadConf(void const * argument);
void taskUdpReceiv(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void countOfLeadSetZero(void);	//обнуление счётчика опережения
void countOfLeadInc(void);	//инкрементирование счётчика опережения
void countOfLeadDec(void); //декрементировать счётчик опережения
uint32_t countOfLeadRead(void); //прочитать счётчик опережения
void initTrackingPower(void);	//предварительная подготовка перед стартом мониторинга питания МАД
struct Sample* getCurAddrDmaSpi3Rx(void);	//сообщает текущий адрес заполнения DMA ADC
void setStopPointAtBegSegment(void);	//установить точку останова в начало следующего сегмента
void sendSecConfigure(void);	//послать в F3 конфигурацию безопасности
void sendEnableTransMonitData(void);	//послать разрешение F3 передавать данные мониторинга
void sendDisableTransMonitData(void);	//послать запрет F3 передавать данные мониторинга
void sendStartMonitPower(void);	//запуск оцифровки мониторной информации по питанию
void activLedCurrentMad(uint16_t numMad);	//активирование светодиода нормального потребления МАД
void deactivLedCurrentMad(uint16_t numMad);	//деактивирование светодиода нормального потребления МАД
void activLed300V(void); //активирование светодиода, указывающего, что значение питающего напряжения 300V находится в пределах нормы
void deactivLed300V(void); //деактивирование светодиода, указывающего, что значение питающего напряжения 300V находится в пределах нормы
void activLed48V(void); //активирование светодиода, указывающего, что значение питающего напряжения 48V находится в пределах нормы
void deactivLed48V(void); //деактивирование светодиода, указывающего, что значение питающего напряжения 48V находится в пределах нормы
void moveOnRelay(uint16_t numMad);	//включение реле МАД
void moveOffRelay(uint16_t numMad);	//выключение реле МАД
void infoMoveToOverheadCurrentMad(uint16_t numMad);	//переход тока потребления МАД в состояние перегрузки
void infoMoveToNormalCurrentMad(uint16_t numMad);	//переход тока потребления МАД в нормальное состояние
void setConfigure(struct Configure* pConf); //установить текущую конфигурацию приложения
void sendBufToSc(void* pBuf, size_t len);	//послать буфер в береговой центр
void sendAnsStatus(int32_t statExecCom);	//послать ответ, содержащий помимо подтверждения(или опровержения) запршенной команды статусную информацию
void sendAnsConfig(int32_t statExecCom);	//послать ответ, содержащий помимо подтверждения(или опровержения) запршенной команды информацию о конфигурации в EEPROM
void sendBufMonitPowerToSc(size_t lenPayolad);	//послать буфер данных мониторинга питания в береговой центр
void sendBufINFOToSc(size_t lenPayolad);	//послать буфер информации в береговой центр
void setModeF3toRec(void);	//установить режим F3 в режим приёмника
void setModeF3toTrans(void);	//установить режим F3 в режим передатчика
void processSampleMonitPower(struct Sample* pSampl);	//обработка отсчёта мониторинга питания
void initHeadPackDataMonitPower(void);	//инициализация заголовка пакета данных мониторинга питания
void initHeadPackInfo(void);	//инициализация заголовка пакета передачи информации
void setPeriodTransDataMonitPower(int32_t time);	//установить период передачи данных мониторинга питания (мкс)
struct Sample normalization(struct Sample* pSampl);	//нормализация значений выборки данных мониторинга питания
void writeToEEPROM(int8_t* pBuf, size_t len);	//запись данных в EEPROM память
void readFromEEPROM(int8_t* pBuf, size_t len);	//считывание данных из EEPROM память
void sendAndRecByteToEEPROM(int8_t bT, int8_t* bR);
void moveCsSPI4ToLow(void);
void moveCsSPI4ToHigh(void);
void setTestPinPA3(void);
void setTestPinEx13(void);
void toggleTestPinEx13(void);
void setTestPinEx15(void);
void toggleTestPinEx15(void);
void setTestPinEx16(void);
void testCount1(void);
void testCount2(void);
void testCount3(void);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_POOLS */
	/* add pools, ... */
	osPoolDef(poolUInt32, 20, uint32_t);
	poolUInt32_Id = osPoolCreate(osPool(poolUInt32));
	osPoolDef(poolSecConfigure, 3, struct SecConfigure);
	poolSecConfigure_Id = osPoolCreate(osPool(poolSecConfigure));
	/* USER CODE END RTOS_POOLS */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of mTask */
	osThreadDef(mTask, taskMain, osPriorityLow, 0, 1024);
	mTaskHandle = osThreadCreate(osThread(mTask), NULL);

	/* definition and creation of iTask */
	osThreadDef(iTask, taskI2cTrans, osPriorityNormal, 0, 512);
	iTaskHandle = osThreadCreate(osThread(iTask), NULL);

	/* definition and creation of wcTask */
	osThreadDef(wcTask, taskWriteConf, osPriorityNormal, 0, 512);
	wcTaskHandle = osThreadCreate(osThread(wcTask), NULL);

	/* definition and creation of rcTask */
	osThreadDef(rcTask, taskReadConf, osPriorityNormal, 0, 512);
	rcTaskHandle = osThreadCreate(osThread(rcTask), NULL);


	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	osMailQDef(qInputTransI2c1, 10, struct I2cMesTrans);
	qInputTransI2c1_Id = osMailCreate(osMailQ(qInputTransI2c1), NULL);

	osMessageQDef(qMainTaskCom, 20, uint32_t);
	qMainTaskComHandle = osMessageCreate(osMessageQ(qMainTaskCom), NULL);

	osMailQDef(qMainTaskInputConf, 1, struct Configure);
	qMainTaskInputConf_Id = osMailCreate(osMailQ(qMainTaskInputConf), NULL);

	/* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
	/* init code for LWIP */
	MX_LWIP_Init();
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartDefaultTask */
}

/* taskMain function */
void taskMain(void const * argument)
{
	/* USER CODE BEGIN taskMain */
	static osEvent event;
	static union I2cMes mes;

	static uint32_t nextTimePointOnPowerMAD;	//момент времени, когда производится включение следующего МАД
	static int32_t seqNextPowerOnMad = 1;	//порядковый номер следующего включаемого МАД
	static bool isFoundNextMadForPoweron = false;	//имеется следующий МАД для включения
	static uint32_t numNextPowerOnMad;		//номер следующего включаемого МАД в массиве МАД
	static struct netbuf* pNetBuf;
	static err_t netError;
	static int8_t bufRecEth[SIZE_BUF_REC_ETH];
	static struct HeadPack* pHead = (struct HeadPack*)bufRecEth;
	static uint32_t* id_com = (uint32_t*)(bufRecEth + sizeof(struct HeadPack));
	static struct OrderMadActivate orderAct[3];
	struct Configure* pConf;
	static int32_t* payoladInfo = (int32_t*)(bufInfo + sizeof(struct HeadPack));

	stFillBufMonitPower.alreadyFill = 0;
	stFillBufMonitPower.curNum = 0;
	status.conf.seqNumSample = DISABLE_TRANS_DATA;
	addrCurPoint = (struct Sample*)addrBegBufData;

	//создание netconn соединения
	conn = netconn_new(NETCONN_UDP);
	netconn_bind(conn, IP_ADDR_ANY, PORT_BAG_DEFAULT);
	IP4_ADDR(&ipSc,SC_IP_1, SC_IP_2, SC_IP_3, SC_IP_4);
	netconn_connect(conn, &ipSc, PORT_SC_DEFAULT);
	netconn_set_recvtimeout(conn, 1);

	//стартует приём управляющих данных по интерфейсу i2c1
	HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&i2c1RecMes, sizeof(uint32_t));

	//приведение системы в исходное состояние
	sendDisableTransMonitData();
	for(int i = 1; i <= 3; i++)
		moveOffRelay(i);
	deactivLed48V();
	deactivLed300V();

	initHeadPackDataMonitPower();
	initHeadPackInfo();
	//чтение сообщений из очереди конфигурации
	event = osMailGet(qMainTaskInputConf_Id, osWaitForever);
	pConf = (struct Configure*)event.value.p;
	setConfigure(pConf);
	for(int i = 0; i < 3; i++)
		orderAct[i] = configureEEPROM.order[i];
	sendSecConfigure();
	sendStartMonitPower();

	//определение первого включаемого МАД
	nextTimePointOnPowerMAD = osKernelSysTick();
	while(seqNextPowerOnMad <= 3 && !isFoundNextMadForPoweron) {
		for(numNextPowerOnMad = 0; numNextPowerOnMad < 3; numNextPowerOnMad++) {
			if (orderAct[numNextPowerOnMad].seq == seqNextPowerOnMad) {
				isFoundNextMadForPoweron = true;
				break;
			}
		}
		++seqNextPowerOnMad;
	}
	if(isFoundNextMadForPoweron)
		nextTimePointOnPowerMAD += osKernelSysTickMicroSec(orderAct[numNextPowerOnMad].timeAfterPrevActivate * 1000);

	osMailFree(qMainTaskInputConf_Id, pConf);
	if(status.conf.seqNumSample != DISABLE_TRANS_DATA)
		sendEnableTransMonitData();
	/* Infinite loop */
	for(;;)
	{
		if(isFoundNextMadForPoweron) {	//если до сих пор производится начальное включение МАД
			if(osKernelSysTick() >= nextTimePointOnPowerMAD) {
				moveOnRelay(numNextPowerOnMad + 1);
				orderAct[numNextPowerOnMad].seq = DISABLE_MAD;
				isFoundNextMadForPoweron = false;
				while(seqNextPowerOnMad <= 3 && !isFoundNextMadForPoweron) {
					for(numNextPowerOnMad = 0; numNextPowerOnMad < 3; numNextPowerOnMad++) {
						if (orderAct[numNextPowerOnMad].seq == seqNextPowerOnMad) {
							isFoundNextMadForPoweron = true;
							break;
						}
					}
					++seqNextPowerOnMad;
				}
				if(isFoundNextMadForPoweron)
					nextTimePointOnPowerMAD += osKernelSysTickMicroSec(orderAct[numNextPowerOnMad].timeAfterPrevActivate * 1000);
			}
		}
		//проверка входной очереди команд
		event = osMessageGet (qMainTaskComHandle, 0);
		if(event.status == osEventMessage) {
			mes.blob = event.value.v;
			switch(mes.frame.id) {
			case MES_GET_CONFIGURE:
				sendDisableTransMonitData();
				sendSecConfigure();
				if(status.conf.seqNumSample != DISABLE_TRANS_DATA)
					sendEnableTransMonitData();
				break;
			case MES_CUR_TO_OVERHEAD:
				*payoladInfo = INF_CUR_TO_OVERHEAD;
				*(payoladInfo +1) = mes.frame.arg;
				sendBufINFOToSc(2 * sizeof(int32_t));
				break;
			case MES_CUR_TO_NORMAL:
				*payoladInfo = INF_CUR_TO_NORMAL;
				*(payoladInfo +1) = mes.frame.arg;
				sendBufINFOToSc(2 * sizeof(int32_t));
				break;
			case MES_VOLT_48V_TO_SEC_MODE:
				*payoladInfo = INF_VOLT_48V_TO_SEC_MODE;
				sendBufINFOToSc(sizeof(int32_t));
				status.stPower.is48vActive = true;
				activLed48V();
				break;
			case MES_VOLT_48V_TO_UNSEC_MODE:
				*payoladInfo = INF_VOLT_48V_TO_UNSEC_MODE;
				sendBufINFOToSc(sizeof(int32_t));
				status.stPower.is48vActive = DEACTIVE;
				deactivLed48V();
				break;
			case MES_VOLT_300V_TO_SEC_MODE:
				*payoladInfo = INF_VOLT_300V_TO_SEC_MODE;
				sendBufINFOToSc(sizeof(int32_t));
				activLed300V();
				status.stPower.is300vActive = ACTIVE;
				break;
			case MES_VOLT_300V_TO_UNSEC_MODE:
				*payoladInfo = INF_VOLT_300V_TO_UNSEC_MODE;
				sendBufINFOToSc(sizeof(int32_t));
				status.stPower.is300vActive = DEACTIVE;
				deactivLed300V();
				break;
			case MES_MONIT_NOT_TIME:
				*payoladInfo = INF_MONIT_F3_NOT_TIME;
				sendBufINFOToSc(sizeof(int32_t));
				break;
			case MES_OVR_ADC:
				sendDisableTransMonitData();
				*payoladInfo = INF_OVR_ADC;
				sendBufINFOToSc(sizeof(int32_t));
				if(status.conf.seqNumSample != DISABLE_TRANS_DATA)
					sendEnableTransMonitData();
				break;
			case MES_SPI_TRANS_NOT_TIME:
				sendDisableTransMonitData();
				*payoladInfo = INF_SPI_TRANS_NOT_TIME;
				sendBufINFOToSc(sizeof(int32_t));
				if(status.conf.seqNumSample != DISABLE_TRANS_DATA)
					sendEnableTransMonitData();
				break;
			}
		}

		//проверка достиг ли счётчик адресов начала очередного сегмента
		if(addrCurPoint == (struct Sample*)addrBegBufData || addrCurPoint == (struct Sample*)addrMeanBufData) {
			countOfLeadDec();
		}
		for(;addrCurPoint < addrStopPoint; ++addrCurPoint)
			processSampleMonitPower(addrCurPoint);
		//проверка: как обработка отсчётов мониторинга отстала от их выдачи
		switch(countOfLeadRead()) {
		case 0:
			addrStopPoint = getCurAddrDmaSpi3Rx();
			break;
		case 1:
			setStopPointAtBegSegment();
			break;
		case 2:
			if(addrCurPoint > getCurAddrDmaSpi3Rx()) {
				setStopPointAtBegSegment();
				break;
			}
			/* no break */
		case 3:
			taskENTER_CRITICAL();
			addrStopPoint = getCurAddrDmaSpi3Rx();
			if(addrStopPoint < (struct Sample*)addrMeanBufData)
				addrCurPoint = (struct Sample*)addrBegBufData;
			else
				addrCurPoint = (struct Sample*)addrMeanBufData;
			countOfLeadSetZero();
			taskEXIT_CRITICAL();
			*payoladInfo = INF_MONIT_F4_NOT_TIME;
			sendBufINFOToSc(sizeof(int32_t));
			break;
		}

		//проверка Ethernet сообщений
		netError =  netconn_recv( conn, &pNetBuf);
		if(netError == ERR_OK) {
			uint16_t len = netbuf_len(pNetBuf);
			if(len <= SIZE_BUF_REC_ETH && len >= sizeof(struct HeadPack) + sizeof(uint32_t)) {
				netbuf_copy ( pNetBuf, bufRecEth, len);
				netbuf_delete(pNetBuf);
				if(pHead->endpoints.idSrc == ID_SC && pHead->endpoints.idBlock == COMMAND) {
					switch(*id_com) {
					case POWERON:
					{
						if(len < sizeof(struct HeadPack) + 2 * sizeof(uint32_t))
							break;
						uint32_t* pNumMad = id_com + 1;
						moveOnRelay((uint16_t)*pNumMad);
						sendAnsStatus(OK);
						break;
					}
					case POWEROFF:
					{
						if(len < sizeof(struct HeadPack) + 2 * sizeof(uint32_t))
							break;
						uint32_t* pNumMad = id_com + 1;
						moveOffRelay((uint16_t)*pNumMad);
						sendAnsStatus(OK);
						break;
					}
					case SET_PERIOD_TRANS_DATA_MONIT_POWER:
					{
						if(len < sizeof(struct HeadPack) + 2 * sizeof(uint32_t))
							break;
						int32_t* numSample = (int32_t*)id_com + 1;
						setPeriodTransDataMonitPower(*numSample);
						sendAnsStatus(OK);
						break;
					}
					case START_MONIT_DATA_POW:
						if(len < sizeof(struct HeadPack) + sizeof(uint32_t))
							break;
						sendStartMonitPower();
						sendAnsStatus(OK);
						break;
					case SET_SEC_CONFIG_CURRENT_MAD:
						if(len < sizeof(struct HeadPack) + 2 * sizeof(uint32_t) + sizeof(struct MCur))
							break;
						int32_t* pNumMad = (int32_t*)id_com + 1;
						if(*pNumMad < 1 || *pNumMad > 3)
							break;
						struct MCur* pSec = (struct MCur*)(id_com + 2);
						pSec->max *= DIVISION_FACTOR_MA;
						pSec->time_overh /= SAMPLING_PERIOD_MONIT_POWER;
						status.conf.sec.mads[*pNumMad - 1] = *pSec;
						sendSecConfigure();
						sendAnsStatus(OK);
						break;
					case SET_SEC_CONFIG_48V:
						if(len < sizeof(struct HeadPack) + sizeof(uint32_t) + sizeof(struct Volt))
							break;
						struct Volt* p48Volt = (struct Volt*)(id_com + 1);
						p48Volt->max *= 1000;
						p48Volt->max /= MULTI_FACTOR_48V_MV;
						p48Volt->min *= 1000;
						p48Volt->min /= MULTI_FACTOR_48V_MV;
						p48Volt->time_overh /= SAMPLING_PERIOD_MONIT_POWER;
						status.conf.sec.v_48 = *p48Volt;
						sendSecConfigure();
						sendAnsStatus(OK);
						break;
					case SET_SEC_CONFIG_300V:
						if(len < sizeof(struct HeadPack) + sizeof(uint32_t) + sizeof(struct Volt))
							break;
						struct Volt* p300Volt = (struct Volt*)(id_com + 1);
						p300Volt->max *= DIVISION_FACTOR_300V_V;
						p300Volt->min *= DIVISION_FACTOR_300V_V;
						p300Volt->time_overh /= SAMPLING_PERIOD_MONIT_POWER;
						status.conf.sec.v_300 = *p300Volt;
						sendSecConfigure();
						sendAnsStatus(OK);
						break;
					case SAVE_CONFIG_IN_EEPROM:
						if(len < sizeof(struct HeadPack) + sizeof(uint32_t))
							break;
						writeToEEPROM((int8_t*)(&status.conf), sizeof(struct Configure));
						sendAnsConfig(OK);
						break;
					}
				}
			}
		}
	}
	/* USER CODE END taskMain */
}

/* taskI2cTrans function */
void taskI2cTrans(void const * argument)
{
	/* USER CODE BEGIN taskI2cTrans */
	/* Infinite loop */
	static osEvent event;
	static struct I2cMesTrans* pMes;
	static uint8_t blob[sizeof(struct SecConfigure)];
	static uint16_t lenBlob;
	static uint16_t devAddress;
	HAL_StatusTypeDef status;
	for(;;)
	{
		//чтение сообщений из очереди команд
		event = osMailGet(qInputTransI2c1_Id, osWaitForever);
		if(event.status == osEventMail) {
			pMes = (struct I2cMesTrans*)event.value.p;
			if(pMes->pool_id == poolUInt32_Id) {
				lenBlob = sizeof(uint32_t);
				devAddress = I2C_ADDR_COM;
			}
			else {
				lenBlob = sizeof(struct SecConfigure);
				devAddress = I2C_ADDR_CONF;
			}

			while(__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == SET)
				osDelay(1);
			setModeF3toRec();
			osDelay(10);
			__HAL_I2C_DISABLE_IT(&hi2c1,I2C_IT_EVT | I2C_IT_BUF  | I2C_IT_ERR);
			hi2c1.State = HAL_I2C_STATE_READY;
			memcpy(blob, pMes->pBuf, lenBlob);
			osPoolFree(pMes->pool_id, pMes->pBuf);
			osMailFree(qInputTransI2c1_Id, pMes);
			do {
				status = HAL_I2C_Master_Transmit_IT(&hi2c1, devAddress, blob, lenBlob);
			} while(status == HAL_BUSY || status == HAL_TIMEOUT);
		}
	}
}
/* USER CODE END taskI2cTrans */


/* taskWriteConf function */
void taskWriteConf(void const * argument)
{
	/* USER CODE BEGIN taskWriteConf */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END taskWriteConf */
}

/* taskReadConf function */
void taskReadConf(void const * argument)
{
	/* USER CODE BEGIN taskReadConf */
	struct Configure* pConf = (struct Configure*)osMailAlloc(qMainTaskInputConf_Id, osWaitForever);
	pConf->isEnableAutoOffPowerMad = false;
	pConf->isEnableMonitorOrientation = false;
	pConf->seqNumSample = DISABLE_TRANS_DATA;
	pConf->sec.v_48.max = 50;
	pConf->sec.v_48.min = 46;
	pConf->sec.v_48.time_overh = 10;
	pConf->sec.v_300.max = 350;
	pConf->sec.v_300.min = 250;
	pConf->sec.v_300.time_overh = 10;
	for(int i = 0; i < 3; i++) {
		pConf->sec.mads[i].max = 65535;
		pConf->sec.mads[i].time_overh = 10;
		pConf->order[i].timeAfterPrevActivate = 10;
		//pConf->order[i].seq = i+1;
		pConf->order[i].seq = DISABLE_MAD;
	}
	osMailPut(qMainTaskInputConf_Id, pConf);
	/* Infinite loop */

	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END taskReadConf */
}

void taskUdpReceiv(void const * argument) {

}

/* USER CODE BEGIN Application */

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
	if(hspi == &hspi3)
		countOfLeadInc();
	toggleTestPinEx15();
	testCount1();
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if(hspi == &hspi3)
		countOfLeadInc();
	toggleTestPinEx15();
	testCount2();
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if(hi2c == &hi2c1) {
		setModeF3toTrans();
		HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&i2c1RecMes, sizeof(uint32_t));
	}
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if(hi2c == &hi2c1) {
		osMessagePut(qMainTaskComHandle, i2c1RecMes, 0);
		HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&i2c1RecMes, sizeof(uint32_t));
	}
}

void countOfLeadSetZero(void) {
	taskDISABLE_INTERRUPTS();
	countOfLead = 0;
	taskENABLE_INTERRUPTS();
}

void countOfLeadInc(void) {
	taskDISABLE_INTERRUPTS();
	if(countOfLead < MAX_VAL_COUNT_LEAD)
		++countOfLead;
	taskENABLE_INTERRUPTS();
}

void countOfLeadDec(void) {
	taskDISABLE_INTERRUPTS();
	if(countOfLead > 0)
		--countOfLead;
	taskENABLE_INTERRUPTS();
}

uint32_t countOfLeadRead(void) {
	return countOfLead;
}

void initTrackingPower(void) {
	HAL_SPI_DMAStop(&hspi3);
	addrStopPoint = (struct Sample*)addrBegBufData;
	addrCurPoint = (struct Sample*)addrBegBufData;
	countOfLeadSetZero();
	//стартует приём данных мониторинга питания МАД
	HAL_SPI_Receive_DMA(&hspi3, (uint8_t *)bufData, /*(uint16_t)sizeof(bufData)*/NUM_ELEM_BUF_DATA);
}

struct Sample* getCurAddrDmaSpi3Rx(void) {
	uint32_t n = NUM_ELEM_BUF_DATA - hspi3.hdmarx->Instance->NDTR;
	return (struct Sample*)addrBegBufData + n/5;
}

void setStopPointAtBegSegment(void){
	if(addrCurPoint < (struct Sample*)addrMeanBufData)
		addrStopPoint = (struct Sample*)addrMeanBufData;
	else if(addrCurPoint < (struct Sample*)addrEndBufData)
		addrStopPoint = (struct Sample*)addrEndBufData;
	else {
		addrCurPoint = (struct Sample*)addrBegBufData;
		addrStopPoint = (struct Sample*)addrMeanBufData;
	}
}

void sendSecConfigure(void) {
	struct SecConfigure* pSec = (struct SecConfigure*)osPoolAlloc(poolSecConfigure_Id);
	if(pSec == NULL)
		return;
	*pSec = status.conf.sec;
	struct I2cMesTrans* pMes = (struct I2cMesTrans*)osMailAlloc(qInputTransI2c1_Id, 0);
	if(pMes == NULL)
		return;
	pMes->pBuf = (void*)pSec;
	pMes->pool_id = poolSecConfigure_Id;
	if(osMailPut(qInputTransI2c1_Id, pMes) != osOK) {
		osPoolFree(poolSecConfigure_Id, pSec);
		return;
	}
}

void sendEnableTransMonitData(void) {
	uint32_t* pCom = (uint32_t*)osPoolAlloc(poolUInt32_Id);
	if(pCom == NULL)
		return;
	*pCom = MES_ENABLE_TRANS_MONIT_DATA;
	struct I2cMesTrans* pMes = (struct I2cMesTrans*)osMailAlloc(qInputTransI2c1_Id, 0);
	if(pMes == NULL)
		return;
	pMes->pBuf = (void*)pCom;
	pMes->pool_id = poolUInt32_Id;
	if(osMailPut(qInputTransI2c1_Id, pMes) != osOK) {
		osPoolFree(poolUInt32_Id, pCom);
		return;
	}
	setTestPinEx16();
}

void sendDisableTransMonitData(void) {
	uint32_t* pCom = (uint32_t*)osPoolAlloc(poolUInt32_Id);
	if(pCom == NULL)
		return;
	*pCom = MES_DISABLE_TRANS_MONIT_DATA;
	struct I2cMesTrans* pMes = (struct I2cMesTrans*)osMailAlloc(qInputTransI2c1_Id, 0);
	if(pMes == NULL)
		return;
	pMes->pBuf = (void*)pCom;
	pMes->pool_id = poolUInt32_Id;
	if(osMailPut(qInputTransI2c1_Id, pMes) != osOK) {
		osPoolFree(poolUInt32_Id, pCom);
		return;
	}
	status.conf.isEnableAutoOffPowerMad = DEACTIVE;
	osDelay(100);
	initTrackingPower();
}

void sendStartMonitPower(void) {
	uint32_t* pCom = (uint32_t*)osPoolAlloc(poolUInt32_Id);
	if(pCom == NULL)
		return;
	*pCom = MES_START_MONIT_DATA;
	struct I2cMesTrans* pMes = (struct I2cMesTrans*)osMailAlloc(qInputTransI2c1_Id, 0);
	if(pMes == NULL)
		return;
	pMes->pBuf = (void*)pCom;
	pMes->pool_id = poolUInt32_Id;
	if(osMailPut(qInputTransI2c1_Id, pMes) != osOK) {
		osPoolFree(poolUInt32_Id, pCom);
		return;
	}
}
void activLedCurrentMad(uint16_t numMad) {
	uint16_t GPIO_Pin;
	if(numMad == 1)
		GPIO_Pin = GPIO_PIN_13;
	else if(numMad == 2)
		GPIO_Pin = GPIO_PIN_14;
	else if(numMad == 3)
		GPIO_Pin = GPIO_PIN_15;
	else
		return;
	HAL_GPIO_WritePin(GPIOC, GPIO_Pin, SET);
}
void deactivLedCurrentMad(uint16_t numMad) {
	uint16_t GPIO_Pin;
	if(numMad == 1)
		GPIO_Pin = GPIO_PIN_13;
	else if(numMad == 2)
		GPIO_Pin = GPIO_PIN_14;
	else if(numMad == 3)
		GPIO_Pin = GPIO_PIN_15;
	else
		return;
	HAL_GPIO_WritePin(GPIOC, GPIO_Pin, RESET);
}

void activLed300V(void) {
	taskDISABLE_INTERRUPTS();
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);
	status.stPower.is300vActive = ACTIVE;
	taskENABLE_INTERRUPTS();
}

void deactivLed300V(void) {
	taskDISABLE_INTERRUPTS();
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);
	status.stPower.is300vActive = DEACTIVE;
	taskENABLE_INTERRUPTS();
}

void activLed48V(void) {
	taskDISABLE_INTERRUPTS();
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, SET);
	status.stPower.is48vActive = ACTIVE;
	taskENABLE_INTERRUPTS();
}

void deactivLed48V(void) {
	taskDISABLE_INTERRUPTS();
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, RESET);
	status.stPower.is48vActive = DEACTIVE;
	taskENABLE_INTERRUPTS();
}

void moveOnRelay(uint16_t numMad) {
	taskDISABLE_INTERRUPTS();
	uint16_t GPIO_Pin;
	if(numMad == 1)
		GPIO_Pin = GPIO_PIN_2;
	else if(numMad == 2)
		GPIO_Pin = GPIO_PIN_4;
	else if(numMad == 3)
		GPIO_Pin = GPIO_PIN_6;
	else {
		taskENABLE_INTERRUPTS();
		return;
	}
	HAL_GPIO_WritePin(GPIOE, GPIO_Pin, SET);
	status.stPower.isMadOn[numMad -1] = ACTIVE;
	activLedCurrentMad(numMad);
	taskENABLE_INTERRUPTS();
}

void moveOffRelay(uint16_t numMad) {
	taskDISABLE_INTERRUPTS();
	uint16_t GPIO_Pin;
	if(numMad == 1)
		GPIO_Pin = GPIO_PIN_2;
	else if(numMad == 2)
		GPIO_Pin = GPIO_PIN_4;
	else if(numMad == 3)
		GPIO_Pin = GPIO_PIN_6;
	else {
		taskENABLE_INTERRUPTS();
		return;
	}
	HAL_GPIO_WritePin(GPIOE, GPIO_Pin, RESET);
	status.stPower.isMadOn[numMad -1] = DEACTIVE;
	deactivLedCurrentMad(numMad);
	taskENABLE_INTERRUPTS();
}

void infoMoveToOverheadCurrentMad(uint16_t numMad) {
	union I2cMes mes;
	mes.frame.id = MES_CUR_TO_OVERHEAD;
	mes.frame.arg = numMad;
	osMessagePut(qMainTaskComHandle, mes.blob, 0);
}

void infoMoveToNormalCurrentMad(uint16_t numMad) {
	union I2cMes mes;
	mes.frame.id = MES_CUR_TO_NORMAL;
	mes.frame.arg = numMad;
	osMessagePut(qMainTaskComHandle, mes.blob, 0);
}

void setConfigure(struct Configure* pConf) {
	taskDISABLE_INTERRUPTS();
	configureEEPROM = *pConf;
	status.conf = configureEEPROM;
	taskENABLE_INTERRUPTS();
}

void sendBufToSc(void* pBuf, size_t len) {
	struct netbuf* pBufNet = netbuf_new();
	char* pData =netbuf_alloc(pBufNet, len);
	memcpy (pData, pBuf, len);
	netconn_send(conn, pBufNet);
	netbuf_delete(pBufNet);
}

void sendAnsStatus(int32_t statExecCom) {
	struct AnsWithStatusInfo ans;
	ans.head.endpoints.idSrc = ID_BAG;
	ans.head.endpoints.idBlock = ANSWER;
	ans.h_ans.id = STATUS;
	ans.h_ans.status = statExecCom;
	ans.curStatus = status;
	sendBufToSc(&ans, sizeof(ans));
}

void sendAnsConfig(int32_t statExecCom) {
	struct AnsWithConfigInfo ans;
	ans.head.endpoints.idSrc = ID_BAG;
	ans.head.endpoints.idBlock = ANSWER;
	ans.h_ans.id = CONFIGURE;
	ans.h_ans.status = statExecCom;
	ans.curConfig = status.conf;
	sendBufToSc(&ans, sizeof(ans));
}

void sendBufMonitPowerToSc(size_t lenPayolad) {
	sendBufToSc(bufDataMonitPower, sizeof(struct HeadPack) + sizeof(int32_t) + lenPayolad);
}

void sendBufINFOToSc(size_t lenPayolad) {
	sendBufToSc(bufInfo, sizeof(struct HeadPack) + lenPayolad);
}

void setModeF3toRec(void) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, SET);
}

void setModeF3toTrans(void) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, RESET);
}

void processSampleMonitPower(struct Sample* pSampl) {
	static struct Sample* const pPayLoad = (struct Sample*)(bufDataMonitPower + sizeof(struct HeadPack) + sizeof(int32_t));
	bool isSingleSamplInPack = true;
	toggleTestPinEx13();
	if(status.conf.seqNumSample < DISABLE_TRANS_DATA)
		return;
	switch(status.conf.seqNumSample) {
	case DISABLE_TRANS_DATA:
		return;
		break;
	case SINGLE_TRANS_DATA:
		*pPayLoad = normalization(pSampl);
		sendBufMonitPowerToSc(sizeof(struct Sample));
		status.conf.seqNumSample = DISABLE_TRANS_DATA;
		sendDisableTransMonitData();
		return;
		break;
	default:
		if((POWER_SAMPL_MONIT_FREQ / status.conf.seqNumSample) > MAX_FREQ_SINGL_SAMPL_MONIT_POWER)
			isSingleSamplInPack = false;
		if(++stFillBufMonitPower.curNum >= status.conf.seqNumSample) {	//необходима ли запись отсчёта
			if(isSingleSamplInPack) {	//один отсчёт на пакет
				*pPayLoad = normalization(pSampl);
				sendBufMonitPowerToSc(sizeof(struct Sample));
			} else {	//100 отсчётов на пакет
				*(pPayLoad + stFillBufMonitPower.alreadyFill++) = normalization(pSampl);
				if(stFillBufMonitPower.alreadyFill >= MAX_SAMPL_IN_PACK_DATA_MONIT_POWER) {
					sendBufMonitPowerToSc(MAX_SAMPL_IN_PACK_DATA_MONIT_POWER * sizeof(struct Sample));
					stFillBufMonitPower.alreadyFill = 0;
				}
			}
			stFillBufMonitPower.curNum = 0;
		}
		break;
	}
}

void initHeadPackDataMonitPower(void) {
	struct HeadPack* pHeadBufMonitPower = (struct HeadPack*)bufDataMonitPower;
	pHeadBufMonitPower->endpoints.idSrc = ID_BAG;
	pHeadBufMonitPower->endpoints.idBlock = DATA;
	int32_t* id_data = (int32_t*)(bufDataMonitPower + sizeof(struct HeadPack));
	*id_data = POWER;
}

void initHeadPackInfo(void) {
	struct HeadPack* pHeadBufInfo = (struct HeadPack*)bufInfo;
	pHeadBufInfo->endpoints.idSrc = ID_BAG;
	pHeadBufInfo->endpoints.idBlock = INFO;
}

void setPeriodTransDataMonitPower(int32_t time) {
	if(time <= DISABLE_TRANS_DATA)
		time = DISABLE_TRANS_DATA;
	else
		time = time / 10;
	if(status.conf.seqNumSample == DISABLE_TRANS_DATA && time != DISABLE_TRANS_DATA)
		sendEnableTransMonitData();
	stFillBufMonitPower.alreadyFill = 0;
	stFillBufMonitPower.curNum = 0;
	status.conf.seqNumSample = time;
	if(time == DISABLE_TRANS_DATA)
		sendDisableTransMonitData();
}

struct Sample normalization(struct Sample* pSampl) {
	struct Sample sampl;
	for(int i = 0; i < 3; i++)
		sampl.iMad[i] = pSampl->iMad[i] / DIVISION_FACTOR_MA;
	sampl.u48v = pSampl->u48v * MULTI_FACTOR_48V_MV;
	sampl.u300v = pSampl->u300v / DIVISION_FACTOR_300V_V;
	return sampl;
}

void writeToEEPROM(int8_t* pBuf, size_t len) {
	int8_t rBuf;
	if(len == 0)
		return;
	moveCsSPI4ToLow();
	//послать команду разрешения записи
	sendAndRecByteToEEPROM(E_WREN, &rBuf);
	moveCsSPI4ToHigh();
	//послать команду записи
	moveCsSPI4ToLow();
	sendAndRecByteToEEPROM(E_WRITE, &rBuf);
	//послать адрес считывания
	sendAndRecByteToEEPROM(ADDRESS_IN_EEPROM_MEM_1, &rBuf);
	sendAndRecByteToEEPROM(ADDRESS_IN_EEPROM_MEM_2, &rBuf);
	//посылка байтов данных
	for(size_t i = 0; i < len; i++) {
		sendAndRecByteToEEPROM(*(pBuf + i), &rBuf);
		moveCsSPI4ToHigh();
		moveCsSPI4ToLow();
		do {
			osDelay(1);
			sendAndRecByteToEEPROM(E_RDSR, &rBuf);
			sendAndRecByteToEEPROM(0xFF, &rBuf);

		} while((rBuf & 0x1) != 0);
	}


}

void readFromEEPROM(int8_t* pBuf, size_t len) {
	if(len == 0)
		return;
	moveCsSPI4ToLow();
	//послать команду считывания
	sendAndRecByteToEEPROM(E_READ, pBuf);
	//послать адрес считывания
	sendAndRecByteToEEPROM(ADDRESS_IN_EEPROM_MEM_1, pBuf);
	sendAndRecByteToEEPROM(ADDRESS_IN_EEPROM_MEM_2, pBuf);
	//считывание данных по вышеуказанному адресу
	for(size_t i = 0; i < len; i++)
		sendAndRecByteToEEPROM(0xFF, pBuf + i);
	moveCsSPI4ToHigh();
}

void sendAndRecByteToEEPROM(int8_t bT, int8_t* bR) {
	while(!(hspi4.Instance->SR & SPI_SR_TXE))
		osDelay(1);
	hspi4.Instance->DR = bT;
	while(!(hspi4.Instance->SR & SPI_SR_RXNE))
		osDelay(1);
	*bR = hspi4.Instance->DR;
}

void moveCsSPI4ToLow(void) {
	GPIOA->BSRR = GPIO_BSRR_BR_4;
}

void moveCsSPI4ToHigh(void) {
	GPIOA->BSRR = GPIO_BSRR_BS_4;
}

void setTestPinPA3(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, SET);
	int32_t* payoladInfo = (int32_t*)(bufInfo + sizeof(struct HeadPack));
	*payoladInfo = INF_SET_TEST_PIN_PA3;
	sendBufINFOToSc(sizeof(int32_t));

}

void setTestPinEx13(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
	int32_t* payoladInfo = (int32_t*)(bufInfo + sizeof(struct HeadPack));
	*payoladInfo = INF_SET_TEST_PIN_EX13;
	sendBufINFOToSc(sizeof(int32_t));
	sendAnsStatus(OK);

}

void toggleTestPinEx13(void) {
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_8);
}

void setTestPinEx15(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, SET);
	int32_t* payoladInfo = (int32_t*)(bufInfo + sizeof(struct HeadPack));
	*payoladInfo = INF_SET_TEST_PIN_EX15;
	sendBufINFOToSc(sizeof(int32_t));
	sendAnsStatus(OK);
}

void toggleTestPinEx15(void) {
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
}

void setTestPinEx16(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
	int32_t* payoladInfo = (int32_t*)(bufInfo + sizeof(struct HeadPack));
	*payoladInfo = INF_SET_TEST_PIN_EX16;
	sendBufINFOToSc(sizeof(int32_t));
	sendAnsStatus(OK);
}

void testCount1(void) {
	++t_count1;
}

void testCount2(void) {
	++t_count2;
}

void testCount3(void) {
	++t_count3;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint16_t numMad;
	if(GPIO_Pin == GPIO_PIN_5)
		numMad = 3;
	else if(GPIO_Pin == GPIO_PIN_6)
		numMad = 2;
	else if(GPIO_Pin == GPIO_PIN_7)
		numMad = 1;
	else
		return;
	if(HAL_GPIO_ReadPin(GPIOD, GPIO_Pin) == GPIO_PIN_SET) {
		if(status.conf.isEnableAutoOffPowerMad == ACTIVE)
			moveOffRelay(numMad);
		infoMoveToOverheadCurrentMad(numMad);
	}
}

void SPI3_error(void) {
	//setTestPinPA3();
	testCount3();
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
