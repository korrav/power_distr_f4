/*
 * configure.h
 *
 *  Created on: 10 янв. 2016 г.
 *      Author: andrej
 */

#ifndef APPLICATION_USER_CONFIGURE_H_
#define APPLICATION_USER_CONFIGURE_H_
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

struct Sample {			//структура, содержащая параметры одного отсчёта
	uint16_t iMad[3];
	uint16_t u48v;
	uint16_t u300v;
};

struct MCur {	//конфигурация безопасности тока МАД
	uint16_t max;	//верхняя граница безопасного значения тока потребления
	uint16_t time_overh;	//количество отсчётов, которое ток потребления может находится за границей max, прежде чем сработает сигнал тревоги
};

struct Volt {
	uint16_t max;	//верхняя граница безопасного значения напряжения питания
	uint16_t min;	//нижняя граница безопасного значения напряжения питания
	uint16_t time_overh;	//количество отсчётов, которое напряжение питания может находится за границей безопасного режима, прежде чем сработает сигнал тревоги
};

struct SecConfigure {	//конфигурация безопасности
	struct MCur mads[3];	//токи потребления МАД
	struct Volt v_48;	//напряжение 48В
	struct Volt v_300; //напряжение 300В
};

#define	DISABLE_MAD  -1

struct OrderMadActivate {	//порядок активации МАД
	int32_t seq;	//порядковый номер включения
	uint32_t timeAfterPrevActivate;	//время в мс, которое пройдёт после предыдущего включения
};

struct Configure {	//конфигурация приложения
	struct SecConfigure sec;
	struct OrderMadActivate order[3];	//очерёдность и время ключения МАД
	uint32_t isEnableAutoOffPowerMad;	//разрешение автовыключения канал МАД при сигнализировании о его перегрузке
	uint32_t isEnableMonitorOrientation;	//разрешение вести мониторинг ориентации в пространстве
	int32_t seqNumSample;	//порядковый номер отсчёта, подлежащего записи в ethernet буфер
};

union I2cMes {
	uint32_t blob;
	struct {	//структура  обычного сообщения I2C
		uint16_t id;	//идентификатор сообщения
		uint16_t arg;	//параметр
	} frame;
};

enum ID_I2C_MES {	//идентификаторы обычных сообщений I2C
	MES_VOLT_48V_TO_SEC_MODE,	//мониторируемое напряжение 48В перешло в состояние безопасного режима (в качестве параметра текущий отсчёт)
	MES_VOLT_48V_TO_UNSEC_MODE,	//мониторируемое напряжение 48В перешло в состояние небезопасного режима (в качестве параметра текущий отсчёт)
	MES_VOLT_300V_TO_SEC_MODE,	//мониторируемое напряжение 300В перешло в состояние безопасного режима (в качестве параметра текущий отсчёт)
	MES_VOLT_300V_TO_UNSEC_MODE, //мониторируемое напряжение 300В перешло в состояние небезопасного режима (в качестве параметра текущий отсчёт)
	MES_MONIT_NOT_TIME,	//мониторинговая программа не успевает обрабатывать данные при текущей скорости потока
	MES_OVR_ADC,	//DMA не успевает забирать данные из входного буфера АЦП
	MES_SPI_TRANS_NOT_TIME,	//передача данных  в SPI порт не успевает вовремя закончится при текущей скорости потока
	MES_GET_CONFIGURE,	//запрос о получении конфигурации безопасности
	MES_DISABLE_TRANS_MONIT_DATA,	//запрет передачи мониторной информации
	MES_ENABLE_TRANS_MONIT_DATA,	//разрешение передачи мониторной информации
	MES_START_MONIT_DATA,	//старт оцифровки мониторной информации по питанию
	MES_CUR_TO_OVERHEAD,	//переход тока потребления МАД в состояние перегрузки
	MES_CUR_TO_NORMAL,	//переход тока потребления МАД в нормальное состояние
};

enum id_info {	//идентификаторы информационных сообщений
	INF_VOLT_48V_TO_SEC_MODE,	//мониторируемое напряжение 48В перешло в состояние безопасного режима (в качестве параметра текущий отсчёт)
	INF_VOLT_48V_TO_UNSEC_MODE,	//мониторируемое напряжение 48В перешло в состояние небезопасного режима (в качестве параметра текущий отсчёт)
	INF_VOLT_300V_TO_SEC_MODE,	//мониторируемое напряжение 300В перешло в состояние безопасного режима (в качестве параметра текущий отсчёт)
	INF_VOLT_300V_TO_UNSEC_MODE, //мониторируемое напряжение 300В перешло в состояние небезопасного режима (в качестве параметра текущий отсчёт)
	INF_MONIT_F3_NOT_TIME,	//мониторинговая программа F3 не успевает обрабатывать данные при текущей скорости потока
	INF_MONIT_F4_NOT_TIME,	//мониторинговая программа F4 не успевает обрабатывать данные при текущей скорости потока
	INF_OVR_ADC,	//DMA не успевает забирать данные из входного буфера АЦП
	INF_SPI_TRANS_NOT_TIME,	//передача данных  в SPI порт не успевает вовремя закончится при текущей скорости потока
	INF_CUR_TO_OVERHEAD,	//переход тока потребления МАД в состояние перегрузки
	INF_CUR_TO_NORMAL,	//переход тока потребления МАД в нормальное состояние
	INF_SET_TEST_PIN_PA3,	//установлен тестовый вывод PA3
	INF_SET_TEST_PIN_EX13,	//установлен тестовый вывод 13 разъёме расширения
	INF_SET_TEST_PIN_EX15,	//установлен тестовый вывод 15 разъёме расширения
	INF_SET_TEST_PIN_EX16,	//установлен тестовый вывод 16 разъёме расширения
};

struct I2cMesTrans {	//сообщения, передаваемые для трансляции по i2c интерфейсу
	void* pBuf;
	osPoolId pool_id;
};

enum {
	DEACTIVE,
	ACTIVE
};

struct StatusOfPower {	//структура характеризующая текущее состояние системы питания акустического стринга
	uint32_t isMadOn[3];
	uint32_t is48vActive;
	uint32_t is300vActive;
};

struct Status {	//текущее состояние приложения
	struct StatusOfPower stPower;
	struct Configure conf;
};

#define NUM_ELEM_BUF_DATA 50000	//количество элементов буфера данных (должно быть кратно 5)
#define MAX_VAL_COUNT_LEAD 3 //максимальное значение счётчика инкрементирования
#define I2C_ADDR_HOST_MCU 8	//I2C адрес управляющего МК : 1
#define I2C_ADDR_CONF 6	//I2C адрес получения конфигурации безопасности
#define I2C_ADDR_COM 2	//I2C адрес получения команд
#define PORT_SC_DEFAULT 32000
#define PORT_BAG_DEFAULT 32000
#define SIZE_BUF_REC_ETH  1524
#define POWER_SAMPL_MONIT_FREQ	100000	//частота дискретизации мониторинга питания (Гц)
#define MAX_FREQ_SINGL_SAMPL_MONIT_POWER 10 //максимальная частота следования пакетов данных мониторинга питания, содержащих один отсчёт (Гц)
#define MAX_SAMPL_IN_PACK_DATA_MONIT_POWER 100//максимальное количество отсчётов, которое может содержаться в одном пакете данных мониторинга питания
#define SAMPLING_PERIOD_MONIT_POWER 10 //период дискретизации мониторной информации по питанию (мкс)

#define DIVISION_FACTOR_MA 3	//коэффициент, на который надо разделить значение тока МАД, чтобы получить его в мА
#define MULTI_FACTOR_48V_MV 17	//коэффициент, на который надо умножить значение, снимаемое с выхода 48V, чтобы получить значение в мВ
#define DIVISION_FACTOR_300V_V 10	//коэффициент, на который надо разделить значение, снимаемое с выхода 300V, чтобы получить значение в В
//структура UDP пакетов
#define ID_SC 4     //идентификатор компьютера берегового центра
#define ID_BAG 5    //идентификатор БЭ

struct SrcPack {
	int idSrc;  //идентификатор источника
	int idBlock;    //идентификатор блока данных
};

enum id_block {	//идентификаторы блоков данных
	DATA,
	COMMAND,
	ANSWER,
	INFO
};

struct MarkPack {
	int id; //уникальный номер блока данных
	int part;   //порядковый номер куска блока данных
	int totalPart;  //общее количество частей в пакете
};

struct HeadPack {   //заголовок пакета
	struct SrcPack endpoints;
	struct MarkPack mark;
};

struct H_pack_ans { //структура, содержащая ответ на команду
	int32_t id; //идентификатор команды
	int32_t status; //результат выполнения команды (OK или NOT_OK)
};

enum ID_ANS {
    STATUS,
    CONFIGURE
};

enum status_ans {
	NOT_OK, OK
};
struct AnsWithStatusInfo {
	struct HeadPack head;
	struct H_pack_ans h_ans;
	struct Status curStatus;
};

struct AnsWithConfigInfo {
	struct HeadPack head;
	struct H_pack_ans h_ans;
	struct Configure curConfig;
};

enum id_data {  //идентификаторы блоков данных
	POWER,  //данные по питанию
	POSITION,   //данные положения в пространстве
};

enum ID_COM_FOR_ETHER { //идентификаторы команд, передаваемых по интернету
	POWERON,    //включить МАД
	POWEROFF,    //выключить МАД
	SET_PERIOD_TRANS_DATA_MONIT_POWER,	//установить период передачи мониторной информации
	START_MONIT_DATA_POW,	//запустить  оцифровку мониоринговых данных питания в F3
	SET_SEC_CONFIG_CURRENT_MAD,	//установить конфигурацию безопасности тока потребления МАД
	SET_SEC_CONFIG_48V, //установить конфигурацию безопасности напряжение на выводе 48В
	SET_SEC_CONFIG_300V, //установить конфигурацию безопасности напряжение на выводе 300В
	SAVE_CONFIG_IN_EEPROM,	//сохранить текущую конфигурацию в EEPROM
};

enum {
	DISABLE_TRANS_DATA = -1,	//запрещено передавать данные оператору
	SINGLE_TRANS_DATA = 0,	//произвести единичную передачу данных
};

struct StatusFillEthBufOfDataMonitPower {	//ствтус заполнения ethernet буфера данными мониторинга питания
	int32_t curNum;	//текущий номер отсчёта
	int32_t alreadyFill;	//сколько уже записано в буфер передачи
};

enum ID_COM_EEPROM {
	E_READ = 3,
	E_WREN = 6,
	E_WRITE = 2,
	E_RDSR	= 5,
};

#define ADDRESS_IN_EEPROM_MEM_1		0	//адрес в памяти EEPROM по которому считываются и записываются полезные данные (1 байт адреса)
#define ADDRESS_IN_EEPROM_MEM_2		0	//адрес в памяти EEPROM по которому считываются и записываются полезные данные (2 байт адреса)

//SC ip
#define SC_IP_1 192
#define SC_IP_2 168
#define SC_IP_3 203
#define SC_IP_4 35

//function
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void SPI3_error(void);	//функция обратного вызова прерывания ошибки SPI3
void setTestPinPA3(void);
void setTestPinEx13(void);
void setTestPinEx15(void);
void setTestPinEx16(void);
#endif /* APPLICATION_USER_CONFIGURE_H_ */
