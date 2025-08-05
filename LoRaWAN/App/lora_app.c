/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
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
#include "platform.h"
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "lora_app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"
//#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"
#include "w25qxx.h"

/* USER CODE BEGIN Includes */
#include "radio.h"
#include "main.h"

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
uint32_t app_nvm = 0;
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief Appdata Transmission external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
  * LEDs period value of the timer in ms
  */

/**
  * Join switch period value of the timer in ms
  */
#define JOIN_TIME 2000

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
  * @brief LoRaWAN NVM Flash address
  * @note last 2 sector of a 128kBytes device
  */

/* USER CODE BEGIN PD */
#define RX_TIME_MARGIN                200
#define PING "PING"
#define PONG "PONG"
#define PAYLOAD_LEN                                 64
#define MAX_APP_BUFFER_SIZE          255
#define RX_TIMEOUT_VALUE             6000


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnNetworkTimerEvent(void *context);

/**
  * @brief  RX timer callback function
  * @param  context ptr of timer context
  */
static void OnP2PTimerEvent(void *context);


/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief callback when LoRaWAN application has sent a frame
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRaWAN application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/**
  * @brief callback when LoRaWAN Beacon status is updated
  * @param params status of Last Beacon
  */
static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params);

/**
  * @brief callback when LoRaWAN application Class is changed
  * @param deviceClass new class
  */
static void OnClassChange(DeviceClass_t deviceClass);

/**
  * @brief  LoRa store context in Non Volatile Memory
  */
static void StoreContext(void);

/**
  * @brief  LoRa restart after StopMode
  */
static void LoRaStart(void);


/**
  * @brief  Force LoRa join procedure
  */
static void Rejoin(void);

///**
//  * @brief  stop current LoRa execution to switch into non default Activation mode
//  */
//static void StopJoin(void);

/**
  * @brief  Join switch timer callback function
  * @param  context ptr of Join switch context
  */
//static void OnStopJoinTimerEvent(void *context);

/**
  * @brief  Notifies the upper layer that the NVM context has changed
  * @param  state Indicates if we are storing (true) or restoring (false) the NVM context
  */
static void OnNvmDataChange(LmHandlerNvmContextStates_t state);

/**
  * @brief  Store the NVM Data context to the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were stored
  */
static void OnStoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * @brief  Restore the NVM Data context from the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were restored
  */
static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * Will be called each time a Radio IRQ is handled by the MAC layer
  *
  */
static void OnMacProcessNotify(void);

/**
  * @brief Change the periodicity of the uplink frames
  * @param periodicity uplink frames period in ms
  * @note Compliance test protocol callbacks
  */
void OnTxPeriodicityChanged(uint32_t periodicity);

/**
  * @brief Change the confirmation control of the uplink frames
  * @param isTxConfirmed Indicates if the uplink requires an acknowledgement
  * @note Compliance test protocol callbacks
  */
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);

/**
  * @brief Change the periodicity of the ping slot frames
  * @param pingSlotPeriodicity ping slot frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/**
  * @brief Will be called to save the date of the last system time update
  */
void OnSysTimeUpdate(void);

/**
  * @brief Will be called to reset the system
  * @note Compliance test protocol callbacks
  */
static void OnSystemReset(void);

/* USER CODE BEGIN PFP */



/**
  * @brief Send OLN to lorawan or lora P2P
  */
 void Network_slot(void);

/**
  * @brief listen for OLN from lora P2P
  */
 void P2P_slot(void);

/**
  * @brief Send OLN to lora P2P
  */
static void OLNTransmit_LORA(void);

/**
  * @brief  LoRa End Node send request
  */
static void OLNTransmit_LORAWAN(void);

/**
  * @brief build OLN message
  */
static void buildOLNData(void);

/**
  * @brief Compute time until next P2P slot
  */
static uint32_t NextP2PSlotDelay(void);

static void PrintTime(void);

static void CheckLink(void);


/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief LoRaWAN default activation type
  */
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN force rejoin even if the NVM context is restored
  */
static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;
bool OverrideForceRejoin = false;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =              GetBatteryLevel,
  .GetTemperature =               GetTemperatureLevel,
  .GetUniqueId =                  GetUniqueId,
  .GetDevAddr =                   GetDevAddr,
  .OnRestoreContextRequest =      OnRestoreContextRequest,
  .OnStoreContextRequest =        OnStoreContextRequest,
  .OnMacProcess =                 OnMacProcessNotify,
  .OnNvmDataChange =              OnNvmDataChange,
  .OnJoinRequest =                OnJoinRequest,
  .OnTxData =                     OnTxData,
  .OnRxData =                     OnRxData,
  .OnBeaconStatusChange =         OnBeaconStatusChange,
  .OnClassChange =                OnClassChange,
  .OnTxPeriodicityChanged =       OnTxPeriodicityChanged,
  .OnTxFrameCtrlChanged =         OnTxFrameCtrlChanged,
  .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
  .OnSysTimeUpdate  =             OnSysTimeUpdate,
  .OnSystemReset =                OnSystemReset,
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .IsTxConfirmed =            LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .PingSlotPeriodicity =      LORAWAN_DEFAULT_PING_SLOT_PERIODICITY,
  .RxBCTimeout =              LORAWAN_DEFAULT_CLASS_B_C_RESP_TIMEOUT
};

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t NetworkTimer;

/**
  * @brief Timer to handle the application Rx
  */
static UTIL_TIMER_Object_t P2PTimer;

/**
  * @brief Tx Timer period
  */
static UTIL_TIMER_Time_t TxPeriodicity = APP_TX_DUTYCYCLE;

/**
  * @brief Rx Timer period
  */
static UTIL_TIMER_Time_t RxPeriodicity = 8000;

/**
  * @brief Join Timer period
  */
static UTIL_TIMER_Object_t StopJoinTimer;

/* USER CODE BEGIN PV */


/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

uint8_t OLNPayload[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

/**
  * @brief Specifies the state of the application LED
  */

static uint32_t NextP2Pslot = 0;
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
static uint8_t repeat_start;
uint8_t Datarate = 0;


/* USER CODE END PV */
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  repeat_start = 5;

  /* USER CODE BEGIN LoRaWAN_Init_LV */


  /* USER CODE END LoRaWAN_Init_LV */

  /* USER CODE BEGIN LoRaWAN_Init_1 */

  /* USER CODE END LoRaWAN_Init_1 */

//  UTIL_TIMER_Create(&StopJoinTimer, JOIN_TIME, UTIL_TIMER_ONESHOT, OnStopJoinTimerEvent, NULL);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_NetworkTimer), UTIL_SEQ_RFU, Network_slot);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_P2PTimer), UTIL_SEQ_RFU, P2P_slot);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), UTIL_SEQ_RFU, StoreContext);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStart), UTIL_SEQ_RFU, LoRaStart);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaRejoin), UTIL_SEQ_RFU, Rejoin);
//  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), UTIL_SEQ_RFU, StopJoin);

/* Init Info table used by LmHandler*/
  LoraInfo_Init();
  PrintTime();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);

  LmHandlerConfigure(&LmHandlerParams);

  /* USER CODE BEGIN LoRaWAN_Init_2 */
  //Check if Beacon was properly shutdown (and thus context was saved)
  W25qxx_ReadBytes(&app_nvm, LYNKX_CFG_ADDRESS, 4 );


  if (app_nvm == 0xDEADBEEF)
  {
    OverrideForceRejoin = ForceRejoin;
    //APP_LOG(TS_ON, VLEVEL_L, "Context correctly saved at shutdown, continue\r\n");
  }
  else
  {
    OverrideForceRejoin = 1; //If beacon was not shutdown properly, force rejoin
    //APP_LOG(TS_ON, VLEVEL_L, "Context incorrectly saved at shutdown, Force rejoin\r\n");
  }

  uint32_t write_address = LYNKX_CFG_ADDRESS;
  W25qxx_EraseSector(write_address>>12);

  /* USER CODE END LoRaWAN_Init_2 */



  LmHandlerJoin(ActivationType, OverrideForceRejoin);

  /* send on LoRawan network every time timer elapses */
  UTIL_TIMER_Create(&NetworkTimer, TxPeriodicity, UTIL_TIMER_ONESHOT, OnNetworkTimerEvent, NULL);
  UTIL_TIMER_Start(&NetworkTimer);
/////* start P2P RX every time timer elapses */
  UTIL_TIMER_Create(&P2PTimer, RxPeriodicity, UTIL_TIMER_ONESHOT, OnP2PTimerEvent, NULL);
//  /* USER CODE BEGIN LoRaWAN_Init_Last */

  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */


//void updateTXState()
//{
//if (emergency_mode_en == 1)
//  {
//  OnTxPeriodicityChanged(APP_TX_EMERGENCY_DUTYCYCLE);
//}
//
//else
//  {
//    OnTxPeriodicityChanged(APP_TX_DUTYCYCLE);
//}
//
//}

#if 0 /* User should remove the #if 0 tement and adapt the below code according with his needs*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case  BUT1_Pin:
      /* Note: when "EventType == TX_ON_TIMER" this GPIO is not initialized */
      if (EventType == TX_ON_EVENT)
      {
        UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_NetworkTimer), CFG_SEQ_Prio_0);
      }
      break;
    case  BUT2_Pin:
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
      break;
    case  BUT3_Pin:
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
      break;
    default:
      break;
  }
}
#endif

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */


/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
  sprintf((char *)&MSG, "\r\n received data!\r\n\r\n");
  APP_LOG(TS_ON, VLEVEL_M, &MSG);

  uint8_t RXDATA[appData->BufferSize];
  memcpy1((uint8_t *) &RXDATA, appData->Buffer, appData->BufferSize );
  APP_LOG(TS_ON, VLEVEL_M, &RXDATA);
  APP_LOG(TS_ON, VLEVEL_M, "\r\n");

  if (strncmp((const char *)RXDATA, PING, sizeof(PING) - 1) == 0)
  {
      sprintf((char *)&MSG, "PING received\r\n");
     APP_LOG(TS_ON, VLEVEL_M, &MSG);
     emergency_mode_en = 1;
  }

    /* USER CODE END OnRxData_1 */
}


void Network_slot(void)
{
  HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_SET);

  sprintf((char *)&MSG, "\r\n\r\n---------- NETWORK SLOT ------------- \r\n");
  APP_LOG(TS_ON, VLEVEL_L, &MSG);

//  PrintTime();
// NOT reliable. Don't know why but use GPS clock instead
//   if (SysTimeUpdateDate == 0)
//      {
//       sprintf((char *)&MSG, " updating time... \r\n");
//       APP_LOG(TS_ON, VLEVEL_M, &MSG);
//       LmHandlerDeviceTimeReq(); //get network time
//      }
//   else if ((TIMESTAMP - SysTimeUpdateDate) > 604800) //update if older than 1 week or never updated before
//      {
//       sprintf((char *)&MSG, "updating time... \r\n");
//       APP_LOG(TS_ON, VLEVEL_M, &MSG);
//       LmHandlerDeviceTimeReq(); //get network time
//      }

//   To test link check command
//   LmHandlerLinkCheckReq();

    buildOLNData();

    Datarate = (Datarate + 2) % 6;
//    Datarate = (Datarate == 0) ? 2 : 0;
//    Datarate = 4;
    LmHandlerSetTxDatarate(Datarate);

    OLNTransmit_LORAWAN();


    HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_RESET);

}

void P2P_slot(void)
{
  HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_SET);

  sprintf((char *)&MSG, "\r\n\r\n---------- P2P SLOT ------------- \r\n");
  APP_LOG(TS_ON, VLEVEL_L, &MSG);
//  PrintTime();

  if (emergency_mode_en == 1)
  {
      sprintf((char *)&MSG, "Send P2P frame\r\n");
      APP_LOG(TS_ON, VLEVEL_M, &MSG);
      HAL_Delay(2000);
      OLNTransmit_LORA();
  }
  else
  {
      sprintf((char *)&MSG, "P2P RX\r\n");
      APP_LOG(TS_ON, VLEVEL_M, &MSG);
      Radio.SetPublicNetwork( false ) ;
      Radio.Sleep();
      Radio.SetChannel(868000000);
      Radio.SetTxConfig(MODEM_LORA, 14, 0, 0, 7, 1, 8, 0,         true, 0, 0, 1, 3000);
      Radio.SetRxConfig(MODEM_LORA, 0,        7, 1, 0, 8, 5, 0, 0, true, 0, 0, 1, true);
      Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
      Radio.Rx(RX_TIMEOUT_VALUE);
      HAL_Delay(RX_TIMEOUT_VALUE+250);
      Radio.SetPublicNetwork( true );
  }
  HAL_GPIO_WritePin(LED_GREEN1_GPIO_Port, LED_GREEN1_Pin, GPIO_PIN_RESET);
}

static void buildOLNData(void)
{

  BATMSUpdate();

  static uint8_t repeat_stop;
  uint8_t i =0;
  uint8_t tmp;

  AppData.Port = LORAWAN_USER_APP_PORT;

  /////////////////// FTYPE
  uint8_t test_mode = 1;        //b7
  uint8_t relay = 0;        //b6
  uint8_t RFU = 0;        //b5
  uint8_t alert_level;        //b4..3
  uint8_t Alert_msg;        //b2..0
  if (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port,EMERGENCY_Pin)==GPIO_PIN_SET)
  {
    alert_level = 0x2;//alert msg
    Alert_msg = 0x1; //alert request
    repeat_stop = 5;
  }
  else if (repeat_stop > 0)
  {
      alert_level = 0x2;//alert msg
      Alert_msg = 0x5; //alert cancel
      repeat_stop--;
  }
  else
  {
    alert_level = 0x0;//No alert msg
    Alert_msg = 0x0; //ping
    if (repeat_start > 0)
    {
      repeat_start--;
      Alert_msg = 0x1; //beacon started
    }

  }

  tmp = ((test_mode <<7)|(relay <<6)|(alert_level <<3)| Alert_msg) & 0xFF ;
  AppData.Buffer[i++] = tmp;

  /////////////////// FINFO
  uint8_t aSTIME = 1;         //b7
  uint8_t aPOS;               //b6
  uint8_t aDEV_EUID = 0;      //b5
  uint8_t aMSG = 0;       //b4
  uint8_t aID_EVENT = 0;      //b3
    RFU = 0x0;        //b2..0

  aPOS = tnl_tmp_gps_data.fix;
  tmp = ((aSTIME <<7)|(aPOS <<6)|(aDEV_EUID <<5)|(aMSG <<5)|(aID_EVENT <<3)| RFU) & 0xFF ;
  AppData.Buffer[i++] = tmp;

  /////////////////// DINFO
  uint8_t activity_level;     //b7..6
  // uint8_t battery_level;     //b5..3
  uint8_t device_type = 0x2;      //b2..0

  if (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port,EMERGENCY_Pin)==GPIO_PIN_SET)
    activity_level = 0x1;
  else
    activity_level = 0x2;


  tmp = ((activity_level <<6)|((battery_level & 0x7) <<3)| device_type) & 0xFF ;
  AppData.Buffer[i++] = tmp;

  /////////////////// STIME
//  uint32_t STIME = HAL_GetTick();
//  SysTime_t curtime = SysTimeGet();
//
//  AppData.Buffer[i++] = (uint8_t)((curtime.Seconds >> 24) & 0xFF);
//  AppData.Buffer[i++] = (uint8_t)((curtime.Seconds >> 16) & 0xFF);
//  AppData.Buffer[i++] = (uint8_t)((curtime.Seconds >> 8 ) & 0xFF);
//  AppData.Buffer[i++] = (uint8_t)((curtime.Seconds      ) & 0xFF);

  AppData.Buffer[i++] = (uint8_t)((tnl_tmp_gps_data.timestamp >> 24) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)((tnl_tmp_gps_data.timestamp >> 16) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)((tnl_tmp_gps_data.timestamp >> 8 ) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)((tnl_tmp_gps_data.timestamp      ) & 0xFF);

  sprintf((char *)&MSG, "timestamp = %ld \r\n",tnl_tmp_gps_data.timestamp);
  APP_LOG(TS_ON, VLEVEL_M, &MSG);

  /////////////////// POS
  AppData.Buffer[i++] = (uint8_t)(((tnl_tmp_gps_data.latitude_fixed/10) >> 24) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(((tnl_tmp_gps_data.latitude_fixed/10) >> 16) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(((tnl_tmp_gps_data.latitude_fixed/10) >> 8 ) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(((tnl_tmp_gps_data.latitude_fixed/10)      ) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(((tnl_tmp_gps_data.longitude_fixed/10) >> 24) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(((tnl_tmp_gps_data.longitude_fixed/10) >> 16) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(((tnl_tmp_gps_data.longitude_fixed/10) >> 8 ) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(((tnl_tmp_gps_data.longitude_fixed/10)      ) & 0xFF);

  sprintf((char *)&MSG, "coordinates = %ld, %ld \r\n",tnl_tmp_gps_data.latitude_fixed,tnl_tmp_gps_data.longitude_fixed);
  APP_LOG(TS_ON, VLEVEL_M, &MSG);


//  /////////////////// DevEID
//  uint64_t EUID ;
//  memcpy1( &EUID, SecureElementGetDevEui( ), 8 );
//  uint8_t EUIDlsb1 = (EUID >> 56) & 0xFF;
//  uint8_t EUIDlsb2 = (EUID >> 48) & 0xFF;
//
//  if (alert_level > 0)
//    {
//    AppData.Buffer[i++] = EUIDlsb2;
//    AppData.Buffer[i++] = EUIDlsb1;
//    }

  /////////////////// Message size
  AppData.BufferSize = i;

  memcpy(OLNPayload, AppData.Buffer, i);

  sprintf((char *)&MSG, "payload = 0x%x%x%x%x%x%x%x%x%x%x%x%x%x%x%x \r\n",\
	  AppData.Buffer[0],AppData.Buffer[1],AppData.Buffer[2],AppData.Buffer[3],AppData.Buffer[4],\
	  AppData.Buffer[5],AppData.Buffer[6],AppData.Buffer[7],AppData.Buffer[8],AppData.Buffer[9],\
	  AppData.Buffer[10],AppData.Buffer[11],AppData.Buffer[12],AppData.Buffer[13],AppData.Buffer[14]);
  APP_LOG(TS_ON, VLEVEL_M, &MSG);

  /* USER CODE END buildOLNData */
}

static uint32_t NextP2PSlotDelay(void)
  {
    uint32_t Delay = 0;
    uint32_t timestamp = 0;

    if (SysTimeUpdateDate!=0)
    {
      timestamp = TIMESTAMP;
      NextP2Pslot = (timestamp/60+1)*60;
      Delay = (NextP2Pslot-timestamp)*1000;
      sprintf((char *)&MSG, "next P2P slot in  : %lu s\r\n", Delay/1000);
      APP_LOG(TS_ON, VLEVEL_M, &MSG);
    }
    return Delay;
  }

static void PrintTime(void)
  {
    struct tm heure;
    SysTimeLocalTime(TIMESTAMP,&heure);
    sprintf((char *)&MSG, " -----       SysTime      = %lu s\r\n",TIMESTAMP);
    APP_LOG(TS_ON, VLEVEL_M, &MSG);
    sprintf((char *)&MSG, " -----       MCUTime      = %lu s %d ms\r\n",SysTimeGetMcuTime().Seconds, SysTimeGetMcuTime().SubSeconds);
    APP_LOG(TS_ON, VLEVEL_M, &MSG);
    sprintf((char *)&MSG, " -----    SysTime Updated = %lu s ago\r\n",((SysTimeUpdateDate==0) ? 0 : (SysTimeGetMcuTime().Seconds - SysTimeUpdateDate)));
    APP_LOG(TS_ON, VLEVEL_M, &MSG);
    sprintf((char *)&MSG, " -----      GMT           %lu:%lu:%lu %lu/%lu/%lu\r\n",  heure.tm_hour, heure.tm_min,heure.tm_sec, heure.tm_mday, heure.tm_mon+1, heure.tm_year-100);
    APP_LOG(TS_ON, VLEVEL_M, &MSG);
  }

static void OLNTransmit_LORA(void)
{
  /* USER CODE BEGIN OLNTransmit_LORAWAN */

  if(LoRaMacIsBusy())
    return;

  LoRaMacStop();

  Radio.SetPublicNetwork( false ) ;

  Radio.Sleep();
  /* Radio Set frequency */
  Radio.SetChannel(868000000);
  /*fills tx buffer*/
  Radio.SetTxConfig(MODEM_LORA, 14, 0, 0, 7, 1, 8, 0,         true, 0, 0, 1, 3000);
  Radio.SetRxConfig(MODEM_LORA, 0,        7, 1, 0, 8, 5, 0, 0, true, 0, 0, 1, true);

  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
  memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);
  Radio.Sleep();
  HAL_Delay(Radio.GetWakeupTime() + RX_TIME_MARGIN);
  memset(BufferTx, 0xE0, sizeof(PING) - 1);
  memcpy(BufferTx+1, PING, sizeof(PING) - 1);
  Radio.Send(BufferTx, 64);
  HAL_Delay(250);

  Radio.SetPublicNetwork( true );

  LoRaMacStart();

  /* USER CODE BEGIN OLNTransmit_LORA */

}

static void OLNTransmit_LORAWAN(void)
{
  UTIL_TIMER_Time_t nextTxIn = 0;

  /* USER CODE BEGIN OLNTransmit_LORAWAN */
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);

  if (LORAMAC_HANDLER_SUCCESS == status)
  {
    APP_LOG(TS_ON, VLEVEL_M, "LmHandlerSend LORAMAC_HANDLER_SUCCESS\r\n");
  }
  else if (LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED == status)
  {
    APP_LOG(TS_ON, VLEVEL_M, "LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED\r\n");
    nextTxIn = LmHandlerGetDutyCycleWaitTime();
    sprintf((char *)&MSG, "next network slot in  : %lu s\r\n", nextTxIn/1000);
    APP_LOG(TS_ON, VLEVEL_M, &MSG);
    UTIL_TIMER_Stop(&NetworkTimer);
    UTIL_TIMER_SetPeriod(&NetworkTimer, nextTxIn);
    UTIL_TIMER_Start(&NetworkTimer);
  }
  /* USER CODE END OLNTransmit_LORAWAN */

}

static void OnNetworkTimerEvent(void *context)
{
  /* USER CODE BEGIN OnNetworkTimerEvent_1 */
//  UTIL_TIMER_Time_t nextTxIn = 0;

//  GNSS_On_request = 1;

  uint32_t Delay = 0;
  uint32_t timestamp = 0;
  timestamp = TIMESTAMP;

  /* USER CODE END OnNetworkTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_NetworkTimer), CFG_SEQ_Prio_0);

  Delay = 0.9*TxPeriodicity + randr(0,0.2*TxPeriodicity); //randomize slot by +/- 10%

  if(NextP2Pslot!=0)
    {
      // check if Next network slot overlaps P2P slot
      if(((Delay/1000)>(NextP2Pslot-timestamp - 15)) && ((Delay/1000)<(NextP2Pslot-timestamp + 15)))
      {
  // back off if overlapping slots
  APP_LOG(TS_OFF, VLEVEL_M, "postponed network slot to free P2P slot\r\n");
  Delay = Delay + 20000;
      }
    }

  sprintf((char *)&MSG, "next network slot in  : %lu s\r\n", Delay/1000);
  APP_LOG(TS_ON, VLEVEL_M, &MSG);
  UTIL_TIMER_Stop(&NetworkTimer);
  UTIL_TIMER_SetPeriod(&NetworkTimer, Delay);
  UTIL_TIMER_Start(&NetworkTimer);

  /* USER CODE BEGIN OnNetworkTimerEvent_2 */

  /* USER CODE END OnNetworkTimerEvent_2 */
}

static void OnP2PTimerEvent(void *context)
{
  /* USER CODE BEGIN OnP2PTimerEvent_1 */

//  APP_LOG(TS_OFF, VLEVEL_M, "\r\n---------- OnP2PTimerEvent\r\n");
//  PrintTime();

  /* USER CODE END OnP2PTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_P2PTimer), CFG_SEQ_Prio_0);

  /*Wait for next P2P slot*/
  UTIL_TIMER_Stop(&P2PTimer);
  UTIL_TIMER_SetPeriod(&P2PTimer, NextP2PSlotDelay());
  UTIL_TIMER_Start(&P2PTimer);

  /* USER CODE BEGIN OnP2PTimerEvent_2 */

  /* USER CODE END OnP2PTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */


/* USER CODE END PrFD_LedEvents */



static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
  //save context after join to keep track of DevNonce used to join
  if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
  {
//    StoreContext();
      LmHandlerNvmDataStore();
  }
  /* USER CODE END OnJoinRequest_1 */
}

static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params)
{
  /* USER CODE BEGIN OnBeaconStatusChange_1 */
  /* USER CODE END OnBeaconStatusChange_1 */
}

static void OnClassChange(DeviceClass_t deviceClass)
{
  /* USER CODE BEGIN OnClassChange_1 */
  /* USER CODE END OnClassChange_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

 void OnTxPeriodicityChanged(uint32_t periodicity)
{
  /* USER CODE BEGIN OnTxPeriodicityChanged_1 */

   sprintf((char *)&MSG, "**update periodicity to - %lu s** \r\n", periodicity/1000);
   APP_LOG(TS_ON, VLEVEL_M, &MSG);

  /* USER CODE END OnTxPeriodicityChanged_1 */
  TxPeriodicity = periodicity;

  if (TxPeriodicity == 0)
  {
    /* Revert to application default periodicity */
    TxPeriodicity = APP_TX_DUTYCYCLE;
  }

  /* Update timer periodicity */
  UTIL_TIMER_Stop(&NetworkTimer);
  UTIL_TIMER_SetPeriod(&NetworkTimer, TxPeriodicity);
  UTIL_TIMER_Start(&NetworkTimer);
  /* USER CODE BEGIN OnTxPeriodicityChanged_2 */

  /* USER CODE END OnTxPeriodicityChanged_2 */
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
  /* USER CODE BEGIN OnTxFrameCtrlChanged_1 */

  /* USER CODE END OnTxFrameCtrlChanged_1 */
  LmHandlerParams.IsTxConfirmed = isTxConfirmed;
  /* USER CODE BEGIN OnTxFrameCtrlChanged_2 */

  /* USER CODE END OnTxFrameCtrlChanged_2 */
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_1 */

  /* USER CODE END OnPingSlotPeriodicityChanged_1 */
  LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_2 */

  /* USER CODE END OnPingSlotPeriodicityChanged_2 */
}

static void OnSystemReset(void)
{
  /* USER CODE BEGIN OnSystemReset_1 */

  /* USER CODE END OnSystemReset_1 */
  if ((LORAMAC_HANDLER_SUCCESS == LmHandlerHalt()) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
  {
    NVIC_SystemReset();
  }
  /* USER CODE BEGIN OnSystemReset_Last */

  /* USER CODE END OnSystemReset_Last */
}

void OnSysTimeUpdate(void)
{

  /* USER CODE BEGIN OnSysTimeUpdate_1 */
  APP_LOG(TS_OFF, VLEVEL_M, " ---------- OnSysTimeUpdate\r\n");

//  UTIL_TIMER_Stop(&NetworkTimer);

  /* USER CODE END OnSysTimeUpdate_1 */
  SysTimeUpdateDate = SysTimeGetMcuTime().Seconds;
//  PrintTime();

  NextP2PSlotDelay();

//  UTIL_TIMER_Stop(&P2PTimer);
//  UTIL_TIMER_SetPeriod(&P2PTimer, NextP2PSlotDelay());
//  UTIL_TIMER_Start(&P2PTimer);

  /* USER CODE BEGIN OnSysTimeUpdate_Last */

  /* USER CODE END OnSysTimeUpdate_Last */
}

//static void StopJoin(void)
//{
//  /* USER CODE BEGIN StopJoin_1 */
//
//  /* USER CODE END StopJoin_1 */
//
//  UTIL_TIMER_Stop(&NetworkTimer);
//
//  if (LORAMAC_HANDLER_SUCCESS != LmHandlerStop())
//  {
//    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stop on going ...\r\n");
//  }
//  else
//  {
//    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stopped\r\n");
//    if (LORAWAN_DEFAULT_ACTIVATION_TYPE == ACTIVATION_TYPE_ABP)
//    {
//      ActivationType = ACTIVATION_TYPE_OTAA;
//      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to OTAA mode\r\n");
//    }
//    else
//    {
//      ActivationType = ACTIVATION_TYPE_ABP;
//      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to ABP mode\r\n");
//    }
//    LmHandlerConfigure(&LmHandlerParams);
//    LmHandlerJoin(ActivationType, true);
//    UTIL_TIMER_Start(&NetworkTimer);
//  }
//  UTIL_TIMER_Start(&StopJoinTimer);
//  /* USER CODE BEGIN StopJoin_Last */
//
//  /* USER CODE END StopJoin_Last */
//}

//static void OnStopJoinTimerEvent(void *context)
//{
//  /* USER CODE BEGIN OnStopJoinTimerEvent_1 */
//
//  /* USER CODE END OnStopJoinTimerEvent_1 */
//  if (ActivationType == LORAWAN_DEFAULT_ACTIVATION_TYPE)
//  {
//    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
//  }
//  /* USER CODE BEGIN OnStopJoinTimerEvent_Last */
//
//  /* USER CODE END OnStopJoinTimerEvent_Last */
//}

static void StoreContext(void)
{
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  /* USER CODE BEGIN StoreContext_1 */

  /* USER CODE END StoreContext_1 */
  status = LmHandlerNvmDataStore();

  if (status == LORAMAC_HANDLER_NVM_DATA_UP_TO_DATE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA UP TO DATE\r\n");
  }
  else if (status == LORAMAC_HANDLER_ERROR)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORE FAILED\r\n");
  }
  /* USER CODE BEGIN StoreContext_Last */

  uint32_t write_address = LYNKX_CFG_ADDRESS;
  uint32_t i=0;
  app_nvm = 0xdeadbeef;
  uint8_t* nvmBytes = &app_nvm;

  W25qxx_WritePage(nvmBytes,write_address>>8,0,256-4);

//  UTIL_TIMER_Stop(&NetworkTimer);
//  UTIL_TIMER_Stop(&P2PTimer);

  next_power_state = next_power_state + 1;


  /* USER CODE END StoreContext_Last */
}

static void LoRaStart(void)
{

//  UTIL_TIMER_Start(&P2PTimer);

}

static void Rejoin(void)
{

  /* USER CODE BEGIN Rejoin_1 */

  /* USER CODE END Rejoin_1 */
  LmHandlerJoin(ActivationType, 1);
  /* USER CODE BEGIN Rejoin_Last */

  /* USER CODE END Rejoin_Last */
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state)
{
  /* USER CODE BEGIN OnNvmDataChange_1 */

  /* USER CODE END OnNvmDataChange_1 */
  if (state == LORAMAC_HANDLER_NVM_STORE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORED\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA RESTORED\r\n");
  }
  /* USER CODE BEGIN OnNvmDataChange_Last */

  /* USER CODE END OnNvmDataChange_Last */
}

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnStoreContextRequest_1 */

  /* USER CODE END OnStoreContextRequest_1 */
  /* store nvm in flash */
  uint32_t i=0;
  uint32_t write_address = LORAWAN_NVM_BASE_ADDRESS;

  W25qxx_EraseSector(write_address>>12);

  uint8_t* nvmBytes = (uint8_t*)nvm;
  W25qxx_WriteSector(nvmBytes,write_address>>12,0,0x1000-nvm_size);
  /* USER CODE BEGIN OnStoreContextRequest_Last */

  /* USER CODE END OnStoreContextRequest_Last */
}

static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnRestoreContextRequest_1 */

  /* USER CODE END OnRestoreContextRequest_1 */
  W25qxx_ReadBytes((uint8_t *) nvm, LORAWAN_NVM_BASE_ADDRESS, nvm_size );

  /* USER CODE BEGIN OnRestoreContextRequest_Last */

  /* USER CODE END OnRestoreContextRequest_Last */
}

//static void CheckLink(void)
//{
//  /* USER CODE BEGIN CheckLink_1 */
//  LmHandlerErrorStatus_t status;
//  /* USER CODE END CheckLink_1 */
//  status = LmHandlerLinkCheckReq();
//
//  if (status == LORAMAC_HANDLER_SUCCESS)
//    {
//    APP_LOG(TS_OFF, VLEVEL_M, "Check Link => success\r\n");
//    }
//  else
//    {
//    APP_LOG(TS_OFF, VLEVEL_M, "Check Link => failure\r\n");
//    }
//  /* USER CODE BEGIN CheckLink_Last */
//
//  /* USER CODE END CheckLink_Last */
//}

