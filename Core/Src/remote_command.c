#include "main.h"
#include "remote_command.h"
#include "w25qxx.h"
#include "sys_app.h"

extern CRC_HandleTypeDef hcrc;



uint8_t ProcessRemoteCommand(uint8_t* data_buffer, uint8_t Nb_bytes, uint8_t* aDataBuffer)
{

  uint8_t Nb_bytes_to_send=0, j=0;

  //return CMD ID
  aDataBuffer[j] = data_buffer[0]; //ID cmd
  j++;

  sprintf((char *)&MSG, " cmd ID = %x      ",data_buffer[0]);
  APP_LOG(TS_ON, VLEVEL_L, &MSG);

  //Check CRC
  if(data_buffer[Nb_bytes-1] != HAL_CRC_Calculate(&hcrc, (uint32_t *)data_buffer, Nb_bytes-1))
  {
    aDataBuffer[j] = LYNKX_ERROR_BAD_CRC8;
    j++;
    APP_LOG(TS_ON, VLEVEL_L, "LYNKX_ERROR_BAD_CRC8\r\n");
  }
  else if  (data_buffer[1] == LYNKX_CMD_GET_FIRM_VER) // get firmware version command
  {
    APP_LOG(TS_ON, VLEVEL_L, "LYNKX_CMD_GET_FIRM_VER\r\n");
    aDataBuffer[j] = LYNKX_ERROR_OK;
    j++;
    aDataBuffer[j] = 0x00; //major
    j++;
    aDataBuffer[j] = 0x07; //minor
    j++;

  }
  else if  (data_buffer[1] == LYNKX_CMD_FIRMWARE_CACHE) // get firmware version write firmware to EXT flash
  {
    APP_LOG(TS_ON, VLEVEL_L, "LYNKX_CMD_FIRMWARE_CACHE\r\n");
    aDataBuffer[j] = LYNKX_ERROR_OK;
    j++;

    uint32_t write_address;
    write_address = FIRMWARE_OTA_BASE_ADDRESS + ((uint32_t) data_buffer[2]<<16) + ((uint32_t) data_buffer[3]<<8) + (uint32_t) data_buffer[4 ];

    if (write_address == 0x00)
    {
//      W25qxx_EraseChip();
      W25qxx_EraseBlock(0);
      W25qxx_EraseBlock(1);
      W25qxx_EraseBlock(2);
      W25qxx_EraseBlock(3);
    }

    // warning : buffer length should not overflow on next page (of 256 bytes)
    W25qxx_WritePage(data_buffer+5,write_address>>8,write_address-((write_address>>8)<<8),Nb_bytes-6);
  }
  else if  (data_buffer[1] == LYNKX_CMD_RESTART) // get firmware version write firmware to EXT flash
  {
      APP_LOG(TS_ON, VLEVEL_L, "LYNKX_CMD_RESTART\r\n");
      aDataBuffer[j] = LYNKX_ERROR_OK;
      j++;
      next_power_state = RESTART;
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
  }
  else if  (data_buffer[1] == LYNKX_CMD_PUSH_FIRMWARE) // check firmware CRC and validate EXT memory content if OK
  {
    APP_LOG(TS_ON, VLEVEL_L, "LYNKX_CMD_PUSH_FIRMWARE\r\n");

    BATMSUpdate();
    if (battery_level<5) // Battery too low to update firmware
    {
      W25qxx_EraseSector(65);
      aDataBuffer[j] = LYNKX_ERROR_FIRM_LOW_BAT;
      j++;
    }
    else // get firmware information from BLE and check consistency with firmware in flash
    {
      //BOOTLOADER CONFIG
      uint8_t  *cfg_ptr;
      BootLoaderConfig_t BootLoaderConfig;
      BootLoaderConfig_t  *myBootLoaderConfig = &BootLoaderConfig;
      cfg_ptr = (uint8_t *)&BootLoaderConfig; //just used to access memory byte by byte

      //udate config from BLE
      uint8_t SIZE_H  =  data_buffer[2];
      uint8_t SIZE_M  =  data_buffer[3];
      uint8_t SIZE_L  =  data_buffer[4];
      uint8_t VER_H   =  data_buffer[5];
      uint8_t VER_L   =  data_buffer[6];
      uint8_t CRC_HH  =  data_buffer[7];
      uint8_t CRC_H   =  data_buffer[8];
      uint8_t CRC_M   =  data_buffer[9];
      uint8_t CRC_L   =  data_buffer[10];

      myBootLoaderConfig->NewFirmwareSize = (SIZE_H<<16) + (SIZE_M<<8) + SIZE_L;
      myBootLoaderConfig->NewFirmwareCRC = (CRC_HH<<24) + (CRC_H<<16) + (CRC_M<<8) + CRC_L;
      myBootLoaderConfig->NewFirmwareVersion_Major = VER_H;
      myBootLoaderConfig->NewFirmwareVersion_Minor = VER_L;
      myBootLoaderConfig->CurrentFirmwareVersion_Major = FIMRWARE_VERSION_MAJOR;
      myBootLoaderConfig->CurrentFirmwareVersion_Minor = FIMRWARE_VERSION_MAJOR;
      myBootLoaderConfig->RFU0 = 0x00;
      myBootLoaderConfig->RFU1 = 0x00;
      myBootLoaderConfig->RFU2 = 0x00;
      myBootLoaderConfig->RFU3 = 0x00;

      //compute CRC32 of the entire firmware on the external memory
      uint8_t i;
      uint8_t a_read_buffer[256];
      MX_CRC32_Init();
      W25qxx_ReadPage(a_read_buffer,0,0,0);
      uint32_t res = HAL_CRC_Calculate(&hcrc, (uint32_t *)a_read_buffer, 64);
      for (i=1; i<myBootLoaderConfig->NewFirmwareSize/256; i++)
      {
	W25qxx_ReadPage(a_read_buffer,i,0,0);
	res = HAL_CRC_Accumulate(&hcrc, (uint32_t *)a_read_buffer, 64);
      }
      W25qxx_ReadPage(a_read_buffer,i,0,256-myBootLoaderConfig->NewFirmwareSize%256);
      res = HAL_CRC_Accumulate(&hcrc, (uint32_t *)a_read_buffer, (myBootLoaderConfig->NewFirmwareSize%256)/4);

      MX_CRC8_Init();// set back CRC8 as default

      if(res != myBootLoaderConfig->NewFirmwareCRC)
      {
	aDataBuffer[j] = LYNKX_ERROR_FIRM_BAD_CRC32; //CRC is not correct => return fail to smartphone
	j++;
	W25qxx_EraseSector(65);
      }
      else //CRC is correct => save config
      {
	aDataBuffer[j] = LYNKX_ERROR_OK;
	j++;
	W25qxx_EraseSector(65);
	W25qxx_WritePage(cfg_ptr, BOOTLOADER_CFG_ADDRESS>>8,0, 256 - sizeof(BootLoaderConfig));
	next_power_state = RESTART;
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
      }
    }
  }
  else
  {
    APP_LOG(TS_ON, VLEVEL_L, "LYNKX_ERROR_NOT_SUPPORTED\r\n");
    aDataBuffer[j] = LYNKX_ERROR_NOT_SUPPORTED;
    j++;
  }

  aDataBuffer[j] = HAL_CRC_Calculate(&hcrc, (uint32_t *)aDataBuffer, j);//CRC
  j++;

  return j++;

}
