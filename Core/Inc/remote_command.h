/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : remote_command.h
  * @brief          : Header for remote_command.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 LYNKX.
  * All rights reserved.
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REMOTE_COMMAND_H
#define __REMOTE_COMMAND_H

const LYNKX_ERROR_OK                    = 0x00;
const LYNKX_ERROR_KO                    = 0x01;
const LYNKX_ERROR_BAD_PARAMETER         = 0x02;
const LYNKX_ERROR_NOT_SUPPORTED         = 0x03;
const LYNKX_ERROR_BAD_CRC8              = 0x04;
const LYNKX_ERROR_FIRM_BAD_CRC32        = 0x05;
const LYNKX_ERROR_FIRM_LOW_BAT          = 0x06;

#define LYNKX_CMD_FIRMWARE_CACHE 0x01
#define LYNKX_CMD_PUSH_FIRMWARE 0x02
#define LYNKX_CMD_SET_ADMIN 0x03
#define LYNKX_CMD_RESTART 0x04
#define LYNKX_CMD_GET_FIRM_VER 0x05
#define LYNKX_CMD_GET_HARD_VER 0x06
#define LYNKX_CMD_SET_USETTINGS 0x07
#define LYNKX_CMD_GET_USETTINGS 0x08
#define LYNKX_CMD_SET_BSETTINGS 0x09
#define LYNKX_CMD_GET_BSETTINGS 0x0A
#define LYNKX_CMD_ACTIVATE_RMA 0x0B
#define LYNKX_CMD_START_LIVE 0x0C
#define LYNKX_CMD_SET_SHIPPING_MODE 0x0D
#define LYNKX_CMD_SET_ALTITUDE 0x0E




uint8_t ProcessRemoteCommand(uint8_t* data_buffer, uint8_t Nb_bytes, uint8_t* aDataBuffer);

#endif /* __REMOTE_COMMAND_H */
