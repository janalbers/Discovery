/*----------------------------------------------------------------------------
 * Name:    CAN.h
 * Purpose: low level CAN definitions
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2009-2011 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __CAN_H
#define __CAN_H

#define STANDARD_FORMAT  0
#define EXTENDED_FORMAT  1

#define CAN_SYS_ID_BOOT        0x0  << 25
#define CAN_SYS_ID_NORMAL_BUS1 0x1 << 25
#define CAN_SYS_ID_INFO_BUS1   0x4 << 25
#define CAN_SYS_ID_NORMAL_BUS2 0x2 << 25
#define CAN_SYS_ID_INFO_BUS2   0x6 << 25

#define CAN_SID_NORMAL_BUS1		0x00000000
#define CAN_SID_NORMAL_BUS1		0x00000000

#define CAN_DEV_PRIORITY_HIGH	0x1 << 22
#define CAN_DEV_PRIORITY_MEDIUM	0x2 << 22
#define CAN_DEV_PRIORITY_LOW	0x4 << 22
#define CAN_DEV_PRIORITY_LOWEST	0x8 << 22

#define CAN_DEV_GROUP_EXTRUDER  0x05 << 16
#define CAN_DEV_GROUP_MAIN		0x09 << 16
#define CAN_DEV_GROUP_BEEHEAD	0x0C << 16
#define CAN_DEV_GROUP_MOD		0x11 << 16
#define CAN_DEV_GROUP_HEAT		0x15 << 16
#define CAN_DEV_GROUP_SMART		0x19 << 16
#define CAN_DEV_GROUP_FILTER	0x1C << 16
#define CAN_DEV_GROUP_DOOR		0x21 << 16
#define CAN_DEV_GROUP_LED 		0x25 << 16
#define CAN_DEVID_0xF           0xF << 12   // 4bit device ID
#define CAN_SUBID_0xFFF			0x00000FFF  // 12bit subid

#define MY_CAN_IDENTIFIER	CAN_SYS_ID_NORMAL_BUS1 | CAN_SID_NORMAL_BUS1 | CAN_DEV_PRIORITY_LOWEST \
						    |CAN_DEV_GROUP_BEEHEAD | CAN_DEVID_0xF | CAN_SUBID_0xFFF

#define DATA_FRAME       0
#define REMOTE_FRAME     1

typedef struct  {
  unsigned int   id;                    /* 29 bit identifier */
  unsigned char  data[8];               /* Data field */
  unsigned char  len;                   /* Length of data field in bytes */
  unsigned char  format;                /* 0 - STANDARD, 1- EXTENDED IDENTIFIER */
  unsigned char  type;                  /* 0 - DATA FRAME, 1 - REMOTE FRAME */
} CAN_msg;

/* Functions defined in module CAN.c */
void CAN_setup         (uint32_t ctrl);
void CAN_start         (uint32_t ctrl);
void CAN_waitReady     (uint32_t ctrl);
void CAN_wrMsg         (uint32_t ctrl, CAN_msg *msg);
void CAN_rdMsg         (uint32_t ctrl, CAN_msg *msg);
void CAN_wrFilter      (uint32_t ctrl, uint32_t id, uint8_t filter_type);
void MX_CAN1_Init		(void);
void MX_CAN2_Init		(void);
void CAN_Init 			(uint32_t identifier);
void CAN_init_message	(uint32_t identifier);
void CAN_send_dummy_message(uint32_t value0);

void CAN_testmode      (uint32_t ctrl, uint32_t testmode);

extern CAN_msg       CAN_TxMsg[2];      /* CAN messge for sending */
extern CAN_msg       CAN_RxMsg[2];      /* CAN message for receiving */                                
extern uint32_t  CAN_TxRdy[2];      /* CAN HW ready to transmit a message */
extern uint32_t  CAN_RxRdy[2];      /* CAN HW received a message */

#endif


