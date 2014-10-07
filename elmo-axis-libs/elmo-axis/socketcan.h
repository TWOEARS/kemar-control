/*
 * Copyright (c) 2005-2011,2014 CNRS/LAAS
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#ifndef _SOCKETCAN_H
#define _SOCKETCAN_H

/** @file
 *
 * Definitions for CAN bus communication using the Linux socketcan API
 */

/**
 * @brief CAN message structure.
 */
typedef struct CAN_MSG {
	uint16_t id;		/**< 12 bits message ID */
	uint8_t type;		/**< message type (usually 0) */
	uint8_t len;		/**< message len [0..8] */
	uint8_t data[8];	/**< actual data */
} CAN_MSG;

extern int socketcanInit(const char *);
extern void socketcanEnd(int);
extern int socketcanTransmitMsg(int, CAN_MSG *);
extern int socketcanReceiveMsg(int, CAN_MSG *);
extern int socketcanReceiveMsgWait(int, CAN_MSG *, int);
extern void canMsgSet(CAN_MSG *, int, int, int, int, int, int, int, int, int);
extern int canMsgGetInt(CAN_MSG *, int);
extern float canMsgGetFloat(CAN_MSG *, int);
extern void canMsgShow(FILE *, CAN_MSG *);

#endif
