/*
 * Copyright (c) 2014 CNRS/LAAS
 *
 * Permission to use, copy, modify, and/or distribute this software for any
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

#include <err.h>
#include <errno.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <elmo-axis/socketcan.h>

//#define CAN_BUS_DEBUG 1

/**
 ** Interface with the linux socket CAN layer
 **/

/*----------------------------------------------------------------------*/
/**
 * Initialize the communication with the CAN bus controller
 */
int
socketcanInit(const char *deviceName)
{
	struct ifreq ifr;
	struct sockaddr_can addr;
	int s;

	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", deviceName);
	if (ioctl(s, SIOCGIFINDEX, &ifr) != 0) {
		warn("SIOCGIFINDEX %s", deviceName);
		return -1;
	}
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		warn("bind");
		return -1;
	}
	return s;
}

/*----------------------------------------------------------------------*/
/**
 * End the communication with the CAN bus controller
 */
void
socketcanEnd(int socket)
{

	close(socket);
}

/*----------------------------------------------------------------------*/
/**
 * Send a message on the BUS
 */
int
socketcanTransmitMsg(int s, CAN_MSG *msg)
{
	ssize_t nbytes;
	struct can_frame frame;

#ifdef CAN_BUS_DEBUG
	printf("-> "); canMsgShow(stdout, msg);
#endif
	frame.can_id = msg->id;
	frame.can_dlc = msg->len;
	memcpy(&frame.data, msg->data, msg->len);

	nbytes = write(s, &frame, sizeof(struct can_frame));
#ifdef CAN_BUS_DEBUG
	if (nbytes < 0)
		warn("socketcanTransmitMsg: write");
#endif
	return nbytes == sizeof(struct can_frame) ? 0 : -1;
}

/*----------------------------------------------------------------------*/
/**
 * Wait for a message and receive it
 *
 * @param timeo number of milli-seconds to wait : -1 => wait forever
 */
int
socketcanReceiveMsgWait(int s, CAN_MSG *msg, int timeo)
{
	struct can_frame frame;
	struct pollfd pfd;
	ssize_t nbytes;
	int rc;

	if (timeo >= 0) {
		pfd.fd = s;
		pfd.events = POLLIN;
		for (;;) {
			rc = poll(&pfd, 1, timeo);
			if (rc == -1 && errno == EINTR)
				continue;
			if (rc == -1)
				return rc;
			if (rc == 0) /* timeout */ {
				errno = ETIMEDOUT;
#ifdef CAN_BUS_DEBUG
				printf("timeout\n");
#endif
				return -2;
			}
			break;
		}
	}
	nbytes = read(s, &frame, sizeof(struct can_frame));
	if (nbytes < 0) {
#ifdef CAN_BUS_DEBUG
		warn("socketcanReceiveMsg: read");
#endif
		return -1;
	}

	/* paranoid check ... */
	if (nbytes < sizeof(struct can_frame)) {
#ifdef CAN_BUS_DEBUG
		warnx("socketcanReceiveMsg: read: incomplete CAN frame");
#endif
		return -1;
	}
	msg->id = frame.can_id;
	msg->len = frame.can_dlc;
	memcpy(msg->data, frame.data, frame.can_dlc);
#ifdef CAN_BUS_DEBUG
	printf("<- "); canMsgShow(stdout, msg);
#endif
	return 0;
}

/*----------------------------------------------------------------------*/
/**
 * Non-Blocking check for next message and receive it
 */
int
socketcanReceiveMsg(int s, CAN_MSG *m)
{
	return socketcanReceiveMsgWait(s, m, 0);
}

/*----------------------------------------------------------------------*/
/**
 * Initialize a CAN message
 */
void
canMsgSet(CAN_MSG *m, int id,
    int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8)
{
	m->id = id;
	m->len = 8;
	m->type = 0;
	m->data[0] = p1;
	m->data[1] = p2;
	m->data[2] = p3;
	m->data[3] = p4;
	m->data[4] = p5;
	m->data[5] = p6;
	m->data[6] = p7;
	m->data[7] = p8;
}

/*----------------------------------------------------------------------*/
/*
 * Extract a little-endian 32 bit integer from a CAN message at given offset
 */
int
canMsgGetInt(CAN_MSG *m, int offset)
{
	return (m->data[offset+3] << 24) | (m->data[offset+2] << 16)
	    | (m->data[offset+1] << 8) | m->data[offset];
}
/*
 * Extract a float from a CAN message at given offset
 */
float
canMsgGetFloat(CAN_MSG *m, int offset)
{
	union {
		int i;
		float f;
	} msg;

	msg.i = (m->data[offset+3] << 24) | (m->data[offset+2] << 16)
	    | (m->data[offset+1] << 8) | m->data[offset];
	return msg.f;
}


/*----------------------------------------------------------------------*/
/**
 * Display the raw contents of a CAN message on stdout
 */
void
canMsgShow(FILE *out, CAN_MSG *m)
{
	int i;

	fprintf(out, "ID: 0x%3x ", m->id);
	fprintf(out, "Type: 0x%3x ", m->type);
	fprintf(out, "len: %2d ", m->len);
	fprintf(out, "data: ");
	for (i = 0; i < m->len; i++) {
		fprintf(out, " 0x%02x", m->data[i]);
	}
	fprintf(out, "\n");
}
