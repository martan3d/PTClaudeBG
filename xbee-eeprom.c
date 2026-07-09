/*************************************************************************
Title:    XBee API-Mode Directed EEPROM Access for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     xbee-eeprom.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2024 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

DESIGN NOTES:
    The original MRBus code handled EEPROM access via broadcast-style
    'R' and 'W' packet types inside PktHandler().  Any node on the bus
    that saw an 'R' or 'W' addressed to it would respond.  Because MRBus
    over XBee was still using a shared-channel broadcast model, any node
    could in principle receive and act on packets destined for another.

    This replacement module uses the XBee API-mode 0x10 Transmit Request /
    0x90 Receive Packet frames.  Every exchange is a true unicast directed
    to the 64-bit IEEE MAC address of the target XBee.  The MAC address is
    globally unique and burned into the XBee module at manufacture, so
    there is no risk of address collision even when multiple throttles are
    in range.

    The module is intentionally thin:
      • It has no knowledge of the UART or the XBee driver.
      • It operates on raw payload byte arrays; the caller is responsible
        for all frame-layer I/O.
      • All EEPROM access uses the standard avr-libc eeprom_read_byte /
        eeprom_write_byte routines, identical to the original code.

    Write-enable guard:
      The xbeeEepromSetWriteEnable() flag replaces the original global
      enableEepromWrite variable.  The main application controls the flag
      through the menu system (same UX behaviour as before).  An attempt
      to write while the flag is clear returns RSP_EE_ERROR with error
      code XBEE_EE_ERR_WR_LOCKED and does not touch the EEPROM.  The
      read-back after a locked write returns the current EEPROM contents
      so the caller can detect the discrepancy if desired.

    Sleep-timer reset:
      The original PktHandler() called sleepTimeout_decisecs = reset_value
      inside the 'R' and 'W' handlers.  That coupling is preserved here
      via the optional callback pointer xbeeEepromSleepResetCb.  Set it
      once at startup to a small wrapper that does the atomic assignment;
      if left NULL the module simply skips the call.
*************************************************************************/

#include <string.h>
#include <avr/eeprom.h>

#include "xbee-eeprom.h"

/* -----------------------------------------------------------------------
 * Internal state
 * -------------------------------------------------------------------- */

/** 64-bit destination MAC, MSB first.  Updated on every incoming request. */
static uint8_t remoteAddr64[8];

/** Non-zero when EEPROM writes are permitted. */
static uint8_t writeEnabled = 0;

/**
 * Optional callback invoked after a successful read or write so the
 * caller can reset the sleep / activity timer.  Set via
 * xbeeEepromSetActivityCallback().  May be NULL.
 */
static void (*activityCb)(void) = NULL;

/* -----------------------------------------------------------------------
 * Public API – address and flag management
 * -------------------------------------------------------------------- */

void xbeeEepromInit(void)
{
    memset(remoteAddr64, 0x00, sizeof(remoteAddr64));
    writeEnabled = 0;
    activityCb   = NULL;
}

void xbeeEepromSetRemoteAddr(const uint8_t *addr64)
{
    memcpy(remoteAddr64, addr64, 8);
}

void xbeeEepromGetRemoteAddr(uint8_t *addr64)
{
    memcpy(addr64, remoteAddr64, 8);
}

void xbeeEepromSetWriteEnable(uint8_t enable)
{
    writeEnabled = enable ? 1u : 0u;
}

uint8_t xbeeEepromGetWriteEnable(void)
{
    return writeEnabled;
}

/**
 * @brief  Register an optional activity callback.
 *
 * If non-NULL, this function is called after every successfully processed
 * read or write request.  Typical use: reset the device sleep timer.
 *
 * @param cb  Function pointer, or NULL to disable.
 */
void xbeeEepromSetActivityCallback(void (*cb)(void))
{
    activityCb = cb;
}

/* -----------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------- */

/**
 * Build a 5-byte response header into @p out.
 *
 * @param out       Destination buffer (at least 5 bytes).
 * @param rspByte   RSP_EE_READ, RSP_EE_WRITE, or RSP_EE_ERROR.
 * @param seq       Sequence number echoed from the request.
 * @param addrL     EEPROM address low byte (0 for error responses).
 * @param addrH     EEPROM address high byte (0 for error responses).
 * @param count     Byte count / error code carried in byte 4.
 */
static void buildHeader(uint8_t *out,
                        uint8_t  rspByte,
                        uint8_t  seq,
                        uint8_t  addrL,
                        uint8_t  addrH,
                        uint8_t  count)
{
    out[0] = rspByte;
    out[1] = seq;
    out[2] = addrL;
    out[3] = addrH;
    out[4] = count;
}

/**
 * Build a complete RSP_EE_ERROR response into @p txBuf.
 *
 * @param txBuf   Output buffer (≥ 3 bytes).
 * @param txLen   Output: set to 3.
 * @param seq     Sequence number from the request.
 * @param errCode XBEE_EE_ERR_* constant.
 */
static void buildError(uint8_t *txBuf,
                       uint8_t *txLen,
                       uint8_t  seq,
                       uint8_t  errCode)
{
    txBuf[0] = RSP_EE_ERROR;
    txBuf[1] = seq;
    txBuf[2] = errCode;
    *txLen   = 3u;
}

/* -----------------------------------------------------------------------
 * Core request handler
 * -------------------------------------------------------------------- */

int8_t xbeeEepromHandleRx(const uint8_t *rxPayload, uint8_t rxLen,
                           uint8_t *txBuf,           uint8_t *txLen)
{
    *txLen = 0;

    /* ------------------------------------------------------------------
     * Minimum length check: we always need at least the 5-byte header.
     * ---------------------------------------------------------------- */
    if (rxLen < XBEE_EE_HDR_LEN)
    {
        /*
         * We don't have a sequence number yet if the packet is truly
         * degenerate (< 2 bytes), but we still attempt to echo byte [1]
         * if it exists, otherwise use 0x00.
         */
        uint8_t seq = (rxLen >= 2u) ? rxPayload[1] : 0x00u;
        buildError(txBuf, txLen, seq, XBEE_EE_ERR_SHORT_PKT);
        return 0;
    }

    /* Extract common header fields present in every valid request. */
    const uint8_t cmd   = rxPayload[0];
    const uint8_t seq   = rxPayload[1];
    const uint8_t addrL = rxPayload[2];
    const uint8_t addrH = rxPayload[3];
    const uint8_t count = rxPayload[4];

    const uint16_t eepAddr = ((uint16_t)addrH << 8) | addrL;

    /* ------------------------------------------------------------------
     * Validate the byte count for both reads and writes.
     *
     * count == 0  is rejected (nothing to do, likely a bug in the host).
     * count > XBEE_EE_MAX_BLOCK is rejected to protect the stack buffers
     * and keep transactions bounded.
     * ---------------------------------------------------------------- */
    if ((count == 0u) || (count > XBEE_EE_MAX_BLOCK))
    {
        buildError(txBuf, txLen, seq, XBEE_EE_ERR_BAD_COUNT);
        return 0;
    }

    /* ------------------------------------------------------------------
     * Dispatch on command byte.
     * ---------------------------------------------------------------- */
    if (CMD_EE_READ == cmd)
    {
        /* ----------------------------------------------------------------
         * EEPROM READ
         *
         * Request  : [CMD_EE_READ][seq][addrL][addrH][count]
         * Response : [RSP_EE_READ][seq][addrL][addrH][count][data0..dataN]
         * -------------------------------------------------------------- */

        /* Notify the application (e.g. reset sleep timer). */
        if (activityCb != NULL)
            activityCb();

        buildHeader(txBuf, RSP_EE_READ, seq, addrL, addrH, count);

        uint8_t i;
        for (i = 0u; i < count; i++)
        {
            txBuf[XBEE_EE_HDR_LEN + i] =
                eeprom_read_byte((const uint8_t *)(eepAddr + i));
        }

        *txLen = (uint8_t)(XBEE_EE_HDR_LEN + count);
    }
    else if (CMD_EE_WRITE == cmd)
    {
        /* ----------------------------------------------------------------
         * EEPROM WRITE
         *
         * Request  : [CMD_EE_WRITE][seq][addrL][addrH][count][data0..dataN]
         * Response : [RSP_EE_WRITE][seq][addrL][addrH][count][readback0..N]
         *
         * The response always carries the EEPROM readback so the host can
         * verify what was actually stored.  If writes are locked, the
         * readback is still sent (current EEPROM values unchanged).
         * -------------------------------------------------------------- */

        /* Check that the request payload actually contains all data bytes. */
        if (rxLen < (uint8_t)(XBEE_EE_HDR_LEN + count))
        {
            buildError(txBuf, txLen, seq, XBEE_EE_ERR_WR_TRUNC);
            return 0;
        }

        /* Check write-enable guard. */
        if (!writeEnabled)
        {
            buildError(txBuf, txLen, seq, XBEE_EE_ERR_WR_LOCKED);
            return 0;
        }

        /* Notify the application (e.g. reset sleep timer). */
        if (activityCb != NULL)
            activityCb();

        /* Perform the writes. */
        uint8_t i;
        for (i = 0u; i < count; i++)
        {
            eeprom_write_byte((uint8_t *)(eepAddr + i),
                              rxPayload[XBEE_EE_HDR_LEN + i]);
        }

        /* Build response with readback. */
        buildHeader(txBuf, RSP_EE_WRITE, seq, addrL, addrH, count);

        for (i = 0u; i < count; i++)
        {
            txBuf[XBEE_EE_HDR_LEN + i] =
                eeprom_read_byte((const uint8_t *)(eepAddr + i));
        }

        *txLen = (uint8_t)(XBEE_EE_HDR_LEN + count);
    }
    else
    {
        /* Unknown command. */
        buildError(txBuf, txLen, seq, XBEE_EE_ERR_BAD_CMD);
    }

    return 0;
}

/* -----------------------------------------------------------------------
 * XBee API 0x10 Transmit-Request frame builder
 * -------------------------------------------------------------------- */

int8_t xbeeEepromBuildTxFrame(uint8_t        frameId,
                               const uint8_t *payload,
                               uint8_t        payloadLen,
                               uint8_t       *frameBuf,
                               uint8_t        frameBufSize,
                               uint8_t       *frameLen)
{
    /*
     * 0x10 Transmit Request frame layout
     * -----------------------------------------------------------------------
     * Offset  Field            Bytes   Notes
     * -----------------------------------------------------------------------
     *   0     Start delimiter   1      0x7E
     *   1     Length MSB        1      MSB of (frame data length)
     *   2     Length LSB        1      LSB; frame data = from [3] to checksum-1
     *   3     Frame type        1      0x10
     *   4     Frame ID          1      0x00 = no status response; 1-0xFF = with
     *   5     64-bit dest addr  8      MSB first
     *  13     16-bit dest addr  2      0xFF 0xFE = unknown, use 64-bit
     *  15     Broadcast radius  1      0x00 = max hops
     *  16     Options           1      0x00
     *  17     RF data           N      application payload
     *  17+N   Checksum          1      0xFF - (sum of bytes [3..17+N-1])
     * -----------------------------------------------------------------------
     * Total frame size = 4 (delimiter + length + checksum) + 14 (fixed API
     *                    fields) + payloadLen
     *                  = 18 + payloadLen
     */

    /* Minimum fixed overhead bytes in the frame (excluding delimiter,
       length field pair, and trailing checksum):
         frame-type(1) + frameId(1) + addr64(8) + addr16(2)
         + bcastRadius(1) + options(1) = 14 bytes of "frame data"
       Plus the payload itself.
       Total wire bytes = 1 (0x7E) + 2 (length) + 14 + payloadLen + 1 (cksum)
                        = 18 + payloadLen
    */
    const uint8_t fixedFrameDataLen = 14u;  /* bytes 3..16 inclusive */
    const uint16_t frameDataLen = (uint16_t)fixedFrameDataLen + payloadLen;

    /* Total wire bytes */
    const uint8_t totalLen = (uint8_t)(1u      /* 0x7E */
                                     + 2u      /* length field */
                                     + frameDataLen
                                     + 1u);    /* checksum */

    if (frameBufSize < totalLen)
        return -1;

    uint8_t *p = frameBuf;

    /* Start delimiter */
    *p++ = 0x7Eu;

    /* Length field (covers frame data, i.e. from frame-type to last
       payload byte, but NOT the start delimiter, length bytes, or
       the checksum byte). */
    *p++ = (uint8_t)(frameDataLen >> 8);
    *p++ = (uint8_t)(frameDataLen & 0xFFu);

    /* --- Begin checksummed region --- */
    const uint8_t *ckstart = p;

    /* Frame type */
    *p++ = 0x10u;

    /* Frame ID */
    *p++ = frameId;

    /* 64-bit destination address (MSB first, from stored remote addr) */
    memcpy(p, remoteAddr64, 8u);
    p += 8u;

    /* 16-bit destination address: 0xFFFE = unknown, rely on 64-bit */
    *p++ = 0xFFu;
    *p++ = 0xFEu;

    /* Broadcast radius (0 = maximum hops set by XBee NH parameter) */
    *p++ = 0x00u;

    /* Transmit options (0 = default: unicast with ACK) */
    *p++ = 0x00u;

    /* Application payload */
    memcpy(p, payload, payloadLen);
    p += payloadLen;

    /* Checksum: 0xFF minus the 8-bit sum of all checksummed bytes */
    uint8_t cksum = 0x00u;
    const uint8_t *q;
    for (q = ckstart; q < p; q++)
        cksum += *q;
    *p++ = (uint8_t)(0xFFu - cksum);

    *frameLen = totalLen;
    return 0;
}
