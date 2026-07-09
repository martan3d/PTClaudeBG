/*************************************************************************
Title:    XBee API-Mode Directed EEPROM Access for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     xbee-eeprom.h
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

OVERVIEW:
    This module replaces the MRBus broadcast-style EEPROM read/write
    commands ('R'/'W' packet types) with XBee API-mode directed unicast
    messages.  Addressing is by the 64-bit IEEE MAC address of the XBee
    module so that packets reach only the intended device.

PROTOCOL - APPLICATION PAYLOAD LAYOUT:
    All payloads carried inside a XBee API 0x10 (Transmit Request) or
    0x90 (Receive Packet) frame.  The first byte of the application
    payload is always a command/response discriminator.

    EEPROM READ REQUEST  (host → throttle)
        Byte 0  : CMD_EE_READ  (0x45 = 'E')
        Byte 1  : seq          – caller-chosen sequence number echoed back
        Byte 2  : addrL        – EEPROM start address low byte
        Byte 3  : addrH        – EEPROM start address high byte
        Byte 4  : count        – number of bytes to read (1–100)

    EEPROM READ RESPONSE (throttle → host)
        Byte 0  : RSP_EE_READ  (0x65 = 'e')
        Byte 1  : seq          – echoed sequence number
        Byte 2  : addrL        – echoed start address low byte
        Byte 3  : addrH        – echoed start address high byte
        Byte 4  : count        – number of bytes that follow
        Byte 5+ : data[count]  – EEPROM bytes read

    EEPROM WRITE REQUEST (host → throttle)
        Byte 0  : CMD_EE_WRITE (0x57 = 'W')  [upper-case = write]
        Byte 1  : seq          – caller-chosen sequence number echoed back
        Byte 2  : addrL        – EEPROM start address low byte
        Byte 3  : addrH        – EEPROM start address high byte
        Byte 4  : count        – number of data bytes that follow (1–100)
        Byte 5+ : data[count]  – bytes to write

    EEPROM WRITE RESPONSE (throttle → host)
        Byte 0  : RSP_EE_WRITE (0x77 = 'w')  [lower-case = response]
        Byte 1  : seq          – echoed sequence number
        Byte 2  : addrL        – echoed start address low byte
        Byte 3  : addrH        – echoed start address high byte
        Byte 4  : count        – number of bytes actually written
        Byte 5+ : data[count]  – readback of the EEPROM after the write

    ERROR RESPONSE (throttle → host)
        Byte 0  : RSP_EE_ERROR (0x21 = '!')
        Byte 1  : seq          – echoed sequence number
        Byte 2  : error code   – XBEE_EE_ERR_* constant

XBEE API FRAME WRAPPING:
    The functions in this module receive raw application payload bytes
    (extracted from a received 0x90 frame) and produce raw application
    payload bytes that the caller wraps in a 0x10 Transmit Request frame
    directed to the 64-bit MAC stored in xbeeEepromRemoteAddr64[].

    The caller is responsible for:
      • Parsing the incoming 0x90 Receive frame and extracting the
        64-bit source address and the payload bytes.
      • Calling xbeeEepromSetRemoteAddr() with that 64-bit address so
        the module knows where to send the response.
      • Building the outgoing 0x10 frame from the response payload
        provided by xbeeEepromHandleRx() and putting it on the wire.

MAX BLOCK SIZE:
    XBEE_EE_MAX_BLOCK (100) bytes per transaction.  This keeps the
    maximum application payload at 105 bytes (5-byte header + 100 data),
    well inside the XBee Series 2 / 3 maximum payload of 255 bytes.
*************************************************************************/

#ifndef _XBEE_EEPROM_H_
#define _XBEE_EEPROM_H_

#include <stdint.h>

/* -----------------------------------------------------------------------
 * Compile-time limits
 * -------------------------------------------------------------------- */

/** Maximum number of EEPROM bytes in a single read or write request. */
#define XBEE_EE_MAX_BLOCK       100U

/** Minimum payload bytes needed to parse any request header. */
#define XBEE_EE_HDR_LEN         5U

/** Maximum response buffer size: header (5) + max data (100) + margin. */
#define XBEE_EE_MAX_RSP_BUF     (XBEE_EE_HDR_LEN + XBEE_EE_MAX_BLOCK)

/* -----------------------------------------------------------------------
 * Application-layer command / response bytes  (payload byte 0)
 * -------------------------------------------------------------------- */

/** Host → throttle: read N bytes of EEPROM. */
#define CMD_EE_READ             0x45U   /* 'E' */

/** Throttle → host: read response carrying the data. */
#define RSP_EE_READ             0x65U   /* 'e' */

/** Host → throttle: write N bytes of EEPROM. */
#define CMD_EE_WRITE            0x57U   /* 'W' */

/** Throttle → host: write response carrying readback data. */
#define RSP_EE_WRITE            0x77U   /* 'w' */

/** Throttle → host: error response. */
#define RSP_EE_ERROR            0x21U   /* '!' */

/* -----------------------------------------------------------------------
 * Error codes  (payload byte 2 of an RSP_EE_ERROR response)
 * -------------------------------------------------------------------- */

/** Request payload was too short to be a valid header. */
#define XBEE_EE_ERR_SHORT_PKT   0x01U

/** Unknown command byte in payload byte 0. */
#define XBEE_EE_ERR_BAD_CMD     0x02U

/** Requested byte count was zero or exceeded XBEE_EE_MAX_BLOCK. */
#define XBEE_EE_ERR_BAD_COUNT   0x03U

/** Write was attempted but the write-enable flag is not set. */
#define XBEE_EE_ERR_WR_LOCKED   0x04U

/** Write payload was shorter than the declared byte count. */
#define XBEE_EE_ERR_WR_TRUNC    0x05U

/* -----------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------- */

/**
 * @brief  Initialise the module.
 *
 * Clears the stored remote MAC address and resets the write-enable flag.
 * Must be called once during system start-up, before any other function
 * in this module.
 */
void xbeeEepromInit(void);

/**
 * @brief  Store the 64-bit MAC address of the remote host.
 *
 * The address is used when building the outgoing 0x10 Transmit Request
 * frames.  Call this every time a new READ or WRITE request arrives,
 * passing in the source MAC extracted from the incoming 0x90 frame.
 *
 * @param addr64  Pointer to 8 bytes, MSB first (same byte order as the
 *                64-bit source address field of the XBee 0x90 frame).
 */
void xbeeEepromSetRemoteAddr(const uint8_t *addr64);

/**
 * @brief  Retrieve the stored 64-bit remote MAC address.
 *
 * @param addr64  Output buffer of at least 8 bytes; receives the address
 *                MSB first.
 */
void xbeeEepromGetRemoteAddr(uint8_t *addr64);

/**
 * @brief  Enable or disable EEPROM write operations.
 *
 * Writes are blocked by default.  The caller (main application) controls
 * this flag, typically exposing it through the menu system exactly as the
 * original enableEepromWrite variable was used.
 *
 * @param enable  Non-zero to allow writes; zero to block them.
 */
void xbeeEepromSetWriteEnable(uint8_t enable);

/**
 * @brief  Query whether EEPROM writes are currently permitted.
 *
 * @return Non-zero if writes are allowed, zero if locked.
 */
uint8_t xbeeEepromGetWriteEnable(void);

/**
 * @brief  Process one incoming application payload from a 0x90 frame.
 *
 * Parses @p rxPayload, performs the requested EEPROM read or write, and
 * writes the response application payload into @p txBuf.
 *
 * The caller must:
 *   1. Have already called xbeeEepromSetRemoteAddr() with the 64-bit
 *      source MAC from the same 0x90 frame.
 *   2. Wrap the bytes written into @p txBuf inside a 0x10 Transmit
 *      Request frame addressed to that same 64-bit MAC and put it on
 *      the XBee serial port.
 *
 * @param rxPayload   Pointer to the application payload bytes (starts
 *                    at the first byte after the XBee 0x90 frame header,
 *                    i.e. immediately after the options byte).
 * @param rxLen       Number of bytes in @p rxPayload.
 * @param txBuf       Caller-supplied buffer that receives the response
 *                    application payload.  Must be at least
 *                    XBEE_EE_MAX_RSP_BUF bytes.
 * @param txLen       Output: set to the number of valid bytes placed in
 *                    @p txBuf.  Zero means no response should be sent.
 *
 * @return  0 on success (response in txBuf/txLen), negative on internal
 *          error (no response sent in that case).
 */
int8_t xbeeEepromHandleRx(const uint8_t *rxPayload, uint8_t rxLen,
                           uint8_t *txBuf,           uint8_t *txLen);

/**
 * @brief  Build a complete XBee API 0x10 Transmit Request frame.
 *
 * Wraps @p payload inside the standard XBee API framing so the result
 * can be written directly to the UART connected to the XBee module.
 *
 * Frame structure (all lengths in bytes):
 *   [0x7E][lenH][lenL][0x10][frameId][addr64 x8][addr16 x2][bcastR][opts]
 *   [payload x payloadLen][checksum]
 *
 * The destination 64-bit address is taken from the value most recently
 * stored via xbeeEepromSetRemoteAddr().  The 16-bit address field is set
 * to 0xFFFE (unknown/use 64-bit), broadcast radius 0, options 0.
 *
 * @param frameId       XBee frame-ID byte (use 0x00 to suppress ACK, or
 *                      1–255 for a delivery-status response).
 * @param payload       Application payload bytes to embed.
 * @param payloadLen    Number of bytes in @p payload (≤ XBEE_EE_MAX_BLOCK
 *                      + XBEE_EE_HDR_LEN).
 * @param frameBuf      Output buffer.  Must be at least
 *                      (14 + payloadLen) bytes.
 * @param frameBufSize  Size of @p frameBuf in bytes.
 * @param frameLen      Output: total number of bytes written to @p frameBuf.
 *
 * @return  0 on success, -1 if @p frameBuf is too small.
 */
int8_t xbeeEepromBuildTxFrame(uint8_t        frameId,
                               const uint8_t *payload,
                               uint8_t        payloadLen,
                               uint8_t       *frameBuf,
                               uint8_t        frameBufSize,
                               uint8_t       *frameLen);

#endif /* _XBEE_EEPROM_H_ */
