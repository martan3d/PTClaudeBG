/*************************************************************************
  INTEGRATION GUIDE – replacing MRBus 'R'/'W' with XBee API directed
  EEPROM access in mrbw-cst.c
  -----------------------------------------------------------------------
  This file is NOT compiled directly.  It shows, in unified-diff style,
  every change needed in mrbw-cst.c plus the reasoning behind each one.
  Apply these changes by hand (or with `patch -p0 < this-file`).

  Files added to the project:
    xbee-eeprom.h  – public API header (new file)
    xbee-eeprom.c  – implementation     (new file)

  Files changed:
    mrbw-cst.c     – see sections below
*************************************************************************/


/* =====================================================================
   SECTION 1 – Add the new header to the include block
   ===================================================================== */

/* In mrbw-cst.c, the existing include block ends around line 48:
      #include "cst-math.h"

   Add immediately after that line:
*/

/* +++ */ #include "xbee-eeprom.h"


/* =====================================================================
   SECTION 2 – Remove the old enableEepromWrite global
   ===================================================================== */

/* Original (around line 154):
      uint8_t enableEepromWrite = 0;

   DELETE that line entirely.  The write-enable state is now managed
   inside xbee-eeprom.c via xbeeEepromSetWriteEnable() /
   xbeeEepromGetWriteEnable().

   Every place in the original code that reads or writes enableEepromWrite
   must be updated as shown in SECTION 5 below.
*/


/* =====================================================================
   SECTION 3 – Initialise the module in init()
   ===================================================================== */

/* Original init() ends at about line 1007.  Add before the closing
   brace:

      xbeeEepromInit();
      xbeeEepromSetActivityCallback(xbeeEepromActivityReset);

   The helper function xbeeEepromActivityReset() is defined in SECTION 4.
*/


/* =====================================================================
   SECTION 4 – Add the activity-reset callback (file scope, before init)
   ===================================================================== */

/*
   Insert this function somewhere above init(), e.g. near the other small
   helper functions around line 270.
*/

/* +++ */
/*
static void xbeeEepromActivityReset(void)
{
    // Called by xbee-eeprom.c after a successful read or write.
    // Resets the sleep timer exactly as the old 'R'/'W' handlers did.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        sleepTimeout_decisecs = sleep_tmr_reset_value;
    }
}
*/


/* =====================================================================
   SECTION 5 – Replace the 'R' and 'W' cases inside PktHandler()
   ===================================================================== */

/*
   Original PktHandler() (lines 357-533).  The relevant section handles
   'W' (EEPROM write) at lines ~416-453 and 'R' (EEPROM read) at lines
   ~454-482.

   REMOVE both else-if blocks shown between the dashed lines.

   --- REMOVE BEGIN ---

    else if ('W' == rxBuffer[MRBUS_PKT_TYPE])
    {
        // ... old broadcast-style write code ~30 lines ...
        goto PktIgnore;
    }
    else if ('R' == rxBuffer[MRBUS_PKT_TYPE])
    {
        // ... old broadcast-style read code ~25 lines ...
        goto PktIgnore;
    }

   --- REMOVE END ---

   REPLACE with the single block below.  The new block:
     • Uses a sub-discriminator ('E'/'W') in the APPLICATION payload
       rather than the MRBus packet-type byte.
     • Stores the 64-bit source MAC from the incoming 0x90 frame so the
       response can be directed back unicast.
     • Delegates all EEPROM logic to xbeeEepromHandleRx().
     • Calls xbeeEepromBuildTxFrame() to produce a ready-to-send wire
       frame and enqueues it.

   NOTE on rxBuffer layout from a decoded 0x90 frame:
     The mrbee driver may present the data differently depending on how
     mrbee.h defines its Receive API.  The mapping below assumes that
     after mrbeePktQueuePop() the rxBuffer contains the full decoded
     0x90 payload starting at rxBuffer[0]:

       rxBuffer[0..7]  = 64-bit source MAC (MSB first)
       rxBuffer[8..9]  = 16-bit source network address
       rxBuffer[10]    = receive options
       rxBuffer[11..]  = application data bytes

     Adjust the offsets if your mrbee driver uses a different layout.
*/

/* +++ */
/*
    else if ( ('E' == rxBuffer[11]) || ('W' == rxBuffer[11]) )
    {
        // XBee API directed EEPROM read or write.
        //
        // rxBuffer[0..7]  – 64-bit source MAC of the requesting host
        // rxBuffer[11..]  – application payload (CMD byte is [11])

        // Length of the application payload embedded in the 0x90 frame.
        // rxBuffer[MRBUS_PKT_LEN] holds the total decoded data length;
        // subtract the 11-byte prefix (MAC + net-addr + options) to get
        // just the application payload length.
        uint8_t appPayloadLen = (uint8_t)(rxBuffer[MRBUS_PKT_LEN] - 11u);

        // Store the caller's 64-bit MAC so the response goes back to them.
        xbeeEepromSetRemoteAddr(&rxBuffer[0]);

        // Response application payload (built by the handler).
        uint8_t  rspPayload[XBEE_EE_MAX_RSP_BUF];
        uint8_t  rspPayloadLen = 0;

        xbeeEepromHandleRx(&rxBuffer[11],   // application payload start
                           appPayloadLen,
                           rspPayload,
                           &rspPayloadLen);

        if (rspPayloadLen > 0u)
        {
            // Wrap the response payload in a 0x10 Transmit Request frame.
            // Maximum wire frame = 18 + XBEE_EE_MAX_RSP_BUF bytes.
            uint8_t  wireBuf[18u + XBEE_EE_MAX_RSP_BUF];
            uint8_t  wireLen = 0;

            if (0 == xbeeEepromBuildTxFrame(
                        0x01u,          // frame ID: request TX status
                        rspPayload,
                        rspPayloadLen,
                        wireBuf,
                        sizeof(wireBuf),
                        &wireLen))
            {
                // Hand the raw frame bytes to the XBee UART driver.
                // Replace mrbeeTransmitRaw() with whatever function your
                // driver exposes for writing pre-built API frames.
                mrbeeTransmitRaw(wireBuf, wireLen);
            }
        }

        goto PktIgnore;
    }
*/


/* =====================================================================
   SECTION 6 – Update all references to enableEepromWrite
   ===================================================================== */

/*
   Search the rest of mrbw-cst.c (and any other file that uses it) for
   'enableEepromWrite' and apply these substitutions:

   a) Reading the flag (e.g. in menu screens that display its state):
          OLD: if (enableEepromWrite)
          NEW: if (xbeeEepromGetWriteEnable())

   b) Setting the flag (e.g. menu SELECT action that arms writes):
          OLD: enableEepromWrite = 1;
          NEW: xbeeEepromSetWriteEnable(1);

   c) Clearing the flag (e.g. menu exit / timeout):
          OLD: enableEepromWrite = 0;
          NEW: xbeeEepromSetWriteEnable(0);

   The semantics are identical: non-zero means writes are allowed,
   zero means writes are blocked.
*/


/* =====================================================================
   SECTION 7 – Build system
   ===================================================================== */

/*
   Add xbee-eeprom.c to the list of source files compiled into the
   firmware.  In a typical Makefile this is:

       SOURCES += xbee-eeprom.c

   No new linker flags or libraries are required; the module depends
   only on <string.h> and <avr/eeprom.h>, both already present.
*/
