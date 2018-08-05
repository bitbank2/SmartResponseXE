//
// SMART Response XE support library
// written by Larry Bank
// Copyright (C) 2018 BitBank Software, Inc.
// Project started 8/4/2018
//
#ifndef __SMART_RESPONSE_XE__
#define __SMART_RESPONSE_XE__

// Font sizes (9x8, 6x8, 15x16)
#define FONT_NORMAL 0
#define FONT_SMALL 1
#define FONT_LARGE 2

// Keyboard info
#define ROWS 6
#define COLS 10

// Display info
#define LCD_WIDTH 384
#define LCD_HEIGHT 136

//
// Power on the LCD
//
void SRXEPowerUp(void);

//
// Power off the LCD
//
void SRXEPowerDown(void);
//
// Initializes the LCD controller
// Parameters: GPIO pin numbers used for the CS/DC/RST control lines
//
int SRXEInit(int iCS, int iDC, int iReset);
//
// Send commands to position the "cursor" to the given
// row and column and width and height of the memory window
//
void SRXESetPosition(int x, int y, int cx, int cy);
//
// Write a block of pixel data to the LCD
// Length can be anything from 1 to 17408 (whole display)
//
void SRXEWriteDataBlock(unsigned char *ucBuf, int iLen);
//
// Scroll the screen N lines vertically (positive or negative)
// The value given represents a delta which affects the current scroll offset
//
void SRXEScroll(int iLines);
//
// Reset the scroll position to 0
//
void SRXEScrollReset(void);
//
// Draw an outline or filled rectangle
// Only draws on byte boundaries (3 pixels wide)
// (display is treated as 128x136)
//
void SRXERectangle(int x, int y, int cx, int cy, byte color, byte bFilled);
//
// Draw a string of normal (9x8), small (6x8) or large (15x16) characters
// At the given col+row
//
int SRXEWriteString(int x, int y, char *szMsg, int iSize, int iFGColor, int iBGColor);
// Fill the frame buffer with a byte pattern
// e.g. all off (0x00) or all on (0xff)
void SRXEFill(byte ucData);
//
// Scan the rows and columns and store the results in the key map
//
void SRXEScanKeyboard(void);
//
// Return a pointer to the internal key map
// (10 bytes with 6 bits each)
//
byte *SRXEGetKeyMap(void);
//
// Return the current key pressed
// includes code to provide shift + sym adjusted output
// internally calls SRXEScanKeyboard()
//
byte SRXEGetKey(void);

#endif // __SMART_RESPONSE_XE__

