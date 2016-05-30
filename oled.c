#include <signal.h>
#include <string.h>
#include <stdarg.h>  
#include "oled.h"
#include <stdlib.h>
#include <stdio.h>
//#include <wiringPiI2C.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>



/*
 #include "oled.h"
 #include <stdio.h>
 #include <stdint.h>
 #include <stdlib.h>
 #include <fcntl.h>
 #include <unistd.h>
 #include <string.h>
 #include <errno.h>
 #include <sys/ioctl.h>
 #include <sys/types.h>
 #include <sys/stat.h>
 #include <linux/i2c-dev.h>
 #include <wiringPiI2C.h>
 #include <stdarg.h>
 #include <signal.h>
 */

char oled_writeBytes(unsigned char oled_devAddr, unsigned int length,
		unsigned char * data) {
	char count = 0;
	unsigned char buf[128];

	if (length > 127) {
		printf("%s,%d: Byte write count (%d) > 127\n", __func__, __LINE__,
				length);
		return (false);
	}

	memcpy(buf, data, length);
	count = write(oled_fd, buf, length);
	if (count < 0) {
		printf("%s,%d: Failed to write device(%d)\n", __func__, __LINE__,
				count);
		return (false);
	} else if (count != length) {
		printf("%s,%d: Short write to device, expected %d, got %d\n", __func__,
				__LINE__, length, count);
		return (false);
	}

	return true;
}

void fastI2Cwrite(unsigned char* tbuf, unsigned int len) {
	if (oled_initSuccess)
		oled_writeBytes(oled_devAddr, len, tbuf);
//  write(oled_fd,tbuf,len);
}

void ssd1306_command1(unsigned char c) {
	char buff[2];
	// Clear D/C to switch to command mode
	buff[0] = SSD_Command_Mode;
	buff[1] = c;
	// Write Data on I2C
	fastI2Cwrite(buff, sizeof(buff));
}

void ssd1306_command2(unsigned char c0, unsigned char c1) {
	char buff[3];
	buff[1] = c0;
	buff[2] = c1;

	// Clear D/C to switch to command mode
	buff[0] = SSD_Command_Mode;

	// Write Data on I2C
	fastI2Cwrite(buff, 3);
}

void ssd1306_command3(unsigned char c0, unsigned char c1, unsigned char c2) {
	char buff[4];
	buff[1] = c0;
	buff[2] = c1;
	buff[3] = c2;

	// Clear D/C to switch to command mode
	buff[0] = SSD_Command_Mode;

	// Write Data on I2C
	fastI2Cwrite(buff, sizeof(buff));
}

void setOledPage(short pageIndex) {
	oledPage = pageIndex;
}

short getOledPage() {
	return oledPage;
}

void oledInit(void) {
	unsigned char multiplex;
	unsigned char chargepump;
	unsigned char compins;
	unsigned char contrast;
	unsigned char precharge;
	oled_devAddr = OLED_DEFAULT_ADDRESS;

	constructor(ssd1306_lcdwidth, ssd1306_lcdheight);

	// depends on OLED type configuration
	multiplex = 0x3F;
	compins = 0x12;
	contrast = 0x9F;
	chargepump = 0x10;
	precharge = 0x22;
	chargepump = 0x14;
	precharge = 0xF1;
	oled_initSuccess = 1;
	oledPage = 1;

	oled_fd = wiringPiI2CSetup(oled_devAddr);
	short count1 = write(oled_fd, "t", 1);

	if (count1 < 0) {
		oled_initSuccess = 0;
//		return;
	}
	ssd1306_command1(SSD_Display_Off);                    // 0xAE
	ssd1306_command2(SSD1306_SETDISPLAYCLOCKDIV, 0x80); // 0xD5 + the suggested ratio 0x80
	ssd1306_command2(SSD1306_SETMULTIPLEX, multiplex);
	ssd1306_command2(SSD1306_SETDISPLAYOFFSET, 0x00);        // 0xD3 + no offset
	ssd1306_command1(SSD1306_SETSTARTLINE | 0x0);            // line #0
	ssd1306_command2(SSD1306_CHARGEPUMP, chargepump);
	ssd1306_command2(SSD1306_MEMORYMODE, 0x00);      // 0x20 0x0 act like ks0108
	ssd1306_command1(SSD1306_SEGREMAP | 0x1);
	ssd1306_command1(SSD1306_COMSCANDEC);
	ssd1306_command2(SSD1306_SETCOMPINS, compins);  // 0xDA
	ssd1306_command2(SSD_Set_ContrastLevel, contrast);
	ssd1306_command2(SSD1306_SETPRECHARGE, precharge); // 0xd9
	ssd1306_command2(SSD1306_SETVCOMDETECT, 0x40);  // 0xDB
	ssd1306_command1(SSD1306_DISPLAYALLON_RESUME);    // 0xA4
	ssd1306_command1(SSD1306_Normal_Display);         // 0xA6

	ssd1306_command3(0x21, 0, 127);
	ssd1306_command3(0x22, 0, 7);
	stopscroll();

	if (poledbuff)
		free(poledbuff);

	// Allocate memory for OLED buffer
	poledbuff = (unsigned char *) malloc(
			(ssd1306_lcdwidth * ssd1306_lcdheight / 8));

	// Empty uninitialized buffer
	clearDisplay();
	ssd1306_command1(SSD_Display_On);                     //--turn on oled panel

}

void stopscroll(void) {
	ssd1306_command1(SSD_Deactivate_Scroll);
}

void display(void) {
	ssd1306_command1(SSD1306_SETLOWCOLUMN | 0x0); // low col = 0
	ssd1306_command1(SSD1306_SETHIGHCOLUMN | 0x0); // hi col = 0
	ssd1306_command1(SSD1306_SETSTARTLINE | 0x0); // line #0

	unsigned short i = 0;

	// pointer to OLED data buffer
	unsigned char * p = poledbuff;

	// SPI
	char buff[17];
	unsigned char x;

	// Setup D/C to switch to data mode
	buff[0] = SSD_Data_Mode;

	// loop trough all OLED buffer and
	// send a bunch of 16 data byte in one xmission
	for (i = 0; i < (ssd1306_lcdwidth * ssd1306_lcdheight / 8); i += 16) {
		for (x = 1; x <= 16; x++)
			buff[x] = *p++;

		fastI2Cwrite(buff, 17);
	}
}

unsigned char getRotation(void) {
	rotation %= 4;
	return rotation;
}

void setRotation(unsigned char x) {
	x %= 4;  // cant be higher than 3
	rotation = x;
	switch (x) {
	case 0:
	case 2:
		_width = WIDTH;
		_height = HEIGHT;
		break;

	case 1:
	case 3:
		_width = HEIGHT;
		_height = WIDTH;
		break;
	}
}

void drawPixel(short x0, short y0, unsigned short color) {
	short x = x0;
	short y = y0;

	unsigned char * p = poledbuff;
	int j = 1;
	if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
		return;

	// check rotation, move pixel around if necessary
	switch (getRotation()) {
	case 1:
		swap(&x, &y);
		x = WIDTH - x - 1;
		break;

	case 2:
		x = WIDTH - x - 1;
		y = HEIGHT - y - 1;
		break;

	case 3:
		swap(&x, &y);
		y = HEIGHT - y - 1;
		break;

	}

	// Get where to do the change in the buffer
	p = poledbuff + (x + (y / 8) * ssd1306_lcdwidth);

	// x is which column
	if (color == WHITE)
		*p |= _BV((y % 8));
	else
		*p &= ~_BV((y % 8));
}

void drawCircle(short x0, short y0, short r, unsigned short color) {
	short f = 1 - r;
	short ddF_x = 1;
	short ddF_y = -2 * r;
	short x = 0;
	short y = r;

	drawPixel(x0, y0 + r, color);
	drawPixel(x0, y0 - r, color);
	drawPixel(x0 + r, y0, color);
	drawPixel(x0 - r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}

		x++;
		ddF_x += 2;
		f += ddF_x;

		drawPixel(x0 + x, y0 + y, color);
		drawPixel(x0 - x, y0 + y, color);
		drawPixel(x0 + x, y0 - y, color);
		drawPixel(x0 - x, y0 - y, color);
		drawPixel(x0 + y, y0 + x, color);
		drawPixel(x0 - y, y0 + x, color);
		drawPixel(x0 + y, y0 - x, color);
		drawPixel(x0 - y, y0 - x, color);

	}
}

void swap(short *a, short *b)

{

	if (*a != *b) {

		*a = *a ^ *b;

		*b = *a ^ *b;

		*a = *a ^ *b;

	}

}

void SSD1306_close(int sig) {
	// De-Allocate memory for OLED buffer if any
	if (poledbuff)
		free(poledbuff);
	poledbuff = NULL;
	printf("clean poledbuff\n");

	// Release Raspberry I2C
//	bcm2835_i2c_end();
	//      printf("Release Raspberry I2C\n");

	// Release Raspberry I/O control
	//	bcm2835_close();
	//    printf("Release Raspberry I/O control\n");
	exit(0);
}

// clear everything (in the buffer)
void clearDisplay(void) {
	memset(poledbuff, 0, (ssd1306_lcdwidth * ssd1306_lcdheight / 8));
}

void drawChar(short x, short y, unsigned char c, unsigned short color,
		unsigned short bg, unsigned char size) {

	if ((x >= _width) || // Clip right
			(y >= _height) || // Clip bottom
			((x + 5 * size - 1) < 0) || // Clip left
			((y + 8 * size - 1) < 0))   // Clip top
		return;
	int8_t i = 0;
	int8_t j = 0;
	for (i = 0; i < 6; i++) {
		unsigned char line;
		if (i == 5)
			line = 0x0;
		else
			//line = pgm_read_byte(font+(c*5)+i);
			line = font[(c * 5) + i];
		for (j = 0; j < 8; j++) {
			if (line & 0x1) {
				if (size == 1) // default size
					drawPixel(x + i, y + j, color);
				else {  // big size
					fillRect(x + (i * size), y + (j * size), size, size, color);
				}
			} else if (bg != color) {
				if (size == 1) // default size
					drawPixel(x + i, y + j, bg);
				else {  // big size
					fillRect(x + i * size, y + j * size, size, size, bg);
				}
			}

			line >>= 1;
		}
	}
}

void fillRect(short x, short y, short w, short h, unsigned short color) {
	// stupidest version - update in subclasses if desired!
	short i = 0;

	for (i = x; i < x + w; i++) {
		drawFastVLine(i, y, h, color);
	}
}

void drawFastVLine(short x, short y, short h, unsigned short color) {
	// stupidest version - update in subclasses if desired!
	drawLine(x, y, x, y + h - 1, color);
}

void drawLine(short xx0, short yy0, short xx1, short yy1, unsigned short color) {
	short x0 = xx0;
	short y0 = yy0;
	short x1 = xx1;
	short y1 = yy1;

	short steep = abs(y1 - y0) > abs(x1 - x0);

	if (steep) {
		swap(&x0, &y0);
		swap(&x1, &y1);
	}

	if (x0 > x1) {
		swap(&x0, &x1);
		swap(&y0, &y1);
	}

	short dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	short err = dx / 2;
	short ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	for (; x0 <= x1; x0++) {
		if (steep) {
			drawPixel(y0, x0, color);
		} else {
			drawPixel(x0, y0, color);
		}
		err -= dy;

		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}
size_t oled_write(unsigned char c) {
	if (c == '\n') {
		cursor_y += textsize * 8;
		cursor_x = 0;
	} else if (c == '\r') {
		// skip em
	} else {
		drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
		cursor_x += textsize * 6;

		/*	if (wrap && (cursor_x > (_width - textsize*6)))
		 {
		 cursor_y += textsize*8;
		 cursor_x = 0;
		 }*/
	}
	return 1;
}

void OLED_printf(const char * format, ...) {

	char buffer[64];
	char * p = buffer;
	int n;
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, sizeof(buffer) - 1, format, args);
	n = strlen(buffer);

	while (*p != 0 && n-- > 0) {
		oled_write((unsigned char) *p++);
	}

	va_end(args);
}

void constructor(short w, short h) {

	_width = WIDTH = w;
	_height = HEIGHT = h;
	rotation = 0;
	cursor_y = cursor_x = 0;
	textsize = 1;
	textcolor = textbgcolor = WHITE;
}

void drawBitmap(short x, short y, const unsigned char *bitmap, short w, short h,
		unsigned short color) {

	short i, j, byteWidth = (w + 7) / 8;

	for (j = 0; j < h; j++) {
		for (i = 0; i < w; i++) {
			if (*(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
				drawPixel(x + i, y + j, color);
			}
		}
	}
}

size_t rewrite(unsigned char c, int line, int index) {
	if (c == '\n') {
	} else if (c == '\r') {
		// skip em
	} else {
		fillRect((textsize * index * 8) + 1, textsize * 8 * line,
				textsize * 8 - 1, textsize * 8, BLACK);
		drawChar(textsize * index * 8 + 2, textsize * 8 * line, c, textcolor,
				textbgcolor, textsize);
	}
	return 1;
}

void setCursor(short x, short y) {
	cursor_x = x;
	cursor_y = y;
}
