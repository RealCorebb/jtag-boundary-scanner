/*
 * JTAG Core library
 * Copyright (c) 2008 - 2024 Viveris Technologies
 *
 * JTAG Core library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * JTAG Core library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with JTAG Core library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

/**
 * @file   ftdi_jtag_drv.c
 * @brief  FTDI based probes driver
 * @author Jean-François DEL NERO <Jean-Francois.DELNERO@viveris.fr>
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if !defined(WIN32)
#include <sys/time.h>
#endif

#include "../drv_loader.h"
#include "../../jtag_core_internal.h"
#include "../../jtag_core.h"

#include "../../bsdl_parser/bsdl_loader.h"

#include "../../dbg_logs.h"

#include "../../os_interface/os_interface.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "ftdi/ftd2xx.h"

#ifdef __cplusplus
}
#endif

#if defined(_WIN64)
#define MACH_WORD long long
#else
#define MACH_WORD int
#endif

#if defined(WIN32)

typedef struct _drv_desc
{
	char drv_id[128];
	char drv_desc[128];
	int id;
	int ftdi_index;
} drv_desc;

#define PROBE_GENERIC_FTDI 0

#define MAX_PROBES_FTDI 8

static drv_desc subdrv_list[MAX_PROBES_FTDI] =
	{
		{"USB_GENERIC_FTDI_PROBE", "GENERIC USB FTDI PROBE", PROBE_GENERIC_FTDI, 0},
		{"USB_GENERIC_FTDI_PROBE", "GENERIC USB FTDI PROBE", PROBE_GENERIC_FTDI, 0},
		{"USB_GENERIC_FTDI_PROBE", "GENERIC USB FTDI PROBE", PROBE_GENERIC_FTDI, 0},
		{"USB_GENERIC_FTDI_PROBE", "GENERIC USB FTDI PROBE", PROBE_GENERIC_FTDI, 0},
		{"USB_GENERIC_FTDI_PROBE", "GENERIC USB FTDI PROBE", PROBE_GENERIC_FTDI, 0},
		{"USB_GENERIC_FTDI_PROBE", "GENERIC USB FTDI PROBE", PROBE_GENERIC_FTDI, 0},
		{"USB_GENERIC_FTDI_PROBE", "GENERIC USB FTDI PROBE", PROBE_GENERIC_FTDI, 0},
		{"USB_GENERIC_FTDI_PROBE", "GENERIC USB FTDI PROBE", PROBE_GENERIC_FTDI, 0}};

///////////////////////////////////////////////////////////////////////////////
// Command Processor for MPSSE and MCU Host Bus Emulation Modes
//
// The data shifting commands are made up of the following definitions:
// Bit 7 : 0
// Bit 6 : Do write TMS
// Bit 5 : Do read TDO
// Bit 4 : Do write TDI
// Bit 3 : LSB first = 1 else MSB first = 0
// Bit 2 : -ve CLK on read
// Bit 1 : bit mode = 1 else byte mode = 0
// Bit 0 : -ve CLK on write

// The write commands to TDI take effect when bits 7 and 6 are '0'. Read TDO will operate with TMS output or TDI output or on its own.

// Clock Data Bytes Out on +ve clock edge MSB first (no read)
// 0x10, LengthL, LengthH, [Byte1..Byte65536 (max)]

#define OP_WR_TMS (0x1 << 6)
#define OP_RD_TDO (0x1 << 5)
#define OP_WR_TDI (0x1 << 4)
#define OP_LSB_FIRST (0x1 << 3)
#define OP_FEDGE_RD (0x1 << 2)
#define OP_BIT_MODE (0x1 << 1)
#define OP_FEDGE_WR (0x1 << 0)

#define CMD_ENABLE_LOOPBACK 0x84
#define CMD_DISABLE_LOOPBACK 0x85
#define CMD_SET_DIVISOR 0x86 // +0xValueL, 0xValueH
#define CMD_WAIT_IO_HIGH 0x88
#define CMD_WAIT_IO_LOW 0x89

///////////////////////////////////////////////////////////////////////////////

static HMODULE lib_handle = 0;

static FT_HANDLE ftdih = NULL;
static FT_DEVICE ftdi_device;

static int trst_oe_pin, trst_state_pin;
static int srst_oe_pin, srst_state_pin;
static int led_pin;

static unsigned char low_direction;
static unsigned char low_polarity;
static unsigned char low_output;

static unsigned char high_output;
static unsigned char high_polarity;
static unsigned char high_direction;

unsigned char ftdi_out_buf[64 * 1024];
unsigned char ftdi_in_buf[64 * 1024];

#if !defined(WIN32)

int Sleep(unsigned int timeout_ms)
{
	struct timeval tv;
	tv.tv_sec = timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;
	select(0, NULL, NULL, NULL, &tv);

	return 0;
}
#endif

#if !defined(FTDILIB)

typedef FT_STATUS(WINAPI *FT_OPEN)(DWORD deviceNumber, FT_HANDLE *pHandle);
typedef FT_STATUS(WINAPI *FT_OPENEX)(PVOID pArg1, DWORD Flags, FT_HANDLE *pHandle);
typedef FT_STATUS(WINAPI *FT_READ)(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD nBufferSize, LPDWORD lpBytesReturned);
typedef FT_STATUS(WINAPI *FT_WRITE)(FT_HANDLE ftHandle, LPVOID lpBuffer, DWORD nBufferSize, LPDWORD lpBytesWritten);
typedef FT_STATUS(WINAPI *FT_GETSTATUS)(FT_HANDLE ftHandle, DWORD *dwRxBytes, DWORD *dwTxBytes, DWORD *dwEventDWord);
typedef FT_STATUS(WINAPI *FT_PURGE)(FT_HANDLE ftHandle, ULONG Mask);
typedef FT_STATUS(WINAPI *FT_SETUSBPARAMETERS)(FT_HANDLE ftHandle, ULONG ulInTransferSize, ULONG ulOutTransferSize);
typedef FT_STATUS(WINAPI *FT_SETLATENCYTIMER)(FT_HANDLE ftHandle, UCHAR ucLatency);
typedef FT_STATUS(WINAPI *FT_SETEVENTNOTIFICATION)(FT_HANDLE ftHandle, DWORD dwEventMask, PVOID pvArg);
typedef FT_STATUS(WINAPI *FT_CLOSE)(FT_HANDLE ftHandle);
typedef FT_STATUS(WINAPI *FT_LISTDEVICES)(LPVOID pArg1, LPVOID pArg2, DWORD Flags);
typedef FT_STATUS(WINAPI *FT_SETBITMODE)(FT_HANDLE ftHandle, UCHAR ucMask, UCHAR ucEnable);
typedef FT_STATUS(WINAPI *FT_SETTIMEOUTS)(FT_HANDLE ftHandle, ULONG ReadTimeout, ULONG WriteTimeout);
typedef FT_STATUS(WINAPI *FT_GETQUEUESTATUS)(FT_HANDLE ftHandle, DWORD *dwRxBytes);
typedef FT_STATUS(WINAPI *FT_GETDEVICEINFO)(FT_HANDLE ftHandle, FT_DEVICE *lpftDevice, LPDWORD lpdwID, PCHAR SerialNumber, PCHAR Description, LPVOID Dummy);
typedef FT_STATUS(WINAPI *FT_RESETDEVICE)(FT_HANDLE ftHandle);
typedef FT_STATUS(WINAPI *FT_SETCHARS)(FT_HANDLE ftHandle, UCHAR EventChar, UCHAR EventCharEnabled, UCHAR ErrorChar, UCHAR ErrorCharEnabled);

FT_OPEN pFT_Open;
FT_OPENEX pFT_OpenEx;
FT_READ pFT_Read;
FT_WRITE pFT_Write;
FT_GETSTATUS pFT_GetStatus;
FT_PURGE pFT_Purge;
FT_SETUSBPARAMETERS pFT_SetUSBParameters;
FT_SETLATENCYTIMER pFT_SetLatencyTimer;
FT_SETEVENTNOTIFICATION pFT_SetEventNotification;
FT_CLOSE pFT_Close;
FT_LISTDEVICES pFT_ListDevices;
FT_SETBITMODE pFT_SetBitMode;
FT_SETTIMEOUTS pFT_SetTimeouts;
FT_GETQUEUESTATUS pFT_GetQueueStatus;
FT_GETDEVICEINFO pFT_GetDeviceInfo;
FT_RESETDEVICE pFT_ResetDevice;
FT_SETCHARS pFT_SetChars;

#else

#endif

static int ft2232_set_data_bits_low_byte(unsigned char value, unsigned char direction)
{
	FT_STATUS status;
	DWORD dw_bytes_written = 0;
	unsigned char buf[3];

	buf[0] = 0x80;		// command "set data bits low byte"
	buf[1] = value;		// value
	buf[2] = direction; // direction

	status = pFT_Write(ftdih, buf, sizeof(buf), &dw_bytes_written);
	if ((status != FT_OK) || (dw_bytes_written != sizeof(buf)))
	{
		return -1;
	}

	return 0;
}

static int ft2232_set_data_bits_high_byte(unsigned char value, unsigned char direction)
{
	FT_STATUS status;
	DWORD dw_bytes_written = 0;
	unsigned char buf[3];

	buf[0] = 0x82;		// command "set data bits high byte"
	buf[1] = value;		// value
	buf[2] = direction; // direction

	status = pFT_Write(ftdih, buf, sizeof(buf), &dw_bytes_written);
	if ((status != FT_OK) || (dw_bytes_written != sizeof(buf)))
	{
		return -1;
	}

	return 0;
}

static int ft2232h_enable_rtck(int enable)
{
	FT_STATUS status;
	DWORD dw_bytes_written = 0;
	unsigned char buf;

	buf = enable ? 0x96 : 0x97;

	status = pFT_Write(ftdih, &buf, sizeof(buf), &dw_bytes_written);
	if ((status != FT_OK) || (dw_bytes_written != sizeof(buf)))
	{
		return -1;
	}

	return 0;
}

int drv_FTDI_Detect(jtag_core *jc)
{
	int numDevs, i, validDevs = 0;
	FT_STATUS status;
	char SerialNumber[512];

	if (lib_handle == NULL)
		lib_handle = LoadLibrary("ftd2xx.dll");

	if (lib_handle)
	{
		pFT_ListDevices = (FT_LISTDEVICES)GetProcAddress(lib_handle, "FT_ListDevices");
		if (!pFT_ListDevices)
			return 0;

		status = pFT_ListDevices(&numDevs, NULL, FT_LIST_NUMBER_ONLY);
		if (status != FT_OK || numDevs == 0)
		{
			jtagcore_logs_printf(jc, MSG_ERROR, "pFT_ListDevices : Error %x !\r\n", status);
			return 0;
		}

		for (i = 0; i < numDevs && i < MAX_PROBES_FTDI; i++)
		{
			status = pFT_ListDevices((LPVOID)(MACH_WORD)i, SerialNumber, FT_LIST_BY_INDEX | FT_OPEN_BY_DESCRIPTION);
			if (status != FT_OK)
			{
				jtagcore_logs_printf(jc, MSG_ERROR, "pFT_ListDevices : Error %x at index %d !\r\n", status, i);
				continue; // Skip this device and try the next one
			}

			strcpy(subdrv_list[validDevs].drv_id, SerialNumber);
			strcpy(subdrv_list[validDevs].drv_desc, SerialNumber);
			strcat(subdrv_list[validDevs].drv_desc, " ");

			status = pFT_ListDevices((LPVOID)(MACH_WORD)i, SerialNumber, FT_LIST_BY_INDEX | FT_OPEN_BY_SERIAL_NUMBER);
			if (status != FT_OK)
			{
				jtagcore_logs_printf(jc, MSG_ERROR, "pFT_ListDevices : Error %x at index %d !\r\n", status, i);
				continue; // Skip this device and try the next one
			}

			strcat(subdrv_list[validDevs].drv_desc, SerialNumber);

			validDevs++; // Increment only for successfully detected devices
		}

		jtagcore_logs_printf(jc, MSG_INFO_1, "drv_FTDI_Detect : %d valid interface(s) found !\r\n", validDevs);

		return validDevs;
	}
	else
	{
		jtagcore_logs_printf(jc, MSG_INFO_1, "drv_FTDI_Detect : ftd2xx.dll not found !\r\n");
	}

	return 0;
}

void update_gpio_state(int index, int state)
{
	if (index >= 0)
	{
		if (index < 8)
		{
			if (state)
				low_output |= (0x01 << index);
			else
				low_output &= ~(0x01 << index);
		}
		else
		{
			if (state)
				high_output |= (0x01 << (index - 8));
			else
				high_output &= ~(0x01 << (index - 8));
		}
	}
}




// 位反转函数：将 01010101 这种顺序彻底颠倒
static uint8_t reverse_bits(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

/**
 * @brief 发送单个字节并产生 GPIOL0 (ADBUS4) 上升沿触发信号
 */
static void bsi_send_byte_with_trigger(jtag_core *jc, uint8_t byte)
{
    // 1. 将 8bit 数据输出到 ACBUS (GPIOH0-7)

	//翻转
	uint8_t reversed_byte = reverse_bits(byte);
    high_output = reversed_byte;
	
	//high_output = byte;
    ft2232_set_data_bits_high_byte(high_output, high_direction);

	Sleep(100);

    // 2. 产生 GPIOL0 (ADBUS4) 的上升沿
    // 先拉低触发引脚 (ADBUS4 对应 bit 4)
	update_gpio_state(4, 0);
    ft2232_set_data_bits_low_byte((unsigned char)(low_output ^ low_polarity), low_direction);

	Sleep(100);

    // 再拉高触发引脚，产生上升沿使 CPLD 采样数据
   	update_gpio_state(4, 1);
    ft2232_set_data_bits_low_byte((unsigned char)(low_output ^ low_polarity), low_direction);

	// 再拉回低
	Sleep(300);
	update_gpio_state(4, 0);
    ft2232_set_data_bits_low_byte((unsigned char)(low_output ^ low_polarity), low_direction);
}

/**
 * @brief 按照 BSI 协议格式发送一帧 64bit (8 Bytes) 数据
 */
static void bsi_send_frame(jtag_core *jc, uint8_t cmd, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4)
{
	update_gpio_state(5, 0);
	ft2232_set_data_bits_low_byte(low_output, low_direction);
	Sleep(100);
    uint8_t frame[8];
    
    frame[0] = 0x55; // 起始帧 
    frame[1] = cmd;  // 命令帧
    frame[2] = d1;   // 数据1
    frame[3] = d2;   // 数据2
    frame[4] = d3;   // 数据3
    frame[5] = d4;   // 数据4
    // 校验帧 = 命令帧 ^ 数据1 ^ 数据2 ^ 数据3 ^ 数据4 
    frame[6] = cmd ^ d1 ^ d2 ^ d3 ^ d4; 
    frame[7] = 0xAA; // 结束帧 

    // 将 frame 内容输出到 log.txt
    FILE *fp = fopen("log.txt", "a");
    if (fp != NULL)
    {
        fprintf(fp, "%02X %02X %02X %02X %02X %02X %02X %02X\n",
                frame[0], frame[1], frame[2], frame[3],
                frame[4], frame[5], frame[6], frame[7]);
        fclose(fp);
    }

    // 循环发送 8 个字节，每次发送都会触发一次上升沿 
    for (int i = 0; i < 8; i++)
    {
        bsi_send_byte_with_trigger(jc, frame[i]);
    }
	Sleep(100);
	update_gpio_state(5, 1);
	ft2232_set_data_bits_low_byte(low_output, low_direction);
}

// --- 业务功能接口 ---

/**
 * @brief DAC 电压设置 (0x01)
 * @param dac_word 16位DAC配置数据 (DATAH 和 DATAL)
 */
void bsi_set_voltage(jtag_core *jc, uint16_t dac_word)
{
    uint8_t data_h = (uint8_t)((dac_word >> 8) & 0xFF);
    uint8_t data_l = (uint8_t)(dac_word & 0xFF);
    bsi_send_frame(jc, 0x01, 0x00, 0x00, data_h, data_l); // [cite: 178]
}

/**
 * @brief 通道使能开关 (A通道: 0x02, B通道: 0x03)
 * @param channel 0 为 A 通道, 1 为 B 通道
 * @param enable  1 为闭合(使能), 0 为断开
 */
void bsi_set_channel(jtag_core *jc, int channel, int enable)
{
    uint8_t cmd = (channel == 0) ? 0x02 : 0x03;
    uint8_t en_val = enable ? 0x01 : 0x00;
    bsi_send_frame(jc, cmd, 0x00, 0x00, 0x00, en_val); // [cite: 180, 182]
}

/**
 * @brief 系统复位 (0x00)
 */
void bsi_reset(jtag_core *jc)
{
    bsi_send_frame(jc, 0x00, 0x00, 0x00, 0x00, 0x00); // 
}







int drv_FTDI_Init(jtag_core *jc, int sub_drv, char *params)
{
	FT_STATUS status;
	DWORD deviceID;
	char SerialNumber[16];
	char Description[64];
	char tmp_str[64];
	int numDevs;
	int baseclock, divisor, tckfreq;
	DWORD devIndex;
	DWORD nbRead, nbtosend;
	int i;

#ifdef WIN32

	if (lib_handle == NULL)
		lib_handle = LoadLibrary("ftd2xx.dll");

	if (lib_handle)
	{
		pFT_Write = (FT_WRITE)GetProcAddress(lib_handle, "FT_Write");
		if (!pFT_Write)
			goto loadliberror;

		pFT_Read = (FT_READ)GetProcAddress(lib_handle, "FT_Read");
		if (!pFT_Read)
			goto loadliberror;

		pFT_GetStatus = (FT_GETSTATUS)GetProcAddress(lib_handle, "FT_GetStatus");
		if (!pFT_GetStatus)
			goto loadliberror;

		pFT_Open = (FT_OPEN)GetProcAddress(lib_handle, "FT_Open");
		if (!pFT_Open)
			goto loadliberror;

		pFT_Close = (FT_CLOSE)GetProcAddress(lib_handle, "FT_Close");
		if (!pFT_Close)
			goto loadliberror;

		pFT_Purge = (FT_PURGE)GetProcAddress(lib_handle, "FT_Purge");
		if (!pFT_Purge)
			goto loadliberror;

		pFT_SetUSBParameters = (FT_SETUSBPARAMETERS)GetProcAddress(lib_handle, "FT_SetUSBParameters");
		if (!pFT_SetUSBParameters)
			goto loadliberror;

		pFT_SetLatencyTimer = (FT_SETLATENCYTIMER)GetProcAddress(lib_handle, "FT_SetLatencyTimer");
		if (!pFT_SetLatencyTimer)
			goto loadliberror;

		pFT_SetEventNotification = (FT_SETEVENTNOTIFICATION)GetProcAddress(lib_handle, "FT_SetEventNotification");
		if (!pFT_SetEventNotification)
			goto loadliberror;

		pFT_OpenEx = (FT_OPENEX)GetProcAddress(lib_handle, "FT_OpenEx");
		if (!pFT_OpenEx)
			goto loadliberror;

		pFT_ListDevices = (FT_LISTDEVICES)GetProcAddress(lib_handle, "FT_ListDevices");
		if (!pFT_ListDevices)
			goto loadliberror;

		pFT_SetBitMode = (FT_SETBITMODE)GetProcAddress(lib_handle, "FT_SetBitMode");
		if (!pFT_SetBitMode)
			goto loadliberror;

		pFT_SetTimeouts = (FT_SETTIMEOUTS)GetProcAddress(lib_handle, "FT_SetTimeouts");
		if (!pFT_SetTimeouts)
			goto loadliberror;

		pFT_GetQueueStatus = (FT_GETQUEUESTATUS)GetProcAddress(lib_handle, "FT_GetQueueStatus");
		if (!pFT_GetQueueStatus)
			goto loadliberror;

		pFT_GetDeviceInfo = (FT_GETDEVICEINFO)GetProcAddress(lib_handle, "FT_GetDeviceInfo");
		if (!pFT_GetDeviceInfo)
			goto loadliberror;

		pFT_ResetDevice = (FT_RESETDEVICE)GetProcAddress(lib_handle, "FT_ResetDevice");
		if (!pFT_ResetDevice)
			goto loadliberror;

		pFT_SetChars = (FT_SETCHARS)GetProcAddress(lib_handle, "FT_SetChars");
		if (!pFT_SetChars)
			goto loadliberror;
	}
	else
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "drv_FTDI_Init : Can't open ftd2xx.dll !\r\n");
		return -1;
	}
#else
	// TODO : Linux lib loader.
	return -1;
#endif

	status = pFT_ListDevices(&numDevs, NULL, FT_LIST_NUMBER_ONLY);
	if (status != FT_OK && !numDevs)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_ListDevices : Error %x !\r\n", status);
		goto loadliberror;
	}

	devIndex = sub_drv;
	status = pFT_ListDevices((LPVOID)(MACH_WORD)devIndex, SerialNumber, FT_LIST_BY_INDEX | FT_OPEN_BY_SERIAL_NUMBER);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_ListDevices : Error %x !\r\n", status);
		goto loadliberror;
	}

	status = pFT_OpenEx(SerialNumber, FT_OPEN_BY_SERIAL_NUMBER, &ftdih);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_OpenEx : Error %x !\r\n", status);
		goto loadliberror;
	}

	status = pFT_ResetDevice(ftdih);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_ResetDevice : Error %x !\r\n", status);
		goto loadliberror;
	}

	status = pFT_GetQueueStatus(ftdih, &nbRead);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_GetQueueStatus : Error %x !\r\n", status);
		goto loadliberror;
	}

	if (nbRead > 0)
		pFT_Read(ftdih, &ftdi_in_buf, nbRead, &nbRead);

	// Set USB request transfer sizes to 64K
	status = pFT_SetUSBParameters(ftdih, 65536, 65535);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_SetUSBParameters : Error %x !\r\n", status);
		goto loadliberror;
	}

	// Disable event and error characters
	status = pFT_SetChars(ftdih, 0, 0, 0, 0);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_SetChars : Error %x !\r\n", status);
		goto loadliberror;
	}

	// Sets the read and write timeouts in milliseconds
	status = pFT_SetTimeouts(ftdih, 5000, 5000);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_SetTimeouts : Error %x !\r\n", status);
		goto loadliberror;
	}

	// Set the latency timer (default is 16mS)
	status = pFT_SetLatencyTimer(ftdih, 2);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_SetLatencyTimer : Error %x !\r\n", status);
		goto loadliberror;
	}

	// Reset controller
	status = pFT_SetBitMode(ftdih, 0x0, 0x00);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_SetBitMode : Error %x !\r\n", status);
		goto loadliberror;
	}

	// Enable MPSSE mode
	status = pFT_SetBitMode(ftdih, 0x0, 0x02);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_SetBitMode : Error %x !\r\n", status);
		goto loadliberror;
	}

	status = pFT_SetBitMode(ftdih, 0x0b, 2);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_SetBitMode : Error %x !\r\n", status);
		goto loadliberror;
	}

	status = pFT_GetDeviceInfo(ftdih, &ftdi_device, &deviceID,
							   SerialNumber, Description, NULL);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_GetDeviceInfo : Error %x !\r\n", status);
		goto loadliberror;
	}

	/*
	Olimex ARM-USB-OCD-H JTAG signals

	VREF – voltage follower input for the output buffers adjust JTAG signals as per your target board voltage levels
	ADBUS0 -> TCK; (out)
	ADBUS1 -> TDI; (out)
	ADBUS2 -> TDO; (in)
	ADBUS3 -> TMS; (out)
	ADBUS4 -> 0 to enable JTAG buffers;  (GPIOL0) (out)
	ADBUS5 -> 0 if target present;       (GPIOL1) (in)
	ADBUS6 -> TSRST in;                  (GPIOL2) (in)
	ADBUS7 -> RTCK; (in)                 (GPIOL3) (in)

	ACBUS0 -> TRST;                      (GPIOH0)
	ACBUS1 -> SRST;                      (GPIOH1)
	ACBUS2 -> TRST buffer enable         (GPIOH2)
	ACBUS3 -> RED LED;                   (GPIOH3)
	*/

	low_direction = 0xFB;
/* 	for (i = 0; i < 8; i++)
	{
		sprintf(tmp_str, "PROBE_FTDI_SET_PIN_DIR_ADBUS%d", i);
		if (jtagcore_getEnvVarValue(jc, tmp_str) > 0)
		{
			low_direction |= (0x01 << i);
		}
	} */

	low_output = 0x00;
	for (i = 0; i < 8; i++)
	{
		sprintf(tmp_str, "PROBE_FTDI_SET_PIN_DEFAULT_STATE_ADBUS%d", i);
		if (jtagcore_getEnvVarValue(jc, tmp_str) > 0)
		{
			low_output |= (0x01 << i);
		}
	}

	low_polarity = 0;
	for (i = 0; i < 8; i++)
	{
		sprintf(tmp_str, "PROBE_FTDI_SET_PIN_POLARITY_ADBUS%d", i);
		if (jtagcore_getEnvVarValue(jc, tmp_str) > 0)
		{
			low_polarity |= (0x01 << i);
		}
	}

	high_direction = 0xFF;
/* 	for (i = 0; i < 8; i++)
	{
		sprintf(tmp_str, "PROBE_FTDI_SET_PIN_DIR_ACBUS%d", i);
		if (jtagcore_getEnvVarValue(jc, tmp_str) > 0)
		{
			high_direction |= (0x01 << i);
		}
	} */

	high_output = 0x00;
	for (i = 0; i < 8; i++)
	{
		sprintf(tmp_str, "PROBE_FTDI_SET_PIN_DEFAULT_STATE_ACBUS%d", i);
		if (jtagcore_getEnvVarValue(jc, tmp_str) > 0)
		{
			high_output |= (0x01 << i);
		}
	}

	high_polarity = 0x00;
	for (i = 0; i < 8; i++)
	{
		sprintf(tmp_str, "PROBE_FTDI_SET_PIN_POLARITY_ACBUS%d", i);
		if (jtagcore_getEnvVarValue(jc, tmp_str) > 0)
		{
			high_polarity |= (0x01 << i);
		}
	}

	trst_oe_pin = jtagcore_getEnvVarValue(jc, "PROBE_FTDI_SET_TRST_OE_PINNUM");
	trst_state_pin = jtagcore_getEnvVarValue(jc, "PROBE_FTDI_SET_TRST_STATE_PINNUM");

	srst_oe_pin = jtagcore_getEnvVarValue(jc, "PROBE_FTDI_SET_SRST_OE_PINNUM");
	srst_state_pin = jtagcore_getEnvVarValue(jc, "PROBE_FTDI_SET_SRST_STATE_PINNUM");

	led_pin = jtagcore_getEnvVarValue(jc, "PROBE_FTDI_SET_CONNECTION_LED_PINNUM");

	/* jtag reset */
	update_gpio_state(trst_oe_pin, 1);
	update_gpio_state(trst_state_pin, 1);

	update_gpio_state(srst_oe_pin, 1);
	update_gpio_state(srst_state_pin, 0);

	/* turn red LED off */
	update_gpio_state(led_pin, 0);

	ft2232_set_data_bits_low_byte((unsigned char)(low_output ^ low_polarity), low_direction);
	ft2232_set_data_bits_high_byte((unsigned char)(high_output ^ high_polarity), high_direction);

	// Clock divisor
	// 0x86 ValueL ValueH
	// FT2232D/H
	// TCK clock = (12Mhz or 60Mhz)/ ((1 + ([ValueH << 8 | ValueL]))*2)

	baseclock = jtagcore_getEnvVarValue(jc, "PROBE_FTDI_INTERNAL_FREQ_KHZ");
	tckfreq = jtagcore_getEnvVarValue(jc, "PROBE_FTDI_TCK_FREQ_KHZ");
	if (baseclock <= 0 || tckfreq <= 0)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "drv_FTDI_Init : Invalid probe clock settings !\r\n");
		goto loadliberror;
	}

	divisor = ((baseclock / tckfreq) - 2) / 2;

	nbtosend = 0;
	ftdi_out_buf[nbtosend++] = CMD_SET_DIVISOR;
	ftdi_out_buf[nbtosend++] = divisor & 0xFF;
	ftdi_out_buf[nbtosend++] = (divisor >> 8) & 0xFF;

	status = pFT_Write(ftdih, ftdi_out_buf, nbtosend, &nbtosend);
	if (status != FT_OK)
	{
		jtagcore_logs_printf(jc, MSG_ERROR, "pFT_Write : Error %x !\r\n", status);
		goto loadliberror;
	}

#if 0 // Loopback
	nbtosend = 0;
	ftdi_out_buf[nbtosend++] = CMD_ENABLE_LOOPBACK;

	status = pFT_Write(ftdih, ftdi_out_buf, nbtosend, &nbtosend);
	if (status != FT_OK) {
		jtagcore_logs_printf(jc,MSG_ERROR,"pFT_Write : Error %x !\r\n",status);
		goto loadliberror;
	}
#endif

	if (jtagcore_getEnvVarValue(jc, "PROBE_FTDI_JTAG_ENABLE_RTCK") > 0)
	{
		ft2232h_enable_rtck(1);
	}

	/* Delay... */
	genos_pause(jtagcore_getEnvVarValue(jc, "PROBE_FTDI_JTAG_TRST_DELAY_MS"));

	/* turn red LED on */
	update_gpio_state(led_pin, 1);

	/* Release system & jtag reset */
	update_gpio_state(trst_state_pin, 0);
	update_gpio_state(srst_state_pin, 0);

	ft2232_set_data_bits_low_byte((unsigned char)(low_output ^ low_polarity), low_direction);
	ft2232_set_data_bits_high_byte((unsigned char)(high_output ^ high_polarity), high_direction);

	jtagcore_logs_printf(jc, MSG_INFO_0, "drv_FTDI_Init : Probe Driver loaded successfully...\r\n");

	update_gpio_state(4, 1);
	update_gpio_state(5, 1);
	update_gpio_state(8, 1);
	update_gpio_state(9, 1);
	update_gpio_state(10, 1);
	update_gpio_state(11, 1);
	update_gpio_state(12, 1);
	update_gpio_state(13, 1);
	update_gpio_state(14, 1);
	update_gpio_state(15, 1);

	ft2232_set_data_bits_low_byte((unsigned char)(low_output ^ low_polarity), low_direction);
	ft2232_set_data_bits_high_byte((unsigned char)(high_output ^ high_polarity), high_direction);

	Sleep(1000);
	//bsi_reset(jc);
	//Sleep(500);
	// 设置电压为 3.3V (对应 DAC 十六进制值 0x0A80)
	bsi_set_voltage(jc, 0x0A80);
	Sleep(500);
	bsi_set_channel(jc, 0, 1); // 使能 A 通道
	Sleep(500);
	bsi_set_channel(jc, 1, 1);

	return 0;

loadliberror:
	FreeLibrary(lib_handle);
	lib_handle = NULL;
	return -1;
}

int drv_FTDI_DeInit(jtag_core *jc)
{
	pFT_Close(ftdih);
	FreeLibrary(lib_handle);
	lib_handle = NULL;
	return 0;
}

int drv_FTDI_TDOTDI_xfer(jtag_core *jc, unsigned char *str_out, unsigned char *str_in, int size)
{
	int wr_bit_index, rd_bit_index;
	int j, l, payloadsize;
	int rounded_size;
	DWORD nbRead, nbtosend;
	FT_STATUS status;
	unsigned char opcode, data;

	rd_bit_index = 0;
	wr_bit_index = 0;

	memset(ftdi_out_buf, 0, sizeof(ftdi_out_buf));
	memset(ftdi_in_buf, 0, sizeof(ftdi_in_buf));

	if (size)
	{
		// Set the first TMS/DOUT
		if (str_out[wr_bit_index] & JTAG_STR_TMS)
		{
			if (str_in)
				opcode = (OP_WR_TMS | OP_LSB_FIRST | OP_BIT_MODE | OP_FEDGE_WR | OP_RD_TDO); // with TDO read back
			else
				opcode = (OP_WR_TMS | OP_LSB_FIRST | OP_BIT_MODE | OP_FEDGE_WR);

			nbtosend = 0;

			ftdi_out_buf[nbtosend++] = opcode;
			ftdi_out_buf[nbtosend++] = 0x00; // Size field : 1 Bit

			data = 0x00;

			if (str_out[wr_bit_index] & JTAG_STR_DOUT)
				data = 0x80; // Bit 7: TDI/DO pin state

			if (str_out[wr_bit_index] & JTAG_STR_TMS)
				data |= 0x3F; // TMS state

			wr_bit_index++;

			ftdi_out_buf[nbtosend++] = data; // Data field

			status = pFT_Write(ftdih, ftdi_out_buf, nbtosend, &nbtosend);

			if (opcode & OP_RD_TDO)
			{
				status = pFT_GetQueueStatus(ftdih, &nbRead);
				while (nbRead < 1)
				{
					// Sleep(3);
					status = pFT_GetQueueStatus(ftdih, &nbRead);
				}

				status = pFT_Read(ftdih, &ftdi_in_buf, nbRead, &nbRead);

				if (ftdi_in_buf[0] & 0x01)
				{
					str_in[rd_bit_index++] = JTAG_STR_DOUT;
				}
				else
				{
					str_in[rd_bit_index++] = 0x00;
				}
			}
		}

		if (wr_bit_index >= size)
			return 0;

		rounded_size = (size - wr_bit_index) & ~(0x7);
		if (rounded_size)
		{
			// byte(s) buffer transmission/reception

			if (str_in)
				opcode = (OP_WR_TDI | OP_LSB_FIRST | OP_FEDGE_WR | OP_RD_TDO);
			else
				opcode = (OP_WR_TDI | OP_LSB_FIRST | OP_FEDGE_WR);

			nbtosend = 0;

			ftdi_out_buf[nbtosend++] = opcode;
			ftdi_out_buf[nbtosend++] = (((rounded_size >> 3) - 1) & 0xff); // (Size-1) Low byte
			ftdi_out_buf[nbtosend++] = (((rounded_size >> 3) - 1) >> 8);   // (Size-1) High byte

			ftdi_out_buf[nbtosend] = 0x00;

			j = 0;
			payloadsize = 0;
			while (payloadsize < rounded_size)
			{
				if (str_out[wr_bit_index] & JTAG_STR_DOUT)
				{
					ftdi_out_buf[nbtosend] |= (0x01 << (j & 0x7));
				}

				j++;

				if (!(j & 0x7))
				{
					nbtosend++;
					ftdi_out_buf[nbtosend] = 0x00;
				}

				payloadsize++;
				wr_bit_index++;
			}

			status = pFT_Write(ftdih, ftdi_out_buf, nbtosend, &nbtosend);

			if (str_in)
			{
				do
				{
					// Sleep(3);
					status = pFT_GetQueueStatus(ftdih, &nbRead);
				} while (nbRead < (unsigned long)(rounded_size >> 3));

				status = pFT_Read(ftdih, &ftdi_in_buf, nbRead, &nbRead);

				for (l = 0; l < rounded_size; l++)
				{
					if (ftdi_in_buf[l >> 3] & (0x01 << (l & 7)))
					{
						str_in[rd_bit_index++] = JTAG_STR_DOUT;
					}
					else
					{
						str_in[rd_bit_index++] = 0x00;
					}
				}
			}
		}

		if (wr_bit_index < size)
		{
			// Send the remaining bits...

			if (str_in)
				opcode = (OP_WR_TDI | OP_LSB_FIRST | OP_BIT_MODE | OP_FEDGE_WR | OP_RD_TDO); // bit mode with TDO read back
			else
				opcode = (OP_WR_TDI | OP_LSB_FIRST | OP_BIT_MODE | OP_FEDGE_WR); // bit mode

			nbtosend = 0;

			ftdi_out_buf[nbtosend++] = opcode;
			ftdi_out_buf[nbtosend++] = (((size - wr_bit_index) - 1) & 0x7); // Size field
			ftdi_out_buf[nbtosend] = 0x00;									// Data field

			j = 0;
			payloadsize = 0;
			while (wr_bit_index < size) // Should left less than 8 bits.
			{
				if (str_out[wr_bit_index++] & JTAG_STR_DOUT)
				{
					ftdi_out_buf[nbtosend] |= (0x01 << (j & 0x7));
				}

				j++;

				payloadsize++;
			}

			nbtosend++;

			status = pFT_Write(ftdih, ftdi_out_buf, nbtosend, &nbtosend);

			if (opcode & OP_RD_TDO)
			{
				do
				{
					// Sleep(3);
					status = pFT_GetQueueStatus(ftdih, &nbRead);
				} while (nbRead < 1);

				status = pFT_Read(ftdih, &ftdi_in_buf, nbRead, &nbRead);

				for (l = 0; l < payloadsize; l++)
				{
					if (ftdi_in_buf[l >> 3] & (0x01 << (l & 7)))
					{
						str_in[rd_bit_index++] = JTAG_STR_DOUT;
					}
					else
					{
						str_in[rd_bit_index++] = 0x00;
					}
				}
			}
		}
	}

	return 0;
}

int drv_FTDI_TMS_xfer(jtag_core *jc, unsigned char *str_out, int size)
{
	int i;
	DWORD nbtosend;
	FT_STATUS status;
	unsigned char databyte;

	status = FT_OK;

	memset(ftdi_out_buf, 0, sizeof(ftdi_out_buf));
	memset(ftdi_in_buf, 0, sizeof(ftdi_in_buf));

	if (size)
	{
		i = 0;
		databyte = 0x00;
		while (size)
		{
			if (str_out[i] & JTAG_STR_TMS)
				databyte |= (0x01 << (i % 6));

			i++;
			size--;
			if (!(i % 6))
			{
				nbtosend = 0;
				ftdi_out_buf[nbtosend++] = (OP_WR_TMS | OP_LSB_FIRST | OP_BIT_MODE | OP_FEDGE_WR); // cmd
				ftdi_out_buf[nbtosend++] = 0x06 - 1;											   // 6 Bit

				if ((databyte & 0x20) && size)
					ftdi_out_buf[nbtosend++] = databyte | 0x40;
				else
					ftdi_out_buf[nbtosend++] = databyte;

				status = pFT_Write(ftdih, ftdi_out_buf, nbtosend, &nbtosend);

				databyte = 0x00;
			}
		}

		if ((i % 6))
		{
			nbtosend = 0;
			ftdi_out_buf[nbtosend++] = (OP_WR_TMS | OP_LSB_FIRST | OP_BIT_MODE | OP_FEDGE_WR);

			ftdi_out_buf[nbtosend++] = (i % 6) - 1; // 1 Bit

			ftdi_out_buf[nbtosend++] = databyte;

			status = pFT_Write(ftdih, ftdi_out_buf, nbtosend, &nbtosend);

			databyte = 0x00;
		}
		else
		{
		}
	}

	if (status != FT_OK)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

int drv_FTDI_libGetDrv(jtag_core *jc, int sub_drv, unsigned int infotype, void *returnvalue)
{

	drv_ptr drv_funcs =
		{
			(DRV_DETECT)drv_FTDI_Detect,
			(DRV_INIT)drv_FTDI_Init,
			(DRV_DEINIT)drv_FTDI_DeInit,
			(DRV_TXTMS)drv_FTDI_TMS_xfer,
			(DRV_TXRXDATA)drv_FTDI_TDOTDI_xfer,
			(DRV_GETMODULEINFOS)drv_FTDI_libGetDrv};

	return GetDrvInfo(
		jc,
		infotype,
		returnvalue,
		subdrv_list[sub_drv].drv_id,
		subdrv_list[sub_drv].drv_desc,
		&drv_funcs);
}

#endif