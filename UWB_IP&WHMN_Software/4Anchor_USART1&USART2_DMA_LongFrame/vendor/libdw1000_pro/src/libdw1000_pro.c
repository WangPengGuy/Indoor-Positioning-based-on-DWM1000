/*
 * Driver for decaWave DW1000 802.15.4 UWB radio chip.
 *
 * Copyright (c) 2016 SnailTech By JustinLee
 * Converted to C from  the Decawave DW1000 library for arduino.
 * which is Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "libdw1000.h"
#include "libdw1000_pro.h"

// Useful shortcuts
#define delayms(delay) dev->ops->delayms(dev, delay)

// Utility functions
static void setBit(uint8_t data[], unsigned int n, unsigned int bit, bool val);
static void writeValueToBytes(uint8_t data[], long val, unsigned int n);
static bool getBit(uint8_t data[], unsigned int n, unsigned int bit);

static void setBit(uint8_t data[], unsigned int n, unsigned int bit, bool val) {
	unsigned int idx;
	unsigned int shift;

	idx = bit / 8;
	if(idx >= n) {
		return; // TODO proper error handling: out of bounds
	}
	uint8_t* targetByte = &data[idx];
	shift = bit % 8;
	if(val) {
		*targetByte |= (1<<shift);
	} else {
	  *targetByte &= ~(1<<shift);
	}
}

static bool getBit(uint8_t data[], unsigned int n, unsigned int bit) {
	unsigned int idx;
	unsigned int shift;

	idx = bit / 8;
	if(idx >= n) {
		return false; // TODO proper error handling: out of bounds
	}
	uint8_t targetByte = data[idx];
	shift = bit % 8;

	return (targetByte>>shift)&0x01;
}

static void writeValueToBytes(uint8_t data[], long val, unsigned int n) {
	unsigned int i;
	for(i = 0; i < n; i++) {
		data[i] = ((val >> (i * 8)) & 0xFF);
	}
}


/*=====================================================================================================*
**函数 : dwConfigureWithFilterAddress
**功能 : 配置DW1000芯片，带有过滤地址，当探测到信号之后，会在MAC层比较地址是否是自己的信息，如果不是直接过滤掉
**输入 : dev,设备
				 address,过滤地址，也是自身的地址	
**输出 : int,配置是否成功
**使用 : dwConfigureWithFilterAddress(dev,address);
**作者 : JustinLee
**日期 : 2016/08/10
**=====================================================================================================*/
int dwConfigureWithFilterAddress(dwDevice_t* dev, uint8_t* address)
{
//	uint8_t EUIAddress[LEN_EUI];
//	uint8_t EUIAddressRead[LEN_EUI];
	
  dwEnableClock(dev, dwClockAuto);
  delayms(5);

  // Reset the chip
  if (dev->ops->reset) {
    dev->ops->reset(dev);
  } else {
    dwSoftReset(dev);
  }

  if (dwGetDeviceId(dev) != 0xdeca0130) {
    return DW_ERROR_WRONG_ID;
  }

  // Set default address
  memset(dev->networkAndAddress, 0xff, LEN_PANADR);
  dwSpiWrite(dev, PANADR, NO_SUB, dev->networkAndAddress, LEN_PANADR);
	
	//Justin Add start
	//memset(dev->networkAndAddress, 0xff, LEN_EUI);
	dev->networkAndAddress[2]=0xcf;
	dev->networkAndAddress[3]=0xbc;	
	dwSpiWrite(dev, PANADR, NO_SUB, dev->networkAndAddress, LEN_PANADR);
//	EUIAddress[0] = 0x09;
//	EUIAddress[1] = 0x00;
//	EUIAddress[2] = 0x00;
//	EUIAddress[3] = 0x00;
//	EUIAddress[4] = 0x00;
//	EUIAddress[5] = 0x00;
//	EUIAddress[6] = 0xcf;
//	EUIAddress[7] = 0xbc;
  //dwSpiWrite(dev, EUI, NO_SUB, EUIAddress, LEN_EUI);
	dwSpiWrite(dev, EUI, NO_SUB, address, LEN_EUI);
//	dwSpiRead(dev, EUI, NO_SUB, EUIAddressRead, LEN_EUI);
//	printf("EUIAddressRead[0]:%d\r\n",EUIAddressRead[0]);
//	printf("EUIAddressRead[1]:%d\r\n",EUIAddressRead[1]);
//	printf("EUIAddressRead[2]:%d\r\n",EUIAddressRead[2]);
//	printf("EUIAddressRead[3]:%d\r\n",EUIAddressRead[3]);
//	printf("EUIAddressRead[4]:%d\r\n",EUIAddressRead[4]);
//	printf("EUIAddressRead[5]:%d\r\n",EUIAddressRead[5]);
//	printf("EUIAddressRead[6]:%d\r\n",EUIAddressRead[6]);
//	printf("EUIAddressRead[7]:%d\r\n",EUIAddressRead[7]);
	//Justin Add end
	
  // default configuration
  memset(dev->syscfg, 0, LEN_SYS_CFG);
  dwSetDoubleBuffering(dev, false);
	dwSetInterruptPolarity(dev, true);
	dwWriteSystemConfigurationRegister(dev);
	// default interrupt mask, i.e. no interrupts
	dwClearInterrupts(dev);
	dwWriteSystemEventMaskRegister(dev);
	// load LDE micro-code
	dwEnableClock(dev, dwClockXti);
	delayms(5);
	dwManageLDE(dev);
	delayms(5);
	dwEnableClock(dev, dwClockPll);
	delayms(5);
  //dev->ops->spiSetSpeed(dev, dwSpiSpeedHigh);

  // //Enable LED clock
  // dwSpiWrite32(dev, PMSC, PMSC_CTRL0_SUB, dwSpiRead32(dev, PMSC, PMSC_CTRL0_SUB) | 0x008C0000);
  //
  // // Setup all LEDs
  //
  // dwSpiWrite32(dev, 0x26, 0x00, dwSpiRead32(dev, 0x26, 0x00) | 0x1540);
  //
  // // Start the pll
  //
  // delayms(1);

  // Initialize for default configuration (as per datasheet)

  return DW_ERROR_OK;
}

/*=====================================================================================================*
**函数 : dwSetDefaultsWithFrameFilter
**功能 : 设置默认模式，同时开启帧过滤功能
**输入 : dev,设备
**输出 : int,配置是否成功
**使用 : dwSetDefaultsWithFrameFilter(dev);
**作者 : JustinLee
**日期 : 2016/08/10
**=====================================================================================================*/
void dwSetDefaultsWithFrameFilter(dwDevice_t* dev) 
{
	if(dev->deviceMode == TX_MODE) {

	} else if(dev->deviceMode == RX_MODE) {

	} else if(dev->deviceMode == IDLE_MODE) {
		dwUseExtendedFrameLength(dev, false);
		dwUseSmartPower(dev, true);
		dwSuppressFrameCheck(dev, false);
    //for global frame filtering
		dwSetFrameFilter(dev, true);//Justin adjust as true
    //for data frame (poll, poll_ack, range, range report, range failed) filtering
    dwSetFrameFilterAllowData(dev, true);//Justin adjust as true
    //for reserved (blink) frame filtering
    dwSetFrameFilterAllowReserved(dev, true);//Justin adjust as true
    dwSetFrameFilterAllowMAC(dev,true);//Justin adjust as true
    dwSetFrameFilterAllowBeacon(dev,true);//Justin adjust as true
    dwSetFrameFilterAllowAcknowledgement(dev,true);//Justin adjust as true
		
		dwInterruptOnSent(dev, true);
		dwInterruptOnReceived(dev, true);
    dwInterruptOnReceiveTimeout(dev, true);
		dwInterruptOnReceiveFailed(dev, false);
		dwInterruptOnReceiveTimestampAvailable(dev, false);
		dwInterruptOnAutomaticAcknowledgeTrigger(dev, false);
		dwSetReceiverAutoReenable(dev, true);
		// default mode when powering up the chip
		// still explicitly selected for later tuning
		dwEnableMode(dev, MODE_LONGDATA_RANGE_LOWPOWER);
	}
}

/*=====================================================================================================*
**函数 : dwSetTxPower
**功能 : 设置dw1000的发射功率
**输入 : dev,设备
**输出 : 无
**使用 : dwSetTxPower(dev);
**作者 : JustinLee
**日期 : 2016/08/10
**=====================================================================================================*/

void dwSetTxPower(dwDevice_t *dev)
{
	uint8_t txpower[LEN_TX_POWER];
// TX_POWER (enabled smart transmit power control)
	if(dev->channel == CHANNEL_1 || dev->channel == CHANNEL_2) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				//writeValueToBytes(txpower, 0x15355575L, LEN_TX_POWER);
				writeValueToBytes(txpower, 0x1B153555L, LEN_TX_POWER);//Justin add -- add 3dB
			} else {
				writeValueToBytes(txpower, 0x75757575L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				//writeValueToBytes(txpower, 0x07274767L, LEN_TX_POWER);
				writeValueToBytes(txpower, 0x0D072747L, LEN_TX_POWER);//Justin add -- add 3dB
			} else {
				writeValueToBytes(txpower, 0x67676767L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_3) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x0F2F4F6FL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x6F6F6F6FL, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x2B4B6B8BL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x8B8B8B8BL, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_4) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x1F1F3F5FL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x5F5F5F5FL, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x3A5A7A9AL, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x9A9A9A9AL, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_5) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x0E082848L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x48484848L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x25456585L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x85858585L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else if(dev->channel == CHANNEL_7) {
		if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x32527292L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0x92929292L, LEN_TX_POWER);
			}
		} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
			if(dev->smartPower) {
				writeValueToBytes(txpower, 0x5171B1D1L, LEN_TX_POWER);
			} else {
				writeValueToBytes(txpower, 0xD1D1D1D1L, LEN_TX_POWER);
			}
		} else {
			// TODO proper error/warning handling
		}
	} else {
		// TODO proper error/warning handling
	}

	dwSpiWrite(dev, TX_POWER, NO_SUB, txpower, LEN_TX_POWER);
}


/*=====================================================================================================*
**函数 : dwEnableEXTPA
**功能 : dw1000加PA控制引脚GPIO4配置为EXTPA模式
**输入 : dev,设备
**输出 : 无
**使用 : dwEnableEXTPA(dev);
**作者 : JustinLee
**日期 : 2016/10/17
**=====================================================================================================*/
void dwEnableEXTPA(dwDevice_t* dev)
{
  uint32_t reg;

  // Set GPIO4 in EXTPA mode
  reg = dwSpiRead32(dev, GPIO_CTRL, GPIO_MODE_SUB);
  reg &= ~0x0000C000ul;
  reg |= 0x00004000ul;
  dwSpiWrite32(dev, GPIO_CTRL, GPIO_MODE_SUB, reg);
}

/*=====================================================================================================*
**函数 : dwEnableEXTTXE
**功能 : dw1000加PA控制引脚GPIO5配置为EXTTXE模式
**输入 : dev,设备
**输出 : 无
**使用 : dwEnableEXTTXE(dev);
**作者 : JustinLee
**日期 : 2016/10/17
**=====================================================================================================*/
void dwEnableEXTTXE(dwDevice_t* dev)
{
  uint32_t reg;

  // Set GPIO5 in EXTTXE mode
  reg = dwSpiRead32(dev, GPIO_CTRL, GPIO_MODE_SUB);
  reg &= ~0x00030000ul;
  reg |= 0x00010000ul;
  dwSpiWrite32(dev, GPIO_CTRL, GPIO_MODE_SUB, reg);
}

/*=====================================================================================================*
**函数 : dwAddPA
**功能 : dw1000增加PA
**输入 : dev,设备
**输出 : 无
**使用 : dwAddPA(dev);
**作者 : JustinLee
**日期 : 2016/10/17
**=====================================================================================================*/
void dwAddPA(dwDevice_t* dev)
{
	//enable GPIOx external mode
	dwEnableEXTPA(dev);//set gpio4 as pa mode
	
	//TODO, gpio5 no config as txe mode, need more test, but gpio4 config as extpa pass
	dwEnableEXTTXE(dev);//set gpio5 as txe mode
	
	//if an external power amplifier is being used, TX fine grain power dequeencing must be disabled
	dwSpiWrite(dev, PMSC, PMSC_TXFSEQ, 0x0000, LEN_PMSC_TXFSEQ);
}

/* ###########################################################################
 * #### Pretty printed device information ####################################
 * ######################################################################### */
/*=====================================================================================================*
**函数 : getPrintableDeviceIdentifier
**功能 : 获取设备ID
**输入 : dev,设备
**			 msgBuffer[],返回的字符串
**输出 : 无
**使用 : char msg[90];getPrintableDeviceIdentifier(dev,msg);printf("%s\r\n",msg);
**作者 : JustinLee
**日期 : 2016/11/27
**=====================================================================================================*/
void getPrintableDeviceIdentifier(dwDevice_t* dev,char msgBuffer[]) {
	uint8_t data[LEN_DEV_ID];
	dwSpiRead(dev,DEV_ID, NO_SUB, data, LEN_DEV_ID);
	sprintf(msgBuffer, "%02X - model: %d, version: %d, revision: %d",
					(uint16_t)((data[3] << 8) | data[2]), data[1], (data[0] >> 4) & 0x0F, data[0] & 0x0F);
}
/*=====================================================================================================*
**函数 : getPrintableExtendedUniqueIdentifier
**功能 : 获取设备EUI
**输入 : dev,设备
**			 msgBuffer[],返回的字符串
**输出 : 无
**使用 : char msg[90];getPrintableExtendedUniqueIdentifier(dev,msg);printf("%s\r\n",msg);
**作者 : JustinLee
**日期 : 2016/11/27
**=====================================================================================================*/
void getPrintableExtendedUniqueIdentifier(dwDevice_t* dev,char msgBuffer[]) {
	uint8_t data[LEN_EUI];
	dwSpiRead(dev,EUI, NO_SUB, data, LEN_EUI);
	sprintf(msgBuffer, "EUI:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
					data[7], data[6], data[5], data[4], data[3], data[2], data[1], data[0]);
}
/*=====================================================================================================*
**函数 : getPrintableNetworkIdAndShortAddress
**功能 : 获取设备PANID和短地址
**输入 : dev,设备
**			 msgBuffer[],返回的字符串
**输出 : 无
**使用 : char msg[90];getPrintableNetworkIdAndShortAddress(dev,msg);printf("%s\r\n",msg);
**作者 : JustinLee
**日期 : 2016/11/27
**=====================================================================================================*/
void getPrintableNetworkIdAndShortAddress(dwDevice_t* dev,char msgBuffer[]) {
	uint8_t data[LEN_PANADR];
	dwSpiRead(dev,PANADR, NO_SUB, data, LEN_PANADR);
	sprintf(msgBuffer, "PAN: %02X, Short Address: %02X",
					(uint16_t)((data[3] << 8) | data[2]), (uint16_t)((data[1] << 8) | data[0]));
}
/*=====================================================================================================*
**函数 : getPrintableDeviceMode
**功能 : 获取设备模式，包括Pulse Frequence,Preamble length,TXRX Rate, Channel and Preamble code
**输入 : dev,设备
**			 msgBuffer[],返回的字符串
**输出 : 无
**使用 : char msg[90];getPrintableDeviceMode(dev,msg);printf("%s\r\n",msg);
**作者 : JustinLee
**日期 : 2016/11/27
**=====================================================================================================*/
void getPrintableDeviceMode(dwDevice_t* dev,char msgBuffer[]) {
	// data not read from device! data is from class
	// TODO
	uint8_t prf;
	uint16_t plen;
	uint16_t dr;
	uint8_t ch;
	uint8_t pcode;
	if(dev->pulseFrequency == TX_PULSE_FREQ_16MHZ) {
		prf = 16;
	} else if(dev->pulseFrequency == TX_PULSE_FREQ_64MHZ) {
		prf = 64;
	} else {
		prf = 0; // error
	}
	if(dev->preambleLength == TX_PREAMBLE_LEN_64) {
		plen = 64;
	} else if(dev->preambleLength == TX_PREAMBLE_LEN_128) {
		plen = 128;
	} else if(dev->preambleLength == TX_PREAMBLE_LEN_256) {
		plen = 256;
	} else if(dev->preambleLength == TX_PREAMBLE_LEN_512) {
		plen = 512;
	} else if(dev->preambleLength == TX_PREAMBLE_LEN_1024) {
		plen = 1024;
	} else if(dev->preambleLength == TX_PREAMBLE_LEN_1536) {
		plen = 1536;
	} else if(dev->preambleLength == TX_PREAMBLE_LEN_2048) {
		plen = 2048;
	} else if(dev->preambleLength == TX_PREAMBLE_LEN_4096) {
		plen = 4096;
	} else {
		plen = 0; // error
	}
	if(dev->dataRate == TRX_RATE_110KBPS) {
		dr = 110;
	} else if(dev->dataRate == TRX_RATE_850KBPS) {
		dr = 850;
	} else if(dev->dataRate == TRX_RATE_6800KBPS) {
		dr = 6800;
	} else {
		dr = 0; // error
	}
	ch    = (uint8_t)dev->channel;
	pcode = (uint8_t)dev->preambleCode;
	sprintf(msgBuffer, "Data rate: %u kb/s, PRF: %u MHz, Preamble: %u symbols (code #%u), Channel: #%u", dr, prf, plen, pcode, ch);
}

