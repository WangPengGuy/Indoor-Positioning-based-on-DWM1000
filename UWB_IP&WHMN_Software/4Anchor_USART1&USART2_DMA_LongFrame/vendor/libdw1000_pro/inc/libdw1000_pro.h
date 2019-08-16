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

#ifndef __LIBDW1000_PRO_H__
#define __LIBDW1000_PRO_H__

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "libdw1000Spi.h"

// PMSC
//#define PMSC 0x36
//#define PMSC_CTRL0_SUB 0x00
//#define LEN_PMSC_CTRL0 4
//#define PMSC_LEDC 0x28
//#define LEN_PMSC_LEDC 4
#define PMSC_TXFSEQ 0x26
#define LEN_PMSC_TXFSEQ 2

int dwConfigureWithFilterAddress(dwDevice_t* dev, uint8_t* address);
void dwSetDefaultsWithFrameFilter(dwDevice_t* dev);
void dwSetTxPower(dwDevice_t *dev);

void dwEnableEXTPA(dwDevice_t* dev);
void dwEnableEXTTXE(dwDevice_t* dev);
void dwAddPA(dwDevice_t* dev);

void getPrintableDeviceIdentifier(dwDevice_t* dev,char msgBuffer[]);
void getPrintableExtendedUniqueIdentifier(dwDevice_t* dev,char msgBuffer[]);
void getPrintableNetworkIdAndShortAddress(dwDevice_t* dev,char msgBuffer[]);
void getPrintableDeviceMode(dwDevice_t* dev,char msgBuffer[]);


#endif

