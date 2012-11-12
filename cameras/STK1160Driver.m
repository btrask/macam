/* Copyright (c) 2012, Ben Trask
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY BEN TRASK ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL BEN TRASK BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
/**
 * @file stk11xx-dev-0408.c
 * @author Ivor Hewitt
 * @date 2009-01-01
 * @version v1.0.x
 *
 * @brief Driver for Syntek USB video camera
 *
 * @note Copyright (C) Nicolas VIVIEN
 *       Copyright (C) Ivor Hewitt
 *       Copyright (C) Ben Trask
 *
 * @par Licences
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#import "STK1160Driver.h"
#import "MyCameraCentral.h"
#import "yuv2rgb.h"

#define numberof(x) (sizeof(x) / sizeof(x[0]))

typedef struct {
	NSUInteger width;
	NSUInteger height;
} ECVIntegerSize;

enum {
	ECVSTK1160HighFieldFlag = 1 << 6,
	ECVSTK1160NewImageFlag = 1 << 7,
};
enum {
	STK0408StatusRegistryIndex = 0x100,
};
enum {
	STK0408StatusStreaming = 1 << 7,
};

static void usb_stk11xx_write_registry(STK1160Driver *dev, u_int16_t i, u_int16_t v)
{
	(void)[dev writeIndex:i value:v];
}
static int dev_stk0408_write0(STK1160Driver *dev, u_int16_t mask, u_int16_t val)
{
	NSCAssert((mask & val) == val, @"Don't set values that will be masked out.");
	usb_stk11xx_write_registry(dev, 0x00, val);
	usb_stk11xx_write_registry(dev, 0x02, mask);
	return 0;
}
static int dev_stk0408_initialize_device(STK1160Driver *dev)
{
	usb_stk11xx_write_registry(dev, 0x0500, 0x0094);
	usb_stk11xx_write_registry(dev, 0x0203, 0x00a0);
	(void)[dev setFeatureAtIndex:1];

	usb_stk11xx_write_registry(dev, 0x0003, 0x0080);
	usb_stk11xx_write_registry(dev, 0x0001, 0x0003);
	dev_stk0408_write0(dev, 0x67, 1 << 5 | 1 << 0);
	struct {
		u_int16_t reg;
		u_int16_t val;
	} const settings[] = {
		{0x203, 0x04a},
		{0x00d, 0x000},
		{0x00f, 0x002},
		{0x103, 0x000},
		{0x018, 0x000},
		{0x01a, 0x014},
		{0x01b, 0x00e},
		{0x01c, 0x046},
		{0x019, 0x000},
		{0x300, 0x012},
		{0x350, 0x02d},
		{0x351, 0x001},
		{0x352, 0x000},
		{0x353, 0x000},
		{0x300, 0x080},
		{0x018, 0x010},
		{0x202, 0x00f},
	};
	NSUInteger i = 0;
	for(; i < numberof(settings); i++) usb_stk11xx_write_registry(dev, settings[i].reg, settings[i].val);
	usb_stk11xx_write_registry(dev, STK0408StatusRegistryIndex, 0x33);
	return 0;
}

static IsocFrameResult STK1160IsocFrameScanner(
	IOUSBIsocFrame const *const frame,
	UInt8 const *const buffer,
	UInt32 *const dataStart,
	UInt32 *const dataLength,
	UInt32 *const tailStart,
	UInt32 *const tailLength,
	GenericFrameInfo const *const frameInfo
	)
{
	UInt16 const length = frame->frActCount;
	if(!length) return invalidChunk;
	BOOL newFrame = NO;
	UInt16 skip = 4;
	if(ECVSTK1160NewImageFlag & buffer[0]) {
		newFrame = YES;
		skip = 8;
	}
	*dataStart = skip;
	*dataLength = length > skip ? length-skip : 0;
	*tailStart = length;
	*tailLength = 0;
	return newFrame ? newChunkFrame : validFrame;
}

@interface STK1160Driver(Private)

- (ECVIntegerSize)_inputSize;
- (BOOL)_initializeResolution;
- (BOOL)_setStreaming:(BOOL)flag;
- (BOOL)_SAA711XExpect:(u_int8_t const)val;

@end

@implementation STK1160Driver

#pragma mark +GenericDriver

+ (NSArray *)cameraUsbDescriptions
{
	return [NSArray arrayWithObjects:
		[NSDictionary dictionaryWithObjectsAndKeys:
			[NSNumber numberWithUnsignedShort:0x0408], @"idProduct",
			[NSNumber numberWithUnsignedShort:0x05e1], @"idVendor",
			@"EasyCap DC60", @"name",
			NULL],
		NULL];
}

#pragma mark -STK1160Driver

- (BOOL)readIndex:(UInt16 const)i value:(out UInt8 *const)outValue
{
	UInt8 v = 0;
	BOOL const r = [self usbReadCmdWithBRequest:kUSBRqGetStatus wValue:0 wIndex:i buf:&v len:sizeof(v)];
	if(outValue) *outValue = v;
	return r;
}
- (BOOL)writeIndex:(UInt16 const)i value:(UInt8 const)v
{
	return [self usbWriteCmdWithBRequest:kUSBRqClearFeature wValue:v wIndex:i buf:NULL len:0];
}
- (BOOL)setFeatureAtIndex:(u_int16_t)i
{
	return [self usbCmdWithBRequestType:USBmakebmRequestType(kUSBOut, kUSBStandard, kUSBDevice) bRequest:kUSBRqSetFeature wValue:0 wIndex:i buf:NULL len:0];
}

#pragma mark -STK1160Driver(Private)

- (ECVIntegerSize)_inputSize
{
	return (ECVIntegerSize){720, [self is60HzFormat] ? 480 : 576};
}
- (BOOL)_initializeResolution
{
	ECVIntegerSize inputSize = [self _inputSize];
	ECVIntegerSize standardSize = inputSize;
	switch(inputSize.width) {
		case 704:
		case 352:
		case 176:
			inputSize.width = 704;
			standardSize.width = 706;
			break;
		case 640:
		case 320:
		case 160:
			inputSize.width = 640;
			standardSize.width = 644;
			break;
	}
	switch(inputSize.height) {
		case 576:
		case 288:
		case 144:
			inputSize.height = 576;
			standardSize.height = 578;
			break;
		case 480:
		case 240:
		case 120:
			inputSize.height = 480;
			standardSize.height = 486;
			break;
	}
	size_t const bpp = 2;//ECVPixelFormatBytesPerPixel([self pixelFormat]);
	struct {
		u_int16_t reg;
		u_int16_t val;
	} settings[] = {
		{0x110, (standardSize.width - inputSize.width) * bpp},
		{0x111, 0},
		{0x112, (standardSize.height - inputSize.height) / 2},
		{0x113, 0},
		{0x114, standardSize.width * bpp},
		{0x115, 5},
		{0x116, standardSize.height / 2},
		{0x117, ![self is60HzFormat]},
	};
	NSUInteger i = 0;
	for(; i < numberof(settings); i++) if(![self writeIndex:settings[i].reg value:settings[i].val]) return NO;
	return YES;
}
- (BOOL)_setStreaming:(BOOL)flag
{
	u_int8_t value;
	if(![self readIndex:STK0408StatusRegistryIndex value:&value]) return NO;
	if(flag) value |= STK0408StatusStreaming;
	else value &= ~STK0408StatusStreaming;
	return [self writeIndex:STK0408StatusRegistryIndex value:value];
}
- (BOOL)_SAA711XExpect:(u_int8_t const)val
{
	NSUInteger retry = 4;
	u_int8_t result = 0;
	while(retry--) {
		if(![self readIndex:0x201 value:&result]) return NO;
		if(val == result) return YES;
		usleep(100);
	}
	NSLog(@"Invalid SAA711X result %x (expected %x)", (unsigned)result, (unsigned)val);
	return NO;
}

#pragma mark -STK1160Driver<SAA711XDevice>

- (BOOL)writeSAA711XRegister:(u_int8_t)reg value:(int16_t)val
{
	if(![self writeIndex:0x204 value:reg]) return NO;
	if(![self writeIndex:0x205 value:val]) return NO;
	if(![self writeIndex:0x200 value:0x01]) return NO;
	if(![self _SAA711XExpect:0x04]) {
		NSLog(@"SAA711X failed to write %x to %x", (unsigned)val, (unsigned)reg);
		return NO;
	}
	return YES;
}
- (BOOL)readSAA711XRegister:(u_int8_t)reg value:(out u_int8_t *)outVal
{
	if(![self writeIndex:0x208 value:reg]) return NO;
	if(![self writeIndex:0x200 value:0x20]) return NO;
	if(![self _SAA711XExpect:0x01]) {
		NSLog(@"SAA711X failed to read %x", (unsigned)reg);
		return NO;
	}
	return [self readIndex:0x209 value:outVal];
}
- (SAA711XMODESource)SAA711XMODESource
{
	return SAA711XMODESVideoAI12_YGain;
}
- (BOOL)SVideo
{
	return YES;
}
- (SAA711XCSTDFormat)SAA711XCSTDFormat
{
	return SAA711XCSTDNTSCM;
}
- (BOOL)is60HzFormat
{
	return YES;
}
- (BOOL)SAA711XRTP0OutputPolarityInverted
{
	return YES;
}

#pragma mark -GenericDriver

- (id)initWithCentral:(MyCameraCentral *const)c
{
	if((self = [super initWithCentral:c])) {
		_SAA711XChip = [[SAA711XChip alloc] init];
		[_SAA711XChip setDevice:self];
		compressionType = proprietaryCompression;
	}
	return self;
}
- (BOOL)supportsResolution:(CameraResolution const)res fps:(short const)rate
{
	return res == ResolutionVGA;
}
- (CameraResolution)defaultResolutionAndRate:(short *const)outRate
{
	if(outRate) *outRate = 60;
	return ResolutionVGA;
}
- (UInt8)getGrabbingPipe
{
	return 2;
}
- (BOOL)setGrabInterfacePipe
{
	return YES;
}
- (void)setIsocFrameFunctions
{
	grabContext.isocFrameScanner = (IsocFrameResult (*)())STK1160IsocFrameScanner;
	grabContext.isocDataCopier = genericIsocDataCopier;
}
- (BOOL)startupGrabStream
{
	dev_stk0408_initialize_device(self);
	if(![_SAA711XChip initialize]) return NO;
//	if(![self _setVideoSource:[self videoSource]]) return NO; // No-op for s-video
	if(![self _initializeResolution]) return NO;
	if(![self usbSetAltInterfaceTo:5 testPipe:[self getGrabbingPipe]]) return NO;
	if(![self _setStreaming:YES]) return NO;
	return YES;
}
- (void)shutdownGrabStream
{
	[self _setStreaming:NO];
	[self usbSetAltInterfaceTo:0 testPipe:[self getGrabbingPipe]];
}
- (BOOL)decodeBufferProprietary:(GenericChunkBuffer *const)buffer
{
	// FIXME: Draws in grayscale with bad aspect ratio/cropping.
	int x, y;
	for(y = 0; y < 240; ++y) {
		for(x = 0; x < 640; ++x) { // HACK: Crop the edge from 720 to 640 because scaling is hard.
			nextImageBuffer[(y*2+0)*640*3+x*3+0] = buffer->buffer[y*720*2+x*2+1];
			nextImageBuffer[(y*2+0)*640*3+x*3+1] = buffer->buffer[y*720*2+x*2+1];
			nextImageBuffer[(y*2+0)*640*3+x*3+2] = buffer->buffer[y*720*2+x*2+1];
			nextImageBuffer[(y*2+1)*640*3+x*3+0] = buffer->buffer[y*720*2+x*2+1];
			nextImageBuffer[(y*2+1)*640*3+x*3+1] = buffer->buffer[y*720*2+x*2+1];
			nextImageBuffer[(y*2+1)*640*3+x*3+2] = buffer->buffer[y*720*2+x*2+1];
		}
	}
	return YES;
}

#pragma mark -MyCameraDriver

- (int)usbGetIsocFrameSize
{
	return 3072;
}

#pragma mark -NSObject

- (void)dealloc
{
	[_SAA711XChip release];
	[super dealloc];
}

@end
