/*
 macam - webcam app and QuickTime driver component
 Copyright (C) 2002 Matthias Krauss (macam@matthias-krauss.de)

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 $Id: MySmartMegaCamDriver.m,v 1.2 2002/08/29 11:03:32 mattik Exp $
 */

#import "MySmartMegaCamDriver.h"


@implementation MySmartMegaCamDriver

#define PRODUCT_SPCA504B 0x504b
#define VENDOR_SUNPLUS 0x4fc

+ (unsigned short) cameraUsbProductID { return PRODUCT_SPCA504B; }
+ (unsigned short) cameraUsbVendorID { return VENDOR_SUNPLUS; }
+ (NSString*) cameraName { return @"Aiptek Smart MegaCam";}


@end
