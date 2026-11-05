/******************************************************************************
*  ILI9341_T4 library for driving an ILI9341 screen via SPI with a Teensy 4/4.1
*  Implements vsync and differential updates from a memory framebuffer.
*
*  Copyright (c) 2020 Arvind Singh.  All right reserved.
*
* This library is free software; you can redistribute it and/or
*  modify it under the terms of the GNU Lesser General Public
*  License as published by the Free Software Foundation; either
*  version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*  Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public
*  License along with this library; if not, write to the Free Software
*  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*******************************************************************************/

#ifndef _II9341_T4_DIFFBUFF_H_
#define _II9341_T4_DIFFBUFF_H_

#ifdef __cplusplus

#include "DiffBuffTemplate.h"

namespace ILI9341_T4
{

    // *** NV3007 / ER-TFT2.79-1 port ***
    //
    // The native (orientation 0) frame is LANDSCAPE 428x142, matching the
    // displays as mounted in the car. The glass is physically a portrait
    // 142x428 bar, so init sets the MADCTL row/column-exchange bit (MV) -
    // the controller's landscape mode - making the long 428-pixel axis the
    // X axis. This is also required by the diff engine, whose 32-bit fast
    // paths need LX divisible by 4 (142 is not, 428 is).
    //
    // MAX_WRITE_LINE bounds the pixel count of a single DMA transfer. The
    // Teensy DMA major-loop counter (CITER) is 15 bits, so one transfer must
    // stay below 32767 pixels: 64 lines * 428 px = 27392 px. (The upstream
    // value 120 was chosen for 240px lines: 120*240 = 28800 px.)
    struct DiffBuffTraits
    {
        static constexpr int LX = 428;
        static constexpr int LY = 142;
        static constexpr int MAX_WRITE_LINE = 64;
        static constexpr int MIN_SCANLINE_SPACE = 8;
    };

    using DiffBuffBase = T4Diff::DiffBuffBaseT<DiffBuffTraits>;
    using DiffBuff = T4Diff::DiffBuffT<DiffBuffTraits>;

    template<int SIZEBUF>
    using DiffBuffStatic = T4Diff::DiffBuffStaticT<DiffBuffTraits, SIZEBUF>;

    using DiffBuffDummy = T4Diff::DiffBuffDummyT<DiffBuffTraits>;

}

extern template class T4Diff::DiffBuffBaseT<ILI9341_T4::DiffBuffTraits>;
extern template class T4Diff::DiffBuffT<ILI9341_T4::DiffBuffTraits>;
extern template class T4Diff::DiffBuffDummyT<ILI9341_T4::DiffBuffTraits>;

#endif

#endif

/** end of file */
