/****************************************************************************
 * boards/arm64/am62x/beagleplay/include/board_memorymap.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 * BeaglePlay (AM6254) — 2 GB LPDDR4 at 0x8000_0000.
 * NuttX load address = 0x8200_0000 to avoid the DDR region reserved by
 * the BeagleBoard.org U-Boot environment.
 * Device space = 0x0000_0000 – 0x7FFF_FFFF (2 GB).
 *
 * defconfig uses only the first 512 MB so images are portable between
 * BeaglePlay and PocketBeagle 2.  Increase CONFIG_RAM_SIZE to use more.
 ****************************************************************************/

#ifndef __BOARDS_ARM64_AM62X_BEAGLEPLAY_INCLUDE_BOARD_MEMORYMAP_H
#define __BOARDS_ARM64_AM62X_BEAGLEPLAY_INCLUDE_BOARD_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#define BEAGLEPLAY_DEVICEIO_BASE    0x00000000ul
#define BEAGLEPLAY_DEVICEIO_SIZE    0x80000000ul
#define BEAGLEPLAY_DDR_BASE         0x80000000ul
#define BEAGLEPLAY_DDR_SIZE         0x20000000ul   /* 512 MB window */
#define BEAGLEPLAY_LOAD_ADDR        0x82000000ul

#endif /* __BOARDS_ARM64_AM62X_BEAGLEPLAY_INCLUDE_BOARD_MEMORYMAP_H */
