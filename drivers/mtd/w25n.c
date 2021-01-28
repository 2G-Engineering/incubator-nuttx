/************************************************************************************
 * drivers/mtd/w25n.c
 * Driver for Winbond W25Nxx SPI nand flash.
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: zhuyanlin <zhuyanlin@fishsemi.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

#ifndef CONFIG_W25N_SPIMODE
#  define CONFIG_W25N_SPIMODE SPIDEV_MODE0
#endif

#ifndef CONFIG_W25N_SPIFREQUENCY
#  define CONFIG_W25N_SPIFREQUENCY  20000000
#endif

/* W25N Instructions ****************************************************************/

/*      Command                    Value     Description             Addr   Data    */
/*                                                                      Dummy       */

#define W25N_GET_FEATURE            0x0f   /* Get features           1   0   1      */
#define W25N_SET_FEATURE            0x1f   /* Set features           1   0   1      */
#define W25N_PAGE_READ              0x13   /* Array read             3   0   0      */
#define W25N_READ_FROM_CACHE        0x03   /* Output cache data
                                            *  on SO                 2   1   1-2112 */
#define W25N_READ_ID                0x9f   /* Read device ID         0   1   2      */
#define W25N_ECC_STATUS_READ        0x7c   /* Internal ECC status
                                            *  output                0   1   1      */
#define W25N_BLOCK_ERASE            0xd8   /* Block erase            3   0   0      */
#define W25N_PROGRAM_EXECUTE        0x10   /* Enter block/page
                                            *  address, execute      3   0   0      */
#define W25N_PROGRAM_LOAD           0x02   /* Load program data with
                                            *  cache reset first     2   0   1-2112 */
#define W25N_PROGRAM_LOAD_RANDOM    0x84   /* Load program data
                                            *  without cache reset   2   0   1-2112 */
#define W25N_WRITE_ENABLE           0x06   /*                        0   0   0      */
#define W25N_WRITE_DISABLE          0x04   /*                        0   0   0      */
#define W25N_RESET                  0xff   /* Reset the device       0   0   0      */

#define W25N_DUMMY                  0x00   /* No Operation           0   0   0      */

/* Feature register *****************************************************************/

/* JEDEC Read ID register values */

#define W25N_MANUFACTURER           0xEF

#define W25N_CAPACITY_512MBIT_B0    0x20  /* 512 Mb */
#define W25N_CAPACITY_512MBIT_B1    0xAA  /* 512 Mb */
#define W25N_CAPACITY_1GBIT_B0      0x21  /* 1 Gb */
#define W25N_CAPACITY_1GBIT_B1      0xAA  /* 1 Gb */
#define W25N_CAPACITY_2GBIT_B0      0x21  /* 2 Gb */
#define W25N_CAPACITY_2GBIT_B1      0xAB  /* 2 Gb */

#define W25N_NSECTORS_512MBIT       512   /* 512x131072 = 512Mbit memory capacity */
#define W25N_NSECTORS_1GBIT         1024  /* 1024x131072 = 1Gbit memory capacity */
#define W25N_NSECTORS_2GBIT         2048  /* 2048x131072 = 2Gbit memory capacity */

#define W25N_SECTOR_SHIFT           17    /* 131072 byte */
#define W25N_PAGE_SHIFT             11    /* 2048 */

/* Register address */

#define W25N_SECURE_OTP             0xb0
#define W25N_STATUS                 0xc0
#define W25N_BLOCK_PROTECTION       0xa0

/* Bit definitions */

/* Secure OTP (On-Time-Programmable) register */

#define W25N_SOTP_QE                (1 << 0)  /* Bit 0: Quad Enable */
#define W25N_SOTP_BUF               (1 << 3)  /* Bit 3: Buffer Read Mode */
#define W25N_SOTP_ECC               (1 << 4)  /* Bit 4: ECC enabled */
#define W25N_SOTP_SOTP_EN           (1 << 6)  /* Bit 6: Secure OTP Enable */
#define W25N_SOTP_SOTP_PROT         (1 << 7)  /* Bit 7: Secure OTP Protect */

/* Status register */

#define W25N_SR_OIP                 (1 << 0)  /* Bit 0: Operation in progress */
#define W25N_SR_WEL                 (1 << 1)  /* Bit 1: Write enable latch */
#define W25N_SR_E_FAIL              (1 << 2)  /* Bit 2: Erase fail */
#define W25N_SR_P_FAIL              (1 << 3)  /* Bit 3: Program Fail */
#define W25N_SR_ECC_S0              (1 << 4)  /* Bit 4-5: ECC Status  */
#define W25N_SR_ECC_S1              (1 << 5)

/* Block Protection register */

#define W25N_BP_SP                  (1 << 0)  /* Bit 0: Solid-protection (1Gb only) */
#define W25N_BP_COMPL               (1 << 1)  /* Bit 1: Complementary (1Gb only) */
#define W25N_BP_INV                 (1 << 2)  /* Bit 2: Invert (1Gb only) */
#define W25N_BP_BP0                 (1 << 3)  /* Bit 3: Block Protection 0 */
#define W25N_BP_BP1                 (1 << 4)  /* Bit 4: Block Protection 1 */
#define W25N_BP_BP2                 (1 << 5)  /* Bit 5: Block Protection 2 */
#define W25N_BP_BPRWD               (1 << 7)  /* Bit 7: Block Protection Register
                                               *        Write Disable */

/* ECC Status register */

#define W25N_FEATURE_ECC_MASK       (0x03 << 4)
#define W25N_FEATURE_ECC_ERROR      (0x02 << 4)
#define W25N_FEATURE_ECC_OFFSET     4
#define W25N_ECC_STATUS_MASK        0x0f

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct w25n_dev_s.
 */

struct w25n_dev_s
{
  struct mtd_dev_s     mtd;             /* MTD interface */
  FAR struct spi_dev_s *dev;            /* Saved SPI interface instance */
  uint32_t             spi_devid;       /* Chip select inputs */
  uint16_t             nsectors;        /* 1024 or 2048 */
  uint8_t              sectorshift;     /* 17 */
  uint8_t              pageshift;       /* 11 */
  uint8_t              eccstatus;       /* Internal ECC status */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* Helpers */

static inline void w25n_lock(FAR struct spi_dev_s *dev);
static inline void w25n_unlock(FAR struct spi_dev_s *dev);

static int w25n_readid(FAR struct w25n_dev_s *priv);
static bool w25n_waitstatus(FAR struct w25n_dev_s *priv, uint8_t mask,
                            bool successif);
static bool w25n_waitstatustimeout(FAR struct w25n_dev_s *priv, uint8_t mask,
                            bool successif, int32_t ustimeout);
static inline void w25n_writeenable(FAR struct w25n_dev_s *priv);
static inline void w25n_writedisable(FAR struct w25n_dev_s *priv);
static bool w25n_sectorerase(FAR struct w25n_dev_s *priv, off_t startsector);
static void w25n_readbuffer(FAR struct w25n_dev_s *priv, uint32_t address,
                            uint8_t *buffer, size_t length);
static bool w25n_read_page(FAR struct w25n_dev_s *priv, uint32_t position);

static void w25n_write_to_cache(FAR struct w25n_dev_s *priv, uint32_t address,
                                const uint8_t *buffer, size_t length);
static bool w25n_execute_write(FAR struct w25n_dev_s *priv, uint32_t position);

static inline void w25n_eccstatusread(FAR struct w25n_dev_s *priv);
static inline void w25n_enable_ecc(FAR struct w25n_dev_s *priv);
static inline void w25n_unlockblocks(FAR struct w25n_dev_s *priv);

/* MTD driver methods */

static ssize_t w25n_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buffer);
static ssize_t w25n_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer);
static ssize_t w25n_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buffer);
static ssize_t w25n_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR const uint8_t *buffer);
static int w25n_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);
static int w25n_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: w25n_lock
 ************************************************************************************/

static inline void w25n_lock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, true);

  SPI_SETMODE(dev, CONFIG_W25N_SPIMODE);
  SPI_SETBITS(dev, 8);
  SPI_HWFEATURES(dev, 0);
  SPI_SETFREQUENCY(dev, CONFIG_W25N_SPIFREQUENCY);
}

/************************************************************************************
 * Name: w25n_unlock
 ************************************************************************************/

static inline void w25n_unlock(FAR struct spi_dev_s *dev)
{
  SPI_LOCK(dev, false);
}

/************************************************************************************
 * Name: w25n_readid
 ************************************************************************************/

static int w25n_readid(FAR struct w25n_dev_s *priv)
{
  uint16_t manufacturer;
  uint16_t deviceid0;
  uint16_t deviceid1;

  finfo("priv: %p\n", priv);

  /* Lock the SPI bus, configure the bus, and select this FLASH part. */

  w25n_lock(priv->dev);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the "Read ID" command and read three ID bytes */

  SPI_SEND(priv->dev, W25N_READ_ID);
  SPI_SEND(priv->dev, W25N_DUMMY);
  manufacturer = SPI_SEND(priv->dev, W25N_DUMMY);
  deviceid1    = SPI_SEND(priv->dev, W25N_DUMMY);
  deviceid0    = SPI_SEND(priv->dev, W25N_DUMMY);

  /* De-select the FLASH and unlock the bus */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
  w25n_unlock(priv->dev);

  finfo("manufacturer: %02x deviceid: %02x, %02x\n",
           manufacturer, deviceid1, deviceid0);

  /* Check for a valid manufacturer */

  if (manufacturer == W25N_MANUFACTURER)
    {

      if ((deviceid0 == W25N_CAPACITY_512MBIT_B0) &&
          (deviceid1 == W25N_CAPACITY_512MBIT_B1))
        {
          priv->nsectors = W25N_NSECTORS_512MBIT;
        }
      else if ((deviceid0 == W25N_CAPACITY_1GBIT_B0) &&
               (deviceid1 == W25N_CAPACITY_1GBIT_B1))
        {
          priv->nsectors = W25N_NSECTORS_1GBIT;
        }
      /* 2Gb part will require some additional work to
       * handle dual-die functionality and is not
       * currently supported by this driver */
      else
        {
          return -ENODEV;
        }

      priv->sectorshift = W25N_SECTOR_SHIFT;
      priv->pageshift   = W25N_PAGE_SHIFT;
      return OK;
    }

  return -ENODEV;
}

/************************************************************************************
 * Name: w25n_waitstatus
 ************************************************************************************/

static bool w25n_waitstatus(FAR struct w25n_dev_s *priv, uint8_t mask, bool successif)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

      /* Get feature command */

      SPI_SEND(priv->dev, W25N_GET_FEATURE);
      SPI_SEND(priv->dev, W25N_STATUS);
      status = SPI_SEND(priv->dev, W25N_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
      nxsig_usleep(1000);
    }
  while ((status & W25N_SR_OIP) != 0);

  finfo("Complete %02x\n", status);

  return successif ? ((status & mask) != 0) : ((status & mask) == 0);
}

/************************************************************************************
 * Name: w25n_waitstatustimeout
 ************************************************************************************/

static bool w25n_waitstatustimeout(FAR struct w25n_dev_s *priv, uint8_t mask,
                                   bool successif, int32_t ustimeout)
{
  uint8_t status;

  /* Loop as long as the memory is busy with a write cycle */

  do
    {
      /* Select this FLASH part */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

      /* Get feature command */

      SPI_SEND(priv->dev, W25N_GET_FEATURE);
      SPI_SEND(priv->dev, W25N_STATUS);
      status = SPI_SEND(priv->dev, W25N_DUMMY);

      /* Deselect the FLASH */

      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
      ustimeout -= 1000;
    }
  while (((status & W25N_SR_OIP) != 0) && (!nxsig_usleep(1000)) && (ustimeout > 0));

  finfo("Complete %02x\n", status);

  return successif ? ((status & mask) != 0) : ((status & mask) == 0);
}

/************************************************************************************
 * Name:  w25n_writeenable
 ************************************************************************************/

static inline void w25n_writeenable(FAR struct w25n_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send Write Enable command */

  SPI_SEND(priv->dev, W25N_WRITE_ENABLE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
}

/************************************************************************************
 * Name:  w25n_writedisable
 ************************************************************************************/

static inline void w25n_writedisable(FAR struct w25n_dev_s *priv)
{
  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send Write Enable command */

  SPI_SEND(priv->dev, W25N_WRITE_DISABLE);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
}

/************************************************************************************
 * Name:  w25n_sectorerase (128K)
 ************************************************************************************/

static bool w25n_sectorerase(FAR struct w25n_dev_s *priv, off_t startsector)
{
  const uint16_t block = (uint16_t) (startsector << (priv->sectorshift - priv->pageshift));

  finfo("block sector: %08lx\n", (long)block);

  /* Send write enable instruction */

  w25n_writeenable(priv);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the Block Erase instruction */

  SPI_SEND(priv->dev, W25N_BLOCK_ERASE);
  SPI_SEND(priv->dev, W25N_DUMMY);
  SPI_SEND(priv->dev, (block >> 8) & 0xff);
  SPI_SEND(priv->dev, block & 0xff);

  /* De-select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  finfo("Erased\n");
  return w25n_waitstatus(priv, W25N_SR_E_FAIL, false);
}

/************************************************************************************
 * Name: w25n_erase
 ************************************************************************************/

static int w25n_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks)
{
  FAR struct w25n_dev_s *priv = (FAR struct w25n_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("Erase: startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the SPI bus until we complete the erase */

  w25n_lock(priv->dev);

  /* Wait all operations complete */

  w25n_waitstatus(priv, W25N_SR_OIP, false);

  while (blocksleft > 0)
    {
      if (!w25n_sectorerase(priv, startblock))
        {
          break;
        }

      startblock++;
      blocksleft--;
    }

  w25n_unlock(priv->dev);
  return nblocks - blocksleft;
}

/************************************************************************************
 * Name: w25n_readbuffer
 ************************************************************************************/

static void w25n_readbuffer(FAR struct w25n_dev_s *priv, uint32_t address,
                            uint8_t *buffer, size_t length)
{
  const uint16_t offset = address & ((1 << priv->pageshift) - 1);

  /* Select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  SPI_SEND(priv->dev, W25N_READ_FROM_CACHE);

  /* Send the address high byte first. */

  SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  SPI_SEND(priv->dev, (offset) & 0xff);

  /* Send a dummy byte */

  SPI_SEND(priv->dev, W25N_DUMMY);

  /* Then read all of the requested bytes */

  SPI_RECVBLOCK(priv->dev, buffer, length);

  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
}

/************************************************************************************
 * Name: w25n_read_page
 ************************************************************************************/

static bool w25n_read_page(FAR struct w25n_dev_s *priv, uint32_t pageaddress)
{
  const uint16_t row = (uint16_t) (pageaddress >> priv->pageshift);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the Read Page instruction */

  SPI_SEND(priv->dev, W25N_PAGE_READ);
  SPI_SEND(priv->dev, W25N_DUMMY);
  SPI_SEND(priv->dev, (row >> 8) & 0xff);
  SPI_SEND(priv->dev, row & 0xff);


  /* Deselect the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  /* Wait Page Read Complete */

  w25n_waitstatus(priv, W25N_SR_OIP, false);

  /* Check HardWare ECC result */

  w25n_eccstatusread(priv);
  if ((priv->eccstatus & W25N_FEATURE_ECC_ERROR))
    {
      /* ECC report uncorrectable, discard data */
      ferr("ECC reports uncorrectable error in page %08lx\n", pageaddress);
      return false;
    }

  return true;
}

/************************************************************************************
 * Name: w25n_read
 ************************************************************************************/

static ssize_t w25n_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct w25n_dev_s *priv = (FAR struct w25n_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;

  finfo("Read: offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Lock the SPI bus and select this FLASH part */

  w25n_lock(priv->dev);

  /* Wait all operations complete */

  w25n_waitstatus(priv, W25N_SR_OIP, false);

  while (bytesleft)
    {
      const uint32_t pageaddress = (position >> priv->pageshift) << priv->pageshift;
      const uint32_t spaceleft = pageaddress + (1 << priv->pageshift) - position;
      const size_t chunklength = bytesleft < spaceleft ? bytesleft : spaceleft;

      finfo("w25 read: page=%d, pos=%d, nb=%d\n",
            pageaddress >> priv->pageshift, position & ((1 << priv->pageshift) - 1),
            nbytes);

      if (!w25n_read_page(priv, pageaddress))
        {
          break;
        }

      w25n_readbuffer(priv, position, buffer, chunklength);

      position += chunklength;
      buffer += chunklength;
      bytesleft -= chunklength;
    }

  w25n_unlock(priv->dev);

  finfo("return nbytes: %d\n", (int)(nbytes - bytesleft));
  return nbytes - bytesleft;
}

/**************************************************************************
 * Name: w25n_bread
 **************************************************************************/

static ssize_t w25n_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buffer)
{
  ssize_t nbytes;
  FAR struct w25n_dev_s *priv = (FAR struct w25n_dev_s *)dev;

  finfo("Bread: startblock: %08lx nblocks: %d\n",
        (long)startblock, (int)nblocks);

  nbytes = w25n_read(dev, startblock << priv->pageshift,
                     nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }

  return nbytes;
}

/************************************************************************************
 * Name: w25n_write_to_cache
 ************************************************************************************/

static void w25n_write_to_cache(FAR struct w25n_dev_s *priv, uint32_t address,
                                const uint8_t *buffer, size_t length)
{
  const uint16_t offset = address & ((1 << priv->pageshift) - 1);

  /* Select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the Program Load command */

  SPI_SEND(priv->dev, W25N_PROGRAM_LOAD);

  /* Send the address high byte first. */

  SPI_SEND(priv->dev, (offset >> 8) & 0xff);
  SPI_SEND(priv->dev, (offset) & 0xff);

  /* Send block of bytes */

  SPI_SNDBLOCK(priv->dev, buffer, length);

  /* De-select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
}

/************************************************************************************
 * Name: w25n_execute_write
 ************************************************************************************/

static bool w25n_execute_write(FAR struct w25n_dev_s *priv, uint32_t pageaddress)
{
  const uint16_t row = (uint16_t) (pageaddress >> priv->pageshift);

  /* Select this FLASH part */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);

  /* Send the Program Execute instruction */

  SPI_SEND(priv->dev, W25N_PROGRAM_EXECUTE);
  SPI_SEND(priv->dev, W25N_DUMMY);
  SPI_SEND(priv->dev, (row >> 8) & 0xff);
  SPI_SEND(priv->dev, row & 0xff);

  /* De-select the FLASH */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  return w25n_waitstatus(priv, W25N_SR_P_FAIL, false);
}

/************************************************************************************
 * Name: w25n_write
 ************************************************************************************/

static ssize_t w25n_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                          FAR const uint8_t *buffer)
{
  FAR struct w25n_dev_s *priv = (FAR struct w25n_dev_s *)dev;
  size_t bytesleft = nbytes;
  uint32_t position = offset;

  finfo("Write: offset: %08lx nbytes: %d \n", (long)offset, (int)nbytes);
  w25n_lock(priv->dev);

  /* Wait all operations complete */

  w25n_waitstatus(priv, W25N_SR_OIP, false);

  while (bytesleft)
    {
      const uint32_t pageaddress = (position >> priv->pageshift) << priv->pageshift;
      const uint32_t spaceleft = pageaddress + (1 << priv->pageshift) - position;
      const size_t chunklength = bytesleft < spaceleft ? bytesleft : spaceleft;

      w25n_writeenable(priv);
      w25n_write_to_cache(priv, position, buffer, chunklength);
      if (!w25n_execute_write(priv, pageaddress))
        {
          break;
        }

      position += chunklength;
      buffer += chunklength;
      bytesleft -= chunklength;
    }

  w25n_unlock(priv->dev);

  return nbytes - bytesleft;
}

/**************************************************************************
 * Name: w25n_bwrite
 **************************************************************************/

static ssize_t w25n_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
    size_t nblocks, FAR const uint8_t *buffer)
{
  ssize_t nbytes;

  FAR struct w25n_dev_s *priv = (FAR struct w25n_dev_s *)dev;

  finfo("Bwrite: startblock: %08lx nblocks: %d\n",
        (long)startblock, (int)nblocks);

  /* Lock the SPI bus and write all of the pages to FLASH */

  nbytes = w25n_write(dev, startblock << priv->pageshift,
                nblocks << priv->pageshift, buffer);
  if (nbytes > 0)
    {
      nbytes >>= priv->pageshift;
    }

  return nbytes;
}

/************************************************************************************
 * Name: mx25l_ioctl
 ************************************************************************************/

static int w25n_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct w25n_dev_s *priv = (FAR struct w25n_dev_s *)dev;
  int ret = -EINVAL;

  finfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
                  (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;

              ret = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                       geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = w25n_erase(dev, 0, priv->nsectors);
        }
        break;

      case MTDIOC_ECCSTATUS:
        {
          uint8_t *result = (uint8_t *)arg;
          *result =
              (priv->eccstatus & W25N_FEATURE_ECC_MASK) >> W25N_FEATURE_ECC_OFFSET;

          ret = OK;
        }
      break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/************************************************************************************
 * Name:  w25n_eccstatusread
 ************************************************************************************/

static inline void w25n_eccstatusread(FAR struct w25n_dev_s *priv)
{
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->dev, W25N_GET_FEATURE);
  SPI_SEND(priv->dev, W25N_STATUS);
  priv->eccstatus = SPI_SEND(priv->dev, W25N_DUMMY);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);
}

/************************************************************************************
 * Name:  w25n_enable_ecc
 ************************************************************************************/

static inline void w25n_enable_buffer(FAR struct w25n_dev_s *priv)
{

    uint8_t regval;
  w25n_lock(priv->dev);

  /* Read the existing configuration register value */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->dev, W25N_GET_FEATURE);
  SPI_SEND(priv->dev, W25N_SECURE_OTP);
  regval = SPI_SEND(priv->dev, W25N_DUMMY);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  /* Set the enable buffer bit */
  regval |= W25N_SOTP_BUF;

  /* Write the modified register back to the chip */

  w25n_writeenable(priv);

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->dev, W25N_SET_FEATURE);
  SPI_SEND(priv->dev, W25N_SECURE_OTP);
  SPI_SEND(priv->dev, regval);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  w25n_writedisable(priv);
  w25n_unlock(priv->dev);
}

/************************************************************************************
 * Name:  w25n_enable_ecc
 ************************************************************************************/

static inline void w25n_enable_ecc(FAR struct w25n_dev_s *priv)
{
  uint8_t regval;

  w25n_lock(priv->dev);

  /* Read the existing configuration register value */

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->dev, W25N_GET_FEATURE);
  SPI_SEND(priv->dev, W25N_SECURE_OTP);
  regval = SPI_SEND(priv->dev, W25N_DUMMY);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  /* Set the enable ECC bit */
  regval |= W25N_SOTP_ECC;

  /* Write the modified register back to the chip */

  w25n_writeenable(priv);

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->dev, W25N_SET_FEATURE);
  SPI_SEND(priv->dev, W25N_SECURE_OTP);
  SPI_SEND(priv->dev, regval);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  w25n_writedisable(priv);
  w25n_unlock(priv->dev);
}

/************************************************************************************
 * Name:  w25n_unlockblocks
 ************************************************************************************/

static inline void w25n_unlockblocks(FAR struct w25n_dev_s *priv)
{
  uint8_t blockprotection = 0x00;

  w25n_lock(priv->dev);
  w25n_writeenable(priv);

  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);
  SPI_SEND(priv->dev, W25N_SET_FEATURE);
  SPI_SEND(priv->dev, W25N_BLOCK_PROTECTION);
  SPI_SEND(priv->dev, blockprotection);
  SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

  w25n_writedisable(priv);
  w25n_unlock(priv->dev);
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: w25n_initialize
 *
 * Description:
 *   Create an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *w25n_initialize(FAR struct spi_dev_s *dev,
                                      uint32_t spi_devid)
{
  FAR struct w25n_dev_s *priv;
  int ret;

  finfo("dev: %p\n", dev);

  priv = (FAR struct w25n_dev_s *)kmm_zalloc(sizeof(struct w25n_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure. (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = w25n_erase;
      priv->mtd.bread  = w25n_bread;
      priv->mtd.bwrite = w25n_bwrite;
      priv->mtd.ioctl  = w25n_ioctl;
      priv->mtd.name   = "w25n";
      priv->dev        = dev;
      priv->spi_devid  = spi_devid;

      /* De-select the FLASH */

      SPI_SELECT(dev, SPIDEV_FLASH(priv->spi_devid), false);

      /* Reset the flash */
      w25n_lock(priv->dev);
      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), true);
      SPI_SEND(priv->dev, W25N_RESET);
      SPI_SELECT(priv->dev, SPIDEV_FLASH(priv->spi_devid), false);

      /* Wait reset complete */

      w25n_waitstatustimeout(priv, W25N_SR_OIP, false, 50 * USEC_PER_MSEC);
      w25n_unlock(priv->dev);

      /* Identify the FLASH chip and get its capacity */

      ret = w25n_readid(priv);
      if (ret != OK)
        {
          /* Unrecognized! Discard all of that work we just did and return NULL */

          ferr("ERROR: Unrecognized\n");
          kmm_free(priv);
          return NULL;
        }

      w25n_enable_ecc(priv);
      w25n_waitstatus(priv, W25N_SR_OIP, false);
      w25n_unlockblocks(priv);
      w25n_enable_buffer(priv);
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;
}
