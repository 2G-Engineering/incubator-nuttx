/****************************************************************************
 * fs/littlefs2/lfs2.h
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *
 * Ported by:
 *
 *   Copyright (C) 2019 Pinecone Inc. All rights reserved.
 *   Author: lihaichen <li8303@163.com>
 *
 * Updated for littleFS v2 by jjlange <jlange@2g-eng.com>
 *
 * This port derives from ARM mbed logic which has a compatible 3-clause
 * BSD license:
 *
 *   Copyright (c) 2017, Arm Limited. All rights reserved.
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
 * 3. Neither the names ARM, NuttX nor the names of its contributors may be
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
 ****************************************************************************/

#ifndef __FS_LITTLEFS2_LFS2_H
#define __FS_LITTLEFS2_LFS2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Version info */

/* Software library version
 * Major (top-nibble), incremented on backwards incompatible changes
 * Minor (bottom-nibble), incremented on feature additions
 */
#define LFS_VERSION 0x00020001
#define LFS_VERSION_MAJOR (0xffff & (LFS_VERSION >> 16))
#define LFS_VERSION_MINOR (0xffff & (LFS_VERSION >>  0))

/* Version of On-disk data structures
 * Major (top-nibble), incremented on backwards incompatible changes
 * Minor (bottom-nibble), incremented on feature additions */
#define LFS_DISK_VERSION 0x00020000
#define LFS_DISK_VERSION_MAJOR (0xffff & (LFS_DISK_VERSION >> 16))
#define LFS_DISK_VERSION_MINOR (0xffff & (LFS_DISK_VERSION >>  0))


/* Maximum name size in bytes, may be redefined to reduce the size of the
 * info struct. Limited to <= 1022. Stored in superblock and must be
 * respected by other littlefs drivers.
 */
#ifndef LFS_NAME_MAX
#  define LFS_NAME_MAX 255
#endif

/* Maximum size of a file in bytes, may be redefined to limit to support other
 * drivers. Limited on disk to <= 4294967296. However, above 2147483647 the
 * functions lfs_file_seek, lfs_file_size, and lfs_file_tell will return
 * incorrect values due to using signed integers. Stored in superblock and
 * must be respected by other littlefs drivers.
 */

#ifndef LFS_FILE_MAX
#  define LFS_FILE_MAX 2147483647
#endif

/* Maximum size of custom attributes in bytes, may be redefined, but there is
 * no real benefit to using a smaller LFS_ATTR_MAX. Limited to <= 1022.
 */

#ifndef LFS_ATTR_MAX
#  define LFS_ATTR_MAX 1022
#endif


/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint32_t lfs_size_t;
typedef uint32_t lfs_off_t;

typedef int32_t  lfs_ssize_t;
typedef int32_t  lfs_soff_t;

typedef uint32_t lfs_block_t;

/* Possible error codes, these are negative to allow
 * valid positive return values
 */

enum lfs_error_e
{
  LFS_ERR_OK          = 0,    /* No error */
  LFS_ERR_IO          = -5,   /* Error during device operation */
  LFS_ERR_CORRUPT     = -84,  /* Corrupted */
  LFS_ERR_NOENT       = -2,   /* No directory entry */
  LFS_ERR_EXIST       = -17,  /* Entry already exists */
  LFS_ERR_NOTDIR      = -20,  /* Entry is not a dir */
  LFS_ERR_ISDIR       = -21,  /* Entry is a dir */
  LFS_ERR_NOTEMPTY    = -39,  /* Dir is not empty */
  LFS_ERR_BADF        = -9,   /* Bad file number */
  LFS_ERR_FBIG        = -27,  /* File too large */
  LFS_ERR_INVAL       = -22,  /* Invalid parameter */
  LFS_ERR_NOSPC       = -28,  /* No space left on device */
  LFS_ERR_NOMEM       = -12,  /* No more memory available */
  LFS_ERR_NOATTR      = -61,  /* No data/attr available */
  LFS_ERR_NAMETOOLONG = -36,  /* File name too long */
};

enum lfs_type_e {

  /* file types */

  LFS_TYPE_REG            = 0x001,
  LFS_TYPE_DIR            = 0x002,

  /* internally used types */

  LFS_TYPE_SPLICE         = 0x400,
  LFS_TYPE_NAME           = 0x000,
  LFS_TYPE_STRUCT         = 0x200,
  LFS_TYPE_USERATTR       = 0x300,
  LFS_TYPE_FROM           = 0x100,
  LFS_TYPE_TAIL           = 0x600,
  LFS_TYPE_GLOBALS        = 0x700,
  LFS_TYPE_CRC            = 0x500,

  /* internally used type specializations */

  LFS_TYPE_CREATE         = 0x401,
  LFS_TYPE_DELETE         = 0x4ff,
  LFS_TYPE_SUPERBLOCK     = 0x0ff,
  LFS_TYPE_DIRSTRUCT      = 0x200,
  LFS_TYPE_CTZSTRUCT      = 0x202,
  LFS_TYPE_INLINESTRUCT   = 0x201,
  LFS_TYPE_SOFTTAIL       = 0x600,
  LFS_TYPE_HARDTAIL       = 0x601,
  LFS_TYPE_MOVESTATE      = 0x7ff,

  /* internal chip sources */

  LFS_FROM_NOOP           = 0x000,
  LFS_FROM_MOVE           = 0x101,
  LFS_FROM_USERATTRS      = 0x102,
};

/* File open flags */

enum lfs_open_flags_e
{
  /* open flags */

  LFS_O_RDONLY = 1,         /* Open a file as read only */
  LFS_O_WRONLY = 2,         /* Open a file as write only */
  LFS_O_RDWR   = 3,         /* Open a file as read and write */
  LFS_O_CREAT  = 0x0100,    /* Create a file if it does not exist */
  LFS_O_EXCL   = 0x0200,    /* Fail if a file already exists */
  LFS_O_TRUNC  = 0x0400,    /* Truncate the existing file to zero size */
  LFS_O_APPEND = 0x0800,    /* Move to end of file on every write */

  /* internally used flags */

  LFS_F_DIRTY   = 0x010000, /* File does not match storage */
  LFS_F_WRITING = 0x020000, /* File has been written since last flush */
  LFS_F_READING = 0x040000, /* File has been read since last flush */
  LFS_F_ERRED   = 0x080000, /* An error occured during write */
  LFS_F_INLINE  = 0x100000, /* Currently inlined in directory entry */
  LFS_F_OPENED  = 0x200000, /* File has been opened */
};

/* File seek flags */

enum lfs_whence_flags_e
{
  LFS_SEEK_SET = 0,   /* Seek relative to an absolute position */
  LFS_SEEK_CUR = 1,   /* Seek relative to the current file position */
  LFS_SEEK_END = 2,   /* Seek relative to the end of the file */
};


/* Configuration provided during initialization of the littlefs */

struct lfs_config_s
{
  /* Opaque user provided context that can be used to pass
   * information to the block device operations
   */

  FAR void *context;

  /* Read a region in a block. Negative error codes are propagated
   * to the user.
   */

  CODE int (*read)(FAR const struct lfs_config_s *c, lfs_block_t block,
                   lfs_off_t off, FAR void *buffer, lfs_size_t size);

  /* Program a region in a block. The block must have previously
   * been erased. Negative error codes are propagated to the user.
   * May return LFS_ERR_CORRUPT if the block should be considered bad.
   */

  CODE int (*prog)(FAR const struct lfs_config_s *c, lfs_block_t block,
                   lfs_off_t off, FAR const void *buffer, lfs_size_t size);

  /* Erase a block. A block must be erased before being programmed.
   * The state of an erased block is undefined. Negative error codes
   * are propagated to the user.
   * May return LFS_ERR_CORRUPT if the block should be considered bad.
   */

  CODE int (*erase)(FAR const struct lfs_config_s *c, lfs_block_t block);

  /* Sync the state of the underlying block device. Negative error codes
   * are propagated to the user.
   */

  CODE int (*sync)(FAR const struct lfs_config_s *c);

  /* Minimum size of a block read. All read operations will be a
   * multiple of this value.
   */

  lfs_size_t read_size;

  /* Minimum size of a block program. All program operations will be a
   * multiple of this value.
   */

  lfs_size_t prog_size;

  /* Size of an erasable block. This does not impact ram consumption and
   * may be larger than the physical erase size. However, non-inlined files
   * take up at minimum one block. Must be a multiple of the read
   * and program sizes.
   */

  lfs_size_t block_size;

  /* Number of erasable blocks on the device. */

  lfs_size_t block_count;

  /* Number of erase cycles before littlefs evicts metadata logs and moves
   * the metadata to another block. Suggested values are in the
   * range 100-1000, with large values having better performance at the cost
   * of less consistent wear distribution.
   *
   * Set to -1 to disable block-level wear-leveling.
   */

  int32_t block_cycles;

  /* Size of block caches. Each cache buffers a portion of a block in RAM.
   * The littlefs needs a read cache, a program cache, and one additional
   * cache per file. Larger caches can improve performance by storing more
   * data and reducing the number of disk accesses. Must be a multiple of
   * the read and program sizes, and a factor of the block size.
   */

  lfs_size_t cache_size;

  /* Size of the lookahead buffer in bytes. A larger lookahead buffer
   * increases the number of blocks found during an allocation pass. The
   * lookahead buffer is stored as a compact bitmap, so each byte of RAM
   * can track 8 blocks. Must be a multiple of 8.
   */

  lfs_size_t lookahead_size;

  /* Optional statically allocated read buffer. Must be cache_size.
   * By default lfs_malloc is used to allocate this buffer.
   */

  FAR void *read_buffer;

  /* Optional statically allocated program buffer. Must be cache_size.
   * By default lfs_malloc is used to allocate this buffer.
   */

  FAR void *prog_buffer;

  /* Optional statically allocated lookahead buffer. Must be lookahead_size
   * and aligned to a 32-bit boundary. By default lfs_malloc is used to
   * allocate this buffer.
   */

  FAR void *lookahead_buffer;

  /* Optional upper limit on length of file names in bytes. No downside for
   * larger names except the size of the info struct which is controlled by
   * the LFS_NAME_MAX define. Defaults to LFS_NAME_MAX when zero. Stored in
   * superblock and must be respected by other littlefs drivers.
   */

  lfs_size_t name_max;

  /* Optional upper limit on files in bytes. No downside for larger files
   * but must be <= LFS_FILE_MAX. Defaults to LFS_FILE_MAX when zero. Stored
   * in superblock and must be respected by other littlefs drivers.
   */

  lfs_size_t file_max;

  /* Optional upper limit on custom attributes in bytes. No downside for
   * larger attributes size but must be <= LFS_ATTR_MAX. Defaults to
   * LFS_ATTR_MAX when zero.
   */

  lfs_size_t attr_max;
};

/* File info structure */

struct lfs_info_s
{
  /* Type of the file, either LFS_TYPE_REG or LFS_TYPE_DIR */

  uint8_t type;

  /* Size of the file, only valid for REG files. Limited to 32-bits. */

  lfs_size_t size;

  /* Name of the file stored as a null-terminated string. Limited to
   * LFS_NAME_MAX+1, which can be changed by redefining LFS_NAME_MAX to
   * reduce RAM. LFS_NAME_MAX is stored in superblock and must be
   * respected by other littlefs drivers.
   */

  char name[LFS_NAME_MAX+1];
};

/* Custom attribute structure, used to describe custom attributes
 * committed atomically during file writes.
 */

struct lfs_attr_s
{
  /* 8-bit type of attribute, provided by user and used to
   * identify the attribute
   */

  uint8_t type;

  /* Pointer to buffer containing the attribute */

  FAR void *buffer;

  /* Size of attribute in bytes, limited to LFS_ATTR_MAX */

  lfs_size_t size;
};

/* Optional configuration provided during lfs_file_opencfg */

struct lfs_file_config_s
{
  /* Optional statically allocated file buffer. Must be cache_size.
   * By default lfs_malloc is used to allocate this buffer.
   */

  FAR void *buffer;

  /* Optional list of custom attributes related to the file. If the file
   * is opened with read access, these attributes will be read from disk
   * during the open call. If the file is opened with write access, the
   * attributes will be written to disk every file sync or close. This
   * write occurs atomically with update to the file's contents.
   *
   * Custom attributes are uniquely identified by an 8-bit type and limited
   * to LFS_ATTR_MAX bytes. When read, if the stored attribute is smaller
   * than the buffer, it will be padded with zeros. If the stored attribute
   * is larger, then it will be silently truncated. If the attribute is not
   * found, it will be created implicitly.
   */

  FAR struct lfs_attr_s *attrs;

  /* Number of custom attributes in the list */

  lfs_size_t attr_count;
};


/* internal littlefs data structures */

typedef struct lfs_cache_s
{
  lfs_block_t block;
  lfs_off_t off;
  lfs_size_t size;
  FAR uint8_t *buffer;
} lfs_cache_t;

typedef struct lfs_mdir_s
{
  lfs_block_t pair[2];
  uint32_t rev;
  lfs_off_t off;
  uint32_t etag;
  uint16_t count;
  bool erased;
  bool split;
  lfs_block_t tail[2];
} lfs_mdir_t;

/* littlefs directory type */

typedef struct lfs_dir_s
{
  FAR struct lfs_dir_s *next;
  uint16_t id;
  uint8_t type;
  lfs_mdir_t m;

  lfs_off_t pos;
  lfs_block_t head[2];
} lfs_dir_t;

/* littlefs file type */

typedef struct lfs_file_s
{
  FAR struct lfs_file_s *next;
  uint16_t id;
  uint8_t type;
  lfs_mdir_t m;

  struct lfs_ctz_s
  {
    lfs_block_t head;
    lfs_size_t size;
  } ctz;

  uint32_t flags;
  lfs_off_t pos;
  lfs_block_t block;
  lfs_off_t off;
  lfs_cache_t cache;

  FAR const struct lfs_file_config_s *cfg;
} lfs_file_t;

typedef struct lfs_superblock_s
{
  uint32_t version;
  lfs_size_t block_size;
  lfs_size_t block_count;
  lfs_size_t name_max;
  lfs_size_t file_max;
  lfs_size_t attr_max;
} lfs_superblock_t;

// The littlefs filesystem type
typedef struct lfs_s
{
  lfs_cache_t rcache;
  lfs_cache_t pcache;

  lfs_block_t root[2];
  FAR struct lfs_mlist {
    FAR struct lfs_mlist *next;
    uint16_t id;
    uint8_t type;
    lfs_mdir_t m;
  } *mlist;
  uint32_t seed;

  struct lfs_gstate_s
  {
    uint32_t tag;
    lfs_block_t pair[2];
  } gstate, gpending, gdelta;

  struct lfs_free_s
  {
    lfs_block_t off;
    lfs_block_t size;
    lfs_block_t i;
    lfs_block_t ack;
    FAR uint32_t *buffer;
  } free;

  FAR const struct lfs_config_s *cfg;
  lfs_size_t name_max;
  lfs_size_t file_max;
  lfs_size_t attr_max;

#ifdef CONFIG_LITTLEFS2_ALLOW_MIGRATION
  FAR struct lfs1 *lfs1;
#endif
} lfs_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Filesystem functions */

/* Format a block device with the littlefs
 *
 * Requires a littlefs object and config struct. This clobbers the littlefs
 * object, and does not leave the filesystem mounted. The config struct must
 * be zeroed for defaults and backwards compatibility.
 *
 * Returns a negative error code on failure.
 */

int lfs_format(FAR lfs_t *lfs, FAR const struct lfs_config_s *config);

/* Mounts a littlefs
 *
 * Requires a littlefs object and config struct. Multiple filesystems
 * may be mounted simultaneously with multiple littlefs objects. Both
 * lfs and config must be allocated while mounted. The config struct must
 * be zeroed for defaults and backwards compatibility.
 *
 * Returns a negative error code on failure.
 */

int lfs_mount(FAR lfs_t *lfs, FAR const struct lfs_config_s *config);

/* Unmounts a littlefs
 *
 * Does nothing besides releasing any allocated resources.
 * Returns a negative error code on failure.
 */

int lfs_unmount(FAR lfs_t *lfs);

/* General operations */

/* Removes a file or directory
 *
 * If removing a directory, the directory must be empty.
 * Returns a negative error code on failure.
 */

int lfs_remove(FAR lfs_t *lfs, FAR const char *path);

/* Rename or move a file or directory
 *
 * If the destination exists, it must match the source in type.
 * If the destination is a directory, the directory must be empty.
 *
 * Returns a negative error code on failure.
 */

int lfs_rename(FAR lfs_t *lfs, FAR const char *oldpath, FAR const char *newpath);

/* Find info about a file or directory
 *
 * Fills out the info structure, based on the specified file or directory.
 * Returns a negative error code on failure.
 */

int lfs_stat(FAR lfs_t *lfs, FAR const char *path, FAR struct lfs_info_s *info);

/* Get a custom attribute
 *
 * Custom attributes are uniquely identified by an 8-bit type and limited
 * to LFS_ATTR_MAX bytes. When read, if the stored attribute is smaller than
 * the buffer, it will be padded with zeros. If the stored attribute is larger,
 * then it will be silently truncated. If no attribute is found, the error
 * LFS_ERR_NOATTR is returned and the buffer is filled with zeros.
 *
 * Returns the size of the attribute, or a negative error code on failure.
 * Note, the returned size is the size of the attribute on disk, irrespective
 * of the size of the buffer. This can be used to dynamically allocate a buffer
 * or check for existance.
 */

lfs_ssize_t lfs_getattr(FAR lfs_t *lfs, FAR const char *path,
                        uint8_t type, FAR void *buffer, lfs_size_t size);

/* Set custom attributes
 *
 * Custom attributes are uniquely identified by an 8-bit type and limited
 * to LFS_ATTR_MAX bytes. If an attribute is not found, it will be
 * implicitly created.
 *
 * Returns a negative error code on failure.
 */

int lfs_setattr(FAR lfs_t *lfs, FAR const char *path,
                uint8_t type, FAR const void *buffer, lfs_size_t size);

/* Removes a custom attribute
 *
 * If an attribute is not found, nothing happens.
 *
 * Returns a negative error code on failure.
 */

int lfs_removeattr(FAR lfs_t *lfs, FAR const char *path, uint8_t type);

/* File operations */

/* Open a file
 *
 * The mode that the file is opened in is determined by the flags, which
 * are values from the enum lfs_open_flags that are bitwise-ored together.
 *
 * Returns a negative error code on failure.
 */

int lfs_file_open(FAR lfs_t *lfs, FAR lfs_file_t *file,
                  FAR const char *path, int flags);

/* Open a file with extra configuration
 *
 * The mode that the file is opened in is determined by the flags, which
 * are values from the enum lfs_open_flags that are bitwise-ored together.
 *
 * The config struct provides additional config options per file as described
 * above. The config struct must be allocated while the file is open, and the
 * config struct must be zeroed for defaults and backwards compatibility.
 *
 * Returns a negative error code on failure.
 */

int lfs_file_opencfg(FAR lfs_t *lfs, FAR lfs_file_t *file,
                     FAR const char *path, int flags,
                     FAR const struct lfs_file_config_s *config);

/* Close a file
 *
 * Any pending writes are written out to storage as though
 * sync had been called and releases any allocated resources.
 *
 * Returns a negative error code on failure.
 */

int lfs_file_close(FAR lfs_t *lfs, FAR lfs_file_t *file);

/* Synchronize a file on storage
 *
 * Any pending writes are written out to storage.
 * Returns a negative error code on failure.
 */

int lfs_file_sync(FAR lfs_t *lfs, FAR lfs_file_t *file);

/* Read data from file
 *
 * Takes a buffer and size indicating where to store the read data.
 * Returns the number of bytes read, or a negative error code on failure.
 */

lfs_ssize_t lfs_file_read(FAR lfs_t *lfs, FAR lfs_file_t *file,
                          FAR void *buffer, lfs_size_t size);

/* Write data to file
 *
 * Takes a buffer and size indicating the data to write. The file will not
 * actually be updated on the storage until either sync or close is called.
 *
 * Returns the number of bytes written, or a negative error code on failure.
 */

lfs_ssize_t lfs_file_write(FAR lfs_t *lfs, FAR lfs_file_t *file,
                           FAR const void *buffer, lfs_size_t size);

/* Change the position of the file
 *
 * The change in position is determined by the offset and whence flag.
 * Returns the old position of the file, or a negative error code on failure.
 */

lfs_soff_t lfs_file_seek(FAR lfs_t *lfs, FAR lfs_file_t *file,
                         lfs_soff_t off, int whence);

/* Truncates the size of the file to the specified size
 *
 * Returns a negative error code on failure.
 */

int lfs_file_truncate(FAR lfs_t *lfs, FAR lfs_file_t *file, lfs_off_t size);

/* Return the position of the file
 *
 * Equivalent to lfs_file_seek(lfs, file, 0, LFS_SEEK_CUR)
 * Returns the position of the file, or a negative error code on failure.
 */

lfs_soff_t lfs_file_tell(FAR lfs_t *lfs, FAR lfs_file_t *file);

/* Change the position of the file to the beginning of the file
 *
 * Equivalent to lfs_file_seek(lfs, file, 0, LFS_SEEK_CUR)
 * Returns a negative error code on failure.
 */

int lfs_file_rewind(FAR lfs_t *lfs, FAR lfs_file_t *file);

/* Return the size of the file
 *
 * Similar to lfs_file_seek(lfs, file, 0, LFS_SEEK_END)
 * Returns the size of the file, or a negative error code on failure.
 */

lfs_soff_t lfs_file_size(FAR lfs_t *lfs, FAR lfs_file_t *file);

/* Directory operations */

/* Create a directory
 *
 * Returns a negative error code on failure.
 */

int lfs_mkdir(FAR lfs_t *lfs, FAR const char *path);

/* Open a directory
 *
 * Once open a directory can be used with read to iterate over files.
 * Returns a negative error code on failure.
 */

int lfs_dir_open(FAR lfs_t *lfs, FAR lfs_dir_t *dir, FAR const char *path);

/* Close a directory
 *
 * Releases any allocated resources.
 * Returns a negative error code on failure.
 */

int lfs_dir_close(FAR lfs_t *lfs, FAR lfs_dir_t *dir);

/* Read an entry in the directory
 *
 * Fills out the info structure, based on the specified file or directory.
 * Returns a negative error code on failure.
 */

int lfs_dir_read(FAR lfs_t *lfs, FAR lfs_dir_t *dir,
                 FAR struct lfs_info_s *info);

/* Change the position of the directory
 *
 * The new off must be a value previous returned from tell and specifies
 * an absolute offset in the directory seek.
 *
 * Returns a negative error code on failure.
 */

int lfs_dir_seek(FAR lfs_t *lfs, FAR lfs_dir_t *dir, lfs_off_t off);

/* Return the position of the directory
 *
 * The returned offset is only meant to be consumed by seek and may not make
 * sense, but does indicate the current position in the directory iteration.
 *
 * Returns the position of the directory, or a negative error code on failure.
 */

lfs_soff_t lfs_dir_tell(FAR lfs_t *lfs, FAR lfs_dir_t *dir);

/* Change the position of the directory to the beginning of the directory
 *
 * Returns a negative error code on failure.
 */

int lfs_dir_rewind(FAR lfs_t *lfs, FAR lfs_dir_t *dir);

/* Filesystem-level filesystem operations */

/* Finds the current size of the filesystem
 *
 * Note: Result is best effort. If files share COW structures, the returned
 * size may be larger than the filesystem actually is.
 *
 * Returns the number of allocated blocks, or a negative error code on failure.
 */

lfs_ssize_t lfs_fs_size(FAR lfs_t *lfs);

/* Traverse through all blocks in use by the filesystem
 *
 * The provided callback will be called with each block address that is
 * currently in use by the filesystem. This can be used to determine which
 * blocks are in use or how much of the storage is available.
 *
 * Returns a negative error code on failure.
 */
int lfs_fs_traverse(FAR lfs_t *lfs, CODE int (*cb)(FAR void *, lfs_block_t),
                 FAR void *data);

#ifdef CONFIG_LITTLEFS2_ALLOW_MIGRATION
/* Attempts to migrate a previous version of littlefs
 *
 * Behaves similarly to the lfs_format function. Attempts to mount
 * the previous version of littlefs and update the filesystem so it can be
 * mounted with the current version of littlefs.
 *
 * Requires a littlefs object and config struct. This clobbers the littlefs
 * object, and does not leave the filesystem mounted. The config struct must
 * be zeroed for defaults and backwards compatibility.
 *
 * Returns a negative error code on failure.
 */

int lfs_migrate(FAR lfs_t *lfs, FAR const struct lfs_config_s *cfg);
#endif


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
