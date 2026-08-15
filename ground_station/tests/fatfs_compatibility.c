#include "ff.h"
#include "diskio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_SECTOR_SIZE 512U
#define TEST_SECTOR_COUNT 512U

static BYTE storage[TEST_SECTOR_SIZE * TEST_SECTOR_COUNT];
static unsigned long write_count;

#if FF_MULTI_PARTITION
PARTITION VolToPart[FF_VOLUMES] = {{0, 0}};
#endif

DSTATUS disk_status(BYTE pdrv) {
  return pdrv == 0 ? 0 : STA_NODISK;
}

DSTATUS disk_initialize(BYTE pdrv) {
  return pdrv == 0 ? 0 : STA_NODISK;
}

#if FF_DEFINED == 86604
typedef DWORD TestLba;
#else
typedef LBA_t TestLba;
#endif

DRESULT disk_read(BYTE pdrv, BYTE *buffer, TestLba sector, UINT count) {
  if (pdrv != 0 || count == 0 || sector >= TEST_SECTOR_COUNT ||
      count > TEST_SECTOR_COUNT - sector) {
    return RES_PARERR;
  }
  memcpy(buffer, storage + (size_t)sector * TEST_SECTOR_SIZE,
         (size_t)count * TEST_SECTOR_SIZE);
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buffer, TestLba sector, UINT count) {
  if (pdrv != 0 || count == 0 || sector >= TEST_SECTOR_COUNT ||
      count > TEST_SECTOR_COUNT - sector) {
    return RES_PARERR;
  }
  memcpy(storage + (size_t)sector * TEST_SECTOR_SIZE, buffer,
         (size_t)count * TEST_SECTOR_SIZE);
  write_count += count;
  return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE command, void *buffer) {
  if (pdrv != 0) {
    return RES_PARERR;
  }
  switch (command) {
    case CTRL_SYNC:
      return RES_OK;
    case GET_SECTOR_COUNT:
      *(TestLba *)buffer = TEST_SECTOR_COUNT;
      return RES_OK;
    case GET_SECTOR_SIZE:
      *(WORD *)buffer = TEST_SECTOR_SIZE;
      return RES_OK;
    case GET_BLOCK_SIZE:
      *(DWORD *)buffer = 8;
      return RES_OK;
    default:
      return RES_PARERR;
  }
}

static int report_fatfs_error(const char *operation, FRESULT result) {
  fprintf(stderr, "%s failed with FatFs result %d\n", operation, (int)result);
  return 1;
}

static int save_image(const char *path) {
  FILE *output = fopen(path, "wb");
  if (!output) {
    perror("fopen output image");
    return 1;
  }
  const size_t written = fwrite(storage, 1, sizeof(storage), output);
  const int close_result = fclose(output);
  if (written != sizeof(storage) || close_result != 0) {
    fprintf(stderr, "Unable to save complete image\n");
    return 1;
  }
  return 0;
}

static int load_image(const char *path) {
  FILE *input = fopen(path, "rb");
  if (!input) {
    perror("fopen input image");
    return 1;
  }
  const size_t received = fread(storage, 1, sizeof(storage), input);
  const int extra = fgetc(input);
  const int close_result = fclose(input);
  if (received != sizeof(storage) || extra != EOF || close_result != 0) {
    fprintf(stderr, "Unexpected image size\n");
    return 1;
  }
  return 0;
}

static int create_image(const char *path) {
  static BYTE partition_work[TEST_SECTOR_SIZE];
  static BYTE format_work[4096];
  static FATFS filesystem;
  static FIL file;
  static const char config[] = "{\"legacy\":true}\n";
  static const char log[] = "legacy-log-entry\n";
  UINT bytes_written = 0;
  FRESULT result;

  memset(storage, 0xFF, sizeof(storage));
  write_count = 0;

#if FF_DEFINED == 86604
  static const DWORD partitions[] = {100, 0, 0, 0};
  result = f_fdisk(0, partitions, partition_work);
#else
  static const LBA_t partitions[] = {100, 0, 0, 0};
  result = f_fdisk(0, partitions, partition_work);
#endif
  if (result != FR_OK) {
    return report_fatfs_error("f_fdisk", result);
  }

#if FF_DEFINED == 86604
  result = f_mkfs("", FM_FAT | FM_SFD, 0, format_work, sizeof(format_work));
#else
  static const MKFS_PARM options = {FM_FAT | FM_SFD, 0, 0, 0, 0};
  result = f_mkfs("", &options, format_work, sizeof(format_work));
#endif
  if (result != FR_OK) {
    return report_fatfs_error("f_mkfs", result);
  }

  result = f_mount(&filesystem, "0:", 1);
  if (result != FR_OK) {
    return report_fatfs_error("f_mount after format", result);
  }
  result = f_setlabel("DRIVE");
  if (result != FR_OK) {
    return report_fatfs_error("f_setlabel", result);
  }

  result = f_open(&file, "config.json", FA_WRITE | FA_CREATE_ALWAYS);
  if (result != FR_OK) {
    return report_fatfs_error("f_open config", result);
  }
  result = f_write(&file, config, sizeof(config) - 1, &bytes_written);
  if (result != FR_OK || bytes_written != sizeof(config) - 1) {
    return report_fatfs_error("f_write config", result);
  }
  if ((result = f_close(&file)) != FR_OK) {
    return report_fatfs_error("f_close config", result);
  }

  result = f_open(&file, "log.txt", FA_WRITE | FA_CREATE_ALWAYS);
  if (result != FR_OK) {
    return report_fatfs_error("f_open log", result);
  }
  result = f_write(&file, log, sizeof(log) - 1, &bytes_written);
  if (result != FR_OK || bytes_written != sizeof(log) - 1) {
    return report_fatfs_error("f_write log", result);
  }
  if ((result = f_close(&file)) != FR_OK) {
    return report_fatfs_error("f_close log", result);
  }
  if ((result = f_unmount("0:")) != FR_OK) {
    return report_fatfs_error("f_unmount created image", result);
  }
  return save_image(path);
}

static int read_and_check_file(const char *path, const char *expected) {
  FIL file;
  char buffer[64] = {0};
  UINT bytes_read = 0;
  FRESULT result = f_open(&file, path, FA_READ);
  if (result != FR_OK) {
    return report_fatfs_error("f_open existing file", result);
  }
  result = f_read(&file, buffer, sizeof(buffer) - 1, &bytes_read);
  if (result != FR_OK) {
    return report_fatfs_error("f_read existing file", result);
  }
  if ((result = f_close(&file)) != FR_OK) {
    return report_fatfs_error("f_close existing file", result);
  }
  if (bytes_read != strlen(expected) || memcmp(buffer, expected, bytes_read) != 0) {
    fprintf(stderr, "Unexpected contents in %s\n", path);
    return 1;
  }
  return 0;
}

static int validate_image(const char *path) {
  static const char config[] = "{\"legacy\":true}\n";
  static const char log[] = "legacy-log-entry\n";
  static const char appended[] = "r016-write-entry\n";
  FATFS filesystem;
  FIL file;
  char label[16] = {0};
  DWORD serial = 0;
  UINT bytes_written = 0;
  FRESULT result;

  if (load_image(path)) {
    return 1;
  }
  write_count = 0;
  result = f_mount(&filesystem, "0:", 1);
  if (result != FR_OK) {
    return report_fatfs_error("f_mount existing image", result);
  }
  if (write_count != 0) {
    fprintf(stderr, "Mounting existing media unexpectedly wrote sectors\n");
    return 1;
  }
  result = f_getlabel("0:", label, &serial);
  if (result != FR_OK || strcmp(label, "DRIVE") != 0) {
    return report_fatfs_error("f_getlabel existing image", result);
  }

  result = f_setlabel("CATS GS");
  if (result != FR_OK) {
    return report_fatfs_error("f_setlabel existing image", result);
  }
  if (write_count == 0) {
    fprintf(stderr, "Renaming the volume did not write any sectors\n");
    return 1;
  }
  if ((result = f_unmount("0:")) != FR_OK) {
    return report_fatfs_error("f_unmount renamed image", result);
  }
  write_count = 0;
  result = f_mount(&filesystem, "0:", 1);
  if (result != FR_OK) {
    return report_fatfs_error("f_mount renamed image", result);
  }
  memset(label, 0, sizeof(label));
  result = f_getlabel("0:", label, &serial);
  if (result != FR_OK || strcmp(label, "CATS GS") != 0) {
    return report_fatfs_error("f_getlabel renamed image", result);
  }

  if (read_and_check_file("config.json", config) || read_and_check_file("log.txt", log)) {
    return 1;
  }
  if (write_count != 0) {
    fprintf(stderr, "Read-only validation unexpectedly wrote sectors\n");
    return 1;
  }

  result = f_open(&file, "log.txt", FA_WRITE | FA_OPEN_APPEND);
  if (result != FR_OK) {
    return report_fatfs_error("f_open log append", result);
  }
  result = f_write(&file, appended, sizeof(appended) - 1, &bytes_written);
  if (result != FR_OK || bytes_written != sizeof(appended) - 1) {
    return report_fatfs_error("f_write log append", result);
  }
  if ((result = f_close(&file)) != FR_OK) {
    return report_fatfs_error("f_close appended log", result);
  }
  if (write_count == 0) {
    fprintf(stderr, "Appending did not write any sectors\n");
    return 1;
  }
  if ((result = f_unmount("0:")) != FR_OK) {
    return report_fatfs_error("f_unmount validated image", result);
  }

  if (load_image(path)) {
    return 1;
  }
  memset(storage, 0, TEST_SECTOR_SIZE);
  write_count = 0;
  result = f_mount(&filesystem, "0:", 1);
  if (result == FR_OK) {
    fprintf(stderr, "Corrupted media unexpectedly mounted\n");
    return 1;
  }
  if (write_count != 0) {
    fprintf(stderr, "Mounting corrupted media unexpectedly wrote sectors\n");
    return 1;
  }
  return 0;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "usage: fatfs_compatibility create|validate image-path\n");
    return 2;
  }
  if (strcmp(argv[1], "create") == 0) {
    return create_image(argv[2]);
  }
  if (strcmp(argv[1], "validate") == 0) {
    return validate_image(argv[2]);
  }
  fprintf(stderr, "Unknown command: %s\n", argv[1]);
  return 2;
}
