#ifndef FFCONF_H
#define FFCONF_H

/* FatFs R0.12c configuration for one fixed, preformatted SD card. */
#define _FFCONF 68300

#define _FS_READONLY 0
#define _FS_MINIMIZE 3
#define _USE_STRFUNC 0
#define _USE_FIND 0
#define _USE_MKFS 0
#define _USE_FASTSEEK 0
#define _USE_EXPAND 0
#define _USE_CHMOD 0
#define _USE_LABEL 0
#define _USE_FORWARD 0

#define _CODE_PAGE 437
#define _USE_LFN 0
#define _MAX_LFN 12
#define _LFN_UNICODE 0
#define _STRF_ENCODE 0
#define _FS_RPATH 0

#define _VOLUMES 1
#define _STR_VOLUME_ID 0
#define _VOLUME_STRS "SD"
#define _MULTI_PARTITION 0
#define _MIN_SS 512
#define _MAX_SS 512
#define _USE_TRIM 0
#define _FS_NOFSINFO 0

#define _FS_TINY 1
#define _FS_EXFAT 0
#define _FS_NORTC 1
#define _NORTC_MON 1
#define _NORTC_MDAY 1
#define _NORTC_YEAR 1980
#define _FS_LOCK 0
#define _FS_REENTRANT 0
#define _USE_MUTEX 0

#endif /* FFCONF_H */
