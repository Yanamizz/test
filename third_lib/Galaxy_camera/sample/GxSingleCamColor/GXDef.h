/**
@File      GXDef.h
@Brief     the define for the GxIAPI dll module.
@Author    Software Department
@Date      2024-12-09
@Version   1.0.2412.9091
*/

#ifndef GX_DEF_H
#define GX_DEF_H

#include "GXErrorList.h"

//////////////////////////////////////////////////////////////////////////
// Chinese??
// ??????��?????????????C??????stdint.h???��??��????????????????
//			VS2010????��?��????????????,???????????????

// English:	The following types are defined in the standard C library header file stdint.h, but are available on
// Microsoft's compilation platform 			This file was not included in previous versions of VS2010, so it
// needs to be
// redefined here
//////////////////////////////////////////////////////////////////////////

#if defined(_WIN32)
#ifndef _STDINT_H
#ifdef _MSC_VER  // Microsoft compiler
#if _MSC_VER < 1600
typedef __int8 int8_t;
typedef __int16 int16_t;
typedef __int32 int32_t;
typedef __int64 int64_t;
typedef unsigned __int8 uint8_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
#else
// In Visual Studio 2010 is stdint.h already included
#include <stdint.h>
#endif
#else
// Not a Microsoft compiler
#include <stdint.h>
#endif
#endif
#else
// Linux
#include <stdint.h>
#endif

//------------------------------------------------------------------------------
// Chinese?? ????????????
// English:	Operating system platform definition
//------------------------------------------------------------------------------

#include <stddef.h>

#ifdef WIN32
#ifndef _WIN32
#define _WIN32
#endif
#endif

#ifdef _WIN32
#include <Windows.h>
#define GX_DLLIMPORT __declspec(dllimport)
#define GX_DLLEXPORT __declspec(dllexport)

#define GX_STDC __stdcall
#define GX_CDEC __cdecl

#if defined(__cplusplus)
#define GX_EXTC extern "C"
#else
#define GX_EXTC
#endif
#else
// remove the None #define conflicting with GenApi
#undef None
#if __GNUC__ >= 4
#define GX_DLLIMPORT __attribute__((visibility("default")))
#define GX_DLLEXPORT __attribute__((visibility("default")))

// On non-Windows platforms, avoid stdcall/cdecl attributes to prevent GCC warnings
#define GX_STDC
#define GX_CDEC

#if defined(__cplusplus)
#define GX_EXTC extern "C"
#else
#define GX_EXTC
#endif
#else
#error Unknown compiler
#endif
#endif

#ifdef GX_GALAXY_DLL
#define GX_DLLENTRY GX_EXTC GX_DLLEXPORT
#else
#define GX_DLLENTRY GX_EXTC GX_DLLIMPORT
#endif

#ifdef _WIN32
#define GX_API GX_EXTC GX_STATUS GX_STDC
#else
#define GX_API GX_EXTC GX_STATUS GX_STDC GX_DLLEXPORT
#endif

#define GX_INFO_LENGTH_8_BYTE (8)      ///< \Chinese 8???				\English 8byte
#define GX_INFO_LENGTH_32_BYTE (32)    ///< \Chinese 32???				\English 32byte
#define GX_INFO_LENGTH_64_BYTE (64)    ///< \Chinese 64???				\English 64byte
#define GX_INFO_LENGTH_128_BYTE (128)  ///< \Chinese 128???			\English 128byte
#define GX_INFO_LENGTH_256_BYTE (256)  ///< \Chinese 256???			\English 256byte

typedef void* GX_DEV_HANDLE;  ///< \Chinese ?��????????GXOpenDevice???????????????��???????
                              ///< \English The device handle, which is obtained through gxopendevice, is controlled and
                              ///< collected by this handle
typedef void* GX_IF_HANDLE;   ///< \Chinese Interface????????GXGetInterfaceHandle????????????????IF????��????
                              ///< \English The interface handle, obtained by gxgetinterfacehandle, makes read and write
                              ///< control on the if layer through this handle
typedef void*
    GX_LOCAL_DEV_HANDLE;  ///< \Chinese ?????��?????????GXGetLocalDeviceHandle???????????????��????��????��????
                          ///< \English The local device layer handle, obtained by gxgetlocaldevicehandle,
                          ///< USES this handle to make read and write control of the local device layer
typedef void* GX_DS_HANDLE;  ///< \Chinese DataStream????????GXGetDataStreamHandle?????????????????????��????
                             ///< \English The datastream handle is obtained by gxgetdatastreamhandle, and the flow
                             ///< layer reads and writes control through this handle
typedef void*
    GX_PORT_HANDLE;  ///< \Chinese
                     ///< ?????(????��?��???????????GX_IF_HANDLE??GX_DEV_HANDLE??GX_DEV_LOCAL_HANDLE??GX_DS_HANDLE)
                     ///< \English Common handle (GX_IF_HANDLE, GX_DEV_HANDLE, GX_DEV_LOCAL_HANDLE, and
                     ///< GX_DS_HANDLE for read/write control)
typedef void* GX_EVENT_CALLBACK_HANDLE;  ///< \Chinese ?��???????????????��????????????????????��??????????
                                         ///< \English Device event callback handle registers device-related event
                                         ///< callback functions, such as the device drop callback function
typedef void* GX_FEATURE_CALLBACK_HANDLE;  ///< \Chinese ?��??????????????????��????????????????????
                                           ///< \English Secondary property update callback handle, obtained when
                                           ///< registering the device property update callback function
typedef void* GX_FEATURE_CALLBACK_BY_STRING_HANDLE;  ///< \Chinese ?��??????????????????��????????????????????
                                                     ///< \English Device property update callback handle, obtained when
                                                     ///< registering the device property update callback function

typedef uint64_t void_64;

typedef enum GX_TL_TYPE_LIST {
  GX_TL_TYPE_UNKNOWN = 0,  ///< \Chinese ��??????��				\English Unknown type device
  GX_TL_TYPE_USB = 1,      ///< \Chinese USB2.0					\English USB2.0
  GX_TL_TYPE_GEV = 2,      ///< \Chinese GEV						\English GEV
  GX_TL_TYPE_U3V = 4,      ///< \Chinese U3V						\English U3V
  GX_TL_TYPE_CXP = 8,      ///< \Chinese CXP						\English CXP
} GX_TL_TYPE_LIST;
typedef int32_t GX_TL_TYPE;

typedef enum GX_ACCESS_MODE {
  GX_ACCESS_READONLY =
      2,  ///< \Chinese ??????					\English Open the device in read-only mode
  GX_ACCESS_CONTROL =
      3,  ///< \Chinese ??????					\English Open the device in controlled mode
  GX_ACCESS_EXCLUSIVE =
      4,  ///< \Chinese ??????					\English Open the device in exclusive mode
} GX_ACCESS_MODE;
typedef int32_t GX_ACCESS_MODE_CMD;

typedef enum GX_FRAME_STATUS_LIST {
  GX_FRAME_STATUS_SUCCESS = 0,  ///< \Chinese ?????						\English Normal frame
  GX_FRAME_STATUS_INCOMPLETE =
      -1,  ///< \Chinese ???						\English Incomplete frame
} GX_FRAME_STATUS_LIST;
typedef int32_t GX_FRAME_STATUS;

//------------------------------------------------------------------------------
// Chinese	?��?????
// English	Open mode of device
//------------------------------------------------------------------------------
typedef enum GX_OPEN_MODE {
  GX_OPEN_SN =
      0,  ///< \Chinese ???SN??					\English Opens the device via a serial number
  GX_OPEN_IP =
      1,               ///< \Chinese ???IP??					\English Opens the device via an IP address
  GX_OPEN_MAC = 2,     ///< \Chinese ???MAC??				\English Opens the device via a MAC address
  GX_OPEN_INDEX = 3,   ///< \Chinese ???Index??				\English Opens the device via a serial number
                       ///< (Start from 1, such as 1, 2, 3, 4...)
  GX_OPEN_USERID = 4,  ///< \Chinese ???????????ID??		\English Opens the device via user defined ID
} GX_OPEN_MODE;
typedef int32_t GX_OPEN_MODE_CMD;

//------------------------------------------------------------------------------
// Chinese	IP???��??
// English	IP configuration
//------------------------------------------------------------------------------
typedef enum GX_IP_CONFIGURE_MODE_LIST {
  GX_IP_CONFIGURE_DHCP = 0x6,  ///< \Chinese ????DHCP????????IP???	\English Enable the DHCP mode to allocate the IP
                               ///< address by the DHCP server
  GX_IP_CONFIGURE_LLA =
      0x4,  ///< \Chinese ???LLA???????IP???		\English Enable the LLA mode to allocate the IP address
  GX_IP_CONFIGURE_STATIC_IP = 0x5,  ///< \Chinese ??????IP???				\English Enable the static IP
                                    ///< mode to configure the IP address
  GX_IP_CONFIGURE_DEFAULT =
      0x7,  ///< \Chinese ????????????IP???		\English Enable the default mode to configure the IP address
};
typedef int32_t GX_IP_CONFIGURE_MODE;

typedef enum GX_NODE_ACCESS_MODE {
  GX_NODE_ACCESS_MODE_NI = 0,    ///< \Chinese ????				\English Not come true
  GX_NODE_ACCESS_MODE_NA = 1,    ///< \Chinese ?????��			\English Not read-write
  GX_NODE_ACCESS_MODE_WO = 2,    ///< \Chinese ?��				\English Write only
  GX_NODE_ACCESS_MODE_RO = 3,    ///< \Chinese ???				\English Read only
  GX_NODE_ACCESS_MODE_RW = 4,    ///< \Chinese ???��				\English read-write
  GX_NODE_ACCESS_MODE_UNDEF = 5  ///< \Chinese ��????				\English Undefined
} GX_NODE_ACCESS_MODE;

/*
\Chinese ?????��??
\English Reset Device Mode
*/
typedef enum GX_RESET_DEVICE_MODE {
  GX_MANUFACTURER_SPECIFIC_RECONNECT =
      0x1,                              ///< \Chinese ?????��				\English reconnect Device
  GX_MANUFACTURER_SPECIFIC_RESET = 0x2  ///< \Chinese ?????��				\English reset Device
} GX_RESET_DEVICE_MODE;

typedef enum GX_LOG_TYPE_LIST {
  GX_LOG_TYPE_OFF = 0x00000000,    ///< \Chinese ???????????????		\English Do not generate any type of log
  GX_LOG_TYPE_FATAL = 0x00000001,  ///< \Chinese ????FATAL???????		\English Generate FATAL type logs
  GX_LOG_TYPE_ERROR = 0x00000010,  ///< \Chinese ????ERROR???????		\English Generate ERROR type logs
  GX_LOG_TYPE_WARN = 0x00000100,   ///< \Chinese ????WARN???????		\English Generate WARN type logs
  GX_LOG_TYPE_INFO = 0x00001000,   ///< \Chinese ????INFO???????		\English Generate INFO type logs
  GX_LOG_TYPE_DEBUG = 0x00010000,  ///< \Chinese ????DEBUG???????		\English Generate DEBUG type logs
  GX_LOG_TYPE_TRACE = 0x00100000,  ///< \Chinese ????TRACE???????		\English Generate TRACE type logs
} GX_LOG_TYPE_LIST;
typedef uint32_t GX_LOG_TYPE;

typedef struct GX_OPEN_PARAM {
  char* pszContent;           ///< \Chinese ???????????,??????????????		\English Standard C string that is decided by
                              ///< openMode. It could be an IP address, a camera serial number
  GX_OPEN_MODE_CMD openMode;  ///< \Chinese ????						\English Device open
                              ///< mode. The device can be open via the SN, IP, MAC, etc. Please refer to GX_OPEN_MODE
  GX_ACCESS_MODE_CMD accessMode;  ///< \Chinese ??????						\English Device access
                                  ///< mode, such as read-only, control, exclusive, etc. Please refer to GX_ACCESS_MODE
} GX_OPEN_PARAM;

#ifdef _WIN32
typedef struct GX_FRAME_CALLBACK_PARAM {
  void* pUserParam;  ///< \Chinese ??????????
                     ///< \English User's private data pointer
  GX_FRAME_STATUS
  status;               ///< \Chinese ?????????
                        ///< \English The image state returned by the callback function. Please refer to GX_FRAME_STATUS
  const void* pImgBuf;  ///< \Chinese ???buffer?????????chunkdata??pImgBuf ???????????????????? ??
                        ///< \English The image data address (After the frame information is enabled, the pImgBuf
                        ///< contains image data and frame information data)
  int32_t nImgSize;  ///< \Chinese ????��?????��????��????????chunkdata??nImgsize?????????��+??????��??	\English Data
                     ///< size, in bytes (After the frame information is enabled, nImgSize is the sum of the size of the
                     ///< image data and the size of the frame information)
  int32_t nWidth;        ///< \Chinese ?????
                         ///< \English Image width
  int32_t nHeight;       ///< \Chinese ?????
                         ///< \English Image height
  int32_t nPixelFormat;  ///< \Chinese ????PixFormat
                         ///< \English PixelFormat of image
  uint64_t nFrameID;     ///< \Chinese ???????
                         ///< \English Frame identification of image
  uint64_t nTimestamp;   ///< \Chinese ????????
                         ///< \English Timestamp of image
  int32_t reserved[1];   ///< \Chinese ????
                         ///< \English 4 bytes,reserved
} GX_FRAME_CALLBACK_PARAM;

typedef struct GX_FRAME_DATA {
  GX_FRAME_STATUS nStatus;  ///< \Chinese ?????????
                            ///< \English The state of the acquired image. Please refer to GX_FRAME_STATUS
  void* pImgBuf;            ///< \Chinese ???buffer?????????chunkdata??pImgBuf ???????????????????? ??			\English
                  ///< The image data address (After the frame information is enabled, the pImgBuf contains image data
                  ///< and frame information data)
  int32_t nWidth;        ///< \Chinese ?????
                         ///< \English Image width
  int32_t nHeight;       ///< \Chinese ?????
                         ///< \English Image height
  int32_t nPixelFormat;  ///< \Chinese ????PixFormat
                         ///< \English Pixel format of image
  int32_t nImgSize;  ///< \Chinese ????��?????��????��????????chunkdata??nImgsize?????????��+??????��??	\English Data
                     ///< size (After the frame information is enabled, nImgSize is the sum of the size of the image
                     ///< data and the size of the frame information)
  uint64_t nFrameID;    ///< \Chinese ???????
                        ///< \English Frame identification of image
  uint64_t nTimestamp;  ///< \Chinese ????????
                        ///< \English Timestamp of image
  void* pUserParam;     ///< \Chinese ???????
                        ///< \English User Param
  int32_t reserved[2];  ///< \Chinese ????
                        ///< \English 4 bytes,reserved
} GX_FRAME_DATA;

typedef struct GX_FRAME_BUFFER {
  uint64_t nFrameID;    ///< \Chinese ???ID
                        ///< \English Frame identification of image
  uint64_t nTimestamp;  ///< \Chinese ??????
                        ///< \English Timestamp of image
  uint64_t nBufID;      ///< \Chinese ??bufID \English BufID
  void_64 pImgBuf;      ///< \Chinese ?????????
                    ///< \English The image data pointer (After the frame information is enabled, the pImgBuf contains
                    ///< image data and frame information data)
  GX_FRAME_STATUS nStatus;  ///< \Chinese ????????
                            ///< \English The state of the acquired image. Please refer to GX_FRAME_STATUS
  int32_t nWidth;           ///< \Chinese ??????
                            ///< \English Image width
  int32_t nHeight;          ///< \Chinese ?????
                            ///< \English Image height
  int32_t nPixelFormat;     ///< \Chinese ??????
                            ///< \English Pixel format of image
  int32_t nImgSize;         ///< \Chinese ???????��
                     ///< \English Data size, in bytes (After the frame information is enabled, nImgSize is the sum of
                     ///< the size of the image data and the size of the frame information)
  int32_t nOffsetX;      ///< \Chinese ???????
                         ///< \English X-direction offset of the image
  int32_t nOffsetY;      ///< \Chinese ????????
                         ///< \English Y-direction offset of the image
  void* pUserParam;      ///< \Chinese ???????
                         ///< \English User Param
  int32_t reserved[23];  ///< \Chinese ????
                         ///< \English reserved
} GX_FRAME_BUFFER;
#else
typedef struct GX_FRAME_CALLBACK_PARAM {
  void* pUserParam;  ///< \Chinese ??????????
                     ///< \English User's private data pointer
  GX_FRAME_STATUS
  status;               ///< \Chinese ?????????
                        ///< \English The image state returned by the callback function. Please refer to GX_FRAME_STATUS
  const void* pImgBuf;  ///< \Chinese ???buffer?????????chunkdata??pImgBuf ???????????????????? ??
                        ///< \English The image data address (After the frame information is enabled, the pImgBuf
                        ///< contains image data and frame information data)
  int32_t nImgSize;  ///< \Chinese ????��?????��????��????????chunkdata??nImgsize?????????��+??????��??		\English
                     ///< Data size, in bytes (After the frame information is enabled, nImgSize is the sum of the size
                     ///< of the image data and the size of the frame information)
  int32_t nWidth;        ///< \Chinese ?????
                         ///< \English Image width
  int32_t nHeight;       ///< \Chinese ?????
                         ///< \English Image height
  int32_t nPixelFormat;  ///< \Chinese ????PixFormat
                         ///< \English PixelFormat of image
  uint64_t nFrameID;     ///< \Chinese ???????
                         ///< \English Frame identification of image
  uint64_t nTimestamp;   ///< \Chinese ????????
                         ///< \English Timestamp of image
  int32_t nOffsetX;  ///< \Chinese ?????????                                                                  \English
                     ///< X-direction offset of the image
  int32_t nOffsetY;  ///< \Chinese ??????????                                                                  \English
                     ///< Y-direction offset of the image
  int32_t reserved[1];  ///< \Chinese ???? \English 4 bytes,reserved
} GX_FRAME_CALLBACK_PARAM;

typedef struct GX_FRAME_DATA {
  GX_FRAME_STATUS nStatus;  ///< \Chinese ?????????
                            ///< \English The state of the acquired image. Please refer to GX_FRAME_STATUS
  void* pImgBuf;            ///< \Chinese ???buffer?????????chunkdata??pImgBuf ???????????????????? ??
                  ///< \English The image data address (After the frame information is enabled, the pImgBuf contains
                  ///< image data and frame information data)
  int32_t nWidth;        ///< \Chinese ?????
                         ///< \English Image width
  int32_t nHeight;       ///< \Chinese ?????
                         ///< \English Image height
  int32_t nPixelFormat;  ///< \Chinese ????PixFormat
                         ///< \English Pixel format of image
  int32_t nImgSize;  ///< \Chinese ????��?????��????��????????chunkdata??nImgsize?????????��+??????��??		\English
                     ///< Data size (After the frame information is enabled, nImgSize is the sum of the size of the
                     ///< image data and the size of the frame information)
  uint64_t nFrameID;    ///< \Chinese ???????
                        ///< \English Frame identification of image
  uint64_t nTimestamp;  ///< \Chinese ????????
                        ///< \English Timestamp of image
  int32_t nOffsetX;  ///< \Chinese ?????????                                                                   \English
                     ///< X-direction offset of the image
  int32_t nOffsetY;  ///< \Chinese ??????????                                                                   \English
                     ///< Y-direction offset of the image
  int32_t reserved[1];  ///< \Chinese ???? \English 4 bytes,reserved
} GX_FRAME_DATA;

typedef struct GX_FRAME_BUFFER {
  GX_FRAME_STATUS nStatus;  ///< \Chinese ????????
                            ///< \English The state of the acquired image. Please refer to GX_FRAME_STATUS
  void* pImgBuf;            ///< \Chinese ?????????
                  ///< \English The image data pointer (After the frame information is enabled, the pImgBuf contains
                  ///< image data and frame information data)
  int32_t nWidth;        ///< \Chinese ?????? \English Image width
  int32_t nHeight;       ///< \Chinese ????? \English Image height
  int32_t nPixelFormat;  ///< \Chinese ??????
                         ///< \English Pixel format of image
  int32_t nImgSize;      ///< \Chinese ???????��
                     ///< \English Data size, in bytes (After the frame information is enabled, nImgSize is the sum of
                     ///< the size of the image data and the size of the frame information)
  uint64_t nFrameID;     ///< \Chinese ???ID
                         ///< \English Frame identification of image
  uint64_t nTimestamp;   ///< \Chinese ??????
                         ///< \English Timestamp of image
  uint64_t nBufID;       ///< \Chinese ??bufID \English BufID
  int32_t nOffsetX;      ///< \Chinese ???????
                         ///< \English X-direction offset of the image
  int32_t nOffsetY;      ///< \Chinese ????????
                         ///< \English Y-direction offset of the image
  void* pUserParam;      ///< \Chinese ???????
                         ///< \English User Param
  int32_t reserved[14];  ///< \Chinese ???? \English 64 bytes,reserved
} GX_FRAME_BUFFER;
#endif

typedef GX_FRAME_BUFFER* PGX_FRAME_BUFFER;

typedef struct GX_CXP_INTERFACE_INFO {
  unsigned char chInterfaceID[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ?????ID
                                                         ///< \English CXP card ID
  unsigned char chDisplayName[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ???????
                                                         ///< \English Display name
  unsigned char chSerialNumber[GX_INFO_LENGTH_64_BYTE];  ///< \Chinese ???��?
                                                         ///< \English Serial number
  unsigned int ui32InitFlag;                             ///< \Chinese ???????
                                                         ///< \English Initialization state
  unsigned int nReserved[65];                            ///< \Chinese ???
                                                         ///< \English reserve
} GX_CXP_INTERFACE_INFO;

typedef struct GX_GEV_INTERFACE_INFO {
  unsigned char chInterfaceID[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ?????ID
                                                         ///< \English GEV card ID
  unsigned char chDisplayName[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ???????
                                                         ///< \English Display name
  unsigned char chSerialNumber[GX_INFO_LENGTH_64_BYTE];  ///< \Chinese ???��?
                                                         ///< \English Serial number
  char szDescription[GX_INFO_LENGTH_256_BYTE];           ///< \Chinese ??????
                                                         ///< \English Card description
  unsigned int ui32InitFlag;                             ///< \Chinese ???????
                                                         ///< \English Initialization state
  unsigned int nReserved[63];                            ///< \Chinese ???
                                                         ///< \English reserve
} GX_GEV_INTERFACE_INFO;

typedef struct GX_U3V_INTERFACE_INFO {
  unsigned char chInterfaceID[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ?????ID
                                                         ///< \English U3 card ID
  unsigned char chDisplayName[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ???????
                                                         ///< \English Display name
  unsigned char chSerialNumber[GX_INFO_LENGTH_64_BYTE];  ///< \Chinese ???��?
                                                         ///< \English Serial number
  char szDescription[GX_INFO_LENGTH_256_BYTE];           ///< \Chinese ??????
                                                         ///< \English Card description
  unsigned int nReserved[64];                            ///< \Chinese ???
                                                         ///< \English reserve
} GX_U3V_INTERFACE_INFO;

typedef struct GX_USB_INTERFACE_INFO {
  unsigned char chInterfaceID[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ?????ID
                                                         ///< \English USB card ID
  unsigned char chDisplayName[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ???????
                                                         ///< \English Display name
  unsigned char chSerialNumber[GX_INFO_LENGTH_64_BYTE];  ///< \Chinese ???��?
                                                         ///< \English Serial number
  char szDescription[GX_INFO_LENGTH_256_BYTE];           ///< \Chinese ??????
                                                         ///< \English Card description
  unsigned int nReserved[64];                            ///< \Chinese ???
                                                         ///< \English reserve
} GX_USB_INTERFACE_INFO;

typedef struct GX_INTERFACE_INFO {
  GX_TL_TYPE emTLayerType;    ///< \Chinese ??????
                              ///< \English Type of card
  unsigned int nReserved[4];  ///< \Chinese ??????? \English reserve
  union {
    GX_CXP_INTERFACE_INFO stCXPIFInfo;  ///< \Chinese CXP????????
                                        ///< \English CXP card Information
    GX_GEV_INTERFACE_INFO stGEVIFInfo;  ///< \Chinese GEV????????
                                        ///< \English GEV card Information
    GX_U3V_INTERFACE_INFO stU3VIFInfo;  ///< \Chinese U3V????????
                                        ///< \English U3V card Information
    GX_USB_INTERFACE_INFO stUSBIFInfo;  ///< \Chinese USB????????
                                        ///< \English USB card Information
    unsigned int nReserved[64];         ///< \Chinese ??????? \English reserve
  } IFInfo;
} GX_INTERFACE_INFO;

//------------------------------------------------------------------------------
// Chinese	?��????????
// English	Device type code definition
//------------------------------------------------------------------------------
typedef enum GX_DEVICE_CLASS_LIST {
  GX_DEVICE_CLASS_UNKNOWN =
      0,                     ///< \Chinese ��??��????								\English Unknown device
  GX_DEVICE_CLASS_USB2 = 1,  ///< \Chinese USB2.0?��
                             ///< \English USB2.0 device
  GX_DEVICE_CLASS_GEV = 2,   ///< \Chinese ??????��
                             ///< \English GEV device
  GX_DEVICE_CLASS_U3V = 3,   ///< \Chinese USB3.0?��
                             ///< \English USB3.0 device
  GX_DEVICE_CLASS_SMART =
      4,  ///< \Chinese Smart camera device						\English Smart camera device
  GX_DEVICE_CLASS_CXP =
      5,  ///< \Chinese CXP?��									\English CXP device
} GX_DEVICE_CLASS_LIST;
typedef int32_t GX_DEVICE_CLASS;

typedef struct GX_CXP_DEVICE_INFO {
  unsigned char chVendorName[GX_INFO_LENGTH_64_BYTE];        ///< \Chinese ?????????
                                                             ///< \English Supplier name
  unsigned char chModelName[GX_INFO_LENGTH_64_BYTE];         ///< \Chinese ???????
                                                             ///< \English Model name
  unsigned char chManufacturerInfo[GX_INFO_LENGTH_64_BYTE];  ///< \Chinese ???????
                                                             ///< \English Vendor information
  unsigned char chDeviceVersion[GX_INFO_LENGTH_64_BYTE];     ///< \Chinese ?��?��
                                                             ///< \English Device version
  unsigned char chSerialNumber[GX_INFO_LENGTH_64_BYTE];      ///< \Chinese ???��?
                                                             ///< \English Serial number
  unsigned char chUserDefinedName[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ????????????
                                                             ///< \English User-defined name
  unsigned int nReserved[64];                                ///< \Chinese ??????? \English reserve
} GX_CXP_DEVICE_INFO;

typedef struct GX_GEV_DEVICE_INFO {
  unsigned int nCurrentIp;                                   ///< \Chinese ???IP???
                                                             ///< \English Current IP configuration
  unsigned int nCurrentSubNetMask;                           ///< \Chinese ???????????
                                                             ///< \English Current subnet mask
  unsigned int nDefaultGateWay;                              ///< \Chinese ???????
                                                             ///< \English Current gateway
  unsigned int nNetExport;                                   ///< \Chinese ????IP???
                                                             ///< \English IP address
  uint64_t nMacAddress;                                      ///< \Chinese MAC???									\English
                                                             ///< MAC address
  unsigned char chVendorName[GX_INFO_LENGTH_64_BYTE];        ///< \Chinese ?????????
                                                             ///< \English Supplier name
  unsigned char chModelName[GX_INFO_LENGTH_64_BYTE];         ///< \Chinese ???????
                                                             ///< \English Model name
  unsigned char chManufacturerInfo[GX_INFO_LENGTH_64_BYTE];  ///< \Chinese ???????
                                                             ///< \English Vendor information
  unsigned char chDeviceVersion[GX_INFO_LENGTH_64_BYTE];     ///< \Chinese ?��?��
                                                             ///< \English Device version
  unsigned char chSerialNumber[GX_INFO_LENGTH_64_BYTE];      ///< \Chinese ???��?
                                                             ///< \English Serial number
  unsigned char chUserDefinedName[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ????????????
                                                             ///< \English User-defined name
  unsigned int nReserved[64];                                ///< \Chinese ??????? \English reserve

} GX_GEV_DEVICE_INFO;

typedef struct GX_U3V_DEVICE_INFO {
  unsigned char chVendorName[GX_INFO_LENGTH_64_BYTE];        ///< \Chinese ?????????
                                                             ///< \English Supplier name
  unsigned char chModelName[GX_INFO_LENGTH_64_BYTE];         ///< \Chinese ???????
                                                             ///< \English Model name
  unsigned char chManufacturerInfo[GX_INFO_LENGTH_64_BYTE];  ///< \Chinese ???????
                                                             ///< \English Vendor information
  unsigned char chDeviceVersion[GX_INFO_LENGTH_64_BYTE];     ///< \Chinese ?��?��
                                                             ///< \English Device version
  unsigned char chSerialNumber[GX_INFO_LENGTH_64_BYTE];      ///< \Chinese ???��?
                                                             ///< \English Serial number
  unsigned char chUserDefinedName[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ????????????
                                                             ///< \English User-defined name
  unsigned int nReserved[64];                                ///< \Chinese ??????? \English reserve
} GX_U3V_DEVICE_INFO;

typedef struct GX_USB_DEVICE_INFO {
  unsigned char chVendorName[GX_INFO_LENGTH_64_BYTE];        ///< \Chinese ?????????
                                                             ///< \English Supplier name
  unsigned char chModelName[GX_INFO_LENGTH_64_BYTE];         ///< \Chinese ???????
                                                             ///< \English Model name
  unsigned char chManufacturerInfo[GX_INFO_LENGTH_64_BYTE];  ///< \Chinese ???????
                                                             ///< \English Vendor information
  unsigned char chDeviceVersion[GX_INFO_LENGTH_64_BYTE];     ///< \Chinese ?��?��
                                                             ///< \English Device version
  unsigned char chSerialNumber[GX_INFO_LENGTH_64_BYTE];      ///< \Chinese ???��?
                                                             ///< \English Serial number
  unsigned char chUserDefinedName[GX_INFO_LENGTH_64_BYTE];   ///< \Chinese ????????????
                                                             ///< \English User-defined name
  unsigned int nReserved[64];                                ///< \Chinese ??????? \English reserve
} GX_USB_DEVICE_INFO;

typedef struct GX_DEVICE_INFO {
  GX_DEVICE_CLASS emDevType;  ///< \Chinese ?��????
                              ///< \English device class
  unsigned int nReserved[4];  ///< \Chinese ??????? \English reserve
  union {
    GX_CXP_DEVICE_INFO stCXPDevInfo;  ///< \Chinese CXP?��???
                                      ///< \English CXP device information
    GX_GEV_DEVICE_INFO stGEVDevInfo;  ///< \Chinese GEV?��???
                                      ///< \English GEV device information
    GX_U3V_DEVICE_INFO stU3VDevInfo;  ///< \Chinese U3V?��???
                                      ///< \English U3V device information
    GX_USB_DEVICE_INFO stUSBDevInfo;  ///< \Chinese USB?��???
                                      ///< \English USB device information
    unsigned int nReserved[256];      ///< \Chinese ??????? \English reserve
  } DevInfo;
} GX_DEVICE_INFO;

typedef struct GX_INT_VALUE {
  int64_t nCurValue;  ///< \Chinese ?????????								\English The
                      ///< integer value is the current value
  int64_t nMin;       ///< \Chinese ???????��?								\English The
                 ///< integer value is the minimum
  int64_t nMax;  ///< \Chinese ?????????								\English The
                 ///< integer value is the maximum
  int64_t nInc;  ///< \Chinese ?????????									\English
                 ///< The integer value is the step
  int32_t reserved[16];  ///< \Chinese ????????? \English reserve
} GX_INT_VALUE;

typedef struct GX_FLOAT_VALUE {
  double dCurValue;  ///< \Chinese ?????????								\English The
                     ///< float value is the current value
  double dMin;       ///< \Chinese ????????��?								\English The
                ///< float value is the minimum
  double dMax;  ///< \Chinese ??????????								\English The
                ///< float value is the maximum
  double dInc;                         ///< \Chinese ?????????									\English The
                                       ///< float value is the step
  bool bIncIsValid;                    ///< \Chinese ??????????????��							\English If a
                                       ///< floating-point step is valid
  char szUnit[GX_INFO_LENGTH_8_BYTE];  ///< \Chinese ???????��
                                       ///< \English Floating point unit
  int32_t reserved[16];                ///< \Chinese ???? \English reserve
} GX_FLOAT_VALUE;

typedef struct GX_ENUM_VALUE_DES {
  int64_t nCurValue;                             ///< \Chinese ????????								\English
                                                 ///< Enumeration current values
  char strCurSymbolic[GX_INFO_LENGTH_128_BYTE];  ///< \Chinese ?????????????????
                                                 ///< \English The symbol name of the current value of the enumeration
                                                 ///< type
  int32_t reserved[4];                           ///< \Chinese ???? \English reserve
} GX_ENUM_VALUE_DES;

typedef struct GX_ENUM_VALUE {
  GX_ENUM_VALUE_DES
      stCurValue;          ///< \Chinese ????????                               \English Enumeration current values
  uint32_t nSupportedNum;  ///< \Chinese ????????????????							\English
                           ///< Enumeration supported subterm number
  GX_ENUM_VALUE_DES nArrySupportedValue[128];  ///< \Chinese ???????????????
                                               ///< \English The value of the subitems supported by the enumeration type
  int32_t reserved[16];                        ///< \Chinese ???? \English reserve
} GX_ENUM_VALUE;

typedef struct GX_STRING_VALUE {
  char strCurValue[GX_INFO_LENGTH_256_BYTE];  ///< \Chinese ????????????
                                              ///< \English The current value of the string type
  int64_t nMaxLength;                         ///< \Chinese ?????????????							\English Maximum
                                              ///< length of string type
  int32_t reserved[4];                        ///< \Chinese ???? \English reserve
} GX_STRING_VALUE;

typedef struct GX_REGISTER_STACK_ENTRY {
  uint64_t nAddress;  ///> \Chinese ????????									\English
                      /// Address of the register
  void* pBuffer;      ///> \Chinese ?????????								\English Pointer
                  ///to the buffer containing the data
  size_t nSize;  ///> \Chinese ??????????								\English Number
                 /// of bytes to read
} GX_REGISTER_STACK_ENTRY;

typedef struct GX_GIGE_ACTION_COMMAND_RESULT {
  char strDeviceAddress[16];  ///< \Chinese ?��IP	12 + 3 + 1 \English IP address of the device

  ///< \Chinese ???��?????ACTION??????????????
  ///< 1. 0: ?????????????
  ///< 2. 0x8013: ?����??????????????????????��?????????????��?PtpEnable????
  ///<           ???????? ??PtpStatus?????Master???????Slave?????????????????????????
  ///< 3. 0x8015: ?��???��?????????????????strDeviceAddress??????��????????????
  ///<           GXGigEIssueScheduledActionCommand????????????????GXGigEIssueScheduledActionCommand???????????
  ///< 4. 0x8016: GXGigEIssueScheduledActionCommand??????Scheduled Action Command ??????
  ///< \English GigE Vision status code returned by the device, There are several situations:
  ///< 1. 0: Indicates that the command was sent successfully.
  ///< 2. 0x8013: The device is not synchronized with the master clock.
  ///<            Before executing this interface, you must enable "PtpEnable" and ensure
  ///<           that "PtpStatus" is "Master" or "Slave" (indicating that it has been synchronized with the master
  ///<           clock).
  ///< 3. 0x8015: The device queue or packet data has overflowed. When the device corresponding
  ///<            to strDeviceAddress is executing the previous GXGigEIssueScheduledActionCommand request,
  ///<            it receives a new GXGigEIssueScheduledActionCommand request again and returns this error.
  ///< 4. 0x8016: The Scheduled Action Command issued by GXGigEIssueScheduledActionCommand is outdated.
  int32_t nStatus;
} GX_GIGE_ACTION_COMMAND_RESULT;
//------------------------------------------------------------------------------
// Chinese   ??????????????
// English   The callback is defined by the number type
//------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
/**
\Chinese??
\brief     ??????????????
\param     pFrameData    ????????????
\return    void

\English:
\brief     Capture callback function type
\param     pFrameData    The pointer to the address that the user introduced to receive the image data
\return    void
*/
//----------------------------------------------------------------------------------
typedef void(GX_STDC* GXCaptureCallBack)(GX_FRAME_CALLBACK_PARAM* pFrameData);
//----------------------------------------------------------------------------------
/**
\Chinese??
\brief     ??????????????
\param     pUserParam    ?????��????????????????????????????
\return    void

\English:
\brief     Device offline callback function type
\param     pUserParam    User private parameter
\return    void
*/
//----------------------------------------------------------------------------------
typedef void(GX_STDC* GXDeviceOfflineCallBack)(void* pUserParam);
//----------------------------------------------------------------------------------
/**
\Chinese??
\brief     ??????????????????
\param     strFeatureName   ?????????????????????????????????????????????
\param     pUserParam       ?????��???????????????????????????????????
\return    void

\English:
\brief     The basic attribute callback is defined
\param     strFeatureName   The property name string is identical to the value passed in when the basic property is
returned to the specified number \param     pUserParam       The user's private parameter is identical to the value
passed in when the basic property is returned to the function number \return    void
*/
//----------------------------------------------------------------------------------
typedef void(GX_STDC* GXFeatureCallBackByString)(const char* strFeatureName, void* pUserParam);

//----------------------------------------------------------------------------------
/**
\Chinese??
\brief     ??????????????????
\param     nFeatureID    ???????ID????????????????????????????????
\param     pUserParam    ?????��???????????????????????????????????
\return    void

\English:
\brief     Device attribute update callback function type
\param     nFeatureID    FeatureID
\param     pUserParam    User private parameter
\return    void
*/
//----------------------------------------------------------------------------------
typedef int32_t GX_FEATURE_ID_CMD;
typedef void(GX_STDC* GXFeatureCallBack)(GX_FEATURE_ID_CMD nFeatureID, void* pUserParam);

#endif  // GX_DEF_H