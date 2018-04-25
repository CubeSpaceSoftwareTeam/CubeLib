#ifndef INC_CUBEACP3_H_
#define INC_CUBEACP3_H_

#include <stdbool.h>

/// FileType enumeration
typedef enum {
    BspBinary = 0, ///< Binary File
    BspEvtLog = 1, ///< Event Log File
    BspTlmLog = 2, ///< Telemetry Log File
    BspJpgImg = 3, ///< JPG Image File
    BspBmpImg = 4, ///< BMP Image File
    BspPayload1 = 5, ///< Payload1 File
    BspPayload2 = 6, ///< Payload2 File
    BspPayload3 = 7, ///< Payload3 File
    BspPayload4 = 8, ///< Payload4 File
    BspIndexFile = 15, ///< Index File
} FILES_FileType_t;

/***************************************************************************//**
 * @brief
 *   Files Load File Download Block message structure.
 * @details
 *   Fill download buffer with file contents
 ******************************************************************************/
typedef struct{
    FILES_FileType_t fileType; ///< File Type
    uint8_t counter; ///< Counter
    uint32_t offset; ///< Offset
    uint16_t length; ///< Block Length
} CUBEACP3_LoadFileBlock_t;

/***************************************************************************//**
 * @brief
 *   Files Download Block Ready message structure.
 * @details
 *   Status about download block preparation
 ******************************************************************************/
typedef struct{
    bool ready; ///< Ready
    bool parameterError; ///< The combination of message length and hole map resulted in invalid array lengths
    uint16_t checksum; ///< Block CRC16 Checksum
    uint16_t length; ///< Block length
} FILES_DownloadBlockReady_t;

/***************************************************************************//**
 * @brief
 *   Files Initiate Download Burst message structure.
 * @details
 *   Initiate Download Burst
 ******************************************************************************/
typedef struct{
    uint8_t messageLength; ///< Message Length
    bool ignoreHoleMap; ///< Ignore Hole Map
} FILES_DownloadBurst_t;

/***************************************************************************//**
 * @brief
 *   Files Download Block Ready message structure.
 * @details
 *   Status about download block preparation
 ******************************************************************************/
typedef struct{
    uint8_t messageID; ///< Ready
    uint16_t packetCounter; ///< The combination of message length and hole map resulted in invalid array lengths
    uint8_t fileBytes [20] ; ///< Block CRC16 Checksum
} FILES_DownloadFilePacket_t;

/***************************************************************************//**
 * @brief
 *   Files File Information message structure.
 * @details
 *   File Information
 ******************************************************************************/
typedef struct{
    FILES_FileType_t fileType; ///< File Type
    bool busyUpdating; ///<
    uint8_t fileCtr; ///< File Counter
    uint32_t size; ///< File Size
    uint32_t unixTime; ///< File Data and Time (unix) (measurment unit is [s])
    uint16_t checksum; ///< File CRC16 Checksum
} CUBEACP3_FileInfo_t;

#endif /* INC_CUBEACP3_H_ */
