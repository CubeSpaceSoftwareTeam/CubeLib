/************************************************************************************
 * This file was auto-generated by CIDEA                           .                *
 * Please do not modify the contents of this file manually.                         *
 ***********************************************************************************/


#include "cubeboot.h"

uint16_t BOOTLOADER_createTelemetryRequest(uint8_t* txBuffer, BOOTLOADER_Telemetry_t tlm)
{
  txBuffer[0] = tlm;

    switch(tlm)
    {
        case BootLoader_BlockChecksum:
            return 2;
        case BootLoader_BootIndexStatus:
            return 2;
        case BootLoader_Cache:
            return 1;
        case BootLoader_CommsStatus:
            return 6;
        case BootLoader_CopyToIntFlashProgress:
            return 1;
        case BootLoader_DownloadBlockReady:
            return 5;
        case BootLoader_EdacErrors:
            return 6;
        case BootLoader_ExtendedIdentification:
            return 6;
        case BootLoader_FileDownload:
            return 22;
        case BootLoader_FileInfo:
            return 12;
        case BootLoader_HoleMap1:
            return 16;
        case BootLoader_HoleMap2:
            return 16;
        case BootLoader_HoleMap3:
            return 16;
        case BootLoader_HoleMap4:
            return 16;
        case BootLoader_HoleMap5:
            return 16;
        case BootLoader_HoleMap6:
            return 16;
        case BootLoader_HoleMap7:
            return 16;
        case BootLoader_HoleMap8:
            return 16;
        case BootLoader_Identification:
            return 8;
        case BootLoader_InitializeUploadComplete:
            return 1;
        case BootLoader_LastLogEvent:
            return 6;
        case BootLoader_LatchupErrors:
            return 6;
        case BootLoader_ProgramInfo:
            return 8;
        case BootLoader_SramScrubSettings:
            return 2;
        case BootLoader_State:
            return 6;
        case BootLoader_TelecommandAcknowledge:
            return 4;
        case BootLoader_UnixTime:
            return 6;
        case BootLoader_UnixTimeSave:
            return 2;
        case BootLoader_UploadBlockComplete:
            return 1;
        default:
            return -1;
    }
}
CUBELIB_Result_t CUBEBOOT_BlockChecksumTlm(uint8_t* rxBuffer, uint16_t* checksum)
{
    if (checksum == 0)
        return PointerIsNull;

    *checksum = *( (uint16_t*) (rxBuffer + 0) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_BootIndexStatusTlm(uint8_t* rxBuffer, CUBEBOOT_BootIndexStatus_t* returnVal)
{
    uint8_t enumVal;

    if (returnVal == 0)
        return PointerIsNull;

    enumVal = rxBuffer[0];
    returnVal->programIndex = (BOOTLOADER_BootProgramsList_t) enumVal;

    enumVal = rxBuffer[1];
    returnVal->bootStatus = (BOOTLOADER_BootStatus_t) enumVal;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_CacheTlm(uint8_t* rxBuffer, bool* enabled)
{
    if (enabled == 0)
        return PointerIsNull;

    *enabled = (rxBuffer[0] & 0x01) >> 0;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_CommsStatusTlm(uint8_t* rxBuffer, CUBEBOOT_CommsStatus_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->tcCounter = *( (uint16_t*) (rxBuffer + 0) );

    returnVal->tlmCounter = *( (uint16_t*) (rxBuffer + 2) );

    returnVal->uartBufferOverrun = (rxBuffer[4] & 0x01) >> 0;

    returnVal->uartProtocolError = (rxBuffer[4] & 0x02) >> 1;

    returnVal->uartMsgIncomplete = (rxBuffer[4] & 0x04) >> 2;

    returnVal->i2CTelemetryError = (rxBuffer[4] & 0x08) >> 3;

    returnVal->i2CBufferError = (rxBuffer[4] & 0x10) >> 4;

    returnVal->cANBufferError = (rxBuffer[4] & 0x20) >> 5;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_CopyToIntFlashProgressTlm(uint8_t* rxBuffer, CUBEBOOT_CopyToIntFlashProgress_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->busy = (rxBuffer[0] & 0x01) >> 0;

    returnVal->error = (rxBuffer[0] & 0x02) >> 1;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_DownloadBlockReadyTlm(uint8_t* rxBuffer, CUBEBOOT_DownloadBlockReady_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->ready = (rxBuffer[0] & 0x01) >> 0;

    returnVal->parameterError = (rxBuffer[0] & 0x02) >> 1;

    returnVal->checksum = *( (uint16_t*) (rxBuffer + 1) );

    returnVal->length = *( (uint16_t*) (rxBuffer + 3) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_EdacErrorsTlm(uint8_t* rxBuffer, CUBEBOOT_EdacErrors_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->singleSRAMSEUs = *( (uint16_t*) (rxBuffer + 0) );

    returnVal->doubleSRAMSEUs = *( (uint16_t*) (rxBuffer + 2) );

    returnVal->multiSRAMSEUs = *( (uint16_t*) (rxBuffer + 4) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_ExtendedIdentificationTlm(uint8_t* rxBuffer, CUBEBOOT_ExtendedIdentification_t* returnVal)
{
    uint8_t enumVal;

    if (returnVal == 0)
        return PointerIsNull;

    enumVal = (rxBuffer[0] >> 0) & 15;
    returnVal->mcuResetCause = (BOOTLOADER_ResetCause_t) enumVal;

    enumVal = (rxBuffer[0] >> 4) & 15;
    returnVal->bootCause = (BOOTLOADER_BootCause_t) enumVal;

    returnVal->bootCounter = *( (uint16_t*) (rxBuffer + 1) );

    enumVal = rxBuffer[3];
    returnVal->runningProgramIndex = (BOOTLOADER_BootProgramsList_t) enumVal;

    returnVal->firmwareMajorVersion = *( (uint8_t*) (rxBuffer + 4) );

    returnVal->firmwareMinorVersion = *( (uint8_t*) (rxBuffer + 5) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_FileDownloadTlm(uint8_t* rxBuffer, CUBEBOOT_FileDownload_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->packetNo = *( (uint16_t*) (rxBuffer + 0) );

    memcpy(returnVal->fileBytes, rxBuffer + 2, 20);

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_FileInfoTlm(uint8_t* rxBuffer, CUBEBOOT_FileInfo_t* returnVal)
{
    uint8_t enumVal;

    if (returnVal == 0)
        return PointerIsNull;

    enumVal = (rxBuffer[0] >> 0) & 15;
    returnVal->fileType = (BOOTLOADER_FileType_t) enumVal;

    returnVal->busyUpdating = (rxBuffer[0] & 0x10) >> 4;

    returnVal->fileCtr = *( (uint8_t*) (rxBuffer + 1) );

    returnVal->size = *( (uint32_t*) (rxBuffer + 2) );

    returnVal->unixTime = *( (uint32_t*) (rxBuffer + 6) );

    returnVal->checksum = *( (uint16_t*) (rxBuffer + 10) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_HoleMap1Tlm(uint8_t* rxBuffer, uint8_t* holeMap)
{
    if (holeMap == 0)
        return PointerIsNull;

    memcpy(holeMap, rxBuffer + 0, 16);

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_HoleMap2Tlm(uint8_t* rxBuffer, uint8_t* holeMap)
{
    if (holeMap == 0)
        return PointerIsNull;

    memcpy(holeMap, rxBuffer + 0, 16);

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_HoleMap3Tlm(uint8_t* rxBuffer, uint8_t* holeMap)
{
    if (holeMap == 0)
        return PointerIsNull;

    memcpy(holeMap, rxBuffer + 0, 16);

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_HoleMap4Tlm(uint8_t* rxBuffer, uint8_t* holeMap)
{
    if (holeMap == 0)
        return PointerIsNull;

    memcpy(holeMap, rxBuffer + 0, 16);

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_HoleMap5Tlm(uint8_t* rxBuffer, uint8_t* holeMap)
{
    if (holeMap == 0)
        return PointerIsNull;

    memcpy(holeMap, rxBuffer + 0, 16);

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_HoleMap6Tlm(uint8_t* rxBuffer, uint8_t* holeMap)
{
    if (holeMap == 0)
        return PointerIsNull;

    memcpy(holeMap, rxBuffer + 0, 16);

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_HoleMap7Tlm(uint8_t* rxBuffer, uint8_t* holeMap)
{
    if (holeMap == 0)
        return PointerIsNull;

    memcpy(holeMap, rxBuffer + 0, 16);

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_HoleMap8Tlm(uint8_t* rxBuffer, uint8_t* holeMap)
{
    if (holeMap == 0)
        return PointerIsNull;

    memcpy(holeMap, rxBuffer + 0, 16);

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_IdentificationTlm(uint8_t* rxBuffer, CUBEBOOT_Identification_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->nodeType = *( (uint8_t*) (rxBuffer + 0) );

    returnVal->interfaceVersion = *( (uint8_t*) (rxBuffer + 1) );

    returnVal->firmwareMajorVersion = *( (uint8_t*) (rxBuffer + 2) );

    returnVal->firmwareMinorVersion = *( (uint8_t*) (rxBuffer + 3) );

    returnVal->runtimeSeconds = *( (uint16_t*) (rxBuffer + 4) );

    returnVal->runtimeMilliseconds = *( (uint16_t*) (rxBuffer + 6) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_InitializeUploadCompleteTlm(uint8_t* rxBuffer, bool* busy)
{
    if (busy == 0)
        return PointerIsNull;

    *busy = (rxBuffer[0] & 0x01) >> 0;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_LastLogEventTlm(uint8_t* rxBuffer, CUBEBOOT_LastLogEvent_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->unixTime = *( (uint32_t*) (rxBuffer + 0) );

    returnVal->eventId = *( (uint8_t*) (rxBuffer + 4) );

    returnVal->eventParam = *( (uint8_t*) (rxBuffer + 5) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_LatchupErrorsTlm(uint8_t* rxBuffer, CUBEBOOT_LatchupErrors_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->sRAM1SELs = *( (uint16_t*) (rxBuffer + 0) );

    returnVal->sRAM2SELs = *( (uint16_t*) (rxBuffer + 2) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_ProgramInfoTlm(uint8_t* rxBuffer, CUBEBOOT_ProgramInfo_t* returnVal)
{
    uint8_t enumVal;

    if (returnVal == 0)
        return PointerIsNull;

    enumVal = rxBuffer[0];
    returnVal->programIndex = (BOOTLOADER_ProgramsList_t) enumVal;

    returnVal->busy = (rxBuffer[1] & 0x01) >> 0;

    returnVal->fileSize = *( (uint32_t*) (rxBuffer + 2) );

    returnVal->crc16 = *( (uint16_t*) (rxBuffer + 6) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_SramScrubSettingsTlm(uint8_t* rxBuffer, uint16_t* scrubSize)
{
    if (scrubSize == 0)
        return PointerIsNull;

    *scrubSize = *( (uint16_t*) (rxBuffer + 0) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_StateTlm(uint8_t* rxBuffer, CUBEBOOT_State_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->runtime = *( (uint16_t*) (rxBuffer + 0) );

    returnVal->sram1Enabled = (rxBuffer[2] & 0x01) >> 0;

    returnVal->sram2Enabled = (rxBuffer[2] & 0x02) >> 1;

    returnVal->sramLatchup = (rxBuffer[2] & 0x04) >> 2;

    returnVal->sramLatchupRecovered = (rxBuffer[2] & 0x08) >> 3;

    returnVal->sDError = (rxBuffer[2] & 0x10) >> 4;

    returnVal->extFlashError = (rxBuffer[2] & 0x20) >> 5;

    returnVal->intFlashError = (rxBuffer[2] & 0x40) >> 6;

    returnVal->eepromError = (rxBuffer[2] & 0x80) >> 7;

    returnVal->bootRegCorrupt = (rxBuffer[3] & 0x01) >> 0;

    returnVal->radioCommsError = (rxBuffer[3] & 0x02) >> 1;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_TelecommandAcknowledgeTlm(uint8_t* rxBuffer, CUBEBOOT_TelecommandAcknowledge_t* returnVal)
{
    uint8_t enumVal;

    if (returnVal == 0)
        return PointerIsNull;

    returnVal->lastTCID = *( (uint8_t*) (rxBuffer + 0) );

    returnVal->processedFlag = (rxBuffer[1] & 0x01) >> 0;

    enumVal = rxBuffer[2];
    returnVal->tCerrorStatus  = (BOOTLOADER_TcErrorReason_t) enumVal;

    returnVal->tCParameterErrorIndex = *( (uint8_t*) (rxBuffer + 3) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_UnixTimeTlm(uint8_t* rxBuffer, CUBEBOOT_UnixTime_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->time = *( (uint32_t*) (rxBuffer + 0) );

    returnVal->milliSec = *( (uint16_t*) (rxBuffer + 4) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_UnixTimeSaveTlm(uint8_t* rxBuffer, CUBEBOOT_UnixTimeSave_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->saveNow = (rxBuffer[0] & 0x01) >> 0;

    returnVal->saveOnUpdate = (rxBuffer[0] & 0x02) >> 1;

    returnVal->savePeriodic = (rxBuffer[0] & 0x04) >> 2;

    returnVal->period = *( (uint8_t*) (rxBuffer + 1) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEBOOT_UploadBlockCompleteTlm(uint8_t* rxBuffer, CUBEBOOT_UploadBlockComplete_t* returnVal)
{
    if (returnVal == 0)
        return PointerIsNull;

    returnVal->busy = (rxBuffer[0] & 0x01) >> 0;

    returnVal->error = (rxBuffer[0] & 0x02) >> 1;

    return CubeLibOk;
}


uint16_t CUBEBOOT_AdvanceFileListPtrCmd(uint8_t* tcBuffer)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 113;


    return 1;
}

uint16_t CUBEBOOT_AdvanceLogPtrCmd(uint8_t* tcBuffer)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 5;


    return 1;
}

uint16_t CUBEBOOT_BootIndexCmd(uint8_t* tcBuffer, BOOTLOADER_BootSetProgramsList_t programIndex)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 100;

    tcBuffer[1] = 
        (programIndex << 0);

    return 2;
}

uint16_t CUBEBOOT_CacheCmd(uint8_t* tcBuffer, bool enabled)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 3;

    tcBuffer[1] = 
        (enabled << 0);

    return 2;
}

uint16_t CUBEBOOT_ClearErrorsCmd(uint8_t* tcBuffer)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 7;


    return 1;
}

uint16_t CUBEBOOT_CopyToInternalFlashCmd(uint8_t* tcBuffer, CUBEBOOT_CopyToInternalFlash_t* setVal)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 103;

    tcBuffer[1] = 
        (setVal->srcIndex << 0);
    *( (uint8_t*)(tcBuffer + 2) ) = setVal->bootloaderOverwrite;

    return 3;
}

uint16_t CUBEBOOT_DownloadBurstCmd(uint8_t* tcBuffer, CUBEBOOT_DownloadBurst_t* setVal)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 119;

    tcBuffer[2] = 
        (setVal->ignoreHoleMap << 0);
    *( (uint8_t*)(tcBuffer + 1) ) = setVal->messageLength;

    return 3;
}

uint16_t CUBEBOOT_EraseFileCmd(uint8_t* tcBuffer, CUBEBOOT_EraseFile_t* setVal)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 108;

    tcBuffer[1] = 
        (setVal->fileType << 0);
    tcBuffer[3] = 
        (setVal->eraseAll << 0);
    *( (uint8_t*)(tcBuffer + 2) ) = setVal->fileCtr;

    return 4;
}

uint16_t CUBEBOOT_FileUploadCmd(uint8_t* tcBuffer, CUBEBOOT_FileUpload_t* setVal)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 115;

    *( (uint16_t*)(tcBuffer + 1) ) = setVal->packetNo;
    memcpy(tcBuffer + 3,setVal->fileBytes,20);

    return 23;
}

uint16_t CUBEBOOT_FinalizeUploadBlockCmd(uint8_t* tcBuffer, CUBEBOOT_FinalizeUploadBlock_t* setVal)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 116;

    tcBuffer[1] = 
        (setVal->destination << 0);
    *( (uint32_t*)(tcBuffer + 2) ) = setVal->offset;
    *( (uint16_t*)(tcBuffer + 6) ) = setVal->blockLen;

    return 8;
}

uint16_t CUBEBOOT_HoleMap1Cmd(uint8_t* tcBuffer, uint8_t* holeMap)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 120;

    memcpy(tcBuffer + 1,holeMap,16);

    return 17;
}

uint16_t CUBEBOOT_HoleMap2Cmd(uint8_t* tcBuffer, uint8_t* holeMap)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 121;

    memcpy(tcBuffer + 1,holeMap,16);

    return 17;
}

uint16_t CUBEBOOT_HoleMap3Cmd(uint8_t* tcBuffer, uint8_t* holeMap)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 122;

    memcpy(tcBuffer + 1,holeMap,16);

    return 17;
}

uint16_t CUBEBOOT_HoleMap4Cmd(uint8_t* tcBuffer, uint8_t* holeMap)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 123;

    memcpy(tcBuffer + 1,holeMap,16);

    return 17;
}

uint16_t CUBEBOOT_HoleMap5Cmd(uint8_t* tcBuffer, uint8_t* holeMap)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 124;

    memcpy(tcBuffer + 1,holeMap,16);

    return 17;
}

uint16_t CUBEBOOT_HoleMap6Cmd(uint8_t* tcBuffer, uint8_t* holeMap)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 125;

    memcpy(tcBuffer + 1,holeMap,16);

    return 17;
}

uint16_t CUBEBOOT_HoleMap7Cmd(uint8_t* tcBuffer, uint8_t* holeMap)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 126;

    memcpy(tcBuffer + 1,holeMap,16);

    return 17;
}

uint16_t CUBEBOOT_HoleMap8Cmd(uint8_t* tcBuffer, uint8_t* holeMap)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 127;

    memcpy(tcBuffer + 1,holeMap,16);

    return 17;
}

uint16_t CUBEBOOT_InitiateFileUploadCmd(uint8_t* tcBuffer, CUBEBOOT_InitiateFileUpload_t* setVal)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 114;

    tcBuffer[1] = 
        (setVal->destination << 0);
    *( (uint8_t*)(tcBuffer + 2) ) = setVal->blockSize;

    return 3;
}

uint16_t CUBEBOOT_LoadDownloadBlockCmd(uint8_t* tcBuffer, BOOTLOADER_FileType_t fileType, uint8_t counter, uint32_t offset, uint16_t length)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 112;

    tcBuffer[1] = 
        (fileType << 0);
    *( (uint8_t*)(tcBuffer + 2) ) = counter;
    *( (uint32_t*)(tcBuffer + 3) ) = offset;
    *( (uint16_t*)(tcBuffer + 7) ) = length;

    return 9;
}

uint16_t CUBEBOOT_ReadProgramInfoCmd(uint8_t* tcBuffer, BOOTLOADER_ProgramsList_t programIndex)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 102;

    tcBuffer[1] = 
        (programIndex << 0);

    return 2;
}

uint16_t CUBEBOOT_ResetCmd(uint8_t* tcBuffer, uint8_t magic)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 1;

    *( (uint8_t*)(tcBuffer + 1) ) = magic;

    return 2;
}

uint16_t CUBEBOOT_ResetBootRegistersCmd(uint8_t* tcBuffer)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 6;


    return 1;
}

uint16_t CUBEBOOT_ResetFileListPtrCmd(uint8_t* tcBuffer)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 118;


    return 1;
}

uint16_t CUBEBOOT_ResetLogPtrCmd(uint8_t* tcBuffer)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 4;


    return 1;
}

uint16_t CUBEBOOT_ResetUploadBlockCmd(uint8_t* tcBuffer)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 117;


    return 1;
}

uint16_t CUBEBOOT_RunSelectedProgramCmd(uint8_t* tcBuffer)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 101;


    return 1;
}

uint16_t CUBEBOOT_SramScrubSettingsCmd(uint8_t* tcBuffer, uint16_t scrubSize)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 8;

    *( (uint16_t*)(tcBuffer + 1) ) = scrubSize;

    return 3;
}

uint16_t CUBEBOOT_UnixTimeCmd(uint8_t* tcBuffer, CUBEBOOT_UnixTime_t* setVal)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 2;

    *( (uint32_t*)(tcBuffer + 1) ) = setVal->time;
    *( (uint16_t*)(tcBuffer + 5) ) = setVal->milliSec;

    return 7;
}

uint16_t CUBEBOOT_UnixTimeSaveCmd(uint8_t* tcBuffer, CUBEBOOT_UnixTimeSave_t* setVal)
{
    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 9;

    tcBuffer[1] = 
        (setVal->saveNow << 0) + 
        (setVal->saveOnUpdate << 1) + 
        (setVal->savePeriodic << 2);
    *( (uint8_t*)(tcBuffer + 2) ) = setVal->period;

    return 3;
}


