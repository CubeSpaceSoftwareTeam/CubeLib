
#include "cubelib.h"
#include "cubeacp3.h"
#include "bsp_i2c.h"

#include <stddef.h>

uint8_t cubeAcpCommsBuffer[568];
/*
CUBELIB_Result_t CUBEACP3_ReqIdentificationTlm(uint8_t nodeid, CUBEACP3_Identification_t* identification)
{
	uint8_t tlmId = 128;
	uint8_t* tlmBuffer = cubeAcpCommsBuffer;
	I2C_TransferReturn_TypeDef i2cResult;

	if (identification == 0)
		return PointerIsNull;

	i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 8);
	if (i2cResult != i2cTransferDone)
		return i2cResult;

	identification->nodeType = *( (uint8_t*) (tlmBuffer + 0) );

	identification->interfaceVersion = *( (uint8_t*) (tlmBuffer + 1) );

	identification->firmwareMajorVersion = *( (uint8_t*) (tlmBuffer + 2) );

	identification->firmwareMinorVersion = *( (uint8_t*) (tlmBuffer + 3) );

	identification->runtimeSeconds = *( (uint16_t*) (tlmBuffer + 4) );

	identification->runtimeMilliseconds = *( (uint16_t*) (tlmBuffer + 6) );

	return CubeLibOk;
}

CUBELIB_Result_t CUBEACP3_ReqUnixTimeTlm(uint8_t nodeid, CUBEACP3_SetTime_t* unixTime)
{
	uint8_t tlmId = 140;
	uint8_t* tlmBuffer = cubeAcpCommsBuffer;
	I2C_TransferReturn_TypeDef i2cResult;

	if (unixTime == 0)
		return PointerIsNull;

	i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 6);
	if (i2cResult != i2cTransferDone)
		return i2cResult;

	unixTime->unixTimeS = *( (uint32_t*) (tlmBuffer + 0) );

	unixTime->unixTimeMs = *( (uint16_t*) (tlmBuffer + 4) );

	return CubeLibOk;
}

CUBELIB_Result_t CUBEACP3_SendAdcsEnabledCmd(uint8_t nodeid, CUBEACP3_AdcsRunMode_t enabled)
{
	uint8_t* tcBuffer = cubeAcpCommsBuffer;
	I2C_TransferReturn_TypeDef i2cResult;

	if (((enabled < 0) || (enabled > 2)))
		return TcInvalidParam;

	// write TtcMessage ID to first element in buffer
	tcBuffer[0] = 10;

	tcBuffer[1] =
			(enabled << 0);

	i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE, tcBuffer, 2, NULL, 0);
	if (i2cResult != i2cTransferDone)
		return i2cResult;

	return CubeLibOk;
}

CUBELIB_Result_t CUBEACP3_ReqAdcsStateTlm(uint8_t nodeid, CUBEACP3_AdcsState_t* adcsState)
{
	uint8_t tlmId = 190;
	uint8_t* tlmBuffer = cubeAcpCommsBuffer;
	int16_t rawRoll;
	int16_t rawPitch;
	int16_t rawYaw;
	int16_t rawRateX;
	int16_t rawRateY;
	int16_t rawRateZ;
	int16_t rawPositionX;
	int16_t rawPositionY;
	int16_t rawPositionZ;
	int16_t rawVelocityX;
	int16_t rawVelocityY;
	int16_t rawVelocityZ;
	int16_t rawLatitude;
	int16_t rawLongitude;
	uint16_t rawAltitude;
	I2C_TransferReturn_TypeDef i2cResult;

	if (adcsState == 0)
		return PointerIsNull;

	i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 48);
	if (i2cResult != i2cTransferDone)
		return i2cResult;

	adcsState->adcsRunMode = (CUBEACP3_AdcsRunMode_t) ((tlmBuffer[0] & 0b00000011) >> 0);

	adcsState->estimMode = (CUBEACP3_EstimModeSelect_t) ((tlmBuffer[0]&0b00011100) >> 2);

	adcsState->controlMode = (CUBEACP3_ConModeSelect_t) ((tlmBuffer[0]&0b11100000) >> 5);

	adcsState->cubeControlSignalPower = (tlmBuffer[7] & 0x01) >> 0;

	adcsState->cubeControlMotorPower = (tlmBuffer[7] & 0x02) >> 1;

	adcsState->cubeSensePower = (tlmBuffer[7] & 0x04) >> 2;

	adcsState->gpsReceiverPower = (tlmBuffer[7] & 0x08) >> 3;

	adcsState->gpsLnaPower = (tlmBuffer[7] & 0x10) >> 4;

	adcsState->motorDriverPower = (tlmBuffer[7] & 0x20) >> 5;

	adcsState->cubeSenseCommsError = (tlmBuffer[8] & 0x02) >> 1;

	adcsState->cubeControlSignalCommsError = (tlmBuffer[8] & 0x04) >> 2;

	adcsState->cubeControlMotorCommsError = (tlmBuffer[8] & 0x08) >> 3;

	adcsState->wheelSpeedRangeError = (tlmBuffer[10] & 0x08) >> 3;

	adcsState->cssError = (tlmBuffer[10] & 0x10) >> 4;

	adcsState->orbitParamsInvalidError = (tlmBuffer[10] & 0x20) >> 5;

	adcsState->configInvalidError = (tlmBuffer[10] & 0x40) >> 6;

	adcsState->magfieldModelError = (tlmBuffer[11] & 0x02) >> 1;

	adcsState->nodeRecoveryError = (tlmBuffer[11] & 0x10) >> 4;

	adcsState->sunAboveHorizon = (tlmBuffer[11] & 0x80) >> 7;

	rawRoll = *( (int16_t*) (tlmBuffer + 18) );
	adcsState->roll = rawRoll*0.01;

	rawPitch = *( (int16_t*) (tlmBuffer + 20) );
	adcsState->pitch = rawPitch*0.01;

	rawYaw = *( (int16_t*) (tlmBuffer + 22) );
	adcsState->yaw = rawYaw*0.01;

	rawRateX = *( (int16_t*) (tlmBuffer + 24) );
	adcsState->rateX = rawRateX*0.01;

	rawRateY = *( (int16_t*) (tlmBuffer + 26) );
	adcsState->rateY = rawRateY*0.01;

	rawRateZ = *( (int16_t*) (tlmBuffer + 28) );
	adcsState->rateZ = rawRateZ*0.01;

	rawPositionX = *( (int16_t*) (tlmBuffer + 24) );
	adcsState->positionX = rawPositionX*0.25;

	rawPositionY = *( (int16_t*) (tlmBuffer + 26) );
	adcsState->positionY = rawPositionY*0.25;

	rawPositionZ = *( (int16_t*) (tlmBuffer + 28) );
	adcsState->positionZ = rawPositionZ*0.25;

	rawVelocityX = *( (int16_t*) (tlmBuffer + 30) );
	adcsState->velocityX = rawVelocityX*0.001;

	rawVelocityY = *( (int16_t*) (tlmBuffer + 32) );
	adcsState->velocityY = rawVelocityY*0.001;

	rawVelocityZ = *( (int16_t*) (tlmBuffer + 34) );
	adcsState->velocityZ = rawVelocityZ*0.001;

	rawLatitude = *( (int16_t*) (tlmBuffer + 36) );
	adcsState->latitude = rawLatitude*0.01;

	rawLongitude = *( (int16_t*) (tlmBuffer + 38) );
	adcsState->longitude = rawLongitude*0.01;

	rawAltitude = *( (uint16_t*) (tlmBuffer + 40) );
	adcsState->altitude = rawAltitude*0.02;

	return CubeLibOk;
}

CUBELIB_Result_t CUBEACP3_ReqAcpLoopStateTlm(uint8_t nodeid, CUBEACP3_AcpLoopState_t* acpLoopState)
{
    uint8_t tlmId = 220;
    uint8_t* tlmBuffer = cubeAcpCommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    if (acpLoopState == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 3);
    	if (i2cResult != i2cTransferDone)
    		return i2cResult;

    acpLoopState->timeSinceLoopStart = *( (uint16_t*) (tlmBuffer + 0) );

    acpLoopState->currentExecutionPoint = (CUBEACP3_ExecutionWaypoints_t) tlmBuffer[2];

    return CubeLibOk;
}
*/
CUBELIB_Result_t CUBEACP3_ReqFileInfoTlm(uint8_t nodeid, CUBEACP3_FileInfo_t* fileInfo)
{
	uint8_t tlmId = 243;
	uint8_t* tlmBuffer = cubeAcpCommsBuffer;
	I2C_TransferReturn_TypeDef i2cResult;

	i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 12);
	if (i2cResult != i2cTransferDone)
		return i2cResult;

	fileInfo->fileType = (uint8_t) ((tlmBuffer[0] & 0b00001111) >> 0);

	fileInfo->busyUpdating = (bool) ((tlmBuffer[0] & 0b0010000) >> 4);

	fileInfo->fileCtr = *( (uint8_t*) (tlmBuffer + 1) );

	fileInfo->size = *( (uint32_t*) (tlmBuffer + 2) );

	fileInfo->unixTime = *( (uint32_t*) (tlmBuffer + 6) );

	fileInfo->checksum = *( (uint16_t*) (tlmBuffer + 10) );

	return CubeLibOk;
}

CUBELIB_Result_t CUBEACP3_SendAdvanceFileReadPointerCmd(uint8_t nodeid)
{
	uint8_t* tcBuffer = cubeAcpCommsBuffer;
	I2C_TransferReturn_TypeDef i2cResult;

	// write TtcMessage ID to first element in buffer
	tcBuffer[0] = 113;

	i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE, tcBuffer, 1, NULL, 0);
	if (i2cResult != i2cTransferDone)
		return i2cResult;

	return CubeLibOk;
}

CUBELIB_Result_t CUBEACP3_SendLoadFileBlockCmd(uint8_t nodeid, CUBEACP3_LoadFileBlock_t* loadFileBlock)
{
	uint8_t* tcBuffer = cubeAcpCommsBuffer;
	I2C_TransferReturn_TypeDef i2cResult;

	// write TtcMessage ID to first element in buffer
	tcBuffer[0] = 112;

	*( (uint8_t*)(tcBuffer + 1) ) = loadFileBlock->fileType;
	*( (uint8_t*)(tcBuffer + 2) ) = loadFileBlock->counter;
	*( (uint32_t*)(tcBuffer + 3) ) = loadFileBlock->offset;
	*( (uint16_t*)(tcBuffer + 7) ) = loadFileBlock->length;

	i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE, tcBuffer, 9, NULL, 0);
	if (i2cResult != i2cTransferDone)
		return i2cResult;

	return CubeLibOk;
}

CUBELIB_Result_t CUBEACP3_ReqDownloadBlockReadyTlm(uint8_t nodeid, FILES_DownloadBlockReady_t* downloadBlockReady)
{
	uint8_t tlmId = 242;
	uint8_t* tlmBuffer = cubeAcpCommsBuffer;
	I2C_TransferReturn_TypeDef i2cResult;

	i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 5);
	if (i2cResult != i2cTransferDone)
		return i2cResult;

	downloadBlockReady->ready = (bool) ((tlmBuffer[0] & 0b0000001) >> 0);
	downloadBlockReady->checksum = *( (uint16_t*) (tlmBuffer + 1) );
	downloadBlockReady->length = *( (uint16_t*) (tlmBuffer + 3) );

	return CubeLibOk;
}

CUBELIB_Result_t CUBEACP3_ReqFilePacketTlm(uint8_t nodeid, FILES_DownloadFilePacket_t* filePacket)
{
	uint8_t tlmId = 241;
	uint8_t* tlmBuffer = cubeAcpCommsBuffer;
	I2C_TransferReturn_TypeDef i2cResult;

	i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 23);
	if (i2cResult != i2cTransferDone)
		return i2cResult;

	filePacket->messageID = tlmBuffer[0];
	filePacket->packetCounter = *( (uint16_t*) (tlmBuffer + 1) );
	memcpy(filePacket->fileBytes,  tlmBuffer+3, 20);

	return CubeLibOk;
}

CUBELIB_Result_t CUBEACP3_SendInitiateDownloadBurstCmd(uint8_t nodeid, FILES_DownloadBurst_t* fileBurst)
{
	uint8_t* tcBuffer = cubeAcpCommsBuffer;
	I2C_TransferReturn_TypeDef i2cResult;

	// write TtcMessage ID to first element in buffer
	tcBuffer[0] = 119;

	*( (uint8_t*)(tcBuffer + 1) ) = fileBurst->messageLength;
	*( (uint8_t*)(tcBuffer + 2) ) = fileBurst->ignoreHoleMap;

	i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE, tcBuffer, 3, NULL, 0);
	if (i2cResult != i2cTransferDone)
		return i2cResult;

	return CubeLibOk;
}

CUBELIB_Result_t CUBEACP3_SendHoleMapCmd(uint8_t nodeid, uint8_t holemapNr , uint8_t* holemap)
{
	uint8_t* tcBuffer = cubeAcpCommsBuffer;
	I2C_TransferReturn_TypeDef i2cResult;

	// write TtcMessage ID to first element in buffer
	tcBuffer[0] = 119 + holemapNr;

	memcpy(tcBuffer + 1,  holemap, 16);

	i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SYS, nodeid, I2C_FLAG_WRITE, tcBuffer, 17, NULL, 0);
	if (i2cResult != i2cTransferDone)
		return i2cResult;

	return CubeLibOk;
}
