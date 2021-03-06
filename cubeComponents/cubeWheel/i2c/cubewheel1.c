/************************************************************************************
 * This file was auto-generated by CIDEA                           .                *
 * Please do not modify the contents of this file manually.                         *
 ***********************************************************************************/


#include "standardnode.h"
#include "bsp_i2c.h"
#include "cubewheel1.h"

uint8_t cWheel1CommsBuffer[8];

CUBELIB_Result_t CUBEWHEEL1_ReqBackupGainTlm(uint8_t nodeid, CUBEWHEEL1_BackupGain_t* backupGain)
{
    uint8_t tlmId = 141;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    if (backupGain == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 6); BSP_TIME_Delay(2);
    if (i2cResult != i2cTransferDone)
        return i2cResult;


    backupGain->ki = *( (uint16_t*) (tlmBuffer + 0) );

    backupGain->kiMultiplier = *( (uint8_t*) (tlmBuffer + 2) );

    backupGain->kd = *( (uint16_t*) (tlmBuffer + 3) );

    backupGain->kdMultiplier = *( (uint8_t*) (tlmBuffer + 5) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_ReqExtendedIdentificationTlm(uint8_t nodeid, CUBEWHEEL1_ExtendedIdentification_t* extendedIdentification)
{
    uint8_t tlmId = 129;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    if (extendedIdentification == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 4); BSP_TIME_Delay(2);
    if (i2cResult != i2cTransferDone)
        return i2cResult;


    extendedIdentification->serialNumber = *( (uint16_t*) (tlmBuffer + 0) );

    extendedIdentification->i2CAddress = *( (uint8_t*) (tlmBuffer + 2) );

    extendedIdentification->cANAddress = *( (uint8_t*) (tlmBuffer + 3) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_ReqIdentificationTlm(uint8_t nodeid, CUBEWHEEL1_Identification_t* identification)
{
    uint8_t tlmId = 128;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    if (identification == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 8); BSP_TIME_Delay(2);
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

CUBELIB_Result_t CUBEWHEEL1_ReqMainGainTlm(uint8_t nodeid, CUBEWHEEL1_MainGain_t* mainGain)
{
    uint8_t tlmId = 140;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    if (mainGain == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 6); BSP_TIME_Delay(2);
    if (i2cResult != i2cTransferDone)
        return i2cResult;


    mainGain->ki = *( (uint16_t*) (tlmBuffer + 0) );

    mainGain->kiMultiplier = *( (uint8_t*) (tlmBuffer + 2) );

    mainGain->kd = *( (uint16_t*) (tlmBuffer + 3) );

    mainGain->kdMultiplier = *( (uint8_t*) (tlmBuffer + 5) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_ReqPWMGainTlm(uint8_t nodeid, CUBEWHEEL1_PWMGain_t* pWMGain)
{
    uint8_t tlmId = 139;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    if (pWMGain == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 3); BSP_TIME_Delay(2);
    if (i2cResult != i2cTransferDone)
        return i2cResult;


    pWMGain->k = *( (int16_t*) (tlmBuffer + 0) );

    pWMGain->kmultiplier = *( (uint8_t*) (tlmBuffer + 2) );

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_ReqStatusErrorFlagsTlm(uint8_t nodeid, CUBEWHEEL1_StatusErrorFlags_t* statusErrorFlags)
{
    uint8_t tlmId = 145;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    if (statusErrorFlags == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 1); BSP_TIME_Delay(2);
    if (i2cResult != i2cTransferDone)
        return i2cResult;


    statusErrorFlags->invalidTelemetry = (tlmBuffer[0] & 0x01) >> 0;

    statusErrorFlags->invalidTelecommand = (tlmBuffer[0] & 0x02) >> 1;

    statusErrorFlags->encoderError = (tlmBuffer[0] & 0x04) >> 2;

    statusErrorFlags->uARTError = (tlmBuffer[0] & 0x08) >> 3;

    statusErrorFlags->i2CError = (tlmBuffer[0] & 0x10) >> 4;

    statusErrorFlags->cANError = (tlmBuffer[0] & 0x20) >> 5;

    statusErrorFlags->configError = (tlmBuffer[0] & 0x40) >> 6;

    statusErrorFlags->speedError = (tlmBuffer[0] & 0x80) >> 7;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_ReqWheelCurrentTlm(uint8_t nodeid, double* wheelCurrent)
{
    uint8_t tlmId = 135;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    uint16_t rawwheelCurrent;
    I2C_TransferReturn_TypeDef i2cResult;

    if (wheelCurrent == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 2); BSP_TIME_Delay(2);
    if (i2cResult != i2cTransferDone)
        return i2cResult;


    rawwheelCurrent = *( (uint16_t*) (tlmBuffer + 0) );
    *wheelCurrent = rawwheelCurrent*0.48828125;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_ReqWheelDataTlm(uint8_t nodeid, CUBEWHEEL1_WheelData_t* wheelData)
{
    uint8_t tlmId = 137;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    int16_t rawWheelSpeed;
    int16_t rawWheelReference;
    uint16_t rawWheelCurrent;
    I2C_TransferReturn_TypeDef i2cResult;

    if (wheelData == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 6); BSP_TIME_Delay(2);
    if (i2cResult != i2cTransferDone)
        return i2cResult;


    rawWheelSpeed = *( (int16_t*) (tlmBuffer + 0) );
    wheelData->wheelSpeed = rawWheelSpeed/2.0;

    rawWheelReference = *( (int16_t*) (tlmBuffer + 2) );
    wheelData->wheelReference = rawWheelReference/2.0;

    rawWheelCurrent = *( (uint16_t*) (tlmBuffer + 4) );
    wheelData->wheelCurrent = rawWheelCurrent*0.48828125;

    if ((wheelData->wheelSpeed < -10000) || (wheelData->wheelSpeed > 10000))
        return TlmRangeError;

    if ((wheelData->wheelReference < -10000) || (wheelData->wheelReference > 10000))
        return TlmRangeError;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_ReqWheelDataAdditionalTlm(uint8_t nodeid, CUBEWHEEL1_WheelDataAdditional_t* wheelDataAdditional)
{
    uint8_t tlmId = 138;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    int16_t rawWheelBackupSpeed;
    I2C_TransferReturn_TypeDef i2cResult;

    if (wheelDataAdditional == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 4); BSP_TIME_Delay(2);
    if (i2cResult != i2cTransferDone)
        return i2cResult;


    wheelDataAdditional->wheelDuty = *( (int16_t*) (tlmBuffer + 0) );

    rawWheelBackupSpeed = *( (int16_t*) (tlmBuffer + 2) );
    wheelDataAdditional->wheelBackupSpeed = rawWheelBackupSpeed/2.0;

    if ((wheelDataAdditional->wheelBackupSpeed < -10000) || (wheelDataAdditional->wheelBackupSpeed > 10000))
        return TlmRangeError;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_ReqWheelReferenceTlm(uint8_t nodeid, double* wheelReference)
{
    uint8_t tlmId = 134;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    int16_t rawwheelReference;
    I2C_TransferReturn_TypeDef i2cResult;

    if (wheelReference == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 2); BSP_TIME_Delay(2);
    if (i2cResult != i2cTransferDone)
        return i2cResult;


    rawwheelReference = *( (int16_t*) (tlmBuffer + 0) );
    *wheelReference = rawwheelReference/2.0;

    if ((*wheelReference < -10000) || (*wheelReference > 10000))
        return TlmRangeError;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_ReqWheelSpeedTlm(uint8_t nodeid, double* wheelSpeed)
{
    uint8_t tlmId = 133;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    int16_t rawwheelSpeed;
    I2C_TransferReturn_TypeDef i2cResult;

    if (wheelSpeed == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 2); BSP_TIME_Delay(2);
    if (i2cResult != i2cTransferDone)
        return i2cResult;


    rawwheelSpeed = *( (int16_t*) (tlmBuffer + 0) );
    *wheelSpeed = rawwheelSpeed/2.0;

    if ((*wheelSpeed < -10000) || (*wheelSpeed > 10000))
        return TlmRangeError;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_ReqWheelStatusTlm(uint8_t nodeid, CUBEWHEEL1_WheelStatus_t* wheelStatus)
{
    uint8_t tlmId = 130;
    uint8_t* tlmBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;
    uint8_t enumVal;

    if (wheelStatus == 0)
        return PointerIsNull;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE_READ, &tlmId, 1, tlmBuffer, 8); BSP_TIME_Delay(2);
    if (i2cResult != i2cTransferDone)
        return i2cResult;


    wheelStatus->runtimeSeconds = *( (uint16_t*) (tlmBuffer + 0) );

    wheelStatus->runtimeMilliseconds = *( (uint16_t*) (tlmBuffer + 2) );

    wheelStatus->temperature = *( (uint16_t*) (tlmBuffer + 4) );

    enumVal = tlmBuffer[6];
    wheelStatus->controlMode = (CWHEEL1_ControlModeVal_t) enumVal;

    wheelStatus->backupMode = (tlmBuffer[7] & 0x01) >> 0;

    wheelStatus->motorSwitch = (tlmBuffer[7] & 0x02) >> 1;

    wheelStatus->hallSwitch = (tlmBuffer[7] & 0x04) >> 2;

    wheelStatus->encoderSwitch = (tlmBuffer[7] & 0x08) >> 3;

    wheelStatus->errorFlag = (tlmBuffer[7] & 0x10) >> 4;

    return CubeLibOk;
}


CUBELIB_Result_t CUBEWHEEL1_SendBackupGainCmd(uint8_t nodeid, uint16_t ki, uint8_t kiMultiplier, uint16_t kd, uint8_t kdMultiplier)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 35;

    *( (uint16_t*)(tcBuffer + 1) ) = ki;
    *( (uint8_t*)(tcBuffer + 3) ) = kiMultiplier;
    *( (uint16_t*)(tcBuffer + 4) ) = kd;
    *( (uint8_t*)(tcBuffer + 6) ) = kdMultiplier;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 7, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendBackupWheelModeCmd(uint8_t nodeid, bool backupMode)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 12;

    tcBuffer[1] =
        (backupMode << 0);

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 2, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendCANMaskCmd(uint8_t nodeid, uint8_t cANAddress)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 32;

    *( (uint8_t*)(tcBuffer + 1) ) = cANAddress;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 2, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendClearErrorsCmd(uint8_t nodeid, uint8_t clearErrorParam)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 20;

    *( (uint8_t*)(tcBuffer + 1) ) = clearErrorParam;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 2, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendControlModeCmd(uint8_t nodeid, CWHEEL1_ControlModeVal_t controlMode)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    if (((controlMode < 0) || (controlMode > 3)))
        return TcInvalidParam;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 10;

    tcBuffer[1] =
        (controlMode << 0);

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 2, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendEncoderPowerCmd(uint8_t nodeid, bool encoderPowerOn)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 8;

    tcBuffer[1] =
        (encoderPowerOn << 0);

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 2, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendHallPowerCmd(uint8_t nodeid, bool hallSensorPowerOn)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 9;

    tcBuffer[1] =
        (hallSensorPowerOn << 0);

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 2, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendI2CAddressCmd(uint8_t nodeid, uint8_t i2CAddress)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 31;

    *( (uint8_t*)(tcBuffer + 1) ) = i2CAddress;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 2, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendMainGainCmd(uint8_t nodeid, uint16_t ki, uint8_t kiMultiplier, uint16_t kd, uint8_t kdMultiplier)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 34;

    *( (uint16_t*)(tcBuffer + 1) ) = ki;
    *( (uint8_t*)(tcBuffer + 3) ) = kiMultiplier;
    *( (uint16_t*)(tcBuffer + 4) ) = kd;
    *( (uint8_t*)(tcBuffer + 6) ) = kdMultiplier;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 7, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendMotorPowerCmd(uint8_t nodeid, bool motorPowerOn)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 7;

    tcBuffer[1] =
        (motorPowerOn << 0);

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 2, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendPWMGainCmd(uint8_t nodeid, int16_t k, uint8_t kmultiplier)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 33;

    *( (int16_t*)(tcBuffer + 1) ) = k;
    *( (uint8_t*)(tcBuffer + 3) ) = kmultiplier;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 4, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendResetCmd(uint8_t nodeid, uint8_t resetParam)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 0;

    *( (uint8_t*)(tcBuffer + 1) ) = resetParam;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 2, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendWheelSpeedRefCmd(uint8_t nodeid, double speedRef)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    if ((speedRef < -10000) || (speedRef > 10000))
        return TcInvalidParam;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 2;

    *( (int16_t*)(tcBuffer + 1) ) = (int16_t) round((speedRef)*2.0);

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 3, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}

CUBELIB_Result_t CUBEWHEEL1_SendWheelTorqueCmd(uint8_t nodeid, int16_t dutyCycle)
{
    uint8_t* tcBuffer = cWheel1CommsBuffer;
    I2C_TransferReturn_TypeDef i2cResult;

    if ((dutyCycle < -100) || (dutyCycle > 100))
        return TcInvalidParam;

    // write TtcMessage ID to first element in buffer
    tcBuffer[0] = 3;

    *( (int16_t*)(tcBuffer + 1) ) = dutyCycle;

    i2cResult = BSP_I2C_masterTransmit(BSP_I2C_SUB, nodeid, I2C_FLAG_WRITE, tcBuffer, 3, NULL, 0);
    if (i2cResult != i2cTransferDone)
        return i2cResult;

    return CubeLibOk;
}
