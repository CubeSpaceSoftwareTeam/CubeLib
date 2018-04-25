/*
 * cubelib.h
 */

#ifndef __CUBELIB_H_
#define __CUBELIB_H_

#include <stdint.h>
#include <stdbool.h>

/// Sensor/actuator interface function result enumeration
typedef enum
{
	CubeLibOk             = 0,
	i2cInProgress         = 1,
	i2cNack               = -1,   /**< NACK received during transfer. */
	i2cBusErr             = -2,   /**< Bus error during transfer (misplaced START/STOP). */
	i2cArbLost            = -3,   /**< Arbitration lost during transfer. */
	i2cUsageFault         = -4,   /**< Usage fault. */
	i2cSwFault            = -5,    /**< SW fault. */
	PointerIsNull         = 3,
	TcInvalidParam        = 4,
	TlmRangeError         = 5,
	TcAckTimeout          = 6
} CUBELIB_Result_t;

#endif /* __CUBELIB_H_ */
