/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file ST_LSM6DSL_registers.hpp
 *
 * ST LSM6DSL registers.
 *
 */

#pragma once

#include <cstdint>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace ST_LSM6DSL
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz SPI clock frequency

static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHO_AM_I_ID = 0b01101010; // Who I am ID
//static constexpr uint8_t WHO_AM_I_ID = 0b01101000; // Who I am ID

static constexpr uint32_t LA_ODR = 6664; // Linear acceleration output data rate
static constexpr uint32_t G_ODR  = 6664; // Angular rate output data rate

enum class Register : uint8_t {
	WHO_AM_I        = 0x0F, /* Who_AM_I register (r). This register is a read-only register. */

        CTRL1_XL        = 0x10,	// Linear accel sensor Control Register 1.
	CTRL2_G         = 0x11, // Angular rate sensor Control Register 2.

	OUT_TEMP_L      = 0x20, /* Temperature data output register (r). */
	OUT_TEMP_H      = 0x21, /* Temperature data output register (r). */

	STATUS_REG      = 0x1E, // Status reg I think is combined for Accel ang Gyro of lsm9d, also The STATUS_REG register is read by the SPI/I2C interface (r).

	OUT_X_L_G       = 0x22, /* Angular rate sensor pitch axis (X) angular rate output register (r). */
	OUT_X_H_G       = 0x23, /* Angular rate sensor pitch axis (X) angular rate output register (r). */
	OUT_Y_L_G       = 0x24, /* Angular rate sensor roll axis (Y) angular rate output register (r). */
	OUT_Y_H_G       = 0x25, /* Angular rate sensor roll axis (Y) angular rate output register (r). */
	OUT_Z_L_G       = 0x26, /* Angular rate sensor roll axis (Z) angular rate output register (r). */
	OUT_Z_H_G       = 0x27, /* Angular rate sensor roll axis (Z) angular rate output register (r). */

        OUT_X_L_XL      = 0x28, /* Linear acceleration sensor X-axis output register (r). */
	OUT_X_H_XL      = 0x29, /* Linear acceleration sensor X-axis output register (r). */
	OUT_Y_L_XL      = 0x2A, /* Linear acceleration sensor Y-axis output register (r). */
	OUT_Y_H_XL      = 0x2B, /* Linear acceleration sensor Y-axis output register (r). */
	OUT_Z_L_XL      = 0x2C, /* Linear acceleration sensor Z-axis output register (r). */
	OUT_Z_H_XL      = 0x2D, /* Linear acceleration sensor Z-axis output register (r). */


	CTRL3_C         = 0x12, // //
	CTRL4_C         = 0x13, // for I2C Disbale //
	CTRL5_C         = 0x14, // 
	CTRL6_C         = 0x15, // Angular rate Sensor Control register 6. 
	CTRL7_G         = 0x16, // Angular rate Sensor Control register 7.
	CTRL8_XL        = 0x17, // Linear acceleration sensor control register 8
	CTRL9_XL        = 0x18, // Linear acceleration sensor control register 9 
	CTRL10_C       = 0x19, // Control register 10 (r/w) 



        /* Not found the detail in nuttx folder or datasheet but avialable in lsm9d
            SHALL BE DELETED LATER
        */

	//CTRL_REG6_XL    = 0x20, // Linear acceleration sensor Control Register 6.
	//CTRL_REG7_XL    = 0x21, // Linear acceleration sensor Control Register 7.
	//CTRL_REG8       = 0x22, // Control register 8.
	//CTRL_REG9       = 0x23, // Control register 9.

	//STATUS_REG_A    = 0x27,
        /* Not found the detail but avialable in lsm9d
        */
	
	
        FIFO_CTRL1      = 0x06, // FIFO control register.// FIFO threshold
        FIFO_CTRL2      = 0x07, // FIFO control register.// to reset FIFO in Bypass mode
        FIFO_CTRL3      = 0x08, // FIFO control register.// data store rate = data decimated rate available decimation factors are 2, 3, 4, 8, 16, 32 
        FIFO_CTRL4      = 0x09, // FIFO control register.// data store rate
        FIFO_CTRL5      = 0x0A, // FIFO control register.
	FIFO_STATUS1    = 0x3A, // FIFO status control register - FIFO overrun events.
	FIFO_STATUS2    = 0x3B, // FIFO status control register - FIFO Full Status
	FIFO_STATUS3    = 0x3C, // FIFO status control register - FIFO empty status
	FIFO_STATUS4    = 0x3D, // FIFO status control register - 
};

// CTRL_REG1_G

enum CTRL2_G_BIT : uint8_t {
        //ODR_G [3:0]
	ODR_G_6660HZ  = Bit7 | Bit5, // 6660 Hz ODR
	// FS_G [1:0]
	FS_G_2000DPS = Bit3 | Bit2,
        FS_125 = Bit1, //At Bit1 Full Scale at 125dps
};
//enum CTRL1_REG1_G_BIT : uint8_t {
//	// ODR_G [2:0]
//	ODR_G_952HZ  = Bit7 | Bit6, // 952 Hz ODR
//	// FS_G [1:0]
//	FS_G_2000DPS = Bit4 | Bit3,
//	// BW_G [1:0]
//	BW_G_100Hz   = Bit1 | Bit0, // BW_G 100 Hz
//};

// STATUS_REG ()
enum STATUS_REG_BIT : uint8_t {
	TDA  = Bit2, // Temperature sensor new data available.
	GDA  = Bit1, // Gyroscope new data available.
	XLDA = Bit0, // Accelerometer new data available.
};

// CTRL_REG6_XL
enum CTRL1_XL_BIT : uint8_t {
        // ODR_XL [3:0]
	ODR_XL_6660HZ  = Bit7 | Bit5, // 6660 Hz ODR
        //FS_XL [1:0]
	FS_XL = Bit3, // FS_XL 01: _16 g
        LPF1_BW_SEL = Bit1,
	BW0_XL_400Hz   = Bit0, // BW0_XL 400 Hz
};

//enum CTRL_REG6_XL_BIT : uint8_t {
//	// ODR_XL [2:0]
//	ODR_XL_952HZ = Bit7 | Bit6, // 952 Hz ODR
//	// FS_XL [1:0]
//	FS_XL_16     = Bit3,        // FS_XL 01: ±16 g
//};

// CTRL_REG7_XL
//enum CTRL_REG7_XL_BIT : uint8_t {
//	HR  = Bit7, // High resolution mode for accelerometer enable.
//	FDS = Bit2, // Filtered data selection. 0: internal filter bypassed
//};

// CTRL3_C
enum CTRL3_C_BIT : uint8_t {
	BDU        = Bit6, // Block data update Default value: 0 (0: continuous update; 1: output registers not updated until MSB and LSB have been read)

	IF_ADD_INC = Bit2, // Register address automatically incremented Default value: 1 (0: disabled; 1: enabled

	SW_RESET   = Bit0, // Software reset Default value: 0  0: normal mode; 1: reset device
};

// CTRL4_C
enum CTRL4_C_BIT : uint8_t {
	I2C_DISABLE = Bit2, // Default value: 0 0: both I2C and SPI enabled; 1: I2C disabled, SPI only enabled
};

// FIFO_CTRL
enum FIFO_CTRL5_BIT : uint8_t {
	// FMODE [2:0]
	FMODE_CONTINUOUS = Bit2 | Bit1, // Continuous mode. If the FIFO is full, the new sample over- writes the older sample.
};

// FIFO_SRC
enum FIFO_STATUS2_BIT : uint8_t {
	OVRN = Bit6, // FIFO overrun status.
	DIFF_FIFO  = Bit2 | Bit1 | Bit0,
};


namespace FIFO
{
static constexpr size_t SIZE = 32 * 12; // 32 samples max
}

} // namespace ST_LSM6DSL
