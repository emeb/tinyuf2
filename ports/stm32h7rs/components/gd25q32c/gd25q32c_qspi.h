#ifndef __GD25Q32C_QSPI_H
#define __GD25Q32C_QSPI_H

#ifdef __cplusplus
  extern "C" {
#endif

#include <stdint.h>

/* =============== GD25Qxx CMD ================ */
#define GD25X_WriteEnable         0x06
#define GD25X_WriteDisable        0x04
#define GD25X_ReadStatusReg1      0x05
#define GD25X_ReadStatusReg2      0x35
#define GD25X_ReadStatusReg3      0x15
#define GD25X_WriteStatusReg1     0x01
#define GD25X_WriteStatusReg2     0x31
#define GD25X_WriteStatusReg3     0x11
#define GD25X_ReadData            0x03
#define GD25X_FastReadData        0x0B
#define GD25X_FastReadDual        0x3B
#define GD25X_PageProgram         0x02
#define GD25X_BlockErase          0xD8
#define GD25X_SectorErase         0x20
#define GD25X_ChipErase           0xC7
#define GD25X_PowerDown           0xB9
#define GD25X_ReleasePowerDown    0xAB
#define GD25X_DeviceID            0xAB
#define GD25X_ManufactDeviceID    0x90
#define GD25X_JedecDeviceID       0x9F
#define GD25X_Enable4ByteAddr     0xB7
#define GD25X_Exit4ByteAddr       0xE9
#define GD25X_SetReadParam        0xC0
#define GD25X_EnterQSPIMode       0x38
#define GD25X_ExitQSPIMode        0xFF

#define GD25X_EnableReset         0x66
#define GD25X_ResetDevice         0x99

#define GD25X_QUAD_INOUT_FAST_READ_CMD             0xEB
#define GD25X_QUAD_INOUT_FAST_READ_DTR_CMD         0xED
#define GD25X_QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xEC
#define GD25X_QUAD_INOUT_FAST_READ_4_BYTE_DTR_CMD  0xEE
#define GD25X_QUAD_ManufactDeviceID                0x94
#define GD25X_QUAD_INPUT_PAGE_PROG_CMD             0x32   /*!< Page Program 3 Byte Address */

/* 4-byte Address Mode Operations */
#define GD25X_ENTER_4_BYTE_ADDR_MODE_CMD           0xB7
#define GD25X_EXIT_4_BYTE_ADDR_MODE_CMD            0xE9

/* Dummy cycles for DTR read mode */
#define GD25X_DUMMY_CYCLES_READ_QUAD_DTR  4U
#define GD25X_DUMMY_CYCLES_READ_QUAD      6U



/**
  * @brief  GD25Qxx Registers
  */
/* Status Register */
#define GD25X_SR_WIP              (0x01)    /*!< Write in progress */
#define GD25X_SR_WREN             (0x02)    /*!< Write enable latch */

void      gd25q32c_Init(void);
uint16_t  gd25q32c_GetID(void);
uint8_t   gd25q32c_ReadAllStatusReg(void);
uint8_t   gd25q32c_ReadSR(uint8_t SR);
uint8_t   gd25q32c_WriteSR(uint8_t SR,uint8_t data);
uint8_t   gd25q32c_SetReadParameters(uint8_t DummyClock,uint8_t WrapLenth);
uint8_t   gd25q32c_EnterQPI(void);
uint8_t   gd25q32c_Startup(uint8_t DTRMode);
uint8_t   gd25q32c_WriteEnable(void);
uint8_t   gd25q32c_EraseSector(uint32_t SectorAddress);
uint8_t   gd25q32c_EraseBlock(uint32_t BlockAddress);
uint8_t   gd25q32c_EraseChip(void);
uint8_t   gd25q32c_PageProgram(uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
uint8_t   gd25q32c_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
void      gd25q32c_WriteNoCheck(uint8_t *pBuffer,uint32_t WriteAddr,uint32_t NumByteToWrite);
uint8_t   gd25q32c_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);

#ifdef __cplusplus
}
#endif

#endif
