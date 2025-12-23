#ifndef __GD25Q32C_QSPI_H
#define __GD25Q32C_QSPI_H

#ifdef __cplusplus
  extern "C" {
#endif

#include <stdint.h>

void      gd25q32c_Init(void);
uint8_t   gd25q32c_Startup(void);
uint8_t   gd25q32c_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
uint8_t   gd25q32c_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
uint8_t   gd25q32c_EraseChip(void);

#ifdef __cplusplus
}
#endif

#endif
