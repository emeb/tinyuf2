#include "gd25q32c_qspi.h"
#include "qspi_status.h"
#include "stm32h7rsxx_hal.h"
#include "board_api.h"

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

#define GD25X_EnableReset         0x66
#define GD25X_ResetDevice         0x99

#define GD25X_QUAD_INOUT_FAST_READ_CMD             0xEB
#define GD25X_QUAD_ManufactDeviceID                0x94
#define GD25X_QUAD_INPUT_PAGE_PROG_CMD             0x32   /*!< Page Program 3 Byte Address */

/* Dummy cycles for DTR read mode */
#define GD25X_DUMMY_CYCLES_READ_QUAD      6U

/**
  * @brief  GD25Qxx Registers
  */
/* Status Register */
#define GD25X_SR_WIP              (0x01)    /*!< Write in progress */
#define GD25X_SR_WREN             (0x02)    /*!< Write enable latch */

extern XSPI_HandleTypeDef hxspi1;

uint8_t gd25q32c_StatusReg[3];

/**
  * @brief  Configure the QSPI in memory-mapped mode
  * @param  hqspi: QSPI handle
  * @retval QSPI memory status
  */
static uint8_t XSPI_EnableMemoryMappedMode(XSPI_HandleTypeDef *hqspi)
{
	/* from EMEB's known-working code */
	XSPI_RegularCmdTypeDef   sCommand = {0};
	XSPI_MemoryMappedTypeDef sMemMappedCfg;

	/* set write & read ops & turn on mem mapped mode */
	/* write mode should not be used on flash but it needs to be set up anyway */
	sCommand.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE; // on this device all instr are single
	sCommand.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	sCommand.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;
	sCommand.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
	sCommand.DataMode			= HAL_XSPI_DATA_4_LINES;
	sCommand.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;
	sCommand.DQSMode            = HAL_XSPI_DQS_DISABLE;
	
	sCommand.OperationType		= HAL_XSPI_OPTYPE_WRITE_CFG;
	sCommand.AddressMode		= HAL_XSPI_ADDRESS_4_LINES;
	sCommand.AddressWidth		= HAL_XSPI_ADDRESS_24_BITS;
	sCommand.Instruction 		= GD25X_QUAD_INPUT_PAGE_PROG_CMD;
	sCommand.DummyCycles		= 0U;
	sCommand.DQSMode            = HAL_XSPI_DQS_ENABLE; // ERRATUM 2.4.1
	if (HAL_XSPI_Command(hqspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return qspi_ERROR;
	}
	
	/* set read ops & turn on mem mapped mode */
	sCommand.OperationType		= HAL_XSPI_OPTYPE_READ_CFG;
	sCommand.Instruction 		= GD25X_QUAD_INOUT_FAST_READ_CMD;
	sCommand.DummyCycles		= GD25X_DUMMY_CYCLES_READ_QUAD;
	sCommand.DQSMode            = HAL_XSPI_DQS_DISABLE;	// return to correct value
	if (HAL_XSPI_Command(hqspi, &sCommand, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return qspi_ERROR;
	}

	sMemMappedCfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_ENABLE;
	sMemMappedCfg.TimeoutPeriodClock      = 0x34;
	if (HAL_XSPI_MemoryMapped(hqspi, &sMemMappedCfg) != HAL_OK)
	{
		return qspi_ERROR;
	}
		
	return qspi_OK;
}

/**
 * @brief	Send QSPI command instruction w/ proper addr/alt/dummy/data setup 
 *
 * @param   instruction
 * @param   address
 * @param   dummyCycles
 * @param   instructionMode
 * @param   addressMode
 * @param   dataMode
 * @param   dataSize
 *
 * @return  qspi status
 */
static uint8_t XSPI_Send_CMD(XSPI_HandleTypeDef *hqspi, uint32_t instruction,
	uint32_t address, uint32_t dummyCycles, uint32_t addressMode,
	uint32_t dataMode, uint32_t dataSize)
{
    XSPI_RegularCmdTypeDef Cmdhandler = {0};

	Cmdhandler.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
    Cmdhandler.Instruction        = instruction;
    Cmdhandler.InstructionMode    = HAL_XSPI_INSTRUCTION_1_LINE;
	Cmdhandler.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;

    Cmdhandler.Address            = address;
    Cmdhandler.AddressWidth       = HAL_XSPI_ADDRESS_24_BITS;
    Cmdhandler.AddressMode        = addressMode;
	Cmdhandler.AddressDTRMode     = HAL_XSPI_ADDRESS_DTR_DISABLE;

    Cmdhandler.AlternateBytes     = 0x00;
    Cmdhandler.AlternateBytesWidth = HAL_XSPI_ALT_BYTES_8_BITS;
    Cmdhandler.AlternateBytesMode  = HAL_XSPI_ALT_BYTES_NONE;
	
    Cmdhandler.DummyCycles        = dummyCycles;

    Cmdhandler.DataMode           = dataMode;
    Cmdhandler.DataLength         = dataSize;
	Cmdhandler.DataDTRMode        = HAL_XSPI_DATA_DTR_DISABLE;

    Cmdhandler.DQSMode            = HAL_XSPI_DQS_DISABLE;

    if(HAL_XSPI_Command(hqspi, &Cmdhandler, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      return qspi_ERROR;

    return qspi_OK;
}

/**
  * @brief  This function reset the QSPI memory.
  * @param  hqspi: QSPI handle
  * @retval qspi status
  */
static uint8_t XSPI_ResetDevice(XSPI_HandleTypeDef *hqspi)
{
	if(XSPI_Send_CMD(hqspi, GD25X_EnableReset, 0, 0,
		HAL_XSPI_ADDRESS_NONE, HAL_XSPI_DATA_NONE, 0) != qspi_OK)
		return qspi_ERROR;
	
	if(XSPI_Send_CMD(hqspi, GD25X_ResetDevice, 0, 0,
		HAL_XSPI_ADDRESS_NONE, HAL_XSPI_DATA_NONE, 0) != qspi_OK)
		return qspi_ERROR;
	
	return qspi_OK;
}

/**
  * @brief  set the write enable bit
  * @param  hqspi: QSPI handle
  * @retval qspi status
  */
static uint8_t XSPI_WriteEnable(XSPI_HandleTypeDef *hqspi)
{
	return XSPI_Send_CMD(hqspi, GD25X_WriteEnable,
		0U, 0U, HAL_XSPI_ADDRESS_NONE, HAL_XSPI_DATA_NONE, 0U);
}

/**
  * @brief Read one status register
  * @param SR: command instruction for reading status reg 1-3
  * @param byte: pointer to uint8_t for result
  * @retval qspi status
  */
static uint8_t XSPI_ReadSR(uint8_t SR, uint8_t *byte)
{
	XSPI_Send_CMD(&hxspi1, SR, 0x00, 0,
		HAL_XSPI_ADDRESS_NONE, HAL_XSPI_DATA_1_LINE, 1);

	if (HAL_XSPI_Receive(&hxspi1, byte, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		return qspi_ERROR;
	
	return qspi_OK;
}

/**
  * @brief Fetch all three status registers into local storage
  * @retval qspi status
  */
static uint8_t XSPI_ReadAllStatusReg(void)
{
	if(XSPI_ReadSR(GD25X_ReadStatusReg1, &gd25q32c_StatusReg[0]) !=  qspi_OK)
		return qspi_ERROR;
	if(XSPI_ReadSR(GD25X_ReadStatusReg2, &gd25q32c_StatusReg[1]) !=  qspi_OK)
		return qspi_ERROR;
	if(XSPI_ReadSR(GD25X_ReadStatusReg3, &gd25q32c_StatusReg[2]) !=  qspi_OK)
		return qspi_ERROR;
	return qspi_OK;
}

/**
  * @brief Wait for Busy flag to deassert
  * @retval none
  */
static void XSPI_Wait_Busy(void)
{
	uint8_t status;
	
	XSPI_ReadSR(GD25X_ReadStatusReg1, &status);
	
	while((status & GD25X_SR_WIP) == GD25X_SR_WIP)
	{
		HAL_Delay(1);
		XSPI_ReadSR(GD25X_ReadStatusReg1, &status);
	}
}

/**
  * @brief Init the external flash
  * @retval none
  */
void gd25q32c_Init(void)
{
	TUF2_LOG1("gd25q32c_Init()\r\n");

	XSPI_ResetDevice(&hxspi1);
	HAL_Delay(0); // 1ms wait device stable
	XSPI_ReadAllStatusReg();
	
	/* check if QE bit set and enable if not */
	if((gd25q32c_StatusReg[1] & 0x02) != 0x02)
	{
		TUF2_LOG1("\tSetting QE bit\r\n");
		uint8_t data = gd25q32c_StatusReg[1] | 0x02;
		XSPI_WriteEnable(&hxspi1);
		XSPI_Send_CMD(&hxspi1, GD25X_WriteStatusReg2, 0x00, 0,
			HAL_XSPI_ADDRESS_NONE, HAL_XSPI_DATA_1_LINE, 1);
		HAL_XSPI_Transmit(&hxspi1, &data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
		XSPI_Wait_Busy();
	}
}

/**
  * @brief  Initializes and configure the QSPI interface for memory map mode.
  * @retval QSPI status
  */
uint8_t gd25q32c_Startup(void)
{
	TUF2_LOG1("gd25q32c_Startup()...\n\r");
	
	/* Enable MemoryMapped mode */
	if( XSPI_EnableMemoryMappedMode(&hxspi1) != qspi_OK )
	{
		return qspi_ERROR;
	}

	return qspi_OK;
}

/**
  * @brief  Read an amount of data from the QSPI memory.
  * @param  pData Pointer to data to be read
  * @param  ReadAddr Read start address
  * @param  Size Size of data to write. Range 1 ~ W25qxx page size
  * @retval QSPI status
  */
uint8_t gd25q32c_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
	TUF2_LOG1("gd25q32c_Read()\r\n");
	if(XSPI_Send_CMD(&hxspi1, GD25X_QUAD_INOUT_FAST_READ_CMD,
		ReadAddr, GD25X_DUMMY_CYCLES_READ_QUAD,
		HAL_XSPI_ADDRESS_4_LINES, HAL_XSPI_DATA_4_LINES, Size) != qspi_OK)
	{
		return qspi_ERROR;
	}

	if(HAL_XSPI_Receive(&hxspi1, pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return qspi_ERROR;
	}
	
	return qspi_OK;
}

/**
  * @brief  Writes an amount of data to the OSPI memory.
  * @param  pData Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size Size of data to write. Range 1 ~ gd25q32 page size (256)
  * @retval QSPI status
  */
uint8_t gd25q32c_PageProgram(uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
	TUF2_LOG1("gd25q32c_PageProgram()\r\n");
	if(XSPI_WriteEnable(&hxspi1) != qspi_OK)
		return qspi_ERROR;
	
	if(XSPI_Send_CMD(&hxspi1, GD25X_QUAD_INPUT_PAGE_PROG_CMD, WriteAddr, 0,
		HAL_XSPI_ADDRESS_1_LINE, HAL_XSPI_DATA_4_LINES, Size) != qspi_OK)
		return qspi_ERROR;

	if(HAL_XSPI_Transmit(&hxspi1, pData, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		return qspi_ERROR;

	XSPI_Wait_Busy();

	return qspi_OK;
}

/**
  * @brief  Write buffer of data to flash
  * @param  pBuffer: pointer to write data
  * @param  WriteAddr: flash address to write to
  * @param  NumByteToWrite: quantity to write
  * @retval QSPI status
  */
uint8_t gd25q32c_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	TUF2_LOG1("gd25q32c_Write()\r\n");
	uint32_t end_addr = 0, current_size = 0, current_addr = 0;
	
	/* 
	 * Calculation of the size between the write address and the end of the page
	 */
	current_addr = (WriteAddr & ~0xff) + 256;
	current_size = current_addr - WriteAddr;

	/*
	 * Check if the size of the data is less than the remaining place in the page
	 */
	if (current_size > NumByteToWrite)
	{
		current_size = NumByteToWrite;
	}

	/* Initialize the address variables */
	current_addr = WriteAddr;
	end_addr = WriteAddr + NumByteToWrite;

	/* Perform the write page by page */
	do
	{
		if (current_size == 0)
		{
			return qspi_OK;
		}
		
		/* write the page */
		if(gd25q32c_PageProgram(pBuffer, current_addr, current_size) != qspi_OK)
		{
			return qspi_ERROR;
		}

		/* Update the address and size variables for next page programming */
		current_addr += current_size;
		pBuffer += current_size;
		current_size = ((current_addr + 256) > end_addr)
						   ? (end_addr - current_addr)
						   : 256;
	}
	while (current_addr <= end_addr);
	
	return qspi_OK;
}

/**
  * @brief  Whole chip erase.
  * @param  SectorAddress: Sector address to erase
  * @retval QSPI status
  */
uint8_t gd25q32c_EraseSector(uint32_t SectorAddress)
{
	/* mask off to max addr of flash chip */
	SectorAddress &= (BOARD_QSPI_FLASH_SIZE-1);
	
	TUF2_LOG1("gd25q32c_EraseSector() SectorAddress = 0x%08lX\r\n", SectorAddress);

	if(XSPI_WriteEnable(&hxspi1) != qspi_OK)
		return qspi_ERROR;
	
	if(XSPI_Send_CMD(&hxspi1, GD25X_SectorErase, SectorAddress, 0,
		HAL_XSPI_ADDRESS_1_LINE, HAL_XSPI_DATA_NONE, 0) != qspi_OK)
		return qspi_ERROR;

	XSPI_Wait_Busy();

	return qspi_OK;
}

/**
  * @brief  Whole chip erase.
  * @retval QSPI status
  */
uint8_t gd25q32c_EraseChip(void)
{
	TUF2_LOG1("gd25q32c_EraseChip()\r\n");
	if(XSPI_WriteEnable(&hxspi1) != qspi_OK)
		return qspi_ERROR;
	
	XSPI_Wait_Busy();

	if(XSPI_Send_CMD(&hxspi1, GD25X_ChipErase, 0x00, 0,
		HAL_XSPI_ADDRESS_NONE, HAL_XSPI_DATA_NONE, 0) != qspi_OK)
		return qspi_ERROR;

	XSPI_Wait_Busy();

	return qspi_OK;
}
