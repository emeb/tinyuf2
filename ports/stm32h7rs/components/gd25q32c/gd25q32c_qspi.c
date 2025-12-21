#include "gd25q32c_qspi.h"
#include "qspi_status.h"
#include "stm32h7rsxx_hal.h"


extern XSPI_HandleTypeDef hxspi1;

static uint32_t XSPI_EnableMemoryMappedMode(XSPI_HandleTypeDef *hqspi,uint8_t DTRMode);
static uint32_t XSPI_ResetDevice(XSPI_HandleTypeDef *hqspi);
static uint8_t XSPI_EnterQPI(XSPI_HandleTypeDef *hqspi);
static uint32_t XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hqspi, uint32_t Timeout);
static uint32_t XSPI_WriteEnable(XSPI_HandleTypeDef *hqspi);
static uint32_t XSPI_EnterFourBytesAddress(XSPI_HandleTypeDef *hqspi);

static uint8_t XSPI_Send_CMD(XSPI_HandleTypeDef *hqspi,uint32_t instruction, uint32_t address,uint32_t addressSize,uint32_t dummyCycles,
                    uint32_t instructionMode,uint32_t addressMode, uint32_t dataMode, uint32_t dataSize);

qspi_StatusTypeDef gd25q32c_Mode = qspi_SPIMode;
uint8_t gd25q32c_StatusReg[3];
uint16_t gd25q32c_ID;

void gd25q32c_Init(void)
{
  XSPI_ResetDevice(&hxspi1);
  HAL_Delay(0); // 1ms wait device stable
  gd25q32c_ID = gd25q32c_GetID();
  gd25q32c_ReadAllStatusReg();
}

uint16_t gd25q32c_GetID(void)
{
  uint8_t ID[6];
  uint16_t deviceID;

  if(gd25q32c_Mode == qspi_SPIMode)
    XSPI_Send_CMD(&hxspi1,GD25X_QUAD_ManufactDeviceID,0x00,HAL_XSPI_ADDRESS_24_BITS,6,HAL_XSPI_INSTRUCTION_1_LINE,HAL_XSPI_ADDRESS_4_LINES, HAL_XSPI_DATA_4_LINES, sizeof(ID));
  else
    XSPI_Send_CMD(&hxspi1,GD25X_ManufactDeviceID,0x00,HAL_XSPI_ADDRESS_24_BITS,0,HAL_XSPI_INSTRUCTION_4_LINES,HAL_XSPI_ADDRESS_4_LINES, HAL_XSPI_DATA_4_LINES, sizeof(ID));

  /* Reception of the data */
  if (HAL_XSPI_Receive(&hxspi1, ID, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return qspi_ERROR;
  }
  deviceID = (ID[0] << 8) | ID[1];

  return deviceID;
}

uint8_t gd25q32c_ReadSR(uint8_t SR)
{
  uint8_t byte=0;
  if(gd25q32c_Mode == qspi_SPIMode)
    XSPI_Send_CMD(&hxspi1,SR,0x00,HAL_XSPI_ADDRESS_8_BITS,0,HAL_XSPI_INSTRUCTION_1_LINE,HAL_XSPI_ADDRESS_NONE, HAL_XSPI_DATA_1_LINE, 1);
  else
    XSPI_Send_CMD(&hxspi1,SR,0x00,HAL_XSPI_ADDRESS_8_BITS,0,HAL_XSPI_INSTRUCTION_4_LINES,HAL_XSPI_ADDRESS_NONE, HAL_XSPI_DATA_4_LINES, 1);

  if (HAL_XSPI_Receive(&hxspi1,&byte,HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {

  }
  return byte;
}

uint8_t gd25q32c_WriteSR(uint8_t SR,uint8_t data)
{
  if(gd25q32c_Mode == qspi_SPIMode)
    XSPI_Send_CMD(&hxspi1,SR,0x00,HAL_XSPI_ADDRESS_8_BITS,0,HAL_XSPI_INSTRUCTION_1_LINE,HAL_XSPI_ADDRESS_NONE, HAL_XSPI_DATA_1_LINE, 1);
  else
    XSPI_Send_CMD(&hxspi1,SR,0x00,HAL_XSPI_ADDRESS_8_BITS,0,HAL_XSPI_INSTRUCTION_4_LINES,HAL_XSPI_ADDRESS_NONE, HAL_XSPI_DATA_4_LINES, 1);

  return HAL_XSPI_Transmit(&hxspi1,&data,HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

uint8_t gd25q32c_ReadAllStatusReg(void)
{
  gd25q32c_StatusReg[0] = gd25q32c_ReadSR(GD25X_ReadStatusReg1);
  gd25q32c_StatusReg[1] = gd25q32c_ReadSR(GD25X_ReadStatusReg2);
  gd25q32c_StatusReg[2] = gd25q32c_ReadSR(GD25X_ReadStatusReg3);
  return qspi_OK;
}

//等待空闲
void gd25q32c_Wait_Busy(void)
{
  while((gd25q32c_ReadSR(GD25X_ReadStatusReg1) & 0x01) == 0x01);
}

// Only use in QPI mode
uint8_t gd25q32c_SetReadParameters(uint8_t DummyClock,uint8_t WrapLenth)
{
  uint8_t send;
  send = (DummyClock/2 -1)<<4 | ((WrapLenth/8 - 1)&0x03);

  gd25q32c_WriteEnable();

  XSPI_Send_CMD(&hxspi1,GD25X_SetReadParam,0x00,HAL_XSPI_ADDRESS_8_BITS,0,HAL_XSPI_INSTRUCTION_4_LINES,HAL_XSPI_ADDRESS_NONE, HAL_XSPI_DATA_4_LINES, 1);

  return HAL_XSPI_Transmit(&hxspi1,&send,HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

uint8_t gd25q32c_EnterQPI(void)
{
  /* Enter QSPI memory in QSPI mode */
  if(XSPI_EnterQPI(&hxspi1) != qspi_OK)
  {
    return qspi_ERROR;
  }

  return gd25q32c_SetReadParameters(8,8);
}


/**
  * @brief  Initializes and configure the QSPI interface.
  * @retval QSPI memory status
  */
uint8_t gd25q32c_Startup(uint8_t DTRMode)
{
  /* Enable MemoryMapped mode */
  if( XSPI_EnableMemoryMappedMode(&hxspi1,DTRMode) != qspi_OK )
  {
    return qspi_ERROR;
  }
  return qspi_OK;
}

uint8_t gd25q32c_WriteEnable(void)
{
  return XSPI_WriteEnable(&hxspi1);
}
/**
  * @brief  Erase 4KB Sector of the OSPI memory.
  * @param  SectorAddress: Sector address to erase
  * @retval QSPI memory status
  */
uint8_t gd25q32c_EraseSector(uint32_t SectorAddress)
{
  uint8_t result;

  gd25q32c_WriteEnable();
  gd25q32c_Wait_Busy();

  if(gd25q32c_Mode == qspi_SPIMode)
    result = XSPI_Send_CMD(&hxspi1,GD25X_SectorErase,SectorAddress,HAL_XSPI_ADDRESS_24_BITS,0,HAL_XSPI_INSTRUCTION_1_LINE,HAL_XSPI_ADDRESS_1_LINE,HAL_XSPI_DATA_NONE,0);
  else
    result = XSPI_Send_CMD(&hxspi1,GD25X_SectorErase,SectorAddress,HAL_XSPI_ADDRESS_24_BITS,0,HAL_XSPI_INSTRUCTION_4_LINES,HAL_XSPI_ADDRESS_4_LINES,HAL_XSPI_DATA_NONE,0);

  /* 等待擦除完成 */
  if(result == qspi_OK)
    gd25q32c_Wait_Busy();

  return result;
}

/**
  * @brief  Erase 64KB Sector of the OSPI memory.
  * @param  SectorAddress: Sector address to erase
  * @retval QSPI memory status
  */
uint8_t gd25q32c_EraseBlock(uint32_t BlockAddress)
{
  uint8_t result;

  gd25q32c_WriteEnable();
  gd25q32c_Wait_Busy();

  if(gd25q32c_Mode == qspi_SPIMode)
    result = XSPI_Send_CMD(&hxspi1,GD25X_BlockErase,BlockAddress,HAL_XSPI_ADDRESS_24_BITS,0,HAL_XSPI_INSTRUCTION_1_LINE,HAL_XSPI_ADDRESS_1_LINE,HAL_XSPI_DATA_NONE,0);
  else
    result = XSPI_Send_CMD(&hxspi1,GD25X_BlockErase,BlockAddress,HAL_XSPI_ADDRESS_24_BITS,0,HAL_XSPI_INSTRUCTION_4_LINES,HAL_XSPI_ADDRESS_4_LINES,HAL_XSPI_DATA_NONE,0);

  /* 等待擦除完成 */
  if(result == qspi_OK)
    gd25q32c_Wait_Busy();

  return result;
}

/**
  * @brief  Whole chip erase.
  * @param  SectorAddress: Sector address to erase
  * @retval QSPI memory status
  */
uint8_t gd25q32c_EraseChip(void)
{
  uint8_t result;

  gd25q32c_WriteEnable();
  gd25q32c_Wait_Busy();

  if(gd25q32c_Mode == qspi_SPIMode)
    result = XSPI_Send_CMD(&hxspi1,GD25X_ChipErase,0x00,HAL_XSPI_ADDRESS_8_BITS,0,HAL_XSPI_INSTRUCTION_1_LINE,HAL_XSPI_ADDRESS_NONE,HAL_XSPI_DATA_NONE,0);
  else
    result = XSPI_Send_CMD(&hxspi1,GD25X_ChipErase,0x00,HAL_XSPI_ADDRESS_8_BITS,0,HAL_XSPI_INSTRUCTION_4_LINES,HAL_XSPI_ADDRESS_NONE,HAL_XSPI_DATA_NONE,0);

  /* 等待擦除完成 */
  if(result == qspi_OK)
    gd25q32c_Wait_Busy();

  return result;
}

/**
  * @brief  Writes an amount of data to the OSPI memory.
  * @param  pData Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size Size of data to write. Range 1 ~ W25qxx page size
  * @retval QSPI memory status
  */
uint8_t gd25q32c_PageProgram(uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  uint8_t result;

  gd25q32c_WriteEnable();

  if(gd25q32c_Mode == qspi_SPIMode)
    result = XSPI_Send_CMD(&hxspi1,GD25X_QUAD_INPUT_PAGE_PROG_CMD,WriteAddr,HAL_XSPI_ADDRESS_24_BITS,0,HAL_XSPI_INSTRUCTION_1_LINE,HAL_XSPI_ADDRESS_1_LINE,HAL_XSPI_DATA_4_LINES,Size);
  else
    result = XSPI_Send_CMD(&hxspi1,GD25X_PageProgram,WriteAddr,HAL_XSPI_ADDRESS_24_BITS,0,HAL_XSPI_INSTRUCTION_4_LINES,HAL_XSPI_ADDRESS_4_LINES,HAL_XSPI_DATA_4_LINES,Size);

  if(result == qspi_OK)
    result = HAL_XSPI_Transmit(&hxspi1,pData,HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

  /* 等待写入完成 */
  if(result == qspi_OK)
    gd25q32c_Wait_Busy();

  return result;
}

//读取SPI FLASH,仅支持QPI模式
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(最大32bit)
//NumByteToRead:要读取的字节数(最大65535)
uint8_t gd25q32c_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  uint8_t result;

  XSPI_RegularCmdTypeDef      s_command;

  /* Configure the command for the read instruction */

  if(gd25q32c_Mode == qspi_QPIMode)
  {
    s_command.Instruction     = GD25X_QUAD_INOUT_FAST_READ_CMD;
    s_command.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
    s_command.DummyCycles     = GD25X_DUMMY_CYCLES_READ_QUAD;
  }
  else
  {
    s_command.Instruction     = GD25X_QUAD_INOUT_FAST_READ_CMD;
    s_command.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
    s_command.DummyCycles     = GD25X_DUMMY_CYCLES_READ_QUAD-2;
  }

  s_command.Address           = ReadAddr;
  s_command.AddressMode       = HAL_XSPI_ADDRESS_4_LINES;
  s_command.AddressWidth      = HAL_XSPI_ADDRESS_24_BITS;

  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_4_LINES;
  s_command.AlternateBytes    = 0xFF;
  s_command.AlternateBytesWidth = HAL_XSPI_ALT_BYTES_8_BITS;

  s_command.DataMode          = HAL_XSPI_DATA_4_LINES;
  s_command.DataLength        = Size;

  s_command.DQSMode           = HAL_XSPI_DQS_DISABLE;

  //s_command.DdrHoldHalfCycle  = XSPI_DDR_HHC_ANALOG_DELAY;
  //s_command.SIOOMode          = XSPI_SIOO_INST_EVERY_CMD;

  result = HAL_XSPI_Command(&hxspi1, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

  if(result == qspi_OK)
    result = HAL_XSPI_Receive(&hxspi1,pData,HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

  return result;
}

//无检验写SPI FLASH
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(最大32bit)
//NumByteToWrite:要写入的字节数(最大65535)
//CHECK OK
void gd25q32c_WriteNoCheck(uint8_t *pBuffer,uint32_t WriteAddr,uint32_t NumByteToWrite)
{
  uint16_t pageremain;
  pageremain = 256 - WriteAddr % 256; //单页剩余的字节数
  if (NumByteToWrite <= pageremain)
  {
    pageremain = NumByteToWrite; //不大于256个字节
  }
  while(1)
  {
    gd25q32c_PageProgram(pBuffer, WriteAddr, pageremain);
    if (NumByteToWrite == pageremain)
    {
      break; //写入结束了
    }
     else //NumByteToWrite>pageremain
    {
      pBuffer += pageremain;
      WriteAddr += pageremain;

      NumByteToWrite -= pageremain; //减去已经写入了的字节数
      if (NumByteToWrite > 256)
        pageremain = 256; //一次可以写入256个字节
      else
        pageremain = NumByteToWrite; //不够256个字节了
    }
  }
}

//写SPI FLASH
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(最大32bit)
//NumByteToWrite:要写入的字节数(最大65535)
uint8_t gd25q32c_Write(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
  uint32_t secpos;
  uint16_t secoff;
  uint16_t secremain;
   uint16_t i;
  uint8_t gd25q32c_BUF[4096];

   secpos = WriteAddr / 4096; //扇区地址
  secoff = WriteAddr % 4096; //在扇区内的偏移
  secremain = 4096 - secoff; //扇区剩余空间大小

   if (NumByteToWrite <= secremain) secremain = NumByteToWrite; //不大于4096个字节
  while(1)
  {
    if (gd25q32c_Read(gd25q32c_BUF, secpos * 4096, 4096) != qspi_OK) {
      return qspi_ERROR;
    } //读出整个扇区的内容
    for (i = 0;i < secremain; i++) //校验数据
    {
      if (gd25q32c_BUF[secoff+i] != 0XFF) break; //需要擦除
    }
    if (i < secremain) //需要擦除
    {
      if (gd25q32c_EraseSector(secpos) != qspi_OK) {
        return qspi_ERROR;
      } //擦除这个扇区
      for (i = 0; i < secremain; i++) //复制
      {
        gd25q32c_BUF[i + secoff] = pBuffer[i];
      }
      gd25q32c_WriteNoCheck(gd25q32c_BUF, secpos * 4096, 4096); //写入整个扇区
    }
    else
    {
      gd25q32c_WriteNoCheck(pBuffer, WriteAddr, secremain); //写已经擦除了的,直接写入扇区剩余区间.
    }
    if (NumByteToWrite == secremain)
    {
      break; //写入结束了
    }
    else//写入未结束
    {
      secpos++; //扇区地址增1
      secoff = 0; //偏移位置为0

      pBuffer += secremain;  //指针偏移
      WriteAddr += secremain;//写地址偏移
      NumByteToWrite -= secremain; //字节数递减
      if (NumByteToWrite > 4096)
        secremain = 4096; //下一个扇区还是写不完
      else
        secremain = NumByteToWrite; //下一个扇区可以写完了
    }
  }
  return qspi_OK;
}

/**
  * @brief  Configure the QSPI in memory-mapped mode   QPI/SPI && DTR(DDR)/Normal Mode
  * @param  hqspi: QSPI handle
  * @param  DTRMode: qspi_DTRMode DTR mode ,qspi_NormalMode Normal mode
  * @retval QSPI memory status
  */
static uint32_t XSPI_EnableMemoryMappedMode(XSPI_HandleTypeDef *hqspi,uint8_t DTRMode)
{
  XSPI_RegularCmdTypeDef   s_command;
  XSPI_MemoryMappedTypeDef s_mem_mapped_cfg;

  /* Configure the command for the read instruction */
  if(gd25q32c_Mode == qspi_QPIMode)
    s_command.InstructionMode   = HAL_XSPI_INSTRUCTION_4_LINES;
  else
    s_command.InstructionMode   = HAL_XSPI_INSTRUCTION_1_LINE;

  s_command.AddressMode       = HAL_XSPI_ADDRESS_4_LINES;
  s_command.Address           = 0;
  s_command.AddressWidth      = HAL_XSPI_ADDRESS_24_BITS;

  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_4_LINES;
  s_command.AlternateBytes    = 0xEF;
  s_command.AlternateBytesWidth = HAL_XSPI_ALT_BYTES_8_BITS;

  s_command.DataMode          = HAL_XSPI_DATA_4_LINES;

  if(DTRMode == qspi_DTRMode)
  {
    s_command.Instruction     = GD25X_QUAD_INOUT_FAST_READ_DTR_CMD;
    s_command.DummyCycles     = GD25X_DUMMY_CYCLES_READ_QUAD_DTR;
    s_command.DQSMode         = HAL_XSPI_DQS_ENABLE;
  }
  else
  {
    s_command.Instruction     = GD25X_QUAD_INOUT_FAST_READ_CMD;

    if(gd25q32c_Mode == qspi_QPIMode)
      s_command.DummyCycles   = GD25X_DUMMY_CYCLES_READ_QUAD;
    else
      s_command.DummyCycles   = GD25X_DUMMY_CYCLES_READ_QUAD-2;

    s_command.DQSMode         = HAL_XSPI_DQS_DISABLE;
  }

  //s_command.DdrHoldHalfCycle  = XSPI_DDR_HHC_ANALOG_DELAY;
  //s_command.SIOOMode          = XSPI_SIOO_INST_ONLY_FIRST_CMD;

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;
  s_mem_mapped_cfg.TimeoutPeriodClock = 0;

  if (HAL_XSPI_MemoryMapped(hqspi, &s_mem_mapped_cfg) != HAL_OK)
  {
    return qspi_ERROR;
  }

  return qspi_OK;
}

/**
  * @brief  This function reset the QSPI memory.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint32_t XSPI_ResetDevice(XSPI_HandleTypeDef *hqspi)
{
  XSPI_RegularCmdTypeDef s_command;

  /* Initialize the reset enable command */
  s_command.InstructionMode   = HAL_XSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = GD25X_EnableReset;
  s_command.AddressMode       = HAL_XSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode          = HAL_XSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DQSMode           = HAL_XSPI_DQS_DISABLE;
  //s_command.DdrHoldHalfCycle  = HAL_XSPI_DDR_HHC_ANALOG_DELAY;
  //s_command.SIOOMode          = XSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_XSPI_Command(hqspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return qspi_ERROR;
  }

  /* Send the reset device command */
  s_command.Instruction = GD25X_ResetDevice;
  if (HAL_XSPI_Command(hqspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return qspi_ERROR;
  }

  s_command.InstructionMode   = HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = GD25X_EnableReset;
  /* Send the command */
  if (HAL_XSPI_Command(hqspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return qspi_ERROR;
  }

  /* Send the reset memory command */
  s_command.Instruction = GD25X_ResetDevice;
  if (HAL_XSPI_Command(hqspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return qspi_ERROR;
  }

  gd25q32c_Mode = qspi_SPIMode;
  return qspi_OK;
}

/**
 * @brief	QSPI发送命令
 *
 * @param   instruction		要发送的指令
 * @param   address			发送到的目的地址
 * @param   addressSize	发送到的目的地址大小
 * @param   dummyCycles		空指令周期数
 * @param   instructionMode		指令模式;
 * @param   addressMode		地址模式; HAL_XSPI_ADDRESS_NONE,XSPI_ADDRESS_1_LINE,XSPI_ADDRESS_2_LINES,HAL_XSPI_ADDRESS_4_LINES
 * @param   dataMode		数据模式; XSPI_DATA_NONE,HAL_XSPI_DATA_1_LINEXSPI_DATA_2_LINES,HAL_XSPI_DATA_4_LINES
 * @param   dataSize        待传输的数据长度
 *
 * @return  uint8_t			qspi_OK:正常
 *                      qspi_ERROR:错误
 */
static uint8_t XSPI_Send_CMD(XSPI_HandleTypeDef *hqspi,uint32_t instruction, uint32_t address,uint32_t addressSize,uint32_t dummyCycles,
                    uint32_t instructionMode,uint32_t addressMode, uint32_t dataMode, uint32_t dataSize)
{
    XSPI_RegularCmdTypeDef Cmdhandler;

    Cmdhandler.Instruction        = instruction;
    Cmdhandler.InstructionMode    = instructionMode;

    Cmdhandler.Address            = address;
    Cmdhandler.AddressWidth       = addressSize;
    Cmdhandler.AddressMode        = addressMode;

    Cmdhandler.AlternateBytes     = 0x00;
    Cmdhandler.AlternateBytesWidth = HAL_XSPI_ALT_BYTES_8_BITS;
    Cmdhandler.AlternateBytesMode  = HAL_XSPI_ALT_BYTES_NONE;
    Cmdhandler.DummyCycles        = dummyCycles;

    Cmdhandler.DataMode           = dataMode;
    Cmdhandler.DataLength         = dataSize;

    Cmdhandler.DQSMode            = HAL_XSPI_DQS_DISABLE;
    //Cmdhandler.DdrHoldHalfCycle   = XSPI_DDR_HHC_ANALOG_DELAY;
    //Cmdhandler.SIOOMode           = XSPI_SIOO_INST_EVERY_CMD;

    if(HAL_XSPI_Command(hqspi, &Cmdhandler, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
      return qspi_ERROR;

    return qspi_OK;
}

/**
  * @brief  This function set the QSPI memory in 4-byte address mode
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint32_t XSPI_EnterFourBytesAddress(XSPI_HandleTypeDef *hqspi)
{
  XSPI_RegularCmdTypeDef s_command;

  /* Initialize the command */
  s_command.InstructionMode   = HAL_XSPI_INSTRUCTION_4_LINES;
  s_command.Instruction       = GD25X_Enable4ByteAddr;
  s_command.AddressMode       = HAL_XSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode          = HAL_XSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DQSMode           = HAL_XSPI_DQS_DISABLE;
  //s_command.DdrHoldHalfCycle  = XSPI_DDR_HHC_ANALOG_DELAY;
  //s_command.SIOOMode          = XSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (XSPI_WriteEnable(hqspi) != qspi_OK)
  {
    return qspi_ERROR;
  }

  /* Send the command */
  if (HAL_XSPI_Command(hqspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return qspi_ERROR;
  }

  /* Configure automatic polling mode to wait the memory is ready */
  if (XSPI_AutoPollingMemReady(hqspi, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != qspi_OK)
  {
    return qspi_ERROR;
  }

  return qspi_OK;
}

/**
  * @brief  This function send a Write Enable and wait it is effective.
  * @param  hqspi: QSPI handle
  * @retval None
  */
static uint32_t XSPI_WriteEnable(XSPI_HandleTypeDef *hqspi)
{
  XSPI_RegularCmdTypeDef  s_command;
  XSPI_AutoPollingTypeDef s_config;

  /* Enable write operations */
  if(gd25q32c_Mode == qspi_QPIMode)
    s_command.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
  else
    s_command.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;

  s_command.Instruction       = GD25X_WriteEnable;
  s_command.AddressMode       = HAL_XSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
  s_command.DataMode          = HAL_XSPI_DATA_NONE;
  s_command.DummyCycles       = 0;
  s_command.DQSMode           = HAL_XSPI_DQS_DISABLE;
  //s_command.DdrHoldHalfCycle  = XSPI_DDR_HHC_ANALOG_DELAY;
  //s_command.SIOOMode          = XSPI_SIOO_INST_EVERY_CMD;

  if (HAL_XSPI_Command(hqspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return qspi_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling */
  s_config.MatchValue      = GD25X_SR_WREN;
  s_config.MatchMask       = GD25X_SR_WREN;
  s_config.MatchMode       = HAL_XSPI_MATCH_MODE_AND;
  s_config.IntervalTime    = 0x10;
  s_config.AutomaticStop   = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

  s_command.Instruction    = GD25X_ReadStatusReg1;

  if(gd25q32c_Mode == qspi_QPIMode)
    s_command.DataMode     = HAL_XSPI_DATA_4_LINES;
  else
    s_command.DataMode     = HAL_XSPI_DATA_1_LINE;

  if (HAL_XSPI_AutoPolling(hqspi, &s_config, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return qspi_ERROR;
  }

  return qspi_OK;
}

/**
  * @brief  This function read the SR of the memory and wait the EOP.
  * @param  hqspi: QSPI handle
  * @param  Timeout
  * @retval None
  */
static uint32_t XSPI_AutoPollingMemReady(XSPI_HandleTypeDef *hqspi, uint32_t Timeout)
{
  XSPI_RegularCmdTypeDef  s_command;
  XSPI_AutoPollingTypeDef s_config;

  /* Configure automatic polling mode to wait for memory ready */

  if(gd25q32c_Mode == qspi_SPIMode)
    s_command.InstructionMode   = HAL_XSPI_INSTRUCTION_1_LINE;
  else
    s_command.InstructionMode   = HAL_XSPI_INSTRUCTION_4_LINES;

  s_command.Instruction       = GD25X_ReadStatusReg1;

  s_command.AddressMode       = HAL_XSPI_ADDRESS_NONE;
  s_command.Address           = 0x00;
  s_command.AddressWidth      = HAL_XSPI_ADDRESS_8_BITS;
  s_command.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;

  if(gd25q32c_Mode == qspi_SPIMode)
    s_command.DataMode        = HAL_XSPI_DATA_1_LINE;
  else
    s_command.DataMode        = HAL_XSPI_DATA_4_LINES;
  s_command.DummyCycles       = 0;
  s_command.DQSMode           = HAL_XSPI_DQS_DISABLE;
  //s_command.DdrHoldHalfCycle  = XSPI_DDR_HHC_ANALOG_DELAY;
  //s_command.SIOOMode          = XSPI_SIOO_INST_EVERY_CMD;

  s_config.MatchValue      = 0;
  s_config.MatchMask       = GD25X_SR_WIP;
  s_config.MatchMode       = HAL_XSPI_MATCH_MODE_AND;
  s_config.IntervalTime    = 0x10;
  s_config.AutomaticStop   = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

  return HAL_XSPI_AutoPolling(hqspi, &s_config, Timeout);

}

/**
  * @brief  This function enter the QSPI memory in QPI mode
  * @param  hqspi QSPI handle
  * @retval QSPI status
  */
static uint8_t XSPI_EnterQPI(XSPI_HandleTypeDef *hqspi)
{
  uint8_t stareg2;
  stareg2 = gd25q32c_ReadSR(GD25X_ReadStatusReg2);
  if((stareg2 & 0X02) == 0) //QE位未使能
  {
    gd25q32c_WriteEnable();
    stareg2 |= 1<<1; //使能QE位
    gd25q32c_WriteSR(GD25X_WriteStatusReg2,stareg2);
  }
  XSPI_Send_CMD(hqspi,GD25X_EnterQSPIMode,0x00,HAL_XSPI_ADDRESS_8_BITS,0,HAL_XSPI_INSTRUCTION_1_LINE,HAL_XSPI_ADDRESS_NONE,HAL_XSPI_DATA_NONE,0);

  /* Configure automatic polling mode to wait the memory is ready */
  if (XSPI_AutoPollingMemReady(hqspi, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != qspi_OK)
  {
    return qspi_ERROR;
  }

  gd25q32c_Mode = qspi_QPIMode;

  return qspi_OK;
}
