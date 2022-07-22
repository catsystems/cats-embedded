/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Handling of the node configuration protocol

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis, Gregory Cristian and Matthieu Verdy

Modified and adapted by Alessandro Carcione for ELRS project
*/

#include "SX1280Driver/SX1280_Regs.h"
#include "SX1280Driver/SX1280_hal.h"
#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

SX1280Hal *SX1280Hal::instance = NULL;

uint8_t OutBuffer[64];

SX1280Hal::SX1280Hal()
{
    instance = this;
}

void SX1280Hal::end()
{
    TXRXdisable(); // make sure the RX/TX amp pins are disabled
}

void SX1280Hal::init()
{

}

void SX1280Hal::reset(void)
{
	// We do not have this pin
}

void SX1280Hal::WriteCommand(SX1280_RadioCommands_t command, uint8_t val)
{
    WaitOnBusy();
    OutBuffer[0] = command;
    OutBuffer[1] = val;
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(RADIO_SPI, OutBuffer, 2, 5);

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    BusyDelay(12);
}

void SX1280Hal::WriteCommand(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    OutBuffer[0] = (uint8_t)command;
    memcpy(OutBuffer + 1, buffer, size);

    WaitOnBusy();
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(RADIO_SPI, OutBuffer, size+1, 5);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    BusyDelay(12);
}

void SX1280Hal::ReadCommand(SX1280_RadioCommands_t command, uint8_t *buffer, uint8_t size)
{
    #define RADIO_GET_STATUS_BUF_SIZEOF 3 // special case for command == SX1280_RADIO_GET_STATUS, fixed 3 bytes packet size

    WaitOnBusy();
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

    if (command == SX1280_RADIO_GET_STATUS)
    {
        OutBuffer[0] = (uint8_t)command;
        OutBuffer[1] = 0x00;
        OutBuffer[2] = 0x00;
        HAL_SPI_TransmitReceive(RADIO_SPI, OutBuffer, OutBuffer, RADIO_GET_STATUS_BUF_SIZEOF, 5);
        buffer[0] = OutBuffer[0];
    }
    else
    {
        OutBuffer[0] = (uint8_t)command;
        OutBuffer[1] = 0x00;
        memcpy(OutBuffer + 2, buffer, size);
        HAL_SPI_TransmitReceive(RADIO_SPI, OutBuffer, OutBuffer, size+2, 5);
        memcpy(buffer, OutBuffer + 2, size);
    }
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

void SX1280Hal::WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    OutBuffer[0] = (SX1280_RADIO_WRITE_REGISTER);
    OutBuffer[1] = ((address & 0xFF00) >> 8);
    OutBuffer[2] = (address & 0x00FF);

    memcpy(OutBuffer + 3, buffer, size);

    WaitOnBusy();
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(RADIO_SPI, OutBuffer, OutBuffer, size+3, 5);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    BusyDelay(12);
}

void SX1280Hal::WriteRegister(uint16_t address, uint8_t value)
{
    WriteRegister(address, &value, 1);
}

void SX1280Hal::ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size)
{
    OutBuffer[0] = (SX1280_RADIO_READ_REGISTER);
    OutBuffer[1] = ((address & 0xFF00) >> 8);
    OutBuffer[2] = (address & 0x00FF);
    OutBuffer[3] = 0x00;

    WaitOnBusy();
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(RADIO_SPI, OutBuffer, OutBuffer, size+4, 5);
    memcpy(buffer, OutBuffer + 4, size);

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

uint8_t SX1280Hal::ReadRegister(uint16_t address)
{
    uint8_t data;
    ReadRegister(address, &data, 1);
    return data;
}

void SX1280Hal::WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    OutBuffer[0] = SX1280_RADIO_WRITE_BUFFER;
    OutBuffer[1] = offset;

    for(int i = 0; i < size; i++){
    	OutBuffer[i+2] = buffer[i];
    }

    WaitOnBusy();

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(RADIO_SPI, OutBuffer, OutBuffer, size+2, 5);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

}

void SX1280Hal::ReadBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size)
{
    OutBuffer[0] = SX1280_RADIO_READ_BUFFER;
    OutBuffer[1] = offset;
    OutBuffer[2] = 0x00;

    WaitOnBusy();
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(RADIO_SPI, OutBuffer, OutBuffer, size+3, 5);

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    for(int i = 0; i < size; i++){
    	buffer[i] = OutBuffer[i+3];
    }
}

bool SX1280Hal::WaitOnBusy()
{
    uint32_t startTime = HAL_GetTick();

    while (HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin)) // wait until not busy or until timeout
    {
        if ((HAL_GetTick() - startTime) >= 2) // 2ms timeout
        {
            return false;
        }
        else
        {
        	asm("NOP");
        }
    }
    return true;
}

void SX1280Hal::TXenable()
{
	// Enable Front End
	HAL_GPIO_WritePin(FE_EN_GPIO_Port, FE_EN_Pin, GPIO_PIN_SET);

	// Enable TX
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_SET);

	// Disable RX
	HAL_GPIO_WritePin(RX_EN_GPIO_Port, RX_EN_Pin, GPIO_PIN_RESET);
}

void SX1280Hal::RXenable()
{
	// Enable Front End
	HAL_GPIO_WritePin(FE_EN_GPIO_Port, FE_EN_Pin, GPIO_PIN_SET);

	// Disable TX
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);

	// Enable RX
	HAL_GPIO_WritePin(RX_EN_GPIO_Port, RX_EN_Pin, GPIO_PIN_SET);
}

void SX1280Hal::TXRXdisable()
{
	// Disable Front End
	HAL_GPIO_WritePin(FE_EN_GPIO_Port, FE_EN_Pin, GPIO_PIN_RESET);

	// Disable TX
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);

	// Disable RX
	HAL_GPIO_WritePin(RX_EN_GPIO_Port, RX_EN_Pin, GPIO_PIN_RESET);
}

// EXTI External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == DIO1_Pin) //
    {
    	SX1280Driver::IsrCallback();
    	//if (SX1280Hal::RadioIsrCallback) SX1280Hal::RadioIsrCallback();
    }
}


