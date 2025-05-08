#include "gpio.h"

void MySPI_W_SS(uint8_t BitValue)
{
	HAL_GPIO_WritePin(spi_ss_GPIO_Port, spi_ss_Pin, (GPIO_PinState)BitValue);
}

void MySPI_W_SCK(uint8_t BitValue)
{
	HAL_GPIO_WritePin(spi_clk_GPIO_Port, spi_clk_Pin, (GPIO_PinState)BitValue);
}

void MySPI_W_MOSI(uint8_t BitValue)
{
    HAL_GPIO_WritePin(spi_mosi_GPIO_Port, spi_mosi_Pin, (GPIO_PinState)BitValue);
}

uint8_t MySPI_R_MISO(void)
{
	return HAL_GPIO_ReadPin(spi_miso_GPIO_Port, spi_miso_Pin);
}

void MySPI_Start(void)
{
	MySPI_W_SS(0);
}

void MySPI_Stop(void)
{
	MySPI_W_SS(1);
}

uint8_t MySPI_SwapByte(uint8_t ByteSend)
{
	uint8_t i, ByteReceive = 0x00;
	
	for (i = 0; i < 8; i ++)
	{
		MySPI_W_MOSI(ByteSend & (0x80 >> i));
		MySPI_W_SCK(1);
		if (MySPI_R_MISO() == 1){ByteReceive |= (0x80 >> i);}
		MySPI_W_SCK(0);
	}
	
	return ByteReceive;
}
