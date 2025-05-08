#include "Blueteeth.h"
#include "PID.h"
#include "Position.h"



uint8_t rxbuffer[rxsize];
rxpack rx_uart3;

uint8_t magstopflag;
uint8_t manipulationflag = 1;
uint8_t navigationflag;
uint8_t positionfalg;

void rxpin_init(void)
{
    rx_uart3.input = rx_uart3.rxpinarr;
    rx_uart3.output = rx_uart3.rxpinarr;
    rx_uart3.top = &rx_uart3.rxpinarr[9];
    rx_uart3.input->start = rxbuffer;
    rx_uart3.output->end = rxbuffer;
    rx_uart3.rxcount = 0;
}


void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART3)
    {
        rx_uart3.input->end = &rxbuffer[rx_uart3.rxcount-1];
        rx_uart3.input++;
        if(rx_uart3.input == rx_uart3.top)
        {
            rx_uart3.input = rx_uart3.rxpinarr;
        }
        if(rxsize - rx_uart3.rxcount <= rxmax)
        {
            rx_uart3.input->start = rxbuffer;
            rx_uart3.rxcount = 0;
        }
        else
        {
            rx_uart3.input->start = &rxbuffer[rx_uart3.rxcount];
        }
        HAL_UART_Receive_DMA(&huart3, rx_uart3.input->start, rxmax);
    }
}

void rxdata_deal(void)
{
    if(rx_uart3.output != rx_uart3.input)
    {
        if(rx_uart3.output->end - rx_uart3.output->start == 0)
        {
            switch(*rx_uart3.output->start)
            {
                case 0x01:
                {
                    target_velocity += 4;
                    target_velocity>52 ? target_velocity=52 : 0;
                    break;
                    
                }
                case 0x02:
                {
                    target_velocity -= 4;
                    target_velocity<-52 ? target_velocity=-52 : 0;
                    break;
                }
                case 0x03:
                {
                    target_turn -= 10;
                    target_turn<=-180 ? target_turn = -target_turn : 0;
                    break;
                }
                case 0x04:
                {
                    target_turn += 10;
                    target_turn>=180 ? target_turn = -target_turn : 0;
                    break;
                }
                case 0x05:
                {
                    target_velocity = 0;
                    break;
                }
                case 0x06:
                {
                    magstopflag = 1;
                    break;
                }
                case 0x07:
                {
                    manipulationflag = 1;
                    navigationflag = 0;
                    break;
                }
                case 0x08:
                {
                    navigationflag = 1;
                    manipulationflag = 0;
                    break;
                }
                case 0x09:
                {
                    positionfalg = 1;
                    break;
                }
                case 0xA0:
                {
                    positionfalg = 0;
                    break;
                }
                case 0xB0:
                {
//                    printf("%f   %f\r\n", px, py);
//                    printf("%f", ki_distance);
                    break;
                }
            }
        }
        else if(navigationflag == 1 && *rx_uart3.output->start == '(' && *rx_uart3.output->end == ')')
        {
            distanceflag = 1;
            sscanf((char*)rx_uart3.output->start+1, "%f,%f", &target_x, &target_y);
            target_turn = atan2(target_x-px, target_y-py)* 57.296;
        }
        rx_uart3.output++;
        if(rx_uart3.output == rx_uart3.top)
            rx_uart3.output = rx_uart3.rxpinarr;
    }
    
}