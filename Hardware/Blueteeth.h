#ifndef __BLUETEETH_H
#define __BLUETEETH_H

#include "usart.h"
#include "dma.h"
#include "string.h"

#define rxsize    100
#define txsize    100
#define rxmax     20

typedef struct rxpin
{
    uint8_t* start;
    uint8_t* end;
}rxpin;

typedef struct txpin
{
    uint8_t* start;
    uint8_t* end;
}txpin;

typedef struct rxpack
{
    rxpin rxpinarr[10];
    rxpin* input;
    rxpin* output;
    rxpin* top;
    uint32_t rxcount;
}rxpack;

typedef struct txpack
{
    txpin txpinarr[10];
    txpin* input;
    txpin* output;
    txpin* top;
    uint32_t inputcount;
    uint8_t busyflag;
}txpack;

extern DMA_HandleTypeDef hdma_usart3_rx;
extern rxpack rx_uart3;
extern uint8_t magstopflag;
extern uint8_t manipulationflag;
extern uint8_t navigationflag;
extern uint8_t positionfalg;

void rxpin_init(void);
void rxdata_deal(void);

#endif
