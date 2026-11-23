#pragma once
#include <stdint.h>
extern uint8_t  g_framMem[];
extern int      g_framMinFree;
extern uint32_t g_framWrites;
void fakeFramDropQueue();
int  fakeFramQueued();
