#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "protocol.h"

typedef struct __attribute__((__packed__)) //0x01
{
    float vx;
    float vy;
    float yaw;
} REMOTE_t;

typedef struct __attribute__((__packed__)) //0x02
{
    float vx;
    float vy;
    float yaw;
} FEEDBACK_t;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

void Get_REMOTE(float *vx, float *vy, float *yaw);
void Get_FEEDBACK(float *vx, float *vy, float *yaw);

extern uint8_t get_robot_id(void);

#endif
