/***************************************************************************************************
* @file     gm_timer.h
* @brief    Statically allocated simple software timer module
* @author   Giuliano Motter
* @date     08/2020
* @details  github.com/TSMotter
***************************************************************************************************/

#ifndef __GM_TIMER_H__
#define __GM_TIMER_H__

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
/***************************************************************************************************
* Defines
***************************************************************************************************/
#define MAX_SOFT_TIM_INSTANCES  4
#define MAX_SOFT_TIM_PERIOD_MS  10000

// Comment/Uncomment one of these lines based on what is your application going to use
//#define USE_TIME_H
#define USE_STM_HAL

/***************************************************************************************************
* Types
***************************************************************************************************/
// Define what the "clock type" is for your application:

// Use this if you are going to use <time.h>
#ifdef USE_TIME_H
#include <time.h>
typedef clock_t tick_type;

// Use this if you are going to use ST's HAL
#elif defined (USE_STM_HAL)
typedef uint32_t tick_type;

// Create your own...
//#elif ...
#endif

typedef enum
{
    TIMER_STOPPED = 0,
    TIMER_RUNNING,
}soft_tim_status_en;

typedef struct
{
    uint8_t     id;
    tick_type   target_tick;
}internal_timer_things_st;

typedef struct
{
    uint16_t            period_ms;
    bool                reload;
    void                (*call_back)(void *par1, uint16_t par2);  
    void                *param1;
    uint16_t            param2;
    soft_tim_status_en  status;
    
    internal_timer_things_st    private_member;
}soft_tim_st;

/***************************************************************************************************
* Prototypes
***************************************************************************************************/
void        SofTim_InitializeModule (void);
soft_tim_st *SofTim_AllocateTimer   (uint16_t period_ms, bool reload, void *call_back, void *par1, uint16_t par2);
bool        SofTim_FreeTimer        (soft_tim_st *Timer);
bool        SofTim_StartTimer       (soft_tim_st *Timer);
bool        SofTim_StartTimer2      (soft_tim_st *Timer, uint16_t period);
bool        SofTim_StopTimer        (soft_tim_st *Timer);
void        SofTim_Tick             (void);

#endif
