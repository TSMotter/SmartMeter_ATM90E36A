/***************************************************************************************************
* @file     gm_timer.c
* @brief    Statically allocated simple software timer module
* @author   Giuliano Motter
* @date     08/2020
* @details  github.com/TSMotter
***************************************************************************************************/

/***************************************************************************************************
* Includes
***************************************************************************************************/
#include <string.h>
#include <stdio.h>

#include "gm_timer.h"

/***************************************************************************************************
* Private Functions Prototypes
***************************************************************************************************/
static int          index_of_first_available_timer  (void);
static uint8_t      num_of_allocated_timers         (void);
static bool         validade_tim_parameters         (uint16_t period_ms, void* cbFunc);
static void         reload_timer                    (soft_tim_st *Timer);

/***************************************************************************************************
* Externals
***************************************************************************************************/

/***************************************************************************************************
* Vars
***************************************************************************************************/
static soft_tim_st TimersArray[MAX_SOFT_TIM_INSTANCES];
static tick_type    tick_to_compare = 0;
// Define what the "get tick" function is for your application:

// Use this if you are going to use <time.h>
#ifdef USE_TIME_H
tick_type   (*Tick_Func)(void) = &clock;

// Use this if you are going to use ST's HAL
#elif defined (USE_STM_HAL)
#include    "stm32f4xx_hal.h"
tick_type   (*Tick_Func)(void) = &HAL_GetTick;

// Create your own...
//#elif ...
#endif

/***************************************************************************************************
* @brief Returns a index of the vector, which will point to the first available slot
***************************************************************************************************/
static int index_of_first_available_timer(void)
{
    for(int i = 0; i < MAX_SOFT_TIM_INSTANCES; i++)
    {
        if(TimersArray[i].private_member.id == 0)
        {
            return i;
        }
    }
    return -1; // Error
}

/***************************************************************************************************
* @brief Returns the number of allocated timers
***************************************************************************************************/
static uint8_t num_of_allocated_timers(void)
{
    int actives = 0;

    for(int i = 0; i < MAX_SOFT_TIM_INSTANCES; i++)
    {
        if(TimersArray[i].private_member.id != 0)
        {
            actives++;
        }
    }
    return actives;
}

/***************************************************************************************************
* @brief Indicate if the timer you are trying to create have valid parameters
***************************************************************************************************/
static bool validade_tim_parameters(uint16_t period_ms, void* cbFunc)
{
    // Check pointer
    if(cbFunc == NULL)
    {
        return false;
    }
    
    // Check period
    if((period_ms > MAX_SOFT_TIM_PERIOD_MS) || (period_ms == 0))
    {
        return false;
    }   

    return true; 
}

/***************************************************************************************************
* @brief Loads/reloads software timer with "target tick" that the timer will expire
***************************************************************************************************/
static void reload_timer(soft_tim_st *Timer)
{
    tick_type time_now = Tick_Func();

    Timer->private_member.target_tick = time_now + Timer->period_ms;
}

/***************************************************************************************************
* @brief Clear the static vector that allocate all the timers instances
***************************************************************************************************/
void SofTim_InitializeModule(void)
{
    memset(TimersArray, 0, (MAX_SOFT_TIM_INSTANCES*sizeof(soft_tim_st)));    
}

/***************************************************************************************************
* @brief Create a new software timer instance and returns the pointer of where it's allocated 
***************************************************************************************************/
soft_tim_st *SofTim_AllocateTimer(uint16_t period_ms, bool reload, void *call_back, void *par1, uint16_t par2)
{
    soft_tim_st LocalInstance;

    // Check for the max number of timers
    if(num_of_allocated_timers() >= MAX_SOFT_TIM_INSTANCES)
    {
        return NULL;
    }

    // Check parameters
    if(validade_tim_parameters(period_ms, call_back) == false)
    {
        return NULL;
    }
   
    int idx = index_of_first_available_timer();

    if(idx < 0)
    {
        return NULL;
    }

    // Arbitrarily adds 10 to the id, so that: 
    // id = 0 means slot available, id != 0 means slot not available, 
    LocalInstance.private_member.id = idx + 10;

    LocalInstance.period_ms = period_ms;
    LocalInstance.reload = reload;
    LocalInstance.status = TIMER_STOPPED;
    LocalInstance.call_back = call_back;
    LocalInstance.param1 = par1;
    LocalInstance.param2 = par2;

    memcpy(&TimersArray[idx], &LocalInstance, sizeof(soft_tim_st));

    return &TimersArray[idx];
}

/***************************************************************************************************
* @brief Loads the target tick and change the status of a software timer instance
***************************************************************************************************/
bool SofTim_StartTimer(soft_tim_st *Timer)
{
    if(Timer == NULL)
    {
        return false;
    }   

    reload_timer(Timer);
    Timer->status = TIMER_RUNNING;
    return true;
}

/***************************************************************************************************
* @brief Loads the target tick and change the status of a software timer instance
***************************************************************************************************/
bool SofTim_StartTimer2(soft_tim_st *Timer, uint16_t period)
{
    if((Timer == NULL) || (period > MAX_SOFT_TIM_PERIOD_MS) || (period == 0))
    {
        return false;
    }   

    Timer->period_ms = period;
    reload_timer(Timer);
    Timer->status = TIMER_RUNNING;
    return true;
}

/***************************************************************************************************
* @brief Change the status of a software timer instance
***************************************************************************************************/
bool SofTim_StopTimer(soft_tim_st *Timer)
{
    if(Timer == NULL)
    {
        return false;
    }  

    Timer->status = TIMER_STOPPED;
    return true;
}

/***************************************************************************************************
* @brief This function needs to be called periodically with a known period, normally it'll be on a 
*        hardware timer interrupt service routine
***************************************************************************************************/
void SofTim_Tick(void)
{ 
    tick_to_compare = Tick_Func();
    
    for(int i = 0; i < MAX_SOFT_TIM_INSTANCES; i++)
    {
        if(TimersArray[i].status == TIMER_RUNNING)
        {
            if(tick_to_compare >= TimersArray[i].private_member.target_tick)
            {
                // Calls the callback
                TimersArray[i].call_back(TimersArray[i].param1, TimersArray[i].param2);

                // If the timer is reloadable, reload it
                if( TimersArray[i].reload == true)
                {
                    reload_timer(&TimersArray[i]);
                }
                // Otherwise, stop it
                else
                {
                    TimersArray[i].status = TIMER_STOPPED;
                }                
            }
        }
    }
}

/***************************************************************************************************
* @brief Deallocate a software timer instance
***************************************************************************************************/
bool SofTim_FreeTimer(soft_tim_st *Timer)
{
    if(Timer == NULL)
    {
        return false;
    }
    
    memset(Timer, 0, sizeof(soft_tim_st));
    return true;
}
