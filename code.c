#include "TM4C123GH6PM.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// getting the exact CPU speed from the system so our distance calculations stay accurate.
extern uint32_t SystemCoreClock;
extern void SystemCoreClockUpdate(void);

static int ticks_per_us = 80;
static float us_per_cm = 58.0f;

#define PWM_MAX_DISTANCE    100.0f   // pwm starts below this distance
#define PWM_MIN_DISTANCE    10.0f    // 100% duty at this distance
#define PWM_PERIOD          4000

#define FILTER_SIZE 5
static float filter_buf1[FILTER_SIZE];
static float filter_buf2[FILTER_SIZE];
static float filter_buf3[FILTER_SIZE];
static float filter_buf4[FILTER_SIZE];
static int filter_idx = 0;

 //fall detection configuration params

#define FALL_LOW_THRESHOLD      10.0f
#define FALL_HIGH_THRESHOLD     200.0f
#define FALL_FORWARD_TIME_MS    30000    // 30 seconds for forward
#define FALL_SIDE_TIME_MS       50000    // 50 seconds for left/right
#define FALL_BACK_TIME_MS       50000    // 50 seconds for back
#define SUDDEN_CHANGE_CM        30.0f

typedef enum 
{
    FALL_NONE,
    FALL_CHECKING_FORWARD,
    FALL_CHECKING_LEFT,
    FALL_CHECKING_RIGHT,
    FALL_CHECKING_BACK,
    FALL_DETECTED
} FallState;

static FallState fall_state = FALL_NONE;
static uint32_t fall_timer_ms = 0;
static float last_down_distance = -1.0f;

void SysTick_Init(void) 
{
    SystemCoreClockUpdate();
    ticks_per_us = SystemCoreClock / 1000000U;
    
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x00000005;
}

static uint32_t systick_elapsed(uint32_t start) {
    uint32_t current = SysTick->VAL;
    if (current <= start) {
        return (start - current);
    } else {
        return (start + (0x00FFFFFF - current) + 1);
    }
}

void delay_us(uint32_t us) {
    uint32_t ticks = us * ticks_per_us;
    uint32_t start = SysTick->VAL;
    while (systick_elapsed(start) < ticks) {}
}

void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

//uart config/initialization

void UART0_Init(void) {
    SYSCTL->RCGCUART |= 0x01;
    SYSCTL->RCGCGPIO |= 0x01;
    while ((SYSCTL->PRGPIO & 0x01) == 0) {}
    
    GPIOA->AFSEL |= 0x03;
    GPIOA->PCTL = (GPIOA->PCTL & 0xFFFFFF00) | 0x00000011;
    GPIOA->DEN |= 0x03;
    GPIOA->AMSEL &= ~0x03;
    
    UART0->CTL = 0;
    UART0->CC = 0x5;
    UART0->IBRD = 8;
    UART0->FBRD = 44;
    UART0->LCRH = 0x70;
    UART0->CTL = 0x301;
}

void UART0_SendChar(char c) {
    while ((UART0->FR & 0x20) != 0) {}
    UART0->DR = c;
}

void UART0_Print(const char *s) {
    while (*s) UART0_SendChar(*s++);
}

//adc config with temp sensor (lm35) 
// dani check if heres any difference with just sensor vs wo extension board
void ADC0_Init(void) {
    SYSCTL->RCGCADC |= 0x01;
    while ((SYSCTL->PRADC & 0x01) == 0);
    
    SYSCTL->RCGCGPIO |= 0x10;
    while ((SYSCTL->PRGPIO & 0x10) == 0);
    
    GPIOE->AFSEL |= 0x04;
    GPIOE->DEN &= ~0x04;
    GPIOE->AMSEL |= 0x04;
    
    ADC0->ACTSS &= ~0x08;
    ADC0->EMUX &= ~0xF000;
    ADC0->SSMUX3 = 1;
    ADC0->SSCTL3 = 0x06;
    ADC0->ACTSS |= 0x08;
}

float LM35_ReadTemperature(void) {
    int sum = 0;
    for (int i = 0; i < 10; i++) {
        ADC0->PSSI |= 0x08;
        while ((ADC0->RIS & 0x08) == 0);
        sum += ADC0->SSFIFO3 & 0xFFF;
        ADC0->ISC = 0x08;
        delay_ms(5);
    }
    return (sum / 10.0f / 4095.0f) * 330.0f;
}

void UpdateSpeedOfSound(float T_C) {
    float v_m_s = 331.3f + 0.606f * T_C;
    us_per_cm = 2000000.0f / (v_m_s * 100.0f);
}

//ultrasonoic initialization 
// for initializing all 4 sensors
void Ultrasonic_Init(void) {
    SYSCTL->RCGCGPIO |= 0x02;
    while ((SYSCTL->PRGPIO & 0x02) == 0) {}
    
    GPIOB->AFSEL &= ~0xFF;
    GPIOB->PCTL &= ~0xFFFFFFFF;
    GPIOB->AMSEL &= ~0xFF;
    GPIOB->DEN |= 0xFF;
    GPIOB->DIR |= 0x55;
    GPIOB->DIR &= ~0xAA;
    GPIOB->DATA &= ~0x55;
}


void PWM_Init(void) {
	
    SYSCTL->RCGCPWM |= 0x02;

    SYSCTL->RCGCGPIO |= 0x28; // clock enable port d, f

    while ((SYSCTL->PRPWM & 0x02) == 0) {}
    while ((SYSCTL->PRGPIO & 0x28) != 0x28) {}
    
    // pwm div 
    SYSCTL->RCC |= (1 << 20);
    SYSCTL->RCC &= ~(0x7 << 17);
    SYSCTL->RCC |= (0x1 << 17);

    GPIOF->LOCK = 0x4C4F434B;
    GPIOF->CR |= 0x1F;
    
    GPIOF->AFSEL |= 0x0E;
    GPIOF->PCTL &= ~0x0000FFF0;
    GPIOF->PCTL |= 0x00005550;
    GPIOF->DEN |= 0x0E;
    GPIOF->AMSEL &= ~0x0E;

    GPIOD->LOCK = 0x4C4F434B;
    GPIOD->CR |= 0x01;
    
    GPIOD->AFSEL |= 0x01;
    GPIOD->PCTL &= ~0x0000000F;
    GPIOD->PCTL |= 0x00000005;
    GPIOD->DEN |= 0x01;
    GPIOD->AMSEL &= ~0x01;
    
    // generator 0: M1PWM0 on PD0 (right sensor)

    PWM1->_0_CTL = 0;
    PWM1->_0_GENA = 0x000000C8;   // LOW on LOAD, HIGH on CMPA down
    PWM1->_0_LOAD = PWM_PERIOD - 1;
    PWM1->_0_CMPA = 0;
    PWM1->_0_CTL = 0x01;
    
    //generator 2: M1PWM5 on PF1 (Front sensor)

    PWM1->_2_CTL = 0;
    PWM1->_2_GENB = 0x00000C08;   // LOW on LOAD, HIGH on CMPB down
    PWM1->_2_LOAD = PWM_PERIOD - 1;
    PWM1->_2_CMPB = 0;
    PWM1->_2_CTL = 0x01;
    
    //generator 3: M1PWM6 on PF2, M1PWM7 on PF3
    PWM1->_3_CTL = 0;
    PWM1->_3_GENA = 0x000000C8;   // LOW on LOAD, HIGH on CMPA down
    PWM1->_3_GENB = 0x00000C08;   // LOW on LOAD, HIGH on CMPB down
    PWM1->_3_LOAD = PWM_PERIOD - 1;
    PWM1->_3_CMPA = 0;
    PWM1->_3_CMPB = 0;
    PWM1->_3_CTL = 0x01;

    PWM1->ENABLE = 0xE1;
}

/******************************************************************************
 * PWM Duty Cycle Functions (0-100%)
 * 
 * duty=0%   --> CMP=0          --> output stays LOW
 * duty=50%  --> CMP=PERIOD/2   --> 50% high time
 * duty=100% -=> CMP=PERIOD-1   --> output mostly HIGH
 *****************************************************************************/

void PWM_SetDuty_Front(uint32_t duty) {
    if (duty > 100) duty = 100;
    if (duty == 0) {
        PWM1->_2_CMPB = 0;
    } else {
        uint32_t cmp = ((PWM_PERIOD - 1) * duty) / 100;
        if (cmp == 0) cmp = 1;
        PWM1->_2_CMPB = cmp;
    }
}

void PWM_SetDuty_Down(uint32_t duty) {
    if (duty > 100) duty = 100;
    if (duty == 0) {
        PWM1->_3_CMPA = 0;
    } else {
        uint32_t cmp = ((PWM_PERIOD - 1) * duty) / 100;
        if (cmp == 0) cmp = 1;
        PWM1->_3_CMPA = cmp;
    }
}

void PWM_SetDuty_Left(uint32_t duty) {
    if (duty > 100) duty = 100;
    if (duty == 0) {
        PWM1->_3_CMPB = 0;
    } else {
        uint32_t cmp = ((PWM_PERIOD - 1) * duty) / 100;
        if (cmp == 0) cmp = 1;
        PWM1->_3_CMPB = cmp;
    }
}

void PWM_SetDuty_Right(uint32_t duty) {
    if (duty > 100) duty = 100;
    if (duty == 0) {
        PWM1->_0_CMPA = 0;
    } else {
        uint32_t cmp = ((PWM_PERIOD - 1) * duty) / 100;
        if (cmp == 0) cmp = 1;
        PWM1->_0_CMPA = cmp;
    }
}

uint32_t DistanceToDuty(float distance_cm) {

    if (distance_cm < 0) return 0;
    if (distance_cm >= PWM_MAX_DISTANCE) return 0;

    if (distance_cm <= PWM_MIN_DISTANCE) return 99; // return 99 not 100 to avoid hardware issue with pwm

    float range = PWM_MAX_DISTANCE - PWM_MIN_DISTANCE;
    float normalized = (PWM_MAX_DISTANCE - distance_cm) / range;

    uint32_t duty = (uint32_t)(normalized * 99.0f);
    if (duty > 99) duty = 99;
    
    return duty;
}

//sends a trigger pulse to the selected sensor 
// also measures the echo duration to calculate distance

float Ultrasonic_Read(uint8_t trig_mask, uint8_t echo_mask) {
    uint32_t timeout_ticks = 30000 * ticks_per_us;
    uint32_t start_wait, pulse_start;
    
    GPIOB->DATA &= ~trig_mask;
    delay_us(2);
    GPIOB->DATA |= trig_mask;
    delay_us(10);
    GPIOB->DATA &= ~trig_mask;
    
    start_wait = SysTick->VAL;
    while ((GPIOB->DATA & echo_mask) == 0) {
        if (systick_elapsed(start_wait) > timeout_ticks) return -1.0f;
    }
    pulse_start = SysTick->VAL;
    
    while ((GPIOB->DATA & echo_mask) != 0) {
        if (systick_elapsed(pulse_start) > timeout_ticks) return -1.0f;
    }
    
    uint32_t pulse_ticks = systick_elapsed(pulse_start);
    uint32_t time_us = (pulse_ticks + (ticks_per_us / 2)) / ticks_per_us;
    float distance = time_us / us_per_cm;
    
    if (distance < 2.0f || distance > 400.0f) return -1.0f;
    return distance;
}

// sorting da sensor readings and stuff

float FindMedian(float *arr) {
    float temp[FILTER_SIZE];
    int i, j;
    
    for (i = 0; i < FILTER_SIZE; i++) temp[i] = arr[i];
    
    for (i = 0; i < FILTER_SIZE - 1; i++) {
        for (j = 0; j < FILTER_SIZE - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float swap = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = swap;
            }
        }
    }
    
    int valid = 0;
    float sum = 0;
    for (i = 0; i < FILTER_SIZE; i++) {
        if (temp[i] > 0) {
            valid++;
            sum += temp[i];
        }
    }
    
    if (valid == 0) return -1.0f;
    if (valid >= 3) return temp[FILTER_SIZE / 2];
    return sum / valid;
}

// to account for garbage (devices and electronics hahahaha) values
//sorry

void InitFilters(void) {
    for (int i = 0; i < FILTER_SIZE; i++) {
        filter_buf1[i] = -1.0f;
        filter_buf2[i] = -1.0f;
        filter_buf3[i] = -1.0f;
        filter_buf4[i] = -1.0f;
    }
    filter_idx = 0;
}

void ReadAllSensors(float *d1, float *d2, float *d3, float *d4) {
    float raw1 = Ultrasonic_Read(0x01, 0x02);
    delay_ms(15);
    float raw2 = Ultrasonic_Read(0x04, 0x08);
    delay_ms(15);
    float raw3 = Ultrasonic_Read(0x10, 0x20);
    delay_ms(15);
    float raw4 = Ultrasonic_Read(0x40, 0x80);
    
    filter_buf1[filter_idx] = raw1;
    filter_buf2[filter_idx] = raw2;
    filter_buf3[filter_idx] = raw3;
    filter_buf4[filter_idx] = raw4;
    
    filter_idx = (filter_idx + 1) % FILTER_SIZE;
    
    *d1 = FindMedian(filter_buf1);
    *d2 = FindMedian(filter_buf2);
    *d3 = FindMedian(filter_buf3);
    *d4 = FindMedian(filter_buf4);
}


float float_abs(float x) {
    return (x < 0) ? -x : x;
}

const char* GetFallStateName(void) {
    switch (fall_state) {
        case FALL_NONE:             return "OK";
        case FALL_CHECKING_FORWARD: return "CHK_F";
        case FALL_CHECKING_LEFT:    return "CHK_L";
        case FALL_CHECKING_RIGHT:   return "CHK_R";
        case FALL_CHECKING_BACK:    return "CHK_B";
        case FALL_DETECTED:         return "FALL!";
        default:                    return "?";
    }
}

/* 
 * fall detection conditions
 * 
 * returns 1 if fall detected, 0 otherwise
 * 
 * logic:
 * - Forward: Down < 10cm AND sudden change (>30cm drop) -> wait 30s
 * - Left:    Left < 10cm -> wait 50s
 * - Right:   Right < 10cm -> wait 50s
 * - Back:    ALL sensors > 200cm -> wait 50s
 */

int CheckFallDetection(float dist_front, float dist_down, 
                       float dist_left, float dist_right,
                       uint32_t loop_time_ms) {
    char buffer[80];

    if (fall_state == FALL_DETECTED) {
        return 1;
    }
    
    // check trigger conditions
    int down_low = (dist_down > 0 && dist_down < FALL_LOW_THRESHOLD);
    int left_low = (dist_left > 0 && dist_left < FALL_LOW_THRESHOLD);
    int right_low = (dist_right > 0 && dist_right < FALL_LOW_THRESHOLD);
    
    // conditions for falling backward
    int front_high = (dist_front < 0 || dist_front > FALL_HIGH_THRESHOLD);
    int down_high = (dist_down < 0 || dist_down > FALL_HIGH_THRESHOLD);
    int left_high = (dist_left < 0 || dist_left > FALL_HIGH_THRESHOLD);
    int right_high = (dist_right < 0 || dist_right > FALL_HIGH_THRESHOLD);
    int all_high = (front_high && down_high && left_high && right_high);
    
    switch (fall_state) {
        
        case FALL_NONE:

            if (down_low && last_down_distance > 0) {
                float change = float_abs(dist_down - last_down_distance);
                if (change > SUDDEN_CHANGE_CM) {
                    fall_state = FALL_CHECKING_FORWARD;
                    fall_timer_ms = 0;
                    UART0_Print("\r\n[FALL] Down<10 + sudden! Wait 30s\r\n");
                }
            }
            // check left fall
            else if (left_low) {
                fall_state = FALL_CHECKING_LEFT;
                fall_timer_ms = 0;
                UART0_Print("\r\n[FALL] Left<10! Wait 50s\r\n");
            }
            // check right fall
            else if (right_low) {
                fall_state = FALL_CHECKING_RIGHT;
                fall_timer_ms = 0;
                UART0_Print("\r\n[FALL] Right<10! Wait 50s\r\n");
            }
            // check back fall
            else if (all_high) {
                fall_state = FALL_CHECKING_BACK;
                fall_timer_ms = 0;
                UART0_Print("\r\n[FALL] All>200! Wait 50s\r\n");
            }
            
            // save last down distance
            if (dist_down > 0) {
                last_down_distance = dist_down;
            }
            break;
            
        case FALL_CHECKING_FORWARD:
            if (!down_low) {
                UART0_Print("[FALL] Down recovered.\r\n");
                fall_state = FALL_NONE;
                fall_timer_ms = 0;
            } else {
                fall_timer_ms += loop_time_ms;
                if (fall_timer_ms >= FALL_FORWARD_TIME_MS) {
                    fall_state = FALL_DETECTED;
                    return 1;
                }
            }
            break;
            
        case FALL_CHECKING_LEFT:
            if (!left_low) {
                UART0_Print("[FALL] Left recovered.\r\n");
                fall_state = FALL_NONE;
                fall_timer_ms = 0;
            } else {
                fall_timer_ms += loop_time_ms;
                if (fall_timer_ms >= FALL_SIDE_TIME_MS) {
                    fall_state = FALL_DETECTED;
                    return 1;
                }
            }
            break;
            
        case FALL_CHECKING_RIGHT:
            if (!right_low) {
                UART0_Print("[FALL] Right recovered.\r\n");
                fall_state = FALL_NONE;
                fall_timer_ms = 0;
            } else {
                fall_timer_ms += loop_time_ms;
                if (fall_timer_ms >= FALL_SIDE_TIME_MS) {
                    fall_state = FALL_DETECTED;
                    return 1;
                }
            }
            break;
            
        case FALL_CHECKING_BACK:
            if (!all_high) {
                UART0_Print("[FALL] Sensors recovered.\r\n");
                fall_state = FALL_NONE;
                fall_timer_ms = 0;
            } else {
                fall_timer_ms += loop_time_ms;
                if (fall_timer_ms >= FALL_BACK_TIME_MS) {
                    fall_state = FALL_DETECTED;
                    return 1;
                }
            }
            break;
            
        case FALL_DETECTED:
            return 1;
    }
    
    return 0;
}

int main(void) {
    char buffer[100];

    SysTick_Init();
    UART0_Init();
    ADC0_Init();
    Ultrasonic_Init();
    PWM_Init();
    InitFilters();

    UART0_Print("\r\n========================================================\r\n");
    UART0_Print("   SONAR CAP - 4 SENSORS + 4 PWM + FALL DETECTION\r\n");
    UART0_Print("========================================================\r\n\r\n");

    UART0_Print("[INIT] Reading temperature...\r\n");
    delay_ms(100);
    float temp = LM35_ReadTemperature();
    UpdateSpeedOfSound(temp);
    
    sprintf(buffer, "[TEMP] %d.%d C\r\n\r\n", (int)temp, (int)((temp - (int)temp) * 10));
    UART0_Print(buffer);
    
    UART0_Print("[PWM OUTPUTS]\r\n");
    UART0_Print("  PF1 = Front (Red LED)\r\n");
    UART0_Print("  PF2 = Down  (Blue LED)\r\n");
    UART0_Print("  PF3 = Left  (Green LED)\r\n");
    UART0_Print("  PD0 = Right (J3 pin 3)\r\n\r\n");
    
    UART0_Print("[BEHAVIOR]\r\n");
    UART0_Print("  >= 100 cm -> 0%% (OFF)\r\n");
    UART0_Print("  55 cm     -> 50%%\r\n");
    UART0_Print("  <= 10 cm  -> 100%% (FULL)\r\n\r\n");
    
    UART0_Print("[FALL DETECTION]\r\n");
    UART0_Print("  Forward: Down<10 + sudden -> 30s\r\n");
    UART0_Print("  Left:    Left<10 -> 50s\r\n");
    UART0_Print("  Right:   Right<10 -> 50s\r\n");
    UART0_Print("  Back:    All>200 -> 50s\r\n\r\n");
    
    // pwm test, should shoe leds 
    UART0_Print("[TEST] PWM sweep 0%% to 100%%...\r\n");
    UART0_Print("  LEDs should go: OFF -> DIM -> BRIGHT\r\n");
    
    // Start at 0% - LEDs should be OFF
    PWM_SetDuty_Front(0);
    PWM_SetDuty_Down(0);
    PWM_SetDuty_Left(0);
    PWM_SetDuty_Right(0);
    UART0_Print("  0%% - LEDs OFF\r\n");
    delay_ms(1000);
    
    // 25%
    PWM_SetDuty_Front(25);
    PWM_SetDuty_Down(25);
    PWM_SetDuty_Left(25);
    PWM_SetDuty_Right(25);
    UART0_Print("  25%% - LEDs DIM\r\n");
    delay_ms(1000);
    
    // 50%
    PWM_SetDuty_Front(50);
    PWM_SetDuty_Down(50);
    PWM_SetDuty_Left(50);
    PWM_SetDuty_Right(50);
    UART0_Print("  50%% - LEDs MEDIUM\r\n");
    delay_ms(1000);
    
    // 75%
    PWM_SetDuty_Front(75);
    PWM_SetDuty_Down(75);
    PWM_SetDuty_Left(75);
    PWM_SetDuty_Right(75);
    UART0_Print("  75%% - LEDs BRIGHT\r\n");
    delay_ms(1000);
    
    // 100%
    PWM_SetDuty_Front(100);
    PWM_SetDuty_Down(100);
    PWM_SetDuty_Left(100);
    PWM_SetDuty_Right(100);
    UART0_Print("  100%% - LEDs FULL\r\n");
    delay_ms(1000);
    
    // 0%
    PWM_SetDuty_Front(0);
    PWM_SetDuty_Down(0);
    PWM_SetDuty_Left(0);
    PWM_SetDuty_Right(0);
    UART0_Print("  0%% - LEDs OFF\r\n");
    delay_ms(500);
    
    UART0_Print("[TEST] Complete!\r\n\r\n");

    UART0_Print("[INIT] Filling filter buffers...\r\n");
    for (int i = 0; i < FILTER_SIZE; i++) {
        float d1, d2, d3, d4;
        ReadAllSensors(&d1, &d2, &d3, &d4);
        delay_ms(50);
    }
    
    // initialize fall detection
    fall_state = FALL_NONE;
    fall_timer_ms = 0;
    last_down_distance = -1.0f;
    
    // get initial down reading
    for (int i = 0; i < 3; i++) {
        float d1, d2, d3, d4;
        ReadAllSensors(&d1, &d2, &d3, &d4);
        if (d2 > 0) last_down_distance = d2;
        delay_ms(50);
    }
    
    UART0_Print("[READY] Starting!\r\n\r\n");
    
    UART0_Print("F=Front D=Down L=Left R=Right | State\r\n");
    UART0_Print("--------------------------------------------------------\r\n");
    
    // loop timing ~150ms per iteration
    uint32_t loop_time_ms = 150;

    while (1) {
        float dist_front, dist_down, dist_left, dist_right;
        ReadAllSensors(&dist_front, &dist_down, &dist_left, &dist_right);
        
        // PWM control (unchanged)
        uint32_t duty_front = DistanceToDuty(dist_front);
        uint32_t duty_down  = DistanceToDuty(dist_down);
        uint32_t duty_left  = DistanceToDuty(dist_left);
        uint32_t duty_right = DistanceToDuty(dist_right);
        
        PWM_SetDuty_Front(duty_front);
        PWM_SetDuty_Down(duty_down);
        PWM_SetDuty_Left(duty_left);
        PWM_SetDuty_Right(duty_right);
        
        // fall detection 
        int fall_detected = CheckFallDetection(dist_front, dist_down, 
                                                dist_left, dist_right, 
                                                loop_time_ms);
        
        if (fall_detected) {
            UART0_Print("\r\n");
            UART0_Print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
            UART0_Print("!!!                                             !!!\r\n");
            UART0_Print("!!!          **** FALL DETECTED ****            !!!\r\n");
            UART0_Print("!!!                                             !!!\r\n");
            UART0_Print("!!!       Person may need assistance!           !!!\r\n");
            UART0_Print("!!!                                             !!!\r\n");
            UART0_Print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
            
            // Set all PWM to max as alert!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            PWM_SetDuty_Front(99);
            PWM_SetDuty_Down(99);
            PWM_SetDuty_Left(99);
            PWM_SetDuty_Right(99);
            
            // Stay in alert loop
            while (1) {
                UART0_Print("*** FALL DETECTED ***\r\n");
                delay_ms(3000);
            }
        }
        
        // Printing all da readings
        UART0_Print("F:");
        if (dist_front > 0) {
            sprintf(buffer, "%3d", (int)dist_front);
            UART0_Print(buffer);
        } else {
            UART0_Print(" OR");
        }
        sprintf(buffer, "[%2d%%] ", (int)duty_front);
        UART0_Print(buffer);
        
        UART0_Print("D:");
        if (dist_down > 0) {
            sprintf(buffer, "%3d", (int)dist_down);
            UART0_Print(buffer);
        } else {
            UART0_Print(" OR");
        }
        sprintf(buffer, "[%2d%%] ", (int)duty_down);
        UART0_Print(buffer);
        
        UART0_Print("L:");
        if (dist_left > 0) {
            sprintf(buffer, "%3d", (int)dist_left);
            UART0_Print(buffer);
        } else {
            UART0_Print(" OR");
        }
        sprintf(buffer, "[%2d%%] ", (int)duty_left);
        UART0_Print(buffer);
        
        UART0_Print("R:");
        if (dist_right > 0) {
            sprintf(buffer, "%3d", (int)dist_right);
            UART0_Print(buffer);
        } else {
            UART0_Print(" OR");
        }
        sprintf(buffer, "[%2d%%] ", (int)duty_right);
        UART0_Print(buffer);
        
        // Show fall state and timer
        UART0_Print("| ");
        sprintf(buffer, "%-5s", GetFallStateName());
        UART0_Print(buffer);
        
        if (fall_state != FALL_NONE && fall_state != FALL_DETECTED) {
            sprintf(buffer, " %2ds", (int)(fall_timer_ms / 1000));
            UART0_Print(buffer);
        }
        
        UART0_Print("\r\n");
        
        delay_ms(100);
    }
}
