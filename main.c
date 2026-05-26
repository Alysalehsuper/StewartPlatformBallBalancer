#include "stm32f1xx.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Include FreeRTOS core libraries
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "message_buffer.h"

// ================== Constants ==================
#define PI 3.14159265358979f
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)

// Stewart geometry
#define RP 125.0f
#define RB 75.0f
#define LA 50.0f
#define LB 100.0f
#define H0 95.0f
#define MAX_TILT 15.0f

#define SERVO_TANGENTIAL_MOUNT 0
#define BETA_SIGN (+1.0f)
#define THETA_OFFSET_DEG -30.0f

// PID defaults
#define KP 0.15f
#define KI 0.00f
#define KD 0.045f // Updated to starting estimate for 75ms delay
#define MAX_I 10.0f
// Note: Hardcoded DT macro has been removed for dynamic DT calculation

// UART / tracker
#define SYSTEM_CLOCK_HZ 64000000UL
#define UART_BAUD       115200UL
#define CAM_W           320.0f
#define CAM_H           240.0f
#define UART_TIMEOUT_MS 120U

// ================== FreeRTOS IPC & Tasks ==================
TaskHandle_t UartTaskHandle = NULL;
TaskHandle_t ControlTaskHandle = NULL;

MessageBufferHandle_t UartMessageBuffer = NULL;
QueueHandle_t TargetMailbox = NULL;

// ================== Global State ==================
int16_t servo_offsets[3] = {0, 0, 0};
const float phi[3] = {0.0f, 120.0f * DEG2RAD, 240.0f * DEG2RAD};

float beta_cos[3], beta_sin[3];
float cam_rot_c, cam_rot_s;
float B_x[3], B_y[3], B_z[3];
float p_local_x[3], p_local_y[3], p_local_z[3];

static volatile float g_kp = KP;
static volatile float g_ki = KI;
static volatile float g_kd = KD;

// Target structure stored in the FreeRTOS Queue
typedef struct {
    int16_t x_px;
    int16_t y_px;
    uint8_t found;
    uint32_t t_ms;
} UartTarget;


// ================== Bare Metal Clock Config ==================
static void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    FLASH->ACR |= FLASH_ACR_PRFTBE;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= RCC_CFGR_PLLMULL16;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

// ================== Helpers ==================
static inline uint32_t millis(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static bool parse_int(char **p, int32_t *out) {
    char *end;
    long v = strtol(*p, &end, 10);
    if (end == *p) return false;
    *out = (int32_t)v;
    *p = end;
    return true;
}

static void process_pid_uart_line(char *line) {
    if (strncmp(line, "PID,", 4) != 0) return;

    char *p = line + 4;
    char *end;

    float kp = strtof(p, &end);
    if (end == p || *end != ',') return;
    p = end + 1;

    float ki = strtof(p, &end);
    if (end == p || *end != ',') return;
    p = end + 1;

    float kd = strtof(p, &end);
    if (end == p) return;
    if (*end != '\0' && *end != '\r') return;

    g_kp = kp;
    g_ki = ki;
    g_kd = kd;
}

static void init_platform_geometry(void) {
    for (int i = 0; i < 3; i++) {
        B_x[i] = RB * cosf(phi[i]);
        B_y[i] = RB * sinf(phi[i]);
        B_z[i] = 0.0f;

        p_local_x[i] = RP * cosf(phi[i]);
        p_local_y[i] = RP * sinf(phi[i]);
        p_local_z[i] = 0.0f;

#if SERVO_TANGENTIAL_MOUNT
        float beta = phi[i] + BETA_SIGN * (PI * 0.5f);
#else
        float beta = phi[i];
#endif
        beta_cos[i] = cosf(beta);
        beta_sin[i] = sinf(beta);
    }

    cam_rot_c = cosf(THETA_OFFSET_DEG * DEG2RAD);
    cam_rot_s = sinf(THETA_OFFSET_DEG * DEG2RAD);
}

static void uart1_init(uint32_t baud) {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    GPIOA->CRH &= ~((0xF << 4) | (0xF << 8));
    GPIOA->CRH |=  (0xB << 4);
    GPIOA->CRH |=  (0x4 << 8);

    USART1->BRR = (SYSTEM_CLOCK_HZ + (baud / 2U)) / baud;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    NVIC_SetPriority(USART1_IRQn, 5);
    NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void) {
    static char rx_buf[64];
    static uint8_t idx = 0;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t sr = USART1->SR;

    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        volatile uint32_t dummy = USART1->DR;
        (void)dummy;
        idx = 0;
        return;
    }

    if (sr & USART_SR_RXNE) {
        char c = (char)USART1->DR;

        if (c == '\r') return;

        if (c == '\n') {
            rx_buf[idx] = '\0';
            xMessageBufferSendFromISR(UartMessageBuffer, rx_buf, idx + 1, &xHigherPriorityTaskWoken);
            idx = 0;
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            return;
        }

        if (idx < (sizeof(rx_buf) - 1)) {
            rx_buf[idx++] = c;
        } else {
            idx = 0;
        }
    }
}

static void process_uart_line(char *line) {
    process_pid_uart_line(line);
    if (strncmp(line, "PID,", 4) == 0) return;

    int32_t a, b, c;
    char *p = line;

    if (!parse_int(&p, &a)) return;
    if (*p != ',') return;
    p++;
    if (!parse_int(&p, &b)) return;

    uint8_t found = 1;
    int32_t x = a, y = b;

    if (*p == ',') {
        p++;
        if (!parse_int(&p, &c)) return;
        if (*p != '\0') return;
        found = (a != 0) ? 1U : 0U;
        x = b;
        y = c;
    } else if (*p != '\0') {
        return;
    }

    if (x < 0) x = 0;
    if (x > (int32_t)(CAM_W - 1.0f)) x = (int32_t)(CAM_W - 1.0f);
    if (y < 0) y = 0;
    if (y > (int32_t)(CAM_H - 1.0f)) y = (int32_t)(CAM_H - 1.0f);

    UartTarget new_target;
    new_target.x_px = (int16_t)x;
    new_target.y_px = (int16_t)y;
    new_target.found = found;
    new_target.t_ms = millis();

    xQueueOverwrite(TargetMailbox, &new_target);
}

// ================== Platform Control (MODIFIED) ==================

// Added actual_dt parameter to correctly map time delta
static void update_platform(float ball_x, float ball_y, float actual_dt) {
    static float integral_x = 0.0f, integral_y = 0.0f;
    static float prev_error_x = 0.0f, prev_error_y = 0.0f;

    float error_x = -ball_x;
    float error_y = -ball_y;

    // Use dynamic time for Integral
    integral_x += error_x * actual_dt;
    integral_y += error_y * actual_dt;

    if (integral_x > MAX_I) integral_x = MAX_I;
    if (integral_x < -MAX_I) integral_x = -MAX_I;
    if (integral_y > MAX_I) integral_y = MAX_I;
    if (integral_y < -MAX_I) integral_y = -MAX_I;

    // Use dynamic time for Derivative
    float derivative_x = (error_x - prev_error_x) / actual_dt;
    float derivative_y = (error_y - prev_error_y) / actual_dt;

    float kp = g_kp;
    float ki = g_ki;
    float kd = g_kd;

    float output_x = (kp * error_x) + (ki * integral_x) + (kd * derivative_x);
    float output_y = (kp * error_y) + (ki * integral_y) + (kd * derivative_y);

    prev_error_x = error_x;
    prev_error_y = error_y;

    float roll_deg  = output_x;
    float pitch_deg = -output_y;

    if (pitch_deg > MAX_TILT) pitch_deg = MAX_TILT;
    if (pitch_deg < -MAX_TILT) pitch_deg = -MAX_TILT;
    if (roll_deg > MAX_TILT) roll_deg = MAX_TILT;
    if (roll_deg < -MAX_TILT) roll_deg = -MAX_TILT;

    float pitch = pitch_deg * DEG2RAD;
    float roll  = roll_deg * DEG2RAD;

    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);

    float R00 = cr,       R01 = sr * sp,   R02 = sr * cp;
    float R10 = 0.0f,     R11 = cp,        R12 = -sp;
    float R20 = -sr,      R21 = cr * sp,   R22 = cr * cp;

    for (int i = 0; i < 3; i++) {
        float Px = R00 * p_local_x[i] + R01 * p_local_y[i] + R02 * p_local_z[i];
        float Py = R10 * p_local_x[i] + R11 * p_local_y[i] + R12 * p_local_z[i];
        float Pz = R20 * p_local_x[i] + R21 * p_local_y[i] + R22 * p_local_z[i] + H0;

        float dx = Px - B_x[i];
        float dy = Py - B_y[i];
        float dz = Pz - B_z[i];

        float dist_sq = dx * dx + dy * dy + dz * dz;
        float rho = dx * beta_cos[i] + dy * beta_sin[i];

        float L_eff = dist_sq + (LA * LA) - (LB * LB);
        float M = 2.0f * LA * dz;
        float N = 2.0f * LA * rho;

        float denom = sqrtf(M * M + N * N);
        if (denom < 1e-6f) denom = 1e-6f;

        float ratio = L_eff / denom;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < -1.0f) ratio = -1.0f;

        float alpha_rad = asinf(ratio) - atan2f(N, M);

        float pulse = 1500.0f + ((90.0f - (alpha_rad * RAD2DEG)) * 5.555f);
        pulse += servo_offsets[i];

        if (pulse > 2500.0f) pulse = 2500.0f;
        if (pulse < 500.0f)  pulse = 500.0f;

        if (i == 0) TIM2->CCR1 = (uint16_t)pulse; // PA0
        if (i == 1) TIM2->CCR2 = (uint16_t)pulse; // PA1
        if (i == 2) TIM3->CCR4 = (uint16_t)pulse; // PB1
    }
}

// ================== FreeRTOS Tasks ==================

void vUartTask(void *pvParameters) {
    char local[64];
    while (1) {
        size_t bytes_received = xMessageBufferReceive(UartMessageBuffer, local, sizeof(local), portMAX_DELAY);
        if (bytes_received > 0) {
            process_uart_line(local);
        }
    }
}

void vControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);

    UartTarget current_target = {0, 0, 0, 0};
    uint32_t last_target_time = 0; // Tracks the last time a NEW frame was processed

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        xQueuePeek(TargetMailbox, &current_target, 0);

        if (current_target.found && ((millis() - current_target.t_ms) < UART_TIMEOUT_MS)) {
            // ONLY execute PID if a totally new frame arrived from the Raspberry Pi
            if (current_target.t_ms != last_target_time) {

                // Calculate true time delta between camera frames (in seconds)
                float actual_dt = (float)(current_target.t_ms - last_target_time) / 1000.0f;

                // Sanity bound actual_dt to prevent PID explosions on startup/severe lag
                if (actual_dt <= 0.001f || actual_dt > 0.5f) {
                    actual_dt = 0.02f; // Fallback
                }

                float temp_x = ((float)current_target.x_px - (CAM_W * 0.5f));
                float temp_y = ((float)current_target.y_px - (CAM_H * 0.5f));

                float ball_x =  cam_rot_c * temp_x + cam_rot_s * temp_y;
                float ball_y = -cam_rot_s * temp_x + cam_rot_c * temp_y;

                // Pass the true dt to the physics update
                update_platform(ball_x, ball_y, actual_dt);

                last_target_time = current_target.t_ms;
            }
            // If it's NOT a new frame, loop does nothing. Servos hold their position.
            // This prevents Derivative dropout.
        } else {
            // Target is completely lost or timed out. Level the platform safely.
            update_platform(0.0f, 0.0f, 0.02f);
        }
    }
}

// ================== Main ==================

int main(void) {
    SystemClock_Config();

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;

    GPIOA->CRL &= ~(0xFF << 0);
    GPIOA->CRL |=  (0xBB << 0);
    GPIOB->CRL &= ~(0xF << 4);
    GPIOB->CRL |=  (0xB << 4);

    TIM2->PSC = 64 - 1;
    TIM2->ARR = 20000 - 1;
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE;
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
    TIM2->CCER  |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM2->CR1   |= TIM_CR1_CEN;

    TIM3->PSC = 64 - 1;
    TIM3->ARR = 20000 - 1;
    TIM3->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE;
    TIM3->CCER  |= TIM_CCER_CC4E;
    TIM3->CR1   |= TIM_CR1_CEN;

    init_platform_geometry();
    uart1_init(UART_BAUD);

    UartMessageBuffer = xMessageBufferCreate(128);
    TargetMailbox = xQueueCreate(1, sizeof(UartTarget));

    xTaskCreate(vControlTask, "ControlLoop", 256, NULL, 3, &ControlTaskHandle);
    xTaskCreate(vUartTask,    "UARTProcess", 256, NULL, 2, &UartTaskHandle);

    vTaskStartScheduler();

    while (1) {}
}
