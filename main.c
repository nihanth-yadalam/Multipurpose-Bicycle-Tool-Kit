#include "stm32f4xx.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// System configuration constants.
#define PCF8574_ADDR7    0x27
#define MPU6050_ADDR7    0x68
#define PASSWORD_LEN 6
#define PW_CHANGE_TIMEOUT_MS 3000
#define PW_ENTRY_TIMEOUT_MS  20000
#define NMEA_BUFFER_SIZE 128
#define IMPACT_THRESHOLD_G     1.0f
#define FALL_THRESHOLD_DPS     250.0f
#define TILT_THRESHOLD_DEG     70.0f
#define CONFIRM_WINDOW_MS      10000u
#define TILT_STREAK_MS         5000u
#define SAMPLE_PERIOD_MS       10u
#define GRACE_WINDOW_MS        30000u
#define LED_BLINK_MS           300u
#define BL_BIT (1<<3)
#define RS_BIT (1<<0)
#define EN_BIT (1<<2)

// Global variables for system state and sensor data.
char password[PASSWORD_LEN + 1] = "123456";
volatile float gps_latitude  = 0.0f;
volatile float gps_longitude = 0.0f;
volatile int   gps_fix_quality = 0;
volatile int   gps_satellites  = 0;
volatile char  gps_time[11]    = "00:00:00";
char nmea_buffer[NMEA_BUFFER_SIZE];
volatile int nmea_index = 0;
volatile int sentence_ready = 0;
volatile int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
volatile float   A_res_g, G_res_dps, tilt_deg;
volatile uint8_t who_am_i;
static uint8_t pcf_output_cache = 0x00;
volatile float accident_lat = 0.0f;
volatile float accident_lon = 0.0f;
volatile uint8_t accident_snapshot_taken = 0;
volatile uint32_t gMillis = 0;

// Initializes the DWT cycle counter for microsecond-precision delays.
static void DWT_DelayInit(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// SysTick interrupt handler, called every millisecond to increment a global counter.
void SysTick_Handler(void) { gMillis++; }

// Returns the number of milliseconds since the program started.
static inline uint32_t millis(void) { return gMillis; }

// Returns the number of microseconds since the DWT was initialized.
static inline uint32_t micros(void) {
    return (uint32_t)(DWT->CYCCNT / (SystemCoreClock / 1000000UL));
}

// Halts execution for a specified number of microseconds.
static void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000UL) * us;
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}

// Halts execution for a specified number of milliseconds.
static void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

// Configures GPIOs and I2C1 peripheral for communication.
static void I2C1_Init_shared(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    GPIOB->MODER &= ~((3<<(8*2)) | (3<<(9*2)));
    GPIOB->MODER |=  (2<<(8*2)) | (2<<(9*2));
    GPIOB->OTYPER |= (1<<8) | (1<<9);
    GPIOB->PUPDR &= ~((3<<(8*2)) | (3<<(9*2)));
    GPIOB->PUPDR |=  (1<<(8*2)) | (1<<(9*2));
    GPIOB->AFR[1] &= ~((0xF<<0) | (0xF<<4));
    GPIOB->AFR[1] |=  (4<<0) | (4<<4);
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= I2C_CR1_PE;
}

// Writes a single byte to an I2C device.
static int I2C1_WriteByte(uint8_t addr7, uint8_t data) {
    uint32_t t;
    I2C1->CR1 |= I2C_CR1_START;
    t = 50000;
    while (!(I2C1->SR1 & I2C_SR1_SB) && --t) { }
    if (!t) return -1;
    I2C1->DR = (addr7 << 1) & 0xFE;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_ADDR) && --t) { }
    if (!t) { I2C1->CR1 |= I2C_CR1_STOP; return -1; }
    (void)I2C1->SR2;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_TXE) && --t) { }
    if (!t) { I2C1->CR1 |= I2C_CR1_STOP; return -1; }
    I2C1->DR = data;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_BTF) && --t) { }
    if (!t) { I2C1->CR1 |= I2C_CR1_STOP; return -1; }
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

// Writes a buffer of bytes to an I2C device.
static int I2C1_WriteBytes(uint8_t addr7, const uint8_t *buf, int len) {
    uint32_t t;
    if (len <= 0) return -1;
    I2C1->CR1 |= I2C_CR1_START;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_SB) && --t) { }
    if (!t) return -1;
    I2C1->DR = (addr7 << 1) & 0xFE;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_ADDR) && --t) { }
    if (!t) { I2C1->CR1 |= I2C_CR1_STOP; return -1; }
    (void)I2C1->SR2;
    for (int i=0;i<len;i++){
        t = 50000;
        while(!(I2C1->SR1 & I2C_SR1_TXE) && --t) { }
        if (!t) { I2C1->CR1 |= I2C_CR1_STOP; return -1; }
        I2C1->DR = buf[i];
    }
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_BTF) && --t) { }
    if (!t) { I2C1->CR1 |= I2C_CR1_STOP; return -1; }
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0;
}

// Reads an 8-bit register from an I2C device.
static uint8_t I2C1_ReadReg8(uint8_t addr7, uint8_t reg) {
    uint8_t result = 0xFF;
    uint32_t t;
    I2C1->CR1 |= I2C_CR1_START;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_SB) && --t) { }
    if (!t) return result;
    I2C1->DR = (addr7 << 1) & 0xFE;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_ADDR) && --t) { }
    if (!t) { I2C1->CR1 |= I2C_CR1_STOP; return result; }
    (void)I2C1->SR2;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_TXE) && --t) { }
    if (!t) { I2C1->CR1 |= I2C_CR1_STOP; return result; }
    I2C1->DR = reg;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_TXE) && --t) { }
    if (!t) { I2C1->CR1 |= I2C_CR1_STOP; return result; }
    I2C1->CR1 |= I2C_CR1_START;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_SB) && --t) { }
    if (!t) return result;
    I2C1->DR = (addr7 << 1) | 1;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_ADDR) && --t) { }
    if (!t) { I2C1->CR1 |= I2C_CR1_STOP; return result; }
    (void)I2C1->SR2;
    I2C1->CR1 &= ~I2C_CR1_ACK;
    t = 50000;
    while(!(I2C1->SR1 & I2C_SR1_RXNE) && --t) { }
    if (!t) { I2C1->CR1 |= I2C_CR1_STOP; I2C1->CR1 |= I2C_CR1_ACK; return result; }
    result = I2C1->DR;
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR1 |= I2C_CR1_ACK;
    return result;
}

// Writes a raw byte to the PCF8574 I/O expander.
static int PCF_WriteRaw(uint8_t data) {
    pcf_output_cache = data;
    return I2C1_WriteByte(PCF8574_ADDR7, data);
}

// Sends a 4-bit nibble to the LCD via the PCF8574.
static void LCD_WriteNibble(uint8_t nibble) {
    PCF_WriteRaw(nibble | EN_BIT);
    delay_us(100);
    PCF_WriteRaw(nibble & ~EN_BIT);
    delay_us(100);
}

// Sends a full byte (command or data) to the LCD in two nibbles.
static void LCD_SendByte(uint8_t data, uint8_t flags) {
    uint8_t hi = (data & 0xF0) | flags;
    uint8_t lo = ((data << 4) & 0xF0) | flags;
    LCD_WriteNibble(hi);
    LCD_WriteNibble(lo);
}

// Initializes the 16x2 LCD in 4-bit mode.
static void LCD_Init(void) {
    delay_ms(50);
    PCF_WriteRaw(BL_BIT);
    LCD_WriteNibble(0x30 | BL_BIT); delay_ms(5);
    LCD_WriteNibble(0x30 | BL_BIT); delay_ms(1);
    LCD_WriteNibble(0x30 | BL_BIT); delay_ms(1);
    LCD_WriteNibble(0x20 | BL_BIT); delay_ms(1);
    LCD_SendByte(0x28, BL_BIT);
    LCD_SendByte(0x0C, BL_BIT);
    LCD_SendByte(0x06, BL_BIT);
    LCD_SendByte(0x01, BL_BIT);
    delay_ms(2);
}

// Sets the LCD cursor to a specified row and column.
static void LCD_SetCursor(uint8_t r, uint8_t c) { LCD_SendByte((r==0?0x80:0xC0)+c, BL_BIT); }

// Prints a null-terminated string to the LCD.
static void LCD_Print(const char* s) { while(*s) LCD_SendByte(*s++, RS_BIT | BL_BIT); }

// Clears the LCD display.
static void LCD_Clear(void) { LCD_SendByte(0x01, BL_BIT); delay_ms(2); }

// Keypad character map.
char keymap[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

// Initializes GPIO pins for the 4x4 matrix keypad.
static void Keypad_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    for (int pin=0; pin<=3; ++pin) {
        GPIOB->MODER &= ~(3 << (pin*2));
        GPIOB->MODER |=  (1 << (pin*2));
    }
    for (int pin=4; pin<=7; ++pin) {
        GPIOB->MODER &= ~(3 << (pin*2));
        GPIOB->PUPDR &= ~(3 << (pin*2));
        GPIOB->PUPDR |=  (1 << (pin*2));
    }
    GPIOB->ODR |= 0x0F;
}

// Scans the keypad matrix and returns the pressed key, with debouncing.
static char Keypad_Scan(void) {
    for (int r=0; r<4; ++r) {
        GPIOB->ODR |= 0x0F;
        GPIOB->ODR &= ~(1<<r);
        delay_us(50);
        for (int c=0; c<4; ++c) {
            if (!(GPIOB->IDR & (1 << (c+4)))) {
                delay_ms(10);
                if (!(GPIOB->IDR & (1 << (c+4)))) {
                    while (!(GPIOB->IDR & (1 << (c+4)))) { }
                    return keymap[r][c];
                }
            }
        }
    }
    return 0;
}

// Initializes the buzzer GPIO pin.
void Buzzer_Init(void){ RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; GPIOA->MODER &= ~(3 << (9*2)); GPIOA->MODER |= (1 << (9*2)); }

// Turns the buzzer on.
void Buzzer_On(void){ GPIOA->BSRR = (1 << 9); }

// Turns the buzzer off.
void Buzzer_Off(void){ GPIOA->BSRR = (1 << (9 + 16)); }

// Generates a short click sound with the buzzer.
void Buzzer_Click(void){ Buzzer_On(); delay_ms(100); Buzzer_Off(); }

// Initializes the LED GPIO pin.
void LED_Init(void){ RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; GPIOC->MODER &= ~(3 << (13*2)); GPIOC->MODER |= (1 << (13*2)); }

// Turns the LED on.
void LED_On(void){ GPIOC->BSRR = (1 << 13); }

// Turns the LED off.
void LED_Off(void){ GPIOC->BSRR = (1 << (13 + 16)); }

// Toggles the state of the LED.
void LED_Toggle(void){ GPIOC->ODR ^= (1<<13); }

// Initializes the button GPIO pin.
void BTN_Init(void){ RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; GPIOA->MODER &= ~(3 << (1*2)); GPIOA->PUPDR &= ~(3 << (1*2)); GPIOA->PUPDR |= (1 << (1*2)); }

// Checks if the user button is currently pressed.
static inline uint8_t BTN_IsPressed(void){ return (GPIOA->IDR & (1<<1)) ? 0 : 1; }

// Initializes USART2 to receive data from the GPS module.
void USART2_Init_GPS(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
    GPIOA->AFR[0] &= ~((0xF << (4*2)) | (0xF << (4*3)));
    GPIOA->AFR[0] |=  ((7 << (4*2)) | (7 << (4*3)));
    USART2->BRR = 0x683;
    USART2->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
    NVIC_EnableIRQ(USART2_IRQn);
}

// USART2 interrupt handler to buffer incoming NMEA sentences.
void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        char ch = (char)(USART2->DR & 0xFF);
        if (ch == '$') {
            nmea_index = 0;
            nmea_buffer[nmea_index++] = ch;
        } else if (nmea_index > 0 && nmea_index < NMEA_BUFFER_SIZE - 1) {
            nmea_buffer[nmea_index++] = ch;
            if (ch == '\n') {
                nmea_buffer[nmea_index] = '\0';
                sentence_ready = 1;
            }
        }
    }
}

// Converts raw UTC float from GPS to a formatted HH:MM:SS string.
static void extract_utc_time(float raw_time) {
    if (raw_time <= 0.0f) { strcpy((char*)gps_time, "00:00:00"); return; }
    int hh = (int)(raw_time / 10000);
    int mm = ((int)(raw_time / 100)) % 100;
    int ss = ((int)raw_time) % 100;
    snprintf((char*)gps_time, sizeof(gps_time), "%02d:%02d:%02d", hh, mm, ss);
}

// Converts NMEA coordinate format (DDMM.MMMM) to decimal degrees.
static float nmea_to_decimal(float nmea_coord, char direction) {
    if (nmea_coord <= 0.0f) return 0.0f;
    int degrees = (int)(nmea_coord / 100);
    float minutes = nmea_coord - (degrees * 100);
    float decimal = degrees + (minutes / 60.0f);
    if (direction == 'S' || direction == 'W') decimal *= -1.0f;
    return decimal;
}

// Parses a GPGGA sentence to extract key GPS data.
void parse_gga_sentence(const char *gga) {
    if (!gga) return;
    float utc_raw=0.0f, lat_raw=0.0f, lon_raw=0.0f;
    char ns='N', ew='E';
    int fix=0, sats=0;
    int parsed = sscanf(gga, "$GPGGA,%f,%f,%c,%f,%c,%d,%d", &utc_raw, &lat_raw, &ns, &lon_raw, &ew, &fix, &sats);
    if (parsed >= 5) {
        extract_utc_time(utc_raw);
        gps_latitude = nmea_to_decimal(lat_raw, ns);
        gps_longitude = nmea_to_decimal(lon_raw, ew);
        if (parsed >= 7) {
            gps_fix_quality = fix;
            gps_satellites = sats;
        }
    }
}

// MPU6050 register map and configuration.
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG  0x1B
#define ACC_SENS  16384.0f
#define GYR_SENS  131.0f

// Writes a single byte to an MPU6050 register.
static void mpu_write8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    I2C1_WriteBytes(MPU6050_ADDR7, buf, 2);
}

// Reads a single byte from an MPU6050 register.
static uint8_t mpu_read8(uint8_t reg) {
    return I2C1_ReadReg8(MPU6050_ADDR7, reg);
}

// Reads a 16-bit big-endian value from the MPU6050.
static int16_t mpu_read16be(uint8_t reg) {
    uint8_t hi = mpu_read8(reg);
    uint8_t lo = mpu_read8(reg+1);
    return (int16_t)((hi << 8) | lo);
}

// Initializes the MPU6050 sensor by waking it up and setting default scales.
void MPU6050_Init(void) {
    mpu_write8(REG_PWR_MGMT_1, 0x00);
    mpu_write8(REG_ACCEL_CONFIG, 0x00);
    mpu_write8(REG_GYRO_CONFIG, 0x00);
    who_am_i = mpu_read8(REG_WHO_AM_I);
}

// Reads one set of raw accelerometer and gyroscope data.
void MPU6050_ReadRawOnce(void) {
    ax_raw = mpu_read16be(REG_ACCEL_XOUT_H);
    ay_raw = mpu_read16be(REG_ACCEL_XOUT_H + 2);
    az_raw = mpu_read16be(REG_ACCEL_XOUT_H + 4);
    gx_raw = mpu_read16be(REG_GYRO_XOUT_H);
    gy_raw = mpu_read16be(REG_GYRO_XOUT_H + 2);
    gz_raw = mpu_read16be(REG_GYRO_XOUT_H + 4);
}

// Computes resultant acceleration, gyro rate, and tilt angle from raw data.
void MPU6050_ComputeMetrics(void) {
    float ax = ax_raw / ACC_SENS;
    float ay = ay_raw / ACC_SENS;
    float az = az_raw / ACC_SENS;
    float gx = gx_raw / GYR_SENS;
    float gy = gy_raw / GYR_SENS;
    float gz = gz_raw / GYR_SENS;
    A_res_g = sqrtf(ax*ax + ay*ay + az*az);
    G_res_dps = sqrtf(gx*gx + gy*gy + gz*gz);
    float denom = (A_res_g > 1e-3f) ? A_res_g : 1e-3f;
    float c = fabsf(az)/denom; if (c>1.0f) c = 1.0f;
    tilt_deg = acosf(c) * (180.0f/3.1415926f);
}

// MPU state machine variables for accident detection.
typedef enum { ST_MONITOR=0, ST_CONFIRM, ST_GRACE } mpu_state_t;
static mpu_state_t mpu_state = ST_MONITOR;
static uint32_t t_confirm_start = 0, t_tilt_streak_start = 0, t_grace_start = 0, t_last_led = 0;
static uint8_t alert_send_flag = 0;

// Processes sensor data through the accident detection state machine.
void MPU6050_SampleAndProcess(void) {
    MPU6050_ReadRawOnce();
    MPU6050_ComputeMetrics();
    switch (mpu_state) {
        case ST_MONITOR:
            if (A_res_g > IMPACT_THRESHOLD_G && G_res_dps > FALL_THRESHOLD_DPS) {
                t_confirm_start = millis();
                t_tilt_streak_start = 0;
                mpu_state = ST_CONFIRM;
            }
            break;
        case ST_CONFIRM: {
            uint32_t elapsed = millis() - t_confirm_start;
            if (tilt_deg > TILT_THRESHOLD_DEG) {
                if (t_tilt_streak_start == 0) t_tilt_streak_start = millis();
                if ((millis() - t_tilt_streak_start) >= TILT_STREAK_MS) {
                    accident_lat = gps_latitude;
                    accident_lon = gps_longitude;
                    accident_snapshot_taken = 1;
                    t_grace_start = millis();
                    t_last_led = millis();
                    LED_Off(); Buzzer_Off();
                    mpu_state = ST_GRACE;
                }
            } else {
                t_tilt_streak_start = 0;
            }
            if (elapsed >= CONFIRM_WINDOW_MS) mpu_state = ST_MONITOR;
        } break;
        case ST_GRACE:
            if ((millis() - t_last_led) >= LED_BLINK_MS) {
                t_last_led = millis();
                LED_Toggle();
            }
            Buzzer_On();
            if ((millis() - t_grace_start) >= GRACE_WINDOW_MS) {
                LED_Off(); Buzzer_Off();
                alert_send_flag = 1;
                mpu_state = ST_MONITOR;
            }
            break;
        default:
            mpu_state = ST_MONITOR;
            break;
    }
}

// Application state definitions.
typedef enum {
    STATE_STARTING,
    STATE_PASSWORD_ENTRY,
    STATE_LOGGED_IN_PROMPT,
    STATE_LOGGED_IN,
    STATE_CHANGE_PASSWORD,
    STATE_ALARM,
    STATE_ACCIDENT,
    STATE_OFF
} AppState;
static AppState state = STATE_OFF;
char sys_seq[4] = {0}; int sys_seq_idx = 0;

// Puts the system into a safe, powered-off state.
void PowerOff_System(void) {
    LED_Off();
    Buzzer_Off();
    LCD_Clear();
    LCD_SetCursor(0,3); LCD_Print("SYSTEM OFF");
    PCF_WriteRaw(0x00);
    state = STATE_OFF;
}

// Displays formatted latitude and longitude coordinates on the LCD.
static void ShowCoordinatesOnLCD(float lat, float lon) {
    char buf[32];
    LCD_Clear();
    LCD_SetCursor(0,0);
    snprintf(buf, sizeof(buf), "LAT: %.5f", lat);
    LCD_Print(buf);
    LCD_SetCursor(1,0);
    snprintf(buf, sizeof(buf), "LON: %.5f", lon);
    LCD_Print(buf);
}

// Main application entry point and logic loop.
int main(void) {
    SystemCoreClockUpdate();
    DWT_DelayInit();
    SysTick_Config(SystemCoreClock / 1000);

    // Initialize all hardware peripherals.
    LED_Init(); Buzzer_Init(); BTN_Init();
    I2C1_Init_shared();
    PCF_WriteRaw(0x00);
    LCD_Init();
    Keypad_Init();
    USART2_Init_GPS();
    MPU6050_Init();

    // Local variables for the main loop state machine.
    char input_buffer[PASSWORD_LEN+1];
    int input_idx = 0;
    int attempts_left = 2;
    uint32_t timeout_counter = 0;
    char last_key = 0;
    int state_changed = 1;
    uint32_t t_last_sample = millis();

    PowerOff_System();

    // Infinite loop containing the main application logic.
    while (1) {
        char key = Keypad_Scan();
        char key_pressed = (key && !last_key) ? key : 0;
        last_key = key;

        // Process special key sequences for power on/off.
        if (key_pressed) {
            if (sys_seq_idx < 3) {
                sys_seq[sys_seq_idx++] = key_pressed;
                sys_seq[sys_seq_idx] = 0;
            } else {
                sys_seq[0] = sys_seq[1];
                sys_seq[1] = sys_seq[2];
                sys_seq[2] = key_pressed;
                sys_seq[3] = 0;
            }
            if (strcmp(sys_seq, "BBB") == 0) {
                PowerOff_System();
                memset(sys_seq,0,sizeof(sys_seq)); sys_seq_idx=0;
                key_pressed = 0;
            }
            if (state == STATE_OFF && strcmp(sys_seq, "AAA") == 0) {
                state = STATE_STARTING; state_changed = 1;
                memset(sys_seq,0,sizeof(sys_seq)); sys_seq_idx=0;
            }
        }

        if (key_pressed && state != STATE_ALARM && state != STATE_ACCIDENT) Buzzer_Click();

        // Main application state machine.
        switch (state) {
            case STATE_STARTING:
                if (state_changed) {
                    LED_Off(); Buzzer_Off();
                    LCD_Clear();
                    LCD_SetCursor(0,4); LCD_Print("WELCOME");
                    timeout_counter = 3000;
                    state_changed = 0;
                }
                if (timeout_counter == 0) {
                    attempts_left = 2;
                    state = STATE_PASSWORD_ENTRY;
                    state_changed = 1;
                }
                break;

            case STATE_PASSWORD_ENTRY:
                if (state_changed) {
                    LCD_Clear(); LCD_SetCursor(0,0); LCD_Print("Enter Password:");
                    LCD_SetCursor(1,0);
                    input_idx = 0; memset(input_buffer,0,sizeof(input_buffer));
                    timeout_counter = PW_ENTRY_TIMEOUT_MS;
                    state_changed = 0;
                }
                if (key_pressed) {
                    timeout_counter = PW_ENTRY_TIMEOUT_MS;
                    if (input_idx < PASSWORD_LEN) {
                        input_buffer[input_idx++] = key_pressed;
                        LCD_SendByte('*', RS_BIT | BL_BIT);
                    }
                    if (input_idx == PASSWORD_LEN) {
                        if (strcmp(input_buffer, password) == 0) {
                            state = STATE_LOGGED_IN_PROMPT; state_changed = 1;
                        } else {
                            attempts_left--;
                            if (attempts_left > 0) {
                                for (int i=0;i<3;i++) { Buzzer_Click(); delay_ms(50); }
                                LCD_Clear(); LCD_SetCursor(0,1); LCD_Print("Wrong Password");
                                delay_ms(1200);
                                state = STATE_PASSWORD_ENTRY; state_changed = 1;
                            } else {
                                state = STATE_ALARM; state_changed = 1;
                            }
                        }
                    }
                }
                if (timeout_counter == 0) PowerOff_System();
                break;

            case STATE_LOGGED_IN_PROMPT:
                if (state_changed) {
                    LCD_Clear();
                    LCD_SetCursor(0,0); LCD_Print("Change PW? (*/B)");
                    LCD_SetCursor(1,0); LCD_Print("DDD to shutdown");
                    LED_On();
                    input_idx = 0; memset(input_buffer,0,sizeof(input_buffer));
                    timeout_counter = PW_CHANGE_TIMEOUT_MS;
                    state_changed = 0;
                }
                if (key_pressed == 'B') {
                    state = STATE_LOGGED_IN; state_changed = 1;
                } else if (key_pressed) {
                    if (input_idx < 3) input_buffer[input_idx++] = key_pressed;
                    if (input_idx == 3) {
                        if (strcmp(input_buffer, "*") == 0) {
                            state = STATE_CHANGE_PASSWORD; state_changed = 1;
                        } else if (strcmp(input_buffer, "DDD") == 0) {
                            PowerOff_System();
                        }
                        input_idx = 0; memset(input_buffer,0,sizeof(input_buffer));
                    }
                }
                if (timeout_counter == 0) {
                    state = STATE_LOGGED_IN; state_changed = 1;
                }
                break;

            case STATE_LOGGED_IN:
                if (state_changed) {
                    LCD_Clear();
                    LCD_SetCursor(0,2); LCD_Print("System Ready");
                    LCD_SetCursor(1,1); LCD_Print("Press DDD to Off");
                    input_idx = 0; memset(input_buffer,0,sizeof(input_buffer));
                    state_changed = 0;
                }
                if (key_pressed) {
                    if (input_idx < 3) input_buffer[input_idx++] = key_pressed;
                    if (input_idx == 3) {
                        if (strcmp(input_buffer, "DDD") == 0) {
                            PowerOff_System();
                        }
                        input_idx = 0; memset(input_buffer,0,sizeof(input_buffer));
                    }
                }
                break;

            case STATE_CHANGE_PASSWORD:
                if (state_changed) {
                    LCD_Clear(); LCD_SetCursor(0,0); LCD_Print("Enter New PW:");
                    LCD_SetCursor(1,0);
                    input_idx = 0; memset(input_buffer,0,sizeof(input_buffer));
                    state_changed = 0;
                }
                if (key_pressed) {
                    if (input_idx < PASSWORD_LEN) {
                        input_buffer[input_idx++] = key_pressed;
                        LCD_SendByte('*', RS_BIT | BL_BIT);
                    }
                    if (input_idx == PASSWORD_LEN) {
                        strcpy(password, input_buffer);
                        LCD_Clear(); LCD_SetCursor(0,0); LCD_Print("Password Changed");
                        delay_ms(2000);
                        state = STATE_LOGGED_IN; state_changed = 1;
                    }
                }
                break;

            case STATE_ALARM:
                if (state_changed) {
                    LCD_Clear(); LCD_SetCursor(0,4); LCD_Print("Theft Detected");
                    input_idx = 0; memset(input_buffer,0,sizeof(input_buffer));
                    state_changed = 0;
                }
                Buzzer_On();
                if (key_pressed) {
                    if (input_idx < 3) input_buffer[input_idx++] = key_pressed;
                    if (input_idx == 3) {
                        if (strcmp(input_buffer, "###") == 0) {
                            Buzzer_Off();
                            PowerOff_System();
                        }
                        input_idx = 0; memset(input_buffer,0,sizeof(input_buffer));
                    }
                }
                break;

            case STATE_ACCIDENT:
                if (state_changed) {
                    LCD_Clear();
                    LCD_SetCursor(0,1); LCD_Print("Accident Detected");
                    t_last_sample = millis();
                    state_changed = 0;
                    Buzzer_On();
                    LED_On();
                }
                if ((millis() - t_last_sample) < 3000) {
                    if (((millis() / 300) & 1) == 0) LED_On(); else LED_Off();
                    Buzzer_On();
                } else {
                    Buzzer_Off();
                    LED_Off();
                    ShowCoordinatesOnLCD(accident_lat, accident_lon);
                }
                if (BTN_IsPressed()) {
                    Buzzer_Off();
                    accident_snapshot_taken = 0;
                    state = STATE_LOGGED_IN; state_changed = 1;
                }
                if (key_pressed) {
                    if (input_idx < 3) input_buffer[input_idx++] = key_pressed;
                    if (input_idx == 3) {
                        if (strcmp(input_buffer, "DDD") == 0) {
                            PowerOff_System();
                        }
                        input_idx = 0; memset(input_buffer,0,sizeof(input_buffer));
                    }
                }
                break;

            case STATE_OFF:
            default:
                break;
        }

        // Handles timeouts and main loop delay.
        if (state != STATE_OFF) {
            uint32_t step = (state == STATE_ALARM || state == STATE_ACCIDENT) ? 0 : 20;
            if (step) delay_ms(step);
            if (timeout_counter > 0) {
                timeout_counter -= (timeout_counter < step) ? timeout_counter : step;
            }
        } else {
            delay_ms(100);
        }

        // Checks for and parses complete GPS sentences.
        if (sentence_ready) {
            if (strncmp(nmea_buffer, "$GPGGA", 6) == 0) {
                parse_gga_sentence(nmea_buffer);
            }
            sentence_ready = 0;
            nmea_index = 0;
        }

        // Periodically samples the MPU6050 when the system is not off.
        if (state != STATE_OFF) {
            if ((millis() - t_last_sample) >= SAMPLE_PERIOD_MS) {
                t_last_sample = millis();
                MPU6050_SampleAndProcess();
            }
        }

        // Transitions to the accident state if the MPU state machine flags an alert.
        if (alert_send_flag) {
            alert_send_flag = 0;
            if (!accident_snapshot_taken) {
                accident_lat = gps_latitude;
                accident_lon = gps_longitude;
                accident_snapshot_taken = 1;
            }
            state = STATE_ACCIDENT;
            state_changed = 1;
            Buzzer_On();
        }

        if (accident_snapshot_taken && state == STATE_LOGGED_IN && alert_send_flag == 0) {
        }
    }
    return 0;
}