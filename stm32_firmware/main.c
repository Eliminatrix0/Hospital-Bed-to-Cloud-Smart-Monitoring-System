#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/*
 * --- FINAL FEATURES ---
 * 1. System Starts in LOCK State (Pass: 4976).
 * 2. BUZZER LOGIC FIXED: Active Low (0=ON, 1=OFF).
 * 3. Logic:
 * - Wrong PIN: Beep 1 sec.
 * - Temp > 100F: Beep Continuous + 5s Update.
 * - Temp < 100F: Silent + 10s Update.
 */

// --- Configuration Macros ---
#define BUZZER_PORT     GPIOD
#define BUZZER_PIN      12
#define BUZZER_MASK     (1 << BUZZER_PIN)

// *** FIXED BUZZER MACROS (Active Low) ***
// Clear Bit (0) to turn ON, Set Bit (1) to turn OFF
#define BUZZER_ON()     (BUZZER_PORT->ODR &= ~BUZZER_MASK)
#define BUZZER_OFF()    (BUZZER_PORT->ODR |= BUZZER_MASK)

#define DHT_PORT        GPIOA
#define DHT_PIN         1
#define DHT_PIN_MASK    (1 << DHT_PIN)
#define TIMEOUT_VAL     50000

#define DS18B20_PORT      GPIOA
#define DS18B20_PIN       0
#define DS18B20_PIN_MASK  (1 << DS18B20_PIN)

#define I2C_PERIPH    I2C1
#define I2C_GPIO_PORT GPIOB
#define I2C_SCL_PIN   6
#define I2C_SDA_PIN   7

#define BMP180_ADDR    0xEE 
#define BMP180_OSS     0    
#define SSD1306_ADDR   0x78
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64

#define MAX30102_ADDR              0xAE
#define MAX30102_REG_FIFO_WR_PTR   0x04
#define MAX30102_REG_FIFO_RD_PTR   0x06
#define MAX30102_REG_FIFO_DATA     0x07
#define MAX30102_REG_FIFO_CONFIG   0x08
#define MAX30102_REG_MODE_CONFIG   0x09
#define MAX30102_REG_SPO2_CONFIG   0x0A
#define MAX30102_REG_LED1_PA       0x0C
#define MAX30102_REG_LED2_PA       0x0D
#define MAX30102_BUFFER_LENGTH     100

#define HX711_PORT      GPIOB
#define HX711_SCK_PIN   0
#define HX711_DOUT_PIN  1
#define HX711_SCK_MASK  (1 << HX711_SCK_PIN)
#define HX711_DOUT_MASK (1 << HX711_DOUT_PIN)

// --- Global Variables ---
volatile int Current_Screen = 0; 
volatile uint32_t Current_Delay_Interval = 10000; // Default 10s

// Lock System Variables
char Password_Buffer[5] = {0};      
uint8_t Pass_Index = 0;             
const char Correct_Pass[] = "4976"; 

// Sensor Variables
volatile float Temperature_DHT = 0.0f;
volatile float Humidity = 0.0f;
volatile float Temperature_DS = 0.0f;
volatile float Temperature_DS_F = 0.0f;
volatile float Weight_g = 0.0f;
volatile float Weight_kg = 0.0f; 
volatile float current_SpO2 = 0.0f;
volatile float current_HR = 0.0f;
volatile float Temperature_BMP = 0.0f;
volatile float Pressure_BMP = 0.0f;
volatile float Altitude_BMP = 0.0f;

// Internal Globals
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2, SUM;
short AC1, AC2, AC3, B1, B2, MB, MC, MD;
unsigned short AC4, AC5, AC6;
char OLED_Buffer[64]; 
uint32_t aun_ir_buffer[MAX30102_BUFFER_LENGTH];
uint32_t aun_red_buffer[MAX30102_BUFFER_LENGTH];
long HX711_Offset = 0;
float CALIBRATION_FACTOR = 415.0f;

char keyMap[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

// --- FONT DATA ---
const uint8_t SSD1306_Font_7x10[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Space
    0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, // !
    0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, // "
    0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, // #
    0x00, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, // $
    0x00, 0x23, 0x13, 0x08, 0x64, 0x62, 0x00, // %
    0x00, 0x36, 0x49, 0x55, 0x22, 0x50, 0x00, // &
    0x00, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, // '
    0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x00, // (
    0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00, // )
    0x00, 0x08, 0x2A, 0x1C, 0x2A, 0x08, 0x00, // *
    0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, // +
    0x00, 0x00, 0x50, 0x30, 0x00, 0x00, 0x00, // ,
    0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, // -
    0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, // .
    0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, // /
    0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, // 0
    0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, // 1
    0x00, 0x42, 0x61, 0x51, 0x49, 0x46, 0x00, // 2
    0x00, 0x21, 0x41, 0x45, 0x4B, 0x31, 0x00, // 3
    0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, // 4
    0x00, 0x27, 0x45, 0x45, 0x45, 0x39, 0x00, // 5
    0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00, // 6
    0x00, 0x01, 0x71, 0x09, 0x05, 0x03, 0x00, // 7
    0x00, 0x36, 0x49, 0x49, 0x49, 0x36, 0x00, // 8
    0x00, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, // 9
    0x00, 0x00, 0x36, 0x36, 0x00, 0x00, 0x00, // :
    0x00, 0x00, 0x56, 0x36, 0x00, 0x00, 0x00, // ;
    0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00, // <
    0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, // =
    0x00, 0x41, 0x22, 0x14, 0x08, 0x00, 0x00, // >
    0x00, 0x02, 0x01, 0x51, 0x09, 0x06, 0x00, // ?
    0x00, 0x32, 0x49, 0x79, 0x41, 0x3E, 0x00, // @
    0x00, 0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00, // A
    0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, // B
    0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, // C
    0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00, // D
    0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, // E
    0x00, 0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, // F
    0x00, 0x3E, 0x41, 0x41, 0x51, 0x32, 0x00, // G
    0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, // H
    0x00, 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, // I
    0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, // J
    0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, // K
    0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, // L
    0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00, // M
    0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, // N
    0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, // O
    0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, // P
    0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, // Q
    0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, // R
    0x00, 0x46, 0x49, 0x49, 0x49, 0x31, 0x00, // S
    0x00, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x00, // T
    0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, // U
    0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, // V
    0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00, // W
    0x00, 0x63, 0x14, 0x08, 0x14, 0x63, 0x00, // X
    0x00, 0x07, 0x08, 0x70, 0x08, 0x07, 0x00, // Y
    0x00, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, // Z
    0x00, 0x7F, 0x41, 0x41, 0x00, 0x00, 0x00, // [
    0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, // \ (Backslash)
    0x00, 0x41, 0x41, 0x7F, 0x00, 0x00, 0x00, // ]
    0x00, 0x04, 0x02, 0x01, 0x02, 0x04, 0x00, // ^
    0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, // _
    0x00, 0x01, 0x02, 0x04, 0x00, 0x00, 0x00, // `
    0x00, 0x20, 0x54, 0x54, 0x54, 0x78, 0x00, // a
    0x00, 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00, // b
    0x00, 0x38, 0x44, 0x44, 0x44, 0x20, 0x00, // c
    0x00, 0x38, 0x44, 0x44, 0x48, 0x7F, 0x00, // d
    0x00, 0x38, 0x54, 0x54, 0x54, 0x18, 0x00, // e
    0x00, 0x08, 0x7E, 0x09, 0x01, 0x02, 0x00, // f
    0x00, 0x08, 0x14, 0x54, 0x54, 0x3C, 0x00, // g
    0x00, 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, // h
    0x00, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00, // i
    0x00, 0x20, 0x40, 0x44, 0x3D, 0x00, 0x00, // j
    0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, // k
    0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00, // l
    0x00, 0x7C, 0x04, 0x18, 0x04, 0x78, 0x00, // m
    0x00, 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00, // n
    0x00, 0x38, 0x44, 0x44, 0x44, 0x38, 0x00, // o
    0x00, 0x7C, 0x14, 0x14, 0x14, 0x08, 0x00, // p
    0x00, 0x08, 0x14, 0x14, 0x18, 0x7C, 0x00, // q
    0x00, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x00, // r
    0x00, 48, 0x54, 0x54, 0x54, 0x20, 0x00,    // s
    0x00, 0x04, 0x3F, 0x44, 0x40, 0x20, 0x00, // t
    0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00, // u
    0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00, // v
    0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00, // w
    0x00, 0x44, 0x28, 0x10, 0x28, 0x44, 0x00, // x
    0x00, 0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00, // y
    0x00, 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, // z
    0x00, 0x08, 0x36, 0x41, 0x00, 0x00, 0x00, // {
    0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, // |
    0x00, 0x41, 0x36, 0x08, 0x00, 0x00, 0x00, // }
    0x00, 0x08, 0x04, 0x08, 0x10, 0x08, 0x00, // ~
};

// --- TIM2 & Delays ---
void TIM2_Config(void) { RCC->APB1ENR |= (1 << 0); TIM2->PSC = 16 - 1; TIM2->ARR = 0xFFFF; TIM2->CR1 |= (1 << 0); }
void Delay_us(uint16_t us) { TIM2->CNT = 0; while (TIM2->CNT < us); }
void Delay_ms(uint16_t ms) { for (uint16_t i = 0; i < ms; i++) Delay_us(1000); }

// --- I2C ---
void I2C1_Config(void) {
    RCC->AHB1ENR |= (1 << 1); RCC->APB1ENR |= (1 << 21);
    I2C_GPIO_PORT->MODER |= (2 << (I2C_SCL_PIN * 2)) | (2 << (I2C_SDA_PIN * 2));
    I2C_GPIO_PORT->OTYPER |= ((1 << I2C_SCL_PIN) | (1 << I2C_SDA_PIN));
    I2C_GPIO_PORT->OSPEEDR |= (3 << (I2C_SCL_PIN * 2)) | (3 << (I2C_SDA_PIN * 2));
    I2C_GPIO_PORT->PUPDR |= (1 << (I2C_SCL_PIN * 2)) | (1 << (I2C_SDA_PIN * 2));
    I2C_GPIO_PORT->AFR[0] |= (4 << (I2C_SCL_PIN * 4)) | (4 << (I2C_SDA_PIN * 4));
    I2C_PERIPH->CR1 |= I2C_CR1_SWRST; I2C_PERIPH->CR1 &= ~I2C_CR1_SWRST;
    I2C_PERIPH->CR2 = 16; I2C_PERIPH->CCR = 80; I2C_PERIPH->TRISE = 17; I2C_PERIPH->CR1 |= I2C_CR1_PE;
}

void I2C_Start(void) { I2C_PERIPH->CR1 |= I2C_CR1_START; while (!(I2C_PERIPH->SR1 & I2C_SR1_SB)) {}; }
void I2C_Stop(void) { I2C_PERIPH->CR1 |= I2C_CR1_STOP; }
void I2C_Write_Addr(uint8_t addr) { I2C_PERIPH->DR = addr; while (!(I2C_PERIPH->SR1 & I2C_SR1_ADDR)) {}; (void)I2C_PERIPH->SR1; (void)I2C_PERIPH->SR2; }
void I2C_Write_Data(uint8_t data) { while (!(I2C_PERIPH->SR1 & I2C_SR1_TXE)) {}; I2C_PERIPH->DR = data; while (!(I2C_PERIPH->SR1 & I2C_SR1_BTF)) {}; }
uint8_t I2C_Read_ACK(void) { I2C_PERIPH->CR1 |= I2C_CR1_ACK; while (!(I2C_PERIPH->SR1 & I2C_SR1_RXNE)) {}; return I2C_PERIPH->DR; }
uint8_t I2C_Read_NACK(void) { I2C_PERIPH->CR1 &= ~I2C_CR1_ACK; I2C_Stop(); while (!(I2C_PERIPH->SR1 & I2C_SR1_RXNE)) {}; return I2C_PERIPH->DR; }

// --- BMP180 Driver ---
void BMP180_WriteReg(uint8_t reg, uint8_t value) { I2C_Start(); I2C_Write_Addr(BMP180_ADDR); I2C_Write_Data(reg); I2C_Write_Data(value); I2C_Stop(); }
uint16_t BMP180_Read16(uint8_t reg) { I2C_Start(); I2C_Write_Addr(BMP180_ADDR); I2C_Write_Data(reg); I2C_Start(); I2C_Write_Addr(BMP180_ADDR | 1); uint8_t msb = I2C_Read_ACK(); uint8_t lsb = I2C_Read_NACK(); return (msb << 8) | lsb; }
void BMP180_Init(void) { AC1 = (short)BMP180_Read16(0xAA); AC2 = (short)BMP180_Read16(0xAC); AC3 = (short)BMP180_Read16(0xAE); AC4 = (unsigned short)BMP180_Read16(0xB0); AC5 = (unsigned short)BMP180_Read16(0xB2); AC6 = (unsigned short)BMP180_Read16(0xB4); B1 = (short)BMP180_Read16(0xB6); B2 = (short)BMP180_Read16(0xB8); MB = (short)BMP180_Read16(0xBA); MC = (short)BMP180_Read16(0xBC); MD = (short)BMP180_Read16(0xBE); }
void BMP180_Read_Convert(void) {
    long X1, X2, X3, B3, B5, B6, UT, UP, p; unsigned long B4, B7;
    BMP180_WriteReg(0xF4, 0x2E); Delay_ms(5); UT = (long)BMP180_Read16(0xF6);
    BMP180_WriteReg(0xF4, 0x34 | (BMP180_OSS << 6)); Delay_ms(5 + (BMP180_OSS * 5));
    I2C_Start(); I2C_Write_Addr(BMP180_ADDR); I2C_Write_Data(0xF6); I2C_Start(); I2C_Write_Addr(BMP180_ADDR | 1);
    uint8_t msb = I2C_Read_ACK(); uint8_t lsb = I2C_Read_ACK(); uint8_t xlsb = I2C_Read_NACK();
    UP = ((msb << 16) | (lsb << 8) | xlsb) >> (8 - BMP180_OSS);
    X1 = (UT - (long)AC6) * (long)AC5 / 32768; X2 = (long)MC * 2048 / (X1 + (long)MD); B5 = X1 + X2;
    Temperature_BMP = (float)((B5 + 8) / 16) / 10.0f;
    B6 = B5 - 4000; X1 = (B2 * (B6 * B6 / 4096)) / 2048; X2 = AC2 * B6 / 2048; X3 = X1 + X2;
    B3 = (((long)AC1 * 4 + X3) << BMP180_OSS) / 4; if (BMP180_OSS > 0) B3 = ((((long)AC1 * 4 + X3) << BMP180_OSS) + 2) / 4;
    X1 = AC3 * B6 / 8192; X2 = (B1 * (B6 * B6 / 4096)) / 65536; X3 = ((X1 + X2) + 2) / 4;
    B4 = AC4 * (unsigned long)(X3 + 32768) / 32768; B7 = ((unsigned long)UP - B3) * (50000 >> BMP180_OSS);
    if (B7 < 0x80000000) p = (B7 * 2) / B4; else p = (B7 / B4) * 2;
    X1 = (p / 256) * (p / 256); X1 = (X1 * 3038) / 65536; X2 = (-7357 * p) / 65536; p = p + (X1 + X2 + 3791) / 16;
    Pressure_BMP = (float)p / 100.0f;
    Altitude_BMP = 44330.0f * (1.0f - powf((Pressure_BMP / 1013.25f), (1.0f / 5.255f)));
}

// --- MAX30102 Driver ---
void MAX30102_WriteReg(uint8_t reg, uint8_t value) { I2C_Start(); I2C_Write_Addr(MAX30102_ADDR); I2C_Write_Data(reg); I2C_Write_Data(value); I2C_Stop(); }
void MAX30102_ReadRegs(uint8_t reg, uint8_t *data, uint8_t len) { I2C_Start(); I2C_Write_Addr(MAX30102_ADDR); I2C_Write_Data(reg); I2C_Start(); I2C_Write_Addr(MAX30102_ADDR | 1); for (uint8_t i = 0; i < len - 1; i++) data[i] = I2C_Read_ACK(); data[len - 1] = I2C_Read_NACK(); }
void MAX30102_Init(void) { MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG, 0x40); Delay_ms(500); MAX30102_WriteReg(MAX30102_REG_FIFO_CONFIG, 0x5F); MAX30102_WriteReg(MAX30102_REG_MODE_CONFIG, 0x03); MAX30102_WriteReg(MAX30102_REG_SPO2_CONFIG, 0x27); MAX30102_WriteReg(MAX30102_REG_LED1_PA, 0x24); MAX30102_WriteReg(MAX30102_REG_LED2_PA, 0x24); }

// --- SSD1306 GRAPHICS & TEXT ---
static uint8_t SSD_X=0, SSD_Y=0;
void SSD_Cmd(uint8_t c) { I2C_Start(); I2C_Write_Addr(SSD1306_ADDR); I2C_Write_Data(0x00); I2C_Write_Data(c); I2C_Stop(); }
void SSD_Init(void) { Delay_ms(100); SSD_Cmd(0xAE); SSD_Cmd(0xD5); SSD_Cmd(0x80); SSD_Cmd(0xA8); SSD_Cmd(0x3F); SSD_Cmd(0xD3); SSD_Cmd(0x00); SSD_Cmd(0x40); SSD_Cmd(0x8D); SSD_Cmd(0x14); SSD_Cmd(0x20); SSD_Cmd(0x00); SSD_Cmd(0xA1); SSD_Cmd(0xC8); SSD_Cmd(0xDA); SSD_Cmd(0x12); SSD_Cmd(0x81); SSD_Cmd(0xCF); SSD_Cmd(0xD9); SSD_Cmd(0xF1); SSD_Cmd(0xDB); SSD_Cmd(0x40); SSD_Cmd(0xA4); SSD_Cmd(0xA6); SSD_Cmd(0xAF); }
void SSD_XY(uint8_t x, uint8_t y) { SSD_X=x; SSD_Y=y; SSD_Cmd(0xB0|(y&0x07)); SSD_Cmd(0x00|(x&0xF)); SSD_Cmd(0x10|(x>>4)); }
void SSD_Clr(void) { 
    for(int page = 0; page < 8; page++) {
        SSD_Cmd(0xB0 | page); SSD_Cmd(0x00); SSD_Cmd(0x10);
        I2C_Start(); I2C_Write_Addr(SSD1306_ADDR); I2C_Write_Data(0x40);
        for(int i = 0; i < 128; i++) { while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = 0x00; }
        while(!(I2C1->SR1 & I2C_SR1_BTF)) {}; I2C_Stop();
    }
    SSD_XY(0,0);
}
void SSD_PutC(char c) {
    if (c < ' ' || c > '~') c = ' ';
    const uint8_t* font = &SSD1306_Font_7x10[(c - ' ') * 7];
    if (SSD_X > (SSD1306_WIDTH - 7)) { SSD_X = 0; SSD_Y += 1; if (SSD_Y > 7) SSD_Y = 0; }
    SSD_XY(SSD_X, SSD_Y); I2C_Start(); I2C_Write_Addr(SSD1306_ADDR); I2C_Write_Data(0x40);
    for (uint8_t i = 0; i < 7; i++) I2C_Write_Data(font[i]); I2C_Stop(); SSD_X += 7;
}
void SSD_Str(char* s) { while(*s) SSD_PutC(*s++); }
void SSD_Center(uint8_t y, char* s) {
    uint8_t len = strlen(s);
    uint8_t x = (128 - (len * 7)) / 2;
    SSD_XY(x, y);
    SSD_Str(s);
}
void SSD_Header(char* s) {
    uint8_t len = strlen(s);
    uint8_t x_start = (128 - (len * 7)) / 2;
    SSD_Cmd(0xB0 | 0); SSD_Cmd(0x00); SSD_Cmd(0x10);
    I2C_Start(); I2C_Write_Addr(SSD1306_ADDR); I2C_Write_Data(0x40);
    for(int i=0; i<x_start; i++) { while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = 0xFF; }
    for(int i=0; i<len; i++) {
        char c = s[i];
        if (c < ' ' || c > '~') c = ' ';
        const uint8_t* font = &SSD1306_Font_7x10[(c - ' ') * 7];
        for(int j=0; j<7; j++) { while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = ~font[j]; }
    }
    for(int i=x_start + (len*7); i<128; i++) { while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = 0xFF; }
    while(!(I2C1->SR1 & I2C_SR1_BTF)) {}; I2C_Stop();
}
uint8_t Scale_Nibble(uint8_t nib) {
    uint8_t res = 0;
    if(nib & 1) res |= 0x03;
    if(nib & 2) res |= 0x0C;
    if(nib & 4) res |= 0x30;
    if(nib & 8) res |= 0xC0;
    return res;
}
void SSD_BigStr(uint8_t x, uint8_t y, char* s) {
    uint8_t startX = x; int len = strlen(s);
    SSD_XY(startX, y); 
    I2C_Start(); I2C_Write_Addr(SSD1306_ADDR); I2C_Write_Data(0x40);
    for(int i=0; i<len; i++) {
        char c = s[i]; if (c < ' ' || c > '~') c = ' ';
        const uint8_t* font = &SSD1306_Font_7x10[(c - ' ') * 7];
        for(int j=0; j<7; j++) {
            uint8_t d = Scale_Nibble(font[j] & 0x0F);
            while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = d; while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = d;
        }
        while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = 0; while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = 0;
    }
    while(!(I2C1->SR1 & I2C_SR1_BTF)) {}; I2C_Stop();
    SSD_XY(startX, y+1); 
    I2C_Start(); I2C_Write_Addr(SSD1306_ADDR); I2C_Write_Data(0x40);
    for(int i=0; i<len; i++) {
        char c = s[i]; if (c < ' ' || c > '~') c = ' ';
        const uint8_t* font = &SSD1306_Font_7x10[(c - ' ') * 7];
        for(int j=0; j<7; j++) {
            uint8_t d = Scale_Nibble((font[j] >> 4) & 0x0F);
            while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = d; while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = d;
        }
        while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = 0; while(!(I2C1->SR1 & I2C_SR1_TXE)) {}; I2C1->DR = 0;
    }
    while(!(I2C1->SR1 & I2C_SR1_BTF)) {}; I2C_Stop();
}
void SSD_CenterBig(uint8_t y, char* s) {
    uint8_t len = strlen(s);
    uint8_t width = (len * 14) + (len * 2);
    uint8_t x = (128 - width) / 2;
    SSD_BigStr(x, y, s);
}

// --- KEYPAD & HARDWARE INIT ---
void Keypad_Init(void) {
    RCC->AHB1ENR |= (1 << 3); // GPIOD for Buzzer
    GPIOD->MODER &= ~(3 << (12 * 2)); GPIOD->MODER |= (1 << (12 * 2));
    
    // *** FIX: Set Output HIGH to silence Active Low Buzzer at startup ***
    BUZZER_OFF();
    
    for(int i=7; i<=10; i++) { GPIOE->MODER |= (1<<(i*2)); GPIOE->ODR |= (1<<i); }
    for(int i=11; i<=14; i++) { GPIOE->MODER &= ~(3<<(i*2)); GPIOE->PUPDR |= (1<<(i*2)); }
}

char Scan_Keypad(void) {
    for(int r = 0; r < 4; r++) {
        GPIOE->ODR |= (1<<7)|(1<<8)|(1<<9)|(1<<10); 
        GPIOE->ODR &= ~(1 << (7 + r));              
        Delay_us(50); // Stabilization
        for(int c = 0; c < 4; c++) {
            if(!(GPIOE->IDR & (1 << (11 + c)))) { 
                while(!(GPIOE->IDR & (1 << (11 + c)))) {}; 
                return keyMap[r][c];
            }
        }
    } return 0;
}

// --- HX711 ---
void HX711_Init(void) { HX711_PORT->MODER |= (1<<(HX711_SCK_PIN*2)); HX711_PORT->OTYPER &= ~HX711_SCK_MASK; HX711_PORT->PUPDR |= (1<<(HX711_DOUT_PIN*2)); HX711_PORT->ODR &= ~HX711_SCK_MASK; }
long HX711_Read(void) { long val=0; uint32_t t=500000; while((HX711_PORT->IDR & HX711_DOUT_MASK) && t--) {}; if(!t) return 0; for(int i=0; i<24; i++) { HX711_PORT->ODR|=HX711_SCK_MASK; Delay_us(1); val<<=1; if(HX711_PORT->IDR & HX711_DOUT_MASK) val++; HX711_PORT->ODR&=~HX711_SCK_MASK; Delay_us(1); } HX711_PORT->ODR|=HX711_SCK_MASK; Delay_us(1); HX711_PORT->ODR&=~HX711_SCK_MASK; Delay_us(1); if(val & 0x800000) val |= 0xFF000000; return val; }
void HX711_Tare(void) { long s=0; for(int i=0; i<10; i++) { s+=HX711_Read(); Delay_ms(10); } HX711_Offset=s/10; }
float HX711_Get_Weight(void) { if(CALIBRATION_FACTOR==0) return 0; return (float)(HX711_Read()-HX711_Offset)/CALIBRATION_FACTOR; }

// --- DHT11 & DS18B20 ---
void Set_DHT_Out(void) { DHT_PORT->MODER = (DHT_PORT->MODER & ~(3<<(DHT_PIN*2))) | (1<<(DHT_PIN*2)); DHT_PORT->OTYPER &= ~DHT_PIN_MASK; }
void Set_DHT_In(void) { DHT_PORT->MODER &= ~(3<<(DHT_PIN*2)); }
void DHT11_Start(void) { Set_DHT_Out(); DHT_PORT->ODR &= ~DHT_PIN_MASK; Delay_ms(18); DHT_PORT->ODR |= DHT_PIN_MASK; Delay_us(20); Set_DHT_In(); }
uint8_t DHT11_Check(void) { uint32_t c=0; Delay_us(40); if(DHT_PORT->IDR & DHT_PIN_MASK) return 0; Delay_us(80); if(!(DHT_PORT->IDR & DHT_PIN_MASK)) return 0; while(DHT_PORT->IDR & DHT_PIN_MASK) { if(++c>TIMEOUT_VAL) return 0; } return 1; }
uint8_t DHT11_Read(void) { uint8_t i=0; for(int j=0;j<8;j++) { uint32_t c=0; while(!(DHT_PORT->IDR & DHT_PIN_MASK)) { if(++c>TIMEOUT_VAL) return 0; } Delay_us(40); if(!(DHT_PORT->IDR & DHT_PIN_MASK)) i&=~(1<<(7-j)); else i|=(1<<(7-j)); c=0; while(DHT_PORT->IDR & DHT_PIN_MASK) { if(++c>TIMEOUT_VAL) return 0; } } return i; }
void DS_Out(void){ DS18B20_PORT->MODER = (DS18B20_PORT->MODER & ~(3<<(DS18B20_PIN*2))) | (1<<(DS18B20_PIN*2)); }
void DS_In(void){ DS18B20_PORT->MODER &= ~(3<<(DS18B20_PIN*2)); }
uint8_t DS_Reset(void){ uint8_t s; DS_Out(); DS18B20_PORT->ODR &= ~DS18B20_PIN_MASK; Delay_us(480); DS_In(); Delay_us(70); s = !(DS18B20_PORT->IDR & DS18B20_PIN_MASK); Delay_us(410); return s; }
void DS_W_Bit(uint8_t b){ DS_Out(); DS18B20_PORT->ODR &= ~DS18B20_PIN_MASK; Delay_us(b?1:60); DS_In(); Delay_us(b?60:1); Delay_us(5); }
uint8_t DS_R_Bit(void){ uint8_t b=0; DS_Out(); DS18B20_PORT->ODR&=~DS18B20_PIN_MASK; Delay_us(1); DS_In(); Delay_us(14); if(DS18B20_PORT->IDR & DS18B20_PIN_MASK) b=1; Delay_us(45); return b; }
void DS_W_Byte(uint8_t b){ for(int i=0;i<8;i++) { DS_W_Bit(b&1); b>>=1; } }
uint8_t DS_R_Byte(void){ uint8_t b=0; for(int i=0;i<8;i++) { b>>=1; if(DS_R_Bit()) b|=0x80; } return b; }

// FIXED: DS18B20 Get Temp (Added delay to ensure conversion finishes!)
float DS_Get_Temp(void){ 
    if(!DS_Reset()) return -127.0f; 
    DS_W_Byte(0xCC); // Skip ROM
    DS_W_Byte(0x44); // Convert T
    
    // IMPORTANT: DS18B20 takes up to 750ms to convert. 
    Delay_ms(750); 
    
    if(!DS_Reset()) return -127.0f; 
    DS_W_Byte(0xCC); // Skip ROM
    DS_W_Byte(0xBE); // Read Scratchpad
    uint8_t l=DS_R_Byte(), m=DS_R_Byte(); 
    return (float)((m<<8)|l)/16.0f; 
}

void Gather_Data_Buffers(void) {
    uint8_t temp[6];
    for(int i=0; i<MAX30102_BUFFER_LENGTH; i++) {
        uint8_t writePtr, readPtr;
        do {
            MAX30102_ReadRegs(MAX30102_REG_FIFO_WR_PTR, &writePtr, 1);
            MAX30102_ReadRegs(MAX30102_REG_FIFO_RD_PTR, &readPtr, 1);
            Delay_ms(1); 
        } while (writePtr == readPtr);
        MAX30102_ReadRegs(MAX30102_REG_FIFO_DATA, temp, 6);
        aun_red_buffer[i] = ((uint32_t)temp[0] << 16 | (uint32_t)temp[1] << 8 | temp[2]) & 0x03FFFF;
        aun_ir_buffer[i]  = ((uint32_t)temp[3] << 16 | (uint32_t)temp[4] << 8 | temp[5]) & 0x03FFFF;
    }
}

void MAX30102_Calculate(void) {
    uint32_t ir_mean = 0, red_mean = 0;
    for (int i = 0; i < MAX30102_BUFFER_LENGTH; i++) { red_mean += aun_red_buffer[i]; ir_mean += aun_ir_buffer[i]; }
    red_mean /= MAX30102_BUFFER_LENGTH; ir_mean /= MAX30102_BUFFER_LENGTH;
    float red_ac_sq = 0, ir_ac_sq = 0;
    for (int i = 0; i < MAX30102_BUFFER_LENGTH; i++) { float red_ac = (float)aun_red_buffer[i] - red_mean; float ir_ac = (float)aun_ir_buffer[i] - ir_mean; red_ac_sq += red_ac * red_ac; ir_ac_sq += ir_ac * ir_ac; }
    float R = (sqrtf(red_ac_sq / MAX30102_BUFFER_LENGTH) / red_mean) / (sqrtf(ir_ac_sq / MAX30102_BUFFER_LENGTH) / ir_mean);
    current_SpO2 = 104.0f - 17.0f * R;
    if (current_SpO2 > 100) current_SpO2 = 100; if (current_SpO2 < 0) current_SpO2 = 0;
    int peaks = 0, rising = 0;
    for (int i = 1; i < MAX30102_BUFFER_LENGTH; i++) { float val = (float)aun_ir_buffer[i] - ir_mean; if (val > 0 && !rising) { rising = 1; peaks++; } else if (val < 0) { rising = 0; } }
    current_HR = (float)peaks * 60.0f;
    if (ir_mean < 50000) { current_SpO2 = 0; current_HR = 0; }
}

// --- XBee USART2 Configuration ---
void USART2_Init(void) {
    RCC->AHB1ENR |= (1 << 0);  // Enable GPIOA clock
    RCC->APB1ENR |= (1 << 17); // Enable USART2 clock
    GPIOA->MODER &= ~(3 << (2 * 2)) & ~(3 << (3 * 2));
    GPIOA->MODER |= (2 << (2 * 2)) | (2 << (3 * 2));
    GPIOA->AFR[0] &= ~(0xF << (2 * 4)) & ~(0xF << (3 * 4));
    GPIOA->AFR[0] |= (7 << (2 * 4)) | (7 << (3 * 4));
    USART2->BRR = 16000000 / 9600; 
    USART2->CR1 = (1 << 13) | (1 << 3); 
}
void USART2_SendChar(char c) { while (!(USART2->SR & (1 << 7))) {}; USART2->DR = c; }
void USART2_SendString(char* str) { while (*str) { USART2_SendChar(*str++); } }

// --- POWER MANAGEMENT (STANDBY) ---
void Enter_Standby_Mode(void) {
    RCC->APB1ENR |= (1 << 28); 
    PWR->CSR |= (1 << 8); // EWUP bit
    PWR->CR |= (1 << 2); // CWUF bit
    PWR->CR |= (1 << 1); // PDDS bit (Power Down Deepsleep)
    SCB->SCR |= (1 << 2); // SLEEPDEEP bit
    SSD_Clr(); 
    __WFI();   
}

// --- HELPER: Update OLED Screen ---
void Update_OLED_Screen(void) {
    SSD_Clr(); 
    switch(Current_Screen) {
        case 1: // DHT11
            SSD_Header("ROOM SENSOR");
            sprintf(OLED_Buffer, "%.0f C", Temperature_DHT);
            SSD_CenterBig(2, OLED_Buffer); 
            sprintf(OLED_Buffer, "Humidity: %.0f %%", Humidity);
            SSD_Center(5, OLED_Buffer);
            break;
        case 2: // BMP180
            SSD_Header("BAROMETER");
            sprintf(OLED_Buffer, "%.0f", Pressure_BMP);
            SSD_CenterBig(2, OLED_Buffer);
            SSD_Center(4, "hPa");
            sprintf(OLED_Buffer, "Temp: %.1f C", Temperature_BMP);
            SSD_Center(6, OLED_Buffer);
            break;
        case 3: // DS18B20
            SSD_Header("BODY TEMP");
            sprintf(OLED_Buffer, "%.1f F", Temperature_DS_F);
            SSD_CenterBig(2, OLED_Buffer);
            if(Temperature_DS_F > 100.0f) { 
                SSD_Center(5, "!! FEVER !!"); 
            } else {
                SSD_Center(5, "Normal");
            }
            break;
        case 4: // MAX30102
            SSD_Header("HEART & O2");
            sprintf(OLED_Buffer, "HR: %.0f", current_HR);
            SSD_XY(10, 2); SSD_Str(OLED_Buffer); 
            sprintf(OLED_Buffer, "O2: %.0f", current_SpO2);
            SSD_XY(10, 4); SSD_Str(OLED_Buffer); 
            if(current_HR > 120.0f) SSD_Center(6, "HIGH RATE!"); 
            break;
        case 5: // WEIGHT SCALE
            SSD_Header("WEIGH SCALE");
            sprintf(OLED_Buffer, "%.2f", Weight_kg);
            SSD_CenterBig(2, OLED_Buffer); 
            SSD_Center(5, "Kilograms (kg)");
            break;
        default: // Home Menu
            SSD_Header("MAIN MENU");
            SSD_XY(0, 1); SSD_Str("1.Room 2.Pres");
            SSD_XY(0, 2); SSD_Str("3.Body 4.Heart");
            SSD_XY(0, 3); SSD_Str("5.Weight");
            SSD_XY(20, 5); SSD_Str("D: LOCK/SLEEP");
            
            // Debug: Show Current Update Rate
            if(Current_Delay_Interval == 5000) SSD_Center(6, "Rate: 5 sec");
            else SSD_Center(6, "Rate: 10 sec");
            break;
    }
}

// --- MAIN ---
int main(void) {
    // 1. Clock and Basic Init
    RCC->AHB1ENR |= (1 << 0) | (1 << 1) | (1 << 4); // GPIO A, B, E
    TIM2_Config(); 
    I2C1_Config();
    
    // 2. Initialize Interface ONLY (Lock Screen)
    Keypad_Init(); 
    SSD_Init(); 
    SSD_Clr();
    
    // Check if we just woke up from Standby
    RCC->APB1ENR |= (1 << 28); 
    if (PWR->CSR & (1 << 0)) PWR->CR |= (1 << 2);

    // 3. SECURITY LOOP
    uint8_t locked = 1;
    Pass_Index = 0;
    memset(Password_Buffer, 0, 5);
    
    SSD_Header("SECURITY LOCK");
    SSD_Center(3, "ENTER PIN TO WAKE");

    while (locked) {
        char key = Scan_Keypad();
        if(Pass_Index == 0) SSD_Center(5, "- - - -");
        if(Pass_Index == 1) SSD_Center(5, "* - - -");
        if(Pass_Index == 2) SSD_Center(5, "* * - -");
        if(Pass_Index == 3) SSD_Center(5, "* * * -");
        
        if (key != 0) {
             if (key == 'D') {
                 SSD_Clr();
                 SSD_Center(3, "SLEEPING...");
                 Delay_ms(1000);
                 Enter_Standby_Mode();
            }
            if(Pass_Index < 4 && key >= '0' && key <= '9') {
                Password_Buffer[Pass_Index++] = key;
            }
            if(Pass_Index == 4) {
                if(strncmp(Password_Buffer, Correct_Pass, 4) == 0) {
                    locked = 0; 
                    SSD_Clr();
                    SSD_Center(3, "ACCESS GRANTED");
                    Delay_ms(1000);
                } else {
                    SSD_Clr();
                    SSD_Center(3, "WRONG PIN");
                    SSD_Header("SECURITY LOCK");
                    
                    // *FIXED: Beep on Wrong PIN using inverted logic*
                    BUZZER_ON();
                    Delay_ms(1000);
                    BUZZER_OFF();
                    
                    Pass_Index = 0; 
                    memset(Password_Buffer, 0, 5);
                }
            }
        }
        Delay_ms(100);
    }

    // 4. SENSOR INITIALIZATION
    SSD_Clr();
    SSD_Center(3, "Init Sensors...");
    MAX30102_Init(); 
    BMP180_Init(); 
    HX711_Init();
    USART2_Init();
    
    SSD_Clr(); 
    HX711_Tare(); 
    Delay_ms(500);
      
    // 5. ACTIVE LOOP (Adaptive Logic)
    while (1) {
        // --- A. READ SENSORS ---
        Gather_Data_Buffers(); 
        MAX30102_Calculate();
        BMP180_Read_Convert();
        
        DHT11_Start();
        if (DHT11_Check()) {
            Rh_byte1 = DHT11_Read(); Rh_byte2 = DHT11_Read();
            Temp_byte1 = DHT11_Read(); Temp_byte2 = DHT11_Read();
            SUM = DHT11_Read();
            if (SUM == (Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2)) {
                Temperature_DHT = (float)Temp_byte1; Humidity = (float)Rh_byte1;
            }
        }
        
        // This function now has a 750ms delay inside to ensure correct temp
        Temperature_DS = DS_Get_Temp();
        if (Temperature_DS > -100.0f) Temperature_DS_F = (Temperature_DS * 1.8f) + 32.0f;
          
        Weight_g = HX711_Get_Weight(); 
        Weight_kg = Weight_g / 1000.0f;
        if(Weight_kg < 0.0f) Weight_kg = 0.0f;

        // *FIXED: Fever/Alert Check using inverted logic*
        if(Temperature_DS_F > 100.0f || current_HR > 120.0f) {
            BUZZER_ON(); 
        } else {
            BUZZER_OFF();
        }

        // --- B. TRANSMIT DATA ---
        char tx_buffer[128];
        char fever_status[20];
        
        if (Temperature_DS_F > 100.0f) strcpy(fever_status, "Fever");
        else strcpy(fever_status, "No Fever");
        
        sprintf(tx_buffer, "DHT:%.1f,Hum:%.1f,BMP:%.1f,DS:%.1f,Stat:%s,HR:%.0f,SpO2:%.0f,Wt:%.2f\r\n", Temperature_DHT, Humidity, Pressure_BMP, Temperature_DS_F, fever_status, current_HR, current_SpO2, Weight_kg);
        
        USART2_SendString(tx_buffer);

        // --- C. DETERMINE NEXT UPDATE INTERVAL ---
        if (Temperature_DS_F > 100.0f) {
            Current_Delay_Interval = 5000;  // 5 Seconds
        } else {
            Current_Delay_Interval = 10000; // 10 Seconds
        }

        // --- D. RESPONSIVE WAIT LOOP ---
        uint32_t elapsed_time = 0;
        
        // Update Screen immediately once before loop start
        Update_OLED_Screen();

        while(elapsed_time < Current_Delay_Interval) {
            char key = Scan_Keypad();
            
            if (key == 'D') {
                SSD_Clr();
                SSD_Center(3, "LOCKING SYSTEM");
                BUZZER_OFF(); // Ensure buzzer is off before sleep
                Delay_ms(1000);
                Enter_Standby_Mode();
            }
            
            int prev_screen = Current_Screen;
            if (key >= '1' && key <= '5') Current_Screen = key - '0';
            else if (key == '*') Current_Screen = 0;
            
            // If user changes screen, update immediately
            if (Current_Screen != prev_screen) {
                Update_OLED_Screen();
            }

            Delay_ms(100);
            elapsed_time += 100;
        }
    }
}
