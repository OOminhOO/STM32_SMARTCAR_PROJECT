
/* <SMARTCAR_HEADER begin> */
// 스마트카 프로젝트 공용 헤더: 상수/타입/외부변수/프로토타입 모음
#ifndef __SMARTCAR_H__
#define __SMARTCAR_H__

#include "main.h"
#include <stdint.h>

/* <EXTERN_PERIPHERALS begin> */
// CubeMX에서 초기화된 주변장치 핸들(다른 .c에서 정의됨)
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;  // HC-SR04(1MHz 타이머)
extern TIM_HandleTypeDef htim2;  // 부저 PWM
extern TIM_HandleTypeDef htim3;  // 서보 PWM
extern TIM_HandleTypeDef htim4;  // DHT11(1MHz 타이머)
extern UART_HandleTypeDef huart3; // BLE(USART3)
/* <EXTERN_PERIPHERALS end> */

/* <FORWARD_TYPES begin> */
// === 상태머신 및 데이터 구조 ===

// 안전 FSM(장애물 감지 시 경고/후진)
typedef enum {
    SAFE_IDLE = 0, // 대기
    SAFE_BEEP,     // 경고음
    SAFE_BACK      // 후진
} SafetyState;

typedef struct {
    SafetyState st;        // 현재 상태
    uint32_t next_ms;      // 상태 종료 시각(ms)
    uint32_t beep_toggle_ms; // 경고음 토글 시각(ms)
    uint8_t  beep_on;      // 부저 on/off 플래그
} SafetyFSM;

// 서보 공격 FSM(한 번 전진 후 복귀)
typedef enum {
    ATT_IDLE = 0,
    ATT_GO_OUT,   // (현재 미사용)
    ATT_PAUSE,    // 전진 후 잠시 정지
    ATT_GO_BACK   // (현재 미사용)
} AttackState;

typedef struct {
    AttackState st;
    uint16_t    cur_ccr;   // 현재 CCR
    uint16_t    tgt_ccr;   // 목표 CCR
    int16_t     step;      // (미사용)
    uint32_t    next_ms;   // 전환 시각
} AttackFSM;

// 부저 음표/멜로디
typedef struct {
    uint16_t frequency;    // Hz (0=쉼표)
    uint16_t duration;     // ms
} Note;

typedef struct {
    const Note *seq;  // 음표 배열
    int length;       // 배열 길이
    int idx;          // 현재 인덱스
    uint32_t next_ms; // 다음 단계 시각
    uint8_t phase;    // 0=load,1=play,2=gap
    uint8_t playing;  // 1=재생중
} MelodyPlayer;
/* <FORWARD_TYPES end> */

/* <GLOBAL_STATE_EXTERN begin> */
// 전역 상태 변수(정의는 main.c)
extern volatile SafetyFSM    g_safe;
extern volatile MelodyPlayer g_player;
extern volatile AttackFSM    g_att;
/* <GLOBAL_STATE_EXTERN end> */

/* <CONSTS_AND_MACROS begin> */
// === 안전 임계값 ===
#define SAFE_MM_THRESHOLD 150     // 전진 시 안전거리(mm)

// === BLE 수신 링버퍼 크기/초음파 타임아웃(1MHz 기준) ===
#define RX3_BUF_SZ    64
#define ULTRA_MIN_US  100         // 최소 유효 에코(≈1.7cm)
#define ULTRA_MAX_US  30000       // 최대 유효 에코(≈5m)

// === DHT11 핀 매핑(main.h에서 생성된 심볼 사용) ===
#define DHT_PORT DHT11_GPIO_Port
#define DHT_PIN  DHT11_Pin

// === ST7735S LCD 명령어 ===
#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09
#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13
#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E
#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36
#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6
#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5
#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD
#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// === LCD 해상도/색상(RGB565) ===
#define LCD_WIDTH  160
#define LCD_HEIGHT 120

#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0

// === 서보 각도·CCR 범위(TIM3 CH3) ===
#define MAX     125   // 2.5ms
#define MIN     25    // 0.5ms
#define CENTER  75    // 1.5ms
#define STEP    1

// === 부저 음계(Hz) ===
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define REST     0

// === 음표 길이(BPM 기반) ===
#define BPM 150
#define BEAT_MS   (60000 / BPM)
#define WHOLE     (4 * BEAT_MS)
#define HALF      (2 * BEAT_MS)
#define QUARTER   (1 * BEAT_MS)
#define EIGHTH    (BEAT_MS / 2)
#define SIXTEENTH (BEAT_MS / 4)

// === 부저 볼륨(%) ===
#define VOL 50

// === LCD 핀 제어 매크로 ===
#define LCD_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define LCD_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define LCD_DC_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
#define LCD_DC_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define LCD_RES_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define LCD_RES_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)

// === 각도→CCR 변환 ===
#define SERVO_CCR_MIN    MIN
#define SERVO_CCR_MAX    MAX
#define SERVO_CCR_CENTER CENTER
#define SERVO_CCR_FROM_DEG(deg) \
    ( (uint16_t)(SERVO_CCR_MIN + ((uint32_t)(deg) * (SERVO_CCR_MAX - SERVO_CCR_MIN) / 180)) )

// === 공격 동작 파라미터 ===
#define ATTACK_OUT_DEG  110   // 전진 각도
#define ATTACK_PAUSE_MS 200   // 정지 시간(ms)
/* <CONSTS_AND_MACROS end> */

/* <SERVO_INLINE_HELPERS begin> */
// 서보 CCR 직접 설정(안전 범위 클램프)
static inline void servo_set_ccr_ch3(uint16_t ccr) {
    if (ccr < SERVO_CCR_MIN) ccr = SERVO_CCR_MIN;
    if (ccr > SERVO_CCR_MAX) ccr = SERVO_CCR_MAX;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr); // TIM3 CH3에 CCR 반영
}
// 서보 각도로 설정(도→CCR 변환 사용)
static inline void servo_set_deg_ch3(int deg) {
    servo_set_ccr_ch3(SERVO_CCR_FROM_DEG(deg));
}
/* <SERVO_INLINE_HELPERS end> */

/* <PROTOTYPES begin> */
// === 모터 ===
void smartcar_forward(void);
void smartcar_backward(void);
void smartcar_left(void);
void smartcar_right(void);
void smartcar_stop(void);

// === 부저/멜로디 ===
void buzzer_set_freq_ch2(uint16_t freq);
void melody_stop(void);
void melody_start(const Note *seq, int len);
void play_tone(uint16_t frequency, uint16_t duration);
void play_mario_theme(void);

// === LCD ===
void LCD_WriteCommand(uint8_t cmd);
void LCD_WriteData(uint8_t data);
void LCD_WriteData16(uint16_t data);
void LCD_Init(void);
void LCD_SetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void LCD_DrawPixel(uint8_t x, uint8_t y, uint16_t color);
void LCD_Fill(uint16_t color);
void LCD_DrawChar(uint8_t x, uint8_t y, char ch, uint16_t color, uint16_t bg_color);
void LCD_DrawString(uint8_t x, uint8_t y, const char *str, uint16_t color, uint16_t bg_color);
/* <PROTOTYPES end> */

#endif /* __SMARTCAR_H__ */
/* <SMARTCAR_HEADER end> */

