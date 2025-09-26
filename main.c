/* <SMARTCAR_HEADER begin> */
// 스마트카 프로젝트 공용 헤더 파일: 상수, 데이터 타입, 전역 변수, 함수 선언 등이 포함됨
#ifndef __SMARTCAR_H__
#define __SMARTCAR_H__

#include "main.h"
#include <stdint.h>

/* <EXTERN_PERIPHERALS begin> */
// 외부에서 초기화된 주변 장치 핸들러들을 참조로 선언 (SPI, 타이머, UART 등)
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart3;
/* <EXTERN_PERIPHERALS end> */

/* <FORWARD_TYPES begin> */
// === FSM 및 데이터 구조 정의 ===

// 안전 FSM (안전모드 상태 표시용 상태 머신)
typedef enum {
    SAFE_IDLE = 0, // 대기 상태 (안전 FSM 동작 없음)
    SAFE_BEEP,     // 경고음 발생 상태
    SAFE_BACK      // 후진 상태 (장애물 회피 이동)
} SafetyState;

typedef struct {
    SafetyState st;       // 현재 안전 FSM 상태
    uint32_t next_ms;     // 현재 상태 종료 예정 시각 (ms 단위)
    uint32_t beep_toggle_ms; // 경고음 토글(next on/off) 시각 (ms)
    uint8_t  beep_on;     // 경고음 출력 여부 플래그 (1=음 출력 중, 0=출력 안 함)
} SafetyFSM;

// 서보 공격 FSM (한 번의 서보 공격 동작 상태 머신)
typedef enum {
    ATT_IDLE = 0,  // 대기 상태 (공격 없음)
    ATT_GO_OUT,    // (미사용) 서보 전진 동작 상태
    ATT_PAUSE,     // 일시 정지 상태 (최대 전진 후 잠시 멈춤)
    ATT_GO_BACK    // (미사용) 서보 복귀 동작 상태
} AttackState;

typedef struct {
    AttackState st;    // 현재 공격 FSM 상태
    uint16_t    cur_ccr; // 현재 서보 위치의 CCR 값
    uint16_t    tgt_ccr; // 목표 서보 위치의 CCR 값
    int16_t     step;    // (미사용) 서보 이동 단계별 증분 값
    uint32_t    next_ms; // 다음 상태로 전환될 시각 (ms)
} AttackFSM;

// 멜로디/음표 정의 (부저로 연주할 음에 대한 구조체)
typedef struct {
    uint16_t frequency; // 음의 주파수 (Hz), REST(0)이면 쉼표
    uint16_t duration;  // 음의 연주 지속 시간 (밀리초)
} Note;

typedef struct {
    const Note *seq;   // 연주할 음표 배열의 포인터
    int length;        // 음표 배열 길이
    int idx;           // 현재 재생 중인 음표 인덱스
    uint32_t next_ms;  // 다음 단계 전환 시각 (ms)
    uint8_t phase;     // 재생 단계 (0=음 시작, 1=음 재생 중, 2=음과 음 사이 간격)
    uint8_t playing;   // 재생 진행 여부 (1이면 재생 중, 0이면 중지됨)
} MelodyPlayer;
/* <FORWARD_TYPES end> */

/* <GLOBAL_STATE_EXTERN begin> */
// 전역 FSM 상태 변수 선언 (실제 정의는 main.c)
extern volatile SafetyFSM  g_safe;
extern volatile MelodyPlayer g_player;
extern volatile AttackFSM g_att;
/* <GLOBAL_STATE_EXTERN end> */

/* <CONSTS_AND_MACROS begin> */
// === 상수 및 매크로 정의 ===

#define SAFE_MM_THRESHOLD 150   // 안전모드 장애물 거리 임계값 (150mm)

// ===== HC-SR04 초음파 센서 관련 상수 (TIM1 타이머 1MHz 기준) =====
#define RX3_BUF_SZ    64       // UART3 수신 링버퍼 크기 (BLE 명령 수신용)
#define ULTRA_MIN_US  100      // 에코 펄스 최소 시간 (마이크로초, 약 1.7cm 거리)
#define ULTRA_MAX_US  30000    // 에코 펄스 최대 시간 (30ms, 약 5m 거리)

// ==== DHT11 센서 드라이버 관련 정의 (PA15 핀, TIM4 1MHz 사용) ====
#define DHT_PORT DHT11_GPIO_Port
#define DHT_PIN  DHT11_Pin

// ===== ST7735S LCD 명령어 코드 =====
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

// LCD 해상도 (픽셀 단위)
#define LCD_WIDTH  160
#define LCD_HEIGHT 120

// 색상 정의 (16비트 RGB565)
#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0

// 서보 모터 각도 위치 정의 (CCR 값 범위)
#define MAX     125   // 최대 (2.5ms 펄스 폭)
#define MIN     25    // 최소 (0.5ms 펄스 폭)
#define CENTER  75    // 중간 (1.5ms 펄스 폭)
#define STEP    1

// 부저 음계 주파수 정의 (단위: Hz)
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
#define REST     0     // 휴지(쉼표): 주파수 0

// 음표 길이 정의 (ms 단위, BPM 기반)
#define BPM 150
#define BEAT_MS   (60000 / BPM)     // 한 박자 길이(ms)
#define WHOLE     (4 * BEAT_MS)     // 온음표
#define HALF      (2 * BEAT_MS)     // 2분음표
#define QUARTER   (1 * BEAT_MS)     // 4분음표
#define EIGHTH    (BEAT_MS / 2)     // 8분음표
#define SIXTEENTH (BEAT_MS / 4)     // 16분음표

// 부저 출력 볼륨 (%)
#define VOL 50

// LCD 제어 핀 매크로 (CS, DC, RES 신호 제어)
#define LCD_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define LCD_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define LCD_DC_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
#define LCD_DC_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define LCD_RES_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define LCD_RES_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)

// 서보모터 CCR 계산 매크로
#define SERVO_CCR_MIN    MIN
#define SERVO_CCR_MAX    MAX
#define SERVO_CCR_CENTER CENTER
#define SERVO_CCR_FROM_DEG(deg) ( (uint16_t)(SERVO_CCR_MIN + ((uint32_t)(deg) * (SERVO_CCR_MAX - SERVO_CCR_MIN) / 180)) )

// 공격 동작 파라미터 (서보모터 움직임 설정)
#define ATTACK_OUT_DEG  110   // 서보 전진 각도 (110도)
#define ATTACK_PAUSE_MS 200   // 전진 후 대기 시간 (ms)
/* <CONSTS_AND_MACROS end> */

/* <SERVO_INLINE_HELPERS begin> */
// --- 서보모터 제어 보조 함수 ---
static inline void servo_set_ccr_ch3(uint16_t ccr) {
    if (ccr < SERVO_CCR_MIN) ccr = SERVO_CCR_MIN; // CCR 값이 최소치보다 작으면 최소값으로
    if (ccr > SERVO_CCR_MAX) ccr = SERVO_CCR_MAX; // CCR 값이 최대치를 넘으면 최대값으로
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr); // TIM3 채널3의 CCR 레지스터에 값 설정 (서보 위치 제어)
}
static inline void servo_set_deg_ch3(int deg) {
    servo_set_ccr_ch3(SERVO_CCR_FROM_DEG(deg));   // 각도를 CCR 값으로 변환하여 서보 위치 설정
}
/* <SERVO_INLINE_HELPERS end> */

/* <PROTOTYPES begin> */
// --- 주요 기능 함수 프로토타입 선언 ---

// 모터 제어
void smartcar_forward(void);
void smartcar_backward(void);
void smartcar_left(void);
void smartcar_right(void);
void smartcar_stop(void);

// 부저/멜로디 제어
void buzzer_set_freq_ch2(uint16_t freq);
void melody_stop(void);
void melody_start(const Note *seq, int len);
void play_tone(uint16_t frequency, uint16_t duration);
void play_mario_theme(void);

// LCD 제어
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

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body (구조화 + 주석 마커 추가)
 ******************************************************************************
 * @attention
 * (C) Copyright ...
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* <INCLUDES begin> */
// 필요한 헤더 파일 포함 (표준 I/O, 표준라이브러리, smartcar.h 등)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "smartcar.h"
/* <INCLUDES end> */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* <LOCAL_TYPES begin> */
/* (STM32CubeMX 기본 생성 부분: 변경 없음) */
/* <LOCAL_TYPES end> */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* <MOVED_TO_HEADER begin> */
// 대부분의 상수/매크로는 smartcar.h로 이동 (이전 코드에서는 삭제 대신 주석 처리)
// #define SAFE_MM_THRESHOLD 150
// ...
/* <MOVED_TO_HEADER end> */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* <MACROS_LOCAL begin> */
// 볼륨, LCD 제어용 매크로는 smartcar.h에 정의됨
/* <MACROS_LOCAL end> */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* <GLOBAL_STATE begin> */
// 전역 상태 변수 정의 및 초기화
static uint8_t safe_trip_cnt = 0;     // 안전모드 트립 카운트 (현재 사용되지 않음; 삭제하지 않고 유지)
static uint8_t ultra_side = 0;        // 초음파 센서 측정 방향 플래그 (0: 왼쪽 센서, 1: 오른쪽 센서)

static uint32_t next_ultra_ms = 0;    // 다음 초음파 측정 예정 시각 (ms)
int echo_time_right = 0; // (미사용) 우측 초음파 Echo 신호 시간 저장용
int echo_time_left  = 0; // (미사용) 좌측 초음파 Echo 신호 시간 저장용
int dist_right = 0;      // 우측 장애물까지의 거리 (mm 단위)
int dist_left  = 0;      // 좌측 장애물까지의 거리 (mm 단위)
int HIGH = 1; // (미사용) 논리 HIGH 정의
int LOW  = 0; // (미사용) 논리 LOW 정의

static uint32_t next_dht_ms = 0;     // DHT11 센서 다음 읽기 시각 (ms)
static uint8_t  last_h = 0, last_t = 0; // 마지막으로 읽은 습도, 온도 값

volatile SafetyFSM    g_safe = { SAFE_IDLE, 0, 0, 0 };                  // 안전 FSM 상태 변수 (초기: SAFE_IDLE)
volatile MelodyPlayer g_player = { 0 };                                 // 멜로디 재생기 상태 변수 (초기값 0으로 초기화)
volatile AttackFSM    g_att = { ATT_IDLE, SERVO_CCR_MIN, SERVO_CCR_MIN, 0, 0 }; // 공격 FSM 상태 변수 (초기: 대기, 서보 CCR 최소로 설정)

static uint8_t is_driving_forward = 0; // 현재 차량 전진 상태 플래그 (전진 중=1, 그 외=0, 안전모드 판단에 사용)
/* <GLOBAL_STATE end> */

/* <FONT8x8 begin> */
// 8x8 픽셀로 이루어진 ASCII 문자 폰트 데이터 (ASCII 32~127 문자 대응)
static const uint8_t font8x8[][8] = {
  { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 }, // ' '
  { 0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00 }, // '!'
  { 0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00 }, // '"'
  { 0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00 }, // '#'
  { 0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00 }, // '$'
  { 0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00 }, // '%'
  { 0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00 }, // '&'
  { 0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00 }, // '''
  { 0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00 }, // '('
  { 0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00 }, // ')'
  { 0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00 }, // '*'
  { 0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00 }, // '+'
  { 0x00,0x00,0x00,0x00,0x00,0x0C,0x06,0x00 }, // ','
  { 0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00 }, // '-'
  { 0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00 }, // '.'
  { 0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00 }, // '/'
  { 0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00 }, // '0'
  { 0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00 }, // '1'
  { 0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00 }, // '2'
  { 0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00 }, // '3'
  { 0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00 }, // '4'
  { 0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00 }, // '5'
  { 0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00 }, // '6'
  { 0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00 }, // '7'
  { 0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00 }, // '8'
  { 0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00 }, // '9'
  { 0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00 }, // ':'
  { 0x00,0x0C,0x0C,0x00,0x00,0x0C,0x06,0x00 }, // ';'
  { 0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00 }, // '<'
  { 0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00 }, // '='
  { 0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00 }, // '>'
  { 0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00 }, // '?'
  { 0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00 }, // '@'
  { 0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00 }, // 'A'
  { 0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00 }, // 'B'
  { 0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00 }, // 'C'
  { 0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00 }, // 'D'
  { 0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00 }, // 'E'
  { 0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00 }, // 'F'
  { 0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00 }, // 'G'
  { 0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00 }, // 'H'
  { 0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00 }, // 'I'
  { 0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00 }, // 'J'
  { 0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00 }, // 'K'
  { 0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00 }, // 'L'
  { 0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00 }, // 'M'
  { 0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00 }, // 'N'
  { 0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00 }, // 'O'
  { 0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00 }, // 'P'
  { 0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00 }, // 'Q'
  { 0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00 }, // 'R'
  { 0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00 }, // 'S'
  { 0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00 }, // 'T'
  { 0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00 }, // 'U'
  { 0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00 }, // 'V'
  { 0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00 }, // 'W'
  { 0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00 }, // 'X'
  { 0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00 }, // 'Y'
  { 0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00 }, // 'Z'
  { 0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00 }, // '['
  { 0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00 }, // '\'
  { 0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00 }, // ']'
  { 0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00 }, // '^'
  { 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF }, // '_'
  { 0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00 }, // '`'
  { 0x00,0x00,0x1E,0x30,0x3E,0x33,0x6E,0x00 }, // 'a'
  { 0x07,0x06,0x06,0x3E,0x66,0x66,0x3B,0x00 }, // 'b'
  { 0x00,0x00,0x1E,0x33,0x03,0x33,0x1E,0x00 }, // 'c'
  { 0x38,0x30,0x30,0x3E,0x33,0x33,0x6E,0x00 }, // 'd'
  { 0x00,0x00,0x1E,0x33,0x3F,0x03,0x1E,0x00 }, // 'e'
  { 0x1C,0x36,0x06,0x0F,0x06,0x06,0x0F,0x00 }, // 'f'
  { 0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x1F }, // 'g'
  { 0x07,0x06,0x36,0x6E,0x66,0x66,0x67,0x00 }, // 'h'
  { 0x0C,0x00,0x0E,0x0C,0x0C,0x0C,0x1E,0x00 }, // 'i'
  { 0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1E }, // 'j'
  { 0x07,0x06,0x66,0x36,0x1E,0x36,0x67,0x00 }, // 'k'
  { 0x0E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00 }, // 'l'
  { 0x00,0x00,0x33,0x7F,0x7F,0x6B,0x63,0x00 }, // 'm'
  { 0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x00 }, // 'n'
  { 0x00,0x00,0x1E,0x33,0x33,0x33,0x1E,0x00 }, // 'o'
  { 0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x0F }, // 'p'
  { 0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x78 }, // 'q'
  { 0x00,0x00,0x3B,0x6E,0x66,0x06,0x0F,0x00 }, // 'r'
  { 0x00,0x00,0x3E,0x03,0x1E,0x30,0x1F,0x00 }, // 's'
  { 0x08,0x0C,0x3E,0x0C,0x0C,0x2C,0x18,0x00 }, // 't'
  { 0x00,0x00,0x33,0x33,0x33,0x33,0x6E,0x00 }, // 'u'
  { 0x00,0x00,0x33,0x33,0x33,0x1E,0x0C,0x00 }, // 'v'
  { 0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00 }, // 'w'
  { 0x00,0x00,0x63,0x36,0x1C,0x36,0x63,0x00 }, // 'x'
  { 0x00,0x00,0x33,0x33,0x33,0x3E,0x30,0x1F }, // 'y'
  { 0x00,0x00,0x3F,0x19,0x0C,0x26,0x3F,0x00 }, // 'z'
  { 0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00 }, // '{'
  { 0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00 }, // '|'
  { 0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00 }, // '}'
  { 0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00 }  // '~'
};
/* <FONT8x8 end> */

int ch; // printf 리다이렉트용 임시 변수 (UART 출력 시 사용)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
/* <FORWARD_DECLS_LOCAL begin> */
// 본 파일 내에서만 사용하는 정적/인라인 함수들의 선언
static inline void uart3_start_rx_it(void);
static inline void melody_tick(void);
static inline void attack_tick(void);
static inline void safety_tick(void);
static inline uint8_t safety_active(void);
static void LCD_PrintTH(uint8_t x, uint8_t y, int ok, uint8_t h, uint8_t t);
static int DHT11_Read(uint8_t *hum, uint8_t *temp);
static inline uint16_t t4_cnt(void);
static inline void t4_zero(void);
static int wait_while(GPIO_PinState level, uint32_t timeout_us);
void timer_start(void);
static inline uint16_t t1_cnt(void);
static inline void t1_zero(void);
static int wait_level_u(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level, uint16_t timeout_us);
static uint16_t read_echo_us(GPIO_TypeDef *port, uint16_t pin);
void delay_us(uint16_t us);
void trig_right(void);
void trig_left(void);
/* <FORWARD_DECLS_LOCAL end> */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* <SAFETY_FSM begin> */
// **안전 FSM 동작 함수들** (장애물 감지 시 차량을 정지 및 후진시키는 안전 모드)

// 안전모드 활성 여부 체크
static inline uint8_t safety_active(void) {
    return (g_safe.st != SAFE_IDLE); // SAFE_IDLE이 아니면 안전모드 진행 중
}

// 경고음 켜기
static inline void safety_beep_on(void)  { buzzer_set_freq_ch2(1000); } // 1kHz 경고음 발생
// 경고음 끄기
static inline void safety_beep_off(void) { HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); }

// 안전모드 시작 처리 함수
static inline void safety_start(void) {
    smartcar_stop();       // 차량 정지
    melody_stop();         // 진행 중인 멜로디 중단 (타이머/채널 공유 충돌 방지)
    LCD_DrawString(10, 75, "SAFETY MODE!", YELLOW, BLACK); // LCD에 안전모드 표시
    g_safe.st = SAFE_BEEP; // 안전 FSM 상태를 경고음 단계로 전환
    uint32_t now = HAL_GetTick();
    g_safe.next_ms = now + 2000;     // 2초 동안 경고음 울릴 예정
    g_safe.beep_toggle_ms = now;     // 경고음 토글 시작 시각 초기화
    g_safe.beep_on = 0;              // 경고음 처음에는 꺼진 상태
}

// 안전 FSM 주기적 업데이트 (주된 안전모드 동작 처리)
static inline void safety_tick(void) {
    if (g_safe.st == SAFE_IDLE) return; // 안전 FSM 비활성 상태면 아무 동작 안 함
    uint32_t now = HAL_GetTick();

    switch (g_safe.st) {
    case SAFE_BEEP:
        // 경고음 단계: 200ms마다 부저 on/off 토글
        if ((int32_t)(now - g_safe.beep_toggle_ms) >= 0) {
            g_safe.beep_toggle_ms = now + 200;
            g_safe.beep_on ^= 1;                  // 비트 반전으로 토글
            if (g_safe.beep_on) safety_beep_on(); // beep_on=1 이면 부저 켬
            else safety_beep_off();              // beep_on=0 이면 부저 끔
        }
        // 경고음 기간(2초)이 끝나면 후진 단계로 전환
        if ((int32_t)(now - g_safe.next_ms) >= 0) {
            safety_beep_off();            // 부저 끄기
            smartcar_backward();          // 차량 후진 시작
            g_safe.st = SAFE_BACK;        // 상태를 후진 단계로 변경
            g_safe.next_ms = now + 1000;  // 1초간 후진 예정
        }
        break;
    case SAFE_BACK:
        // 후진 단계: 설정된 후진 시간 경과하면 정지하고 안전모드 종료
        if ((int32_t)(now - g_safe.next_ms) >= 0) {
            smartcar_stop();      // 차량 정지
            g_safe.st = SAFE_IDLE; // 상태를 대기로 복귀 (안전모드 종료)
            is_driving_forward = 0; // 전진 상태 플래그도 해제
        }
        break;
    default:
        g_safe.st = SAFE_IDLE;
        break;
    }
}
/* <SAFETY_FSM end> */

/* <DHT11_DRIVER begin> */
// **DHT11 온습도 센서 드라이버** (습도, 온도 값 읽기 함수들)

static void DHT_to_output_low(void) {
    // DHT11 핀을 출력(Push-Pull) 저항값 없이 설정하고 Low로 내려 시작 신호를 준비
    GPIO_InitTypeDef g = {0};
    HAL_GPIO_DeInit(DHT_PORT, DHT_PIN);
    g.Pin = DHT_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT_PORT, &g);
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);
}
static void DHT_to_input_pullup(void) {
    // DHT11 핀을 입력(Pull-up)으로 설정하여 센서의 신호를 읽을 준비
    GPIO_InitTypeDef g = {0};
    HAL_GPIO_DeInit(DHT_PORT, DHT_PIN);
    g.Pin = DHT_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT_PORT, &g);
}
static inline uint16_t t4_cnt(void)  {
    return __HAL_TIM_GET_COUNTER(&htim4);  // TIM4 카운터 값 읽기
}
static inline void     t4_zero(void) {
    __HAL_TIM_SET_COUNTER(&htim4, 0);      // TIM4 카운터 0으로 리셋
}

// 특정 상태(level) 유지 시간을 대기 (timeout_us 마이크로초 넘으면 실패)
static int wait_while(GPIO_PinState level, uint32_t timeout_us) {
    t4_zero();
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == level) {
        if (t4_cnt() > timeout_us) return 0; // 설정 시간 초과 시 실패 반환
    }
    return 1; // 상태 변화 감지 성공
}

// DHT11 센서로부터 습도 및 온도 값 읽어오기
static int DHT11_Read(uint8_t *hum, uint8_t *temp) {
    uint8_t d[5] = {0};
    DHT_to_output_low(); HAL_Delay(18);  // DHT11 시작 신호: 핀을 18ms 동안 Low 유지
    DHT_to_input_pullup();              // 입력으로 전환하여 DHT11 응답 대기
    if (!wait_while(GPIO_PIN_SET,   200)) return 0; // DHT 응답 신호 (Low 가 될 때까지 대기)
    if (!wait_while(GPIO_PIN_RESET, 200)) return 0; // DHT 응답 신호 Low 끝 (High 될 때까지 대기)
    if (!wait_while(GPIO_PIN_SET,   200)) return 0; // DHT 준비 신호 High 끝 (Low 될 때까지 대기)

    // 40bit (5바이트) 데이터 수신 루프
    for (int i = 0; i < 40; i++) {
        if (!wait_while(GPIO_PIN_SET,   200)) return 0; // Low 유지 기간 대기 (시작비트)
        t4_zero();
        if (!wait_while(GPIO_PIN_RESET, 200)) return 0; // High 전환까지 시간 측정 시작
        (void)t4_cnt(); // (High 전환 시점 캡처, 사용하지 않음)
        t4_zero();
        if (!wait_while(GPIO_PIN_SET,   200)) return 0; // High 유지 시간 측정 시작
        uint16_t high_us = t4_cnt();     // High 유지된 시간 (마이크로초)
        d[i / 8] <<= 1;
        d[i / 8] |= (high_us > 40) ? 1 : 0; // 26~28us -> 0, 70us -> 1 판정 (High 시간 기준)
    }
    uint8_t sum = (uint8_t)(d[0] + d[1] + d[2] + d[3]);
    if (sum != d[4]) return 0; // 체크섬 검증 실패 시 오류
    *hum = d[0]; *temp = d[2]; // 습도(정수부), 온도(정수부) 값 반환
    return 1; // 성공
}

// LCD에 온도/습도 출력 (혹은 오류 출력)
static void LCD_PrintTH(uint8_t x, uint8_t y, int ok, uint8_t h, uint8_t t) {
    char buf[24];
    if (ok) {
        // 기존 영역 지우기 (공백 출력)
        LCD_DrawString(x, y,     "                             ", BLACK, BLACK);
        LCD_DrawString(x, y + 15,"                             ", BLACK, BLACK);
        // 온도와 습도 값을 화면에 출력
        snprintf(buf, sizeof(buf), "Temp: %2d C", t);
        LCD_DrawString(x, y, buf, WHITE, BLACK);
        snprintf(buf, sizeof(buf), "Hum : %2d %%", h);
        LCD_DrawString(x, y + 15, buf, WHITE, BLACK);
    } else {
        // 오류 발생 시 메시지 출력
        LCD_DrawString(x, y,     "DHT11 ERR                ", RED, BLACK);
        LCD_DrawString(x, y + 15,"                         ", BLACK, BLACK);
    }
}
/* <DHT11_DRIVER end> */

/* <LCD_DRIVER begin> */
// **LCD 제어 함수 구현부** (ST7735S LCD 초기화 및 화면 제어)
void LCD_WriteCommand(uint8_t cmd) {
    LCD_CS_LOW();  // CS Low: 명령 전송 시작
    LCD_DC_LOW();  // DC Low: 명령 모드
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY); // SPI로 1바이트 명령 전송
    LCD_CS_HIGH(); // CS High: 전송 종료
}
void LCD_WriteData(uint8_t data) {
    LCD_CS_LOW();  LCD_DC_HIGH(); // 데이터 전송 모드 설정
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY); // SPI로 1바이트 데이터 전송
    LCD_CS_HIGH();
}
void LCD_WriteData16(uint16_t data) {
    uint8_t buffer[2] = { (data >> 8) & 0xFF, data & 0xFF };
    LCD_CS_LOW(); LCD_DC_HIGH(); // 16비트 데이터 전송
    HAL_SPI_Transmit(&hspi1, buffer, 2, HAL_MAX_DELAY); // 2바이트(16비트) 전송
    LCD_CS_HIGH();
}
void LCD_Init(void) {
    // LCD 초기화 시퀀스 (데이터시트 규격에 따른 명령 전송)
    LCD_RES_LOW();  HAL_Delay(100);
    LCD_RES_HIGH(); HAL_Delay(100);
    LCD_WriteCommand(ST7735_SWRESET); HAL_Delay(150);
    LCD_WriteCommand(ST7735_SLPOUT);  HAL_Delay(500);

    LCD_WriteCommand(ST7735_FRMCTR1);
    LCD_WriteData(0x01); LCD_WriteData(0x2C); LCD_WriteData(0x2D);
    LCD_WriteCommand(ST7735_FRMCTR2);
    LCD_WriteData(0x01); LCD_WriteData(0x2C); LCD_WriteData(0x2D);
    LCD_WriteCommand(ST7735_FRMCTR3);
    LCD_WriteData(0x01); LCD_WriteData(0x2C); LCD_WriteData(0x2D);
    LCD_WriteData(0x01); LCD_WriteData(0x2C); LCD_WriteData(0x2D);
    LCD_WriteCommand(ST7735_INVCTR);  LCD_WriteData(0x07);
    LCD_WriteCommand(ST7735_PWCTR1);  LCD_WriteData(0xA2); LCD_WriteData(0x02); LCD_WriteData(0x84);
    LCD_WriteCommand(ST7735_PWCTR2);  LCD_WriteData(0xC5);
    LCD_WriteCommand(ST7735_PWCTR3);  LCD_WriteData(0x0A); LCD_WriteData(0x00);
    LCD_WriteCommand(ST7735_PWCTR4);  LCD_WriteData(0x8A); LCD_WriteData(0x2A);
    LCD_WriteCommand(ST7735_PWCTR5);  LCD_WriteData(0x8A); LCD_WriteData(0xEE);
    LCD_WriteCommand(ST7735_VMCTR1);  LCD_WriteData(0x0E);
    LCD_WriteCommand(ST7735_INVOFF);

    LCD_WriteCommand(ST7735_MADCTL);  LCD_WriteData(0x60); // 메모리 데이터 접근제어 (화면 회전 설정 등)
    LCD_WriteCommand(ST7735_COLMOD);  LCD_WriteData(0x05); // 컬러모드: 16비트 (RGB565)

    // 화면 출력 영역 설정 (전체 영역)
    LCD_WriteCommand(ST7735_CASET);
    LCD_WriteData(0x00); LCD_WriteData(0x00);
    LCD_WriteData(0x00); LCD_WriteData(0x9F);
    LCD_WriteCommand(ST7735_RASET);
    LCD_WriteData(0x00); LCD_WriteData(0x00);
    LCD_WriteData(0x00); LCD_WriteData(0x77);

    // 감마 설정
    LCD_WriteCommand(ST7735_GMCTRP1);
    LCD_WriteData(0x0f); LCD_WriteData(0x1a); LCD_WriteData(0x0f); LCD_WriteData(0x18);
    LCD_WriteData(0x2f); LCD_WriteData(0x28); LCD_WriteData(0x20); LCD_WriteData(0x22);
    LCD_WriteData(0x1f); LCD_WriteData(0x1b); LCD_WriteData(0x23); LCD_WriteData(0x37);
    LCD_WriteData(0x00); LCD_WriteData(0x07); LCD_WriteData(0x02); LCD_WriteData(0x10);

    LCD_WriteCommand(ST7735_GMCTRN1);
    LCD_WriteData(0x0f); LCD_WriteData(0x1b); LCD_WriteData(0x0f); LCD_WriteData(0x17);
    LCD_WriteData(0x33); LCD_WriteData(0x2c); LCD_WriteData(0x29); LCD_WriteData(0x2e);
    LCD_WriteData(0x30); LCD_WriteData(0x30); LCD_WriteData(0x39); LCD_WriteData(0x3f);
    LCD_WriteData(0x00); LCD_WriteData(0x07); LCD_WriteData(0x03); LCD_WriteData(0x10);

    LCD_WriteCommand(ST7735_NORON); HAL_Delay(10);
    LCD_WriteCommand(ST7735_DISPON); HAL_Delay(100);
}
void LCD_SetWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    // 지정된 좌표 영역만 그리도록 설정
    uint8_t x_offset = 0, y_offset = 0;
    LCD_WriteCommand(ST7735_CASET);
    LCD_WriteData(0x00); LCD_WriteData(x0 + x_offset);
    LCD_WriteData(0x00); LCD_WriteData(x1 + x_offset);
    LCD_WriteCommand(ST7735_RASET);
    LCD_WriteData(0x00); LCD_WriteData(y0 + y_offset);
    LCD_WriteData(0x00); LCD_WriteData(y1 + y_offset);
    LCD_WriteCommand(ST7735_RAMWR);
}
void LCD_DrawPixel(uint8_t x, uint8_t y, uint16_t color) {
    // 한 픽셀 점 찍기 (좌표 범위 확인 후 진행)
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    LCD_SetWindow(x, y, x, y);
    LCD_WriteData16(color);
}
void LCD_Fill(uint16_t color) {
    // 전체 화면을 지정한 색으로 채우기
    LCD_SetWindow(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    LCD_CS_LOW(); LCD_DC_HIGH();
    for (uint16_t i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        uint8_t buffer[2] = { (color >> 8) & 0xFF, color & 0xFF };
        HAL_SPI_Transmit(&hspi1, buffer, 2, HAL_MAX_DELAY);
    }
    LCD_CS_HIGH();
}
void LCD_DrawChar(uint8_t x, uint8_t y, char ch, uint16_t color, uint16_t bg_color) {
    // 한 문자 출력 (8x8 픽셀 폰트 사용)
    if (ch < 32 || ch > 126) ch = 32;           // 지원하지 않는 문자는 공백 처리
    const uint8_t *font_char = font8x8[ch - 32];
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t line = font_char[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (0x01 << j))
                LCD_DrawPixel(x + j, y + i, color);   // 비트가 1이면 폰트 색상 픽셀 출력
            else
                LCD_DrawPixel(x + j, y + i, bg_color); // 0이면 배경색 픽셀 출력
        }
    }
}
void LCD_DrawString(uint8_t x, uint8_t y, const char *str, uint16_t color, uint16_t bg_color) {
    // 문자열 출력 (개행 처리 등 포함)
    uint8_t orig_x = x;
    while (*str) {
        if (*str == '\n') {
            y += 8; x = orig_x;               // 개행 문자: y좌표 한 글자 밑으로 내림, x를 원래 위치로
        }
        else if (*str == '\r') {
            x = orig_x;                       // 복귀(carriage return): x를 처음 위치로
        }
        else {
            if (x + 8 > LCD_WIDTH) {          // 화면 폭을 넘길 경우 줄 바꿈
                x = orig_x;
                y += 8;
            }
            if (y + 8 > LCD_HEIGHT) break;    // 화면 높이를 넘으면 출력 종료
            LCD_DrawChar(x, y, *str, color, bg_color);
            x += 8;                           // 다음 문자 위치로 x 이동
        }
        str++;
    }
}
/* <LCD_DRIVER end> */

/* <BLE_RINGBUFFER begin> */
// **BLE(UART3) 링버퍼 및 인터럽트 처리** (블루투스 시리얼 통신)
volatile uint8_t rx3_byte;
volatile uint8_t rx3_buf[RX3_BUF_SZ];
volatile uint8_t rx3_head = 0, rx3_tail = 0;

static inline void uart3_start_rx_it(void) {
    // UART3 수신 인터럽트 시작 설정 (1바이트씩 수신)
    HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx3_byte, 1);
}
static inline void rb_put(uint8_t b) {
    // 수신 버퍼에 바이트 저장 (헤드 이동, 가득 찬 경우 데이터 버림)
    uint8_t next = (rx3_head + 1) % RX3_BUF_SZ;
    if (next != rx3_tail) {
        rx3_buf[rx3_head] = b;
        rx3_head = next;
    }
}
static inline int rb_get(uint8_t *out) {
    // 수신 버퍼에서 바이트 하나 읽어오기 (없으면 0 반환)
    if (rx3_head == rx3_tail) return 0;
    *out = rx3_buf[rx3_tail];
    rx3_tail = (rx3_tail + 1) % RX3_BUF_SZ;
    return 1;
}
static inline void ble_send_line(const char *s) {
    // BLE로 문자열 한 줄 전송 (문자열 끝에 CRLF 추가)
    HAL_UART_Transmit(&huart3, (uint8_t*)s, (uint16_t)strlen(s), 20);
    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 20);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // UART3 수신 인터럽트 완료 콜백 -> 수신 데이터 처리
    if (huart->Instance == USART3) {
        rb_put(rx3_byte);  // 수신한 바이트를 링버퍼에 저장
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx3_byte, 1); // 다음 바이트 수신 대기 재개
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // UART3 에러 발생 시 콜백 -> 에러 플래그 초기화 후 다시 수신 대기
    if (huart->Instance == USART3) {
        volatile uint32_t _sr = huart->Instance->SR;
        volatile uint32_t _dr = huart->Instance->DR;
        (void)_sr; (void)_dr;
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx3_byte, 1);
    }
}
/* <BLE_RINGBUFFER end> */

/* <MELODY_PLAYER begin> */
// **멜로디 재생 기능** (TIM2 PWM으로 부저 음을 내어 멜로디 연주)
void buzzer_set_freq_ch2(uint16_t freq) {
    // TIM2 채널4 PWM으로 주어진 주파수 음 내기 (freq=0이면 PWM 정지)
    if (freq == 0) {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
        return;
    }
    // 주파수에 맞춰 ARR(자동 리로드) 값 계산 (TIM2 프리스케일러=63 -> 1MHz 타이머)
    uint32_t arr = 1000000u / freq - 1u;
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    uint32_t period = arr + 1u;
    uint32_t duty = (period / 2u) * VOL / 100u;  // 듀티 사이클 = 50% * 볼륨 비율
    if (VOL && duty == 0) duty = 1;              // 볼륨이 0이 아니면서 duty가 0이면 최소 1로 조정
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}
void melody_start(const Note *seq, int len) {
    // 멜로디 재생 시작 설정 (노트 배열과 길이를 지정)
    g_player.seq = seq;
    g_player.length = len;
    g_player.idx = 0;
    g_player.phase = 0;
    g_player.playing = 1;
    g_player.next_ms = 0;
}
void melody_stop(void) {
    // 멜로디 재생 중지
    g_player.playing = 0;
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
}
static inline void melody_tick(void) {
    // 멜로디 재생 상태를 주기적으로 업데이트 (HAL_GetTick() 기반)
    if (!g_player.playing) return;           // 재생 중이 아니면 처리 안 함
    uint32_t now = HAL_GetTick();
    if (now < g_player.next_ms) return;      // 아직 다음 단계 시각이 안 되면 대기

    switch (g_player.phase) {
    case 0: {  // 새로운 음을 로드하는 단계
        if (g_player.idx >= g_player.length) {
            melody_stop(); // 모든 음표 연주 완료 시 멜로디 종료
            return;
        }
        Note n = g_player.seq[g_player.idx++];     // 다음 음표 가져오기
        if (n.frequency == 0)
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4); // 주파수 0이면 (REST) 부저 OFF
        else
            buzzer_set_freq_ch2(n.frequency);       // 해당 음표 주파수로 부저 음 출력 시작
        g_player.phase = 1;
        g_player.next_ms = now + n.duration;        // 음표 지속 시간 만큼 대기 설정
        break;
    }
    case 1: // 음표 지속 시간 다 됨 -> 부저 끄고 잠깐 쉼
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
        g_player.phase = 2;
        g_player.next_ms = now + 10; // 10ms 짧은 쉼
        break;
    case 2: // 쉼 시간 후 다음 음으로 넘어감
        g_player.phase = 0;
        break;
    }
}

// 마리오 테마곡 음표 배열 (Note 구조체의 시퀀스)
const Note mario_theme[] = {
    { NOTE_E5, EIGHTH }, { NOTE_DS5, EIGHTH }, { NOTE_E5, EIGHTH }, { NOTE_DS5, EIGHTH },
    { NOTE_E5, EIGHTH }, { NOTE_B4, EIGHTH },  { NOTE_D5, EIGHTH }, { NOTE_C5, EIGHTH },
    { NOTE_A4, QUARTER },{ REST, EIGHTH },
    { NOTE_C4, EIGHTH }, { NOTE_E4, EIGHTH }, { NOTE_A4, EIGHTH }, { NOTE_B4, EIGHTH },
    { NOTE_E4, EIGHTH }, { NOTE_GS4,EIGHTH }, { NOTE_B4, EIGHTH }, { NOTE_C5, EIGHTH },
    { NOTE_E4, EIGHTH }, { NOTE_E5, EIGHTH }, { NOTE_DS5,EIGHTH }, { NOTE_E5, EIGHTH },
    { NOTE_DS5,EIGHTH }, { NOTE_E5, EIGHTH }, { NOTE_B4, EIGHTH }, { NOTE_D5, EIGHTH },
    { NOTE_C5, EIGHTH }, { NOTE_A4, QUARTER },
    { REST, EIGHTH },    { NOTE_C4, EIGHTH }, { NOTE_E4, EIGHTH }, { NOTE_A4, EIGHTH },
    { NOTE_A4, QUARTER }
};
const int mario_theme_length = sizeof(mario_theme) / sizeof(mario_theme[0]);

void play_tone(uint16_t frequency, uint16_t duration) {
    // 단일 음을 지정 시간만큼 연주 (동기 지연 방식)
    if (frequency == 0 || frequency == REST)
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    else
        buzzer_set_freq_ch2(frequency);
    HAL_Delay(duration);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    HAL_Delay(10);
}
void play_mario_theme(void) {
    melody_start(mario_theme, mario_theme_length); // 마리오 테마 멜로디 재생 시작
}
/* <MELODY_PLAYER end> */

/* <ULTRASONIC_DRIVER begin> */
// **HC-SR04 초음파 센서 제어** (거리 측정용 트리거/에코 처리 함수들)
void timer_start(void) {
    HAL_TIM_Base_Start(&htim1); // TIM1 기본 타이머 시작 (초음파 센서 시간 측정용)
}
static inline uint16_t t1_cnt(void)  {
    return __HAL_TIM_GET_COUNTER(&htim1); // TIM1 카운터 값 읽기
}
static inline void     t1_zero(void) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);     // TIM1 카운터 0으로 초기화
}

// 지정된 핀이 원하는 신호(level)가 될 때까지 대기 (timeout_us 마이크로초 이내)
static int wait_level_u(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level, uint16_t timeout_us) {
    t1_zero();
    while (HAL_GPIO_ReadPin(port, pin) != level) {
        if (t1_cnt() > timeout_us) return 0; // 제한 시간 초과 시 실패
    }
    return 1; // 원하는 신호 감지
}

// 초음파 센서 Echo 핀으로부터 왕복 시간 측정 (마이크로초 단위 반환)
static uint16_t read_echo_us(GPIO_TypeDef *port, uint16_t pin) {
    wait_level_u(port, pin, GPIO_PIN_RESET, 1000);    // 이전에 남은 High 펄스가 있으면 리셋될 때까지 대기
    if (!wait_level_u(port, pin, GPIO_PIN_SET, 5000)) return 0; // 일정 시간 내에 High 시작되지 않으면 실패
    t1_zero();
    if (!wait_level_u(port, pin, GPIO_PIN_RESET, ULTRA_MAX_US)) return 0; // High -> Low 전환까지 대기
    uint16_t us = t1_cnt();
    if (us < ULTRA_MIN_US || us > ULTRA_MAX_US) return 0; // 범위 밖 값은 잘못된 것으로 간주
    return us; // 초음파 신호 왕복 시간(마이크로초) 반환
}
void delay_us(uint16_t us) {
    // 주어진 마이크로초만큼 지연 (TIM1 이용)
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us) ;
}
void trig_right(void) {
    // 오른쪽 초음파 센서 트리거 펄스 발생 (10us High)
    HAL_GPIO_WritePin(TRIG_RIGHT_GPIO_Port, TRIG_RIGHT_Pin, GPIO_PIN_RESET);
    delay_us(3);
    HAL_GPIO_WritePin(TRIG_RIGHT_GPIO_Port, TRIG_RIGHT_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_RIGHT_GPIO_Port, TRIG_RIGHT_Pin, GPIO_PIN_RESET);
}
void trig_left(void) {
    // 왼쪽 초음파 센서 트리거 펄스 발생 (10us High)
    HAL_GPIO_WritePin(TRIG_LEFT_GPIO_Port, TRIG_LEFT_Pin, GPIO_PIN_RESET);
    delay_us(3);
    HAL_GPIO_WritePin(TRIG_LEFT_GPIO_Port, TRIG_LEFT_Pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_LEFT_GPIO_Port, TRIG_LEFT_Pin, GPIO_PIN_RESET);
}

/* (구 버전 직접 측정 함수: 현재 미사용 상태, 삭제 대신 주석 처리) */
/* <LEGACY_UNUSED begin> */
// long unsigned int echo_right(void) { /* ... */ return 0; }
// long unsigned int echo_left(void)  { /* ... */ return 0; }
// static uint16_t med3(uint16_t a, uint16_t b, uint16_t c) { /* ... */ return b; }
 /* <LEGACY_UNUSED end> */
/* <ULTRASONIC_DRIVER end> */

/* <PRINTF_RETARGET begin> */
// **printf 리디렉트 구현** (printf 함수 출력이 USART2를 통해 나가도록 설정)
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int __putchar(int ch)
#endif
PUTCHAR_PROTOTYPE {
    if (ch == '\n') HAL_UART_Transmit(&huart2, (uint8_t*)"\r", 1, 0xFFFF);
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
    return ch;
}
/* <PRINTF_RETARGET end> */

/* <MOTOR_CONTROL begin> */
// **모터 제어 함수 구현** (H-브릿지 방식으로 각 바퀴 회전 방향 제어)
void smartcar_forward() {
    // 전진: 좌/우측 바퀴 모두 forward 방향 회전
    HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, GPIO_PIN_SET);   // 왼쪽 앞바퀴 전진
    HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, GPIO_PIN_RESET); // 왼쪽 앞바퀴 후진 정지
    HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, GPIO_PIN_SET);   // 오른쪽 앞바퀴 전진
    HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, GPIO_PIN_RESET); // 오른쪽 앞바퀴 후진 정지
    HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, GPIO_PIN_SET);   // 왼쪽 뒷바퀴 전진
    HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, GPIO_PIN_RESET); // 왼쪽 뒷바퀴 후진 정지
    HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, GPIO_PIN_SET);   // 오른쪽 뒷바퀴 전진
    HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, GPIO_PIN_RESET); // 오른쪽 뒷바퀴 후진 정지
}
void smartcar_backward() {
    // 후진: 좌/우측 바퀴 모두 backward 방향 회전
    HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, GPIO_PIN_RESET); // 왼앞 전진 정지
    HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, GPIO_PIN_SET);   // 왼앞 후진
    HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, GPIO_PIN_RESET); // 오앞 전진 정지
    HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, GPIO_PIN_SET);   // 오앞 후진
    HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, GPIO_PIN_RESET); // 왼뒤 전진 정지
    HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, GPIO_PIN_SET);   // 왼뒤 후진
    HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, GPIO_PIN_RESET); // 오른뒤 전진 정지
    HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, GPIO_PIN_SET);   // 오른뒤 후진
}
void smartcar_right() {
    // 우회전: 좌측 바퀴 전진 + 우측 바퀴 후진 (제자리 우회전)
    HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, GPIO_PIN_SET);   // 왼앞 전진
    HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, GPIO_PIN_RESET); // 왼앞 후진 정지
    HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, GPIO_PIN_RESET); // 오앞 전진 정지
    HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, GPIO_PIN_SET);   // 오앞 후진
    HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, GPIO_PIN_SET);   // 왼뒤 전진
    HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, GPIO_PIN_RESET); // 왼뒤 후진 정지
    HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, GPIO_PIN_RESET); // 오른뒤 전진 정지
    HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, GPIO_PIN_SET);   // 오른뒤 후진
}
void smartcar_left() {
    // 좌회전: 우측 바퀴 전진 + 좌측 바퀴 후진 (제자리 좌회전)
    HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, GPIO_PIN_RESET); // 왼앞 전진 정지
    HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, GPIO_PIN_SET);   // 왼앞 후진
    HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, GPIO_PIN_SET);   // 오앞 전진
    HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, GPIO_PIN_RESET); // 오앞 후진 정지
    HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, GPIO_PIN_RESET); // 왼뒤 전진 정지
    HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, GPIO_PIN_SET);   // 왼뒤 후진
    HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, GPIO_PIN_SET);   // 오른뒤 전진
    HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, GPIO_PIN_RESET); // 오른뒤 후진 정지
}
void smartcar_stop() {
    // 정지: 모든 바퀴에 대한 전진/후진 신호 Off
    HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, GPIO_PIN_RESET);
}
/* <MOTOR_CONTROL end> */

/* <SERVO_ATTACK_FSM begin> */
// **서보 공격 FSM** (한 번의 공격 동작 수행: 서보 전진 후 복귀)
static inline void attack_start_once(void) {
    if (g_att.st != ATT_IDLE) return; // 이미 동작 중이면 무시
    // 현재 서보 위치 기록 후 서보를 공격 자세(OUT_DEG 각도)로 이동
    g_att.cur_ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);
    if (g_att.cur_ccr < SERVO_CCR_MIN || g_att.cur_ccr > SERVO_CCR_MAX)
        g_att.cur_ccr = SERVO_CCR_MIN;
    servo_set_deg_ch3(ATTACK_OUT_DEG);   // 서보를 지정된 전진 각도로 이동
    g_att.st = ATT_PAUSE;
    g_att.next_ms = HAL_GetTick() + ATTACK_PAUSE_MS; // 일정 시간 대기 후 복귀 예정
}
static inline void attack_cancel(void) {
    // 공격 동작 취소: 상태 리셋하고 서보를 0도로 복귀
    g_att.st = ATT_IDLE;
    servo_set_deg_ch3(0);
}
static inline void attack_tick(void) {
    // 공격 FSM 상태 업데이트 (주기적으로 호출)
    if (g_att.st == ATT_IDLE) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - g_att.next_ms) < 0) return; // 아직 대기 시간 지나지 않음
    switch (g_att.st) {
    case ATT_PAUSE:
        servo_set_deg_ch3(0); // 대기 시간 후 서보를 0도 위치(초기 위치)로 되돌림
        g_att.st = ATT_IDLE;  // 공격 동작 완료 (상태를 대기로)
        break;
    default:
        g_att.st = ATT_IDLE;
        break;
    }
}
/* <SERVO_ATTACK_FSM end> */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  /* <PRE_INIT begin> */
  /* (사용자 사전 초기화 코드 없음) */
  /* <PRE_INIT end> */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();                // HAL 라이브러리 초기화
  SystemClock_Config();      // 시스템 클록 설정 (PLLCLK = 64MHz 등)

  /* Initialize all configured peripherals */
  MX_GPIO_Init();            // GPIO 초기화 (핀 모드 설정)
  MX_USART2_UART_Init();     // USART2 초기화 (PC 연결 시 디버그 출력용)
  MX_TIM1_Init();            // TIM1 초기화 (초음파 센서 타이머)
  MX_USART3_UART_Init();     // USART3 초기화 (BLE 블루투스 통신용)
  MX_TIM2_Init();            // TIM2 초기화 (부저 PWM 타이머)
  MX_TIM3_Init();            // TIM3 초기화 (서보 PWM 타이머)
  MX_SPI1_Init();            // SPI1 초기화 (LCD 통신용)
  MX_TIM4_Init();            // TIM4 초기화 (DHT11 타이머)

  /* USER CODE BEGIN 2 */
  /* <BOOT_UI begin> */
  // LCD에 부팅 화면 표시
  LCD_Init();
  LCD_Fill(BLACK);
  LCD_DrawString(10, 30, "HELLO MINHO",        WHITE,  BLACK);
  LCD_DrawString(10, 45, "RC CAR PROJECT",     GREEN,  BLACK);
  LCD_DrawString(10, 60, "SYSTEM BOOTING...",  CYAN,   BLACK);
  LCD_DrawString(10, 75, "LET'S KICK THE WORLD", YELLOW, BLACK);
  /* <BOOT_UI end> */

  /* <PERIPHERAL_START begin> */
  // 각종 주변장치 및 타이머 동작 시작
  next_dht_ms = HAL_GetTick() + 2000;             // 2초 후 첫 DHT11 측정 예약
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);       // TIM3 서보 PWM 채널 시작
  HAL_TIM_Base_Start(&htim4);                     // TIM4 타이머 시작 (DHT11용)
  servo_set_ccr_ch3(SERVO_CCR_MIN);               // 서보모터를 최소 위치로 설정

  timer_start();           // TIM1 초음파용 타이머 시작
  uart3_start_rx_it();     // UART3(BLE) 인터럽트 수신 시작

  printf("READY: USART3@9600, BLE BT04-A\r\n");   // PC 터미널에 BLE 모듈 준비 메시지 출력
  printf("Ranging with HC-SR04\n");               // 초음파 거리 측정 시작 메시지 출력

  next_ultra_ms = HAL_GetTick() + 200;            // 200ms 후 초음파 첫 측정 예약

  // 한 번 초기 호출로 FSM 상태 업데이트 (필요한 경우)
  melody_tick();
  attack_tick();
  safety_tick();
  /* <PERIPHERAL_START end> */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* <MAIN_LOOP begin> */
    // 주기적으로 FSM 및 센서 처리
    melody_tick();    // 멜로디 재생 상태 업데이트
    attack_tick();    // 공격 서보 상태 업데이트
    safety_tick();    // 안전모드 FSM 상태 업데이트

    uint32_t now = HAL_GetTick();

    // 1) DHT11 주기적 측정
    if ((int32_t)(now - next_dht_ms) >= 0) {
      uint8_t h = 0, t = 0;
      int ok = DHT11_Read(&h, &t);            // DHT11 센서로부터 습도/온도 읽기
      if (ok) { last_h = h; last_t = t; }     // 읽기 성공 시 최근 값 갱신
      LCD_PrintTH(10, 60, ok, last_h, last_t); // LCD에 온도/습도 값 또는 에러 출력
      next_dht_ms = now + 10000;             // 10초 후 다음 측정
    }

    // 2) BLE 입력 처리 (리모컨 명령 수신)
    uint8_t b, cmd = 0;
    // 링버퍼에 쌓인 UART3(BLE) 데이터 처리하여 마지막 명령 한 글자 추출
    while (rb_get(&b)) {
      if (b == '\r' || b == '\n' || b == '\t' || b == ' ') continue;      // 줄바꿈/공백 제거
      if (b >= 'A' && b <= 'Z') b = (uint8_t)(b - 'A' + 'a');             // 대문자는 소문자로 변환
      if (b=='w'||b=='a'||b=='s'||b=='d'||b=='f'||b=='x') cmd = b;        // 유효한 명령 문자만 저장
    }
    if (cmd && safety_active() && cmd != 'f') cmd = 0; // 안전모드 중엔 'f'(정지) 외 명령 무시

    if (cmd && !safety_active()) {
      const char *label = NULL;
      switch (cmd) {
      case 'w':
        smartcar_forward(); melody_stop(); label = "forward";
        is_driving_forward = 1; printf("forward\n");        // 'w': 전진 (멜로디 중단, 전진상태 플래그 ON)
        break;
      case 'a':
        smartcar_left();    melody_stop(); label = "left";
        printf("left\n");                                  // 'a': 좌회전 (멜로디 중단)
        break;
      case 'd':
        smartcar_right();   melody_stop(); label = "right";
        printf("right\n");                                 // 'd': 우회전 (멜로디 중단)
        break;
      case 's':
        smartcar_backward(); melody_start(mario_theme, mario_theme_length);
        label = "backward";
        is_driving_forward = 0; printf("back + melody\n"); // 's': 후진 (마리오 멜로디 재생 시작, 전진상태 플래그 OFF)
        break;
      case 'f':
        smartcar_stop();   melody_stop(); label = "stop";
        is_driving_forward = 0; printf("stop\n");          // 'f': 정지 (모터/멜로디 모두 정지, 전진상태 플래그 OFF)
        break;
      case 'x':
        attack_start_once(); label = "attack";
        printf("attack\n");                                // 'x': 공격 서보 동작 (한번 펀치)
        break;
      default:
        label = "invalid";
        printf("invalid\n");                               // 기타: 알 수 없는 명령
        break;
      }
      if (label) {
        // 처리한 명령을 BLE로 에코(응답) 전송
        HAL_UART_Transmit(&huart3, (uint8_t*)label, (uint16_t)strlen(label), 20);
        HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 20);
      }
    }

    // 3) 초음파 센서 교대 측정
    if ((int32_t)(now - next_ultra_ms) >= 0) {
      if (ultra_side == 0) {
        trig_left(); // 왼쪽 초음파 트리거 펄스 발신
        uint16_t us = read_echo_us(ECHO_LEFT_GPIO_Port, ECHO_LEFT_Pin); // 왼쪽 Echo 시간 측정
        if (us) {
          dist_left  = (int)(17 * us / 100);
          printf("LEFT=%dmm\n",  dist_left);  // 거리(mm) 계산 (음속 이용) 후 출력
        } else {
          dist_left  = 0; // 유효한 신호 못 받으면 0으로 설정
        }
      } else {
        trig_right(); // 오른쪽 초음파 트리거 발신
        uint16_t us = read_echo_us(ECHO_RIGHT_GPIO_Port, ECHO_RIGHT_Pin); // 오른쪽 Echo 시간 측정
        if (us) {
          dist_right = (int)(17 * us / 100);
          printf("RIGHT=%dmm\n", dist_right); // 거리 계산 후 출력
        } else {
          dist_right = 0;
        }
      }
      ultra_side ^= 1;              // 다음 번에는 반대쪽 센서 측정
      next_ultra_ms = now + 60;     // 60ms 후 다음 측정 (센서마다 120ms 간격)
    }

    // 4) 안전 거리 체크 및 안전모드 트리거
    int min_mm = 0;
    if (dist_left > 0 && dist_right > 0)
        min_mm = (dist_left < dist_right) ? dist_left : dist_right;
    else if (dist_left > 0)
        min_mm = dist_left;
    else if (dist_right > 0)
        min_mm = dist_right;

    if (!safety_active() && is_driving_forward && min_mm > 0 && min_mm <= SAFE_MM_THRESHOLD) {
      safety_start(); // 전진 중 장애물이 임계 거리 이내로 감지되면 안전모드 진입
    }
    /* <MAIN_LOOP end> */
  }
  /* USER CODE END 3 */
}

/* ===== STM32CubeMX 생성 초기화 코드 (변경 없음, 마커로 구분) ===== */

void SystemClock_Config(void) {
  /* <CUBEMX_CLOCK_INIT begin> */
  // 시스템 클록 설정 (HSI 8MHz -> PLL -> 64MHz SYSCLK 등)
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
  /* <CUBEMX_CLOCK_INIT end> */
}

static void MX_SPI1_Init(void) {
  /* <CUBEMX_SPI1_INIT begin> */
  // SPI1 초기화 (ST7735S LCD 통신 SPI 설정)
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) { Error_Handler(); }
  /* <CUBEMX_SPI1_INIT end> */
}

static void MX_TIM1_Init(void) {
  /* <CUBEMX_TIM1_INIT begin> */
  // TIM1 초기화 (프리스케일러 63 -> 1MHz, 초음파 딜레이/측정용)
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  /* <CUBEMX_TIM1_INIT end> */
}

static void MX_TIM2_Init(void) {
  /* <CUBEMX_TIM2_INIT begin> */
  // TIM2 초기화 (프리스케일러 63 -> 1MHz, PWM 채널4 사용 부저 출력)
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim2);
  /* <CUBEMX_TIM2_INIT end> */
}

static void MX_TIM3_Init(void) {
  /* <CUBEMX_TIM3_INIT begin> */
  // TIM3 초기화 (프리스케일러 1279, PWM 채널3 사용 서보 제어)
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1279;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim3);
  /* <CUBEMX_TIM3_INIT end> */
}

static void MX_TIM4_Init(void) {
  /* <CUBEMX_TIM4_INIT begin> */
  // TIM4 초기화 (프리스케일러 63 -> 1MHz, DHT11 신호 측정용)
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) { Error_Handler(); }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) { Error_Handler(); }
  /* <CUBEMX_TIM4_INIT end> */
}

static void MX_USART2_UART_Init(void) {
  /* <CUBEMX_USART2_INIT begin> */
  // USART2 초기화 (BaudRate 115200, 디버깅용 시리얼)
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
  /* <CUBEMX_USART2_INIT end> */
}

static void MX_USART3_UART_Init(void) {
  /* <CUBEMX_USART3_INIT begin> */
  // USART3 초기화 (BaudRate 9600, BLE 모듈 BT04-A 통신용)
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) { Error_Handler(); }
  /* <CUBEMX_USART3_INIT end> */
}

static void MX_GPIO_Init(void) {
  /* <CUBEMX_GPIO_INIT begin> */
  // GPIO 초기화 (모터, 센서, LED, LCD, 버튼 핀 설정)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // 초기 출력 핀 상태 설정 (일단 모두 Low로 초기화)
  HAL_GPIO_WritePin(GPIOA, LED2_Pin|LCD_RES_Pin|LCD_DC_Pin|TRIG_LEFT_Pin|LBF_Pin|LFB_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LBB_Pin|LFF_Pin|RFB_Pin|RFF_Pin|LCD_CS_Pin|RBF_Pin|RBB_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TRIG_RIGHT_GPIO_Port, TRIG_RIGHT_Pin, GPIO_PIN_RESET);

  // 사용자 버튼 B1 (PC13) 핀: 상승엣지 인터럽트 입력
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  // GPIOA 포트 출력 설정: LED2, LCD_RES, LCD_DC, TRIG_LEFT, LBF, LFB 핀
  GPIO_InitStruct.Pin = LED2_Pin|LCD_RES_Pin|LCD_DC_Pin|TRIG_LEFT_Pin|LBF_Pin|LFB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // 초음파 Echo Left 핀 입력 (풀다운)
  GPIO_InitStruct.Pin = ECHO_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ECHO_LEFT_GPIO_Port, &GPIO_InitStruct);
  // 초음파 Echo Right 핀 입력 (풀다운)
  GPIO_InitStruct.Pin = ECHO_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(ECHO_RIGHT_GPIO_Port, &GPIO_InitStruct);

  // GPIOB 포트 출력 설정: LBB, LFF, RFB, RFF, LCD_CS, RBF, RBB 핀
  GPIO_InitStruct.Pin = LBB_Pin|LFF_Pin|RFB_Pin|RFF_Pin|LCD_CS_Pin|RBF_Pin|RBB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // 초음파 Trig Right 핀 출력 설정
  GPIO_InitStruct.Pin = TRIG_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_RIGHT_GPIO_Port, &GPIO_InitStruct);

  // DHT11 센서 핀 입력 설정 (풀업)
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  // 외부 인터럽트 (B1 버튼) 우선순위 설정 및 활성화
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* <CUBEMX_GPIO_INIT end> */
}

/* USER CODE BEGIN 4 */
/* <ERROR_HANDLER begin> */
// 오류 발생 시 불리는 함수 (무한 루프에 진입하여 프로그램 중단)
void Error_Handler(void) {
  __disable_irq();
  while (1) { }
}
/* <ERROR_HANDLER end> */

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
  /* <ASSERT_FAILED begin> */
  // (디버그용) 잘못된 파라미터로 함수 호출될 경우 실행됨
  // printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* <ASSERT_FAILED end> */
}
#endif
/* USER CODE END 4 */
