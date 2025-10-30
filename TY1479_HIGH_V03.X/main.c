/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules
  selected in the GUI. Generation Information : Product Revision  :  PIC10 /
  PIC12 / PIC16 / PIC18 MCUs - 1.81.8 Device            :  PIC16F1936 Driver
  Version    :  2.00
 */

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip software
   and any derivatives exclusively with Microchip products. It is your
   responsibility to comply with third party license terms applicable to your
   use of third party software (including open source software) that may
   accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
 */
//**********************************************************************************
//                        PIC16F1936 Pinout for this example
//                                  ------------
//                             MCLR |1       28| RB7 ->
//                    HB_EN ->  RA0 |2       27| RB6 ->
//           (未使用)  P2_EN ->  RA1 |3       26| RB5 ->
//                          ->  RA2 |4       25| RB4 ->
//                          ->  RA3 |5       24| RB3 ->
//                          ->  RA4 |6       23| RB2 ->
//                          ->  RA5 |7       22| RB1 ->
//                              GND |8       21| RB0 -> LB_EN (CCP4)
//                          ->  RA7 |9       20| VDD
//                   DRL_EN ->  RA6 |10      19| VSS
//                          ->  RC0 |11      18| RC7
//                          ->  RC1 |12      17| RC6 -> ECCP3 IN
//             (CCP1) DT_EN ->  RC2 |13      16| RC5 ->
//                          ->  RC3 |14      15| RC4 ->
//                                  ------------
//
//***********************************************************************************
// 20250915 TY1479_HIGH_R V01 CS:86C1
// 20250915 TY1479_HIGH_L V01 CS:8741
// 參考 TY1479_HIGH件動作表.xlsx
// 缺少的CS由0x00修正為0xFF
// HiBeam active-high
// DRL active-low
// DRL(5晶) 串 POS(3晶)
// TY1479 沒有P2 變成第2行車(功能同P2)
// 行車白光 P2黃光

//20251016 TY1479_HIGH V02 CS:EC60
//新增RB3判斷是左邊的燈還是右邊的燈
//加RB3電阻對地 是R邊
//沒加RB3電阻對地 是L邊

//20251030 TY1479_HIGH V03 CS:D7CE
//修改CHECKSUM


#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/pin_manager.h"
#include <pic.h>
#include <stdint.h> //uint8_t
// #include <stdio.h>  //printf

void POS_ON(void);

typedef union {
  struct {
    _Bool b0 : 1;
    _Bool b1 : 1;
    _Bool POS : 1;
    _Bool DRL : 1;
    _Bool LoBeam : 1;
    _Bool HiBeam : 1;
    _Bool b6 : 1;
    _Bool b7 : 1;
  };
  uint8_t Byte;
} flag_D2;
typedef union {
  struct {
    _Bool b0 : 1;
    _Bool b1 : 1;
    _Bool b2 : 1;
    _Bool b3 : 1;
    _Bool b4 : 1;
    _Bool turnLight_L : 1;
    _Bool turnLight_R : 1;
    _Bool b7 : 1;
  };
  uint8_t Byte;
} flag_D3;
typedef union {
  struct {
    _Bool turnLight_R : 1;
    _Bool turnLight_L : 1;
    _Bool HiBeam : 1;
    _Bool LoBeam : 1;
    _Bool DRL : 1;
    _Bool POS : 1;
    _Bool b6 : 1;
    _Bool b7 : 1;
  };
  uint8_t Byte;
} flag_CS_index;

uint8_t rx_data = 0;
volatile bool data_1byte_OK = 0;
bool fFrame_OK = 0;
uint8_t data_buf[11];
uint8_t step = 0;

flag_D2 D2;
flag_D3 D3;
flag_CS_index csFlag;

volatile uint8_t T10MS_CNT = 0; // 單位:10ms
// bool fPWM1_ON = 0;
bool fException = 0;
bool fException2 = 0;
bool fLampSide = 0;

#define LOGIC_LEVEL (16)
// #define LOGIC_LEVEL (20)
#define HiBeam_ON() HB_EN_SetHigh() // active-high
#define HiBeam_OFF() HB_EN_SetLow()
// #define LoBeam_ON() LB_EN_SetHigh()
#define LoBeam_OFF() LB_EN_SetLow()
#define P2_ON() P2_EN_SetHigh() // 未使用
#define P2_OFF() P2_EN_SetLow() // 未使用
#define P2_OFF() P2_EN_SetLow() // 未使用

#define DRL_EN_ON() DRL_EN_SetLow() // active-low
#define POS_EN_ON() DRL_EN_SetHigh()
#define DRL_PWM_SET() CCPR1L = 201
#define POS_PWM_SET() CCPR1L = 47
#define DRL_OFF() CCPR1L = 0

#define LampSideSelect() Lamp_R_EN_PORT
#define LAMP_R_SIDE 0
#define LAMP_L_SIDE 1

void LoBeam_ON() {
  LB_EN_SetHigh();
  NOP();
  NOP();
  NOP();
}
static uint8_t CS[64] = {
    // 行車,晝行,近燈,遠燈,左方,右方
    0x4C, // 000000 關
    0x9F, // 000001 右方
    0xED, // 000010 左方
    // 0x62, // 000011 右方+左方(警示)
    0x3E, // 000011 右方+左方(警示)

    0x9A, // 000100 遠燈
    0x00, // 000101 遠燈+右方
    0x00, // 000110 遠燈+左方
    0x00, // 000111 遠燈+右方+左方(警示)

    0x27, // 001000 近燈
    0x00, // 001001 近燈+右方
    0x00, // 001010 近燈+左方
    0x00, // 001011 近燈+右方+左方(警示)

    // 0x73, // 001100 遠燈+近燈(超車)
    // 0xD6, // 001100 遠燈+近燈(超車)
    0xF1, // 001100 遠燈+近燈(超車)
    0xA0, // 001101 遠燈+近燈(超車)+右方
    // 0x71, // 001110 遠燈+近燈(超車)+左方
    0xD2, // 001110 遠燈+近燈(超車)+左方
    0x01, // 001111 遠燈+近燈(超車)+右方+左方(警示)

    // 0x8E, // 010000 晝行
    0xB1, // 010000 晝行
    0x5D, // 010001 晝行+右方
    0x2F, // 010010 晝行+左方
    0xFC, // 010011 晝行+右方+左方(警示)

    0x67, // 010100 晝行+遠燈
    0x00, // 010101 晝行+遠燈+右方
    0x00, // 010110 晝行+遠燈+左方
    0x00, // 010111 晝行+遠燈+右方+左方(警示)

    0xDA, // 011000 晝行+近燈
    0x00, // 011001 晝行+近燈+右方
    0x00, // 011010 晝行+近燈+左方
    0x00, // 011011 晝行+近燈+右方+左方(警示)

    // 0xB1, // 011100 畫行+遠燈+近燈(超車)
    0x0C, // 011100 畫行+遠燈+近燈(超車)
    // 0xA0, // 011101 畫行+遠燈+近燈(超車)+右方
    0x62, // 011101 畫行+遠燈+近燈(超車)+右方
    // 0x71, // 011110 畫行+遠燈+近燈(超車)+左方
    0x10, // 011110 畫行+遠燈+近燈(超車)+左方
    // 0x01, // 011111 晝行+遠燈+近燈(超車)+右方+左方(警示)
    0xC3, // 011111 晝行+遠燈+近燈(超車)+右方+左方(警示)

    // 0x2D, // 100000 行車
    0xFA, // 100000 行車
    0xFE, // 100001 行車+右方
    0x8C, // 100010 行車+左方
    0x5F, // 100011 行車+右方+左方(警示)

    0x2C, // 100100 行車+遠燈
    0x00, // 100101 行車+遠燈+右方
    0x00, // 100110 行車+遠燈+左方
    0x00, // 100111 行車+遠燈+右方+左方(警示)

    // 0x38, // 101000 行車+近燈
    0x91, // 101000 行車+近燈
    0xEB, // 101001 行車+近燈+右方
    0x99, // 101010 行車+近燈+左方
    0x4A, // 101011 行車+近燈+右方+左方(警示)

    // 0x12, // 101100 行車+遠燈+近燈(超車)
    0x47, // 101100 行車+遠燈+近燈(超車)
    0xC1, // 101101 行車+遠燈+近燈(超車)+右方
    0xB3, // 101110 行車+遠燈+近燈(超車)+左方
    0x60, // 101111 行車+遠燈+近燈(超車)+右方+左方(警示)

    // 0xEF, // 110000 行車+晝行
    0x07, // 110000 行車+晝行
    0x3C, // 110001 行車+晝行+右方
    0x4E, // 110010 行車+晝行+左方
    0x9D, // 110011 行車+晝行+右方+左方(警示)

    0xD1, // 110100 行車+晝行+遠燈
    0x00, // 110101 行車+晝行+遠燈+右方
    0x00, // 110110 行車+晝行+遠燈+左方
    0x00, // 110111 行車+晝行+遠燈+右方+左方(警示)

    0x6C, // 111000 行車+晝行+近燈
    0x00, // 111001 行車+晝行+近燈+右方
    0x00, // 111010 行車+晝行+近燈+左方
    0x00, // 111011 行車+晝行+近燈+右方+左方(警示)

    // 0xD0, // 111100 行車+晝行+遠燈+近燈
    0xBA, // 111100 行車+晝行+遠燈+近燈
    0x03, // 111101 行車+晝行+遠燈+近燈+右方
    0x71, // 111110 行車+晝行+遠燈+近燈+左方
    0xA2, // 111111 行車+晝行+遠燈+近燈(超車)+右方+左方(警示)
};

// 特殊信號(unlock) 亮P2

void Exception_handling(void) {
  if ((data_buf[2] == 6) && (data_buf[10] == 0xD5)) {
    P2_ON(); // P2亮
    fException = 1;
    // EPWM1_LoadDutyValue(PWM_DUTY_0_PERCENT); // 0%
    LoBeam_OFF();  // 遠燈 OFF
    HiBeam_OFF();  // 遠燈 OFF
    POS_ON();      // 亮POS
    T10MS_CNT = 0; // 重置2000ms計數器
  } else {
    fException = 0;
  }
}

void DRL_ON(void) {
if(fLampSide==LAMP_R_SIDE){
    // 右方向 關畫行
  if (csFlag.turnLight_R == 1) {
    DRL_EN_ON();
    DRL_OFF();
  } else {
    // DRL_ON
    DRL_EN_ON();
    DRL_PWM_SET();
  }
}else{
    // 左方向 關畫行
  if (csFlag.turnLight_L == 1) {
    DRL_EN_ON();
    DRL_OFF();
  } else {
    // DRL_ON
    DRL_EN_ON();
    DRL_PWM_SET();
  }
}

// 方向和畫行共用光條 有方向時關畫行，防止混光變色
// #ifdef LIGHT_LEFT
//   // 左方向 關畫行
//   if (csFlag.turnLight_L == 1) {
// #else
//   // 右方向 關畫行
//   if (csFlag.turnLight_R == 1) {
// #endif
//     // DRL_OFF
//     DRL_EN_ON();
//     DRL_OFF();
//   } else {
//     // DRL_ON
//     DRL_EN_ON();
//     DRL_PWM_SET();
//   }
}

void POS_ON(void) {
  POS_EN_ON();
  POS_PWM_SET();
}

void LED_output(void) {
  uint8_t crc8 = CS[csFlag.Byte];
  // CRC檢查
  if (data_buf[10] != crc8)
    return;
  if (data_buf[0] != 0xC8)
    return; // CRC檢查
  if (data_buf[1] != 0x8B)
    return; // CRC檢查

  T10MS_CNT = 0; // 重置2000ms計數器

  // TY1479 沒有P2 變成第2行車(功能同P2)
  if (csFlag.POS == 1) {
    P2_ON();
  } else {
    P2_OFF();
  }

  // 0 ALL OFF
  if (csFlag.HiBeam == 0 && csFlag.LoBeam == 0 && csFlag.DRL == 0 &&
      csFlag.POS == 0) {
    LoBeam_OFF(); // 近燈 OFF
    HiBeam_OFF(); // 遠燈 OFF
    DRL_OFF();    // 晝行 OFF
  } // 1 POS = POS
  else if (csFlag.HiBeam == 0 && csFlag.LoBeam == 0 && csFlag.DRL == 0 &&
           csFlag.POS == 1) {
    LoBeam_OFF(); // 近燈 OFF
    HiBeam_OFF(); // 遠燈 OFF
    POS_ON();
  } // 2 DRL = DRL
  else if (csFlag.HiBeam == 0 && csFlag.LoBeam == 0 && csFlag.DRL == 1 &&
           csFlag.POS == 0) {
    LoBeam_OFF(); // 近燈 OFF
    HiBeam_OFF(); // 遠燈 OFF
    DRL_ON();
  } // 3 POS+DRL = DRL
  else if (csFlag.HiBeam == 0 && csFlag.LoBeam == 0 && csFlag.DRL == 1 &&
           csFlag.POS == 1) {
    LoBeam_OFF(); // 近燈 OFF
    HiBeam_OFF(); // 遠燈 OFF
    DRL_ON();
  } // 4 LoBeam = LoBeam
  else if (csFlag.HiBeam == 0 && csFlag.LoBeam == 1 && csFlag.DRL == 0 &&
           csFlag.POS == 0) {
    LoBeam_ON();  // 近燈 ON
    HiBeam_OFF(); // 遠燈 OFF
    DRL_OFF();    // 晝行 OFF
  } // 5 LoBeam+POS = LoBeam+POS
  else if (csFlag.HiBeam == 0 && csFlag.LoBeam == 1 && csFlag.DRL == 0 &&
           csFlag.POS == 1) {
    LoBeam_ON();  // 近燈 ON
    HiBeam_OFF(); // 遠燈 OFF
    POS_ON();
  } // 6 LoBeam+DRL=LoBeam
  else if (csFlag.HiBeam == 0 && csFlag.LoBeam == 1 && csFlag.DRL == 1 &&
           csFlag.POS == 0) {
    LoBeam_ON();  // 近燈 ON
    HiBeam_OFF(); // 遠燈 OFF
    DRL_OFF();
  } // 7 LoBeam+POS+DRL = LoBeam+POS
  else if (csFlag.HiBeam == 0 && csFlag.LoBeam == 1 && csFlag.DRL == 1 &&
           csFlag.POS == 1) {
    LoBeam_ON();  // 近燈 ON
    HiBeam_OFF(); // 遠燈 OFF
    POS_ON();
  } // 8 HiBeam = HiBeam
  else if (csFlag.HiBeam == 1 && csFlag.LoBeam == 0 && csFlag.DRL == 0 &&
           csFlag.POS == 0) {
    HiBeam_ON();  // 遠燈 ON
    LoBeam_OFF(); // 近燈 OFF
    DRL_OFF();    // 晝行 OFF
  } // 9 HiBeam+POS=POS
  else if (csFlag.HiBeam == 1 && csFlag.LoBeam == 0 && csFlag.DRL == 0 &&
           csFlag.POS == 1) {
    HiBeam_OFF(); // 遠燈 OFF
    LoBeam_OFF(); // 近燈 OFF
    POS_ON();
  } // 10 HiBeam+DRL=DRL
  else if (csFlag.HiBeam == 1 && csFlag.LoBeam == 0 && csFlag.DRL == 1 &&
           csFlag.POS == 0) {
    HiBeam_OFF(); // 遠燈 OFF
    LoBeam_OFF(); // 近燈 OFF
    DRL_ON();
  } // 11 HiBeam+POS+DRL = DRL
  else if (csFlag.HiBeam == 1 && csFlag.LoBeam == 0 && csFlag.DRL == 1 &&
           csFlag.POS == 1) {
    HiBeam_OFF(); // 遠燈 OFF
    LoBeam_OFF(); // 近燈 OFF
    DRL_ON();
  } // 12 HiBeam+LoBeam=HiBeam+LoBeam
  else if (csFlag.HiBeam == 1 && csFlag.LoBeam == 1 && csFlag.DRL == 0 &&
           csFlag.POS == 0) {
    HiBeam_ON(); // 遠燈 ON
    LoBeam_ON(); // 近燈 ON
    DRL_OFF();
  } // 13 HiBeam+LoBeam+DRL=HiBeam+LoBeam+DRL
  else if (csFlag.HiBeam == 1 && csFlag.LoBeam == 1 && csFlag.DRL == 1 &&
           csFlag.POS == 0) {
    HiBeam_ON(); // 遠燈 ON
    LoBeam_ON(); // 近燈 ON
    DRL_OFF();
  } // 14 HiBeam+LoBeam+POS=HiBeam+LoBeam+POS
  else if (csFlag.HiBeam == 1 && csFlag.LoBeam == 1 && csFlag.DRL == 0 &&
           csFlag.POS == 1) {
    HiBeam_ON(); // 遠燈 ON
    LoBeam_ON(); // 近燈 ON
    POS_ON();
  } // 15 HiBeam+LoBeam+POS+DRL=HiBeam+LoBeam+POS
  else if (csFlag.HiBeam == 1 && csFlag.LoBeam == 1 && csFlag.DRL == 1 &&
           csFlag.POS == 1) {
    HiBeam_ON(); // 遠燈 ON
    LoBeam_ON(); // 近燈 ON
    POS_ON();
  }

  for (uint8_t i = 0; i < 11; i++) {
    data_buf[i] = 0; // 清除資料緩衝區
  }

  // // 左方向
  // if (csFlag.turnLight_L == 1) {
  //   // LED5_SetHigh();
  // } else {
  //   // LED5_SetLow();
  // }

  // 右方向
  // if (csFlag.turnLight_R == 1) {
  //   // LED6_SetHigh();
  // } else {
  //   // LED6_SetLow();
  // }
}

void check_input(void) {
  D2.Byte = data_buf[2];
  D3.Byte = data_buf[3];
  csFlag.Byte = 0;

  // 行車
  if (D2.POS == 1)
    csFlag.POS = 1;
  else
    csFlag.POS = 0;

  // 晝行
  if (D2.DRL == 1)
    csFlag.DRL = 1;
  else
    csFlag.DRL = 0;

  // 近燈
  if (D2.LoBeam == 1)
    csFlag.LoBeam = 1;
  else
    csFlag.LoBeam = 0;

  // 遠燈
  if (D2.HiBeam == 1)
    csFlag.HiBeam = 1;
  else
    csFlag.HiBeam = 0;

  // 左方向
  if (D3.turnLight_L == 1)
    csFlag.turnLight_L = 1;
  else
    csFlag.turnLight_L = 0;

  // 右方向
  if (D3.turnLight_R == 1)
    csFlag.turnLight_R = 1;
  else
    csFlag.turnLight_R = 0;
}

void getFrameData(void) {
  if (data_1byte_OK == 1) {
    data_1byte_OK = 0;
    // printf("%0
    // printf("\n
    switch (step) {
    case 0:
      if (rx_data == 0xC8) {
        step = 1;
        data_buf[0] = rx_data;
      } else {
        step = 0;
      }
      break;
    case 1:
      if (rx_data == 0x8B) {
        data_buf[1] = rx_data;
        step = 2;
      } else {
        step = 0;
      }
      break;
    case 2:
      data_buf[2] = rx_data;
      step = 3;
      break;
    case 3:
      data_buf[3] = rx_data;
      step = 4;
      break;
    case 4:
      data_buf[4] = rx_data;
      step = 5;
      break;
    case 5:
      data_buf[5] = rx_data;
      step = 6;
      break;
    case 6:
      data_buf[6] = rx_data;
      step = 7;
      break;
    case 7:
      data_buf[7] = rx_data;
      step = 8;
      break;
    case 8:
      data_buf[8] = rx_data;
      step = 9;
      break;
    case 9:
      data_buf[9] = rx_data;
      step = 10;
      break;
    case 10:
      data_buf[10] = rx_data;
      fFrame_OK = 1;
      step = 0;
      break;
    default:
      step = 0;
      break;
    }
  }
}

void ECCP3_CallBack(uint16_t capturedValue) {
  static uint16_t pluse_width_LO = 0;
  static uint16_t pluse_width_HI = 0;
  static uint8_t HI_us = 0;
  static uint8_t LO_us = 0;
  static uint8_t diff_us = 0;
  static uint16_t falling_edge_time = 0;
  static uint16_t rising_edge_time = 0;
  static bool rise_edge_flag = 0;
  static bool fHead = 0;
  static uint8_t data_cnt = 0;

  // falling edge tringgered
  if (CCP3CON == 0x04) {
    falling_edge_time = capturedValue;
    CCP3CON = 0x05; // set to rising edge trigger
    if (rise_edge_flag == 1) {
      rise_edge_flag = 0;

      if (falling_edge_time >= rising_edge_time) {
        pluse_width_HI = falling_edge_time - rising_edge_time;
      } else {
        pluse_width_HI = (0xffff - rising_edge_time) + falling_edge_time;
      }
      LO_us = (pluse_width_LO >> 3) & 0xff;
      HI_us = (pluse_width_HI >> 3) & 0xff;
      if (HI_us > LO_us)
        diff_us = HI_us - LO_us;
      else
        diff_us = LO_us - HI_us;

      if (fHead == 0) {
        UART_TX_SetHigh();
        // init
        if (diff_us < LOGIC_LEVEL) {
          // logical '0'
          fHead = 1;
          data_cnt = 0;
          UART_TX_SetLow();
        }
      } else {
        // data
        if (diff_us < LOGIC_LEVEL) {
          // logical '0'
          UART_TX_SetLow();
          rx_data &= ~(1 << data_cnt);
        } else {
          // logical '1'
          UART_TX_SetHigh();
          rx_data |= (1 << data_cnt);
        }

        data_cnt++;
        if (data_cnt > 7) {
          fHead = 0;
          data_1byte_OK = 1;
        }
      }
    }

  } // rising edge tringgered
  else if (CCP3CON == 0x05) {
    rising_edge_time = capturedValue;
    CCP3CON = 0x04; // set to falling edge trigger
    // 計算低電位脈衝寬度
    if (rising_edge_time >= falling_edge_time) {
      pluse_width_LO = rising_edge_time - falling_edge_time;
    } else {
      pluse_width_LO = (0xffff - falling_edge_time) + rising_edge_time;
    }
    rise_edge_flag = 1;
  }
}

void TMR0_EvenHandler(void) {
  T10MS_CNT++;
  if (T10MS_CNT >= 200) { // 2000ms
    fException2 = 1;      // 設定異常狀態
    T10MS_CNT = 0;
  }
}

/*
                         Main application
 */
int main(void) {
  // initialize the device
  SYSTEM_Initialize();
  TMR0_SetInterruptHandler(TMR0_EvenHandler);
  LoBeam_OFF();
  DRL_OFF();

  // When using interrupts, you need to set the Global and Peripheral
  // Interrupt Enable bits Use the following macros to:

  // Enable the Global Interrupts
  INTERRUPT_GlobalInterruptEnable();

  // Enable the Peripheral Interrupts
  INTERRUPT_PeripheralInterruptEnable();

  // Disable the Global Interrupts
  // INTERRUPT_GlobalInterruptDisable();

  // Disable the Peripheral Interrupts
  // INTERRUPT_PeripheralInterruptDisable();
  
  //左邊燈具選擇
  if(LampSideSelect()==LAMP_R_SIDE)
    fLampSide=LAMP_R_SIDE;
  else
    fLampSide= LAMP_L_SIDE;

    while (1) {
    getFrameData();
    check_input();
    // LED_output();
    Exception_handling();
    if (fException2 == 0) {
      if (fException == 1) {
        // 異常處理
        fException = 0; // 重置異常狀態
      } else
        LED_output();
    } else {
      // 2秒沒收到正確的資料開啟近燈+P2
      fException2 = 0;
      P2_ON();      // P2亮
      HiBeam_OFF(); // 遠燈 OFF
      LoBeam_ON();  // 近燈 ON
      POS_ON();     // POS ON
    }
  }
}

/**
 End of File
 */