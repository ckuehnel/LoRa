/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 *
 * MODIFIED BY IN-CIRCUIT GMBH
 * SX1272 radio-Chip on In-Circuit radino32 SX1272 modules
 * for more information: www.in-circuit.de or www.radino.cc 
 */

#ifndef __RADINO32_SX1272_H__
#define __RADINO32_SX1272_H__

#ifndef __GNUC__
#define inline
#endif

#include <Arduino.h>
#include "SPI.h"

#define GET_TICK_COUNT( )                           ( millis() )

/*!
 *  SPI DEFINES
*/
//#define PIN_SPI_CLK                                 
#define SPI_INTERFACE                               SPI2
#define SPI_CLK                                     RCC_APB1Periph_SPI2

#define SCK_PIN                                     34
#define SPI_PIN_SCK_PORT                            GPIOB
#define SPI_PIN_SCK_PORT_CLK                        RCC_AHBPeriph_GPIOB
#define SPI_PIN_SCK                                 GPIO_Pin_13
#define SPI_SCK_AF_PIN                              GPIO_PinSource13

#define MISO_PIN                                    35
#define SPI_PIN_MISO_PORT                           GPIOB
#define SPI_PIN_MISO_PORT_CLK                       RCC_AHBPeriph_GPIOB
#define SPI_PIN_MISO                                GPIO_Pin_14
#define SPI_MISO_AF_PIN   							GPIO_PinSource14

#define MOSI_PIN                                    36
#define SPI_PIN_MOSI_PORT                           GPIOB
#define SPI_PIN_MOSI_PORT_CLK                       RCC_AHBPeriph_GPIOB
#define SPI_PIN_MOSI                                GPIO_Pin_15
#define SPI_MOSI_AF_PIN   							GPIO_PinSource15

/*!
 * SX1272 RESET I/O definitions
 */
#define PIN_RESET                                   28
#define RESET_PIN                                   PIN_RESET

/*!
 * SX1272 SPI NSS I/O definitions
 */
#define PIN_SPI_NSS                                 29
#define SS_PIN                                      PIN_SPI_NSS

/*!
 * SX1272 DIO pins  I/O definitions
 */
#define PIN_DIO0                                    31
#define PIN_DIO1                                    33
#define PIN_DIO2                                    30     
#define PIN_DIO3                                    32

// NOTE: all functions that control DIO4/DIO5/RXTX have no effect, if DIO4_DUMMY/DIO5_DUMMY/RXTX_DUMMY set to 1
// these functions will than always return 0. 
// Set to 0 to activate usage of PIN_DIO4 / PIN_DIO5 PIN_RXTX.   
#define USE_DIO4_DUMMY             1       // defined for dummy (set 0 for using PIN_DIO4)
#define USE_DIO5_DUMMY             1       // defined for dummy (set 0 for using PIN_DIO5)
#define USE_RXTX_DUMMY             1       // defined for dummy (set 0 for using PIN_RXTX)

// NOTE: not connect on radino32 SX1272, using 0xFF as dummy for initalisation.
#define PIN_DIO4                                    0xFF
#define PIN_DIO5                                    0xFF
#define PIN_RXTX                                    0xFF

/*!
 * SX1272 LoRa General parameters definition
 */
typedef struct sLoRaSettings
{
    uint32_t RFFrequency;
    int8_t Power;
    uint8_t SignalBw;                   // LORA [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved] 
    uint8_t SpreadingFactor;            // LORA [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
    uint8_t ErrorCoding;                // LORA [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
    bool CrcOn;                         // [0: OFF, 1: ON]
    bool ImplicitHeaderOn;              // [0: OFF, 1: ON]
    bool RxSingleOn;                    // [0: Continuous, 1 Single]
    bool FreqHopOn;                     // [0: OFF, 1: ON]
    uint8_t HopPeriod;                  // Hops every frequency hopping period symbols
    uint32_t TxPacketTimeout;
    uint32_t RxPacketTimeout;
    uint8_t PayloadLength;
}tLoRaSettings;

/*!
 * RF packet definition
 */
#define RF_BUFFER_SIZE_MAX                          256
#define RF_BUFFER_SIZE                              256

/*!
 * RF state machine
 */
//LoRa
typedef enum
{
    RFLR_STATE_IDLE,
    RFLR_STATE_RX_INIT,
    RFLR_STATE_RX_RUNNING,
    RFLR_STATE_RX_DONE,
    RFLR_STATE_RX_TIMEOUT,
    RFLR_STATE_TX_INIT,
    RFLR_STATE_TX_RUNNING,
    RFLR_STATE_TX_DONE,
    RFLR_STATE_TX_TIMEOUT,
    RFLR_STATE_CAD_INIT,
    RFLR_STATE_CAD_RUNNING,
}tRFLRStates;

/*!
 * Functions return codes definition
 */
typedef enum
{
    SX_OK,
    SX_ERROR,
    SX_BUSY,
    SX_EMPTY,
    SX_DONE,
    SX_TIMEOUT,
    SX_UNSUPPORTED,
    SX_WAIT,
    SX_CLOSE,
    SX_YES,
    SX_NO,
}tReturnCodes;

//extern volatile uint32_t TickCounter;

/*!
 * SX1272 definitions
 */
#define XTAL_FREQ                                   32000000
#define FREQ_STEP                                   61.03515625

/*!
 * SX1272 Internal registers Address
 */
#define REG_FIFO                                    0x00
// Common settings
#define REG_OPMODE                                  0x01
#define REG_BITRATEMSB                              0x02
#define REG_BITRATELSB                              0x03
#define REG_FDEVMSB                                 0x04 
#define REG_FDEVLSB                                 0x05
#define REG_FRFMSB                                  0x06
#define REG_FRFMID                                  0x07
#define REG_FRFLSB                                  0x08
// Tx settings
#define REG_PACONFIG                                0x09
#define REG_PARAMP                                  0x0A
#define REG_OCP                                     0x0B 
// Rx settings
#define REG_LNA                                     0x0C
#define REG_RXCONFIG                                0x0D
#define REG_RSSICONFIG                              0x0E
#define REG_RSSICOLLISION                           0x0F
#define REG_RSSITHRESH                              0x10
#define REG_RSSIVALUE                               0x11
#define REG_RXBW                                    0x12 
#define REG_AFCBW                                   0x13
#define REG_OOKPEAK                                 0x14
#define REG_OOKFIX                                  0x15
#define REG_OOKAVG                                  0x16
#define REG_RES17                                   0x17
#define REG_RES18                                   0x18
#define REG_RES19                                   0x19
#define REG_AFCFEI                                  0x1A
#define REG_AFCMSB                                  0x1B
#define REG_AFCLSB                                  0x1C
#define REG_FEIMSB                                  0x1D
#define REG_FEILSB                                  0x1E
#define REG_PREAMBLEDETECT                          0x1F
#define REG_RXTIMEOUT1                              0x20
#define REG_RXTIMEOUT2                              0x21
#define REG_RXTIMEOUT3                              0x22
#define REG_RXDELAY                                 0x23
// Oscillator settings
#define REG_OSC                                     0x24
// Packet handler settings
#define REG_PREAMBLEMSB                             0x25
#define REG_PREAMBLELSB                             0x26
#define REG_SYNCCONFIG                              0x27
#define REG_SYNCVALUE1                              0x28
#define REG_SYNCVALUE2                              0x29
#define REG_SYNCVALUE3                              0x2A
#define REG_SYNCVALUE4                              0x2B
#define REG_SYNCVALUE5                              0x2C
#define REG_SYNCVALUE6                              0x2D
#define REG_SYNCVALUE7                              0x2E
#define REG_SYNCVALUE8                              0x2F
#define REG_PACKETCONFIG1                           0x30
#define REG_PACKETCONFIG2                           0x31
#define REG_PAYLOADLENGTH                           0x32
#define REG_NODEADRS                                0x33
#define REG_BROADCASTADRS                           0x34
#define REG_FIFOTHRESH                              0x35
// SM settings
#define REG_SEQCONFIG1                              0x36
#define REG_SEQCONFIG2                              0x37
#define REG_TIMERRESOL                              0x38
#define REG_TIMER1COEF                              0x39
#define REG_TIMER2COEF                              0x3A
// Service settings
#define REG_IMAGECAL                                0x3B
#define REG_TEMP                                    0x3C
#define REG_LOWBAT                                  0x3D
// Status
#define REG_IRQFLAGS1                               0x3E
#define REG_IRQFLAGS2                               0x3F
// I/O settings
#define REG_DIOMAPPING1                             0x40
#define REG_DIOMAPPING2                             0x41
// Version
#define REG_VERSION                                 0x42
// Additional settings
#define REG_AGCREF                                  0x43
#define REG_AGCTHRESH1                              0x44
#define REG_AGCTHRESH2                              0x45
#define REG_AGCTHRESH3                              0x46
#define REG_PLLHOP                                  0x4B
#define REG_TCXO                                    0x58
#define REG_PADAC                                   0x5A
#define REG_PLL                                     0x5C
#define REG_PLLLOWPN                                0x5E
#define REG_FORMERTEMP                              0x6C
#define REG_BITRATEFRAC                             0x70

/*!
 * SX1272 Internal registers Address
 */
#define REG_LR_FIFO                                 0x00 
// Common settings
#define REG_LR_OPMODE                               0x01 
#define REG_LR_FRFMSB                               0x06 
#define REG_LR_FRFMID                               0x07
#define REG_LR_FRFLSB                               0x08 
// Tx settings
#define REG_LR_PACONFIG                             0x09 
#define REG_LR_PARAMP                               0x0A 
#define REG_LR_OCP                                  0x0B 
// Rx settings
#define REG_LR_LNA                                  0x0C 
// LoRa registers
#define REG_LR_FIFOADDRPTR                          0x0D 
#define REG_LR_FIFOTXBASEADDR                       0x0E 
#define REG_LR_FIFORXBASEADDR                       0x0F 
#define REG_LR_FIFORXCURRENTADDR                    0x10 
#define REG_LR_IRQFLAGSMASK                         0x11 
#define REG_LR_IRQFLAGS                             0x12 
#define REG_LR_NBRXBYTES                            0x13 
#define REG_LR_RXHEADERCNTVALUEMSB                  0x14 
#define REG_LR_RXHEADERCNTVALUELSB                  0x15 
#define REG_LR_RXPACKETCNTVALUEMSB                  0x16 
#define REG_LR_RXPACKETCNTVALUELSB                  0x17 
#define REG_LR_MODEMSTAT                            0x18 
#define REG_LR_PKTSNRVALUE                          0x19 
#define REG_LR_PKTRSSIVALUE                         0x1A 
#define REG_LR_RSSIVALUE                            0x1B 
#define REG_LR_HOPCHANNEL                           0x1C 
#define REG_LR_MODEMCONFIG1                         0x1D 
#define REG_LR_MODEMCONFIG2                         0x1E 
#define REG_LR_SYMBTIMEOUTLSB                       0x1F 
#define REG_LR_PREAMBLEMSB                          0x20 
#define REG_LR_PREAMBLELSB                          0x21 
#define REG_LR_PAYLOADLENGTH                        0x22 
#define REG_LR_PAYLOADMAXLENGTH                     0x23 
#define REG_LR_HOPPERIOD                            0x24 
#define REG_LR_FIFORXBYTEADDR                       0x25
#define REG_LR_FEIMSB                               0x28
#define REG_LR_FEIMIB                               0x29
#define REG_LR_FEILSB                               0x2A
#define REG_LR_LORADETECTOPTIMIZE                   0x31
#define REG_LR_INVERTIQ                             0x33
#define REG_LR_DETECTIONTHRESHOLD                   0x37
// end of documented register in datasheet
// I/O settings
#define REG_LR_DIOMAPPING1                          0x40
#define REG_LR_DIOMAPPING2                          0x41
// Version
#define REG_LR_VERSION                              0x42
// Additional settings
#define REG_LR_AGCREF                               0x43
#define REG_LR_AGCTHRESH1                           0x44
#define REG_LR_AGCTHRESH2                           0x45
#define REG_LR_AGCTHRESH3                           0x46
#define REG_LR_PLLHOP                               0x4B
#define REG_LR_TCXO                                 0x58
#define REG_LR_PADAC                                0x5A
#define REG_LR_PLL                                  0x5C
#define REG_LR_PLLLOWPN                             0x5E
#define REG_LR_FORMERTEMP                           0x6C

/*!
 * SX1272 LoRa bit control definition
 */

/*!
 * RegFifo
 */

/*!
 * RegOpMode
 */
#define RFLR_OPMODE_LONGRANGEMODE_MASK              0x7F 
#define RFLR_OPMODE_LONGRANGEMODE_OFF               0x00 // Default
#define RFLR_OPMODE_LONGRANGEMODE_ON                0x80 

#define RFLR_OPMODE_ACCESSSHAREDREG_MASK            0xBF 
#define RFLR_OPMODE_ACCESSSHAREDREG_ENABLE          0x40 
#define RFLR_OPMODE_ACCESSSHAREDREG_DISABLE         0x00 // Default

#define RFLR_OPMODE_MASK                            0xF8 
#define RFLR_OPMODE_SLEEP                           0x00 
#define RFLR_OPMODE_STANDBY                         0x01 // Default
#define RFLR_OPMODE_SYNTHESIZER_TX                  0x02 
#define RFLR_OPMODE_TRANSMITTER                     0x03 
#define RFLR_OPMODE_SYNTHESIZER_RX                  0x04 
#define RFLR_OPMODE_RECEIVER                        0x05 
// LoRa specific modes
#define RFLR_OPMODE_RECEIVER_SINGLE                 0x06 
#define RFLR_OPMODE_CAD                             0x07 

/*!
 * RegFrf (MHz)
 */
#define RFLR_FRFMSB_863_MHZ                         0xD7
#define RFLR_FRFMID_863_MHZ                         0xC0
#define RFLR_FRFLSB_863_MHZ                         0x00
#define RFLR_FRFMSB_864_MHZ                         0xD8
#define RFLR_FRFMID_864_MHZ                         0x00
#define RFLR_FRFLSB_864_MHZ                         0x00
#define RFLR_FRFMSB_865_MHZ                         0xD8
#define RFLR_FRFMID_865_MHZ                         0x40
#define RFLR_FRFLSB_865_MHZ                         0x00
#define RFLR_FRFMSB_866_MHZ                         0xD8
#define RFLR_FRFMID_866_MHZ                         0x80
#define RFLR_FRFLSB_866_MHZ                         0x00
#define RFLR_FRFMSB_867_MHZ                         0xD8
#define RFLR_FRFMID_867_MHZ                         0xC0
#define RFLR_FRFLSB_867_MHZ                         0x00
#define RFLR_FRFMSB_868_MHZ                         0xD9
#define RFLR_FRFMID_868_MHZ                         0x00
#define RFLR_FRFLSB_868_MHZ                         0x00
#define RFLR_FRFMSB_869_MHZ                         0xD9
#define RFLR_FRFMID_869_MHZ                         0x40
#define RFLR_FRFLSB_869_MHZ                         0x00
#define RFLR_FRFMSB_870_MHZ                         0xD9
#define RFLR_FRFMID_870_MHZ                         0x80
#define RFLR_FRFLSB_870_MHZ                         0x00

#define RFLR_FRFMSB_902_MHZ                         0xE1
#define RFLR_FRFMID_902_MHZ                         0x80
#define RFLR_FRFLSB_902_MHZ                         0x00
#define RFLR_FRFMSB_903_MHZ                         0xE1
#define RFLR_FRFMID_903_MHZ                         0xC0
#define RFLR_FRFLSB_903_MHZ                         0x00
#define RFLR_FRFMSB_904_MHZ                         0xE2
#define RFLR_FRFMID_904_MHZ                         0x00
#define RFLR_FRFLSB_904_MHZ                         0x00
#define RFLR_FRFMSB_905_MHZ                         0xE2
#define RFLR_FRFMID_905_MHZ                         0x40
#define RFLR_FRFLSB_905_MHZ                         0x00
#define RFLR_FRFMSB_906_MHZ                         0xE2
#define RFLR_FRFMID_906_MHZ                         0x80
#define RFLR_FRFLSB_906_MHZ                         0x00
#define RFLR_FRFMSB_907_MHZ                         0xE2
#define RFLR_FRFMID_907_MHZ                         0xC0
#define RFLR_FRFLSB_907_MHZ                         0x00
#define RFLR_FRFMSB_908_MHZ                         0xE3
#define RFLR_FRFMID_908_MHZ                         0x00
#define RFLR_FRFLSB_908_MHZ                         0x00
#define RFLR_FRFMSB_909_MHZ                         0xE3
#define RFLR_FRFMID_909_MHZ                         0x40
#define RFLR_FRFLSB_909_MHZ                         0x00
#define RFLR_FRFMSB_910_MHZ                         0xE3
#define RFLR_FRFMID_910_MHZ                         0x80
#define RFLR_FRFLSB_910_MHZ                         0x00
#define RFLR_FRFMSB_911_MHZ                         0xE3
#define RFLR_FRFMID_911_MHZ                         0xC0
#define RFLR_FRFLSB_911_MHZ                         0x00
#define RFLR_FRFMSB_912_MHZ                         0xE4
#define RFLR_FRFMID_912_MHZ                         0x00
#define RFLR_FRFLSB_912_MHZ                         0x00
#define RFLR_FRFMSB_913_MHZ                         0xE4
#define RFLR_FRFMID_913_MHZ                         0x40
#define RFLR_FRFLSB_913_MHZ                         0x00
#define RFLR_FRFMSB_914_MHZ                         0xE4
#define RFLR_FRFMID_914_MHZ                         0x80
#define RFLR_FRFLSB_914_MHZ                         0x00
#define RFLR_FRFMSB_915_MHZ                         0xE4  // Default
#define RFLR_FRFMID_915_MHZ                         0xC0  // Default
#define RFLR_FRFLSB_915_MHZ                         0x00  // Default
#define RFLR_FRFMSB_916_MHZ                         0xE5
#define RFLR_FRFMID_916_MHZ                         0x00
#define RFLR_FRFLSB_916_MHZ                         0x00
#define RFLR_FRFMSB_917_MHZ                         0xE5
#define RFLR_FRFMID_917_MHZ                         0x40
#define RFLR_FRFLSB_917_MHZ                         0x00
#define RFLR_FRFMSB_918_MHZ                         0xE5
#define RFLR_FRFMID_918_MHZ                         0x80
#define RFLR_FRFLSB_918_MHZ                         0x00
#define RFLR_FRFMSB_919_MHZ                         0xE5
#define RFLR_FRFMID_919_MHZ                         0xC0
#define RFLR_FRFLSB_919_MHZ                         0x00
#define RFLR_FRFMSB_920_MHZ                         0xE6
#define RFLR_FRFMID_920_MHZ                         0x00
#define RFLR_FRFLSB_920_MHZ                         0x00
#define RFLR_FRFMSB_921_MHZ                         0xE6
#define RFLR_FRFMID_921_MHZ                         0x40
#define RFLR_FRFLSB_921_MHZ                         0x00
#define RFLR_FRFMSB_922_MHZ                         0xE6
#define RFLR_FRFMID_922_MHZ                         0x80
#define RFLR_FRFLSB_922_MHZ                         0x00
#define RFLR_FRFMSB_923_MHZ                         0xE6
#define RFLR_FRFMID_923_MHZ                         0xC0
#define RFLR_FRFLSB_923_MHZ                         0x00
#define RFLR_FRFMSB_924_MHZ                         0xE7
#define RFLR_FRFMID_924_MHZ                         0x00
#define RFLR_FRFLSB_924_MHZ                         0x00
#define RFLR_FRFMSB_925_MHZ                         0xE7
#define RFLR_FRFMID_925_MHZ                         0x40
#define RFLR_FRFLSB_925_MHZ                         0x00
#define RFLR_FRFMSB_926_MHZ                         0xE7
#define RFLR_FRFMID_926_MHZ                         0x80
#define RFLR_FRFLSB_926_MHZ                         0x00
#define RFLR_FRFMSB_927_MHZ                         0xE7
#define RFLR_FRFMID_927_MHZ                         0xC0
#define RFLR_FRFLSB_927_MHZ                         0x00
#define RFLR_FRFMSB_928_MHZ                         0xE8
#define RFLR_FRFMID_928_MHZ                         0x00
#define RFLR_FRFLSB_928_MHZ                         0x00

/*!
 * RegPaConfig
 */
#define RFLR_PACONFIG_PASELECT_MASK                 0x7F 
#define RFLR_PACONFIG_PASELECT_PABOOST              0x80 
#define RFLR_PACONFIG_PASELECT_RFO                  0x00 // Default

#define RFLR_PACONFIG_OUTPUTPOWER_MASK              0xF0 
 
/*!
 * RegPaRamp
 */
#define RFLR_PARAMP_LOWPNTXPLL_MASK                 0xE0 
#define RFLR_PARAMP_LOWPNTXPLL_OFF                  0x10 // Default
#define RFLR_PARAMP_LOWPNTXPLL_ON                   0x00 

#define RFLR_PARAMP_MASK                            0xF0 
#define RFLR_PARAMP_3400_US                         0x00 
#define RFLR_PARAMP_2000_US                         0x01 
#define RFLR_PARAMP_1000_US                         0x02
#define RFLR_PARAMP_0500_US                         0x03 
#define RFLR_PARAMP_0250_US                         0x04 
#define RFLR_PARAMP_0125_US                         0x05 
#define RFLR_PARAMP_0100_US                         0x06 
#define RFLR_PARAMP_0062_US                         0x07 
#define RFLR_PARAMP_0050_US                         0x08 
#define RFLR_PARAMP_0040_US                         0x09 // Default
#define RFLR_PARAMP_0031_US                         0x0A 
#define RFLR_PARAMP_0025_US                         0x0B 
#define RFLR_PARAMP_0020_US                         0x0C 
#define RFLR_PARAMP_0015_US                         0x0D 
#define RFLR_PARAMP_0012_US                         0x0E 
#define RFLR_PARAMP_0010_US                         0x0F 

/*!
 * RegOcp
 */
#define RFLR_OCP_MASK                               0xDF 
#define RFLR_OCP_ON                                 0x20 // Default
#define RFLR_OCP_OFF                                0x00   

#define RFLR_OCP_TRIM_MASK                          0xE0
#define RFLR_OCP_TRIM_045_MA                        0x00
#define RFLR_OCP_TRIM_050_MA                        0x01   
#define RFLR_OCP_TRIM_055_MA                        0x02 
#define RFLR_OCP_TRIM_060_MA                        0x03 
#define RFLR_OCP_TRIM_065_MA                        0x04 
#define RFLR_OCP_TRIM_070_MA                        0x05 
#define RFLR_OCP_TRIM_075_MA                        0x06 
#define RFLR_OCP_TRIM_080_MA                        0x07  
#define RFLR_OCP_TRIM_085_MA                        0x08
#define RFLR_OCP_TRIM_090_MA                        0x09 
#define RFLR_OCP_TRIM_095_MA                        0x0A 
#define RFLR_OCP_TRIM_100_MA                        0x0B  // Default
#define RFLR_OCP_TRIM_105_MA                        0x0C 
#define RFLR_OCP_TRIM_110_MA                        0x0D 
#define RFLR_OCP_TRIM_115_MA                        0x0E 
#define RFLR_OCP_TRIM_120_MA                        0x0F 
#define RFLR_OCP_TRIM_130_MA                        0x10
#define RFLR_OCP_TRIM_140_MA                        0x11   
#define RFLR_OCP_TRIM_150_MA                        0x12 
#define RFLR_OCP_TRIM_160_MA                        0x13 
#define RFLR_OCP_TRIM_170_MA                        0x14 
#define RFLR_OCP_TRIM_180_MA                        0x15 
#define RFLR_OCP_TRIM_190_MA                        0x16 
#define RFLR_OCP_TRIM_200_MA                        0x17  
#define RFLR_OCP_TRIM_210_MA                        0x18
#define RFLR_OCP_TRIM_220_MA                        0x19 
#define RFLR_OCP_TRIM_230_MA                        0x1A 
#define RFLR_OCP_TRIM_240_MA                        0x1B

/*!
 * RegLna
 */
#define RFLR_LNA_GAIN_MASK                          0x1F 
#define RFLR_LNA_GAIN_G1                            0x20 // Default
#define RFLR_LNA_GAIN_G2                            0x40 
#define RFLR_LNA_GAIN_G3                            0x60 
#define RFLR_LNA_GAIN_G4                            0x80 
#define RFLR_LNA_GAIN_G5                            0xA0 
#define RFLR_LNA_GAIN_G6                            0xC0 

#define RFLR_LNA_BOOST_MASK                         0xFC 
#define RFLR_LNA_BOOST_OFF                          0x00 // Default
#define RFLR_LNA_BOOST_ON                           0x03 

/*!
 * RegFifoAddrPtr
 */
#define RFLR_FIFOADDRPTR                            0x00 // Default

/*!
 * RegFifoTxBaseAddr
 */
#define RFLR_FIFOTXBASEADDR                         0x80 // Default

/*!
 * RegFifoTxBaseAddr
 */
#define RFLR_FIFORXBASEADDR                         0x00 // Default

/*!
 * RegFifoRxCurrentAddr (Read Only)
 */

/*!
 * RegIrqFlagsMask
 */
#define RFLR_IRQFLAGS_RXTIMEOUT_MASK                0x80 
#define RFLR_IRQFLAGS_RXDONE_MASK                   0x40 
#define RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK          0x20 
#define RFLR_IRQFLAGS_VALIDHEADER_MASK              0x10 
#define RFLR_IRQFLAGS_TXDONE_MASK                   0x08 
#define RFLR_IRQFLAGS_CADDONE_MASK                  0x04 
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK       0x02 
#define RFLR_IRQFLAGS_CADDETECTED_MASK              0x01 

/*!
 * RegIrqFlags
 */
#define RFLR_IRQFLAGS_RXTIMEOUT                     0x80 
#define RFLR_IRQFLAGS_RXDONE                        0x40 
#define RFLR_IRQFLAGS_PAYLOADCRCERROR               0x20 
#define RFLR_IRQFLAGS_VALIDHEADER                   0x10 
#define RFLR_IRQFLAGS_TXDONE                        0x08 
#define RFLR_IRQFLAGS_CADDONE                       0x04 
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL            0x02 
#define RFLR_IRQFLAGS_CADDETECTED                   0x01 



/*!
 * RegFifoRxNbBytes (Read Only)    //
 */

 
 /*!
 * RegRxHeaderCntValueMsb (Read Only)    //
 */
 
 
  /*!
 * RegRxHeaderCntValueLsb (Read Only)    //
 */
 
 
/*!
 * RegRxPacketCntValueMsb (Read Only)    //
 */
 
 
 /*!
 * RegRxPacketCntValueLsb (Read Only)    //
 */
 
 
 /*!
 * RegModemStat (Read Only)    //
 */
#define RFLR_MODEMSTAT_RX_CR_MASK                   0x1F 
#define RFLR_MODEMSTAT_MODEM_STATUS_MASK            0xE0 
 
/*!
 * RegPktSnrValue (Read Only)    //
 */

 
 /*!
 * RegPktRssiValue (Read Only)    //
 */
 
 
/*!
 * RegRssiValue (Read Only)    //
 */

 /*!
 * RegHopChannel (Read Only)    //
 */
#define RFLR_HOP_CHANNEL_PAYLOAD_CRC_ON_MASK       0xBF
#define RFLR_HOP_CHANNEL_PAYLOAD_CRC_ON            0x40 
#define RFLR_HOP_CHANNEL_PAYLOAD_CRC_OFF           0x00 
 
 /*!
 * RegModemConfig1
 */
#define RFLR_MODEMCONFIG1_BW_MASK                   0x3F 
#define RFLR_MODEMCONFIG1_BW_125_KHZ                0x00 // Default
#define RFLR_MODEMCONFIG1_BW_250_KHZ                0x40 
#define RFLR_MODEMCONFIG1_BW_500_KHZ                0x80 
                                                    
#define RFLR_MODEMCONFIG1_CODINGRATE_MASK           0xC7 
#define RFLR_MODEMCONFIG1_CODINGRATE_4_5            0x08
#define RFLR_MODEMCONFIG1_CODINGRATE_4_6            0x10 // Default
#define RFLR_MODEMCONFIG1_CODINGRATE_4_7            0x18 
#define RFLR_MODEMCONFIG1_CODINGRATE_4_8            0x20 
                                                    
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK       0xFB 
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_ON         0x04 
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF        0x00 // Default
                                                    
#define RFLR_MODEMCONFIG1_RXPAYLOADCRC_MASK         0xFD 
#define RFLR_MODEMCONFIG1_RXPAYLOADCRC_ON           0x02 
#define RFLR_MODEMCONFIG1_RXPAYLOADCRC_OFF          0x00 // Default
                                                    
#define RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_MASK  0xFE 
#define RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_ON    0x01 
#define RFLR_MODEMCONFIG1_LOWDATARATEOPTIMIZE_OFF   0x00 // Default

 /*!
 * RegModemConfig2
 */
#define RFLR_MODEMCONFIG2_SF_MASK                   0x0F 
#define RFLR_MODEMCONFIG2_SF_6                      0x60 
#define RFLR_MODEMCONFIG2_SF_7                      0x70 // Default
#define RFLR_MODEMCONFIG2_SF_8                      0x80 
#define RFLR_MODEMCONFIG2_SF_9                      0x90 
#define RFLR_MODEMCONFIG2_SF_10                     0xA0 
#define RFLR_MODEMCONFIG2_SF_11                     0xB0 
#define RFLR_MODEMCONFIG2_SF_12                     0xC0 

#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_MASK     0xF7 
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_ON       0x08 
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF      0x00 

#define RFLR_MODEMCONFIG2_AGCAUTO_MASK              0xFB 
#define RFLR_MODEMCONFIG2_AGCAUTO_ON                0x04 // Default 
#define RFLR_MODEMCONFIG2_AGCAUTO_OFF               0x00 
 
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK       0xFC 
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB            0x00 // Default
                                                    
                                                    
/*!                                                 
 * RegHopChannel (Read Only)                        
 */                                                 
#define RFLR_HOPCHANNEL_PLL_LOCK_TIMEOUT_MASK       0x7F 
#define RFLR_HOPCHANNEL_PLL_LOCK_FAIL               0x80 
#define RFLR_HOPCHANNEL_PLL_LOCK_SUCCEED            0x00 // Default
                                                    
#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_MASK          0xBF
#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_ON            0x40
#define RFLR_HOPCHANNEL_PAYLOAD_CRC16_OFF           0x00 // Default

#define RFLR_HOPCHANNEL_CHANNEL_MASK                0x3F 


/*!
 * RegSymbTimeoutLsb
 */
#define RFLR_SYMBTIMEOUTLSB_SYMBTIMEOUT             0x64 // Default

/*!
 * RegPreambleLengthMsb
 */
#define RFLR_PREAMBLELENGTHMSB                      0x00 // Default

/*!
 * RegPreambleLengthLsb
 */
#define RFLR_PREAMBLELENGTHLSB                      0x08 // Default

/*!
 * RegPayloadLength
 */
#define RFLR_PAYLOADLENGTH                          0x0E // Default

/*!
 * RegPayloadMaxLength
 */
#define RFLR_PAYLOADMAXLENGTH                       0xFF // Default

/*!
 * RegHopPeriod
 */
#define RFLR_HOPPERIOD_FREQFOPPINGPERIOD            0x00 // Default


/*!
 * RegDioMapping1
 */
#define RFLR_DIOMAPPING1_DIO0_MASK                  0x3F
#define RFLR_DIOMAPPING1_DIO0_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO0_01                    0x40
#define RFLR_DIOMAPPING1_DIO0_10                    0x80
#define RFLR_DIOMAPPING1_DIO0_11                    0xC0

#define RFLR_DIOMAPPING1_DIO1_MASK                  0xCF
#define RFLR_DIOMAPPING1_DIO1_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO1_01                    0x10
#define RFLR_DIOMAPPING1_DIO1_10                    0x20
#define RFLR_DIOMAPPING1_DIO1_11                    0x30

#define RFLR_DIOMAPPING1_DIO2_MASK                  0xF3
#define RFLR_DIOMAPPING1_DIO2_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO2_01                    0x04
#define RFLR_DIOMAPPING1_DIO2_10                    0x08
#define RFLR_DIOMAPPING1_DIO2_11                    0x0C

#define RFLR_DIOMAPPING1_DIO3_MASK                  0xFC
#define RFLR_DIOMAPPING1_DIO3_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO3_01                    0x01
#define RFLR_DIOMAPPING1_DIO3_10                    0x02
#define RFLR_DIOMAPPING1_DIO3_11                    0x03

/*!
 * RegDioMapping2
 */
#define RFLR_DIOMAPPING2_DIO4_MASK                  0x3F
#define RFLR_DIOMAPPING2_DIO4_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO4_01                    0x40
#define RFLR_DIOMAPPING2_DIO4_10                    0x80
#define RFLR_DIOMAPPING2_DIO4_11                    0xC0

#define RFLR_DIOMAPPING2_DIO5_MASK                  0xCF
#define RFLR_DIOMAPPING2_DIO5_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO5_01                    0x10
#define RFLR_DIOMAPPING2_DIO5_10                    0x20
#define RFLR_DIOMAPPING2_DIO5_11                    0x30

#define RFLR_DIOMAPPING2_MAP_MASK                   0xFE
#define RFLR_DIOMAPPING2_MAP_PREAMBLEDETECT         0x01
#define RFLR_DIOMAPPING2_MAP_RSSI                   0x00  // Default

/*!
 * RegVersion (Read Only)
 */

/*!
 * RegAgcRef
 */

/*!
 * RegAgcThresh1
 */

/*!
 * RegAgcThresh2
 */

/*!
 * RegAgcThresh3
 */
 
/*!
 * RegFifoRxByteAddr (Read Only)
 */
 
/*!
 * RegPllHop
 */
#define RFLR_PLLHOP_FASTHOP_MASK                    0x7F
#define RFLR_PLLHOP_FASTHOP_ON                      0x80
#define RFLR_PLLHOP_FASTHOP_OFF                     0x00 // Default

/*!
 * RegTcxo
 */
#define RFLR_TCXO_TCXOINPUT_MASK                    0xEF
#define RFLR_TCXO_TCXOINPUT_ON                      0x10
#define RFLR_TCXO_TCXOINPUT_OFF                     0x00  // Default

/*!
 * RegPaDac
 */
#define RFLR_PADAC_20DBM_MASK                       0xF8
#define RFLR_PADAC_20DBM_ON                         0x07
#define RFLR_PADAC_20DBM_OFF                        0x04  // Default

/*!
 * RegPll
 */
#define RFLR_PLL_BANDWIDTH_MASK                     0x3F
#define RFLR_PLL_BANDWIDTH_75                       0x00
#define RFLR_PLL_BANDWIDTH_150                      0x40
#define RFLR_PLL_BANDWIDTH_225                      0x80
#define RFLR_PLL_BANDWIDTH_300                      0xC0  // Default

/*!
 * RegPllLowPn
 */
#define RFLR_PLLLOWPN_BANDWIDTH_MASK                0x3F
#define RFLR_PLLLOWPN_BANDWIDTH_75                  0x00
#define RFLR_PLLLOWPN_BANDWIDTH_150                 0x40
#define RFLR_PLLLOWPN_BANDWIDTH_225                 0x80
#define RFLR_PLLLOWPN_BANDWIDTH_300                 0xC0  // Default

/*!
 * Define XRange hardware version
 */
#define XR_VER_0_1      0
#define XR_VER_1_0      1

#define XRANGE_HW_VER   XR_VER_1_0

/*!
 * RF process function return codes
 */
typedef enum
{
    RF_IDLE,
    RF_BUSY,
    RF_RX_DONE,
    RF_RX_TIMEOUT,
    RF_TX_DONE,
    RF_TX_TIMEOUT,
    RF_LEN_ERROR,
    RF_CHANNEL_EMPTY,
    RF_CHANNEL_ACTIVITY_DETECTED,
}tRFProcessReturnCodes;

/*!
 * Radio driver structure defining the different function pointers
 */
typedef struct sRadioDriver
{
    void ( *Init )( void );
    void ( *Reset )( void );
    void ( *StartRx )( void );
    void ( *GetRxPacket )( void *buffer, uint16_t *size );
    void ( *SetTxPacket )( const void *buffer, uint16_t size );
    uint32_t ( *Process )( void );
}tRadioDriver;

/*!
 * RegFormerTemp
 */

typedef struct sSX1272LR
{
    uint8_t RegFifo;                                // 0x00 
    // Common settings
    uint8_t RegOpMode;                              // 0x01 
    uint8_t RegRes02;                               // 0x02 
    uint8_t RegRes03;                               // 0x03 
    uint8_t RegRes04;                               // 0x04 
    uint8_t RegRes05;                               // 0x05 
    uint8_t RegFrfMsb;                              // 0x06 
    uint8_t RegFrfMid;                              // 0x07 
    uint8_t RegFrfLsb;                              // 0x08 
    // Tx settings
    uint8_t RegPaConfig;                            // 0x09 
    uint8_t RegPaRamp;                              // 0x0A 
    uint8_t RegOcp;                                 // 0x0B 
    // Rx settings
    uint8_t RegLna;                                 // 0x0C 
    // LoRa registers
    uint8_t RegFifoAddrPtr;                         // 0x0D 
    uint8_t RegFifoTxBaseAddr;                      // 0x0E 
    uint8_t RegFifoRxBaseAddr;                      // 0x0F 
    uint8_t RegFifoRxCurrentAddr;                   // 0x10 
    uint8_t RegIrqFlagsMask;                        // 0x11 
    uint8_t RegIrqFlags;                            // 0x12 
    uint8_t RegNbRxBytes;                           // 0x13 
    uint8_t RegRxHeaderCntValueMsb;                 // 0x14 
    uint8_t RegRxHeaderCntValueLsb;                 // 0x15 
    uint8_t RegRxPacketCntValueMsb;                 // 0x16 
    uint8_t RegRxPacketCntValueLsb;                 // 0x17 
    uint8_t RegModemStat;                           // 0x18 
    uint8_t RegPktSnrValue;                         // 0x19 
    uint8_t RegPktRssiValue;                        // 0x1A 
    uint8_t RegRssiValue;                           // 0x1B 
    uint8_t RegHopChannel;                          // 0x1C 
    uint8_t RegModemConfig1;                        // 0x1D 
    uint8_t RegModemConfig2;                        // 0x1E 
    uint8_t RegSymbTimeoutLsb;                      // 0x1F 
    uint8_t RegPreambleMsb;                         // 0x20 
    uint8_t RegPreambleLsb;                         // 0x21 
    uint8_t RegPayloadLength;                       // 0x22 
    uint8_t RegMaxPayloadLength;                    // 0x23 
    uint8_t RegHopPeriod;                           // 0x24 
    uint8_t RegFifoRxByteAddr;                      // 0x25     
    uint8_t RegTestReserved26[0x27 - 0x26];         // 0x26-0x27 
    uint8_t RegFeiMsb;                              // 0x28
    uint8_t RegFeiMib;                              // 0x29 
    uint8_t RegFeiLsb;                              // 0x2A
    uint8_t RegTestReserved2B[0x30 - 0x2B];         // 0x2B-0x30 
    uint8_t RegDetectOptimize;                  // 0x31    
    uint8_t RegTestReserved32;                      // 0x32   
    uint8_t RegInvertIQ;                            // 0x33 
    uint8_t RegTestReserved34[0x36 - 0x34];         // 0x34-0x36 
    uint8_t RegDetectionThreshold;                  // 0x37
    uint8_t RegTestReserved38[0x3F - 0x38];         // 0x38-0x3F    
    // I/O settings                
    uint8_t RegDioMapping1;                         // 0x40 
    uint8_t RegDioMapping2;                         // 0x41 
    // Version
    uint8_t RegVersion;                             // 0x42
    // Additional settings
    uint8_t RegAgcRef;                              // 0x43
    uint8_t RegAgcThresh1;                          // 0x44
    uint8_t RegAgcThresh2;                          // 0x45
    uint8_t RegAgcThresh3;                          // 0x46
    // Test
    uint8_t RegTestReserved47[0x4B - 0x47];         // 0x47-0x4A
    // Additional settings
    uint8_t RegPllHop;                              // 0x4B
    // Test
    uint8_t RegTestReserved4C[0x58-0x4C];           // 0x4C-0x57
    // Additional settings
    uint8_t RegTcxo;                                // 0x58
    // Test
    uint8_t RegTestReserved59;                      // 0x59
    // Additional settings
    uint8_t RegPaDac;                               // 0x5A
    // Test
    uint8_t RegTestReserved5B;                      // 0x5B
    // Additional settings
    uint8_t RegPll;                                 // 0x5C
    // Test
    uint8_t RegTestReserved5D;                      // 0x5D
    // Additional settings
    uint8_t RegPllLowPn;                            // 0x5E
    // Test
    uint8_t RegTestReserved5F[0x6C - 0x5F];         // 0x5F-0x6B
    // Additional settings
    uint8_t RegFormerTemp;                          // 0x6C
    // Test
    uint8_t RegTestReserved6D[0x71 - 0x6D];         // 0x6D-0x70
}tSX1272LR;

extern tSX1272LR* SX1272LR;

// RXTX pin control see errata note
#define RXTX( txEnable )                            SX1272WriteRxTx( txEnable );

#define TICK_RATE_MS( ms )                          ( ms )

typedef enum
{
    RADIO_RESET_OFF,
    RADIO_RESET_ON,
}tRadioResetState;

#ifdef __cplusplus
 extern "C" {
#endif 

/*!
 * \brief Initializes the RadioDriver structure with specific radio
 *        functions.
 *
 * \retval radioDriver Pointer to the radio driver variable
 */
tRadioDriver* RadioDriverInit( void );

/*!
 * \brief SX1272 registers array
 */
extern uint8_t SX1272Regs[0x70];

/*!
 * \brief Enables LoRa modem
 *
 * \param [IN] opMode New operating mode
 */
void SX1272SetLoRaOn( bool enable );

/*!
 * \brief Gets the LoRa modem state
 *
 * \retval LoraOn Current LoRa modem mode
 */
bool SX1272GetLoRaOn( void );

/*!
 * \brief Initializes the SX1272
 */
void SX1272Init( void );

/*!
 * \brief Resets the SX1272
 */
void SX1272Reset( void );

/*!
 * \brief Sets the SX1272 operating mode
 *
 * \param [IN] opMode New operating mode
 */
void SX1272SetOpMode( uint8_t opMode );

/*!
 * \brief Gets the SX1272 operating mode
 *
 * \retval opMode Current operating mode
 */
uint8_t SX1272GetOpMode( void );

/*!
 * \brief Reads the current Rx gain setting
 *
 * \retval rxGain Current gain setting
 */
uint8_t SX1272ReadRxGain( void );

/*!
 * \brief Trigs and reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
double SX1272ReadRssi( void );

/*!
 * \brief Gets the Rx gain value measured while receiving the packet
 *
 * \retval rxGainValue Current Rx gain value
 */
uint8_t SX1272GetPacketRxGain( void );

/*!
 * \brief Gets the SNR value measured while receiving the packet
 *
 * \retval snrValue Current SNR value in [dB]
 */
int8_t SX1272GetPacketSnr( void );

/*!
 * \brief Gets the RSSI value measured while receiving the packet
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
double SX1272GetPacketRssi( void );

/*!
 * \brief Sets the new state of the RF state machine
 *
 * \param [IN]: state New RF state machine state
 */
void SX1272SetRFState( uint8_t state );

/*!
 * \brief Initializes the radio interface I/Os
 */
void SX1272InitIo( void );

/*!
 * \brief Set the radio reset pin state
 * 
 * \param state New reset pin state
 */
void SX1272SetReset( uint8_t state );

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void SX1272Write( uint8_t addr, uint8_t data );

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [OUT]: data Register value
 */
void SX1272Read( uint8_t addr, uint8_t *data );

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void SX1272WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void SX1272ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );

/*!
 * \brief Writes the buffer contents to the radio FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void SX1272WriteFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Reads the contents of the radio FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void SX1272ReadFifo( uint8_t *buffer, uint8_t size );

/*!
 * \brief Gets the SX1272 DIO0 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
inline uint8_t SX1272ReadDio0( void );

/*!
 * \brief Gets the SX1272 DIO1 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
inline uint8_t SX1272ReadDio1( void );

/*!
 * \brief Gets the SX1272 DIO2 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
inline uint8_t SX1272ReadDio2( void );

/*!
 * \brief Gets the SX1272 DIO3 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
inline uint8_t SX1272ReadDio3( void );

/*!
 * \brief Gets the SX1272 DIO4 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
inline uint8_t SX1272ReadDio4( void );

/*!
 * \brief Gets the SX1272 DIO5 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
inline uint8_t SX1272ReadDio5( void );

/*!
 * \brief Writes the external RxTx pin value
 *
 * \remark see errata note
 *
 * \param [IN] txEnable [1: Tx, 0: Rx]
 */
inline void SX1272WriteRxTx( uint8_t txEnable );

/*!
 * \brief Initializes the SX1272
 */
void SX1272LoRaInit( void );

/*!
 * \brief Sets the SX1272 to datasheet default values
 */
void SX1272LoRaSetDefaults( void );

/*!
 * \brief Enables/Disables the LoRa modem
 *
 * \param [IN]: enable [true, false]
 */
void SX1272LoRaSetLoRaOn( bool enable );

/*!
 * \brief Sets the SX1272 operating mode
 *
 * \param [IN] opMode New operating mode
 */
void SX1272LoRaSetOpMode( uint8_t opMode );

/*!
 * \brief Gets the SX1272 operating mode
 *
 * \retval opMode Current operating mode
 */
uint8_t SX1272LoRaGetOpMode( void );

/*!
 * \brief Reads the current Rx gain setting
 *
 * \retval rxGain Current gain setting
 */
uint8_t SX1272LoRaReadRxGain( void );

/*!
 * \brief Trigs and reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
double SX1272LoRaReadRssi( void );

/*!
 * \brief Sets the radio in Rx mode. Waiting for a packet
 */
void SX1272LoRaStartRx( void );

/*!
 * \brief Gets a copy of the current received buffer
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void SX1272LoRaGetRxPacket( void *buffer, uint16_t *size );

/*!
 * \brief Sets a copy of the buffer to be transmitted
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void SX1272LoRaSetTxPacket( const void *buffer, uint16_t size );

/*!
 * \brief Gets the current RFState
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint8_t SX1272LoRaGetRFState( void );

/*!
 * \brief Sets the new state of the RF state machine
 *
 * \param [IN]: state New RF state machine state
 */
void SX1272LoRaSetRFState( uint8_t state );

/*!
 * \brief Process the LoRa modem Rx and Tx state machines depending on the
 *        SX1272 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint32_t SX1272LoRaProcess( void );

/*!
 * \brief Writes the new RF frequency value
 *
 * \param [IN] freq New RF frequency value in [Hz]
 */
void SX1272LoRaSetRFFrequency( uint32_t freq );

/*!
 * \brief Reads the current RF frequency value
 *
 * \retval freq Current RF frequency value in [Hz]
 */
uint32_t SX1272LoRaGetRFFrequency( void );

/*!
 * \brief Writes the new RF output power value
 *
 * \param [IN] power New output power value in [dBm]
 */
void SX1272LoRaSetRFPower( int8_t power );

/*!
 * \brief Reads the current RF output power value
 *
 * \retval power Current output power value in [dBm]
 */
int8_t SX1272LoRaGetRFPower( void );

/*!
 * \brief Writes the new Signal Bandwidth value
 *
 * \remark This function sets the IF frequency according to the datasheet
 *
 * \param [IN] factor New Signal Bandwidth value [0: 125 kHz, 1: 250 kHz, 2: 500 kHz]
 */
void SX1272LoRaSetSignalBandwidth( uint8_t bw );

/*!
 * \brief Reads the current Signal Bandwidth value
 *
 * \retval factor Current Signal Bandwidth value [0: 125 kHz, 1: 250 kHz, 2: 500 kHz] 
 */
uint8_t SX1272LoRaGetSignalBandwidth( void );

/*!
 * \brief Writes the new Spreading Factor value
 *
 * \param [IN] factor New Spreading Factor value [7, 8, 9, 10, 11, 12]
 */
void SX1272LoRaSetSpreadingFactor( uint8_t factor );

/*!
 * \brief Reads the current Spreading Factor value
 *
 * \retval factor Current Spreading Factor value [7, 8, 9, 10, 11, 12] 
 */
uint8_t SX1272LoRaGetSpreadingFactor( void );

/*!
 * \brief Writes the new Error Coding value
 *
 * \param [IN] value New Error Coding value [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 */
void SX1272LoRaSetErrorCoding( uint8_t value );

/*!
 * \brief Reads the current Error Coding value
 *
 * \retval value Current Error Coding value [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 */
uint8_t SX1272LoRaGetErrorCoding( void );

/*!
 * \brief Enables/Disables the packet CRC generation
 *
 * \param [IN] enaable [true, false]
 */
void SX1272LoRaSetPacketCrcOn( bool enable );

/*!
 * \brief Reads the current packet CRC generation status
 *
 * \retval enable [true, false]
 */
bool SX1272LoRaGetPacketCrcOn( void );

/*!
 * \brief Enables/Disables the Implicit Header mode in LoRa
 *
 * \param [IN] enable [true, false]
 */
void SX1272LoRaSetImplicitHeaderOn( bool enable );

/*!
 * \brief Check if implicit header mode in LoRa in enabled or disabled
 *
 * \retval enable [true, false]
 */
bool SX1272LoRaGetImplicitHeaderOn( void );

/*!
 * \brief Enables/Disables Rx single instead of Rx continuous
 *
 * \param [IN] enable [true, false]
 */
void SX1272LoRaSetRxSingleOn( bool enable );

/*!
 * \brief Check if LoRa is in Rx Single mode
 *
 * \retval enable [true, false]
 */
bool SX1272LoRaGetRxSingleOn( void );

/*!
 * \brief Enables/Disables the frequency hopping 
 *
 * \param [IN] enable [true, false]
 */
void SX1272LoRaSetFreqHopOn( bool enable );

/*!
 * \brief Get the frequency hopping status 
 *
 * \param [IN] enable [true, false]
 */
bool SX1272LoRaGetFreqHopOn( void );

/*!
 * \brief Set symbol period between frequency hops
 *
 * \param [IN] value
 */
void SX1272LoRaSetHopPeriod( uint8_t value );

/*!
 * \brief Get symbol period between frequency hops
 *
 * \retval value symbol period between frequency hops
 */
uint8_t SX1272LoRaGetHopPeriod( void );

/*!
 * \brief Set timeout Tx packet (based on MCU timer, timeout between Tx Mode entry Tx Done IRQ)
 *
 * \param [IN] value timeout (ms)
 */
void SX1272LoRaSetTxPacketTimeout( uint32_t value );

/*!
 * \brief Get timeout between Tx packet (based on MCU timer, timeout between Tx Mode entry Tx Done IRQ)
 *
 * \retval value timeout (ms)
 */
uint32_t SX1272LoRaGetTxPacketTimeout( void );

/*!
 * \brief Set timeout Rx packet (based on MCU timer, timeout between Rx Mode entry and Rx Done IRQ)
 *
 * \param [IN] value timeout (ms)
 */
void SX1272LoRaSetRxPacketTimeout( uint32_t value );

/*!
 * \brief Get timeout Rx packet (based on MCU timer, timeout between Rx Mode entry and Rx Done IRQ)
 *
 * \retval value timeout (ms)
 */
uint32_t SX1272LoRaGetRxPacketTimeout( void );

/*!
 * \brief Set payload length
 *
 * \param [IN] value payload length
 */
void SX1272LoRaSetPayloadLength( uint8_t value );

/*!
 * \brief Get payload length
 *
 * \retval value payload length
 */
uint8_t SX1272LoRaGetPayloadLength( void );

/*!
 * \brief Enables/Disables the 20 dBm PA
 *
 * \param [IN] enable [true, false]
 */
void SX1272LoRaSetPa20dBm( bool enale );

/*!
 * \brief Gets the current 20 dBm PA status
 *
 * \retval enable [true, false]
 */
bool SX1272LoRaGetPa20dBm( void );

/*!
 * \brief Set the RF Output pin 
 *
 * \param [IN] RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 */
void SX1272LoRaSetPAOutput( uint8_t outputPin );

/*!
 * \brief Gets the used RF Ouptu pin
 *
 * \retval RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 */
uint8_t SX1272LoRaGetPAOutput( void );
/*!
 * \brief Writes the new PA rise/fall time of ramp up/down value
 *
 * \param [IN] value New PaRamp value
 */
void SX1272LoRaSetPaRamp( uint8_t value );

/*!
 * \brief Reads the current PA rise/fall time of ramp up/down value
 *
 * \retval freq Current PaRamp value
 */
uint8_t SX1272LoRaGetPaRamp( void );

/*!
 * \brief Set Symbol Timeout based on symbol length
 *
 * \param [IN] value number of symbol
 */
void SX1272LoRaSetSymbTimeout( uint16_t value );

/*!
 * \brief  Get Symbol Timeout based on symbol length
 *
 * \retval value number of symbol
 */
uint16_t SX1272LoRaGetSymbTimeout( void );

/*!
 * \brief  Configure the device to optimize low datarate transfers
 *
 * \param [IN] enable Enables/Disables the low datarate optimization
 */
void SX1272LoRaSetLowDatarateOptimize( bool enable );

/*!
 * \brief  Get the status of optimize low datarate transfers
 *
 * \retval LowDatarateOptimize enable or disable
 */
bool SX1272LoRaGetLowDatarateOptimize( void );

/*!
 * \brief Get the preamble length
 *
 * \retval value preamble length
 */
uint16_t SX1272LoRaGetPreambleLength( void );

/*!
 * \brief Set the preamble length
 *
 * \param [IN] value preamble length
 */
void SX1272LoRaSetPreambleLength( uint16_t value );

/*!
 * \brief Set the number or rolling preamble symbol needed for detection
 *
 * \param [IN] value number of preamble symbol
 */
void SX1272LoRaSetNbTrigPeaks( uint8_t value );

/*!
 * \brief Get the number or rolling preamble symbol needed for detection
 *
 * \retval value number of preamble symbol
 */
uint8_t SX1272LoRaGetNbTrigPeaks( void );

/*!
 * Initializes board peripherals
 */
void BoardInit( void );

void SpiInit( void );

uint8_t SpiInOut( uint8_t outData );

void RTC_Config(void);

#ifdef __cplusplus
}
#endif

#endif //__RADINO32_SX1272_H__
