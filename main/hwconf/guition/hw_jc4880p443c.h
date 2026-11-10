#ifndef MAIN_HWCONF_GUITION_HW_JC4880P443C_H_
#define MAIN_HWCONF_GUITION_HW_JC4880P443C_H_

#define HW_NAME					"JC4880P443C"
#define HW_TARGET				"esp32p4_nrw32n16c6"
#define HW_USE_TINYSG
#define HW_NO_UART

#define HW_INIT_HOOK()			hw_init()

// Guition JC4880P443_V1.0 schematic, JC-ESP32P4-M3 module.
// https://github.com/wegi1/ESP32P4-JC4880P443C-I-W/tree/main/5-Schematic

// Display (ST7701S, dedicated MIPI-DSI lanes)
#define DISP_RESET				5
#define DISP_BL					23

// I2C (shared by GT911, ES8311, camera and external connectors)
#define I2C_SDA					7
#define I2C_SCL					8

// Touch (GT911)
#define TOUCH_RESET				21
#define TOUCH_INT				20

// Audio (ES8311, I2S directions relative to ESP32-P4)
#define AUDIO_MCLK				13
#define AUDIO_BCLK				12
#define AUDIO_WS				10
#define AUDIO_DOUT				9
#define AUDIO_DIN				48
#define AUDIO_AMP_EN			11

// microSD (4-bit SDMMC; TF_VCC uses LDO4, GPIO45 is not connected)
#define SDMMC_CLK				43
#define SDMMC_CMD				44
#define SDMMC_D0				39
#define SDMMC_D1				40
#define SDMMC_D2				41
#define SDMMC_D3				42

// ESP32-C6 (internal ESP-Hosted SDIO bus)
#define C6_SDIO_CLK				18
#define C6_SDIO_CMD				19
#define C6_SDIO_D0				14
#define C6_SDIO_D1				15
#define C6_SDIO_D2				16
#define C6_SDIO_D3				17
#define C6_RESET				54

// UART0 connector (automatic firmware UART remains disabled by HW_NO_UART)
#define UART_NUM				0
#define UART_BAUDRATE			115200
#define UART_TX					37
#define UART_RX					38

// RS485 (UART1, automatic direction control)
#define RS485_TX				26
#define RS485_RX				27

// Boot button and battery divider
#define PIN_BOOT				35
#define BATTERY_SENSE_GPIO		53

// Expansion header JP1 (P4 GPIOs, indexed by connector pin number)
#define EXP_IO_7				52
#define EXP_IO_8				33
#define EXP_IO_9				51
#define EXP_IO_10				31
#define EXP_IO_11				50
#define EXP_IO_12				30
#define EXP_IO_13				49
#define EXP_IO_14				29
#define EXP_IO_15				35
#define EXP_IO_17				34
#define EXP_IO_19				32
#define EXP_IO_21				28
#define EXP_IO_23				I2C_SDA
#define EXP_IO_25				I2C_SCL
// Pins 1/3: 3V3; 2/4: 5V; 5/6/16: GND; 18: ESP_3V3.
// Pins 20/22/24/26: C6_U0RXD / C6_U0TXD / C6_IO9 / C6_CHIP_PU.

// ADC
// ADC1 GPIO16..19: C6 SDIO; GPIO20/21: touch; GPIO23: backlight.
// All firmware ADC inputs alias the remaining ADC1 channel, GPIO22.
#define HW_ADC_CH0				ADC_CHANNEL_6
#define HW_ADC_CH1				ADC_CHANNEL_6
#define HW_ADC_CH2				ADC_CHANNEL_6
#define HW_ADC_CH3				ADC_CHANNEL_6
#define HW_ADC_CH4				ADC_CHANNEL_6

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_GUITION_HW_JC4880P443C_H_ */
