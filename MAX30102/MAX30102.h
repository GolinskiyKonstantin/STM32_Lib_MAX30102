
#ifndef _MAX30102_H_
#define _MAX30102_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include "algorithm.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MAX30102_ADDRESS 0xAE	//( 0x57<<1 )


#define MAX30102_MEASUREMENT_SECONDS 		5		// время измерения в секундах ( по умолчанию 5 с )
#define MAX30102_SAMPLES_PER_SECOND			100 	// выборок в секунду ( по усолчанию 100 ) может иметь следующие значения 50, 100, 200, 400, 800, 100, 1600, 3200 sample rating
#define MAX30102_FIFO_ALMOST_FULL_SAMPLES 	17		// кол-во измерений для вызова прерывания ( от 17 до максимум 32 сэмпла по умолчанию 17 )

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
//	Status enum
//
typedef enum{
	MAX30102_ERROR,	
	MAX30102_OK	
} MAX30102_STATUS;


typedef enum{
	MAX30102_STATE_BEGIN,
	MAX30102_STATE_CALIBRATE,
	MAX30102_STATE_CALCULATE_HR,
	MAX30102_STATE_COLLECT_NEXT_PORTION
}MAX30102_STATE;

	
#define MAX30102_BUFFER_LENGTH	((MAX30102_MEASUREMENT_SECONDS+1)*MAX30102_SAMPLES_PER_SECOND)

//
//	Calibration
//

//-----------------------------------------------------------------
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch
  
#define MAX30102_IR_LED_CURRENT_LOW				0x02
#define MAX30102_RED_LED_CURRENT_LOW			0x02
#define MAX30102_GREEN_LED_CURRENT_LOW			0x02

#define MAX30102_IR_LED_CURRENT_HIGH			0x24
#define MAX30102_RED_LED_CURRENT_HIGH			0x24
#define MAX30102_GREEN_LED_CURRENT_HIGH			0x24

//------------------------------------------------------------------

#define MAX30102_IR_VALUE_FINGER_ON_SENSOR 		1600
#define MAX30102_IR_VALUE_FINGER_OUT_SENSOR 	50000
//
//	Register addresses
//
#define REG_INTR_STATUS_1 		0x00	// состояние прерывания 1
#define REG_INTR_STATUS_2 		0x01	// состояние прерывания 2 ( нужен только для температуры )
#define REG_INTR_ENABLE_1 		0x02	// разрешение прерывание 1
#define REG_INTR_ENABLE_2 		0x03	// разрешение прерывание 2 ( нужен только для температуры )
#define REG_FIFO_WR_PTR 		0x04
#define REG_OVF_COUNTER 		0x05
#define REG_FIFO_RD_PTR 		0x06
#define REG_FIFO_DATA 			0x07
#define REG_FIFO_CONFIG 		0x08
#define REG_MODE_CONFIG 		0x09
#define REG_SPO2_CONFIG 		0x0A
#define REG_LED1_PA 			0x0C	// IR
#define REG_LED2_PA 			0x0D	// RED
#define REG_LED3_PA 			0x0E	// GREEN
#define REG_PILOT_PA 			0x10
#define REG_MULTI_LED_CTRL1 	0x11	// слот 1 ( 0,1,2 байт ) и слот 2 ( 5,6,7 байт )
#define REG_MULTI_LED_CTRL2 	0x12	// слот 3 ( 0,1,2 байт ) и слот 4 ( 5,6,7 байт )
#define REG_TEMP_INTR 			0x1F
#define REG_TEMP_FRAC 			0x20
#define REG_TEMP_CONFIG 		0x21
#define REG_PROX_INT_THRESH 	0x30
#define REG_REV_ID 				0xFE
#define REG_PART_ID 			0xFF

//
//	Interrupt Status 1 (0x00)
//	Interrupt Status 2 (0x01)
//	Interrupt Enable 1 (0x02)
//	Interrupt Enable 2 (0x03)
//
#define	INT_A_FULL_BIT			7	// В режимах SpO2 и HR это прерывание срабатывает, когда у указателя записи FIFO остается определенное количество свободных мест.
#define	INT_PPG_RDY_BIT			6	// В режимах SpO2 и HR это прерывание срабатывает при появлении новой выборки в FIFO данных.
#define	INT_ALC_OVF_BIT			5	// Это прерывание срабатывает, когда функция подавления окружающего света фотодиода SpO2 / HR достигла своего максимального предела, и, следовательно, окружающий свет влияет на выход АЦП
#define	INT_DIE_TEMP_RDY_BIT	1	// Когда внутреннее преобразование температуры матрицы закончено, это прерывание запускается, чтобы процессор мог прочитать регистры данных температуры.
#define	INT_PWR_RDY_BIT			0	// При включении питания или после состояния отключения питания, когда напряжение питания VDD переходит от напряжения блокировки пониженного напряжения (UVLO) к напряжению,
									// превышающему напряжение UVLO, срабатывает прерывание готовности к питанию, сигнализирующее о том, что модуль включен и готов к работе. собирать данные.

//
//	FIFO Configuration (0x08)
//
#define FIFO_CONF_SMP_AVE_BIT 			7
#define FIFO_CONF_SMP_AVE_LENGHT 		3
#define FIFO_CONF_FIFO_ROLLOVER_EN_BIT 	4
#define FIFO_CONF_FIFO_A_FULL_BIT 		3
#define FIFO_CONF_FIFO_A_FULL_LENGHT 	4

// выборочное усреднение (SMP_AVE)
#define FIFO_SMP_AVE_1		0	// без усреднения
#define FIFO_SMP_AVE_2		1
#define FIFO_SMP_AVE_4		2
#define FIFO_SMP_AVE_8		3
#define FIFO_SMP_AVE_16		4
#define FIFO_SMP_AVE_32		5

//
//	Mode Configuration (0x09)
//
#define MODE_SHDN_BIT		7
#define MODE_RESET_BIT		6
#define MODE_MODE_BIT		2
#define MODE_MODE_LENGTH	3

// режимы
#define MODE_HEART_RATE_MODE	2	// Режим сердечного ритма Только красный
#define MODE_SPO2_MODE			3	// Режим SpO2 Красный и ИК
#define MODE_MULTI_LED_MODE		7	// Мульти-светодиодный режим Красный и ИК

//
//	SpO2 Configuration (0x0A)
//
#define SPO2_CONF_ADC_RGE_BIT		6
#define SPO2_CONF_ADC_RGE_LENGTH	2
#define SPO2_CONF_SR_BIT			4
#define SPO2_CONF_SR_LENGTH			3
#define SPO2_CONF_LED_PW_BIT		1
#define SPO2_CONF_LED_PW_LENGTH		2

// Этот регистр устанавливает полный диапазон АЦП датчика SpO2
#define	SPO2_ADC_RGE_2048	0
#define	SPO2_ADC_RGE_4096	1
#define	SPO2_ADC_RGE_8192	2
#define	SPO2_ADC_RGE_16384	3

// SpO 2 Контроль частоты дискретизации
#define	SPO2_SAMPLE_RATE_50		0
#define	SPO2_SAMPLE_RATE_100	1
#define	SPO2_SAMPLE_RATE_200	2
#define	SPO2_SAMPLE_RATE_400	3
#define	SPO2_SAMPLE_RATE_800	4
#define	SPO2_SAMPLE_RATE_1000	5
#define	SPO2_SAMPLE_RATE_1600	6
#define	SPO2_SAMPLE_RATE_3200	7

#if(MAX30102_SAMPLES_PER_SECOND == 50)
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_50
#elif((MAX30102_SAMPLES_PER_SECOND == 100))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_100
#elif((MAX30102_SAMPLES_PER_SECOND == 200))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_200
#elif((MAX30102_SAMPLES_PER_SECOND == 400))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_400
#elif((MAX30102_SAMPLES_PER_SECOND == 800))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_800
#elif((MAX30102_SAMPLES_PER_SECOND == 1000))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_1000
#elif((MAX30102_SAMPLES_PER_SECOND == 1600))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_1600
#elif((MAX30102_SAMPLES_PER_SECOND == 3200))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_3200
#else
#error "Wrong Sample Rate value. Use 50, 100, 200, 400, 800, 1000, 1600 or 3200."
#endif

// управление шириной импульса светодиода и разрешение АЦП
#define	SPO2_PULSE_WIDTH_69			0
#define	SPO2_PULSE_WIDTH_118		1
#define	SPO2_PULSE_WIDTH_215		2
#define	SPO2_PULSE_WIDTH_411		3


// Slots
#define	SLOT_RED_LED 		0x01
#define	SLOT_IR_LED			0x02
#define	SLOT_GREEN_LED		0x03

//
//	Functions
//
MAX30102_STATUS Max30102_Init(I2C_HandleTypeDef *i2c);
/*
	ВНИМАНИЕ!!! часто в модулях Max30102 и Max30105 перепутаны каналы светодиодов ir_led  и red_led поэтому если показания данных не корректное
	то тогда в функции Max30102_ReadFifo в файле MAX30102.c и MAX30102.h меняем местами входящие параметры. 
	Ниже приведен пример:

	MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_red_led, volatile uint32_t *pun_ir_led){}	// вариант 1
	MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_ir_led, volatile uint32_t *pun_red_led){}	// вариант 2
*/
MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_ir_led, volatile uint32_t *pun_red_led);
MAX30102_STATUS Max30102_WriteReg(uint8_t uch_addr, uint8_t uch_data);
MAX30102_STATUS Max30102_ReadReg(uint8_t uch_addr, uint8_t *puch_data);
//
//	Interrupts
//
MAX30102_STATUS Max30102_ReadInterruptStatus(uint8_t *Status);
MAX30102_STATUS Max30102_SetIntAlmostFullEnabled(uint8_t Enable);
MAX30102_STATUS Max30102_SetIntFifoDataReadyEnabled(uint8_t Enable);
MAX30102_STATUS Max30102_SetIntAmbientLightCancelationOvfEnabled(uint8_t Enable);

MAX30102_STATUS Max30102_SetIntInternalTemperatureReadyEnabled(uint8_t Enable);


void Max30102_InterruptCallback(void);
//
//	FIFO Configuration
//
MAX30102_STATUS Max30102_FifoWritePointer(uint8_t Address);
MAX30102_STATUS Max30102_FifoOverflowCounter(uint8_t Address);
MAX30102_STATUS Max30102_FifoReadPointer(uint8_t Address);
MAX30102_STATUS Max30102_FifoSampleAveraging(uint8_t Value);
MAX30102_STATUS Max30102_FifoRolloverEnable(uint8_t Enable);
MAX30102_STATUS Max30102_FifoAlmostFullValue(uint8_t Value); // 17-32 samples ready in FIFO
//
//	Mode Configuration
//
MAX30102_STATUS Max30102_ShutdownMode(uint8_t Enable);
MAX30102_STATUS Max30102_Reset(void);
MAX30102_STATUS Max30102_SetMode(uint8_t Mode);
MAX30102_STATUS Max30102_Slot(uint8_t slotNumber, uint8_t Value);
//
//	SpO2 Configuration
//
MAX30102_STATUS Max30102_SpO2AdcRange(uint8_t Value);
MAX30102_STATUS Max30102_SpO2SampleRate(uint8_t Value);
MAX30102_STATUS Max30102_SpO2LedPulseWidth(uint8_t Value);
//
//	LEDs Pulse Amplitute Configuration
//
MAX30102_STATUS Max30102_Led1PulseAmplitude(uint8_t Value);
MAX30102_STATUS Max30102_Led2PulseAmplitude(uint8_t Value);
MAX30102_STATUS Max30102_Led3PulseAmplitude(uint8_t Value);
//
//	Usage functions
//
MAX30102_STATUS Max30102_IsFingerOnSensor(void);

void Max30102_Task(void);
int32_t Max30102_GetHeartRate(void);
int32_t Max30102_GetSpO2Value(void);
float Max30102_ReadTemperature(void);

#ifdef __cplusplus
}
#endif

#endif /* _MAX30102_H_ */
