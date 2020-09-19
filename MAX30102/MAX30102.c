
#include "MAX30102.h"


#define I2C_TIMEOUT	HAL_MAX_DELAY

I2C_HandleTypeDef *i2c_max30102;

volatile uint32_t IrBuffer[MAX30102_BUFFER_LENGTH]; //IR LED sensor data
volatile uint32_t RedBuffer[MAX30102_BUFFER_LENGTH];    //Red LED sensor data
volatile uint32_t BufferHead;
volatile uint32_t BufferTail;
volatile uint32_t CollectedSamples;
volatile uint8_t IsFingerOnScreen;
int32_t Sp02Value;
int8_t Sp02IsValid;
int32_t HeartRate;
int8_t IsHrValid;


MAX30102_STATE StateMachine;


MAX30102_STATUS Max30102_WriteReg(uint8_t uch_addr, uint8_t uch_data)
{
	if(HAL_I2C_Mem_Write(i2c_max30102, MAX30102_ADDRESS, uch_addr, 1, &uch_data, 1, I2C_TIMEOUT) == HAL_OK)
		return MAX30102_OK;
	return MAX30102_ERROR;
}

MAX30102_STATUS Max30102_ReadReg(uint8_t uch_addr, uint8_t *puch_data)
{
	if(HAL_I2C_Mem_Read(i2c_max30102, MAX30102_ADDRESS, uch_addr, 1, puch_data, 1, I2C_TIMEOUT) == HAL_OK)
		return MAX30102_OK;
	return MAX30102_ERROR;
}

MAX30102_STATUS Max30102_WriteRegisterBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(Register, &tmp))
		return MAX30102_ERROR;
	tmp &= ~(1<<Bit);
	tmp |= (Value&0x01)<<Bit;
	if(MAX30102_OK != Max30102_WriteReg(Register, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}

/*
	ВНИМАНИЕ!!! часто в модулях Max30102 и Max30105 перепутаны каналы светодиодов ir_led  и red_led поэтому если показания данных не корректное
	то тогда в функции Max30102_ReadFifo в файле MAX30102.c и MAX30102.h меняем местами входящие параметры. 
	Ниже приведен пример:

	MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_red_led, volatile uint32_t *pun_ir_led){}	// вариант 1
	MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_ir_led, volatile uint32_t *pun_red_led){}	// вариант 2
*/
MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_ir_led, volatile uint32_t *pun_red_led)
{
	uint32_t un_temp;
	*pun_red_led=0;
	*pun_ir_led=0;
	uint8_t ach_i2c_data[6];

	if(HAL_I2C_Mem_Read(i2c_max30102, MAX30102_ADDRESS, REG_FIFO_DATA, 1, ach_i2c_data, 6, I2C_TIMEOUT) != HAL_OK)
	{
		return MAX30102_ERROR;
	}
	un_temp=(unsigned char) ach_i2c_data[0];
	un_temp<<=16;
	*pun_red_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[1];
	un_temp<<=8;
	*pun_red_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[2];
	*pun_red_led+=un_temp;

	un_temp=(unsigned char) ach_i2c_data[3];
	un_temp<<=16;
	*pun_ir_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[4];
	un_temp<<=8;
	*pun_ir_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[5];
	*pun_ir_led+=un_temp;
	*pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
	*pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]

	return MAX30102_OK;
}

//
//	Interrupts
//
MAX30102_STATUS Max30102_SetIntAlmostFullEnabled(uint8_t Enable)
{
	return Max30102_WriteRegisterBit(REG_INTR_ENABLE_1, INT_A_FULL_BIT, Enable);
}

MAX30102_STATUS Max30102_SetIntFifoDataReadyEnabled(uint8_t Enable)
{

	return Max30102_WriteRegisterBit(REG_INTR_ENABLE_1, INT_PPG_RDY_BIT, Enable);
}

MAX30102_STATUS Max30102_SetIntAmbientLightCancelationOvfEnabled(uint8_t Enable)
{

	return Max30102_WriteRegisterBit(REG_INTR_ENABLE_2, INT_ALC_OVF_BIT, Enable);
}

MAX30102_STATUS Max30102_SetIntInternalTemperatureReadyEnabled(uint8_t Enable)
{

	return Max30102_WriteRegisterBit(REG_INTR_ENABLE_2, INT_DIE_TEMP_RDY_BIT, Enable);
}

MAX30102_STATUS Max30102_ReadInterruptStatus(uint8_t *Status)
{
	uint8_t tmp;
	*Status = 0;

	if(MAX30102_OK != Max30102_ReadReg(REG_INTR_STATUS_1, &tmp))
		return MAX30102_ERROR;
	*Status |= tmp & 0xE1; // 3 highest bits

	if(MAX30102_OK != Max30102_ReadReg(REG_INTR_STATUS_2, &tmp))
		return MAX30102_ERROR;
	*Status |= tmp & 0x02;

	return MAX30102_OK;
}

void Max30102_InterruptCallback(void)
{
	uint8_t Status;
	while(MAX30102_OK != Max30102_ReadInterruptStatus(&Status));

	// Almost Full FIFO Interrupt handle
	if(Status & (1<<INT_A_FULL_BIT))
	{
		for(uint8_t i = 0; i < MAX30102_FIFO_ALMOST_FULL_SAMPLES; i++)
		{
			while(MAX30102_OK != Max30102_ReadFifo((RedBuffer+BufferHead), (IrBuffer+BufferHead)));
			if(IsFingerOnScreen)
			{
				if(IrBuffer[BufferHead] < MAX30102_IR_VALUE_FINGER_OUT_SENSOR) IsFingerOnScreen = 0;
			}
			else
			{
				if(IrBuffer[BufferHead] > MAX30102_IR_VALUE_FINGER_ON_SENSOR) IsFingerOnScreen = 1;
			}
			BufferHead = (BufferHead + 1) % MAX30102_BUFFER_LENGTH;
			CollectedSamples++;
		}
	}

	// New FIFO Data Ready Interrupt handle
	if(Status & (1<<INT_PPG_RDY_BIT))
	{
		while(MAX30102_OK != Max30102_ReadFifo((RedBuffer+BufferHead), (IrBuffer+BufferHead)));
		if(IsFingerOnScreen)
		{
			if(IrBuffer[BufferHead] < MAX30102_IR_VALUE_FINGER_OUT_SENSOR) IsFingerOnScreen = 0;
		}
		else
		{
			if(IrBuffer[BufferHead] > MAX30102_IR_VALUE_FINGER_ON_SENSOR) IsFingerOnScreen = 1;
		}
		BufferHead = (BufferHead + 1) % MAX30102_BUFFER_LENGTH;
		CollectedSamples++;
	}

	//  Ambient Light Cancellation Overflow Interrupt handle
	if(Status & (1<<INT_ALC_OVF_BIT))
	{

	}

	// Power Ready Interrupt handle
	if(Status & (1<<INT_PWR_RDY_BIT))
	{
		
	}	
	
	// Internal Temperature Ready Interrupt handle
	if(Status & (1<<INT_DIE_TEMP_RDY_BIT))
	{

	}
}

//
//	FIFO Configuration
//
MAX30102_STATUS Max30102_FifoWritePointer(uint8_t Address)
{
	if(MAX30102_OK != Max30102_WriteReg(REG_FIFO_WR_PTR,(Address & 0x1F)))  //FIFO_WR_PTR[4:0]
			return MAX30102_ERROR;
	return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoOverflowCounter(uint8_t Address)
{
	if(MAX30102_OK != Max30102_WriteReg(REG_OVF_COUNTER,(Address & 0x1F)))  //OVF_COUNTER[4:0]
			return MAX30102_ERROR;
	return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoReadPointer(uint8_t Address)
{
	if(MAX30102_OK != Max30102_WriteReg(REG_FIFO_RD_PTR,(Address & 0x1F)))  //FIFO_RD_PTR[4:0]
			return MAX30102_ERROR;
	return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoSampleAveraging(uint8_t Value)
{
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_FIFO_CONFIG, &tmp))
		return MAX30102_ERROR;
	tmp &= ~(0x07);
	tmp |= (Value&0x07)<<5;
	if(MAX30102_OK != Max30102_WriteReg(REG_FIFO_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoRolloverEnable(uint8_t Enable)
{
	return Max30102_WriteRegisterBit(REG_FIFO_CONFIG, FIFO_CONF_FIFO_ROLLOVER_EN_BIT, (Enable & 0x01));
}

MAX30102_STATUS Max30102_FifoAlmostFullValue(uint8_t Value)
{
	if(Value < 17) Value = 17;
	if(Value > 32) Value = 32;
	Value = 32 - Value;
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_FIFO_CONFIG, &tmp))
		return MAX30102_ERROR;
	tmp &= ~(0x0F);
	tmp |= (Value & 0x0F);
	if(MAX30102_OK != Max30102_WriteReg(REG_FIFO_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}
//
//	Mode Configuration
//
MAX30102_STATUS Max30102_ShutdownMode(uint8_t Enable)
{
	return Max30102_WriteRegisterBit(REG_MODE_CONFIG, MODE_SHDN_BIT, (Enable & 0x01));
}

MAX30102_STATUS Max30102_Reset(void)
{
	uint8_t tmp = 0xFF;
    if(MAX30102_OK != Max30102_WriteReg(REG_MODE_CONFIG,0x40))
        return MAX30102_ERROR;
    do
    {
    	if(MAX30102_OK != Max30102_ReadReg(REG_MODE_CONFIG, &tmp))
    		return MAX30102_ERROR;
    } while(tmp & (1<<6));

    return MAX30102_OK;
}

MAX30102_STATUS Max30102_SetMode(uint8_t Mode)
{
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_MODE_CONFIG, &tmp))
		return MAX30102_ERROR;
	tmp &= ~(0x07);
	tmp |= (Mode & 0x07);
	if(MAX30102_OK != Max30102_WriteReg(REG_MODE_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}
//
//	SpO2 Configuration
//
MAX30102_STATUS Max30102_SpO2AdcRange(uint8_t Value)
{
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_SPO2_CONFIG, &tmp))
		return MAX30102_ERROR;
	tmp &= ~(0x03);
	tmp |= ((Value & 0x03) << 5);
	if(MAX30102_OK != Max30102_WriteReg(REG_SPO2_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}

MAX30102_STATUS Max30102_SpO2SampleRate(uint8_t Value)
{
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_SPO2_CONFIG, &tmp))
		return MAX30102_ERROR;
	tmp &= ~(0x07);
	tmp |= ((Value & 0x07) << 2);
	if(MAX30102_OK != Max30102_WriteReg(REG_SPO2_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}

MAX30102_STATUS Max30102_SpO2LedPulseWidth(uint8_t Value)
{
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_SPO2_CONFIG, &tmp))
		return MAX30102_ERROR;
	tmp &= ~(0x03);
	tmp |= (Value & 0x03);
	if(MAX30102_OK != Max30102_WriteReg(REG_SPO2_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}

  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

//	LEDs Pulse Amplitute Configuration
//	LED Current = Value * 0.2 mA
//
MAX30102_STATUS Max30102_Led1PulseAmplitude(uint8_t Value)
{
	if(MAX30102_OK != Max30102_WriteReg(REG_LED1_PA, Value))
		return MAX30102_ERROR;
	return MAX30102_OK;
}

MAX30102_STATUS Max30102_Led2PulseAmplitude(uint8_t Value)
{
	if(MAX30102_OK != Max30102_WriteReg(REG_LED2_PA, Value))
		return MAX30102_ERROR;
	return MAX30102_OK;
}

MAX30102_STATUS Max30102_Led3PulseAmplitude(uint8_t Value)
{
	if(MAX30102_OK != Max30102_WriteReg(REG_LED3_PA, Value))
		return MAX30102_ERROR;
	return MAX30102_OK;
}

//
//	Usage functions
//
MAX30102_STATUS Max30102_IsFingerOnSensor(void)
{
	if(IsFingerOnScreen){
		return MAX30102_OK;
	}
	else{
		return MAX30102_ERROR;
	}
}

int32_t Max30102_GetHeartRate(void)
{
	return HeartRate;
}

int32_t Max30102_GetSpO2Value(void)
{
	return Sp02Value;
}

void Max30102_Task(void)
{
	switch(StateMachine)
	{
		//  Палец не размещен на датчике. Красный диод выключен, ИК на
		//	минимуме. ИК-диод предназначен для обнаружения пальца, расположенного на датчике. Всякий
		//	раз, когда палец убирается с датчика, программа возвращается в это состояние, погасив
		//	светодиоды
		case MAX30102_STATE_BEGIN:	//--------------------------------------
			HeartRate = 0;
			Sp02Value = 0;
			if(IsFingerOnScreen)
			{
				CollectedSamples = 0;
				BufferTail = BufferHead;
				Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_HIGH);
				Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_HIGH);
				Max30102_Led3PulseAmplitude(MAX30102_GREEN_LED_CURRENT_HIGH);
				StateMachine = MAX30102_STATE_CALIBRATE;
			}
			break;

		// Поставив палец, заполните буфер «действительными
		// данными». По умолчанию расчеты производятся на выборках за последние 5
		// секунд. Калибровка, то есть заполнение буфера, так что это займет так много времени.
		case MAX30102_STATE_CALIBRATE:	//-----------------------------------
				if(IsFingerOnScreen)
				{
					if(CollectedSamples > (MAX30102_BUFFER_LENGTH-MAX30102_SAMPLES_PER_SECOND))
					{
						StateMachine = MAX30102_STATE_CALCULATE_HR;
					}
				}
				else
				{
					Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_LOW);
					Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_LOW);
					Max30102_Led3PulseAmplitude(MAX30102_GREEN_LED_CURRENT_LOW);
					StateMachine = MAX30102_STATE_BEGIN;
				}
			break;

		// После сбора соответствующего количества образцов,
		// расчеты производятся по алгоритму Максим. На этом этапе циклический буфер с количеством
		// выборок, соответствующих одной секунде, перемещается.
		case MAX30102_STATE_CALCULATE_HR:	//---------------------------------
			if(IsFingerOnScreen)
			{
				maxim_heart_rate_and_oxygen_saturation(IrBuffer, RedBuffer, MAX30102_BUFFER_LENGTH-MAX30102_SAMPLES_PER_SECOND, BufferTail, &Sp02Value, &Sp02IsValid, &HeartRate, &IsHrValid);
				BufferTail = (BufferTail + MAX30102_SAMPLES_PER_SECOND) % MAX30102_BUFFER_LENGTH;
				CollectedSamples = 0;
				StateMachine = MAX30102_STATE_COLLECT_NEXT_PORTION;
			}
			else
			{
				Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_LOW);
				Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_LOW);
				Max30102_Led3PulseAmplitude(MAX30102_GREEN_LED_CURRENT_LOW);
				StateMachine = MAX30102_STATE_BEGIN;
			}
			break;

		// Служба датчиков находится в этом состоянии
		// до тех пор, пока не будет собрана следующая партия образцов, то есть со следующей
		// секунды. После этого он переходит в состояние номер 3 и цикл повторяется до тех пор, пока
		// палец не будет удален с датчика.
		case MAX30102_STATE_COLLECT_NEXT_PORTION:	//--------------------------------------
			if(IsFingerOnScreen)
			{
				if(CollectedSamples > MAX30102_SAMPLES_PER_SECOND)
				{
					StateMachine = MAX30102_STATE_CALCULATE_HR;
				}
			}
			else
			{
				Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_LOW);
				Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_LOW);
				Max30102_Led3PulseAmplitude(MAX30102_GREEN_LED_CURRENT_LOW);
				StateMachine = MAX30102_STATE_BEGIN;
			}
			break;
	}
}

//
//	Initialization
//
MAX30102_STATUS Max30102_Init(I2C_HandleTypeDef *i2c)
{
	uint8_t uch_dummy;
	
	i2c_max30102 = i2c;
	
	if(MAX30102_OK != Max30102_Reset()) //resets the MAX30102
		return MAX30102_ERROR;	
	
	if(MAX30102_OK != Max30102_ReadReg(0,&uch_dummy))	//read and clear status register
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_SetIntAlmostFullEnabled(1))
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_SetIntFifoDataReadyEnabled(1))
		return MAX30102_ERROR;

	// для температуры прерывание
	Max30102_WriteReg(REG_INTR_ENABLE_2,0x00);
	
	if(MAX30102_OK != Max30102_FifoWritePointer(0x00))
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_FifoOverflowCounter(0x00))
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_FifoReadPointer(0x00))
		return MAX30102_ERROR;
	
	// Чтобы уменьшить объем данных, соседние выборки (в каждом отдельном канале) могут быть усреднены и прорежены на кристалле путем установки этого регистра.
	if(MAX30102_OK != Max30102_FifoSampleAveraging(FIFO_SMP_AVE_1))
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_FifoRolloverEnable(0))
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_FifoAlmostFullValue(MAX30102_FIFO_ALMOST_FULL_SAMPLES))
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_SetMode( MODE_SPO2_MODE /*MODE_MULTI_LED_MODE*/ ))
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_SpO2AdcRange(SPO2_ADC_RGE_4096))
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_SpO2SampleRate(SPO2_SAMPLE_RATE))
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_SpO2LedPulseWidth(SPO2_PULSE_WIDTH_411))
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_Led1PulseAmplitude(MAX30102_IR_LED_CURRENT_LOW))
		return MAX30102_ERROR;
	 
	if(MAX30102_OK != Max30102_Led2PulseAmplitude(MAX30102_RED_LED_CURRENT_LOW))
		return MAX30102_ERROR;
	
	if(MAX30102_OK != Max30102_Led3PulseAmplitude(MAX30102_GREEN_LED_CURRENT_LOW))
		return MAX30102_ERROR;

	if(MAX30102_OK != Max30102_WriteReg(REG_PILOT_PA,0x1F))   // Choose value for ~ 25mA for Pilot LED Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
		return MAX30102_ERROR;	
	
	
	//-- SLOT  включать  только в режиме мульти ( MODE_MULTI_LED_MODE )------------------------
//	if(MAX30102_OK != Max30102_Slot( 1, SLOT_RED_LED ))
//		return MAX30102_ERROR;
//	
//	if(MAX30102_OK != Max30102_Slot( 2, SLOT_IR_LED ))
//		return MAX30102_ERROR;
	//---------------------------------------------------------------------------------------
	
	
	//--- Clear FIFO ------------------------------------------------------------------------
	if(MAX30102_OK != Max30102_WriteReg(REG_FIFO_WR_PTR,0x00)) 
		return MAX30102_ERROR;
	if(MAX30102_OK != Max30102_WriteReg(REG_OVF_COUNTER,0x00)) 
		return MAX30102_ERROR;
	if(MAX30102_OK != Max30102_WriteReg(REG_FIFO_RD_PTR,0x00)) 
		return MAX30102_ERROR;
	//---------------------------------------------------------------------------------------
	
	
	StateMachine = MAX30102_STATE_BEGIN;
	
		return MAX30102_OK;
}


float Max30102_ReadTemperature(void)
{
    uint8_t temp_inter, temp_fra;
    float temp_value;
	
	if(MAX30102_OK != Max30102_WriteReg(REG_TEMP_CONFIG,0x01))   // Enable TEMP
		return MAX30102_ERROR;
	
    Max30102_ReadReg(REG_TEMP_INTR,&temp_inter);
    Max30102_ReadReg(REG_TEMP_FRAC,&temp_fra);
    
    temp_value = temp_inter + temp_fra*0.0625;
	
    return  temp_value;
}

MAX30102_STATUS Max30102_Slot(uint8_t slotNumber, uint8_t Value)
{	
	uint8_t tmp;
	
	switch (slotNumber) {
    case (1):
		//----------------------------------------------------------------------
		if(MAX30102_OK != Max30102_ReadReg(REG_MULTI_LED_CTRL1, &tmp))
			return MAX30102_ERROR;
		tmp &= ~(0x03);
		tmp |= ((Value & 0x03) << 0);
		if(MAX30102_OK != Max30102_WriteReg(REG_MULTI_LED_CTRL1, tmp))
			return MAX30102_ERROR;
		//----------------------------------------------------------------------
      break;
    case (2):
		//----------------------------------------------------------------------
		if(MAX30102_OK != Max30102_ReadReg(REG_MULTI_LED_CTRL1, &tmp))
			return MAX30102_ERROR;
		tmp &= ~(0x03);
		tmp |= ((Value & 0x03) << 4);
		if(MAX30102_OK != Max30102_WriteReg(REG_MULTI_LED_CTRL1, tmp))
			return MAX30102_ERROR;
		//----------------------------------------------------------------------
      break;
    case (3):
		//----------------------------------------------------------------------
		if(MAX30102_OK != Max30102_ReadReg(REG_MULTI_LED_CTRL2, &tmp))
			return MAX30102_ERROR;
		tmp &= ~(0x03);
		tmp |= ((Value & 0x03) << 0);
		if(MAX30102_OK != Max30102_WriteReg(REG_MULTI_LED_CTRL2, tmp))
			return MAX30102_ERROR;
		//----------------------------------------------------------------------
      break;
    case (4):
		//----------------------------------------------------------------------
		if(MAX30102_OK != Max30102_ReadReg(REG_MULTI_LED_CTRL2, &tmp))
			return MAX30102_ERROR;
		tmp &= ~(0x03);
		tmp |= ((Value & 0x03) << 4);
		if(MAX30102_OK != Max30102_WriteReg(REG_MULTI_LED_CTRL2, tmp))
			return MAX30102_ERROR;
		//----------------------------------------------------------------------
      break;
    default:
      //Shouldn't be here!
      break;
  }

	return MAX30102_OK;
}
