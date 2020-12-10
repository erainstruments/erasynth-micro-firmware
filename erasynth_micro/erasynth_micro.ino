#include <SPI.h>

#define ledPin 13
#define currentPin A2
#define adf4002_LE 4
#define lmx_LE 8
#define eeprom_LE 5
#define dac1_LE 12
#define rfsa_LE 6
#define tcxoPin 11
#define ampPin 9
#define trigPin 7
#define micPin A1
#define extPin A0
#define vibMotor 10
#define temp_LE A5
#define lmxRead A4
#define adfRead A3

#define ledBlinkInterval 500000
#define maxSamplesNum 120
#define cmdCharBuffer 25

#define MODULATION_ON 1
#define MODULATION_OFF 0

#define MODULATION_AM 0
#define MODULATION_FM 1
#define MODULATION_PULSE 2

#define MODULATION_INTERNAL 0
#define MODULATION_EXTERNAL 1
#define MODULATION_MICROPHONE 2

#define MODULATION_SINE 0
#define MODULATION_RAMP 1
#define MODULATION_SQUARE 2
#define MODULATION_TRIANGLE 3

#define FSK_ADDRESS 0x7C

#define EEPROM_START_ADDR 20
#define VERSION_SIZE 10
#define VERSION_ADDR 2

#define EXTERNAL_EEPROM_READ 0x03
#define EXTERNAL_EEPROM_WRITE 0x02
#define EXTERNAL_EEPROM_WREN 0x06
#define EXTERNAL_EEPROM_MAX_CLOCK 5e6

#define LMX_CAL_15 32000
#define LMX_CAL_21 33000
#define F2250_CAL_15 34000
#define F2250_CAL_14 36000
#define F2250_CAL_13 38000
#define F2250_CAL_12 40000
#define F2250_CAL_11 42000
#define F2250_CAL_10 44000
#define F2250_CAL_21 46000

#define LMX_REG_ADDR 51
#define ADF_REG_ADDR 500

#define SINE_WAVEFORM_ADDR 601
#define RAMP_WAVEFORM_ADDR 1001
#define SQUARE_WAVEFORM_ADDR 1401
#define TRIANGLE_WAVEFORM_ADDR 1801

#define SETTINGS_ADDR 2201
#define SETTINGS_LENGTH 800
#define INITIATION_ADDR 0
#define REMEMBER_ADDR 1

#define MINUMUM_FREQUENCY 12500000
#define MAXIMUM_FREQUENCY 6400000000
#define MAXIMUM_INT_MOD_FREQ 20000
#define MINUMUM_DWELL 1000
#define MAXIMUM_DEVIATION 100000000000
#define MINUMUM_PULSE 50

#define DEFAULT_DELAY_AM 6000 // nanosecond
#define DEFAULT_DELAY_FM 5250 // nanosecond

uint8_t ledState = LOW;

uint32_t LMX_R0_reset = 0x00211E;
uint32_t LMX_R0 = 0x00211C;
uint32_t LMX_R0_ADDR_HOLD = 0x00291C;
uint32_t LMX_R0_ADDR_HOLD_mute = 0x00291D;
uint32_t LMX_R0_mute = 0x00211D;

// Serial interface parameters
bool stringComplete = false;
bool isCmdExist = false;
bool serialActivityExist = false;
char commandInString[32] = { 0 };
char cmdString[32] = { 0 };
char cmd1String[32] = { 0 };
char lcdCommand[32];
uint8_t termCount = 0;

bool isModulationEnable = false;
uint8_t modType = 0;
uint8_t modInput = 0;
uint8_t modWaveForm = 0;
uint32_t modIntFreq = 1000;

uint8_t amDepth = 99;
uint32_t fmDev = 50000;
uint32_t pulseWidth = 5000;
uint32_t pulsePeriod = 10000;

// 120 for internal 
// 128 for external
uint16_t modArray[maxSamplesNum + 8];
uint32_t modDelayVal = 0;

uint64_t sweepStartFrequency = 1e9;
uint64_t sweepStopFrequency = 2e9;
uint64_t sweepStepFrequency = 100e6;
uint64_t sweepPoints = 100;
uint32_t sweepDwellTime = 1000000;
int32_t sweepIndex = 0;
uint8_t sweepType = 0;
bool isSweepOn = false;

uint32_t LMXOutPwr = 0;
uint64_t Frequency = 1e9;
bool isRFOnOff = true;
int amplitude = 0;

uint8_t reference = 0;
uint16_t valueOfDAC = 0;
uint8_t valueofRFSA = 0;
bool isVibrationOn = true;
uint16_t eepromAddr = 0;
bool isExtModAttch = false;
float fskMultiplier = 0;
bool nextFreq = false;
char firmwareVersion[VERSION_SIZE] = "v1.0.8";
bool isUploadCode = false;

uint8_t decimationValue = 0;

void setup()
{
	Serial.begin(9600);
	Serial1.begin(9600);
	SPI.begin();

	pinMode(eeprom_LE, OUTPUT);
	digitalWrite(eeprom_LE, HIGH);

	pinMode(temp_LE, OUTPUT);
	digitalWrite(temp_LE, HIGH);

	pinMode(rfsa_LE, OUTPUT);
	digitalWrite(rfsa_LE, HIGH);

	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, LOW);

	pinMode(tcxoPin, OUTPUT);
	digitalWrite(tcxoPin, LOW);

	pinMode(ampPin, OUTPUT);
	digitalWrite(ampPin, HIGH);

	pinMode(vibMotor, OUTPUT);
	digitalWrite(vibMotor, HIGH);

	pinMode(adf4002_LE, OUTPUT);
	digitalWrite(adf4002_LE, HIGH);

	pinMode(lmx_LE, OUTPUT);
	digitalWrite(lmx_LE, HIGH);

	pinMode(dac1_LE, OUTPUT);
	digitalWrite(dac1_LE, HIGH);
	
	//
	// Load or Reset the EEPROM due to version difference
	//
	if (readFromExEEPROM(0) == 0x12)
	{
		uint16_t a1, a2 = 0;

		char tmp[VERSION_SIZE] = { 0 };
		for (int i = 0; i < strlen(firmwareVersion); i++) { if (isdigit(firmwareVersion[i])) { strncat(tmp, &firmwareVersion[i], 1); } }
		a1 = get64Bit(&tmp[0]);

		tmp[0] = 0;
		for (uint16_t i = VERSION_ADDR; i < (VERSION_ADDR + VERSION_SIZE); i++)
		{
			uint8_t ch = readFromExEEPROM(i);
			if (isdigit(ch)) { strncat(tmp, (char*)&ch, 1); }
		}
		a2 = get64Bit(&tmp[0]);

		if (a1 != a2)
		{
			eepromClear();

			writeToExEEPROM(INITIATION_ADDR, 0x12);

			for (uint8_t i = 0; i < sizeof(firmwareVersion); i++) { writeToExEEPROM(VERSION_ADDR + i, firmwareVersion[i]); }
			eepromSave();
		}

		if (readFromExEEPROM(REMEMBER_ADDR) == 1) { eepromLoad(); }
	}
	else
	{
		eepromClear();
		writeToExEEPROM(INITIATION_ADDR, 0x12);
		for (uint8_t i = 0; i < sizeof(firmwareVersion); i++) { writeToExEEPROM(VERSION_ADDR + i, firmwareVersion[i]); }
		eepromSave();
	}

	SPI.beginTransaction(SPISettings(5e6, MSBFIRST, SPI_MODE0));
	for (uint8_t i = 0; i < 4; i++)
	{
		digitalWrite(adf4002_LE, LOW);
		uint8_t b1 = readFromExEEPROM((ADF_REG_ADDR + (3 * i) + 0));
		uint8_t b2 = readFromExEEPROM((ADF_REG_ADDR + (3 * i) + 1));
		uint8_t b3 = readFromExEEPROM((ADF_REG_ADDR + (3 * i) + 2));

		SPI.transfer(b1);
		SPI.transfer(b2);
		SPI.transfer(b3);
		digitalWrite(adf4002_LE, HIGH);
	}
	SPI.endTransaction();

	spiWrite_LMX(&LMX_R0_reset);
	spiWrite_LMX(&LMX_R0);

	for (uint16_t i = 0; i < 126; i++)
	{
		uint32_t reg = 0;
		readFromExEEPROM(((3 * i) + LMX_REG_ADDR), (uint8_t*)&reg);
		spiWrite_LMX(&reg);
		delay(1);
	}

	spiWrite_LMX(&LMX_R0);

	setRFSA3714(0);
	setF2250(0);
	setReferenceType(reference);
	setAmplitude();
	setLMX(Frequency);
	rfOnOff();

	//// Set Timer1 for Led Blink
	TCCR1B = _BV(WGM13) | _BV(CS11) | _BV(CS10);
	TCCR1A = 0;
	ICR1 = 62500;
	TIMSK1 = _BV(TOIE1);
	
	// Setup ADC for Free Running Mode.Default is ADC 5
	ADMUX = 0x45;
	ADCSRA = 0xE1;
	ADCSRB = 0x80;

	if (isSweepOn) { setSweepParams(); }
	if (isModulationEnable) { setModulationSettings(); }
}

void loop()
{
	handleSerial();
	if (!isUploadCode)
	{
		startModulation();
	}

}

void setDAC1(uint16_t value)
{
	// 12 bit DAC
	// Last 2 bit (MSB)
	valueOfDAC = value;

	byte DAC[2] = { 0 };

	value = value << 2;

	DAC[1] = (byte)value;			//LSB of DAC_Value
	DAC[0] = (byte)(value >> 8);	//MSB of DAC_Value

	cli();
	SPI.beginTransaction(SPISettings(10e6, MSBFIRST, SPI_MODE1));
	PORTD &= ~(1 << PORTD6);
	SPI.transfer(DAC, 2);
	PORTD |= (1 << PORTD6);
	SPI.endTransaction();
	sei();
}

void setMinAmplitude()
{
	LMXOutPwr = 0;
	setLMXPower();
	setF2250(4095);
	setRFSA3714(127);
}

void setLMX(uint64_t freq)
{
	cli();
	
	Frequency = freq;
	if (Frequency < MINUMUM_FREQUENCY) { Frequency = MINUMUM_FREQUENCY; }
	if (Frequency > MAXIMUM_FREQUENCY) { Frequency = MAXIMUM_FREQUENCY; }
	freq = Frequency;

	setMinAmplitude();

	uint32_t CHDIV = 0, DIVRAT = 1;

	if (freq < 25e6 && freq >= 125e5) { DIVRAT = 256;  CHDIV = 14; }
	else if (freq < 50e6 && freq >= 25e6) { DIVRAT = 128; CHDIV = 12; }
	else if (freq < 100e6 && freq >= 50e6) { DIVRAT = 64; CHDIV = 9; }
	else if (freq < 200e6 && freq >= 100e6) { DIVRAT = 32; CHDIV = 7; }
	else if (freq < 400e6 && freq >= 200e6) { DIVRAT = 16; CHDIV = 5; }
	else if (freq < 800e6 && freq >= 400e6) { DIVRAT = 8; CHDIV = 3; }
	else if (freq < 1600e6 && freq >= 800e6) { DIVRAT = 4; CHDIV = 1; }
	else if (freq < 3200e6 && freq >= 1600e6) { DIVRAT = 2; CHDIV = 0; }

	uint64_t freq_vco = (uint64_t)DIVRAT * freq;

	uint32_t R75 = 0x4B0800 | (CHDIV << 6);

	uint64_t Nlmx = freq_vco / 100000000;
	uint32_t R36 = (uint32_t)Nlmx + 2359296;
	uint64_t pll_num = freq_vco % 100000000;

	// PLL_DEN registers. These are set to 100MHz
	uint32_t R39 = 0x27E100;
	uint32_t R38 = 0x2605F5;

	if (isModulationEnable && modType == 1)
	{
		// 1 MHz
		R39 = 0x274240;
		R38 = 0x26000F;
		pll_num /= 100;
	}

	uint32_t R42 = 0x2A0000 | (pll_num >> 16);
	uint32_t R43 = 0x2B0000 | (pll_num & (0x00FFFF));
	uint32_t R45 = 0x2DC601;

	if (DIVRAT == 1) { R45 = 0x2DCE01; }
	R45 &= (LMXOutPwr | 0xFFFF00);

	uint32_t r0 = isRFOnOff ? LMX_R0 : LMX_R0_mute;
	if (isModulationEnable && modType == 1) 
	{ 
		r0 = isRFOnOff ? LMX_R0_ADDR_HOLD : LMX_R0_ADDR_HOLD_mute;
	}

	spiWrite_LMX(&R39);
	spiWrite_LMX(&R38);
	spiWrite_LMX(&R75);
	spiWrite_LMX(&R45);
	spiWrite_LMX(&R43);
	spiWrite_LMX(&R42);
	spiWrite_LMX(&R36);
	spiWrite_LMX(&r0);

	setAmplitude();
	sei();
}

void setLMXPower()
{
	uint32_t R44 = 0x2C3FA3;
	uint32_t R45 = 0x2DC63F;
	if (Frequency >= 3200e6) { R45 = 0x2DCE3F; }

	R44 &= ((LMXOutPwr << 8) | 0xFF00FF);
	R45 &= (LMXOutPwr | 0xFFFF00);

	spiWrite_LMX(&R44);
	spiWrite_LMX(&R45);
}

void rfOnOff()
{
	if (!isRFOnOff)
	{
		PORTB |= (1 << PORTB5);
		spiWrite_LMX(&LMX_R0_mute);
	}
	else
	{
		PORTB &= ~(1 << PORTB5);
		spiWrite_LMX(&LMX_R0);
	}
}

void setRFSA3714(int value)
{
	valueofRFSA = value;
	cli();
	SPI.beginTransaction(SPISettings(10e6, LSBFIRST, SPI_MODE0));
	PORTD &= ~(1 << PORTD7);
	SPI.transfer(value);
	PORTD |= (1 << PORTD7);
	SPI.endTransaction();
	sei();
}

void setF2250(int value)
{
	setDAC1(value);
}

void setReferenceType(int type)
{
	if (type == 0)
	{
		// INTERNAL
		digitalWrite(tcxoPin, HIGH);
	}
	else
	{
		// EXTERNAL
		digitalWrite(tcxoPin, LOW);
	}
}

void startModulation()
{
	if (isModulationEnable)
	{
		setModulationSettings();

		if (modType == MODULATION_AM)
		{
			if (modInput == MODULATION_INTERNAL)
			{
				while (isModulationEnable)
				{					
					SPI.beginTransaction(SPISettings(10e6, MSBFIRST, SPI_MODE1));
					for (int i = 0; i < maxSamplesNum; i += decimationValue)
					{
						uint16_t value = modArray[i] << 2;
						PORTD &= ~(1 << PORTD6);
						SPI.transfer((uint8_t)(value >> 8));
						SPI.transfer((uint8_t)value);
						PORTD |= (1 << PORTD6);
						delayNanoseconds(modDelayVal);
					}
					if (nextFreq) { sweep(); }
					if (Serial.available() || Serial1.available()) { break; }
				}
				SPI.endTransaction();

				setAmplitude();
			}
			else if (modInput == MODULATION_EXTERNAL || modInput == MODULATION_MICROPHONE) // External
			{
				ADMUX = modInput == MODULATION_EXTERNAL ? 0x47 : 0x46;
				ADCSRA = 0xE2;

				SPI.beginTransaction(SPISettings(10e6, MSBFIRST, SPI_MODE1));

				float volt = (float)valueOfDAC * (3.3 / 4096.0);
				volt = ((volt - 0.8) * 30) + 5;

				for (uint8_t i = 0; i < (maxSamplesNum + 8); i++) 
				{
					float sample = (5.0 / 1024.0) * i * 8;
					sample -= 2.5; // substract the 2.5V offset
					sample /= 5; // Scale down to 1V
					sample *= ((float)amDepth / 100.0);
					sample += 0.5;

					sample = 10 + 20 * log10(sample);

					sample = (10 - sample + volt) * (1.0 / 30.0);
					sample = (sample + 0.65) / (3.3 / 4096.0);

					modArray[i] = ((uint16_t)sample) << 2;
				}

				while (isModulationEnable)
				{
					if (Serial.available() || Serial1.available()) { break; }
					if (nextFreq) { sweep(); SPI.beginTransaction(SPISettings(10e6, MSBFIRST, SPI_MODE1)); }
					while (!(ADCSRA & (1 << ADIF)));

					//uint16_t sample = (ADCL | (ADCH << 8));
					//uint16_t value = modArray[sample >> 3];
					uint16_t value = modArray[(ADCL >> 3) | (ADCH << 5)];

					PORTD &= ~(1 << PORTD6);
					SPI.transfer((uint8_t)(value >> 8));
					SPI.transfer((uint8_t)value);
					PORTD |= (1 << PORTD6);
				}

				SPI.endTransaction();

				setAmplitude();
			}
		}
		else if (modType == MODULATION_FM)
		{
			if (modInput == MODULATION_INTERNAL) // Internal
			{
				SPI.beginTransaction(SPISettings(10e6, MSBFIRST, SPI_MODE0));

				while (isModulationEnable)
				{
					PORTB &= ~(1 << PORTB4);
					SPI.transfer(FSK_ADDRESS);

					for (int i = 0; i < maxSamplesNum; i += decimationValue)
					{
						SPI.transfer((uint8_t)(modArray[i] >> 8));
						SPI.transfer((uint8_t)modArray[i]);
						delayNanoseconds(modDelayVal);
					}

					PORTB |= (1 << PORTB4);

					if (nextFreq) { sweep(); setModulationSettings(); }
					if (Serial.available() || Serial1.available()) { break; }
				}

				SPI.endTransaction();

				// Write FSK Disable
				uint32_t R114 = 0x727C03 & 0xFFFBFF;
				spiWrite_LMX(&R114);
				setLMX(Frequency);
			}
			else if (modInput == MODULATION_EXTERNAL || modInput == MODULATION_MICROPHONE) // External
			{
				ADMUX = modInput == MODULATION_EXTERNAL ? 0x47 : 0x46;
				ADCSRA = 0xE2;
				
				SPI.beginTransaction(SPISettings(10e6, MSBFIRST, SPI_MODE0));
				PORTB &= ~(1 << PORTB4);
				SPI.transfer(FSK_ADDRESS);
				
				uint64_t fmDev_scaled = fmDev / (5.0 / 1024.0);
				uint64_t fskMultiplier_scaled = fskMultiplier / (5.0 / 1024.0);

				for (uint8_t i = 0; i < (maxSamplesNum + 8); i++)
				{
					float micVal = (5.0 / 1024.0) * i * 8;
					micVal -= 2.5;
					micVal /= 2.5; // Scale down to 1V
					micVal *= (float)fmDev;

					// This converts deviation frequency to register value
					uint16_t fskValue = 0;
					if (micVal < 0) { fskValue = (65536 + ((uint16_t)round(micVal * fskMultiplier))); }
					else { fskValue = ((uint16_t)round(micVal * fskMultiplier)); }

					modArray[i] = fskValue;
				}

				while (isModulationEnable)
				{
					if (Serial.available() || Serial1.available() || nextFreq) { break; }
					
					while (!(ADCSRA & (1 << ADIF)));
					// Read external input
					uint16_t m = ADCL | (ADCH << 8);

					uint16_t fskValue = modArray[m >> 3];

					SPI.transfer((uint8_t)(fskValue >> 8));
					SPI.transfer((uint8_t)fskValue);
				}
				
				PORTB |= (1 << PORTB4);
				SPI.endTransaction();

				if (nextFreq) { sweep(); }

				// Write FSK Disable
				uint32_t R114 = 0x727C03 & 0xFFFBFF;
				spiWrite_LMX(&R114);
				setLMX(Frequency);
			}
		}
		else if (modType == MODULATION_PULSE)
		{
			if (modInput == MODULATION_INTERNAL)
			{
				detachInterrupt(digitalPinToInterrupt(trigPin));
				isExtModAttch = false;

				uint32_t offTime = pulsePeriod - pulseWidth - 15; // 15 is default delay because of transactions
				uint32_t waitTime = pulseWidth - 10; // 10 is default delay because of transactions
				uint16_t dac = valueOfDAC;
				uint8_t rfsa = valueofRFSA;

				while (isModulationEnable)
				{
					setDAC1(dac);
					setRFSA3714(rfsa);
					delay_micro(waitTime);
					
					setDAC1(4095);
					setRFSA3714(127);
					delay_micro(offTime);
					
					if (nextFreq) { sweep(); }
					if (Serial.available() || Serial1.available()) { break; }
				}
				setAmplitude();
			}
			else
			{
				if (!isExtModAttch) { attachInterrupt(digitalPinToInterrupt(trigPin), pulse_rising, RISING); isExtModAttch = true; }
			}
		}
	}
}

void pulse_rising()
{
	if (nextFreq) { sweep(); }
	setF2250(valueOfDAC);
	setRFSA3714(valueofRFSA);
	attachInterrupt(digitalPinToInterrupt(trigPin), pulse_falling, FALLING);
}          

void pulse_falling()
{
	if (nextFreq) { sweep(); }
	byte DAC[2] = { 0 };

	uint16_t value = 4095 << 2;

	DAC[1] = (byte)value;			//LSB of DAC_Value
	DAC[0] = (byte)(value >> 8);	//MSB of DAC_Value

	SPI.beginTransaction(SPISettings(10e6, MSBFIRST, SPI_MODE1));
	PORTD &= ~(1 << PORTD6);
	SPI.transfer(DAC, 2);
	PORTD |= (1 << PORTD6);
	SPI.endTransaction();

	SPI.beginTransaction(SPISettings(10e6, LSBFIRST, SPI_MODE0));
	PORTD &= ~(1 << PORTD7);
	SPI.transfer(127);
	PORTD |= (1 << PORTD7);
	SPI.endTransaction();

	attachInterrupt(digitalPinToInterrupt(trigPin), pulse_rising, RISING);
}

float getFSKMultiplier()
{
	float pll_den = 1e6;
	float chdiv = 1;
	float f_pd = 100e6;
	float scale = pow(2, 0);

	if (Frequency < 25e6 && Frequency >= 125e5) { chdiv = 256; if (fmDev > 10e3) { fmDev = 10e3; } }
	else if (Frequency < 50e6 && Frequency >= 25e6) { chdiv = 128; if (fmDev > 25e3) { fmDev = 25e3; } }
	else if (Frequency < 100e6 && Frequency >= 50e6) { chdiv = 64; if (fmDev > 50e3) { fmDev = 50e3; } }
	else if (Frequency < 200e6 && Frequency >= 100e6) { chdiv = 32; if (fmDev > 100e3) { fmDev = 100e3; } }
	else if (Frequency < 400e6 && Frequency >= 200e6) { chdiv = 16; if (fmDev > 200e3) { fmDev = 200e3; } }
	else if (Frequency < 800e6 && Frequency >= 400e6) { chdiv = 8; if (fmDev > 400e3) { fmDev = 400e3; } }
	else if (Frequency < 1600e6 && Frequency >= 800e6) { chdiv = 4; if (fmDev > 800e3) { fmDev = 800e3; } }
	else if (Frequency < 3200e6 && Frequency >= 1600e6) { chdiv = 2; if (fmDev > 1600e3) { fmDev = 1600e3; } }
	else if (Frequency < 6400e6 && Frequency >= 3200e6) { chdiv = 1; if (fmDev > 3200e3) { fmDev = 3200e3; } }

	return (pll_den * chdiv) / (f_pd * scale);
}

uint16_t getWaveform(uint8_t index)
{
	uint16_t addr = (400 * modWaveForm) + (2 * index) + SINE_WAVEFORM_ADDR;
	uint16_t a1 = readFromExEEPROM(addr); addr++;
	uint16_t a2 = readFromExEEPROM(addr);
	return a1 | (a2 << 8);
}

void setModulationSettings()
{
	if (modType == MODULATION_AM)
	{
		decimationValue = ceil((float)modIntFreq / 1000.0);
		float delayAmount = (1e9 / (modIntFreq * ceil((float)maxSamplesNum / decimationValue)));
		modDelayVal = ((delayAmount - DEFAULT_DELAY_AM) < 0 ? 0 : round(delayAmount - DEFAULT_DELAY_AM)) / 250;

		float v[maxSamplesNum];
		for (int i = 0; i < maxSamplesNum; i++)
		{
			// Convert samples as 1V peek sine wave
			v[i] = (((getWaveform(i) * (1.0 / 4096.0)) - 0.5));
			// Apply AM Depth
			v[i] *= ((float)amDepth / 100.0);
			// Add center voltage offset
			v[i] += 0.5;

			//Current F2250 dB calculation from calibration data
			float volt = (float)valueOfDAC * (3.3 / 4096.0);
			volt = ((volt - 0.8) * 30) + 5;

			// Convert to Decibel
			v[i] = 10 + 20 * log10(v[i]);
			// 10 is the max db for max 1V. Then we multiply subtraction with slope of the attenuater
			// thus it gives the voltage for necessery attenuation
			
			v[i] = ((10 - v[i]) + volt) * (1.0 / 30.0); // Slope of the F2250 this function gives the voltage for attn
			// Slope calculation of attenutor starts the voltage from 0.8. so we add them up. then convert samples to bits for DAC
			v[i] = (v[i] + 0.65) / (3.3 / 4096.0);
		}

		for (int i = 0; i < maxSamplesNum; i++) { modArray[i] = (uint16_t)v[i]; }
	}
	else if (modType == MODULATION_FM)
	{
		// Set the frequency therefore pll_den will be calculated again as 1 MHz
		setLMX(Frequency);

		fskMultiplier = getFSKMultiplier();

		// To be able to use block programming, first write this value to register otherwise write LMX_R0
		uint32_t R0 = 0x00291C; // For ADD_HOLD value
		uint32_t R114 = 0x727C03;
		uint32_t R115 = 0x730000;
		spiWrite_LMX(&R114);
		spiWrite_LMX(&R115);
		spiWrite_LMX(&R0);

		float samples[maxSamplesNum];
		
		for (int i = 0; i < maxSamplesNum; i++)
		{
			samples[i] = ((getWaveform(i) * (1.0 / 4096.0)) - 0.5) * 2 * fmDev;
		}

		// This converts deviation frequency to register value
		for (int i = 0; i < maxSamplesNum; i++)
		{
			if (samples[i] < 0) { modArray[i] = (65536 + (round(samples[i] * fskMultiplier))); }
			else { modArray[i] = (round(samples[i] * fskMultiplier)); }
		}

		// Calculate the sampling period 
		decimationValue = ceil((float)modIntFreq / 1000.0);
		//decimationValue = 4;
		float delayAmount = (1e9 / (modIntFreq * ceil((float)maxSamplesNum / decimationValue)));
		modDelayVal = ((delayAmount - DEFAULT_DELAY_FM) < 0 ? 0 : round(delayAmount - DEFAULT_DELAY_FM)) / 250;
	}

	if (isSweepOn) { setSweepParams(); }
}

float getTemp()
{
	cli();
	SPI.beginTransaction(SPISettings(EXTERNAL_EEPROM_MAX_CLOCK, MSBFIRST, SPI_MODE0));
	digitalWrite(temp_LE, LOW);
	uint8_t a1 = SPI.transfer(0x00);
	uint8_t a2 = SPI.transfer(0x00);
	digitalWrite(temp_LE, HIGH);
	SPI.endTransaction();

	int a3 = ((a1 << 8) | (a2 & (0xF8))) >> 3;
	sei();
	return (float)a3 * 0.0625;
}

float getCurrent()
{
	ADMUX = 0x45;
	ADCSRA = 0xE7;

	while (!(ADCSRA & (1 << ADIF)));

	int val = ADCL | (ADCH << 8);
	return (5.0 / 1024.0) * val / 20 / 0.3;
}

void blinkLed()
{
	ledState = !ledState;
	digitalWrite(ledPin, ledState);
}

void spiWrite_LMX(uint32_t* data_u32)
{
	cli();
	SPI.beginTransaction(SPISettings(10e6, MSBFIRST, SPI_MODE0));
	digitalWrite(lmx_LE, LOW);
	SPI.transfer(((uint8_t*)data_u32)[2]);
	SPI.transfer(((uint8_t*)data_u32)[1]);
	SPI.transfer(((uint8_t*)data_u32)[0]);
	digitalWrite(lmx_LE, HIGH);
	SPI.endTransaction();
	sei();
}

void spiWrite_FSK(uint16_t* data_u16)
{
	cli();
	SPI.beginTransaction(SPISettings(10e6, MSBFIRST, SPI_MODE0));
	digitalWrite(lmx_LE, LOW);
	SPI.transfer(FSK_ADDRESS);
	SPI.transfer(((uint8_t*)data_u16)[1]);
	SPI.transfer(((uint8_t*)data_u16)[0]);
	digitalWrite(lmx_LE, HIGH);
	SPI.endTransaction();
	sei();
}

void handleSerial()
{
	if (Serial.available())
	{
		if (isUploadCode) { Serial1.write(Serial.read()); }
		else
		{
			while (Serial.available())
			{

				char inChar = (char)Serial.read();

				if (inChar == '\r' || inChar == '\n') { serialActivityExist = true; }
				if (inChar == '>' || isCmdExist) { isCmdExist = true; if (inChar != '\r' && inChar != '\n') { strncat(cmdString, &inChar, 1); } }
				if (inChar == '\r' && isCmdExist) { stringComplete = true; }

			}

			if (stringComplete)
			{
				stringComplete = false;
				command(cmdString);
				cmdString[0] = 0;
				isCmdExist = false;
			}
		}
	}

	if (Serial1.available())
	{
		if (isUploadCode) { Serial.write(Serial1.read()); }
		else
		{
			while (Serial1.available())
			{
				unsigned char inChar = Serial1.read();
				if (inChar == 0xFF) { termCount++; }
				else { strncat(cmd1String, &inChar, 1); }
				if (termCount == 3)
				{
					termCount = 0;
					if (cmd1String[0] == 0x70) { command(&cmd1String[1]); }
					cmd1String[0] = 0;
				}
			}
		}
	}
}

uint64_t get64Bit(char *input)
{
	uint64_t result = 0;
	uint64_t len = strlen(input);

	for (int i = len - 1; i >= 0; i--)
	{
		uint64_t multiplier = 1;
		for (int j = 0; j < (len - i - 1); j++) multiplier *= 10;
		result += (input[i] - '0') * multiplier;
	}

	return result;
}

void uint64ToString(uint64_t input, char* chr)
{
	unsigned char tmp = 0;
	uint8_t index = 0;

	chr[0] = '\0';

	do {
		tmp = input % 10;
		input /= 10;

		if (tmp < 10)
			tmp += '0';
		else
			tmp += 'A' - 10;

		index++;
		strncat(chr, &tmp, 1);
	} while (input);

	strrev(chr);
}

void delay_micro(uint32_t val)
{
	if (val < 16000) { delayMicroseconds(val); }
	else { delay(val / 1000); }
}

void delayNanoseconds(unsigned int it) // Delay = (250 x it) ns
{
	// busy wait
	__asm__ __volatile__(
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (it) : "0" (it) // 2 cycles
	);
	// return = 4 cycles
}

void command(char* commandBuffer)
{
	char commandID = commandBuffer[1];
	commandInString[0] = 0;
	char tmp[32] = { 0 };

	if (commandID == 'F')
	{
		// Set the frequency of LMX
		strcat(commandInString, &commandBuffer[2]);
		setLMX(get64Bit(commandInString));
	}
	else if (commandID == 'A')
	{
		strcat(commandInString, &commandBuffer[3]);

		if (commandBuffer[2] == '1')
		{
			// Set RFSA3714
			setRFSA3714(get64Bit(commandInString));
		}
		else if (commandBuffer[2] == '2')
		{
			// Set F2250
			setF2250(get64Bit(commandInString));
		}
	}
	else if (commandID == 'S')
	{
		if (commandBuffer[2] == 'A')
		{
			// Set Amplitude
			if (commandBuffer[3] == '-')
			{
				strcat(commandInString, &commandBuffer[4]);
				amplitude = get64Bit(commandInString) * -1;
			}
			else
			{
				strcat(commandInString, &commandBuffer[3]);
				amplitude = get64Bit(commandInString);
			}
			setAmplitude();
		}
		else if (commandBuffer[2] == 'R')
		{
			// Set Reference
			reference = commandBuffer[3] - '0';
			setReferenceType(reference);
		}
		else if (commandBuffer[2] == 'M')
		{
			// Modulation Settings
			if (commandBuffer[3] == '0') // DISABLE MODULATION
			{
				isModulationEnable = false;
				detachInterrupt(digitalPinToInterrupt(trigPin));
				setAmplitude();
			}
			else if (commandBuffer[3] == '1') // ENABLE MODULATION
			{
				isModulationEnable = true;
			}
			else if (commandBuffer[3] == 'F')
			{
				if (commandBuffer[4] == 'D')
				{
					// FM Deviation Frequency
					strcat(commandInString, &commandBuffer[5]);
					fmDev = get64Bit(commandInString);
					if (fmDev < 1) { fmDev = 1; }
					if (fmDev > MAXIMUM_DEVIATION) { fmDev = MAXIMUM_DEVIATION; }
				}
				else
				{
					// Internal Modulation Frequency
					strcat(commandInString, &commandBuffer[4]);
					modIntFreq = get64Bit(commandInString);
					if (modIntFreq < 1) { modIntFreq = 1; }
					if (modIntFreq > MAXIMUM_INT_MOD_FREQ) { modIntFreq = MAXIMUM_INT_MOD_FREQ; }
				}
			}
			else if (commandBuffer[3] == 'T')
			{
				// Modulation Type
				modType = commandBuffer[4] - '0';
			}
			else if (commandBuffer[3] == 'I')
			{
				// Modulation Input
				modInput = commandBuffer[4] - '0';
			}
			else if (commandBuffer[3] == 'W')
			{
				// Modulation Waveform Type
				modWaveForm = commandBuffer[4] - '0';
			}
			else if (commandBuffer[3] == 'A')
			{
				// AM Depth
				strcat(commandInString, &commandBuffer[4]);
				amDepth = get64Bit(commandInString);
				if (amDepth < 1) { amDepth = 1; }
				if (amDepth > 99) { amDepth = 99; }
			}
			else if (commandBuffer[3] == 'P')
			{
				if (commandBuffer[4] == 'P')
				{
					// Pulse Period
					strcat(commandInString, &commandBuffer[5]);
					pulsePeriod = get64Bit(commandInString);
					if (pulsePeriod < pulseWidth) { pulsePeriod = pulseWidth + MINUMUM_PULSE; }
					if (pulsePeriod - pulseWidth < MINUMUM_PULSE) { pulsePeriod += (MINUMUM_PULSE - (pulsePeriod - pulseWidth)); }
				}
				else if (commandBuffer[4] == 'W')
				{
					// Pulse Width
					strcat(commandInString, &commandBuffer[5]);
					pulseWidth = get64Bit(commandInString);
					if (pulseWidth < MINUMUM_PULSE) { pulseWidth = MINUMUM_PULSE; }
					if (pulsePeriod < pulseWidth) { pulsePeriod = pulseWidth + MINUMUM_PULSE; }
					if (pulsePeriod - pulseWidth < MINUMUM_PULSE) { pulsePeriod += (MINUMUM_PULSE - (pulsePeriod - pulseWidth)); }
				}
			}
			fskMultiplier = getFSKMultiplier();
		
			if (isModulationEnable)
			{
				detachInterrupt(digitalPinToInterrupt(trigPin));
				isExtModAttch = false;
				setModulationSettings();
			}
		}
		else if (commandBuffer[2] == 'P')
		{
			strcat(commandInString, &commandBuffer[3]);
			LMXOutPwr = get64Bit(commandInString);
			setLMXPower();
		}
		else if (commandBuffer[2] == 'S')  // Set Sweep
		{
			strcat(commandInString, &commandBuffer[4]);

			if (commandBuffer[3] == '1') // Start Freq
			{
				sweepStartFrequency = get64Bit(commandInString);
				if (sweepStartFrequency < MINUMUM_FREQUENCY) { sweepStartFrequency = MINUMUM_FREQUENCY; }
				if (sweepStartFrequency > MAXIMUM_FREQUENCY) { sweepStartFrequency = MAXIMUM_FREQUENCY; }
			}
			else if (commandBuffer[3] == '2') // Stop Freq
			{
				sweepStopFrequency = get64Bit(commandInString);
				if (sweepStopFrequency < MINUMUM_FREQUENCY) { sweepStopFrequency = MINUMUM_FREQUENCY; }
				if (sweepStopFrequency > MAXIMUM_FREQUENCY) { sweepStopFrequency = MAXIMUM_FREQUENCY; }
			}
			else if (commandBuffer[3] == '3') // Step Freq
			{
				sweepStepFrequency = get64Bit(commandInString);
				if (sweepStepFrequency < 1) { sweepStepFrequency = 1; }
				if (sweepStepFrequency > MAXIMUM_FREQUENCY) { sweepStepFrequency = MAXIMUM_FREQUENCY; }
			}
			else if (commandBuffer[3] == '4') // Dwell Time
			{
				sweepDwellTime = get64Bit(commandInString);
				if (sweepDwellTime < MINUMUM_DWELL) { sweepDwellTime = MINUMUM_DWELL; }
			}
			else if (commandBuffer[3] == '5') // Sweep ON
			{
				isSweepOn = true;
				setSweepParams();
			}
			else if (commandBuffer[3] == '6') // Sweep OFF
			{
				isSweepOn = false;
				//detach timer interrupt
				TIMSK3 = 0;
				sweepIndex = 0;
				detachInterrupt(digitalPinToInterrupt(trigPin));
			}
			else if (commandBuffer[3] == '7') // Set Sweep Type 0:Free run 1:External
			{
				if (commandBuffer[4] == '0') { sweepType = 0; }
				else if (commandBuffer[4] == '1') { sweepType = 1; }
			}

			if (isSweepOn)
			{
				TIMSK3 = 0;
				TCNT3 = 0;
				sweepIndex = 0;
				isExtModAttch = false;
				detachInterrupt(digitalPinToInterrupt(trigPin));
				setSweepParams();
			}

		}
		else if (commandBuffer[2] == 'F')
		{
			// RF ON OFF
			if (commandBuffer[3] == '0') { isRFOnOff = false; rfOnOff(); }
			else if (commandBuffer[3] == '1') { isRFOnOff = true; rfOnOff(); }
		}
		else if (commandBuffer[2] == 'E')
		{
			// EEPROM Settings
			switch (commandBuffer[3])
			{
			case 'S': eepromSave();  break;
			case 'C': eepromClear(); break;
			case 'L':
				// Load from EEPROM
				switch (commandBuffer[4])
				{
					case '0': writeToExEEPROM(REMEMBER_ADDR, 0); break;
					case '1': writeToExEEPROM(REMEMBER_ADDR, 1); break;
					default: eepromLoad(); break;
				}
				break;
			case 'A':
				strcat(commandInString, &commandBuffer[4]);
				eepromAddr = (uint16_t)get64Bit(commandInString);
				break;
			case 'R':
				Serial.println(readFromExEEPROM(eepromAddr));
				break;
			case 'W':
				strcat(commandInString, &commandBuffer[4]);
				writeToExEEPROM(eepromAddr, (uint8_t)get64Bit(commandInString));
				break;
			}
		}
		else if (commandBuffer[2] == 'V')
		{
			// Vibration
			switch (commandBuffer[3])
			{
			case '0': isVibrationOn = false; break;
			case '1': isVibrationOn = true; break;
			}
		}
	}
	else if (commandID == 'R')
	{
		if (commandBuffer[2] == 'C')
		{
			Serial.println(getCurrent(), 5);
		}
		else if (commandBuffer[2] == 'T')
		{
			Serial.println(getTemp(), 4);
		}
	}
	else if (commandID == 'G')
	{
		// LCD ReadBacks
		if (commandBuffer[2] == 'H')
		{
			strcpy(lcdCommand, "freqIN.txt=\"");
			uint64ToString(Frequency, tmp);
			strcat(lcdCommand, tmp);
			strcat(lcdCommand, "\"");
			sendLCDCommand(lcdCommand);

			if (amplitude < 0)
			{
				strcpy(lcdCommand, "ampIN.txt=\"-");
				uint64ToString((amplitude*(-1)), tmp);

			}
			else
			{
				strcpy(lcdCommand, "ampIN.txt=\"");
				uint64ToString(amplitude, tmp);
			}

			strcat(lcdCommand, tmp);
			strcat(lcdCommand, "\"");
			sendLCDCommand(lcdCommand);

			sendLCDCommand("unit1.val=0");

			if (isRFOnOff)
			{
				sendLCDCommand("b11.pic=99");
			}
			else
			{
				sendLCDCommand("b11.pic=98");
			}
		}
		else if (commandBuffer[2] == 'S')
		{
			if (sweepType == 0)
			{
				// FREE RUN
				sendLCDCommand("b4.pic=17");
				sendLCDCommand("b5.pic=67");
			}
			else if (sweepType == 1)
			{
				// External
				sendLCDCommand("b4.pic=66");
				sendLCDCommand("b5.pic=18");
			}
			
			if (isSweepOn)
			{
				sendLCDCommand("b6.pic=19");
				sendLCDCommand("b7.pic=69");
			}
			else
			{
				sendLCDCommand("b6.pic=68");
				sendLCDCommand("b7.pic=20");
			}
			
			strcpy(lcdCommand, "t5.txt=\"");
			uint64ToString(sweepStartFrequency, tmp);
			strcat(lcdCommand, tmp);
			strcat(lcdCommand, "\"");
			sendLCDCommand(lcdCommand);

			strcpy(lcdCommand, "t6.txt=\"");
			uint64ToString(sweepStopFrequency, tmp);
			strcat(lcdCommand, tmp);
			strcat(lcdCommand, "\"");
			sendLCDCommand(lcdCommand);

			strcpy(lcdCommand, "t7.txt=\"");
			uint64ToString(sweepStepFrequency, tmp);
			strcat(lcdCommand, tmp);
			strcat(lcdCommand, "\"");
			sendLCDCommand(lcdCommand);

			strcpy(lcdCommand, "t8.txt=\"");
			uint64ToString(sweepDwellTime, tmp);
			strcat(lcdCommand, tmp);
			strcat(lcdCommand, "\"");
			sendLCDCommand(lcdCommand);

			sendLCDCommand("unit1.val=0");
			sendLCDCommand("unit2.val=0");
			sendLCDCommand("unit3.val=0");
			sendLCDCommand("unit4.val=0");
		}
		else if (commandBuffer[2] == 'D')
		{
			float t = getTemp();

			Serial1.print("t0.txt=\"");
			Serial1.print(t, 1);
			Serial1.print("C / ");
			t *= (float)(9.0 / 5.0);
			t += 32;
			Serial1.print(t, 1);
			Serial1.print("F");
			Serial1.print("\"");
			Serial1.write(0xFF);
			Serial1.write(0xFF);
			Serial1.write(0xFF);

			int lmx = digitalRead(lmxRead);
			int adf = digitalRead(adfRead);

			Serial1.print("t3.txt=\"");
			if (lmx == LOW) { Serial1.print("Not Locked \""); }
			else { Serial1.print("Locked \""); }
			Serial1.write(0xFF);
			Serial1.write(0xFF);
			Serial1.write(0xFF);

			Serial1.print("t3.pco=");
			Serial1.print(lmx == LOW ? "63488" : "8002");
			Serial1.write(0xFF);
			Serial1.write(0xFF);
			Serial1.write(0xFF);

			Serial1.print("t2.txt=\"");
			if (adf == LOW) { Serial1.print("Not Locked \""); }
			else { Serial1.print("Locked \""); }
			Serial1.write(0xFF);
			Serial1.write(0xFF);
			Serial1.write(0xFF);

			Serial1.print("t2.pco=");
			Serial1.print(adf == LOW ? "63488" : "8002");
			Serial1.write(0xFF);
			Serial1.write(0xFF);
			Serial1.write(0xFF);

			float c = getCurrent();
			Serial1.print("t1.txt=\"5Vx");
			Serial1.print(c, 2);
			Serial1.print("A=");
			Serial1.print((c * 5), 2);
			Serial1.print("W\"");
			Serial1.write(0xFF);
			Serial1.write(0xFF);
			Serial1.write(0xFF);

			strcpy(lcdCommand, "t5.txt=\"");
			strcat(lcdCommand, firmwareVersion);
			strcat(lcdCommand, "\"");
			sendLCDCommand(lcdCommand);
		}
		else if (commandBuffer[2] == 'R')
		{
			switch (reference)
			{
				case 0: sendLCDCommand("b0.pic=15"); sendLCDCommand("b1.pic=65"); break;
				case 1: sendLCDCommand("b0.pic=64"); sendLCDCommand("b1.pic=16"); break;
			}
		}
		else if (commandBuffer[2] == 'M')
		{
			if (commandBuffer[3] == '0')
			{
				if (isModulationEnable)
				{
					sendLCDCommand("b0.pic=21");
					sendLCDCommand("b1.pic=71");
				}
				else
				{
					sendLCDCommand("b0.pic=70");
					sendLCDCommand("b1.pic=22");
				}
				
				if (modType == MODULATION_AM)
				{
					sendLCDCommand("b2.pic=26");
					sendLCDCommand("b3.pic=73");
					sendLCDCommand("b4.pic=74");
				}
				else if (modType == MODULATION_FM)
				{
					sendLCDCommand("b2.pic=72");
					sendLCDCommand("b3.pic=27");
					sendLCDCommand("b4.pic=74");
				}
				else if (modType == MODULATION_PULSE)
				{
					sendLCDCommand("b2.pic=72");
					sendLCDCommand("b3.pic=73");
					sendLCDCommand("b4.pic=28");
				}
			}
			else if (commandBuffer[3] == '1')
			{
				if (modInput == MODULATION_INTERNAL)
				{
					sendLCDCommand("b0.pic=23");
					sendLCDCommand("b1.pic=67");
					sendLCDCommand("b2.pic=75");
				}
				else if (modInput == MODULATION_EXTERNAL)
				{
					sendLCDCommand("b0.pic=64");
					sendLCDCommand("b1.pic=24");
					sendLCDCommand("b2.pic=75");
				}
				else if (modInput == MODULATION_MICROPHONE)
				{
					sendLCDCommand("b0.pic=64");
					sendLCDCommand("b1.pic=67");
					sendLCDCommand("b2.pic=25");
				}

				if (modWaveForm == MODULATION_SINE)
				{
					sendLCDCommand("b3.pic=29");
					sendLCDCommand("b5.pic=77");
					sendLCDCommand("b6.pic=78");
					sendLCDCommand("b4.pic=79");
				}
				else if (modWaveForm == MODULATION_RAMP)
				{
					sendLCDCommand("b3.pic=76");
					sendLCDCommand("b5.pic=30");
					sendLCDCommand("b6.pic=78");
					sendLCDCommand("b4.pic=79");
				}
				else if (modWaveForm == MODULATION_SQUARE)
				{
					sendLCDCommand("b3.pic=76");
					sendLCDCommand("b5.pic=77");
					sendLCDCommand("b6.pic=32");
					sendLCDCommand("b4.pic=79");
				}
				else if (modWaveForm == MODULATION_TRIANGLE)
				{
					sendLCDCommand("b3.pic=76");
					sendLCDCommand("b5.pic=77");
					sendLCDCommand("b6.pic=78");
					sendLCDCommand("b4.pic=31");
				}


				strcpy(lcdCommand, "t2.txt=\"");
				uint64ToString(modIntFreq, tmp);
				strcat(lcdCommand, tmp);
				strcat(lcdCommand, "\"");
				sendLCDCommand(lcdCommand);

				sendLCDCommand("unit1.val=0");

				strcpy(lcdCommand, "t4.txt=\"");
				uint64ToString(amDepth, tmp);
				strcat(lcdCommand, tmp);
				strcat(lcdCommand, "\"");
				sendLCDCommand(lcdCommand);

			}
			else if (commandBuffer[3] == '2')
			{
				if (modInput == MODULATION_INTERNAL)
				{
					sendLCDCommand("b0.pic=23");
					sendLCDCommand("b1.pic=67");
					sendLCDCommand("b2.pic=75");
				}
				else if (modInput == MODULATION_EXTERNAL)
				{
					sendLCDCommand("b0.pic=64");
					sendLCDCommand("b1.pic=24");
					sendLCDCommand("b2.pic=75");
				}
				else if (modInput == MODULATION_MICROPHONE)
				{
					sendLCDCommand("b0.pic=64");
					sendLCDCommand("b1.pic=67");
					sendLCDCommand("b2.pic=25");
				}

				if (modWaveForm == MODULATION_SINE)
				{
					sendLCDCommand("b3.pic=29");
					sendLCDCommand("b5.pic=77");
					sendLCDCommand("b6.pic=78");
					sendLCDCommand("b4.pic=79");
				}
				else if (modWaveForm == MODULATION_RAMP)
				{
					sendLCDCommand("b3.pic=76");
					sendLCDCommand("b5.pic=30");
					sendLCDCommand("b6.pic=78");
					sendLCDCommand("b4.pic=79");
				}
				else if (modWaveForm == MODULATION_SQUARE)
				{
					sendLCDCommand("b3.pic=76");
					sendLCDCommand("b5.pic=77");
					sendLCDCommand("b6.pic=32");
					sendLCDCommand("b4.pic=79");
				}
				else if (modWaveForm == MODULATION_TRIANGLE)
				{
					sendLCDCommand("b3.pic=76");
					sendLCDCommand("b5.pic=77");
					sendLCDCommand("b6.pic=78");
					sendLCDCommand("b4.pic=31");
				}

				strcpy(lcdCommand, "t2.txt=\"");
				uint64ToString(modIntFreq, tmp);
				strcat(lcdCommand, tmp);
				strcat(lcdCommand, "\"");
				sendLCDCommand(lcdCommand);

				sendLCDCommand("unit1.val=0");

				strcpy(lcdCommand, "t4.txt=\"");
				uint64ToString(fmDev, tmp);
				strcat(lcdCommand, tmp);
				strcat(lcdCommand, "\"");
				sendLCDCommand(lcdCommand);

				sendLCDCommand("unit2.val=0");

			}
			else if (commandBuffer[3] == '3')
			{
				if (modInput == MODULATION_INTERNAL)
				{
					sendLCDCommand("b0.pic=23");
					sendLCDCommand("b1.pic=67");
				}
				else if (modInput == MODULATION_EXTERNAL)
				{
					sendLCDCommand("b0.pic=64");
					sendLCDCommand("b1.pic=24");
				}

				strcpy(lcdCommand, "t2.txt=\"");
				uint64ToString(pulsePeriod, tmp);
				strcat(lcdCommand, tmp);
				strcat(lcdCommand, "\"");
				sendLCDCommand(lcdCommand);

				sendLCDCommand("unit1.val=0");

				strcpy(lcdCommand, "t4.txt=\"");
				uint64ToString(pulseWidth, tmp);
				strcat(lcdCommand, tmp);
				strcat(lcdCommand, "\"");
				sendLCDCommand(lcdCommand);

				sendLCDCommand("unit2.val=0");
			}
		}
		else if (commandBuffer[2] == 'E')
		{
			if (readFromExEEPROM(REMEMBER_ADDR) == 0x01)
			{
				sendLCDCommand("b3.pic=122");
				sendLCDCommand("b2.pic=124");
			}
			else
			{
				sendLCDCommand("b3.pic=121");
				sendLCDCommand("b2.pic=123");
			}

			if (isVibrationOn)
			{
				sendLCDCommand("b4.pic=95");
			}
			else
			{
				sendLCDCommand("b4.pic=96");
			}
		}
		else if (commandBuffer[2] == 'V')
		{
			vibrate();
		}
	}
	else if (commandID == 'P')
	{
		command(">SR0");  // Internal Reference
		command(">SS70"); // Sweep Free Run
		command(">SS6");  // Sweep Stopped
		command(">SS11000000000"); // Sweep Start Freq 1GHz
		command(">SS22000000000"); // Sweep Stop Freq 2 GHz
		command(">SS3100000000"); // Sweep Step Freq 100 MHz
		command(">SS410000"); // Sweep Dwell time 10 ms
		command(">SM0"); // Modulation is OFF
		command(">SMT0"); // Modulation type is AM
		command(">SMI0"); // Modulation input is Internal
		command(">SMW0"); // Modulation waveform is Sine
		command(">SMF1000"); // Modulation Internal Frequency is 1KHz
		command(">SMFD50000"); // Modulation FM Deviation 50KHz
		command(">SMA99"); // AM Depth is 99%
		command(">SMPP10000"); // Pulse Period is 10ms
		command(">SMPW5000"); // Pulse Width is 5ms
		command(">SEL0"); // Remember last settings on start is off
		command(">SV1"); // Vibration is ON
		command(">F1000000000"); // 1GHZ
		command(">SA0");  // 0dBm
		command(">SF1");  // RF ON
		command(">GH"); // send home settings
	}
	else if (commandID == 'X')
	{
		Serial.println("Upload is active");
		isUploadCode = true;
		Serial.begin(250000);
		Serial1.begin(9600);
		delay(2000);
		sendLCDCommand("baud=115200");
		delay(2000);
		Serial1.begin(115200);
	}
}

void sendLCDCommand(char* input)
{
	Serial1.write(input, strlen(input));
	Serial1.write(0xFF);
	Serial1.write(0xFF);
	Serial1.write(0xFF);
	delay(5);
}

void vibrate()
{
	if (isVibrationOn)
	{
		digitalWrite(vibMotor, LOW);
		delay(30);
		digitalWrite(vibMotor, HIGH);
	}
}

void setAmplitude()
{
	if (amplitude < -50) { amplitude = -50; }
	if (amplitude > 15) { amplitude = 15; }

	float f1 = Frequency;

	f1 /= 1e7;
	f1 = round(f1);
	f1--;

	uint16_t lmxAddr = 0;
	uint16_t f2250Addr = 0;
	uint8_t rfsa = 0;

	if (amplitude == 15)
	{
		lmxAddr = LMX_CAL_15;
		f2250Addr = F2250_CAL_15;
	}
	else if (amplitude == 14)
	{
		lmxAddr = LMX_CAL_15;
		f2250Addr = F2250_CAL_14;
	}
	else if (amplitude == 13)
	{
		lmxAddr = LMX_CAL_15;
		f2250Addr = F2250_CAL_13;
	}
	else if (amplitude == 12)
	{
		lmxAddr = LMX_CAL_15;
		f2250Addr = F2250_CAL_12;
	}
	else if (amplitude == 11)
	{
		lmxAddr = LMX_CAL_15;
		f2250Addr = F2250_CAL_11;
	}
	else if (amplitude == 10)
	{
		lmxAddr = LMX_CAL_15;
		f2250Addr = F2250_CAL_10;
	}
	else if (amplitude < 10 & amplitude > -21)
	{
		lmxAddr = LMX_CAL_15;
		f2250Addr = F2250_CAL_10;
		rfsa = ((amplitude - 10)*(-4));
	}
	else if (amplitude == -21)
	{
		lmxAddr = LMX_CAL_21;
		f2250Addr = F2250_CAL_21;
	}
	else if (amplitude < -21 & amplitude >= -50)
	{
		lmxAddr = LMX_CAL_21;
		f2250Addr = F2250_CAL_21;
		rfsa = ((amplitude + 21) * (-4));
	}

	uint16_t addr = f2250Addr + ((uint16_t)f1 * 2);

	LMXOutPwr = readFromExEEPROM(lmxAddr + (uint16_t)f1);

	uint16_t f2250 = readFromExEEPROM(addr + 1) << 8 | readFromExEEPROM(addr);

	setLMXPower();
	setF2250(f2250);
	setRFSA3714(rfsa);
}

void setSweepParams()
{
	if (sweepStopFrequency < sweepStartFrequency)
	{
		sweepPoints = (sweepStartFrequency - sweepStopFrequency) / sweepStepFrequency;
	}
	else
	{
		sweepPoints = (sweepStopFrequency - sweepStartFrequency) / sweepStepFrequency;
	}

	if (isModulationEnable && modType == MODULATION_PULSE && modInput == MODULATION_EXTERNAL) 
	{
		if (!isExtModAttch) { attachInterrupt(digitalPinToInterrupt(trigPin), pulse_rising, RISING); isExtModAttch = true; }
	}

	if (sweepType == 0)
	{
		TCCR3B = _BV(WGM33);
		TCCR3A = 0;
		if (sweepDwellTime < 8192) { TCCR3B |= _BV(CS30); ICR3 = sweepDwellTime * 8; }
		else if (sweepDwellTime < 65536) { TCCR3B |= _BV(CS31); ICR3 = sweepDwellTime; }
		else if (sweepDwellTime < 524288) { TCCR3B |= _BV(CS31) | _BV(CS30); ICR3 = sweepDwellTime / 8; }
		else if (sweepDwellTime < 2097152) { TCCR3B |= _BV(CS32); ICR3 = sweepDwellTime / 32; }
		else if (sweepDwellTime < 8388608) { TCCR3B |= _BV(CS32) | _BV(CS30); ICR3 = sweepDwellTime / 128; }
		else { TCCR3B |= _BV(CS32) | _BV(CS30);  ICR3 = 65535; }
		TIMSK3 = _BV(TOIE3);
	}
	else
	{
		attachInterrupt(digitalPinToInterrupt(trigPin), sweepInterrupt, RISING);
	}
}

void sweepInterrupt()
{
	if (isModulationEnable && !(modType == MODULATION_PULSE && modInput == MODULATION_EXTERNAL) ) { nextFreq = true; }
	else { sweep(); }
}

void sweep()
{
	cli();
	nextFreq = false;
	setLMX(sweepStartFrequency + (sweepStepFrequency * sweepIndex));
	if (sweepStartFrequency > sweepStopFrequency)
	{
		sweepIndex--;
		if (abs(sweepIndex) > sweepPoints || (sweepStepFrequency > abs(sweepStartFrequency- sweepStopFrequency))) { sweepIndex = 0; }
	}
	else
	{
		sweepIndex++;
		if (sweepIndex > sweepPoints || (sweepStepFrequency > abs(sweepStartFrequency - sweepStopFrequency))) { sweepIndex = 0; }
	}
	sei();
}

ISR(TIMER1_OVF_vect)
{
	blinkLed();
}

ISR(TIMER3_OVF_vect)
{
	sweepInterrupt();
}

void eepromLoad()
{
	uint16_t address = SETTINGS_ADDR;

	// 1 byte
	isModulationEnable = readFromExEEPROM(address); address++;
	isSweepOn = readFromExEEPROM(address); address++;
	isRFOnOff = readFromExEEPROM(address); address++;
	isVibrationOn = readFromExEEPROM(address); address++;
	modType = readFromExEEPROM(address); address++;
	modInput = readFromExEEPROM(address); address++;
	modWaveForm = readFromExEEPROM(address); address++;
	amDepth = readFromExEEPROM(address); address++;
	sweepType = readFromExEEPROM(address); address++;
	reference = readFromExEEPROM(address); address++;

	// 2 byte
	recoverLocals((uint8_t*)&sweepIndex, 2, address); address += 2;
	recoverLocals((uint8_t*)&amplitude, 2, address); address += 2;

	// 4 byte
	recoverLocals((uint8_t*)&modDelayVal, 4, address);		address += 4;
	recoverLocals((uint8_t*)&modIntFreq, 4, address);		address += 4;
	recoverLocals((uint8_t*)&fmDev, 4, address);			address += 4;
	recoverLocals((uint8_t*)&pulseWidth, 4, address);		address += 4;
	recoverLocals((uint8_t*)&pulsePeriod, 4, address);		address += 4;
	recoverLocals((uint8_t*)&sweepDwellTime, 4, address);	address += 4;

	// 8 byte
	recoverLocals((uint8_t*)&Frequency, 8, address); address += 8;
	recoverLocals((uint8_t*)&sweepStartFrequency, 8, address); address += 8;
	recoverLocals((uint8_t*)&sweepStopFrequency, 8, address); address += 8;
	recoverLocals((uint8_t*)&sweepStepFrequency, 8, address); address += 8;
	recoverLocals((uint8_t*)&sweepPoints, 8, address); address += 8;

	setReferenceType(reference);
	setAmplitude();
	setLMX(Frequency);
	rfOnOff();

	if (isSweepOn) { setSweepParams(); }
	if (isModulationEnable) { setModulationSettings(); }
}

void recoverLocals(uint8_t* ptr, uint8_t size, uint16_t addr)
{
	for (uint8_t i = 0; i < size; i++)
	{
		*ptr = readFromExEEPROM(addr);
		ptr++; addr++;
	}
}

void eepromSave()
{
	uint16_t address = SETTINGS_ADDR;

	// 1 byte
	writeToExEEPROM(address, isModulationEnable); address++;
	writeToExEEPROM(address, isSweepOn); address++;
	writeToExEEPROM(address, isRFOnOff); address++;
	writeToExEEPROM(address, isVibrationOn); address++;
	writeToExEEPROM(address, modType); address++;
	writeToExEEPROM(address, modInput); address++;
	writeToExEEPROM(address, modWaveForm); address++;
	writeToExEEPROM(address, amDepth); address++;
	writeToExEEPROM(address, sweepType); address++;
	writeToExEEPROM(address, reference); address++;

	// 2 byte
	for (uint8_t i = 0; i < 2; i++) { writeToExEEPROM(address, (uint8_t)(sweepIndex >> (8 * i))); address++; }
	for (uint8_t i = 0; i < 2; i++) { writeToExEEPROM(address, (uint8_t)(amplitude >> (8 * i))); address++; }

	// 4 byte
	for (uint8_t i = 0; i < 4; i++) { writeToExEEPROM(address, (uint8_t)(modDelayVal >> (8 * i))); address++; }
	for (uint8_t i = 0; i < 4; i++) { writeToExEEPROM(address, (uint8_t)(modIntFreq >> (8 * i))); address++; }
	for (uint8_t i = 0; i < 4; i++) { writeToExEEPROM(address, (uint8_t)(fmDev >> (8 * i))); address++; }
	for (uint8_t i = 0; i < 4; i++) { writeToExEEPROM(address, (uint8_t)(pulseWidth >> (8 * i))); address++; }
	for (uint8_t i = 0; i < 4; i++) { writeToExEEPROM(address, (uint8_t)(pulsePeriod >> (8 * i))); address++; }
	for (uint8_t i = 0; i < 4; i++) { writeToExEEPROM(address, (uint8_t)(sweepDwellTime >> (8 * i))); address++; }

	// 8 byte
	for (uint8_t i = 0; i < 8; i++) { writeToExEEPROM(address, (uint8_t)(Frequency >> (8 * i))); address++; }
	for (uint8_t i = 0; i < 8; i++) { writeToExEEPROM(address, (uint8_t)(sweepStartFrequency >> (8 * i))); address++; }
	for (uint8_t i = 0; i < 8; i++) { writeToExEEPROM(address, (uint8_t)(sweepStopFrequency >> (8 * i))); address++; }
	for (uint8_t i = 0; i < 8; i++) { writeToExEEPROM(address, (uint8_t)(sweepStepFrequency >> (8 * i))); address++; }
	for (uint8_t i = 0; i < 8; i++) { writeToExEEPROM(address, (uint8_t)(sweepPoints >> (8 * i))); address++; }

}

void eepromClear()
{
	for (uint16_t i = SETTINGS_ADDR; i < SETTINGS_ADDR + SETTINGS_LENGTH; i++) { writeToExEEPROM(i, 0); }
}

uint8_t readFromExEEPROM(uint16_t address)
{
	cli();
	SPI.beginTransaction(SPISettings(EXTERNAL_EEPROM_MAX_CLOCK, MSBFIRST, SPI_MODE0));
	digitalWrite(eeprom_LE, LOW);
	SPI.transfer(EXTERNAL_EEPROM_READ);
	SPI.transfer((uint8_t)(address >> 8));
	SPI.transfer((uint8_t)(address));
	uint8_t a1 = SPI.transfer(0x00);
	digitalWrite(eeprom_LE, HIGH);
	SPI.endTransaction();
	sei();
	return a1;
}

void readFromExEEPROM(uint16_t address, uint8_t *ptr)
{
	cli();
	SPI.beginTransaction(SPISettings(EXTERNAL_EEPROM_MAX_CLOCK, MSBFIRST, SPI_MODE0));
	digitalWrite(eeprom_LE, LOW);
	SPI.transfer(EXTERNAL_EEPROM_READ);
	SPI.transfer((uint8_t)(address >> 8));
	SPI.transfer((uint8_t)(address));
	*ptr = SPI.transfer(0x00); ptr++;
	*ptr = SPI.transfer(0x00); ptr++;
	*ptr = SPI.transfer(0x00);
	digitalWrite(eeprom_LE, HIGH);
	SPI.endTransaction();
	sei();
}

void writeToExEEPROM(uint16_t address, uint8_t data)
{
	cli();
	SPI.beginTransaction(SPISettings(EXTERNAL_EEPROM_MAX_CLOCK, MSBFIRST, SPI_MODE0));
	// SEND WRITE ENABLE FIRST
	digitalWrite(eeprom_LE, LOW);
	SPI.transfer(EXTERNAL_EEPROM_WREN);
	digitalWrite(eeprom_LE, HIGH);

	digitalWrite(eeprom_LE, LOW);
	SPI.transfer(EXTERNAL_EEPROM_WRITE);
	SPI.transfer((uint8_t)(address >> 8));
	SPI.transfer((uint8_t)(address));
	SPI.transfer(data);
	digitalWrite(eeprom_LE, HIGH);
	sei();
	delay(10);
}
