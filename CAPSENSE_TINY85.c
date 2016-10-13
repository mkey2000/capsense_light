/*
 * CAPSENSE_TINY85.c
 * ATTNY85
 *
 * Created: 10.12.2015 21:04:34
 * Egert Loss
 *
 * Capsense from http://blog.aleksander.kaweczynski.pl/touch-sensor-on-avr-done-without-qtouch-library/ 
 * Millis() from https://github.com/sourceperl/millis/blob/master/millis.c
 */ 

#define F_CPU 8000000
#define SPIN PB3			//Send pin, trigger
#define RPIN PB4			//receive pin, sensor/antenna
#define LED PB1				//diagnostic LED
#define STRIP PB0			//LED strip
#define NUMREADS 20			//how many capsense readings
#define RECAL 300000		//5Minutes = 5*60*1000
#define SENSORREFRESH 20	//sensor check time in milliseconds. !!!! 25 flickers?!
#define EXTRATHRES 0

#define NOOP asm volatile("nop" ::)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//declarations
void adc_setup(void);		
void timer0_setup(void);			//for millis()
void ioinit(void);
uint16_t getNumReadings(uint16_t samples);
uint32_t getCapSenseReading(void);	//get one readings
uint32_t calibrate(void);			//get max sensor value for number of readings
uint64_t millis(void);				//get ms from last reset

uint64_t currentMillis = 0;				//current time
uint64_t previousCalMillis = 0;			//calibration time
uint64_t previousSensorMillis = 0;		//sensor time
volatile uint64_t _millis = 0;
volatile uint16_t _1000us = 0;

int main(void)
{
	ioinit();								
	adc_setup();	
	timer0_setup();
	
	sei();
	
	uint32_t threshold = calibrate();		//get baseline sensor reading
	uint16_t sensorRaw = getNumReadings(NUMREADS);
	uint16_t currentState = (sensorRaw > (threshold + ADCH));
	uint16_t previousState = currentState;
	
	//blink to let know it's ready
	PORTB ^= (1 << LED);
	_delay_ms(200);
	PORTB ^= (1 << LED);
	_delay_ms(200);
	PORTB ^= (1 << LED);
	_delay_ms(200);
	PORTB ^= (1 << LED);
		
    while(1)
    {		
		currentMillis = millis();				//get current time
													
		if(currentMillis - previousSensorMillis >= SENSORREFRESH)	
		{
				previousSensorMillis = currentMillis;
				
				sensorRaw = getNumReadings(NUMREADS);
				currentState = (sensorRaw > (EXTRATHRES + threshold + (ADCH*2)));		//is the sensor active?
				if(currentState > previousState)
				{
					PORTB ^= (1 << LED);
					PORTB ^= (1 << STRIP);
				} //end if
				previousState = currentState;	
		}//end if
		
		if(currentMillis - previousCalMillis >= RECAL)			//is it time to recalibrate?
		{
			previousCalMillis = currentMillis;					//save the time
			if(!currentState && (sensorRaw * 1.2 < threshold) && (sensorRaw * 0.8 > threshold))	//If sensor isn't touched
			{
				threshold = calibrate();
			} //end if		
		}//end if
    }//end while
}//end main

void ioinit(void)
{
	DDRB |= (1 << LED) | (1 << STRIP) | (1 << SPIN);;		//LED, strip and sendpin ouputs
	MCUCR |= (1<<PUD);										//Pullups disabled
}

void timer0_setup(void)				//Setup timer0 to OVF at 0.256ms, used for timing
{
	/*
	8000000Mhz / 8 = 1000000Mhz
	1/ (1000000Mhz / (255) - 1) = 0.256ms
	*/
	TCCR0A = 0;	
	TCCR0B = 0;				//Clear timer registers
	TCCR0B |= (1 << CS01);	//Prescaler 8
	TIMSK |= (1 << TOIE0);	//Enable OVF intterupt	
}


void adc_setup(void)			//Setup adc to read PB2
{
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1);	//ADC PRESCALER 8000000 / 64 = 125kHz
	
	ADMUX = 0;
	ADMUX |= (1 << ADLAR);					//Left adjust
	ADMUX |= (1 << MUX0);					//select PB02
	DIDR0 = 0;
	//default is freerunning
	ADCSRA |= (1 << ADEN) | (1 << ADATE);	//start adc, default is Freerunning mode, check ADCSRB, auto trigger enable
	ADCSRA |= (1 << ADSC);					//start first conversion
}

uint16_t getNumReadings(uint16_t samples)	//Reads capsense for specified times and returns average of them
{
	uint32_t sumTotal = 0;	
	for(uint16_t counter = 0; counter < samples; counter++)
	{
		sumTotal += getCapSenseReading();
	} //end for
	return (sumTotal / samples);
}

uint32_t getCapSenseReading(void)			//Gets one reading from sensor
{
	cli();
	uint32_t i = 0;
	while(PORTB & (1 << RPIN));	//wait for discharge sensor pin
	DDRB |= (1 << SPIN);			//set sensor pin as output, dicharge it to end
	NOOP;
	NOOP;
	NOOP;
	DDRB &= ~(1 << SPIN);		//set "charger" as input, i third state (as disconnected)
	DDRB &= ~(1 << RPIN);		//set sensor pin as input,
	DDRB |= (1 << SPIN);		//now "plug in the charger"
	PORTB |= (1 << SPIN);		//start charging sensor
	
	while(!(PINB & (1 << RPIN))) //increment counter
	{							//if the pin is in high state
		i++;
	}
	PORTB &= ~(1 << SPIN);			//stop charging
	sei();
	return i;
}

uint32_t calibrate(void)					//Finds the maximum sensor reading during calibration
{
	uint32_t maximum = 0;
	uint32_t tempMaximum = 0;
	
	for(uint8_t j = 0; j < 100; j++)
	{
		tempMaximum = getNumReadings(NUMREADS);
		if(tempMaximum > maximum)
		{
			maximum = tempMaximum;
		}
	}
	return maximum;
}

uint64_t millis(void)						//safe access to millis
{
	uint64_t count;
	cli();
	count = _millis;
	sei();
	return count;	
}

ISR(TIMER0_OVF_vect)
{
	_1000us += 256;
	while(_1000us > 1000)
	{
		_millis++;
		_1000us -= 1000;
	}
}
