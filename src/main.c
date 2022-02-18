#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "../lib/liquid_crystal_i2c/liquid_crystal_i2c.h"
#include "../lib/i2c_master/i2c_master.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define BATTERY_RAW PORTC0;
#define BATTERY_SHUNT PORTC1; 

#define DATAPOINT0_X 4.2
#define DATAPOINT0_Y 100

#define DATAPOINT1_X 3.35
#define DATAPOINT1_Y 50

#define DATAPOINT2_X 3.0
#define DATAPOINT2_Y 0

#define POWERBUTTON PORTD2
#define VOLTAGEREF 5 // reference voltage

#define OVERFLOW_VECT 5

void initADC();
void initTimer1();
void initINT0();
void readADC();
void initLCD();
float convertLevelToVoltage(uint16_t);
float estimateBatteryPercentage(float);

volatile uint8_t powerState = 0; // 0 OFF, 1 ON
volatile uint8_t overflowTick = 0; 
volatile uint8_t byteH;
volatile uint8_t byteL;
volatile uint16_t currentADCRead;

LiquidCrystalDevice_t device;

float calculateL0(float estP, float t0, float t1, float t2, float t0dep);
float calculateL1(float estP, float t0, float t1, float t2, float t1dep);
float calculateL2(float estP, float t0, float t1, float t2, float t2dep);

ISR(INT0_vect){
    _delay_ms(5);
    if(powerState == 0){
        powerState = 1;
    } else if(powerState == 1){
        powerState = 0;
    }
}

int main(){
    initADC();
    initINT0();
    initLCD();
    sei();
    // DEBUGGING
    DDRD |= (1 << PORTD4);
    char* convReading = malloc(sizeof(char)*20);
    char* percentConvReading = malloc(sizeof(char)*20);
    float batteryPercent = 24;
    float voltageReading = 0; 
    uint16_t _currentADCRead;
    while(1){
        if(powerState == 1){
            readADC();
            _currentADCRead = currentADCRead;
            voltageReading = convertLevelToVoltage(_currentADCRead);
            batteryPercent = estimateBatteryPercentage(voltageReading);
            uint8_t voltageWhole = (uint8_t) voltageReading;
            uint8_t voltageDecimal = (uint8_t) (voltageReading * 100) % 100; 
            sprintf(convReading, "VOLTAGE:%u.%u", voltageWhole, voltageDecimal);
            lq_setCursor(&device, 0, 0);
            lq_print(&device, convReading);

            uint8_t batteryWhole = (uint8_t) truncf(batteryPercent);
            //uint8_t batteryDecimal = (uint8_t) (batteryPercent * 100) % 100; 

            sprintf(percentConvReading, "PERCENT:%u", batteryWhole);
            lq_setCursor(&device, 1, 0);
            lq_print(&device, percentConvReading);

            _delay_ms(1000);
            lq_clear(&device);
        } else {
            lq_setCursor(&device, 0, 0);
            lq_print(&device, "STANDBY");
        }
    }
    return 0;
}

void initADC(){
    PRR &= ~(1 << PRADC); // ensure ADC power bit is disabled
    DDRC &= ~(1 << PORTC1);
    ADMUX |= (0b01 << REFS0) | (0b0001 << MUX0);
    ADCSRA |= (1 << ADEN) | (1 << ADSC) | (0b011 << ADPS0); 
}

// INT0 will start a read. 
void initINT0(){
    DDRD &= ~(1 << POWERBUTTON);
    PORTD |= (1 << POWERBUTTON); // enable pullup resistor
    EICRA |= (0b10 << ISC00);
    EIMSK |= (1 << INT0);
}

// assumes max 5V
float convertLevelToVoltage(uint16_t level){
    float voltage = (5.0*level)/(1024.0);
    return voltage;
}

void initLCD(){
    i2c_master_init(100000L); // set clock frequency to 100 kHz
    device = lq_init(0x27, 20, 4, LCD_5x8DOTS); // intialize 4-lines display
    lq_turnOnBacklight(&device); // simply turning on the backlight
}

// estimation battery taken from quadratic lagrange interpolation
float estimateBatteryPercentage(float voltage){
    float L0; 
    float L1;
    float L2;
    float interp;

    L0 = calculateL0(voltage, DATAPOINT0_X, DATAPOINT1_X, DATAPOINT2_X, DATAPOINT0_Y);
    L1 = calculateL1(voltage, DATAPOINT0_X, DATAPOINT1_X, DATAPOINT2_X, DATAPOINT1_Y);
    L2 = calculateL2(voltage, DATAPOINT0_X, DATAPOINT1_X, DATAPOINT2_X, DATAPOINT2_Y);

    interp = L0 + L1 + L2;
    return interp;
}

float calculateL0(float estP, float t0, float t1, float t2, float t0dep){
    float num = ((estP - t1)/(t0 - t1)*((estP - t2)/(t0 - t2)))*t0dep;
    return num; 
}

float calculateL1(float estP, float t0, float t1, float t2, float t1dep){
    float num = ((estP - t0)/(t1 - t0)*((estP - t2)/(t1 - t2)))*t1dep;
    return num; 
}

float calculateL2(float estP, float t0, float t1, float t2, float t2dep){
    float num = ((estP - t0)/(t2 - t0)*((estP - t1)/(t2 - t1)))*t2dep;
    return num; 
}

void readADC(){
    ADCSRA |= (1 << ADSC); 
    while(ADCSRA & (1 << ADSC)); // wait for conversion to finish 
    byteL = ADCL; 
    byteH = ADCH; 
    uint16_t byteBuffer = (byteH << 8) | byteL;
    currentADCRead = byteBuffer;
}