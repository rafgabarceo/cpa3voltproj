#include <avr/io.h>
#include <avr/interrupt.h>

#define BATTERY_RAW PORTC0;
#define BATTERY_SHUNT PORTC1; 

ISR(INT0_vect){

}

void initADC();
void initTimer1();
void initINT0();
void readShunt();
void readBattery();
float* estimateBatteryPercentage();


int main(){
    while(1){

    }
    return 0;
}

void initADC(){
    PRR &= ~(1 << PRADC); // ensure ADC power bit is disabled
    ADMUX |= (0b11 << REFS0) | (0b0000 << MUX0); // utilize internal 1.1V as reference. ADC0 as input.
    ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE) | (1 << ADIE) | (0b011 << ADPS0); 
    ADCSRB |= (0b110 << ADTS0); // new reading after every overflow of Timer1.
}

void initTimer1(){
    PRR &= ~(1 << PRTIM1);
    TCCR1A |= (0b00 << COM1A0); 
    TCCR1B |= (0b110 << CS10); // ENABLE 1024 PRESCALER
}

void initINT0(){

}

void readShunt(){

}

void readBattery(){

}

float* estimateBatteryPercentage(){
    
}
