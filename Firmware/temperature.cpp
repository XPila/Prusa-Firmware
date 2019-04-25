#include "Marlin.h"
#include <avr/wdt.h>
#include "adc.h"

int target_temperature[EXTRUDERS] = { 0 };
int target_temperature_bed = 0;
float current_temperature[EXTRUDERS] = { 20.0F };
float current_temperature_bed = 20.0F;
float current_temperature_pinda = 0.0;
float current_temperature_ambient = 0.0;

float Kp=DEFAULT_Kp;
float Ki=(DEFAULT_Ki*PID_dT);
float Kd=(DEFAULT_Kd/PID_dT);
float Kc=DEFAULT_Kc;

float bedKp=DEFAULT_bedKp;
float bedKi=(DEFAULT_bedKi*PID_dT);
float bedKd=(DEFAULT_bedKd/PID_dT);

float _Kp, _Ki, _Kd;
int pid_cycle, pid_number_of_cycles;
bool pid_tuning_finished = false;

volatile int babystepsTodo[3]={0,0,0};

int current_voltage_raw_pwr = 0;

int current_voltage_raw_bed = 0;


void manage_heater()
{
#ifdef WATCHDOG
    wdt_reset();
#endif //WATCHDOG
}

void setWatch() 
{  
}

int getHeaterPower(int heater)
{
}

void disable_heater()
{
}

float scalePID_i(float i)
{
}

float unscalePID_i(float i)
{
}

float scalePID_d(float d)
{
}

float unscalePID_d(float d)
{
}

void updatePID()
{
}

void PID_autotune(float temp, int extruder, int ncycles)
{
}

void tp_init()
{
	adc_init();
#ifdef SYSTEM_TIMER_2
  timer02_init();
  OCR2B = 128;
  TIMSK2 |= (1<<OCIE2B);  
#else //SYSTEM_TIMER_2
  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
  TIMSK0 |= (1<<OCIE0B);  
#endif //SYSTEM_TIMER_2
}

void setExtruderAutoFanState(int pin, bool state)
{
}

#ifdef SYSTEM_TIMER_2
ISR(TIMER2_COMPB_vect)
#else //SYSTEM_TIMER_2
ISR(TIMER0_COMPB_vect)
#endif //SYSTEM_TIMER_2
{
	adc_cycle();
	current_temperature[0] = target_temperature[0];
	current_temperature_bed = target_temperature_bed;
}


extern "C" {

void adc_ready(void) //callback from adc when sampling finished
{
}

}