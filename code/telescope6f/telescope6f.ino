
/*

  
 Mark I Stepper Motor Control - for Celstron G4 dual axis drive

 TASK
 -----
 Drive two stepper motors on the G4 dual axis telescope drive.
 One motor drives the Right ascension (RA), the other the  Declination (DEC).
 The RA is the celestial longitude. The Earth rotates across its RA at 15deg hour
 Thus the RA drive must run continuously for tracking purposes, that is compensating
 for the Earth's rotation. Fast movement must also be available for location of
 astronomical objects, be must revert to normal tracking speed.
 
 Declination is the  equivalent of astronomical latitude. Movement across this axis
 is used for locating astronomical objects. This then is only required when locating.

 Hardware
 --------
 An Arduino UNO was used for this task in conjunction with the DRV8834 stepper
 motor driver. This utilises the three counter timers on the ATMega328P used by the 
 Arduino UNO for the motor drive. The Ardunio UNO is used as supplied. No changes
 have been made to any fuse settings or original bootloader.

 Software 
 ---------
 Timer 1 on the ATMega328P is a 16 bit timer and can be used  to generate interrupts.
 Some stepper motor control applications have used this interrupt facility.
 Timers 0 and 2 are used for the Arduino delay and tone functions respectively.
 For this task all the timers will be used in a non interrupt mode as pulse generators,
 using their PWM capabilites.  This necessitates this use of direct register control and
 avoiding the use of any Arduino delay function.

 Solution
 ----------
 Timer 1 will be used for driving the RA drive. For normal tracking at 15deg hour the
 DRV8834 in full step mode requires a 110ms ( 9.090906...Hz) square wave pulse. For
 locating a x8 pulse is used. These are obtained from the 16Mhz clock via the prescalar
 and divider in compare match mode ( details below).

 The DEC drive requires a 3Hz ( 333mS ) pulse to the DRV8834 for a sidereal 15deg hour rate.
 Individually timers 0 and 2 cannot be used to generated a 3Hz output pulse for the DEC drive
 with the standard fuse settings, however they can be used in conjunction with each other.
 Timer 2 output OC2A is fed into timer 0 input TO and this can generate the range of pulses required.
 
 
 
Note x1 speed refered to as 'Normal' is the sidereal rotation rate of 15deg hour.
 


 */

int led_toggle =0;
unsigned char last_control_status = 0x00;
const unsigned char RA_SLEEP_CONTROL = PB4;         // PIN12  PB4 -> DRV8834 Sleep
const unsigned char RA_DIRECTION_CONTROL = PB2;     // PIN10  PB2 -> DRV8834 Direction

const unsigned char DEC_SLEEP_CONTROL = PD7;        // PIN13  PD7 -> DRV8834 Sleep
const unsigned char DEC_DIRECTION_CONTROL = PD3;    // PIN3   PD3 -> DRV8834 Direction



//** setting for x1 sidreal tracking
const int SIDREAL_RA_ICR1 = 1736;  //** 16 bit counter 1736 for 9.0909Hz
const int SIDREAL_RA_OCR1A =  868; //** OCR1A = 977 50:50 9.0909Hz

const int DEC_SPEED_STANDARD = 83; //** for normal 15 deg hour


void setRAForward( unsigned char speed_select );
void setRAReverse( unsigned char speed_select );
void setDecUp(unsigned char speed_select );
void setDecDown(unsigned char speed_select );
void setDECClock( unsigned char speed_select );
void disableDECClock( void );

void setRANormal( void );


void setup()
{

   Serial.begin(9600);

  //** Timers
  
  //** RA Drive
  //** used OC1A timer 1 op (PIN 9 -> PB1 ) for RA drive

  //** DEC Drive 
  //** OC2A timer 2 op ( PB3 -> OC2A )connected to T0 timer 0 input (PD4 <- T0 )
  //** OC0A timer 0 op ( PD6 -> OC0A ) 

   
  // set PORT B 
  // PIN 9  PB1 -> OC1A
  // PIN 11 PB3 -> OC2A
  // PIN10  PB2 -> DRV8834 Direction
  // PIN12  PB4 -> DRV8834 Sleep
   DDRB = DDRB | B00011110;

  // set port C direction reqister 0-4 inputs; 5 output
  //DDRC = DDRC | B00100000;
  DDRC =  B00100000;
  //** If set as input then this will apply pull-up resistors
  PORTC = B00011111;

  
  // set PORT D
  // PIN13  PD7 -> DRV8834 Sleep
  // PIN3   PD3 -> DRV8834 Direction
  // PIN6   PD6 -> OC0A
  // PIN4   PD4 <- T0
  DDRD = DDRD | B11001000;
     
  //** disable DEC
  disableDECClock();

  //** start normal sidereal tracking;
  setRANormal();

}

void loop()
{
  unsigned char control_status;
  unsigned char speed_status;
//  char buff[20];
//  sprintf(buff, "%X\n", ~PINC);
//  Serial.print( buff );
  
  

  //** Get control status
  // control_status = if bits 0 and 1 =0, then default sidereal RA tracking which is x1 speed

  //** BITS 0->3 = 0000 then normal sidereal RA tracking; BIT 4 is ignored
  //** BIT 0 = 1 advance RA forward
  //** BIT 1 = 1 RA reverse
  //** BIT 2 = 1 DEC up
  //** BIT 3 = 1 DEC down
  //** BIT 4 = Fast tracking = 1, Sidereal rate tracking = 0
  
  control_status = ~PINC & B00001111;
  
  speed_status = PINC & B00010000;

  
  //** set motor control status
  //** If action requested
    switch(   control_status )
      {
      case 0x00:
         if( last_control_status != control_status )
           {
           disableDECClock();
           setRANormal();
           last_control_status = control_status;
           }
      break;  
      
      case 0x01:
         if( last_control_status != control_status )
           {
           setRAForward( speed_status);
           last_control_status = control_status;
           }
      break;

      case 0x02:
         if( last_control_status != control_status )
           {
           setRAReverse( speed_status);
           last_control_status = control_status;
           }
      break;

     case 0x04:
        if( last_control_status != control_status )
           {
           setDecUp(speed_status);
           last_control_status = control_status;
           }
     break;

     case 0x08:
        if( last_control_status != control_status )
           {
           setDecDown(speed_status);
           last_control_status = control_status;
           }
     break;

     default:
     break;
     }
 

  loop_delay(2);
  
  //** flash led
  if( led_toggle == 0 )
    {
    led_toggle = 1;
    PORTC = PORTC & B11011111;  
    }
  else
    {
    led_toggle = 0;
    PORTC = PORTC | B00100000;  
    }
    
}

//**************************************************************************************
//**
//** Set RA drive to advance
//**
//** Parameters; unsigned char speed_select 0x00= one times normal, 0x10 =  eight times
//**
//**************************************************************************************
void setRAForward( unsigned char speed_select )
{

  char buff[20];
  sprintf(buff, "%X\n", PINC);
  Serial.print( buff );
  Serial.println( "SetRAFwr " );


  PORTB &= ~( 1 << RA_SLEEP_CONTROL );     // set low; disable
  
   //** halt oscillator
   TCCR1B = B00011000;
   TCNT1H = 0x00;
   TCNT1L = 0x00;

  
  //** change speed
  if( speed_select == 0 )
    {
   //** x2 i.e. advance equal to 2 time sidreal speed   
   ICR1 = SIDREAL_RA_ICR1/2; 
   OCR1A = SIDREAL_RA_OCR1A/2;
   Serial.println( " SpeedNorm\n" );
    }
  else if ( speed_select == 0x10 )
    {
    //** x8
    ICR1 = SIDREAL_RA_ICR1/8; 
    OCR1A = SIDREAL_RA_OCR1A/8;
    Serial.println( " SpeedFast\n" );
    }
  
  PORTB &= ~( 1 << RA_DIRECTION_CONTROL );     // set low; direction
  
  //** restart oscillator
  TCCR1B = B00011101;

  PORTB |= ( 1 << RA_SLEEP_CONTROL );     // set high; enable

}



//**************************************************************************************
//**
//** Set RA drive to reverse
//**
//** Parameters; unsigned char speed_select 0x00= one times normal, 0x10 =  eight times
//**
//**************************************************************************************

void setRAReverse( unsigned char speed_select )
{
  char buff[20];
  sprintf(buff, "%X\n", PINC);
  Serial.print( buff );
  Serial.println( "SetRARev " );
  
  PORTB &= ~( 1 << RA_SLEEP_CONTROL );     // set low; disable
  
  //** halt oscillator
   TCCR1B = B00011000;
   TCNT1H = 0x00;
   TCNT1L = 0x00;

    
   //** reverse sidereal just requires halting
  //** change speed hence if speed_select == 0 no action required 
  if ( speed_select == 0x10 )
    {
    Serial.println( " SpeedFast\n" );  
    //** x7
    ICR1 = SIDREAL_RA_ICR1/8; 
    OCR1A = SIDREAL_RA_OCR1A/8;
    PORTB |= ( 1 << RA_DIRECTION_CONTROL );     // set high; direction
    
      //** restart oscillator
    TCCR1B = B00011101;
    PORTB |= ( 1 << RA_SLEEP_CONTROL );     // set high; enable
    }
  else
    {
     //** oscillator stopped; Earths rotation applies reverse x1
     Serial.println( " SpeedNormt\n" );  
    }
  

}


//*********************************************************************
//**
//** Sets normal sideral RA tracking
//**
//** Set stepper motor to drive RA at 15 deg hour
//**
//********************************************************************
void setRANormal( void )
{
  char buff[20];
  sprintf(buff, "%X\n", PINC);
  Serial.print( buff );
  Serial.println( "SetRANorm\n" );
  
  PORTB &= ~( 1 << RA_SLEEP_CONTROL );     // set low; disable DRV8834
  
  //** halt oscillator
  TCCR1B = B00011000;
  TCNT1H = 0x00;
  TCNT1L = 0x00;


  PORTB &= ~( 1 << RA_DIRECTION_CONTROL );     // set low; direction

  //** For RA tracking RA moves 15deg hour ( 24*15 =360 ); pulse required on RA drive is 9.090909 Hz
  //** timer 1; set for normal continious operation of 9.09090Hz 110.0mS square wave via drv8834
  //** COM1A1 clear OC1A( port pin PB1 ) on compare match.
  //** WGM13 WGM12 WGM11 ; =FAST PWM, TOP=ICR1
  //** CS12 = clk/1024 : 16MHz/1024 = 15,625Hz
  //** 15,625/1736 = 9.0909Hz; 110.0mS
  //** set OCR1A = 868, clear OC1A on compare match, so op goes low at 55.0mS, hence op is square wave
  
  ICR1 = SIDREAL_RA_ICR1; 
  OCR1A = SIDREAL_RA_OCR1A; 
  TCCR1A = _BV(COM1A1) | _BV(WGM11) ;//** op pin 9

  //** restart oscillator
  TCCR1B = B00011101;

  
  PORTB |= ( 1 << RA_SLEEP_CONTROL );     // set high; enable DRV8834
}

//**************************************************************************************
//**
//** Set DEC drive to advance
//**
//** Parameters; unsigned char speed_select 0x00= one times normal, 0x10 =  eight times
//**
//**************************************************************************************
void setDecUp(unsigned char speed_select )
{

  char buff[20];
  sprintf(buff, "%X\n", PINC);
  Serial.print( buff );
  Serial.println( "SetDECUp\n" );

  PORTD &= ~( 1 << DEC_SLEEP_CONTROL );     // set low; disable
  
  setDECClock( speed_select );
  
  PORTD &= ~( 1 << DEC_DIRECTION_CONTROL );     // set low; direction
  
  PORTD |= ( 1 << DEC_SLEEP_CONTROL );     // set high; enable

}

//******************************************************************************************
//**
//** Set DEC drive to reverse
//**
//** Parameters; unsigned char speed_select 0x00= one times normal, 0x10 =  ten times normal
//**
//******************************************************************************************

void setDecDown(unsigned char speed_select )
{
  char buff[20];
  sprintf(buff, "%X\n", PINC);
  Serial.print( buff );
  Serial.println( "SetDECDwn\n" );

  PORTD &= ~( 1 << DEC_SLEEP_CONTROL );     // set low; disable
  
  setDECClock( speed_select );
  
  PORTD |= ( 1 << DEC_DIRECTION_CONTROL );     // set high; direction
  
  PORTD |= ( 1 << DEC_SLEEP_CONTROL );     // set high; enable
  
}

//*********************************************************************************
//**
//** Set DEC drive clock
//**
//**  OP timer 2 feeds input timer 0; 
//**  OP timer 0 drives DRV8834
//**   
//**
//**  OP Timer 0; 3Hz (333mS) = 15deg hour 
//** 
//** Parameters; unsigned char speed_select 0x00= normal, 0x10 =  times ten speed
//**
//*********************************************************************************

void setDECClock( unsigned char speed_select )
{
  int speed_div;
  if( speed_select == 0x10 )
    {
    speed_div = 10;   //** approx 2.5 deg minute
    Serial.println( "SetDECSpeedFast\n" );
    }
  else
    {
    speed_div = 1;
    Serial.println( "SetDECSpeedNorm\n" );
    }
  
  //** combination timeres 2 and 0; only utilised for DEC movement when required
  //** accuracy not required approximate values will surffice
  //** for pulse 3Hz = sidereal rate of 15deg hour
  
 
  //** TIMER 2
  //** WGM22 WGM20 - PHASE CORRECT; DELAYS ARE DOUBLE
  //** WGM22 WGM21 WGM20 - FAST PMW; DELAYS AS CALCULATED
  //** CS22  CS20 Prescalar = 128; timer clock = 16,000,000/128 = 125,000
  //** COM2A0 Toggle OC2A on compare match
  //** Hence if OCR2A = 125 Clock op will toggle every 1ms preducing 2mS pulse 
  
  OCR2A = 125; //** set to produce 2mS output pulse from timer 2
  
  //** COM2A0 Toggle OC2A on compare match
  TCCR2A = _BV( WGM21) |_BV( WGM20) | _BV(COM2A0) ; //** T2 op pin 11 OC2A -> to input pin4 T0 for timer 0
  //** Prescalar = 128
   TCCR2B = _BV(CS20) | _BV(CS22) | _BV(WGM22);
  

  //** TIMER 0
  //** Input from timer 2 divided by OCR0A output toggled
  //** Hence if input 2mS pulse op and OCR0A = 25 output will be 50mS toggled producing 100mS square wave
  //** CS02 CS01 CS00 - External clock source on T0 pin. Clock on rising edge ( from timer 2 )
  //** COM0A0 Toggle OC0A on compare match
  //** timer 0; output port pin PD6 - PIN 6 OC0A
  OCR0A = DEC_SPEED_STANDARD/speed_div;
  TCCR0A = _BV( WGM00) | _BV( WGM01) | _BV(COM0A0) ;
  // Set to CTC Mode
  TCCR0B = _BV(CS00) | _BV(CS01) | _BV(CS02) | _BV(WGM02); //** input TO from timer2

  
}


//***************************************************************
//**
//** Disables DEC clock and DEC motor drv8834 driver
//**
//***************************************************************
void disableDECClock( void )
{

 PORTD &= ~( 1 << DEC_SLEEP_CONTROL );     // set low; disable DEC motor drv8834
  
 //** disable timer 2 clock 
 //TCCR2B = _BV(WGM22);
 TCCR2B = B00001000; //** clock stopped
 TCNT2 = 0x00;       //** clear counter
 
 //** disable timer 0 clock
 TCCR0B = B00001000; //** clock stopped
 TCNT0 = 0x00;       //** clear counter
  
}


//***********************************************************
//**
//** In normal Ardunio use timer 0 is used by delay functions.
//** However since this is utilised by the motor driver code
//** this delay loop has been added.
//** 
//** Parmameter unsigned long val; 1=30mS approx
//**********************************************************
void loop_delay(unsigned long val)
{
  volatile unsigned long i,k,c;
  
  for( k=0; k<val; k++ ) 
    {
    for( i=0; i< 10000L; i++ )
      c=c+1;
    }
}
