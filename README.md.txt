 Mark I Stepper Motor Control - for Celstron G4 dual axis drive
 replacing original 93522 dual axis motor drive.

 TASK
 -----
 Drive two stepper motors on the Celestron G4 dual axis telescope drive.
 One motor drives the Right ascension (RA), the other the  Declination (DEC).
 The RA is the celestial longitude. The Earth rotates across its RA at 15deg hour,
 thus the RA drive must run continuously for tracking purposes, that is compensating
 for the Earth's rotation. Fast movement must also be available for location of
 astronomical objects, be must revert to normal tracking speed.
 
 Declination is the  equivalent of astronomical latitude. Movement across this axis
 is used for locating astronomical objects. This then is only required when locating
 celestial objects.

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
 and divider in compare match mode ( details in code comments).

 The DEC drive requires a 3Hz ( 333mS ) pulse to the DRV8834 for a sidereal 15deg hour rate.
 Individually timers 0 and 2 cannot be used to generated a 3Hz output pulse for the DEC drive
 with the standard fuse settings, however they can be used in conjunction with each other.
 Timer 2 output OC2A is fed into timer 0 input TO and this can generate the range of pulses required.
 