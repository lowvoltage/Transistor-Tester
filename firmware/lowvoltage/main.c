// Original Source from http://www.mikrocontroller.net/articles/AVR-Transistortester
// Including some modifications from R.Bauer
#include <avr/io.h>
#include "lcd-routines.h"
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <math.h>

/*
	Factors for the gate capacity calculating at N-and P-channel enhancement type MOSFETs
	These factors depend on manufacturing tolerances on the AVR and hence need to be adjusted, if necessary
*/
#define N_GATE_CAPACITY_FACTOR 387
#define P_GATE_CAPACITY_FACTOR 142

/*
	Exact values of the resistors used in ohms.
	The face value is 680 ohms for R_L, 470kOhm for R_H
	To use the program to deviations from these values (eg due to component tolerances)
	To calibrate, enter the values in ohms resistance in the following defines:
*/
#define R_L_VAL 678			//R_L; standard value 680 ohms
#define R_H_VAL 468000UL	//R_H; 470 000 ohm standard value, specify an unsigned long

// The program resistor values needed to calculate
#define RH_RL_RATIO (R_H_VAL / R_L_VAL)
#define R_READ_RH (R_H_VAL / 100)


/*
	Factor for the case of capacitors capacity-measurement
	These factors depend on manufacturing tolerances on the AVR and hence need to be adjusted, if necessary
	H_CAPACITY_FACTOR is to measure with 470k resistor (low capacity)
	L_CAPACITY_FACTOR for the measurement with 680-ohm resistor (high capacity)
	The entire range is about 0.2 nF to 1000μF
*/
#define H_CAPACITY_FACTOR 394
#define L_CAPACITY_FACTOR 283
#define MCU_STATUS_REG MCUCSR
#define UseM8

// Strings in EEPROM
unsigned char TestRunning[] EEMEM = "Teste...";
unsigned char Bat[] EEMEM = "Bat. ";
unsigned char BatWeak[] EEMEM = "schwach";
unsigned char BatEmpty[] EEMEM = "leer!";
unsigned char TestFailed1[] EEMEM = "nicht ";
unsigned char TestFailed2[] EEMEM = "erkanntes ";
unsigned char Bauteil[] EEMEM = "Bauteil ";
unsigned char Unknown[] EEMEM = " unbekanntes, ";
unsigned char OrBroken[] EEMEM = "defektes ";
unsigned char Diode[] EEMEM = "Diode: ";
unsigned char DualDiode[] EEMEM = "Doppeldiode ";
unsigned char TwoDiodes[] EEMEM = "2 Dioden ";
unsigned char Antiparallel[] EEMEM = "Z ? ";
unsigned char InSeries[] EEMEM = "in Serie A=";
unsigned char mosfet[] EEMEM = "-MOS";
unsigned char emode[] EEMEM = "-E";
unsigned char dmode[] EEMEM = "-D";
unsigned char jfet[] EEMEM = "-JFET";
unsigned char Thyristor[] EEMEM = "Thyristor";
unsigned char Triac[] EEMEM = "Triac";
unsigned char A1[] EEMEM = ";A1=";
unsigned char A2[] EEMEM = ";A2=";
unsigned char hfestr[] EEMEM ="hFE=";
unsigned char NPN[] EEMEM = "NPN";
unsigned char PNP[] EEMEM = "PNP";
unsigned char bstr[] EEMEM = " B=";
unsigned char cstr[] EEMEM = ";C=";
unsigned char estr[] EEMEM = ";E=";
unsigned char gds[] EEMEM = "GDS=";
unsigned char Uf[] EEMEM = "Uf=";
unsigned char vt[] EEMEM = "Vt=";
unsigned char mV[] EEMEM = "mV";
unsigned char Anode[] EEMEM = "A=";
unsigned char Gate[] EEMEM = "G=";
unsigned char TestTimedOut[] EEMEM = "Timeout!";
unsigned char DiodeIcon[] EEMEM = {4,31,31,14,14,4,31,4,0};	//Diode-Icon

unsigned char Resistor[] EEMEM = "Widerstand: ";	// only available on Mega8
unsigned char NullDot[] EEMEM = "0,";
unsigned char GateCap[] EEMEM = " C=";
unsigned char Capacitor[] EEMEM = "Kondensator: ";


struct Diode {
	uint8_t Anode;
	uint8_t Cathode;
	int Voltage;
};

void CheckPins(uint8_t HighPin, uint8_t LowPin, uint8_t TristatePin);
void DischargePin(uint8_t PinToDischarge, uint8_t DischargeDirection);
unsigned int ReadADC(uint8_t mux);
void GetGateThresholdVoltage(void);
void lcd_show_format_cap(char outval[], uint8_t strlength, uint8_t CommaPos);
void ReadCapacity(uint8_t HighPin, uint8_t LowPin);		// capacitance measurement available only on Mega8


#define R_DDR DDRB
#define R_PORT PORTB

/*
	Port for the test resistors
	The resistors must be connected to the bottom of the 6-pin ports,
	and in the following order:
	RLx = 680R resistor for test-pin x
	RHx = 470k resistor for test-pin x

	RL1 at Pin 0
	RH1 at Pin 1
	RL2 at Pin 2
	RH2 at Pin 3
	RL3 at Pin 4
	RH3 at Pin 5
*/


#define ADC_PORT PORTC
#define ADC_DDR DDRC
#define ADC_PIN PINC
#define TP1 PC0
#define TP2 PC1
#define TP3 PC2

/*  Port for the test pins
	This port needs to have an ADC (ie when Mega8 PORTC).
	For the test pins to the bottom of this port 3 pins are used.
	Please do not change the definitions for TP1, TP2 and TP3!
*/

#define ON_DDR DDRD
#define ON_PORT PORTD
#define ON_PIN_REG PIND
#define ON_PIN PD6	// pin that can be drawn on high, in order to keep circuit in operation
#define RST_PIN PD7	// pin that is pulled low when the switch button is pressed

// Components
#define PART_NONE 0
#define PART_DIODE 1
#define PART_TRANSISTOR 2
#define PART_FET 3
#define PART_TRIAC 4
#define PART_THYRISTOR 5
#define PART_RESISTOR 6
#define PART_CAPACITOR 7

// End (Components)
// Specific definitions for components
// FETs
#define PART_MODE_N_E_MOS 1
#define PART_MODE_P_E_MOS 2
#define PART_MODE_N_D_MOS 3
#define PART_MODE_P_D_MOS 4
#define PART_MODE_N_JFET 5
#define PART_MODE_P_JFET 6

//Bipolar
#define PART_MODE_NPN 1
#define PART_MODE_PNP 2


struct Diode diodes[6];
uint8_t NumOfDiodes;

uint8_t b,c,e;		// terminals of the transistor
unsigned long hfe;	// gain
uint8_t PartReady;	// component detected done
unsigned int hfe1, hfe2;	// gains
unsigned int uBE1, uBE2;
uint8_t PartMode;
uint8_t tmpval, tmpval2;

uint8_t ra, rb;			// Resistance Pins
unsigned int rv1, rv2;	// voltage drop across the resistor
unsigned int radcmax1, radcmax2;	//maximum achievable ADC value (lower than 1023, because the voltage is low-pin for resistance measurement with zero)
uint8_t ca, cb;			// Capacitor Pins
unsigned long cv;


uint8_t PartFound, tmpPartFound;	// the found component
char outval[8];
unsigned int adcv[4];
uint8_t tmpval, tmpval2;
char outval2[6];

// Program start
int main(void) {
	// Turn on
	ON_DDR = (1<<ON_PIN);
	ON_PORT = (1<<ON_PIN) | (1<<RST_PIN);	// pullup for power on and reset pin
	uint8_t tmp;
	// Init ADC
	ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0);	// prescaler = 8

	lcd_init();

	if(MCU_STATUS_REG & (1<<WDRF)) {	
		/*
			Check for watchdog reset
			This occurs when the watchdog is not reset 2s
			May occur when the program has "entangled" in a continuous loop.
		*/
		lcd_eep_string(TestTimedOut);	// Timeout message
		_delay_ms(3000);
		ON_PORT = 0;	// Switch off!
		return 0;
	}
	LCDLoadCustomChar();	// Custom-characters
	// Diode symbol in LCD load
	lcd_eep_string(DiodeIcon);
	Line1();	// 1 line
	// Entry point when the start button is pressed again during operation
	start:
	wdt_enable(WDTO_2S);	//Watchdog on
	PartFound = PART_NONE;
	tmpPartFound = PART_NONE;
	NumOfDiodes = 0;
	PartReady = 0;
	PartMode = 0;
	ca = 0;
	cb = 0;
	lcd_clear();
	// Measure supply voltage
	ReadADC(5 | (1<<REFS1));	// Dummy readout
	hfe1 = ReadADC(5 | (1<<REFS1)); 	// With internal reference
	if (hfe1 < 650) {			// VCC <7.6 V; Warning Display -> 650
		lcd_eep_string(Bat);		// Displays: "Battery"
		if(hfe1 < 600) {					// VCC <7.15 V, not more reliable operation possible
			lcd_eep_string(BatEmpty);		// Battery empty!
			_delay_ms(1000);
			PORTD = 0;	// Switch off
			return 0;
		}
		lcd_eep_string(BatWeak);		// Battery low
		Line2();
	}
	// Start test
	lcd_eep_string(TestRunning);	//String: Test running
	// Check all 6 possible combinations for the 3 pins
	CheckPins(TP1, TP2, TP3);
	CheckPins(TP1, TP3, TP2);
	CheckPins(TP2, TP1, TP3);
	CheckPins(TP2, TP3, TP1);
	CheckPins(TP3, TP2, TP1);
	CheckPins(TP3, TP1, TP2);

	// Separate measurements on the test capacitor
	if((PartFound == PART_NONE) || (PartFound == PART_RESISTOR) || (PartFound == PART_DIODE)) {
		// Capacitor is discharged, is otherwise possible may not measure
		R_PORT = 0;
		R_DDR = (1<<(TP1 * 2)) | (1<<(TP2 * 2)) | (1<<(TP3 * 2));
		_delay_ms(50);
		R_DDR = 0;
		// Capacity in all 6-pin combinations measure
		ReadCapacity(TP3, TP1);
		ReadCapacity(TP3, TP2);
		ReadCapacity(TP2, TP3);
		ReadCapacity(TP2, TP1);
		ReadCapacity(TP1, TP3);
		ReadCapacity(TP1, TP2);
	}

	// Done, now followed by the evaluation
	GetGateThresholdVoltage();
	lcd_clear();
	if(PartFound == PART_DIODE) {
		if(NumOfDiodes == 1) {
			// Standard diode
			lcd_eep_string(Diode);	//"Diode: "
			lcd_eep_string(Anode);
			lcd_data(diodes[0].Anode + 49);
			lcd_string(";K=");
			lcd_data(diodes[0].Cathode + 49);
			Line2();	//2nd line
			lcd_eep_string(Uf);	//"Uf = "
			lcd_string(itoa(diodes[0].Voltage, outval, 10));
			lcd_eep_string(mV);
			goto end;
		} else if(NumOfDiodes == 2) {
			// Dual diode
			if(diodes[0].Anode == diodes[1].Anode) {
				// Common anode
				lcd_eep_string(DualDiode);	// Dual diode
				lcd_string("CA");
				Line2(); //2nd line
				lcd_eep_string(Anode);
				lcd_data(diodes[0].Anode + 49);
				lcd_string(";K1=");
				lcd_data(diodes[0].Cathode + 49);
				lcd_string(";K2=");
				lcd_data(diodes[1].Cathode + 49);
				goto end;
			} else if(diodes[0].Cathode == diodes[1].Cathode) {
				// Common cathode
				lcd_eep_string(DualDiode);	// Dual diode
				lcd_string("CC");
				Line2(); //2nd line
				lcd_string("K=");
				lcd_data(diodes[0].Cathode + 49);
				lcd_eep_string(A1);		//";A1="
				lcd_data(diodes[0].Anode + 49);
				lcd_eep_string(A2);		//";A2="
				lcd_data(diodes[1].Anode + 49);
				goto end;
			} else if ((diodes[0].Cathode == diodes[1].Anode) && (diodes[1].Cathode == diodes[0].Anode)) {
				// Antiparallel
				lcd_eep_string(TwoDiodes);	// Two diodes or a Zener diode
				lcd_eep_string(Antiparallel);	//Antiparallel
				Line2(); //2nd line
				lcd_string(itoa(diodes[0].Voltage, outval, 10));
				lcd_string(",");
				lcd_string(itoa(diodes[1].Voltage, outval, 10));
				lcd_eep_string(mV);
				goto end;
			}
		} else if(NumOfDiodes == 3) {
			// Series connection of two diodes, is recognized as a 3 diodes
			b = 3;
			c = 3;
			/*  Checking for a for a series connection of 2 diodes possible constellation
				This requires 2 of the cathode and anode 2 of the match.
				This is because the diodes are used as two individual diodes and ADDITIONALLY recognized as a "big" diode
			*/
			if((diodes[0].Anode == diodes[1].Anode) || (diodes[0].Anode == diodes[2].Anode)) b = diodes[0].Anode;
			if(diodes[1].Anode == diodes[2].Anode) b = diodes[1].Anode;

			if((diodes[0].Cathode == diodes[1].Cathode) || (diodes[0].Cathode == diodes[2].Cathode)) c = diodes[0].Cathode;
			if(diodes[1].Cathode == diodes[2].Cathode) c = diodes[1].Cathode;
			if((b<3) && (c<3)) {
				lcd_eep_string(TwoDiodes);// Two diodes
				Line2(); //2nd line
				lcd_eep_string(InSeries); //"in series A="
				lcd_data(b + 49);
				lcd_string(";K=");
				lcd_data(c + 49);
				goto end;
			}
		}
	} else if (PartFound == PART_TRANSISTOR) {
		if(PartReady == 0) {	// If second Examination never made, e.g. for transistor protection diode
			hfe2 = hfe1;
			uBE2 = uBE1;
		}
		if((hfe1>hfe2)) {	// If the gain was higher in the first test: C interchange and E
			hfe2 = hfe1;
			uBE2 = uBE1;
			tmp = c;
			c = e;
			e = tmp;
		}

		if(PartMode == PART_MODE_NPN) {
			lcd_eep_string(NPN);
		} else {
			lcd_eep_string(PNP);
		}
		lcd_eep_string(bstr);	//B=
		lcd_data(b + 49);
		lcd_eep_string(cstr);	//;C=
		lcd_data(c + 49);
		lcd_eep_string(estr);	//;E=
		lcd_data(e + 49);
		Line2(); //2nd line
		// Calculate gain
		// = hFE emitter current / base current
		hfe = hfe2;
		hfe *= RH_RL_RATIO;	// ratio of high-to low-resistance
		if(uBE2<11) uBE2 = 11;
		hfe /= uBE2;
		hfe2 = (unsigned int) hfe;
		lcd_eep_string(hfestr);	//"hFE="
		lcd_string(utoa(hfe2, outval, 10));
		SetCursor(2,7);			// ursor on line 2, character 7
		if(NumOfDiodes > 2) {	// transistor with a protection diode
			lcd_data(LCD_CHAR_DIODE);	// display diode symbol
		} else {
		lcd_data(' ');
		}

			for(c=0;c<NumOfDiodes;c++) {
				if(((diodes[c].Cathode == e) && (diodes[c].Anode == b) && (PartMode == PART_MODE_NPN)) || ((diodes[c].Anode == e) && (diodes[c].Cathode == b) && (PartMode == PART_MODE_PNP))) {
					lcd_eep_string(Uf);	//"Uf="
					lcd_string(itoa(diodes[c].Voltage, outval, 10));
					lcd_data('m');
					goto end;
				}
			}

		goto end;
	} else if (PartFound == PART_FET) {	//JFET or MOSFET
		if(PartMode&1) {	//N-Channel
			lcd_data('N');
		} else {
			lcd_data('P');	//P-Channel
		}
		if((PartMode==PART_MODE_N_D_MOS) || (PartMode==PART_MODE_P_D_MOS)) {
			lcd_eep_string(dmode);	//"-D"
			lcd_eep_string(mosfet);	//"-MOS"
		} else {
			if((PartMode==PART_MODE_N_JFET) || (PartMode==PART_MODE_P_JFET)) {
				lcd_eep_string(jfet);	//"-JFET"
			} else {
				lcd_eep_string(emode);	//"-E"
				lcd_eep_string(mosfet);	//"-MOS"
			}
		}

			if(PartMode < 3) {	// enhancement mode MOSFET
				lcd_eep_string(GateCap);	//" C="
				tmpval = strlen(outval2);
				tmpval2 = tmpval;
				if(tmpval>4) tmpval = 4;	// with capacity> 100 nF last no longer specify decimal place (otherwise not fit on the LCD)
				lcd_show_format_cap(outval2, tmpval, tmpval2);
				lcd_data('n');
			}

		Line2(); //2nd line
		lcd_eep_string(gds);	//"GDS="
		lcd_data(b + 49);
		lcd_data(c + 49);
		lcd_data(e + 49);
		if((NumOfDiodes > 0) && (PartMode < 3)) {	// MOSFET with protection diode, is there only for enhancement FETs
			lcd_data(LCD_CHAR_DIODE);	// display diode symbol
		} else {
			lcd_data(' ');	// space
		}
		if(PartMode < 3) {	// enhancement mode MOSFET
			lcd_eep_string(vt);
			lcd_string(outval);	// gate threshold voltage was determined previously
			lcd_data('m');
		}
		goto end;
	} else if (PartFound == PART_THYRISTOR) {
		lcd_eep_string(Thyristor);	//"Thyristor"
		Line2(); //2nd line
		lcd_string("GAK=");
		lcd_data(b + 49);
		lcd_data(c + 49);
		lcd_data(e + 49);
		goto end;
	} else if (PartFound == PART_TRIAC) {
		lcd_eep_string(Triac);	//"Triac"
		Line2(); //2nd line
		lcd_eep_string(Gate);
		lcd_data(b + 49);
		lcd_eep_string(A1);		//";A1="
		lcd_data(c + 49);
		lcd_eep_string(A2);		//";A2="
		lcd_data(e + 49);
		goto end;

		} else if(PartFound == PART_RESISTOR) {
			lcd_eep_string(Resistor); //"Resistor: "
			lcd_data(ra + 49);	// Pin information
			lcd_data('-');
			lcd_data(rb + 49);
			Line2(); //2nd line
			lcd_string ("R = ");
			if(rv1>512) {		// Check to see how far the resistance applied to the test voltages differ from 512
				hfe1 = (rv1 - 512);
			} else {
				hfe1 = (512 - rv1);
			}
			if(rv2>512) {
				hfe2 = (rv2 - 512);
			} else {
				hfe2 = (512 - rv2);
			}
			if(hfe1 > hfe2)  {
				radcmax1 = radcmax2;
				rv1 = rv2;	// use the result, which is closer to 512 (higher accuracy)
				rv2 = R_READ_RH;	// 470k resistance test
				
			} else {
				rv2 = R_L_VAL;	//680R resistance test
			}
			if(rv1==0) rv1 = 1;
			hfe = (unsigned long)((unsigned long)((unsigned long)rv2 * (unsigned long)rv1) / (unsigned long)((unsigned long)radcmax1 - (unsigned long)rv1));	//Widerstand berechnen
			ultoa(hfe,outval,10);

			if(rv2==R_READ_RH) {	// 470k resistor?
				ra = strlen(outval);	// Necessary to display comma
				for(rb=0;rb<ra;rb++) {
					lcd_data(outval[rb]);
					if(rb==(ra-2)) lcd_data('.');	// Comma
				}
				lcd_data ('k'); // kilo-ohms, if 470k resistor is used
			} else {
				lcd_string(outval);
			}
			lcd_data(LCD_CHAR_OMEGA);	//Omega for Ohm
			goto end;

		} else if(PartFound == PART_CAPACITOR) {	// capacity measurement only available on Mega8
			lcd_eep_string(Capacitor);
			lcd_data(ca + 49);	// Pin information
			lcd_data('-');
			lcd_data(cb + 49);
			Line2(); //2nd line
			tmpval2 = 'n';
			if(cv > 99999) {	// Above 1µF
				cv /= 1000;
				tmpval2 = LCD_CHAR_U;
			}
			ultoa(cv, outval, 10);
			tmpval = strlen(outval);
			lcd_show_format_cap(outval, tmpval, tmpval);
			lcd_data(tmpval2);
			lcd_data('F');
			goto end;
	}

		if(NumOfDiodes == 0) {
			// No diodes found
			lcd_eep_string(TestFailed1); // "No, unknown, or"
			Line2(); //2nd line
			lcd_eep_string(TestFailed2); // "deffective "
			lcd_eep_string(Bauteil);
		} else {
			lcd_eep_string(Bauteil);
			lcd_eep_string(Unknown); //" unknown."
			Line2(); //2nd line
			lcd_eep_string(OrBroken); //" or deffective"
			lcd_data(NumOfDiodes + 48);
			lcd_data('d');
		}

	end:
	while(!(ON_PIN_REG & (1<<RST_PIN)));		// wait until you release button
	_delay_ms(200);
	for(hfe1 = 0;hfe1<10000;hfe1++) {
		if(!(ON_PIN_REG & (1<<RST_PIN))) {
			/*
			 If the button was pressed again ...
			 jump back to the beginning and perform new test
			*/
			goto start;
		}
		wdt_reset();
		_delay_ms(1);
	}
	ON_PORT &= ~(1<<ON_PIN);	// shutdown
	wdt_disable();	//Watchdog off
	// Infinite loop
	while(1) {
		if(!(ON_PIN_REG & (1<<RST_PIN))) {	
			/* Can only be achieved when the automatic switch-off has not been installed */
			goto start;
		}
	}
	return 0;
}

void CheckPins(uint8_t HighPin, uint8_t LowPin, uint8_t TristatePin) {
	/*
		Function to test the properties of the component at the specified pin assignments
		parameters:
		HighPin: pin, which is initially set at a positive potential
		LowPin: pin, which is initially set at a negative potential
		TristatePin: pin, which is initially left open

		During testing TristatePin will of course also be connected positively or negatively
	*/
	unsigned int adcv[6];
	uint8_t tmpval, tmpval2;
	/*
		HighPin is placed firmly on Vcc
		LowPin is placed over R_L to GND
		TristatePin is switched to high, but no action is required
	*/
	wdt_reset();
	// Set pins
	tmpval = (LowPin * 2);			// required because of the arrangement of the resistors
	R_DDR = (1<<tmpval);			// Low-Pin as output over R_L to ground
	R_PORT = 0;
	ADC_DDR = (1<<HighPin);			// High-Pin as output
	ADC_PORT = (1<<HighPin);		// High-pin fixed to Vcc
	_delay_ms(5);
	// Some MOSFETs must have gate (TristatePin) first discharge
	// N-channel:
	DischargePin(TristatePin,0);
	// Voltage at Low-Pin determined
	adcv[0] = ReadADC(LowPin);
	if(adcv[0] < 20) goto next;	// Lock the device now?
	// else: for P-channel discharge (gate to plus)
	DischargePin(TristatePin,1);
	// Voltage at Low-Pin determined
	adcv[0] = ReadADC(LowPin);

	next:
	if(adcv[0] < 20) {	// If the component has no continuity between HighPin and LowPin
		tmpval = (TristatePin * 2);		// required because of the arrangement of the resistors
		R_DDR |= (1<<tmpval);			// TristatePin to ground over R_L
		_delay_ms(2);
		adcv[0] = ReadADC(LowPin);		// Measure voltage
		if(adcv[0] > 700) {
			// Component passes => pnp transistor, etc.
			// Gain measured in both directions
			R_DDR &= ~(1<<tmpval);		// Tristate-Pin (Base) high impedance
			tmpval++;
			R_DDR |= (1<<tmpval);		// Tristate-Pin (Base) to ground over R_H

			_delay_ms(10);
			adcv[0] = ReadADC(LowPin);		// Measure voltage at Low-Pin (collector suspected)
			adcv[2] = ReadADC(TristatePin);	// Measure base voltage
			R_DDR &= ~(1<<tmpval);		// Tristate-Pin (Base) high impedance
			// Check that test run ever
			if((PartFound == PART_TRANSISTOR) || (PartFound == PART_FET)) {
				PartReady = 1;
				hfe2 = adcv[0];
				uBE2 = adcv[2];
			} else {
				hfe1 = adcv[0];
				uBE1 = adcv[2];
			}
			if(adcv[2] > 200) {
				if(PartFound != PART_THYRISTOR) {
					PartFound = PART_TRANSISTOR;	// PNP transistor found (Base is pulled "up")
					PartMode = PART_MODE_PNP;
				}
			} else {
				if(PartFound != PART_THYRISTOR) {
				 	PartFound = PART_FET;			// P-Channel-MOSFET found (Base/Gate is NOT pulled "up")
					PartMode = PART_MODE_P_E_MOS;
				}
			}
			if(PartFound != PART_THYRISTOR) {
				b = TristatePin;
				c = LowPin;
				e = HighPin;
			}
		}

		// Tristate Pin (assumed base) set HIGH, to test for npn
		ADC_PORT = 0;					// Low-Pin fixed to ground
		tmpval = (TristatePin * 2);		// required because of the arrangement of the resistors
		tmpval2 = (HighPin * 2);		// required because of the arrangement of the resistors
		R_DDR = (1<<tmpval) | (1<<tmpval2);			// High-Pin and Tristate-Pin as outputs
		R_PORT = (1<<tmpval) | (1<<tmpval2);		// High-Pin and Tristate-Pin over R_L to Vcc
		ADC_DDR = (1<<LowPin);			// Low-Pin as output
		_delay_ms(10);
		adcv[0] = ReadADC(HighPin);		// Measure voltage at High-Pin
		if(adcv[0] < 500) {
			if(PartReady==1) goto testend;
			// Component passes => NPN transistor etc.

			// Test on thyristor:
			// Unload gate

			
			R_PORT &= ~(1<<tmpval);			// Tristate-Pin (Gate) over R_L to ground
			_delay_ms(10);
			R_DDR &= ~(1<<tmpval);			// Tristate-Pin (Gate) high impedance
			//Test on Thyristor
			_delay_ms(5);
			adcv[1] = ReadADC(HighPin);		// Measure voltage at High-Pin (assumed anode)
			
			R_PORT &= ~(1<<tmpval2);	// High-Pin (anode suspected) to ground
			_delay_ms(10);
			R_PORT |= (1<<tmpval2);	// High-Pin (anode suspected) to positive rail
			_delay_ms(5);
			adcv[2] = ReadADC(HighPin);		// Measure the voltage at High-Pin (anode suspected) again
			if((adcv[1] < 500) && (adcv[2] > 900)) {	//Lock after switching off the holding current, the thyristor
				// in front of the trigger current disconnection was activated and is still connected even though gate off => thyristor
				if(PartFound == PART_THYRISTOR) {
					PartFound = PART_TRIAC;	// This result been there once before => triac (Dual-thyristor)
					PartReady = 1;
					goto saveresult;
				} else {
					PartFound = PART_THYRISTOR;
					goto saveresult;
				}
			}
			// Test for transistor or MOSFET
			tmpval++;
			R_DDR |= (1<<tmpval);		// Tristate-Pin (Base) as output
			R_PORT |= (1<<tmpval);		// Tristate-Pin (Basis) over R_H to positive rail
			_delay_ms(50);
			adcv[0] = ReadADC(HighPin);		// Measure voltage at High-Pin (collector suspected)
			adcv[2] = ReadADC(TristatePin);	// Measure voltage at base
			R_PORT &= ~(1<<tmpval);			// Tristate-Pin (Base) high impedance
			R_DDR &= ~(1<<tmpval);			// Tristate-Pin (Base) as input

			if((PartFound == PART_TRANSISTOR) || (PartFound == PART_FET)) {	// heck whether the test run ever
				PartReady = 1;
				hfe2 = 1023 - adcv[0];
				uBE2 = 1023 - adcv[2];
			} else {
				hfe1 = 1023 - adcv[0];
				uBE1 = 1023 - adcv[2];
			}
			if(adcv[2] < 500) {
				PartFound = PART_TRANSISTOR;	// NPN transistor found (base is pulled "down")
				PartMode = PART_MODE_NPN;
			} else {
				PartFound = PART_FET;			// N-Channel-MOSFET found (Base/Gate is NOT pulled "down")
				PartMode = PART_MODE_N_E_MOS;
			}
			saveresult:
			b = TristatePin;
			c = HighPin;
			e = LowPin;
		}
		ADC_DDR = 0;
		ADC_PORT = 0;
		// Done
	} else {	// Passage
		// Test for N-JFET or self conducting N-MOSFET
		R_DDR |= (2<<(TristatePin*2));	// Tristate-Pin (presumed Gate) over R_H to ground
		_delay_ms(20);
		adcv[0] = ReadADC(LowPin);		// Measure the voltage at the suspected source
		R_PORT |= (2<<(TristatePin*2));	// Tristate-Pin (presumed Gate) over R_H to Plus
		_delay_ms(20);
		adcv[1] = ReadADC(LowPin);		// Measure the voltage at the suspected source
		// If there is a self-conducting MOSFET or JFET would, ADCV [1]> ADCV [0] be
		if(adcv[1]>(adcv[0]+100)) {
			// Measure voltage at the gate, to distinguish between the MOSFET and JFET
			ADC_PORT = 0;
			ADC_DDR = (1<<LowPin);		// Low-Pin fixed to ground
			tmpval = (HighPin * 2);		// required because of the arrangement of the resistors
			R_DDR |= (1<<tmpval);			// High-Pin as output
			R_PORT |= (1<<tmpval);			// High-Pin over R_L to Vcc
			_delay_ms(20);
			adcv[2] = ReadADC(TristatePin);		// Measure the voltage at the suspected gate
			if(adcv[2]>800) {	//MOSFET
				PartFound = PART_FET;			// N-Channel-MOSFET
				PartMode = PART_MODE_N_D_MOS;	// depletion-mode-MOSFET
			} else {	// JFET (pn junction between the G and S conducts)
				PartFound = PART_FET;			//N-Channel-JFET
				PartMode = PART_MODE_N_JFET;
			}
			PartReady = 1;
			b = TristatePin;
			c = HighPin;
			e = LowPin;
		}
		ADC_PORT = 0;

		// Test for P-JFET or self-conducting P-MOSFET
		ADC_DDR = (1<<LowPin);	// Low-Pin (suspected drain) set to ground, Tristate-Pin (presumed gate) is still over R_H to Plus
		tmpval = (HighPin * 2);			// required because of the arrangement of the resistors
		R_DDR |= (1<<tmpval);			// High-Pin as output
		R_PORT |= (1<<tmpval);			// High-Pin over R_L to Vcc
		_delay_ms(20);
		adcv[0] = ReadADC(HighPin);		// Measure the voltage at the suspected source
		R_PORT &= ~(2<<(TristatePin*2));	// Tristate-Pin (suspected Gate) over R_H to ground
		_delay_ms(20);
		adcv[1] = ReadADC(HighPin);		// Measure the voltage at the suspected source again
		// If there is a self-conducting P-MOSFET or JFET would, ADCV [0]> ADCV [1] have
		if(adcv[0]>(adcv[1]+100)) {
			// Measure voltage at the gate, to distinguish between the MOSFET and JFET
			ADC_PORT = (1<<HighPin);	// High-Pin set to Plus
			ADC_DDR = (1<<HighPin);		// High-Pin as output
			_delay_ms(20);
			adcv[2] = ReadADC(TristatePin);		// Measure the voltage at the suspected gate
			if(adcv[2]<200) {	// MOSFET
				PartFound = PART_FET;			// P-Channel-MOSFET
				PartMode = PART_MODE_P_D_MOS;	// depletion-mode MOSFET
			} else {	// JFET (pn junction between the G and S conducts)
				PartFound = PART_FET;			//P-Kanal-JFET
				PartMode = PART_MODE_P_JFET;
			}
			PartReady = 1;
			b = TristatePin;
			c = LowPin;
			e = HighPin;
		}

		tmpval2 = (2<<(2*HighPin));	//R_H
		tmpval = (1<<(2*HighPin));	//R_L
		ADC_PORT = 0;
		// Test for diode
		ADC_DDR = (1<<LowPin);	// Low-Pin to ground, High-Pin is still over R_L to Vcc
		DischargePin(TristatePin,1);	// Discharge for P-Channel MOSFET
		_delay_ms(5);
		adcv[0] = ReadADC(HighPin) - ReadADC(LowPin);
		R_DDR = tmpval2;	// High-Pin over R_H to Plus
		R_PORT = tmpval2;
		_delay_ms(5);
		adcv[2] = ReadADC(HighPin) - ReadADC(LowPin);
		R_DDR = tmpval;	// High-Pin over R_L auf Plus
		R_PORT = tmpval;
		DischargePin(TristatePin,0);	// Discharge for N-Channel MOSFET
		_delay_ms(5);
		adcv[1] = ReadADC(HighPin) - ReadADC(LowPin);
		R_DDR = tmpval2;	// High-Pin over R_H to Plus
		R_PORT = tmpval2;
		_delay_ms(5);
		adcv[3] = ReadADC(HighPin) - ReadADC(LowPin);
		/*
			Can without unloading it in false detections, because the gate of a MOSFET can still be charged.
			The additional measurement with the "big" R_H resistance is carried out to anti-parallel diode of
			Different resistances to.
			A diode has a forward current of relatively independent Durchlassspg.
			With a resistance, the voltage drop changes greatly (linear) with the current.
		*/
		if(adcv[0] > adcv[1]) {
			adcv[1] = adcv[0];	// the higher value wins
			adcv[3] = adcv[2];
		}

		if((adcv[1] > 30) && (adcv[1] < 950)) { // voltage is above 0.15 V and 4.64 V => OK
			if((PartFound == PART_NONE) || (PartFound == PART_RESISTOR)) PartFound = PART_DIODE;	// specify diode only if no other component was found. Otherwise there would be problems in transistors with protection diode
			diodes[NumOfDiodes].Anode = HighPin;
			diodes[NumOfDiodes].Cathode = LowPin;
			diodes[NumOfDiodes].Voltage = (adcv[1]*52/11);	// multiply the value from the ADC approximately by 4.9 to obtain the voltage in millivolts
			NumOfDiodes++;
			for(uint8_t i=0;i<NumOfDiodes;i++) {
				if((diodes[i].Anode == LowPin) && (diodes[i].Cathode == HighPin)) {	// two antiparallel diodes: Defect or duo-LED
					if((adcv[3]*64) < (adcv[1] / 5)) {	// forward voltage drops sharply at lower test current => defect
						if(i<NumOfDiodes) {
							for(uint8_t j=i;j<(NumOfDiodes-1);j++) {
								diodes[j].Anode = diodes[j+1].Anode;
								diodes[j].Cathode = diodes[j+1].Cathode;
								diodes[j].Voltage = diodes[j+1].Voltage;
							}
						}
						NumOfDiodes -= 2;
					}
				}
			}
		}
	}

		// Test for resistor
		tmpval2 = (2<<(2*HighPin));	//R_H
		tmpval = (1<<(2*HighPin));	//R_L
		ADC_PORT = 0;
		ADC_DDR = (1<<LowPin);	// Low-Pin set to ground
		R_DDR = tmpval;			// High-Pin over R_L to Plus
		R_PORT = tmpval;
		adcv[2] = ReadADC(LowPin);
		adcv[0] = ReadADC(HighPin) - adcv[2];
		R_DDR = tmpval2;		//High-Pin over R_H to Plus
		R_PORT = tmpval2;
		adcv[3] = ReadADC(LowPin);
		adcv[1] = ReadADC(HighPin) - adcv[3];

		// Measure the voltage difference between the positive pole of R_L and R_H and Vcc
		tmpval2 = (2<<(2*LowPin));	//R_H
		tmpval = (1<<(2*LowPin));	//R_L
		ADC_DDR = (1<<HighPin);		// High-Pin as output
		ADC_PORT = (1<<HighPin);	// High-Pin set to Plus
		R_PORT = 0;
		R_DDR = tmpval;				// Low-Pin over R_L to ground
		adcv[2] += (1023 - ReadADC(HighPin));
		R_DDR = tmpval2;				// Low-Pin over R_H to ground
		adcv[3] += (1023 - ReadADC(HighPin));
		
		if(((adcv[0] - adcv[2]) < 900) && ((adcv[1] - adcv[3]) > 20)) goto testend; 	// Voltage drops and low current test does not go far enough
		if(((adcv[1] * 32) / 31) < adcv[0]) {	// Falling voltage drops sharply at lower test current and there is no "near-short" => Resistance
			if((PartFound == PART_DIODE) || (PartFound == PART_NONE) || (PartFound == PART_RESISTOR)) {
				if((tmpPartFound == PART_RESISTOR) && (ra == LowPin) && (rb == HighPin)) {
					/* 	The device was tested once before with reversed polarity.
						Now compare the two results together. If they are quite similar,
						it is (most likely) by a resistor. */
					if(!((((adcv[0] + 100) * 11) >= ((rv1 + 100) * 10)) && (((rv1 + 100) * 11) >= ((adcv[0] + 100) * 10)) && (((adcv[1] + 100) * 11) >= ((rv2 + 100) * 10)) && (((rv2 + 100) * 11) >= ((adcv[1] + 100) * 10)))) {
						// min. 10% deviation => no resistance
						tmpPartFound = PART_NONE;
						goto testend;
					}
					PartFound = PART_RESISTOR;
				}
				rv1 = adcv[0];
				rv2 = adcv[1];

				radcmax1 = 1023 - adcv[2];	// Voltage at Low-Pin not quite zero, but around 0.1 V (but measured). The resulting error is compensated here
				radcmax2 = 1023 - adcv[3];
				ra = HighPin;
				rb = LowPin;
				tmpPartFound = PART_RESISTOR;
			}
		}
	testend:
	ADC_DDR = 0;
	ADC_PORT = 0;
	R_DDR = 0;
	R_PORT = 0;
}

void ReadCapacity(uint8_t HighPin, uint8_t LowPin) {
	// Test capacitor (only possible on ATMega8)
	if((HighPin == cb) && (LowPin == ca)) return;	// test already run with reversed polarity
	unsigned long gcval = 0;
	unsigned int tmpint = 0;
	uint8_t extcnt = 0;
	uint8_t tmpx = 0;
	
	tmpval2 = (2<<(2*HighPin));	//R_H
	tmpval = (1<<(2*HighPin));	//R_L
	ADC_PORT = 0;
	R_PORT = 0;
	R_DDR = 0;
	ADC_DDR = (1<<LowPin);	// Low-Pin to ground
	R_DDR = tmpval2;		// HighPin over R_H to ground
	_delay_ms(5);
	adcv[0] = ReadADC(HighPin);
	DischargePin(HighPin,1);
	_delay_ms(5);
	adcv[1] = ReadADC(HighPin);
	wdt_reset();
	if(adcv[1] > (adcv[0] + 1)) {	// Voltage is increased
		R_DDR = tmpval;			// High-Pin over R_L to ground
		while(ReadADC(HighPin) > (ReadADC(LowPin) + 10)) {
			wdt_reset();
			tmpint++;
			if(tmpint==0) {
				extcnt++;
				if(extcnt == 30) break; // Timeout for Discharge
			}
		}
		tmpint = 0;
		extcnt = 0;
		R_PORT = tmpval;			// High-Pin over R_L to Plus
		_delay_ms(5);
		adcv[2] = ReadADC(HighPin);
		_delay_ms(80);
		adcv[3] = ReadADC(HighPin);
		if((adcv[3] < (adcv[2] + 3)) && (adcv[3] < 850)) return;	// Voltage is not increased appreciably => Demolition
		if((NumOfDiodes > 0) && (adcv[3] > 950)) return; // most likely one (or more) diode(s) in the reverse direction, which is usually incorrectly identified as a capacitor
		R_PORT = 0;
		while(ReadADC(HighPin) > (ReadADC(LowPin) + 10)) {
			wdt_reset();
			tmpint++;
			if(tmpint==0) {
				extcnt++;
				if(extcnt == 30) break; // Timeout for Discharge
			}
		}
		tmpint = 0;
		extcnt = 0;
		ADC_DDR = 7;					// All pins as outputs and to ground
		R_PORT = tmpval;  	   			// HighPin over R_L to Plus
		tmpval=(1<<HighPin);
		_delay_ms(2);
		ADC_DDR=(1<<LowPin);          // Capacitor load slowly over R_H
		while (!(ADC_PIN & tmpval)) {  // Wait until HighPin goes high; loop takes 7 cycles
			wdt_reset();
			tmpint++;
			if(tmpint==0) {
				extcnt++;
				if(extcnt == 30) break; // Timeout for charge
			}
		}
		if((extcnt == 0) && (tmpint<256)) {	// Low capacity
			// measure again with R_H
			R_PORT = 0;
			tmpint = 0;
			extcnt = 0;
			while(ReadADC(HighPin) > (ReadADC(LowPin) + 10)) {
				wdt_reset();
				tmpint++;
				if(tmpint==0) {
					extcnt++;
					if(extcnt == 30) break; // Timeout for Discharge
				}
			}
			tmpint = 0;
			extcnt = 0;
			ADC_DDR = 7;					// All pins as outputs
			ADC_PORT = 0;					// All pins to ground
			R_DDR = tmpval2;        		// HighPin over R_H as output
			R_PORT = tmpval2;  	   			// HighPin over R_H to Plus
			_delay_ms(2);
			ADC_DDR=(1<<LowPin);          // capacitor load slowly over R_H
			while (!(ADC_PIN & tmpval)) {  // Wait until HighPin goes high; loop takes 7 cycles
				wdt_reset();
				tmpint++;
				if(tmpint==0) {
					extcnt++;
					if(extcnt == 30) break; // timeout for capacitance measurement
				}
			}
			tmpx = 1;
		}
		if(tmpx) {
			gcval = H_CAPACITY_FACTOR;
			if((extcnt == 0) && (tmpint < 5)) goto end;	// capacity is too low
			cv = 1;
		} else {
			gcval = L_CAPACITY_FACTOR;
			cv = 1000;
		}

		gcval *= (unsigned long)(((unsigned long)extcnt * 65536) + (unsigned long)tmpint);	// Store value
		gcval /= 100;
		cv *= gcval;

		PartFound = PART_CAPACITOR;	// Capacitor found

		ca = HighPin;
		cb = LowPin;
		// Capacitor is discharged again
		tmpint = 0;
		extcnt = 0;
		R_DDR = (1<<(2*HighPin));			// High-Pin over R_L to ground
		R_PORT = 0;
		while(ReadADC(HighPin) > (ReadADC(LowPin) + 10)) {
			wdt_reset();
			tmpint++;
			if(tmpint==0) {
				extcnt++;
				if(extcnt == 30) break; // Timeout for discharge
			}
		}
		ADC_DDR = 7;	// fully discharged
		ADC_PORT = 7;
		_delay_ms(10);
		// Done
	}
	end:
	ADC_DDR = 0;				
	ADC_PORT = 0;
	R_DDR = 0;
	R_PORT = 0; 
}



unsigned int ReadADC(uint8_t mux) {
	// Read ADC value of the specified channel and return as unsigned int
	unsigned int adcx = 0;
	ADMUX = mux | (1<<REFS0);
	for(uint8_t j=0;j<20;j++) {	// 20 measurements, for better accuracy
		ADCSRA |= (1<<ADSC);
		while (ADCSRA&(1<<ADSC));
		adcx += ADCW;
	}
	adcx /= 20;
	return adcx;
}

void GetGateThresholdVoltage(void) {
	unsigned int tmpint = 0;
	unsigned int tmpintb = 0;
	unsigned long gcval = 0;
	
	uint8_t extcnt = 0;
	tmpval = (1<<(2*c) | (2<<(2*b)));
	tmpval2=(1<<(2*c));
	R_DDR = tmpval;        // Drain over R_L as output, Gate over R_H as output
	ADC_DDR=(1<<e)|(1<<b);	// Gate and Source as outputs
	if((PartFound==PART_FET) && (PartMode == PART_MODE_N_E_MOS)) {
		// Gate threshold voltage measured
		ADC_PORT = 0;			// Gate and Source to ground
		R_PORT = tmpval;  	   // Drain over R_L to Plus, Gate over R_H to Plus
		tmpval=(1<<c);
		_delay_ms(10);
		ADC_DDR=(1<<e);          // Gate over R_H load slowly
	
		while ((ADC_PIN&tmpval)) {  // Wait for the MOSFET to switch on and Drain goes low; loop takes 7 cycles
			wdt_reset();
			tmpint++;
			if(tmpint==0) {
				extcnt++;
				if(extcnt == 8) break; // Timeout for gate threshold voltage measurement
			}
		}
		
		R_PORT=tmpval2;          // Gate to high impedance
		R_DDR=tmpval2;          // Gate to high impedance
		tmpintb=ReadADC(b);

		// Gate capacitance measurement
		tmpint = 0;
		extcnt = 0;
		ADC_DDR = ((1<<e) | (1<<b) | (1<<c));	//Gate, Drain and Source as outputs
		ADC_PORT = 0;					// Gate, Drain and Source to ground
		tmpval = (2<<(2*b));			// Gate over R_H to Plus
		R_DDR = tmpval;        // Drain over R_L as output, Gate over R_H as output
		R_PORT = tmpval;  	   // Drain over R_L to Plus, Gate over R_H to Plus
		tmpval=(1<<b);
		_delay_ms(10);
		ADC_DDR=((1<<e) | (1<<c));          // Gate over R_H load slowly
		while (!(ADC_PIN & tmpval)) {  // Wait until the gate goes high; loop takes 7 cycles
			wdt_reset();
			tmpint++;
			if(tmpint==0) {
				extcnt++;
				if(extcnt == 8) break; // Timeout for gate threshold voltage measurement
			}
		}
		gcval = N_GATE_CAPACITY_FACTOR;	// Value for N-Channel MOSFET

	} else if((PartFound==PART_FET) && (PartMode == PART_MODE_P_E_MOS)) {
		ADC_PORT = (1<<e)|(1<<b);	// Gate and Source to positive
		R_PORT = 0;					// Drain over R_L to ground, Gate over R_H to ground
		tmpval=(1<<c);
		_delay_ms(10);
		ADC_DDR=(1<<e);          // Gate over R_H load slowly (Gate as input)
		ADC_PORT=(1<<e);          // Gate over R_H load slowly (Gate-Pullup off)
		while (!(ADC_PIN&tmpval)) {  // Wait until the MOSFET is turned on and switches Drain to high
			wdt_reset();
			tmpint++;
			if(tmpint==0) {
				extcnt++;
				if(extcnt == 8) break; // Timeout for gate threshold voltage measurement
			}
		}
		R_DDR=tmpval2;          // Gate to high impedance
		tmpintb=ReadADC(b);

		// Gate capacitance measurement
		tmpint = 0;
		extcnt = 0;
		ADC_DDR = ((1<<e) | (1<<b) | (1<<c));	// Gate, Drain and Source as outputs
		ADC_PORT = ((1<<e) | (1<<b) | (1<<c));	// Gate, Drain and Source to Plus
		tmpval = (2<<(2*b));			// Gate over R_H to ground
		R_DDR = tmpval;        			// Gate over R_H as output
		R_PORT = 0;  	   	  			// Gate over R_H to ground
		tmpval=(1<<b);
		_delay_ms(10);
		tmpval2 = ((1<<e) | (1<<c));	// Gate over R_H load slowly
		ADC_DDR=tmpval2;
		ADC_PORT=tmpval2;
		while (ADC_PIN & tmpval) {  // Wait for gate goes high; loop takes 7 cycles
			wdt_reset();
			tmpint++;
			if(tmpint==0) {
				extcnt++;
				if(extcnt == 8) break; // Timeout for gate threshold voltage measurement
			}
		}
		gcval = P_GATE_CAPACITY_FACTOR;	// Value for P-channel MOSFET

	}
	R_DDR = 0;
	R_PORT = 0;
	ADC_DDR = 0;
	ADC_PORT = 0;
	if((tmpint > 0) || (extcnt > 0)) {
		if(PartMode == PART_MODE_P_E_MOS) {
			tmpintb = 1023-tmpintb;
		}
		tmpintb=(tmpintb*39/8);
		utoa(tmpintb, outval, 10);
		/*
			Calculation of the gate capacitance
			When Vcc = 5V switch the AVR port pins to 3.6 V.
			At this voltage, the gate is now loaded.
			This is 72% of the operating voltage and thus yields a time constant tau of 1.28 (1.28 R * C).
			For unknown reasons, the actual value deviates significantly from it.
			The calculation can be found as factors defines the top and may need to be adjusted.
		*/
		gcval *= (unsigned long)(((unsigned long)extcnt * 65536) + (unsigned long)tmpint);	// Store value
		gcval /= 100;
		tmpint = (unsigned int)gcval;
		if(tmpint>2) tmpint -= 3;
		utoa(tmpint, outval2, 10);
	}
}

void DischargePin(uint8_t PinToDischarge, uint8_t DischargeDirection) {
	/*
		Connect a component for a short time (10 ms) to a particular potential
		This function is provided for discharging of MOSFET gate in order for the protection diodes to be seen
		Parameters:
		PinToDischarge: the pin to be discharged
		DischargeDirection: 0 = to ground (N-channel FET), 1 = connected to positive (P-channel FET)
	*/
	uint8_t tmpval;
	tmpval = (PinToDischarge * 2);		// required because of the arrangement of the resistors

	if(DischargeDirection) R_PORT |= (1<<tmpval);			//R_L aus
	R_DDR |= (1<<tmpval);			// Pin as output and over R_L to ground
	_delay_ms(10);
	R_DDR &= ~(1<<tmpval);			// Pin as input again
	if(DischargeDirection) R_PORT &= ~(1<<tmpval);			//R_L aus
}

void lcd_show_format_cap(char outval[], uint8_t strlength, uint8_t CommaPos) {
	if(strlength < 3) {
		if(strlength==1) {
			lcd_string("0.");
			lcd_data('0');
			lcd_data(outval[0]);
		} else {
			lcd_string("0.");
			lcd_data(outval[0]);
			lcd_data(outval[1]);
		}
	} else {
		for(PartReady=0;PartReady<strlength;PartReady++) {
			if((PartReady + 2) == CommaPos) lcd_data('.');
			lcd_data(outval[PartReady]);
		}
	}
}
