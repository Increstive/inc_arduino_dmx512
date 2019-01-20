/***********************************************************
  DMX-512 Reception
  Developed by Max Pierson
  Version Rev15 9 Oct 2010
  Released under the WTFPL license, although I would
  appreciate Attribution and Share-Alike
  See blog.wingedvictorydesign.com for the latest version.

  Bonus's Version 20 Jan 2019
  Original Code >> https://www.maxpierson.me/2009/03/20/receive-dmx-512-with-an-arduino/2/

************************************************************/

// Addressing variables
#define NUMBER_OF_CHANNELS 512
unsigned int dmxaddress = 1;

// Pin variables
#define RX_PIN 0
#define TX_PIN 1

// DMX variables

volatile byte dmxreceived = 0;                //the latest received value
volatile unsigned int dmxcurrent = 0;         //counter variable that is incremented every time we receive a value.
volatile byte dmxvalue[NUMBER_OF_CHANNELS];   //DMX values
volatile boolean dmxnewvalue = false;         //set to 1 when updated dmx values are received

// Timer2 variables
volatile byte zerocounter = 0;
/* a counter to hold the number of zeros received in sequence on the serial receive pin.
   When we've received a minimum of 11 zeros in a row, we must be in a break.  */

void setup() {

  for (int i = 0; i < NUMBER_OF_CHANNELS; i++) {
    dmxvalue[i] = 0;
  }

  // DEBUG Config
  Serial2.begin(115200);
  Serial2.println("Start");
  
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(11, OUTPUT);

  // Pin Mode
  pinMode(RX_PIN, INPUT);  //sets serial pin to receive data

  // * USART configuration * //

  Serial.begin(250000);
  /* Each bit is 4uS long, hence 250Kbps baud rate */

  cli(); //disable interrupts while we're setting bits in registers

  bitClear(UCSR0B, RXCIE0);  //disable USART reception interrupt

  // Timer2 configuration

  //NOTE:  this will disable PWM on pins 3 and 11.
  bitClear(TCCR2A, COM2A1);
  bitClear(TCCR2A, COM2A0); //disable compare match output A mode
  bitClear(TCCR2A, COM2B1);
  bitClear(TCCR2A, COM2B0); //disable compare match output B mode
  bitSet(TCCR2A, WGM21);
  bitClear(TCCR2A, WGM20);  //set mode 2, CTC.  TOP will be set by OCRA.

  bitClear(TCCR2B, FOC2A);
  bitClear(TCCR2B, FOC2B);  //disable Force Output Compare A and B.
  bitClear(TCCR2B, WGM22);  //set mode 2, CTC.  TOP will be set by OCRA.
  bitClear(TCCR2B, CS22);
  bitClear(TCCR2B, CS21);
  bitSet(TCCR2B, CS20);   // no prescaler means the clock will increment every 62.5ns (assuming 16Mhz clock speed).

  OCR2A = 64;
  /* Set output compare register to 64, so that the Output Compare Interrupt will fire
     every 4uS.  */

  bitClear(TIMSK2, OCIE2B);  //Disable Timer/Counter2 Output Compare Match B Interrupt
  bitSet(TIMSK2, OCIE2A);    //Enable Timer/Counter2 Output Compare Match A Interrupt
  bitClear(TIMSK2, TOIE2);   //Disable Timer/Counter2 Overflow Interrupt Enable

  sei();                     //reenable interrupts now that timer2 has been configured.

}  //end setup()

void loop()  {
  // the processor gets parked here while the ISRs are doing their thing.

  if (dmxnewvalue == 1) {
    byte curr = dmxvalue[4];
    Serial2.println(curr);
    
    analogWrite(5, dmxvalue[4]);
    analogWrite(6, dmxvalue[5]);
    analogWrite(11, dmxvalue[6]);

    dmxnewvalue = 0;
    zerocounter = 0;           //and then when finished reset variables and enable timer2 interrupt
    bitSet(TIMSK2, OCIE2A);    //Enable Timer/Counter2 Output Compare Match A Interrupt
  }
} //end loop()

//Timer2 compare match interrupt vector handler
ISR(TIMER2_COMPA_vect) {
  if (bitRead(PIND, PIND0)) {  // if a one is detected, we're not in a break, reset zerocounter.
    zerocounter = 0;
  }
  else {
    zerocounter++;             // increment zerocounter if a zero is received.
    if (zerocounter == 20)     // if 20 0's are received in a row (80uS break)
    {
      bitClear(TIMSK2, OCIE2A);    //disable this interrupt and enable reception interrupt now that we're in a break.
      bitSet(UCSR0B, RXCIE0);
      dmxcurrent = 0;
    }
  }
} //end Timer2 ISR

ISR(USART0_RX_vect) {
  
  dmxreceived = UDR0;

  /* The receive buffer (UDR0) must be read during the reception ISR, or the ISR will just
     execute again immediately upon exiting. */

  dmxvalue[dmxcurrent] = dmxreceived;

  if (++dmxcurrent > NUMBER_OF_CHANNELS - 1) {
    bitClear(UCSR0B, RXCIE0);
    dmxnewvalue = 1;
    dmxcurrent = 0;
  }

} // end ISR
