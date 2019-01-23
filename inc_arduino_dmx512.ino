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

#define ADDRESS 0
#define CHANNELS 3
unsigned int currentChannel = 0;

// DMX variables

volatile unsigned int dmxcurrent = 0;         //counter variable that is incremented every time we receive a value.
volatile boolean dmxnewvalue = false;         //set to 1 when updated dmx values are received
volatile byte dmxvalue[CHANNELS];   //DMX values

// Timer2 variables
volatile byte zerocounter = 0;
/* a counter to hold the number of zeros received in sequence on the serial receive pin.
   When we've received a minimum of 11 zeros in a row, we must be in a break.  */

int outputPins[] = {5, 6, 9};

#define DEBUG

#ifdef DEBUG
#include <SoftwareSerial.h>
SoftwareSerial mySerial(7, 8);
#endif

void setup() {

//#ifdef DEBUG
//  mySerial.begin(115200);
//  mySerial.println("Start");
//#endif

  pinMode(LED_BUILTIN, OUTPUT);
  //  pinMode(5, OUTPUT);
  //  pinMode(6, OUTPUT);
  //  pinMode(9, OUTPUT);
  //  pinMode(11, OUTPUT); MEGA

  for (int i = 0; i < sizeof(outputPins); i++) {
    pinMode(outputPins[i], OUTPUT);
  }

  //  pinMode(8, OUTPUT);

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
  
  delay(500);
  
}  //end setup()

void loop()  {
  // the processor gets parked here while the ISRs are doing their thing.

  if (dmxnewvalue == 1) {
    digitalWrite(LED_BUILTIN, HIGH);

    for (int i = 0; i < sizeof(outputPins); i++) {
      analogWrite(outputPins[i], dmxvalue[i]);
      //#ifdef DEBUG
      //      mySerial.print(dmxvalue[i]);
      //      mySerial.print(",");
      //#endif
    }
    //#ifdef DEBUG
    //    mySerial.println("");
    //#endif

    //    analogWrite(5, dmxvalue[0]);
    //    analogWrite(6, dmxvalue[1]);
    //    analogWrite(9, dmxvalue[2]);
    //    analogWrite(11, dmxvalue[2]); MEGA

    dmxnewvalue = 0;
    zerocounter = 0;           //and then when finished reset variables and enable timer2 interrupt
    delay(1);

    bitSet(TIMSK2, OCIE2A);    //Enable Timer/Counter2 Output Compare Match A Interrupt
    digitalWrite(LED_BUILTIN, LOW);
  }
} //end loop()

//Timer2 compare match interrupt vector handler
ISR(TIMER2_COMPA_vect) {

  //#ifdef DEBUG
  //  mySerial.println(zerocounter);
  //#endif

  //  if (!bitRead(PINE, PINE0)) {  // Arduino Mega Italy
  //  if (bitRead(PIND, PIND0)) {  // Arduino Mega Chaina
  if (!bitRead(PIND, PIND0)) {  // Arduino Uno
    zerocounter = 0;
  }
  else {
    zerocounter++;             // increment zerocounter if a zero is received.
    if (zerocounter == 20)     // if 20 0's are received in a row (80uS break)
    {
      bitClear(TIMSK2, OCIE2A);    //disable this interrupt and enable reception interrupt now that we're in a break.
      bitSet(UCSR0B, RXCIE0);
      //#ifdef DEBUG
      //      mySerial.println("Start RX");
      //#endif
    }
  }
} //end Timer2 ISR

//ISR(USART0_RX_vect) { // Arduino Mega
ISR(USART_RX_vect) { // Arduino Uno

  byte dmxreceived = UDR0;
  /* The receive buffer (UDR0) must be read during the reception ISR, or the ISR will just
     execute again immediately upon exiting. */
  int realChannel = (dmxcurrent - 4) % NUMBER_OF_CHANNELS;

  dmxcurrent++;

  if (realChannel >= ADDRESS && realChannel <= ADDRESS + CHANNELS) {
    dmxvalue[currentChannel] = dmxreceived;
    if (++currentChannel == CHANNELS) {
      dmxnewvalue = 1;
      dmxcurrent = 0;
      currentChannel = 0;
      bitClear(UCSR0B, RXCIE0);
      //#ifdef DEBUG
      //      mySerial.println("Start RX");
      //#endif
    }
  }

} // end ISR
