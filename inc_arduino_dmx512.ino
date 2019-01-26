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

// Pin variables
#define RX_PIN 0
#define TX_PIN 1

#define ADDRESS 0
#define CHANNELS 3
unsigned int currentChannel = 0;

// DMX variables

volatile unsigned int dmxcurrent = 0;         //counter variable that is incremented every time we receive a value.
volatile boolean dmxnewvalue = false;         //set to 1 when updated dmx values are received
volatile byte dmxvalue[CHANNELS];             //DMX values

// Timer2 variables
volatile byte zerocounter = 0;
/* a counter to hold the number of zeros received in sequence on the serial receive pin.
   When we've received a minimum of 11 zeros in a row, we must be in a break.  */

int outputPins[] = {5, 8, 9, 3, 4, 6};

#define DEBUG

#ifdef DEBUG
#include <SoftwareSerial.h>
SoftwareSerial mySerial(11, 12);
#endif

#define potPin    A0
#define enA       5
#define in1       9
#define in2       8

void setup() {

  //#ifdef DEBUG
  //  mySerial.begin(115200);
  //  mySerial.println("Start");
  //#endif

  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < sizeof(outputPins); i++) {
    pinMode(outputPins[i], OUTPUT);
  }

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

  //  delay(500);

}  //end setup()

void loop()  {
  // the processor gets parked here while the ISRs are doing their thing.

  if (dmxnewvalue == 1) {
    digitalWrite(LED_BUILTIN, HIGH);

    //    for (int i = 0; i < sizeof(outputPins); i++) {
    //      analogWrite(outputPins[i], dmxvalue[i]);
    //#ifdef DEBUG
    //      mySerial.print(dmxvalue[i]);
    //      mySerial.print(",");
    //#endif
    //    }
    //#ifdef DEBUG
    //    mySerial.println("");
    //#endif

    SlideToValue(map(dmxvalue[1], 0, 255, 0, 1023));

    SlideToValue2(map(dmxvalue[0], 0, 255, 0, 1023));
                   
    dmxnewvalue = 0;
    zerocounter = 0;           //and then when finished reset variables and enable timer2 interrupt
    delay(10);

    bitSet(TIMSK2, OCIE2A);    //Enable Timer/Counter2 Output Compare Match A Interrupt
    digitalWrite(LED_BUILTIN, LOW);
  }
} //end loop()

void SlideToValue(int targetValue) {
  int val = analogRead(potPin);
  //  int targetValue = analogRead(potPin2);
  if (abs(val - targetValue) > 1) {
    if (val > targetValue) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    } else {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    //    analogWrite(enA, max(min(abs(val - targetValue), 255), 200));
    analogWrite(enA, map(abs(val - targetValue), 1, 1023, 60, 250));
  } else {
    // Turn off motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  }
}

void SlideToValue2(int targetValue) {
  int val = analogRead(A1);
  //  int targetValue = analogRead(potPin2);
  if (abs(val - targetValue) > 1) {
    if (val > targetValue) {
      digitalWrite(3, LOW);
      digitalWrite(4, HIGH);
    } else {
      digitalWrite(3, HIGH);
      digitalWrite(4, LOW);
    }
    //    analogWrite(enA, max(min(abs(val - targetValue), 255), 200));
    analogWrite(6, map(abs(val - targetValue), 1, 1023, 60, 250));
  } else {
    // Turn off motor
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    analogWrite(6, 0);
  }
}

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
