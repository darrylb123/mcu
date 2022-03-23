/*
  IO22D08
  DC 12V 8 Channel Pro mini PLC Board Relay Shield Module
  for Arduino Multifunction Delay Timer Switch Board
  Arduino Pro Mini

  Hardware:
  4 bit Common Cathode Digital Tube Module (two shift registers)
  8 relays (one shift register)
  8 optocoupler
  4 discrete input keys (to GND)

  ---Segment Display Screen----
  --A--
  F---B
  --G--
  E---C
  --D--
   __  __   __  __
  |__||__|.|__||__|
  |__||__|'|__||__|
  ----------------------

  available on Aliexpress: https://s.click.aliexpress.com/e/_A0tJEK

  some code parts based on the work of
  cantone-electonics
  http://www.canton-electronics.com

  this version

  by noiasca
  2021-04-01 OOP (2340/122)
  2021-03-31 initial version (2368/126)
  1999-99-99 OEM Version (2820/101)
*/

//Pin connected to latch of Digital Tube Module
// ST Store 
// de: Der Wechsel von Low auf High kopiert den Inhalt des Shift Registers in das Ausgaberegister bzw. Speicherregister
const uint8_t latchPin = A2;           
//Pin connected to clock of Digital Tube Module
// SH clock Shift Clock Pin
//de: Übernahme des Data Signals in das eigentliche Schieberegister
const uint8_t clockPin = A3;           
//Pin connected to data of Digital Tube Module
const uint8_t dataPin = 13;
//Pin connected to 595_OE of Digital Tube Module
// Output Enable to activate outputs Q0 – Q7  - first device: Relay IC
const uint8_t OE_595 = A1;             
// A4 - unused - not connected - I2C SDA - can be used for an additional LCD display
// A5 - unused - not connected - I2C SCL - can be used for an additional LCD display
// A6 - unused - not connected
// A7 - unused - not connected
const uint8_t optoInPin[] {2, 3, 4, 5, 6, A0, 12, 11};     // the input GPIO's with optocoupler - LOW active

const uint8_t keyInPin[] {7, 8, 9, 10};                    // the input GPIO's with momentary buttons - LOW active
const uint8_t noOfOptoIn = sizeof(optoInPin);              // calculate the number of opto inputs
const uint8_t noOfKeyIn = sizeof(keyInPin);                // calculate the number of discrete input keys
const uint8_t noOfRelay = 8;                               // relays on board driven connected to shift registers
byte key_value;                                            // the last pressed key
uint8_t inState[noOfOptoIn];                               // Debounced Opto State
uint32_t scanMillis;                                       // Scan Input timing
const int scanPeriod = 50;

// the base class implements the basic functionality
// which should be the same for all sketches with this hardware:
// begin()     init the hardware
// setNumber() send an integer to the internal display buffer
// pinWrite()  switch on/off a relay
// update()    refresh the display
// tick()      keep internals running
class IO22D08 {
  protected:
    uint8_t dat_buf[4];                // the display buffer - reduced to 4 as we only have 4 digits on this board
    uint8_t relay_port;                // we need to keep track of the 8 relays in this variable
    uint8_t com_num;                   // Digital Tube Common - actual digit to be shown
    uint32_t previousMillis = 0;       // time keeping for periodic calls
    uint32_t blinkMillis = 0;       // time keeping for periodic calls

    // low level HW access to shift registers
    // including mapping of pins
    void update()
    {
      static const uint8_t TUBE_NUM[4] = {0xfe, 0xfd, 0xfb, 0xf7}; // Tuble bit number - the mapping to commons
      // currently only the first 10 characters (=numbers) are used, but I keep the definitions
      //        NO.:0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22 23 24 25 26 27 28*/
      // Character :0,1,2,3,4,5,6,7,8,9,A, b, C, c, d, E, F, H, h, L, n, N, o, P, r, t, U, -,  ,*/
      const uint8_t TUBE_SEG[31] =
      {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90, // 0 .. 9
       0x88, 0x83, 0xc6, 0xa7, 0xa1, 0x86, 0x8e, 0x89, 0x8b, 0xc7, // A, b, C, c, d, E, F, H, h, L,
       0xab, 0xc8, 0xa3, 0x8c, 0xaf, 0x87, 0xc1, 0xbf, 0xff,0xcf,0xc9};      // n, N, o, P, r, t, U, -, |, || ,
      uint8_t tube_dat;                                    // Common Cathode Digital Tube, bit negated - sum of all segments to be activated
      uint8_t bit_num;                                     // digital tube common gemappt auf tuble bit number
      uint8_t display_l, display_h, relay_dat;             // three databytes of payload to be shifted to the shift registers
      if (com_num < 3) com_num ++; else com_num = 0;       // next digit
      uint8_t dat = dat_buf[com_num];                      // Data to be displayed
      tube_dat = TUBE_SEG[dat];                            // Common Cathode Digital Tube, bit negated - sum of all segments
      bit_num = ~TUBE_NUM[com_num];                        // digital tube common gemappt auf tuble bit number
      display_l  = ((tube_dat & 0x10) >> 3);     //Q4   <-D1 -3    SEG_E
      display_l |= ((bit_num  & 0x01) << 2);     //DIGI0<-D2 +2
      display_l |= ((tube_dat & 0x08) >> 0);     //Q3   <-D3 0     SEG_D
      display_l |= ((tube_dat & 0x01) << 4);     //Q0   <-D4 -4    SEG_A
      display_l |= ((tube_dat & 0x80) >> 2);     //Q7   <-D5 -2    SEG_DP - Colon - only on digit 1 ?
      display_l |= ((tube_dat & 0x20) << 1);     //Q5   <-D6 1     SEG_F
      display_l |= ((tube_dat & 0x04) << 5);     //Q2   <-D7 5     SEG_C
      // output U3-D0 is not connected, 
      // on the schematic the outputs of the shiftregisters are internally marked with Q, here we use U3-D to refeer to the latched output)
      display_h  = ((bit_num  & 0x02) >> 0);     //DIGI1<-D1 0
      display_h |= ((bit_num  & 0x04) >> 0);     //DIGI2<-D2 0
      display_h |= ((tube_dat & 0x40) >> 3);     //Q6   <-D3 -3    SEG_G
      display_h |= ((tube_dat & 0x02) << 3);     //Q1   <-D4 3     SEG_B
      display_h |= ((bit_num  & 0x08) << 2);     //DIGI3<-D5 2
      // Outputs U4-D0, U4-D6 and U4-D7 are not connected

      relay_dat = ((relay_port & 0x7f) << 1);    // map Pinout 74HC595 to ULN2803: 81234567
      relay_dat = relay_dat | ((relay_port & 0x80) >> 7);

      //ground latchPin and hold low for as long as you are transmitting
      digitalWrite(latchPin, LOW);
      // as the shift registers are daisy chained we need to shift out to all three 74HC595
      // hence, one single class for the display AND the relays ...
      // de: das ist natürlich ein Käse dass wir hier einen gemischten Zugriff auf das Display und die Relais machen müssen
      shiftOut(dataPin, clockPin, MSBFIRST, display_h);    // data for U3 - display
      shiftOut(dataPin, clockPin, MSBFIRST, display_l);    // data for U4 - display
      shiftOut(dataPin, clockPin, MSBFIRST, relay_dat);    // data for U5 - Relay
      //return the latch pin high to signal chip that it no longer needs to listen for information
      digitalWrite(latchPin, HIGH);
    }

  public:
    IO22D08() {}
     uint8_t blinkState = 1;// Blinker latch flags
    int latchLeft = 0;
    int latchRight = 0;
    const int minBlink = 20;
    uint32_t blinkDebounce;
    const int minDebounce = 500;

    void begin() {
      digitalWrite(OE_595, LOW);       // Enable Pin of first 74HC595
      pinMode(latchPin, OUTPUT);
      pinMode(clockPin, OUTPUT);
      pinMode(dataPin, OUTPUT);
      pinMode(OE_595, OUTPUT);
    }

    // fills the internal buffer for the digital outputs (relays)
    void pinWrite(uint8_t pin, uint8_t mode)
    {
      // pin am ersten shiftregister ein oder ausschalten
      if (mode == LOW)
        bitClear(relay_port, pin);
      else
        bitSet(relay_port, pin);
      update();    // optional: call the shiftout process (but will be done some milliseconds later anyway)
    }

    // this is a first simple "print number" method
    // right alligned, unused digits with zeros
    // should be reworked for a nicer print/write
    void setNumber(int display_dat)
    {
      dat_buf[0] = display_dat / 1000;
      display_dat = display_dat % 1000;
      dat_buf[1] = display_dat / 100;
      display_dat = display_dat % 100;
      dat_buf[2] = display_dat / 10;
      dat_buf[3] = display_dat % 10;
    }

    // Allow setting of arbitrary segments using additional codes
    void setSeg(uint8_t whichDisp,uint8_t segment) {
      dat_buf[whichDisp] = segment;
    }
    // this method should be called in loop as often as possible
    // it will refresh the multiplex display
    void tick() {
      uint32_t currentMillis = millis();
      if (currentMillis - previousMillis > 1)    // each two milliseconds gives a stable display on pro Mini 8MHz
      {
        update();
        previousMillis = currentMillis;
      }
      if (currentMillis - blinkMillis > 500)    // each 500 milliseconds invert the blinker state
      {
        blinkState = !blinkState;
        blinkMillis = currentMillis;
        // Count down blinks for latched blinkers
        if (latchLeft > 0)
          latchLeft--;
        if (latchRight > 0)
          latchRight--;
      }
      
    }

};



IO22D08 board;               // create an instance of the relay board with timer extension

void readInput()
{
  /* Blinker logic: 
   *  Blinkers latch & operate for a minimum of minBlink state changed
   *  unless other blinker is is operated which unlatches
   *  eg Right blinker switch initiates minBlink flashes unless the left blinker switch resets it
   */
  if (board.latchLeft)
    board.pinWrite(0,board.blinkState);
  else
    board.pinWrite(0,LOW);
  if (board.latchRight)
    board.pinWrite(1,board.blinkState);
  else
    board.pinWrite(1,LOW);
  
  if ((digitalRead(optoInPin[0]) == LOW) && (board.blinkDebounce < millis())) { // Left Blinker
    board.setSeg(0,29); // display |
    if ( board.latchRight == 0 ) {
      board.latchLeft = board.minBlink;
    } else {
      board.latchRight = 0;
      board.blinkDebounce = millis() + board.minDebounce;
      
    }
  } 

  if ((digitalRead(optoInPin[1]) == LOW) && (board.blinkDebounce < millis()) ) { // Right Blinker
    board.setSeg(0,1);
    if ( board.latchLeft == 0 ) {
      board.latchRight = board.minBlink;
    } else {
      board.latchLeft = 0;
      board.blinkDebounce = millis() + board.minDebounce;
    }
   }
    
    
  // Pair of inputs state indicators 
  if ((digitalRead(optoInPin[0]) == LOW) && (digitalRead(optoInPin[1]) == LOW)) {
    board.setSeg(0,30);
  }
  if ((digitalRead(optoInPin[0]) == HIGH) && (digitalRead(optoInPin[1]) == HIGH)) {
    board.setSeg(0,28);
  }
    
  if (digitalRead(optoInPin[2]) == LOW) { // Lights
    board.pinWrite(2,HIGH);
    board.setSeg(1,29); // display |
  } else 
    board.pinWrite(2,LOW);
    
  if (digitalRead(optoInPin[3]) == LOW) { // High Beam
    board.pinWrite(3,HIGH);
    board.setSeg(1,1);
  } else 
    board.pinWrite(3,LOW);
    
  // Pair of inputs state indicators 
  if ((digitalRead(optoInPin[2]) == LOW) && (digitalRead(optoInPin[3]) == LOW)) {
    board.setSeg(1,30);
  }
  if ((digitalRead(optoInPin[2]) == HIGH) && (digitalRead(optoInPin[3]) == HIGH)) {
    board.setSeg(1,28);
  }
    
  if (digitalRead(optoInPin[4]) == LOW) { // High Beam
    board.pinWrite(4,HIGH);
    board.setSeg(2,29);
  } else 
    board.pinWrite(4,LOW);
 
  if (digitalRead(optoInPin[5]) == LOW) { // Brake Light
    board.pinWrite(5,HIGH);
    board.setSeg(2,1);
  } else 
    board.pinWrite(5,LOW);
  // Pair of inputs state indicators 
  if ((digitalRead(optoInPin[4]) == LOW) && (digitalRead(optoInPin[5]) == LOW)) {
    board.setSeg(2,30);
  }
  if ((digitalRead(optoInPin[4]) == HIGH) && (digitalRead(optoInPin[5]) == HIGH)) {
    board.setSeg(2,28);
  } 
    
  if (digitalRead(optoInPin[6]) == LOW) { // Ignition
    board.pinWrite(6,HIGH);
    board.setSeg(3,29);
  } else 
    board.pinWrite(6,LOW);

  if (digitalRead(optoInPin[7]) == LOW) { // Start
    board.pinWrite(7,HIGH);
    board.setSeg(3,1);
  } else 
    board.pinWrite(7,LOW);
    
  // Pair of inputs state indicators 
  if ((digitalRead(optoInPin[6]) == LOW) && (digitalRead(optoInPin[7]) == LOW)) {
    board.setSeg(3,30);
  }
  if ((digitalRead(optoInPin[6]) == HIGH) && (digitalRead(optoInPin[7]) == HIGH)) {
    board.setSeg(3,28);
  } 
}

void setup() {
  //Serial.begin(9600); // slow for 8Mhz Pro Mini
  //Serial.println("\nIO22D08 board");

  for (auto &i : optoInPin) pinMode(i, INPUT_PULLUP);      // init the optocoupler
  for (auto &i : keyInPin) pinMode(i, INPUT_PULLUP);       // init the discrete input keys
  board.begin();                                           // prepare the board hardware
  board.blinkDebounce = millis();
  scanMillis = millis() + scanPeriod;
  
}

void loop() {
  
  if (millis() > scanMillis){
    scanMillis = millis() + scanPeriod;  
    readInput();            // handle input pins
  }
  board.tick();      // timekeeping for display/
}
