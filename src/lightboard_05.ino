
// pins 15~17 to GND, I2C bus address is 0x20
#include "Wire.h"

// MCP23017 registers (everything except direction defaults to 0)

#define IODIRA   0x00   // IO direction  (0 = output, 1 = input (Default))
#define IODIRB   0x01
#define IOPOLA   0x02   // IO polarity   (0 = normal, 1 = inverse)
#define IOPOLB   0x03
#define GPINTENA 0x04   // Interrupt on change (0 = disable, 1 = enable)
#define GPINTENB 0x05
#define DEFVALA  0x06   // Default comparison for interrupt on change (interrupts on opposite)
#define DEFVALB  0x07
#define INTCONA  0x08   // Interrupt control (0 = interrupt on change from previous, 1 = interrupt on change from DEFVAL)
#define INTCONB  0x09
#define IOCON    0x0A   // IO Configuration: bank/mirror/seqop/disslw/haen/odr/intpol/notimp
#define GPPUA    0x0C   // Pull-up resistor (0 = disabled, 1 = enabled)
#define GPPUB    0x0D
#define INFTFA   0x0E   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define INFTFB   0x0F
#define INTCAPA  0x10   // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define INTCAPB  0x11
#define GPIOA    0x12   // Port value. Write to change, read to obtain value
#define GPIOB    0x13
#define OLLATA   0x14   // Output latch. Write to latch output.
#define OLLATB   0x15


#define POWINT_1 = 0x01;
#define POWINT_2 = 0x02;
#define POWINT_3 = 0x04;
//other

#define ISR_INDICATOR 12  // pin 12
#define ONBOARD_LED 13    // pin 13
#define D2 2

#define NUMBOARDS 3
#define LIGHTS_PER_BOARD  3

volatile bool buttonPressed;
unsigned long time = 0;
unsigned int buttonValue = 0;
int target = 0;
int solved = 0;
int boardAddrs[NUMBOARDS] = {0x20, 0x21, 0x23};
//int boardAddrs[NUMBOARDS] = {0x23};

int lightNumToPin(int lightNum) {
  int pin = lightNum - 1;
  pin = pin % LIGHTS_PER_BOARD;
  return (0x01 << pin);
}

int lightNumToBoardIndex(int lightNum) {
  if(lightNum <= 0 || lightNum > (NUMBOARDS * LIGHTS_PER_BOARD)) {
    Serial.print("ERROR: lightNum was: ");
    Serial.print(lightNum);
    Serial.println(" returning index: 0");
    return 0;
  }

  
  return ((lightNum - 1) / LIGHTS_PER_BOARD);
}

int lightNumToBoardAddr(int lightNum ) {
  int board = -1;
  int data = lightNumToBoardIndex(lightNum);
  if(data >= 0) {
    board = boardAddrs[data];
    Serial.print("target: ");
    Serial.print(target);
    Serial.print(" data: ");
    Serial.print(data);
    Serial.print(" lightnum: ");
    Serial.print(lightNum);
    Serial.print(" boardNum: ");
    Serial.print(data, HEX);
    Serial.print(" board: ");
    Serial.println(board, HEX);
  }
  
  return board;
  
}

//addr = i2c address of the mcp23017 as defined by pins 15, 16, and 17
//reg = register bytes (see register definitions above)
//data = byte of data to send to the register
void expanderWrite(const byte addr, const byte reg, const byte data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// read a byte from the expander
unsigned int expanderRead (const byte addr, const byte reg) 
{
  Wire.beginTransmission (addr);
  Wire.write (reg);
  Wire.endTransmission ();
  Wire.requestFrom ((int) addr, (int) 1);
  return Wire.read();
} // end of expanderRead


void initBoard(int boardAddr) {

  // set I/O pins to outputs
  expanderWrite(boardAddr, IODIRA, 0xFF); // IODIRA, all port A to inputs (8 inputs)
  expanderWrite(boardAddr, IODIRB, 0x00); // IODIRB, all port B to outputs (8 outputs)
  expanderWrite(boardAddr, IOCON, 0b01000000);  //mirror interrupts
  expanderWrite(boardAddr, GPPUA, 0xFF); //enable pull-up resistors on all port A pins
  expanderWrite(boardAddr, GPINTENA, 0xFF); //enable interrupts on port A pins
  expanderWrite(boardAddr, IOPOLA, 0xFF); //invert port A polarity so 1 = pressed
  expanderWrite(boardAddr, GPIOB, 0x00);
  expanderRead(boardAddr, INTCAPA); //read from port A interrupt capture port to clear it
  
}

void setup() {
  Serial.begin(9600);

  randomSeed(analogRead(0));
  target = random(1, 10);

  Serial.println("setup init");
  Serial.print("target: ");
  Serial.println(target);

  pinMode (ISR_INDICATOR, OUTPUT);  // for testing (ISR indicator)
  pinMode (ONBOARD_LED, OUTPUT);  // for onboard LED
  pinMode(D2, INPUT);
  digitalWrite(2, HIGH);
  digitalWrite (ONBOARD_LED, LOW);  // 

  buttonPressed = false;

  Wire.begin(); // wake up I2C bus
  Serial.println("woke up I2C bus");
  
  for(int i=0; i<NUMBOARDS; i++) {
    initBoard(boardAddrs[i]);
  }
  
  Serial.println("about to attach interrupt");
  // pin 19 of MCP23017 is plugged into D2 of the Arduino which is interrupt 0
  attachInterrupt(0, buttonpress, FALLING);

  interrupts();
  Serial.println("setup complete");
  Serial.println();
}

// interrupt service routine, called when pin D2 goes from 1 to 0
void buttonpress () {
  digitalWrite (ISR_INDICATOR, HIGH);  // debugging
  buttonPressed = true;   // set flag so main loop knows
}  // end of keypress

// called from main loop when we know we had an interrupt
void handleButtonPress () {

  Serial.println("Handling button Press");
  delay (100);  // de-bounce before we re-enable interrupts
  buttonPressed = false;  // ready for next time through the interrupt service routine
  digitalWrite (ISR_INDICATOR, LOW);  // debugging


  //TODO: need to capture the state of all boards
  unsigned int boardInterrupts[NUMBOARDS]  = {0x00};
  for(unsigned int i=0; i<NUMBOARDS; i++) {
    // Read port values, as required. Note that this re-arms the interrupts.
    if (expanderRead(boardAddrs[i], INFTFA)) {
      boardInterrupts[i] &= 0xFF00;
      boardInterrupts[i] |= expanderRead (boardAddrs[i], INTCAPA);        // port A is in low-order byte
    } else {
      boardInterrupts[i] = 0x00;
    }
  }

  int targetBoard = lightNumToBoardIndex(target);
  if(boardInterrupts[targetBoard]) {
    time = millis ();  // remember when
    if(boardInterrupts[targetBoard] == lightNumToPin(target)) {
        Serial.println("Target!");
        solved = 1;
    }
  }  // end if

  Serial.print("button value: ");
  Serial.print(boardInterrupts[targetBoard]);
  Serial.print(" target value: ");
  Serial.println(lightNumToPin(target));
  
}  // end of handleKeypress



void loop() {
  

  int i, oldTarget;

  if(solved == 1) {
    solved = 0;
  }

  oldTarget = target;
  while (oldTarget == target) {
    target = random(1, 10);
  }
  
  for(i=0; i<4; i++) {   

     checkButton();
    
    if(solved == 1) {
      Serial.println("Solved!");
      break;
    }
    
    delay(200);		
    on(target, 6);
    checkButton();

    delay(800);
    off(6);
  }
  
  delay(200);
  
  for(i=0; i<8; i++) { 

    checkButton();  

    if(solved == 1) {
      Serial.println("Solved!");
      break;
    }
    
    delay(100);		
    on(target, 6);
    checkButton();
    
    delay(400);
    off(6);

  }


  delay(1200);
 
}

void on(int lightNum, int soundPin) {

    int pin = lightNumToPin(target);
    int boardAddr = lightNumToBoardAddr(target);
    Serial.print("turning on pin: ");
    Serial.print(pin, HEX);
    Serial.print(", board: ");
    Serial.println(boardAddr, HEX);
    Wire.beginTransmission(boardAddr);
    Wire.write(GPIOB); // GPIOB
    Wire.write(pin); // port B (all)
    Wire.endTransmission();
  
    // play a note on pin 6 for 200 ms:
    tone(soundPin, 290, 200);
  
}

void off(int soundPin) {
  
  // play a note on pin 6 for 200 ms:
    noTone(soundPin);

    for(unsigned int i=0; i<NUMBOARDS; i++) {
      Wire.beginTransmission(boardAddrs[i]);
      Wire.write(GPIOB); // GPIOB
      Wire.write(0x00); // port B
      Wire.endTransmission();
    }
  
}

void checkButton() {

  // was there an interrupt?
  if (buttonPressed) {
    handleButtonPress ();
  }
}



