/***************************************************************************************
  Name    : LED CUBE 8x8x8 74HC595
  By      : Liam Jackson

  /*Combined with Voxel512.ino to work with Voxel512 cube with an Arduino uno by Jonathan P. Powell, DMD   http://www.clockworksdental.com/

  Based on code by Joseph Francis (74hc595 SPI)
  and by chr at instructables
  http://www.instructables.com/id/Led-Cube-8x8x8/step70/Run-the-cube-on-an-Arduino/
  Font found somewhere
****************************************************************************************/
/* Modified to work with Voxel
   Adriaan Delport
   Hackable Designs
 ***************************************************************************************/

/***************************************************************************************
   Name   : Calibrtion Cube
   By     : Jimmy Tabet
   
   Modified to work with calibration projects for NEL
   ****INSTALL 'TimerOne' PACKAGE FROM Sketch -> Include Library -> Manage Libraries****
 ***************************************************************************************/

// ==========================================================================================
//   USER INPUT - to change pattern, go to calib function at bottom
// ==========================================================================================
int d_on = 10;    // time (in ms) LED stays on
int d_off = 50;   // time (in ms) between LED's turning on
int d_end = 10e3; // time (in ms) to delay at end of loop (make large to stop recording/turn off arduino

// ==========================================================================================
//   SETUP - not sure what this does so I left it all
// ==========================================================================================

#include <TimerOne.h>
#define AXIS_X 1
#define AXIS_Y 2
#define AXIS_Z 3

//--- Pins for shift registers
#define SHIFTREGISTER_PORT  PORTD
#define SER_DATA_OUT        0x10
#define SER_SHFT_CLK        0x08
#define SER_LOAD_CLK        0x04
//--- Pin connected to ST_CP of 74HC595
int latchPin = 2;
//--- Pin connected to SH_CP of 74HC595
int clockPin = 3;
//--- Pin connected to DS of 74HC595
int dataPin = 4;
//--- Used for faster latching
int latchPinPORTB = 0x04;

//---Plane Selection
int PlaneSel3 = A0;
int PlaneSel2 = A1;
int PlaneSel1 = A2;
int PlaneSel0 = A3;

int PlaneEn_n = 5;

//-- Buttons
int But1 = 9;
int But0 = 8;

//holds value for all the pins, [x][y][z]
byte cube[8][8];

const float Test[8][8] =
{ { 4.949747468, 4.301162634, 3.807886553, 3.535533906, 3.535533906, 3.807886553, 4.301162634, 4.949747468},
  {4.301162634, 3.535533906, 2.915475947, 2.549509757, 2.549509757, 2.915475947, 3.535533906, 4.301162634},
  {3.807886553, 2.915475947, 2.121320344, 1.58113883, 1.58113883, 2.121320344, 2.915475947, 3.807886553},
  {3.535533906, 2.549509757, 1.58113883, 0.707106781, 0.707106781, 1.58113883, 2.549509757, 3.535533906},
  {3.535533906, 2.549509757, 1.58113883, 0.707106781, 0.707106781, 1.58113883, 2.549509757, 3.535533906},
  {3.535533906, 4.301162634, 4.301162634, 4.301162634, 4.301162634, 4.301162634, 4.301162634, 4.301162634},
  {4.301162634, 3.535533906, 2.915475947, 2.549509757, 2.549509757, 2.915475947, 3.535533906, 4.301162634},
  {4.949747468, 4.301162634, 3.807886553, 3.535533906, 3.535533906, 3.807886553, 4.301162634, 4.949747468}
};

//Counts through the layers
int current_layer = 0;

// LED CUBE Overall Brightness
int Brightness = 0;
// Temp Button Value
int valButton = HIGH;
//--- This process is run by the timer and does the PWM control
void iProcess() {
  //last layer store
  int oldLayerBit = current_layer + 2;

  //increment layer count
  current_layer++;
  if (current_layer >= 8) {
    current_layer = 0;
  }

  //--- Run through all the shift register values and send them (last one first)
  // latching in the process
  latchOff();
  for (int i = 0 ; i < 8 ; i++) {
    ShiftRegisterByteTransfer(cube[current_layer][i]);
  }

  //Hide the old layer
  //digitalWrite(oldLayerBit, LOW);
  digitalWrite(PlaneSel3, HIGH);

  //New data on the pins
  latchOn();
  //new layer high
  //digitalWrite(current_layer + 2, HIGH);
  SelectLayer(current_layer);
  digitalWrite(PlaneSel3, LOW);
  latchOn();
}

//--- Direct port access latching
void latchOn() {
  //bitSet(PORTB,latchPinPORTB);
  SHIFTREGISTER_PORT |= SER_LOAD_CLK;
}
void latchOff() {
  //bitClear(PORTB,latchPinPORTB);
  SHIFTREGISTER_PORT &= ~(SER_LOAD_CLK);
}

//--- Used to setup SPI based on current pin setup
//    this is called in the setup routine;
void setupSPI() {
  byte clr;
  SPCR |= ( (1 << SPE) | (1 << MSTR) ); // enable SPI as master
  SPCR &= ~( (1 << SPR1) | (1 << SPR0) ); // clear prescaler bits
  clr = SPSR; // clear SPI status reg
  clr = SPDR; // clear SPI data reg
  SPSR |= (1 << SPI2X); // set prescaler bits
  delay(10);
}

void SelectLayer(int _iLayer)
{
  switch (_iLayer)
  {
    case 0:
      digitalWrite(PlaneSel2, LOW);
      digitalWrite(PlaneSel1, LOW);
      digitalWrite(PlaneSel0, LOW);
      break;
    case 1:
      digitalWrite(PlaneSel2, LOW);
      digitalWrite(PlaneSel1, LOW);
      digitalWrite(PlaneSel0, HIGH);
      break;
    case 2:
      digitalWrite(PlaneSel2, LOW);
      digitalWrite(PlaneSel1, HIGH);
      digitalWrite(PlaneSel0, LOW);
      break;
    case 3:
      digitalWrite(PlaneSel2, LOW);
      digitalWrite(PlaneSel1, HIGH);
      digitalWrite(PlaneSel0, HIGH);
      break;
    case 4:
      digitalWrite(PlaneSel2, HIGH);
      digitalWrite(PlaneSel1, LOW);
      digitalWrite(PlaneSel0, LOW);
      break;
    case 5:
      digitalWrite(PlaneSel2, HIGH);
      digitalWrite(PlaneSel1, LOW);
      digitalWrite(PlaneSel0, HIGH);
      break;
    case 6:
      digitalWrite(PlaneSel2, HIGH);
      digitalWrite(PlaneSel1, HIGH);
      digitalWrite(PlaneSel0, LOW);
      break;
    case 7:
      digitalWrite(PlaneSel2, HIGH);
      digitalWrite(PlaneSel1, HIGH);
      digitalWrite(PlaneSel0, HIGH);
      break;
  }
}
void ShiftRegisterByteTransfer(byte data)
{
  for (int i = 0; i < 8; i++)
  {
    // Set data pin to match data bit
    if ((data >> i) & 0x1 == 1)
      SHIFTREGISTER_PORT |= SER_DATA_OUT;
    else
      SHIFTREGISTER_PORT &= ~(SER_DATA_OUT);

    // Toggle shift clock to load in bit
    SHIFTREGISTER_PORT ^= SER_SHFT_CLK;
    SHIFTREGISTER_PORT ^= SER_SHFT_CLK;
  }
}
//--- The really fast SPI version of shiftOut
byte spi_transfer(byte data)
{
  SPDR = data;        // Start the transmission
  loop_until_bit_is_set(SPSR, SPIF);
  return SPDR;        // return the received byte, we don't need that
}

void setup() {
  Serial.begin(9600);

  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  digitalWrite(latchPin, LOW);
  digitalWrite(dataPin, LOW);
  digitalWrite(clockPin, LOW);

  pinMode(PlaneSel3, OUTPUT);
  pinMode(PlaneSel2, OUTPUT);
  pinMode(PlaneSel1, OUTPUT);
  pinMode(PlaneSel0, OUTPUT);
  pinMode(PlaneEn_n, OUTPUT);

  digitalWrite(PlaneSel3, HIGH);
  digitalWrite(PlaneSel2, LOW);
  digitalWrite(PlaneSel1, LOW);
  digitalWrite(PlaneSel0, LOW);
  analogWrite(PlaneEn_n, Brightness);
  digitalWrite(PlaneEn_n, LOW);

  pinMode(But1, INPUT);    // declare pushbutton as input
  pinMode(But0, INPUT);    // declare pushbutton as input

  //--- Setup to run SPI
  //setupSPI();
  //-- Wait for button press before starting demo
  //  while (valButton == HIGH)
  //    valButton = digitalRead(But1);
  //--- Activate the PWM timer
  Timer1.initialize(500); // Timer for updating pwm pins
  Timer1.attachInterrupt(iProcess);
}

// ==========================================================================================
//   RUN
// ==========================================================================================

void loop() {
  // turn each LED on one by one with d delay in between
  calib(d_on, d_off);
  
  // ensure everything is off when done
  fill(0X00);
  
  // if loop repeats forever, put in large delay at end so arduino can be turned off
  delay(d_end);
}


void calib(int d_on, int d_off)
{
  int x,y,z;
  for (x = 0; x < 8; ++x) {
    for (z = 0; z < 8; ++z) {
      for (y = 0; y < 8; ++y) {
        clrvoxel(x, y, z);        // no idea why but this needs to be run first?
        setvoxel(x,y,z);
        delay(d_on);
        clrvoxel(x,y,z);
        delay(d_off);
      }
    }
  }
}

// ==========================================================================================
//   DRAW FUNCTIONS
// ==========================================================================================

// Set a single voxel to ON
void setvoxel(int x, int y, int z)
{
  if (inrange(x, y, z))
    cube[z][y] |= (1 << x);
}

// Set a single voxel to OFF
void clrvoxel(int x, int y, int z)
{
  if (inrange(x, y, z))
    cube[z][y] &= ~(1 << x);
}

// This function validates that we are drawing inside the cube.
unsigned char inrange(int x, int y, int z)
{
  if (x >= 0 && x < 8 && y >= 0 && y < 8 && z >= 0 && z < 8)
  {
    return 0x01;
  } else
  {
    // One of the coordinates was outside the cube.
    return 0x00;
  }
}

// Fill a value into all 64 byts of the cube buffer
// Mostly used for clearing. fill(0x00)
// or setting all on. fill(0xff)
void fill (unsigned char pattern)
{
  int z;
  int y;
  for (z = 0; z < 8; ++z)
  {
    for (y = 0; y < 8; ++y)
    {
      cube[z][y] = pattern;
    }
  }
}
