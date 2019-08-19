#include <PID_v1.h>

//Pins for PWM controlled H bridge
#define PIN_BRIDGE_A  10
#define PIN_BRIDGE_B  11

//Pins for axis-encoder board
#define PIN_QUAD_UP   5
#define PIN_QUAD_DN   4
#define PIN_LIM_1     3
#define PIN_LIM_2     2

#define CONTROL_PERIOD_MILLISECONDS 10

typedef struct{
  uint32_t  microsNow     = 100;
  uint32_t  microsPre     = 0;
  int32_t   microsPeriod  = 1000000; //Arbitrary initial period of 1s to prevent NaN / Inf
  int32_t   counter       = 0;
  int32_t   counterRange  = 500;
  uint8_t   cyclesPerRev  = 0; //Requires initialising elsewhere
  double    shaftRPM      = 0;
  double    shaftRPM_max  = 10000.0; //Estimate to allow normalisation of control
}axisEncoder_t;

axisEncoder_t encD;


/*
 * TODO
 * 
 * normalise inputs and outputs of loops elsewhere so the below parameters are
 * insensitive to design changes
 */

double velSetNorm, velInNorm, velOutNorm; //All normalised to 1
PID velPID(&velInNorm, &velOutNorm, &velSetNorm,4,200,0, DIRECT); //2, 200, 0

double posSetNorm, posInNorm, posOutNorm; //All normalised to 1
PID posPID(&posInNorm, &posOutNorm, &posSetNorm,2,0,0, DIRECT);

//PORTB
ISR (PCINT0_vect)
{    
  //Nothing connected
}

//PORTC
ISR (PCINT1_vect)
{
  //Nothing connected
}  

//PORTD
ISR (PCINT2_vect)
{
  //counter tracks position
  //period tracks velocity (negative period will result in negative velocity)
  if(1 == digitalRead(PIN_QUAD_UP))
  {
    ++encD.counter;
    encD.microsPre = encD.microsNow;
    encD.microsNow = micros();
    encD.microsPeriod = +(int32_t)(encD.microsNow - encD.microsPre);
  }
  
  if(1 == digitalRead(PIN_QUAD_DN))
  {
    --encD.counter;
    encD.microsPre = encD.microsNow;
    encD.microsNow = micros();
    encD.microsPeriod = -(int32_t)(encD.microsNow - encD.microsPre);
  }
  
}  

void pciSetup(byte pin)
{
    /* Pin to interrupt map:
     * D0-D7 = PCINT 16-23 = PCIR2 = PD = PCIE2 = pcmsk2
     * D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
     * A0-A5 (D14-D19) = PCINT 8-13 = PCIR1 = PC = PCIE1 = pcmsk1
     */
  
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}
 
void setup() 
{  
  Serial.begin(115200);

  //H Bridge output, initialised to both sides off
  pinMode(PIN_BRIDGE_A, OUTPUT);
  pinMode(PIN_BRIDGE_B, OUTPUT);
  digitalWrite(PIN_BRIDGE_A, LOW);
  digitalWrite(PIN_BRIDGE_B, LOW);
  //analogWrite(PIN_BRIDGE_B, 100);
  //https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/
  //PWM frequency is 490Hz, this is too slow
  //Consider a pwm shield:
  //1.6kHz https://www.adafruit.com/product/1411

  //PORTD encoder pins init
  pinMode(PIN_QUAD_UP,INPUT);
  pinMode(PIN_QUAD_DN,INPUT);
  pinMode(PIN_LIM_1,  INPUT);
  pinMode(PIN_LIM_2,  INPUT);
  //PORTD encoder pin change interrupts enable
  pciSetup(PIN_QUAD_UP);
  pciSetup(PIN_QUAD_DN);
  //pciSetup(PIN_LIM_1); //not implemented
  //pciSetup(PIN_LIM_2); //not implemented

  //Configure encoder
  encD.cyclesPerRev = 16;
  /*
   * Careful with counts per rev, which is different from cycles per rev.
   * Counts per rev refers to the number of quadrature state changes per rev
   * which is 4 transitions per cycle.
   * Therefore, cycles per revolution = counts per revolution /4
   */
    
  //turn the velocity control PID system on
  velInNorm = 0;
  velSetNorm = 0;
  velPID.SetSampleTime(CONTROL_PERIOD_MILLISECONDS); 
  velPID.SetOutputLimits(-1,1);
  velPID.SetMode(AUTOMATIC);

  //turn the position control PID system on
  posInNorm = 0;
  posSetNorm = 0;
  posPID.SetSampleTime(CONTROL_PERIOD_MILLISECONDS); 
  posPID.SetOutputLimits(-1,1);
  posPID.SetMode(AUTOMATIC);

  
}



//The following variables determine the range of PWM duty cycle used in seeking the home
byte seekSpeeds[] = {255, 127, 64, 32};

void seekLimit(int directionSign)
{
  //for each step size
  for(byte i = 0; i < sizeof(seekSpeeds)/sizeof(seekSpeeds[0]); ++i)
  {
    byte seekSpeed = seekSpeeds[i];

    Serial.println((String)"Seeking limit at speed "+seekSpeed+"...");
    
    //Go to home
    if(directionSign > 0)
    {
      digitalWrite(PIN_BRIDGE_A, LOW);
      analogWrite(PIN_BRIDGE_B, seekSpeed);
    }
    if(directionSign < 0)
    {
      digitalWrite(PIN_BRIDGE_B, LOW);
      analogWrite(PIN_BRIDGE_A, seekSpeed);
    }

    while(0 == digitalRead(PIN_LIM_1))
    {
      //wait
    }
    //when at home, reverse direction until off home again

    if(directionSign > 0)
    {
      digitalWrite(PIN_BRIDGE_B, LOW);
      analogWrite(PIN_BRIDGE_A, seekSpeed);
    }
    if(directionSign < 0)
    {
      digitalWrite(PIN_BRIDGE_A, LOW);
      analogWrite(PIN_BRIDGE_B, seekSpeed);
    }

    //Time delay is more reliable than looking for a change of state
    //As a change of state may be falsely triggered by contact bounce
    delay(250);
    
    //once cleared home, turn motor off
    digitalWrite(PIN_BRIDGE_A, LOW);
    digitalWrite(PIN_BRIDGE_B, LOW);
  }
}

void velControlUpdateRPM(double setRPM)
{
    velSetNorm = setRPM/encD.shaftRPM_max;

    double encoderFreq = 1/((double)(encD.microsPeriod)/1000000.0);
    double shaftFreq = encoderFreq/encD.cyclesPerRev;
    encD.shaftRPM = shaftFreq * 60.0;
    velInNorm = encD.shaftRPM/encD.shaftRPM_max; //10000rpm assumed max speed of shaft (allows normalisation of control)
   
    //This allows us to reduce the sensitivity of the loop
    //As we get closer, this is an alternative to deadbands
    //And will prevent the motor from hunting due to backlash etc
    double eNorm = velSetNorm - velInNorm;
    velPID.SetTunings(100.0*abs(eNorm), 50.0, 0);
    velPID.Compute();
    
    byte controlByte = (byte)abs(velOutNorm*255.0);

      if(velOutNorm >= 0.0)
      {
        digitalWrite(PIN_BRIDGE_A, LOW);
        analogWrite(PIN_BRIDGE_B, controlByte);
      }
      if(velOutNorm < 0.0)
      {
        digitalWrite(PIN_BRIDGE_B, LOW);
        analogWrite(PIN_BRIDGE_A, controlByte);
      }

}

void posControlUpdateCnt(double input_zero_to_one)
{
    posSetNorm = input_zero_to_one;
    posInNorm = (double)encD.counter/(double)encD.counterRange;

    posPID.Compute();

      double setRPM = posOutNorm * encD.shaftRPM_max;
      velControlUpdateRPM(setRPM);
  
}

 double state = 1.0;
 bool flagRangeInitialised = false;
void loop() {

  /* Initialise motion control range
   *  
   */

  if (false == flagRangeInitialised)
  {
    seekLimit(-1);
    //reset quadrature counters after motor has settled
    delay(1000);
    encD.counter = 0;
    
    seekLimit(+1);
    //observe quadrature counters after motor has settled
    delay(1000);
    encD.counterRange = encD.counter;
    Serial.println((String)"Quadrature range count: "+encD.counterRange);

    //Go to home -1 using closed loop feedback from encoder and trapezoidal motion profile
    

    flagRangeInitialised = true;
  }


  //every 10ms, print the counter to the screen
  if(millis() % CONTROL_PERIOD_MILLISECONDS == 0)
  {
    double timeSeconds = (double)millis()/1000.0;

    //The position shall follow the full range of motion allowable by home/limits
    //posSet = 500.0*(sin(2.0*PI*0.5*timeSeconds)+1)/2.0;
    
    //velControlUpdateRPM(3000.0);
    double freqHz = 0.5;
    double peakRPM = 6000.0;
    velControlUpdateRPM(peakRPM*sin(2.0*PI*freqHz*timeSeconds));


    if(millis() % 5000 == 0)
    {
      //flip state
      state = (double)rand()/double(RAND_MAX);
    }
    //posControlUpdateCnt(state);

    //posControlUpdateCnt(abs(0.01*sin(2.0*PI*freqHz*timeSeconds)));
    

    
    Serial.print(encD.counter);
    Serial.print(",");
    Serial.print(encD.shaftRPM);
    Serial.print(",");
    Serial.print(velSetNorm*encD.shaftRPM_max);
    Serial.print(",");
    Serial.print(velOutNorm*10000.0);
    Serial.print(",");
    Serial.print(posSetNorm*(double)encD.counterRange);
    //Serial.print(",");
    //Serial.print(encD.microsPeriod);
    Serial.println("");
  }
  
}
