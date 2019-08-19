#define pin_bridge_a 10
#define pin_bridge_b 11


/*
 * https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
 * Timer0: default use is for delay()/millis()/micros() function
 * Timer1: 16 bit, defaults to servo library
 * Timer2: default use is for tone() function
 * 
 * http://sphinx.mythic-beasts.com/~markt/ATmega-timers.html
 * 
 */

uint32_t g_tick_sec_sub_micros = 0; // 0 to 999999
uint32_t g_tick_sec = 0;
uint8_t  g_tick_flag = 0; //Set on each ms increment, to be reset by handler function

void setup() {

  Serial.begin(115200);

  //Timer/Counter0
  //COUNTER mode on pin D4
  //8 bits, set to increment on rising edge
    OCR0A = 255;   //256 counts
    TCCR0A = _BV(WGM10) | _BV(WGM11) | _BV(COM1A0); //   
    TCCR0B = _BV(WGM12) | _BV(WGM13) | _BV(CS12) | _BV(CS11);

  //Timer/Counter1
  //COUNTER mode on pin D5
  //16 bits, set to increment on rising edge
  
    OCR1A = 255;   //256 counts
    TCCR1A = _BV(WGM10) | _BV(WGM11) | _BV(COM1A0); //   
    TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS12) | _BV(CS11); //input pin D5 

  //Timer/Counter2
  //Timer mode: Clear Timer on Compare Match (CTC)
  //Will reset TCNT2 to 0 and trigger OCF2A flag
  TCCR2A = bit(WGM21);              // CTC Mode, output compare pin disconnected
  TCCR2B = bit(CS22);               // /64 pre-scaling (16e6/64 = 250kHz)
  OCR2A = 250;                      // compare A register value (250e3/250 = 1ms)
  //OCR2A = 125;                      // compare A register value (250e3/125 = 0.5ms)
  //OCR2A = 50;                       // compare A register value (250e3/250 = 0.2ms)
  //OCR2A = 5;                        // compare A register value (250e3/250 = 0.1ms)
  TIMSK2 = bit(OCIE2A);             // interrupt on Compare A Match
  
  interrupts();             // enable all interrupts


  //H Bridge output
  pinMode(pin_bridge_a, OUTPUT);
  pinMode(pin_bridge_b, OUTPUT);
}


ISR(TIMER2_COMPA_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  //Timer currently set to 10ms intervals, = 1000uS
  g_tick_sec_sub_micros += 1000; 
  if(g_tick_sec_sub_micros >= 1000000)
  {
    g_tick_sec_sub_micros = 0;
    ++g_tick_sec;
  }
  g_tick_flag = 1;
}

/*
 * The periodic_event function is where you should put the following:
 * -Diagnostic printing
 * -Closed loop control algorithm
 */
void periodic_event(    uint32_t period_ms,
                        uint32_t encoder_position_now, 
                        uint32_t encoder_position_prev,
                        uint32_t encoder_position_prev_prev)
{
  
  digitalWrite(pin_bridge_a, HIGH);
  digitalWrite(pin_bridge_b, LOW);

  // digitalWrite(pin_bridge_a, LOW);
  // digitalWrite(pin_bridge_b, HIGH);  

  //Print diagnostic info
  /*
  Serial.print(g_tick_sec);
  Serial.print(",");
  Serial.print(encoder_position);
  Serial.println("");
  */

  //Print s, s', s''
  int32_t pos    = encoder_position_now;
  int32_t pos_p  = encoder_position_now-encoder_position_prev;

  int16_t pulse_per_rev = 4;

  /*
   * Use the following with serial plotter in arduino IDE
   */
  //Serial.print(pos);
  //Serial.print(",");
  Serial.print(pos_p);
  Serial.print(",");
  Serial.print((double(pos_p)/(double)pulse_per_rev)*(1000.0/(double)period_ms) * 60.0);
  Serial.println("");
}

void loop() {

  uint16_t overflows_T0 = 0;
  uint16_t overflows_T1 = 0;

  uint16_t count_T0 = 0;
  uint16_t count_T1 = 0;
  
  int32_t  count_abs_0 = 0;
  int32_t  count_abs_1 = 0;
  int32_t  count_abs_2 = 0;

  while(true)
  {

    //Handle hardware counter overflows
    if(TIFR0 & (1<<OCF0A)) 
    {
      TIFR0 = _BV(OCF0A);
      ++overflows_T0;
    }
    if(TIFR1 & (1<<OCF1A)) 
    {
      TIFR1 = _BV(OCF1A);
      ++overflows_T1;
    }

    //On system tick
    if(1 == g_tick_flag)
    {
      g_tick_flag = 0;

      //Grab snapshot of counters
      count_T0 = overflows_T0*256 + TCNT0;
      count_T1 = TCNT1;

      //Execute period event, useful for control loops and printing diagnostics
      //Update period, valid numbers should be:
      //-integer multiples of 1
      //-factors of 1000
      //Examples: 1,2,4,5,10,20,40, 100, 1000 etc
      uint32_t event_period_ms = 100;
      if(0 == g_tick_sec_sub_micros % (event_period_ms * (uint32_t)1000) )
      {
        count_abs_2 = count_abs_1;
        count_abs_1 = count_abs_0;
        count_abs_0 = (int32_t)count_T0 - (int32_t)count_T1;
        
        periodic_event(event_period_ms, count_abs_0, count_abs_1, count_abs_2);
      }
    }

   
  }//End of infinite loop *_0
    
}
