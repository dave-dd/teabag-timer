/*
 * teabag-timer is an Arduino based device that autmatically lifts a teabag and measures the tee's temperature
 * Copyright (C) 2014 David Darmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Dieses Programm ist Freie Software: Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
 * veröffentlichten Version, weiterverbreiten und/oder modifizieren.
 * 
 * Dieses Programm wird in der Hoffnung, dass es nützlich sein wird, aber
 * OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
 * Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License für weitere Details.
 * 
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
 */
 

#include <Servo.h>
#include <Encoder.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>     // Core graphics library
#include "Adafruit_ILI9340.h" // Hardware-specific library
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>
#include <math.h>
#include "Due_TeaBagRemover.h"

/* Servo Stuff */
Servo servo_cord;
Servo servo_arm;
Servo servo_sensor;
int servo_cord_pos= SERVO_CORD_MAX;
int servo_arm_pos= SERVO_ARM_MAX;
int servo_sensor_pos= SERVO_SENSOR_MAX;

int8_t servo_increment= -1;

/* Knob and Button Stuff */
Encoder knob;
int knob_value= 0;
boolean button_knob_value= HIGH;
boolean button_knob_handled= 1;
unsigned long button_knob_last_press_time= -1;
boolean button_white_value= HIGH;
boolean button_white_handled= 1;
unsigned long button_white_last_press_time= -1;
boolean button_red_value= HIGH;
boolean button_red_handled= 1;
unsigned long button_red_last_press_time= -1;


/* Audio Stuff */
Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, SD_CS);

/* LED Strip Stuff */
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

/* TFT Stuff */
Adafruit_ILI9340 tft = Adafruit_ILI9340(TFT_CS, TFT_DC, TFT_RST);

/* Global Vars */
state_type state= START;
uint steeping_time= 240; //seconds, equals 4 min
uint end_temp= 60;       //degrees C
uint8_t arm_pos_leds= 1;
uint8_t steeping_time_leds= 0;
unsigned long last_temp_measurement_time= 0;

unsigned long steeping_start_time= 0;
unsigned long last_repaint_time= 0;
int temp_int_val= 0;
uint remaining_steeping_time= 0; //seconds
uint remaining_time= -1;
int last_led_refresh_temp_int_val= 0;
uint32_t temp_color= 0;

double temperature= 0;
double peak_temperature= 0;
unsigned long peak_time= -1;
double avg_end_temp_time= 0;
unsigned long end_temp_counter= 0;
uint32_t new_remaining_temp_time= -1;
uint32_t remaining_temp_time= -1;

int8_t knob_change_history[4]= {0};
unsigned long knob_time_history[4]= {0};
uint32_t knob_counter= 0;

void setup() {
  pinMode(AMPLIFIER_SHDN, OUTPUT);
  disableAmplifier();                 //shutdown amplifier
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);         //silence display
  pinMode(VS1053_SDCS, OUTPUT);
  digitalWrite(VS1053_SDCS, HIGH);    //silence unused sd card reader

  analogReadResolution(12);
  pinMode(TEMP_SENSOR_PIN, INPUT);

  Serial.begin(9600);
  
  // Init SD Card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("Initialized SD Card");
  
  //Init VS1053 Audio Decoder
  int player_begin_retval= musicPlayer.begin();
  if (!player_begin_retval) {
    Serial.print("VS1053 not found, return value is ");
    Serial.println(player_begin_retval);
    while (1);  // don't do anything more
  }
  musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT);
  Serial.print("Initialized Music Player, return value was ");
  Serial.println(player_begin_retval);
  musicPlayer.setVolume(20,20);
  Serial.println("Set volume");
  musicPlayer.dumpRegs();

  //Init LED Strip
  strip.begin();
  initLEDs();

  //Init TFT
  tft.begin();
  bmpDraw("start.bmp", 0, 0);
  //bmpDraw("begintxt.bmp", 0, 0);

  //Init Knob
  knob.init(ENCODER_PIN1, ENCODER_PIN2);

  pinMode(BUTTON_KNOB_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RED_PIN, INPUT_PULLUP);
  pinMode(BUTTON_WHITE_PIN, INPUT_PULLUP);
  attachInterrupt(BUTTON_KNOB_PIN, buttonKnobISR, CHANGE);
  attachInterrupt(BUTTON_RED_PIN, buttonRedISR, CHANGE);
  attachInterrupt(BUTTON_WHITE_PIN, buttonWhiteISR, CHANGE);

  //Init Servos
  servo_cord.attach(SERVO_CORD_PIN);
  servo_cord.write(servo_cord_pos+1);
  delay(100);
  servo_cord.write(servo_cord_pos);

  servo_arm.attach(SERVO_ARM_PIN);
  servo_arm.write(servo_arm_pos+1);
  delay(100);
  servo_arm.write(servo_arm_pos);
  
  servo_sensor.attach(SERVO_SENSOR_PIN);
  servo_sensor.write(servo_sensor_pos+1);
  delay(100);
  servo_sensor.write(servo_sensor_pos);
  delay(200);

  //Play Welcome Sound
  musicPlayer.startPlayingFile("intro.mp3");
  enableAmplifier();
}


void loop() {
  if(button_knob_value == LOW && !button_knob_handled && (millis()-button_knob_last_press_time) > DEBOUNCE_DELAY){
    KnobPressed();
    button_knob_handled= 1;
  }
  if(button_white_value == LOW && !button_white_handled && (millis()-button_white_last_press_time) > DEBOUNCE_DELAY){
    WhitePressed();
    button_white_handled= 1;
  }
  if(button_red_value == LOW && !button_red_handled && (millis()-button_red_last_press_time) > DEBOUNCE_DELAY){
    RedPressed();
    button_red_handled= 1;
  }

  /* Knob Reading */
  int new_knob_value;
  new_knob_value = knob.read()/2;
  if(new_knob_value != knob_value) {
    KnobTurned(new_knob_value - knob_value);
    knob_value = new_knob_value;
  }

  if(isAmplifierEnabled() && !musicPlayer.playingMusic){
    disableAmplifier();
  }

  if(state == WAITING_FOR_START && millis()-last_temp_measurement_time > 1000){
  // if(millis()-last_temp_measurement_time > 1000){
    temperature= getTemperature();
    Serial.print("Temperature: ");
    Serial.println(temperature);
    uint new_waiting_temp_int_val= (uint)(temperature*10+0.5);
    Serial.print("Temperature integer value: ");
    Serial.println(new_waiting_temp_int_val);
    repaintWaitingTemp(new_waiting_temp_int_val);
    temperatureLEDs(new_waiting_temp_int_val);
    temp_int_val= new_waiting_temp_int_val;

    Serial.print("temp_int_val= ");
    Serial.println(temp_int_val);

    if(temp_int_val >= 500){
      lowerTeabag();
      state= TEABAG_IN;
    }

    last_temp_measurement_time= millis();
  }

  if((state == TEABAG_IN || state == WAITING_FOR_END_TEMP) && millis()-last_repaint_time > 1000){
    last_repaint_time= millis();
    temperature= getTemperature();
    uint new_temp_int_val= (uint)(temperature*10+0.5);
    repaintTeabagIn(new_temp_int_val);
    repaintRemainingTimeToEndTemp();
    temperatureLEDs(new_temp_int_val);
    temp_int_val= new_temp_int_val;
  }

  //Peak Temperature Detection
  if(state == TEABAG_IN){
    if(temperature > peak_temperature){
      peak_temperature= temperature;
      peak_time= millis();
      Serial.print("New Peak is ");
      Serial.println(peak_temperature);
    }
  }
}

void repaintRemainingTimeToEndTemp(){
  //Check if peak is at least 60 seconds old
  unsigned long ts_now= millis();
  uint32_t end_temp_time_seconds= ts_now;
  if(peak_time < ts_now && (ts_now - peak_time) > 60000){
    // Serial.println("Peak is at least 60 seconds old, start guessing...");
    if(temperature > end_temp){
      double r_temp= rtemp_approx();
      double a= peak_approx()- r_temp;
      double b= (1.0/((ts_now-peak_time)/1000.0))*log((temperature-r_temp)/a);

      double end_temp_time = (1/b)*log(((long)end_temp-r_temp)/a);

      const float alpha= 0.1;

      if(avg_end_temp_time == 0){
        avg_end_temp_time= end_temp_time;
      }else{
        avg_end_temp_time= alpha*end_temp_time + (1-alpha)*avg_end_temp_time;
      }

      end_temp_time_seconds= (uint32_t)(avg_end_temp_time+peak_time/1000+0.5);
    }
    Serial.print("Estimated end time: ");
    Serial.print(end_temp_time_seconds);
    Serial.println(" s");

    if((ts_now >= end_temp_time_seconds*1000 || temperature < (end_temp-0.1)) && state==WAITING_FOR_END_TEMP){
      state= DONE;
    }
    end_temp_counter++;

    if(end_temp_counter%10 == 0){
      //Update Estimate
      if(new_remaining_temp_time == -1){
        bmpDraw("time_0.bmp", 33, 257);
      }
      new_remaining_temp_time= end_temp_time_seconds-ts_now/1000;
    }else{
      if(new_remaining_temp_time != -1){
        new_remaining_temp_time= remaining_temp_time-1;
      }
    }


    if(new_remaining_temp_time != -1){
      //Repaint
      char filename_buffer[15];
      uint8_t seconds= new_remaining_temp_time%60;
      uint8_t seconds_tens= seconds/10;
      uint8_t seconds_ones= seconds%10;

      uint8_t minutes_tens= (new_remaining_temp_time/60)/10;
      uint8_t minutes_ones= (new_remaining_temp_time/60)%10;    

      if((remaining_temp_time/60)/10 != minutes_tens){
        sprintf(filename_buffer, "nr_%d.bmp", minutes_tens);
        bmpDraw(filename_buffer, 33, 257);
      }

      if((remaining_temp_time/60)%10 != minutes_ones){
        sprintf(filename_buffer, "nr_%d.bmp", minutes_ones);
        bmpDraw(filename_buffer, 72, 257);
      }

      if(remaining_temp_time/10 != seconds_tens){
        sprintf(filename_buffer, "nr_%d.bmp", seconds_tens);
        bmpDraw(filename_buffer, 129, 257);    
      }
      sprintf(filename_buffer, "nr_%d.bmp", seconds_ones);
      bmpDraw(filename_buffer, 168, 257);    

      remaining_temp_time= new_remaining_temp_time;          
    }
    if(state == DONE){
      liftTemperatureSensor();
      bmpDraw("done.bmp", 0, 0);

      char filename_buffer[15];
      uint8_t degrees_tens= end_temp/10;
      uint8_t degrees_ones= end_temp%10;

      sprintf(filename_buffer, "t_nr_%d.bmp", degrees_tens);
      bmpDraw(filename_buffer, 97, 258);
      sprintf(filename_buffer, "t_nr_%d.bmp", degrees_ones);
      bmpDraw(filename_buffer, 108, 258);

      musicPlayer.startPlayingFile("outro.mp3");
      enableAmplifier();

    }
  }
}

double peak_approx(){
  double correction= 21.980197041340592 - 0.9751109070063636*temperature + 0.019384565147841027*pow(temperature,2) - 0.00012695121874592227*pow(temperature,3);
  double peak_temp_approx= peak_temperature- constrain(correction, 0, 6);
  if((peak_temp_approx - temperature) < 2){
    Serial.println("Difference between peak and temperature is less than 2");
    return peak_temperature;
  }else{
    return peak_temp_approx;
  }
}

double rtemp_approx(){
  return constrain(35 + exp(-17.38676484432635 + 0.2190439945832813*temperature), 34, 40);
}

void repaintTeabagIn(uint new_temp_int_val){
  char filename_buffer[15];

  if(state != WAITING_FOR_END_TEMP){  
    long new_remaining_steeping_time= (((long)steeping_start_time+((long)steeping_time*1000))-(long)millis())/1000;
    // Serial.print("New remaining steeping time: ");
    // Serial.println(new_remaining_steeping_time);
    if(new_remaining_steeping_time < 0){
      liftTeabag();
      state= WAITING_FOR_END_TEMP;
    }else{
      uint8_t seconds= new_remaining_steeping_time%60;
      uint8_t seconds_tens= seconds/10;
      uint8_t seconds_ones= seconds%10;

      uint8_t minutes_tens= (new_remaining_steeping_time/60)/10;
      uint8_t minutes_ones= (new_remaining_steeping_time/60)%10;

      if((remaining_steeping_time/60)/10 != minutes_tens){
        sprintf(filename_buffer, "nr_%d.bmp", minutes_tens);
        bmpDraw(filename_buffer, 33, 42);
      }

      if((remaining_steeping_time/60)%10 != minutes_ones){
        sprintf(filename_buffer, "nr_%d.bmp", minutes_ones);
        bmpDraw(filename_buffer, 72, 42);
      }

      if(remaining_steeping_time/10 != seconds_tens){
        sprintf(filename_buffer, "nr_%d.bmp", seconds_tens);
        bmpDraw(filename_buffer, 129, 42);    
      }
      sprintf(filename_buffer, "nr_%d.bmp", seconds_ones);
      bmpDraw(filename_buffer, 168, 42);    

      remaining_steeping_time= new_remaining_steeping_time;
    }
  }

  uint8_t degrees_tens= new_temp_int_val/100;
  uint8_t degrees_ones= (new_temp_int_val/10)%10;
  uint8_t degrees_tenths= new_temp_int_val%10;

  if((temp_int_val/100) != degrees_tens){
    sprintf(filename_buffer, "nr_%d.bmp", degrees_tens);
    bmpDraw(filename_buffer, 11, 151);
  }
  if((temp_int_val/10)%10 != degrees_ones){
    sprintf(filename_buffer, "nr_%d.bmp", degrees_ones);
    bmpDraw(filename_buffer, 50, 151);    
  }
  if(temp_int_val%10 != degrees_tenths){
    sprintf(filename_buffer, "nr_%d.bmp", degrees_tenths);
    bmpDraw(filename_buffer, 108, 151);    
  }

}

void lowerTeabag(){
  for(uint8_t i= servo_cord_pos; i >= SERVO_CORD_MIN; i--){
    servo_cord_pos= i;
    servo_cord.write(servo_cord_pos);
    delay(20);
  }
  bmpDraw("tea_in.bmp", 0, 0);
  temp_int_val= 600;
  steeping_start_time= millis();
}

void liftTeabag(){
  for(uint8_t i= servo_cord_pos; i <= SERVO_CORD_MAX; i++){
    servo_cord_pos= i;
    servo_cord.write(servo_cord_pos);
    delay(20);
  }
}

double getTemperature(){
  uint adc_read= 0;
  const double u_q= 3.31; //Volts
  const double r_1= 22430; //Ohm
  const double temp_a= 0.0009085560162130007;
  const double temp_b= 0.00023867799417482118;
  const double temp_c= 5.993722882854634e-8;

  for(uint8_t i=0; i<10; i++){
    adc_read+= analogRead(TEMP_SENSOR_PIN);
    delay(5);
  }
  adc_read/= 10;
  double u_t= u_q-(adc_read*(u_q/4096));
  double r_temp= r_1/((u_q/u_t)-1);
  double temp= 1/(temp_a + temp_b*log(r_temp) + temp_c*pow(log(r_temp),3)) - 273.15;
  double correction= -0.34246170354326105+ temp*0.0019588532019141707 + temp*temp*0.0001933676099855523;
  if(correction < 0){
    correction= 0;
  }
  temp-= correction;
  Serial.print((double)millis()/1000);
  Serial.print(", ");
  Serial.println(temp);
  return temp;
}

void KnobPressed(){
  Serial.println("Knob pressed.");

  switch (state) {
      case START:
        Serial.println("State was START, doing LED stuff and going to SETUP");
        introLEDs(strip.Color(0,0,255));
        state= SETUP_STEEPING_TIME;
        bmpDraw("setup.bmp", 0, 0);
        tft.drawRoundRect(20, 60, 200, 81, 7, tft.Color565(142, 142, 142));
        steepingTimeLEDs(steeping_time);
        break;
      case SETUP_STEEPING_TIME:
        Serial.println("State was SETUP.");
        tft.drawRoundRect(20, 60, 200, 81, 7, tft.Color565(255, 255, 255));
        tft.drawRoundRect(20, 221, 200, 81, 7, tft.Color565(142, 142, 142));
        state= SETUP_END_TEMP;
        endTemperatureLEDs(end_temp);
        break;
      case SETUP_END_TEMP:
        tft.drawRoundRect(20, 221, 200, 81, 7, tft.Color565(255, 255, 255));
        tft.drawRoundRect(20, 60, 200, 81, 7, tft.Color565(142, 142, 142));
        state= SETUP_STEEPING_TIME;
        break;
      case LOWER_TEMP_SENSOR:
        resetLEDs();
        bmpDraw("water.bmp", 0, 0);
        temp_int_val= 0;
        state= WAITING_FOR_START;
        break;
      default:
        Serial.println("STATE was not defined!");
        break;
  }
}

void KnobTurned(int8_t change){
  Serial.print("Knob turned, change= ");
  Serial.println(change);
  
  int16_t knob_histroy_sum= 0;
  unsigned long ts_now= millis();
  for(uint8_t i=0; i<4; i++){
    if(ts_now - knob_time_history[i] < 500){
      knob_histroy_sum+= knob_change_history[i];
    }
  }

  knob_time_history[knob_counter%4]= ts_now;
  if((knob_histroy_sum > 3 && change < 1) || (knob_histroy_sum < -3 && change > 1)){
    knob_change_history[knob_counter%4]= 0;
    return;
  }else{
    knob_change_history[knob_counter%4]= change;
  }

  knob_counter++;

  int new_steeping_time= 0;
  int new_end_temp= 0;
  switch (state) {
      case SETUP_STEEPING_TIME:
        new_steeping_time= steeping_time + (change*5);
        if(new_steeping_time <= 600 && new_steeping_time >= 5 ){
          steepingTimeLEDs(new_steeping_time);
          repaintSteepingTime(new_steeping_time);
          steeping_time= new_steeping_time;
        }
        break;
      case SETUP_END_TEMP:
        new_end_temp= end_temp + change;
        if(new_end_temp <= 70 && new_end_temp >= 40 ){
          endTemperatureLEDs(new_end_temp);
          repaintEndTemp(new_end_temp);
          end_temp= new_end_temp;
        }
        break;
      case LOWER_TEMP_SENSOR:
        servo_sensor_pos+= change;
        servo_sensor.write(servo_sensor_pos);
        break;
      default:
        // do nothing
        break;
  }
}

void RedPressed(){
  Serial.println("Red button pressed.");
  // resetServos();
  // initLEDs();
  // state= START;
  if(state == LOWER_TEMP_SENSOR){
    while(digitalRead(BUTTON_RED_PIN) == LOW){
      //sensor up again
      if(servo_arm_pos < SERVO_ARM_MAX){
        armPosLEDs();
        servo_arm_pos++;
        servo_sensor_pos= calcServoSensorFromServoArm();
        servo_sensor.write(servo_sensor_pos);
        servo_arm.write(servo_arm_pos);
        // Serial.print("Arm-Servo Position is now ");
        // Serial.print(servo_arm_pos);
        // Serial.print(", ");
        // Serial.println(servo_sensor_pos);
        delay(40);
      }else{
        servo_sensor.write(servo_sensor_pos+1);
        delay(100);
        servo_sensor.write(servo_sensor_pos);
        return;
      }
    }
  }
  if(state == WAITING_FOR_END_TEMP){
    liftTemperatureSensor();
  }
}

void liftTemperatureSensor(){
  for(uint8_t i= servo_arm_pos; i <= SERVO_ARM_MAX; i++){
      servo_arm_pos= i;
      servo_sensor_pos= calcServoSensorFromServoArm();
      servo_sensor.write(servo_sensor_pos);
      servo_arm.write(servo_arm_pos);
      // Serial.print("Arm-Servo Position is now ");
      // Serial.print(servo_arm_pos);
      // Serial.print(", ");
      // Serial.println(servo_sensor_pos);
      delay(40);
  }
  servo_sensor.write(servo_sensor_pos+1);
  delay(100);
  servo_sensor.write(servo_sensor_pos);
}

void WhitePressed(){
  Serial.println("White button pressed.");

  if(state == SETUP_END_TEMP || state == SETUP_STEEPING_TIME){
    Serial.println("Could now put down temperature sensor.");
    resetLEDs();
    strip.setPixelColor(0,strip.Color(0,0,50));
    strip.show();
    state= LOWER_TEMP_SENSOR;
    bmpDraw("tempsens.bmp", 0, 0);
    return;
  }
  if(state == LOWER_TEMP_SENSOR){
    while(digitalRead(BUTTON_WHITE_PIN) == LOW){
      if(servo_arm_pos > SERVO_ARM_MIN){
        armPosLEDs();
        servo_arm_pos--;
        servo_sensor_pos= calcServoSensorFromServoArm();
        servo_sensor.write(servo_sensor_pos);
        servo_arm.write(servo_arm_pos);
        // Serial.print("Arm-Servo Position is now ");
        // Serial.print(servo_arm_pos);
        // Serial.print(", ");
        // Serial.println(servo_sensor_pos);
        delay(40);
      }else{
        servo_sensor.write(servo_sensor_pos-1);
        delay(100);
        servo_sensor.write(servo_sensor_pos);
        servo_arm.write(servo_arm_pos-1);
        delay(100);
        servo_arm.write(servo_arm_pos);
        return;
      }
    }
  }

  if(state == WAITING_FOR_START){
    lowerTeabag();
    state= TEABAG_IN;
  }
}

int calcServoSensorFromServoArm(){
  double sensor_pos= 14.710668961683725 - 265.46336120933404*exp(-0.15768582232739814*servo_arm_pos) + 0.9230871805688087*servo_arm_pos - 0.001571486799838978*pow(servo_arm_pos,2);
  return (int) (sensor_pos+0.5);
  // return servo_sensor_pos;
}

void initLEDs(){
  resetLEDs(); //turn all LEDs off
  strip.setPixelColor(7,strip.Color(0, 0, 50));
  strip.setPixelColor(8,strip.Color(0, 0, 50));
  strip.show();
}

void resetLEDs(){
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, 0);
  }
  strip.show();
}

void endTemperatureLEDs(int temperature){
  uint8_t number_leds= map(temperature, 40, 70, 1, 16);
  uint32_t color= valueToColor(map(temperature, 40, 70, 255, 0));
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    if(i < number_leds){
      strip.setPixelColor(i, color);
    }else{
      strip.setPixelColor(i, 0);
    }
  }
  strip.show();
}

void temperatureLEDs(int new_temperature_int_val){
  // Serial.print("Temperature diff: ");
  // Serial.println(abs(new_temperature_int_val - last_led_refresh_temp_int_val));
  uint32_t new_color= valueToColor(map(constrain(new_temperature_int_val, 400, 700), 400, 700, 255, 0));
  if(abs(new_temperature_int_val - last_led_refresh_temp_int_val) > 5 && new_color != temp_color){
    // uint8_t number_leds= map(constrain(remaining_time, 0, ))
    Serial.println("Refreshed LEDs.");
    for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, new_color);
      //delay(random(100));
    }
    strip.show();
    last_led_refresh_temp_int_val= new_temperature_int_val;
    temp_color= new_color;
  }
}

void steepingTimeLEDs(int new_steeping_time){
  uint8_t number_leds= map(new_steeping_time, 5, 600, 1, 16);
  boolean changed= false;

  //New number is lower than current, switch off some LEDs
  for(uint8_t i= number_leds; i < steeping_time_leds; i++){
    strip.setPixelColor(i, 0);
    // Serial.print("Switched off LED with index ");
    // Serial.println(i);
    changed= true;
  }

  //New number is higher, switch on some LEDs
  for(uint8_t j= steeping_time_leds; j < number_leds; j++){
    strip.setPixelColor(j, strip.Color(15, 150, 14));
    // Serial.print("Switched on LED with index ");
    // Serial.println(j);
    changed= true;
  }

  if(changed){
    steeping_time_leds= number_leds;
    strip.show();
  }
}

void armPosLEDs(){
  uint8_t number_leds= map(servo_arm_pos, 25, 146, 16, 0);
  boolean changed= false;

  // Serial.print("Number of LEDs to switch on: ");
  // Serial.println(number_leds);

  //New number is lower than current, switch off some LEDs
  for(uint8_t i= number_leds; i < arm_pos_leds; i++){
    strip.setPixelColor(i, 0);
    // Serial.print("Switched off LED with index ");
    // Serial.println(i);
    changed= true;
  }

  //New number is higher, switch on some LEDs
  for(uint8_t j= arm_pos_leds; j < number_leds; j++){
    strip.setPixelColor(j, strip.Color(0,0,50));
    // Serial.print("Switched on LED with index ");
    // Serial.println(j);
    changed= true;
  }

  if(changed){
    arm_pos_leds= number_leds;
    strip.show();
  }
}

uint32_t valueToColor(int value) {

  // Serial.print("Called valueToColor with argument ");
  // Serial.println(value);
  uint8_t rgb[3]= {0,0,0};
  int offset= 64;

  for(uint8_t i= 0; i < 3; i++){
    offset= ((i-1)*128);
    int val= value - offset;
    if(val <= 64 && val >= 0){
      rgb[i]= 4*val;
    }
    if(val <= 192 && val > 64){
      rgb[i]= 255;
    }
    if(val <= 255 && val > 192){
      rgb[i]= -4*(val-192)+255;
    }
  }

  // Serial.print("Calculated color rgb(");
  // Serial.print(rgb[0]);
  // Serial.print(", ");
  // Serial.print(rgb[1]);
  // Serial.print(", ");
  // Serial.print(rgb[2]);
  // Serial.println(")");

  return strip.Color(rgb[0], rgb[1], rgb[2]);
}

void introLEDs(uint32_t color){
  for(uint16_t i=0; i<8; i++) {
    strip.setPixelColor(8+i, color);
    strip.setPixelColor(7-i, color);
    strip.setPixelColor(8+(i-1),0);
    strip.setPixelColor(7-(i-1),0);
    strip.show();
    delay(20);
  }
  resetLEDs();
}

void repaintWaitingTemp(uint new_waiting_temp){
  uint8_t degrees_tens= new_waiting_temp/100;
  uint8_t degrees_ones= (new_waiting_temp/10)%10;
  uint8_t degrees_tenths= new_waiting_temp%10;

  char filename_buffer[15];

  if((temp_int_val/100) != degrees_tens){
    sprintf(filename_buffer, "t_nr_%d.bmp", degrees_tens);
    bmpDraw(filename_buffer, 152, 286);
  }

  if((temp_int_val/10)%10 != degrees_ones){
    sprintf(filename_buffer, "t_nr_%d.bmp", degrees_ones);
    bmpDraw(filename_buffer, 164, 286);    
  }

  if(temp_int_val%10 != degrees_tenths){
    sprintf(filename_buffer, "t_nr_%d.bmp", degrees_tenths);
    bmpDraw(filename_buffer, 181, 286);    
  }
}

void repaintSteepingTime(int new_steeping_time){
  uint8_t seconds= new_steeping_time%60;
  uint8_t seconds_tens= seconds/10;
  uint8_t seconds_ones= seconds%10;
  
  uint8_t minutes_tens= (new_steeping_time/60)/10;
  uint8_t minutes_ones= (new_steeping_time/60)%10;

  char filename_buffer[15];

  if((steeping_time/60)/10 != minutes_tens){
    sprintf(filename_buffer, "nr_%d.bmp", minutes_tens);
    bmpDraw(filename_buffer, 33, 76);    
  }

  if((steeping_time/60)%10 != minutes_ones){
    sprintf(filename_buffer, "nr_%d.bmp", minutes_ones);
    bmpDraw(filename_buffer, 71, 76);    
  }

  if(steeping_time/10 != seconds_tens){
    sprintf(filename_buffer, "nr_%d.bmp", seconds_tens);
    bmpDraw(filename_buffer, 129, 76);
  }

  sprintf(filename_buffer, "nr_%d.bmp", seconds_ones);
  bmpDraw(filename_buffer, 168, 76);
}

void repaintEndTemp(int new_end_temp){
  uint8_t ones= new_end_temp%10;
  uint8_t tens= new_end_temp/10;
  
  char filename_buffer[15];

  if(end_temp/10 != tens){
    sprintf(filename_buffer, "nr_%d.bmp", tens);
    bmpDraw(filename_buffer, 40, 238);    
  }

  sprintf(filename_buffer, "nr_%d.bmp", ones);
  bmpDraw(filename_buffer, 79, 238);    
}

void bmpDraw(char *filename, uint16_t x, uint16_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if((x >= tft.width()) || (y >= tft.height())) return;

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    read32(bmpFile);
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    read32(bmpFile);
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed
        goodBmp = true; // Supported BMP format -- proceed!
        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;
        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.Color565(r,g,b));
          } // end pixel
        } // end scanline
      } // end goodBmp
    }
  }
  bmpFile.close();
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File & f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File & f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

void buttonKnobISR(){
  button_knob_value= digitalRead(BUTTON_KNOB_PIN);
  button_knob_last_press_time= millis();
  button_knob_handled= 0;
}

void buttonWhiteISR(){
  button_white_value= digitalRead(BUTTON_WHITE_PIN);
  button_white_last_press_time= millis();
  button_white_handled= 0;
}

void buttonRedISR(){
  button_red_value= digitalRead(BUTTON_RED_PIN);
  button_red_last_press_time= millis();
  button_red_handled= 0;  
}