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


#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

/* Servo Stuff */
#define SERVO_CORD_PIN 7
#define SERVO_ARM_PIN 8
#define SERVO_SENSOR_PIN 9

#define SERVO_CORD_MAX 165
#define SERVO_CORD_MIN 20
#define SERVO_ARM_MAX 146
#define SERVO_ARM_MIN 25
#define SERVO_SENSOR_MAX 117
#define SERVO_SENSOR_MIN 53

/* Knob and Button Stuff */
#define ENCODER_PIN1 47
#define ENCODER_PIN2 45
#define BUTTON_KNOB_PIN 49
#define BUTTON_RED_PIN 53
#define BUTTON_WHITE_PIN 51
#define DEBOUNCE_DELAY 60

/* LED Strip Stuff */
#define LED_STRIP_PIN 12

/* TFT Stuff */
#define TFT_RST 3
#define TFT_DC 5
#define TFT_CS 4
#define SD_CS 10
#define BUFFPIXEL 40

/* Audio Stuff */
#define VS1053_RESET 50     // VS1053 reset pin (output)
#define VS1053_CS 52        // VS1053 chip select pin (output)
#define VS1053_DCS 48        // VS1053 Data/command select pin (output)
#define VS1053_DREQ 46       // VS1053 Data request, ideally an Interrupt pin
#define AMPLIFIER_SHDN 42			//Amplifier Shutdown pin
#define VS1053_SDCS 44				// SD Card Reader on VS1053 Breakout Board

#define enableAmplifier() digitalWrite(AMPLIFIER_SHDN, LOW)
#define disableAmplifier() digitalWrite(AMPLIFIER_SHDN, HIGH)
#define isAmplifierEnabled() !digitalRead(AMPLIFIER_SHDN)

/* Temperature Measurement Stuff */
#define TEMP_SENSOR_PIN A7


enum state_type {START, SETUP_STEEPING_TIME, SETUP_END_TEMP, LOWER_TEMP_SENSOR, WAITING_FOR_START, TEABAG_IN, WAITING_FOR_END_TEMP, DONE};