/*
 * IRrecord: record and play back IR signals as a minimal 
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * An IR LED must be connected to the output PWM pin 3.
 * A button must be connected to the input BUTTON_PIN; this is the
 * send button.
 * A visible LED can be connected to STATUS_PIN to provide status.
 *
 * The logic is:
 * If the button is pressed, send the IR code.
 * If an IR code is received, record it.
 *
 * IRremote
 * Version 0.11 September, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 */

#include <IRremote.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"

struct changeValue
{
  unsigned long tv;
};

typedef struct changeValue ChangeValue;

ChangeValue changeValue;

int RECV_PIN = A5;
int MODE = 10;
int SEND1 = 7;
int SEND2 = 8;
int state = 1;
unsigned long changeTV;

IRrecv irrecv(RECV_PIN);
IRsend irsend;

decode_results results;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  pinMode(MODE, INPUT);
  pinMode(SEND1, OUTPUT);
  pinMode(SEND2, OUTPUT);
  digitalWrite(SEND1,LOW);
  digitalWrite(SEND2,LOW);
  EEPROM_readAnything(0,changeValue);
  changeTV = changeValue.tv;
  Serial.print(changeTV,HEX);
}

// Storage for the recorded code
int codeType = -1; // The type of code
unsigned long codeValue; // The code value if not raw
unsigned long oldCodeValue;
unsigned int rawCodes[RAWBUF]; // The durations if raw
int codeLen; // The length of the code
int toggle = 0; // The RC5/6 toggle state

// Stores the code for later playback
// Most of this code is just logging
void storeCode(decode_results *results) {
  oldCodeValue = codeValue;
  codeType = results->decode_type;
  int count = results->rawlen;
  if (codeType == UNKNOWN) {
    Serial.println("Received unknown code, saving as raw");
    codeLen = results->rawlen - 1;
    // To store raw codes:
    // Drop first value (gap)
    // Convert from ticks to microseconds
    // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
    for (int i = 1; i <= codeLen; i++) {
      if (i % 2) {
        // Mark
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK - MARK_EXCESS;
        Serial.print(" m");
      } 
      else {
        // Space
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK + MARK_EXCESS;
        Serial.print(" s");
      }
      Serial.print(rawCodes[i - 1], DEC);
    }
    Serial.println("");
  }
  else {
    if (codeType == NEC) {
      Serial.print("Received NEC: ");
      if (results->value == REPEAT) {
        // Don't record a NEC repeat value as that's useless.
        codeValue = oldCodeValue;
        Serial.println("repeat; ignoring.");
        return;
      }
    } 
    else if (codeType == SONY) {
      Serial.print("Received SONY: ");
    } 
    else if (codeType == RC5) {
      Serial.print("Received RC5: ");
    } 
    else if (codeType == RC6) {
      Serial.print("Received RC6: ");
    }
   else if (codeType == SAMSUNG){
      Serial.print("Recieved SAMSUNG: ");
   }
    else {
      Serial.print("Unexpected codeType ");
      Serial.print(codeType, DEC);
      Serial.println("");
    }
    Serial.println(results->value, HEX);
    codeValue = results->value;
    codeLen = results->bits;
  }
}

void sendCode(int repeat) {
  if (codeType == NEC) {
    if (repeat) {
      irsend.sendNEC(REPEAT, codeLen);
      Serial.println("Sent NEC repeat");
    } 
    else {
      irsend.sendNEC(codeValue, codeLen);
      Serial.print("Sent NEC ");
      Serial.println(codeValue, HEX);
    }
  } 
  else if (codeType == SONY) {
    irsend.sendSony(codeValue, codeLen);
    Serial.print("Sent Sony ");
    Serial.println(codeValue, HEX);
  } 
  else if (codeType == RC5 || codeType == RC6) {
    if (!repeat) {
      // Flip the toggle bit for a new button press
      toggle = 1 - toggle;
    }
    // Put the toggle bit into the code to send
    codeValue = codeValue & ~(1 << (codeLen - 1));
    codeValue = codeValue | (toggle << (codeLen - 1));
    if (codeType == RC5) {
      Serial.print("Sent RC5 ");
      Serial.println(codeValue, HEX);
      irsend.sendRC5(codeValue, codeLen);
    } 
    else {
      irsend.sendRC6(codeValue, codeLen);
      Serial.print("Sent RC6 ");
      Serial.println(codeValue, HEX);
    }
  } 
  else if (codeType == UNKNOWN /* i.e. raw */) {
    // Assume 38 KHz
    irsend.sendSamsung(oldCodeValue,codeLen);
    Serial.println("Sent raw");
  }
  else if (codeType == SAMSUNG){
    irsend.sendSamsung(codeValue,codeLen);
    Serial.print("Send SAMSUNG ");
    Serial.println(codeValue,HEX);
  }
}
void loop() {
while(digitalRead(MODE)==HIGH){
    digitalWrite(SEND1,LOW);
    digitalWrite(SEND2,HIGH);
    delay(100);
    digitalWrite(SEND1,HIGH);
    digitalWrite(SEND2,LOW);
    delay(100);
    if(irrecv.decode(&results)) {
      Serial.print("recvd");
      storeCode(&results);
      changeTV = codeValue;
      changeValue.tv = changeTV;
      EEPROM_writeAnything(0,changeValue);
      Serial.print(changeTV,HEX);
      irrecv.enableIRIn();
    }
}
if (irrecv.decode(&results)) {
    storeCode(&results);
    if(codeValue == changeTV){
    Serial.print("Match\n");
    irrecv.enableIRIn();
    cycleTV(state);
    state++;
    state = state % 4;
    }
    else{ 
    sendCode(0);
    irrecv.enableIRIn(); // resume receiver
    }
  }
}
void cycleTV(int state){
  Serial.print("state:");
  Serial.print(state);
  Serial.print("\n");
  if(state == 0){
    Serial.print("case 1\n");
    digitalWrite(SEND1,LOW);
    digitalWrite(SEND2,HIGH);
  }
  if(state == 1){
    digitalWrite(SEND1,HIGH);
    digitalWrite(SEND2,HIGH);
  }
  if(state == 2){
    digitalWrite(SEND2,LOW);
  }
  if(state == 3){
    digitalWrite(SEND1,LOW);
  }
}


