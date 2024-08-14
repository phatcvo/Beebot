#include <Keypad.h>
#include <stdio.h>
#include <string.h>
#include <SoftwareSerial.h>

#define led 13
#define buzz      9
#define lock_relayPin  A2
#define cover_buttonPin A3

// button
#define Pin_button_sys A4 
#define Pin_button_go A5

// software serial
#define rxPin 11
#define txPin 12
SoftwareSerial portOne(rxPin, txPin);

using namespace std;
void keypad_rtn(void);

const byte ROWS = 4; // four rows
const byte COLS = 3; // three columns

int   key_cnt=0;
String keypad_input = "";
String in_msg;
String password_default = "9999";
String robot_status;
int door_state = 0; //close
unsigned long previousMillis = 0;
unsigned long previousMillis_2 = 0;

char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

byte rowPins[ROWS] = {2, 3, 4, 5}; //{5, 4, 3, 2}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {8, 7, 6};    // connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

int aTOP = 100;
int led_state = 0;

int count_led_time_slow = 0;
void slow_blink_LED()
{
  if (led_state == 0) 
  {
    if(count_led_time_slow == 40) // 2s/0.05
    {
      led_state = aTOP;
      count_led_time_slow = 0;
    }
    else
    {
      ++count_led_time_slow;
    }
  } 
  else 
  {
    if(count_led_time_slow == 2)  //0.1s/0.05
    {
      led_state = 0;
      count_led_time_slow = 0;
    }
    else
    {
      ++count_led_time_slow;
    }
  }
  analogWrite(led, led_state);
}

void blink_LED()
{
  if (led_state == 0) 
  {
    led_state = aTOP;
  } 
  else 
  {
    led_state = 0;
  }
  analogWrite(led, led_state);
}

void on_LED()
{
  analogWrite(led, aTOP);
}

void off_LED()
{
  analogWrite(led, 0);
}

void keypress_buzz()
{
  digitalWrite(buzz, HIGH);
  delay(50);
  digitalWrite(buzz, LOW);
}

void set_null()
{
  key_cnt = 0;
  keypad_input = "";
  previousMillis = 0;
  previousMillis_2 = 0;
  // Serial.print("keypad_input: ");
  // Serial.println(keypad_input);
}

void send_UnoLCD_keypad(String pwd)
{
  String pwd_;
  if(pwd == "")
    pwd_ = "12345";
  else
    pwd_ = pwd;

  String message_send = "";
  message_send += "<";
  message_send += "EE";
  message_send += ",";
  message_send += pwd_;
  message_send += ">";

  // Serial.print("send UNO Keypad: ");
  // Serial.println(message_send);
  portOne.println(message_send);
  // portOne.write(message_send.c_str());
}

void send_UnoLCD_state()
{
  String message_send = "";
  message_send += "<";
  message_send += "EF";
  message_send += ",";
  message_send += robot_status;
  message_send += ">";

  // Serial.print("send UNO State: ");
  // Serial.println(message_send);
  portOne.println(message_send);
  // portOne.write(message_send.c_str());
}

void setup (void)
{
  // lock
  pinMode(led, OUTPUT);   
  pinMode(buzz, OUTPUT);
  pinMode(lock_relayPin, OUTPUT);
  pinMode(cover_buttonPin, INPUT_PULLUP);

  // button
  pinMode(Pin_button_sys, INPUT_PULLUP);
  pinMode(Pin_button_go, INPUT_PULLUP);
  
  // software serial
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  Serial.begin(115200);
  portOne.begin(9600);

  digitalWrite(lock_relayPin, LOW);
}

const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
boolean pc_conection_flag = false;
boolean coming_data = false;
unsigned long previousMillis_coming_data = 0;
unsigned long previousMillis_state = 0;

void loop() {
  // button----------------
  // blink_LED();
  // slow_blink_LED();
  // on_LED();
  // off_LED();

  if (!pc_conection_flag)
  {
    robot_status = "5";
    slow_blink_LED();
  }
  else
    on_LED();

  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  if(Serial.available() > 0)
  {
    pc_conection_flag = true;
    coming_data = true;
    previousMillis_coming_data = millis();

    while (Serial.available() > 0 && newData == false) 
    {
      rc = Serial.read();
      if (recvInProgress == true) 
      {
        if (rc != endMarker) 
        {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars) 
          {
            ndx = numChars - 1;
          }
        }
        else 
        {
          receivedChars[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          ndx = 0;
          newData = true;
        }
      }
      else if (rc == startMarker) 
      {
        recvInProgress = true;
      }
    }
  }
  else
  {
    coming_data = false;
  }

  if(previousMillis_coming_data != 0 && !coming_data && millis() - previousMillis_coming_data > 3000)
  {
    // Serial.println(">3s, no coming data");
    pc_conection_flag = false;
    previousMillis_coming_data = 0;
  }

  if (newData == true) 
  {
    char* strtokIndx;
    strtokIndx = strtok(receivedChars, ",");
    
    in_msg = strtokIndx;

    strtokIndx = strtok(NULL, ",");
    robot_status = strtokIndx;

    newData = false;
  }

  int system_button = digitalRead(Pin_button_sys)==0?0:1;
  int go_button = digitalRead(Pin_button_go)==0?0:1;
  int door_pin = digitalRead(cover_buttonPin)==0?0:1;
  byte message[4] = {0xEE, system_button, go_button, door_pin};
  Serial.write(message, 4);

  keypad_rtn();

  if(key_cnt == 4)
  {
    if(keypad_input == in_msg||keypad_input == password_default)
    {
      // Serial.println("========> correct password");
      send_UnoLCD_keypad("-OK-");
      digitalWrite(lock_relayPin, HIGH); 
      delay(300);
      digitalWrite(lock_relayPin, LOW);
    }
    else
    {
      send_UnoLCD_keypad("-NG-");
    }

    delay(1500);

    set_null();

    send_UnoLCD_keypad(keypad_input);
  }

  unsigned long currentMillis_state = millis();
  if(currentMillis_state - previousMillis_state > 500)
  {
    previousMillis_state = currentMillis_state;
    send_UnoLCD_state();
    if(door_pin == 1)
      send_UnoLCD_keypad("OPEN");
    else
      send_UnoLCD_keypad(keypad_input);
  }
}

void keypad_rtn(void)
{
  int door_state = digitalRead(cover_buttonPin)==0?0:1;
  if(door_state==1)
  {
    // Serial.println("door opened!!!");
    set_null();
    delay(50);
    
    return;
  }

  char key = keypad.getKey();
  
  if(key)
  {
    keypress_buzz();

    if (key_cnt == 0)
    {
      previousMillis = millis();
      previousMillis_2 = millis();
    }
    
    unsigned long currentMillis = millis();
    if(previousMillis !=0 && currentMillis - previousMillis > 5000)
    {
      // Serial.println(">5s, retype");
      set_null();
      send_UnoLCD_keypad(keypad_input);
      delay(50);

      return;
    }
    else
      previousMillis = currentMillis;
        
    if(key!='*')
    {
      key_cnt++;
      keypad_input += key;
    }
    else
    {
      // Serial.println("===> CLEAR");
      set_null();
    }

    // Serial.print("keypad_input: ");
    // Serial.println(keypad_input);
    send_UnoLCD_keypad(keypad_input);
  }

  if(previousMillis_2 !=0 && millis() - previousMillis_2 > 20000)
  {
    // Serial.println(">20s, retype");
    set_null();
    
    send_UnoLCD_keypad("keypad_input");
    delay(50);

    return;
  }

  delay(50);
}