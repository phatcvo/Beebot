#include <LCDWIKI_GUI.h>
#include <LCDWIKI_KBV.h>

// Colors
#define BLACK   0x0000
#define CYAN    0x07FF
#define DARK_GRAY 0x7BEF

LCDWIKI_KBV my_lcd(ILI9486, A3, A2, A1, A0, A4);

int screenWidth, screenHeight;
int eyeWidth = 100;
int eyeHeight = 100;
int pupilRadius = 20;

String currentMood = "normal"; 
String targetDirection = "center"; 
int targetOffset = 0; 

// Random micro move
unsigned long lastRandomMoveTime = 0;
int randomMoveOffset = 0;

// Breathing
unsigned long lastBreathTime = 0;
float breathPhase = 0.0;
float breathSpeed = 0.001; // How fast breathing is

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(A0));

  my_lcd.Init_LCD();
  my_lcd.Set_Rotation(1);
  my_lcd.Fill_Screen(BLACK);

  screenWidth = my_lcd.Get_Display_Width();
  screenHeight = my_lcd.Get_Display_Height();

  drawEyes(currentMood, targetOffset, 0);
  Serial.println("Send: normal, happy, sad, angry, surprised, left, right, center");
}

void loop() {
  // Handle serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      if (input == "left") {
        targetDirection = "left";
        targetOffset = -20;
      } 
      else if (input == "right") {
        targetDirection = "right";
        targetOffset = 20;
      } 
      else if (input == "center") {
        targetDirection = "center";
        targetOffset = 0;
      } 
      else {
        currentMood = input; 
      }
    }

    drawEyes(currentMood, targetOffset, breathingOffset());
  }

  // Idle: random micro move + breathing
  unsigned long now = millis();
  randomMoveOffset = random(-10, 10);
  if (now - lastRandomMoveTime > 5000) {
    randomMoveOffset = random(-3, 4);
    drawEyes(currentMood, targetOffset + randomMoveOffset, breathingOffset());
    lastRandomMoveTime = now;
  }
}

// breathing effect: returns tiny up-down offset
int breathingOffset() {
  breathPhase += breathSpeed;
  if (breathPhase > TWO_PI) breathPhase -= TWO_PI;
  
  float breath = sin(breathPhase); // breathing curve -1.0 to +1.0
  int offset = (int)(breath * 5);  // scale breathing movement (±5px)
  return offset;
}

void drawEyes(String mood, int moveX, int moveY) {
  my_lcd.Fill_Screen(BLACK);
  int offset = 20;
  int centerX_L = screenWidth / 3 - offset;
  int centerX_R = 2 * screenWidth / 3 + offset;
  int centerY = screenHeight / 2;

  drawOneEye(centerX_L, centerY, mood, moveX, moveY);
  drawOneEye(centerX_R, centerY, mood, moveX, moveY);
}

void drawOneEye(int cx, int cy, String mood, int moveX, int moveY) {
  my_lcd.Set_Draw_color(CYAN);

  int w = eyeWidth;
  int h = eyeHeight;
  int cut = 10;

  if (mood == "normal") {
    cut = 10;
    my_lcd.Fill_Round_Rectangle(cx - w/2, cy - h/2, cx + w/2, cy + h/2, cut);
    drawPupil(cx + moveX, cy + moveY);
    return;
  }
  else if (mood == "happy") {
    h = eyeHeight * 0.6;
    cut = 15;my_lcd.Fill_Round_Rectangle(cx - w/2, cy - h/2, cx + w/2, cy + h/2, cut);
    drawPupil(cx + moveX, cy + moveY);
    return;
  }
  else if (mood == "sad") {
    w = eyeWidth * 0.7;
    cut = 20;
    my_lcd.Fill_Round_Rectangle(cx - w/2, cy - h/2, cx + w/2, cy + h/2, cut);
    drawPupil(cx + moveX, cy + moveY);
    return;
  }
  else if (mood == "angry") {
    my_lcd.Fill_Triangle(cx - w/2, cy - h/2, cx + w/2, cy - h/2, cx, cy);
    my_lcd.Fill_Rectangle(cx - w/2, cy, cx + w/2, cy + h/2);
    drawPupil(cx + moveX, cy + moveY);
    return;
  }
  else if (mood == "surprised") {
    my_lcd.Fill_Circle(cx, cy, w/2);
    drawPupil(cx + moveX, cy + moveY);
    return;
  }
}

void drawPupil(int cx, int cy) {
  my_lcd.Set_Draw_color(BLACK);
  my_lcd.Fill_Circle(cx, cy, pupilRadius);
}
