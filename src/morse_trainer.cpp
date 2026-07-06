#include <U8g2lib.h>
#include <SPI.h>
#include <EEPROM.h>

/*
  Morse Trainer for Arduino Nano with ST7920 LCD.
  Supports vertical key decoding and iambic keyer input,
  with menu-driven configuration for key type and speed.
*/

// ST7920 display on hardware SPI (CS=D10, MOSI=D11, SCK=D13).
U8G2_ST7920_128X64_1_HW_SPI u8g2(U8G2_R0,  10,  U8X8_PIN_NONE);

// Hardware pin mapping.
const uint8_t PIN_BEEPER   = 9;
const uint8_t PIN_BTN_ENC  = 8;
const uint8_t PIN_ENC_A    = 2;
const uint8_t PIN_ENC_B    = 3;
const uint8_t PIN_KEY_DIT  = 4;
const uint8_t PIN_KEY_DAH  = 5;

enum KeyType   { KEY_VERTICAL, KEY_IAMBIC };
enum SpeedMode { SPEED_MANUAL, SPEED_AUTO };

struct Config {
  KeyType   keyType   = KEY_VERTICAL;
  SpeedMode speedMode  = SPEED_MANUAL;
  uint8_t   wpm        = 15;     
} cfg;

const int     EEPROM_ADDR   = 0;
const uint8_t CONFIG_MAGIC  = 0xA5;

struct StoredConfig {
  uint8_t   magic;
  KeyType   keyType;
  SpeedMode speedMode;
  uint8_t   wpm;
};

void saveConfig() {
  // Save current runtime configuration to EEPROM.
  StoredConfig sc;
  sc.magic     = CONFIG_MAGIC;
  sc.keyType   = cfg.keyType;
  sc.speedMode = cfg.speedMode;
  sc.wpm       = cfg.wpm;
  EEPROM.put(EEPROM_ADDR, sc);   
}

void loadConfig() {
  // Load persisted configuration and enforce valid runtime constraints.
  StoredConfig sc;
  EEPROM.get(EEPROM_ADDR, sc);
  if (sc.magic == CONFIG_MAGIC) {
    cfg.keyType   = sc.keyType;
    cfg.speedMode = sc.speedMode;
    cfg.wpm       = sc.wpm;
  }

  cfg.wpm = constrain(cfg.wpm, 5, 40);
  
  if (cfg.keyType == KEY_IAMBIC) {
    cfg.speedMode = SPEED_MANUAL;
  }
  
}

volatile int8_t encDelta = 0;
volatile uint8_t lastEncState = 0;
bool btnPressed = false;
unsigned long lastBtnTime = 0;

void encoderISR() {
  // Quadrature decoder ISR: updates encoder delta incrementally.
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t state = (a << 1) | b;
  
  static const int8_t table[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  uint8_t idx = (lastEncState << 2) | state;
  encDelta += table[idx & 0x0F];
  lastEncState = state;
}

int8_t readEncoderStep() {
  // Returns -1, 0, or +1 according to accumulated encoder movement.
  noInterrupts();
  int8_t d = 0;
  if (encDelta >= 4)      { d = 1;  encDelta = 0; }
  else if (encDelta <= -4) { d = -1; encDelta = 0; }
  interrupts();
  return d;
}

bool readEncoderButton() {
  // Debounced encoder push button edge detector.
  bool raw = (digitalRead(PIN_BTN_ENC) == LOW);
  if (raw && !btnPressed && (millis() - lastBtnTime > 200)) {
    btnPressed = true;
    lastBtnTime = millis();
    return true;
  }
  if (!raw) btnPressed = false;
  return false;
}

struct MorseChar { const char* label; const char* code; };
const MorseChar MORSE_TABLE[] = {
  {"A",".-"},   {"B","-..."}, {"C","-.-."}, {"D","-.."},  {"E","."},
  {"F","..-."}, {"G","--."},  {"H","...."}, {"I",".."},   {"J",".---"},
  {"K","-.-"},  {"L",".-.."}, {"M","--"},   {"N","-."},   {"O","---"},
  {"P",".--."}, {"Q","--.-"}, {"R",".-."},  {"S","..."},  {"T","-"},
  {"U","..-"},  {"V","...-"}, {"W",".--"},  {"X","-..-"}, {"Y","-.--"},
  {"Z","--.."},
  {"1",".----"},{"2","..---"},{"3","...--"},{"4","....-"},{"5","....."},
  {"6","-...."},{"7","--..."},{"8","---.."},{"9","----."},{"0","-----"},
  
  {".",".-.-.-"}, {",","--..--"}, {"?","..--.."}, {"'",".----."},
  {"!","-.-.--"}, {"/","-..-."},  {"(","-.--."},  {")","-.--.-"},
  {"&",".-..."},  {":","---..."}, {";","-.-.-."}, {"=","-...-"},
  
  
  {"+",".-.-."},  {"-","-....-"}, {"_","..--.-"}, {"\"",".-..-."},
  {"$","...-..-"},{"@",".--.-."},
  
  {"<SK>","...-.-"}
};
const uint8_t MORSE_TABLE_SIZE = sizeof(MORSE_TABLE) / sizeof(MorseChar);

const char* decodeMorse(const char* pattern) {
  // Decode a dot/dash pattern to the corresponding label.
  for (uint8_t i = 0; i < MORSE_TABLE_SIZE; i++) {
    if (strcmp(MORSE_TABLE[i].code, pattern) == 0) return MORSE_TABLE[i].label;
  }
  return "?";
}

unsigned long ditLengthMs() {
  // Convert WPM to Morse time unit (dit length).
  return 1200UL / cfg.wpm;
}

#define CHARS_PER_LINE   20
#define TRAINING_LINES   2
#define TEXT_BUF_LEN     (CHARS_PER_LINE * TRAINING_LINES)  

char textBuf[TEXT_BUF_LEN + 1] = "";
uint8_t textBufPos = 0;

bool needRedraw = true;

void clearTextBuffer() {
  // Clear decoded text area.
  textBufPos = 0;
  textBuf[0] = '\0';
  needRedraw = true;
}

void appendChar(char c) {
  // Append one character and wrap by clearing when buffer is full.
  if (textBufPos >= TEXT_BUF_LEN) {
    
    textBufPos = 0;
    textBuf[0] = '\0';
  }
  textBuf[textBufPos++] = c;
  textBuf[textBufPos] = '\0';
  needRedraw = true;
}

void appendString(const char* s) {
  // Append a zero-terminated string to the text buffer.
  for (const char* p = s; *p != '\0'; p++) {
    appendChar(*p);
  }
}

struct KeyState {
  bool keyDown = false;
  unsigned long downStart = 0;
  unsigned long upStart = 0;
  char activeSymbol = '\0';   
  char pattern[16] = "";
  uint8_t patternLen = 0;

  
  float avgDitMs = 80; 
} key;

const float LETTER_GAP_UNITS = 2.5;
const float WORD_GAP_UNITS   = 6.0;

void closeGroupIfNeeded(unsigned long gap, float unit) {
  // Close current symbol group as letter/word based on inter-element gap.
  if (key.patternLen == 0) return;
  if (gap > unit * WORD_GAP_UNITS) {
    appendString(decodeMorse(key.pattern));
    appendChar(' ');
    key.patternLen = 0; key.pattern[0] = '\0';
  } else if (gap > unit * LETTER_GAP_UNITS) {
    appendString(decodeMorse(key.pattern));
    key.patternLen = 0; key.pattern[0] = '\0';
  }
  
}

float currentUnitMs() {
  // Returns active timing unit in milliseconds for current mode/key type.
  if (cfg.keyType == KEY_IAMBIC) return (float)ditLengthMs();
  return (cfg.speedMode == SPEED_AUTO) ? key.avgDitMs : (float)ditLengthMs();
}

#define AUTO_HISTORY_LEN 12
const unsigned long AUTO_MIN_VALID_MS = 35;
const unsigned long AUTO_MAX_VALID_MS = 600;
unsigned long recentDurations[AUTO_HISTORY_LEN] = {0};
uint8_t recentDurIdx = 0;
uint8_t recentDurCount = 0;

float autoDotEstimateMs() {
  // Estimate dot duration from recent valid samples using a low percentile.
  if (recentDurCount < 4) return key.avgDitMs;

  unsigned long sorted[AUTO_HISTORY_LEN];
  uint8_t n = 0;
  for (uint8_t i = 0; i < recentDurCount; i++) {
    unsigned long v = recentDurations[i];
    if (v >= AUTO_MIN_VALID_MS && v <= AUTO_MAX_VALID_MS) {
      sorted[n++] = v;
    }
  }
  if (n < 4) return key.avgDitMs;

  
  for (uint8_t i = 1; i < n; i++) {
    unsigned long x = sorted[i];
    int8_t j = i - 1;
    while (j >= 0 && sorted[j] > x) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = x;
  }

  
  uint8_t idx = (uint8_t)((n - 1) * 3UL / 10UL);
  return (float)sorted[idx];
}

void updateAutoSpeed(unsigned long elementDur) {
  // Update auto WPM estimate (enabled only for vertical key mode).
  if (cfg.speedMode != SPEED_AUTO) return;
  if (cfg.keyType != KEY_VERTICAL) return;  

  if (elementDur < AUTO_MIN_VALID_MS || elementDur > AUTO_MAX_VALID_MS) return;

  recentDurations[recentDurIdx] = elementDur;
  recentDurIdx = (recentDurIdx + 1) % AUTO_HISTORY_LEN;
  if (recentDurCount < AUTO_HISTORY_LEN) recentDurCount++;

  float targetDot = autoDotEstimateMs();

  
  float maxStep = key.avgDitMs * 0.08f;  
  if (targetDot > key.avgDitMs + maxStep) targetDot = key.avgDitMs + maxStep;
  if (targetDot < key.avgDitMs - maxStep) targetDot = key.avgDitMs - maxStep;

  
  key.avgDitMs = key.avgDitMs * 0.90f + targetDot * 0.10f;

  cfg.wpm = constrain((int)(1200.0f / key.avgDitMs + 0.5f), 5, 40);
}

void handleVerticalKey() {
  // Read straight key press/release and classify dot vs dash by press duration.
  static unsigned long lastDebounceTime = 0;
  static bool lastKeyState = true;  
  
  bool raw = (digitalRead(PIN_KEY_DIT) == LOW);
  unsigned long now = millis();
  
  
  if (raw != lastKeyState) {
    lastDebounceTime = now;
    lastKeyState = raw;
    return;
  }
  
  
  if (now - lastDebounceTime < 15) return;
  
  bool down = raw;  

  if (down && !key.keyDown) {
    key.keyDown = true;
    key.downStart = now;
    unsigned long gap = now - key.upStart;
    closeGroupIfNeeded(gap, currentUnitMs());
    tone(PIN_BEEPER, 650);
  }

  if (!down && key.keyDown) {
    key.keyDown = false;
    key.upStart = now;
    noTone(PIN_BEEPER);

    unsigned long elementDur = now - key.downStart;
    if (elementDur < AUTO_MIN_VALID_MS) return;  

    float unit = currentUnitMs();

    
    char symbol = (elementDur >= unit * 2.0f) ? '-' : '.';

    if (key.patternLen < 15) {
      key.pattern[key.patternLen++] = symbol;
      key.pattern[key.patternLen] = '\0';
    }
    updateAutoSpeed(elementDur);
    
    
    
  }
}

enum IambicPhase { IAMBIC_IDLE, IAMBIC_SENDING, IAMBIC_GAP };
IambicPhase iambicPhase = IAMBIC_IDLE;
unsigned long iambicPhaseStart = 0;

void handleIambicKey() {
  // Generate self-timed iambic elements and append them to current pattern.
  bool ditDown = (digitalRead(PIN_KEY_DIT) == LOW);
  bool dahDown = (digitalRead(PIN_KEY_DAH) == LOW);
  unsigned long now = millis();
  float unit = currentUnitMs();

  switch (iambicPhase) {

    case IAMBIC_IDLE:
      if (ditDown || dahDown) {
        char symbol;
        if (ditDown && dahDown) {
          symbol = (key.activeSymbol == '.') ? '-' : '.';
        } else {
          symbol = ditDown ? '.' : '-';
        }
        key.activeSymbol = symbol;

        
        
        
        
        unsigned long gap = now - key.upStart;
        closeGroupIfNeeded(gap, unit);

        if (key.patternLen < 15) {
          key.pattern[key.patternLen++] = symbol;
          key.pattern[key.patternLen] = '\0';
        }

        key.keyDown = true;
        iambicPhaseStart = now;
        iambicPhase = IAMBIC_SENDING;
        tone(PIN_BEEPER, 650);
      }
      break;

    case IAMBIC_SENDING: {
      unsigned long elemLen = (key.activeSymbol == '.') ? (unsigned long)unit : (unsigned long)(unit * 3);
      if (now - iambicPhaseStart >= elemLen) {
        noTone(PIN_BEEPER);
        iambicPhaseStart = now;
        iambicPhase = IAMBIC_GAP;
      }
      break;
    }

    case IAMBIC_GAP:
      if (now - iambicPhaseStart >= (unsigned long)unit) {
        key.keyDown = false;
        key.upStart = now;
        iambicPhase = IAMBIC_IDLE;
      }
      break;
  }
}

enum Screen { SCREEN_TRAINING, SCREEN_MENU_ROOT, SCREEN_MENU_KEYTYPE, SCREEN_MENU_SPEED, SCREEN_MENU_WPM };
Screen screen = SCREEN_TRAINING;
uint8_t menuIndex = 0;

const char* keyTypeName(KeyType k) {
  // Short label used on the status line.
  switch (k) {
    case KEY_VERTICAL: return "Vert.";
    case KEY_IAMBIC:   return "Imb.";
  }
  return "";
}

void drawTrainingScreen() {
  // Draw main training view with status and decoded text.
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0, 10, "Morse Trainer by IU0PXK");
  u8g2.drawStr(0, 22, keyTypeName(cfg.keyType));

  char speedStr[16];
  if (cfg.speedMode == SPEED_AUTO) snprintf(speedStr, sizeof(speedStr), "Auto~%dWPM", cfg.wpm);
  else snprintf(speedStr, sizeof(speedStr), "%dWPM", cfg.wpm);
  u8g2.drawStr(40, 22, speedStr);

  u8g2.drawHLine(0, 26, 128);

  
  
  
  const uint8_t TEXT_LINE1_Y  = 37;  
  const uint8_t TEXT_LINE_H   = 11;  

  u8g2.setFont(u8g2_font_6x10_tf);
  uint8_t len = strlen(textBuf);
  char lineBuf[CHARS_PER_LINE + 1];
  for (uint8_t line = 0; line < TRAINING_LINES; line++) {
    uint8_t start = line * CHARS_PER_LINE;
    if (start >= len) break;
    uint8_t chunk = min((uint8_t)(len - start), (uint8_t)CHARS_PER_LINE);
    memcpy(lineBuf, textBuf + start, chunk);
    lineBuf[chunk] = '\0';
    u8g2.drawStr(0, TEXT_LINE1_Y + line * TEXT_LINE_H, lineBuf);
  }

  u8g2.drawStr(0, 62, "Press encoder: menu");
}

void drawMenuRoot() {
  // Draw root settings menu.
  const char* items[] = {"Key type", "Speed", "Clear screen", "Back"};
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Settings menu");
  for (uint8_t i = 0; i < 4; i++) {
    if (i == menuIndex) u8g2.drawStr(0, 20 + i * 11, ">");
    u8g2.drawStr(10, 20 + i * 11, items[i]);
  }
}

void drawMenuKeyType() {
  // Draw key type selection menu.
  const char* items[] = {"Vertical", "Iambic (2 lever)"};
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Key type");
  for (uint8_t i = 0; i < 2; i++) {
    if (i == menuIndex) u8g2.drawStr(0, 26 + i * 14, ">");
    u8g2.drawStr(10, 26 + i * 14, items[i]);
  }
}

void drawMenuSpeed() {
  // Draw speed mode selection menu.
  const char* items[] = {"Manual", "Automatic"};
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Speed mode");
  for (uint8_t i = 0; i < 2; i++) {
    if (i == menuIndex) u8g2.drawStr(0, 26 + i * 14, ">");
    u8g2.drawStr(10, 26 + i * 14, items[i]);
  }
}

void drawMenuWpm() {
  // Draw manual WPM editor.
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "Speed (WPM)");
  char buf[10];
  snprintf(buf, sizeof(buf), "%d", cfg.wpm);
  u8g2.setFont(u8g2_font_10x20_tf);
  u8g2.drawStr(40, 40, buf);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 60, "Rotate to change");
}

void updateMenu() {
  // Handle encoder-driven UI state transitions and setting changes.
  int8_t step = readEncoderStep();
  bool click = readEncoderButton();

  switch (screen) {
    case SCREEN_TRAINING:
      if (click) { screen = SCREEN_MENU_ROOT; menuIndex = 0; }
      break;

    case SCREEN_MENU_ROOT:
      if (step) menuIndex = constrain(menuIndex + step, 0, 3);
      if (click) {
        if (menuIndex == 0) { screen = SCREEN_MENU_KEYTYPE; menuIndex = cfg.keyType; }
        else if (menuIndex == 1) { screen = SCREEN_MENU_SPEED; menuIndex = cfg.speedMode; }
        else if (menuIndex == 2) { clearTextBuffer(); screen = SCREEN_TRAINING; needRedraw = true; }
        else { screen = SCREEN_TRAINING; needRedraw = true; }
      }
      break;

    case SCREEN_MENU_KEYTYPE:
      if (step) menuIndex = constrain(menuIndex + step, 0, 1);
      if (click) {
        cfg.keyType = (KeyType)menuIndex;
        if (cfg.keyType == KEY_IAMBIC) {
          cfg.speedMode = SPEED_MANUAL;
        }
        saveConfig();
        screen = SCREEN_MENU_ROOT; menuIndex = 0;
      }
      break;

    case SCREEN_MENU_SPEED:
      if (step) menuIndex = constrain(menuIndex + step, 0, 1);
      if (click) {
        if (cfg.keyType == KEY_IAMBIC) {
          cfg.speedMode = SPEED_MANUAL;
          screen = SCREEN_MENU_WPM;
          menuIndex = 0;
          break;
        }

        cfg.speedMode = (SpeedMode)menuIndex;
        if (cfg.speedMode == SPEED_MANUAL) { screen = SCREEN_MENU_WPM; }
        else { saveConfig(); screen = SCREEN_MENU_ROOT; menuIndex = 0; }
      }
      break;

    case SCREEN_MENU_WPM:
      if (step) cfg.wpm = constrain(cfg.wpm + step, 5, 40);
      if (click) { saveConfig(); screen = SCREEN_MENU_ROOT; menuIndex = 0; needRedraw = true; }
      break;
  }
}

void drawScreen() {
  // Render current UI screen through the U8g2 page loop.
  
  
  
  u8g2.firstPage();
  do {
    switch (screen) {
      case SCREEN_TRAINING:     drawTrainingScreen(); break;
      case SCREEN_MENU_ROOT:    drawMenuRoot(); break;
      case SCREEN_MENU_KEYTYPE: drawMenuKeyType(); break;
      case SCREEN_MENU_SPEED:   drawMenuSpeed(); break;
      case SCREEN_MENU_WPM:     drawMenuWpm(); break;
    }
  } while (u8g2.nextPage());
}

void setup() {
  // Initialize GPIO, interrupts, persisted settings, and display.
  pinMode(PIN_BTN_ENC, INPUT_PULLUP);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_KEY_DIT, INPUT_PULLUP);
  pinMode(PIN_KEY_DAH, INPUT_PULLUP);
  pinMode(PIN_BEEPER, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  loadConfig();
  
  
  key.upStart = millis();

  u8g2.begin();
}

void loop() {
  // Main runtime loop: UI update, key processing, decode flush, redraw.
  updateMenu();

  
  if (screen == SCREEN_TRAINING) {
    if (cfg.keyType == KEY_VERTICAL) {
      handleVerticalKey();
    } else {
      handleIambicKey();
    }

    
    
    
    
    if (!key.keyDown && key.patternLen > 0) {
      unsigned long idleGap = millis() - key.upStart;
      float unit = currentUnitMs();
      if (idleGap > unit * WORD_GAP_UNITS) {
        closeGroupIfNeeded(idleGap, unit);
      }
    }

    
    
    
    
    
    if (needRedraw) {
      drawScreen();
      needRedraw = false;
    }
  } else {
    drawScreen();
  }
}
