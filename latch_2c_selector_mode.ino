// F1SSF 06/10/2025
// Pilotage de 2 relais Latch TLL RP12-LA0TL0 de chez RAPIDTEK par un ATtiny1614 
// PTT_A et PTT_B et PTT_A/B + inversion logique sauvegardée
// Alimentation des relais en 12V, ATtiny 5V par un regulateur 78L05
// Entrée PTT_A/PTT_B (0/5V) avec pull-down 100K externes.
// Les sorties ATtiny SET/RESET pilote directement les relais qui ont leur propre drivers TTL, via des 120
// Un front haut sur PTT => SET ; un front bas => RESET.
// Pins:
//   Canal A: PTT_A=PA4, SET_A=PA3, RESET_A=PA2
//   Canal B: PTT_B=PA7, SET_B=PB0,  RESET_B=PB1
// Setup du core dans l'IDE: megaTinyCore (ATtiny1614 @ 16/20 MHz @4,2V 20Mhz or less)
// On laisse les autres options par defaut
// Prog en udpi via serial UDPI 230400Bd
// On peut ajuster dans le code la durée de l'impultion , et la durée du debounce, la durée des flashs
// dual control + PTT_C >> partagé OFF/A/B/AB + BP 10" >> selection de normal ou inversé, mem en eeprom
// Pins confirmées:
//  PTT_A=PA4,  SET_A=PA3, RESET_A=PA2
//  PTT_B=PA7,  SET_B=PB0, RESET_B=PB1
//  PTT_SHARED=PB3, SELECT=PB2, LED_A=PA6, LED_B=PA5
// ATtiny1614 — RP12 dual + PTT partagé OFF/A/B/AB + inversion logique par fronts + EEPROM

#include <Arduino.h>
#include <EEPROM.h>

// ===================== Brochage =====================
const uint8_t PIN_PTT_A   = PIN_PA4;
const uint8_t PIN_PTT_B   = PIN_PA7;
const uint8_t PIN_SET_A   = PIN_PA3;
const uint8_t PIN_RESET_A = PIN_PA2;
const uint8_t PIN_SET_B   = PIN_PB0;
const uint8_t PIN_RESET_B = PIN_PB1;
const uint8_t PIN_PTT_SHARED = PIN_PB3;
const uint8_t PIN_PTT_SELECT = PIN_PB2;
const uint8_t PIN_LED_A   = PIN_PA6;
const uint8_t PIN_LED_B   = PIN_PA5;

// ===================== Paramètres =====================
const uint16_t PULSE_MS        = 200;  //durée de l'impulsion ms
const uint16_t DEBOUNCE_MS     = 25;   //durée de l'anti rebond ms
const uint16_t SEL_DEBOUNCE_MS = 200;  //durée entre deux lectures entrées ms pour eviter double bascule
const uint16_t LONGPRESS_MS    = 5000;  //durée de press BP pour select config
const uint16_t FLASH_ON_MS     = 80;   // durée de flash ON des leds
const uint16_t FLASH_OFF_MS    = 80;   //durée de flash OFF des leds

// ===================== EEPROM =====================
struct Cfg { uint8_t magic; uint8_t invert; };
const int EEPROM_ADDR = 0;
Cfg cfg;
bool triggerInverted = false; // logique active actuelle

// ===================== Structures =====================
struct Chan {
  uint8_t pinPTT, pinSET, pinRST;
  bool lastRaw;           // dernier état brut lu
  unsigned long tChanged;
  unsigned long tEndSET, tEndRST;
};
Chan A{PIN_PTT_A, PIN_SET_A, PIN_RESET_A, 0,0,0,0};
Chan B{PIN_PTT_B, PIN_SET_B, PIN_RESET_B, 0,0,0,0};

// Sélecteur par BP
enum SelMode { SEL_OFF=0, SEL_A=1, SEL_B=2, SEL_AB=3 };
volatile SelMode selMode = SEL_OFF;
bool lastSel=0; unsigned long tSel=0; unsigned long tPressStart=0;
bool lastRawShared=0; unsigned long tPTTsh=0;

// ===================== Helpers =====================
inline void startPulse(uint8_t pin, unsigned long &tEnd){ digitalWrite(pin, HIGH); tEnd = millis() + PULSE_MS; }
inline void endPulse  (uint8_t pin, unsigned long &tEnd){ if(tEnd && (long)(millis()-tEnd) >= 0){ digitalWrite(pin, LOW); tEnd = 0; } }

void flashLeds(uint8_t n){
  for(uint8_t i=0;i<n;i++){
    digitalWrite(PIN_LED_A,HIGH); digitalWrite(PIN_LED_B,HIGH);
    delay(FLASH_ON_MS);
    digitalWrite(PIN_LED_A,LOW);  digitalWrite(PIN_LED_B,LOW);
    delay(FLASH_OFF_MS);
  }
}

void updateSelLeds(){
  digitalWrite(PIN_LED_A, (selMode==SEL_A || selMode==SEL_AB) ? HIGH : LOW);
  digitalWrite(PIN_LED_B, (selMode==SEL_B || selMode==SEL_AB) ? HIGH : LOW);
}

// ===================== Init =====================
void initChan(Chan &c){
  pinMode(c.pinPTT, INPUT);
  pinMode(c.pinSET, OUTPUT);  digitalWrite(c.pinSET, LOW);
  pinMode(c.pinRST, OUTPUT);  digitalWrite(c.pinRST, LOW);
  c.lastRaw = digitalRead(c.pinPTT);
  c.tChanged = millis();
}

// ===================== Services =====================
void pulseChan(Chan &C, bool level){
  if(C.tEndSET==0 && C.tEndRST==0){
    if(level) startPulse(C.pinSET, C.tEndSET);
    else      startPulse(C.pinRST, C.tEndRST);
  }
}

void serviceChan(Chan &c){
  static bool lastStableA=0, lastStableB=0; // une instance par appel -> on duplique par canal
  bool &lastStable = (&c==&A)? lastStableA : lastStableB;

  bool raw = digitalRead(c.pinPTT);
  if(raw != c.lastRaw){                 // début d'un changement
    c.lastRaw = raw;
    c.tChanged = millis();              // démarre fenêtre debounce
  }
  // état stable atteint ?
  if((millis()-c.tChanged) >= DEBOUNCE_MS && raw != lastStable){
    bool rising  = (!lastStable && raw);
    bool falling = ( lastStable && !raw);
    lastStable = raw;

    if(!triggerInverted){
      if(rising  && c.tEndSET==0 && c.tEndRST==0) startPulse(c.pinSET, c.tEndSET);
      if(falling && c.tEndSET==0 && c.tEndRST==0) startPulse(c.pinRST, c.tEndRST);
    } else {
      if(falling && c.tEndSET==0 && c.tEndRST==0) startPulse(c.pinSET, c.tEndSET);
      if(rising  && c.tEndSET==0 && c.tEndRST==0) startPulse(c.pinRST, c.tEndRST);
    }
  }

  endPulse(c.pinSET, c.tEndSET);
  endPulse(c.pinRST, c.tEndRST);
}

// ---- Sélecteur OFF/A/B/AB + appui long inversion ----
void serviceSelector3(){
  bool r = digitalRead(PIN_PTT_SELECT);
  unsigned long now = millis();

  // Appui long -> inversion logique
  if(r && tPressStart==0) tPressStart = now;
  if(!r && tPressStart>0){
    if(now - tPressStart >= LONGPRESS_MS){
      triggerInverted = !triggerInverted;
      cfg.magic = 0xA5;
      cfg.invert = triggerInverted ? 1 : 0;
      EEPROM.put(EEPROM_ADDR, cfg);
      flashLeds(3); // confirmation
    }
    tPressStart = 0;
  }

  // Appui court -> cycle mode
  if(r && !lastSel && (now - tSel) > SEL_DEBOUNCE_MS){
    selMode = (SelMode)((selMode+1)%4); // OFF -> A -> B -> A+B -> OFF
    updateSelLeds();
    tSel = now;
  }
  lastSel = r;
}

void serviceSharedPTT3(){
  static bool lastStableSh=0;
  bool raw = digitalRead(PIN_PTT_SHARED);
  if(raw != lastRawShared){ lastRawShared = raw; tPTTsh = millis(); }
  if(selMode==SEL_OFF) return;

  if((millis()-tPTTsh) >= DEBOUNCE_MS && raw != lastStableSh){
    bool rising  = (!lastStableSh && raw);
    bool falling = ( lastStableSh && !raw);
    lastStableSh = raw;

    if(!triggerInverted){
      if(rising){  if(selMode==SEL_A) pulseChan(A,1); else if(selMode==SEL_B) pulseChan(B,1); else {pulseChan(A,1); pulseChan(B,1);} }
      if(falling){ if(selMode==SEL_A) pulseChan(A,0); else if(selMode==SEL_B) pulseChan(B,0); else {pulseChan(A,0); pulseChan(B,0);} }
    } else {
      if(falling){ if(selMode==SEL_A) pulseChan(A,1); else if(selMode==SEL_B) pulseChan(B,1); else {pulseChan(A,1); pulseChan(B,1);} }
      if(rising){  if(selMode==SEL_A) pulseChan(A,0); else if(selMode==SEL_B) pulseChan(B,0); else {pulseChan(A,0); pulseChan(B,0);} }
    }
  }
}

// ===================== Setup =====================
void setup(){
  EEPROM.get(EEPROM_ADDR, cfg);
  if(cfg.magic != 0xA5){ cfg.magic = 0xA5; cfg.invert = 0; EEPROM.put(EEPROM_ADDR, cfg); }
  triggerInverted = (cfg.invert != 0);

  pinMode(PIN_LED_A, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  digitalWrite(PIN_LED_A, LOW);
  digitalWrite(PIN_LED_B, LOW);

  initChan(A); initChan(B);
  pinMode(PIN_PTT_SHARED, INPUT);
  pinMode(PIN_PTT_SELECT, INPUT);

  updateSelLeds();
  flashLeds(triggerInverted ? 2 : 1);
}

// ===================== Boucle =====================
void loop(){
  serviceChan(A);
  serviceChan(B);
  serviceSelector3();
  serviceSharedPTT3();
}
