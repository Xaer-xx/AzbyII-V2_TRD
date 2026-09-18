/* ==========================================================================
   azbyII V4 - SLEDOVANIE HRANY  (ESP32-S3 + QTR-16RC)

   ZAKLADNA LOGIKA
   ---------------
   Polovica bara ide po ciernej, polovica po bielej. Hrana teda sedi presne
   v strede bara. Meria sa to takto:

       darkUnits = sucet(vsetky kanaly) / 1000      -> 0..16
                   "kolko senzorov je na ciernej" (vratane polovic - senzor
                   presne na hrane da ~500, takze to je plynule, nie skokove)

       cierna VLAVO  ->  hrana je na indexe  darkUnits
       cierna VPRAVO ->  hrana je na indexe  16 - darkUnits

       chyba = (index_hrany - 8) * 8mm

   darkUnits == 8  =>  chyba 0  =>  presne pol bara na ciernej. To je cely ciel.

   NEZAVISLE OD STRANY
   -------------------
   Stranu urci porovnanie tmy v lavej a pravej polovici bara. Ked je cierna
   suvisla od jedneho konca (co vzdy je), toto porovnanie nemoze dat zlu
   odpoved. Riadenie samotne je pritom pre obe strany ROVNAKE - vzdy sa toci
   smerom k hrane. Strana rozhoduje len o tom, kam hladat pri strate hrany.

   PRERUSENIA
   ----------
   1) Hardverovy timer 200 Hz -> ISR nastavi priznak -> regulacia bezi presne
      a rovnomerne (bez toho PD derivacia nema zmysel).
   2) GPIO prerusenie na tlacidle -> stlacenie sa nestrati ani pocas citania
      senzora (to blokuje az 2.5 ms).
   Citanie QTR sa v ISR NEROBI - je to blokujuce a dlhe. ISR len odpali tik.

   TLACIDLO (pin 15)
   -----------------
   drzane pri zapnuti  -> RACE MODE (jediny serialovy vypis v celom kode)
   v stave READY       -> START
   kedykolvek inokedy  -> NUDZOVE VYPNUTIE (kalibracia, probe, jazda)

   SIGNALIZACIA LED (seriak je inak vypnuty)
   -----------------------------------------
   svieti trvalo         kalibracia
   rychle bliknutia 6x   zly vysledok kalibracie, niektory kanal nevidel
                         obe farby -> zopakuj ju
   blika rychlo (120ms)  READY, bar je spravne na hrane, mozes stlacit
   blika pomaly (500ms)  READY, ale bar NIE je na hrane
   svieti pocas jazdy    vidi hranu
   zhasnuta pocas jazdy  hranu stratil, hlada ju
   blika (300ms)         STOP

   POLARITA
   --------
   Robot si sam zmeria, ci "doprava" v kode znamena doprava aj v realite
   (probe pri starte). Riesi to aj prehodene motory aj zrkadleny bar.
   ========================================================================== */

#include <Arduino.h>
#include <QTRSensors.h>

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Treba ESP32 Arduino core 3.x (ledcAttach / timerAlarm API)."
#endif

// ------------------------------------------------------------------- PINY
#define LEFT_FWD   4
#define LEFT_BWD   5
#define RIGHT_FWD  6
#define RIGHT_BWD  7
#define BUTTON_PIN 15
#ifndef LED_BUILTIN
#define LED_BUILTIN 48
#endif

#define PWM_FREQ       2000
#define PWM_RESOLUTION 8
#define PWM_MAX        255

static const uint8_t SensorCount = 16;
static const uint8_t SensorPins[SensorCount] = {
  16, 17, 18, 8, 3, 9, 10, 11,
  12, 13, 14, 1, 2, 35, 36, 37
};
#define EMITTER_EVEN 21
#define EMITTER_ODD  20

#define PITCH_MM 8.0f        // QTR-MD-16 = 8 mm, QTR-HD-16 = 4 mm

// ------------------------------------------------- LADENIE - NORMAL MODE
int   baseSpeed = 150;       // PWM na rovine
int   minSpeed  = 95;        // PWM v najostrejsej korekcii
float kp        = 3.0f;      // PWM na mm chyby
float kd        = 100.0f;    // PWM na (mm/ms) zmeny chyby
float posLpf    = 0.50f;     // vyhladenie polohy   (1 = ziadne)
float dLpf      = 0.35f;     // vyhladenie derivacie
float slowStart = 10.0f;     // od tejto chyby zacni spomalovat
float slowFull  = 40.0f;     // pri tejto chybe uz ides minSpeed
float corrMax   = 200.0f;    // strop korekcie

// --------------------------------------------------- LADENIE - RACE MODE
// Zadanie: "nemusi ist presne po ciare, nech je najrychlejsi". Presnost je
// teda obetovana zamerne:
//  - spomalovat zacina az od 14 mm chyby (predtym 6) - do tej doby drzi
//    plnych 255, aj ked sa hrana hupe po bare. Pruh je siroky 300 mm,
//    takze huhanie o +-15 mm nicomu nevadi.
//  - posLpf 0.80 = takmer nefiltrovana poloha. Filter znizuje sum, ale
//    pridava oneskorenie, a pri plnej rychlosti je oneskorenie horsie.
//  - minSpeed 165 = ani v najostrejsej korekcii nespadne pod tuto hranicu.
//    TOTO je prvy parameter, ktory znizuj, ak robot vylietava z oblúka.
#define RACE_BASE      255
#define RACE_MIN       165
#define RACE_KP        3.8f
#define RACE_KD        150.0f
#define RACE_POS_LPF   0.80f
#define RACE_D_LPF     0.50f
#define RACE_SLOW_FROM 14.0f
#define RACE_SLOW_FULL 45.0f
#define RACE_CORR_MAX  255.0f

// Ked hrana zmizne, dosadi sa "virtualna chyba" s tymto znamienkom a
// velkostou. Ide to cez UPLNE ROVNAKU regulaciu, takze znamienko nemoze
// byt nekonzistentne s nameranou polaritou.
#define LOST_ERROR_MM   45.0f
#define LOST_TIMEOUT_MS 1500

#define MOTOR_FLOOR 30       // pod tymto PWM motor len pisti
#define MAX_REVERSE 150      // meter, kolko smie cuvat vnutorne koleso

// ------------------------------------------------------------------ PRAHY
#define SIDE_MIN_DIFF 1500   // rozdiel tmy L/R polovice pre urcenie strany
#define EDGE_MIN      0.8f   // treba aspon tolko ciernej AJ bielej

// 400 Hz. Skrateny timeout senzora po kalibracii to utiahne. Ked by nahodou
// citanie predsa len pretiahlo tik, nic sa nedeje - derivacia sa pocita z
// realneho dt, takze ladenie sa tym neposunie.
#define CONTROL_US    2500

#define CAL_STEPS 180        // kalibracny sweep
#define CAL_FLIP  18
#define CAL_SPEED 115
#define CAL_MIN_RANGE 300    // mensi rozsah kanala = zla kalibracia

// Probe polarity: 0 = zmeraj automaticky, +1 / -1 = natvrdo
#define FORCE_STEER_SIGN 0
#define PROBE_PWM 115
#define PROBE_MS  140

// ------------------------------------------------------------------- STAVY
enum { ST_CALIB, ST_READY, ST_PROBE, ST_RUN, ST_STOP };

QTRSensors qtr;
uint16_t   sensorValues[SensorCount];

volatile bool controlTick = false;
volatile bool buttonFlag  = false;
hw_timer_t   *ctrlTimer   = nullptr;

int    stav      = ST_CALIB;
int8_t steerSign = 1;        // +1 = "doprava" v kode je doprava aj realne
int8_t blackSide = 0;        // -1 cierna vlavo, +1 cierna vpravo
float  posFilt   = 0.0f;
float  lastError = 0.0f;
float  dFilt     = 0.0f;
bool   acquired  = false;    // uz sme niekedy videli hranu?
unsigned long lostSince  = 0;
unsigned long lastCtrlUs = 0;   // cas posledneho tiku, pre derivaciu

struct Edge {
  bool  valid;
  float posMm;       // + = hrana je VPRAVO od stredu bara
  float darkUnits;   // 0..16
};

// --------------------------------------------------------------------- ISR
void IRAM_ATTR onControlTimer() { controlTick = true; }
void IRAM_ATTR onButton()       { buttonFlag  = true; }

// --------------------------------------------------------------- TLACIDLO
// Priznak nastavi prerusenie, potvrdenie a odkmitanie sa robi tu, mimo ISR.
// Clovek drzi tlacidlo 50 ms+, najdlhsia slucka je ~25 ms (kalibracia),
// takze stlacenie sa nestrati.
bool stlacene() {
  if (!buttonFlag) return false;
  buttonFlag = false;
  static unsigned long last = 0;
  if (millis() - last < 300)          return false;
  if (digitalRead(BUTTON_PIN) != LOW) return false;   // zakmit / rusenie
  last = millis();
  return true;
}

// ------------------------------------------------------------------ MOTORY
void setMotors(int left, int right) {
  left  = constrain(left,  -PWM_MAX, PWM_MAX);
  right = constrain(right, -PWM_MAX, PWM_MAX);

  if (left  != 0 && abs(left)  < MOTOR_FLOOR) left  = (left  > 0) ?  MOTOR_FLOOR : -MOTOR_FLOOR;
  if (right != 0 && abs(right) < MOTOR_FLOOR) right = (right > 0) ?  MOTOR_FLOOR : -MOTOR_FLOOR;

  ledcWrite(LEFT_FWD,  left  > 0 ?  left  : 0);
  ledcWrite(LEFT_BWD,  left  < 0 ? -left  : 0);
  ledcWrite(RIGHT_FWD, right > 0 ?  right : 0);
  ledcWrite(RIGHT_BWD, right < 0 ? -right : 0);
}

void zastav() { setMotors(0, 0); }

// Kladna hodnota = otacaj sa DOPRAVA (nominalne, pred zmeranim polarity).
void tocSa(int pwm) { setMotors(pwm, -pwm); }

// Rozdiel medzi kolesami JE zatacanie. Pri baseSpeed 255 by sa vonkajsie
// koleso orezalo na 255 a rozdiel by klesol na polovicu - robot by presne
// pri plnej rychlosti prisiel o polovicu riadenia. Preto oba posunieme
// dole o presah namiesto orezania.
void setMotorsDiff(int left, int right) {
  int over = max(left, right) - PWM_MAX;
  if (over > 0) { left -= over; right -= over; }
  // Poistka: pri extremnej korekcii by posun dole vyrobil plne protibezne
  // tocenie na 255/-255. To uz nie je riadenie ale hodenie robota bokom.
  left  = max(left,  -MAX_REVERSE);
  right = max(right, -MAX_REVERSE);
  setMotors(left, right);
}

// ----------------------------------------------------------------- CITANIE
Edge readEdge() {
  qtr.readCalibrated(sensorValues);       // 0 = biela, 1000 = cierna

  uint32_t sum = 0, halfL = 0, halfR = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    sum += sensorValues[i];
    if (i < SensorCount / 2) halfL += sensorValues[i];
    else                     halfR += sensorValues[i];
  }

  Edge e;
  e.darkUnits = sum / 1000.0f;

  // Strana: cierna je suvisla od jedneho konca, takze tazisko tmy v lavej
  // vs pravej polovici ju urci jednoznacne. Pri malom rozdiele (cely bar
  // na bielej / cely na ciernej) si drzime poslednu znamu hodnotu.
  int32_t d = (int32_t)halfL - (int32_t)halfR;
  if      (d >  SIDE_MIN_DIFF) blackSide = -1;
  else if (d < -SIDE_MIN_DIFF) blackSide = +1;

  e.valid = (blackSide != 0) &&
            (e.darkUnits > EDGE_MIN) &&
            (e.darkUnits < SensorCount - EDGE_MIN);

  float idx = (blackSide < 0) ? e.darkUnits : (SensorCount - e.darkUnits);
  e.posMm = (idx - SensorCount * 0.5f) * PITCH_MM;
  return e;
}

// Priemer z n citani - na probe, kde potrebujeme cislo bez sumu.
float readPosAvg(uint8_t n) {
  float s = 0;
  for (uint8_t i = 0; i < n; i++) { s += readEdge().posMm; delay(3); }
  return s / n;
}

// --------------------------------------------------------------- REGULACIA
int speedFor(float absErr) {
  if (absErr <= slowStart) return baseSpeed;
  if (absErr >= slowFull)  return minSpeed;
  float t = (absErr - slowStart) / (slowFull - slowStart);
  return (int)(baseSpeed + (minSpeed - baseSpeed) * t);
}

void controlStep() {
  // Skutocny cas medzi tikmi. Derivacia sa pocita na milisekundu, nie na
  // tik, takze kd nezavisi od frekvencie slucky ani od toho, ci citanie
  // senzora obcas pretiahne svoj tik.
  unsigned long nowUs = micros();
  float dtMs = (lastCtrlUs == 0) ? (CONTROL_US / 1000.0f)
                                 : (nowUs - lastCtrlUs) / 1000.0f;
  lastCtrlUs = nowUs;
  dtMs = constrain(dtMs, 0.5f, 20.0f);

  Edge e = readEdge();
  bool  lost = !e.valid;
  float error;

  if (!lost) {
    if (lostSince != 0 || !acquired) {
      // Prave sme (znovu) chytili hranu - naseeduj filtre aktualnym
      // meranim, inak stara lastError vyrobi obrovsku derivaciu a robot
      // sekne do strany presne vo chvili, ked hranu najde.
      posFilt   = e.posMm;
      lastError = e.posMm;
      dFilt     = 0.0f;
      lostSince = 0;
      acquired  = true;
    }
    posFilt += (e.posMm - posFilt) * posLpf;
    error = posFilt;

  } else {
    // Hrana mimo bara. Dosadime virtualnu chybu - znamienko vyplyva z toho,
    // na ktorej strane je cierna a ci sme uleteli do bielej alebo do ciernej.
    if (lostSince == 0) lostSince = millis();
    if (millis() - lostSince > LOST_TIMEOUT_MS) { zastav(); stav = ST_STOP; return; }

    bool allWhite = (e.darkUnits <= SensorCount * 0.5f);
    int8_t s = (blackSide != 0) ? blackSide : (lastError >= 0 ? +1 : -1);
    error   = (allWhite ? s : -s) * LOST_ERROR_MM;
    posFilt = error;
  }

  // Derivacia sa pocita LEN z realneho merania. Virtualna chyba je skok,
  // z ktoreho by derivacia vyrobila nezmyselny kop do motorov.
  if (lost) {
    dFilt = 0.0f;
  } else {
    float dErr = (error - lastError) / dtMs;      // mm za milisekundu
    dFilt += (dErr - dFilt) * dLpf;
  }

  float corr = kp * error + kd * dFilt;
  corr = constrain(corr, -corrMax, corrMax);

  int speed = speedFor(fabsf(error));
  int c     = (int)(corr * steerSign);
  setMotorsDiff(speed + c, speed - c);

  lastError = error;
  digitalWrite(LED_BUILTIN, e.valid);      // svieti = vidi hranu
}

// --------------------------------------------------------------- LED VAROV
void blikni(uint8_t krat, uint16_t ms) {
  for (uint8_t i = 0; i < krat; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(ms);
    digitalWrite(LED_BUILTIN, LOW);  delay(ms);
  }
}

// -------------------------------------------------------------- KALIBRACIA
// false = prerusene tlacidlom
bool kalibracia() {
  digitalWrite(LED_BUILTIN, HIGH);

  for (uint16_t i = 0; i < CAL_STEPS; i++) {
    if (stlacene()) { zastav(); digitalWrite(LED_BUILTIN, LOW); return false; }
    tocSa(((i / CAL_FLIP) % 2 == 0) ? CAL_SPEED : -CAL_SPEED);
    qtr.calibrate();
  }
  zastav();
  digitalWrite(LED_BUILTIN, LOW);

  uint16_t worst = 65535, darkest = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    uint16_t mn = qtr.calibrationOn.minimum[i];
    uint16_t mx = qtr.calibrationOn.maximum[i];
    uint16_t sp = (mx > mn) ? (mx - mn) : 0;
    if (sp < worst)   worst   = sp;
    if (mx > darkest) darkest = mx;
  }

  // NAJVACSI ZDROJ RYCHLOSTI V CELOM KODE.
  // QTR-RC meria cas vybitia kondenzatora. Default timeout 2500 us znamena,
  // ze kazde citanie ciernej trva plnych 2.5 ms - to je cista latencia medzi
  // tym, kde robot je, a tym, ako na to zareaguju motory. Teraz uz vieme z
  // kalibracie, aka tmava je najtmavsia cierna, takze timeout stiahneme
  // tesne nad nu. Kratsia latencia = vyssie gainy bez rozkmitania = vyssia
  // rychlost. Tmavsi povrch nez pri kalibracii sa len oreze na 1000, cize
  // sa tym nic nepokazi.
  qtr.setTimeout(constrain((uint16_t)(darkest * 1.15f) + 60, 700, 2500));

  // Bez seriaku je toto jediny sposob, ako sa dozvies o zlej kalibracii.
  if (worst < CAL_MIN_RANGE) blikni(6, 70);
  return true;
}

// ------------------------------------------------------- MERANIE POLARITY
// Otocime sa nominalne doprava a pozrieme, kam sa posunula hrana v bare.
// Ked sa robot naozaj otoci doprava, hrana sa v bare posunie DOLAVA
// (posMm klesne). Ak stupne, mame prehodene motory alebo zrkadleny bar -
// v oboch pripadoch staci obratit znamienko korekcie.
// false = prerusene tlacidlom
bool zmeratPolaritu() {
#if FORCE_STEER_SIGN != 0
  steerSign = FORCE_STEER_SIGN;
  return true;
#else
  for (uint8_t pokus = 0; pokus < 3; pokus++) {
    if (stlacene()) { zastav(); return false; }

    int pwm = PROBE_PWM + pokus * 25;
    int ms  = PROBE_MS  + pokus * 60;

    if (!readEdge().valid) { delay(200); continue; }
    float p0 = readPosAvg(5);

    tocSa(pwm);  delay(ms);  zastav();  delay(120);
    float p1 = readPosAvg(5);
    tocSa(-pwm); delay(ms);  zastav();  delay(120);   // vrat sa spat

    float delta = p1 - p0;
    if (fabsf(delta) >= 4.0f) {
      steerSign = (delta < 0) ? +1 : -1;
      return true;
    }
  }
  steerSign = 1;            // nepodarilo sa; ak robot uteka od hrany,
  return true;              // daj FORCE_STEER_SIGN na -1
#endif
}

// ------------------------------------------------------------------ SETUP
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(800);

  ledcAttach(LEFT_FWD,  PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(LEFT_BWD,  PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_FWD, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_BWD, PWM_FREQ, PWM_RESOLUTION);
  zastav();

  qtr.setTypeRC();
  qtr.setSensorPins(SensorPins, SensorCount);
  qtr.setEmitterPins(EMITTER_EVEN, EMITTER_ODD);

  // Tlacidlo drzane pri zapnuti = RACE MODE.
  if (digitalRead(BUTTON_PIN) == LOW) {
    baseSpeed = RACE_BASE;
    minSpeed  = RACE_MIN;
    kp        = RACE_KP;
    kd        = RACE_KD;
    posLpf    = RACE_POS_LPF;
    dLpf      = RACE_D_LPF;
    slowStart = RACE_SLOW_FROM;
    slowFull  = RACE_SLOW_FULL;
    corrMax   = RACE_CORR_MAX;

    Serial.println("RACE MODE");          // jediny vypis v celom kode

    while (digitalRead(BUTTON_PIN) == LOW) delay(10);   // pockaj na pustenie
    delay(100);
  }

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButton, FALLING);
  buttonFlag = false;                     // zahod zakmit z pustenia

  ctrlTimer = timerBegin(1000000);                  // 1 MHz
  timerAttachInterrupt(ctrlTimer, &onControlTimer);
  timerAlarm(ctrlTimer, CONTROL_US, true, 0);       // 200 Hz

  stav = ST_CALIB;
}

// ------------------------------------------------------------------- LOOP
void loop() {
  switch (stav) {

    case ST_CALIB:
      stav = kalibracia() ? ST_READY : ST_STOP;
      break;

    case ST_READY: {
      Edge e = readEdge();
      static unsigned long t = 0;
      // rychle blikanie = bar je na hrane, mozes stlacit
      if (millis() - t > (unsigned long)(e.valid ? 120 : 500)) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        t = millis();
      }
      if (stlacene()) {
        digitalWrite(LED_BUILTIN, LOW);
        delay(400);                      // nechaj pustit tlacidlo
        buttonFlag = false;
        stav = ST_PROBE;
      }
      break;
    }

    case ST_PROBE:
      if (!zmeratPolaritu()) { stav = ST_STOP; break; }
      posFilt = lastError = dFilt = 0.0f;
      acquired    = false;
      lostSince   = 0;
      lastCtrlUs  = 0;
      controlTick = false;
      stav = ST_RUN;
      break;

    case ST_RUN:
      if (stlacene()) { zastav(); stav = ST_STOP; break; }   // nudzovy stop
      if (controlTick) {                                     // tik z timer ISR
        controlTick = false;
        controlStep();
      }
      break;

    case ST_STOP:
    default: {
      zastav();
      static unsigned long t = 0;
      if (millis() - t > 300) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        t = millis();
      }
      if (stlacene()) stav = ST_READY;
      break;
    }
  }
}
