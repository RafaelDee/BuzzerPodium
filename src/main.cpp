/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp8266-nodemcu-arduino-ide/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Button.h>
#include <FastLED.h>
#include <../../commonlib/networking.h>
#include <clock.h>
#include <Battery.h>
// #define PIN_CHRG 1
// #define PIN_BTN_SW 3
#define PIN_BTN_LED 2
#define PIN_BTN_SW 1
#define PIN_CHRG 3
#define PIN_FACE_LED 0
#define SENSE_PIN A0
#define COUNT_FACE_LED 22
#define COUNT_BTN_LED 3

Battery battery = Battery(3150, 3900, SENSE_PIN);
uint16_t averageBattVoltage = 0;
uint8_t averageBattLevel = 0;
#define LED_COLS 11
#define LED_ROWS 2
int ledMap[LED_ROWS][LED_COLS] = {
    {16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6},
    {17, 18, 19, 20, 21, 0, 1, 2, 3, 4, 5}};
enum LEDState
{
  OFF,
  StandBy,
  SpotLightFlash,
  SpotLight,
  SpotLightLeft,
  SpotLightRight,
  CorrectAnswer,
  WrongAnswer,
  SuspenseAnswer,
  CustomColorLight
};
// 3c:8a:1f:0b:aa:14
bool serialMode = false;
uint8_t broadcastAddress[] = {0x3C, 0x8A, 0x1F, 0x0B, 0xAA, 0x14};
// 08:D1:F9:99:22:58
// uint8_t broadcastAddress[] = {0x08, 0xD1, 0xF9, 0x99, 0x22, 0x58};
Button buzzerBtn(PIN_BTN_SW);
CRGB ledsFace[COUNT_FACE_LED];
CRGB ledsButton[COUNT_BTN_LED];
bool isCharging = false;
// Define the colors we'll be using
CRGB standByColor = CRGB::Yellow;
CRGB customFceColor = standByColor;
CRGB customBtnColor = standByColor;
// Create a struct_message called myData=
uint8_t fceBrightness = 5;
uint8_t btnBrightness = 5;

Networking networking = Networking();
// unsigned long lastTime = 0;
// unsigned long timerDelay = 2000; // send readings timer
bool initialized = false;
template <typename T>
T updateExponentialAverage(T avgSoFar, T newValue, float alpha = 0.2)
{
  // alpha is smoothing factor (0-1). Higher = more weight to new values
  return (alpha * newValue) + ((1.0 - alpha) * avgSoFar);
}
int ledIndex(int targetIndex, int ledCount)
{
  return (targetIndex % ledCount + ledCount) % ledCount;
}

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)
{
  // networking.sentPacket = true;
  /* Serial.print("Last Packet Send Status: "); */
  /* if (sendStatus == 0)
  {
    if (!initialized)
      initialized = true;
    // Serial.print("Delivery success pos: ");
    // Serial.println(podiumPos);
  } */
  /* else if (!initialized)
    Serial.println("Init fail");
  else
  {
    Serial.println("Delivery fail");
  } */
}

bool resetAnimation = true;
LEDState ledState = OFF;
void setLedState(LEDState state)
{
  ledState = state;
  resetAnimation = true;
}

void spotlight(SpotlightPacket spotlightPacket)
{
  // Serial.print("SPOTLIGHT RECEIVED pos: ");
  // Serial.println(String(spotlightPacket.pos));
  int dir = spotlightPacket.dir;
  if (isnan(dir))
  {
    if (serialMode)
      Serial.println("invalid pos");
    return;
  }
  switch (dir)
  {
  case SpotlightDirection::Self:
    setLedState(spotlightPacket.flash ? SpotLightFlash : SpotLight);
    break;
  case SpotlightDirection::Right:
    setLedState(SpotLightRight);
    break;
  case SpotlightDirection::Left:
    setLedState(SpotLightLeft);
    break;

  default:
    break;
  }
  /*  if (dir == SpotlightDirection::Self)
   {

     // animate flashing
     return;
   }
   bool toTheRight = spotlightPacket.pos < podiumPos;
   if (toTheRight)
   {
     setLedState(SpotLightRight);
   }
   else
   {
     setLedState(SpotLightLeft);
   } */
  // turn off brightness after
}
void resetState()
{
  setLedState(StandBy);
}
void initialize(InitPacket initPacket)
{
  // Serial.print("Received Init Packet:");
  // Serial.println(initPacket.pos);
  initialized = true;
}
void sendBattStatus()
{
  networking.sendPacket(broadcastAddress, BatteryStatPacket(averageBattLevel, averageBattVoltage, isCharging));
  // networking.sendPacket(broadcastAddress, BatteryStatPacket(battery.level(), battery.voltage()));
}
void batteryCheck()
{

  // we cant measure voltage while led is on!, gives voltage sag results
  averageBattLevel = updateExponentialAverage(averageBattLevel, battery.level());
  averageBattVoltage = updateExponentialAverage(averageBattVoltage, battery.voltage());
  sendBattStatus();
}
void setCustomColor(CustomColorPacket packet)
{
  customBtnColor = packet.buttonColor;
  customFceColor = packet.faceColor;
  setLedState(LEDState::CustomColorLight);
}
bool brightnessTransition = false;
void setBrightness(LedBrightnessPacket packet)
{
  brightnessTransition = true;
  if (serialMode)
  {
    Serial.print("SETTING B ");
    Serial.print(packet.fceBright);
    Serial.print(":");
    Serial.println(packet.btnBright);
  }

  fceBrightness = packet.fceBright;
  btnBrightness = packet.btnBright;
}
// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len)
{
  serverPacketType type = networking.getPacketType<serverPacketType>(incomingData);
  if (serialMode)
  {
    Serial.println("REC: DATA");

    Serial.print("TYPE: ");
    Serial.println(type);
  }
  if (type == serverPacketType::Spotlight)
  {
    spotlight(networking.receivePacket<SpotlightPacket>(mac, incomingData, len));
    return;
  }
  if (type == serverPacketType::Initialize)
  {
    initialize(networking.receivePacket<InitPacket>(mac, incomingData, len));
    batteryCheck();
    return;
  }
  if (type == serverPacketType::BatteryPing)
  {
    sendBattStatus();
    return;
  }
  if (type == serverPacketType::ResetState)
  {
    resetState();
    return;
  }
  if (type == serverPacketType::CorrectAns)
  {
    setLedState(CorrectAnswer);
    return;
  }
  if (type == serverPacketType::WrongAns)
  {
    setLedState(WrongAnswer);
    return;
  }
  if (type == serverPacketType::Suspense)
  {
    setLedState(SuspenseAnswer);
    return;
  }
  if (type == serverPacketType::CustomColor)
  {
    setCustomColor(networking.receivePacket<CustomColorPacket>(mac, incomingData, len));
    return;
  }
  if (type == serverPacketType::LedBrightness)
  {
    setBrightness(networking.receivePacket<LedBrightnessPacket>(mac, incomingData, len));
    return;
  }
  /* switch (type)
  {
  case serverPacketType::Spotlight:

    break;
  case serverPacketType::Initialize:

    // sendBattStatus();
    return;
  case serverPacketType::BatteryPing:
    sendBattStatus();
    return;
  case serverPacketType::ResetState:
    resetState();
    return;
  case serverPacketType::CorrectAns:
    setLedState(CorrectAnswer);
    return;
  case serverPacketType::WrongAns:
    setLedState(WrongAnswer);
    return;
  case serverPacketType::Suspense:
    if (serialMode)
      Serial.println("TRIGGERED SUSPENSE");
    setLedState(SuspenseAnswer);
    return;
  case serverPacketType::CustomColor:
    setCustomColor(networking.receivePacket<CustomColorPacket>(mac, incomingData, len));
    return;

  case serverPacketType::LedBrightness:
    setBrightness(networking.receivePacket<LedBrightnessPacket>(mac, incomingData, len));
    return;
  } */
}

void setup()
{
  // Init Serial Monitor
  if (serialMode)
    Serial.begin(115200);
  initialized = false;
  battery.begin(1000, 4.2, &sigmoidal);
  // Set device as a Wi-Fi Station
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  if (!serialMode)
    buzzerBtn.begin();
  pinMode(PIN_CHRG, INPUT);
  // Init ESP-NOW
  if (esp_now_init() != 0)
  {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  FastLED.addLeds<NEOPIXEL, PIN_FACE_LED>(ledsFace, COUNT_FACE_LED);
  FastLED.addLeds<NEOPIXEL, PIN_BTN_LED>(ledsButton, COUNT_BTN_LED);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted Packet

  // Register peer

  // send an init packet
  // should send infinitely
  // use ondata sent to check if init success
  FastLED.delay(10);
  for (int i = 0; i < COUNT_BTN_LED; i++)
  {
    ledsButton[i] = CRGB::Black;
  }
  for (int i = 0; i < COUNT_FACE_LED; i++)
  {
    ledsFace[i] = CRGB::Black;
  }
  FastLED.delay(30);
  FastLED.show();
  FastLED.delay(300);
  averageBattLevel = battery.level();
  averageBattVoltage = battery.voltage();
  resetState();
  if (serialMode)
    Serial.println("INIT");
}

void initializeSystem()
{
  static unsigned long lit = false;

  static unsigned long lastTime = 0;
  if (Clock::TimePassed(lastTime, 400, true))
  {

    // Set values to send
    // Send message via ESP-NOW
    networking.sendPacket<InitializationPacket>(broadcastAddress, InitializationPacket());
    
    ledsFace[0] = lit ? CRGB::White : CRGB::Black;
    lit = !lit;
    FastLED.show();
  }
}
void ButtonPressed()
{
  networking.sendPacket(broadcastAddress, ButtonPacket());
}

bool spotLightSlide(/* CRGB color, */ bool slideLeft, bool localReset = false, bool useLocalReset = false)
{
  static int keyframe = 0;
  static int extraFrame = 0;
  static bool animDone = false;
  static unsigned long lastTick = 0;
  static unsigned long refreshTick = 0;

  static unsigned int animSpeed = 30;
  static bool slideDone = false;
  if (useLocalReset ? localReset : resetAnimation)
  {

    if (!useLocalReset)
      resetAnimation = false;
    slideDone = false;
    extraFrame = 0;
    keyframe = 0;
    refreshTick = millis();
    lastTick = millis();
    animDone = false;
  }
  if (animDone)
    return true;
  if (!Clock::TimePassed(refreshTick, 5, true))
    return false;
  // int ledsToFadeSize = (keyframe + 1) * LED_ROWS;
  //  CRGB ledsToFade[ledsToFadeSize] = {};
  if (slideDone)
  {
    if (extraFrame > 50)
    {
      animDone = true;
    }
    extraFrame++;
  }
  for (int r = 0; r < LED_ROWS; r++)
  {
    int kf = slideLeft ? keyframe : LED_COLS - 1 - keyframe;
    int loopUntil = (slideLeft ? kf : LED_COLS);
    for (int t = (slideLeft ? 0 : kf); t < loopUntil; t++)
    {
      int ledIndex = ledMap[r][t];
      if (ledIndex == -1)
        continue;
      ledsFace[ledIndex].nscale8(200);
    }

    /* ledsFace[ledIndex] = standByColor; */
  }
  // fadeToBlackBy(ledsToFade, ledsToFadeSize, 50);
  if (!Clock::TimePassed(lastTick, animSpeed, true))
    return false;

  // offset by 50 to end the whole dimming process
  if (keyframe >= LED_COLS)
  {
    slideDone = true;
    return false;
  }
  else
  {
    keyframe++;
  }
  return false;
}
void suspenseSlide()
{
  static bool slideLeft = true;
  static int keyframe = 0;
  static unsigned long lastTick = 0;
  static unsigned long startMilis = 0;
  if (resetAnimation)
  {
    keyframe = 0;
    lastTick = millis();
    startMilis = millis();
    resetAnimation = false;
  }
  // unsigned long relativeMilis = millis() - startMilis;
  if (!Clock::TimePassed(lastTick, 10, true))
    return;
  keyframe = beatsin8(60, 0, LED_COLS - 1, startMilis, 7); // Pulse with a sine wave
  // uint8_t brightnessFce = beatsin8(120, 0, 255, startMilis, 0); // Pulse with a sine wave
  /* if (LED_COLS <= keyframe)
  {
    keyframe = 0;
  } */

  for (int r = 0; r < LED_ROWS; r++)
  {
    int ledIndex = ledMap[r][keyframe];
    if (ledIndex == -1)
      continue;
    ledsFace[ledIndex] = standByColor;
    // fadeToBlackBy(&ledsFace[ledIndex], 1, brightnessFce);
  }
  uint8_t brightness = beatsin8(60, 0, 255); // Pulse with a sine wave
  for (int i = 0; i < COUNT_BTN_LED; i++)
  {
    ledsButton[i] = standByColor;
  }
  fadeToBlackBy(ledsButton, COUNT_BTN_LED, brightness);
  fadeToBlackBy(ledsFace, COUNT_FACE_LED, 30);
}
float easeInOut(float t)
{
  return t < 0.5f ? 2 * t * t : 1 - pow(-2 * t + 2, 2) / 2;
}
// hate it
bool transitionToColorLeds(CRGB *ledArray, int ledCount, CRGB targetColor, unsigned long duration, unsigned long *previousMillis, unsigned long *lastTime, int *incrementPerSecond, boolean *done, float *blendVal, float *blendTotal, bool localReset = false, bool useLocalReset = false)
{
  static int targetValue = 255;
  if (useLocalReset ? localReset : resetAnimation)
  {
    *previousMillis = millis();
    if (!useLocalReset)
      resetAnimation = false;
    // startTime = millis();
    *lastTime = millis();
    *blendVal = 0;
    *blendTotal = 0;
    *done = false;
    *incrementPerSecond = targetValue / (duration / 1000.0f);
  }
  if (*done)
    return true;
  if (!Clock::TimePassed(*lastTime, 10, true))
    return false;
  unsigned long currentMillis = millis();
  // unsigned long elapsedTime = currentMillis - startTime;
  float deltaTime = (currentMillis - *previousMillis) / 1000.0f;
  *previousMillis = currentMillis;
  // does not like elapsed time based done
  // not blending completely
  if (*blendTotal > 255 & targetColor == ledArray[0])
  {
    *done = true;
  }
  *blendVal += *incrementPerSecond * deltaTime;
  *blendTotal += *blendVal;
  if (*blendVal > targetValue)
  {
    *blendVal = targetValue;
  }
  // accumulate until whole number
  if (*blendVal > 1)
  {
    // remove mult if shit is resolved, blend is done at 90%
    int wholeBlendAmount = floor(*blendVal);
    for (int i = 0; i < ledCount; i++)
    {
      ledArray[i] = blend(ledArray[i], targetColor, wholeBlendAmount * 2);
    }
    // subtract whole numbers so data is not lost
    *blendVal -= wholeBlendAmount;
  }
  return false;
}

bool transitionToColorFce(CRGB targetColor, unsigned long duration, bool localReset = false, bool useLocalReset = false)
{
  static unsigned long previousMillis = 0;
  static unsigned long lastTime = 0;
  static int incrementPerSecond;
  // static unsigned long startTime = 0;
  static boolean done = false;
  static float blendVal = 0;
  static float blendTotal = 0;
  return transitionToColorLeds(ledsFace, COUNT_FACE_LED, targetColor, duration, &previousMillis, &lastTime, &incrementPerSecond, &done, &blendVal, &blendTotal, localReset, useLocalReset);
}
bool transitionToColorBtn(CRGB targetColor, unsigned long duration, bool localReset = false, bool useLocalReset = false)
{
  static unsigned long previousMillis = 0;
  static unsigned long lastTime = 0;
  static int incrementPerSecond;
  // static unsigned long startTime = 0;
  static boolean done = false;
  static float blendVal = 0;
  static float blendTotal = 0;
  return transitionToColorLeds(ledsButton, COUNT_BTN_LED, targetColor, duration, &previousMillis, &lastTime, &incrementPerSecond, &done, &blendVal, &blendTotal, localReset, useLocalReset);
}
bool setAllLights(CRGB colorFce, CRGB colorBtn, bool faceplate, bool button, unsigned long durationMs = 1000, bool localReset = false, bool useLocalReset = false)
{
  static bool done = false;
  static bool doneF = false;
  static bool doneB = false;
  static bool reset = true;
  if (useLocalReset ? localReset : resetAnimation)
  {
    if (!useLocalReset)
      resetAnimation = false;
    done = false;
    doneF = false;
    doneB = false;
    reset = true;
  }
  if (done)
  {
    return true;
  }
  if (button)
    doneB = transitionToColorBtn(colorBtn, durationMs, reset, true);
  else
    doneB = true;
  if (faceplate)
    doneF = transitionToColorFce(colorFce, durationMs, reset, true);
  else
    doneF = true;
  reset = false;
  if (doneF && doneB)
    done = true;
  return false;
}

bool setAllLights(CRGB color, bool faceplate, bool button, unsigned long durationMs = 1000, bool localReset = false, bool useLocalReset = false)
{
  return setAllLights(color, color, faceplate, button, durationMs, localReset, useLocalReset);
}

bool setLightsOFF(bool faceplate, bool button, unsigned long durationMs = 1000, bool localReset = false, bool useLocalReset = false)
{
  static bool done = false;
  static bool battChecked = false;
  if (useLocalReset ? localReset : resetAnimation)
  {
    if (!useLocalReset)
      resetAnimation = false;
    done = false;
    battChecked = false;
  }
  if (done)
  {
    if (!battChecked)
    {
      // delay(50);
      batteryCheck();
      battChecked = true;
    }
    return true;
  }
  done = setAllLights(CRGB::Black, faceplate, button, durationMs, localReset, useLocalReset);
  return false;
}

void flashing(CRGB color = CRGB::Black, unsigned long fDur = 1000)
{
  static bool done = false;
  static unsigned long lastLedStateTime = 0;
  static unsigned long lastTickTime = 0;
  static bool isFlashing = false;
  if (resetAnimation)
  {
    isFlashing = false;
    done = false;
    lastLedStateTime = millis();
    lastTickTime = millis();
    resetAnimation = false;
  }
  if (done)
    return;
  if (!Clock::TimePassed(lastTickTime, 100, true))
    return;
  if (Clock::TimePassed(lastLedStateTime, fDur))
  {
    isFlashing = true;
    done = true;
  }

  if (isFlashing)
  {
    for (int i = 0; i < COUNT_FACE_LED; i++)
    {
      ledsFace[i] = color;
    }
    for (int i = 0; i < COUNT_BTN_LED; i++)
    {
      ledsButton[i] = color;
    }
  }
  else if (!done)
  {
    fadeToBlackBy(ledsFace, COUNT_FACE_LED, 150);
    fadeToBlackBy(ledsButton, COUNT_BTN_LED, 150);
  }
  isFlashing = !isFlashing;
}
void spotLigtOnThenSlide(bool slideLeft)
{
  static int animStage = 0;
  static bool done = false;
  static bool btnReset = true;
  static bool battChecked = false;
  if (resetAnimation)
  {
    animStage = 0;
    done = false;
    resetAnimation = false;
    btnReset = true;
    battChecked = false;
  }
  if (done)
  {
    if (!battChecked)
    {
      batteryCheck();
      battChecked = true;
    }
    return;
  }
  transitionToColorBtn(CRGB::Black, 200, btnReset, true);
  btnReset = false;
  bool doneTrans = transitionToColorFce(standByColor, 200, animStage == 0, true);
  if (!doneTrans)
    animStage = 1;
  if (doneTrans)
  {
    bool doneSp = spotLightSlide(slideLeft, animStage == 1, true);
    animStage = 2;
    if (doneSp)
      done = true;
    // to allow a chance to measure battery
  }
}
void standBy()
{
  static int animStage = 0;
  static bool done = false;
  static bool battChecked = false;
  if (resetAnimation)
  {
    resetAnimation = false;
    animStage = 0;
    done = false;
    battChecked = false;
  }
  if (done)
    return;
  bool doneTrans = setLightsOFF(true, true, 200, animStage == 0, true);
  if (!doneTrans)
    animStage = 1;
  if (doneTrans)
  {
    if (!battChecked)
    {
      // FastLED.delay(30);
      batteryCheck();
      battChecked = true;
    }
    bool doneSp = setAllLights(standByColor, true, true, 500, animStage == 1, true);
    animStage = 2;
    if (doneSp)
      done = true;
  }
}
void RenderLights()
{
  switch (ledState)
  {
  case StandBy:
    standBy();
    break;
  case SpotLightFlash:
    flashing(standByColor);
    break;
  case SpotLightLeft:
    spotLigtOnThenSlide(true);
    break;
  case SpotLightRight:
    spotLigtOnThenSlide(false);
    break;
  case OFF:
    setLightsOFF(true, true);
    break;
  case SpotLight:
    setAllLights(standByColor, true, true, 500);
    break;
  case CorrectAnswer:
    flashing(CRGB::Green, 700);
    break;
  case WrongAnswer:
    flashing(CRGB::Red, 700);
    break;
  case SuspenseAnswer:
    suspenseSlide();
    break;
  default:
    break;
  }
}
void loop()
{
  if (!serialMode && buzzerBtn.pressed())
  {
    ButtonPressed();
  }

  if (initialized)
  {
    networking.heartbeat(broadcastAddress, 0);
  }
  else
  {
    initializeSystem();
    return;
  }
  static unsigned long lastTick = 0;
  if (!serialMode && Clock::TimePassed(lastTick, 20, true))
  {
    bool readChrg = digitalRead(PIN_CHRG);
    if (readChrg != isCharging)
    {
      isCharging = readChrg;
      sendBattStatus();
    }
  }
  static unsigned long battTick = 0;
  if (Clock::TimePassed(battTick, 5000, true) && (ledState == LEDState::OFF))
    batteryCheck();
  static unsigned long clockTick = 0;
  if (Clock::TimePassed(clockTick, 20, true))
  {
    RenderLights();
    if (brightnessTransition)
    {
      float currentBrightness = FastLED.getBrightness();
      float targetBrightness = btnBrightness;

      // Lerp factor (0.1 = smooth transition, increase for faster transition)
      float t = 0.1;

      // If very close to target, snap to it and end transition
      if (abs(currentBrightness - targetBrightness) < 0.5)
      {
        currentBrightness = targetBrightness;
        brightnessTransition = false;
      }
      else
      {
        // Linear interpolation
        currentBrightness = constrain(currentBrightness + (targetBrightness - currentBrightness) * t, 0, 255);
      }

      FastLED.setBrightness(currentBrightness);
    }
    FastLED.show();
  }

  delay(1);
}
