/*  ###   ##  #    # #  ### ####  ## 
 *  #  # #  # #    # # #    #    #  #
 *  ###  #  # #     #   ##  ###  #  #
 *  #    #  # #     #     # #    #  #
 *  #     ##  ####  #  ###  ####  ### 
 *  
 *  VERSION 1.1
 *  2023.05.31
 *  
 *  VERSION 1.2
 *  2024.04.17
 *  
 *  SEBSONGS MODULAR
 */

/* VERSION 1.1 Features
 *  
 * Designed with performance in mind, POLYSEQ delivers a flexible polyphonic step sequencing workflow. 
 * Any of the 8 memory slots can be programmed via external CV/GATE up to 64 steps in length. 
 * In playback mode, any of the three outputs can play back from any memory slot, making it possible to create polyphonic and polymetric sequences with low effort. 
 * POLYSEQ can be clocked internally with swing or externally via the clock input.
 *
 * Polyseq can be clocked internally (5-300 BPM) or externally. 
 * The internal clock has swing fsrom 5 to 95% (Hold MODE and adjust the tempo pot). 
 *
 * Beside CV and GATE, sequences can have rests, ties and glide. 
 * There’s also a skip function which steps forward in the sequence without overwriting anything. 
 * This is handy for editing sequences or changing the sequence length. 
 * Each playback channel can do forward, backward, ping-pong and random playback (Hold MODE and press channel switch to toggle through the modes). 
 * Each playback channel can also be clock divided from 1 through 8 (Hold MODE+CH and adjust the memory division rotary switch). 
 * Finally, each channel can be  individually muted (Short press on channel switch, latching).
 * 
 * VERSION 1.2 additions
 * 
 * - Monitoring CV and GATE while recording now reflects the selected memory location. 
 *   For instance, if Channel 1 is tied to memory location 4 and recording is done on memory location 4, CV and GATE monitoring will come out of channel 1.
 * - Monitoring CV and GATE in idle mode. Works the same as for recording and reflects which channel has been dedicated to which memory location. 
 *   If the memory rotary switch is set to a memory location that hasn't been dedicated to a channel, monitoring will come out of channel 1.
 * - Mute press time has got its own variable "muteTimeConstant" and has been increased to make it easier to mute and unmute.
 * - V/OCT transposition of all three cv channels are now possible. To use this, connect both CV and Gate from a v/oct source. 
 *   Transposition happens when polyseq receives a gate input. The transposition offset is set back down to zero when playback is stopped to avoid confusing situations.
 * - Dynamic adjustable glide time. Hold CH2 switch and adjust tempo potentiometer to adjust. Glide time is based on the tempo, and default glide time is one step long.
 *   Adjusting the glide time multiplies the default glide time by a factor of 0 to 4. Glide time is also dynamically adjusted based on channel clock division, so really long glides are possible.
 * - CV input for V/OCT is now more precise and stable than version 1.1.
 * - Resetting all sequences to the first step during playback is now possible. While a sequence is playing, hold MODE and press PLAY to reset the sequence.
 * - Programming random sequences without the use of CV/GATE input has been implemented. In idle mode, hold either of the CH1 (major), CH2 (minor), or CH3 (chromatic) switches and then press and hold REC for a while.
 *   The REC LED will flash to indicate that a new sequence has been programmed. The programmed random sequence is always 32 steps in length. It programs random notes, rests, ties and glides.
 * - Holding MODE+REC has changed behaviour slightly. Holding for approximately 1 second now resets only the playback mode and clock division for all channels. 
 *   The REC LED flashes once to indicate this. Holding the MODE+REC switches for approximately 5 seconds resets the whole memory. 
 *   The REC LED flashes once after second to indicate the first reset and then lights up and stays on after 5 seconds to indicate that the whole memory has been reset.
 */

#include <EEPROM.h>
#include <Ramp.h>
#include <Smoothed.h>

// PIN 23 is the power save pin for the rp2040. Powersaving is turned off to get more stable ADC readings.
#define PS_PIN 23

// Serial debug, set to 1 to turn on debugging.
bool DEBUG = 0;

// Define Ramp variables for CV glide.
rampFloat glideCh1;
rampFloat glideCh2;
rampFloat glideCh3;

// Digital pin declarations
int cvOutput1 = 0;
int cvOutput2 = 1;
int cvOutput3 = 2;
int gateOutput1 = 3;
int gateOutput2 = 4;
int gateOutput3 = 5;
int gateInput = 6;
int clockInput = 7;
int playSwitch = 8;
int recSwitch = 9;
int modeSwitch = 10;
int channelSwitch1 = 11;
int channelSwitch2 = 12;
int channelSwitch3 = 13;
int clockLED = 14;
int recLED = 15;
int playLED = 16;
int inputLED = 17;
int channelLED1 = 18;
int channelLED2 = 19;
int channelLED3 = 20;

// Analog input pin declarations
int voctInput = A0;
int tempoPot = A1;
int memorySwitch = A2;

// Variables for smooth LED flashing in calibration mode.
int LEDBrightness = 0;
int fadeAmount = 1;

// Timing and counters.
unsigned long timing = 0;
unsigned long gateTime = 0;
int swingCounter = 0;
unsigned int cvInput = 0;

// Define Smoothed variables for analog input smoothing.
Smoothed <unsigned int> tempoSmoothed;
Smoothed <unsigned int> memorySmoothed;
Smoothed <unsigned int> cvInputSmoothed;
Smoothed <unsigned int> calibrationSmoothing;
float smoothedCV = 0;

// Variable to store semitone multiplication factor for CV output.
float semiTone = 0;

// Realtime varibles for all switches and potentiometers.
bool RECSWITCH = 0;
bool LATCHRECSWITCH = 0;
bool PLAYSWITCH = 0;
bool LATCHPLAYSWITCH = 0;
bool MODESWITCH = 0;
bool CH1SWITCH = 0;
bool CH2SWITCH = 0;
bool CH3SWITCH = 0;
int MEMSWITCH = 0;
int DIVISION = 0;
int TEMPO = 0;
int SWING = 0;
int CLOCKINPUT = 0;
int GATEINPUT = 0;
float GLIDE = 1;

// Boolean for checking if any LEDs are on.
bool LEDSFLASHING = 0;

// Variables for edge detection of switches and hysteresis check on potentiometers.
bool currentRecSwitch = 0;
bool currentPlaySwitch = 0;
bool currentModeSwitch = 0;
bool currentCh1Switch = 0;
bool currentCh2Switch = 0;
bool currentCh3Switch = 0;
uint32_t rawMemSwitch = 0;
int currentMemSwitch = 0;
int currentDivision = 0;
int currentTempoPot = 0;
uint32_t rawTempoPot = 0;
int currentSwing = 50;
bool currentClockInput = 0;
bool currentGateInput = 0;
int currentCVInput = 0;
char currentGlide = 128;

bool lastRecSwitch = 0;
bool lastPlaySwitch = 0;
bool lastModeSwitch = 0;
bool lastCh1Switch = 0;
bool lastCh2Switch = 0;
bool lastCh3Switch = 0;
uint32_t lastRawMemSwitch = 0;
int lastMemSwitch = 0;
int lastDivision = 0;
int lastTempoPot = 0;
int lastSwing = 50;
uint32_t lastRawTempoPot = 0;
bool lastClockInput = 0;
bool lastGateInput = 0;
unsigned int lastCVInput = 0;
char lastGlide = 0;

// Variables for the different 'while' modes in the main loop.
bool playMode = 0;
bool recMode = 0;
bool calibrationMode = 0;

// Debounce variables.
unsigned long lastDebounceTime = 0;
unsigned long lastDebounceTimeMicros = 0;
unsigned long lastMemoryResetTime = 0;
unsigned long lastPlaybackModeAndDivisionResetTime = 0;
unsigned long lastCalibrationTime = 0;
unsigned long debounceDelay = 5; 

// Clocking/Timing variables.
float internalClockDuration = 0;
unsigned long clockForward = 0;
unsigned long lastClockForward = 0;
unsigned long clockCounter = 0;
bool clockTriggered = 0;
unsigned long extClockRate = 0;
unsigned long lastExtClockRate = 0;
unsigned long timeSinceTrig = 0;

// The main counter variables for each sequencer output.
int sequenceCounterCh1 = 0;
int sequenceCounterCh2 = 0;
int sequenceCounterCh3 = 0;

// Counter variable used when step recording. Used to store sequence length.
int recordingCounter = 0;

// Varibles to store which memory location has been chosen on which channel.
int channel1Memory = 0;
int channel2Memory = 0;
int channel3Memory = 0;

// Varibles to store the sequence length for each channel.
int channel1Length = 0;
int channel2Length = 0;
int channel3Length = 0;

// Varibles to store the playback mode for each channel.
int channel1PlaybackMode = 0;
int channel2PlaybackMode = 0;
int channel3PlaybackMode = 0;

// Boolean variables used for pingpong playback mode.
bool pingPongCh1 = 0;
bool pingPongCh2 = 0;
bool pingPongCh3 = 0;

// Varibles to store the clock division factors and counters for each channel.
int channel1Division = 1;
int channel2Division = 1;
int channel3Division = 1;
int channel1DivCtr = 0;
int channel2DivCtr = 0;
int channel3DivCtr = 0;

// Boolean variables that are high if there's anything happening on a channel.
bool ch1Active = 0;
bool ch2Active = 0;
bool ch3Active = 0;

// Boolean variables to check if a channel is muted.
bool ch1Muted = 0;
bool ch2Muted = 0;
bool ch3Muted = 0;
bool ch1LastMuted = 0;
bool ch2LastMuted = 0;
bool ch3LastMuted = 0;

// Variables for storing the time each channel switch has been pressed in order to check for mutes.
unsigned long ch1MuteTime = 0;
unsigned long ch2MuteTime = 0;
unsigned long ch3MuteTime = 0;
int muteTimeConstant = 500;

// Variable to check if the division parameter has been changed in the last loop iteration.
bool divisionChanged = 0;

// Variable to define the LED on timer when changing playback mode.
unsigned long LEDTimer = 0;

// Variables to store the timing for swing.
float swingRear = 0;
float swingFront = 0;

// Variable for realtime transposition.
int transpose = 0;

// Variables to store CV input calibration ADC values and calculate the resolution of one CV Input semitone.
unsigned int ZEROVCAL = 0;
unsigned int SIXVCAL = 0;
float SEMITONECAL = 0; //ADC range of 6 Volts divided by the amount of semitones.
int zeroVCalAddr = 1200;
int sixVCalAddr = 1210;

// Variables for calibration mode timing and mode changes.
long calibrationCalc = 0;
long lastCalibrationCalc = 0;
int calibrationCounter = 0;
int calibrationStepCounter = 0;

// Variables for random sequence programming.
int randomGate = 0;
int randomGlide = 0;
int randomNote = 0;

int sequenceArray[1043] = {};

// Default array written into EEPROM when clearing all data.
char defaultArray[1043] = {
/*Sequence 0*/
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
/*Sequence 1*/
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
/*Sequence 2*/
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
/*Sequence 3*/
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
/*Sequence 4*/
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
/*Sequence 5*/
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
/*Sequence 6*/
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
/*Sequence 7*/
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
/*Sequence Length 0-7*/
16,16,16,16,16,16,16,16,
/*Channels Memory Position*/
0,0,0,
/*Playback Mode Ch 1-3*/
0,0,0,
/*Division Ch 1-3*/
0,0,0,
/*Swing setting*/
50,
/*Glide time factor*/
128
};

void setup() {

  // Define size of simulated RP2040 EEPROM.
  EEPROM.begin(2048);

  // Set pin modes.
  pinMode(cvOutput1, OUTPUT);
  pinMode(cvOutput2, OUTPUT);
  pinMode(cvOutput3, OUTPUT);
  pinMode(gateOutput1, OUTPUT);
  pinMode(gateOutput2, OUTPUT);
  pinMode(gateOutput3, OUTPUT);
  pinMode(gateInput, INPUT);
  pinMode(clockInput, INPUT_PULLUP);
  pinMode(playSwitch, INPUT_PULLUP);
  pinMode(recSwitch, INPUT_PULLUP);
  pinMode(modeSwitch, INPUT_PULLUP);
  pinMode(channelSwitch1, INPUT_PULLUP);
  pinMode(channelSwitch2, INPUT_PULLUP);
  pinMode(channelSwitch3, INPUT_PULLUP);
  pinMode(clockLED, OUTPUT);
  pinMode(recLED, OUTPUT);
  pinMode(playLED, OUTPUT);
  pinMode(inputLED, OUTPUT);
  pinMode(channelLED1, OUTPUT);
  pinMode(channelLED2, OUTPUT);
  pinMode(channelLED3, OUTPUT);
  pinMode(PS_PIN, OUTPUT);

  // Set power saving pin high to disable power saving.
  digitalWrite(PS_PIN, HIGH);

  // Set analog input/output resolution.
  analogReadResolution(12);
  analogWriteResolution(16);

  // Set PWM frequency.
  analogWriteFreq(100000);

  // Initiate smoothing for ADC inputs.
  memorySmoothed.begin(SMOOTHED_AVERAGE, 5);
  tempoSmoothed.begin(SMOOTHED_AVERAGE, 15);
  cvInputSmoothed.begin(SMOOTHED_AVERAGE, 4);
  calibrationSmoothing.begin(SMOOTHED_AVERAGE, 50);

  // Start serial port if DEBUG variable is set high.
  if(DEBUG == 1){
    Serial.begin(9600);
  }

  // Wait for a second.
  delay(1000);

  // Set the resolution for one semitone, used for V/OCT CV output.
  semiTone = 9980 * 0.08333333333;
  
  if(DEBUG == 1){
    Serial.println("POLYSEQ V1.0 Init!");
  }

  // Read the eeprom and update the sequencer arrays and stored parameters.
  eepromRead();
  setParams();

  // Update the front panel switches and potentiometers.
  readSwitches();

  // Wait 100 ms to make sure switch and potentiometer readings are stable.
  delay(100);

  TEMPO = currentTempoPot;

  randomSeed(analogRead(A3));

  // Enter calibration mode if MODE switch is held at startup.
  if(currentModeSwitch == HIGH && currentRecSwitch == LOW && currentPlaySwitch == LOW){
    calibrationMode = 1;
  }
  
  // Read the calibration values from EEPROM at startup.
  ZEROVCAL = readIntFromEEPROM(zeroVCalAddr);
  if(ZEROVCAL == 0){
    ZEROVCAL = 0;
    writeIntIntoEEPROM(zeroVCalAddr,ZEROVCAL);
  }
  SIXVCAL = readIntFromEEPROM(sixVCalAddr);
  if(SIXVCAL == 0){
    SIXVCAL = 3000;
    writeIntIntoEEPROM(sixVCalAddr,SIXVCAL);
  }
  SEMITONECAL = float(SIXVCAL-ZEROVCAL)/72;

  // Set initial gate time so first gate on first playback plays back fully.
  internalClockDuration = 15000/TEMPO;
  swingFront = ((internalClockDuration*2)-swingRear);
  gateTime = swingFront/2;
}

void loop() {

  /* - - - IDLE MODE - - - */

  // Update the millisecond timing variable.
  timing = millis();
  
  // Read CV inputs, switches, potentiometers and internal/external clock.
  clockHandler();
  readSwitches();
  readGateInput();
  updateCVInput();
  
  // Timing for turning off LEDs if playback mode is changed in IDLE mode.
  if(timing % 2 == 0 && LEDSFLASHING){
    LEDTimer++;
    if(LEDTimer > 2000){
      flashLED(0);
      LEDTimer = 0; 
    }
  }

  /* - - - CALIBRATION MODE - - - */

  while(calibrationMode){

    // Update the millisecond timing variable.
    timing = millis();

    // Flash input LED in a sinewave pattern.
    if(timing % 1 == 0 ){
      float sinewave = sin(((timing%360)/2) * 3.1415 / 180);
      analogWrite(inputLED, sinewave*65536);    
    }

    // Read CV/GATE inputs, switches, potentiometers.
    readSwitches();
    updateCVInput();
    readGateInput();

    transpose = 0;
    
    // Calibration procedure for reading the 0 Volt and 6 Volt ADC values for the CV input.
    // It looks for HIGH gate input and then waits for the CV input to settle. 
    // It first checks for a value close to 0V and then a value close to 6V.
    if(timing % 100 == 0 ){
      calibrationSmoothing.add(smoothedCV);
      if(GATEINPUT == HIGH){
        for (int i = 0; i < 1000; i++) {
          calibrationCalc = calibrationCalc + calibrationSmoothing.get();
          if(i >= 999){
            calibrationCalc = calibrationCalc/1000;
            if(DEBUG == 1){
              Serial.print("Cal Value: ");
              Serial.println(calibrationCalc);
            }
            if(calibrationCalc == lastCalibrationCalc){
              calibrationCounter++;
              if(calibrationCounter == 1){
                if(DEBUG == 1){
                  Serial.print("Calibration step counter: ");
                  Serial.println(calibrationStepCounter);
                }
                if(calibrationStepCounter == 0 && calibrationCalc < 100){
                  if(DEBUG == 1){
                    Serial.print("0V Calibration: ");
                    Serial.println(calibrationCalc);
                  }
                  writeIntIntoEEPROM(zeroVCalAddr,calibrationCalc);
                  flashLED(2);
                  calibrationCounter = 0;
                  calibrationStepCounter++;
                }
                if(calibrationStepCounter == 1 && calibrationCalc > 2450){
                  if(DEBUG == 1){
                    Serial.print("6V Calibration: ");
                    Serial.println(calibrationCalc);
                  }
                  writeIntIntoEEPROM(sixVCalAddr,calibrationCalc);
                  flashLED(3);
                  calibrationCounter = 0;
                  calibrationStepCounter++;
                }
              }
              break;
            }
            else{
              calibrationCounter = 0;
            }
            lastCalibrationCalc = calibrationCalc;
            calibrationCalc = 0;
          }
        } 
      }
    }

    // When calibration is done, check if MODE switch is pressed and store the measure values in EEPROM.
    if(MODESWITCH == HIGH && calibrationStepCounter > 1){
      ZEROVCAL = readIntFromEEPROM(zeroVCalAddr);
      SIXVCAL = readIntFromEEPROM(sixVCalAddr);
      SEMITONECAL = float(SIXVCAL-ZEROVCAL)/72;
      if(DEBUG == 1){
        Serial.print("Calibration range: ");
        Serial.print(ZEROVCAL);
        Serial.print(" - ");
        Serial.println(SIXVCAL);
        Serial.print("Semitone Value: ");
        Serial.println(SEMITONECAL);
      }
      flashLED(0);
      calibrationMode = 0;
    }
  }

  /* - - - RECORD MODE - - - */

  while(recMode){

    // Update the millisecond timing variable.
    timing = millis();

    // Set the internal clock counter to zero to be ready for playback after recording. 
    clockCounter = 0;
    
    // Read CV/GATE inputs, switches, potentiometers.
    readSwitches();
    updateCVInput();
    readGateInput();

  }

  /* - - - PLAYBACK MODE - - - */

  while(playMode){

    // Update timing, switches and clock.
    timing = millis();
    updateCVInput();
    readSwitches();
    readGateInput();
    clockHandler();

    // Move sequences forward when clock is triggered.
    if(clockCounter == 0 && !clockTriggered){
      clockTriggered = 1;
      readSequences(ch1Active,ch2Active,ch3Active);
      channelCounters();
    }

    // Timing for turning off LEDs if playback mode is changed in PLAY mode.
    if(timing % 2 == 0 && LEDSFLASHING){
      LEDTimer++;
      if(LEDTimer > 2000){
        flashLED(0);
        LEDTimer = 0;
        if(ch1Muted){
          digitalWrite(channelLED1, 1);
        }
        if(ch2Muted){
          digitalWrite(channelLED2, 1);
        }
        if(ch3Muted){
          digitalWrite(channelLED3, 1);
        }
      }
    }

    // Update CV outputs.
    float glideNoteCh1 = glideCh1.update();
    analogWrite(cvOutput1, glideNoteCh1);
    
    float glideNoteCh2 = glideCh2.update();
    analogWrite(cvOutput2, glideNoteCh2);
    
    float glideNoteCh3 = glideCh3.update();
    analogWrite(cvOutput3, glideNoteCh3);
    
    // Turn off all gate outputs when gate time has been reached (50% of the clock duration).
    if(clockCounter >= gateTime){
      resetAllOutputs();
    }

    // Edge detection for clock pulses, both internal and external.
    clockForward = millis();
    if (clockForward - lastClockForward >= 1)
    {
      if(clockCounter > 0){
        clockTriggered = 0;  
      }
      clockCounter++;
      lastClockForward = clockForward;
    }
    
  }

  // Reset all counters in idle mode.
  if(TEMPO >= 5){
    // Set the internal clock counter to zero to be ready for playback after recording. Only reset internal clock counter if on internal clock, not external.
    clockCounter = 0; 
  }
  else if(TEMPO < 5){
    clockCounter = 1;
    clockTriggered = 0;  
  }
  
  resetChannelCounters(1);
  resetChannelCounters(2);
  resetChannelCounters(3);
  channel1DivCtr = 0;
  channel2DivCtr = 0;
  channel3DivCtr = 0;
  ch1Active = 1;
  ch2Active = 1;
  ch3Active = 1;
  swingCounter = 0;
  resetAllOutputs();
}

// Read sequences from the chosen memory locations for each channel and output corresponding CV and GATE values on the outputs.
void readSequences(int ch1, int ch2, int ch3){

  digitalWrite(clockLED, 1);

  if(ch1 && !ch1Muted){
    digitalWrite(gateOutput1, bitRead(sequenceArray[sequenceCounterCh1 + channel1Memory + 64], 0));
    if(!LEDSFLASHING){
      digitalWrite(channelLED1, bitRead(sequenceArray[sequenceCounterCh1 + channel1Memory + 64], 0));
    }
    if(bitRead(sequenceArray[sequenceCounterCh1 + channel1Memory + 64], 2) == 1){
      glideCh1.go((sequenceArray[sequenceCounterCh1 + channel1Memory] + transpose) * semiTone,internalClockDuration*float(GLIDE*(channel1Division+1))/64);
    }
    else{
      glideCh1.go((sequenceArray[sequenceCounterCh1 + channel1Memory] + transpose) * semiTone,0);
    }
  }
  if(ch2 && !ch2Muted){
    digitalWrite(gateOutput2, bitRead(sequenceArray[sequenceCounterCh2 + channel2Memory + 64], 0));
    if(!LEDSFLASHING){
      digitalWrite(channelLED2, bitRead(sequenceArray[sequenceCounterCh2 + channel2Memory + 64], 0));
    }
    if(bitRead(sequenceArray[sequenceCounterCh2 + channel2Memory + 64], 2) == 1){
      glideCh2.go((sequenceArray[sequenceCounterCh2 + channel2Memory] + transpose) * semiTone,internalClockDuration*float(GLIDE*(channel2Division+1))/64);
    }
    else{
      glideCh2.go((sequenceArray[sequenceCounterCh2 + channel2Memory] + transpose) * semiTone,0);  
    }
  }

  if(ch3 && !ch3Muted){
    digitalWrite(gateOutput3, bitRead(sequenceArray[sequenceCounterCh3 + channel3Memory + 64], 0));
    if(!LEDSFLASHING){
      digitalWrite(channelLED3, bitRead(sequenceArray[sequenceCounterCh3 + channel3Memory + 64], 0));
    }
    if(bitRead(sequenceArray[sequenceCounterCh3 + channel3Memory + 64], 2) == 1){
      glideCh3.go((sequenceArray[sequenceCounterCh3 + channel3Memory] + transpose) * semiTone,internalClockDuration*float(GLIDE*(channel3Division+1))/64);
    }
    else{
      glideCh3.go((sequenceArray[sequenceCounterCh3 + channel3Memory] + transpose) * semiTone,0);
    }
  }
}

// Keep track of each individual sequence length.
void resetChannelCounters(int chosenChannel){

  if(chosenChannel == 1){
    if(channel1PlaybackMode == 1){
      sequenceCounterCh1 = channel1Length;
    }
    else{
      sequenceCounterCh1 = 0;
      pingPongCh1 = 0;
    }
  }

  if(chosenChannel == 2){
    if(channel2PlaybackMode == 1){
      sequenceCounterCh2 = channel2Length;
    }
    else{
      sequenceCounterCh2 = 0;
      pingPongCh2 = 0;
    }
  }

  if(chosenChannel == 3){
    if(channel3PlaybackMode == 1){
      sequenceCounterCh3 = channel3Length;
    }
    else{
      sequenceCounterCh3 = 0;
      pingPongCh3 = 0;
    }
  }
}

// Function to turn off all outputs and reset parameters.
void resetAllOutputs(){
  if(!LEDSFLASHING && !GATEINPUT){
    digitalWrite(inputLED, 0);
  }
  digitalWrite(clockLED, 0);

  bool tieCh1 = bitRead((sequenceArray[sequenceCounterCh1 + channel1Memory + 64])%64, 1);
  bool tieCh2 = bitRead((sequenceArray[sequenceCounterCh2 + channel2Memory + 64])%64, 1);
  bool tieCh3 = bitRead((sequenceArray[sequenceCounterCh3 + channel3Memory + 64])%64, 1);
  bool nextStepTieCh1 = bitRead((sequenceArray[sequenceCounterCh1 + 1 + channel1Memory + 64])%64, 1);
  bool nextStepTieCh2 = bitRead((sequenceArray[sequenceCounterCh2 + 1 + channel2Memory + 64])%64, 1);
  bool nextStepTieCh3 = bitRead((sequenceArray[sequenceCounterCh3 + 1 + channel3Memory + 64])%64, 1);

  if(!tieCh1 && !nextStepTieCh1 && !recMode && !GATEINPUT && !ch1Muted && channel1DivCtr == 0){
    digitalWrite(gateOutput1, 0);
    if(!LEDSFLASHING){
      digitalWrite(channelLED1, 0);
    }
  }
  if(!tieCh2 && !nextStepTieCh2 && !recMode && !GATEINPUT && !ch2Muted && channel2DivCtr == 0){
    digitalWrite(gateOutput2, 0);
    if(!LEDSFLASHING){
      digitalWrite(channelLED2, 0);
    }
  }
  if(!tieCh3 && !nextStepTieCh3 && !recMode && !GATEINPUT && !ch3Muted && channel3DivCtr == 0){
    digitalWrite(gateOutput3, 0);
    if(!LEDSFLASHING){
      digitalWrite(channelLED3, 0);
    }
  }

  if(!tieCh1 && playMode && !ch1Muted && channel1DivCtr == 0){
    digitalWrite(gateOutput1, 0);
    if(!LEDSFLASHING){
      digitalWrite(channelLED1, 0);
    }
  }
  if(!tieCh2 && playMode && !ch2Muted && channel2DivCtr == 0){
    digitalWrite(gateOutput2, 0);
    if(!LEDSFLASHING){
      digitalWrite(channelLED2, 0);
    }
  }
  if(!tieCh3 && playMode && !ch3Muted && channel3DivCtr == 0){
    digitalWrite(gateOutput3, 0);
    if(!LEDSFLASHING){
      digitalWrite(channelLED3, 0);
    }
  }
  
  if(recMode == 0 && playMode == 0 && !GATEINPUT){
    digitalWrite(gateOutput1, 0);
    digitalWrite(gateOutput2, 0);
    digitalWrite(gateOutput3, 0);
    pingPongCh1 = 0;
    pingPongCh2 = 0;
    pingPongCh3 = 0;
  }

  if(recMode == 0 && playMode == 0 && !LEDSFLASHING && !GATEINPUT && !ch1Muted){
    digitalWrite(channelLED1, 0);
    pingPongCh1 = 0;
  }

  if(recMode == 0 && playMode == 0 && !LEDSFLASHING && !GATEINPUT && !ch2Muted){
    digitalWrite(channelLED2, 0);
    pingPongCh2 = 0;
  }

  if(recMode == 0 && playMode == 0 && !LEDSFLASHING && !GATEINPUT && !ch3Muted){
    digitalWrite(channelLED3, 0);
    pingPongCh3 = 0;
  }

  if(recMode == 0 && playMode == 1 && !LEDSFLASHING && !tieCh1 && !GATEINPUT && !ch1Muted && channel1DivCtr == 0){
    digitalWrite(channelLED1, 0);
  }

  if(recMode == 0 && playMode == 1 && !LEDSFLASHING && !tieCh2 && !GATEINPUT && !ch2Muted && channel2DivCtr == 0){
    digitalWrite(channelLED2, 0);
  }
  
  if(recMode == 0 && playMode == 1 && !LEDSFLASHING && !tieCh3 && !GATEINPUT && !ch3Muted && channel3DivCtr == 0){
    digitalWrite(channelLED3, 0); 
  }
}

// Read the CV input and quantize to semitones, used for recording.
void updateCVInput(){

  cvInput = (analogRead(voctInput) * -1) + 4095; // Read the analog input and invert the range.
  int initValue = round(float(cvInput - ZEROVCAL)* (72/float(SIXVCAL-ZEROVCAL))); // Make an initial note value for the for loop to increase speed.
  
  cvInputSmoothed.add(cvInput); //  Filter the incoming CV to get a more stable reading.
  smoothedCV = cvInputSmoothed.get();

  // Find the correct note in the v/oct range.
  for (int i = initValue; i < 73; i++) {
    if(smoothedCV > (ZEROVCAL+float(SEMITONECAL*i)) - 18 && smoothedCV < (ZEROVCAL+float(SEMITONECAL*i)) + 18){
      currentCVInput = i;
      break;
    }
  }

  // Live monitoring of incoming CV.
  if(!playMode && !calibrationMode){
    if(currentMemSwitch == channel1Memory/128){
      if(GATEINPUT){
        analogWrite(cvOutput1, round(currentCVInput * semiTone));
      }
    }
    else if(currentMemSwitch == channel2Memory/128){
      if(GATEINPUT){
        analogWrite(cvOutput2, round(currentCVInput * semiTone));
      }
    }
    else if(currentMemSwitch == channel3Memory/128){
      if(GATEINPUT){
        analogWrite(cvOutput3, round(currentCVInput * semiTone));
      }
    }
    else{
      if(GATEINPUT){
        analogWrite(cvOutput1, round(currentCVInput * semiTone));
      }
    }
  }
 
}

// Read the GATE input, used for recording.
void readGateInput(){

  currentGateInput = !digitalRead(gateInput);

  if (currentGateInput != lastGateInput) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (currentGateInput != GATEINPUT) {
      GATEINPUT = currentGateInput;
      if(GATEINPUT && recMode == 1 && recordingCounter < 64 && millis() - timeSinceTrig > 5){
        digitalWrite(inputLED, 1);
        sequenceArray[recordingCounter + (MEMSWITCH * 128)] = currentCVInput;
        bitWrite(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0, GATEINPUT);
        bitWrite(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 1, 0);
        bitWrite(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 2, 0);
        
        if(DEBUG == 1){Serial.print("NOTE:  Note: ");
        Serial.print(sequenceArray[recordingCounter + (MEMSWITCH * 128)],DEC);}
        if(DEBUG == 1){Serial.print(" Gate: ");
        Serial.print(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0));}
        if(DEBUG == 1){Serial.print(" Tie: ");}
        if(DEBUG == 1){
          Serial.print(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 1));
          Serial.print(" Glide: ");
          Serial.println(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 2));
        }
        recordingCounter++;
      }
      else if(!GATEINPUT && recMode == 1 && recordingCounter < 64){
        digitalWrite(inputLED, 0);  
      }
      
      if(GATEINPUT && playMode == 0 && !calibrationMode){
        digitalWrite(inputLED, 1);
        if(currentMemSwitch == channel1Memory/128){
          digitalWrite(gateOutput1, 1);
          digitalWrite(channelLED1, 1);
        }
        else if(currentMemSwitch == channel2Memory/128){
          digitalWrite(gateOutput2, 1);
          digitalWrite(channelLED2, 1);
        }
        else if(currentMemSwitch == channel3Memory/128){
          digitalWrite(gateOutput3, 1);
          digitalWrite(channelLED3, 1);
        }
        else{
          digitalWrite(gateOutput1, 1);
        }
      }
      else if(!GATEINPUT && playMode == 0 && !calibrationMode){
        digitalWrite(inputLED, 0);
        digitalWrite(gateOutput1, 0);
        digitalWrite(gateOutput2, 0);
        digitalWrite(gateOutput3, 0);
        if(!ch1Muted){
          digitalWrite(channelLED1, 0);
        }
        if(!ch2Muted){
          digitalWrite(channelLED2, 0);
        }
        if(!ch3Muted){
          digitalWrite(channelLED3, 0);
        }
      }

      if(GATEINPUT && playMode == 1 && !calibrationMode){
        digitalWrite(inputLED, 1);
        transpose = currentCVInput;
      }
      else if(!GATEINPUT && playMode == 1 && !calibrationMode){      
        digitalWrite(inputLED, 0);
      }
    }
  }

  lastGateInput = currentGateInput;
  
}

// Handle internal/external clock and swing calculations.
void clockHandler(){

  // Set the internal tempo with tempo potentiometer.
  if (TEMPO >= 5){
    internalClockDuration = 15000/TEMPO;
  }

  // External clock
  currentClockInput = !digitalRead(clockInput);
  
  extClockRate = millis();
  
  if (currentClockInput != lastClockInput) {
    lastDebounceTimeMicros = micros();
  }

  if ((micros() - lastDebounceTimeMicros) > 5) {
    if (currentClockInput != CLOCKINPUT && TEMPO < 5 && playMode) {
      CLOCKINPUT = currentClockInput;
      if(CLOCKINPUT == 1){
        internalClockDuration = (extClockRate-lastExtClockRate)/2;
        lastExtClockRate = extClockRate;
        gateTime = internalClockDuration/2;
        clockCounter = 0;
      }
      if(CLOCKINPUT == 0){
        resetAllOutputs();
      }
    }
  } 

  swingRear = ((internalClockDuration/50)*currentSwing);
  swingFront = ((internalClockDuration*2)-swingRear);

  if (clockCounter >= internalClockDuration + swingFront && TEMPO >= 5 && swingCounter == 0) {
    clockCounter = 0;
    swingCounter++;
    gateTime = swingFront/2;
  }
  if (clockCounter >= internalClockDuration + swingRear && TEMPO >= 5 && swingCounter != 0) {
    clockCounter = 0;
    swingCounter = 0;
    gateTime = swingRear/2;
  }

  lastClockInput = currentClockInput;
  
}

// Counters for each playback channel and playback mode (FORWARD, BACKWARDS, PINGPONG and RANDOM).
void channelCounters(){
  channel1DivCtr++;
  if(channel1DivCtr > channel1Division){
    channel1DivCtr = 0;
  }
  if(channel1DivCtr == 0){
    ch1Active = 1;
    switch(channel1PlaybackMode){
      case 0:
        if(sequenceCounterCh1 >= channel1Length){
          sequenceCounterCh1 = 0;
        }
        else{
          sequenceCounterCh1++; 
        }
        break;
      case 1:
        if(sequenceCounterCh1 == 0){
          sequenceCounterCh1 = channel1Length;
        }
        else{
          sequenceCounterCh1--;
        }
        break;
      case 2:
        if(!pingPongCh1){
          sequenceCounterCh1++;
          if(sequenceCounterCh1 > channel1Length){
            sequenceCounterCh1--;
            pingPongCh1 = 1;
          }
        }
        else if(pingPongCh1){
          sequenceCounterCh1--;
          if(sequenceCounterCh1 < 0){
            sequenceCounterCh1++;
            pingPongCh1 = 0;
          }
        }
        break;
      case 3:
        sequenceCounterCh1 = random(channel1Length+1);
        break;  
    }
    
  }
  else{
    ch1Active = 0;  
  }
  
  channel2DivCtr++;
  if(channel2DivCtr > channel2Division){
    channel2DivCtr = 0;
  }
  if(channel2DivCtr == 0){
    ch2Active = 1;
    switch(channel2PlaybackMode){
      case 0:
        if(sequenceCounterCh2 >= channel2Length){
          sequenceCounterCh2 = 0;
        }
        else{
          sequenceCounterCh2++; 
        }
        break;
      case 1:
        if(sequenceCounterCh2 == 0){
          sequenceCounterCh2 = channel2Length;
        }
        else{
          sequenceCounterCh2--;
        }
        break;
      case 2:
        if(!pingPongCh2){
          sequenceCounterCh2++;
          if(sequenceCounterCh2 > channel2Length){
            sequenceCounterCh2--;
            pingPongCh2 = 1;
          }
        }
        else if(pingPongCh2){
          sequenceCounterCh2--;
          if(sequenceCounterCh2 < 0){
            sequenceCounterCh2++;
            pingPongCh2 = 0;
          }
        }
        break;
      case 3:
        sequenceCounterCh2 = random(channel2Length+1);
        break; 
     }
  }
  else{
    ch2Active = 0;  
  }
  
  
  channel3DivCtr++;
  if(channel3DivCtr > channel3Division){
    channel3DivCtr = 0;
  }
  if(channel3DivCtr == 0){
    ch3Active = 1;
    switch(channel3PlaybackMode){
    case 0:
      if(sequenceCounterCh3 >= channel3Length){
        sequenceCounterCh3 = 0;
      }
      else{
        sequenceCounterCh3++; 
      }
      break;
    case 1:
      if(sequenceCounterCh3 == 0){
        sequenceCounterCh3 = channel3Length;
      }
      else{
        sequenceCounterCh3--;
      }
      break;
    case 2:
      if(!pingPongCh3){
        sequenceCounterCh3++;
        if(sequenceCounterCh3 > channel3Length){
          sequenceCounterCh3--;
          pingPongCh3 = 1;
        }
      }
      else if(pingPongCh3){
        sequenceCounterCh3--;
        if(sequenceCounterCh3 < 0){
          sequenceCounterCh3++;
          pingPongCh3 = 0;
        }
      }
      break;
    case 3:
      sequenceCounterCh3 = random(channel3Length+1);
      break;
    }
    channel3DivCtr = 0;
  }
  else{
    ch3Active = 0;  
  }
  
  
}

// Reads switches and potentiometers and handles all key combinations.
void readSwitches(){

  currentRecSwitch = !digitalRead(recSwitch);
  currentPlaySwitch = !digitalRead(playSwitch);
  currentModeSwitch = !digitalRead(modeSwitch);
  currentCh1Switch = !digitalRead(channelSwitch1);
  currentCh2Switch = !digitalRead(channelSwitch2);
  currentCh3Switch = !digitalRead(channelSwitch3);
  
  if(timing % 25 == 0){
    memorySmoothed.add(analogRead(memorySwitch));
    rawMemSwitch = memorySmoothed.get();
  }

  if (rawMemSwitch < lastRawMemSwitch - 50 || rawMemSwitch >= lastRawMemSwitch + 50) {
    
    if(!MODESWITCH){
      currentMemSwitch = ((rawMemSwitch * -1)+4096)/512;
    }
    else if(MODESWITCH){
      currentDivision = ((rawMemSwitch * -1)+4096)/512;
    }
  }


  if(timing % 75 == 0){
    tempoSmoothed.add(analogRead(tempoPot));
    rawTempoPot = tempoSmoothed.get();
  }

  if (rawTempoPot < lastRawTempoPot - 4 || rawTempoPot >= lastRawTempoPot + 5) {
    if(!MODESWITCH && !CH2SWITCH){
      currentTempoPot = map(rawTempoPot,0,4095,0,600);
    }
    else if(MODESWITCH){
      currentSwing = map(rawTempoPot,0,4095,95,5);
    }
    
    if(CH2SWITCH && !MODESWITCH){
      currentGlide = rawTempoPot/16;
    }
  }

  /*********************************************************/

  if (currentRecSwitch != lastRecSwitch) {
    lastDebounceTime = millis();
    lastMemoryResetTime = millis();
    lastPlaybackModeAndDivisionResetTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay && !MODESWITCH && !CH1SWITCH && !CH2SWITCH && !CH3SWITCH) {

    if (currentRecSwitch != RECSWITCH) {
      RECSWITCH = currentRecSwitch;

      if (RECSWITCH == HIGH && recMode == 0 && playMode == 0 && !MODESWITCH && !calibrationMode) {
        recMode = 1;
        digitalWrite(recLED,1);
        digitalWrite(channelLED1, 0);
        digitalWrite(channelLED2, 0);
        digitalWrite(channelLED3, 0);
        if(DEBUG == 1){Serial.println("Rec mode ON!");}
        recordingCounter = 0;
      }
      else if(RECSWITCH == HIGH && recMode == 1 && !MODESWITCH && !calibrationMode){
        recMode = 0;
        if(DEBUG == 1){
          Serial.print("Memory Switch: ");
          Serial.println(MEMSWITCH);
        }
        if(recordingCounter > 0){
          sequenceArray[1024+MEMSWITCH] = recordingCounter-1;
          if(DEBUG == 1){
            Serial.print("Recording Counter in memory: ");
            Serial.println(sequenceArray[1024+MEMSWITCH]);
          }
          recordingCounter = 0;
          if(DEBUG == 1){Serial.println("Rec mode OFF!");}
          channel1Length = sequenceArray[1024+sequenceArray[1032]];
          channel2Length = sequenceArray[1024+sequenceArray[1033]];
          channel3Length = sequenceArray[1024+sequenceArray[1034]];
          digitalWrite(recLED,0);
          eepromWrite();
          if(ch1Muted){
            digitalWrite(channelLED1, 1);
          }
          if(ch2Muted){
            digitalWrite(channelLED2, 1);
          }
          if(ch3Muted){
            digitalWrite(channelLED3, 1);
          }
        }
        else if(!MODESWITCH){
          recMode = 0;
          recordingCounter = 0;
          digitalWrite(recLED,0);
          if(DEBUG == 1){Serial.println("No notes recorded, REC mode OFF!");}
        }
      }
    }
  }

  /*********************************************************/

  // Program 32 step long random sequences in major, minor och chromatic scales.
  if ((millis() - lastMemoryResetTime) > 100 && (millis() - lastMemoryResetTime) < 1000 && CH1SWITCH || (millis() - lastMemoryResetTime) > 100 && (millis() - lastMemoryResetTime) < 1000 && CH2SWITCH || (millis() - lastMemoryResetTime) > 100 && (millis() - lastMemoryResetTime) < 1000 && CH3SWITCH){
    if (currentRecSwitch != RECSWITCH) {
      RECSWITCH = currentRecSwitch;
      if (RECSWITCH && !recMode && !playMode && !calibrationMode) {
        if(DEBUG == 1){Serial.println("PROGRAMMING RANDOM SEQUENCE");}
        digitalWrite(recLED,1);
        for(int rdmSeq = 0; rdmSeq < 32; rdmSeq++){
          if(rdmSeq == 0){
            randomGate = 1;
          }
          else{
            randomGate = random(2);
          }
          randomGlide = random(7);
          if(randomGate){
            if(rdmSeq == 0){
              randomNote = 12*random(3);
            }
            else{
              randomNote = random(12);
            }
            if(CH1SWITCH){
              if(randomNote == 1 || randomNote == 3 || randomNote == 6 || randomNote == 8 || randomNote == 10){
                randomNote = randomNote - 1;
              }
              sequenceArray[rdmSeq + (MEMSWITCH * 128)] = randomNote + 12*random(2);
            }
            else if(CH2SWITCH){
              if(randomNote == 1 || randomNote == 4 || randomNote == 6 || randomNote == 9 || randomNote == 11){
                randomNote = randomNote - 1;
              }
              sequenceArray[rdmSeq + (MEMSWITCH * 128)] = randomNote + 12*random(2);
            }
            else if(CH3SWITCH){
              sequenceArray[rdmSeq + (MEMSWITCH * 128)] = randomNote + 12*random(2);
            }
            
            if(randomGlide == 1){
              bitWrite(sequenceArray[rdmSeq + (MEMSWITCH * 128) + 64], 2, 1);
            }
            else{
              bitWrite(sequenceArray[rdmSeq + (MEMSWITCH * 128) + 64], 2, 0);
            }
          }
          else{
            sequenceArray[rdmSeq + (MEMSWITCH * 128)] = sequenceArray[rdmSeq-1 + (MEMSWITCH * 128)];
          }
          
          bitWrite(sequenceArray[rdmSeq + (MEMSWITCH * 128) + 64], 0, randomGate);
          bitWrite(sequenceArray[rdmSeq + (MEMSWITCH * 128) + 64], 1, random(2));
          
          
        }
        sequenceArray[1024+MEMSWITCH] = 31;
        channel1Length = sequenceArray[1024+sequenceArray[1032]];
        channel2Length = sequenceArray[1024+sequenceArray[1033]];
        channel3Length = sequenceArray[1024+sequenceArray[1034]];
        
        eepromWrite();
        delay(20);
        digitalWrite(recLED,0);
        if(DEBUG == 1){Serial.println("DONE");}
      }
    }
  }

  if ((millis() - lastPlaybackModeAndDivisionResetTime) > 1000 && MODESWITCH){
    if (currentRecSwitch != RECSWITCH) {
      RECSWITCH = currentRecSwitch;
      if (RECSWITCH && !recMode && !playMode && !calibrationMode) {
        if(DEBUG == 1){Serial.println("RESETTING PLAYBACK MODES AND DIVISIONS");}
        digitalWrite(recLED, 1);
        channel1PlaybackMode = 0;
        channel2PlaybackMode = 0;
        channel3PlaybackMode = 0;
        sequenceArray[1035] = channel1PlaybackMode;
        sequenceArray[1036] = channel2PlaybackMode;
        sequenceArray[1037] = channel3PlaybackMode;
        
        channel1Division = 0;
        channel2Division = 0;
        channel3Division = 0;
        sequenceArray[1038] = channel1Division;
        sequenceArray[1039] = channel2Division;
        sequenceArray[1040] = channel3Division;
        
        eepromWrite();
        setParams();
        delay(100);
        digitalWrite(recLED, 0);
      }
    }
  }

  if ((millis() - lastMemoryResetTime) > 5000 && MODESWITCH){
      RECSWITCH = currentRecSwitch;
      if (RECSWITCH && !recMode && !playMode && !calibrationMode) {
        if(DEBUG == 1){Serial.println("RESETTING MEMORY");}
        for(int y = 0; y < 1043; y++){
          sequenceArray[y] = defaultArray[y];
          digitalWrite(recLED, 1);
        }
        eepromWrite();
        setParams();
        digitalWrite(recLED, 0);
        RECSWITCH = 0;
      }
  }
  
  /*********************************************************/

  if (currentPlaySwitch != lastPlaySwitch) {
    lastDebounceTime = millis();
    lastCalibrationTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (currentPlaySwitch != PLAYSWITCH) {
      PLAYSWITCH = currentPlaySwitch;

      if (PLAYSWITCH == HIGH && playMode == 0  && !MODESWITCH && !calibrationMode) {
        if (recMode == 1) {
          if(recordingCounter > 0){
            sequenceArray[1024+MEMSWITCH] = recordingCounter-1;
            channel1Length = sequenceArray[1024+sequenceArray[1032]];
            channel2Length = sequenceArray[1024+sequenceArray[1033]];
            channel3Length = sequenceArray[1024+sequenceArray[1034]];
            recMode = 0;
            recordingCounter = 0;
            digitalWrite(recLED,0);
            if(DEBUG == 1){Serial.println("Switched from Rec mode, Play mode ON!");}
            eepromWrite();
          }
          else{
            recMode = 0;
            recordingCounter = 0;
            digitalWrite(recLED,0);
            if(DEBUG == 1){Serial.println("No notes recorded, Play mode ON!");}
          }
        }
        playMode = 1;
        digitalWrite(playLED,1);
        if(DEBUG == 1){Serial.println("Play mode ON!");}
      }
      else if(PLAYSWITCH == HIGH && playMode == 1 && !MODESWITCH && !calibrationMode){
        playMode = 0;
        transpose = 0;
        eepromWrite();
        digitalWrite(playLED,0);
        if(DEBUG == 1){Serial.println("Play mode OFF!");}
      }
      else if(PLAYSWITCH == HIGH && playMode == 1 && MODESWITCH && !calibrationMode){
        
        resetChannelCounters(1);
        resetChannelCounters(2);
        resetChannelCounters(3);
        swingCounter = 1;
      }
      else if(PLAYSWITCH == HIGH && playMode == 0 && !calibrationMode){
        playMode = 0;
        transpose = 0;
        eepromWrite();
        digitalWrite(playLED,0);
        if(DEBUG == 1){Serial.println("Play mode OFF!");}
      }
    }
  }

  /*********************************************************/

  if (currentModeSwitch != lastModeSwitch) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (currentModeSwitch != MODESWITCH) {
      MODESWITCH = currentModeSwitch;
      if(MODESWITCH && recMode && recordingCounter < 64 && !calibrationMode){
        digitalWrite(clockLED, 1);
        sequenceArray[recordingCounter + (MEMSWITCH * 128)] = currentCVInput;
        bitWrite(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0, 0); //GATE
        bitWrite(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 1, 0); //TIE
        bitWrite(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 2, 0); //GLIDE
        if(DEBUG == 1){
          Serial.print("REST:  Note: ");
          Serial.print(sequenceArray[recordingCounter + (MEMSWITCH * 128)],DEC);
          Serial.print(" Gate: ");
          Serial.print(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0));
          Serial.print(" Tie: ");
          Serial.print(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 1));
          Serial.print(" Glide: ");
          Serial.println(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 2));
        }
        recordingCounter++;
      }
      else if(!MODESWITCH && recMode && recordingCounter < 64 && !calibrationMode){
        digitalWrite(clockLED, 0);
      }
    }
  }

  /*********************************************************/

  if (currentCh1Switch != lastCh1Switch) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (currentCh1Switch != CH1SWITCH && !calibrationMode) {
      CH1SWITCH = currentCh1Switch;
      if(CH1SWITCH && recMode && recordingCounter > 0 && recordingCounter < 64){
        digitalWrite(clockLED, 1);
        sequenceArray[recordingCounter + (MEMSWITCH * 128)] = sequenceArray[(recordingCounter + (MEMSWITCH * 128))-1];
        bitWrite(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0, 1); //GATE
        //bitWrite(sequenceArray[(recordingCounter + (MEMSWITCH * 128) + 64)-1], 1, 1); //TIE on previous step.
        bitWrite(sequenceArray[(recordingCounter + (MEMSWITCH * 128) + 64)], 1, 1); //TIE on current step.
        bitWrite(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 2, 0); //GLIDE
        if(DEBUG == 1){
          Serial.print("TIE:   Note: ");
          Serial.print(sequenceArray[recordingCounter + (MEMSWITCH * 128)],DEC);
          Serial.print(" Gate: ");
          Serial.print(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0));
          Serial.print(" Tie: ");
          Serial.print(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 1));
          Serial.print(" Glide: ");
          Serial.println(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 2));
        }
        recordingCounter++;
      }
      if(!CH1SWITCH && recMode && recordingCounter > 0 && recordingCounter < 64){
        digitalWrite(clockLED, 0);
      }
      if(!CH1SWITCH && !recMode && MODESWITCH && !divisionChanged){  
        channel1PlaybackMode++;
        if(channel1PlaybackMode > 3){
          channel1PlaybackMode = 0;
        }
        resetChannelCounters(1);
        if(DEBUG == 1){
          Serial.print("Play mode Ch1: ");
          Serial.println(channel1PlaybackMode);
        }
        sequenceArray[1035] = channel1PlaybackMode;
        flashLED(channel1PlaybackMode+1);
      }
      if(!CH1SWITCH && divisionChanged){
        divisionChanged = 0;
      }
      if(CH1SWITCH && !MODESWITCH && !recMode){
        ch1MuteTime = millis();
      }
      if(!CH1SWITCH && !MODESWITCH && !recMode && millis() - ch1MuteTime < muteTimeConstant){
        if(!ch1LastMuted){
          ch1Muted = 1;
          digitalWrite(gateOutput1, 0);
          digitalWrite(channelLED1, 1);
          ch1LastMuted = ch1Muted;
        }
        else if(ch1LastMuted){
          ch1Muted = 0;
          ch1LastMuted = ch1Muted;
        }
      }
    }
  }

  /*********************************************************/

  if (currentCh2Switch != lastCh2Switch) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (currentCh2Switch != CH2SWITCH && !calibrationMode) {
      CH2SWITCH = currentCh2Switch;
      if(CH2SWITCH && recMode && recordingCounter > 0 && recordingCounter < 64){
        digitalWrite(clockLED, 1);
        recordingCounter--;
        bitWrite(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 2, 1); //GLIDE
        recordingCounter++;
        if(DEBUG == 1){
          Serial.print("GLIDE: Note: ");
          Serial.print(sequenceArray[recordingCounter + (MEMSWITCH * 128)],DEC);
          Serial.print(" Gate: ");
          Serial.print(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0));
          Serial.print(" Tie: ");
          Serial.print(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 1));
          Serial.print(" Glide: ");
          Serial.println(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 2));
        }
      }
      if(!CH2SWITCH && recMode && recordingCounter > 0 && recordingCounter < 64){
        digitalWrite(clockLED, 0);
      }
      if(!CH2SWITCH && !recMode && MODESWITCH && !divisionChanged){  
        channel2PlaybackMode++;
        if(channel2PlaybackMode > 3){
          channel2PlaybackMode = 0;
        }
        if(DEBUG == 1){
          Serial.print("Play mode Ch2: ");
          Serial.println(channel2PlaybackMode);
        }
        resetChannelCounters(2);
        sequenceArray[1036] = channel2PlaybackMode;
        flashLED(channel2PlaybackMode+1);
      }
      if(!CH2SWITCH && divisionChanged){
        divisionChanged = 0;
      }
      if(CH2SWITCH && !MODESWITCH && !recMode){
        ch2MuteTime = millis();
      }
      if(!CH2SWITCH && !MODESWITCH && !recMode && millis() - ch2MuteTime < muteTimeConstant){
        if(!ch2LastMuted){
          ch2Muted = 1;
          digitalWrite(gateOutput2, 0);
          digitalWrite(channelLED2, 1);
          ch2LastMuted = ch2Muted;
        }
        else if(ch2LastMuted){
          ch2Muted = 0;
          ch2LastMuted = ch2Muted;
        }
      }
    }
  }

  /*********************************************************/

  if (currentCh3Switch != lastCh3Switch) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {

    if (currentCh3Switch != CH3SWITCH && !calibrationMode) {
      CH3SWITCH = currentCh3Switch;
      if(CH3SWITCH && recMode && recordingCounter < 64){
        digitalWrite(clockLED, 1);
        if(currentMemSwitch == channel1Memory/128){
          analogWrite(cvOutput1, sequenceArray[recordingCounter + (MEMSWITCH * 128)] * semiTone);
          digitalWrite(gateOutput1, bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0));
          digitalWrite(channelLED1, bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0));
          
        }
        else if(currentMemSwitch == channel2Memory/128){
          analogWrite(cvOutput2, sequenceArray[recordingCounter + (MEMSWITCH * 128)] * semiTone);
          digitalWrite(gateOutput2, bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0));
          digitalWrite(channelLED2, bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0));
        }
        else if(currentMemSwitch == channel3Memory/128){
          analogWrite(cvOutput3, sequenceArray[recordingCounter + (MEMSWITCH * 128)] * semiTone);
          digitalWrite(gateOutput3, bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0));
          digitalWrite(channelLED3, bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0));
        }
        else{
          digitalWrite(gateOutput1, 1);
        }
        if(DEBUG == 1){
          Serial.print("SKIP: Note: ");
          Serial.print(sequenceArray[recordingCounter + (MEMSWITCH * 128)],DEC);
          Serial.print(" Gate: ");
          Serial.print(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 0));
          Serial.print(" Tie: ");
          Serial.print(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 1));
          Serial.print(" Glide: ");
          Serial.println(bitRead(sequenceArray[recordingCounter + (MEMSWITCH * 128) + 64], 2));
        }
        recordingCounter++;
      }
      else if(recMode && !CH3SWITCH){
        digitalWrite(clockLED, 0);
        digitalWrite(gateOutput1, 0);
        digitalWrite(channelLED1, 0);
        digitalWrite(gateOutput2, 0);
        digitalWrite(channelLED2, 0);
        digitalWrite(gateOutput3, 0);
        digitalWrite(channelLED3, 0);
      }
      if(!CH3SWITCH && !recMode && MODESWITCH && !divisionChanged){  
        channel3PlaybackMode++;
        if(channel3PlaybackMode > 3){
          channel3PlaybackMode = 0;
        }
        if(DEBUG == 1){
          Serial.print("Play mode Ch3: ");
          Serial.println(channel3PlaybackMode);
        }
        resetChannelCounters(3);
        sequenceArray[1037] = channel3PlaybackMode;
        flashLED(channel3PlaybackMode+1);
      }
      if(!CH3SWITCH && divisionChanged){
        divisionChanged = 0;
      }
      if(CH3SWITCH && !recMode && !MODESWITCH){
        ch3MuteTime = millis();
      }
      if(!CH3SWITCH && !recMode && !MODESWITCH && millis() - ch3MuteTime < muteTimeConstant){
        if(!ch3LastMuted){
          ch3Muted = 1;
          digitalWrite(gateOutput3, 0);
          digitalWrite(channelLED3, 1);
          ch3LastMuted = ch3Muted;
        }
        else if(ch3LastMuted){
          ch3Muted = 0;
          ch3LastMuted = ch3Muted;
        }
      }
    }
  }


 /*********************************************************/

  if (currentMemSwitch != lastMemSwitch) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 50) {
  
    if (currentMemSwitch != MEMSWITCH && !MODESWITCH && !calibrationMode) {
      MEMSWITCH = currentMemSwitch;
      if(CH1SWITCH && !recMode){
        channel1Memory = MEMSWITCH * 128;
        sequenceArray[1032] = MEMSWITCH;
        sequenceCounterCh1 = 0;
        channel1Length = sequenceArray[1024+MEMSWITCH];
        EEPROM.write(1032, MEMSWITCH);
      }
      if(CH2SWITCH && !recMode){
        channel2Memory = MEMSWITCH * 128;
        sequenceArray[1033] = MEMSWITCH;
        sequenceCounterCh2 = 0;
        channel2Length = sequenceArray[1024+MEMSWITCH];
        EEPROM.write(1033, MEMSWITCH);
      }
      if(CH3SWITCH && !recMode){
        channel3Memory = MEMSWITCH * 128;
        sequenceArray[1034] = MEMSWITCH;
        sequenceCounterCh3 = 0;
        channel3Length = sequenceArray[1024+MEMSWITCH];
        EEPROM.write(1034, MEMSWITCH);
      }
    }
  }

  /*********************************************************/
  
  if (currentDivision != lastDivision) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 50) {
  
    if (currentDivision != DIVISION && MODESWITCH && !calibrationMode) {
      DIVISION = currentDivision;
      if(CH1SWITCH && !recMode){
        channel1Division = DIVISION;
        sequenceArray[1038] = channel1Division;
        divisionChanged = 1;
      }
      if(CH2SWITCH && !recMode){
        channel2Division = DIVISION;
        sequenceArray[1039] = channel2Division;
        divisionChanged = 1;
      }
      if(CH3SWITCH && !recMode){
        channel3Division = DIVISION;
        sequenceArray[1040] = channel3Division;
        divisionChanged = 1;
      }
    }
  }

  /*********************************************************/

  if (currentTempoPot != lastTempoPot) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 1) {

    if (currentTempoPot != TEMPO && !MODESWITCH && !calibrationMode) {
      TEMPO = currentTempoPot;
      if(DEBUG == 1){
        Serial.print("Tempo BPM: ");
        Serial.println(TEMPO/2);
      }
    }
  }

  /*********************************************************/

  if (currentSwing != lastSwing) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 1) {

    if (currentSwing != SWING && MODESWITCH && !calibrationMode) {
      SWING = currentSwing;
      sequenceArray[1041] = SWING;
      if(DEBUG == 1){
        Serial.print("Swing: ");
        Serial.println(SWING);
      }
    }
  }

  /*********************************************************/

  if (currentGlide != lastGlide) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > 1) {

    if (currentGlide != GLIDE && CH2SWITCH && !calibrationMode) {
      GLIDE = currentGlide;
      sequenceArray[1042] = GLIDE;
      if(DEBUG == 1){
        Serial.print("Glide: ");
        Serial.println(float(GLIDE)/64);
      }
    }
  }

  /*********************************************************/
  
  lastRecSwitch = currentRecSwitch;
  lastPlaySwitch = currentPlaySwitch;
  lastModeSwitch = currentModeSwitch;
  lastCh1Switch = currentCh1Switch;
  lastCh2Switch = currentCh2Switch;
  lastCh3Switch = currentCh3Switch;
  lastMemSwitch = currentMemSwitch;
  lastTempoPot = currentTempoPot;
  lastSwing = currentSwing;
  lastRawTempoPot = rawTempoPot;
  lastRawMemSwitch = rawMemSwitch;
  lastDivision = currentDivision;
  lastGlide = currentGlide;
}

// Set LEDs on and off.
void flashLED(int ledNumber){

  switch(ledNumber){
    case 0:
      LEDSFLASHING = 0;
      digitalWrite(inputLED, 0);
      digitalWrite(channelLED1, 0);
      digitalWrite(channelLED2, 0);
      digitalWrite(channelLED3, 0);
      break;
    case 1:
      LEDSFLASHING = 1;
      LEDTimer = 0;
      digitalWrite(inputLED, 1);
      digitalWrite(channelLED1, 0);
      digitalWrite(channelLED2, 0);
      digitalWrite(channelLED3, 0);
      break;
    case 2:
      LEDSFLASHING = 1;
      LEDTimer = 0;
      digitalWrite(inputLED, 0);
      digitalWrite(channelLED1, 1);
      digitalWrite(channelLED2, 0);
      digitalWrite(channelLED3, 0);
      break;
    case 3:
      LEDSFLASHING = 1;
      LEDTimer = 0;
      digitalWrite(inputLED, 0);
      digitalWrite(channelLED1, 0);
      digitalWrite(channelLED2, 1);
      digitalWrite(channelLED3, 0);
      break;
    case 4:
      LEDSFLASHING = 1;
      LEDTimer = 0;
      digitalWrite(inputLED, 0);
      digitalWrite(channelLED1, 0);
      digitalWrite(channelLED2, 0);
      digitalWrite(channelLED3, 1);
      break;
    default:
      LEDSFLASHING = 0;
      digitalWrite(inputLED, 0);
      digitalWrite(channelLED1, 0);
      digitalWrite(channelLED2, 0);
      digitalWrite(channelLED3, 0);
      break;
  }
}

// EEPROM Read function, reads through the EEPROM and stores it in the sequence array.
void eepromRead(){
  if(DEBUG == 1){Serial.println("EEPROM READ!");}
  for(int pos; pos < 1043; pos++){
    sequenceArray[pos] = EEPROM.read(pos);
    if(DEBUG == 1){
      Serial.print("Reading eeprom pos: ");
      Serial.print(pos);
      Serial.print(" Data: ");
      Serial.println(sequenceArray[pos],DEC);
    }
  }
}

// EEPROM Write function, reads the sequence array, compares with the EEPROM and updates any new data.
void eepromWrite(){
  if(DEBUG == 1){Serial.println("EEPROM WRITE!");}
  for(int pos; pos < 1043; pos++){
    if(sequenceArray[pos] != EEPROM.read(pos)){
      EEPROM.write(pos, sequenceArray[pos]);
      if(DEBUG == 1){
        Serial.print("Writing eeprom pos: ");
        Serial.print(pos);
        Serial.print(" Data: ");
        Serial.println(sequenceArray[pos],DEC);
      }
    }
  }
  EEPROM.commit();
}

// Set sequencer paramters stored in the sequence array (and effectively stored in EEPROM).
void setParams(){
  channel1Memory = sequenceArray[1032] * 128;
  channel2Memory = sequenceArray[1033] * 128;
  channel3Memory = sequenceArray[1034] * 128;

  channel1Length = sequenceArray[1024+sequenceArray[1032]];
  channel2Length = sequenceArray[1024+sequenceArray[1033]];
  channel3Length = sequenceArray[1024+sequenceArray[1034]];

  channel1PlaybackMode = sequenceArray[1035];
  channel2PlaybackMode = sequenceArray[1036];
  channel3PlaybackMode = sequenceArray[1037];

  channel1Division = sequenceArray[1038];
  channel2Division = sequenceArray[1039];
  channel3Division = sequenceArray[1040];

  SWING = sequenceArray[1041];
  GLIDE = sequenceArray[1042];
}

// Function to write 16 bit numbers to the 8 bit EEPROM.
void writeIntIntoEEPROM(int address, int number)
{ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);

  EEPROM.commit();
}

// Function to read 16 bit numbers from the 8 bit EEPROM.
int readIntFromEEPROM(int address)
{
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}
