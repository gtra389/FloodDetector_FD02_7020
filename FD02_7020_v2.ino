#include "src\DS3231\DS3231.h" // https://github.com/NorthernWidget/DS3231
#include "src\SIM7020\SIMcore.h"
#include "SIM7020_httpGet.h"
//#include <histogram.h> 
#include <Sleep_n0m1.h>
#include <Battery.h>

/* Definition of variable
*/

const char Text_1[] PROGMEM = "/update_general.php?site=FD01_RMO";
const char Text_2[] PROGMEM = "7599";
const char* const TextTable[] PROGMEM = {
    Text_1,
    Text_2,
};

const byte PROGMEM pin_sleep = 9;
const byte PROGMEM pin_tx = 7;
const byte PROGMEM pin_rx = 8;
const byte PROGMEM pin_k = 6;
const byte PROGMEM pin_battery_activation = 5;
const byte PROGMEM pinTrigger = 10; // pin for sending out the signal
const byte PROGMEM pinEcho = 11; // pin for receiving the echo of the signal

const byte  samplingNum = 10;
float sampleArray[samplingNum];
float pipeLength; // Unit in cm
const float  minDepth         = -1; //  Unit in cm
const float  officalThreshold = 5.0;  //  Unit in cm
unsigned int histogramIntv[7];

const float  freqThreshold = 0.90;
int timeCount   = -1;
int timeCountThreshold = 3;
int BatLev;
String tS;

unsigned long addtime; // Unit in second
unsigned long timeNow;
unsigned long timeFuture;
//const unsigned long PROGMEM sleepTime = 1800000; // Unit in microsecond // 30*60*1000 = 1800000
const unsigned long PROGMEM sleepTime = 120000; // Unit in microsecond // 2*60*1000 = 120000


SoftwareSerial mySerial(pin_rx, pin_tx);
SIMcore sim7020(&mySerial, pin_k, pin_sleep);
DS3231 Clock;
Sleep sleep;
Battery battery(3400, 4200, A0); 

void RTCread(String &str) {
  bool Century = false;
  char DateStr[14];
  bool h12;
  bool PM;
  byte year; byte month;  byte day;
  byte hour; byte minute; byte second;

  year = Clock.getYear();
  month = Clock.getMonth(Century);
  day = Clock.getDate();
  hour = Clock.getHour(h12, PM);
  minute = Clock.getMinute();
  second = Clock.getSecond();

  memset(DateStr, 0, 14);
  sprintf(DateStr, "20%02d%02d%02d%02d%02d%02d",
          year,
          month,
          day,
          hour,
          minute,
          second);
  str = DateStr;
}

/* Upload format
 *  [TimeStamp, Depth, Length, Batery level]
 *  [Time     , arg1   arg2  , arg3        ]
*/  
bool upload(const char* time, float arg1, float arg2, int arg3) {  
  char pathBuffer[150];
  char url[72];
  char id[5];
  strcpy_P(url,(char*) pgm_read_word(&(TextTable[0])));
  strcpy_P(id, (char*) pgm_read_word(&(TextTable[1])));  
  sprintf(pathBuffer,"%s&t=%s&id=%s&d=%s&f1=%s&f2=%s",
          url, time, id, String(arg1).c_str(), String(arg2).c_str(), String(arg3).c_str()); 
  Serial.println(F("-----------The path is here!-----------------"));
  Serial.println(pathBuffer);
      
  sim7020.init();
  delay(2000);
  sim7020.checkNetwork(12);
    if (!sim7020.isNetworkReady()) {
      sim7020.turnOFF();
      return false;
    }
  sim7020.httpGet("http://ec2-54-175-179-28.compute-1.amazonaws.com", pathBuffer);
  sim7020.turnOFF();
  return true;
}

int senseBatteryLevel(){
    digitalWrite(pin_battery_activation, LOW);
    battery.begin(3300, 1.47, &sigmoidal);
    float value = 0;
    uint8_t sampling_num = 10;
    for (uint8_t i=0; i<sampling_num; i++) {
        float sample_val = battery.level();
        value += sample_val;
    }
    float avg = value/sampling_num;
    digitalWrite(pin_battery_activation, HIGH);
    return avg;
}

float sampling() {
  float duration; // the roundtrip duration of the signal in us
  /* Turn off the trigger
  */
  digitalWrite(pinTrigger, LOW); 
  delayMicroseconds(5);
  noInterrupts(); // disable interrupts as they might interfere with the measurement

  /* Turn on the trigger  
   */
  digitalWrite(pinTrigger, HIGH);
  delayMicroseconds(20); // wait for 10us (module sends signal only, if trigger had a HIGH signal for at least 10 us)
  digitalWrite(pinTrigger, LOW); // module sends signal now

  // Receive a HIGH signal on the echo pin
  duration = pulseIn(pinEcho, HIGH); 
  interrupts(); 
                                                                          
  // Convert roundtrip duration to distance
  float durationOneWay = duration / 2; // divided by two, since duration is a roundtrip signal
  float distance = durationOneWay * ((331.4 + 0.606 * 25)/10000); // Unit in cm
   
   delay(500); // wait for 500ms
   return distance;
}


void BubbleSort(float a[], uint32_t size) {
    for(uint32_t ii = 0; ii <(size-1); ii ++) {
        for(uint32_t jj = 0; jj <(size-(ii+1)); jj++) {
                if(a[jj] > a[jj+1]) {
                    float temp = a[jj];
                    a[jj] = a[jj+1];
                    a[jj+1] = temp;
                }
        }
    }
}

float pipeLgMeasurement() {
  // Take a measurement
  //
  Serial.println(F("Start to measure the total length of pipeline......"));
  float dist;
  uint8_t aa;
  for ( aa = 0; aa < samplingNum; aa++) {
    dist = sampling();    
    sampleArray[aa] = dist;
  }
  
  // Find a median in the sampling array 
  //    
  BubbleSort(sampleArray,samplingNum);
  float pipeLgMedian = sampleArray[samplingNum/2];
  sampleArray[samplingNum] = 0.0;  
  return pipeLgMedian;  
}

float * periodicMeasurement() {
  static float MeasData[3]; // [Median, Freq, pipeLength]
  // Take periodic measurements every half hour
  //
  Serial.println(F("Start measurement......"));
  uint8_t pp;

  for ( pp = 0; pp < samplingNum; pp++) {
    float dist = sampling();    
    float tempData = pipeLength - dist;

    if (tempData >= minDepth) {
      sampleArray[pp] = tempData;
      fillHistogram(tempData);      
    } else if (tempData < minDepth) {      
      sampleArray[pp] = 0;
      fillHistogram(tempData); 
    }
    
    Serial.print(F("Sampling Number: ")); Serial.println(pp);
    delay(500);
    Serial.print(F("tempData: ")); Serial.println(tempData);
    delay(500);    
  }  

  // Find a median in the sampling array 
  //    
  BubbleSort(sampleArray,samplingNum);
  MeasData[0] = sampleArray[samplingNum/2];

  // Find a frequency distribution in the sampling array

  MeasData[1] = (histogramIntv[0] + histogramIntv[1])/samplingNum;
  dumpHistogram();

  //Serial.print(F("MeasData[1]: ")); Serial.println(MeasData[1]);
  delay(500);  
  cleanHistogram();
  return MeasData;  
}

void fillHistogram(float sampVal) {
    //Serial.print("gal: ");Serial.print(gal);Serial.println();
    if (sampVal < 0.5) {
        histogramIntv[0] += 1;
    }
    else if (sampVal < 1.0) {
        histogramIntv[1]  += 1;
    }
    else if (sampVal < 1.5) {
        histogramIntv[2] += 1;
    }
    else if (sampVal < 2.0) {
        histogramIntv[3] += 1;
    }
    else if (sampVal < 2.5) {
        histogramIntv[4] += 1;
    }
    else if (sampVal < 3) {
        histogramIntv[5] += 1;
    }   
    else {
        histogramIntv[6] += 1;
    }
}

void cleanHistogram() {
    for (int j = 0; j<7; j++) {
        histogramIntv[j] = 0;
    }
}

void dumpHistogram() {
    for (int j = 0; j<7; j++) {
        Serial.print("index: ");Serial.print(j);Serial.print(" ");
        Serial.println(histogramIntv[j]);
        delay(1);
    }
    Serial.println();
}


int scenarioDetect(float dep, float feq) {
  // Scenario detection
  // 
  // Mode 1 (wait for 180 min)
  // Mode 2 (wait for 10  min) 
  // Mode 3 (wait for 05  min)
  int modeNum;  
  if ( dep > 0.0 && feq <= freqThreshold && dep < officalThreshold) {
    modeNum = 2;
  } else if ( dep > 0.0 && feq <= freqThreshold && dep >= officalThreshold) {
    modeNum = 3;
  } else {
    modeNum = 1;
  }
  return modeNum;
}

void setup() {
  pinMode(pin_battery_activation, OUTPUT);
  digitalWrite(pin_battery_activation, HIGH);
  
  /* Start the serial interface
  */
  Serial.begin(19200); // open serial connection to print out distance values
  while (!Serial) {
    yield(); // wait for serial port to connect. Needed for native USB port only
  }
  
  /* Enable battery sense
  */
  senseBatteryLevel();

  /* Set the ultrasonic sensor
  */
  pinMode(pinTrigger, OUTPUT);
  pinMode(pinEcho, INPUT);
  delay(1000);
}

void loop() {
  /* Start to measure the total length of pipeline
  */
  while (timeCount == -1) {
    pipeLength = pipeLgMeasurement();
    Serial.print(F("Pipe length: "));
    Serial.print(pipeLength);
    Serial.println(F(" cm"));
    Serial.println(F("----------------------------"));

    BatLev = senseBatteryLevel();
    Serial.print(F("Battery level: "));
    Serial.print(BatLev);
    Serial.println(F(" %"));
    
    RTCread(tS);
    Serial.print(F("TimeStamp = "));
    Serial.println(tS);
    delay(1000);    
    //int timeStamp = atoi(tS.c_str());
    //Serial.println(timeStamp);

    /* Upload format
     *  [TimeStamp, Depth, Length, Batery level]
     *  [Time     , arg1   arg2  , arg3        ]
    */ 
    
        
    if (upload(tS.c_str(), 0.0, pipeLength, BatLev)) {
        Serial.println(F("Upload completed."));
    } else {
        Serial.println(F("Upload failure."));
    } 
    delay(1000);
    
    timeCount ++;
    break;
  }
   
  float *GetMeasData = periodicMeasurement(); // GetMeasData = [Median, Freq]
  
  Serial.print(F("Median: "));
  Serial.println(GetMeasData[0]);
  Serial.print(F("Freq: "));
  Serial.println(GetMeasData[1]);
  
  int modeNum = scenarioDetect(GetMeasData[0], GetMeasData[1]);
  Serial.print(F("Mode number: "));
  Serial.println(modeNum);
  Serial.print(F("Time count: "));
  Serial.println(timeCount);

  // Mode 1 (wait for 180 min) : modeNum = 1
  // Mode 2 (wait for 10  min) : modeNum = 2 
  // Mode 3 (wait for 05  min) : modeNum = 3
  if (modeNum == 1 && timeCount < timeCountThreshold ) {
      timeCount ++;
      Serial.println(F("-----It is time to sleep.-----"));
      delay(1000);
      sleep.pwrDownMode(); // Set sleep mode
      sleep.sleepDelay(sleepTime);  // Sleep for: sleepTime // Unit in microsecond
      
  } else if (modeNum == 1 && timeCount == timeCountThreshold) {
      timeCount = 0;
      Serial.println(F("No flood."));
      Serial.println(F("Communication mode: 1"));
      Serial.println(F("----------------------------"));
      Serial.println(F("Sending a text message"));
      Serial.print(F("Water depth: "));
      Serial.print(GetMeasData[0]);
      Serial.println(F(" cm"));
      Serial.println(F("----------------------------"));       
  
      BatLev = senseBatteryLevel();
      Serial.print(F("Battery level: "));
      Serial.print(BatLev);
      Serial.println(F(" %"));
      
      RTCread(tS);
      Serial.print(F("TimeStamp = "));
      Serial.println(tS);

      //int timeStamp = atoi(tS.c_str()); 
        if (upload(tS.c_str(), 0.0, 0.0, BatLev)) {
          Serial.println(F("Upload completed."));
        } else {
          Serial.println(F("Upload failure."));
        }     
       
      } else if (modeNum == 2) {      
      timeCount = 0;
      Serial.println(F("Flooding."));
      Serial.println(F("Communication mode: 2"));
      Serial.println(F("----------------------------"));
      Serial.print(F("Water depth: "));
      Serial.print(GetMeasData[0]);
      Serial.println(F(" cm"));

      BatLev = senseBatteryLevel();
      Serial.print(F("Battery level: "));
      Serial.print(BatLev);
      Serial.println(F(" %"));
      
      RTCread(tS);
      Serial.print(F("TimeStamp = "));
      Serial.println(tS);

      //int timeStamp = atoi(tS.c_str());

      if (upload(tS.c_str(), GetMeasData[0], 0.0, BatLev)) {
          Serial.println(F("Upload completed."));
      } else {
          Serial.println(F("Upload failure."));
      }
      
      /* Setting the waiting time for proMini
      */
      addtime    = 600*1000; // Unit in second
      sleep.pwrDownMode(); // Set sleep mode
      sleep.sleepDelay(addtime);  // Sleep for: sleepTime // Unit in microsecond
            
      } else if (modeNum == 3) {
      timeCount = 0;
      Serial.println(F("Flooding."));
      Serial.println(F("Communication mode: 3"));
      Serial.println(F("----------------------------"));
      Serial.print(F("Water depth: "));
      Serial.print(GetMeasData[0]);
      Serial.println(F(" cm"));
      
      BatLev = senseBatteryLevel();
      Serial.print(F("Battery level: "));
      Serial.print(BatLev);
      Serial.println(F(" %"));
      
      RTCread(tS);
      Serial.print(F("TimeStamp = "));
      Serial.println(tS);

      //int timeStamp = atoi(tS.c_str());   
      if (upload(tS.c_str(), GetMeasData[0], 0.0, BatLev)) {
          Serial.println(F("Upload completed."));
      } else {
          Serial.println(F("Upload failure."));
      }      

      // Setting the waiting time for proMini
      //
      addtime    = 300*1000; // Unit in second
      sleep.pwrDownMode(); // Set sleep mode
      sleep.sleepDelay(addtime);  // Sleep for: sleepTime // Unit in microsecond
      } 
}
