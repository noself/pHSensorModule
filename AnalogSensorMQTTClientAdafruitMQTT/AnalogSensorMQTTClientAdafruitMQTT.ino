#include "Arduino.h"
#include "Average.h"
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include "mdns.h"
#include <EEPROM.h>


struct eepromStruct {
	char sensorID[20];
	char wifi_ssid[30];
	char wifi_password[20];
	char AIO_SERVER[30];
	unsigned short int  AIO_SERVERPORT;
	char AIO_USERNAME[10];
	char AIO_KEY[10];
	char serverAddress[20];
	float slopeValue;
	float interceptValue;
  char dummy[20];
};

#define EEPROM_read(record, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) {pp[i]=EEPROM.read(offsetof(struct eepromStruct, record)+i);}}
#define EEPROM_write(record, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) {EEPROM.write(offsetof(struct eepromStruct, record)+i, pp[i]);}}

char sensorID[20];
char wifi_ssid[30];
char wifi_password[20];
char AIO_SERVER[30];
unsigned short int AIO_SERVERPORT;
char AIO_USERNAME[10];
char AIO_KEY[10];
char serverAddress[20];
float slopeValue;
float interceptValue;
char dummy[20];



#if 0
char SensorID[16]="08024";
char wifi_ssid[] = "IoThingsWareBus";    //  your network SSID (name)
char wifi_password[] = "07B04U1957S";   // your network password
/************************* Adafruit.io Setup *********************************/
#define AIO_SERVER      "raspberrypi.local"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    ""
#define AIO_KEY         ""
/************ Global State (you don't need to change this!) ******************/
#endif

int status = WL_IDLE_STATUS;
WiFiClient espClient;
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
//Adafruit_MQTT_Client mqtt(&espClient, serverAddress, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
//è stato modificato il codice della classe Adafruit_MQTT_Client per rendere possibile
//runtime l'inserimento del serverAddress (si potrebbe modificare ulteriormente per variare 
//runtime anche la porta). ATTENZIONE QUINDI A COMPILARE QUESTO CODICE SOLO DOVE E'
//PRESENTE QUESTA CLASSE MODIFICATA.
Adafruit_MQTT_Client mqtt(&espClient, serverAddress, 1883, "", "");

/*************************** Sketch Code ************************************/
static int mqttServerDiscovered;
void MQTT_connect();


//#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) {EEPROM.write(address+i, pp[i]); delay(100);}}
//#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) {pp[i]=EEPROM.read(address+i); delay(100);}}

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength+1];   // store the serial command
byte receivedBufferIndex = 0;

#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the sample voltage
int analogBufferIndex = 0;

//#define SlopeValueAddress 0     // (slope of the ph probe)store at the beginning of the EEPROM. The slope is a float number,occupies 4 bytes.
//#define InterceptValueAddress (SlopeValueAddress+4)
//#define SensorIdAddress (InterceptValueAddress+4)
#define ReadID EEPROM_read(sensorId, SensorID)

float averageVoltage;
boolean enterCalibrationFlag = 0;

#define SensorPin A0
#define VREF 1000.0  //for arduino uno, the ADC reference is the power(AVCC), that is 5000mV






// When an mDNS packet gets parsed this optional callback gets called once per Query.
// See mdns.h for definition of mdns::Answer.
void answerCallback(const mdns::Answer* answer) {
  if (!strcmp(answer->name_buffer, AIO_SERVER) && answer->rrtype == MDNS_TYPE_A)
  {
    Serial.println(answer->rdata_buffer);
    strcpy(serverAddress, answer->rdata_buffer);
    answer->Display();
    mqttServerDiscovered = true;
  }
}

// Initialise MDns.
// If you don't want the optional callbacks, just provide a NULL pointer as the callback.
//mdns::MDns my_mdns(packetCallback, queryCallback, answerCallback);
mdns::MDns my_mdns(NULL, NULL, answerCallback);


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  //WiFi.begin("IoThingsWareBus", "07B04U1957S");
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void upper_string(char s[]) {
   int c = 0;
 
   while (s[c] != '\0') {
      if (s[c] >= 'a' && s[c] <= 'z') {
         s[c] = s[c] - 32;
      }
      c++;
   }
}


boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while (Serial.available()>0) 
  {   
    if (millis() - receivedTimeOut > 1000U) 
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer,0,(ReceivedBufferLength+1));
    }
    receivedTimeOut = millis();
    receivedChar = (char)Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex==ReceivedBufferLength){
    receivedBufferIndex = 0;
    upper_string(receivedBuffer);
    return true;
    }
    else{
      receivedBuffer[receivedBufferIndex] = receivedChar;
      receivedBufferIndex++;
    }
  }
  return false;
}

byte uartParse()
{
  byte modeIndex = 0;
  if(strstr(receivedBuffer, "CALIBRATION") != NULL) 
      modeIndex = 1;
  else if(strstr(receivedBuffer, "QUIT") != NULL) 
      modeIndex = 5;
  else if(strstr(receivedBuffer, "COMMIT") != NULL) 
      modeIndex = 4;
  else if(strstr(receivedBuffer, "ACID:") != NULL)   
      modeIndex = 2;  
  else if(strstr(receivedBuffer, "ALKALI:") != NULL)
      modeIndex = 3;
  return modeIndex;
}

void phCalibration(byte mode)
{
    char *receivedBufferPtr;
    static byte acidCalibrationFinish = 0, alkaliCalibrationFinish = 0;
    static float acidValue,alkaliValue;
    static float acidVoltage,alkaliVoltage;
    float acidValueTemp,alkaliValueTemp,newSlopeValue,newInterceptValue;
    switch(mode)
    {
      case 0:
      if(enterCalibrationFlag)
         Serial.println(F("Command Error"));
      break;
      
      case 1:
      receivedBufferPtr=strstr(receivedBuffer, "CALIBRATION");
      enterCalibrationFlag = 1;
      acidCalibrationFinish = 0;
      alkaliCalibrationFinish = 0;
      Serial.println(F("Enter Calibration Mode"));
      EEPROM_read(slopeValue, slopeValue);     // After calibration, the new slope and intercept should be read ,to update current value.
      Serial.print("slopeValue: ");
      Serial.println(slopeValue);
      EEPROM_read(interceptValue, interceptValue);
      Serial.print("interceptValue; ");
      Serial.println(interceptValue);

      break;
     
      case 2:
      if(enterCalibrationFlag)
      {
          receivedBufferPtr=strstr(receivedBuffer, "ACID:");
          receivedBufferPtr+=strlen("ACID:");
          acidValueTemp = strtod(receivedBufferPtr,NULL);
          if((acidValueTemp>3)&&(acidValueTemp<5))        //typical ph value of acid standand buffer solution should be 4.00
          {
             acidValue = acidValueTemp;
             acidVoltage = averageVoltage;        // mV -> V
             acidCalibrationFinish = 1;
             Serial.println(F("Acid Calibration Successful"));
           }else {
             acidCalibrationFinish = 0;
             Serial.println(F("Acid Value Error"));
           }
      }
      break;
 
       case 3:
       if(enterCalibrationFlag)
       {
           receivedBufferPtr=strstr(receivedBuffer, "ALKALI:");
           receivedBufferPtr+=strlen("ALKALI:");
           alkaliValueTemp = strtod(receivedBufferPtr,NULL);
           if((alkaliValueTemp>8)&&(alkaliValueTemp<11))        //typical ph value of alkali standand buffer solution should be 9.18 or 10.01
           {
                 alkaliValue = alkaliValueTemp;
                 alkaliVoltage = averageVoltage;
                 alkaliCalibrationFinish = 1;
                 Serial.println(F("Alkali Calibration Successful"));
            }else{
               alkaliCalibrationFinish = 0;
               Serial.println(F("Alkali Value Error"));
            }
       }
       break;

        case 4:
        if(enterCalibrationFlag)
        {
            if(acidCalibrationFinish && alkaliCalibrationFinish)
            {
              newSlopeValue = (acidValue-alkaliValue)/(acidVoltage - alkaliVoltage);
              EEPROM_write(slopeValue, newSlopeValue);
              newInterceptValue = acidValue - (newSlopeValue*acidVoltage);
              EEPROM_write(interceptValue, newInterceptValue);
              EEPROM.commit();
              slopeValue=newSlopeValue;
              interceptValue=newInterceptValue;
              Serial.print(F("Calibration Successful"));
            }
            else Serial.print(F("Calibration Failed"));       
            Serial.println(F(",Commit and Exit Calibration Mode"));
            acidCalibrationFinish = 0;
            alkaliCalibrationFinish = 0;
            enterCalibrationFlag = 0;
        }
        break;
        
        case 5:
        if(enterCalibrationFlag)
        { 
            Serial.println(F(",Quit Calibration Mode"));
            acidCalibrationFinish = 0;
            alkaliCalibrationFinish = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}

int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
    bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
    for (i = 0; i < iFilterLen - j - 1; i++) 
          {
      if (bTab[i] > bTab[i + 1]) 
            {
    bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
    bTab[i + 1] = bTemp;
       }
    }
      }
      if ((iFilterLen & 1) > 0)
  bTemp = bTab[(iFilterLen - 1) / 2];
      else
  bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}

void readCharacteristicValues()
{
    EEPROM_read(sensorID, sensorID);
    Serial.println(sensorID);
    EEPROM_read(wifi_ssid, wifi_ssid);
    Serial.println(wifi_ssid);
    EEPROM_read(wifi_password, wifi_password);
    Serial.println(wifi_password);
    EEPROM_read(AIO_SERVER, AIO_SERVER);
    Serial.println(AIO_SERVER);
    EEPROM_read(AIO_SERVERPORT, AIO_SERVERPORT);
    Serial.println(AIO_SERVERPORT);
    EEPROM_read(AIO_USERNAME, AIO_USERNAME);
    Serial.println(AIO_USERNAME);
    EEPROM_read(AIO_KEY, AIO_KEY);
    Serial.println(AIO_KEY);
    EEPROM_read(serverAddress, serverAddress);
    Serial.println(serverAddress);
    EEPROM_read(slopeValue, slopeValue);
    Serial.println(slopeValue);
    EEPROM_read(interceptValue, interceptValue);
    Serial.println(interceptValue);
}

void setup() {
  mqttServerDiscovered = false;
  EEPROM.begin(512);
  Serial.begin(115200);
  delay(100);
  readCharacteristicValues(); //read the slope and intercept of the ph probe
  setup_wifi();
  // Query for all host information for a paticular name. ("raspberrypi.local" in this case.)
  my_mdns.Clear();
  struct mdns::Query query_server;
  strncpy(query_server.qname_buffer, AIO_SERVER, MAX_MDNS_NAME_LEN);
  query_server.qtype = MDNS_TYPE_A;
  query_server.qclass = 1;    // "INternet"
  query_server.unicast_response = 0;
  my_mdns.AddQuery(query_server);
  my_mdns.Send();
}


void MQTT_connect() {
  int8_t ret;
  // Exit if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(2000);  // wait 2 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      ESP.restart();  //manually reset after serial flashing, the latest work around
    }
  }
  Serial.println("MQTT Connected!");
}

/*
void getValue(byte i)
{
  float tempC = sensors.getTempC(SensorID[i].bin);
  if (tempC == -127.00)
  {
    Serial.print("Error getting temperature  ");
  }
  else
  {
    Serial.print("C: ");
    Serial.println(tempC);
    //   Serial.print(" F: ");
    //   Serial.print(DallasTemperature::toFahrenheit(tempC));
    Adafruit_MQTT_Publish obj = Adafruit_MQTT_Publish(&mqtt, (const char *)SensorID[i].hex);
//    Adafruit_MQTT_Publish obj = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME SensorID[i].hex);
//    Adafruit_MQTT_Publish obj = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "jghj");
    if (! obj.publish(String(tempC).c_str())) {
      Serial.println(F("Failed"));
      ESP.restart();  //manually reset after serial flashing, the latest work around
    } else {
      Serial.println(F("OK!"));
    }
  }
}
*/




void loop() {
  if (!mqttServerDiscovered)
  {
    my_mdns.Check();
    return;
  }
  // get sensors falue and write on channel 168044
  MQTT_connect();
    if(serialDataAvailable() > 0)
  {
      byte modeIndex = uartParse();
      phCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
  }
  
   static unsigned long sampleTimepoint = millis();
   if(millis()-sampleTimepoint>500U) //40U
   {
     sampleTimepoint = millis();
     analogBuffer[analogBufferIndex] = analogRead(SensorPin);    //read the voltage and store into the buffer,every 40ms
     analogBufferIndex++;
     if(analogBufferIndex == SCOUNT) 
         analogBufferIndex = 0;
     averageVoltage = getMedianNum(analogBuffer,SCOUNT);   // read the stable value by the median filtering algorithm
   }
   
   static unsigned long printTimepoint = millis();
   if(millis()-printTimepoint>3000U) //1000U
   {
     printTimepoint = millis();
     if(enterCalibrationFlag)             // in calibration mode, print the voltage to user, to watch the stability of voltage
     {
       Serial.print("Voltage:");
       Serial.print(averageVoltage/1024.0*VREF);
       Serial.println("mV");
     }else{
     Serial.print("pH:");              // in normal mode, print the ph value to user
     float pHvalue=averageVoltage*slopeValue+interceptValue;
     Serial.println(pHvalue);
     Adafruit_MQTT_Publish obj = Adafruit_MQTT_Publish(&mqtt, (const char *)sensorID);
    if (! obj.publish(String(pHvalue).c_str())) {
      Serial.println(F("Failed"));
      ESP.restart();  //manually reset after serial flashing, the latest work around
    } else {
      Serial.println(F("OK!"));
    }     
     }
   }
//  delay(2000); // Note that the weather station only updates once a minute
}


