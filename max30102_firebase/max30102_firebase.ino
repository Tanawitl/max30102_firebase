//#include "FirebaseESP8266.h"
//#include <ESP8266WiFi.h>
#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#endif

#include <time.h>
//#include <WiFi.h>
#include <Wire.h>
//#include <Adafruit_MLX90614.h>
#include "MAX30105.h" //sparkfun MAX3010X library
#include "heartRate.h"

//#include "U8g2lib.h"
//#include "Wire.h"
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);


String device = "device001";
#define SSID        "bank_2G"
#define PASSWORD    "bank10086"

#define FIREBASE_HOST "esp1-146ec-default-rtdb.firebaseio.com" //project-42dbf-default-rtdb.asia-southeast1.firebasedatabase.app
#define FIREBASE_AUTH "TUnpCwP8iv2XcKfDIvW8Rnf3eIY9S3MPNrvdbDjT"
//#define I2C_SPEED_STANDARD        1000
MAX30105 particleSensor;


//Adafruit_MLX90614 mlx = Adafruit_MLX90614();

double avered = 0; double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 50;//calculate SpO2 by this sampling interval 100

double ESpO2 = 95.0;//initial value of estimated SpO2
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 88.0 //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 5 //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 3000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 80.0

TaskHandle_t Task0;
//TaskHandle_t Task1;
//TaskHandle_t Task2;
//TaskHandle_t Task3;

const byte RATE_SIZE = 5; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
float temp;
uint32_t ir, red , green;
#define USEFIFO
int r=0;


int selec_dsp = 0;

FirebaseData firebaseData;

//const String month_name[12] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
//const String day_name[7] = {"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
//int timezone = 7 * 3600; //ตั้งค่า TimeZone ตามเวลาประเทศไทย
//int dst = 0;//7 * 3600; //กำหนดค่า Date Swing Time
//String datess,timess;
void setup()
{
  pinMode(13,OUTPUT);
  //digitalWrite(13,1);//buzer off
  Serial.begin(115200);
  //u8g2.begin();
  WiFi.begin(SSID, PASSWORD);
  Serial.printf("WiFi connecting to %s\n",  SSID);
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    //u8g2.clearBuffer();          // clear the internal memory
    //u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font
    //u8g2.drawStr(7,42, "WIFI CONNECTED ...");  // write something to the internal memory
    //u8g2.sendBuffer(); 
    delay(400);
  }
  Serial.printf("\nWiFi connected\nIP : ");
  Serial.println(WiFi.localIP());


  
  Serial.println("Initializing...");

  delay(1000);
  // Initialize sensor
  //u8g2.setBusClock(100000);
  while (!particleSensor.begin(Wire, 100000)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    //u8g2.clearBuffer();          // clear the internal memory
    //u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font
    //u8g2.drawStr(7,42, "MAX30102 was not found.");  // write something to the internal memory
    // u8g2.sendBuffer(); 
    //delay(1000);
    //Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    //u8g2.clearBuffer();          // clear the internal memory
    //u8g2.setFont(u8g2_font_ncenB10_tr); // choose a suitable font
    //u8g2.drawStr(7,42, "");  // write something to the internal memory
    //u8g2.sendBuffer(); 
    //while (1);
    delay(500);
    //ESP.restart(); 
  }

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  //particleSensor.enableDIETEMPRDY();
  //mlx.begin();
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

 // xTaskCreatePinnedToCore(loop_T3,"Task3",4096,NULL,3,NULL,1);
 // delay(1000);
 // xTaskCreatePinnedToCore(loop_T2,"Task2",4096,NULL,2,NULL,0);
 // delay(500);
 // xTaskCreatePinnedToCore(loop_T1,"Task1",10000,NULL,1,NULL,1);
 // delay(500);
  

 /* configTime(timezone, dst, "pool.ntp.org", "time.nist.gov"); //ดึงเวลาจาก Server
  Serial.println("\nLoading time");
  while (!time(nullptr)) 
  {
     Serial.print("*");
     delay(1000);
  }
  
  Serial.println("");
  time_t now = time(nullptr);
  struct tm* p_tm = localtime(&now);
  delay(1000);

 while(String(p_tm->tm_year+ 1900).toInt() < 2000)
   {
      time_t now = time(nullptr);
      struct tm* p_tm = localtime(&now);
      Serial.println("Time Error!");
      delay(1000);
   }*/


  //xTaskCreatePinnedToCore(loop_T0,"Task0",10000,NULL,0,NULL,0);
  digitalWrite(23,0);//buzer on
  delay(100);
  digitalWrite(23,1);//buzer off
  delay(100);
}


/*void loop_T0(void * parameter) 
{
  //for (;;) 
  //{
     while(1)
     { 
      

      if(selec_dsp==0)
      {
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.firstPage();
          do {
             u8g2.setCursor(0, 13);u8g2.print("");
              u8g2.setCursor(0, 29);u8g2.print("");
              u8g2.setCursor(20,38);u8g2.print("Place finger");
              u8g2.setCursor(0, 47);u8g2.print("");
           } while ( u8g2.nextPage() );
        delay(20);
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.firstPage();
          do {
             u8g2.setCursor(0, 13);u8g2.print("");
              u8g2.setCursor(0, 29);u8g2.print("");
              u8g2.setCursor(20,38);u8g2.print("");
              u8g2.setCursor(0, 47);u8g2.print("");
           } while ( u8g2.nextPage() );
        delay(20);
        
      }
      else if(selec_dsp==1)
      {
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.firstPage();
          do {
              u8g2.setCursor(0, 13);u8g2.print("BPM  : "+ String(beatAvg));
              u8g2.setCursor(0, 29);u8g2.print("Oxy  : "+ String(ESpO2,0)+"%");
              u8g2.setCursor(0, 47);u8g2.print("TEMP : "+ String(temp,1)+"'C");
              
           } while ( u8g2.nextPage() );
      }

      else if(selec_dsp==2)
      {
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.firstPage();
          do {
              u8g2.setCursor(0, 13);u8g2.print("BPM  : "+ String(beatAvg));
              u8g2.setCursor(0, 29);u8g2.print("Oxy  : "+ String(ESpO2,0)+"%");
              u8g2.setCursor(0, 47);u8g2.print("TEMP : "+ String(temp,1)+"'C");

              u8g2.setCursor(0, 63);u8g2.print("-- SUCCESS OK --");
           } while ( u8g2.nextPage() );
           
      }
         
     }
  //}

}*/
/*
void loop_T0(void * parameter) 
{
  for (;;) 
  {
    
  }

}

void loop_T2(void * parameter) 
{
  for (;;) 
  {
    read_spo2();
  }

}

void loop_T3(void * parameter) 
{
  for (;;) 
  {
    
  }

}
*/


void loop()
{
  //read_temp();
  //read_bpm();
  //read_spo2();

    //time_online();
    read_bpm();
    read_spo2();
    //read_temp();
    
    //Serial.println("r : "+String(r)+" ir : "+ String(ir)+"   BPM = : "+ String(beatAvg)+" Oxygen = " + String(ESpO2)+" % Temp = " + String(temp));
      
    if(ir >=FINGER_ON)
    {
      selec_dsp = 1;
      Serial.println("r : "+String(r)+" ir : "+ String(ir)+"   BPM = : "+ String(beatAvg)+" Oxygen = " + String(ESpO2));
     
    }
    else
    {
      selec_dsp = 0;
      r=0;
      ir=0;
      beatAvg=0;
      ESpO2=0;
      //temp=0;
      //display.clear();
      //display.print("Place finger",3,2);
      //display.on();
    }

    if(r>=15)
       {
         //Firebase.setString(firebaseData,"/"+device+"/"+"DATE", String(datess));
         //Firebase.setString(firebaseData,"/"+device+"/"+"TIME", String(timess));
         Firebase.setString(firebaseData,"/"+device+"/"+"BPM", String(beatAvg));
         Firebase.setString(firebaseData,"/"+device+"/"+"Oxygen", String(ESpO2));
         //Firebase.setString(firebaseData,"/"+device+"/"+"TEMP", String(temp));
         selec_dsp = 2;
         r=0;
         Serial.println("Send firebase....");
         //display.print("OK...",3,7);
         //display.on();
         //
         digitalWrite(23,0);//buzer on
         delay(1000);
         digitalWrite(23,1);//buzer off
         
         while(ir >=FINGER_ON)
           {
            read_spo2();
           }
       }
    
}





void read_bpm()
{
  long irValue = particleSensor.getIR();
   if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
      r++;
    }
  }

    
 
}


/*void read_temp()
{
  temp = mlx.readObjectTempC();
}*/

void read_spo2()
{
  
  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered

  #ifdef USEFIFO
    particleSensor.check(); //Check the sensor, read up to 3 samples
  
    while (particleSensor.available()) 
    {//do we have new data
  #ifdef MAX30105
      red = particleSensor.getFIFORed(); //Sparkfun's MAX30105
      ir = particleSensor.getFIFOIR();  //Sparkfun's MAX30105
  #else
      red = particleSensor.getFIFOIR(); //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
      ir = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
  #endif

    
    
    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level
    
      
    if (ir < FINGER_ON) ESpO2 = MINIMUM_SPO2; //indicator for finger detached
        
       

    if ((i % Num) == 0) 
    {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      // Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100; //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      if(ESpO2>=100){ESpO2=100;}
      //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      sumredrms = 0.0; sumirrms = 0.0; i = 0;
      break;
    }
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    //Serial.println(SpO2);

    
    
  }
#endif
}

/*void time_online()
{
  time_t now = time(nullptr);
              struct tm* p_tm = localtime(&now);
              String M,D,h,m,s ;
        
        
              if (String(p_tm->tm_mon+1).toInt() < 10)
              {
                M += "0";
                M += String(p_tm->tm_mon+1);
              }
              else
              {
                M = String(p_tm->tm_mon+1);
              }
        
              if (String(p_tm->tm_mday).toInt() < 10)
              {
                D += "0";
                D += String(p_tm->tm_mday);
              }
              else
              {
                D = String(p_tm->tm_mday);
              }
            
              if (String(p_tm->tm_hour).toInt() < 10)
              {
                h += "0";
                h += String(p_tm->tm_hour);
              }
              else
              {
                h = String(p_tm->tm_hour);
              }
        
              if (String(p_tm->tm_min).toInt() < 10)
              {
                m += "0";
                m += String(p_tm->tm_min);
              }
              else
              {
                m = String(p_tm->tm_min);
              }
        
              if (String(p_tm->tm_sec).toInt() < 10)
              {
                s += "0";
                s += String(p_tm->tm_sec);
              }
              else
              {
                s = String(p_tm->tm_sec);
              }
        
              
              datess = D+"_"+M+"_"+String(p_tm->tm_year+ 1900);//+ 2443
              timess = String(h)+":"+String(m)+":"+String(s);
            
              Serial.println(datess + "  "+ timess);

    

}*/
