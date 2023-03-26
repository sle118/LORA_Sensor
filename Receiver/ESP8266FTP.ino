/**
 * This sketch reads input from a MAX1820 temperature sensor then saves it to a file
 * which is then uploaded to an FTP server. Once a week, the data file will be renamed
 * and uploaded. This creates a new file for every week's worth of data.
 * 
 * Other's sketches that made this possible include:
 *   SurferTim's FTP
 *   Michael Margolis, Tom Igoe, and Ivan Grokhotkov's FTP script
 *   
 * 
 * Posted 09 May 2017 by Patrick Duensing (fryguy128)
  */
#include <ESP8266WiFi.h>
#include <FS.h>
#include <WiFiUdp.h>
#include <NTPClient.h>   // NTPClient by Fabrice Weinberg aviable in arduino lib manager

//Change these variables to meet your needs
//#############################################
//Wifi

const char* ssid = "SSID";
const char* password_wifi = "password";

//FTP stuff
const char* host = "ftp.xxxxxx.com";
const char* userName = "xxxxxxxxxxx";
const char* password = "xxxxxxxxx";

//############################################
//DO NOT CHANGE THESE UNLESS YOU KNOW WHAT YOU'RE DOING
//FTP buffers
char outBuf[128];
char outCount;
//WiFi Clients
WiFiClient client;
WiFiClient dclient;

//NTP Stuff
const long utcOffsetInSeconds = -14400; // ossfet from UTC in segond  3600 = 1 hour

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);


//LED
int led = 2;


//###########################################
//Move into setup()
void setup() {
  // Serial setup
  Serial.begin(115200);
  delay(100);

  //Wifi Setup
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid,password_wifi);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //SPIFF setup
  SPIFFS.begin();

 
  timeClient.begin(); 

  //led Pin setup
  pinMode(led, OUTPUT);

}

//#########################################
//Move into loop()
void loop() {

  if (Serial.available()) {
  String incomingString = Serial.readStringUntil('@');  // '\n'
  
  //This is the filename where your newest data will be stored
  String fileNameData = "/donnes.txt";
  //This is the filename that contains a single number used to create a new file
  String textVar = "/number.txt";
  //This string will hold the name of the new file to be created at the end of a week
  String newFileName = "";

  //Turn the led on for the entire loop
  digitalWrite(led, HIGH);
  
  //Get the time from the NTP server
//  setSyncProvider(getNtpTime);
  timeClient.update();
  Serial.println("1) NTP GOTTEN");
  time_t epochTime = timeClient.getEpochTime();  //format date and time in human format
  struct tm *ptm = gmtime ((time_t *)&epochTime); 
  int monthDay = ptm->tm_mday;
  int currentMonth = ptm->tm_mon+1;
//  String currentMonthName = months[currentMonth-1];
  int currentYear = ptm->tm_year+1900;
   String currentDate = String(currentYear) + "-" + String(currentMonth)+ "-" + String(monthDay)  ;

  //This runs if the time was found. If it wasn't, the date would be set to sometime in 1970.

    //Used to determine if a week has passed
    int count = 0;

    //Opens the data file to read
    File j = SPIFFS.open(fileNameData, "r");
    if (!j) {
      Serial.println("file open failed");
    }
    else {
      //Finds out how many records there are in the main data file
      while (j.available()) {
        count++;
        j.readStringUntil('\n');
      }
      j.close();
    }
    
      Serial.println(count);
      if (count > 1007) {                                        // 1008 records is equal to a week of recording every 10 minutes
        File h = SPIFFS.open(textVar, "r");                      // Open the file holding the number to read
        if (!h) {
          Serial.println("file open failed");
        }
        else {
            int inputVal = h.parseInt();                         // Read the integer from the number file
            //Close the number file
            h.close();
            Serial.println(inputVal);                            // Create the new file name
            newFileName = "/data" + String(inputVal);
            newFileName += ".txt";
            Serial.println("newFileName = " + newFileName);
            SPIFFS.rename(fileNameData, newFileName);           // Rename the data file
            File x = SPIFFS.open(textVar, "w");                 // Reopen the number file in write mode
            if(!x){
              Serial.println("file open failed");
            }
            else{
              inputVal++;                                      // Increment the value read in from the number file
              x.println(inputVal);                             // Write the number to the number file
              x.close();                                       // Closes the number file
            }
        }
      }
    File f = SPIFFS.open(fileNameData, "a");                   // Open the data file in append mode
    if (!f) {
      Serial.println("file open failed");
    }
    else {
      Serial.println("3) FILE OPENED");
      
      f.print(incomingString); 
      f.print(",");
      f.print(currentDate);
      f.print(",");
      f.println(timeClient.getFormattedTime()); 
      f.close();                                            //Close the data file, writing is done      
      Serial.println("5) FILE PRINTED AND CLOSED");

      if (doFTP(fileNameData)) Serial.println(F("FTP Data OK"));            //Upload the data file using the doFTP function
      else Serial.println(F("FTP Data FAIL"));
      
      if (newFileName.length() > 0) {                                       //If it has been a week, upload the new data file using the doFTP function
        if (doFTP(newFileName))  Serial.println(F("FTP DataNum OK"));
        else Serial.println(F("FTP DataNum FAIL"));
      }
      Serial.println("6) FTP DONE");
    }

  digitalWrite(led, LOW);                                 //Turn off the LED

  Serial.println("3/7) GOING TO SLEEP");
  Serial.flush();
   while(Serial.available() > 0) {char t = Serial.read(); } // flush serial buffer
   if (Serial.available()) {
    Serial.println("BOG");
    Serial.println(Serial.available()); }  // do noting jsut clear serial buffer
  }
}

//#######################################
/*
   This is the function where the magic happens.
   all credit goes to SurferTim. This will open
   and write the file to your FTP server
*/
byte doFTP(String fileName)
{
  File fh = SPIFFS.open(fileName, "r");
  if (!fh) {
    Serial.println("file open failed");
    return 0;
  }
  if (client.connect(host, 21)) {
    Serial.println(F("Command connected"));
  }
  else {
    fh.close();
    Serial.println(F("Command connection failed"));
    return 0;
  }

  if (!eRcv()) return 0;

  client.print("USER ");
  client.println(userName);

  if (!eRcv()) return 0;

  client.print("PASS ");
  client.println(password);

  if (!eRcv()) return 0;

  client.println("SYST");

  if (!eRcv()) return 0;

  client.println("Type I");

  if (!eRcv()) return 0;

  client.println("PASV");

  if (!eRcv()) return 0;

  char *tStr = strtok(outBuf, "(,");
  int array_pasv[6];
  for ( int i = 0; i < 6; i++) {
    tStr = strtok(NULL, "(,");
    array_pasv[i] = atoi(tStr);
    if (tStr == NULL)
    {
      Serial.println(F("Bad PASV Answer"));
      return 0;
    }
  }

  unsigned int hiPort, loPort;
  hiPort = array_pasv[4] << 8;
  loPort = array_pasv[5] & 255;
  Serial.print(F("Data port: "));
  hiPort = hiPort | loPort;
  Serial.println(hiPort);
  if (dclient.connect(host, hiPort)) {
    Serial.println("Data connected");
  }
  else {
    Serial.println("Data connection failed");
    client.stop();
    fh.close();
    return 0;
  }

  client.print("STOR ");
  client.println(fileName);
  if (!eRcv())
  {
    dclient.stop();
    return 0;
  }
  Serial.println(F("Writing"));

  byte clientBuf[64];
  int clientCount = 0;

  while (fh.available())
  {
    clientBuf[clientCount] = fh.read();
    clientCount++;

    if (clientCount > 63)
    {
      dclient.write((const uint8_t *)clientBuf, 64);
      clientCount = 0;
    }
  }
  if (clientCount > 0) dclient.write((const uint8_t *)clientBuf, clientCount);

  dclient.stop();
  Serial.println(F("Data disconnected"));
  client.println();
  if (!eRcv()) return 0;

  client.println("QUIT");

  if (!eRcv()) return 0;

  client.stop();
  Serial.println(F("Command disconnected"));

  fh.close();
  Serial.println(F("File closed"));
  return 1;
}

byte eRcv()
{
  byte respCode;
  byte thisByte;

  while (!client.available()) delay(1);

  respCode = client.peek();

  outCount = 0;

  while (client.available())
  {
    thisByte = client.read();
 //   Serial.write(thisByte);

    if (outCount < 127)
    {
      outBuf[outCount] = thisByte;
      outCount++;
      outBuf[outCount] = 0;
    }
  }

  if (respCode >= '4')
  {
    efail();
    return 0;
  }

  return 1;
}


void efail()
{
  byte thisByte = 0;

  client.println(F("QUIT"));

  while (!client.available()) delay(1);

  while (client.available())
  {
    thisByte = client.read();
    Serial.write(thisByte);
  }

  client.stop();
  Serial.println(F("Command disconnected"));
}

//#################################################################
