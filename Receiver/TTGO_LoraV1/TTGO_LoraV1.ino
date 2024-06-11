/***
 * MIT License
 * 
 * Copyright (c) 2024 Sebastien Leclerc
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ***/

/***
 *
 * Loosely inspired by other people's efforts, like the one below: 
 * https://github.com/StorageB/Google-Sheets-Logging/tree/master?tab=readme-ov-file
 *
 * At the time of writing this code, it was difficult to get the ESP32 ssl/tls stack
 * to reliably POST a request to the google scripts app, so instead the code is 
 * using a GET request with parameters passed in the URL.
 *
 * The base configuration is intended to be used with a TTGO Lora32 V1 board with
 * the corresponding PIN definitions. 
 *
 ***/

/**
 * The below includes are used to align radio information between the transmitter and
 * receivers as well as to define a common data structure to be exchanged between the two
 * for measurements.
 *
 * #include <Definitions.h>
 * #include <Configuration.h>
 *
 * These files reside in a different part of the project directory and must be made accessible
 * to this sketch. To achieve this, we use symbolic links (symlinks). If the symlinks are not
 * already set up or are not properly propagated after cloning the repository, follow these
 * steps to create them manually from the root of the project:
 *
 * For Windows:
 * 1. Open Command Prompt as Administrator.
 * 2. Navigate to the root of the project (where you cloned the repository).
 * 3. Execute the following commands:
 *    mklink Receiver\TTGO_LoraV1\Definitions.h transmitter\Sensor_firmware\Feather9x_TX\Definitions.h
 *    mklink Receiver\TTGO_LoraV1\Configuration.h transmitter\Sensor_firmware\Feather9x_TX\Configuration.h
 *
 * For Linux or macOS:
 * 1. Open Terminal.
 * 2. Navigate to the root of the project (where you cloned the repository).
 * 3. Execute the following commands:
 *    ln -s ../transmitter/Sensor_firmware/Feather9x_TX/Definitions.h Receiver/TTGO_LoraV1/Definitions.h
 *    ln -s ../transmitter/Sensor_firmware/Feather9x_TX/Configuration.h Receiver/TTGO_LoraV1/Configuration.h
 *
 * Ensure that these commands are executed at the project root level to correctly set up the paths.
 */


/**************************
 *
 * WARNING:
 * For the TTGO Lora-V1 Oled board, you HAVE to stick to the esp32 boards with version 2.x.x as version 3.x.x
 * dropped support for boards with a 26MHz crystal. Without proper support, wifi will NOT work as the 
 * system clock configuration will assume a frequency of 40MHz during build.
 *
 **************************/
 


#define WM_DEBUG_LEVEL WM_DEBUG_VERBOSE
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <Wire.h>
#include "Definitions.h"
#include "Configuration.h"
#if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
#include <NetworkClientSecure.h>
#else
#include <WiFiClientSecure.h>
#endif
#include <LoraEncoder.h>

JsonDocument json;
#define SERIAL_CONSOLE_SPEED 115200

// Constants for configuring the OLED display
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C  // 0x3D for 128x64, 0x3C for 128x32

// Constants for LoRa module
#define RFM95_CS 18
#define RFM95_RST 14
#define RFM95_INT 26
#define SX1278_MISO 19
#define SX1278_MOSI 27
#define SX1278_SCLK 5
#define SX1278_CS 18
#define SX1278_DIO 26
#define SX1278_RST 14

#define LED 13  // Blinky on receipt

// Debounce and portal control
#define TRIGGER_PIN 0
const unsigned long debounceDelay = 50;
const unsigned long minInterval = 5000;
const char * apPassword = "loragateway";

// Default certificate for NetworkClientSecure/WifiClientSecure
const char* root_ca = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFVzCCAz+gAwIBAgINAgPlk28xsBNJiGuiFzANBgkqhkiG9w0BAQwFADBHMQsw
CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU
MBIGA1UEAxMLR1RTIFJvb3QgUjEwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw
MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp
Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjEwggIiMA0GCSqGSIb3DQEBAQUA
A4ICDwAwggIKAoICAQC2EQKLHuOhd5s73L+UPreVp0A8of2C+X0yBoJx9vaMf/vo
27xqLpeXo4xL+Sv2sfnOhB2x+cWX3u+58qPpvBKJXqeqUqv4IyfLpLGcY9vXmX7w
Cl7raKb0xlpHDU0QM+NOsROjyBhsS+z8CZDfnWQpJSMHobTSPS5g4M/SCYe7zUjw
TcLCeoiKu7rPWRnWr4+wB7CeMfGCwcDfLqZtbBkOtdh+JhpFAz2weaSUKK0Pfybl
qAj+lug8aJRT7oM6iCsVlgmy4HqMLnXWnOunVmSPlk9orj2XwoSPwLxAwAtcvfaH
szVsrBhQf4TgTM2S0yDpM7xSma8ytSmzJSq0SPly4cpk9+aCEI3oncKKiPo4Zor8
Y/kB+Xj9e1x3+naH+uzfsQ55lVe0vSbv1gHR6xYKu44LtcXFilWr06zqkUspzBmk
MiVOKvFlRNACzqrOSbTqn3yDsEB750Orp2yjj32JgfpMpf/VjsPOS+C12LOORc92
wO1AK/1TD7Cn1TsNsYqiA94xrcx36m97PtbfkSIS5r762DL8EGMUUXLeXdYWk70p
aDPvOmbsB4om3xPXV2V4J95eSRQAogB/mqghtqmxlbCluQ0WEdrHbEg8QOB+DVrN
VjzRlwW5y0vtOUucxD/SVRNuJLDWcfr0wbrM7Rv1/oFB2ACYPTrIrnqYNxgFlQID
AQABo0IwQDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4E
FgQU5K8rJnEaK0gnhS9SZizv8IkTcT4wDQYJKoZIhvcNAQEMBQADggIBAJ+qQibb
C5u+/x6Wki4+omVKapi6Ist9wTrYggoGxval3sBOh2Z5ofmmWJyq+bXmYOfg6LEe
QkEzCzc9zolwFcq1JKjPa7XSQCGYzyI0zzvFIoTgxQ6KfF2I5DUkzps+GlQebtuy
h6f88/qBVRRiClmpIgUxPoLW7ttXNLwzldMXG+gnoot7TiYaelpkttGsN/H9oPM4
7HLwEXWdyzRSjeZ2axfG34arJ45JK3VmgRAhpuo+9K4l/3wV3s6MJT/KYnAK9y8J
ZgfIPxz88NtFMN9iiMG1D53Dn0reWVlHxYciNuaCp+0KueIHoI17eko8cdLiA6Ef
MgfdG+RCzgwARWGAtQsgWSl4vflVy2PFPEz0tv/bal8xa5meLMFrUKTX5hgUvYU/
Z6tGn6D/Qqc6f1zLXbBwHSs09dR2CQzreExZBfMzQsNhFRAbd03OIozUhfJFfbdT
6u9AWpQKXCBfTkBdYiJ23//OYb2MI3jSNwLgjt7RETeJ9r/tSQdirpLsQBqvFAnZ
0E6yove+7u7Y/9waLd64NnHi/Hm3lCXRSHNboTXns5lndcEZOitHTtNCjv0xyBZm
2tIMPNuzjsmhDYAPexZ3FL//2wmUspO8IFgV6dtxQ/PeEMMA3KgqlbbC1j+Qa3bb
bP6MvPJwNQzcmRk13NfIRmPVNnGuV/u3gm3c
-----END CERTIFICATE-----
)EOF";
char cert_buffer[4096];  // Buffer large enough to hold the certificate
const char* custom_cert_html =
  "<br/><label for='certificate'>Root Certificate</label><textarea name='certificate' rows='10' cols='70'>%s</textarea>";

// Configuration parameters for Google Sheets integration
struct Config {
  String GScriptId;
  String SheetName = "distance";
  String DocumentID;
  String host = "script.google.com";
  String httpsPort = "443";
  String cert = root_ca;
  bool Changed = false;
} conf;

String LastPublish = "";
// Declarations
WiFiManager wifiManager;  // Global WiFiManager instance
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
RH_RF95 rf95(RFM95_CS, RFM95_INT);
uint8_t rx_buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t rx_len = sizeof(rx_buf);
unsigned long lastMessageReceived = 0;
int lastSNR = 0;
bool portalRunning = false;
unsigned long lastButtonPressTime = 0;
unsigned long lastDisplay = 0;


// Custom parameters for WiFiManager
WiFiManagerParameter custom_script_id(
  "gscript_id", "Google Script Deployment ID", conf.GScriptId.c_str(), 80);
WiFiManagerParameter custom_doc_id(
  "custom_doc_id", "Google Spreadsheet Document ID", conf.DocumentID.c_str(), 80);
WiFiManagerParameter custom_sheet_name("sheet_name", "Sheet name", conf.SheetName.c_str(), 40);
WiFiManagerParameter custom_host("host_name", "Host name", conf.host.c_str(), 40);
WiFiManagerParameter custom_port("port_number", "Service Port", conf.httpsPort.c_str(), 6);
WiFiManagerParameter custom_cert(cert_buffer);

// Forward declarations
void setupDisplay();
void setupConfig();
void setupWiFi();
void setupLoRa();
void printInitialDisplay();
void saveConfigIfNeeded();
void receiveLoRaMessages();
void updateDisplayIfNeeded();
void toggleWiFiPortal();
void updateCustomFields();
void saveConfigCallback();
void pushLogs(const String& message);
void displayMeasurements(measurements_t* measures, unsigned long lastMessageTime, int snr);
void checkButton();
String bytesToHexString(uint8_t* buf, unsigned int len);

/**
 * Initialize system components and services.
 */
void setup() {
  setCpuFrequencyMhz(240);
  Serial.begin(SERIAL_CONSOLE_SPEED);
  

  auto upto = millis()+2000;
  while(!Serial && millis()<upto){}
  Wire.setPins(OLED_SDA, OLED_SCL);
  pinMode(TRIGGER_PIN, INPUT);
  setupDisplay();
  setupConfig();
  setupWiFi();
  setupLoRa();
  printInitialDisplay();
}

/**
 * @brief The main loop where essential system functions are orchestrated.
 *
 * This function represents the core execution cycle of the firmware. It continuously processes WiFi
 * management tasks, checks the status of a system button, manages configuration saving, receives
 * messages from LoRa, and updates the display. It is designed to run indefinitely, handling both
 * regular tasks and event-driven operations to maintain the system's responsiveness and
 * functionality.
 *
 * Details of operations handled in the loop:
 * - `wifiManager.process()`: Handles WiFi events and manages network connectivity.
 * - `checkButton()`: Monitors for button presses to potentially toggle the WiFi configuration
 * portal.
 * - `saveConfigIfNeeded()`: Saves the current configuration to persistent storage if there have
 * been any changes.
 * - `receiveLoRaMessages()`: Checks for and processes incoming LoRa messages.
 * - `updateDisplayIfNeeded()`: Updates the OLED display if there are changes to display content.
 *
 * @note This function must be optimized to avoid long blocking operations as it impacts the
 * responsiveness of user interactions and system communications. Ensure that any function called
 * within this loop adheres to real-time constraints appropriate for your application's
 * requirements.
 *
 * Usage Example:
 *
 * The `loop` function is automatically called by the Arduino framework and should be implemented to
 * handle all periodic and event-driven tasks necessary for the application. Here it's used to
 * integrate several subsystems (WiFi, button interface, LoRa communication, display updates) into a
 * cohesive operation:
 *
 *     void setup() {
 *         // Initialization routines for WiFi, button, LoRa, display
 *     }
 *
 *     void loop() {
 *         // Core loop code here
 *     }
 */
void loop() {
  wifiManager.process();    // Handle WiFi connection management
  checkButton();            // Check for user input via the button
  saveConfigIfNeeded();     // Persist configuration changes if needed
  receiveLoRaMessages();    // Handle incoming LoRa communications
  updateDisplayIfNeeded();  // Update display content if there's new data
}

/**
 * @brief Checks the state of a system button and toggles the WiFi portal if necessary.
 *
 * This function reads the state of a button connected to the `TRIGGER_PIN`. If the button is
 * pressed (detected as LOW), it implements a debounce logic and checks the button state again after
 * a brief delay. If the button is still pressed after the delay, the function calculates if the
 * minimum interval since the last recognized press has elapsed to avoid rapid toggling. If this
 * interval has passed, it updates the timestamp and calls `toggleWiFiPortal` to change the state of
 * the WiFi configuration portal.
 *
 * @note The button should be wired to pull up to VCC normally and connect to GND when pressed,
 * ensuring `TRIGGER_PIN` reads LOW when pressed. The system should handle `minInterval` and
 * `debounceDelay` to optimize responsiveness and prevent bouncing issues.
 *
 * Usage Example:
 *
 *     pinMode(TRIGGER_PIN, INPUT);  // Set the button pin as input
 *     attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), checkButton, CHANGE);  // Attach an
 * interrupt to the button pin
 *
 * This configuration allows for responsive user interaction without blocking the main program
 * execution, as the check is performed efficiently on button press.
 */
void checkButton() {
  if (digitalRead(TRIGGER_PIN) == LOW) {
    delay(debounceDelay);  // Debounce by waiting for a fixed short delay
    if (digitalRead(TRIGGER_PIN) == LOW) {
      unsigned long currentTime = millis();  // Get the current time in milliseconds
      // Check if the required time has passed since the last button press
      if (currentTime - lastButtonPressTime >= minInterval) {
        lastButtonPressTime = currentTime;  // Update the last button press time
        toggleWiFiPortal();                 // Toggle the state of the WiFi portal
      }
    }
  }
}

/**
 * @brief Toggles the WiFi configuration portal on or off.
 *
 * This function checks the current state of the WiFi portal. If the portal is not running, it
 * starts the portal and sets the `portalRunning` flag to true. If the portal is already running, it
 * stops the portal and sets the `portalRunning` flag to false. This allows for dynamic control over
 * the WiFi configuration portal, facilitating user interaction for network configuration without
 * needing to restart the device.
 *
 * @note This function should be tied to a user-triggered event, such as pressing a button, to
 *       allow manual control over the WiFi configuration state.
 *
 * Usage Example:
 *
 * Typically, this function is bound to a physical button press:
 *
 *     pinMode(buttonPin, INPUT_PULLUP); // Setup button pin
 *     attachInterrupt(digitalPinToInterrupt(buttonPin), toggleWiFiPortal, FALLING);
 *
 * This setup allows users to open the configuration portal by pressing a button, enabling them
 * to connect the device to new networks or change WiFi settings.
 */
void toggleWiFiPortal() {
  if (!portalRunning) {
    Serial.println("Button Pressed, Starting Portal");
    updateCustomFields();          // Refresh custom fields to ensure current settings are displayed
    wifiManager.startWebPortal();  // Start the WiFi configuration portal
    portalRunning = true;          // Set flag to indicate the portal is running
  } else {
    Serial.println("Button Pressed, Stopping Portal");
    wifiManager.stopWebPortal();  // Stop the WiFi configuration portal
    portalRunning = false;        // Reset flag to indicate the portal is not running
  }
}

/**
 * @brief Displays current configuration settings to the serial output.
 *
 * This function prints the values of key configuration parameters including the Google Script ID,
 * sheet name, document ID, server host, HTTPS port, etc. It is useful for debugging
 * and verifying that configuration settings have been loaded and set correctly.
 *
 * @note This function outputs data to the serial monitor, which should be opened and set to the
 * correct baud rate to view the output.
 *
 * Usage Example:
 *
 * Call this function after configuration settings are loaded or updated to verify their values:
 *
 *     setupConfig();
 *     printContext();  // Verify the loaded configuration settings.
 */
void printContext() {
  Serial.println("Context data:");
  Serial.print("conf.GScriptId: ");
  Serial.println(conf.GScriptId);
  Serial.print("conf.SheetName: ");
  Serial.println(conf.SheetName);
  Serial.print("conf.DocumentID: ");
  Serial.println(conf.DocumentID);
  Serial.print("conf.host: ");
  Serial.println(conf.host);
  Serial.print("conf.httpsPort: ");
  Serial.println(conf.httpsPort);
  Serial.print("conf.cert: ");
  Serial.println(conf.cert);

}
/**
 * Retrieves a string value from a JSON document or returns a default if the key is not present or
 * the value is null.
 * @param doc The JSON document from which to extract the value.
 * @param key The key associated with the value to retrieve.
 * @param defaultValue The default value to return if the key is not found or the value is null.
 * @return The string value from the JSON document or the default value.
 */
String jsonValueOrDefault(
  const JsonDocument& doc, const char* key, const String& defaultValue = "") {
  Serial.printf("Checking if value exists for key=%s. ", key);
  if (doc.containsKey(key) && !doc[key].isNull()) {
    Serial.printf("Existing value = %s", doc[key].as<String>().c_str());
    return doc[key].as<String>();
  } else {
    Serial.printf("Default value = %s", defaultValue.c_str());
    return defaultValue;
  }
}

/**
 * @brief Initializes and loads configuration settings from SPIFFS.
 *
 * This function is designed to initialize the SPIFFS file system and load configuration settings
 * from a JSON file stored on the device. It reads the 'config.json' file from SPIFFS, parses its
 * content to extract configuration parameters like Google Script ID, Sheet Name, Document ID, host,
 * and HTTPS port. The function updates these parameters into the global
 * configuration structure if the file exists and is successfully parsed.
 *
 * @note If 'config.json' does not exist or the SPIFFS cannot be initialized, the configuration will
 *       not be updated, and the device will use default settings or previously loaded
 * configurations.
 *
 * @pre SPIFFS must be properly set up in the platform's filesystem before calling this function.
 *
 * @post On successful execution, the global configuration structure is updated with values loaded
 *       from 'config.json'. Custom fields are also updated to reflect these new configurations.
 *
 * @see updateCustomFields()
 *
 * Usage Example:
 *
 * Assuming 'config.json' is properly formatted and stored, calling this function at startup
 * helps in initializing device settings without hard-coding them into the firmware.
 */
void setupConfig() {
  display.printf("Setting up config\n");
  display.display();
  Serial.println("setupConfig: Initializing SPIFFS");
  if (SPIFFS.begin(true)) {
    Serial.println("setupConfig: Checking if config exists");
    if (SPIFFS.exists("/config.json")) {
      Serial.println("setupConfig: Opening config");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("setupConfig: Loading content");
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        configFile.close();
        Serial.println("setupConfig: Parsing json:");
        Serial.println(buf.get());
        json.clear();
        deserializeJson(json, buf.get());
        conf.GScriptId = jsonValueOrDefault(json, "gscript_id");
        conf.SheetName = jsonValueOrDefault(json, "sheet_name", "distance");
        conf.DocumentID = jsonValueOrDefault(json, "document");
        conf.host = jsonValueOrDefault(json, "host", "script.google.com");
        conf.httpsPort = jsonValueOrDefault(json, "httpsPort", "443");
        conf.cert = jsonValueOrDefault(json, "certificate", root_ca);

        json.clear();
        updateCustomFields();  // Update custom fields to reflect new configurations
      }
    } else {
      Serial.println("setupConfig: Config file not found");
    }
  } else {
    Serial.println("setupConfig: Failed to initialize SPIFFS");
    display.println("SPIFFS init failed. Halting");
    display.display();
    while(true){delay(1000);}
  }
}

/**
 * Print initial status on the OLED display.
 */
void printInitialDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("System Initialized");
  display.display();
}

/**
 * @brief Initializes the OLED display.
 *
 * This function is responsible for setting up the OLED display using the Adafruit SSD1306 library.
 * It starts by resetting the display, then attempts to initialize it with specified configuration
 * parameters. If the initialization fails, the function halts the program execution and prints
 * an error message to the serial monitor.
 *
 * @note This function will enter an infinite loop if the OLED display fails to initialize.
 *       Check the display connections and configurations if this occurs.
 *
 * @pre OLED_RST must be connected to a valid GPIO pin configured as an output.
 *
 * @post If the function returns (does not halt), the OLED display is ready for use.
 */
void setupDisplay() {
  
  // Attempt to initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED initialization failed");  // Initialization failed message
    for (;;)
      ;  // Halt execution on failure
  }
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("Initializing \n");
  display.display();
}

/**
 * @brief Initializes the LoRa module and configures communication settings.
 *
 * This function sets up the LoRa communication module using the RH_RF95 library. It starts by
 * initializing the SPI interface specifically configured for the connected LoRa module. It resets
 * the LoRa module, initializes it, and sets the frequency and transmission power.
 *
 * @note This function will halt the program in an infinite loop if the LoRa module fails to
 * initialize. Ensure that the LoRa module is correctly connected and configured in the hardware
 * setup.
 *
 * @pre SPI pins (SX1278_SCLK, SX1278_MISO, SX1278_MOSI, SX1278_CS) must be correctly defined and
 * connected to the corresponding pins of the LoRa module.
 * @pre RFM95_RST must be connected to a valid GPIO pin configured as an output.
 *
 * @post If the function returns (does not halt), the LoRa module is ready for sending and receiving
 * data.
 *
 * @see RH_RF95::init()
 * @see RH_RF95::setFrequency()
 * @see RH_RF95::setTxPower()
 */
void setupLoRa() {
  display.printf("Setting up Radio\n");
  display.display();
  Serial.println("Setting up LoRa radio.");
  SPI.begin(SX1278_SCLK, SX1278_MISO, SX1278_MOSI, SX1278_CS);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if (!rf95.init()) {                              // Initialize the LoRa module
    Serial.println("LoRa initialization failed");  // Report failure
    for (;;)
      ;  // Halt execution if initialization fails
  }
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(23, false);
  rf95.setModemConfig(RF95_MODE);
  Serial.println("Setting up LoRa radio completed successfully.");
}

/**
 * @brief Initializes and connects to WiFi using the WiFiManager library.
 *
 * This function sets up the WiFi on the ESP32 to station mode and configures the WiFiManager
 * for user interaction. It adds custom parameters for Google Script configuration to the
 * WiFiManager, sets a timeout for the configuration portal, and attempts to auto-connect to a known
 * network. If the connection is successful, it synchronizes the device time with NTP servers and
 * sends a dummy measurement to ensure functionality. If unable to connect automatically, it starts
 * a configuration portal accessible via a web browser to allow a user to input WiFi credentials
 * manually.
 *
 * @note This function will block execution until WiFi is configured and connected, or until the
 * configuration portal times out. Ensure that the device can enter configuration mode without
 * disruptions.
 *
 * @see WiFiManager
 * @see prepareAndSendData(measurements_t*, int)
 */
void setupWiFi() {
  display.printf("Setting up WiFi\n");
  display.display();
  wifiManager.setDebugOutput(true, WM_DEBUG_VERBOSE);  // Enable detailed debug output
  
  // Set callback function to save configuration changes
  wifiManager.setSaveConfigCallback(saveConfigCallback);                  
  wifiManager.setAPCallback( wifiManagerAPStarted);
  wifiManager.setConfigPortalTimeoutCallback(wifiManagerPortalTimeout_cb);
  wifiManager.setBreakAfterConfig(true);  // Ensure the device restarts after configuration
  wifiManager.setConfigPortalTimeout(5*60);  // Set timeout for the WiFi configuration portal to 120 seconds

  // Log that custom parameters are being added to the WiFiManager
  Serial.println("Adding custom parameters to the wifiManager's list");

  // Add custom parameters for Google Script configuration
  wifiManager.addParameter(&custom_script_id);
  wifiManager.addParameter(&custom_doc_id);
  wifiManager.addParameter(&custom_sheet_name);
  wifiManager.addParameter(&custom_host);
  wifiManager.addParameter(&custom_port);
  snprintf(cert_buffer, sizeof(cert_buffer), custom_cert_html, conf.cert.c_str());
  wifiManager.addParameter(&custom_cert);
  // Attempt to connect to WiFi automatically using stored credentials
  String hostString = String(WIFI_getChipId(),HEX);
  hostString.toUpperCase();
  hostString="LORAGW_"+hostString;
  if (wifiManager.autoConnect(hostString.c_str(),apPassword)) {
    Serial.println("connected...yeey :)");
    display.printf("Connected to %s\nSyncing with NTP\n",WiFi.SSID().c_str());
    display.display();

    // Synchronize time with NTP servers
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print("Waiting for NTP time sync: ");
    time_t nowSecs = time(nullptr);
    while (nowSecs < 8 * 3600 * 2) {
      delay(500);
      Serial.print(".");
      nowSecs = time(nullptr);
    }
    Serial.println("done!");

    // Send a dummy measurement to test the configuration
    pushLogs("Receiver started");
  } else {
    Serial.println("Configportal running");  // Notify that the configuration portal is running
  }
}
bool checkConfig() {
  bool result = true;
  if (conf.DocumentID.isEmpty()) {
    Serial.println("DocumentID empty");
    result = false;
  }
  if (conf.host.isEmpty()) {
    Serial.println("host empty");
    result = false;
  }
  if (conf.GScriptId.isEmpty()) {
    Serial.println("GScriptId empty");
    result = false;
  }
  if (conf.httpsPort.isEmpty()) {
    Serial.println("httpsPort empty");
    result = false;
  }
  if (conf.cert.isEmpty()) {
    Serial.println("Certificate empty");
    result = false;
  }

  return result;
}
/**
 * Helper function to URL encode a string.
 *
 * @param value The string to encode.
 * @return The URL-encoded string.
 */
String urlEncode(const String& value) {
  String encodedString = "";
  const char* hex = "0123456789abcdef";
  byte* data = (byte*)value.c_str();
  for (size_t i = 0; i < value.length(); i++) {
    if (('a' <= data[i] && data[i] <= 'z')
        || ('A' <= data[i] && data[i] <= 'Z')
        || ('0' <= data[i] && data[i] <= '9')) {
      encodedString += char(data[i]);
    } else {
      encodedString += '%';
      encodedString += char(hex[data[i] >> 4]);
      encodedString += char(hex[data[i] & 0x0F]);
    }
  }
  return encodedString;
}


/**
 * Sends JSON data to a specified Google Sheet.
 *
 * @param doc A reference to the JSON document containing data to be sent.
 * @param sheetName Name of the sheet to which data will be appended or inserted.
 */
void sendToGoogleSheet(JsonDocument& doc, const String& sheetName) {
  if(!WiFi.isConnected()){
    LastPublish = "WiFi not connected!";
    return;
  }
  #if defined(ESP_ARDUINO_VERSION) && (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
  NetworkClientSecure* client = new NetworkClientSecure;
  #else
  WiFiClientSecure* client = new WiFiClientSecure;
  #endif
  if(conf.cert.isEmpty()){
    Serial.println("Certificate empty!");
    LastPublish = "Certificate empty!";
    return;
  }
  if(conf.GScriptId.isEmpty()){
    LastPublish = "Google Script not configured";
    return;
  }
  if(conf.DocumentID.isEmpty()){
    LastPublish = "DocumentID not configured";
    return;
  }  
  if (client) {
    client->setCACert(conf.cert.c_str());
    doc["command"] = "append_row";
    String urlFinal = "https://" + conf.host + String("/macros/s/") + conf.GScriptId + "/exec?" + "sheet_name=" + sheetName;

    // Iterate over all the keys in the JSON document and add them as URL parameters
    for (JsonPair kv : doc.as<JsonObject>()) {
      urlFinal += String("&") + kv.key().c_str() + "=" + urlEncode(kv.value().as<String>());
    }

    Serial.print("Sending data to spreadsheet:");
    Serial.println(urlFinal);
    HTTPClient https;
    if (https.begin(*client, urlFinal.c_str())) {  // HTTPS
      https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
      int httpCode = https.GET();
      Serial.print("HTTP Status Code: ");
      Serial.println(httpCode);
      //---------------------------------------------------------------------
      //getting response from google sheet
      if (httpCode == 200) {
        DeserializationError err = deserializeJson(json, https.getString());
        if (err) {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(err.c_str());
        } else {
          Serial.printf("Result: %s. Message: %s\n", json["status"].as<String>().c_str(), json["message"].as<String>().c_str());
          LastPublish = json["message"].as<String>().c_str();
        }
      } else if (httpCode > 0) {
        Serial.println("Content: ");
        Serial.println(https.getString());
      }
      //---------------------------------------------------------------------
      https.end();
    } else {
      Serial.println("https begin failed!");
    }
  } else {
    Serial.println("SSL Client creation failed. Insufficient memory?");
  }
}

/**
 * Prepares measurement data for transmission and sends it to the specified Google Sheet.
 *
 * @param measures Pointer to the measurements_t struct containing sensor data.
 * @param snr Signal-to-noise ratio of the last transmission, used as part of the data payload.
 */
void prepareAndSendData(measurements_t* measures, int snr) {
  json.clear();
  json["bat_voltage"] = measures->bat_voltage;
  json["rawVbat"] = measures->rawVbat;
  json["distance"] = measures->distance;
  json["duration"] = measures->duration;
  json["temperature"] = measures->temperature;
  json["internal_temp"] = measures->internal_temp;
  json["packetnum"] = measures->packetnum;
  json["snr"] = snr;
  json["uuid"] =
    bytesToHexString((uint8_t*)&measures->uuid, sizeof(measures->uuid) / sizeof(uint8_t));
  Serial.print("UUID (HEX)");
  Serial.println(bytesToHexString((uint8_t*)&measures->uuid, sizeof(measures->uuid) / sizeof(uint8_t)));  
  sendToGoogleSheet(json, conf.SheetName);
  json.clear();
}
/**
 * Logs messages to a "logs" sheet in Google Sheets.
 *
 * @param message The log message to be recorded.
 */
void pushLogs(const String& message) {
  json.clear();
  json["Message"] = message;
  sendToGoogleSheet(json, "logs");
  json.clear();
}


/**
 * @brief Converts a byte array to a hexadecimal string.
 *
 * This helper function takes a byte array and its length and converts it to a
 * string of hexadecimal characters for easier logging and debugging.
 *
 * @param buf The byte array to convert.
 * @param len The length of the byte array.
 * @return A String representing the hexadecimal values of the bytes.
 */
String bytesToHexString(uint8_t* buf, unsigned int len) {
  String str = "";
  for (unsigned int i = 0; i < len; ++i) {
    if (buf[i] < 0x10) str += "0";  // Pad with zero for single digit hex values
    str += String(buf[i], HEX) + ":";
  }
  return str.substring(0, str.length() - 1);  // Remove the last colon
}

/**
 * @brief Receives messages from a LoRa device and processes them if available.
 *
 * This function checks if there is a message available from the LoRa module. If a message is
 * available, it attempts to read it into a buffer. If the read operation is successful and the
 * length of the message matches the size of the `measurements_t` structure, it processes the
 * message. Processing involves casting the buffer to a `measurements_t` structure, updating the
 * timestamp of the last message received, storing the signal-to-noise ratio (SNR), and pushing the
 * measurements data to a remote server or display. If the received message does not match the
 * expected size, it logs the message content as a hexadecimal string.
 *
 * @note This function must be called within the main loop to continuously check for new LoRa
 * messages.
 *
 * @see prepareAndSendData(measurements_t*, int)
 * @see displayMeasurements(measurements_t*, unsigned long, int)
 */
void receiveLoRaMessages() {
  if (rf95.available()) {
    Serial.println("Lora message detected");
    if (rf95.recv(rx_buf, &rx_len)) {
      if (rx_len == sizeof(measurements_t)) {
        measurements_t* measures = reinterpret_cast<measurements_t*>(rx_buf);
        lastMessageReceived = millis();
        lastSNR = rf95.lastSNR();
        // show we've received a message, as sending takes a bit of time
        displayMeasurements(measures, lastMessageReceived, lastSNR);
        prepareAndSendData(measures, lastSNR);
        // update display with publish results
        displayMeasurements(measures, lastMessageReceived, lastSNR);
      } else {
        String hexString = bytesToHexString(rx_buf, rx_len);
        String errorMessage = "Received unexpected LoRa message of length " + String(rx_len) + ", expected: "+String(sizeof(measurements_t))+ ": " + hexString;
        Serial.println(errorMessage);
        pushLogs(errorMessage);
      }
    } else {
      Serial.println("Failed to receive LoRa message.");
    }
  }
}

/**
 * Update the OLED display if needed.
 */
void updateDisplayIfNeeded() {
  if (millis() - lastDisplay > 1000) {  // Update every second
    lastDisplay = millis();
    displayMeasurements(
      reinterpret_cast<measurements_t*>(rx_buf), lastMessageReceived, lastSNR);
  }
}


/**
 * Display measurement data on the OLED screen.
 */
void displayMeasurements(measurements_t* measures, unsigned long lastMessageTime, int snr) {
  display.clearDisplay();
  
  display.setCursor(0, 0);
  unsigned long elapsedTime = (millis() - lastMessageTime) / 1000;
  display.printf("Last: %02lu:%02lu:%02lu\n", elapsedTime / 3600, (elapsedTime % 3600) / 60, elapsedTime % 60);
  display.printf("%s\n",LastPublish.c_str());
  display.printf("Dist: %ldmm\n", measures->distance, measures->duration);
  display.printf("Bat:%.2fV Raw:%lu\n", measures->bat_voltage, measures->rawVbat);
  display.printf("T:%.1fC IT:%.1fC\n", measures->temperature, measures->internal_temp);
  display.printf("Pkt#: %d SNR: %ddB\n", measures->packetnum, snr);
  display.display();
}


//==========================================================
// WifiManager/configuration


/**
 * Save system configuration if it has been modified.
 */
void saveConfigIfNeeded() {
  if (conf.Changed) {
    conf.Changed = false;
    Serial.println("Saving configuration");
    LastPublish = "Saving configuration";
    printContext();
    File configFile = SPIFFS.open("/config.json", "w");
    if (configFile) {
      json.clear();
      json["gscript_id"] = conf.GScriptId;
      json["sheet_name"] = conf.SheetName;
      json["document"] = conf.DocumentID;
      json["host"] = conf.host;
      json["httpsPort"] = conf.httpsPort;
      json["certificate"] = conf.cert;
      serializeJson(json, configFile);
      json.clear();
      configFile.close();
    } else {
      Serial.println("**** Failed to open config file for writing");
    }
  }
}


/**
 * Update custom fields in the WiFiManager configuration portal.
 */
void updateCustomFields() {
  Serial.println("Updating wifi manager custom values");
  printContext();
  custom_script_id.setValue(conf.GScriptId.c_str(), 80);
  custom_doc_id.setValue(conf.DocumentID.c_str(), 80);
  custom_sheet_name.setValue(conf.SheetName.c_str(), 40);
  custom_host.setValue(conf.host.c_str(), 40);
  custom_port.setValue(conf.httpsPort.c_str(), 6);
  snprintf(cert_buffer, sizeof(cert_buffer), custom_cert_html, conf.cert.c_str());
}
void wifiManagerPortalTimeout_cb(){
  display.println("Settings portal timeout.");
  display.display();
}
void wifiManagerAPStarted(WiFiManager*manager){
  display.printf("Starting AP.\nSSID: %s\nPASSWORD: %s", manager->getConfigPortalSSID().c_str(),apPassword);
  
  display.display();
}


/**
 * Callback to save configuration when WiFi settings are modified.
 */
void saveConfigCallback() {
  Serial.print("Configuration has been updated and will be saved.");
  conf.GScriptId = custom_script_id.getValue();
  conf.DocumentID = custom_doc_id.getValue();
  conf.SheetName = custom_sheet_name.getValue();
  conf.host = custom_host.getValue();
  conf.httpsPort = custom_port.getValue();

  if (wifiManager.server->hasArg("certificate")) {
    conf.cert = wifiManager.server->arg("certificate");
    Serial.printf("\nReceived Cert: \n\n%s\n", conf.cert.c_str());
  } else {
    Serial.println("No certificate in the data");
  }
  conf.Changed = true;
}
