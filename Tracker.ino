#include <EEPROM.h>
#include "BigNumber.h"
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SD.h>
#include <sim800.h>
#include "timestamp32bits.h"


#define gprsContextAddress 0
#define gprsAuthAddress 1
#define ApnAddress 2
#define ApnUsernameAddress 34
#define ApnPasswordAddress 66
#define mainServerIpAddress 98
#define mainServerPortAddress 102
#define mainServerProtocolAddress 104
#define mainServerTlsEncryptionAddress 105
#define IoStructureArrayAddress 106

void(* resetFunc) (void) = 0;

timestamp32bits stamp = timestamp32bits();
#define config_file_name "config.txt"
#define log_file_name "LOG.txt"
#define M_PI       3.14159265358979323846   // pi
#define DEFAULT_BUFLEN 512  

Adafruit_GPS GPS(&Serial1);

// variables for SIM800L Code
int s = 0;
int k = 0;
int iRet = 0;
boolean sendCompleted = false;
boolean timeToSend = false;
boolean gprsEnabled = false;
boolean connectedToServer = false;
int connectionTimeout = 0;
char IMEI[20] = "";
char c;
boolean connectionInteruot = false;
uint8_t recv[21];
unsigned long lastRead = millis();
unsigned long timeEnd = 0, timeBegin = 0;
unsigned long duration = 0;
double averageDuration = 0.0;
uint8_t r = 0;
#define FONA_RST 4  // no need to connect this; 
Modem modem = Modem(FONA_RST);
bool defineSerial = false;
///////////////////////////

File config_file;
File log_file;
int i = 0;
char* value_2 = new char[2];
char* value_4 = new char[4];
char* value_8 = new char[8];
unsigned int low, high;
char* value_16 = new char[16];
char* value_data = new char[512];
String str_temp = "";
BigNumber temp = "";
BigNumber byte_8 = "18446744073709551616";
BigNumber byte_7 = "72057594037927936";
BigNumber byte_6 = "281474976710656";
BigNumber byte_5 = "1099511627776";
BigNumber byte_4 = "4294967296";
BigNumber byte_3 = "16777216";
BigNumber byte_2 = "65536";
BigNumber byte_1 = "256";
BigNumber byte_0 = "1"; 
int data_timer_counter = 0;

// input output define 
  int internal_voltage_in =0;
  int internal_voltage =0;
  
  int external_voltage_in =0;
  int external_voltage =0;
  
  int car_is_working_in =0;
  int car_is_working =0;

  int fuel_sensor_in =0;
  int fuel_sensor =0;
  
  int charge_battery_in = 13;
  int charge_battery = 0;
  
  int cutoff_command_in = 7;
  int cutoff_command = 0;
  
  int cutoff_engine = 12;

// time
  int year;
  int mon;
  int mday;
  int hour;
  int min;
  int sec;
  int number;
  unsigned int timeAcc = 0;
  BigNumber timeAccBig;
  char cstr[6] = "";
  static char bufferr[32];
  static byte idx = 0;
  boolean recvb = false;
  
// GPS_Element
  float longitude;
  float latitude;
  float altitude;
  float angle;
  float satellites;
  float speed;
  float HDOP;
  float VDOP;
  float PDOP;
  boolean fixedForFirstTime = false;
  
  // GPS_Backup_Element
  float longitude_Backup;
  float latitude_Backup;
  float altitude_Backup;
  float angle_Backup;
  float satellites_Backup;
  float speed_Backup;
  float HDOP_Backup;
  float VDOP_Backup;
  float PDOP_Backup;

// IO_Elemnet
  int Event_ID;
  int Element_Count;
  int _1b_Element_Count;
  unsigned int _1b_DATA[20][2];
  int _2b_Element_Count;
  unsigned int _2b_DATA[20][2];
  int _4b_Element_Count;
  unsigned long _4b_DATA[20][2];
  int _8b_Element_Count;
  int _8b_sensor_id[10];
  BigNumber _8b_DATA[10];

// AVL_DATA
  BigNumber now = 0;
  int priority;

// DATA
  int Codec_id;
  int AVL_DATA_COUNT;

// TCP_AVL_DATA_PACKET
  unsigned int preamble;
  unsigned long AVL_DATA_LENGTH;
  unsigned long Crc;

// tracker_type
  int socket;
  //unsigned char* APN = new unsigned char[20];
  unsigned char* user = new unsigned char[20];
  unsigned char* pass = new unsigned char[20];
  unsigned char* imei = new unsigned char[20];
  unsigned char* received_message = new unsigned char[DEFAULT_BUFLEN];

//////##########################################################//////
//////##########################################################//////
//////############# Configurator Code ##########################//////
//////#############      GLOBAL       ##########################//////
//////#############                   ##########################//////
//////##########################################################//////
//////##########################################################//////
bool heartBeatBegin = false;
unsigned long lastHeartBeat = 0, timeAfterHeartBeat = 0;
int resetPin = 3;
int redPin = 2;
bool breakToCheckConf   = false;
bool configuration_mode = false;
bool checkConfigConnection       = false;
int  configConnectTimeout = 0;
// APN Variables
byte gprsContext        = 0;
byte gprsAuthentication = 0;
char APN[32];
char APN_Username[32];
char APN_Password[32];
byte APNLen = 0;
byte APN_UsernameLen = 0;
byte APN_PasswordLen = 0;

// data Aquisition Periods Variables
int dataAqOnStopVars[9]           = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int dataAqMovingVars[18]          = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int dataAqOnDemandTrackingVars[2] = {0, 0};

// main Server Setting
char  mainServerIp[4];
char IpFormatted[15]; 
int   mainServerPort           = 0;
byte   mainServerProtocol      = 0;
byte   mainServerTlsEncryption = 0;

// second Server Setting
byte   secondServerMode          = 0;
char   secondServerIp[4];
int    secondServerPort          = 0;
byte   secondServerProtocol      = 0;
byte   secondServerTlsEncryption = 0;

// recordTimeoutSetting
int openLinkTimeout      = 0;
int responseTimeout      = 0;
int networkPingTimeout   = 0;
byte sortBy              = 0;
byte recordTlsEncryption = 0;


// IO Setting 
int ioType = 0;
typedef struct 
{
  byte priority = 0;
  int lowLevel   = 0;
  int highLevel = 0;
  byte eventsOnly = 0;
  byte operand   = 0;
  int avgConstant = 0;
  byte sendSmsToLength = 0;
  char sendSmsTo[10];
} IO_System_Event;
IO_System_Event ioEventsArray[36];

// send variables to configurator
int  infoType = 0;
byte varIndex = 0;


struct time_struct
{
  int seconds;
  int hours;
  int minutes;
  int days;
  int months;
  int years;
};
// readed message from server variables
int availableBytes = 0;


// parser variables
int parserheader = 0;
int parserdataLength = 0;
int parserdirectionOfData = 0;
char* parserdataMessage = new char[128];
int parsercommand = 0;
time_struct parserdate = {0, 0, 0, 0, 0, 0};
int parsercheckSum = 0;
int parserfooter = 0;

//#include "MessageParser.h"
int incomingByte = 0; // for incoming serial data
char* message_to_server = new char[512];
char* deviceType = new char[6];
unsigned long time_now = 0;

// message variables
int Header = 255;
int dataLength = 0;
int directionOfData = 0;
int command = 0;
int checkSumm = 0;
int Footer = 205;

//////##########################################################//////
//////##########################################################//////
//////############# Configurator FUNC ##########################//////
//////#############                   ##########################//////
//////#############                   ##########################//////
//////##########################################################//////
//////##########################################################//////
void cahrToIpFarmat()
{
  memset(IpFormatted,0,15);
  strcat(IpFormatted, (char *)String((byte)mainServerIp[0]).c_str());
  strcat(IpFormatted, ".");
  strcat(IpFormatted, (char *)String((byte)mainServerIp[1]).c_str());
  strcat(IpFormatted, ".");
  strcat(IpFormatted, (char *)String((byte)mainServerIp[2]).c_str());
  strcat(IpFormatted, ".");
  strcat(IpFormatted, String((byte)mainServerIp[3]).c_str());
}

void getDataFromEprom()
{
  Serial.begin(115200);
  memset(APN,0,32);
  memset(APN_Username,0,32);
  memset(APN_Password,0,32);
  memset(mainServerIp,0,4);
  memset(IpFormatted,0,15);
  for(k = 0; k < 36; k++)
  {
    memset(ioEventsArray[k].sendSmsTo,0,10);
  }
  EEPROM.get(gprsContextAddress, gprsContext); //write the updated run count back to eeprom
  EEPROM.get(gprsAuthAddress, gprsAuthentication);
  EEPROM.get(ApnAddress, APN);
  EEPROM.get(ApnUsernameAddress, APN_Username);
  EEPROM.get(ApnPasswordAddress, APN_Password);
  EEPROM.get(mainServerIpAddress, mainServerIp);
  EEPROM.get(mainServerPortAddress, mainServerPort);
  EEPROM.get(mainServerProtocolAddress, mainServerProtocol);
  EEPROM.get(mainServerTlsEncryptionAddress, mainServerTlsEncryption);
  EEPROM.get(IoStructureArrayAddress, ioEventsArray);
  APNLen = strlen(APN);
  APN_UsernameLen = strlen(APN_Username);
  APN_PasswordLen = strlen(APN_Password);
}

void printEpromData()
{
  Serial.println("after reading from eprom: ");
  Serial.print("gprsContext: "); Serial.println(gprsContext);
  Serial.print("APN: "); Serial.println(APN);
  Serial.print("APN_Username: "); Serial.println(APN_Username);
  Serial.print("APN_Password: "); Serial.println(APN_Password);
  cahrToIpFarmat();
  Serial.print("mainServerIp: "); Serial.println(IpFormatted);
  Serial.print("mainServerPort: "); Serial.println(mainServerPort);
  Serial.print("mainServerProtocol: "); Serial.println(mainServerProtocol);
  Serial.print("mainServerTlsEncryption: "); Serial.println(mainServerTlsEncryption);
  Serial.print("ioEventsArray[5].priority: "); Serial.println(ioEventsArray[5].priority);
  Serial.print("ioEventsArray[5].lowLevel: "); Serial.println(ioEventsArray[5].lowLevel);
  Serial.print("ioEventsArray[5].highLevel: "); Serial.println(ioEventsArray[5].highLevel);
  Serial.print("ioEventsArray[5].eventsOnly: "); Serial.println(ioEventsArray[5].eventsOnly);
  Serial.print("ioEventsArray[5].operand: "); Serial.println(ioEventsArray[5].operand);
  Serial.print("ioEventsArray[5].avgConstant: "); Serial.println(ioEventsArray[5].avgConstant);
  Serial.print("ioEventsArray[5].sendSmsToLength: "); Serial.println(ioEventsArray[5].sendSmsToLength);
  Serial.print("ioEventsArray[5].sendSmsTo: "); Serial.println(ioEventsArray[5].sendSmsTo);
}

void gen_error_infoType_onCalling_VarValue()
{
  // infoType
  dataLength = 19;
  directionOfData = 0;
  command = 0;
  checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
  
  memset(message_to_server, 0, sizeof(message_to_server));
  to_String_X2(255);
  strcat(message_to_server, value_2);    // Header
  to_String_X4(dataLength);
  strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
  to_String_X2(directionOfData);
  strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
  to_String_X2(parserdate.years);
  strcat(message_to_server, value_2);    // time years
  to_String_X2(parserdate.months);
  strcat(message_to_server, value_2);    // time months
  to_String_X2(parserdate.days);
  strcat(message_to_server, value_2);    // time days
  to_String_X2(parserdate.hours);
  strcat(message_to_server, value_2);    // time hours
  to_String_X2(parserdate.minutes);
  strcat(message_to_server, value_2);    // time minuts
  to_String_X2(parserdate.seconds);
  strcat(message_to_server, value_2);    // time seconds
  to_String_X4(command);
  strcat(message_to_server, value_4);    // command 0x01 connection
  to_String_X2(1);
  strcat(message_to_server,value_2);  // data errorType
  to_String_X2((int) infoType);
  strcat(message_to_server,value_2);  // date infoType
  to_String_X2((int) 'N');
  strcat(message_to_server,value_2);  // data message "not valid infoType"
  to_String_X2((int) 'O');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'T');
  strcat(message_to_server,value_2);
  to_String_X2((int) ' ');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'V');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'A');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'I');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'D');
  strcat(message_to_server,value_2);
  to_String_X2((int) ' ');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'I');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'N');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'F');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'O');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'T');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'Y');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'P');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'E');
  strcat(message_to_server,value_2);
  to_String_X4(checkSumm);
  strcat(message_to_server,value_4);
  to_String_X2(Footer);
  strcat(message_to_server,value_2);
}

void gen_error_commandType_onUpdatingVars()
{
  // infoType
  dataLength = 18;
  directionOfData = 0;
  command = 0;
  checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
  
  memset(message_to_server, 0, sizeof(message_to_server));
  to_String_X2(255);
  strcat(message_to_server, value_2);    // Header
  to_String_X4(dataLength);
  strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
  to_String_X2(directionOfData);
  strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
  to_String_X2(parserdate.years);
  strcat(message_to_server, value_2);    // time years
  to_String_X2(parserdate.months);
  strcat(message_to_server, value_2);    // time months
  to_String_X2(parserdate.days);
  strcat(message_to_server, value_2);    // time days
  to_String_X2(parserdate.hours);
  strcat(message_to_server, value_2);    // time hours
  to_String_X2(parserdate.minutes);
  strcat(message_to_server, value_2);    // time minuts
  to_String_X2(parserdate.seconds);
  strcat(message_to_server, value_2);    // time seconds
  to_String_X4(command);
  strcat(message_to_server, value_4);    // command 0x01 connection
  to_String_X2(2);
  strcat(message_to_server,value_2);  // data errorType
  to_String_X2((int) parsercommand);
  strcat(message_to_server,value_2);  // date parsercommand
  to_String_X2((int) 'N');
  strcat(message_to_server,value_2);  // data message "not valid command"
  to_String_X2((int) 'O');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'T');
  strcat(message_to_server,value_2);
  to_String_X2((int) ' ');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'V');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'A');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'I');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'D');
  strcat(message_to_server,value_2);
  to_String_X2((int) ' ');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'C');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'O');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'M');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'M');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'A');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'N');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'D');
  strcat(message_to_server,value_2);
  to_String_X4(checkSumm);
  strcat(message_to_server,value_4);
  to_String_X2(Footer);
  strcat(message_to_server,value_2);
}

void refresh_data()
{
  parserheader = 0;
  parserdataLength = 0;
  parserdirectionOfData = 0;
  memset(parserdataMessage, 0, sizeof(parserdataMessage));
  parsercommand = 0;
  parserdate = {0, 0, 0, 0, 0, 0};
  parsercheckSum = 0;
  parserfooter = 0;
}

void parseMessage()
{
  refresh_data();
  parserheader = ((int) message_to_server[0]) + (1 - (((int) message_to_server[0]) >= 0))*256;
  parserdataLength = (((int) message_to_server[2]) + (1 - (((int) message_to_server[2]) >= 0))*256 + 256 * (((int) message_to_server[1])) + (1 - (((int) message_to_server[1]) >= 0))*256);
  parserdirectionOfData = ((int) message_to_server[3]) + (1 - (((int) message_to_server[3]) >= 0))*256;
  parserdate.years = (int) message_to_server[4] + (1 - (((int) message_to_server[4]) >= 0))*256;
  parserdate.months = (int) message_to_server[5] + (1 - (((int) message_to_server[5]) >= 0))*256;
  parserdate.days = (int) message_to_server[6] + (1 - (((int) message_to_server[6]) >= 0))*256;
  parserdate.hours = (int) message_to_server[7] + (1 - (((int) message_to_server[7]) >= 0))*256;
  parserdate.minutes = (int) message_to_server[8] + (1 - (((int) message_to_server[8]) >= 0))*256;
  parserdate.seconds = (int) message_to_server[9] + (1 - (((int) message_to_server[9]) >=0 ))*256;
  parsercommand = ((int) message_to_server[11]) + ((int) message_to_server[10])*256; //((int) message_to_server[11] + (1 - (((int) message_to_server[11]) >= 0))*256) + (((int) message_to_server[10] + (1 - (((int) message_to_server[10]) >= 0))*256))*256;
  strncat(parserdataMessage,message_to_server + 12,parserdataLength);
  parsercheckSum = (((int) message_to_server[12 + dataLength + 1]) + (1 - (((int) message_to_server[12 + dataLength + 1]) >= 0))*256 + ((int) message_to_server[12 + dataLength] + (1 - (((int) message_to_server[12 + dataLength]) >= 0))*256)*256); 
  parserfooter   = (int) message_to_server[12 + dataLength + 2] + (1 - (((int) message_to_server[12 + dataLength + 2]) >= 0))*256;
}

void gen_connection_message()
{
  dataLength = 6;
  directionOfData = 0;
  command = 1;
  checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
  
  memset(message_to_server, 0, sizeof(message_to_server));
  to_String_X2(255);
  strcat(message_to_server, value_2);    // Header
  to_String_X4(dataLength);
  strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
  to_String_X2(directionOfData);
  strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
  to_String_X2(parserdate.years);
  strcat(message_to_server, value_2);    // time years
  to_String_X2(parserdate.months);
  strcat(message_to_server, value_2);    // time months
  to_String_X2(parserdate.days);
  strcat(message_to_server, value_2);    // time days
  to_String_X2(parserdate.hours);
  strcat(message_to_server, value_2);    // time hours
  to_String_X2(parserdate.minutes);
  strcat(message_to_server, value_2);    // time minuts
  to_String_X2(parserdate.seconds);
  strcat(message_to_server, value_2);    // time seconds
  to_String_X4(command);
  strcat(message_to_server, value_4);    // command 0x01 connection
  to_String_X2((int) 'S');
  strcat(message_to_server,value_2);  // dataType "SCTR01"
  to_String_X2((int) 'C');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'T');
  strcat(message_to_server,value_2);
  to_String_X2((int) 'R');
  strcat(message_to_server,value_2);
  to_String_X2((int) '0');
  strcat(message_to_server,value_2);
  to_String_X2((int) '1');
  strcat(message_to_server,value_2);
  to_String_X4(checkSumm);
  strcat(message_to_server,value_4);
  to_String_X2(Footer);
  strcat(message_to_server,value_2);
}

void genApnSettingMessageToServer()
{
  dataLength = 5 + (int) APNLen + (int) APN_UsernameLen + (int) APN_PasswordLen;
  directionOfData = 0;
  command = 2;
  checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
  
  memset(message_to_server, 0, sizeof(message_to_server));
  to_String_X2(255);
  strcat(message_to_server, value_2);    // Header
  to_String_X4(dataLength);
  strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
  to_String_X2(directionOfData);
  strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
  to_String_X2(parserdate.years);
  strcat(message_to_server, value_2);    // time years
  to_String_X2(parserdate.months);
  strcat(message_to_server, value_2);    // time months
  to_String_X2(parserdate.days);
  strcat(message_to_server, value_2);    // time days
  to_String_X2(parserdate.hours);
  strcat(message_to_server, value_2);    // time hours
  to_String_X2(parserdate.minutes);
  strcat(message_to_server, value_2);    // time minuts
  to_String_X2(parserdate.seconds);
  strcat(message_to_server, value_2);    // time seconds
  to_String_X4(command);
  strcat(message_to_server, value_4);    // command 0x01 connection
  // data
  to_String_X2((int) gprsContext);       // gprs context
  strcat(message_to_server, value_2); 
  to_String_X2((int) gprsAuthentication);// gprs authentication
  strcat(message_to_server, value_2);
  to_String_X2((int) APNLen);// length of APN
  strcat(message_to_server, value_2);
  to_String_X2((int) APN_UsernameLen);// length of APN_Username
  strcat(message_to_server, value_2);
  to_String_X2((int) APN_PasswordLen);// length of APN_Password
  strcat(message_to_server, value_2);
  for(k=0; k < (int) APNLen; k++)// APN message
  {
    to_String_X2((int) APN[k]);
    strcat(message_to_server, value_2); 
  }
  for(k=0; k < (int) APN_UsernameLen; k++)// APN_Username message
  {
    to_String_X2((int) APN_Username[k]);
    strcat(message_to_server, value_2); 
  }

  for(k=0; k < (int) APN_PasswordLen; k++)// APN_Password message
  {
    to_String_X2((int) APN_Password[k]);
    strcat(message_to_server, value_2); 
  }
  to_String_X4(checkSumm);
  strcat(message_to_server,value_4);
  to_String_X2(Footer);
  strcat(message_to_server,value_2);
}

void gen_dataAquisition_onStop_message()
{
  dataLength = 18;
  directionOfData = 0;
  command = 3;
  checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
  
  memset(message_to_server, 0, sizeof(message_to_server));
  to_String_X2(255);
  strcat(message_to_server, value_2);    // Header
  to_String_X4(dataLength);
  strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
  to_String_X2(directionOfData);
  strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
  to_String_X2(parserdate.years);
  strcat(message_to_server, value_2);    // time years
  to_String_X2(parserdate.months);
  strcat(message_to_server, value_2);    // time months
  to_String_X2(parserdate.days);
  strcat(message_to_server, value_2);    // time days
  to_String_X2(parserdate.hours);
  strcat(message_to_server, value_2);    // time hours
  to_String_X2(parserdate.minutes);
  strcat(message_to_server, value_2);    // time minuts
  to_String_X2(parserdate.seconds);
  strcat(message_to_server, value_2);    // time seconds
  to_String_X4(command);
  strcat(message_to_server, value_4);    // command 0x01 connection
  to_String_X4(dataAqOnStopVars[0]);
  strcat(message_to_server,value_4);  // data: on stop minPeriodHome
  to_String_X4(dataAqOnStopVars[1]);
  strcat(message_to_server,value_4);  // data: on stop minPeriodRoaming
  to_String_X4(dataAqOnStopVars[2]);
  strcat(message_to_server,value_4);  // data: on stop minPeriodUnkinoun
  to_String_X4(dataAqOnStopVars[3]);
  strcat(message_to_server,value_4);  // data: on stop minSavedRecordsHome
  to_String_X4(dataAqOnStopVars[4]);
  strcat(message_to_server,value_4);  // data: on stop minSavedRecordsRoaming
  to_String_X4(dataAqOnStopVars[5]);
  strcat(message_to_server,value_4);  // data: on stop minSavedRecordsUnkinoun
  to_String_X4(dataAqOnStopVars[6]);
  strcat(message_to_server,value_4);  // data: on stop sendPeriodHome
  to_String_X4(dataAqOnStopVars[7]);
  strcat(message_to_server,value_4);  // data: on stop sendPeriodRoaming
  to_String_X4(dataAqOnStopVars[8]);
  strcat(message_to_server,value_4);  // data: on stop sendPeriodUnknown
  
  to_String_X4(checkSumm);
  strcat(message_to_server,value_4);
  to_String_X2(Footer);
  strcat(message_to_server,value_2);
}

void gen_dataAquisition_onMoving_message()
{
  dataLength = 36;
  directionOfData = 0;
  command = 4;
  checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
  
  memset(message_to_server, 0, sizeof(message_to_server));
  to_String_X2(255);
  strcat(message_to_server, value_2);    // Header
  to_String_X4(dataLength);
  strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
  to_String_X2(directionOfData);
  strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
  to_String_X2(parserdate.years);
  strcat(message_to_server, value_2);    // time years
  to_String_X2(parserdate.months);
  strcat(message_to_server, value_2);    // time months
  to_String_X2(parserdate.days);
  strcat(message_to_server, value_2);    // time days
  to_String_X2(parserdate.hours);
  strcat(message_to_server, value_2);    // time hours
  to_String_X2(parserdate.minutes);
  strcat(message_to_server, value_2);    // time minuts
  to_String_X2(parserdate.seconds);
  strcat(message_to_server, value_2);    // time seconds
  to_String_X4(command);
  strcat(message_to_server, value_4);    // command 0x01 connection
  to_String_X4(dataAqMovingVars[0]);
  strcat(message_to_server,value_4);  // data: on moving minPeriodHome
  to_String_X4(dataAqMovingVars[1]);
  strcat(message_to_server,value_4);  // data: on moving min Period Roaming
  to_String_X4(dataAqMovingVars[2]);
  strcat(message_to_server,value_4);  // data: on moving min Period Unkinoun
  to_String_X4(dataAqMovingVars[3]);
  strcat(message_to_server,value_4);  // data: on moving min Distance Home
  to_String_X4(dataAqMovingVars[4]);
  strcat(message_to_server,value_4);  // data: on moving min Distance Roaming
  to_String_X4(dataAqMovingVars[5]);
  strcat(message_to_server,value_4);  // data: on moving min Distance Unkinoun
  to_String_X4(dataAqMovingVars[6]);
  strcat(message_to_server,value_4);  // data: on moving min Angle Home
  to_String_X4(dataAqMovingVars[7]);
  strcat(message_to_server,value_4);  // data: on moving min Angle Roaming
  to_String_X4(dataAqMovingVars[8]);
  strcat(message_to_server,value_4);  // data: on moving min Angle Unknown
  to_String_X4(dataAqMovingVars[9]);
  strcat(message_to_server,value_4);  // data: on moving min SpeedDelta Home
  to_String_X4(dataAqMovingVars[10]);
  strcat(message_to_server,value_4);  // data: on moving min SpeedDelta Roaming
  to_String_X4(dataAqMovingVars[11]);
  strcat(message_to_server,value_4);  // data: on moving min SpeedDelta Unknown
  to_String_X4(dataAqMovingVars[12]);
  strcat(message_to_server,value_4);  // data: on moving min SavedRecords Home
  to_String_X4(dataAqMovingVars[13]);
  strcat(message_to_server,value_4);  // data: on moving min SavedRecords Roaming
  to_String_X4(dataAqMovingVars[14]);
  strcat(message_to_server,value_4);  // data: on moving min SavedRecords Unknown
  to_String_X4(dataAqMovingVars[15]);
  strcat(message_to_server,value_4);  // data: on moving Period Home
  to_String_X4(dataAqMovingVars[16]);
  strcat(message_to_server,value_4);  // data: on moving Period Roaming
  to_String_X4(dataAqMovingVars[17]);
  strcat(message_to_server,value_4);  // data: on moving Period Unknown
  
  to_String_X4(checkSumm);
  strcat(message_to_server,value_4);
  to_String_X2(Footer);
  strcat(message_to_server,value_2);
}

void gen_dataAquisition_onDemandTracking_message()
{
  dataLength = 4;
  directionOfData = 0;
  command = 5;
  checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
  
  memset(message_to_server, 0, sizeof(message_to_server));
  to_String_X2(255);
  strcat(message_to_server, value_2);    // Header
  to_String_X4(dataLength);
  strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
  to_String_X2(directionOfData);
  strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
  to_String_X2(parserdate.years);
  strcat(message_to_server, value_2);    // time years
  to_String_X2(parserdate.months);
  strcat(message_to_server, value_2);    // time months
  to_String_X2(parserdate.days);
  strcat(message_to_server, value_2);    // time days
  to_String_X2(parserdate.hours);
  strcat(message_to_server, value_2);    // time hours
  to_String_X2(parserdate.minutes);
  strcat(message_to_server, value_2);    // time minuts
  to_String_X2(parserdate.seconds);
  strcat(message_to_server, value_2);    // time seconds
  to_String_X4(command);
  strcat(message_to_server, value_4);    // command 0x01 connection
  to_String_X4(dataAqOnDemandTrackingVars[0]);
  strcat(message_to_server,value_4);  // data: period
  to_String_X4(dataAqOnDemandTrackingVars[1]);
  strcat(message_to_server,value_4);  // data: duration
  to_String_X4(checkSumm);
  strcat(message_to_server,value_4);
  to_String_X2(Footer);
  strcat(message_to_server,value_2);
}

void gen_mainServerSetting_message()
{
  dataLength = 8;
  directionOfData = 0;
  command = 6;
  checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
  
  memset(message_to_server, 0, sizeof(message_to_server));
  to_String_X2(255);
  strcat(message_to_server, value_2);    // Header
  to_String_X4(dataLength);
  strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
  to_String_X2(directionOfData);
  strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
  to_String_X2(parserdate.years);
  strcat(message_to_server, value_2);    // time years
  to_String_X2(parserdate.months);
  strcat(message_to_server, value_2);    // time months
  to_String_X2(parserdate.days);
  strcat(message_to_server, value_2);    // time days
  to_String_X2(parserdate.hours);
  strcat(message_to_server, value_2);    // time hours
  to_String_X2(parserdate.minutes);
  strcat(message_to_server, value_2);    // time minuts
  to_String_X2(parserdate.seconds);
  strcat(message_to_server, value_2);    // time seconds
  to_String_X4(command);
  strcat(message_to_server, value_4);    // command 0x01 connection
  to_String_X2(mainServerIp[0]);
  strcat(message_to_server,value_2);  // data: domain 1st byte
  to_String_X2(mainServerIp[1]);
  strcat(message_to_server,value_2);  // data: domain 2nd byte
  to_String_X2(mainServerIp[2]);
  strcat(message_to_server,value_2);  // data: domain 3rd byte
  to_String_X2(mainServerIp[3]);
  strcat(message_to_server,value_2);  // data: domain 4th byte
  to_String_X4(mainServerPort);
  strcat(message_to_server,value_4);  // data: port 
  to_String_X2(mainServerProtocol);
  strcat(message_to_server,value_2);  // data: mainServerProtocol 
  to_String_X2(mainServerTlsEncryption);
  strcat(message_to_server,value_2);  // data: mainServerTlsEncryption 
  to_String_X4(checkSumm);
  strcat(message_to_server,value_4);
  to_String_X2(Footer);
  strcat(message_to_server,value_2);
}

void gen_secondServerSetting_message()
{
  dataLength = 9;
  directionOfData = 0;
  command = 7;
  checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
  
  memset(message_to_server, 0, sizeof(message_to_server));
  to_String_X2(255);
  strcat(message_to_server, value_2);    // Header
  to_String_X4(dataLength);
  strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
  to_String_X2(directionOfData);
  strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
  to_String_X2(parserdate.years);
  strcat(message_to_server, value_2);    // time years
  to_String_X2(parserdate.months);
  strcat(message_to_server, value_2);    // time months
  to_String_X2(parserdate.days);
  strcat(message_to_server, value_2);    // time days
  to_String_X2(parserdate.hours);
  strcat(message_to_server, value_2);    // time hours
  to_String_X2(parserdate.minutes);
  strcat(message_to_server, value_2);    // time minuts
  to_String_X2(parserdate.seconds);
  strcat(message_to_server, value_2);    // time seconds
  to_String_X4(command);
  strcat(message_to_server, value_4);    // command 0x01 connection
  to_String_X2(secondServerMode);
  strcat(message_to_server,value_2);  // data: secondServerMode
  to_String_X2(secondServerIp[0]);
  strcat(message_to_server,value_2);  // data: domain 1st byte
  to_String_X2(secondServerIp[1]);
  strcat(message_to_server,value_2);  // data: domain 2nd byte
  to_String_X2(secondServerIp[2]);
  strcat(message_to_server,value_2);  // data: domain 3rd byte
  to_String_X2(secondServerIp[3]);
  strcat(message_to_server,value_2);  // data: domain 4th byte
  to_String_X4(secondServerPort);
  strcat(message_to_server,value_4);  // data: port 
  to_String_X2(secondServerProtocol);
  strcat(message_to_server,value_2);  // data: mainServerProtocol 
  to_String_X2(secondServerTlsEncryption);
  strcat(message_to_server,value_2);  // data: mainServerTlsEncryption 
  to_String_X4(checkSumm);
  strcat(message_to_server,value_4);
  to_String_X2(Footer);
  strcat(message_to_server,value_2);
}

void gen_recordTimeoutSetting_message()
{
  dataLength = 8;
  directionOfData = 0;
  command = 8;
  checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
  
  memset(message_to_server, 0, sizeof(message_to_server));
  to_String_X2(255);
  strcat(message_to_server, value_2);    // Header
  to_String_X4(dataLength);
  strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
  to_String_X2(directionOfData);
  strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
  to_String_X2(parserdate.years);
  strcat(message_to_server, value_2);    // time years
  to_String_X2(parserdate.months);
  strcat(message_to_server, value_2);    // time months
  to_String_X2(parserdate.days);
  strcat(message_to_server, value_2);    // time days
  to_String_X2(parserdate.hours);
  strcat(message_to_server, value_2);    // time hours
  to_String_X2(parserdate.minutes);
  strcat(message_to_server, value_2);    // time minuts
  to_String_X2(parserdate.seconds);
  strcat(message_to_server, value_2);    // time seconds
  to_String_X4(command);
  strcat(message_to_server, value_4);    // command 0x01 connection
  to_String_X4(openLinkTimeout);
  strcat(message_to_server,value_4);  // data: openLinkTimeout
  to_String_X4(responseTimeout);
  strcat(message_to_server,value_4);  // data: responseTimeout
  to_String_X4(networkPingTimeout);
  strcat(message_to_server,value_4);  // data: networkPingTimeout
  to_String_X2(sortBy);
  strcat(message_to_server,value_2);  // data: sortBy 
  to_String_X2(recordTlsEncryption);
  strcat(message_to_server,value_2);  // data: recordTlsEncryption 
  to_String_X4(checkSumm);
  strcat(message_to_server,value_4);
  to_String_X2(Footer);
  strcat(message_to_server,value_2);
}

void gen_IoSetting_message()
{
  dataLength = 11 + (int) ioEventsArray[ioType].sendSmsToLength;
  directionOfData = 0;
  command = 9;
  checkSumm = Header + dataLength + directionOfData + command + parserdate.years + parserdate.months + parserdate.days;
  memset(message_to_server, 0, sizeof(message_to_server));
  to_String_X2(255);
  strcat(message_to_server, value_2);    // Header
  to_String_X4(dataLength);
  strcat(message_to_server, value_4);    // DataLength & data = "SCTR01
  to_String_X2(directionOfData);
  strcat(message_to_server, value_2);    // Direction 0x00 (From Arduino To PC)
  to_String_X2(parserdate.years);
  strcat(message_to_server, value_2);    // time years
  to_String_X2(parserdate.months);
  strcat(message_to_server, value_2);    // time months
  to_String_X2(parserdate.days);
  strcat(message_to_server, value_2);    // time days
  to_String_X2(parserdate.hours);
  strcat(message_to_server, value_2);    // time hours
  to_String_X2(parserdate.minutes);
  strcat(message_to_server, value_2);    // time minuts
  to_String_X2(parserdate.seconds);
  strcat(message_to_server, value_2);    // time seconds
  to_String_X4(command);
  strcat(message_to_server, value_4);    // command 0x01 connection
  to_String_X2(ioType);
  strcat(message_to_server,value_2);  // data: ioType
  to_String_X2(ioEventsArray[ioType].priority);
  strcat(message_to_server,value_2);  // data: priority[ioType]   lowLevel[ioType]
  to_String_X4(ioEventsArray[ioType].lowLevel);
  strcat(message_to_server,value_4); // data: lowLevel[ioType]
  to_String_X4(ioEventsArray[ioType].highLevel);
  strcat(message_to_server,value_4); // data: highLevel[ioType]
  to_String_X2(ioEventsArray[ioType].eventsOnly);
  strcat(message_to_server,value_2); // data: eventsOnly[ioType]
  to_String_X2(ioEventsArray[ioType].operand);
  strcat(message_to_server,value_2); // data: operand[ioType]
  to_String_X4(ioEventsArray[ioType].avgConstant);
  strcat(message_to_server,value_4); // data: avgConstant[ioType]
  to_String_X2((int)ioEventsArray[ioType].sendSmsToLength);
  strcat(message_to_server,value_2); // data: smsMessage length
  for(k=0;k< (int) ioEventsArray[ioType].sendSmsToLength;k++)
  {
    to_String_X2(ioEventsArray[ioType].sendSmsTo[k]);
    strcat(message_to_server,value_2); // data: sendSmsTo
  }
  to_String_X4(checkSumm);
  strcat(message_to_server,value_4);
  to_String_X2(Footer);
  strcat(message_to_server,value_2);
}

void to_String_X2(unsigned int v)
{
  v &= 0xff;
  memset(value_2, 0, sizeof(value_2));
  sprintf(value_2, "%02X", v);
}

void to_String_X4(unsigned int v)
{
  v &= 0xffff;
  memset(value_4, 0, sizeof(value_4));
  sprintf(value_4, "%04X", v);
}

void to_String_X8(unsigned long v)
{
  memset(value_8, 0, sizeof(value_8));
  memset(value_4, 0, sizeof(value_4));
  
  v &= 4294967295UL;
  low = (unsigned int)(v % 65536UL);
  high = (unsigned int)(v / 65536UL);
  
  sprintf(value_4, "%04X", high);
  strcat(value_8, value_4);
  
  memset(value_4, 0, sizeof(value_4));
  sprintf(value_4, "%04X", low);
  strcat(value_8, value_4);
}

void to_String_X16(BigNumber c)
{
  memset(value_16, 0, sizeof(value_16));
  
  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_8) * byte_8;
  sprintf(value_2, "%02X", (long)(c / byte_7));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_7) * byte_7;
  sprintf(value_2, "%02X", (long)(c / byte_6));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_6) * byte_6;
  sprintf(value_2, "%02X", (long)(c / byte_5));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_5) * byte_5;
  sprintf(value_2, "%02X", (long)(c / byte_4));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_4) * byte_4;
  sprintf(value_2, "%02X", (long)(c / byte_3));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_3) * byte_3;
  sprintf(value_2, "%02X", (long)(c / byte_2));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_2) * byte_2;
  sprintf(value_2, "%02X", (long)(c / byte_1));
  strcat(value_16, value_2);

  memset(value_2, 0, sizeof(value_2));
  c -= (c / byte_1) * byte_1;
  sprintf(value_2, "%02X", (long)(c / byte_0));
  strcat(value_16, value_2);
  
}
void gen_Tracker_packet()
{
  memset(value_data, 0, sizeof(value_data));
  to_String_X8(preamble);
  strcat(value_data, value_8);
  to_String_X8(AVL_DATA_LENGTH);
  strcat(value_data, value_8);
  to_String_X2(Codec_id);
  strcat(value_data, value_2);
  to_String_X2(AVL_DATA_COUNT);
  strcat(value_data, value_2);
  // start AVL data
  to_String_X16(now);
  strcat(value_data, value_16);
  to_String_X2(priority);
  strcat(value_data, value_2);
  // GPS
  to_String_X8(longitude);
  strcat(value_data, value_8);
  to_String_X8(latitude);
  strcat(value_data, value_8);
  to_String_X4(altitude);
  strcat(value_data, value_4);
  to_String_X4(angle);
  strcat(value_data, value_4);
  to_String_X2(satellites);
  strcat(value_data, value_2);
  to_String_X4((unsigned int)speed);
  strcat(value_data, value_4);
    // I/O
  to_String_X2(Event_ID);
  strcat(value_data, value_2);
  to_String_X2(Element_Count);
  strcat(value_data, value_2);
  to_String_X2(_1b_Element_Count);
  strcat(value_data, value_2);
  for (i = 0; i < _1b_Element_Count; i++)
  {
    to_String_X2(_1b_DATA[i][0]);
    strcat(value_data, value_2);
    to_String_X2(_1b_DATA[i][1]);
    strcat(value_data,value_2);
  }
  to_String_X2(_2b_Element_Count);
  strcat(value_data, value_2);
  for (i = 0; i < _2b_Element_Count; i++)
  {
    to_String_X2(_2b_DATA[i][0]);
    strcat(value_data, value_2);
    to_String_X4(_2b_DATA[i][1]);
    strcat(value_data,value_4);
  }
  to_String_X2(_4b_Element_Count);
  strcat(value_data, value_2);
  for (i = 0; i < _4b_Element_Count; i++)
  {
    to_String_X2(_4b_DATA[i][0]);
    strcat(value_data, value_2);
    to_String_X8(_4b_DATA[i][1]);
    strcat(value_data,value_8);
  }
  to_String_X2(_8b_Element_Count);
  strcat(value_data, value_2);
  for (i = 0; i < _8b_Element_Count; i++)
  {
    to_String_X2(_8b_sensor_id[i]);
    strcat(value_data, value_2);
    to_String_X16(_8b_DATA[i]);
    strcat(value_data,value_16);
  }
  // end AVL data
  to_String_X2(AVL_DATA_COUNT);
  strcat(value_data, value_2);
  to_String_X8(Crc);
  strcat(value_data, value_8);

  strcat(value_data, "\0");
}

char convertCharToHex(char ch)
{
  char returnType;
  switch(ch)
  {
    case '0':
    returnType = 0;
    break;
    case  '1' :
    returnType = 1;
    break;
    case  '2':
    returnType = 2;
    break;
    case  '3':
    returnType = 3;
    break;
    case  '4' :
    returnType = 4;
    break;
    case  '5':
    returnType = 5;
    break;
    case  '6':
    returnType = 6;
    break;
    case  '7':
    returnType = 7;
    break;
    case  '8':
    returnType = 8;
    break;
    case  '9':
    returnType = 9;
    break;
    case  'A':
    returnType = 10;
    break;
    case  'B':
    returnType = 11;
    break;
    case  'C':
    returnType = 12;
    break;
    case  'D':
    returnType = 13;
    break;
    case  'E':
    returnType = 14;
    break;
    case  'F' :
    returnType = 15;
    break;
    default:
    returnType = 0;
    break;
  }
  return returnType;
}

void stringToCharArray(char* hex, int len)
{
    s = 0;
    for (k = 0; k < len; k += 2) {
        hex[s] = (byte) (convertCharToHex(hex[k]) << 4 | convertCharToHex(hex[k+1]));
        s++;
    }
     hex[s] = 0;
}

void setIMEI()
{
  IMEI[0] = 0x00;
  IMEI[1] = 0x0F;
  IMEI[2] = 0x33;
  IMEI[3] = 0x35;
  IMEI[4] = 0x37;
  IMEI[5] = 0x39; // 0x31
  IMEI[6] = 0x35;
  IMEI[7] = 0x34;
  IMEI[8] = 0x30;
  IMEI[9] = 0x38;
  IMEI[10] = 0x39;
  IMEI[11] = 0x32;
  IMEI[12] = 0x37;
  IMEI[13] = 0x37;
  IMEI[14] = 0x31;
  IMEI[15] = 0x34;
  IMEI[16] = 0x36;
}

void sendToServer()
{
  sendCompleted = false;
  if(gprsEnabled == false)
  {
    modem.enableGPRS(true);
    gprsEnabled = true;
  }
  TIMSK5 |= (1 << OCIE1A);
  connectionInteruot = true;
  connectionTimeout = 0;
  while(modem.TCPconnect(IpFormatted, mainServerPort) == false)
  {
    if(breakToCheckConf == true)
    {
      breakToCheckConf = false;
      Serial.println("will close now");
      return;
    }
  }
  connectionInteruot = false;
  connectionTimeout = 0;
  TIMSK5 &= ~(1 << OCIE1A);
  //Serial.println("connected");
  iRet = modem.TCPsend(IMEI, 17);
  receivData();
  if(recv[0] == 0x31 || (int)recv[0] == 1)
  {
    //Serial.println("rece IMEI");
    modem.TCPsend(value_data, 155);
    receivData();
    sendCompleted = true;
    //Serial.println("Send Completed");
    //Serial3.println("AT+CIPCLOSE");
  }
}

void receivData()
{
  lastRead = millis();
  while (millis() - lastRead < 1500){
    while (modem.TCPavailable()){
      r = modem.TCPread(recv, 20);
      recv[r] = 0;
      lastRead = millis();
    }
  }
   //print_receivedSig();
}

void print_receivedSig()
{
  for(i = 0; i<r; i++)
  {
    if(recv[i] == 0)
    {
      recv[i] = 0x30;
    }
  }
  //Serial.println((char *) recv);
}

void simRegistration()
{
  // set SIM800L
  Serial3.begin(9600);
  //Begin serial communication with Arduino and SIM800L
  //Serial.println(F("Initializing modem... (May take a few seconds)"));
  if (! modem.begin(Serial3)) {
    //Serial.println(F("Couldn't find modem"));
    while(1);
  }
  //Serial.print(F("Checking for Cell network..."));
  while (modem.getNetworkStatus() != 1)
  {
    Serial3.println("AT+CPIN=\"0000\"");
  }
  modem.setGPRSNetworkSettings((__FlashStringHelper *)APN);  // set APN
  setIMEI();
  delay(3000);
  //Serial.println(F("Registered..."));
  data_timer_counter = 0;
}

void setup() 
{
  digitalWrite(resetPin, HIGH);
  delay(200);
  // initialize the digital pin as an output.
  // put your setup code here, to run once:
  BigNumber::begin();
  Serial.begin(115200);
  getDataFromEprom();
  printEpromData();
  breakToCheckConf   = false;
  configuration_mode = false;
  checkConfigConnection = false;
  configConnectTimeout = 0;

  simRegistration();
  
  // input output pin defined
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(charge_battery_in, INPUT);
  pinMode(cutoff_command_in, INPUT);
  pinMode(cutoff_engine, OUTPUT);
  pinMode(redPin, OUTPUT);
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(charge_battery_in, LOW);
  digitalWrite(cutoff_command_in, LOW);
  digitalWrite(cutoff_engine, LOW);
  digitalWrite(redPin, LOW);
  pinMode(resetPin, OUTPUT);  
  
  // GPS init
  GPS.begin(9600);
  while (!SD.begin(53))
  {
    //Serial.println("initialization SD Card failed!");
  }
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  connectionInteruot = false;
  connectionTimeout = 0;
  // Timer 1 init
  cli();                   //Disable interrupts while setting registers       
  TCCR1A = 0;              // Make sure it is zero
  TCCR1B = (1 << WGM12);   // Configure for CTC mode (Set it; don't OR stuff into it)
  TCCR1B |= ((1 << CS10) | (1 << CS12)); // Prescaler @ 1024
  TIMSK1 = (1 << OCIE1A);  // Enable interrupt
  OCR1A = 15624;           // compare value = 1 sec (16MHz AVR)

 // Timer 5 init
  //cli();                   //Disable interrupts while setting registers       
  TCCR5A = 0;              // Make sure it is zero
  TCCR5B = (1 << WGM12);   // Configure for CTC mode (Set it; don't OR stuff into it)
  TCCR5B |= ((1 << CS10) | (1 << CS12)); // Prescaler @ 1024
  TIMSK5 = (1 << OCIE1A);  // Enable interrupt
  OCR5A = 15624;           // compare value = 1 sec (16MHz AVR)

  // Timer 3 init
  TCCR3A = 0;              // Make sure it is zero
  TCCR3B = (1 << WGM12);   // Configure for CTC mode (Set it; don't OR stuff into it)
  TCCR3B |= ((1 << CS10) | (1 << CS12)); // Prescaler @ 1024
  TIMSK3 = (1 << OCIE1A);  // Enable interrupt
  OCR3A = 15624;       // compare value = 1 sec (16MHz AVR)
  
  sei();

  
  
}

void serialEvent() {
    r = 0;
    //memset(message_to_server,0,sizeof(message_to_server));
    while (Serial.available()) {
      if(configuration_mode == false)
      {
        //getDataFromEprom();
        configuration_mode = true;
        digitalWrite(redPin, HIGH);
        break;
      }
      c = Serial.read();
      message_to_server[r] = c;
      r++;
      //Serial.print(c);
      //Serial.println('*');
    }
    if(configuration_mode == true)
    {
      lastHeartBeat = micros();
      parsercommand = ((int) message_to_server[11]) + ((int) message_to_server[10])*256;
      parseMessage();
      //Serial.write(parsercommand);
      if(parsercommand == 1)
      {
        heartBeatBegin = true;
        configConnectTimeout = 0;
        gen_connection_message();
        stringToCharArray(message_to_server, 2*(parserdataLength + 16));
        Serial.write(message_to_server, dataLength + 15);
      }
      else if(parsercommand == 2)
      {
        memset(APN, 0, sizeof(APN));
        memset(APN_Username, 0, sizeof(APN_Username));
        memset(APN_Password, 0, sizeof(APN_Password)); 
        gprsContext = (byte) message_to_server[12];
        gprsAuthentication = (byte) message_to_server[13];
        APNLen = (byte) message_to_server[14];
        APN_UsernameLen = (byte) message_to_server[15];
        APN_PasswordLen = (byte) message_to_server[16];
        strncat(APN,message_to_server + 17,(int) APNLen);
        strncat(APN_Username,message_to_server + 17 + (int) APNLen,(int) APN_UsernameLen);
        strncat(APN_Password,message_to_server + 17 + (int) APNLen + (int) APN_UsernameLen,(int) APN_PasswordLen);
        //Serial.println(APN);
        //Serial.println(APN_Username);
        //Serial.println(APN_Password);
        genApnSettingMessageToServer();
        EEPROM.put(ApnAddress, APN);
        EEPROM.put(ApnUsernameAddress, APN_Username);
        EEPROM.put(ApnPasswordAddress, APN_Password);
        stringToCharArray(message_to_server, 2*(dataLength + 15));
        Serial.write(message_to_server, dataLength + 15);
      }
      else if(parsercommand == 3)
      {
        // dataAqOnStopVars
        dataAqOnStopVars[0] = (byte) message_to_server[13] + 256*((byte) message_to_server[12]);
        dataAqOnStopVars[1] = (byte) message_to_server[15] + 256*((byte) message_to_server[14]);
        dataAqOnStopVars[2] = (byte) message_to_server[17] + 256*((byte) message_to_server[16]);
        dataAqOnStopVars[3] = (byte) message_to_server[19] + 256*((byte) message_to_server[18]);
        dataAqOnStopVars[4] = (byte) message_to_server[21] + 256*((byte) message_to_server[20]);
        dataAqOnStopVars[5] = (byte) message_to_server[23] + 256*((byte) message_to_server[22]);
        dataAqOnStopVars[6] = (byte) message_to_server[25] + 256*((byte) message_to_server[24]);
        dataAqOnStopVars[7] = (byte) message_to_server[27] + 256*((byte) message_to_server[26]);
        dataAqOnStopVars[8] = (byte) message_to_server[29] + 256*((byte) message_to_server[28]);
        gen_dataAquisition_onStop_message();
        stringToCharArray(message_to_server, 2*(dataLength + 15));
        Serial.write(message_to_server, dataLength + 15);
        //gen_connection_message();
      }
      else if(parsercommand == 4)
      {
        //dataAqMovingVars
        dataAqMovingVars[0] = (byte) message_to_server[13] + 256*((byte) message_to_server[12]);
        dataAqMovingVars[1] = (byte) message_to_server[15] + 256*((byte) message_to_server[14]);
        dataAqMovingVars[2] = (byte) message_to_server[17] + 256*((byte) message_to_server[16]);
        dataAqMovingVars[3] = (byte) message_to_server[19] + 256*((byte) message_to_server[18]);
        dataAqMovingVars[4] = (byte) message_to_server[21] + 256*((byte) message_to_server[20]);
        dataAqMovingVars[5] = (byte) message_to_server[23] + 256*((byte) message_to_server[22]);
        dataAqMovingVars[6] = (byte) message_to_server[25] + 256*((byte) message_to_server[24]);
        dataAqMovingVars[7] = (byte) message_to_server[27] + 256*((byte) message_to_server[26]);
        dataAqMovingVars[8] = (byte) message_to_server[29] + 256*((byte) message_to_server[28]);
        dataAqMovingVars[9] = (byte) message_to_server[31] + 256*((byte) message_to_server[30]);
        dataAqMovingVars[10] = (byte) message_to_server[33] + 256*((byte) message_to_server[32]);
        dataAqMovingVars[11] = (byte) message_to_server[35] + 256*((byte) message_to_server[34]);
        dataAqMovingVars[12] = (byte) message_to_server[37] + 256*((byte) message_to_server[36]);
        dataAqMovingVars[13] = (byte) message_to_server[39] + 256*((byte) message_to_server[38]);
        dataAqMovingVars[14] = (byte) message_to_server[41] + 256*((byte) message_to_server[40]);
        dataAqMovingVars[15] = (byte) message_to_server[43] + 256*((byte) message_to_server[42]);
        dataAqMovingVars[16] = (byte) message_to_server[45] + 256*((byte) message_to_server[44]);
        dataAqMovingVars[17] = (byte) message_to_server[47] + 256*((byte) message_to_server[46]);
    
        gen_dataAquisition_onMoving_message();
        stringToCharArray(message_to_server, 2*(dataLength + 15));
        Serial.write(message_to_server, dataLength + 15);
      }
      else if(parsercommand == 5)
      {
        //dataAqOnDemandTrackingVars
        dataAqOnDemandTrackingVars[0] = (byte) message_to_server[13] + 256*((byte) message_to_server[12]);
        dataAqOnDemandTrackingVars[1] = (byte) message_to_server[15] + 256*((byte) message_to_server[14]);
    
        gen_dataAquisition_onDemandTracking_message();
        stringToCharArray(message_to_server, 2*(dataLength + 15));
        Serial.write(message_to_server, dataLength + 15);
      }
      else if(parsercommand == 6)
      {
        memset(mainServerIp,0,sizeof(mainServerIp));
        for(k = 0; k < 4; k++)
        {
          mainServerIp[k] = message_to_server[12 + k];
        }
        mainServerPort          = (byte) message_to_server[17] + 256*((byte) message_to_server[16]);
        mainServerProtocol      = (byte) message_to_server[18];
        mainServerTlsEncryption = (byte) message_to_server[19];
        //Serial.write(mainServerIp);
        gen_mainServerSetting_message();
        EEPROM.put(mainServerIpAddress, mainServerIp);
        EEPROM.put(mainServerPortAddress, mainServerPort);
        EEPROM.put(mainServerProtocolAddress, mainServerProtocol);
        EEPROM.put(mainServerTlsEncryptionAddress, mainServerTlsEncryption);
        stringToCharArray(message_to_server, 2*(dataLength + 15));
        Serial.write(message_to_server, dataLength + 15);
      }
      else if(parsercommand == 7)
      {
        memset(secondServerIp,0,sizeof(secondServerIp));
        secondServerMode      = (byte) message_to_server[12];
        for(k = 0; k < 4; k++)
        {
          secondServerIp[k] = message_to_server[13 + k];
        }
        secondServerPort          = (byte) message_to_server[18] + 256*((byte) message_to_server[17]);
        secondServerProtocol      = (byte) message_to_server[19];
        secondServerTlsEncryption = (byte) message_to_server[20];
        gen_secondServerSetting_message();
        stringToCharArray(message_to_server, 2*(dataLength + 15));
        Serial.write(message_to_server, dataLength + 15);
      }
      else if(parsercommand == 8)
      {
        openLinkTimeout          = (byte) message_to_server[13] + 256*((byte) message_to_server[12]);
        responseTimeout          = (byte) message_to_server[15] + 256*((byte) message_to_server[14]);
        networkPingTimeout          = (byte) message_to_server[17] + 256*((byte) message_to_server[16]);
        sortBy                   = (byte) message_to_server[18];
        recordTlsEncryption      = (byte) message_to_server[19];
        gen_recordTimeoutSetting_message();
        stringToCharArray(message_to_server, 2*(dataLength + 15));
        Serial.write(message_to_server, dataLength + 15);
        
      }
      else if(parsercommand == 9)
      {
        ioType = (byte) message_to_server[12] + (1 - (((byte) message_to_server[12]) >=0 ))*256;
        ioEventsArray[ioType].priority     = (byte) message_to_server[13];
        ioEventsArray[ioType].lowLevel     = (byte) message_to_server[15] + 256*((byte) message_to_server[14]);
        ioEventsArray[ioType].highLevel       = (byte) message_to_server[17] + 256*((byte) message_to_server[16]);
        ioEventsArray[ioType].eventsOnly      = (byte) message_to_server[18];
        ioEventsArray[ioType].operand         = (byte) message_to_server[19];
        ioEventsArray[ioType].avgConstant     = (byte) message_to_server[21] + 256*((byte) message_to_server[20]);
        ioEventsArray[ioType].sendSmsToLength = (byte) message_to_server[22];
        for(k=0;k<(byte) ioEventsArray[ioType].sendSmsToLength;k++)
        {
          ioEventsArray[ioType].sendSmsTo[k] = message_to_server[23 + k];
        }
        gen_IoSetting_message();
        EEPROM.put(IoStructureArrayAddress, ioEventsArray);
        stringToCharArray(message_to_server, 2*(dataLength + 15));
        Serial.write(message_to_server, dataLength + 15);
      }
      else if(parsercommand == 2049)
      {
        infoType = (byte) message_to_server[12] + (1 - (((byte) message_to_server[12]) >=0 ))*256;
        varIndex = (byte) message_to_server[13];
        if(infoType == 2)
        {
          //EEPROM.get(ApnAddress, APN);
          //EEPROM.get(ApnUsernameAddress, APN_Username);
          //EEPROM.get(ApnPasswordAddress, APN_Password);
          APNLen = strlen(APN);
          APN_UsernameLen = strlen(APN_Username);
          APN_PasswordLen = strlen(APN_Password);
          //Serial.println(APN);
          //Serial.println(APN_Username);
          //Serial.println(APN_Password);
          //Serial.println(APNLen);
          //Serial.println(APN_UsernameLen);
          //Serial.println(APN_PasswordLen);
          genApnSettingMessageToServer();
          stringToCharArray(message_to_server, 2*(dataLength + 15));
          Serial.write(message_to_server, dataLength + 15);
        }
        else if(infoType == 3)
        {
          gen_dataAquisition_onStop_message();
          stringToCharArray(message_to_server, 2*(dataLength + 15));
          Serial.write(message_to_server, dataLength + 15);
        }
        else if(infoType == 4)
        {
          gen_dataAquisition_onMoving_message();
          stringToCharArray(message_to_server, 2*(dataLength + 15));
          Serial.write(message_to_server, dataLength + 15);
        }
        else if(infoType == 5)
        {
          gen_dataAquisition_onDemandTracking_message();
          stringToCharArray(message_to_server, 2*(dataLength + 15));
          Serial.write(message_to_server, dataLength + 15);
        }
        else if(infoType == 6)
        {
          gen_mainServerSetting_message();
          stringToCharArray(message_to_server, 2*(dataLength + 15));
          Serial.write(message_to_server, dataLength + 15);
        }
        else if(infoType == 7)
        {
          gen_secondServerSetting_message();
          stringToCharArray(message_to_server, 2*(dataLength + 15));
          Serial.write(message_to_server, dataLength + 15);
        }
        else if(infoType == 8)
        {
          gen_recordTimeoutSetting_message();
          stringToCharArray(message_to_server, 2*(dataLength + 15));
          Serial.write(message_to_server, dataLength + 15);
        }
        else if(infoType == 9)
        {
          ioType = varIndex;
          gen_IoSetting_message();
          stringToCharArray(message_to_server, 2*(dataLength + 15));
          Serial.write(message_to_server, dataLength + 15);
        }
        else
        {
          gen_error_infoType_onCalling_VarValue();
          stringToCharArray(message_to_server, 2*(dataLength + 15));
          Serial.write(message_to_server, dataLength + 15);
        }
      }
      else
      {
        gen_error_commandType_onUpdatingVars();
        stringToCharArray(message_to_server, 2*(dataLength + 15));
        Serial.write(message_to_server, dataLength + 15);
      }
      //handle_received_message_from_configurator();
      
      Serial.flush();     
    } 
}

void loop() 
{
  // put your main code here, to run repeatedly:
  
  if(configuration_mode == false)
  {
    int i = 0;
    char c = GPS.read();
    if (GPS.newNMEAreceived()) 
    {
      //Serial.print(GPS.lastNMEA());
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    if (data_timer_counter == 1) 
    {
      TIMSK5 &= ~(1 << OCIE1A);
      connectionInteruot = true;
      TIMSK1 &= ~(1 << OCIE1A);
      delay(100);
      if(GPS.fix) // fixedForFirstTime == true
      {
        
        //Serial.println("first time fixed");
        TIMSK3 &= ~(1 << OCIE1A);
        //Serial.println("Timer running ...");
        //Serial.println("function ...");
        gen_Tracker_packet();
        stringToCharArray(value_data, 310);
        //Serial.write(value_data,310);
        timeBegin = micros();
        sendToServer();
        timeEnd = micros();
        duration = timeEnd - timeBegin;
        averageDuration = (double)duration / 1000.0;
        //Serial.println(averageDuration);
        data_timer_counter = 2;
        delay(8000 - averageDuration > 0 ? 8000 - averageDuration : 0);
        TIMSK3 |= (1 << OCIE1A);
      }
      else
      {
        //Serial.println("not fixed");
        data_timer_counter = 2;
      }
      TIMSK1 |= (1 << OCIE1A);
      delay(100);
      //Serial.println("end of loop");
    }
  }
  else
  {
    delay(100);
    if(heartBeatBegin == true)
    {
      timeAfterHeartBeat = micros();
      
      if(timeAfterHeartBeat > lastHeartBeat)
      {
        if(timeAfterHeartBeat - lastHeartBeat > 3500000)
        {
          Serial.println("reset now");
          digitalWrite(resetPin, LOW);
        }
      }
    }
  }
}


ISR(TIMER1_COMPA_vect) 
{
  if(configuration_mode == false)
  {
    year = GPS.year;
    mon = GPS.month;
    mday = GPS.day;
    hour = GPS.hour;
    min = GPS.minute;
    sec = GPS.seconds;
    //GPS.milliseconds;
    
    now = (unsigned int)(stamp.timestamp(year, mon, mday, hour, min, sec) / 65536);
    now *= byte_2;
    timeAcc = (unsigned int)(stamp.timestamp(year, mon, mday, hour, min, sec) % 65536);
    timeAccBig = timeAcc/256;
    timeAccBig *= byte_1;
    timeAccBig += timeAcc % 256;
    now += timeAccBig;
    now *= 1000;
    
    if (GPS.fix) 
    {
      
      longitude = (GPS.longitudeDegrees) * 10000000.0f;
      latitude = (GPS.latitudeDegrees) * 10000000.0f;
      altitude = GPS.altitude;
      angle = GPS.angle;
      satellites = GPS.satellites;
      speed = GPS.speed * 1.852f;
      HDOP = GPS.HDOP;
      VDOP = GPS.VDOP;
      PDOP = GPS.PDOP;
  
      longitude_Backup = longitude;
      latitude_Backup = latitude;
      altitude_Backup = altitude;
      angle_Backup = angle;
      satellites_Backup = satellites;
      speed_Backup = speed;
      HDOP_Backup = HDOP;
      VDOP_Backup = VDOP;
      PDOP_Backup = PDOP;
    }
  
    external_voltage_in = analogRead(A0);
    delay(50);
    internal_voltage_in = analogRead(A1);
    delay(50);
    car_is_working_in = analogRead(A2);
    delay(50);
    fuel_sensor_in = analogRead(A3);
    external_voltage = (int)((external_voltage_in * (5.0 / 1023.0)) * 3000);
    internal_voltage = (int)((internal_voltage_in * (5.0 / 1023.0)) * 1000);
    fuel_sensor = (int)((fuel_sensor_in * (5.0 / 1023.0)) * 16);
    charge_battery = digitalRead(charge_battery_in);
    if((int)(car_is_working_in * (5.0 / 1023.0)) > 2)
    {
      car_is_working = 1;
    }
    else
    {
      car_is_working = 0;
    }
    cutoff_command = digitalRead(cutoff_command_in);
    if(cutoff_command == 1)
    {
      digitalWrite(cutoff_engine, HIGH);
    }
    else
    {
      digitalWrite(cutoff_engine, LOW);
    }
    
    preamble = 0;
    AVL_DATA_LENGTH = 1263;
    Codec_id = 8;
    AVL_DATA_COUNT = 1;
    now;
    priority = 2;
    // io
    Event_ID = 9;
    Element_Count = 30;
    // == 1b ==
    _1b_Element_Count = 8;
    if(ioEventsArray[0].priority > 0)
    {
      _1b_DATA[0][0] = 239;
      _1b_DATA[0][1] = car_is_working;
    }
    else
    {
      _1b_DATA[0][0] = 239;
      _1b_DATA[0][1] = 0;
    }
    _1b_DATA[1][0] = 240;
    _1b_DATA[1][1] = 0;
    _1b_DATA[2][0] = 80;
    _1b_DATA[2][1] = 5;
    _1b_DATA[3][0] = 21;
    _1b_DATA[3][1] = 0;
    _1b_DATA[4][0] = 200;
    _1b_DATA[4][1] = 0;
    _1b_DATA[5][0] = 69;
    _1b_DATA[5][1] = 1;
    _1b_DATA[6][0] = 179;
    _1b_DATA[6][1] = 0;
    _1b_DATA[7][0] = 113;
    _1b_DATA[7][1] = 89;
    // == 2b ==
    _2b_Element_Count = 14;
    _2b_DATA[0][0] = 181;
    _2b_DATA[0][1] = 1;
    _2b_DATA[1][0] = 182;
    _2b_DATA[1][1] = 1;
    if(ioEventsArray[24].priority > 0)
    {
      _2b_DATA[2][0] = 66;
      _2b_DATA[2][1] = external_voltage;
    }
    else
    {
      _2b_DATA[2][0] = 66;
      _2b_DATA[2][1] = 0;
    }
    _2b_DATA[3][0] = 24;
    _2b_DATA[3][1] = 0;
    _2b_DATA[4][0] = 205;
    _2b_DATA[4][1] = 0;
    _2b_DATA[5][0] = 206;
    _2b_DATA[5][1] = 0;
    if(ioEventsArray[13].priority > 0)
    {
      _2b_DATA[6][0] = 67;
      _2b_DATA[6][1] = internal_voltage;
    }
    else
    {
      _2b_DATA[6][0] = 67;
      _2b_DATA[6][1] = 0;
    }
    _2b_DATA[7][0] = 68;
    _2b_DATA[7][1] = 141;
    _2b_DATA[8][0] = 9;
    _2b_DATA[8][1] = 218;
    _2b_DATA[9][0] = 13;
    _2b_DATA[9][1] = 128;
    _2b_DATA[10][0] = 17;
    _2b_DATA[10][1] = 68;
    _2b_DATA[11][0] = 18;
    _2b_DATA[11][1] = 8;
    _2b_DATA[12][0] = 19;
    _2b_DATA[12][1] = 64578;
    _2b_DATA[13][0] = 15;
    _2b_DATA[13][1] = 0;
    // == 4b ==
    _4b_Element_Count = 5;
    _4b_DATA[0][0] = 241;
    _4b_DATA[0][1] = 0;
    _4b_DATA[1][0] = 199;
    _4b_DATA[1][1] = 0;
    _4b_DATA[2][0] = 16;
    _4b_DATA[2][1] = 1191;
    _4b_DATA[3][0] = 12;
    _4b_DATA[3][1] = 125019UL;
    _4b_DATA[4][0] = 4;
    _4b_DATA[4][1] = 0;
    // == 8b ==
    _8b_Element_Count = 3;
    _8b_sensor_id[0] = 11;
    _8b_DATA[0] = 0;
    _8b_sensor_id[1] = 238;
    _8b_DATA[1] = 0;
    _8b_sensor_id[2] = 14;
    _8b_DATA[2] = 0;
    // Crc
    Crc = 51151UL;
    //Serial.println("inside ISR");
    
  }
  
}

ISR(TIMER3_COMPA_vect) 
{
      if(configuration_mode == false)
      {
        //Serial.println(data_timer_counter);
        if(data_timer_counter >= 3)
        {
          //sdWriting = true;
          TIMSK3 &= ~(1 << OCIE1A);
          if(GPS.fix)
          {
            
            log_file = SD.open(log_file_name, FILE_WRITE);
            //log_file.println("date: " + String(year) + "/" + String(mon) + "/" + String(mday) + " " + String(hour) + "/" + String(min) + "/" + String(sec));
            log_file.println("data: ");
            log_file.println(value_data);
            log_file.close();
            //Serial.println("saved to SD card");
            //sdWriting = false;
            
          }
          TIMSK3 |= (1 << OCIE1A);
          data_timer_counter = 0;
          
        }
        else
        {
          data_timer_counter++;
        }
      }
      
      
}

ISR(TIMER5_COMPA_vect) 
{
  
  //Serial.println(connectionTimeout);
  if(connectionInteruot == true)
  {
    //Serial.println(connectionTimeout);
    if(connectionTimeout > 15)
    {
      breakToCheckConf = true;
    }
    if(connectionTimeout > 60)
    {
      digitalWrite(resetPin, LOW);
    }
    else
    {
      connectionTimeout++;
    }
  }
  
}
