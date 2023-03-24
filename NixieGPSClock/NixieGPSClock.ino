#include <SoftwareSerial.h>
#include <EEPROM.h>

const uint32_t COMMAND_DELAY = 250;

SoftwareSerial gpsPort(11, 10); // RX, TX
SoftwareSerial debugPort(4, 5); // RX, TX

//
static const int pinData = 5;
static const int pinClk  = 25;
static const int pinLatch = 22;

char hoursHI[] = { 9,  2, 10, 14,  6,  4, 12,  8,  0,  1,  9,  9,  9,  9,  9,  9};
char hoursLO[] = { 3,  4,  5, 13, 12,  8,  9,  1,  0,  2,  3,  3,  3,  3,  3,  3};

char minutesHI[] = { 3,  4,  5, 13, 12,  8,  9,  1,  0,  2,  3,  3,  3,  3,  3,  3};
char minutesLO[] = { 3,  4,  5, 13, 12,  8,  9,  1,  0,  2,  3,  3,  3,  3,  3,  3};

char secondsHI[] = { 3,  4,  5, 13, 12,  8,  9,  1,  0,  2,  3,  3,  3,  3,  3,  3};
char secondsLO[] = { 3,  4,  5, 13, 12,  8,  9,  1,  0,  2,  3,  3,  3,  3,  3,  3};
//

static int globalYear = 0;
static int globalMonth = 0;
static int globalDay = 0;
static int globalHours = 0;
static int globalMinutes = 0;
static int globalSeconds = 0;

//Netherlands
static int UTC_OFFSET = 1;
static int isDaylightSavingTime = 0;

const int eepromAddrForUtcOffset = 0;

//Daylight saving time functions
int DayOfWeek(int day, int month, int year) {
  int a = (14 - month) / 12;
  int y = year - a;
  int m = month + 12 * a - 2;
  return (7000 + (day + y + y / 4 - y / 100 + y / 400 + (31 * m) / 12)) % 7;
}

int IsLeapYear (int year) {
  if ((year & 3) == 0 && ((year % 25) != 0 || (year & 15) == 0)) {
    return 1;
  } else {
    return 0;
  }
}

static int MonthDay(int Month, int Year) {
  if(Month == 1) {
    return 31;
  } else if (Month == 2 && IsLeapYear(Year) == 1) {
    return 29;
  } else if (Month == 2 && IsLeapYear(Year) == 0) {
    return 28;
  } else if (Month == 3) {
    return 31;
  } else if (Month == 4) {
    return 30;
  } else if (Month == 5) {
    return 31;
  } else if (Month == 6) {
    return 30;
  } else if (Month == 7) {
    return 31;
  } else if (Month == 8) {
    return 31;
  } else if (Month == 9) {
    return 30;
  } else if (Month == 10) {
    return 31;
  } else if (Month == 11) {
    return 30;
  } else if (Month == 12) {
    return 31;
  } else return 0;
}

void correctSummertime() {
  int summertimeOffset = 0;

  //Last Sunday of March
  int M = 31 - DayOfWeek(31,3,globalYear);
  //Last Sunday of October
  int O = 31 - DayOfWeek(31,10,globalYear);

  if (((globalMonth > 3) && (globalMonth < 10)) || 
      ((globalMonth == 3) && (globalDay >= M)) || 
      ((globalMonth == 10) && (globalDay <= O))) {

      isDaylightSavingTime = 1;

      if ((globalMonth == 3) && (globalDay == M) && globalHours < 3 ) {
        isDaylightSavingTime = 0;
      }

      if ((globalMonth == 10) && (globalDay >= O) && globalHours >=3 ) {
        isDaylightSavingTime = 0;
      }
      
      if (isDaylightSavingTime > 0) {
        summertimeOffset = 1;
      }
  }

  if ((globalHours + summertimeOffset) < 24) {
    globalHours = globalHours + summertimeOffset;
  } else {
    globalHours = globalHours + summertimeOffset - 24;
    if ((globalDay + 1) <= MonthDay(globalMonth, globalYear)) {
      globalDay = globalDay + 1;
    } else {
      globalDay = 1;
      if ((globalMonth + 1) <= 12) {
        globalMonth = globalMonth + 1;
      } else {
        globalMonth = 1;
        globalYear = globalYear + 1;
      }
    }
  }
}

void correctTimezone() {
  int zone = UTC_OFFSET;
  
  if ((globalHours + zone) < 24) {
    globalHours = globalHours + zone;
  } else {
    globalHours = globalHours + zone - 24;
    if ((globalDay + 1) <= MonthDay(globalMonth, globalYear)) {
      globalDay = globalDay + 1;
    } else {
      globalDay = 1;
      if ((globalMonth + 1) <= 12) {
        globalMonth = globalMonth + 1;
      } else {
        globalMonth = 1;
        globalYear = globalYear + 1;
      }
    }
  }
}
//End of Daylight saving time functions

const unsigned char ubxAntennaConfig[] PROGMEM =
{ 0x06, 0x13, 0x04, 0x00, 0x18, 0x00, 0xf0, 0x7d };

const unsigned char ubxAntennaConfig2[] PROGMEM =
{ 0x06, 0x13, 0x00, 0x00 };

const unsigned char ubxTimeAndDateInvalidValuesEnable[] PROGMEM =
{ 0x06, 0x17, 0x04, 0x00, 0x0c, 0x40, 0x00, 0x02 };

const unsigned char ubxTimeAndDateInvalidValuesEnable2[] PROGMEM =
{ 0x06, 0x17, 0x00, 0x00 };

const unsigned char ubxEnableZDA[] PROGMEM =
{ 0x06, 0x01, 0x03, 0x00, 0xf0, 0x08, 0x01 };

const unsigned char ubxEnableDebug[] PROGMEM =
{ 0x06, 0x01, 0x03, 0x00, 0x0c, 0x10, 0x01 };

void sendUBX( const unsigned char *progmemBytes, size_t len )
{
  gpsPort.write( 0xB5 ); // SYNC1
  gpsPort.write( 0x62 ); // SYNC2

  uint8_t a = 0, b = 0;
  while (len-- > 0) {
    uint8_t c = pgm_read_byte( progmemBytes++ );
    a += c;
    b += a;
    gpsPort.write( c );
  }

  gpsPort.write( a ); // CHECKSUM A
  gpsPort.write( b ); // CHECKSUM B

} // sendUBX

static char buf[80];
static byte currentIndex = 0;

void validateUtcOffset() {
  if (UTC_OFFSET >= 24) {
    UTC_OFFSET = 24;
  }

  if (UTC_OFFSET <= 0) {
    UTC_OFFSET = 0;
  }
}

void firstTest() {
  digitalWrite(0, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(50);
  digitalWrite(1, HIGH);
  delay(50);

  //First test
  for(int i = 0; i < 10; i++) {
    globalHours = i* 10 + i;
    globalMinutes = i* 10 + i;
    globalSeconds = i* 10 + i;
    showTime();
    delay(100);
  }
  
  digitalWrite(0, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(50);
  digitalWrite(1, LOW);
  delay(50);
}

void setup() {
  // set the data rate for the SoftwareSerial port
  // debugPort.begin( 38400 );
  // debugPort.println("Hello, world?");

  //EEPROM.get(eepromAddrForUtcOffset, UTC_OFFSET);
  //validateUtcOffset();

  gpsPort.begin( 9600 );
  delay( COMMAND_DELAY );

  //sendUBX( ubxAntennaConfig, sizeof(ubxAntennaConfig) );
  //delay( COMMAND_DELAY );

  //sendUBX( ubxAntennaConfig2, sizeof(ubxAntennaConfig2) );
  //delay( COMMAND_DELAY );

  sendUBX( ubxTimeAndDateInvalidValuesEnable, sizeof(ubxTimeAndDateInvalidValuesEnable) );
  delay( COMMAND_DELAY );

  sendUBX( ubxTimeAndDateInvalidValuesEnable2, sizeof(ubxTimeAndDateInvalidValuesEnable2) );
  delay( COMMAND_DELAY );

  sendUBX( ubxEnableZDA, sizeof(ubxEnableZDA) );
  delay( COMMAND_DELAY );
/*
  pinMode(2, INPUT);
  attachInterrupt(0, buttonPressedPlus, RISING);

  pinMode(3,INPUT);
  attachInterrupt(1, buttonPressedMinus, RISING);
*/
  cleanBuffer();

  pinMode(pinData, OUTPUT); //LED on Model B
  pinMode(pinClk, OUTPUT); //LED on Model B
  pinMode(pinLatch, OUTPUT); //LED on Model B

  //just for debug
  pinMode(0, OUTPUT); //LED on Model B
  pinMode(1, OUTPUT); //LED on Model A  or Pro

  firstTest();
}
/*
void buttonPressedPlus() {
  UTC_OFFSET++;
  validateUtcOffset();
  EEPROM.put(eepromAddrForUtcOffset, UTC_OFFSET);
}

void buttonPressedMinus() {
  UTC_OFFSET--;
  validateUtcOffset();
  EEPROM.put(eepromAddrForUtcOffset, UTC_OFFSET);
}
*/
void writeByte(unsigned char data) {
  int delayValue = 3;
  for (int i = 0; i < 8; i++) {
    int b = (data >> i) & 0x1;

    delay(delayValue);
    digitalWrite(pinClk, LOW);
    delay(delayValue);
    if (b>0) {
      delay(delayValue);
      digitalWrite(pinData, HIGH);
      delay(delayValue);
    } else {
      delay(delayValue);
      digitalWrite(pinData, LOW);
      delay(delayValue);
    }
    delay(delayValue);
    digitalWrite(pinClk, HIGH);
    delay(delayValue);
  }
}

void cleanBuffer() {
  for (int i = 0; i < 80; i++) {
    buf[i] = 0;
  }
}

void extractTime(char * rmc_data) {
  if (rmc_data[0] == '$' && rmc_data[1] == 'G' && rmc_data[3] == 'Z' && rmc_data[4] == 'D' && rmc_data[5] == 'A') {
    int i = 0;
    int got_hour;
    int got_min;
    int got_sec;
    int got_day;
    int got_month;
    int got_year;

    while (rmc_data[i++] != ',') {
      if (i > 79) return;
    };
    got_hour = (rmc_data[i] - '0') * 10 + (rmc_data[i + 1] - '0');
    got_min = ((rmc_data[i + 2] - '0') * 10) + (rmc_data[i + 3] - '0');
    got_sec = ((rmc_data[i + 4] - '0') * 10) + (rmc_data[i + 5] - '0');
    i += 6;
    while (rmc_data[i++] != ',') {
      if (i > 79) return;
    };
    got_day = ((rmc_data[i] - '0') * 10) + (rmc_data[i + 1] - '0');
    while (rmc_data[i++] != ',') {
      if (i > 79) return;
    };
    got_month = ((rmc_data[i] - '0') * 10) + (rmc_data[i + 1] - '0');
    while (rmc_data[i++] != ',') {
      if (i > 79) return;
    };
    got_year = ((rmc_data[i] - '0') * 1000) + (rmc_data[i + 1] - '0') * 100 + (rmc_data[i + 2] - '0') * 10 + (rmc_data[i + 3] - '0');

    /*debugPort.print(got_hour);
    debugPort.print(":");
    debugPort.print(got_min);
    debugPort.print(":");
    debugPort.print(got_sec);
    debugPort.print(" ");
    debugPort.print(got_day);
    debugPort.print(".");
    debugPort.print(got_month);
    debugPort.print(".");
    debugPort.println(got_year);*/

    globalYear = got_year;
    globalMonth = got_month;
    globalDay = got_day;
    globalHours = got_hour;
    globalMinutes = got_min;
    globalSeconds = got_sec;

    //Add timezone offset
    correctTimezone();

    //Correct hours due to summertime
    if (globalYear >= 2020) {
      correctSummertime();
    }
    //SHOW TIME
    showTime();
    //END SHOW TIME
  }
}

void showTime() {

  int hourHi = globalHours / 10;
  int hourLo = globalHours % 10;
  int minHi = globalMinutes / 10;
  int minLo = globalMinutes % 10;
  int secHi = globalSeconds / 10;
  int secLo = globalSeconds % 10;

  delay(3);
  digitalWrite(pinLatch, LOW);
  delay(3);

  writeByte(secondsLO[secLo & 0xF] + secondsHI[secHi & 0xF]*16);
  writeByte(minutesLO[minLo & 0xF] + minutesHI[minHi & 0xF]*16);
  writeByte(hoursLO[hourLo & 0xF] + hoursHI[hourHi & 0xF]*16);  

  delay(3);
  digitalWrite(pinLatch, HIGH);
  delay(3);
}


static int ix = 0;

void loop() { // run over and over

  gpsPort.listen();

  while (gpsPort.available() > 0) {
    char inByte = gpsPort.read();
    if (currentIndex < 80) {
      buf[currentIndex++] = inByte;
      if (inByte == 10 || inByte == 13) {
        if (strstr(buf, "$G") != NULL && strstr(buf, "ZDA") != NULL) {
          digitalWrite(0, (ix % 2 == 0) ? HIGH : LOW);
          extractTime(buf);
          ix++;
        }

        //debugPort.println(myString);
        currentIndex = 0;
        cleanBuffer();
      }
    } else {
      currentIndex = 0;
      cleanBuffer();
    }
  }
}
