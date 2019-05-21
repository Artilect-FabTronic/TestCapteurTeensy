// Enregistreur d'essais pour sonde d'humiditée de terrain
// Le 20/05/2019
// martorell.philippe@gmail.com
// Sur Teensy 3.6 (3.6V)
// avec GPS gm22u7
// et CAD ads1115
// commande d'alimentation de ces deux composants par transistor 2N2907
// résistance 2200 ohms sur la base
// collecteur sur les alims du GPS et du CAD ads1115
//




#include "Wire.h"
#include "SD.h"
#include "SPI.h"
#include "TimeLib.h"
//#include "SoftwareSerial.h"
#include "TinyGPS.h"
#include "Adafruit_ADS1015.h"



//CONSTANTES et VARIABLES

// Constantes SD
// Pin CS
// Materiel, par défaut sur cartes Arduino habituelles ; pin 10
// Mega pin 53
// Autres cas
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// Teensy audio board: pin 10
// Teensy 3.5 & 3.6 on-board: BUILTIN_SDCARD
// Wiz820+SD board: pin 4
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
const int chipSelect = BUILTIN_SDCARD ;
// Pins du port SPI
// MOSI - pin 11, pin 7 on Teensy with audio board
// MISO - pin 12
// CLK  - pin 13, pin 14 on Teensy with audio board
// CS   - pin 4,  pin 10 on Teensy with audio board


// Converisseur ADS1115 ads 16 bits =  65536
// 1 bit = 0.1875mV/ADS1115
//

// Cablage GPS
// Sur le proto maison
#define gpsPort Serial2
// TX du GPS vers pin 9 RX2
// RX du GPS vers pin 10 TX2
const int pinAlimGps = 12 ; // commande aussi l'alimentation du CAD liaison vers résistance
// Autres cas et dans ce cas validez SoftwareSerial dans les includes
// SoftwareSerial gpsPort(4, 3);

// Variables GPS
float flat, flon;
unsigned long age, chars = 0;
unsigned short sentences = 0, failed = 0;
int yearGPS = 0 ;
byte monthGPS, dayGPS, hourGPS, minuteGPS, secondGPS, hundredths;
int ageMaxi = 2000 ;


// Constantes Fichiers et reglages
const char* nomFichierData = "datalog.csv" ;
//Variables Fichiers et reglages
String trame = "" ;


// Gestion de la cadence
// nombre d'unitees entre deux trames
// voir en début de loop
// Variables gestion trame
byte secondePrecedente = 61 ;
byte minutePrecedente = 61 ;
byte heurePrecedente = 25 ;

// Démarrage fonctions
TinyGPS gps;
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */



// Une fonction qu'il vaut mieux mettre ici
String miseEnFormeDizaines ( int valeur )
{
  String misEnForme ;
  if (valeur < 10)
  {
    misEnForme = "0" ;
  }
  return String ( misEnForme + valeur ) ;
}


void setup ( )
{

  // A décommenter si on veux debbuger

  //Serial.begin ( 115200 ) ;
  while ( !Serial )
  {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  while (!Serial);  // Wait for Arduino Serial Monitor to open
  delay(100);

  // Mise sous tension
  // GPS
  pinMode ( pinAlimGps , OUTPUT ) ;
  digitalWrite ( pinAlimGps , LOW ) ;
  delay ( 2000 ) ;


  // GPS
  gpsPort.begin(9600);

  // Bus
  Wire.begin();

  // ADC
  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.begin();
  // SCL en 19 SCL0
  // SDA en 18 SDA0



  //Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  //Serial.println("card initialized.");
  if ( SD.exists ( nomFichierData ) )
  {
    //Serial.println("datalog.csv exists.");
  }
  else
  {
    //Serial.println("datalog.csv doesn't exist.");
    // open a new file and immediately close it:
    //Serial.println("Creating datalog.csv...");
    File dataFile = SD.open ( nomFichierData , FILE_WRITE ) ;
    dataFile.close ( ) ;
  }

  Teensy3Clock.get();

  // Attente accrochage heure GPS
  attenteAccrochageGPS ( ) ;

  // Mise a l'heure GPS de la RTC
  setTime ( hourGPS , minuteGPS , secondGPS , dayGPS , monthGPS , yearGPS ) ;
  Teensy3Clock.set ( now ( ) ) ;
  setTime ( now ( ) ) ;

  // Vérifie que l'horloge RTC est bien utilisée
  if (timeStatus() != timeSet)
  {
    //Serial.println ("Impossible de synchroniser l'horodatage avec l'horloge RTC");
  }
  else
  {
    //Serial.println ("L'horodatage a correctement ete synchronisee avec l'horloge RTC");
  }

  // Lecture de l'horloge RTC
  Teensy3Clock.get();
  trame = "Debut d'enregistrement ";
  // Temps ( cadence principale basée sur l'horloge temps réel RTC )
  trame += String ( miseEnFormeDizaines ( day ( ) ) ) ;
  trame += ':';
  trame += String ( miseEnFormeDizaines ( month ( ) ) ) ;
  trame += ':';
  trame += String ( miseEnFormeDizaines ( year ( ) ) ) ;
  trame += ';';
  trame += String ( miseEnFormeDizaines ( hour ( ) ) ) ;
  trame += ':';
  trame += String ( miseEnFormeDizaines ( minute ( ) ) ) ;
  trame += ':';
  trame += String ( miseEnFormeDizaines ( second ( ) ) ) ;
  //Serial.println ( trame ) ;
  enregistrementTrameSurSD (  ) ;
}



void loop ( )
{
  // Analyse des trames GPS en permamence
  lectureDecodageGPS ( ) ;

  // Lecture de l'horloge temps réel Teensy et gestion de la cadence
  Teensy3Clock.get();
  //if ( second ( ) != secondePrecedente )
  if ( minute() != minutePrecedente )
    //if ( hour() != heurePrecedente )
  {
    // mise a ajour pour cadencement
    secondePrecedente = second ( ) ;
    minutePrecedente = minute ( ) ;
    heurePrecedente = hour ( ) ;

    // Mise sous tension
    // GPS
    digitalWrite ( pinAlimGps , LOW ) ;
    //delay ( 2000 ) ;

    // Attente accrochage heure GPS
    attenteAccrochageGPS ( ) ;

    //Lecture GPS
    gps.f_get_position(&flat, &flon, &age);
    //Serial.println ( age ) ;

    int yearGPS;
    byte monthGPS, dayGPS, hourGPS, minuteGPS, secondGPS, hundredths;
    unsigned long age;
    gps.crack_datetime(&yearGPS, &monthGPS, &dayGPS, &hourGPS, &minuteGPS, &secondGPS, &hundredths, &age);

    // Generation de la trame enregistree
    // Remise à zero de la trame
    trame = "";

    // Temps basée sur l'horloge temps réel RTC
    trame += String ( miseEnFormeDizaines ( day ( ) ) ) ;
    trame += '/';
    trame += String ( miseEnFormeDizaines ( month ( ) ) ) ;
    trame += '/';
    trame += String ( miseEnFormeDizaines ( year ( ) ) ) ;
    trame += ';';
    trame += String ( miseEnFormeDizaines ( hour ( ) ) ) ;
    trame += ':' ;
    trame += String ( miseEnFormeDizaines ( minute ( ) ) ) ;
    trame += ':' ;
    trame += String ( miseEnFormeDizaines ( second ( ) ) ) ;
    trame += ';' ;

    // Temps basée sur le GPS
    trame += String ( miseEnFormeDizaines ( dayGPS ) ) ;
    trame += '/' ;
    trame += String ( miseEnFormeDizaines ( monthGPS ) ) ;
    trame += '/' ;
    trame += String ( miseEnFormeDizaines ( yearGPS ) ) ;
    trame += ';' ;
    trame += String ( miseEnFormeDizaines ( hourGPS ) ) ;
    trame += ':' ;
    trame += String ( miseEnFormeDizaines ( minuteGPS ) ) ;
    trame += ':' ;
    trame += String ( miseEnFormeDizaines ( secondGPS ) ) ;
    trame += ';' ;

    // Position GPS
    // Si position invalide
    if (age == TinyGPS::GPS_INVALID_AGE)
      // generer un texte
    {
      trame += String ( "lat" ) ;
      trame += ';' ;
      trame += String ( "long" ) ;
      trame += ';' ;
      trame += String ( "alti" ) ;
      trame += ';' ;
    }
    else
      // sinon les bonnes valeurs
    {
      trame += String ( flat , 6 ) ;
      trame += ';' ;
      trame += String ( flon , 6 ) ;
      trame += ';' ;
      trame += String ( gps.f_altitude() ) ;
      trame += ';' ;
    }

    // Mesures analogiques

    trame += String ( flat , 6 ) ;
    trame += ';' ;
    trame += String ( flon , 6 ) ;
    trame += ';' ;
    trame += String ( gps.f_altitude() ) ;
    trame += ';' ;

    trame += String ( ads.readADC_SingleEnded ( 0 ) ) ;
    trame += ';' ;
    trame += String ( ads.readADC_SingleEnded ( 1 ) ) ;
    trame += ';' ;
    trame += String ( ads.readADC_SingleEnded ( 2 ) ) ;
    trame += ';' ;
    trame += String ( ads.readADC_SingleEnded ( 3 ) ) ;
    trame += ';' ;

    // Affichage de la trame pour controle
    // Serial.println(trame);

    // Enregistrement de la trame sur SD
    enregistrementTrameSurSD (  ) ;
  }

  // Mise hors tension
  // GPS et convertisseur
  digitalWrite ( pinAlimGps , HIGH ) ;
}


void lectureDecodageGPS ( void )
{
  while (gpsPort.available())
  {
    gps.encode(gpsPort.read());
  }
}


void enregistrementTrameSurSD ( void )
{
  // Ouvrir le fichier
  File dataFile = SD.open( nomFichierData, FILE_WRITE);
  // si le fichier existe
  if ( dataFile )
  {
    // on enregistre
    dataFile.println ( trame ) ;
    dataFile.close ( ) ;
  }
  // sinon on pleure
  else
  {
    //Serial.println ( "error opening datalog.csv" ) ;
  }
}


void attenteAccrochageGPS ( void )
{
  monthGPS = 13 ;
  dayGPS = 32 ;
  hourGPS = 25 ;
  minuteGPS = 61 ;
  secondGPS = 61 ;
  yearGPS = 0 ;
  // Analyse des trames GPS en permamence
  lectureDecodageGPS ( ) ;
  gps.f_get_position(&flat, &flon, &age);

  while ( yearGPS < 2001 || age > ageMaxi )
    //while ( yearGPS < 2001 )
  {
    // Analyse des trames GPS en permamence+-
    lectureDecodageGPS ( ) ;

    // Decodage GPS
    gps.f_get_position(&flat, &flon, &age);
    gps.crack_datetime(&yearGPS, &monthGPS, &dayGPS, &hourGPS, &minuteGPS, &secondGPS, &hundredths, &age);
    //Serial.println ( yearGPS ) ;
    //Serial.println ( age ) ;
    //delay (1000) ;
  }
}
