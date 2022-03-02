// AUTOPILOT
#include <Servo.h>
#include <Adafruit_MPL3115A2.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <LoRa.h>




// GLOBAL VARS

// statics
#define gyroAddr 0x68                     // i2c address of BMG160 gyroscope is 0x68 (104 in decimal)
static float pi = 3.14159;                // pi
static int groundButtonPin = 2;           // button to set ground assigned to pin 2
SoftwareSerial gpsSerial(3,4);            // gps serial object, pins 3 and 4
static int dropPin = 5;                   // pin for detecting when pada is dropped
static int startPin = 13;                 // button that starts program

// constants
float landingLat;                         // coords of landing zone, set by markLandingCoords()
float landingLon;                         // **

// variables
int ground = 0;                           // ground altitude (final height)
int dropHeight;                           // drop height (initial height)
int height, lastHeight;                   // current and previous height
int dist, lastDist;                       // current and previous distance
int angle, lastAngle;                     // current and previous path angle
bool isFirst = true;                      // true only when loop is in its first iteration (can't find deltas until second iteration)
bool manualOverride = false;              // manual override initialized to false



// CLASS DEFINITION
class Plane                                                   // Plane object
{
  private:
    int defaultElevatorAngle = 90;                            // default servo angles (TBD)
    int defaultRudderAngle = 90;                              // **
    int maxElevatorAngle = 40;                                // max servo angles (TBD)
    int maxRudderAngle = 40;                                  // **
    float lat, lon;                                           // lat and lon variables
    Servo elevatorServo;                                      // ctrl servos
    Servo rudderServo;                                        // **
    Adafruit_MPL3115A2 altimeter;                             // altimeter
    TinyGPS gps;                                              // gps
  public:
    Plane();                                                  // <- default constructor
    void elevatorPitchUp(int=15);                             // <- elevator controls (default 15 deg)
    void elevatorPitchDown(int=15);                           // <- **
    void elevatorReset();                                     // <- **
    void rudderTurnLeft(int=15);                              // <- rudder controls (default 15 deg)
    void rudderTurnRight(int=15);                             // <- **
    void rudderReset();                                       // <- **
    void setGround();                                         // <- sets ground level to current altitude (also in default constructor)
    int  getHeight();                                         // <- get altitude
    void markLandingCoords();                                 // <- marks lat/lon of landing zone, along with drop height
    void getLatLon(float &, float &);                         // <- gets lat & lon (passed in by reference)
    void getGyroData(int &, int &, int &);                    // <- BMG160 gyro reads angle AND acceleration, function gets both (passed by reference)
    void getDirOfMot(float &, float &);                       // <- gets direction of motion (from gyroscope) (again, passed in by reference)
    int  getDistToLandingZone();                              // <- gets 2D cartesian distance from landing zone
    int  getVectorAngle();                                    // <- gets vector angle relative to landing zone
};




// MEMBER FUNCTIONS
Plane::Plane()                                        // default constructor (SETUP STUFF GOES IN HERE)
{
  Wire.begin();                                       // initialize i2c with master             ** -=-=-=-=-=-=-=-=-=-=-=-=-=-
  Wire.beginTransmission(gyroAddr);                   // BEGIN TRANSMISSION                     ** REGISTER DATA FOR GYRO
  Wire.write(0x0F);                                   //    %% selects register (for range)     ** - opens register 0x0F (15)
  Wire.write(0x80);                                   //    %% set full scale range             ** - writes data to register
  Wire.endTransmission();                             // END TRANSMISSION                       **
  Wire.beginTransmission(gyroAddr);                   // BEGIN NEW TRANSMISSION                 **
  Wire.write(0x10);                                   //    %% selects register (for bandwidth) ** - opens register 0x10 (16)
  Wire.write(0x04);                                   //    %% set bandwidth                    ** - writes data to register
  Wire.endTransmission();                             // END TRANSMISSION                       ** -=-=-=-=-=-=-=-=-=-=-=-=-=-
  elevatorServo.attach(9);                            // attach servos to pins                  **
  rudderServo.attach(8);                              // **                                     **
  altimeter.setSeaPressure(1013.26);                  // set sea pressure for altimeter         **
  gpsSerial.begin(9600);                              // begin gps softwareSerial               **
  elevatorServo.write(defaultElevatorAngle);          // set servos to default angles           **
  rudderServo.write(defaultRudderAngle);              // **                                     **
  ground = altimeter.getAltitude();                   // sets ground                            **
}

void Plane::elevatorPitchUp(int angle)
{
  if(angle <= maxElevatorAngle)
    elevatorServo.write(defaultElevatorAngle + angle);
}

void Plane::elevatorPitchDown(int angle)
{
  if(angle <= maxElevatorAngle)
    elevatorServo.write(defaultElevatorAngle - angle);
}
void Plane::elevatorReset()
{
  elevatorServo.write(defaultElevatorAngle);
}

void Plane::rudderTurnLeft(int angle)
{
  if(angle <= maxRudderAngle)
    rudderServo.write(defaultRudderAngle + angle);
}

void Plane::rudderTurnRight(int angle)
{
  if(angle <= maxRudderAngle)
    rudderServo.write(defaultRudderAngle - angle);
}

void Plane::rudderReset()
{
  rudderServo.write(defaultRudderAngle);
}

void Plane::setGround()
{
  ground = altimeter.getAltitude();
}

int Plane::getHeight()
{
  return altimeter.getAltitude() - ground;
}

void Plane::markLandingCoords()
{
  if(gpsSerial.available() && gps.encode(gpsSerial.read()))             // check for gps data in serial ports and encode data
  {                                                                     //
    gps.f_get_position(&landingLat, &landingLon);                       // call position accessor, passing references, save to public vars
    dropHeight = altimeter.getAltitude() - ground;                      // mark drop height
  }                                                                     // 
}

void Plane::getLatLon(float &lat, float &lon)
{
  if(gpsSerial.available() && gps.encode(gpsSerial.read()))             // check for gps data in serial ports and encode data
    gps.f_get_position(&lat, &lon);                                     // call position accessor, passing references
}

void Plane::getGyroData(int &x, int &y, int &z)
{
  unsigned int x0, x1, y0, y1, z0, z1;                    // each axis has most significant (1) and least significant (0) bits
  Wire.beginTransmission(gyroAddr);                       // opens gyrometer data register
  Wire.write(0x02);                                       // **
  Wire.endTransmission();                                 // **
                                                          //
  Wire.requestFrom(gyroAddr, 6);                          // requests 6 bytes of datat from gyroAddr
  if(Wire.available() == 6)                               // checks num of available bytes
  {                                                       // reads data into x,y,z, with LSBs and MSBs
    x0 = Wire.read();                                     // **
    x1 = Wire.read();                                     // **
    y0 = Wire.read();                                     // **
    y1 = Wire.read();                                     // **
    z0 = Wire.read();                                     // **
    z1 = Wire.read();                                     // **
  }                                                       // **
                                                          //
  x = 256*x1 + x0;                                        // converts to data
  y = 256*y1 + y0;                                        // **
  z = 256*z1 + z0;                                        // **
}

void Plane::getDirOfMot(float &cLat, float &cLon)
{
  int p, r, y;
  getGyroData(p,r,y);           // figure out calibration for gyroscope to get lat/lon
}

int Plane::getDistToLandingZone()
{
  float pLat, pLon;
  this->getLatLon(pLat, pLon);
  return sqrt(pow(pLat - landingLat, 2) + pow(pLon - landingLon, 2));             // cartesian distance
}

int Plane::getVectorAngle()
{
  // PSEUDO
  // - calculate vector from current position to landing zone (path vector)
  // - get plane's dir of motion (from gyro)
  // - return angle between vectors (if plane is going too far right, returns positive angle, negative angle for left)
  
  float pLat, pLon, cLat, cLon;                                                                                     // latitude and longitude components of path vector
  this->getLatLon(pLat, pLon);                                                                                      // get current lat & lon
  pLat -= landingLat;                                                                                               // the vector <pLat, pLon> points from the plane to the landing zone (2D)
  pLon -= landingLon;                                                                                               // **
  this->getDirOfMot(cLat, cLon);                                                                                    // the vector <cLat, cLon> points in the direction the plane is moving (direction of motion)
  int pTh = atan(pLat/pLon) * 180/pi;
  int cTh = atan(cLat/cLon) * 180/pi;
  return cTh - pTh;                                                                                                 // returned angle is positive if plane is going too far right, else negative
}




// PROTOTYPES





// MAIN
Plane p1;                      // declare plane object w/ default constructor

void setup()
{
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Serial has started successfully!");
  while(digitalRead(startPin) == LOW);                              // wait for start button...
}

void loop()
{ 
  
  // GET VALS
  int pitch, roll, yaw;                                             // gyro measure for each axis, (TEMPORARY NAMES, IDK WHICH ONE IS WHICH)
  p1.getGyroData(pitch, roll, yaw);                                 // gets values
  height = p1.getHeight();                                          // difference between current and ground altitudes
  dist = p1.getDistToLandingZone();                                 // cartesian dist to landing zone
  

  // COMPUTATIONS
  float heightDistRatio = 0;                                                                // declare ratio
  if(!isFirst)                                                                              // if loop is not in its first iteration:
  {                                                                                         //
    heightDistRatio = (height-lastHeight) / (dist-lastDist);                                // calculate height-distance ratio
  }
  int pathGradeDesired = atan((float)height / (float)dist) * 180/pi;                        // computes DESIRED path grade to landing zone (degrees BELOW horizontal)
  int pathGradeCurrent = atan(heightDistRatio) * 180/pi;                                    // computes CURRENT path grade from change in height-distance ratio
  Serial.println("Difference between pitch angle and trianglulated angle:"); 
  Serial.print("The plane's flight direction is "); Serial.print(pitch - pathGradeCurrent); Serial.println(" degrees above its pitch.");

/*
  // STABILITY CORRECTIONS
  
  
  
  // PATH CORRECTIONS
  if(pathGradeCurrent > pathGradeDesired+10)
    p1.elevatorPitchUp(8);
  else if(pathGradeCurrent < pathGradeDesired-10)
    p1.elevatorPitchDown(8);
  else
    p1.elevatorReset();

  
  // PREVIOUOS VALS
  lastHeight = height;              // sets last height
  lastDist = dist;                  // sets last distance
  if(isFirst)                       // if isFirst is true, set it to false to end first iteration
    isFirst = false;                // **
  */
  while(digitalRead(startPin)==LOW);
}




// FUNCTIONS





// NOTES
/*
 * FOR PLANE CLASS:
 * - (CHECK) implement servos
 * - (CHECK) implement altimeter
 * - (CHECK) implement gps object
 * - (CHECK) implement gyroscope
 * - implement transceivers
 * 
 * CONCERNS
 * - size of program must be under 32256 bytes (shouldn't be a problem)
 * - how does gyro measure axes? how to calibrate?
 * - how to set ground?
 * - ROLL!!!!!!!!\
 * - for simplicity's sake, CONVERT ALL ANGLES TO DEGREES!!
 */






// ALGORITHM OUTLINE
/*
 * plane starts some distance from landing zone
 * - need rate of change in height
 * - need rate of change in position
 * 
 * 
 */























 
