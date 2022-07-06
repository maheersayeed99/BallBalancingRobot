
// LIBRARIES USED

#include <IRremote.h>
#include <ir_Lego_PF_BitStreamEncoder.h>
#include <SimpleKalmanFilter.h>
#include <PID_v1.h>
#include <Servo.h>

//-----------------------------------------------------------------------------------------------------------

// VARIABLES USED

Servo myservo1;      //DEFINE BOTH SERVOS
Servo myservo2;

int angle1;          //DEFINE INITIAL SERVO ANGLE
int angle2;

int maxAngle = 120;  //DEFINE SERVO ROTATION LIMITS
int minAngle = 60;

int timeInc = 20;     //DEFINE HOW OFTEN CALCULATIONS ARE DONE

int Y1 = A8;          //DEFINE PINS FOR TOUCH SCREEN
int Y2 = A9;
int X1 = A10;
int X2 = A11;

float x = 0;          //DEFINE VARIABLES FOR TOUCHSCREEEN XYZ COMPONENTS
float y = 0;
float x1 = 0;
float y1 = 0;
float x0 = 0;
float y0 = 0;
float z1 = 0;

double Setpointx;     //DEFINE SETPOINT XY FOR BALL ON PLATE
double Setpointy;

double KPx;           //DEFINE PID VARIABLES FOR X AND Y CONTROLLER
double KIx;
double KDx;

double KPy;
double KIy;
double KDy;

double Inputx;        //DEFINE INPUT FOR EACH PID CONTROLLER
double Inputy;

double Outputx;       //DEFINE OUTPUT FOR EACH PID CONTROLLER
double Outputy;

PID myPIDx(&Inputx, &Outputx, &Setpointx, KPx, KIx, KDx, DIRECT);         //DEFINE PID CONTROLLERS
PID myPIDy(&Inputy, &Outputy, &Setpointy, KPy, KIy, KDy, REVERSE);

int e_mea = 1;      //DEFINE KALMAN FILTER CONSTANTS
int e_est = 1;
double q = 0.15;

SimpleKalmanFilter kf(e_mea,e_est,q);   //DEFINE KALMAN FILTER


int circleOn;         //DEFINE CONSTANTS FOR CIRCULAR MOTION
double r = 30;
double a = 30;
double d = 0;
double theta = 0;
double pi = 3.14159;
double divider;
int count = 0;


int receiverPin = 5;          //DEFINE PIN FOR IR RECEIVER
IRrecv irrecv(receiverPin);   //DEFINE IR RECEIVER
decode_results results;
int SPN = 25;                 //DEFINE STEP INPUT DISTANCE

int negative = -1;
double ctrl = 5;              //DEFINE CONTROLLER STEP

//--------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);         //START SERIAL MONITOR
  
  irrecv.enableIRIn();        //START IR RECEIVER
  irrecv.blink13(true);
  
  myservo1.attach(4);         //ATTACH SERVOS TO PINS
  myservo2.attach(3);
  
  angle1 = 90;                //SET SERVO ANGLES TO MAKE PLATE FLAT
  angle2 = 90;
  myservo1.write(angle1);
  myservo2.write(angle2);

  Setpointx = 0;              //SET SETPOINT TO INITIALIZE BALL TO THE MIDDLE OF THE PLATE
  Setpointy = 0;

  KPx = 0.3;                   //GIVE NUMERICAL VALUES TO PID CONSTANTS
  KIx = 0.08;
  KDx = 0.1;

  KPy = 0.3;
  KIy = 0.08;
  KDy = 0.1;

  myPIDx.SetMode(AUTOMATIC);                  //SET UP PID CONTROLLERS
  myPIDy.SetMode(AUTOMATIC);
  myPIDx.SetTunings(KPx, KIx, KDx);
  myPIDy.SetTunings(KPy, KIy, KDy);
  myPIDx.SetOutputLimits(minAngle, maxAngle); //SET PID LIMITS
  myPIDy.SetOutputLimits(minAngle, maxAngle);
  myPIDx.SetSampleTime(timeInc);              //SET HOW OFTEN PID IS COMPUTED
  myPIDy.SetSampleTime(timeInc);

  pinMode(A2, INPUT);                         //INPUT PIN FOR POTENTIOMETER THAT IS USED SOMETIMES
  pinMode(A1, INPUT);
  
}

//----------------------------------------------------------------------------

void loop() {

  if (irrecv.decode(&results)){               //GET VALUE FROM INFRARED REMOTE
    Serial.println(results.value, HEX);
    irrecv.resume();
  }
  
  switch(results.value){                      //SWITCH CASE FOR THE BUTTON PRESSED
    
          case 0xFF30CF:
          Setpointx = SPN;
          Setpointy = -SPN;
          break;
          case 0xFF18E7:
          Setpointx = 0;
          Setpointy = -SPN;
          break;
          case 0xFF7A85:
          Setpointx = -SPN;
          Setpointy = -SPN;
          break;
          case 0xFF10EF:
          Setpointx = SPN;                    //CASES FOR STEP INPUTS FROM ORIGIN
          Setpointy = 0;
          break;
          case 0xFF38C7:
          Setpointx = 0;
          Setpointy = 0;
          break ;
          case 0xFF5AA5:
          Setpointx = -SPN;
          Setpointy = 0;
          break ;
          case 0xFF42BD:
          Setpointx = SPN;
          Setpointy = SPN;
          break ;
          case 0xFF4AB5:
          Setpointx = 0;
          Setpointy = SPN;
          break ;
          case 0xFF52AD:
          Setpointx = -SPN;
          Setpointy = SPN;
          break ;


          case 0xFF629D:
          Setpointy = Setpointy - ctrl;
          results.value = 0xFF1111;
          break ;
          case 0xFFA857:
          Setpointy = Setpointy + ctrl;
          results.value = 0xFF1111;
          break ;
          case 0xFFC23D:
          Setpointx = Setpointx - ctrl;             //CASES FOR TINY STEP INPUTS THAT LET US MOVE THE BALL AROUND THE PLATE
          results.value = 0xFF1111;
          break ;
          case 0xFF22DD:
          Setpointx = Setpointx + ctrl;
          results.value = 0xFF1111;
          break ;
          case 0xFF1111:
          break;
            
  }


  circleOn = analogRead(A2);                      // GET RADIUS VALUE FROM POTENTIOMETER
  
  if (results.value == 0xFF6897 && count%10 == 0){
    a = map(circleOn,0,1023,5,30);
    d = map(circleOn,0,1023,4,12);
    Setpointy = a*sin(theta);                       // SINUSOIDAL INPUT RESULTING IN CIRCULAR MOTION
    Setpointx = a*cos(theta);                       // POLAR EQUATION FOR CIRCLE USED
    theta = theta +(pi/d);
  }
  else if (results.value == 0xFFE21D && count%10 == 0){
    a = map(circleOn,0,1023,5,30);
    d = map(circleOn,0,1023,4,12);                  // FLIPS THE DIRECTION OF THE CIRCULAR MOTION
    Setpointy = a*sin(theta);
    Setpointx = a*cos(theta);
    theta = theta -(pi/d);
  }
  else if (results.value == 0xFFB04F && count%10 == 0){
    //a = float(map(circleOn,0,1023,5,30));
    r = sqrt(abs((a*a)*cos(2*theta)));
    d = map(circleOn,0,1023,4,12);
    Serial.print(Setpointx);                        // LEMNISCATE EQUATION USED FOR PETAL MOTION
    Serial.println(Setpointy);
    Setpointy = r*sin(theta);
    Setpointx = r*cos(theta);
    theta = theta + (negative)*(pi/d);
  }
  
  count+=1;                                           //COUNT IS ITERATED

  pinMode(Y1, INPUT);
  pinMode(Y2, INPUT);                                 //PINMODES CHANGED TO GET X AXIS VOLTAGE
  digitalWrite(Y1, HIGH);
  pinMode(X2, OUTPUT);
  digitalWrite(X2, HIGH);
  pinMode(X1, OUTPUT);
  digitalWrite(X1, LOW);
  x = analogRead(Y2);
  x1 = double(map(x, 150, 850, -9000, 9000)) / 100;     //VOLTAGE CONVERTED TO CM
  
  Inputx = kf.updateEstimate(x1);                       // MEASUREMENT SMOOTHED WITH KALMAN FILTER
  delay(7);
  
  pinMode(X1, INPUT);
  pinMode(X2, INPUT);
  digitalWrite(X1, HIGH);                               //PINMODES CHANGED TO GET X AXIS VOLTAGE
  pinMode(Y2, OUTPUT);
  digitalWrite(Y2, HIGH);
  pinMode(Y1, OUTPUT);
  digitalWrite(Y1, LOW);
  y = analogRead(X2);
  y1 = double(map(y, 200, 800, 6000, -6000)) / 100;     //VOLTAGE CONVERTED TO CM
  
  Inputy = kf.updateEstimate(y1);                       // MEASUREMENT SMOOTHED WITH KALMAN FILTER
  delay(7);
  
  pinMode(Y2, INPUT);
  pinMode(X1, INPUT);                                   //PINMODES CHANGED TO GET z VOLTAGE THAT CHECKS IF BALL IS ON PLATE
  digitalWrite(Y2, HIGH);
  digitalWrite(X1, LOW);
     
  z1 = analogRead(X1);
  delay(5); 
  
  
  if (z1 < 10 && z1>-10) {                      //IF BALL IS ON PLATE
    
     myPIDx.Compute();
     myPIDy.Compute();                          //COMPUTE PID OUTPUT SERVO ANGLE
     if (Outputx > maxAngle) {                  //IF ANGLE IS OUTSIDE LIMITS
      Outputx = maxAngle;                       //MAKE ANGLE EQUAL TO THE LIMITS TO AVOID MECHANICAL FAILURE
      }
     else if (Outputx < minAngle) {
      Outputx = minAngle;
      }
     if (Outputy > maxAngle) {
      Outputy = maxAngle;
      }
     else if (Outputy < minAngle) {
      Outputy = minAngle;
     }
     myservo1.write(Outputx);                   //MAKE SERVO TURN TO OUTPUT ANGLE
     myservo2.write(Outputy);
  }
        else{                                   //IF BALL IS NOT ON PLATE
        myservo1.write(angle1);                 //MAKE PLATE FLAT
        myservo2.write(angle2);
  }
  
  
}

  
