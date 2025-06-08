
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define fast 1
#define slow 0


unsigned int pos0=103; // ancho de pulso en cuentas para pocicion 0°
unsigned int pos180=512; // ancho de pulso en cuentas para la pocicion 180°
Adafruit_PWMServoDriver quad_servos = Adafruit_PWMServoDriver(0x40);

const int idle = 0;
const int arm = 10;
const int robot =  20;
const int  step_1 = 30;
const int  reset_orig = 40;
const int  orig = 50;
const int step_2 = 60;
const int step_3 = 70;
const int right = 80;
const int left = 90;


Servo theta_0;  // create servo object to control a servo
Servo theta_1;  // create servo object to control a servo
Servo theta_2;  // create servo object to control a servo
Servo theta_3;  // create servo object to control a servo
Servo theta_4;  // create servo object to control a servo
Servo gripper;  // create servo object to control a servo
Servo test_leg;
Servo test_leg2;
Servo FL_0;
Servo FL_1;
Servo FL_2;

Servo FR_0;
Servo FR_1;
Servo FR_2;

Servo BL_0;
Servo BL_1;
Servo BL_2;

Servo BR_0;
Servo BR_1;
Servo BR_2;
// Constantes de origen

int origin_FR_1 = 20+90;
int origin_FR_2 = 110;

int origin_FL_1 = 90 +20;
int origin_FL_2 = 120; // LOW 40

int origin_BR_1 = 27;
int origin_BR_2 = 110;

int origin_BL_1 = 30;
int origin_BL_2 = 120;


// Movimientos
int move1 = -11;
int move2 = 30;

int move1b = -11;
int move2b = 30;

int steps = 1;

int delay_ms = 50;
int delay_ms2 = 80;
int origin_FR_0 = 5;

int move_FR1 = origin_FR_1 + move1;

int move_FR2 = origin_FR_2 + move2;


int move_FL1 = origin_FL_1 + move1;
int move_FL2 = origin_FL_2 + move2-5;


int move_BR1 = origin_BR_1 + move1b;
int move_BR2 = origin_BR_2 + move2b;


int move_BL1 = origin_BL_1 + move1b;
int move_BL2 = origin_BL_2 + move2b;


int origin_m0 = 85;
int origin_m1 = 0;
int origin_m2 = 0;
int origin_m3 = 0;
int origin_m4 = 0;
int EF_Open = 200;
int EF_Close = 200;
int arm_joint[6] ={origin_m0,origin_m1,origin_m2,origin_m3,origin_m4};

int quad_joint[12] ={0};

int arm_joint_past[6] ={origin_m0,origin_m1,origin_m2,origin_m3,origin_m4};
int quad_joint_past[12] ={0};
int Index = 0;

unsigned long MOVING_TIME = 2500; // moving time is 3 seconds
unsigned long moveStartTime;

int command = idle;
bool isValidCommand(const String& s) {
  // Ajusta esta lista según tus comandos reales
  if (s == "m2" || s == "m3" || s == "m4" || s == "m5"
      || s == "r"  || s == "l"
      || s == "reset" || s == "origin") return true;
  if (s.startsWith("ARM,"))     return true;  // ARM,10,20,30,40,50,60
  return false;
}
void move_FR(){

   delay(delay_ms);
  FR_2.write(move_FR2 );
  delay(delay_ms);
  FR_1.write(move_FR1);
  delay(delay_ms);
  FR_2.write(origin_FR_2);
  delay(delay_ms);
  FR_1.write(origin_FR_1);
  delay(delay_ms);
}
void move_BL(){
  // STEP LEG BL
   delay(delay_ms);
   
  BL_2.write(move_BL2);
  delay(delay_ms);
  BL_1.write(move_BL1);
  delay(delay_ms);
  BL_2.write(origin_BL_2);
  delay(delay_ms);
  BL_1.write(origin_BL_1);
  delay(delay_ms);

  
}
void move_BR(){
  // STEP LEG BR 
  delay(delay_ms);
  BR_2.write(move_BR2);
  delay(delay_ms);
  BR_1.write(move_BR1);
  delay(delay_ms);
  BR_2.write(origin_BR_2);
  delay(delay_ms);
  BR_1.write(origin_BR_1);
  delay(delay_ms);
}
void move_FL(){

   delay(delay_ms);
   FL_2.write(move_FL2 );
  
  delay(delay_ms);
  FL_1.write(move_FL1);
  delay(delay_ms);
  FL_2.write(origin_FL_2 );

  delay(delay_ms);
  FL_1.write(origin_FL_1);
  delay(delay_ms);
}

void move_step_1(){
  Serial.println("Moving step 1");
  // Step ONE 
  // ORIGIN IS
 
  //move_pos();
 
  
  move_BR();
  Serial1.println("BR");

  move_FR();
  Serial1.println("FR");
   
  move_BL();
  Serial1.println("BL");
  move_FL();
  Serial1.println("FL");

 // return_up();
 // origin();

  /*move_pos();
  delay(200);
  return_up();
  delay(200);*/
  Serial.println("Finish step 1");
  
}
void move_step_2(){

  for(int i = 0; i<20;i++){

    move_step_1();
  }
  origin();
}
void moveServo(Servo& servo,int  angle, int past_angle){
  unsigned long progress = millis() - moveStartTime;
  int move = past_angle -angle;
  Serial.print("New angle");
  Serial.print(angle);
  Serial.print("Current angle");
  Serial.println(past_angle);
  if (angle == past_angle){
    return;
  }
  
  if ( move <  0){
    move = move*-1;
  }
  if ( move < 20 ){
    MOVING_TIME = 500;
  }
  else {
    MOVING_TIME = 2500;
  }
  MOVING_TIME = 2500;
  moveStartTime = millis();
  progress = 0;
  while (progress <= MOVING_TIME) {
    progress = millis() - moveStartTime;
    long ang = map(progress, 0, MOVING_TIME, past_angle, angle);
    servo.write(ang); 
    
   
  }
}
void move_step_3(){

  // STEP LEG FR  // STEP LEG bL
  FR_2.write(0);
  BL_2.write(0);
  delay(100);
  FR_2.write(35);
  BL_2.write(35);
  delay(100);
  FR_1.write(60);
  BL_1.write(50);
  delay(100);
  FR_2.write(20);
  BL_2.write(10);
  delay(100);
  
  
 // STEP LEG BR  // STEP LEG FL
  BR_2.write(0);
  FL_2.write(0);
  delay(100);
  FL_2.write(0);
  delay(100);
  BR_2.write(35);
  FL_2.write(35);
  delay(100);
  BR_1.write(55);
  FL_1.write(45);
  delay(100);
  BR_2.write(10);
  FL_2.write(10);
  delay(100);
  
  
 

  origin2();
  
  Serial.println("Finish step 1");
}

void move_left(){
  move_FR();
  delay(100);
  move_BR();
  delay(100);
  origin2();
}
void move_right(){
  move_FL();
  delay(100);
  move_BL();
  delay(100);
  origin2();
}
void moveArm2(){
  Serial.println("Moving arm");
  delay(2500);
  
}
void sit(){
 // STEP LEG FR  // STEP LEG bL
  FR_1.write(150);
  FL_1.write(150);
  delay(3000);
  BR_1.write(150);
  BL_1.write(150);
  
}
void moveQuad(){
  Serial.println("Moving quad");
  
  delay(1000);
  Serial.println("Finish armm");
  
}
void moveArm(){
  
  //theta_0.write(arm_joint[0]); 
  // Mapea linealmente de 103-520  →  0-180

  
  arm_joint[0] = origin_m0 + arm_joint[0]-1;
  arm_joint[1] = origin_m1 + arm_joint[1]+7;
  arm_joint[2] = origin_m2 + arm_joint[2] - 27;
  if ( arm_joint[3] < 35){
     arm_joint[3] = 45; 
  }
  else
  {
     arm_joint[3] = origin_m3 + arm_joint[3]+15;
  }
 
  
  moveServo(theta_0, arm_joint[0],arm_joint_past[0]);
  
  moveServo(theta_3, arm_joint[3],arm_joint_past[3]);
 theta_4.write(arm_joint[4]);
  

  

  
  arm_joint[1] = arm_joint[1];
  arm_joint[2] = arm_joint[2];

  if(arm_joint[1] < 0)
  {
    arm_joint[1] = 0;
  }

  if(arm_joint[2] < 0)
  {
    arm_joint[2] = 0;
  }
  moveServo(theta_1,arm_joint[1],arm_joint_past[1]);
  //theta_1.write(arm_joint[1]);
  delay(100);
  // theta_2.write(arm_joint[2]);
  moveServo(theta_2,arm_joint[2],arm_joint_past[2]);
  delay(100);
   

  delay(100);
  Serial.println("Finish armm");
  
  Serial.println(arm_joint_past[0]);
  

  if (arm_joint[5] == 0)
  {
    Serial.println("Open EF");
    quad_servos.setPWM(8, 0, EF_Open); // EF
    delay(100);
  }
  else
  {
    Serial.println("CLose EF");
    quad_servos.setPWM(8, 0, 320); // EF
    delay(100);
  }
  arm_joint_past[0] =  arm_joint[0];
  arm_joint_past[1] = arm_joint[1];
  arm_joint_past[2] = arm_joint[2];
  arm_joint_past[3] = arm_joint[3];
  arm_joint_past[4] = arm_joint[4];
  
 


}

// ang  is calculated as the 4096*(duty*f(Hz)/1000)
// Servos frecuency 50
// Cero position is at 0.5 ms   down = 103
// 180 position is at 2.5 ms down  = 520

void setServo(uint8_t n_servo, int angulo) {
  int duty;
  duty=map(angulo,0,180,pos0, pos180);
  quad_servos.setPWM(n_servo, 0, duty);  
}
int BL_00 = 160;
int BR_00 = 170;
void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.setTimeout(50);
  Serial1.setTimeout(5);
   //pinMode(13, OUTPUT);  
 // digitalWrite(13, LOW);
  int minPulse = 1000; // minimum servo position, us (microseconds) (Original program was 640)
  int maxPulse = 2000; // maximum servo position, us (original program was 2400)
  theta_0.attach(8,minPulse,maxPulse);  // attaches the servo on pin 9 to the servo object
  theta_1.attach(9);  
  theta_2.attach(10);   
  theta_3.attach(11);  
  theta_4.attach(12);  
  test_leg.attach(2); 
  test_leg2.attach(3);
  
  
  
  FL_1.attach(44);
  FL_2.attach(13);
 
  FR_1.attach(45);
  FR_2.attach(46);

 
  BL_1.attach(4);
  BL_2.attach(5);
 
  BR_1.attach(6);
  BR_2.attach(7);
  //gripper.attach(12);

  
  theta_0.write(origin_m0);
  
  theta_1.write(origin_m1);
  
  theta_2.write(origin_m2);
  
  theta_3.write(origin_m3);
  
  theta_4.write(origin_m4);
  
  FL_1.write(origin_FL_1);
  
  FL_2.write(origin_FL_2);
  
  BL_1.write(origin_BL_1);
 
  BL_2.write(origin_BL_2);
  
  FR_1.write(origin_FR_1);
  
  FR_2.write(origin_FR_2);
  
  BR_1.write(origin_BR_1);
 
  BR_2.write(origin_BR_2);


  
  test_leg2.write(50); // FL
  test_leg.write(50); // FL
  //setServo(0,0); // BR
  // setServo(1,0); // BL
  //gripper.write(0);

  
  quad_servos.begin();
  quad_servos.setPWMFreq(60);
  //quad_servos.setPWM(6, 0, 150); // S
  quad_servos.setPWM(11, 0, 150); // BR
  quad_servos.setPWM(12, 0, 170); // FR
  quad_servos.setPWM(1, 0, 180); // BL
  quad_servos.setPWM(8, 0,200);
 
  origin();
   
  
  Serial.println("Start Up");

}
//----- helper para partir "a,b,c" en enteros -----
int parseCSV(char *line, int *out, int maxLen) {
  int n = 0;
  for (char *tok = strtok(line, ","); tok && n < maxLen; tok = strtok(NULL, ",")) {
    out[n++] = atoi(tok);
  }
  return n;                           // nº de valores realmente leídos
}
void flushSerial1() {
  while (Serial1.available()) Serial1.read();
}
int repeat_gait = 0;
int current_gait = 0;
void loop() {
  
 
  String command_s = "";
  String command_s2 = "";

    if (Serial1.available() == 0) {command_s = "Idle";}     //wait for data available
    command_s =  Serial1.readStringUntil('\n');
    
      // Print the received string
 
  if (Serial.available() == 0) {command_s2 = "Idle";}     //wait for data available
    command_s2 =  Serial.readStringUntil('\n');

 Serial1.println("True");
  //Serial.println("This");
 /* command.trim();                        // remove any \r \n whitespace at the end of th|e String
  String motor = command.substring(0, 2); // first two posistions to define motor 
  String motor2 = command.substring(1, 2); // first two posistions to define motor 
  String angle = command.substring(2, 6); // position from  2 - 4 angle 
  int movement_motor = angle.toInt();
  */
  /*Serial.print("Message from orange: ");
  Serial.println(command_s);
  Serial.print("Message from PC: ");
  Serial.println(command_s2); */
  if(command_s == "check")
  {
    Serial1.println("check");
  }
  switch (command) {

    case arm:
   
    if (command_s.length() > 0) {               // ha llegado la 2ª línea
      char buf[64];
      command_s.toCharArray(buf, sizeof(buf));  // String → char[]
      int vals[6];    
      int x = 0;
      // 5 joints + tiempo
      if (parseCSV(buf, vals, 6) == 6) {        // todo correcto
        for (int i = 0; i <= 5; i++) 
        {
          Serial.println(vals[i]);
         
          arm_joint[i] = vals[i]+x;
          
        }
        MOVING_TIME   = vals[5];
  
        moveStartTime = millis();
        moveArm();
        Serial1.println("True");
        Serial1.print("True");// respuesta OK
      } else {
        Serial1.println("False");               // formato erróneo
      }
      Serial1.println("True");
      Serial1.print("True");
      command = idle;                           // vuelve al estado inactivo
    }
    else if (command_s2.length() > 0) {               // ha llegado la 2ª línea
      char buf[64];
      command_s2.toCharArray(buf, sizeof(buf));  // String → char[]
      int vals[6];   
       Serial.println("Move arm2");// 5 joints + tiempo
      if (parseCSV(buf, vals, 6) == 6) {        // todo correcto
        for (int i = 0; i <= 5; i++) 
        {
          Serial.println(vals[i]);
          arm_joint[i] = vals[i];
        }
        MOVING_TIME   = vals[5];
  
        moveStartTime = millis();
        moveArm();
        Serial1.println("True");
        Serial1.print("True");// respuesta OK
      } else {
        Serial1.println("False");               // formato erróneo
      }
      Serial1.println("True");
      Serial1.print("True");
      command = idle;                           // vuelve al estado inactivo
    }
    break;

    case robot:
      Serial.print("robot ");
      if (Index < 12)
      {
        quad_joint[Index] = command_s.toInt();
        Index = Index +1;
      }
      else if (Index == 12){
        Index = 0;
        command = idle;
        MOVING_TIME = command_s.toInt();
        Serial1.print("RObot movements");
        for(int i =0;i<=11;i++){
          Serial1.println(quad_joint[i]);
        }
        quad_joint[3] = 40;
        moveStartTime = millis(); // start moving
        moveQuad();
        Serial.print("True");
        Serial1.println("True");
      }
    break;
    case step_1:
      Serial.print("step_1 ");
      Serial1.println("move");
      move_step_1();
      Serial1.println("True");
      Serial.print("True");
      command = idle;
    break;
    case step_2:
      Serial.print("step_2 ");
      Serial1.println("move");
      move_step_1();
      current_gait +=1;
      if ( current_gait <= 5 )
      {

        Serial1.println("True");
        Serial1.print("True");
        Serial.println("True");
        command = idle;
      }
      Serial.println("Gait");
      break;
    case step_3:
      Serial.print("step_3 ");
      move_step_3();
      Serial1.print("True");
      Serial.print("True");
      command = idle;
      break;
    case right:
      Serial.print("step_3 ");
      move_right();
      Serial1.print("True");
      Serial.print("True");
      command = idle;
      break;
    case left:

      Serial.print("step_3 ");
      move_left();
      Serial1.print("True");
      Serial.print("True");
      command = idle;
      break;
    case reset_orig:
      Serial.print("reset_orig ");
      reset_move();
      command = idle;
      Serial.print("True");
      Serial1.print("True");
    break;
    case orig:
      Serial.print("origin ");
      origin();
      command = idle;
      Serial.print("True");
      Serial1.println("True");
    
    case idle: 
      Serial.println("Idle");
     Serial1.println("True");
     Serial1.print("True");
      if (command_s == "ARM" || command_s2 == "ARM"  )
      {
        command = arm;
        Serial.println("move ARM");
        Serial1.println("move");
      }
      else if (command_s == "m2" || command_s2 == "m2" )
      {
        command = robot;
      }
      else if (command_s == "m4" || command_s2 == "m4"  )
      {
        command = step_1;
        Serial1.println("move");
      }
      else if (command_s == "m3" )
      {
        command = step_2;
        Serial1.println("move");
       
      }
       else if (command_s == "m5" )
      {
        command = step_3;
      }
      else if (command_s == "r" )
      {
        command = right;
      }
      else if (command_s == "l" )
      {
        command = left;
      }
      else if (command_s == "reset" )
      {
        command = reset_orig;
      }
      else if (command_s == "origin"  || command_s2 == "origin"  )
      {
        command = orig;
        Serial1.println("move");
      }
      else if (command_s == "sit" ||command_s2 == "sit"  )
      {
        sit();
        command = orig;
        Serial1.println("move");
      }
    break;

    default:
       Serial.println("Idle");
       Serial1.println("True");
       Serial1.print("True");
       Serial1.println(" True");
  
    
  }
   if( command_s2 == "o" )
  {
    Serial.println("Open this");
    quad_servos.setPWM(8, 0, EF_Open); // EF
   move_step_1();
    
   
   
   

   

   
    
  }
  if( command_s2 == "c" )
  {
    Serial.println("Close this");
    quad_servos.setPWM(8, 0, EF_Close); // EF
    
   delay(2000);
   
    FL_1.write(0);
  
   delay(2000);
    
       FL_1.write(90);
  
  }

 
  

}



void origin(){
  Serial1.println("Origin M0");
  arm_joint[0] = origin_m0;
  arm_joint[1] = origin_m1;
  arm_joint[2] = origin_m2;
  arm_joint[3] = origin_m3;
  arm_joint[4] = origin_m4;
  delay(100);
  moveServo(theta_1,arm_joint[1],arm_joint_past[1]);
  moveServo(theta_2,arm_joint[2],arm_joint_past[2]);
  moveServo(theta_0, arm_joint[0],arm_joint_past[0]);
  moveServo(theta_3,arm_joint[3],arm_joint_past[3]);
  moveServo(theta_4,arm_joint[4],arm_joint_past[4]); 
  Serial.println("Moving armm");
  
  Serial.println("Finish armm");
 
  arm_joint_past[0] =  arm_joint[0];
  arm_joint_past[1] = arm_joint[1];
  arm_joint_past[2] = arm_joint[2];
  arm_joint_past[3] = arm_joint[3];
  arm_joint_past[4] = arm_joint[4];
  


 Serial.println("1");
  FL_1.write(origin_FL_1);
  FL_2.write(origin_FL_2);

  delay(50);
  BL_1.write(origin_BL_1);
  BL_2.write(origin_BL_2);

  delay(50);
  FR_1.write(origin_FR_1);
  FR_2.write(origin_FR_2);

  delay(50);
  BR_1.write(origin_BR_1);
  BR_2.write(origin_BR_2);
  delay(50);
  
  
   

  
}
void origin2(){

  delay(20);
  FL_1.write(origin_FL_1);
  delay(20);
  FL_2.write(origin_FL_2);
  BL_1.write(origin_BL_1);
  delay(20);
  BL_2.write(origin_BL_2);
  FR_1.write(origin_FR_1);
  delay(20);
  FR_2.write(origin_FR_2);
  delay(20);
  BR_1.write(origin_BR_1);
  delay(20);
  BR_2.write(origin_BR_2);
  delay(20);
}
void reset_move(){
  Serial.println("Reset");
  

}
void move_pos(){
  FR_2.write(origin_FR_2+15);
  FL_2.write(origin_FR_2+15);
  BR_2.write(origin_FR_2+15);
  BL_2.write(origin_FR_2+15);
  delay(1000);
}

void return_up(){
 FR_1.write(origin_FR_1);
 FR_2.write(origin_FR_2);
 delay(50);
 BL_1.write(origin_BL_1);
 BL_2.write(origin_BL_2);
 delay(50);
 BR_1.write(origin_BR_1);
 BR_2.write(origin_BR_2);
 delay(50);
 FL_1.write(origin_FL_1);
 FL_2.write(origin_FL_2);
 delay(50);
  
}