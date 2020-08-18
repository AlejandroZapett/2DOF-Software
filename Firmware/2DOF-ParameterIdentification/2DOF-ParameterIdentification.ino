
/*--------------------------------------------Variables for Space State------------------------------*/

double q1, q2, dq1, dq2, ddq1, ddq2;
double last_q1, last_q2, last_dq1, last_dq2;
const float pi = 3.1416;
const float MAX_ANGLE_RAD = 2*pi;
const int MAX_ANGLE_DEG = 360;

/*--------------------------------------------Variebles for LM298N----------------------------------*/
//u1
int IN1_1  = 11; 
int IN1_2  = 12;
int PWM1 = 13;
//u2 8,9,10
int IN2_1  = 10; 
int IN2_2  = 9;
int PWM2 = 8;
int MIN_PWM= 50; 
int MAX_PWM= 255 - MIN_PWM;

/*--------------------------------------Variables for incremental encoder-----------------------------*/
//q1
volatile long contador1 =  0;   
byte          ant1      =  0;    
byte          act1      =  0;
int           encA1     =  44;                  // Signal for channel A
int           encB1     =  42; 
//q2
volatile long contador2 =  0;   
byte          ant2      =  0;    
byte          act2      =  0;
int           encA2     =  40;                  // Signal for channel A
int           encB2     =  38; // Signal for channel B

const int   MIN_MAX_POST     =  2400;                 // Limit the maximun position

/*------------------------------------- Interruption Functions----------------------------------------*/

// Encoder x4. Execute when interruption pin jumps.
void encoder1(void){ 
  //Serial.println(ant);
  ant1=act1;                            // Saved act (current read) in ant (last read)
  act1 = digitalRead(encA1)<<1|digitalRead(encB1);
  if(ant1==0 && act1==1)        contador1++;  // Increase the counter for forward movement
  else if(ant1==1  && act1==3)  contador1++;
  else if(ant1==3  && act1==2)  contador1++;
  else if(ant1==2  && act1==0)  contador1++;
  else contador1--;                         // Reduce the counter for backward movement

  if(contador1 > MIN_MAX_POST){ contador1 = contador1 - 2400; }
  else if(contador1 < -MIN_MAX_POST){ contador1 = contador1 + 2400; }

  last_q1 = q1;
  
  //Set positions
  //q1 = (double) contador1 * (MAX_ANGLE_RAD / 2400); // en radianes

  q1 = (double) contador1 * (0.15); // en grados
}

void encoder2(void){ 
  //Serial.println(ant2);
  ant2=act2;                            // Saved act (current read) in ant (last read)
  act2 = digitalRead(encA2)<<1|digitalRead(encB2);
  if(ant2==0 && act2==1)        contador2++;  // Increase the counter for forward movement
  else if(ant2==1  && act2==3)  contador2++;
  else if(ant2==3  && act2==2)  contador2++;
  else if(ant2==2  && act2==0)  contador2++;
  else contador2--;                         // Reduce the counter for backward movement

  if(contador2 > MIN_MAX_POST){ contador2 = contador2 - 2400; }
  else if(contador2 < -MIN_MAX_POST){ contador2 = contador2 + 2400;}
    
  last_q2 = q2;
  //Set positions
  //q2 = (double) contador2 * (MAX_ANGLE_RAD / 2400); // en radianes

  q2 = (double) contador2 * (0.15); // en grados
}

/*----------------------------------------------Variables for algorithm------------------------------*/

double q1d, q2d, dq1d, dq2d, ddq1d, ddq2d;      // para el seguimiento de trayectorias
double sumQ1, sumQ2;
double Setpoint;                                // para control de posición
double SampleTime = 5;                         // en milisegundos           
double Kp1=6, Kd1=0.2, Ki1 = 0.05;                        // PD gain
double Kp2=4, Kd2=0.002, Ki2 = 0.15;                        // PD gain

/*----------------------------------------------------------------------------------------------------*/
unsigned long t0, t; 
unsigned long t_actual, t_anterior; //For update state
unsigned long current_t, last_t; //For inner loop

/*---------------------------------------------- Setup section ---------------------------------------*/
void setup() {
  
  Serial.begin(115200);
  while(!Serial){
    ;
  }
  
  //---- Motors setup ----:
  for(int i=8;i<=13;i++){
    pinMode(i,OUTPUT);
  }
  //u1
  moveMotor2(0);
  //u2
  moveMotor1(0);

  //---- Encoder setup ----:
  //q1
  attachInterrupt(digitalPinToInterrupt(encA1), encoder1, CHANGE); // rising and falling flank
  attachInterrupt(digitalPinToInterrupt(encB1), encoder1, CHANGE); // rising and falling flank
  //q2
  attachInterrupt(digitalPinToInterrupt(encA2), encoder2, CHANGE); // rising and falling flank
  attachInterrupt(digitalPinToInterrupt(encB2), encoder2, CHANGE); // rising and falling flank

  //---- Time setup ----:
  t0 = millis();
  last_t = t0;

  setInitialConditions();

  delay(2000);

}

/*-------------------------------------------------- Main Loop ---------------------------------------*/
void loop() {

  if(!Serial.available()){
    // Time variables 
    t = millis() - t0;
    
    current_t = millis();
    
    if(current_t - last_t >= SampleTime){
      
      last_t = current_t;
      //Establece las velocidades y aceleraciones
      setState();
      //Establece la referencia a seguir
      setReference();
      
      //inputForDebugPurposes();

      //Estable las entradas de control por medio del PID
      pidInput();
      
    }
  }

}


/*-------------------------------------------- Backup functions ---------------------------------------*/

void setState() {
  
  last_dq1 = dq1;
  last_dq2 = dq2;

  //Set velocities
  dq1 = (1000*(q1 - last_q1))/(SampleTime);
  dq2 = (1000*(q2 - last_q2))/(SampleTime);

  //Set accelerations
  ddq1 = (1000*(dq1 - last_dq1))/(SampleTime);
  ddq2 = (1000*(dq2 - last_dq2))/(SampleTime);
  
}

void moveMotor2(int pwm){
  if(pwm > 0){
     //----- Giro positivo ---------
    digitalWrite(IN2_1,LOW);
    digitalWrite(IN2_2,HIGH);
    pwm = pwm + MIN_PWM;
    //-----------------------------
  } else if(pwm < 0) {
    // ------Giro negativo----------
    digitalWrite(IN2_1,HIGH);
    digitalWrite(IN2_2,LOW);
    pwm = (-1)*pwm + MIN_PWM;
    //-----------------------------
  }
  analogWrite(PWM2,pwm);
}

void moveMotor1(int pwm){
  if(pwm > 0){
     //----- Giro positivo ---------
    digitalWrite(IN1_2,LOW);
    digitalWrite(IN1_1,HIGH);
    pwm = pwm + MIN_PWM;
    //-----------------------------
  } else if(pwm < 0) {
    // ------Giro negativo----------
    digitalWrite(IN1_2,HIGH);
    digitalWrite(IN1_1,LOW);
    pwm = (-1)*pwm + MIN_PWM;
    //-----------------------------
  }
  analogWrite(PWM1,pwm);
}

void setInitialConditions(){
  contador1 = 600; // equivalente 90º
  contador2 = 0; // equivalente a 0º

  //last_q1 = contador1 * (MAX_ANGLE_RAD / 2400);
  //last_q2 = contador2 * (MAX_ANGLE_RAD / 2400);

  last_q1 = contador1 * (0.15);
  last_q2 = contador2 * (0.15);

  last_dq1 = 0;
  last_dq2 = 0;

  q1d = 115; // Grados
  q2d = -25; // Grados

  sumQ1 = 0;
  sumQ2 = 0;
}

void setReference(){
  q1d = (8*sin(0.0008*t-1.4)+90);// 90 a 120 º
  q2d = (-30*sin(0.001*t-1.4)-30);// 0 a -60 º

  dq1d = (8*0.0008*sin(0.001*t-1.4));
  dq2d = (-30*0.001*sin(0.001*t-1.4));
}

void pidInput(){

  sumQ1 += ( Ki1*(q1d-q1) );
      if(sumQ1 > MAX_PWM) sumQ1 = MAX_PWM;
      else if(sumQ1 < -MAX_PWM) sumQ1 = -MAX_PWM;
  sumQ2 += ( Ki2*(q2d-q2) );
      if(sumQ2 > MAX_PWM) sumQ2 = MAX_PWM;
      else if(sumQ2 < -MAX_PWM) sumQ2 = -MAX_PWM;
  
  int u1 = (int) ( Kp1*(q1d - q1) + Kd1*(dq1d - dq1) + sumQ1 + 200*cos(q1*(pi/180)) );
      if(u1 > MAX_PWM) u1 = MAX_PWM;
      else if(u1 < -MAX_PWM) u1 = -MAX_PWM;
  int u2 = (int) (Kp2*(q2d - q2) + Kd2*(dq2d - dq2) + sumQ2 );
      if(u2 > MAX_PWM) u2 = MAX_PWM;
      else if(u2 < -MAX_PWM) u2 = -MAX_PWM;

  moveMotor1(u1);
  moveMotor2(u2);
  
  //Envía los datos por comunicación serie
  printSerialMessages(u1,u2);
  //printDebugMessages(u1, u2);
  
}

void printSerialMessages(int u1, int u2){
  //Entradas
  Serial.print(u1); Serial.print(",");
  Serial.print(u2); Serial.print(",");
  //Posiciones
  Serial.print(q1d); Serial.print(",");
  Serial.print(q1); Serial.print(",");
  Serial.print(q2d); Serial.print(",");
  Serial.print(q2); Serial.print(",");
  //Velocidades
  Serial.print(dq1d); Serial.print(",");
  Serial.print(dq1); Serial.print(",");
  Serial.print(dq2d); Serial.print(",");
  Serial.print(dq2); Serial.print(",");
  //Aceleraciones
  Serial.print(ddq1d); Serial.print(",");
  Serial.print(ddq1); Serial.print(",");
  Serial.print(ddq2d); Serial.print(",");
  Serial.println(ddq2);
  
}

/*-------------------------------------------- Debug functions ---------------------------------------*/

void printDebugMessages(int u1, int u2){

  Serial.println(q1d);
  Serial.println(q2d);
  
  //Serial.print("q1: "); Serial.println(q1);
  //Serial.print("q2: "); Serial.println(q2);
  //Serial.print("dq1: "); Serial.println(dq1);
  //Serial.print("dq2: "); Serial.println(dq2);
  //Serial.print("ddq1: "); Serial.println(ddq1);
  //Serial.print("ddq2: "); Serial.println(ddq2);
  //Serial.print("u1: "); Serial.println(u1);
  //Serial.print("u1: "); Serial.println(u1);
  //Serial.println();
  
}

void inputForDebugPurposes(){
  
  //moveMotor1(205);
  //moveMotor2(u2);

  //printDebugMessages(u1,u2);
  
}
