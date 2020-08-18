/******************************************************************************************************
      CDMX - México 2020
      AUTOR: Alejandro Zapett
      PROYECTO: Brazo robótico de dos grados de libertad
       
      PERIFÉRICOS:
      
      > ENCODER: (5V) 600PPR
        360grados = 2400 pulsos
        0.15 grados / Pulso
        
      > L298N (5v & 19v)
        IN1
        IN2
        PWM 

******************************************************************************************************/

/*--------------------------------------------Variables for Space State------------------------------*/

double q1, q2, dq1, dq2, ddq1, ddq2;
double last_q1, last_q2, last_dq1, last_dq2;
const float pi = 3.1416;
const float MAX_ANGLE_RAD = 2*pi;
const int MAX_ANGLE_DEG = 360;

/*--------------------------------------------Variables for LM298N----------------------------------*/
//u1
int IN1_1  = 11; 
int IN1_2  = 12;
int PWM1 = 13;
//u2 8,9,10
int IN2_1  = 10; 
int IN2_2  = 9;
int PWM2 = 8;
int MIN_PWM= 60; 
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
  q1 = (double) contador1 * (MAX_ANGLE_RAD / 2400); // en radianes

  //q1 = (double) contador1 * (0.15); // en grados
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
  q2 = (double) contador2 * (MAX_ANGLE_RAD / 2400); // en radianes

  //q2 = (double) contador2 * (0.15); // en grados
}

/*---------------------------------------Variables for control algorithm------------------------------*/

double q1d, q2d, dq1d, dq2d, ddq1d, ddq2d;      // para el seguimiento de trayectorias
double last_q1d, last_q2d, last_dq1d, last_dq2d;
double SampleTime = 10;                         // en milisegundos   
        
//------------------------ Par calculado
double Kp=2, Kv=3;

//------------------------ Passivity Based Control
double K1 = 20, K2 = 15;
double L1 = 20, L2 = 15;

//------------------------ Referencia
unsigned long t1=3, t2=3, t_retraso = 5; 

const int idx = (3+3+5)/0.01;

double pos1_d[idx];
double vel1_d[idx];
double ace1_d[idx];

double pos2_d[idx];
double vel2_d[idx];
double ace2_d[idx];

double x1=0.05, x2=0.08, x3=0.08, x4=0.05;
double y=0.22, y2=0.22, y3=0.19, y4=0.19;
double x_p = 0.05, y_p = 0.24;
double dx_p = 0, dy_p = 0;
double ddx_p = 0, ddy_p = 0;

/*---------------------------------------Variables of mathematical model -----------------------------*/

struct M {
  double m1;
  double m2;
  double m3;
  double m4;
};

struct C {
  double c1;
  double c2;
};

struct G {
  double g1;
  double g2;
};

struct F {
  double f1;
  double f2;
};

M M1 = {53.6695, 9.074, -3.147, -9.452};
C C1 = {-12.8635, -6.2741};
G G1 = {376.7097, 109.7585};
F F1 = {54.7886, 30.0440};

M M2 = {-2.7536, 5.7987, 50.9851, 0};
C C2 = {13.6817, 0};
G G2 = {77.5768, 0};
F F2 = {34.9586, 34.2284};

//Cinemática direca e inversa
double l1 = 0.12, l2 = 0.14;

/*----------------------------------------------------------------------------------------------------*/
unsigned long t0, t; 
unsigned long t_anterior; //For update reference
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

  //----Reference setup --:
  trayectoryReference2();
  
  delay(3000);
  
  //------ Time setup -----:
  t0 = millis();
  last_t = t0;

  //---- Control setup ----:
  setInitialConditions();

}

/*-------------------------------------------------- Main Loop ---------------------------------------*/
void loop() {
  
  if(!Serial.available()){
    // Time variables 
    t = millis() - t0;
    current_t = millis();
    
    if(current_t - last_t >= SampleTime){
      
      //********* Actualización del estado *******
      last_t = current_t;
      setState();
      //*********** Entradas de control *********
      parCalculadoInput();
      //passivityBasedControlInput();
      //*****************************************

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

/*--------------------------------------- Condiciones iniciales ---------------------------------------*/

void setInitialConditions(){
  
  contador1 = 600; // equivalente 90º
  contador2 = 0; // equivalente a 0º

  last_q1 = contador1 * (MAX_ANGLE_RAD / 2400); // Radianes
  last_q2 = contador2 * (MAX_ANGLE_RAD / 2400); // Radianes

  //last_q1 = contador1 * (0.15); //Grados 
  //last_q2 = contador2 * (0.15); // Grados

  q1 = last_q1;
  q2 = last_q2;

  last_dq1 = 0;
  last_dq2 = 0;

  dq1 = 0;
  dq2 = 0;
}

/*----------------------------------------------- Referencia ----------------------------------------*/
void positionReference(){
  x_p = x1;
  y_p = y;
  cinematicaInversa(x_p,y_p);
  dq1d = 0;
  dq2d = 0;
  ddq1d = 0;
  ddq2d = 0;
}

void trayectoryReference2(){ // --> La referencia es una línea horizontal Línea 
  //last_q1d = q1d; last_q2d = q2d; last_dq1d = dq1d; last_dq2d = dq2d;
  int t_aux = 0;
  for(int i=0; i<1100; i++){
    t_aux = i;
    if(i/100 <= t_retraso) {
      x_p = x1; y_p = y; dx_p = 0; dy_p = 0; ddx_p = 0; ddy_p = 0;
    }
    else if(i/100 <= t_retraso+t1 && i/100 > t_retraso){
      double Po = x1; double Pf = x1 + 0.08;
      unsigned long ti = (t_aux/100) - (t_retraso);
  
      float lambda = (float)computeLambda(ti,t1);
      float dlambda = (float)computeDLambda(ti,t1);
      float ddlambda = (float)computeDDLambda(ti,t1);
  
      x_p = Po + lambda*(Pf-Po);  y_p = y;
      dx_p = dlambda*(Pf-Po);     dy_p = 0;
      ddx_p = ddlambda*(Pf-Po);   ddy_p = 0;
    }
    else if(i/100 <= t_retraso+t1+t2 && i/100 > t_retraso+t1){
      double Po = x1 + 0.08; double Pf = x1;
      unsigned long ti = (t_aux/100)-(t1+t_retraso);
  
      float lambda = (float)computeLambda(ti,t2);
      float dlambda = (float)computeDLambda(ti,t2);
      float ddlambda = (float)computeDDLambda(ti,t2);
  
      x_p = Po + lambda*(Pf-Po);   y_p = y2;
      dx_p = dlambda*(Pf-Po);      dy_p = 0;
      ddx_p = ddlambda*(Pf-Po);    ddy_p = 0;
    }
    cinematicaInversa(x_p,y_p);
    cinematicaInversaDif(dx_p,dy_p,ddx_p,ddy_p);

    pos1_d[i] = q1d;     pos2_d[i] = q2d;
    vel1_d[i] = dq1d;    vel2_d[i] = dq2d;
    ace1_d[i] = ddq1d;   ace2_d[i] = ddq2d;

  }
}

void cinematicaInversa(double xp, double yp){
  
  double r=sqrt(pow(xp,2)+pow(yp,2));
  double alfa=atan2(yp,xp);
  double beta=acos((pow(l1,2)+pow(l2,2)-pow(r,2))/(2*l1*l2));
  double gama=acos((pow(l1,2)-pow(l2,2)+pow(r,2))/(2*l1*r));

  double th1b=alfa+gama;
  double th2b=pi+beta;
  q1d=th1b;
  q2d=th2b-2*pi;
}

void cinematicaInversaDif(double dxp,double dyp, double ddxp, double ddyp){
  double j11 = -cos(q1d + q2d)/(l1*cos(q1d + q2d)*sin(q1d) - l1*sin(q1d + q2d)*cos(q1d));
  double j12 = -sin(q1d + q2d)/(l1*cos(q1d + q2d)*sin(q1d) - l1*sin(q1d + q2d)*cos(q1d));
  double j21 = (l2*cos(q1d + q2d) + l1*cos(q1d))/(l1*l2*cos(q1d + q2d)*sin(q1d) - l1*l2*sin(q1d + q2d)*cos(q1d));
  double j22 = (l2*sin(q1d + q2d) + l1*sin(q1d))/(l1*l2*cos(q1d + q2d)*sin(q1d) - l1*l2*sin(q1d + q2d)*cos(q1d));
  dq1d = ( j11 )*dxp + ( j12 )*dyp;
  dq2d = ( j21 )*dxp + ( j22 )*dyp;
  cinematicaInversa2Dif(dxp, ddxp, dyp, ddyp, j11,j12,j21,j22);
}

void cinematicaInversa2Dif(double dxp, double ddxp, double dyp, double ddyp, double j11, double j12, double j21, double j22){
  double dj11=(cos(q1d + q2d)*(l1*cos(q1d)*cos(q1d + q2d)*dq1d - l1*cos(q1d)*cos(q1d + q2d)*(dq1d + dq2d) + l1*sin(q1d)*sin(q1d + q2d)*dq1d - l1*sin(q1d)*sin(q1d + q2d)*(dq1d + dq2d)))/pow((l1*cos(q1d)*sin(q1d + q2d) - l1*sin(q1d)*cos(q1d + q2d)),2) - (sin(q1d + q2d)*(dq1d + dq2d))/(l1*cos(q1d)*sin(q1d + q2d) - l1*sin(q1d)*cos(q1d + q2d));
  double dj12=(sin(q1d + q2d)*(l1*cos(q1d)*cos(q1d + q2d)*dq1d - l1*cos(q1d)*cos(q1d + q2d)*(dq1d + dq2d) + l1*sin(q1d)*sin(q1d + q2d)*dq1d - l1*sin(q1d)*sin(q1d + q2d)*(dq1d + dq2d)))/pow((l1*cos(q1d)*sin(q1d + q2d) - l1*sin(q1d)*cos(q1d + q2d)),2) + (cos(q1d + q2d)*(dq1d + dq2d))/(l1*cos(q1d)*sin(q1d + q2d) - l1*sin(q1d)*cos(q1d + q2d));
  double dj21=(l2*sin(q1d + q2d)*(dq1d + dq2d) + l1*sin(q1d)*dq1d)/(l1*l2*cos(q1d)*sin(q1d + q2d) - l1*l2*sin(q1d)*cos(q1d + q2d)) + ((l1*cos(q1d) + l2*cos(q1d + q2d))*(l1*l2*sin(q1d)*sin(q1d + q2d)*(dq1d + dq2d) - l1*l2*cos(q1d)*cos(q1d + q2d)*dq1d + l1*l2*cos(q1d)*cos(q1d + q2d)*(dq1d + dq2d) - l1*l2*sin(q1d)*sin(q1d + q2d)*dq1d))/pow((l1*l2*cos(q1d)*sin(q1d + q2d) - l1*l2*sin(q1d)*cos(q1d + q2d)),2);
  double dj22=((l1*sin(q1d) + l2*sin(q1d + q2d))*(l1*l2*sin(q1d)*sin(q1d + q2d)*(dq1d + dq2d) - l1*l2*cos(q1d)*cos(q1d + q2d)*dq1d + l1*l2*cos(q1d)*cos(q1d + q2d)*(dq1d + dq2d) - l1*l2*sin(q1d)*sin(q1d + q2d)*dq1d))/pow((l1*l2*cos(q1d)*sin(q1d + q2d) - l1*l2*sin(q1d)*cos(q1d + q2d)),2) - (l2*cos(q1d + q2d)*(dq1d + dq2d) + l1*cos(q1d)*dq1d)/(l1*l2*cos(q1d)*sin(q1d + q2d) - l1*l2*sin(q1d)*cos(q1d + q2d));
  ddq1d = ( dj11 )*dxp + ( dj12 )*dyp + ( j11 )*ddxp + ( j12 )*ddyp;
  ddq2d = ( dj21 )*dxp + ( dj22 )*dyp + ( j21 )*ddxp + ( j22 )*ddyp;
}

double computeLambda(unsigned long ti,unsigned long tf){
  double lambda = (10/(pow(tf,3)))*pow(ti,3) - (15/(pow(tf,4)))*pow(ti,4) + (6/(pow(tf,5)))*pow(ti,5);
  return lambda;
}

double computeDLambda(unsigned long ti,unsigned long tf){
  double dlambda = (30/(pow(tf,3)))*pow(ti,2) - (60/(pow(tf,4)))*pow(ti,3) + (30/(pow(tf,5)))*pow(ti,4);
  return dlambda;
}

double computeDDLambda(unsigned long ti,unsigned long tf){
  double dlambda = (60/(pow(tf,3)))*ti - (180/(pow(tf,4)))*pow(ti,2) + (120/(pow(tf,5)))*pow(ti,3);
  return dlambda;
}

/*----------------------------------------- Entradas de control ---------------------------------------*/


void parCalculadoInput(){
  if(t/1000){
    q1d = pos1_d[(int) (t/10)];   q2d = pos2_d[(int) (t/10)];
    dq1d = vel1_d[(int) (t/10)];  dq2d = vel2_d[(int) (t/10)];
    ddq1d = ace1_d[(int) (t/10)]; ddq2d = ace2_d[(int) (t/10)];
  } else {
    positionReference();
  }
  
  double M11 = (M1.m1 + M1.m2*cos(q1));
  double M12 = (M1.m3 + M1.m4*cos(q2));
  double M21 = (M2.m1 + M2.m2*cos(q2));
  double M22 = M2.m3;

  double C11 = C1.c1*sin(q2)*dq2;
  double C12 = C1.c2*sin(q2)*dq2;
  double C21 = C2.c1*sin(q2)*dq1;
  
  double G11 = G1.g1*cos(q1)+G1.g2*cos(q1+q2);
  double G21 = G2.g1*cos(q1+q2);
  
  double F11 = F1.f1*dq1;
  double F21 = F2.f1*dq2;

  double e1 = q1d - q1;
  double e2 = q2d - q2;

  double de1 = dq1d - dq1;
  double de2 = dq1d - dq1;

  int u1 = (int) (M11*(ddq1d+Kv*de1+Kp*e1)+M12*(ddq2d+Kv*de2+Kp*e2)+C11*dq1+C12*dq2+G11);
      if(u1 > MAX_PWM) u1 = MAX_PWM;
      else if(u1 < -MAX_PWM) u1 = -MAX_PWM;
  
  int u2 = (int) (M21*(ddq1d+Kv*de1+Kp*e1)+M22*(ddq2d+Kv*de2+Kp*e2)+C21*dq1+G21);
      if(u2 > MAX_PWM) u2 = MAX_PWM;
      else if(u2 < -MAX_PWM) u2 = -MAX_PWM;

  moveMotor1(u1);
  moveMotor2(u2);

  //Envía los datos por comunicación serie
  printSerialMessages(u1,u2);

}

void passivityBasedControlInput(){
  
  double M11 = (M1.m1 + M1.m2*cos(q1));
  double M12 = (M1.m3 + M1.m4*cos(q2));
  double M21 = (M2.m1 + M2.m2*cos(q2));
  double M22 = M2.m3;

  double C11 = C1.c1*sin(q2)*dq2;
  double C12 = C1.c2*sin(q2)*dq2;
  double C21 = C2.c1*sin(q2)*dq1;
  
  double G11 = G1.g1*cos(q1)+G1.g2*cos(q1+q2);
  double G21 = G2.g1*cos(q1+q2);
  
  double F11 = F1.f1*dq1;
  double F21 = F2.f1*dq2;

  double e1 = q1 - q1d;
  double e2 = q2 - q2d;

  double de1 = dq1 - dq1d;
  double de2 = dq1 - dq1d;

  double dqr1 = dq1d-L1*e1;
  double dqr2 = dq2d-L2*e2;

  double ddqr1 = ddq1d-L1*de1;
  double ddqr2 = ddq2d-L2*de2;

  double s1 = dq1-dqr1;
  double s2 = dq2-dqr2;

  int u1 = (int) (M11*ddqr1+M12*ddqr2+C11*dqr1+C12*dqr2+G11+F11-K1*s1);
      if(u1 > MAX_PWM) u1 = MAX_PWM;
      else if(u1 < -MAX_PWM) u1 = -MAX_PWM;
  
  int u2 = (int) (M21*ddqr1+M22*ddqr2+C21*dqr1+G21+F21-K2*s2);
      if(u2 > MAX_PWM) u2 = MAX_PWM;
      else if(u2 < -MAX_PWM) u2 = -MAX_PWM;

  moveMotor1(u1);
  moveMotor2(u2);

  //Envía los datos por comunicación serie
  printSerialMessages(u1,u2);
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

int sign(double var){
  if(var > 0) return 1;
  else if(var < 0) return -1;
  else return 0;
}
