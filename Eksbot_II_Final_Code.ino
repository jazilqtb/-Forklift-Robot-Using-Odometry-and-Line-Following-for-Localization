#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16,2);

Servo servo;
#define pinSrv 10
int sudut=0;

#define pbenter 19 //A
#define pbatas 48 //B
#define pbbawah 50 //C
#define pbback 18 //D

int up,down;
int upLastState = 0;
int downLastState = 0;
bool latchenter = false;
bool latchback = false;

#define IN1 42 // deklarasi pin IN1
#define IN2 40  // deklarasi pin IN2
#define IN3 38  // deklarasi pin IN3
#define IN4 36  // deklarasi pin IN4
#define ENA 7 // deklarasi pin ENA
#define ENB 6  // deklarasi pin ENB

int sr,sl;

int sensor[5] = {A14,A9,A13,A11,A15};
int val[5];
int valLast[5];
byte sensorValue = 0b00000;

// PID
int error;
const int Kp = 50;
const int Ki = 0;
const int Kd = 0;
double P,I,D;
double errorSum = 0;
double lastError = 0;
double output;


int mode = 0;
int mode1 = 1;
int jumlahcase = 4;

// Mobile Robot
int from=1;
int to=1;
int fromtocondition = 0;

int detectgaris;

//encoder variabel
#define chAL 2
#define chBL 22
#define chAR 3
#define chBR 23
volatile long countPulsesL=0;//volatile menandakan variabel countpulses mudah berubah;
volatile long countPulsesR=0;//volatile menandakan variabel countpulses mudah berubah;

// Variabel Roda dan Motor
float N = 11.0; //Ppr motor
float p = 46.8; //step up gear Ratio
float keliling = 22;
float rpm;

// Variabel Algoritma
int th = 32;
byte a1=0;
byte a2=0;

// Variabel perhitungan RPM
long lTimeL=0;
long lTimeR=0;
int point;
float vl=0;
float vr=0;
float ler;
float lel;

void pbInit(){
  pinMode(pbatas,INPUT);
  pinMode(pbbawah,INPUT);
  pinMode(pbenter,INPUT);
  pinMode(pbback,INPUT);
}

void pbInterruptInit(){
  attachInterrupt(digitalPinToInterrupt(pbback),buttonback,FALLING); 
  attachInterrupt(digitalPinToInterrupt(pbenter),buttonpress,FALLING);
}

void motorInit(){
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void setup() {
  Serial.begin(9600);
  servo.attach(pinSrv);

  motorInit();

  pbInit();
  pbInterruptInit();

  lcd.init();
  lcd.backlight();

  servo.write(130);
  delay(1000);

  for(int i=0;i<5;i++){
    pinMode(sensor[i],INPUT);
  }
  
  // Encoder Kiri
  pinMode(chAL,INPUT);
  pinMode(chBL,INPUT);
  attachInterrupt(digitalPinToInterrupt(chAL),encoderAL,RISING);

  // Encoder Kanan
  pinMode(chAR,INPUT);
  pinMode(chBR,INPUT);
  attachInterrupt(digitalPinToInterrupt(chAR),encoderAR,RISING);

  mode = 0;
  mode1 = 1;
  up = 0;
  upLastState = 0;
  down = 0;
  downLastState = 0;

  servo.write(180);
}

void encoderAL(){
  if(digitalRead(chBL)){
    //clockwise
    countPulsesL++;
  }else{
    countPulsesL--;
  }
}

void encoderAR(){
  if(digitalRead(chBR)){
    //clockwise
    countPulsesR--;
  }else{
    countPulsesR++;
  }
}

void buttonup1(){
  if(mode==0){
    mode=0;
  }else{
    if(mode<2){
      if(latchenter==0){
        mode++;
      }
    }else{
      mode=2;
    }
  }
}

void buttondown1(){
  if(mode==0){
    mode=0;
  }else{
    if(mode>0 && latchenter==0){
      if(latchenter==0){
        mode--;
      }
    }else{
      mode=0;
    }
  }
  //lcd.clear();
}

void buttonpress(){
  if(mode!=0){
    latchenter=!latchenter;
  }
  if(mode==0){
    mode=1;
  }
  //lcd.clear();
}

void buttonback(){
  if(mode!=0){
    latchback=!latchback;
  }
  if(mode==0){
    mode=1;
  }
  //lcd.clear();
}

void buttonup2(){
  if(fromtocondition==0){
    if(from>3){
      from = 3;
    }else if(from<1){
      from = 1;
    }else{
      from++;
    }
  }else if(fromtocondition==1){
    if(to>3){
      to = 3;
    }else if(to<1){
      to = 1;
    }else{
      to++;
    }
  }
}

void buttondown2(){
  if(fromtocondition==0){
    if(from>3){
      from = 3;
    }else if(from<1){
      from = 1;
    }else{
      from--;
    }
  }else if(fromtocondition==1){
    if(to>3){
      to = 3;
    }else if(to<1){
      to = 1;
    }else{
      to--;
    }
  }
}

void buttonup3(){
    if(mode1<4){
      if(latchenter==0){
        mode1++;
      }
    }else{
      mode1=4;
    }
}

void buttondown3(){
    if(mode1>1){
      if(latchenter==0){
        mode1--;
      }
    }else{
      mode1=1;
    }
}

void loop() {
  up = digitalRead(pbatas);
  down = digitalRead(pbbawah);

  if(up==1 && upLastState==0){
    buttonup1();
  }else if(down==1 && downLastState==0){
    buttondown1();
  }

  upLastState = up;
  downLastState = down;

  switch (mode){
    case 0:
      while(mode==0){
        up = digitalRead(pbatas);
        down = digitalRead(pbbawah);

        if(up==1 && upLastState==0){
          buttonup1();
        }else if(down==1 && downLastState==0){
          buttondown1();
        }

        upLastState = up;
        downLastState = down;

        printlcd(0,0,"   PASLON 01    ");
        printlcd(0,1,"  SLEPETTT!!    ");
      }
      break;
    case 1:
      if(latchenter){
        latchenter = false;
        fromtocondition = 0;
        String TO[3] = {"A","B","C"};
        delay(100);
        while(true){
          up = digitalRead(pbatas);
          down = digitalRead(pbbawah);

          if(up==1 && upLastState==0){
            buttonup2();
          }else if(down==1 && downLastState==0){
            buttondown2();
          }
          upLastState = up;
          downLastState = down;

          switch (fromtocondition){
            case 0:
              printlcd(0,0,">FROM : ");
              printlcd(8,0,String(from));
              printlcd(9,0,"        ");
              printlcd(0,1," TO   : ");
              printlcd(8,1,TO[to-1]);
              printlcd(9,1,"        ");
              break;
            case 1:
              printlcd(0,0," FROM : ");
              printlcd(8,0,String(from));
              printlcd(9,0,"        ");
              printlcd(0,1,">TO   : ");
              printlcd(8,1,TO[to-1]);
              printlcd(9,1,"        ");
              break;
            case 2:
              delay(100);
              int set = from*10 + to;
              while(true){
                printlcd(0,0,"MOBILE ROBOT    ");
                printlcd(0,1,"RUNNING!!!      ");

                if(set==11){
                  delay(1000);
                  moveTo(175,"MAJU");
                  delay(99999999);
                }else if(set==12){
                  delay(1000);
                  moveTo(100,"MAJU");
                  turnRight(16);
                  moveTo(55,"MAJU");
                  turnLeft(16,false);
                  moveTo(75,"MAJU");
                  delay(99999999);
                }else if(set==13){
                  delay(1000);
                  moveTo(100,"MAJU");
                  turnRight(16);
                  moveTo(110,"MAJU");
                  turnLeft(16,false);
                  moveTo(70,"MAJU");
                  delay(9999999999);

                }else if(set==21){
                  delay(1000);
                  moveTo(100,"MAJU");
                  turnLeft(16,false);
                  moveTo(55,"MAJU");
                  turnRight(16);
                  moveTo(70,"MAJU");
                  delay(99999999);
                }else if(set==22){
                  delay(1000);
                  moveTo(175,"MAJU");
                  delay(99999999);
                }else if(set==23){
                  delay(1000);
                  moveTo(100,"MAJU");
                  turnRight(16);
                  moveTo(55,"MAJU");
                  turnLeft(16,false);
                  moveTo(75,"MAJU");
                  delay(99999999);
                }else if(set==31){
                  delay(1000);
                  moveTo(100,"MAJU");
                  turnLeft(16,false);
                  moveTo(110,"MAJU");
                  turnRight(16);
                  moveTo(70,"MAJU");
                  delay(9999999999);
                }else if(set==32){
                  delay(1000);
                  moveTo(100,"MAJU");
                  turnLeft(16,false);
                  moveTo(60,"MAJU");
                  turnRight(16);
                  moveTo(75,"MAJU");
                  delay(99999999);
                }else if(set==33){
                  delay(1000);
                  moveTo(175,"MAJU");
                  delay(99999999);
                }

                if(latchback){
                  latchback=false;
                  fromtocondition = 0;
                  break;
                }
              }
              break;
          }

          if(latchback){
            latchback=false;
            if(fromtocondition<=0){
              fromtocondition=0;
              mode = 1;
              break;
            }else{
              fromtocondition--;
            }
          }
          if(latchenter){
            latchenter=false;
            if(fromtocondition>=0&&fromtocondition<2){
              fromtocondition++;
            }
          }
        }

      }else if(latchback){
        latchback = false;
        mode = 0;
        break;
      }else{
        printlcd(0,0,">Mobile Robot    ");
        printlcd(0,1," Eksbot          ");
      }
      break;
    case 2:
      if(latchenter){
        latchenter=false;
        delay(100);
        while(true){
          up = digitalRead(pbatas);
          down = digitalRead(pbbawah);

          if(up==1 && upLastState==0){
            buttonup3();
          }else if(down==1 && downLastState==0){
            buttondown3();
          }
          upLastState = up;
          downLastState = down;

          switch(mode1){
            case 1:
              if(latchenter){
                latchenter = false;
                delay(100);
                while(true){
                  printlcd(0,0," TES SENSOR...   ");
                  printlcd(0,1,"                 ");
                  tes_sensor();
                  printlcd(0,1,String(val[0]));
                  printlcd(3,1,String(val[1]));
                  printlcd(6,1,String(val[2]));
                  printlcd(9,1,String(val[3]));
                  printlcd(12,1,String(val[4]));

                  if(latchback){
                    latchback=false;
                    mode1=1;
                    break;
                  }
                }
              }else{
                printlcd(0,0,">Tes Sensor      ");
                printlcd(0,1," Tes Servo       ");
              }
              break;
            case 2:
              if(latchenter){
                latchenter = false;
                delay(100);
                while(true){
                  printlcd(0,0," TES SERVO...    ");
                  printlcd(0,1,"                 ");
                  tesServo();
                  if(latchback){
                    latchback=false;
                    mode1=2;
                    break;
                  }
                }
              }else{
                printlcd(0,0," Tes Sensor      ");
                printlcd(0,1,">Tes Servo       ");
              }
              break;
            case 3:
              if(latchenter){
                latchenter=false;
                delay(100);
                while(true){
                  printlcd(0,0," TES MOTOR...    ");
                  printlcd(0,1,"                 ");

                  rpmL();
                  rpmR();
                  Backward1(100);
                  Serial.print(vl);
                  Serial.print(" ");
                  Serial.println(vr);

                  moveTo(50,"MUNDUR");

                  if(latchback){
                    latchback=false;
                    mode1=3;
                    break;
                  }
                }
              }else{
                printlcd(0,0,">Tes Motor       ");
                printlcd(0,1," Running         ");
              }
              break;
            case 4:
              if(latchenter){
                latchenter=false;
                printlcd(0,0,"EKSPERIMEN ROBOT ");
                printlcd(0,1,"                 ");
                delay(3000);
                while(true){
                  printlcd(0,0,"EKSPERIMEN ROBOT ");
                  printlcd(0,1,"                 ");

                  running();

                  if(latchback){
                    latchback=false;
                    mode1=4;
                    break;
                  }
                }
              }else{
                printlcd(0,0," Tes Motor       ");
                printlcd(0,1,">Running         ");
              }
              break;
          }
          if(latchback){
            latchback=false;
            mode=2;
            break;
          }
        }
      }else if(latchback){
        mode = 0;
        break;
      }else{
        printlcd(0,0," Mobile Robot    ");
        printlcd(0,1,">Eksbot          ");
      }
      break;
  }
  
}

void printlcd(int x, int y, String z){
  lcd.setCursor(x,y);
  lcd.print(z);
}

void tesServo(){
  for(sudut = 130; sudut > 0; sudut--) // perulangan untuk posisi 0 sampai 180 derajat
  { // step setiap 1 derajat
   servo.write(sudut); // memerintahkan servo ke posisi derajat sesuai nilai variabel sudut
   printlcd(0,1,String(sudut));
   delay(10); // menunggu 15 milidetik
  }
  delay(1000);
  for(sudut = 0; sudut<130; sudut++) // perulangan untuk posisi 180 sampai 0 derajat
  {
   servo.write(sudut); // memerintahkan servo ke posisi derajat sesuai nilai variabel pos
   printlcd(0,1,String(sudut));
   delay(10); // menunggu 15 milidetik
  }
  delay(2000);
}

void tes_sensor(){
  sensorValue = 0b00000;
  byte y=0;
  for(int i=0;i<5;i++){
    val[i]=analogRead(sensor[i]);
    Serial.print(val[i]); Serial.print(" ");
    if(i==4){
      val[i] = (val[i]>40)?1:0;
    }else if(i==2){
      val[i] = (val[i]>90)?1:0;
    }else{
      val[i] = (val[i]>90)?1:0;
    }
    sensorValue |= val[i]<<i;
    Serial.print(val[i]); Serial.print(" | ");
  }
  Serial.println();
}

void running(){
  tes_sensor();
  // MAJU PERTAMA MELEWATI LIMA GARIS
  int iii=0;
  int lm=0;
  int cm=0;
  int dm=0;
  while(a1<5){
    tes_sensor();
    if(iii>=0 && iii<=50){
      Forward1(3*iii);
    }else{
      Forward1(150);
    }
    if(a1>0){
      cm = millis();
      dm = cm-lm;
      if((val[2]==1 && valLast[2]==0)||(dm>1600)){
        a1++;
        printlcd(0,1,String(a1));
        lm = millis();
      }
    }else{
      if((val[2]==1 && valLast[2]==0)){
        a1++;
        printlcd(0,1,String(a1));
        lm = millis();
      }
    }
    valLast[2]=val[2];
    valLast[1]=val[1];
    iii++;
  }
  iii=0;
  a1=0;

  // BELOK KIRI
  moveTo(13,"MAJU");
  turnLeft(15,true);

  // MAJU MENGAMBIL OBJEK
  int a=0;
  valLast[4]=0;
  while(a<=a2){
    tes_sensor();
    lineFollow(1);
    if(val[4]==0 && valLast[4]==1){
      a++;
    }
    valLast[4]=val[4];
  }
  valLast[4]=0;

  // BELOK KANAN
  moveTo(13,"MAJU");
  turnRight(18);
  delay(100);

  // MAJU MENGAMBIL OBJEK
  moveTo(25,"MAJU");

  // MENGANGKAT OBJEK
  servo.write(0);
  delay(1000);

  // MUNDUR MEMBAWA OBJEK
  moveTo(40,"MUNDUR");
  Stop(100);

  // PUTAR BALIK
  delay(1000);
  moveTo(10,"MAJU");
  turnLeft(40,false);
  delay(1000);

  // MAJU MELEWATI 5 Garis
  moveTo(160,"MAJU");

  // MELETAKKAN OBJEK
  servo.write(180);
  delay(500);

  moveTo(40,"MUNDUR");

  Stop(2000);

  // RESET dan INCREMENT VARIABEL
  for(int i=0; i<=4; i++){
    valLast[i]=0;
  }
  a2++;
}

void lineFollow(int cond){
  tes_sensor();
  if(cond==1){
    error = val[0]*(-2)+val[1]*(-1)+val[3]*(1)+val[4]*(2);
  }else if(cond==2){
    if(val[1]==1&&val[0]==1){
      return;
    }else{
      error = val[1]*(-1)+val[3]*(1);
    }
  }else if(cond==3){
    if(val[3]==1&&val[4]==1){
      return;
    }else{
      error = val[1]*(-1)+val[3]*(1);
    }
  }else if(cond==4){
    if(val[0]==1&&val[1]==1){
      return;
    }else{
      error = val[1]*(-1)+val[3]*(1);
    }
  }
  Serial.print(error);
  Serial.print(" ");

  P = Kp*error;
  errorSum += error;
  I = Ki*errorSum;
  D = Kd*(error-lastError);

  output = P+I+D;

  sl = 150 + output;
  sr = 150 - output;

  if(sr>=255){
    sr=255;
  }else if(sr<=0){
    sr=0;
  }
  if(sl>=255){
    sl=255;
  }else if(sl<=0){
    sl=0;
  }

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, sl); 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, sr); 
}

void Stop(int d){
  //Serial.println("Stop");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4 , LOW);
  delay(d);
  
}

void Forward1(int speed){
  rpmL();
  rpmR();

  point = ((abs(vl)+abs(vr))/2);

  double errorleft = point-abs(vl);
  double errorright = point-abs(vr);

  float outputLeft = 3*errorleft;
  float outputRight = 3*errorright;
  
  lel = errorleft;
  ler = errorright;

  int ll = speed+outputLeft;
  int rr = speed+outputRight;

  if(rr>=255){
    rr=255;
  }else if(rr<=0){
    rr=0;
  }
  if(ll>=255){
    ll=255;
  }else if(ll<=0){
    ll=0;
  }

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, ll); 
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, rr); 
}

void Backward1(int speed){
  rpmL();
  rpmR();

  point = (((vl)+(vr))/2);

  double errorleft = (point)-(vl);
  double errorright = (point)-(vr);

  float outputLeft = 3*errorleft;
  float outputRight = 3*errorright;
  
  lel = errorleft;
  ler = errorright;

  int ll = speed+outputLeft;
  int rr = speed+outputRight;

  if(rr>=255){
    rr=255;
  }else if(rr<=0){
    rr=0;
  }
  if(ll>=255){
    ll=255;
  }else if(ll<=0){
    ll=0;
  }

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(ENA, ll); // Mengatur kecepatan motor A (0-255)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(ENB, rr); // Mengatur kecepatan motor B (0-255)
}

void moveTo(float inCm, String cond){
  countPulsesL = 0;
  countPulsesR = 0;
  float putaran = inCm/keliling; 
  float targetpulse = putaran*N*p;
  if(cond=="MUNDUR"){
    targetpulse=targetpulse*(-1);
  }

  while(true){
    if((countPulsesL<=targetpulse)&&(countPulsesR<=targetpulse)){
      if(cond=="MAJU"){
        printlcd(0,1,"MAJU");
        analogWrite(ENA, 132); 
        analogWrite(ENB, 130);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      }else if(cond=="MUNDUR"){
        analogWrite(ENA, 132); 
        analogWrite(ENB, 130);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN1, LOW);
        digitalWrite(IN4, HIGH);
        digitalWrite(IN3, LOW);
      }
    }else if((countPulsesL<=targetpulse)&&(countPulsesR>=targetpulse)){
      if(cond=="MAJU"){
        analogWrite(ENA, 150); // Kiri
        analogWrite(ENB, 0);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      }else if(cond=="MUNDUR"){
        analogWrite(ENA, 150); // Kiri
        analogWrite(ENB, 0);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN1, LOW);
        digitalWrite(IN4, HIGH);
        digitalWrite(IN3, LOW);
      }
    }else if((countPulsesL>=targetpulse)&&(countPulsesR<=targetpulse)){
      if(cond=="MAJU"){
        analogWrite(ENA, 0); // Kiri
        analogWrite(ENB, 150);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      }else if(cond=="MUNDUR"){
        analogWrite(ENA, 0); // Kiri
        analogWrite(ENB, 150);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN1, LOW);
        digitalWrite(IN4, HIGH);
        digitalWrite(IN3, LOW);
      }
    }else{
      if(cond=="MUNDUR"){
        analogWrite(ENA, 100); // Kiri
        analogWrite(ENB, 100);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      }else if(cond=="MAJU"){
        analogWrite(ENA, 100); // Kiri
        analogWrite(ENB, 100);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN1, LOW);
        digitalWrite(IN4, HIGH);
        digitalWrite(IN3, LOW);
      }
      delay(100);
      analogWrite(ENA, 0); // Kiri
      analogWrite(ENB, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      countPulsesL = 0;
      countPulsesR = 0;
      delay(100);
      break;
    }
  }
}

void turnLeft(int inCm, bool k){
  countPulsesL = 0;
  countPulsesR = 0;
  float putaran = inCm/keliling; 
  float targetpulse = putaran*N*p;
  while(true){
    if((countPulsesR<=targetpulse)){
      analogWrite(ENA, 150);
      analogWrite(ENB, 150);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }else{
      analogWrite(ENA, 0); // Kiri
      analogWrite(ENB, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      countPulsesL = 0;
      countPulsesR = 0;
      delay(500);
      break;
    }
    if(k){
      if(val[1]==1||val[2]==1||val[3]==1){
        break;
      }
    }
  }
}

void turnRight(int inCm){
  countPulsesL = 0;
  countPulsesR = 0;
  float putaran = inCm/keliling; 
  float targetpulse = putaran*N*p;
  while(true){
    if((countPulsesL<=targetpulse)){
      analogWrite(ENA, 150);
      analogWrite(ENB, 150);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }else{
      analogWrite(ENA, 0); // Kiri
      analogWrite(ENB, 0);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      countPulsesL = 0;
      countPulsesR = 0;
      Serial.println("Berhenti.........");
      delay(500);
      break;
    }
  }
}

void rpmL(){
  unsigned long currentTime = millis();
  float deltaTime = currentTime - lTimeL;
  if(deltaTime>=100){
    vl = countPulsesL;
    countPulsesL = 0;
    lTimeL = currentTime;
  }
}

void rpmR(){
  unsigned long currentTime = millis();
  float deltaTime = currentTime - lTimeR;
  if(deltaTime>=100){
    vr = countPulsesR;
    countPulsesR = 0;
    lTimeR = millis();
  }
}