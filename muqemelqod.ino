#include <QTRSensors.h>

#define KP 0.2  // Proportional (Oransal) Kazanç
#define KI 0.001  // Integral Kazanç (Yeni eklendi)
#define KD 2.7   // Derivative (Türev) Kazanç

#define rightMotor1 10
#define rightMotor2 9
#define rightMotorPWM 11
#define leftMotor1 6
#define leftMotor2 7
#define leftMotorPWM 5
int  maxHiz =75;
#define LED 13

QTRSensors qtr;

int fark = 0;      // Motorlara uygulanan fark.
int sonHata;       // Orantılı son değer. (Hatanın türevini hesaplamak için kullanılır.)
int hedef = 3500;  // Sensörden gelen 0 - 7000 arası değerin orta noktası.
int sayac = 0;
int hata = 0, turev = 0;
unsigned int pozisyon = 3500;
const uint8_t SensorCount = 8;
unsigned int sensor[SensorCount];

long integral = 0;  // Integral hesaplamaları için değişken

void setup() {
  Serial.begin(9600);
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ A0, A1, A2, A3, A4, A5, A5, A7 }, SensorCount);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  int i;
  kalibrasyon(0);  // 0 elle, 1 otomatik kafa sallama.
  delay(500);      // Ana döngüye girmeden önce botu konumlandırmak için 1 saniye bekleyin
}

bool degerim = 1;
int lastError = 0;

void loop() {
  qtr.read(sensor);
  if(sayac==11&&sensor[0]>300 && sensor[1]>300 && sensor[2]>300 && sensor[3]>300 && sensor[4]>300 && sensor[5]>300 && sensor[6]>300 && sensor[7]>300 ){
    motor(0,0);
    delay(200);
    motor(-maxHiz,maxHiz);
    delay(200);
    motor(maxHiz,maxHiz);
    delay(200);
    motor(-maxHiz,-maxHiz);
    delay(200);
    motor(maxHiz,-maxHiz);
    delay(200);
    motor(maxHiz,maxHiz);
    delay(1500);
    motor(0,0);
  }
  if(sayac==10&&sensor[0]>300 && sensor[1]>300 && sensor[2]>300 && sensor[3]>300 && sensor[4]>300 && sensor[5]>300 && sensor[6]>300 && sensor[7]>300 ){
    sayac += 1;
  }

  if(sayac==0&&sensor[0]>300 && sensor[1]>300 && sensor[2]>300 && sensor[3]>300 && sensor[4]>300 && sensor[5]>300 && sensor[6]>300 && sensor[7]>300 ){
    digitalWrite(LED,1);
    delay(100);
    digitalWrite(LED,0);
    delay(100);
    sayac += 1;
    motor(maxHiz,maxHiz);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    delay(400);
    sayac += 1;
  }
  if(sayac==2){maxHiz=80;}
  if (sayac==2 && sensor[1] > 300 && (sensor[3] < 300 || sensor[4] < 300) && sensor[6] > 30){
    motor(maxHiz,-maxHiz);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    delay(180);
    sayac+=1;
    maxHiz=100;
  }
  if (sensor[0] < 300 && (sensor[3] > 300 || sensor[4] > 300) && sensor[7] < 300) {
    degerim = 1;
  }
  
  if (sayac==3 && sensor[0] > 300 && (sensor[3] < 300 || sensor[4] < 300) && sensor[7] > 300) {
    motor(70,70);
    delay(50);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    degerim = 0;
    sayac += 1 ;
    maxHiz=90;
  }
  if(sayac == 9 && sensor[0]>300 && sensor[1]>300 && sensor[2]>300 && sensor[3]>300 && sensor[4]>300 && sensor[5]>300 && sensor[6]>300 && sensor[7]>300 ){
    sayac+= 1;
    motor(maxHiz-15,maxHiz);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    delay(400);
  }
  if(sayac == 8 && sensor[0]>300 && sensor[1]>300 && sensor[2]>300 && sensor[3]>300 && sensor[4]>300 && sensor[5]>300 && sensor[6]>300 && sensor[7]>300 ){
    motor(0,0);
    delay(1000);
    motor(-maxHiz,maxHiz);
    delay(865);
    sayac+= 1;
  }
  if(sayac==6  && sensor[0]>300 && sensor[1]>300 && sensor[2]>300 && sensor[3]>300 && sensor[4]>300 && sensor[5]>300 && sensor[6]>300 && sensor[7]>300 ){
    sayac=7;
  }
  if(sayac==7  && sensor[0]>300 && sensor[1]>300 && sensor[2]>300 && sensor[3]>300 && sensor[4]>300 && sensor[5]>300 && sensor[6]>300 && sensor[7]>300 ){
    motor(0,0);
    delay(1000);
    motor(maxHiz,-maxHiz);
    delay(840);
    sayac+= 1;
  }
  if(sayac==4 && sensor[0]>300 && sensor[1]>300 && sensor[2]>300 && sensor[3]>300 && sensor[4]>300 && sensor[5]>300 && sensor[6]>300 && sensor[7]>300 ){
    sayac=6;
    motor(maxHiz,maxHiz);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    digitalWrite(LED,HIGH);
    delay(20);
    digitalWrite(LED,LOW);
    delay(20);
    delay(400);
  }
  
  sensorOku(degerim);
  // Serial.println(String(sensor[0]) + " " + String(sensor[1]) + " " + String(sensor[2]) + " " + String(sensor[3]) + " " + 
  //                String(sensor[4]) + " " + String(sensor[5]) + " " + String(sensor[6]) + " " + String(sensor[7]) + " ");
}

void sensorOku(int deger) {
  if (deger == 1) {
    pozisyon = qtr.readLineBlack(sensor);  // Siyah çizginin pozisyonunu oku. (0 - 7000)
  }
  
  if (deger == 0) {
    pozisyon = qtr.readLineWhite(sensor);  // Beyaz çizginin pozisyonunu oku. (0 - 7000)
  }

  hata = pozisyon - hedef;  // Pozisyondan 3500 (hedef) çıkar. Hatayı bul.
  qtr.read(sensor);         // Sekiz sensörün ham değerlerini oku.
  turev = hata - sonHata;   // Hatadan bir önceki hatayı çıkar.
  sonHata = hata;           // Şimdiki hatayı kaydet.
  integral += hata;         // Integral hesaplama (Hata birikimi)

  // PID çıkışını hesapla
  fark = (hata * KP) + (turev * KD) + (integral * KI);  

  constrain(fark, -maxHiz, maxHiz);  // fark en fazla maxHiz olsun.

  if (fark < 0)  // fark negatif ise
    motor(maxHiz, maxHiz + fark);    // Sağ motorun hızını düşür.
  else           // fark negatif değilse
    motor(maxHiz - fark, maxHiz);    // Sol motorun hızını düşür.

  if (pozisyon > 3400 && pozisyon < 3600) {
    if (deger == 1) {
      pozisyon = qtr.readLineBlack(sensor);  // Siyah çizginin pozisyonunu oku. (0 - 7000)
    }
    if (deger == 0) {
      pozisyon = qtr.readLineWhite(sensor);  // Siyah çizginin pozisyonunu oku. (0 - 7000)
    }
    motor(maxHiz, maxHiz);
  }

  if (pozisyon == 7000) {
    if (deger == 1) {
      pozisyon = qtr.readLineBlack(sensor);  // Siyah çizginin pozisyonunu oku. (0 - 7000)
    }
    if (deger == 0) {
      pozisyon = qtr.readLineWhite(sensor);  // Siyah çizginin pozisyonunu oku. (0 - 7000)
    }
    motor(-75, 75);
  }

  if (pozisyon == 0) {
    if (deger == 1) {
      pozisyon = qtr.readLineBlack(sensor);  // Siyah çizginin pozisyonunu oku. (0 - 7000)
    }
    if (deger == 0) {
      pozisyon = qtr.readLineWhite(sensor);  // Siyah çizginin pozisyonunu oku. (0 - 7000)
    }
    motor(75, -75);
  }
}

void motor(int solMotorPWM, int sagMotorPWM) {
  if (solMotorPWM >= 0) {  // İleri.
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
  } else {  // Negatifse geri döndür.
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    solMotorPWM *= -1;
  }
  analogWrite(leftMotorPWM, solMotorPWM);

  if (sagMotorPWM >= 0) {  // İleri.
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
  } else {  // Negatifse geri döndür.
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    sagMotorPWM *= -1;
  }
  analogWrite(rightMotorPWM, sagMotorPWM);
}

void kalibrasyon(bool secim) {      // 1 otomatik, 0 elle.
  if (secim) {                      // secim 1 ise otomatik kalibrasyon yap.
    int hiz = 100;                  // Aracın kafasını sallama hızı.
    for (byte i = 0; i < 20; i++) {  // Sağa sola üç kez kafa salla.
      motor(hiz, -hiz);
      delay(300);
      qtr.calibrate();
      motor(-hiz, hiz);
      delay(300);
      qtr.calibrate();
    }
  } else {  // secim 0 ise elle kalibrasyon yap.
    for (byte i = 0; i < 100; i++) {
      delay(20);
      qtr.calibrate();
      delay(20);
      qtr.calibrate();
    }
  }
  motor(0, 0);
  delay(200);  // Kalibrasyondan sonra 3 sn bekle.
}
