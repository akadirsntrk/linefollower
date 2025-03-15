#include <QTRSensors.h>

#define KP 0.008 // Kendi deneyimlerinizle bulmanız gerekli küçük bir değer ile başlayıp, büyütebilirsiniz en kararlı Kp değerini bulana kadar.. 0.4
#define KD 2.8 // Bunu da küçük bir değerden başlatın ve deneyimleyerek büyütün. ( Not: Kp < Kd) 2.0

#define rightMotor1 10
#define rightMotor2 9
#define rightMotorPWM 11
#define leftMotor1 6
#define leftMotor2 7
#define leftMotorPWM 5
#define maxHiz 255
#define LED 13


QTRSensors qtr;

int fark = 0; // Motorlara uygulanan fark.
int sonHata; // Orantılı son değer. (Hatanın türevini hesaplamak için kullanılır.)
int hedef = 3500; // Sensörden gelen 0 - 7000 arası değerin orta noktası.
int hata = 0, turev = 0;
unsigned int pozisyon = 3500;
const uint8_t SensorCount = 8;
unsigned int sensor[SensorCount];



void setup()
{
Serial.begin(9600);
// configure the sensors
qtr.setTypeAnalog();
qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A5, A7}, SensorCount);
pinMode(rightMotor1, OUTPUT);
pinMode(rightMotor2, OUTPUT);
pinMode(rightMotorPWM, OUTPUT);
pinMode(leftMotor1, OUTPUT);
pinMode(leftMotor2, OUTPUT);
pinMode(leftMotorPWM, OUTPUT);


pinMode(LED_BUILTIN, OUTPUT);
digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
int i;
kalibrasyon(0); // 0 elle, 1 otomatik kafa sallama.
digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

delay(1000); // Ana döngüye girmeden önce botu konumlandırmak için 1 saniye bekleyin
}
int lastError = 0;
void loop() {
sensorOku();
pd();
}
void sensorOku() {
pozisyon = qtr.readLineBlack(sensor); // Siyah çizginin pozisyonunu oku. (0 - 7000)
// while(pozisyon>6998 || pozisyon<10){
// motor(0,0);
// }
hata = pozisyon - hedef; // Pozisyondan 3500 (hedef) çıkar. Hatayı bul.
qtr.read(sensor); // Sekiz sensörün ham değelerini oku.
}
void pd() {
turev = hata - sonHata; // Hatadan bir önceki hatayı çıkar.
sonHata = hata; // Şimdiki hatayı kaydet.

fark = ( hata * KP) + ( turev * KD ); // Motorlara uygulanacak farkı hesapla.

constrain(fark, -maxHiz, maxHiz); // fark en fazla maxHiz olsun.
if ( fark < 0 ) // fark negatif ise
motor(maxHiz, maxHiz + fark); // Sağ motorun hızını düşür.
else // fark negatif değilse
motor(maxHiz - fark, maxHiz); // Sol motorun hızını düşür.
while(pozisyon>3200 && pozisyon<3800){
pozisyon = qtr.readLineBlack(sensor);
motor(maxHiz,maxHiz);
}
while(pozisyon==7000){
pozisyon = qtr.readLineBlack(sensor);
motor(-maxHiz,maxHiz);
}
while(pozisyon==0){
pozisyon = qtr.readLineBlack(sensor);
motor(maxHiz,-maxHiz);
}
}
void motor(int solMotorPWM, int sagMotorPWM) {

if ( solMotorPWM >= 0 ) { // İleri.
digitalWrite(leftMotor1, HIGH);
digitalWrite(leftMotor2, LOW);
}
else { // Negatifse geri döndür.
digitalWrite(leftMotor1, LOW);
digitalWrite(leftMotor2, HIGH);
solMotorPWM *= -1;
}
analogWrite(leftMotorPWM, solMotorPWM);

if ( sagMotorPWM >= 0 ) { // İleri.
digitalWrite(rightMotor1, HIGH);
digitalWrite(rightMotor2, LOW);
}
else { // Negatifse geri döndür.
digitalWrite(rightMotor1, LOW);
digitalWrite(rightMotor2, HIGH);
sagMotorPWM *= -1;
}
analogWrite(rightMotorPWM, sagMotorPWM);
}
void kalibrasyon(bool secim) { // 1 otomatik, 0 elle.
if (secim) { // secim 1 ise otomatik kalibrasyon yap.
byte hiz =200; // Aracın kafasını sallama hızı.
for (byte i = 0; i < 6; i++) { // Sağa sola üç kez kafa salla.
while (sensor[7] < 300) {
motor(hiz, -hiz);
qtr.calibrate();
sensorOku();
}
while (sensor[7] > 700) {
motor(hiz, -hiz);
qtr.calibrate();
sensorOku();
}
while (sensor[0] < 300) {
motor(-hiz, hiz);
qtr.calibrate();
sensorOku();
}
while (sensor[0] > 700) {
motor(-hiz, hiz);
qtr.calibrate();
sensorOku();
}
while (sensor[3] > 500) { // Ortada dur.
motor(hiz, -hiz);
qtr.calibrate();
sensorOku();
}
}
} else { // secim 0 ise elle kalibrasyon yap.
for ( byte i = 0; i < 100; i++) { // Dahili LED yanıp söndüğü sürece (3 sn) elle kalibrasyon yap.
digitalWrite(LED, HIGH); delay(20);
qtr.calibrate();
digitalWrite(LED, LOW); delay(20);
qtr.calibrate();
}
}
motor(0, 0);
delay(2000); // Kalbirasyondan sonra 3 sn bekle.
}
