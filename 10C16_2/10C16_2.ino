/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/faq/how-to-control-speed-of-servo-motor
 */

#include <Servo.h>
#include <math.h>

#define PIN_SERVO 10
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 0     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)
#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define _EMA_ALPHA 0.2    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.
#define MEDIAN_SIZE 30
#define START_ANGLE 0
#define STOP_ANGLE 90
#define DURATION_MS 2000
#define SIGMOID_K 3


Servo myServo;
unsigned long MOVING_TIME = 3000; // moving time is 3 seconds
unsigned long moveStartTime;
int startAngle = 30; // 30°
int stopAngle  = 90; // 90°
int angle = 0;

unsigned long last_sampling_time;   // unit: msec
float dist_prev = _DIST_MAX;        // Distance last-measured
float dist_ema;                    // EMA distance
float alpha = 0.8;
int count =0;
float mid_prev[MEDIAN_SIZE];
bool car = true;
bool up = true;

const float TH_NEAR = 180.0;  // 이 이하 → 올림
const float TH_FAR  = 240.0;  // 이 이상 → 내림 (원하면 조정)


void setup() {
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  digitalWrite(PIN_TRIG, LOW);
  myServo.attach(PIN_SERVO);
  moveStartTime = millis(); // start moving

  myServo.write(START_ANGLE); 
  delay(500);
}

void loop() {
  float dist_raw, dist_filtered,mid;
  
  
  // wait until next sampling time. 
  // millis() returns the number of milliseconds since the program started. 
  // will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL) return;
  last_sampling_time = millis();

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

  // Modify the below if-else statement to implement the range filter
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
      dist_filtered = _DIST_MAX;
  } else if (dist_raw < _DIST_MIN) {
      dist_filtered = dist_prev;
  } else {    // In desired Range
      dist_filtered = dist_raw;
      dist_prev = dist_raw;
  }
  count++;
  
  if(count>=MEDIAN_SIZE){
  for(int i=1;i<MEDIAN_SIZE;i++){
    mid_prev[i-1] = mid_prev[i];
  }
  mid_prev[MEDIAN_SIZE-1] = dist_filtered;
  float median[MEDIAN_SIZE];
  for(int i=0;i<MEDIAN_SIZE;i++){
    median[i] = mid_prev[i];
  }
  insertionSort(median,MEDIAN_SIZE);
  mid = median[MEDIAN_SIZE/2];
}else{
  mid = dist_filtered;
}


if (!car && mid <= TH_NEAR) {      
  car = true;
  sigmoid(true); 
}
if (car && (mid >= TH_FAR)) {    
  car = false;
  sigmoid(false);  
}
}
void sigmoid(bool up) {
  // 시작/목표 각도 설정 (현재 위치에서 시작 → 점프 방지)
  int fromAngle = myServo.read();
  int toAngle   = up ? STOP_ANGLE : START_ANGLE;

  // 이미 목표면 바로 종료
  if (fromAngle == toAngle) {
    myServo.write(toAngle);
    return;
  }

  unsigned long t0  = millis();
  const unsigned long T = DURATION_MS;  // 총 이동 시간(ms)

  while (true) {
    unsigned long t = millis() - t0;
    if (t >= T) {
      myServo.write(toAngle);     // 최종 위치 고정
      break;
    }

     float s = (float)t / (float)T;    // 0..1
    float u = 1.0f - s;
    float p = 1.0f - u*u*u*u*u;

    int angle = (int)(fromAngle + (toAngle - fromAngle) * p + 0.5f);
    myServo.write(constrain(angle, 0, 180));

    delay(5);   // 업데이트 간격(더 부드럽게 보이면 3~10ms 사이로 조절)
  }
}


void insertionSort(float* arr, int n) {
  for (int i = 1; i < n; i++) {
    float key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm

  // Pulse duration to distance conversion example (target distance = 17.3m)
  // - pulseIn(ECHO, HIGH, timeout) returns microseconds (음파의 왕복 시간)
  // - 편도 거리 = (pulseIn() / 1,000,000) * SND_VEL / 2 (미터 단위)
  //   mm 단위로 하려면 * 1,000이 필요 ==>  SCALE = 0.001 * 0.5 * SND_VEL
  //
  // - 예, pusseIn()이 100,000 이면 (= 0.1초, 왕복 거리 34.6m)
  //        = 100,000 micro*sec * 0.001 milli/micro * 0.5 * 346 meter/sec
  //        = 100,000 * 0.001 * 0.5 * 346
  //        = 17,300 mm  ==> 17.3m
}