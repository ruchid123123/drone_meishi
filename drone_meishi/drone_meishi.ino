#define PWM1 12
#define PWM2 13
#define PWM3 14
#define PWM4 15
#define SBUS 16
#define LED1 18
#define LED2 19
#define MPUINT 23


void setup() {
  // put your setup code here, to run once:
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(PWM4,OUTPUT);
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED1,HIGH);
  digitalWrite(LED2,HIGH);
  delay(500);
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  delay(500);
}
