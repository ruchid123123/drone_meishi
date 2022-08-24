#define PWM1 12
#define PWM2 13
#define PWM3 14
#define PWM4 15
#define SBUS 16
#define LED1 18
#define LED2 19
#define MPUINT 23


void stopall(){
  for(int i=0; i<4; i++){
    ledcWrite(i, 0); //0ch 50/255%
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(PWM4,OUTPUT);
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);

  ledcSetup(0, 500, 8); //ch0 5khz 8bit
  ledcAttachPin(PWM1, 0); //0ch=PWM1

  ledcSetup(1, 500, 8); //ch0 5khz 8bit
  ledcAttachPin(PWM2, 1); //0ch=PWM1

  ledcSetup(2, 500, 8); //ch0 5khz 8bit
  ledcAttachPin(PWM3, 2); //0ch=PWM1

  ledcSetup(3, 500, 8); //ch0 5khz 8bit
  ledcAttachPin(PWM4, 3); //0ch=PWM1

  stopall();
}

void loop() {
  // put your main code here, to run repeatedly:
  ledcWrite(0, 10); //0ch 50/255%
  delay(3000);
  stopall();
  delay(1000);
  
  ledcWrite(1, 10); //0ch 50/255%
  delay(3000);
  stopall();
  delay(1000);
  
  ledcWrite(2, 10); //0ch 50/255%
  delay(3000);
  stopall();
  delay(1000);
  
  ledcWrite(3, 10); //0ch 50/255%
  delay(3000);
  stopall();
  delay(1000);
}
