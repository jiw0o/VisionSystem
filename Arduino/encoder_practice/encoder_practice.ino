int IN1Pin = 32; //모터 방향, (0:backward, 1:forward)
int IN2Pin = 33; //Break, (0:rotate, 1:break)
int ENPin = 11; //회전속도 0~255
int HA = 2; //홀센서A
int HB = 3; //홀센서B
int cnt = 0;
char in_data;   

void cmd_release();
void cmd_forward();
void cmd_stop();
void count();

void setup() {
  pinMode(IN1Pin, OUTPUT);
  pinMode(IN2Pin, OUTPUT);
  pinMode(HA, INPUT_PULLUP);
  pinMode(HB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HA), count, RISING);
  analogWrite(ENPin, 30); //속도값 30으로 제어
  Serial.begin(9600);
  Serial.println("1:release, 2:forward, 3:stop");
  cmd_stop();
}


void cmd_release() {
  digitalWrite(IN2Pin, 0);
  digitalWrite(IN1Pin, 0);
}
void cmd_forward() {
  cnt = 0;
  while (cnt < 115) {
    digitalWrite(IN2Pin, 0);
    digitalWrite(IN1Pin, 1);
    Serial.println(cnt);
  }
  cmd_stop();
  cnt = 0;
}
void cmd_stop() { digitalWrite(IN2Pin, 1); }

void count() { cnt++; }

void loop() {
  if(Serial.available())  //시리얼모니터에서 데이터가 들어오면
  {
    in_data = Serial.read();  //입력된 데이터 in_data에 저장
    Serial.print("data : ");
    Serial.println(in_data);
  }

  if(in_data == '0') cmd_stop();
  else if(in_data == '1') cmd_forward();
  else if(in_data == '2') cmd_release();
}
