//CODE FOR ARDUINO MEGA

int w1_ticks = 0;
int w2_ticks = 0;

int w1_hall_A = 2; // Define the pin numbers for w1 hall sensors
int w1_hall_B = 3;
int w1_hall_C = 18;

int w2_hall_A = 19; // Define the pin numbers for w2 hall sensors
int w2_hall_B = 20;
int w2_hall_C = 21;

String prev_w1_state = "000"; 
String prev_w2_state = "000";

void setup() {
  pinMode(w1_hall_A, INPUT);
  pinMode(w1_hall_B, INPUT);
  pinMode(w1_hall_C, INPUT);

  pinMode(w2_hall_A, INPUT);
  pinMode(w2_hall_B, INPUT);
  pinMode(w2_hall_C, INPUT);

  Serial.begin(115200);
}

void loop() {
  String w1_hall_state = String(digitalRead(w1_hall_A)) + String(digitalRead(w1_hall_B)) + String(digitalRead(w1_hall_C));
  String w2_hall_state = String(digitalRead(w2_hall_A)) + String(digitalRead(w2_hall_B)) + String(digitalRead(w2_hall_C));

  if (prev_w1_state != w1_hall_state) {
    w1_ticks += 1;
  }

  if (prev_w2_state != w2_hall_state) {
    w2_ticks += 1;
  }

  Serial.print("W1 ticks:");
  Serial.print(w1_ticks);
  Serial.print("|");
  Serial.print("W2 ticks:");
  Serial.println(w2_ticks);

  prev_w1_state = w1_hall_state;
  prev_w2_state = w2_hall_state;

  // delay(10);
}