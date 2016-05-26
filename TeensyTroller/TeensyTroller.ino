

long int count;
int buttonPin = 2;
void setup() {
  pinMode(buttonPin,INPUT_PULLUP);

  Serial1.begin(9600);
  Serial3.begin(9600);
  Serial3.write(254);
  Serial3.write(1);
  Serial3.print("Hello Sarah");
  delay(1000);


}

void loop() {
  if (!digitalRead(buttonPin)){count=0;  }

  Serial3.write(0xFE);
  Serial3.write(0x01);
  Serial3.print(count);
  count+=1;
  delay(100);

}



