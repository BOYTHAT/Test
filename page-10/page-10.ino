void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {

  if (Serial.available()) {
    String data_from_display="";
    delay(30);
    while(Serial.available()){
      data_from_display += char(Serial.read());
    }
    senData(data_from_display);
  }
}

void senData(String data_from_display){
  if(data_from_display == "A"){
    digitalWrite(13,HIGH);
  }
  if(data_from_display == "B"){
    digitalWrite(13 , LOW);
  }
}


