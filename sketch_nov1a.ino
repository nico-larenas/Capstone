
int speed = 0;
unsigned long time_ant = 0;
unsigned long newtime;
const int Period = 10000; 
char msgEnd = '\n';
bool newMsg = false;
String instruccion;

void setup() {
  // put your setup code here, to run once:
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(9, OUTPUT);
  Serial.begin(9600); 
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);
}

void loop() {
  if ((micros() - time_ant) >= Period) {
    newtime = micros();

    if (Serial.available() > 0) {
      instruccion = readBuff(); // Read the incoming message

      speed = instruccion.toInt();
      Serial.print(speed);

      if (speed > 0) {
        // Spin counterclockwise
        digitalWrite(2, HIGH);
        digitalWrite(3, LOW);
        analogWrite(9, abs(speed)); // Use abs(speed) to make sure it's positive
      } else if (speed < 0) {
        // Spin clockwise
        digitalWrite(2, LOW);
        digitalWrite(3, HIGH);
        analogWrite(9, abs(speed)); // Use abs(speed) to make sure it's positive
      } else {
        // Stop the motor
        digitalWrite(9, LOW);
        digitalWrite(8, LOW);
        analogWrite(9, 0);
      }
    }
    
    time_ant = newtime; 
  }
}



String readBuff() {
  String buffArray;
  //int i = 0;

  while (Serial.available() > 0) { //Entro a este while mientras exista algo en el puerto serial
    char buff = Serial.read(); //Leo el byte entrante
    if (buff == msgEnd) {
      newMsg = true;
      break; //Si el byte entrante coincide con mi delimitador, me salgo del while
    } else {
      buffArray += buff; //Si no, agrego el byte a mi string para construir el mensaje
      //i += 1;
    }
    delay(10);
  }

  return buffArray;  //Retorno el mensaje
}

