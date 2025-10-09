#include <Wire.h>
#include <Adafruit_INA260.h>

// === Pines Motores ===
// MOTOR IZQUIERDO
const int AIN1 = 8;
const int BIN1 = 7;
const int PWMA = 5;   // PWM motor izquierdo

// MOTOR DERECHO
const int AIN2 = 9;
const int BIN2 = 4;
const int PWMB = 6;   // PWM motor derecho

// === INA260 ===
Adafruit_INA260 inaLeft = Adafruit_INA260();   // Dirección 0x40
Adafruit_INA260 inaRight = Adafruit_INA260();  // Dirección 0x44

unsigned volatile int  DatoL = 0, DatoH = 0;

// --- Setup ---
void setup() {
  Serial.begin(115200);  // Comunicación con Jetson
  Serial1.begin(19200); //Puerto serial 1, SI necesita estar a 19200, a ese baudrate se comunican los encoders

  while (!Serial) { ; }

  // Pines motores
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(AIN2, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Estado inicial motores apagados
  setMotor(PWMA, AIN1, BIN1, 0);
  setMotor(PWMB, AIN2, BIN2, 0);

  // Inicializar INA260
  if (!inaLeft.begin(0x40)) {
    Serial.println("ERROR: INA260 (0x40) no encontrado");
  } else {
    Serial.println("INA260 (0x40) OK");
  }

  if (!inaRight.begin(0x44)) {
    Serial.println("ERROR: INA260 (0x44) no encontrado");
  } else {
    Serial.println("INA260 (0x44) OK");
  }

  Serial.println("ARDUINO_READY");
}

// --- Loop principal ---
void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    int Lspeed = 0, Rspeed = 0;

    // Comando tipo: L100 R100
    if (sscanf(input.c_str(), "L%d R%d", &Lspeed, &Rspeed) == 2) {
      setMotor(PWMA, AIN1, BIN1, Lspeed);  // Motor izquierdo
      setMotor(PWMB, AIN2, BIN2, Rspeed);  // Motor derecho

      Serial.print("ACK: L"); Serial.print(Lspeed);
      Serial.print(" R"); Serial.println(Rspeed);

      unsigned int DatoDer = 0, DatoIzq = 0; //Variables para guardar los datos de cada encoder por separado
      DatoDer = LeerEncoderDer(0x08,0x02); //Se le pregunta al encoder 2 la cantidad de "clics" (Motor derecho viendolo por donde estan los swtches)

      
      DatoIzq = LeerEncoderIzq(0x08,0x01); //Se pregunta por la cantidad de "clics" del motor 1 (Motor izquierdo viendolo por donde estan los swtches)
        Serial.print("Encoder derecho:");
  Serial.print(DatoDer);
  Serial.print("  ");  // <--- estos espacios permiten el \s+ de tu regex
  Serial.print("Encoder izquierdo:");
  Serial.println(DatoIzq);

    }

    // Comando: GET_CURR -> corrientes, voltajes y potencia
    if (input == "GET_CURR") {
      float currL = inaLeft.readCurrent();
      float voltL = inaLeft.readBusVoltage();
      float powerL = inaLeft.readPower();

      float currR = inaRight.readCurrent();
      float voltR = inaRight.readBusVoltage();
      float powerR = inaRight.readPower();

      Serial.print("INA_L (0x40) -> I(mA): "); Serial.println(currL);


      Serial.print("INA_R (0x44) -> I(mA): "); Serial.println(currR);


      // DatoH = 0; 
      // DatoL = 0; 

    }
  }
}

// === Función control de motor ===
void setMotor(int pwmPin, int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}


unsigned int LeerEncoderDer(int Comando, int Direccion) //Funcion para leer el encoder 2 o derecho, viendo el carro por la parte de los switches
{
  UCSR1B &= ~(1 << RXEN1); //Apagar RX, esta linea de codigo es tomada del lenguaje para ATmega
  UCSR1B |= (1 << TXEN1); //Enceder Tx, esta linea de codigo es tomada del lenguaje para ATmega
  
  Serial1.write(0x30 + Direccion); //invertir sentido llanta de lado derecho, esto es para que cuente en el mismo sentido que la llanta izquierda, si no, cada clic resta en lugar de sumar
  Serial1.write(Comando + Direccion); //Tx dato, 5bits de instruccion y 3 de direccion, manda la instruccion mas la direccion
  
  UCSR1B &= ~(1 << TXEN1);//Apagar Tx, esta linea de codigo es tomada del lenguaje para ATmega
  UCSR1B |= (1 << RXEN1); //Encender Rx, esta linea de codigo es tomada del lenguaje para ATmega


  delay(5);
  serialEvent1(); //funcion en donde se guardan los datos recibidos del encoder
  return (DatoH<<8) + DatoL; //Combina ponderadamente los dos datos de 8bits recibidos del encoder 
}

unsigned int LeerEncoderIzq(int Comando, int Direccion) //La diferencia con la funcion para leer encoder derecho es que en este no se invierte el sentido y se cambia la direccion
{
  UCSR1B &= ~(1 << RXEN1); //Apagar RX
  UCSR1B |= (1 << TXEN1); //Enceder Tx
  
  Serial1.write(0x05); //comando dummy, asegura una buena lectura de los datos de entrada
  Serial1.write(Comando + Direccion); //Tx dato, 5bits de instruccion y 3 de direccion
  
  UCSR1B &= ~(1 << TXEN1);//Apagar Tx
  //sei(); //Encender interrupciones
  UCSR1B |= (1 << RXEN1); //Encender Rx

  delay(5);
  serialEvent1();
  //Serial.println("Encoder 2");
  //Serial.println(DatoIzq);

  return (DatoH<<8) + DatoL;
}

void serialEvent1() //Funcion donde se leen los datos recibidos de manera serial y se guardan en DatoH y DatoL
{
  while (Serial1.available()) 
  {
    DatoH = Serial1.read();
    DatoL = Serial1.read();
  }
}


