
//*****************************************************************************
// Universidad del Valle de Guatemala
// BE3015 - Electrónica Digital 2
// Fernando Delgado
// Proyecto 2 
//*****************************************************************************

//*****************************************************************************
// Librerías
//-----------------------------------------------------------------------------
#include <SPI.h>
#include <SD.h>




//Pines de comunicación USART


#define sensorPin PB_5

#define leda PF_2
#define ledv PF_3
#define ledr PF_1

#define btn1 PF_0
#define btn2 PF_4






//----------------------------------------------------------------------------------------------------------------------
//Prototipos de funciones
//----------------------------------------------------------------------------------------------------------------------
void writeSD(void);


//---------------------------------------------------------------------------------------------------------------------
//void
//---------------------------------------------------------------------------------------------------------------------
int sensorValue = 0;  // variable to store the value coming from the sensor
int voltaje2 = 0;
int voltaje1 = 0;
String dutycycleled1 = "";

//se refiere a la señal PWM a la que trabajaran los leds desde un inicio.
int dutycycleled2 = 0;
int dutycycleled3 = 0;
int contador1 = 0;
int subirDato = 0;
int encendido = 255;
int apagado = 0;
File myFile;

//---------------------------------------------------------------------------------------------------------------------
//setup
//---------------------------------------------------------------------------------------------------------------------

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin (115200);
  Serial3.begin(115200);

  pinMode (leda, OUTPUT);
  pinMode (ledv, OUTPUT);
  pinMode (ledr, OUTPUT);

  pinMode (btn1, INPUT_PULLUP);
  pinMode (btn2, INPUT_PULLUP);

  pinMode(PA_3, OUTPUT);
  SPI.setModule(0);

  if (!SD.begin(PA_3)) {
    Serial.println("Fallo tu logica");
    return;
  }

}

//---------------------------------------------------------------------------------------------------------------------
//loop
//---------------------------------------------------------------------------------------------------------------------
void loop() {

  if (digitalRead(btn2) == LOW)
  {
    if (contador1 >= 0)
    {
      contador1 = contador1 + 1; //subira de uno en uno esto teniendo un mejor control
      Serial.println(contador1);
    }
    if (contador1 >= 2) //esto es para recetear el contador para numero mayores de 255
    {
      contador1 = 0;
      Serial.println(contador1);
    }
  }
  if (digitalRead(btn1) == LOW)
  {
    if (subirDato <= 0)
    {
      subirDato = subirDato + 1; //se le resta uno por uno al contador de 8 bits
      Serial.println(subirDato);
    }
    if (subirDato >= 2) //no puede ser menor a 0
    {
      subirDato = 0;
      Serial.println(subirDato);
    }
  }
  if (contador1 == 1)
  {
    if (Serial3.available() > 0) {
      //Se leen los datos provenientes del ESP32
      dutycycleled1 = Serial3.readStringUntil('\n');
    }
    Serial3.println(dutycycleled1);
    analogWrite(ledv, encendido);
    delay(100);
    analogWrite(ledv, apagado);
    contador1 = 0;
  }

  if (subirDato == 1)
  {
    writeSD();
    analogWrite(leda, encendido);
    delay(100);
    analogWrite(leda, apagado);
    subirDato = 0;

  }

  delay(200);
}

void writeSD(void) {

  myFile = SD.open("test.csv", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("Escribiendo data");

    Serial.print("Temperatura: ");
    Serial.print(dutycycleled1);



    myFile.print(dutycycleled1.toInt());
    myFile.println(",");



    //archivo.println("27/10/21,14:34,25,50,30,9");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening data.csv");
  }

}
