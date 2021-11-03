//*****************************************************************************
// Universidad del Valle de Guatemala
// BE3015 - Electrónica Digital 2
// Fernando Delgado
// LCD LAB4
//*****************************************************************************

//*****************************************************************************
// Librerías
//*****************************************************************************
#include <Arduino.h>
#include <LiquidCrystal.h> // se incluye la libreria para el LCD

//*****************************************************************************
// Definición de pines
//*****************************************************************************
#define d4 27
#define d5 14
#define d6 12
#define d7 13
#define en 26
#define rs 25

#define pot1 15
//#define pot2 15



//*****************************************************************************
// Prototipos de función
//*****************************************************************************

//*****************************************************************************
// Variables Globales
//*****************************************************************************
// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]
LiquidCrystal LCD(rs, en, d4, d5, d6, d7);
uint8_t voltaje3;

int adcRaw;
float voltaje;
float adcFiltradoEMA = 0; // S(0) = Y(0)

float dutycycleled1 = 0; //se refiere a la señal PWM a la que trabajaran los leds desde un inicio.
int dutycycleled2 = 0;
double alpha = 0.09; // Factor de suavizado (0-1)

String voltaje2 = "";

//Contador
String dutycycleled3 = "";

//************************************************************************
//ISR
//*************************************************************************

//*****************************************************************************
// Configuración
//*****************************************************************************
void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);
  //LCD_Init(rs, en, d4, d5, d6, d7);
  // Initialize The LCD. Parameters: [ Columns, Rows ]
  LCD.begin(16, 2);
}
//*****************************************************************************
// Loop Principal
//*****************************************************************************
void loop()
{

  voltaje = analogReadMilliVolts(pot1);                                  //se fija un ajuste a la señal de entrada para poder trabajar con el sensor de temperatura lm35
  voltaje = (voltaje / 10) - 20 ;                                                //se divide poor cada cuantos mV se tiene un grado centigrado
  adcFiltradoEMA = (alpha * voltaje) + ((1.0 - alpha) * adcFiltradoEMA); //se procede con el filtro EMA
  dutycycleled1 = adcFiltradoEMA;       
           Serial.println(dutycycleled1);                        //esto me servira para mandar el valor a lso servicios de adafruit
  Serial2.println(dutycycleled1);

  if (Serial2.available() > 0)
  {
    //Se le debe dar enter
    voltaje2 = Serial2.readStringUntil('\n');

    Serial.print("Tempperatura es:");
    Serial.println(voltaje2);
  }

  voltaje3 = voltaje2.toInt();

  LCD.clear(); //s einicia la impresion en el LCD ya que debe limpiars ede primero para no sobreescibir sobre algun caracter anterior
  LCD.print("");
  LCD.print("Temperatura es:");
  LCD.setCursor(1,1);
  LCD.print("       ");
  LCD.print(voltaje3);

  delay(250); // es recomendado tener un delay por el LCD y como se debe refrescar el mismo
}
