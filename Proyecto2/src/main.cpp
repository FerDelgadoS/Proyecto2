//*****************************************************************************
// Universidad del Valle de Guatemala
// BE3015 - Electrónica Digital 2
// Fernando Delgado 19144
// Proyecto #2
//*****************************************************************************

//*****************************************************************************
// Librerías
//*****************************************************************************
#include <Arduino.h>       // se incluye la librereria de arduino
#include <LiquidCrystal.h> // se incluye la libreria para el LCD

//*****************************************************************************
// Definición de pines
//****************************************************************************

//se nombran los pines de la LCD
#define d4 27
#define d5 14
#define d6 12
#define d7 13
#define en 26
#define rs 25

//se nombran el pin del sensor de temperatura
#define pot1 15

//*****************************************************************************
// Prototipos de función
//*****************************************************************************

//*****************************************************************************
// Variables Globales
//*****************************************************************************
// Create An LCD Object. Signals: [ RS, EN, D4, D5, D6, D7 ]
//se crea la libreria para inicializar el LCD
LiquidCrystal LCD(rs, en, d4, d5, d6, d7);
uint8_t voltaje3; //se nombra a una variable la cual recibira el texto de la comunicacion USART

int adcRaw;    //la netrada del sensor analogico
float voltaje; //la medida que se tendra con decimales del sensor

//filtro para el sensor
float adcFiltradoEMA = 0; // S(0) = Y(0)

float dutycycleled1 = 0; //Se dara uso para la señal que recibira este codigo atravez de la comunicacion UART
int dutycycleled2 = 0;
double alpha = 0.09; // Factor de suavizado (0-1) Factor necesario para el filtro aplicado en el sensor

String voltaje2 = ""; //se denota otra variale tipo texto

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
  //se iniciara las dos comunicasiones sriales donde unsa sera con el monitor del programa y la otra con la Tiva C
  Serial.begin(115200);
  Serial2.begin(115200);
  //LCD_Init(rs, en, d4, d5, d6, d7);
  // Initialize The LCD. Parameters: [ Columns, Rows ]
  LCD.begin(16, 2); //se iniciara lacomunicacion con la LCD
}
//*****************************************************************************
// Loop Principal
//*****************************************************************************
void loop()
{

  voltaje = analogReadMilliVolts(pot1);                                  //se fija un ajuste a la señal de entrada para poder trabajar con el sensor de temperatura lm35
  voltaje = (voltaje / 10) - 10;                                         //se divide poor cada cuantos mV se tiene un grado centigrado
  adcFiltradoEMA = (alpha * voltaje) + ((1.0 - alpha) * adcFiltradoEMA); //se procede con el filtro EMA
  dutycycleled1 = adcFiltradoEMA;
  Serial.println(dutycycleled1);  //esto me servira para mandar el valor al monitor y saber si esta fncionando el codigo
  Serial2.println(dutycycleled1); // mandara el dato por medio de la comunicacion UART a la tiva c

  if (Serial2.available() > 0) //si se estara recibiendo datos desde la tiva funcionar
  {
    //Se le debe dar enter
    voltaje2 = Serial2.readStringUntil('\n');
    //imprimir que dato fue el que entro
    Serial.print("Tempperatura es:");
    Serial.println(voltaje2);
  }

  voltaje3 = voltaje2.toInt(); // pasar de texto numero entreo que sera el que despliegue en la LCD 

  LCD.clear(); //s einicia la impresion en el LCD ya que debe limpiars ede primero para no sobreescibir sobre algun caracter anterior
  LCD.print("");
  LCD.print("Temperatura es:");
  LCD.setCursor(1, 1); //un espaciador para representar el dato que obtuvo el SP32 
  LCD.print("       ");
  LCD.print(voltaje3);

  delay(250); // es recomendado tener un delay por el LCD y como se debe refrescar el mismo
}
