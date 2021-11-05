
//*****************************************************************************
// Universidad del Valle de Guatemala
// BE3015 - Electrónica Digital 2
// Fernando Delgado 19144
// Proyecto 2
//*****************************************************************************

//*****************************************************************************
// Librerías
//-----------------------------------------------------------------------------
#include <SPI.h> //se incluye la libreria de la comunicacion SPI para que gfunciona la comunicacion con la SD 
#include <SD.h> //Se incluye la libreria de la SD 

//se incluye las librerias para la pantalla TFT
#include <stdint.h>
#include <stdbool.h>
#include <TM4C123GH6PM.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "bitmaps.h"
#include "font.h"
#include "lcd_registers.h"


//pines para las notas del Buzzer

#define note_cc 261
#define note_dd 294
#define note_ee 329
#define note_ff 349
#define note_g 391
#define note_gS 415
#define note_a 440
#define note_aS 455
#define note_b 466
#define note_cH 523
#define note_cSH 554
#define note_dH 587
#define note_dSH 622
#define note_eH 659
#define note_fH 698
#define note_fSH 740
#define note_gH 784
#define note_gSH 830
#define note_aH 880

//pin del Buzzer
#define sensorPin PB_5

//pines de las leds integradas en la Tiva c
#define leda PF_2
#define ledv PF_3
#define ledr PF_1

// Pines de los push buttons que trae integrado la Tiva C
#define btn1 PF_0
#define btn2 PF_4

//Se define lsopines que afectaran diectamente la pantalla TFT
#define LCD_RST PD_0
#define LCD_CS PD_1
#define LCD_RS PD_2
#define LCD_WR PD_3
#define LCD_RD PE_1
int DPINS[] = {PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7};



//----------------------------------------------------------------------------------------------------------------------
//Prototipos de funciones
//----------------------------------------------------------------------------------------------------------------------

void writeSD(void);//se llama a la funcion que me ayudara a guardar datos en la SD

//Se llama a las funciones que ayudaran a imprimir en la TFT
void LCD_Init(void);
void LCD_CMD(uint8_t cmd);
void LCD_DATA(uint8_t data);
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void LCD_Clear(unsigned int c);
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void LCD_Print(String text, int x, int y, int fontSize, int color, int background);

void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]);
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset);


//---------------------------------------------------------------------------------------------------------------------
//void
//---------------------------------------------------------------------------------------------------------------------
//Se llamara al pi den Buzzer
int buzzerPin = PF_2;

int sensorValue = 0;  // variable to store the value coming from the sensor
int voltaje2 = 0;//seran las variables asignadas para la entrada de comunicacion UART
int voltaje1 = 0;
String dutycycleled1 = ""; //Se llamara a esta funcion ya que sera la que guardara el dato del sensor del sp32

//se refiere a la señal PWM a la que trabajaran los leds desde un inicio.
int dutycycleled2 = 0;
int dutycycleled3 = 0;
int contador1 = 0;//contador para boton 1
int subirDato = 0;//contador para boton 2
int encendido = 255;
int apagado = 0;
File myFile;//se llama a etsa funcion para nombrar el tipo de archivo que se tendr apara guardar en la SD

extern uint8_t fondo[]; //Fondo//funcion para imprimir libreria en la pantalla TFT


//---------------------------------------------------------------------------------------------------------------------
//setup
//---------------------------------------------------------------------------------------------------------------------

void setup() {
  // declare the ledPin as an OUTPUT:
  Serial.begin (115200);//se llama a las dos comunicaciones seriales para que interactuen losdos microcontroladores
  Serial3.begin(115200);

  pinMode(buzzerPin, OUTPUT);//se declara el pin de salida al Buzzer
  pinMode (leda, OUTPUT);//se declaran las salidas de las leds integradas
  pinMode (ledv, OUTPUT);
  pinMode (ledr, OUTPUT);

  pinMode (btn1, INPUT_PULLUP);//de delclara el estado en el que se encuentran lso botones integrados en la Tiva C
  pinMode (btn2, INPUT_PULLUP);

  pinMode(PA_3, OUTPUT);//Se define la salida
  SPI.setModule(0);//se define el modulo del SPI en el canal 0 de 3

  // se tendra los comandos base para que fincione la comunicacion con la SD
  if (!SD.begin(PA_3)) {
    Serial.println("Fallo tu logica");//esto es progamacion preventba que si no tengo la SD conectada me mandara este mensaje al igual que si algo fallo
    return;
  }
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  Serial.begin(115200);
  GPIOPadConfigSet(GPIO_PORTB_BASE, 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
  Serial.println("Inicio");
  LCD_Init();
  LCD_Clear(0x00);
  // FillRect(0, 0, 319, 239, 0xFFFF);
  LCD_Bitmap(0, 0, 320, 240, fondo);//se llama a la imagen que se tiene en la libreria d efondo de 8 bits
  /*FillRect(50, 60, 20, 20, 0xF800);
    FillRect(70, 60, 20, 20, 0x07E0);
    FillRect(90, 60, 20, 20, 0x001F);*/

  //FillRect(0, 0, 319, 206, 0x421b);
  String text1 = "Sensor de temp";//se imprime en la pantalla TTF la frase
  //dutycycleled1 = "<HOLA";
  LCD_Print(text1, 50, 30, 2, 0x001F, 0xFFFF);//se coloca en el lugar donde se quiere la frase dando cordenadas especificas como tipo deletra y relleno
  LCD_Print(dutycycleled1, 120, 160, 2, 0x001F, 0xFFFF);
}

//---------------------------------------------------------------------------------------------------------------------
//loop
//---------------------------------------------------------------------------------------------------------------------
void loop() {

  if (digitalRead(btn2) == LOW)//Se inician loscontadores para encontrar flancos que inciilcen la sintrucciones
  {
    if (contador1 >= 0)
    {
      contador1 = contador1 + 1; //subira de uno en uno esto teniendo un mejor control
      Serial.println(contador1);//me iondicara en que valor esta el contador si esta conectado al momitor serial del programa
    }
    if (contador1 >= 2) //esto es para recetear el contador para numero mayores de 255
    {
      contador1 = 0;
      Serial.println(contador1);
    }
  }
  if (digitalRead(btn1) == LOW)//esto sera para inciializar otra funcion
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
  if (contador1 == 1)//se tiene el primer flanco en donde s epide el dato al SP32 del sensor y lo manda de regreso al PS32
  {
    if (Serial3.available() > 0) {//si el SP32 puede enviar algo lo toma
      //Se leen los datos provenientes del ESP32
      dutycycleled1 = Serial3.readStringUntil('\n');

    }
    Serial3.println(dutycycleled1);// envia el dato de regreso al sp32
    LCD_Print(dutycycleled1, 120, 100, 2, 0x001F, 0xFFFF);//imprime el dato en la TFT
    analogWrite(ledv, encendido);//me demuestra por sonido y led que funciono la funcion
    delay(100);
    analogWrite(ledv, apagado);
    contador1 = 0;
    beep(note_eH, 500);//nota del buzzer

  }

  if (subirDato == 1)//segunda funcion para guardar en SD
  {
    writeSD();//se tiene la funcion de escritura
    analogWrite(leda, encendido);//si funciona todo se prender una led e iniciara una melodia
    delay(100);
    analogWrite(leda, apagado);
    subirDato = 0;//se baja el flanco para que no suba mas datos 
    beep(note_a, 500);
    beep(note_a, 500);
    beep(note_a, 500);
    beep(note_ff, 350);
    beep(note_cH, 150);
    beep(note_a, 500);
    beep(note_ff, 350);
    beep(note_cH, 150);
    beep(note_a, 650);

  }

  delay(200);
}
//***************************************************************************************************************************************
// Función para escribir en la SD
//***************************************************************************************************************************************
void writeSD(void) {

  myFile = SD.open("test.csv", FILE_WRITE);//abrira una carpeta en formato excel o hoja de calculo 

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("Escribiendo data");

    Serial.print("Temperatura: ");
    Serial.print(dutycycleled1);//sobreescribe sobre el dato que guardo en el ultimo momento del sensor 


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
  for (int x = 0; x < 320 - 32; x++) {//uncion de animacion ya que se tien varias columnas reproducria cara una proporcionando un efecto de movimiento 
    int anim2 = (x / 35) % 3;
    LCD_Sprite(60, 100, 50, 50, pesaSprite, 3, anim2, 0, 1);
    delay(15);
  }
}
//***************************************************************************************************************************************
// Función para inicializar LCD
//***************************************************************************************************************************************
void LCD_Init(void) {
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RD, OUTPUT);
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(DPINS[i], OUTPUT);
  }
  //****************************************
  // Secuencia de Inicialización
  //****************************************
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_RD, HIGH);
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  delay(20);
  digitalWrite(LCD_RST, HIGH);
  delay(150);
  digitalWrite(LCD_CS, LOW);
  //****************************************
  LCD_CMD(0xE9);  // SETPANELRELATED
  LCD_DATA(0x20);
  //****************************************
  LCD_CMD(0x11); // Exit Sleep SLEEP OUT (SLPOUT)
  delay(100);
  //****************************************
  LCD_CMD(0xD1);    // (SETVCOM)
  LCD_DATA(0x00);
  LCD_DATA(0x71);
  LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0xD0);   // (SETPOWER)
  LCD_DATA(0x07);
  LCD_DATA(0x01);
  LCD_DATA(0x08);
  //****************************************
  LCD_CMD(0x36);  // (MEMORYACCESS)
  LCD_DATA(0x40 | 0x80 | 0x20 | 0x08); // LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0x3A); // Set_pixel_format (PIXELFORMAT)
  LCD_DATA(0x05); // color setings, 05h - 16bit pixel, 11h - 3bit pixel
  //****************************************
  LCD_CMD(0xC1);    // (POWERCONTROL2)
  LCD_DATA(0x10);
  LCD_DATA(0x10);
  LCD_DATA(0x02);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC0); // Set Default Gamma (POWERCONTROL1)
  LCD_DATA(0x00);
  LCD_DATA(0x35);
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC5); // Set Frame Rate (VCOMCONTROL1)
  LCD_DATA(0x04); // 72Hz
  //****************************************
  LCD_CMD(0xD2); // Power Settings  (SETPWRNORMAL)
  LCD_DATA(0x01);
  LCD_DATA(0x44);
  //****************************************
  LCD_CMD(0xC8); //Set Gamma  (GAMMASET)
  LCD_DATA(0x04);
  LCD_DATA(0x67);
  LCD_DATA(0x35);
  LCD_DATA(0x04);
  LCD_DATA(0x08);
  LCD_DATA(0x06);
  LCD_DATA(0x24);
  LCD_DATA(0x01);
  LCD_DATA(0x37);
  LCD_DATA(0x40);
  LCD_DATA(0x03);
  LCD_DATA(0x10);
  LCD_DATA(0x08);
  LCD_DATA(0x80);
  LCD_DATA(0x00);
  //****************************************
  LCD_CMD(0x2A); // Set_column_address 320px (CASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x3F);
  //****************************************
  LCD_CMD(0x2B); // Set_page_address 480px (PASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0xE0);
  //  LCD_DATA(0x8F);
  LCD_CMD(0x29); //display on
  LCD_CMD(0x2C); //display on

  LCD_CMD(ILI9341_INVOFF); //Invert Off
  delay(120);
  LCD_CMD(ILI9341_SLPOUT);    //Exit Sleep
  delay(120);
  LCD_CMD(ILI9341_DISPON);    //Display on
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para enviar comandos a la LCD - parámetro (comando)
//***************************************************************************************************************************************
void LCD_CMD(uint8_t cmd) {
  digitalWrite(LCD_RS, LOW);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = cmd;
  digitalWrite(LCD_WR, HIGH);
}
//***************************************************************************************************************************************
// Función para enviar datos a la LCD - parámetro (dato)
//***************************************************************************************************************************************
void LCD_DATA(uint8_t data) {
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = data;
  digitalWrite(LCD_WR, HIGH);
}
//***************************************************************************************************************************************
// Función para definir rango de direcciones de memoria con las cuales se trabajara (se define una ventana)
//***************************************************************************************************************************************
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
  LCD_CMD(0x2a); // Set_column_address 4 parameters
  LCD_DATA(x1 >> 8);
  LCD_DATA(x1);
  LCD_DATA(x2 >> 8);
  LCD_DATA(x2);
  LCD_CMD(0x2b); // Set_page_address 4 parameters
  LCD_DATA(y1 >> 8);
  LCD_DATA(y1);
  LCD_DATA(y2 >> 8);
  LCD_DATA(y2);
  LCD_CMD(0x2c); // Write_memory_start
}
//***************************************************************************************************************************************
// Función para borrar la pantalla - parámetros (color)
//***************************************************************************************************************************************
void LCD_Clear(unsigned int c) {
  unsigned int x, y;
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  SetWindows(0, 0, 319, 239); // 479, 319);
  for (x = 0; x < 320; x++)
    for (y = 0; y < 240; y++) {
      LCD_DATA(c >> 8);
      LCD_DATA(c);
    }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una línea horizontal - parámetros ( coordenada x, cordenada y, longitud, color)
//***************************************************************************************************************************************
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + x;
  SetWindows(x, y, l, y);
  j = l;// * 2;
  for (i = 0; i < l; i++) {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una línea vertical - parámetros ( coordenada x, cordenada y, longitud, color)
//***************************************************************************************************************************************
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + y;
  SetWindows(x, y, x, l);
  j = l; //* 2;
  for (i = 1; i <= j; i++) {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar un rectángulo - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  H_line(x  , y  , w, c);
  H_line(x  , y + h, w, c);
  V_line(x  , y  , h, c);
  V_line(x + w, y  , h, c);
}
//***************************************************************************************************************************************
// Función para dibujar un rectángulo relleno - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
/*void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  unsigned int i;
  for (i = 0; i < h; i++) {
    H_line(x  , y  , w, c);
    H_line(x  , y+i, w, c);
  }
  }
*/

void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + w;
  y2 = y + h;
  SetWindows(x, y, x2 - 1, y2 - 1);
  unsigned int k = w * h * 2 - 1;
  unsigned int i, j;
  for (int i = 0; i < w; i++) {
    for (int j = 0; j < h; j++) {
      LCD_DATA(c >> 8);
      LCD_DATA(c);

      //LCD_DATA(bitmap[k]);
      k = k - 2;
    }
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar texto - parámetros ( texto, coordenada x, cordenada y, color, background)
//***************************************************************************************************************************************
void LCD_Print(String text, int x, int y, int fontSize, int color, int background) {
  int fontXSize ;
  int fontYSize ;

  if (fontSize == 1) {
    fontXSize = fontXSizeSmal ;
    fontYSize = fontYSizeSmal ;
  }
  if (fontSize == 2) {
    fontXSize = fontXSizeBig ;
    fontYSize = fontYSizeBig ;
  }

  char charInput ;
  int cLength = text.length();
  Serial.println(cLength, DEC);
  int charDec ;
  int c ;
  int charHex ;
  char char_array[cLength + 1];
  text.toCharArray(char_array, cLength + 1) ;
  for (int i = 0; i < cLength ; i++) {
    charInput = char_array[i];
    Serial.println(char_array[i]);
    charDec = int(charInput);
    digitalWrite(LCD_CS, LOW);
    SetWindows(x + (i * fontXSize), y, x + (i * fontXSize) + fontXSize - 1, y + fontYSize );
    long charHex1 ;
    for ( int n = 0 ; n < fontYSize ; n++ ) {
      if (fontSize == 1) {
        charHex1 = pgm_read_word_near(smallFont + ((charDec - 32) * fontYSize) + n);
      }
      if (fontSize == 2) {
        charHex1 = pgm_read_word_near(bigFont + ((charDec - 32) * fontYSize) + n);
      }
      for (int t = 1; t < fontXSize + 1 ; t++) {
        if (( charHex1 & (1 << (fontXSize - t))) > 0 ) {
          c = color ;
        } else {
          c = background ;
        }
        LCD_DATA(c >> 8);
        LCD_DATA(c);
      }
    }
    digitalWrite(LCD_CS, HIGH);
  }
}
//***************************************************************************************************************************************
// Función para dibujar una imagen a partir de un arreglo de colores (Bitmap) Formato (Color 16bit R 5bits G 6bits B 5bits)
//***************************************************************************************************************************************
void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + width;
  y2 = y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  unsigned int k = 0;
  unsigned int i, j;

  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      LCD_DATA(bitmap[k]);
      LCD_DATA(bitmap[k + 1]);
      //LCD_DATA(bitmap[k]);
      k = k + 2;
    }
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una imagen sprite - los parámetros columns = número de imagenes en el sprite, index = cual desplegar, flip = darle vuelta
//***************************************************************************************************************************************
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 =   x + width;
  y2 =    y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  int k = 0;
  int ancho = ((width * columns));
  if (flip) {
    for (int j = 0; j < height; j++) {
      k = (j * (ancho) + index * width - 1 - offset) * 2;
      k = k + width * 2;
      for (int i = 0; i < width; i++) {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k - 2;
      }
    }
  } else {
    for (int j = 0; j < height; j++) {
      k = (j * (ancho) + index * width + 1 + offset) * 2;
      for (int i = 0; i < width; i++) {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k + 2;
      }
    }


  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// FUNCION DE BUZZER
//***************************************************************************************************************************************

void beep(int note, int duration)//funcion del buzzer para que funcionen la melodias
{
  tone(buzzerPin, note, duration / 2);
  delay(duration / 2);
  noTone(buzzerPin);
  delay(duration / 2 + 20);
}
