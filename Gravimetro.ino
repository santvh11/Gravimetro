/*
 * PROTOTIPO DE CÓDIGO PARA GRAVÍMETRO AUTOMATIZADO
 * * Versión con soporte para Teclado USB (Host Shield) y Nextion.
 * * Lógica de teclado y Nextion fusionada desde 'Combined_Keyboard_Nextion.ino'
 * * Lógica especial para 'page 2' (Selección de ángulo).
 */

// --- LIBRERÍAS DEL USB HOST SHIELD ---
#include <hidboot.h>
#include <usbhub.h> // Añadido para robustez
#include <SPI.h>    // Añadido (requerido por el Host Shield)

// --- LIBRERÍAS EXISTENTES ---
#include <math.h> // Para cálculos (pow, PI)
// #include "Nextion.h" // No la usamos, comunicación directa

// --- DEFINICIONES DE NEXTION  ---
#define nextionSerial Serial3 // Usamos Serial3 (Pines 14 y 15)
#define NEXTION_BAUD 9600     // Velocidad confirmada: 9600

// --- ESTADOS DE LA MÁQUINA ---
enum State
{
  STATE_IDLE,        // Esperando a Iniciar
  STATE_POSITIONING, // Posicionando el pendulo
  STATE_CALCULATING, // Soltar el pendulo y calculando g
  STATE_DISPLAY      // Mostrando resultado en Nextion
};

State currentState = STATE_IDLE;

// --- VARIABLES GLOBALES Y PINES ---

// --- OBJETOS DEL USB HOST (Método de Clase) ---
USB Usb;
HIDBoot<USB_HID_PROTOCOL_KEYBOARD> kbd(&Usb); // Driver para Teclado

// --- Variables de control de página Nextion ---
const int TOTAL_PAGES = 6; // 6 páginas (0, 1, 2, 3, 4, 5)
int currentPage = 0;

// --- Códigos de Tecla HID ---
#define KEY_SPACE 0x2C // Barra espaciadora
#define KEY_A 0x04     // Tecla 'A'
#define KEY_S 0x16     // Tecla 'S'
#define KEY_D 0x07     // Tecla 'D'

// --- Variables existentes ---
// PINES DE CONTROL
const int PIN_MOTOR_DIR1 = -1;
const int PIN_MOTOR_DIR2 = -1;
const int PIN_MOTOR_PWM = -1;
const int PIN_ELECTROMAGNET = -1;
const int PIN_ELECTROMAGNET_INV = -1;

// PINES DE SENSORES
const int PIN_SENSOR_IR = 2;
const int PIN_LIMIT_SWITCH = -1;

// PARÁMETROS DE ENTRADA
int selectedAngle = 5;
int selectedPeriods = 10;
bool startMeasurement = false;

// CONSTANTE FÍSICA
const double L_eq = -1.0;

// VARIABLES DE MEDICIÓN
volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;
volatile int stepCount = 0;
int pasos_objetivo = 0;

// VARIABLES DE CÁLCULO
double T_medido = 0.0;
double T_0 = 0.0;
double g_calculado = 0.0;

class KbdRptParser : public KeyboardReportParser
{
protected:
  // Se llama cuando una tecla es presionada
  void OnKeyDown(uint8_t mod, uint8_t key);
};

KbdRptParser Prs; // Nuestra instancia del parser

// --- PROTOTIPOS
void selectAngleAndAdvance(int angle);
void set_grav_nextion(float valor);
void sendNextionEnd();
void avanzar_pagina();

void setup()
{
  // 1. Iniciar monitor de PC
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("--- Inicio de Setup ---");

  // 2. Iniciar puerto para Nextion
  Serial.print("Iniciando Serial3 para Nextion a ");
  Serial.println(NEXTION_BAUD);
  nextionSerial.begin(NEXTION_BAUD);
  delay(1000); // Dar tiempo a la Nextion para que arranque
  Serial.println("Enviando 'page 0' a Nextion...");
  nextionSerial.print("page 0");
  sendNextionEnd();
  Serial.println("Comando 'page 0' enviado.");

  // 3. Configurar pines del gravímetro
  pinMode(PIN_MOTOR_DIR1, OUTPUT);

  // 4. INICIALIZACIÓN DEL USB HOST SHIELD
  Serial.println("Iniciando USB Host Shield...");
  if (Usb.Init() == -1)
  {
    Serial.println("Error: No se pudo iniciar el USB Host Shield. Deteniendo.");
    while (1)
      ; // Detener todo si el shield falla
  }
  Serial.println("USB Host listo. Conecte un teclado.");

  // Asignamos la instancia de nuestra CLASE parser
  kbd.SetReportParser(0, &Prs);

  Serial.println("Gravímetro listo. En estado IDLE.");
}

void loop()
{
  // Revisa el bus USB y llama a OnKeyDown() si hay una tecla.
  Usb.Task();

  // Revisar máquina de estados:
  switch (currentState)
  {
  case STATE_POSITIONING:
    currentState = STATE_CALCULATING;
    break;

  case STATE_CALCULATING:
    currentState = STATE_DISPLAY;
    break;

  case STATE_DISPLAY:
    currentState = STATE_IDLE;
    break;
  }
}

// --- FUNCIÓN OnKeyDown (Lógica de Teclado Centralizada) ---
// Esta función es llamada automáticamente por "Usb.Task()"
void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key)
{
  // Logica para la tecla principal
  if (key == KEY_SPACE) // 0x2C
  {
    switch (currentState)
    {
    case STATE_IDLE:
      currentState = STATE_CALCULATING;
      break;

    case STATE_POSITIONING:
      if (currentPage == 3)
      {
        avanzar_pagina();
      }

      break;

    case STATE_DISPLAY:
      avanzar_pagina();
      currentState = STATE_IDLE;
      break;
    }

    return;
  }

  // --- LÓGICA ESPECIAL PARA PÁGINA 2 ---
  if (currentPage == 2)
  {
    switch (key)
    {
    case KEY_A:
      selectAngleAndAdvance(10);
      break;

    case KEY_S:
      selectAngleAndAdvance(15);
      break;

    case KEY_D:
      selectAngleAndAdvance(20);
      break;

    default:
      Serial.println("currentPage:");
      Serial.println(currentPage);
      Serial.println("Presione A, S o D para seleccionar el ángulo.");
      break;
    }
    return;
  }
}

/**
 * @brief Selecciona un ángulo, actualiza la UI y avanza a la página 3.
 * @param angle El ángulo (10, 15, o 20) que fue seleccionado.
 */
void selectAngleAndAdvance(int angle)
{
  // 1. Imprimir en el Serial Monitor
  Serial.print("Teclado: Ángulo seleccionado: ");
  Serial.println(angle);

  // 2. Asignar el ángulo a la variable global
  selectedAngle = angle;

  // 3. Actualizar la UI de Nextion
  function_UpdateNextionUI_Angle(selectedAngle);

  // 4. Forzar avance a la siguiente página
  currentPage = 3;
  nextionSerial.print("page 3");
  sendNextionEnd();
}

/**
 * @brief Actualiza el texto de un objeto en la pantalla Nextion con un valor de gravedad formateado.
 * @param valor El valor flotante (ej. gravedad) que se mostrará en la pantalla.
 */
void set_grav_nextion(float valor)
{
  // 1. Crear el texto formateado
  // Convierte el float a String con 2 decimales y agrega "g="
  String texto_a_enviar = "g=" + String(valor, 2);

  // 2. Iniciar el comando para el objeto 'result' en la pantalla Nextion
  Serial3.print("result.txt=\"");

  // 3. Enviar el texto formateado
  Serial3.print(texto_a_enviar);

  sendNextionEnd();
}

void avanzar_pagina()
{
  Serial.println("Teclado: Barra espaciadora (Cambiando página)");

  // 1. Avanza a la siguiente página
  currentPage++;
  if (currentPage >= TOTAL_PAGES)
  {
    currentPage = 0;
  }

  // 3. Envía el comando a la Nextion
  Serial.print("Enviando 'page ");
  Serial.print(currentPage);
  Serial.println("' a Nextion...");

  nextionSerial.print("page ");
  nextionSerial.print(currentPage);
  sendNextionEnd();

  Serial.println("Comando enviado.");
}

void sendNextionEnd()
{
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  // Pequeña pausa para que Nextion procese el comando
  delay(50);
}

void function_UpdateNextionUI_Angle(int angle)
{
  // Esta función es necesaria para que si el teclado cambia
  // el ángulo, la pantalla Nextion lo refleje.
  Serial.print("UI Debería actualizarse a ángulo: ");
  Serial.println(angle);
  // ej. nextionSerial.print("page2.t_angulo.txt=\""); ...
  // sendNextionEnd();
}
