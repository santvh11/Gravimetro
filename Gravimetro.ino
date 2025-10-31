/*
 * PROTOTIPO DE CÓDIGO PARA GRAVÍMETRO AUTOMATIZADO
 * * Versión con soporte para Teclado USB (Host Shield) y Nextion.
 * * Lógica de teclado y Nextion fusionada desde 'Combined_Keyboard_Nextion.ino'
 * * Lógica especial para 'page 2' (Selección de ángulo).
 */

// --- LIBRERÍAS DEL USB HOST SHIELD ---
#include <hidboot.h>
#include <usbhub.h> // Añadido para robustez
#include <SPI.h>     // Añadido (requerido por el Host Shield)

// --- LIBRERÍAS EXISTENTES ---
#include <math.h> // Para cálculos (pow, PI)
// #include "Nextion.h" // No la usamos, comunicación directa

// --- NUEVO: DEFINICIONES DE NEXTION (del código combinado) ---
#define nextionSerial Serial3 // Usamos Serial3 (Pines 14 y 15)
#define NEXTION_BAUD 9600     // Velocidad confirmada: 9600

// --- 1. ESTADOS DE LA MÁQUINA (Sin cambios) ---
enum State {
  STATE_IDLE,      // Esperando órdenes
  STATE_HOMING,    // Calibrando posición del mecanismo
  STATE_POSITIONING, // Moviendo el péndulo al ángulo deseado
  STATE_RELEASE,     // Soltando el péndulo
  STATE_MEASURING,   // Midiendo el periodo (esperando a la ISR)
  STATE_CALCULATING, // Calculando g
  STATE_DISPLAY      // Mostrando resultado en Nextion
};
State currentState = STATE_IDLE;


// --- 2. VARIABLES GLOBALES Y PINES ---

// --- OBJETOS DEL USB HOST (Método de Clase) ---
USB     Usb;
HIDBoot<USB_HID_PROTOCOL_KEYBOARD> kbd(&Usb); // Driver para Teclado

// --- Variables de control de página Nextion ---
const int TOTAL_PAGES = 6; // 6 páginas (0, 1, 2, 3, 4, 5)
int currentPage = 0;

// --- Códigos de Tecla HID ---
#define KEY_SPACE 0x2C // Barra espaciadora
#define KEY_A 0x04     // Tecla 'A'
#define KEY_S 0x16     // Tecla 'S'
#define KEY_D 0x07     // Tecla 'D'

// --- Variables existentes (Sin cambios) ---
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

// --- NUEVO: CLASE PARSER DEL TECLADO (Método robusto) ---
class KbdRptParser : public KeyboardReportParser
{
  protected:
    // Se llama cuando una tecla es presionada
    void OnKeyDown(uint8_t mod, uint8_t key);
};

KbdRptParser Prs; // Nuestra instancia del parser

// --- NUEVO: FUNCIÓN HELPER DE NEXTION ---
/**
 * @brief Envía el comando de terminación a la pantalla Nextion.
 */
void sendNextionEnd() {
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  nextionSerial.write(0xFF);
  // Pequeña pausa para que Nextion procese el comando
  delay(50);
}


// --- 3. CONFIGURACIÓN INICIAL (SETUP) ---

void setup() {
  // 1. Iniciar monitor de PC
  Serial.begin(115200);
  while (!Serial);
  Serial.println("--- Inicio de Setup (Gravímetro Integrado) ---");

  // 2. Iniciar puerto para Nextion
  Serial.print("Iniciando Serial3 para Nextion a ");
  Serial.println(NEXTION_BAUD);
  nextionSerial.begin(NEXTION_BAUD);
  delay(1000); // Dar tiempo a la Nextion para que arranque
  Serial.println("Enviando 'page 0' a Nextion...");
  nextionSerial.print("page 0");
  sendNextionEnd();
  Serial.println("Comando 'page 0' enviado.");

  // 3. Configurar pines del gravímetro (Sin cambios)
  pinMode(PIN_MOTOR_DIR1, OUTPUT);
  // ... (etc.)

  // 4. INICIALIZACIÓN DEL USB HOST SHIELD
  Serial.println("Iniciando USB Host Shield...");
  if (Usb.Init() == -1) {
    Serial.println("Error: No se pudo iniciar el USB Host Shield. Deteniendo.");
    while (1); // Detener todo si el shield falla
  }
  Serial.println("USB Host listo. Conecte un teclado.");

  // Asignamos la instancia de nuestra CLASE parser
  kbd.SetReportParser(0, &Prs);

  Serial.println("Gravímetro listo. En estado IDLE.");
}

// --- 4. BUCLE PRINCIPAL (LOOP) - LA MÁQUINA DE ESTADOS ---

void loop() {

  // --- TAREA CRÍTICA DEL HOST SHIELD ---
  // Revisa el bus USB y llama a OnKeyDown() si hay una tecla.
  Usb.Task();

  // La máquina de estados FSM (sin cambios)
  switch (currentState) {

    // --- ESTADO 1: REPOSO ---
    case STATE_IDLE:
      // 1. Escuchar a la Nextion (como antes)
      checkNextionCommands();

      // 2. Escuchar al Teclado (manejado automáticamente por Usb.Task())
      //    Ambas funciones pueden poner 'startMeasurement = true'

      if (startMeasurement) {
        startMeasurement = false; // Resetea el flag
        Serial.println("Comando 'Start' recibido. Pasando a HOMING.");
        currentState = STATE_HOMING;
      }
      break;

    // --- (El resto de estados: HOMING, POSITIONING, RELEASE, MEASURING, ---
    // --- CALCULATING, DISPLAY... son EXACTAMENTE IGUALES) ---

    case STATE_HOMING:
      // ... (código idéntico) ...
      currentState = STATE_POSITIONING;
      break;

    case STATE_POSITIONING:
      // ... (código idéntico) ...
      currentState = STATE_RELEASE;
      break;

    case STATE_RELEASE:
      // ... (código idéntico) ...
      currentState = STATE_MEASURING;
      break;

    case STATE_MEASURING:
      // ... (código idéntico) ...
      if (endTime > 0) {
        currentState = STATE_CALCULATING;
      }
      break;

    case STATE_CALCULATING:
      // ... (código idéntico) ...
      currentState = STATE_DISPLAY;
      break;

    case STATE_DISPLAY:
      // ... (código idéntico) ...
      currentState = STATE_IDLE;
      break;
  }
}

// --- 5. RUTINA DE SERVICIO DE INTERRUPCIÓN (ISR) ---
// (Sin cambios)
void sensorISR() {
  // ... (código idéntico) ...
}

// --- 6. FUNCIÓN OnKeyDown (Lógica de Teclado Centralizada) ---
// Esta función es llamada automáticamente por "Usb.Task()"
void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key) {

  // IMPORTANTE: Solo procesamos teclas si estamos en IDLE
  if (currentState != STATE_IDLE) {
    return;
  }

  // --- LÓGICA ESPECIAL PARA PÁGINA 2 ---
  if (currentPage == 2) {
    switch (key) {
      case KEY_A: // Tecla 'A'
        Serial.println("Teclado: Ángulo seleccionado: 10");
        selectedAngle = 10;
        function_UpdateNextionUI_Angle(selectedAngle); // Actualiza la UI
        
        // Forzar avance a la siguiente página
        currentPage = 3; 
        nextionSerial.print("page 3");
        sendNextionEnd();
        break;

      case KEY_S: // Tecla 'S'
        Serial.println("Teclado: Ángulo seleccionado: 15");
        selectedAngle = 15;
        function_UpdateNextionUI_Angle(selectedAngle); // Actualiza la UI
        
        // Forzar avance a la siguiente página
        currentPage = 3;
        nextionSerial.print("page 3");
        sendNextionEnd();
        break;

      case KEY_D: // Tecla 'D'
        Serial.println("Teclado: Ángulo seleccionado: 20");
        selectedAngle = 20;
        function_UpdateNextionUI_Angle(selectedAngle); // Actualiza la UI
        
        // Forzar avance a la siguiente página
        currentPage = 3;
        nextionSerial.print("page 3");
        sendNextionEnd();
        break;
      
      default:
        // Ignorar cualquier otra tecla (incluyendo ESPACIO)
        Serial.println("currentPage:");
        Serial.println(currentPage);
        Serial.println("Presione A, S o D para seleccionar el ángulo.");
        break;
    }
    return; // Importante: Salir de la función aquí
  }

  // --- LÓGICA NORMAL (Para todas las demás páginas) ---
  switch (key) {
    // --- Lógica de Navegación Nextion ---
    case KEY_SPACE: // 0x2C
    if (currentPage != 2) {
      Serial.println("Teclado: Barra espaciadora (Cambiando página)");

      // 1. Avanza a la siguiente página
      currentPage++;
      if (currentPage >= TOTAL_PAGES) {
        currentPage = 0;
      }
      
      // 3. Envía el comando a la Nextion
      Serial.print("Enviando 'page ");
      Serial.print(currentPage);
      Serial.println("' a Nextion...");

      nextionSerial.print("page ");
      nextionSerial.print(currentPage);
      sendNextionEnd(); // Envía el terminador

      Serial.println("Comando enviado.");
      }
      break;
  }
}


// --- 7. FUNCIONES "PLACEHOLDER" (Sin cambios) ---

void checkNextionCommands() {
  // ... (código idéntico para leer de la Nextion) ...
}

void function_HomingMotor(int limitSwitchPin) {
  // ... (código idéntico) ...
}

void function_MoveToAngle(int angle) {
  // ... (código idéntico) ...
}

void function_RetractMotor() {
  // ... (código idéntico) ...
}

void function_SendToNextion(double g_value) {
  // ... (código idéntico) ...
}

void function_UpdateNextionUI_Angle(int angle) {
  // Esta función es necesaria para que si el teclado cambia
  // el ángulo, la pantalla Nextion lo refleje.
  Serial.print("UI Debería actualizarse a ángulo: ");
  Serial.println(angle);
  // ej. nextionSerial.print("page2.t_angulo.txt=\""); ...
  // sendNextionEnd();
}

void function_UpdateNextionUI_Periods(int periods) {
  // Idem para el número de periodos
}
