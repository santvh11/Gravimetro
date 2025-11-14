/*
 * CÓDIGO SIMPLIFICADO PARA CONTROLAR MOTOR Y ELECTROIMÁN
 *
 * FUNCIONALIDAD:
 * 1. Al arrancar, enciende el electroimán.
 * 2. Al presionar 'A', el motor se mueve en reversa por 'PULSE_DURATION' ms y se detiene.
 * 3. Al presionar 'D', el motor se mueve hacia adelante por 'PULSE_DURATION' ms y se detiene.
 * 4. Usa un método no-bloqueante (millis()) para no congelar el USB Host Shield.
 */

// --- LIBRERÍAS DEL USB HOST SHIELD (ESENCIALES) ---
#include <hidboot.h>
#include <usbhub.h>
#include <SPI.h>

// --- PINES DE CONTROL ---
// ¡Asegúrate de que estos pines coincidan con tu hardware!
const int PIN_ELECTROMAGNET = 8; // Pin para el relevador del electroimán

const int PIN_MOTOR_DIR1 = 9;  // Pin de Dirección 1 del Puente H
const int PIN_MOTOR_DIR2 = 10; // Pin de Dirección 2 del Puente H
const int PIN_MOTOR_PWM = 11;  // Pin de Velocidad (PWM) del Puente H

// Velocidad del motor (0 a 255)
const int VELOCIDAD_MOTOR = 200;

// --- NUEVO: DURACIÓN DEL PULSO DE MOVIMIENTO ---
const unsigned long PULSE_DURATION = 50; // ¡AJUSTA ESTO! Duración del pulso en milisegundos

// --- CÓDIGOS DE TECLA HID (ESENCIALES) ---
#define KEY_A 0x04 // Tecla 'A'
#define KEY_D 0x07 // Tecla 'D'

// --- OBJETOS DEL USB HOST (ESENCIALES) ---
USB Usb;
HIDBoot<USB_HID_PROTOCOL_KEYBOARD> kbd(&Usb); // Driver para Teclado

// --- NUEVAS VARIABLES GLOBALES PARA MOVIMIENTO NO-BLOQUEANTE ---
bool motorIsMoving = false;           // Flag para saber si el motor está en un pulso
unsigned long motorMoveStartTime = 0; // Para guardar cuándo empezó el pulso

// --- CLASE PARSER DEL TECLADO (MODIFICADA) ---
// Ya no necesitamos OnKeyUp
class KbdRptParser : public KeyboardReportParser
{
protected:
    void OnKeyDown(uint8_t mod, uint8_t key); // Se llama cuando presionas
};

KbdRptParser Prs; // Nuestra instancia del parser

// --- 3. CONFIGURACIÓN INICIAL (SETUP) ---

void setup()
{
    // 1. Iniciar monitor de PC
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("--- Inicio de Setup (Control por Pulsos) ---");

    // 2. Configurar pines de control
    Serial.println("Configurando pines...");
    pinMode(PIN_ELECTROMAGNET, OUTPUT);
    pinMode(PIN_MOTOR_DIR1, OUTPUT);
    pinMode(PIN_MOTOR_DIR2, OUTPUT);
    pinMode(PIN_MOTOR_PWM, OUTPUT);

    // 3. Encender el electroimán
    digitalWrite(PIN_ELECTROMAGNET, HIGH); // Activa el electroimán
    Serial.println("Electroimán ENCENDIDO.");

    // 4. Detener el motor por si acaso
    motorDetener();
    Serial.println("Motor inicializado (detenido).");

    // 5. INICIALIZACIÓN DEL USB HOST SHIELD
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
}

// --- 4. BUCLE PRINCIPAL (LOOP) ---
void loop()
{
    // --- TAREA CRÍTICA DEL HOST SHIELD ---
    // Revisa el bus USB y llama a OnKeyDown() si hay una tecla.
    Usb.Task();

    // --- LÓGICA DE MOVIMIENTO (NO-BLOQUEANTE) ---
    // Esto se ejecuta en cada ciclo del loop

    // 1. Preguntamos: ¿Está el motor en medio de un pulso?
    if (motorIsMoving)
    {

        // 2. Si es así, ¿Ya pasó el tiempo del pulso?
        unsigned long currentTime = millis();
        if (currentTime - motorMoveStartTime >= PULSE_DURATION)
        {

            // 3. Si ya pasó, detenemos el motor y bajamos la bandera
            Serial.println("Pulso terminado. Motor Detenido.");
            motorDetener();
            motorIsMoving = false;
        }
    }
}

// --- 5. IMPLEMENTACIÓN DE LAS FUNCIONES DEL PARSER ---

/**
 * @brief Se llama automáticamente cuando una tecla es PRESIONADA.
 * --- MODIFICADO ---
 */
void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key)
{

    // Si el motor YA se está moviendo, ignoramos nuevas pulsaciones.
    if (motorIsMoving)
    {
        return;
    }

    Serial.print("Presionada: ");
    Serial.println(key, HEX); // Muestra el código de la tecla

    switch (key)
    {
    case KEY_A: // Tecla 'A'
        Serial.println("Tecla A: Iniciando pulso en Reversa...");
        motorIsMoving = true;          // 1. Levantamos la bandera
        motorMoveStartTime = millis(); // 2. Guardamos el tiempo de inicio
        motorReversa();                // 3. Arrancamos el motor
        break;

    case KEY_D: // Tecla 'D'
        Serial.println("Tecla D: Iniciando pulso Adelante...");
        motorIsMoving = true;          // 1. Levantamos la bandera
        motorMoveStartTime = millis(); // 2. Guardamos el tiempo de inicio
        motorAdelante();               // 3. Arrancamos el motor
        break;
    }
}

// --- 6. FUNCIONES DE CONTROL DEL MOTOR (Sin cambios) ---

/**
 * @brief Mueve el motor hacia "adelante".
 */
void motorAdelante()
{
    digitalWrite(PIN_MOTOR_DIR1, HIGH);
    digitalWrite(PIN_MOTOR_DIR2, LOW);
    analogWrite(PIN_MOTOR_PWM, VELOCIDAD_MOTOR); // Aplica velocidad
}

/**
 * @brief Mueve el motor en "reversa".
 */
void motorReversa()
{
    digitalWrite(PIN_MOTOR_DIR1, LOW);
    digitalWrite(PIN_MOTOR_DIR2, HIGH);
    analogWrite(PIN_MOTOR_PWM, VELOCIDAD_MOTOR); // Aplica velocidad
}

/**
 * @brief Detiene el motor (freno).
 */
void motorDetener()
{
    digitalWrite(PIN_MOTOR_DIR1, LOW);
    digitalWrite(PIN_MOTOR_DIR2, LOW);
    analogWrite(PIN_MOTOR_PWM, 0); // Velocidad 0
}