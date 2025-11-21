/*
 * PROTOTIPO DE CÓDIGO PARA GRAVÍMETRO AUTOMATIZADO (CON PÉNDULO)
 * MODIFICADO: Migración de Sensor Analógico a Digital
 * ACTUALIZACIÓN: Lógica invertida (HIGH = Detectado)
 */

// --- LIBRERÍAS DEL USB HOST SHIELD ---
#include <hidboot.h>
#include <usbhub.h>
#include <SPI.h> // Requerido por el Host Shield

// --- OTRAS LIBRERÍAS ---
#include <math.h> // Para M_PI y pow()

// --- DEFINICIONES DE NEXTION  ---
#define nextionSerial Serial2 // Usamos Serial3 (Pines 14 y 15)
#define NEXTION_BAUD 9600     // Velocidad confirmada: 9600

// --- DEFINICIONES DE PÁGINAS (Mejora de legibilidad) ---
#define PAGE_HOME 0           // Página inicial`
#define PAGE_POSITIONING 1    // Página de posicionamiento del péndulo en el ángulo inicial
#define PAGE_CALCULATING 2    // Página de calculando 'g'
#define PAGE_INFO 3           // Página de información y para ir a resultados
#define PAGE_DISPLAY_RESULT 4 // Página de resultados
#define PAGE_ERROR 5          // Página de error

// --- ESTADOS DE LA MÁQUINA ---
enum State
{
    STATE_IDLE,        // Esperando a Iniciar
    STATE_POSITIONING, // Posicionando el pendulo
    STATE_CALCULATING, // Calculando con cuenta regresiva
    STATE_INFO,        // Mostrando info y esperando para ir a resultados
    STATE_DISPLAY,     // Mostrando resultado en Nextion
    STATE_ERROR        // Estado de error
};

State currentState = STATE_IDLE;
State lastState = STATE_IDLE; // Variable para evitar "spam" en el Serial Monitor

// --- VARIABLES GLOBALES Y PINES ---

// --- a. Obj del host USB ---
USB Usb;
HIDBoot<USB_HID_PROTOCOL_KEYBOARD> kbd(&Usb); // Driver para Teclado

// --- b. Variables de control de página Nextion ---
const int TOTAL_PAGES = 5;
int currentPage = 0;

// --- c. Códigos de Tecla HID ---
#define KEY_SPACE 0x2C // Barra espaciadora
#define KEY_A 0x04     // Tecla 'A'
#define KEY_S 0x16     // Tecla 'S'
#define KEY_D 0x07     // Tecla 'D'
#define KEY_Z 0x1D     // Tecla 'Z'
#define KEY_C 0x06     // Tecla 'C'
#define KEY_V 0x19     // Tecla 'V'
#define KEY_B 0x05     // Tecla 'B'

// --- d. Pines ---
const int PIN_SENSOR_IR = A9; // Asegúrate de que este pin soporte digitalRead (A9 suele soportarlo)

// NOTA: Lógica invertida por solicitud del usuario.
// Lógica actual: HIGH = Detectado (Objeto presente), LOW = Libre.

// --- e.  Constantes medición ---
const int NUM_PERIODS_TO_MEASURE = 10; // Número de oscilaciones a promediar
const double G_STANDARD = 9.80665;     // Gravedad estándar m/s^2 (para cálculo ideal)

// Longitud Equivalente 'L_eq' calculada desde SolidWorks
// L_eq = I_pivote / (masa * dist_centro_de_masa)
// L_eq = 4684350.95 g·mm^2 / (225.46 g * 119.96 mm) = 173.2 mm
const double PENDULUM_LENGTH_METERS = 0.1732;

// --- f. Variables de medición ---
int selectedAngle = 60; // Ángulo por defecto

// --- g. Variables de cálculo ---
double T_medido = 0.0;    // Período real medido (T)
double T_0_ideal = 0.0;   // Período ideal (T_0) (para ángulo pequeño)
double T_esperado = 0.0;  // Período teórico esperado para el ángulo grande
double g_calculado = 0.0; // Gravedad 'g' calculada

class KbdRptParser : public KeyboardReportParser
{
protected:
    // Se llama cuando una tecla es presionada
    void OnKeyDown(uint8_t mod, uint8_t key);
};

KbdRptParser Prs; // Nuestra instancia del parser

// --- PROTOTIPOS ---
void set_grav_nextion(float g);
void sendNextionEnd();
void goto_next_page();
void function_UpdateNextionUI_Angle(int angle);
double perform_pendulum_measurement();
double calculate_T0_small_angle(double length_m, double gravity_m_s2);
double calculate_large_angle_correction(int angle_deg);

// --- CONFIGURACIÓN ---
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
    delay(1000);
    Serial.println("Enviando 'page 0' a Nextion...");
    nextionSerial.print("page 0");
    sendNextionEnd();
    currentPage = PAGE_HOME;
    Serial.println("Comando 'page 0' enviado.");

    // 3. Configurar pines del gravímetro
    pinMode(PIN_SENSOR_IR, INPUT); // Input digital

    // 6. Inizializar USB Host Shield
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

// --- BUCLE PRINCIPAL ---
void loop()
{
    Usb.Task(); // Tarea de fondo esencial para el teclado

    // Imprime el estado solo cuando cambia (evita spam)
    if (currentState != lastState)
    {
        Serial.print("Cambiando de estado: ");
        Serial.print(lastState);
        Serial.print(" -> ");
        Serial.println(currentState);
        lastState = currentState;
    }

    // Revisar máquina de estados:
    switch (currentState)
    {
    case STATE_IDLE:
        // No hacer nada, esperar al teclado
        break;

    case STATE_POSITIONING:
        // CAMBIO LOGICA: Esperar mientras HIGH (detectado/bloqueado)
        // Si el sensor está detectando el péndulo, esperamos a que salga.
        while (digitalRead(PIN_SENSOR_IR) == HIGH)
        {
            // Esperar a que el péndulo salga del sensor
            delay(10);
        }

        Serial.println("Ya el pendulo no está en el sensor. Avanzando...");
        Serial.println(digitalRead(PIN_SENSOR_IR)); // Debug del valor digital

        delay(2000); // Pequeña pausa antes de avanzar
        goto_next_page(); // Ir a pág "Calculando"
        currentState = STATE_CALCULATING;

        break;

    case STATE_CALCULATING:
        Serial.println("Iniciando medición del péndulo...");

        // 1. Realizar la medición (soltar, medir periodos)
        T_medido = perform_pendulum_measurement();

        if (T_medido > 0.0) // Revisión de éxito (devuelve 0.0 en error)
        {
            // 2. Calcular el factor de corrección para el ángulo usado
            double correction_factor = calculate_large_angle_correction(selectedAngle);

            // 3. Calcular 'g' USANDO LA FÓRMULA EXACTA
            // g = (4 * pi^2 * L * C^2) / T_med^2
            g_calculado = (4.0 * M_PI * M_PI * PENDULUM_LENGTH_METERS * pow(correction_factor, 2)) / (T_medido * T_medido);
            Serial.println("Gravedad = " + String(g_calculado, 6) + " m/s^2");

            // 4. Calcular valores teóricos para comparar
            T_0_ideal = calculate_T0_small_angle(PENDULUM_LENGTH_METERS, G_STANDARD);
            T_esperado = T_0_ideal * correction_factor; // El período que esperábamos medir

            // 5. Imprimir datos de depuración al Monitor Serial
            Serial.println("------ Resumen de Períodos (Depuración) ------");
            Serial.print("Periodo Esperado (T_esp): ");
            Serial.print(T_esperado, 6);
            Serial.println(" s");
            Serial.print("Periodo Ideal (T_0): ");
            Serial.print(T_0_ideal, 6);
            Serial.println(" s");
            Serial.println("-------------------------------------------------");

            // 6. Ir a la página de información
            goto_next_page(); // Ir a pág "Info"

            set_grav_nextion((float)g_calculado); // Enviar resultado a Nextion
            currentState = STATE_INFO;
        }
        else
        {
            // Hubo un error en la medición (timeout)
            Serial.println("Error de medición. Enviando 0.0 a Nextion.");
            goto_error_page();
            currentState = STATE_ERROR;
        }

        break;

    case STATE_INFO:
        // No hacer nada, esperar al teclado
        break;

    case STATE_DISPLAY:
        // No hacer nada, esperar al teclado
        break;

    case STATE_ERROR:
        // No hacer nada, esperar al teclado
        break;

    default:
        break;
    }
    delay(5); // Pequeña pausa para estabilizar el bucle
}

// --- LÓGICA DEL TECLADO ---
void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key)
{
    if (key == KEY_SPACE)
    {
        switch (currentState)
        {
        case STATE_IDLE:
            // goto_specific_page(PAGE_DISPLAY_RESULT);
            // ir a pág "Posicionando" y cambiar estado
            goto_next_page(); // Ir a pág 1 (PAGE_POSITIONING)
            currentState = STATE_POSITIONING;
            break;

        case STATE_POSITIONING:
            // ir a pág "Calculando" y cambiar estado
            goto_next_page(); // Ir a pág 2 (PAGE_CALCULATING)
            currentState = STATE_CALCULATING;
            break;

        case STATE_CALCULATING:
            goto_next_page(); // Ir a pág "Info"
            currentState = STATE_INFO;
            break;

        case STATE_INFO:
            // Ir a pág "Resultados" y cambiar estado
            goto_next_page(); // Ir a la pág 5 (PAGE_DISPLAY_RESULT)
            currentState = STATE_DISPLAY;
            break;

        case STATE_DISPLAY:
            // Reset: Volver a pág "Home" y cambiar estado
            goto_next_page(); // Volver a la página 0 (PAGE_HOME)
            currentState = STATE_IDLE;
            break;

        case STATE_ERROR:
            // Reset desde error: Volver a la página de posicionamiento y cambiar estado
            goto_next_page(); // Ir a pág 1 (PAGE_POSITIONING)
            currentState = STATE_POSITIONING;
            break;
        }
        return;
    }
}

// --- FUNCIONES DE NEXTION ---
/**
 * @brief Actualiza el objeto de gravedad en la pantalla Nextion.
 * @param g El valor de 'g' calculado.
 */
void set_grav_nextion(float g)
{
    String cmd;
    cmd = "page4.t0.txt=\"" + String(g, 4) + "\"";
    Serial.print("CMD: ");
    Serial.println(cmd);
    nextionSerial.print(cmd);
    sendNextionEnd();
    Serial.println("Resultado de 'g' enviado a Nextion.");
}

void goto_specific_page(int pageNumber)
{
    Serial.print("Enviando 'page ");
    Serial.print(pageNumber);
    Serial.println("' a Nextion...");
    nextionSerial.print("page ");
    nextionSerial.print(pageNumber);
    sendNextionEnd();
    currentPage = pageNumber;
}

void goto_next_page()
{
    currentPage++;
    if (currentPage >= TOTAL_PAGES)
    {
        currentPage = 0;
    }
    Serial.print("Enviando 'page ");
    Serial.print(currentPage);
    Serial.println("' a Nextion...");
    nextionSerial.print("page ");
    nextionSerial.print(currentPage);
    sendNextionEnd();
}

void goto_error_page()
{
    Serial.println("Enviando a página de error en Nextion...");
    nextionSerial.print("page ");
    nextionSerial.print(PAGE_ERROR);
    sendNextionEnd();
    currentPage = PAGE_ERROR;
}

void sendNextionEnd()
{
    nextionSerial.write(0xFF);
    nextionSerial.write(0xFF);
    nextionSerial.write(0xFF);
    delay(100); // Pausa robusta para Nextion
}

void function_UpdateNextionUI_Angle(int angle)
{
    // Placeholder para actualizar la UI de Nextion si es necesario
    Serial.print("UI Debería actualizarse a ángulo: ");
    Serial.println(angle);
    // ej. nextionSerial.print("page2.t_angulo.txt=\""); ...
    // sendNextionEnd();
}

// --- FUNCIONES DE MEDICIÓN ---

double perform_pendulum_measurement()
{
    double total_time_s, avg_period_T;
    int pass_count = 0;
    bool sensor_state = false; // false = FUERA, true = DENTRO
    bool current_reading = false;

    Serial.println("--- Iniciando Medición de Péndulo ---");

    // 3. Esperar a que el péndulo entre al sensor por primera vez
    unsigned long wait_start = millis();
    
    // CAMBIO DIGITAL: Esperar mientras LOW (libre/no detectado)
    // Esperamos a que el sensor se ponga en HIGH (detecte algo)
    while (digitalRead(PIN_SENSOR_IR) == LOW) 
    {
        if (millis() - wait_start > 20000) // 20s timeout
        {
            Serial.println("Error: Péndulo no detectado. Abortando.");
            return 0.0;
        }
    }

    // 4. Iniciar el cronómetro y contar N periodos (N*2 pases por el sensor)
    unsigned long startTime = micros();
    pass_count = 0;
    sensor_state = true; // Empezamos DENTRO del sensor (acaba de salir del while LOW)
    int passes_to_count = NUM_PERIODS_TO_MEASURE * 2;

    Serial.println("Medición iniciada. Contando pases...");

    Serial.print("Valor inicial del sensor: ");
    Serial.println(digitalRead(PIN_SENSOR_IR)); // Debug Digital

    while (pass_count < passes_to_count)
    {
        // CAMBIO DIGITAL: 'true' (DENTRO) si el valor es HIGH
        current_reading = (digitalRead(PIN_SENSOR_IR) == HIGH);

        if (current_reading != sensor_state)
        {
            if (current_reading == true) // Acaba de entrar al sensor (FUERA -> DENTRO)
            {
                Serial.print("Pase ");
                Serial.println(pass_count + 1);
                pass_count++;
            }
            sensor_state = current_reading; // Actualizar estado
        }

        if (micros() - startTime > 60000000) // 60 segundos (timeout de medición)
        {
            Serial.println("Error: Medición demasiado larga. Abortando.");
            return 0.0;
        }
    }

    // 5. Detener el cronómetro después del último pase
    unsigned long endTime = micros();
    Serial.println("Medición completada.");

    // 6. Calcular y devolver el período promedio
    total_time_s = (endTime - startTime) / 1000000.0;
    avg_period_T = total_time_s / (double)NUM_PERIODS_TO_MEASURE;

    Serial.print("Tiempo total (s): ");
    Serial.println(total_time_s, 6);
    Serial.print("Periodo Promedio (T_med): ");
    Serial.println(avg_period_T, 6);

    return avg_period_T;
}

/**
 * @brief Calcula el factor de corrección para ángulos grandes.
 * @param angle_deg El ángulo de soltura en GRADOS.
 * @return El factor de corrección (ej. 1.0019 para 10 grados).
 */
double calculate_large_angle_correction(int angle_deg)
{
    // Convertir el ángulo de soltura a radianes
    double theta_0 = angle_deg * (M_PI / 180.0);

    // Usar los primeros 3 términos de la serie para precisión
    // T = T_0 * (1 + (1/16)th^2 + (11/3072)th^4 + ...)
    double term1 = 1.0;
    double term2 = (1.0 / 16.0) * pow(theta_0, 2);
    double term3 = (11.0 / 3072.0) * pow(theta_0, 4);

    return term1 + term2 + term3;
}

/**
 * @brief Calcula el período ideal (T_0) para un péndulo de ángulo pequeño.
 * @param length_m La longitud del péndulo en metros.
 * @param gravity_m_s2 El valor de gravedad estándar (ej. 9.80665).
 * @return El período teórico T_0 en segundos.
 */
double calculate_T0_small_angle(double length_m, double gravity_m_s2)
{
    // Fórmula del péndulo simple: T_0 = 2 * PI * sqrt(L / g)
    return 2.0 * M_PI * sqrt(length_m / gravity_m_s2);
}