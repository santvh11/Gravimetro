/*
 * SKETCH DE PRUEBA MÍNIMA PARA MEDICIÓN DE GRAVEDAD
 * * Este código prueba ÚNICAMENTE la lógica de medición (sensor + física).
 * NO usa el motor, el electroimán ni la Nextion.
 * * CÓMO USAR:
 * 1. Sostén el péndulo manualmente en el ángulo definido (ej. 15 grados).
 * 2. Presiona la BARRA ESPACIADORA.
 * 3. Suelta el péndulo INMEDIATAMENTE.
 * 4. El programa medirá 10 períodos y mostrará el resultado en el Monitor Serial.
 * 5. Presiona ESPACIO de nuevo para repetir la prueba.
 * * CONEXIONES:
 * - USB Host Shield (para el teclado)
 * - Sensor IR (PIN A0)
 */

// --- LIBRERÍAS DEL USB HOST SHIELD ---
#include <hidboot.h>
#include <usbhub.h>
#include <SPI.h> // Requerido por el Host Shield

// --- OTRAS LIBRERÍAS ---
#include <math.h> // Para M_PI y pow()

// --- ESTADOS DE LA MÁQUINA (Simplificada) ---
enum State
{
    STATE_IDLE,       // Esperando a Iniciar
    STATE_CALCULATING // Soltar el pendulo y calculando g
};
State currentState = STATE_IDLE;

// --- CONFIGURACIÓN DE LA PRUEBA ---
// ¡Define el ángulo (en grados) al que soltarás el péndulo!
const int ANGLE_TO_TEST = 15;

// --- PIN DEL SENSOR ---
const int PIN_SENSOR_IR = A0;

// --- CONSTANTES DE FÍSICA (Copiadas de tu código principal) ---
const int NUM_PERIODS_TO_MEASURE = 10;
const double G_STANDARD = 9.80665;
const double PENDULUM_LENGTH_METERS = 0.1732; // Tu L_eq
int SENSOR_THRESHOLD = 535;                   // Tu umbral calibrado

// --- VARIABLES GLOBALES ---
USB Usb;
HIDBoot<USB_HID_PROTOCOL_KEYBOARD> kbd(&Usb); // Driver para Teclado

class KbdRptParser : public KeyboardReportParser
{
protected:
    void OnKeyDown(uint8_t mod, uint8_t key);
};
KbdRptParser Prs;

// --- PROTOTIPOS ---
double perform_pendulum_measurement();
double calculate_T0_small_angle(double length_m, double gravity_m_s2);
double calculate_large_angle_correction(int angle_deg);

// --- CONFIGURACIÓN ---
void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("--- Inicio de Prueba Mínima de Medición ---");

    pinMode(PIN_SENSOR_IR, INPUT);

    Serial.println("Iniciando USB Host Shield...");
    if (Usb.Init() == -1)
    {
        Serial.println("Error: No se pudo iniciar el USB Host Shield. Deteniendo.");
        while (1)
            ; // Detener todo si el shield falla
    }
    Serial.println("USB Host listo. Conecte un teclado.");

    kbd.SetReportParser(0, &Prs);

    Serial.println("-------------------------------------------------");
    Serial.print("Listo para probar con ángulo = ");
    Serial.print(ANGLE_TO_TEST);
    Serial.println(" grados.");
    Serial.println("Sostén el péndulo en posición y presiona ESPACIO para soltar.");
}

// --- BUCLE PRINCIPAL ---
void loop()
{
    Usb.Task(); // Tarea de fondo esencial para el teclado
    delay(5);   // Pequeña pausa
}

// --- LÓGICA DEL TECLADO ---
void KbdRptParser::OnKeyDown(uint8_t mod, uint8_t key)
{
    // Solo reaccionar si presionamos ESPACIO y estamos en reposo
    if (key == 0x2C /* KEY_SPACE */ && currentState == STATE_IDLE)
    {
        // 1. Cambiar estado para evitar pruebas duplicadas
        currentState = STATE_CALCULATING;
        Serial.println("\n--- NUEVA PRUEBA INICIADA ---");
        Serial.println("Suelte el péndulo AHORA...");

        // 2. Realizar la medición (esta función es bloqueante)
        double T_medido = perform_pendulum_measurement();

        if (T_medido > 0.0) // Si la medición fue exitosa
        {
            // 3. Calcular factor de corrección
            double correction_factor = calculate_large_angle_correction(ANGLE_TO_TEST);

            // 4. Calcular 'g'
            double g_calculado = (4.0 * M_PI * M_PI * PENDULUM_LENGTH_METERS * pow(correction_factor, 2)) / (T_medido * T_medido);

            // 5. Calcular valores teóricos
            double T_0_ideal = calculate_T0_small_angle(PENDULUM_LENGTH_METERS, G_STANDARD);
            double T_esperado = T_0_ideal * correction_factor;

            // 6. Imprimir TODOS los resultados
            Serial.println("\n------ RESULTADOS DE LA PRUEBA ------");
            Serial.print("Periodo Medido (T_med):   ");
            Serial.print(T_medido, 6);
            Serial.println(" s");
            Serial.print("Periodo Esperado (T_esp): ");
            Serial.print(T_esperado, 6);
            Serial.println(" s");
            Serial.print("Periodo Ideal (T_0):    ");
            Serial.print(T_0_ideal, 6);
            Serial.println(" s");
            Serial.print("GRAVEDAD CALCULADA (g): ");
            Serial.print(g_calculado, 6);
            Serial.println(" m/s^2");
            Serial.println("-------------------------------------");
        }
        else
        {
            // La medición falló (timeout)
            Serial.println("------ PRUEBA FALLIDA ------");
            Serial.println("La medición reportó un error (timeout).");
            Serial.println("Asegúrate de que el sensor esté bien calibrado y alineado.");
            Serial.println("-------------------------------------");
        }

        // 7. Volver al estado IDLE para permitir una nueva prueba
        Serial.println("\nPrueba terminada. Presione ESPACIO para una nueva prueba.");
        currentState = STATE_IDLE;
    }
}

// --- FUNCIONES DE MEDICIÓN (Copiadas de tu código principal) ---

double perform_pendulum_measurement()
{
    double total_time_s, avg_period_T;
    int pass_count = 0;
    bool sensor_state = false; // false = FUERA, true = DENTRO
    bool current_reading = false;

    Serial.println("--- (Iniciando Medición...) ---");
    Serial.println("Verificando sensor...");
    unsigned long wait_start = millis();

    // LÓGICA DE SENSOR (ACTIVO-BAJO Confirmada)
    // Esperar a que el valor sea ALTO (despejado)
    while (analogRead(PIN_SENSOR_IR) < SENSOR_THRESHOLD)
    {
        if (millis() - wait_start > 5000) // 5s timeout
        {
            Serial.println("Error: Sensor bloqueado. Abortando.");
            return 0.0;
        }
    }
    Serial.println("Sensor despejado. Esperando primer pase...");

    // Esperar a que el péndulo entre al sensor por primera vez
    wait_start = millis();
    // Esperar a que el valor sea BAJO (bloqueado)
    while (analogRead(PIN_SENSOR_IR) > SENSOR_THRESHOLD)
    {
        if (millis() - wait_start > 5000) // 5s timeout
        {
            Serial.println("Error: Péndulo no detectado. Abortando.");
            return 0.0;
        }
    }

    // 4. Iniciar el cronómetro y contar N periodos (N*2 pases por el sensor)
    Serial.println("¡Primer pase detectado! Iniciando cronómetro...");
    unsigned long startTime = micros();
    pass_count = 0;
    sensor_state = true; // Empezamos DENTRO del sensor
    int passes_to_count = NUM_PERIODS_TO_MEASURE * 2;

    while (pass_count < passes_to_count)
    {
        // 'true' (DENTRO) si el valor es BAJO
        current_reading = (analogRead(PIN_SENSOR_IR) < SENSOR_THRESHOLD);

        if (current_reading != sensor_state)
        {
            if (current_reading == true) // Acaba de entrar al sensor (FUERA -> DENTRO)
            {
                pass_count++;
                Serial.print("Pase: ");
                Serial.println(pass_count);
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

double calculate_large_angle_correction(int angle_deg)
{
    double theta_0 = angle_deg * (M_PI / 180.0);
    double term1 = 1.0;
    double term2 = (1.0 / 16.0) * pow(theta_0, 2);
    double term3 = (11.0 / 3072.0) * pow(theta_0, 4);
    return term1 + term2 + term3;
}

double calculate_T0_small_angle(double length_m, double gravity_m_s2)
{
    return 2.0 * M_PI * sqrt(length_m / gravity_m_s2);
}