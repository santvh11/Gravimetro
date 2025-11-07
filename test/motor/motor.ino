/*
 * SKETCH DE PRUEBA MÍNIMA PARA MOTOR DC
 *
 * Conecta tu driver de motor (L298N, etc.) a los pines 2 y 3.
 * Este código moverá el motor en ambas direcciones.
 *
 * Abre el Monitor Serial a 115200 baudios para ver los mensajes.
 */

// Define los pines del motor (basado en tu código principal)
const int PIN_MOTOR_DIR1 = 3;
const int PIN_MOTOR_DIR2 = 2;

// --- Definiciones de control del motor (copiadas de tu código) ---
enum MotorDirection
{
    MOTOR_STOP,
    MOTOR_FORWARD,
    MOTOR_REVERSE
};

void setup()
{
    // 1. Iniciar el Monitor Serial
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Esperar a que el puerto serial se conecte
    }

    // 2. Configurar los pines como SALIDA
    pinMode(PIN_MOTOR_DIR1, OUTPUT);
    pinMode(PIN_MOTOR_DIR2, OUTPUT);

    // 3. Asegurarse de que empiece apagado
    move_motor(MOTOR_STOP);

    Serial.println("--- Prueba de Motor DC Iniciada ---");
    Serial.println("El motor se moverá en ambas direcciones.");
}

void loop()
{
    // Mover en una dirección
    Serial.println("Moviendo: FORWARD");
    move_motor(MOTOR_FORWARD);
    delay(3000); // Mover por 3 segundos

    // Detener
    Serial.println("Moviendo: STOP");
    move_motor(MOTOR_STOP);
    delay(1000); // Esperar 1 segundo

    // Mover en la otra dirección
    Serial.println("Moviendo: REVERSE");
    move_motor(MOTOR_REVERSE);
    delay(3000); // Mover por 3 segundos

    // Detener
    Serial.println("Moviendo: STOP");
    move_motor(MOTOR_STOP);
    delay(1000); // Esperar 1 segundo
}

/**
 * @brief Controla el motor de DC (copiado de tu código).
 * @param direction Un valor de MotorDirection.
 */
void move_motor(int direction)
{
    switch (direction)
    {
    case MOTOR_FORWARD:
        digitalWrite(PIN_MOTOR_DIR1, HIGH);
        digitalWrite(PIN_MOTOR_DIR2, LOW);
        break;
    case MOTOR_REVERSE:
        digitalWrite(PIN_MOTOR_DIR1, LOW);
        digitalWrite(PIN_MOTOR_DIR2, HIGH);
        break;
    case MOTOR_STOP:
    default:
        digitalWrite(PIN_MOTOR_DIR1, LOW);
        digitalWrite(PIN_MOTOR_DIR2, LOW);
        break;
    }
}