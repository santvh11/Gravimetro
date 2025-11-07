/*
 * SKETCH DE PRUEBA MÍNIMA PARA ELECTROIMÁN
 *
 * Conecta tu electroimán (o el módulo que lo controla) al pin 24.
 * Este código lo encenderá por 3 segundos y lo apagará por 3 segundos,
 * en un bucle infinito.
 *
 * Abre el Monitor Serial a 115200 baudios para ver los mensajes.
 */

// Define el pin del electroimán (basado en tu código principal)
const int PIN_ELECTROMAGNET = 24;

void setup()
{
    // 1. Iniciar el Monitor Serial
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Esperar a que el puerto serial se conecte
    }

    // 2. Configurar el pin como SALIDA
    pinMode(PIN_ELECTROMAGNET, OUTPUT);

    // 3. Asegurarse de que empiece apagado
    digitalWrite(PIN_ELECTROMAGNET, LOW);

    Serial.println("--- Prueba de Electroimán Iniciada ---");
    Serial.println("El electroimán se encenderá y apagará cada 3 segundos.");
}

void loop()
{
    // Encender el electroimán
    Serial.println("Electroimán: ENCENDIDO (HIGH)");
    digitalWrite(PIN_ELECTROMAGNET, HIGH);
    delay(3000); // Esperar 3 segundos

    // Apagar el electroimán
    Serial.println("Electroimán: APAGADO (LOW)");
    digitalWrite(PIN_ELECTROMAGNET, LOW);
    delay(3000); // Esperar 3 segundos
}