/*
 * SKETCH DE CALIBRACIÓN PARA SENSOR IR (MH-Sensor / FC-51)
 * * Sube este código a tu Arduino Mega para encontrar el valor
 * ideal para tu SENSOR_THRESHOLD.
 * * CONEXIÓN:
 * 1. Conecta el pin VCC del sensor a 5V.
 * 2. Conecta el pin GND del sensor a GND.
 * 3. Conecta el pin A0 (Salida Analógica) del sensor al pin A0 del Arduino.
 *
 * (No necesitas el Host Shield, la Nextion, ni el motor conectados para esto).
 */

// Define el pin que estás usando para la lectura analógica
const int PIN_SENSOR_IR = A0;

void setup()
{
    // Iniciar el Monitor Serial (Asegúrate de ponerlo a 115200)
    Serial.begin(115200);
    while (!Serial)
    {
        ; // Esperar a que el puerto serial se conecte
    }

    // El pin A0 es entrada por defecto, pero podemos ser explícitos.
    pinMode(PIN_SENSOR_IR, INPUT);

    Serial.println("--- Inicio de Calibración del Sensor IR ---");
    Serial.println("Mueve el péndulo (esfera) delante del sensor.");
}

void loop()
{
    // Leer el valor analógico del sensor (0-1023)
    int valorSensor = analogRead(PIN_SENSOR_IR);

    // Imprimir el valor en el Monitor Serial
    Serial.print("Valor del Sensor (A0): ");
    Serial.println(valorSensor);

    // Esperar un poco para no saturar el monitor
    delay(250);
}