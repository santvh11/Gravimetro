/*
 * PROTOTIPO DE CÓDIGO PARA GRAVÍMETRO AUTOMATIZADO (CON PÉNDULO)
 * * Versión con soporte para Teclado USB (Host Shield) y Nextion.
 */

// --- LIBRERÍAS DEL USB HOST SHIELD ---
#include <hidboot.h>
#include <usbhub.h>
#include <SPI.h> // Requerido por el Host Shield

// --- OTRAS LIBRERÍAS ---
#include <math.h>

// --- DEFINICIONES DE NEXTION  ---
#define nextionSerial Serial3 // Usamos Serial3 (Pines 14 y 15)
#define NEXTION_BAUD 9600     // Velocidad confirmada: 9600

// --- ESTADOS DE LA MÁQUINA ---
enum State
{
  STATE_IDLE,            // Esperando a Iniciar
  STATE_POSITIONING,     // Posicionando el pendulo
  STATE_SELECTING_ANGLE, // Esperando a que el usuario seleccione el angulo correspondiente y ejecute
  STATE_CALCULATING,     // Soltar el pendulo y calculando g
  STATE_DISPLAY          // Mostrando resultado en Nextion
};

State currentState = STATE_IDLE;

// --- Definiciones de control del motor ---
enum MotorDirection
{
  MOTOR_STOP,
  MOTOR_FORWARD,
  MOTOR_REVERSE
};

// --- VARIABLES GLOBALES Y PINES ---

// --- a. Obj del host USB ---
USB Usb;
HIDBoot<USB_HID_PROTOCOL_KEYBOARD> kbd(&Usb); // Driver para Teclado

// --- b. Variables de control de página Nextion ---
const int TOTAL_PAGES = 6;
int currentPage = 0;

// --- c. Códigos de Tecla HID ---
#define KEY_SPACE 0x2C // Barra espaciadora
#define KEY_A 0x04     // Tecla 'A'
#define KEY_S 0x16     // Tecla 'S'
#define KEY_D 0x07     // Tecla 'D'

// --- d. Pines ---
const int PIN_MOTOR_DIR1 = 3;
const int PIN_MOTOR_DIR2 = 2;
const int PIN_ELECTROMAGNET = 24;
const int PIN_SENSOR_IR = A0;
// TODO: Ver si es necesario y asignar pin del interruptor de límite
// const int PIN_LIMIT_SWITCH = A1; // Podrías usar A1 y A2 para los sensores azules
// const int PIN_LIMIT_SWITCH_2 = A2;

// --- e.  Constantes medición ---
const int NUM_PERIODS_TO_MEASURE = 10; // Número de oscilaciones a promediar
const double L_eq = -1.0;
// TODO: Ajustar
const double PENDULUM_LENGTH_METERS = 1.0; // AJUSTAR medida (metros)
// TODO: Calibrar
int SENSOR_THRESHOLD = 500; // UMBRAL de detección del sensor IR (CALIBRAR)

// --- f. Variables de medición ---
int selectedAngle = 5;
volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;
volatile int stepCount = 0;
int pasos_objetivo = 0;

// --- g. Variables de cálculo ---
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
void goto_next_page();
void find_pendulm();
void move_motor(int direction);
void center_motor();
void function_UpdateNextionUI_Angle(int angle);
double perform_measurement();
void position_at_angle(int angle);

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
  pinMode(PIN_MOTOR_DIR2, OUTPUT);
  pinMode(PIN_ELECTROMAGNET, OUTPUT);
  pinMode(PIN_SENSOR_IR, INPUT);

  // 4. Asegurarse que todo esté apagado al inicio
  move_motor(MOTOR_STOP);               // Motor en estado STOP por seguridad
  digitalWrite(PIN_ELECTROMAGNET, LOW); // Electroimán apagado

  // 5. Inizializar USB Host Shield
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
  case STATE_IDLE:
    Serial.println("Estado: IDLE (Esperando inicio)...");
    // Esperar... (la lógica del teclado cambia el estado)
    break;

  case STATE_POSITIONING:
    Serial.println("Estado: Posicionando...");
    // 1. Buscar el pendulo moviendo el electroimán de un extremo al otro
    find_pendulm();

    // 2. Centrarlo
    center_motor();

    // 3. Cuando termine, cambiar de estado
    Serial.println("Posicionamiento completado.");
    currentState = STATE_SELECTING_ANGLE;
    goto_next_page(); // Ir a la página de selección de ángulo
    break;

  case STATE_SELECTING_ANGLE:
    Serial.println("Estado: Seleccionando ángulo...");
    // Esperar... (la lógica del teclado cambia el estado)
    break;

  case STATE_CALCULATING:
    Serial.println("Estado: Calculando...");

    // 1. Mover el péndulo a la posición del ángulo seleccionado
    //    (Esto asume que el electroimán ya está encendido desde STATE_IDLE)
    position_at_angle(selectedAngle);

    // 2. Realizar la medición (soltar, medir periodos, calcular g)
    g_calculado = perform_measurement();

    // 3. Enviar el resultado a la pantalla Nextion
    set_grav_nextion(g_calculado);
    currentState = STATE_DISPLAY;

  case STATE_DISPLAY:
    Serial.println("Estado: Mostrando resultado...");
    // ... Lógica para mostrar g_calculado en Nextion ...
    // Esperar... (la lógica del teclado cambia el estado)
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
      // Iniciar el proceso de posicionamiento
      Serial.println("Activando electroimán para posicionar.");
      digitalWrite(PIN_ELECTROMAGNET, HIGH); // Encender el imán ANTES de posicionar
      delay(500);                            // Dar tiempo a que el imán se active
      currentState = STATE_POSITIONING;
      break;

    case STATE_SELECTING_ANGLE:
      // Asumimos que el ángulo está seleccionado y queremos calcular
      if (currentPage == 3) // Si estamos en la página de "listo para soltar"
      {
        goto_next_page(); // Ir a la pág de "calculando"
        currentState = STATE_CALCULATING;
      }
      break;

    case STATE_DISPLAY:
      goto_next_page(); // Volver a la página 0
      currentState = STATE_IDLE;
      break;
    }

    return;
  }

  // --- LÓGICA ESPECIAL PARA PÁGINA DE SELECCIÓN DE ANGULO (se asume que es la 2) ---
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
      Serial.println("Error: Presione A, S o D para seleccionar el ángulo.");
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

  goto_next_page(); // Avanza a la página 3 (listo para soltar)
  // sendNextionEnd(); // goto_next_page ya lo hace
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
  nextionSerial.print("result.txt=\""); // Usar nextionSerial, no Serial3

  // 3. Enviar el texto formateado
  nextionSerial.print(texto_a_enviar);

  sendNextionEnd();
}

void goto_next_page()
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

void find_pendulm()
{
  // --- LÓGICA DE EJEMPLO ---
  Serial.println("Buscando péndulo (ej. moviendo FORWARD 2 seg)...");
  move_motor(MOTOR_FORWARD);
  // TODO: Calcular el tiempo que toma moverse de un extremo al otro y setearlo en el delay.
  delay(2000); // Ejemplo: 2 segundos
  move_motor(MOTOR_STOP);
  Serial.println("...Péndulo encontrado.");
}

/**
 * @brief Controla el motor de DC (velocidad constante).
 * @param direction Un valor de MotorDirection (MOTOR_STOP, MOTOR_FORWARD, MOTOR_REVERSE).
 */
void move_motor(int direction)
{
  switch (direction)
  {
  case MOTOR_FORWARD: // Mover en una dirección
    digitalWrite(PIN_MOTOR_DIR1, HIGH);
    digitalWrite(PIN_MOTOR_DIR2, LOW);
    Serial.println("Motor: FORWARD");
    break;

  case MOTOR_REVERSE: // Mover en la otra dirección
    digitalWrite(PIN_MOTOR_DIR1, LOW);
    digitalWrite(PIN_MOTOR_DIR2, HIGH);
    Serial.println("Motor: REVERSE");
    break;

  case MOTOR_STOP: // Detener (frenar)
  default:
    digitalWrite(PIN_MOTOR_DIR1, LOW);
    digitalWrite(PIN_MOTOR_DIR2, LOW);
    Serial.println("Motor: STOP");
    break;
  }
}

void center_motor()
{
  // --- LÓGICA DE EJEMPLO ---
  // Moverse a la posición central o al otro extremo.
  // REEMPLAZA esto con tu lógica de sensores.
  Serial.println("Centrando motor (ej. moviendo REVERSE 2 seg)...");
  move_motor(MOTOR_REVERSE);
  delay(2000); // Ejemplo: moverse 2 segundos
  move_motor(MOTOR_STOP);
  Serial.println("...Motor centrado (supuestamente).");
}

/**
 * @brief Realiza la medición de caída libre.
 * @return El valor de 'g' calculado en m/s^2.
 */
double perform_measurement()
{
  unsigned long time_ms;
  double time_s;
  double g_result;

  Serial.println("--- Iniciando Medición de Gravedad ---");

  // 1. Esperar a que el sensor esté despejado
  //    Esto previene una falsa lectura si la esfera ya estaba sobre el sensor.
  Serial.println("Verificando sensor...");
  time_ms = millis();
  while (analogRead(PIN_SENSOR_IR) > SENSOR_THRESHOLD)
  {
    // Si la esfera no se quita en 5s, abortar.
    if (millis() - time_ms > 5000)
    {
      Serial.println("Error: Sensor bloqueado. Abortando.");
      return 0.0; // Retorna 0 como error
    }
  }

  Serial.println("Sensor despejado. Listo para soltar.");
  delay(100); // Pequeña pausa

  // 2. Soltar la esfera y tomar el tiempo inicial
  digitalWrite(PIN_ELECTROMAGNET, LOW); // Soltamos el pendulo
  unsigned long startTime = micros();   // Tiempo inicial (microsegundos)
  unsigned long endTime;

  // 3. Esperar a que el sensor detecte la esfera
  //    Nos quedamos en este bucle hasta que la lectura supere el umbral
  while (analogRead(PIN_SENSOR_IR) < SENSOR_THRESHOLD)
  {
    // Opcional: Timeout por si la esfera nunca cae
    if (micros() - startTime > 2000000) // 2 segundos
    {
      Serial.println("Error: Timeout. La esfera no fue detectada.");
      return 0.0;
    }
  }

  // 4. La esfera fue detectada, tomar el tiempo final
  endTime = micros();
  Serial.println("¡Esfera detectada!");

  // 5. Calcular el tiempo y la gravedad
  time_s = (endTime - startTime) / 1000000.0; // Convertir micros a segundos (flotante)
  g_result = (2.0 * FALL_HEIGHT_METERS) / (time_s * time_s);

  Serial.print("Tiempo de caída (s): ");
  Serial.println(time_s, 6); // Imprimir con 6 decimales
  Serial.print("Gravedad calculada (m/s^2): ");
  Serial.println(g_result);
  Serial.println("----------------------------------------");

  return g_result;
}

/**
 * @brief Mueve el motor para posicionar el péndulo en el ángulo deseado.
 * @param angle El ángulo (10, 15, 20) seleccionado por el usuario.
 */
void position_at_angle(int angle)
{
  // Esta es la función donde el motor desde el sensor IR hasta el ángulo seleccionado.
  // SIN SENSORES DE ÁNGULO

  Serial.print("Posicionando motor en ángulo: ");
  Serial.println(angle);

  // Asumimos que el electroimán YA está encendido (desde STATE_IDLE + Spacebar)
  // Lógica: Moverse por un tiempo basado en el ángulo
  // TODO: Calibrar este tiempo según tu sistema
  int time_to_move_ms = angle * 100; // Ej: 100ms por grado desde el centro

  move_motor(MOTOR_FORWARD);
  delay(time_to_move_ms);
  move_motor(MOTOR_STOP);

  Serial.println("Péndulo en posición. Listo para soltar.");
  delay(1000); // Dar tiempo a que el péndulo se estabilice
}

/**
 * @brief Realiza la medición de oscilación del péndulo.
 * @return El valor de 'g' calculado en m/s^2.
 */
double perform_measurement()
{
  double total_time_s, avg_period_T, g_result;
  int pass_count = 0;
  bool sensor_state = false;
  bool current_reading = false;

  Serial.println("--- Iniciando Medición de Péndulo ---");

  // 1. Verificar que el sensor esté despejado (por si acaso)
  Serial.println("Verificando sensor...");
  while (analogRead(PIN_SENSOR_IR) > SENSOR_THRESHOLD)
  {
    // Esperar a que el péndulo no esté en el sensor
  }
  Serial.println("Sensor despejado.");

  // 2. Soltar el péndulo
  digitalWrite(PIN_ELECTROMAGNET, LOW);
  Serial.println("¡Péndulo suelto! Iniciando medición...");

  // 3. Esperar a que el péndulo entre al sensor por primera vez
  while (analogRead(PIN_SENSOR_IR) < SENSOR_THRESHOLD)
  {
    // Esperar al primer pase
  }

  // 4. Iniciar el cronómetro y contar N periodos (N*2 pases por el sensor)
  unsigned long startTime = micros();
  pass_count = 0;
  sensor_state = true; // Empezamos DENTRO del sensor

  // Contamos N periodos, lo que significa N*2 pases por el sensor
  int passes_to_count = NUM_PERIODS_TO_MEASURE * 2;

  while (pass_count < passes_to_count)
  {
    current_reading = (analogRead(PIN_SENSOR_IR) > SENSOR_THRESHOLD);

    if (current_reading != sensor_state) // Si el estado cambió
    {
      if (current_reading == true) // Acaba de entrar al sensor (LOW -> HIGH)
      {
        pass_count++;
        Serial.print("Pase: ");
        Serial.println(pass_count);
      }
      sensor_state = current_reading; // Actualizar el estado
    }
    // Pequeña pausa para evitar "rebotar" en el sensor
    delayMicroseconds(500);
  }

  // 5. Detener el cronómetro después del último pase
  unsigned long endTime = micros();
  Serial.println("Medición completada.");

  // 6. Calcular el tiempo y la gravedad
  total_time_s = (endTime - startTime) / 1000000.0;
  avg_period_T = total_time_s / (double)NUM_PERIODS_TO_MEASURE; // Tiempo total / N oscilaciones
  g_result = (4.0 * M_PI * M_PI * PENDULUM_LENGTH_METERS) / (avg_period_T * avg_period_T);

  Serial.print("Tiempo total (s): ");
  Serial.println(total_time_s, 6);
  Serial.print("Periodo Promedio (T) (s): ");
  Serial.println(avg_period_T, 6);
  Serial.print("Gravedad calculada (m/s^2): ");
  Serial.println(g_result);
  Serial.println("----------------------------------------");

  return g_result;
}
