#include <I2Cdev.h> // Para comunicación en serie
#include <MPU6050_6Axis_MotionApps20.h> // Para gestión del sensor MPU6050
#include <PID_v1.h>

const byte pinInterrupcion = 2; // El pin que detecta la interrupcion es el 2

MPU6050 mpu; // Instanciamos el sensor

Quaternion tempQuaternion; // Formado por [w, x, y, z]
VectorFloat gravedad; // Gravedad, necesaria para coger pitch, yaw y roll
float ypr[3]; //[yaw, pitch, roll] resultadio de la medición

bool mpuPreparado = false; // Se pone a true si el dmp esta listo 
uint8_t estadoOperacion; // Indica el estado de la última operación del sensor
uint8_t mpuStatus; // Contiene el byte de interrupcion del MPU
uint16_t tamanoPaquete; // Contiene el tamano de paquete de datos que el MPU envia
uint16_t bytesEnCola; // Contador de cuantos bytes estan pendientes en la cola de llegada
uint8_t paqueteLeido[64]; // Aqui se van a volcar los paquetes segun los vayamos leyendo

double entradaInclinacion, salidaVelMotores;

volatile bool interrupcionActivada = false;
// Esta rutina se ejecuta cada vez que llega una interrupcion,
// pone el flag de activacion a true
void rutinaAtencionInterrupcion(){
  interrupcionActivada = true;
}

/******** AJUSTAMOS LOS VALORES **********/
double puntoAjuste = 183;
double Kp = 21;
double Kd = 0.8;
double Ki = 140;
/******** FIN DE AJUSTE DE LOS VALORES *******/

// Creamos el algoritmo PID con los valores que hemos establecido
PID pid(&entradaInclinacion, &salidaVelMotores, &puntoAjuste, Kp, Ki, Kd, DIRECT);
  
void setup() {
  Serial.begin(115200); // Esta es la frecuencia a la que trabaja el sensor
  Serial.println("Iniciando comunicación serie I2C");
  mpu.initialize(); // Iniciamos el sensor MPU6050
  if(mpu.testConnection()==true){ // conectamos el arduino y el sensor
    Serial.println("Conexion MPU-Arduino exitosa"); // Ha funcionado la conexion
  } else {
    Serial.println("Conexion MPU-arduino fallida"); // No ha funcionado, no iniciamos nada
    return; //Paramos el ajuste porque no ha funcionado la conexion
  }

  // Iniciamos el procesador interno del MPU6050,
  // aplica el primer filtro a los datos
  estadoOperacion = mpu.dmpInitialize();

  // Calibramos el giroscopio con MPU6050_calibration
  mpu.setXGyroOffset(31);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(85);
  mpu.setZAccelOffset(1406);
  
  // el estado 0 indica todo bien, otro numero es un codigo de error
  if(estadoOperacion != 0){
    Serial.print("ERROR INICIANDO EL DMP, CODIGO DE ERROR: ");
    Serial.println(estadoOperacion);
    return;
  }
  // Si llegamos aqui es que se ha iniciado el DMp correctamente, seguimos configurando
  Serial.println("Se ha iniciado correctamente el DMP");
  mpu.setDMPEnabled(true); // Tras iniciarlo lo activamos
  
  // Fijamos el pin 2 atendiendo a las interrupciones, se ejecuta la rutina creada en flanco ascendente
  attachInterrupt(digitalPinToInterrupt(pinInterrupcion), rutinaAtencionInterrupcion, RISING);
  mpuStatus = mpu.getIntStatus();

  Serial.println("MPU configurado y listo para enviar interrupciones");
  mpuPreparado = true; // Ponemos el flag a true porque ya esta listo para enviar

  // Tenemos que saber cual es el tamano de los paquetes que manda
  // porque vamos a tener que esperar a que esten completos para leerlos
  tamanoPaquete = mpu.dmpGetFIFOPacketSize();

  // Ajustamos el PID
  pid.SetMode(AUTOMATIC); // Le decimos que opere por su cuenta
  pid.SetSampleTime(10); // Le indicamos la frecuencia a la que va a trabajar
  pid.SetOutputLimits(-255,255); //Para que coincida con los del PWM de los motores

  // Configuramos los pines del motor y lo ponemos parado
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pararMotores();
  
}

void loop() {
  // Si dmp esta a False quiere decir que nos hemos salido antes de configurar
  // porque ha habido un fallo, no ejecutamos el código
  if(!mpuPreparado) return;

  // Si estamos en el bucle es porque no hemos recibido una interrupcion
  // o porque todavia no tenemos un mensaje completo
  while(!interrupcionActivada && bytesEnCola < tamanoPaquete){
    // Mientras no recibamos nuevos paquetes seguimos actualizando el PID
    pid.Compute(); // Calculamos la siguiente salida a motores
    Serial.print(entradaInclinacion);
    Serial.print(" ==> ");
    Serial.print(salidaVelMotores); // Para debugear, así sabemos que esta sacando a motores
    Serial.print(" ==>");
    
    // Si la inclinacion es menor de 130 o mayor de 210 suponemos que
    // el robot esta tumbado y por tanto paramos las ruedas, estamos en espera
    if(entradaInclinacion > 130 && entradaInclinacion < 210){
      if(salidaVelMotores > 0){
        // El PID nos indica que movamos hacia delante, activamos los pines adecuados
        moverAdelante(salidaVelMotores);
        Serial.println("Adelante");
      } else if (salidaVelMotores < 0){
        // El PID nos indica que movamos hacia atras, activamos el otro par de pines
        // Hay que corregir el PWM para que sea positivo
        moverDetras(-1*salidaVelMotores);
        Serial.println("Atras");
      } else{
        pararMotores();
        Serial.println("Parado");
      }
    } else { // Paramos el motor, nos quedamos a la espera
      pararMotores();
      Serial.println("Parado");
    }
  }

  // Si llegamos aqui es porque hemos recibido una interrupcion del MPU,
  // vamos a comprobar la cola
  interrupcionActivada = false; // Volvemos a quedar a la espera de interrupciones
  mpuStatus = mpu.getIntStatus(); // Vemos que interrupcion hemos recibido
  bytesEnCola = mpu.getFIFOCount(); // Vemos cuantos bytes estan pendientes

  // Uno de los bits del byte de estado nos indica que ha habido un error,
  // aplicamos una AND para ver si ese bit esta activo. Por otro lado
  // si tenemos 1024 bytes en la cola quiere decir que ha habido un overflow
  // En ambos casos hay que vaciar la cola
  if ((mpuStatus & 0x10) || bytesEnCola == 1024){
    mpu.resetFIFO();
    Serial.println("Desbordamiento, vamos a limpiar la cola");
  } else if (mpuStatus & 0x02){ // Si tenemos datos que leer
    // Es posible que tengamos datos que leer pero no hayan llegado todos
    // Con este bucle nos quedamos esperando a que lleguen todos
    while(bytesEnCola < tamanoPaquete){
      bytesEnCola = mpu.getFIFOCount();
    }

    // Si hemos llegado aqui quiere decir que tenemos paquetes completos por leer
    // Leemos el primero de ellos
    mpu.getFIFOBytes(paqueteLeido, tamanoPaquete);

    // Actualizamos la cuenta de bytes, quitando el paquete ya leido
    bytesEnCola = bytesEnCola - tamanoPaquete;

    // Ahora vamos a leer los mensajes e imprimirlos para comprobar que los hemos leido bien
    // Primero leemos el quaternion
    mpu.dmpGetQuaternion(&tempQuaternion, paqueteLeido);
    // Cogemos la gravedad a partir del quaternion
    mpu.dmpGetGravity(&gravedad, &tempQuaternion);
    // Finalmente obtenemos el cabeceo, la guiñada y el balanceo
    mpu.dmpGetYawPitchRoll(ypr, &tempQuaternion, &gravedad);

    // Calculamos la inclinacion que va a usar el PID para ajustarse
    entradaInclinacion = ypr[1] * 180/M_PI + 180;
  }
}

// Movemos los motores hacia delante con la velocidad indicada
void moverDetras(int velocidad){
  analogWrite(6, velocidad);
  analogWrite(9,0);
  analogWrite(10, 0);
  analogWrite(11, velocidad);
}

// Movemos los motores hacia atras con la velocidad indicada
void moverAdelante(int velocidad){
  analogWrite(6, 0);
  analogWrite(9, velocidad);
  analogWrite(10, velocidad);
  analogWrite(11, 0);
}

// Paramos el movimiento
void pararMotores(){
  analogWrite(6, 0);
  analogWrite(9,0);
  analogWrite(10, 0);
  analogWrite(11,0);
}
