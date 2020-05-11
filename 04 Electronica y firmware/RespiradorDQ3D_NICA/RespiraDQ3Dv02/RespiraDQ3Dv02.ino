/* Control de respirador DQ3D 
 * Arduino Nano 
 * Motor Nema23
 * DQ3D.org
 */

//Definición pines Digitales y Análogos
int alarmaPin = 2;        // Pin para activar alarma
int cierrePin = 3;        // Pin (Switch mecanico) para confirmar cierre de caja. NO NECESARIO si pin faltante, solo led con switch es suficiente
int origenPin = 4;        // Pin de sensor optico para confirmar si Motor regresó a origen. No se han saltado pasos (motor en posicion adecuada) despues de un ciclo
int direccionPin = 5;     // Pin direccion Motor 
int pasosPin = 6;         // Pin step pasos Motor
int botonPin = 7;         // Pin pulsador  (encendido ciclo)
int enablePin = 8;        // Pin activa y desactiva motor Motor 
int brazoPin = 9;         // Pin (endstop) para confirmar brazo en punto de partida
int SPIssPin = 10;        // Reservado para SPI
int SPImosiPin = 11;      // Reservado para SPI
int SPImisoPin = 12;      // Reservado para SPI

int potCPM;               // A0 lectura del potenciometro de velocidad (ciclos por minuto) ,  Leída
int potPI = 0;            // A1 lectura del potenciometro de volumen (pasos por ciclo) ,  Leída 
int SDAPin = 4;           // Reservado para SDA de I2C Data 
int SCLPin = 5;           // Reservado para SCL de I2C Clock

int PREFLIGHT = 1;        // Chequeo antes de iniciar operación
bool ALARMA = false;      // Logica (estado de alarma por falla) generada
  
//bool Activa = false;      //   Logica (encendida o apagada) ,  Leída

//Variables mecánicas
int PasosInsp = 600;      // Pasos maximos por Inspiración (depende de mecanismo usado), 200pasos per turn, definida (define volumen)
int MicroStep = 2;        // Micropasos utilizados en el driver para suavizar motor (1/Microstep)
bool dirEspira = false;    // dirEspira = 0 es inspira, dirEspira = 1 es espira (si el motor esta conectado inverso, cambiar)
int Vueltas = 2;          // para 10 mm diametro, debe dar un maximo de dos vueltas 
bool BrazoHome = false;   // Suponemos que el brazo no esta en posicion correcta

//Variables médicas
int ciclosmin = 16;       // Minimo de ciclos por minuto
int ciclosmax = 25;       // Maximo de ciclos por minuto
float CPM =16.0;          // Ciclos por minuto
float TCiclo = 60/CPM;    // duración del ciclo respiratorio (s)
float FInsp = .42;        // fracción inspiratoria = Tiempo Inspiracion/Tiempo total ciclo .42 recomendado SE PUEDE PONER UN POTENCIOMETRO PARA VARIAR
//int Pausa = 0;          // Duracion de la pausa entre mareas en milisegundos,  MODIFICABLE
int Tentrepasos = 900 ;   // Tiempo entre pasos para asegurar un numero de ciclos por minuto (Pausa en milisegundos, calculada)
int TMeseta = 200 ;       // Tiempo de Meseta en milisegundos (Presión alta mantenida antes de pasar a Exhalación)

//float Tinicio = millis();
//float Trespiro = 0;

void setup() {                
  
  // inicializamos pines.
  pinMode(alarmaPin, OUTPUT); 
  pinMode(cierrePin, INPUT); 
  pinMode(origenPin, INPUT); 
  pinMode(direccionPin, OUTPUT); 
  pinMode(pasosPin, OUTPUT);
  pinMode(botonPin, INPUT);
  pinMode(enablePin, OUTPUT); 

//  unsigned long TInicioVent = millis();

  Serial.begin(250000);

//preflight check
  RegresaOrigen();
  PREFLIGHT =0;
  tone(alarmaPin, 1000, 1000);
  delay (500);
  tone(alarmaPin, 2000, 1500);

}

void chequeo() {    //chequear temp y humedad
  
}

void alarma() {
  //  for (int j = 1; j <= 10; j++){
    for (int i = 100; i <= 200; i++){
      tone(alarmaPin, i*4); // Emit the noise
      delay(20);
    }
    delay(100); // A short break in between each whoop
    tone(alarmaPin, 1000, 500);
    delay (1000);
    tone(alarmaPin, 2000, 500);
//  }
}

void RegresaOrigen(){
  potPI = 100;          // PasosInsp para sacar de punto muerto
  dirEspira = !dirEspira;
  MueveMotor();
  potPI = 1000;          // PasosInsp para regresar a punto origen)
  dirEspira = !dirEspira;
  MueveMotor();
  potPI = 10;          // PasosInsp para sacar de punto muerto
  dirEspira = !dirEspira;
  MueveMotor();
  dirEspira = !dirEspira;
  Serial.print(" RegresaOrigen terminado. Logica Origen y brazo: ");  
Serial.print(digitalRead(cierrePin) );
Serial.println(digitalRead(brazoPin));
if (!digitalRead(brazoPin))alarma();
  tone(alarmaPin, 1000, 100);
}

void MueveMotor() {
      ALARMA = false;
      int optLog =0;
      int optVuel = 0;
      digitalWrite(enablePin, LOW);         // Activa el driver del motor
      for (int i = 0; i <= potPI; i++) {
        digitalWrite(direccionPin, dirEspira);    // inhala o retrocede
        digitalWrite(pasosPin, HIGH);       // Aqui generamos un flanco de bajada HIGH - LOW
        delayMicroseconds(Tentrepasos);     // Pequeño retardo para formar el pulso en STEP DQ era 10
        digitalWrite(pasosPin, LOW);        // y el driver de avanzara un paso el motor
        delayMicroseconds(Tentrepasos);     // Retardo para llegar a una velocidad de ciclo
        if (digitalRead(origenPin)!= optLog){
          optLog = !optLog;
          if (optLog = 0) optVuel = optVuel + 1;
        }
        if (digitalRead(origenPin)){
          if (PREFLIGHT == 1){
            digitalWrite(enablePin, HIGH);                  // desactiva el driver del motor
            break;       
          }
/*          if (PREFLIGHT == 0 && optVuel > Vueltas && dirEspira == 1 && i>= 100) {       // En inspiracion, Parar avance cuando llego a punto de origen (desinflado maximo AMBU), exceso de pasos    
            digitalWrite(enablePin, HIGH);                  // desactiva el driver del motor
            ALARMA = true;
            break;
          }*/
       } 
    }
    if (dirEspira) delay (TMeseta);
    digitalWrite(enablePin, HIGH);                  // desactiva el driver del motor
}

void loop() {

   potCPM = map(analogRead(A0),0,1023,ciclosmin*10,ciclosmax*10);  // leemos Ciclos por minuto y adaptamos el valor a un numero de (decimos de) ciclo por minuto, entre X y Y (para  rangos mas precisos)
// Serial.println(potCPM);
    CPM =  potCPM;
    CPM =  CPM/10;
    TCiclo = 60/CPM;      //Ciclo en segundos
 Serial.print("Ciclos por minuto: " );
 Serial.println(CPM);
   potPI = map(analogRead(A1),0,1023,150,PasosInsp);              // leemos numero de pasos para inspiración, lo cual definirá volumen 
// PasosInsp = potPI;
// Serial.print("pasos por ciclo (luego volumen): " );
// Serial.println(potPI);
    Tentrepasos = (((TCiclo*1000000 *FInsp) / potPI)/ MicroStep) ;     // Tiempo entre pasos en microsegundos dividido por los micropasos
//    Tentrepasos = Tentrepasos * .49;
Serial.print(" Microsegundos entre pasos: ");  
Serial.println(Tentrepasos);
 Serial.println(digitalRead(origenPin));
 
    if (digitalRead(botonPin) == HIGH){     // leemos el boton de encendido. Funciona solo si encendido (switch)
//Serial.print(" Inicia aspiración    ");
      float Tinicio = millis();                   // Marca tiempo de inicio de la respiración
      dirEspira = !dirEspira;
      MueveMotor();
      if (ALARMA) RegresaOrigen();
      float Tinspira = millis()-Tinicio;
 // ciclo espiracion        
 //     int mediodelay = (TCiclo * 1000 - TCiclo * 1000 * FInsp)/2; // generamos un retardo de espiración (manteniendo la presion) en milisegundos POR DEFINIR SI SE USA
 //     delay(mediodelay);
      dirEspira = !dirEspira;
      MueveMotor();
      delay ((60/CPM)*1000 - (millis()- (Tinicio + TMeseta)));    // Pausa restante para ciclos por minuto establecidos
//Reporta datos
     float Trespiro = millis()- Tinicio;      //Duracion respiro 
     Serial.print(" Fin respiro. Duración: ");
     Serial.println(Trespiro/1000); 
     Serial.print(" Fraccion Inspiratoria: ");
     Serial.println(Tinspira/Trespiro);
     Serial.print(" Minutos desde inicio del respirador:  ");
     Serial.println(millis()/60000);
     Serial.println();
   }

}
