/* Control de respirador DQ3D NICA
 * Mayo 2020
 * 
 * Arduino Nano 
 * Motor Nema23
 * DQ3D.org
 */

//incluir librerias necesarias para sensores
#include <Wire.h>
#include "i2c.h"
#include "i2c_BMP280.h"
BMP280 bmp280;

//Variables médicas modificables en firmware
int   ciclosmin = 16;     // Minimo de ciclos por minuto
int   ciclosmax = 25;     // Maximo de ciclos por minuto
float FInsp = .42;        // fracción inspiratoria = Tiempo Inspiracion/Tiempo total ciclo (.42 recomendado) SE PUEDE PONER UN POTENCIOMETRO PARA VARIAR
int   TMeseta = 500 ;     // Tiempo de Meseta en milisegundos (Presión alta mantenida antes de pasar a Exhalación)
int   Pausa = 0;          // Duracion de la pausa adicional entre mareas en milisegundos,  
float presionmax = 2942;  // Presion maxima en cualquier momento en Pascal. 30cmH2O     PARA EL EQUIPO.
float presionmin = -490;  // Presion minima durante espiracion en Pascal. -10cmH2O       Señal advertencia
float tempmax = 39;       // Temperatura maxima del aire
                          // volumen segun ambu y posicion del reostato

//Variables medicas derivadas
float CPM = ciclosmin;    // Ciclos por minuto inicio
float TCiclo = 60/CPM;    // duración del ciclo respiratorio (s)

//Definición pines Digitales y Análogos
int alarmaPin = 2;        // Pin para activar alarma
int cierrePin = 3;        // Pin (Switch mecanico) para confirmar cierre de caja. NO NECESARIO si pin faltante, solo led con switch es suficiente
int origenMotPin = 4;     // Pin de sensor optico para confirmar si Motor regresó a origen. No se han saltado pasos (motor en posicion adecuada) despues de un ciclo
int direccionPin = 5;     // Pin direccion Motor 
int pasosPin = 6;         // Pin step pasos Motor
int botonPin = 7;         // Pin pulsador  (encendido ciclo)
int enablePin = 8;        // Pin activa y desactiva motor Motor (Verificar con el driver. Driver HY-DIV268N 6600 activa motor con LOW.
int origenBraPin = 9;     // Pin (endstop) para confirmar brazo en punto de partida
int SPIssPin = 10;        // Reservado para SPI
int SPImosiPin = 11;      // Reservado para SPI
int SPImisoPin = 12;      // Reservado para SPI

int potCPM;               // A0 lectura del potenciometro de velocidad (ciclos por minuto) ,  Leída
int potPI = 0;            // A1 lectura del potenciometro de volumen (pasos por ciclo) ,  Leída 
int SDAPin = 4;           // Reservado para SDA de I2C Data (BMP280 purpura, etc)
int SCLPin = 5;           // Reservado para SCL de I2C Clock (BMP280 amarillo, etc)

int CALIBRA = 1;          // Chequeo antes de iniciar operación (1) y en marcha para busca de punto origen motor (2)
int ALARMA = 0;           // Diferentes alarmas (estado de alarma por falla) generada 
                          /* 0= Normal, 
                             1= falla de motor/origen, 
                             2= Falla de origen del brazo, 
                             3= Exceso de presion 
                             4= Presion demasiado baja
                             5= Sobre temperatura
                             6= Sensor presion no detectado
                             9= Señal de ok
                          */
                          
//Variables para BMP280
float temperature, pascal;  // Crea las variables de temperatura y presion 
float cmH2O;                // Presion en cmH2O
float presionI =0;          // Presion Inicial, Ambiente antes de medición, en hPa
//float temperatura= 0;
//float temperaturaMin = 40;
//float temperaturaMax = 0;
float presion = 0;
float presionMax = 0;
float presionMin = 1500;
int j =0;
int jj =0;

//Variables mecánicas
int PasosInsp = 550;      // Pasos maximos por Inspiración (depende de mecanismo usado), 200pasos por vuelta*Microstep, definida (define volumen)
int MicroStep = 2;        // Micropasos utilizados en el driver para suavizar motor (1/Microstep)
bool dirEspira = false;   // dirEspira = 0 es inspira, dirEspira = 1 es espira (si el motor esta conectado inverso, cambiar)
bool dirMotor=!dirEspira; // Motor siempre empieza con inspiración
int Vueltas = 2;          // para 10 mm diametro, debe dar un maximo de dos vueltas 
bool BrazoHome = false;   // Suponemos que el brazo no esta en posicion correcta
int Tentrepasos = 900 ;   // Tiempo entre pasos para asegurar un numero de ciclos por minuto (Pausa en microsegundos, calculada)
int regresaorigen =1;     // Tipo de regreso a Origen
int ciclosCalibra = 0;        // Control de ciclos para recalibrar
//int optVuel = 0;          // Vueltas completas del motor para llegar al punto maximo inspiracion
//float Trespiro = 0;
//bool Activa = false;      //   Logica (encendida o apagada) ,  Leída


void setup() {                
  
  // inicializamos pines.
  pinMode(alarmaPin, OUTPUT); 
  pinMode(cierrePin, INPUT); 
  pinMode(origenMotPin, INPUT); 
  pinMode(direccionPin, OUTPUT); 
  pinMode(pasosPin, OUTPUT);
  pinMode(botonPin, INPUT);
  pinMode(enablePin, OUTPUT); 

  Serial.begin(250000);

  Serial.print("Probando BMP280: ");
    if (bmp280.initialize()) Serial.println("Sensor bien");
    else    {
        ALARMA = 6;
        alarma();
        while (1) {}
    }

    // Primera medida BMP presion y temperatura:
    bmp280.setEnabled(1);
//    bmp280.triggerMeasurement();
    bmp280.awaitMeasurement();
    bmp280.getTemperature(temperature);
    bmp280.getPressure(pascal);
    
//    while (presionI == 0){
  //    bmp280.getPressure(pascal);
   //   bmp280.getTemperature(temperature);
      presionI = pascal;
//      temperatura = temperature;
      Serial.print(presionI/100);    
      Serial.println(F(" hPa Presión Atmosférica de referencia "));
//      temperatura= temperature;
//      temperaturaMin = temperature;
//      temperaturaMax = temperature;
      presionmax = pascal+presionmax;
      presionmin = pascal+presionmin;
      presion = pascal;
      presionMax = pascal;
      presionMin = pascal;
 //   }
 
  //preflight check
  digitalWrite(enablePin, HIGH);                  // desactiva el driver del motor
  if (!digitalRead(origenMotPin)) RegresaOrigen();
  
Serial.print(" RegresaOrigen terminado. Logica Origen y brazo: ");  
Serial.print(digitalRead(origenMotPin) );
Serial.println(digitalRead(origenBraPin));
   if (!digitalRead(origenBraPin)){
     ALARMA =2;
     alarma();
   }
   CALIBRA = 0;
   TODOBIEN();
}

void chequeoPT() {    //chequear presion y temp
    
//    bmp280.awaitMeasurement();
    bmp280.getPressure(pascal);
//    bmp280.triggerMeasurement();
    if (pascal < presionmin){
      Serial.print(pascal);    
      Serial.print(F("pascal leidos      pascal minimos    "));
      Serial.print(presionmax);
      ALARMA = 4;
      alarma(); 
    }
    if (pascal > presionmax){
      Serial.print(pascal);    
      Serial.print(F("pascal leidos      pascal maximos    "));
      Serial.print(presionmax);
      ALARMA = 3;
      alarma();
    }
    presion = presion +pascal;
    if (presionMin > pascal) presionMin = pascal;
    if (presionMax < pascal) presionMax = pascal; 
}

void alarma() {           //habria que hacerlo como interrupt para advertencias que no paren la maquina...............
       if (ALARMA ==6)     Serial.println("Sensor fallando o ausente");
       if (ALARMA != 2)  digitalWrite(enablePin, HIGH);   // desactiva el driver del motor
    while (ALARMA >0){                       // Alarma sigue hasta reinicio, a menos que haya break (fallas menores)
//      delay(100); // pequeña pausa
      for (int j = 1; j <= 10; j++){         // Suena 10 veces el numero de la alarma
          if (ALARMA ==2) j=10;
          for (int k =1; k<= ALARMA; k++){   // Señal por numero de alarma
            tone(alarmaPin, 1000, 500);
            delay (500);
          }
          if (ALARMA ==2) break; 
          delay (1000);
      }
      if (ALARMA ==2) break; 
      
      tone(alarmaPin, 2000, 500);
      for (int i = 100; i <= 200; i++){       // Emitir sonido sirena problema al final
        tone(alarmaPin, i*4); 
        delay(20);
      }
    }
}

void TODOBIEN () {                //señal que anuncia todo bien
   tone(alarmaPin, 1000, 1000);
   delay (500);
   tone(alarmaPin, 2000, 1500); 
}

void RegresaOrigen(){
  digitalWrite(enablePin, HIGH);                  // desactiva el driver del motor 
  if (CALIBRA == 1){              //Calibracion inicial de punto de origen
    dirMotor = dirEspira;
    for (potPI = 50; potPI < PasosInsp; potPI= potPI+100){        // PasosInsp para limite de pasos
      dirMotor = !dirMotor;
Serial.print(" Valor de pasos a moverse durante calibración y dir: ");  
Serial.println(potPI);
      MueveMotor();
Serial.print(" RegresaOrigen Origen : ");  
Serial.println(digitalRead(origenMotPin) );
      if (digitalRead(origenMotPin) && digitalRead(origenBraPin)){
        Serial.print(" DEBE PASAR POR ACA................................: ");
        potPI = PasosInsp;
        TODOBIEN();
        CALIBRA = 0;
        break;
      }
    }  
    if (!digitalRead(origenMotPin) || !digitalRead(origenBraPin)){ ;
      ALARMA = 1;
      alarma(); 
    }
  }
   
  if (CALIBRA ==2){            //Calibracion sobre la marcha de punto de origen
    dirMotor = !dirEspira; 
    potPI = 1000;        // PasosInsp para recalibrar origen
//Serial.print(" Valor de pasos a moverse durante calibración y dir: ");  
//Serial.println(potPI, dirMotor);
    MueveMotor();
  }
}
 
void MueveMotor() {
  ALARMA = 0;
//      int optLog =0;
  int optVue = 0;
  digitalWrite(enablePin, LOW);         // Activa el driver del motor
  for (int i = 0; i <= potPI; i++) {
    digitalWrite(direccionPin, dirMotor);    // inhala o retrocede
    digitalWrite(pasosPin, HIGH);       // Aqui generamos un flanco de bajada HIGH - LOW menos el tiempo de medicion
    delayMicroseconds(Tentrepasos-400/2);     // Pequeño retardo para formar el pulso en STEP DQ era 10
    digitalWrite(pasosPin, LOW);        // y el driver de avanzara un paso el motor
    delayMicroseconds(Tentrepasos-400/2);     // Retardo para llegar a una velocidad de ciclo menos el tiempo de medicion
//int iniciomedida = micros();
//int duracionmedida = micros() - iniciomedida;
//Serial.print(" duracion de la medida en microseg: ");  
//Serial.println(duracionmedida);        
//Serial.print(" Muevemotor. Logica Origen y brazo: ");  
//Serial.print(digitalRead(origenMotPin) );
//Serial.println(digitalRead(origenBraPin));
    if (CALIBRA >0){
      delay (1);
//if (digitalRead(origenMotPin) && CALIBRA == 2 && optVuel <= optVue){
      if (digitalRead(origenMotPin) && digitalRead(origenBraPin)){
        digitalWrite(enablePin, HIGH);                // desactiva el driver del motor
// Serial.print("Se salta movimientos por haber llegado a origen.");
        i=potPI;                                      //Termina movimientos
      }
    } 
/*    if (digitalRead(origenMotPin) == optLog ){        // Lleva control de las veces que se pasa el punto de origen del motor (vueltas)
      optLog = !optLog;
      if (optLog == 0) optVue = optVue + 1;
//Serial.print("Motor dio x vueltas al sensor, logicaVueltas : " );
//Serial.print(optVuel);
//Serial.println(optLog);
    }
*/
    if (CALIBRA == 0 ){
      chequeoPT();
//    inicio = millis()-inicio ;
//        Serial.print(inicio); 
//        Serial.print(F("millis duracion 100 medidas con print "));
//      inicio = millis();
       if (optVue > Vueltas && dirMotor == !dirEspira && i>= 100) {       // En inspiracion, Parar avance cuando llego a punto de origen (desinflado maximo AMBU), exceso de pasos    
              digitalWrite(enablePin, HIGH);                  // desactiva el driver del motor
              ALARMA = 1;
              break;
        }
     }
  }
  //Serial.println(potPI);
 // Serial.println(jj);
  //Serial.println(presion);
  presion = presion/(potPI+2);
  //Serial.println(presion);
    bmp280.getTemperature(temperature);
  if (temperature > tempmax){
    ALARMA = 5;
    alarma(); 
  }
  cmH2O = ((presion - presionI)/100)*1.019744289 ;     //Pa a cmH2O
  presionMin = ((presionMin - presionI)/100)*1.019744289 ;     //Pa a cmH2O
  presionMax = ((presionMax - presionI)/100)*1.019744289 ;     //Pa a cmH2O
  if (dirMotor != dirEspira) Serial.print("Presiones aspiración   " );    
  if (dirMotor == dirEspira) Serial.print("Presiones espiración   " ); 
  Serial.print(cmH2O); 
  Serial.print(F("cmH2O promedio   "));
  Serial.print(presionMin);    
  Serial.print(F("cmH2O Min    "));
  Serial.print(presionMax);    
  Serial.println(F("cmH2O Max    "));
//    Serial.print(" Pressure: ");
//    Serial.println(pascal/100);
  Serial.print("  T: ");
  Serial.print(temperature);
  Serial.println(" grados C");
  
  //resetea los valores PT del ciclo
    presion = presionI;
    presionMax = presionI;
    presionMin = presionI;
    jj=0;
}

void loop() {

   if (ALARMA != 0) alarma();
   potCPM = map(analogRead(A0),0,1023,ciclosmin*10,ciclosmax*10);  // leemos Ciclos por minuto y adaptamos el valor a un numero de (decimos de) ciclo por minuto, entre X y Y (para  rangos mas precisos)
// Serial.println(potCPM);
   CPM =  potCPM;
   CPM =  CPM/10;
   TCiclo = 60/CPM;      //Ciclo en segundos
// Serial.print("Ciclos por minuto: " );
// Serial.println(CPM);
   potPI = map(analogRead(A1),0,1023,300,PasosInsp);              // leemos numero de pasos para inspiración, lo cual definirá volumen 
// PasosInsp = potPI;
// Serial.print("pasos por ciclo (luego volumen): " );
// Serial.println(potPI);
   Tentrepasos = (((TCiclo*1000000 *FInsp) / potPI)/ MicroStep);  // Tiempo entre pasos en microsegundos dividido por los micropasos
//Serial.print(" Microsegundos entre pasos: ");  
//Serial.println(Tentrepasos);
//Serial.println(digitalRead(origenMotPin));

   if (digitalRead(botonPin) == HIGH){     // leemos el boton de encendido. Funciona solo si encendido (switch)
// ciclo inspiracion   
// Serial.print(" Inicia aspiración    ");
      float Tinicio = millis();            // Marca tiempo de inicio de la respiración
      dirMotor = !dirEspira;
      MueveMotor();
      
      delay (TMeseta);
//      if (ALARMA == 1) RegresaOrigen();// Modificar: si brazo no en origen, o motor no en origen, regresaorigen. si no resuelve: ALARMA
      float Tinspira = millis()-Tinicio;

// ciclo espiracion        
//     int mediodelay = (TCiclo * 1000 - TCiclo * 1000 * FInsp)/2; // generamos un retardo de espiración (manteniendo la presion) en milisegundos POR DEFINIR SI SE USA
      Tentrepasos = 600;   //acelerando espiracion (microsegundos)
      dirMotor = dirEspira;
      MueveMotor();
      
      digitalWrite(enablePin, HIGH);       // desactiva el driver del motor, regresa a origen si AMBU presionado
      int pausita = (TCiclo*1000 - (millis()- Tinicio)) + Pausa;
      if (pausita > 0) delay (pausita);    // Pausa restante para ciclos por minuto establecidos en millis
      ciclosCalibra = ciclosCalibra+1;

      if (ALARMA !=0) alarma();

//cada 5 ciclos confirma y restablece origen
      if (ciclosCalibra == 5 && !digitalRead(origenMotPin)){ 
         CALIBRA = 2;
         RegresaOrigen();  
//  Serial.print("                            Recalibró origen.");
         ciclosCalibra = 0;
         CALIBRA = 0;
      }
    
//cada hora, recalibra contra presion atmosferica
    
    //Reporta datos
     float Trespiro = millis()- Tinicio;      //Duracion respiro 
     Serial.print(" Fin respiro. Duración: ");
     Serial.println(Trespiro/1000); 
     Serial.print(" Ciclos por minuto planteados / efectivos: " );
     Serial.print(CPM);
     Serial.print("\t");
     Serial.println(60/(Trespiro/1000));
     Serial.print(" Fraccion Inspiratoria incluyendo meseta: ");
     Serial.println(Tinspira/Trespiro);
     Serial.print(" Milisegundos de pausa al final de espiracion: ");
     Serial.println(pausita);
     Serial.print(" Minutos desde inicio del respirador:  ");
     Serial.println(millis()/60000);
     Serial.println();
   }

}
