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
#include "i2c_BMP280.h"     //manejador de sensor I2C 1
BMP280 bmp280;
#include "i2c_BMP2802.h"    //manejador de sensor I2C 2
BMP2802 bmp2802;
#include "MegunoLink.h"     //manejador de graficas seriales
TimePlot MyPlot;            //no channel selected

//Variables médicas modificables en firmware
int   ciclosmin = 16;     // Minimo de ciclos por minuto
int   ciclosmax = 25;     // Maximo de ciclos por minuto
float FInsp = .42;        // fracción inspiratoria = Tiempo Inspiracion/Tiempo total ciclo (.42 recomendado) SE PUEDE PONER UN POTENCIOMETRO PARA VARIAR
unsigned long TMeseta = 600 ;     // Tiempo de Meseta en milisegundos (Max600)(Presión alta mantenida antes de pasar a Exhalación)
unsigned long Pausa = 0;          // Duracion de la pausa adicional entre mareas en milisegundos,  
float presionmax = 2942;  // Presion maxima en cualquier momento en Pascal. 30cmH2O     PARA EL EQUIPO.
float presionmin = -490;  // Presion minima durante espiracion en Pascal. -5cmH2O       Señal advertencia
float tempmax = 39;       // Temperatura maxima del aire
                          // volumen segun ambu y posicion del reostato

//Variables medicas derivadas
float CPM = ciclosmin;    // Ciclos por minuto inicio
float TCiclo = 60/CPM;    // duración del ciclo respiratorio (s)

//Definición pines Digitales y Análogos
const int ResetAlarmaPin = 2;   // (Interrupt) Boton reset alarma
const int cierrePin = 3;        // (Interrupt) Pin libre
const int origenMotPin = 4;     // Pin de sensor optico para confirmar si Motor regresó a origen. No se han saltado pasos (motor en posicion adecuada) despues de un ciclo
const int direccionPin = 5;     // Pin direccion Motor 
const int pasosPin = 6;         // Pin step pasos Motor
const int botonPin = 7;         // Pin pulsador  (encendido ciclo)
const int enablePin = 8;        // Pin activa y desactiva motor Motor (Verificar con el driver. Driver HY-DIV268N 6600 activa motor con LOW.
const int origenBraPin = 9;     // Pin (endstop) para confirmar brazo en punto de partida
const int SPIssPin = 10;        // Reservado para SPI
const int SPImosiPin = 11;      // Reservado para SPI
const int SPImisoPin = 12;      // Reservado para SPI
const int alarmaPin = 13;       // Boton reset alarma
//(Switch mecanico) para confirmar cierre de caja. SIN pin, solo led con switch es suficiente

int potCPM;                     // A0 lectura del potenciometro de velocidad (ciclos por minuto) ,  Leída
int potPI = 0;                  // A1 lectura del potenciometro de volumen (pasos por ciclo) ,  Leída 
const int SDAPin = 4;           // Reservado para SDA de I2C Data (BMP280 purpura, etc)
const int SCLPin = 5;           // Reservado para SCL de I2C Clock (BMP280 amarillo, etc)

int CALIBRA = 1;                // Chequeo antes de iniciar operación (1) y en marcha para busca de punto origen motor (2)
unsigned long Tinicio = 0;      // Tiempo de inicio de ciclo en milisegundos, 
unsigned long TultimaAlarma =0; // cuando se paso por la alarma
unsigned long TultimaSerieAlarma = 0; // Tiempo de inicio de la serie
unsigned long tiempoAlarma = 500;     // Tiempo del pitido de alarma en milisegundos
int pitosAlarma = 0;            // Numero de pitos tocados por alarma
bool alarmaON = false;
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
volatile int ResetAlarma = 0;         // variable for reading the pushbutton status
                          
//Variables para BMP280
float temperature, pascal;  // Crea las variables de temperatura y presion 
float cmH2O;                // Presion en cmH2O
float presionI1 =0;         // Presion Inicial, Ambiente antes de medición, en hPa
float presionI2 =0;         // Presion Inicial, Ambiente antes de medición, en hPa
float presionMax1 = 0;      //  en cmH2O
float presionMin1 = 0;      //  en cmH2O
float presionMax2 = 0;      //  en cmH2O
float presionMin2 = 0;      //  en cmH2O
int j =0;
unsigned long TCheqPT = 100;// Tiempo en  millis entre chequeos de Presion
unsigned long TultmedPT =0; // Tiempo transcurrido desde ultima medida en  millis 

//Variables mecánicas
const int PasosInsp = 550;  // Pasos maximos por Inspiración (depende de mecanismo usado), 200pasos por vuelta*Microstep, definida (define volumen)
const int MicroStep = 2;    // Micropasos utilizados en el driver para suavizar motor (1/Microstep)
const bool dirEspira = false;   // dirEspira = 0 es inspira, dirEspira = 1 es espira (si el motor esta conectado inverso, cambiar)
bool dirMotor=!dirEspira;   // Motor siempre empieza con inspiración
const int Vueltas = 2;      // para 10 mm diametro, debe dar un maximo de dos vueltas 
bool BrazoHome = false;     // Suponemos que el brazo no esta en posicion correcta
int Tentrepasos = 900 ;     // Tiempo entre pasos para asegurar un numero de ciclos por minuto (Pausa en microsegundos, calculada)
int regresaorigen = 1;      // Tipo de regreso a Origen
//int optVuel = 0;          // Vueltas completas del motor para llegar al punto maximo inspiracion
//float Trespiro = 0;
//bool Activa = false;      //   Logica (encendida o apagada) ,  Leída

// variables provisionales para grafica MegunoLink
int ipagra = 0;
int inpagra =10;    //solo envia señal cada 10 medidas
float iprompagra = 0;
float iprompagra2 = 0;

void setup() {                
  
  // inicializamos pines.
  pinMode(alarmaPin, OUTPUT); 
  pinMode(cierrePin, INPUT); 
  pinMode(origenMotPin, INPUT); 
  pinMode(direccionPin, OUTPUT); 
  pinMode(pasosPin, OUTPUT);
  pinMode(botonPin, INPUT);
  pinMode(enablePin, OUTPUT); 
  pinMode(ResetAlarmaPin, INPUT_PULLUP);      // initialize the pushbutton pin as an input:
  attachInterrupt(0, pin_ISR, FALLING);       // Attach an interrupt to the ISR vector
  
  Serial.begin(230400);

//   MegunoLink      //manejador de graficas seriales
  MyPlot.SetTitle("Presiones en tuberia");
  MyPlot.SetXlabel("Tiempo (min)");
  MyPlot.SetYlabel("Presion (cmH2O)");
  MyPlot.SetSeriesProperties("Distal", Plot::Blue, Plot::Solid, 1, Plot::NoMarker);
  MyPlot.SetSeriesProperties("Proximal", Plot::Red, Plot::Solid, 1, Plot::NoMarker);

  
  Serial.print("Probando BMP280: ");
  //...................................................Verificar si usando BMP solo cuando se necesita.......
  if (bmp280.initialize()) Serial.println("Sensor1 bien");
  else    {
      ALARMA = 6;
      alarma();
  //    while (1) {}
  }
// Primera medida BMP presion sensor 1:
  bmp280.setEnabled(1);
  bmp280.awaitMeasurement();
  bmp280.getTemperature(temperature);
  bmp280.getPressure(pascal);
  presionI1 = pascal;
  Serial.print(presionI1/100);    
  Serial.println(F(" hPa Presión Atmosférica de referencia 1 "));
  presionmax = pascal+presionmax;
  presionmin = pascal+presionmin;
  presionMax1 = ((pascal- presionI1)/100)*1.019744289 ;
  presionMin1 = presionMax1 ;
 /*    Serial.print(F("pascales "));
     Serial.println(pascal);   
     Serial.print(F("Presión DISTAL  Máxima: "));
     Serial.print(presionMax1);       
     Serial.print(F("  Minima:"));
     Serial.println(presionMin1); 
*/
// Primera medida BMP presion sensor 2:
  if (bmp2802.initialize()){
    Serial.println("Sensor2 bien");
    bmp2802.setEnabled(1);
    bmp2802.awaitMeasurement();
    bmp2802.getTemperature(temperature);
    bmp2802.getPressure(pascal);
    presionI2 = pascal;
  //      Serial.print(presionI2/100);    
  //      Serial.println(F(" hPa Presión Atmosférica de referencia 2 "));
    presionMax2 = ((pascal- presionI1)/100)*1.019744289 ;
    presionMin2 = presionMax2;
  }  
  else    {
    Serial.println(F(" Sensor 2 Ausente "));
  }

//preflight check
//  Tinicio = millis(); 
  digitalWrite(enablePin, HIGH);                      // desactiva el driver del motor
  if (!digitalRead(origenMotPin)) RegresaOrigen();
Serial.print(" RegresaOrigen terminado. Logica Origen y brazo: ");  
Serial.print(digitalRead(origenMotPin) );
Serial.println(digitalRead(origenBraPin));
  if (!digitalRead(origenBraPin)){
    ALARMA =2;
    alarma();
  }
  CALIBRA = 0;
  TODOBIEN();   // Problema si alarma encendida...................................................
}

void chequeoPT() {                                    //chequear presion Sensores 1 y 2
    bmp280.getPressure(pascal);
    if (pascal < presionmin){
      Serial.print(pascal);    
      Serial.print(F("DISTAL pascal leidos      pascal minimos    "));
      Serial.print(presionmin);
      ALARMA = 4;
      alarma(); 
    }
    if (pascal > presionmax){
      Serial.print(pascal);    
      Serial.print(F("DISTAL pascal leidos      pascal maximos    "));
      Serial.print(presionmax);
      ALARMA = 3;
      alarma();
    }
    float pascalH2O= ((pascal- presionI1)/100)*1.019744289;    // lo convierte en cmH2O
//    MyPlot.SendFloatData(F("Distal"),pascal, 1);    // Distal = nombre de la serie, pascal  = data to plot
    if (presionMin1 > pascalH2O) presionMin1 = pascalH2O;   //  en cmH2O
    if (presionMax1 < pascalH2O) presionMax1 = pascalH2O;   //  en cmH2O

         
/*    bmp2802.getPressure(pascal);                      //lectura del sensor 2 (77)
    if (pascal < presionmin){
      Serial.print(pascal);    
      Serial.print(F("PROXIMAL pascal leidos      pascal minimos    "));
      Serial.print(presionmin);
      ALARMA = 4;
      alarma(); 
    }
    if (pascal > presionmax){
      Serial.print(pascal);    
      Serial.print(F("PROXIMAL pascal leidos      pascal maximos    "));
      Serial.print(presionmax);
      ALARMA = 3;
      alarma();
    }
    pascalH2O= ((pascal - presionI2)/100)*1.019744289;     // lo convierte en cmH2O
//    MyPlot.SendFloatData(F("Proximal"),pascal, 1);    // Proximal = nombre de la serie, pascal  = data to plot
    if (presionMin2 > pascalH2O) presionMin2 = pascalH2O;
    if (presionMax2 < pascalH2O) presionMax2 = pascalH2O;
*/    
  TultmedPT = millis();
}

void alarma() {           
  if  (ResetAlarma ==1){
    ResetAlarma = 0;
    ALARMA =0;
    noTone(alarmaPin);  
  }
  
  if (ALARMA >0){                 // Alarma sigue hasta reinicio, a menos que haya break (fallas menores)
    if (ALARMA ==6)     Serial.println("Sensor fallando o ausente");
    if (!alarmaON){
      if(pitosAlarma < ALARMA){   // Señal por numero de alarma
        tone(alarmaPin, 1000);    // encender por tiempoAlarma milis
        TultimaAlarma = millis();
        alarmaON = true;
      }
      //iniciar pausa entre serie de pitidos
      else if (pitosAlarma >= ALARMA && millis()-TultimaSerieAlarma >= (tiempoAlarma+200)*(ALARMA-1) + 1000){  
        alarmaON = false;
        pitosAlarma = 0;
        TultimaSerieAlarma =millis();;
      }
    }
    
    if (alarmaON){ 
      if( millis()-TultimaAlarma >= tiempoAlarma){          // para el pitido por cumplir tiempo
        noTone(alarmaPin);
        if (millis()-TultimaAlarma >= tiempoAlarma + 200){  // 200 de pausa entre pitidos
          pitosAlarma = pitosAlarma +1;
          alarmaON = false;
        }
      }
    }
  }

} 

void TODOBIEN () {                        //señal que anuncia todo bien
   tone(alarmaPin, 1000, 1000);
   delay (500);
   tone(alarmaPin, 2000, 1500); 
}

void TODOMAL(){                           // Sirena cuando para definitivo
    while(1){
      for (int i = 100; i <= 200; i++){   // Emitir sonido sirena problema al final
        tone(alarmaPin, i*4); 
        delay(20);
      }
      delay (1000);
    }
}

void RegresaOrigen(){
  digitalWrite(enablePin, HIGH);               // desactiva el driver del motor 
  
  if (CALIBRA ==2){               // Calibracion sobre la marcha de punto de origen, llamada por motor fuera de punto
    Tentrepasos = 600;            // Maxima aceleracion
    dirMotor = !dirEspira;        // Primera calibracion hacia Inspira (suponemos que salto pasos durante inspira
    potPI = 1000;                 // PasosInsp para recalibrar origen
    MueveMotor();
    if (digitalRead(origenMotPin)){            // Si motor regreso a origen
      if (digitalRead(origenBraPin)){   //Todo bien
//      Serial.print(" Recalibró en operación................................: ");
        potPI = PasosInsp;
        CALIBRA = 0;
//        break;
      }
      else if (!digitalRead(origenBraPin)){        // Si brazo no llego a origen, Suponemos que se paso una vuelta, y retrocedemos 1 vuelta
            dirMotor = dirEspira;
            potPI = 200*MicroStep;
            MueveMotor();   
            if (!digitalRead(origenBraPin)) TODOMAL();       
      }
    }
    else TODOMAL();       //No regresó a origen de motor, algo anda mal. Paraliza la maquina. Reiniciar
  }
  else if (CALIBRA == 1){                              // Calibracion inicial de punto de origen
    dirMotor = dirEspira;
    for (potPI = 50; potPI < PasosInsp; potPI= potPI+50){        // PasosInsp para limite de pasos
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
      }
    }  
    if (!digitalRead(origenMotPin) || !digitalRead(origenBraPin)){ ;
      ALARMA = 1;
      alarma(); 
    }
  }
}

void MueveMotor() {
  int optLog =0;
  int optVue = 0;
  digitalWrite(enablePin, LOW);         // Activa el driver del motor
  for (int i = 0; i <= potPI; i++) {    // BUCLE DE PASOS
    digitalWrite(direccionPin, dirMotor);    // inhala o retrocede
    digitalWrite(pasosPin, HIGH);       // Aqui generamos un flanco de bajada HIGH - LOW menos el tiempo de medicion
    delayMicroseconds(Tentrepasos);     // Pequeño retardo para formar el pulso en STEP DQ era 10
    digitalWrite(pasosPin, LOW);        // y el driver de avanzara un paso el motor
    delayMicroseconds(Tentrepasos);     // Retardo para llegar a una velocidad de ciclo menos el tiempo de medicion
    
    if (digitalRead(origenMotPin) == optLog ){                      // Lleva control de las veces que se pasa el punto de origen del motor (vueltas)
      optLog = !optLog;
      if (optLog == 0) optVue = optVue + 1;
    }
 
    if (CALIBRA == 0 ){                                            // Durante operacion normal
      if (millis() - TultmedPT > TCheqPT) chequeoPT();
      if (ALARMA != 0) alarma();
      if (optVue > Vueltas && dirMotor == !dirEspira && i>= 100) { // En inspiracion, Parar avance cuando llego a punto de origen (desinflado maximo AMBU), exceso de pasos    PROBABLEMENTE INNECESARIO
        digitalWrite(enablePin, HIGH);                             // desactiva el driver del motor
        ALARMA = 1;
        break;
      } 
    }
    else if (CALIBRA >0){                                           // Durante calibracion punto origen, para salir en cuanto llega a origen
//    delay (1);
      if (digitalRead(origenMotPin) && digitalRead(origenBraPin)){  // Si ambos puntos de origen OK
        digitalWrite(enablePin, HIGH);                              // desactiva el driver del motor
        i=potPI;                                                    // Termina movimientos saliendo del bucle de pasos
      }
    }    
  }                                     // FIN BUCLE DE PASOS
  
  bmp280.getTemperature(temperature);    
  if (temperature > tempmax){
    ALARMA = 5;
    alarma(); 
  }

}

void loop() {
   Tinicio = millis();                  // Marca tiempo de inicio de la respiración
   if (ALARMA != 0)alarma();
   
   potCPM = map(analogRead(A0),0,1023,ciclosmin*10,ciclosmax*10); // leemos Ciclos por minuto y adaptamos el valor a un numero de (decimos de) ciclo por minuto, entre min y max (para  rangos mas precisos)
   CPM =  potCPM;
   CPM =  CPM/10;
   TCiclo = 60/CPM;                                               // Ciclo en segundos
   potPI = map(analogRead(A1),0,1023,300,PasosInsp);              // leemos numero de pasos para inspiración, lo cual definirá volumen 
   Tentrepasos = (((TCiclo*1000000 *FInsp) / potPI)/ MicroStep);  // Tiempo entre pasos en microsegundos dividido por los micropasos
   
   if (digitalRead(botonPin) == HIGH){                            // leemos el boton de encendido. Funciona solo si encendido (switch)
   
// CICLO INSPIRACION   
      dirMotor = !dirEspira;
      MueveMotor();
      unsigned long iniciomeseta = millis();                      //Meseta
      while (millis() -iniciomeseta < TMeseta){
        if (millis() - TultmedPT > TCheqPT) chequeoPT();
      }
      //      if (ALARMA == 1) RegresaOrigen();// Modificar: si brazo no en origen, o motor no en origen, regresaorigen. si no resuelve: ALARMA
      unsigned long Tinspira = millis()-Tinicio;                  // Guarda duracion de inspiración

// CICLO ESPIRACION        
      Tentrepasos = 600;                                          // acelerando espiracion (microsegundos)
      dirMotor = dirEspira;
      MueveMotor();
      digitalWrite(enablePin, HIGH);                              // desactiva el driver del motor, regresa a origen si AMBU presionado

// Pausa restante para ciclos por minuto establecidos en millis
      unsigned long pausita = (TCiclo*1000 - (millis()- Tinicio)) + Pausa;
      if (pausita>100){
        unsigned long  iniciopausita = millis();
        while (millis() - iniciopausita < pausita){    
          if (millis() - TultmedPT > TCheqPT) chequeoPT();        // Si paso el tiempo para medir, medir
        }
      }
      
      if (!digitalRead(origenMotPin) || !digitalRead(origenBraPin) ){ 
        CALIBRA = 2;
        RegresaOrigen();  
        Serial.println("                            Recalibró origen.");
        CALIBRA = 0;
      }
    
//cada hora, recalibra contra presion atmosferica.........................................................

                                                                  //Reporta datos Msg Usando MegunoLink
     Message Msg; // Message Monitor will display 
Msg.Begin(); // Message Monitor will display 
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
     Serial.print(F("Presión DISTAL  Máxima: "));
     Serial.print(presionMax1);       
     Serial.print(F("  Minima:"));
     Serial.println(presionMin1);   
     Serial.print(" Temperatura:  ");
     Serial.println(temperature);
     Serial.print(" Minutos desde inicio del respirador:  ");
     Serial.println(millis()/60000);
     Serial.println();
Msg.End();
   }
  presionMax1 = 0;              //resetea los valores Presion del ciclo
  presionMin1 = 0;
  presionMax2 = 0;
  presionMin2 = 0;
}

void pin_ISR() {    //Interrupt para leer el boton de reset
  ResetAlarma = 1;
}
