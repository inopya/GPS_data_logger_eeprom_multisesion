/*
  #       _\|/_   A ver..., ¿que tenemos por aqui?
  #       (O-O)        
  # ---oOO-(_)-OOo---------------------------------
   
   
  ##########################################################
  # ****************************************************** #
  # *            DOMOTICA PARA AFICIONADOS               * #
  # *       GPS DATALOGGER  PARA @NORMA, LA PERRA        * #
  # *          Autor:  Eulogio López Cayuela             * #
  # *            https://github.com/inopya               * #
  # *                                                    * #
  # *       Versión v1.1      Fecha: 30/06/2020          * #
  # ****************************************************** #
  ##########################################################
*/

#define __VERSION__ "GPS Datalogger NORMA Version v1.1 multisesion\n"

/*
  
   ===== NOTAS DE LA VERSION =====
   GPS data logger para Norma la perra,
   Version para tratar de corregir los problemas de la version con grabacio en SD
   que corrompia las tarjetas si las grabaciones eran muy largas y la bateria se agotaba.
   Ante la imposibilidad de saber cuando va a volver... 
   en este caso realizamos la grabacion de datos en eeprom externa.
   Inicialmente usaremos una eeprom del tipo 24LC256, es decir 32Kbyte
   Si realizamos la grabacion de posicion unicamente (latitud y longitud) a intervalos de 5 segundos
   disponemos de un tiempo de algo mas de 5 horas y media. O de mas de 11 horas si tomamos datos cada 10 segundos  :)

   Añadimos un acelerometro para descartar el "baile" que produce el GPS aun estando en absoluto reposo.
   De esta manera si el acelerometro no muestra indicios de movimiento, no grabamos ningun dato


   <<< CODIGOS DE COLOR SEGUN ESTADOS DEL DATALOGGER  >>> 

    * AZUL: Inicio, en espera de ordenes de teclado (xx segundos)
  
    * AZUL/PARPADEO: en espera de ordenes de teclado en bucle infinito
  
      Si se pulsa 'L' se listan datos de la sesion anterior:
        * NARANJA/AMARILLO: Modo Usuario, listando datos.
        * VERDE:  Modo Usuario, pausa infinita tras listar datos. La eeprom queda reservada de uso.

    
    * ROJO: Modo Activo, a espera de señal GPS valida para iniciar grabacion.
    * APAGADO: Modo Activo, GPS ok, esperando entre muestras.
    * ROSA/MORADO: Modo Activo, guardando una muestra.
*/


/*
 ---  COLORES DE REFERENCIA PARA EL LED R/G/B  --- 

  (255, 000, 000);    // rojo
  (000, 255, 000);    // verde
  (000, 000, 255);    // azul
  (255, 255, 000);    // amarillo
  (255, 000, 255);    // morado / rosa
  (255, 140, 000);    // naranja
  (255, 105, 180);    // rosa
  (175, 255, 045);    // amarillo verdoso
  (000, 255, 255);    // celeste/agua
  (000, 180, 255);    // azul cielo
  (000, 000, 140);    // azul oscuro
  (000, 000, 128);    // navy
  (255, 255, 255);    // blanco
  (80, 80, 80);       // gris 
  
*/


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        IMPORTACION DE LIBRERIAS 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#include <SoftwareSerial.h>
#include <Temporizador_inopya.h>
#include <Universal_GPS_inopya.h>
#include <I2C_EEPROM_inopya.h>
#include <Adafruit_NeoPixel.h>    // accesorio (por si tenemos un led RGB de estado)
#include <Wire.h>

/* VARIABLES Y CONSTANTES PARA EL ACELEROMETRO ADXL345 */

#define ACELEROMETRO_ADDR     0x53       // Direccion i2c del acelerometro ADXL345
byte _buff[6];                           // buffer apra datos
 
//Direcciones de los registros del ADXL345
char POWER_CTL = 0x2D;
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;   //X-Axis Data 0
char DATAX1 = 0x33;   //X-Axis Data 1
char DATAY0 = 0x34;   //Y-Axis Data 0
char DATAY1 = 0x35;   //Y-Axis Data 1
char DATAZ0 = 0x36;   //Z-Axis Data 0
char DATAZ1 = 0x37;   //Z-Axis Data 1


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DECLARACION DE CONSTANTES  Y  VARIABLES GLOBALES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#define PIN_Rx         4
#define PIN_Tx         5
#define LED_OnBoard   13


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DECLARACION DE CONSTANTES  Y  VARIABLES GLOBALES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#define CHIP_ID           0x50         // la direccion de memoria del chip (24LC256) si A0, A1 y A2 estan a GND es 0x50
#define EEPROM_SIZE		   32700		     // recordar que el chip (24LC256) dispone de 32768 posiciones de memoria (0-32767)

/*  POS_MEM... direccion  de memoria  -->  (...)  tipo de dato que almacenan / detalles */
#define POS_MEM_USER_DATA_START    100 // (struct) primera posicion usada para datos
#define POS_MEM_READ_ONLY            0 // (1byte) registro para opcion de bloqueo de escritura por software
#define POS_MEM_SESION_NUMBER        1 // (1byte) registro para el numero de sesiones (ahora se permite multisesion)
#define POS_MEM_LAST_COUNT           2 // (2bytes) contador del numero de datos de la ultima sesion,
                                       // o de la primera si solo hubiese una.
#define POS_MEM_FIRST_SESION_POINTER 4 // (2bytes) almacena el puntero hacia la primera sesion de datos,
                                       // si hay mas de uan sesion incrementaremos (+2) esta posicion tantas veces como
                                       // sesiones indique POS_MEM_SESION_NUMBER


#define USER_DATA_SIZE               8 // para los incrementos de puntero al guardar datos

#define WAIT_SERIAL_AT_STARTUP      45000                // por defecto 45 segundos esperando antes de iniciar el programa
								
uint16_t puntero_eeprom = POS_MEM_USER_DATA_START;       // unsigned int, para disponer del rango 0-65535 

/* creacion de un nuevo tipo de datos para contener las muestras */
struct struct_datos_gps { 
                            float latitud;
                            float longitud;
                         };

struct struct_datos_acelerometro { 
                            int x;
                            int y;
                            int z;
                         };


//ENUM COMANDOS SERIAL
enum enumComandosSerial 
{
  CMD_FULL_ERASE      =  0,    // borrado por completo de la eeprom
  CMD_REG_ERASE       =  1,    // borrado aparcial, contadores y punteros de sesiones
  CMD_BREAK           =  2,    // salir del bucle de espera
  CMD_LISTAR_DATOS    =  3,    // listar todas las sesiones
  CMD_LOCK_EEPROM     =  4,    // protecccion de escritura de la eeprom
  CMD_WRITE_ENABLE    =  5,    // permiso de escritura de la eeprom
  CMD_SERIAL_FOREVER  =  6,    // permanecer siempre en el monitor serial hasta pulsar Q
  CMD_WAIT            =  7,    // sin comando, valido, preguntar de nuevo
  CMD_INFO            =  8,    // mostar informacion de sesiones previas  ¿lo implementamos?
};

                         
/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION PARA DECLARACION DE OBJETOS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

I2C_EEPROM_inopya memory_chip(CHIP_ID);   // Creamos un objeto del tipo i2c_eeprom (eeprom externa)

SoftwareSerial gpsPort(PIN_Rx, PIN_Tx);   // puerto serie para conectar el modulo GPS

Universal_GPS_inopya NEO_gps(&gpsPort);   // Creamos un objeto gps pero diciendole el puerto
                                          // de mosdo que se simplifica el acceso a la recogida  de datos

/* Crear el 'objeto' para controlar el led RGB */
#define PIN_TIRA_LED      8                    // patilla para la tira
#define TIPO_LED         NEO_GRB + NEO_KHZ800  //tipo controlador de los led
byte LONGITUD_TIRA = 1;                        // total de leds
Adafruit_NeoPixel tiraLEDS = Adafruit_NeoPixel(LONGITUD_TIRA, PIN_TIRA_LED, TIPO_LED);

/* Crear temporizadores */
Temporizador_inopya updateGPS;    
Temporizador_inopya parpadeoLED;   
Temporizador_inopya lecturaAcelerometro;   

Temporizador_inopya tiempoWaitSerial;   



//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//         FUNCION DE CONFIGURACION
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void setup() 
{
  /* Iniciar el puerto serie de Arduino para Debug y transferencia de datos */
  Serial.begin(115200);
  Serial.println(F(__VERSION__));

  /* Inicializr el led RGB */
  tiraLEDS.begin();
  tiraLEDS.setBrightness(20);
  mostrarColor(000, 000, 000);   //NEGRO, led apagado

  /* Iniciar el acelerometro conectado al bus i2c */
  Wire.begin();
  writeTo(ACELEROMETRO_ADDR, DATA_FORMAT, 0x01); //(0x02)Poner ADXL345 en:  (00)+-2G, (01)+-4G,(02)+-8G, (03)+-16G,
  writeTo(ACELEROMETRO_ADDR, POWER_CTL, 0x08);   // POWER_CTL = 0x08, pasar a modo activo, medicion
  
  /* Iniciar un puerto serie sofware para comunicar con el GPS */
  NEO_gps.begin(9600);  //iniciamos el GPS a la velocidad standard

  /* Activar el led de la placa */
  pinMode(LED_OnBoard, OUTPUT);
  
  Serial.print(F("GPS por defecto en modo: "));Serial.println(NEO_gps.get_mode());
  
  //NEO_gps.timeout(1000);      // por defecto la libreria escucha al GPS hasta un maximo de 1200 ms
  //NEO_gps.set_mode(1);        // cambiamos a modo 1 decodificar solo GNGGA/GPGGA.
  //uint8_t modo_actual = NEO_gps.get_mode();
  //Serial.print(F("Modo de trabajo actual: "));Serial.print(modo_actual);

  /* Test manual de la eeprom */
//  memory_chip.save(2, 1375.24); 
//  float temporal;
//  memory_chip.load(2, temporal); 
//  Serial.print(F("temporal se grabo como: "));Serial.print(temporal);

  Serial.println(F("\nEsperando orden (45 segundos):"));
  Serial.println(F("Pulsa 'L' si quieres ver los datos grabados\n"));
  mostrarColor(0, 0, 50);   //Azul a la espera de comando y/o señal valida del gps

  tiempoWaitSerial.begin(WAIT_SERIAL_AT_STARTUP);
  bool FLAG_serial_forever = false;
  bool FLAG_blink_blue = false;

  /* INICIO ----Boque DEBUG para corregir manualmente la posicion y tamañode la ultima sesion */
  //uint16_t mierda=5;
  //memory_chip.save(POS_MEM_LAST_COUNT, mierda);
  //uint8_t caca =4;
  //memory_chip.save(POS_MEM_SESION_NUMBER, caca);  //MIERDA
  /* FIN ------  Boque DEBUG para corregir manualmente la posicion y tamañode la ultima sesion */
  
  while(tiempoWaitSerial.estado() == true || FLAG_serial_forever){
    uint8_t comando = comandosPuertoSerie(); 

    /* Mostar el numero de seiones que hay grabadas */    
    if (comando == CMD_INFO){ 
      uint8_t sesiones_viejas;
      memory_chip.load(POS_MEM_SESION_NUMBER, sesiones_viejas);
      Serial.print(F("LA EEPROM CONTIENE "));
      Serial.print(sesiones_viejas);
      Serial.println(F(" SESIONES\n"));
    }

    /* permnecer indefinidamente en el monitor serie */    
    if (comando == CMD_SERIAL_FOREVER){ 
      FLAG_serial_forever = true;
      parpadeoLED.stop();           // temporizador para mostar un parpadeo  el el led Direccionable
      parpadeoLED.begin(500);       // que idique que estamos en espera infinita
      Serial.println(F("MONITOR SERIAL (FOREVER or Q)"));
    }
    
    if (FLAG_serial_forever){ 
      if( parpadeoLED.estado() == false){
      parpadeoLED.begin(500);
      if(FLAG_blink_blue){
        mostrarColor(0, 0, 50);   //Azul a la espera de comando y/o señal valida del gps
      }
      else{
        mostrarColor(0, 0, 0);   //Azul a la espera de comando y/o señal valida del gps
      }
      FLAG_blink_blue=!FLAG_blink_blue;
      }
    }
        
    /* permnecer indefinidamente en el monitor serie */    
    if (comando == CMD_BREAK){ 
      tiempoWaitSerial.stop();
      FLAG_serial_forever = false;  //por si estabamos en espera infinita
      Serial.println(F("BREAK SERIAL"));
    }
        
    /* borrado completo de la eeprom */    
    if (comando == CMD_FULL_ERASE){  
      full_erase_eeprom();
      Serial.println(F("FULL ERASE"));
    }
    /* borradpo parcial de la eeprom */    
    if (comando == CMD_REG_ERASE){ 
      soft_erase_eeprom();
      Serial.println(F("SOFT ERASE"));
    }
        
    /* activar el uso de la eeprom */    
    if (comando == CMD_WRITE_ENABLE){
      //FLAG_uso_eeprom = true;
      //memory_chip.write(POS_MEM_READ_ONLY,0);
      Serial.println(F("EEPROM Activada (DEMO)"));
    }
    
    /* desactivar el uso de la eeprom */ 
    if (comando == CMD_LOCK_EEPROM){  //sin uso
      //FLAG_uso_eeprom = false;
      //memory_chip.write(POS_MEM_READ_ONLY,101);
      Serial.println(F("EEPROM Desactivada(DEMO)"));
    }

    /* listar datos GPS almacenados en eeprom */ 
    if (comando == CMD_LISTAR_DATOS){
      mostrarColor(90, 45, 000);  // NARANJA, listando datos
      read_multi_sesion();
      mostrarColor(0, 50, 0);     // Verde. Reposo infinito, la eeprom queda preservada
      while(true);                // si lo preferimos podemos activar la bandera 'FLAG_serial_forever'
                                  // que nos permitiría salir pulsando la letra Q
    }                                    
  }

  Serial.println(F("\nGRABANDO DATOS...\n"));
  mostrarColor(50, 0, 0);   //rojo, comienza el proceso de toma de datos

  /* Esperar a tener datos validos del GPS para iniciar la grabacion en EEPROM (solo validamos el año) */
  while( NEO_gps.year != 2020 ){
    NEO_gps.get(); 
    //Serial.print(F("."));
    Serial.println(NEO_gps.year);
    delay(2000);
  }


  /* Establecemos el puntero de grabacion para los datos GPS de la sesion*/
  puntero_eeprom = POS_MEM_USER_DATA_START;  // por defecto presuponemos que es la primera sesion... ahora lo veremos
  uint8_t numero_sesiones = 0; 

  //leemos el contador de sesiones para averiguar si hay sesiones anteriores
  memory_chip.load(POS_MEM_SESION_NUMBER, numero_sesiones);
  
  if( numero_sesiones>0){
    /* localizar donde empieza la ultima sesion grabada. */
     uint16_t p1;
     memory_chip.load((2*(numero_sesiones-1))+POS_MEM_FIRST_SESION_POINTER, p1); 
     
    /* incrementar dicho puntero en funcion de los datos de sesion indicados por POS_MEM_LAST_COUNT */
    uint16_t last_sesion_size;
    memory_chip.load(POS_MEM_LAST_COUNT, last_sesion_size);
    
    /* incrementar dicho puntero en funcion de los datos de sesion indicados por POS_MEM_LAST_COUNT */
    p1 += (USER_DATA_SIZE * last_sesion_size);
    /* registar ese nuevo puntero */
    memory_chip.save((2*numero_sesiones)+POS_MEM_FIRST_SESION_POINTER, p1);
    /* establecer ese nuevo puntero como puntero de trabajo */ 
    puntero_eeprom = p1;
  }
  else{
    /* si es la primera sesion guardamos POS_MEM_USER_DATA_START en POS_MEM_FIRST_SESION_POINTER */
    memory_chip.save(POS_MEM_FIRST_SESION_POINTER, POS_MEM_USER_DATA_START);
  } 
  
  /* incrementamos el numero de sesiones y actualizamos dicho registro */
  numero_sesiones++;
  memory_chip.save(POS_MEM_SESION_NUMBER, numero_sesiones);

  Serial.print(F("\nIniciando sesion nº "));
  Serial.println(numero_sesiones);

  /* borrar el valor de posibles contadores antigüos POS_MEM_LAST_COUNT */
  memory_chip.save(POS_MEM_LAST_COUNT, 2);  //forzamos 2 datos, la fecha y un primer punto tomado justo al inciciar

  
  /* Posicionar el puntero eeprom para grabar el registro de inicio (Fecha/Hora) de la sesion */
  //puntero eeprom apunta en este momento a  p1, es decir inicio de la sesion actual 
  //Grabamos como primer dato la fecha y hora de la sesion
  save_moment();

  delay(1500); //para darle dramatismo a la escena

}



//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 
//***************************************************************************************************
//  BUCLE PRINCIPAL DEL PROGRAMA   (SISTEMA VEGETATIVO)
//***************************************************************************************************
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm 

void loop()
{
  static uint16_t contador = 0;
  static bool FLAG_enMovimiento;
  static int8_t contadorReposo=0;
  
  if(parpadeoLED.estado()==false){    //lo usamos en setu para parpadear el led azul, lo reusamos aqui
    parpadeoLED.begin(250);           //nuestro reloj no obliga, impide que se haga antes de ese tiempo
    digitalWrite(LED_OnBoard, !digitalRead(LED_OnBoard));
  }


  if(updateGPS.estado()==false){
    //updateGPS.begin(1000);    //nuestro reloj no obliga, impide que se haga antes de ese tiempo
    NEO_gps.get();             //actualizar la informacion desde el gps

    //FLAG_enMovimiento = true;  //DEBUG forzamos grabacion de gps aun estando quietos
    
    if (FLAG_enMovimiento && NEO_gps.year == 2020){  //solo si hay movimiento y los datos de gps son correctos
      updateGPS.begin(5000);  //si la muestra correcta, 5 segundos para la siguiente.
      /* Mostar Contador de muestras */
      if(contador < 10){ Serial.print(F("0")); }
      if(contador < 100){ Serial.print(F("0")); }
      if(contador < 1000){ Serial.print(F("0")); }
      if(contador < 10000){ Serial.print(F("0")); }
      Serial.print(contador++);
      //Serial.print(F(",\t"));
      Serial.print(F(",")); 
  
      /* Mostar Posicion */
      Serial.print(NEO_gps.latitud,6); 
      Serial.print(F(",")); 
      Serial.print(NEO_gps.longitud,6);
      Serial.println();
  
      /* Guardar LATITUD  y LONGITUD */
      struct_datos_gps puntoRecorrido;
      puntoRecorrido.latitud = NEO_gps.latitud;
      puntoRecorrido.longitud = NEO_gps.longitud;
      memory_chip.save(puntero_eeprom, puntoRecorrido); 
      mostrarColor(55, 000, 55);  //rosa/morado
      delay(100);
      Serial.println(F("punto GPS salvado!"));
      puntero_eeprom+=USER_DATA_SIZE;
      if(contador%12==0){  //cada minuto...
        memory_chip.save(POS_MEM_LAST_COUNT, contador);   // <--- mejor guardar contador de datos
        Serial.println(F("\n  *** Contador salvado! *** \n")); 
        //memory_chip.save(POS_MEM_LAST_COUNT, puntero_eeprom); 
        //Serial.println(F("puntero_eeprom salvado!"));
      }
    }
    else{
      updateGPS.begin(1000);  // si hubo error, 1 segundo y volver a intentar hablar con el GPS
    }
    FLAG_enMovimiento ? Serial.println(F("RUNNING...")) : Serial.println(F("STOP"));
    mostrarColor(000, 000, 000);   //NEGRO, led apagado
  }


  if(lecturaAcelerometro.estado()==false){
    lecturaAcelerometro.begin(100);
    /* Leer acelerometro: x, y, z */
    //FLAG_enMovimiento = readAccel(); 
    if( readAccel()==false ){ contadorReposo++; }
    else{ contadorReposo--; }
    if( contadorReposo<0 ) { contadorReposo=0; }
    if( contadorReposo>3 ) { contadorReposo=3; }
    if( contadorReposo <2 ) { FLAG_enMovimiento = true; }
    else{ FLAG_enMovimiento = false; }
  }
}

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
   ###################################################################################################### 
                                         BLOQUE DE FUNCIONES
   ###################################################################################################### 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   SALVAR FECHA Y HORA DE INICIO COMO REFERENCIA 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void save_moment()
{  
  /* Guardar Fecha */
  memory_chip.save(puntero_eeprom, NEO_gps.dia); 
  puntero_eeprom+=1;

  memory_chip.save(puntero_eeprom, NEO_gps.mes);
  puntero_eeprom+=1;

  memory_chip.save(puntero_eeprom, NEO_gps.year);
  puntero_eeprom+=2;
  
  /* Guardar Reloj */
  memory_chip.save(puntero_eeprom, NEO_gps.hora);
  puntero_eeprom+=1;

  memory_chip.save(puntero_eeprom, NEO_gps.minuto);
  puntero_eeprom+=1;

  memory_chip.save(puntero_eeprom, NEO_gps.segundo);
  puntero_eeprom+=1;

  /* rellenar hasta 8 bytes y asi ser una muestra mas a efectos de puntero general de datos */
  memory_chip.save(puntero_eeprom, 101);  //dato de relleno (¡que pánico!)
  puntero_eeprom+=1;
  
  Serial.println(F("Fecha y Hora de sesion guardados"));
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   RECUPERAR DE EEPROM LA FECHA Y HORA DE INICIO DE UNA SESION
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void load_moment(uint16_t puntero)
{ 
  uint16_t puntero_memoria = puntero;
  uint8_t dato;
  
  /* cargar DIA */
  memory_chip.load(puntero_memoria, dato); 
  if(dato < 10){ Serial.print(F("0")); }
  if( dato>31 ){ printDateError(); return; }
  Serial.print(dato); Serial.print(F("/"));
  puntero_memoria+=1;

  /* cargar MES */
  memory_chip.load(puntero_memoria, dato);
  if(dato < 10){ Serial.print(F("0")); }
  if( dato>12 ){ printDateError(); return; }
  Serial.print(dato); Serial.print(F("/"));
  puntero_memoria+=1;

  /* cargar AÑO */
  uint16_t dato2;
  memory_chip.load(puntero_memoria, dato2);
  if( dato>2100 ){ printDateError(); return; }
  Serial.print(dato2); Serial.print(F("\t"));
  puntero_memoria+=2;
  
  /* cargar HORAS */
  memory_chip.load(puntero_memoria, dato);
  if( dato>23 ){ printDateError(); return; }
  if(dato < 10){ Serial.print(F("0")); }
  Serial.print(dato); Serial.print(F(":"));
  puntero_memoria+=1;

  /* cargar MINUTOS */
  memory_chip.load(puntero_memoria, dato);
  if( dato>59 ){ printDateError(); return; }
  if(dato < 10){ Serial.print(F("0")); }
  Serial.print(dato); Serial.print(F(":"));
  puntero_memoria+=1;

  /* cargar SEGUNDOS */
  memory_chip.load(puntero_memoria, dato);
  if( dato>59 ){ printDateError(); return; }
  if(dato < 10){ Serial.print(F("0")); }
  Serial.println(dato);
  puntero_memoria+=1;
}



void printDateError()
{
  Serial.println(F("Fecha/Hora de sesion no validas\nPosible error en los datos\n\n"));
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//      COMUNICACIONES (LECTURA DE CARACTERES POR PUERTO SERIE) 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

uint8_t comandosPuertoSerie() 
{
  char orden_recibida = ' ';
   while(Serial.available()) {
    orden_recibida = Serial.read();
    
    if(orden_recibida == 'e' || orden_recibida == 'E'){  //desbloqueamos acceso a la eeprom (solo DEBUG)
      return CMD_WRITE_ENABLE;
    }
    
    if(orden_recibida == 'd' || orden_recibida == 'D'){  // bloquear acceso eeprom (solo DEBUG)
      return CMD_LOCK_EEPROM;
    }

    if(orden_recibida == 'l' || orden_recibida == 'L'){  // listar datos de eeprom
      return CMD_LISTAR_DATOS;
    }
    
    if(orden_recibida == 'q' || orden_recibida == 'Q'){  // Saltar tiempo de espera e iniciar modo activo
      return CMD_BREAK;
    }
    
    if(orden_recibida == 'f' || orden_recibida == 'F'){  // borrado completo de la eeprom
      return CMD_FULL_ERASE;
    }
    
    if(orden_recibida == 's' || orden_recibida == 'S'){  // borrado parcial de eeprom. Solo zona de registros
      return CMD_REG_ERASE; 
    }
    
    if(orden_recibida == 'o' || orden_recibida == 'O'){  // permanecer en el puerto serial a espera de ordenes
      return CMD_SERIAL_FOREVER; 
    }
    
    if(orden_recibida == 'i' || orden_recibida == 'I'){  // mostar informacion??
      return CMD_INFO; 
    }
  }
  return CMD_WAIT;
}

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    CONTROL DE LA TIRA DE LEDS (ireccionable) PARA CREAR LOS EFECTOS DE COLOR
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  MOSTRAR UN COLOR (tira completa)
//========================================================

void mostrarColor(byte red, byte green, byte blue)
{
  for (int pixel=0; pixel< tiraLEDS.numPixels(); pixel++){
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(red, green, blue)); //color unico 
  } 
  tiraLEDS.show();
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    CONTROL DEL ACELEROMETRO SIN LIBRERIA
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  DETECTAR MOVIMIENTO CON EL ACELEROMETRO
//========================================================
 
bool readAccel() 
{
  static int old_x;
  static int old_y;
  static int old_z;
  bool FLAG_movimiento = false;
  
  //Leer los datos
  uint8_t numBytesToRead = 6;
  readFrom(ACELEROMETRO_ADDR, DATAX0, numBytesToRead, _buff);
 
  //Leer los valores del registro y convertir a int (Cada eje tiene 10 bits, en 2 Bytes LSB)
  int x = (((int)_buff[1]) << 8) | _buff[0];   
  int y = (((int)_buff[3]) << 8) | _buff[2];
  int z = (((int)_buff[5]) << 8) | _buff[4];
  
//  Serial.print("x: ");
//  Serial.print( x );
//  Serial.print(" y: ");
//  Serial.print( y );
//  Serial.print(" z: ");
//  Serial.println( z );

  struct_datos_acelerometro info_acelerometro;
  info_acelerometro.x = x;
  info_acelerometro.y = y;
  info_acelerometro.z = z;


//  Serial.print("x: ");
//  Serial.print( abs(old_x-x) );
//  Serial.print(" y: ");
//  Serial.print( abs(old_y-y) );
//  Serial.print(" z: ");
//  Serial.println( abs(old_z-z) );

  //if((abs(old_x-x)+abs(old_y-y)+abs(old_z-z)) > 5){
  if(abs(old_x-x) > 2){
    FLAG_movimiento = true;
  }
  if(abs(old_y-y)>2){
    FLAG_movimiento = true;
  }
  if(abs(old_z-z)>2){
    FLAG_movimiento = true;
  }
  old_x=x;
  old_y=y;
  old_z=z;
  
  return FLAG_movimiento;
  
}

//========================================================
//  Funcion auxiliar de escritura en el bus i2c
//========================================================

void writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission(); 
}

//========================================================
//  Funcion auxiliar de lectura en el bus i2c
//========================================================

void readFrom(int device, byte address, int num, byte _buff[]) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.endTransmission();
 
  Wire.beginTransmission(device);
  Wire.requestFrom(device, num);
 
  int i = 0;
  while(Wire.available())
  { 
    _buff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    CONTROL DE MULTISESIONES EN EEPROM
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  ORGANIZAR BUSQUEDA DE MULTISESIONES
//========================================================

void read_multi_sesion()
{
  uint8_t contador=0;
  uint16_t p1;
  uint16_t p2;
  uint16_t sesion_size;
  uint8_t numero_sesiones;
  
  memory_chip.load(POS_MEM_SESION_NUMBER, numero_sesiones); 
  memory_chip.load(POS_MEM_FIRST_SESION_POINTER, p2);  // dejamos cargado p2 por si no hay multisesion
  
  Serial.print(F("DEBUG >> puntero lectura cargado: "));Serial.println(p2);
  
  if(numero_sesiones==0){
    Serial.println(F("\n\nNO HAY SESIONES, EEPROM VACIA\n\n"));
    return;
  }
  
  if(numero_sesiones>1){
    while( contador < (numero_sesiones-1)){

      Serial.print(F("DEBUG >> sesion: "));Serial.println(contador+1);
      
      p1= memory_chip.load((2*contador)+POS_MEM_FIRST_SESION_POINTER, p1); 
      p2= memory_chip.load(2*(contador+1)+POS_MEM_FIRST_SESION_POINTER, p2); 
      sesion_size = p2-p1;  //ojo sesion size es la diferencia entre punteros, 
                            //el numero de datos es ese valor dividido entre el tamaño de un dato (struct alamacenado)
                            
      sesion_size = sesion_size / USER_DATA_SIZE; 
      Serial.print(F("DEBUG >> sesion_size: "));Serial.println(sesion_size);
      Serial.print(F("DEBUG >> p1: "));Serial.println(p1);
      Serial.print(F("DEBUG >> p2: "));Serial.println(p2);   
      if(sesion_size>0){                  
        listar_datos_multisesion(p1, sesion_size);
      }
      else{
        Serial.println(F("\nDEBUG >> ESTA SESION NO ES VALIDA, la saltamos\n"));
      }
      contador++;
    }
  }

  memory_chip.load(POS_MEM_LAST_COUNT, sesion_size);   //tamaño de la ultima sesion (o de la unica, si es es el caso) 
  
  Serial.print(F("DEBUG >> puntero lectura enviado: "));Serial.println(p2);
  Serial.print(F("DEBUG >> sesion_size: ")); Serial.println(sesion_size);
  listar_datos_multisesion(p2, sesion_size);           //leemos la sesion del ultimo puntero
}


//========================================================
//  LISTAR CADA UNA DE LAS SESIONES ( MULTISESION )
//========================================================

void listar_datos_multisesion(uint16_t _p2, uint16_t _size)
{
  Serial.print(F("DEBUG >>Mostrando "));
  Serial.print(_size);
  Serial.println(F(" registros...\n"));
  
  /* Leer los datos de sesion (Fecha/Hora) */
  uint16_t puntero_lectura = _p2;
  //Serial.print(F("DEBUG >> puntero lectura llegado: "));Serial.println(puntero_lectura);
  Serial.println(F("DEBUG >> ***************************"));
  uint16_t sesion_size = _size-1; //ya que el primero sera el de fecha y hora
  load_moment(puntero_lectura);
  puntero_lectura+=USER_DATA_SIZE;  //incrementamos el puntero tras leer e interpretar la fecha/hora


  /* recuperar los puntos del recorrido */
  uint16_t contador=0;

  while( contador < sesion_size ){ 
    struct_datos_gps dato_leido;  
    memory_chip.load(puntero_lectura, dato_leido);  //recuperar un punto gps
    puntero_lectura+=USER_DATA_SIZE;                //aumenter el puntero

    /* Mostar Contador de muestras */
    if(contador < 10){ Serial.print(F("0")); }
    if(contador < 100){ Serial.print(F("0")); }
    if(contador < 1000){ Serial.print(F("0")); }
    if(contador < 10000){ Serial.print(F("0")); }
    Serial.print(contador++);
    Serial.print(F(","));
    
    Serial.print(dato_leido.latitud,6); 
    Serial.print(F(","));
    Serial.println(dato_leido.longitud,6);
  }
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   BORRADO PARCIAL DE EEPROM LA FECHA (SOLO INFORMACION DE SESIONES)
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void soft_erase_eeprom()
{
  for(uint16_t reg=0; reg< POS_MEM_USER_DATA_START-1 ;reg++){
    memory_chip.write( reg, 0 );
  }
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   BORRADO COMPLETO DE EEPROM 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void full_erase_eeprom()
{
  for(uint16_t reg=0; reg< EEPROM_SIZE+10 ;reg++){
    memory_chip.write( reg, 0 );
  }
}

//*******************************************************
//                    FIN DE PROGRAMA
//*******************************************************
