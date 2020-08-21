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
  # *        Versión v1.2     Fecha: 16/07/2020          * #
  # ****************************************************** #
  ##########################################################
*/

#define __VERSION__ "INOPYA - GPS Datalogger NORMA v1.2 \n"


/* Parametros que mas frecuentemente se necesita modificar */
bool FLAG_disable_acelerometro  =   false;  // TRUE: graba puntos GPS independientemente de que se detecte o no movimiento

#define DEBUG_MODE                  false   // (false) TRUE: activar info general
#define DEBUG_MODE_ACELETOMETRO     false   // (false) TRUE: activar info detallada del acelerometro
#define INTERVALO_MUESTRAS           5000   // (5000)  5000ms, 5 segundos
//#define EEPROM_SIZE               32650   // El chip (24LC256) dispone de 32768 posiciones de memoria (0-32767)
#define EEPROM_SIZE                 65400   // El chip (24LC512) dispone de 65536 posiciones de memoria (0-65535)



/*
  
   ===== NOTAS DE LA VERSION =====
   GPS data logger para Norma la perra,
   Version para corregir los problemas de la version con grabacion en SD
   que corrompia las tarjetas si las sesiones eran muy largas (y la bateria se agotaba).
   
   Ante la imposibilidad de saber cuando va a volver la muy perra,
   en este montaje realizamos la grabacion de datos en eeprom externa.
   Inicialmente usaremos una eeprom del tipo 24LC256, es decir 32Kbytes, 
   pero el programa admite memorias de hasta 64Kbytes.
   Si realizamos la grabacion de posicion unicamente (latitud y longitud) a intervalos de 5 segundos
   disponemos de un tiempo de algo más de 5 horas y media. O de más de 11 horas si tomamos datos cada 10 segundos.
   (Obviamente el programa se puede adaptar para que grabe otro tipo de informacion).

   - Añadimos un acelerometro para descartar el "baile" que produce el GPS aun estando en absoluto reposo.
     De esta manera si el acelerometro no muestra indicios de movimiento, no grabamos ningun dato 
     y alargamos aun mas la duracion de la eeprom.
   
   - Añadida la opcion de grabar sesiones multiples mientras quede memoria disponible.
     Ayuda a no sobreescribir siempre la misma zona, ya que podemos ir añadiendo sesiones 
     hasta completar todo el espacio de laeeprom y de paso protege los datos ya grabados
     si se produce algun reinicio durante alguna sesion.
   
   - Añadido un menu Serial para la recuperacion de datos y otras tareas sobre la eeprom.
     Consultar la funcion "mostrar_menu_serial()" para mas detalles de todas las opciones disponibles.

   - Añadidos control de tiempo disponible para grabacion y bloqueo del programa para evitar sobreescribir
     si una sesion fuese mas larga que el tiempo disponible.

   - Anadida opcion de menu para la lectura en modo RAW de la eeprom (por si falla la FAT)
     y deseamos tener acceso a su contenido para tratar de recuperar informacion de forma manual.

   - Cambiado el sistema de FAT (respecto de la version beta),
     para hacerlo mas intuitivo y facil de manejar en caso de "catastrofe".

   - Posibilidad de acceso y consulta de sesiones individuales o listado completo del contenido grabado.

   - Incrementado el espacio FAT respecto a la version Beta, pasamos de 48 posibles sesiones a 148**.
     (El usuario puede controlar dicho tamaño a voluntad, modificando donde comienza 
      el espacio de 'datos de usuario', ya que la FAT se extiende  hasta dicha zona).
      ** Maximas sesiones admitidas 255, ya que el registro destinado a ellas es un solo byte.
         Si fuese necesario se puede modificar sin demasiada complicacion.


   <<< CODIGOS DE COLOR DEL LED RGB, SEGUN ESTADO DEL DATALOGGER  >>> 

    * AMARILLO: 1 segundo al iniciar, Avisa de puesta en marcha y advierte de posibles reinicios.

    * AZUL: al inicio color fijo, en espera de ordenes de teclado (xx segundos), 
            Pasado ese tiempo se inicia automaticamente el proceso de toma de muestras.
  
    * AZUL/PARPADEO: en espera de ordenes de teclado en bucle infinito (podemos interrumpir pulsando 'Q').
  
      Si se pulsa 'L' se listan datos de la sesion anterior:
        * NARANJA/AMARILLO: Modo Usuario, listando datos.
        * VERDE:  Modo Usuario, pausa infinita tras listar datos. La eeprom queda reservada de uso.

    
    * ROJO: Modo Activo, a espera de señal GPS valida para iniciar grabacion.
    * ROSA/MORADO: Modo Activo, guardando una muestra. Un parpadeo que se apaga tras guardarla.
    * ROSA/MORADO: Modo Activo, Si queda encendido permanente, avisa de Fin de memoria, o fin de FAT
    * APAGADO: Modo Activo, GPS ok, esperando entre muestras.
    
*/


/*
 ---  ALGUNOS COLORES DE REFERENCIA PARA EL LED R/G/B  --- 

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
  ( 80,  80,  80);    // 'gris' 
  (  0,   0,   0);    // 'negro', Apagado
  
*/


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        IMPORTACION DE LIBRERIAS 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#include <Wire.h>                   // necesaria para comunicaciones I2C
#include <Adafruit_NeoPixel.h>      // para el control del led RGB de estado
#include <SoftwareSerial.h>         // creaccion de Puertos Serie Virtuales en 'pines convecionales'
#include <Temporizador_inopya.h>    // gestion sencilla de temposizadores no bloqueantes
#include <Universal_GPS_inopya.h>   // acceso sencillo a GPS Neo6/7/8 
#include <I2C_EEPROM_inopya.h>      // acceso sencillo a eeprom externa

// librerias inopya disponibles en -->  https://github.com/inopya 


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        PROTOTIPADO DE FUNCICIONES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void save_moment( void );
void load_moment(uint16_t puntero);
void printDateError( void );
void mostrar_menu_serial( void );
uint8_t leerComandosPuertoSerie( void );
float mostrar_tiempo_restante( void );
void mostrarColor(byte red, byte green, byte blue);
void init_MPU6050( void );
bool read_aceleracion( void );
void writeToWire(int device, byte address, byte val);
void readFromWire(int device, byte address, int num, byte _buff[]);
void read_all_sesion( void );
void read_one_sesion( uint8_t contador );
void print_error_size( void );
void listar_datos_de_sesion(uint16_t _puntero_lectura, uint16_t _size);
void mostrar_informacion_raw( void );
void soft_erase_eeprom( void );
void full_erase_eeprom( void );
void check_eeprom( void );


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DECLARACION DE PINES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#define PIN_Rx             4     // comunicacion con el Tx del GPS
#define PIN_Tx             5     // comunicacion con el Rx del GPS
#define LED_OnBoard       13
#define PIN_TIRA_LED       8     // patilla para la 'tira de leds'


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION DE DECLARACION DE CONSTANTES  Y  VARIABLES GLOBALES
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

#define CHIP_ID                   0x50 // la direccion de memoria del chip (24LC256) si A0, A1 y A2 estan a GND es 0x50
#define USER_DATA_SIZE               8 // para los incrementos de puntero al guardar datos
#define WAIT_SERIAL_AT_STARTUP   45000 // tiempo (ms) mostrando menu serial antes de iniciar el programa


/* (POS_MEM...) direccion  de memoria  --> (...)  tipo de dato que almacena y detalles */
#define POS_MEM_USER_DATA_START    300 // (struct) inicio de zona de datos de usuario (hasta aqui es la fat)
#define POS_MEM_READ_ONLY_REGISTER   0 // (1byte) registro para opcion de bloqueo de escritura por software
#define POS_MEM_SESION_NUMBER        1 // (1byte) registro para el numero de sesiones (ahora se permite multisesion)

#define POS_MEM_SESION_FAT_START     4 // (2bytes) Inicio de la FAT, cada posicion almacena el fin de una sesion de datos,
                                       // con una eeprom sin sesiones, contiene POS_MEM_USER_DATA_START


uint16_t puntero_eeprom = POS_MEM_USER_DATA_START;       // unsigned int, para disponer del rango 0-65535 
uint16_t puntero_fat = POS_MEM_SESION_FAT_START;         // unsigned int, para disponer del rango 0-65535 


/* creacion de un nuevo tipo de datos para contener las muestras */
struct struct_datos_gps { 
                            float latitud;
                            float longitud;
                         };

// ---> sin uso por ahora
struct struct_datos_acelerometro { 
                            int x;
                            int y;
                            int z;
                         };


bool FLAG_info_extra = false;   // Informa a la funcion de listado de sesiones 
                                // si se esta pidiendo solo informacion de la sesion o la sesion como tal.
uint8_t bank_index = 0;         // En caso de mas de 10 sesiones controla el bloque de estas que se esta mostrando
                                // de (1-10) de (10-20) .... etc.




/* VARIABLES Y CONSTANTES PARA EL MPU6050 */

bool FLAG_MPU6050_presente = false;      // ...en el setup detectaremos o no al MPU y su acelerometro
byte _buff[6];                           // buffer para datos
 
#define MPU6050_ADDR        0x68         // direccion i2c del MPU6050

/* Direcciones de algunos de los registros del MPU6050 */
#define GIRO_SENS           0x1B    //control de la sensibilidad del giroscopio
#define ACEL_SENS           0x1C    //control de la sensibilidad del acelerometro
#define WHO_AM_I            0x75    //almacena un ID conocido (0x34), Consultarlo nos ayuda a saber si esta conectado
#define PWR_MGMT            0x6B    //pormite configurar el modo de operacion: On, sleep...

#define DATA_AX_H           0x3B    //Aceleracion eje X MSB
#define DATA_AX_L           0x3C    //Aceleracion eje X LSB
#define DATA_AY_H           0x3D    //Aceleracion eje Y MSB
#define DATA_AY_L           0x3E    //Aceleracion eje Y LSB
#define DATA_AZ_H           0x3F    //Aceleracion eje Z MSB
#define DATA_AZ_L           0x40    //Aceleracion eje Z LSB

//Temperature en ºC = (valor del registro con signo)/340.0 + 36.53
#define DATA_TEMP_H         0x41    //temperatura MSB
#define DATA_TEMP_L         0x42    //temperatura LSB

#define DATA_GX_H           0x43    //Giro eje X MSB
#define DATA_GX_L           0x44    //Giro eje X LSB
#define DATA_GY_H           0x45    //Giro eje Y MSB
#define DATA_GY_L           0x46    //Giro eje Y LSB
#define DATA_GZ_H           0x47    //Giro eje Z MSB
#define DATA_GZ_L           0x48    //Giro eje Z LSB


/* ENUM COMANDOS SERIAL */
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
  CMD_INFO            =  8,    // mostrar informacion de sesiones previas
  CMD_SESION_1        =  9,    // seleccionde la sesion 1 (si banco 0) o la 11  (si banco 1)... etc
  CMD_SESION_2        = 10,    // seleccionde la sesion 2 (si banco 0) o la 12  (si banco 1)... etc
  CMD_SESION_3        = 11,    // seleccionde la sesion 3 (si banco 0) o la 13  (si banco 1)... etc
  CMD_SESION_4        = 12,    // seleccionde la sesion 4 (si banco 0) o la 14  (si banco 1)... etc
  CMD_SESION_5        = 13,    // seleccionde la sesion 5 (si banco 0) o la 15  (si banco 1)... etc
  CMD_SESION_6        = 14,    // seleccionde la sesion 6 (si banco 0) o la 16  (si banco 1)... etc
  CMD_SESION_7        = 15,    // seleccionde la sesion 7 (si banco 0) o la 17  (si banco 1)... etc
  CMD_SESION_8        = 16,    // seleccionde la sesion 8 (si banco 0) o la 18  (si banco 1)... etc
  CMD_SESION_9        = 17,    // seleccionde la sesion 9 (si banco 0) o la 19  (si banco 1)... etc
  CMD_SESION_10       = 18,    // seleccionde la sesion 10 (si banco 0) o la 20  (si banco 1)... etc
  CMD_EXTRA_INFO      = 19,    // muestra informacion detaalda de las sesiones:inicio eeprom, fecha, nºregistros
  CMD_NEXT_BANK       = 20,    // selecciona el siguiente banco de sesiones (si las hay).
  CMD_PREV_BANK       = 21,    // selecciona el banco de sesiones previo.
  CMD_SHOW_HELP       = 22,    // muestra de nuevo el menu ayuda.
  CMD_CHECK_ERASE     = 23,    // Comprobacion de contenido tras borrado completo de la eeprom.
  CMD_REC_TIME        = 24,    // calcula y muestra el tiempo disponible para grabaciones
  CMD_RAW_READ        = 25,    // Acceso a datos de la eeprom sin uso de la fat
  CMD_DISABLE_ACEL    = 26,    // En modo DEBUG/SERIAL permite habilitar y deshabilitar el acelerometro
};

                       
/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//        SECCION PARA DECLARACION DE OBJETOS
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

I2C_EEPROM_inopya memory_chip(CHIP_ID);   // Creamos un objeto del tipo i2c_eeprom (eeprom externa)

SoftwareSerial gpsPort(PIN_Rx, PIN_Tx);   // puerto serie para conectar el modulo GPS

Universal_GPS_inopya NEO_gps(&gpsPort);   // Creamos un objeto gps pero diciendole el puerto
                                          // de mosdo que se simplifica el acceso a la recogida  de datos

/* Crear el 'objeto' para controlar el led RGB */

#define TIPO_LED         NEO_GRB + NEO_KHZ800  //tipo de controlador de los led
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

  memory_chip.recPause(5); //establecer el tiempo de asentamiento de datos de la eeprom en 5 milisegundos
  
  /* Inicializar el led RGB */
  tiraLEDS.begin();
  tiraLEDS.setBrightness(20);
  mostrarColor(000, 000, 000);   //NEGRO, led apagado

  /* Iniciar el MPU6050 conectado al bus i2c */
  mostrarColor(40, 40, 000);   //
  init_MPU6050();
  delay(1000);
  mostrarColor(000, 000, 000);   //NEGRO, led apagado
  
  /* Iniciar un puerto serie sofware para comunicar con el GPS */
  NEO_gps.begin(9600);  //iniciamos el GPS a la velocidad standard

  /* Permitir el uso del led de la placa y apagarlo*/
  pinMode(LED_OnBoard, OUTPUT);
  digitalWrite(LED_OnBoard, LOW);
   

  /* ( DEBUG ) Zona de pruebas */
  if(DEBUG_MODE){
      Serial.print(F("GPS por defecto en modo: "));Serial.println(NEO_gps.get_mode()); 
      Serial.print(F("FLAG_MPU6050_presente: "));Serial.println(FLAG_MPU6050_presente);
      Serial.print(F("FLAG_disable_acelerometro: "));Serial.println(FLAG_disable_acelerometro);
      Serial.print(F("\n"));
      //NEO_gps.timeout(1000);      // por defecto la libreria escucha al GPS hasta un maximo de 1200 ms
      //NEO_gps.set_mode(1);        // cambiamos a modo 1 decodificar solo GNGGA/GPGGA.
      //uint8_t modo_actual = NEO_gps.get_mode();
      //Serial.print(F("Modo de trabajo actual: "));Serial.print(modo_actual);
  }
  

  /* <<<<  MOSTRAR EL MENU DE OPCIONES 'SERIAL' AL INICIAR >>>> */
  mostrar_menu_serial();    
  mostrarColor(0, 0, 50);   //Azul a la espera de comando y/o señal valida del gps

  tiempoWaitSerial.begin(WAIT_SERIAL_AT_STARTUP);
  bool FLAG_serial_forever = false;
  bool FLAG_blink_blue = false;
  uint8_t last_comando=CMD_WAIT;

  uint8_t sesiones_viejas;
  memory_chip.load(POS_MEM_SESION_NUMBER, sesiones_viejas);

  /* <<<<  ATENDER PUERTO 'SERIAL' Y GESTIONAR COMANDOS >>>> */
  while(tiempoWaitSerial.estado() == true || FLAG_serial_forever){
    uint8_t comando = leerComandosPuertoSerie(); 

    /* Evitar que se introduzca por error el m1smo comando dos veces seguidas */
    if( comando != last_comando ){ last_comando = comando; }
    else{ comando=CMD_WAIT; }
    
    /* Mostrar el numero de sesiones que hay grabadas */    
    if (comando == CMD_INFO){
      memory_chip.load(POS_MEM_SESION_NUMBER, sesiones_viejas); 
      Serial.print(F("LA EEPROM CONTIENE "));
      Serial.print(sesiones_viejas);
      Serial.println(F(" SESIONES\n"));
      mostrar_tiempo_restante();
    }

    /* Permanecer indefinidamente en el monitor serie */    
    if( comando == CMD_SERIAL_FOREVER ){ 
      FLAG_serial_forever = true;
      parpadeoLED.stop();           // temporizador para mostrar un parpadeo  el el led Direccionable
      parpadeoLED.begin(500);       // que idique que estamos en espera infinita
      Serial.println(F("MONITOR SERIAL 'FOREVER' (Pulse Q para Salir)"));
    }

    /* Gestion de parpadeo para indicar que permanecemos indefinidamente en el monitor serie */ 
    if ( FLAG_serial_forever ){ 
      if( parpadeoLED.estado() == false ){
        parpadeoLED.begin(500);
        FLAG_blink_blue=!FLAG_blink_blue;
        if( FLAG_blink_blue ){ mostrarColor(0, 0, 50); }  // azul
        else{ mostrarColor(0, 0, 0); }                    // apagado
      }
    }
        
    /* Salir del monitor serie y comenzar a tomar datos */    
    if( comando == CMD_BREAK ){ 
      tiempoWaitSerial.stop();
      FLAG_serial_forever = false;  //por si estabamos en espera infinita
      Serial.println(F("\n<< BREAK SERIAL >>\n"));
    }

    /* Mostrar menu de ayuda */    
    if( comando == CMD_SHOW_HELP ){ 
      FLAG_serial_forever = true; 
      mostrar_menu_serial();
    }

    /* Mostrar tiempo de grabacion disponible */    
    if( comando == CMD_REC_TIME ){ 
      FLAG_serial_forever = true; 
      mostrar_tiempo_restante();
    }
    
    /* Mostrar todo el contenido de la eeprom en modo RAW */    
    if( comando == CMD_RAW_READ ){ 
      FLAG_serial_forever = true; 
      mostrar_informacion_raw();
    }
    /* ( DEBUG ) Activar desactivar el uso del acelerometro */    
    if( comando == CMD_DISABLE_ACEL ){ 
      FLAG_disable_acelerometro = !FLAG_disable_acelerometro; 
      Serial.print(F("\n\t *** ACELEROMETRO: "));
      FLAG_disable_acelerometro ? Serial.print(F("Desactivado")) : Serial.print(F("Activo"));
      Serial.print(F(" ***\n"));
    }
    
    /* Borrado completo de la eeprom */    
    if( comando == CMD_FULL_ERASE ){
      FLAG_serial_forever = true;
      Serial.println(F("\nInicio FULL ERASE")); 
      full_erase_eeprom();
      Serial.println(F("FULL ERASE -> OK"));
      Serial.println(F("\ninicio CHECK ZERO"));
      check_eeprom();
      Serial.println(F("fin CHECK ZERO"));
    }

    /* Check del borrado completo de la eeprom (debug) */    
    if( comando == CMD_CHECK_ERASE ){ 
      Serial.println(F("TEST READ ZERO START"));
      FLAG_serial_forever = true; 
      check_eeprom();
      Serial.println(F("TEST READ ZERO END"));
    }
    
    /* Borrado parcial de la eeprom (solo borra la FAT) */    
    if( comando == CMD_REG_ERASE ){ 
      FLAG_serial_forever = true;
      soft_erase_eeprom();
      Serial.println(F("SOFT ERASE -> OK"));
    }
        
    /* activar el uso de la eeprom */    
    if( comando == CMD_WRITE_ENABLE ){  //sin uso por ahora
      //FLAG_uso_eeprom = true;
      //memory_chip.write(POS_MEM_READ_ONLY_REGISTER,0);
      Serial.println(F("EEPROM Activada (DEMO)"));
    }
    
    /* desactivar el uso de la eeprom */ 
    if (comando == CMD_LOCK_EEPROM){  //sin uso por ahora
      //FLAG_uso_eeprom = false;
      //memory_chip.write(POS_MEM_READ_ONLY_REGISTER,101);
      Serial.println(F("EEPROM Desactivada(DEMO)"));
    }

    /* Listado de los datos de todas las sesiones GPS almacenadas en eeprom */ 
    if( comando == CMD_LISTAR_DATOS ){
      FLAG_info_extra = false;
      mostrarColor(90, 45, 0);    // NARANJA, listando datos
      read_all_sesion();
      mostrarColor(0, 50, 0);     // Verde. Reposo infinito, la eeprom queda preservada
      while(true);                // si lo preferimos podemos activar la bandera 'FLAG_serial_forever'
                                  // que nos permitiría salir pulsando la letra Q
    } 

    /* Listar una sesion GPS en concreto */ 
    if( comando >= CMD_SESION_1 && comando <= CMD_SESION_10 ){
      FLAG_serial_forever = true;
      FLAG_info_extra = false;
      Serial.print(F("\n>> Leyendo sesion "));
      Serial.println( 1 + comando + bank_index - CMD_SESION_1 );
      read_one_sesion( 1 + comando + bank_index - CMD_SESION_1 );  
    }

    /* Mostrar detalles de las sesiones grabadas */ 
    if( comando == CMD_EXTRA_INFO ){
      Serial.println(F("MOSTRANDO INFORMACION EXTENDIDA DE SESIONES "));  
      FLAG_info_extra = true;
      FLAG_serial_forever = true;
      read_all_sesion(); 
      mostrar_tiempo_restante();
    }

    /* Seleccionar las proximas 10 sesiones */ 
    if( comando == CMD_NEXT_BANK ){
      bank_index+=10;
      if(bank_index>sesiones_viejas){ bank_index-=10; }
      Serial.print(F("\nACTIVAS SESIONES DE "));
      Serial.print(bank_index+1);Serial.print(F(" A "));Serial.println(bank_index+10);
    }

    /* Seleccionar las 10 sesiones anteriores */ 
    if( comando == CMD_PREV_BANK ){
      if( bank_index>=10 ){ bank_index-=10; }
      Serial.print(F("\nACTIVAS SESIONES DE "));
      Serial.print(bank_index+1);Serial.print(F(" A "));Serial.println(bank_index+10); 
    }  
  }

  /* No inicicar si no hay espacio suficiente, evitando generar sesiones  basura en la FAT */
  if( mostrar_tiempo_restante()<0.02 ){
    mostrarColor(55, 000, 55);   // color rosa fijo para indicar que se ha agotado la eeprom
    Serial.print(F("MEMORIA LLENA\n >> REINICIE GPS y BORRE DATOS"));
    while(true); //pausa infinita
  }
  
  /* Esperar a tener datos validos del GPS para iniciar la grabacion en EEPROM (solo validamos latitud) */
  Serial.print(F("\nESPERANDO GPS"));
  mostrarColor(50, 0, 0);   //rojo, comienza el proceso de espra/toma de datos

  uint8_t contador=0;
  bool FLAG_gps_data_ok = false;
  while( !FLAG_gps_data_ok){
    NEO_gps.get();
    if( NEO_gps.latitud != 0.00 && NEO_gps.longitud != 0.00 && NEO_gps.year>=2020 ){
      if( NEO_gps.mes>0 && NEO_gps.mes<=12 ){
        if( NEO_gps.dia>0 && NEO_gps.dia<=31 ){
          FLAG_gps_data_ok = true;
        }
      }
    }
    
    contador++; 
    if(DEBUG_MODE){
      Serial.print(F(" > ESPERANDO GPS...  "));
      Serial.print(NEO_gps.year);
      Serial.print(F(","));
      Serial.println(NEO_gps.latitud,6);
    }
    else{ 
      Serial.print(F("."));
      if( contador>=20 ){
        contador=0;
        Serial.println();
      } 
    }
    delay(1000);
  }

  /* En este punto tenemos unos datos GPS validos */
  Serial.println(F("\nInicio Grabacion\n"));  

  /* Determinar el numero de sesior y establecer el puntero de grabacion para los datos GPS */
  uint8_t numero_sesiones = 0;  //inicializado en cero, ahora lo actualizamos con la informacion de la eeprom

  /* leemos el contador de sesiones para averiguar si hay sesiones anteriores */
  memory_chip.load(POS_MEM_SESION_NUMBER, numero_sesiones);

  /* si no existen sesiones previas... */
  if( numero_sesiones==0){
	  puntero_fat = POS_MEM_SESION_FAT_START;         // primera posicion de la FAT
    puntero_eeprom = POS_MEM_USER_DATA_START;       // primera posicion para datos de usuario
  }
  else{
    /* si ya hay sesiones, localizar donde termina la ultima sesion grabada para determianr el puntero de trabajo */
    memory_chip.load((2*(numero_sesiones-1))+POS_MEM_SESION_FAT_START, puntero_eeprom);
    
    /* actualizar el puntero FAT para la nueva sesion */
	  puntero_fat=POS_MEM_SESION_FAT_START+(2*numero_sesiones);

    /* Comprobar si aun hay espacio en la FAT */
    if( puntero_fat>=POS_MEM_USER_DATA_START ){
      mostrarColor(55, 000, 55);    // color rosa fijo para indicar que se hay eeprom disponible
      Serial.println(F("\nSin espacio FAT. Memoria no disponible\n"));  
      while(true);                  // Se ha acabado la FAT, pausa infinita, por 'memoria llena'
    }
  }

  //recordemos que en este punto ya tenemos unos datos GPS validos
  
  /* incrementamos el numero de sesiones y actualizamos dicho registro */
  numero_sesiones++;
  memory_chip.save(POS_MEM_SESION_NUMBER, numero_sesiones); 
  
  //almacenamos como fin de la sesion recien iniciada el (puntero_fin_sesion_anterior + 2*datos_de_usuario)
  //ya que en el momento de iniciar una sesion grabamos (fecha/hora)  y un (primer punto),
  //lo que ocupa dos datos de usuario, uno para la fecha/hora y otro el punto gps como tal  
  memory_chip.save(puntero_fat, puntero_eeprom+(2*USER_DATA_SIZE)); 
   
  //el puntero FAT está ya incrementado, ahora escribimos esos dos datos...
  /* Grabamos como primer dato: la (fecha y hora) de la sesion */
  save_moment();
  
  /* segundo dato: (Un punto GPS) */
  struct_datos_gps puntoRecorrido;
  puntoRecorrido.latitud = NEO_gps.latitud;
  puntoRecorrido.longitud = NEO_gps.longitud;
  memory_chip.save(puntero_eeprom, puntoRecorrido);
  
  //incrementar el puntero de trabajo  
  puntero_eeprom+=USER_DATA_SIZE;

  if(DEBUG_MODE){
      Serial.println(F("Fecha y Hora de sesion guardados: "));
      Serial.print(NEO_gps.year);Serial.print(F("/"));
      Serial.print(NEO_gps.mes);Serial.print(F("/"));
      Serial.print(NEO_gps.dia);Serial.print(F("  "));
      Serial.print(NEO_gps.hora);Serial.print(F(":"));
      Serial.print(NEO_gps.minuto);Serial.print(F(":"));
      Serial.println(NEO_gps.segundo);
           
      Serial.print(F("\nIniciando sesion nº "));
      Serial.println(numero_sesiones);
  }
  
  delay(1500); //para darle dramatismo a la escena

  /* Opcionalmente podemos desactivar el puerto Serie durante el resto de la sesion */
  //Serial.end();
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

  /* ( DEBUG ) Parpadeo del led de la placa para mostrar que el sistema está funcionando */
  if(DEBUG_MODE){
      if(parpadeoLED.estado()==false){    //lo usamos en setup para parpadear el led azul, lo reusamos aqui
        parpadeoLED.begin(250);           //nuestro reloj no obliga, impide que se haga antes de ese tiempo
        digitalWrite(LED_OnBoard, !digitalRead(LED_OnBoard));
      }
  }



  /* Chequeo del temporizador que nos marca el intervalo entre toma de muestras */
  if( updateGPS.estado()==false ){
	  updateGPS.begin(INTERVALO_MUESTRAS);  //volver a reactivar el temporizador.
    NEO_gps.get();                        //actualizar la informacion desde el gps

    /* ( DEBUG ) Podemos forzar grabacion de puntos GPS aun estando quietos ( DEBUG ) */
    if( FLAG_disable_acelerometro ){
        FLAG_enMovimiento = true;
    }  

    /* Condiciones para grabacion de puntos GPS */
    //solo si hay movimiento, los datos de gps son correctos y queda memoria disponible
    if( FLAG_enMovimiento && NEO_gps.latitud != 0.0  && puntero_eeprom<EEPROM_SIZE ){   
      if(DEBUG_MODE){
            /* Mostrar Contador de muestras */
            if(contador < 10){ Serial.print(F("0")); }
            if(contador < 100){ Serial.print(F("0")); }
            if(contador < 1000){ Serial.print(F("0")); }
            if(contador < 10000){ Serial.print(F("0")); }
            Serial.print(contador++);
            //Serial.print(F(",\t"));
            Serial.print(F(",")); 

            /* Mostrar año */
            Serial.print(NEO_gps.year);
            Serial.print(F(",")); 
            
            /* Mostrar Posicion */
            Serial.print(NEO_gps.latitud,6); 
            Serial.print(F(",")); 
            Serial.print(NEO_gps.longitud,6);
            Serial.println();
      }
  
      /* Guardar LATITUD  y LONGITUD */
      struct_datos_gps puntoRecorrido;
      puntoRecorrido.latitud = NEO_gps.latitud;
      puntoRecorrido.longitud = NEO_gps.longitud;
      memory_chip.save(puntero_eeprom, puntoRecorrido); 
	    
	    /* incrementar el puntero de rabajo */
	    puntero_eeprom+=USER_DATA_SIZE; 

      /* Indicar con un parpadeo que se ha grabado un nuevo punto de la ruta */
      mostrarColor(55, 000, 55);    // rosa/morado
      delay(100);                   // para dar tiempo a ver el parpadeo

      /* ( DEBUG) si esta activado el debug, indicar por Serial la grabacion del punto de ruta */
      if(DEBUG_MODE){ Serial.println(F("punto GPS salvado!"));}
      
	    /* cada 12 datos (12x5 = 1 minuto si se respeta INTERVALO_MUESTRAS=5000 ms )... */
      if(contador%12==0){
		    /* actualizamos la FAT con el puntero_eeprom que tiene la sesion actualmente (es decir donde acaba) */
        memory_chip.save(puntero_fat, puntero_eeprom);
        
        /* ( DEBUG) si esta activado el debug, indicar por Serial la actualizacion de la FAT */ 		  
        if(DEBUG_MODE){ Serial.println(F("\n\t  *** FAT actualizada *** \n")); }
      }
    }
    /* Si hay error y no se puede guardar el punto GPS, ver que hacer... */
    else{
      /* Si el problema es la falta de memoria, entramos en una pausa infinita */
      if( puntero_eeprom>=EEPROM_SIZE ){
        memory_chip.save(puntero_fat, puntero_eeprom);  //actualizamos la fat con la info de la ultima sesion
        mostrarColor(55, 000, 55);                      // color rosa fijo para indicar que se ha agotado la eeprom

        if(DEBUG_MODE){ Serial.println(F("\n\t  *** MEMORIA AGOTADA *** \n")); }
        while(true); //pausa infinita
      }
      /* Si es error de GPS, cancelar el temporizador para volver a 'hablar' con el GPS en el proxino ciclo */
      updateGPS.stop();  // 
    }
    
    /* ( DEBUG) si esta activado el debug, mostrar por Serial la bandera de movimiento */     
    if(DEBUG_MODE){ FLAG_enMovimiento ? Serial.println(F("RUNNING...")) : Serial.println(F("STOP")); }
    
    mostrarColor(000, 000, 000);   // NEGRO, led apagado
  }

  /* OJO: Solo si hay acelerometro (MPU6050) procedemos a tener control de movimiento */
  if( FLAG_MPU6050_presente && lecturaAcelerometro.estado()==false ){
    lecturaAcelerometro.begin(100);
    /* Leer acelerometro: x, y, z y avaluar si hay movimiento */
    if( read_aceleracion()==false ){ contadorReposo++; }
    else{ contadorReposo--; }
    if( contadorReposo<0 ) { contadorReposo=0; }
    if( contadorReposo>3 ) { contadorReposo=3; }
    if( contadorReposo <2 ) { FLAG_enMovimiento = true; }
    else{ FLAG_enMovimiento = false; }
  }
  
  /* Por seguridad, si no se detecta el acelerometro (MPU6050), forzamos la bandera de movimiento */
  if(!FLAG_MPU6050_presente) { FLAG_enMovimiento = true; }
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
  /*
   * Guarda el regitro de fecha hora al iniciar una sesion de toma de muestras.
  */
  
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
}


/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   RECUPERAR DE EEPROM LA FECHA Y HORA DE INICIO DE UNA SESION
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void load_moment(uint16_t puntero)
{
  /*
  * Recupera el regitro de fecha hora al listar una sesion de datos.
  */
  uint16_t puntero_memoria = puntero;
  uint8_t dato;
  bool FLAG_error_fecha = false;
  /* cargar DIA */
  memory_chip.load(puntero_memoria, dato); 
  if(dato < 10){ Serial.print(F("0")); }
  if( dato>31 ){ FLAG_error_fecha=true; }
  Serial.print(dato); Serial.print(F("/"));
  puntero_memoria+=1;

  /* cargar MES */
  memory_chip.load(puntero_memoria, dato);
  if(dato < 10){ Serial.print(F("0")); }
  if( dato>12 ){ FLAG_error_fecha=true; }
  Serial.print(dato); Serial.print(F("/"));
  puntero_memoria+=1;

  /* cargar AÑO */
  uint16_t dato2;
  memory_chip.load(puntero_memoria, dato2);
  if( dato2<2020 ){ FLAG_error_fecha=true; }
  if( dato2>2100 ){ FLAG_error_fecha=true; }
  Serial.print(dato2); Serial.print(F("\t"));
  puntero_memoria+=2;
  
  /* cargar HORAS */
  memory_chip.load(puntero_memoria, dato);
  if( dato>23 ){ FLAG_error_fecha=true; }
  if(dato < 10){ Serial.print(F("0")); }
  Serial.print(dato); Serial.print(F(":"));
  puntero_memoria+=1;

  /* cargar MINUTOS */
  memory_chip.load(puntero_memoria, dato);
  if( dato>59 ){ FLAG_error_fecha=true; }
  if(dato < 10){ Serial.print(F("0")); }
  Serial.print(dato); Serial.print(F(":"));
  puntero_memoria+=1;

  /* cargar SEGUNDOS */
  memory_chip.load(puntero_memoria, dato);
  if( dato>59 ){ FLAG_error_fecha=true; }
  if(dato < 10){ Serial.print(F("0")); }
  Serial.println(dato);
  puntero_memoria+=1;
  if( FLAG_error_fecha ){ printDateError(); }
}



//========================================================
//   MENSAJE DE ERROR RELACIONADO CON FECHA Y HORA
//========================================================

void printDateError()
{
  /*
   * Mensaje de advertencia de hora o fecha no validas, lo que podria ser debido a 
   * una corrupcion de los datos de esa sesion. 
  */
  Serial.println(F("Fecha/Hora de sesion no validas\nPosible error en los datos\n\n"));
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//      COMUNICACIONES (LECTURA DE CARACTERES POR PUERTO SERIE) 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  MOSTRAR MENU DE AYUDA PUERTO SERIE
//========================================================

void mostrar_menu_serial()
{
  /*
   * Muestra el menu serial al iniciar, para permitir el acceso a 
   * operaciones sobre los datos de las sesiones guardadas 
   * y otras operaciones como formateo completo de la eeprom, 
   * o el borrado simple de la FAT para liberar el espacio.
   * Permite el acceso a otras opciones de debug.
  */
  Serial.print(F("\n >> Esperando orden ("));
  Serial.print(WAIT_SERIAL_AT_STARTUP/1000);
  Serial.println(F(" segundos):"));
  Serial.println(F("\t'o' MOSTRAR este menu de forma indefinida"));
  Serial.println(F("\t'q' SALIR de este menu y comenzar a grabar"));
  Serial.println(F("\t'i' INFORMACION Breve de sesiones"));
  Serial.println(F("\t'x' Informacion EXTENDIDA de sesiones"));
  Serial.println(F("\t's' borra la FAT para eliminar sesiones"));
  Serial.println(F("\t'f' BORRA por completo la eeprom"));
  Serial.println(F("\t'numeros: 1,2...9,0', listar una sesion (de la 1-10)"));
  Serial.println(F("\t'+' selecciona bloque SIGUIENTE de 10 sesiones"));
  Serial.println(F("\t'-' selecciona bloque PREVIO de 10 sesiones"));
  Serial.println(F("\t'L' LISTAR todos los datos grabados"));
  //Serial.println(F("\t'v' DEBUG - Test Read Zero"));
  Serial.println(F("\t'r' DEBUG - Mostrar RAW"));
  Serial.println(F("\t'a' DEBUG - Activa/Desactiva el Acelerometro"));
  Serial.println(F("\t't' mostrar Tiempo restante"));
  Serial.println(F("\t'h' Mostrar este menu\n\n"));
}



//========================================================
//  ATENDER COMANDOS PUERTO SERIE
//========================================================

uint8_t leerComandosPuertoSerie() 
{
  /*
   * Atiende el puerto serie en busca de ordenes de usuario. Se ejecuta en cada reinicio
   * durante un periodo de tiempo determinado en las variables de sistema (aprox. 30 segundos)
   * Para permitir el acceso a los datos guardados cuando estamos conectados al PC.
  */
  
  char orden_recibida = ' ';
   while(Serial.available()) {
    orden_recibida = Serial.read();
    
    if(orden_recibida == 'e' || orden_recibida == 'E'){  //desbloqueamos acceso a la eeprom (solo DEBUG)
      return CMD_WRITE_ENABLE;
    }

    if(orden_recibida == 'h' || orden_recibida == 'H'){  //volver a mostrar el menu de ayuda por pantalla
      return CMD_SHOW_HELP;
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
    
    if(orden_recibida == 'i' || orden_recibida == 'I'){  // mostrar informacion simple
      return CMD_INFO;
    }

    if(orden_recibida == 'x' || orden_recibida == 'X'){  // mostrar informacion extra
      return CMD_EXTRA_INFO;
    }

    if(orden_recibida == 'r' || orden_recibida == 'R'){  // mostrar informacion RAW, para ver sesiones erroneas
      return CMD_RAW_READ;
    }
    
    if(orden_recibida == 'a' || orden_recibida == 'A'){  // mostrar informacion RAW, para ver sesiones erroneas
      return CMD_DISABLE_ACEL;
    }

    if( orden_recibida == 'v' || orden_recibida == 'V'){
      return CMD_CHECK_ERASE; 
    }
    
    if( orden_recibida == 't' || orden_recibida == 'T'){ 
      return CMD_REC_TIME; 
    } 
    
    if( orden_recibida == '1' ){ 
      return CMD_SESION_1; 
    }
    if( orden_recibida == '2' ){ 
      return CMD_SESION_2; 
    }
    if( orden_recibida == '3' ){ 
      return CMD_SESION_3; 
    }   
    if( orden_recibida == '4' ){ 
      return CMD_SESION_4; 
    } 
    if( orden_recibida == '5' ){ 
      return CMD_SESION_5; 
    }   
    if( orden_recibida == '6' ){ 
      return CMD_SESION_6; 
    } 
    if( orden_recibida == '7' ){ 
      return CMD_SESION_7; 
    }   
    if( orden_recibida == '8' ){ 
      return CMD_SESION_8; 
    }
    if( orden_recibida == '9' ){ 
      return CMD_SESION_9; 
    }   
    if( orden_recibida == '0' ){ 
      return CMD_SESION_10; 
    }
    if( orden_recibida == '+' ){ 
      return CMD_NEXT_BANK; 
    }
    if( orden_recibida == '-' ){ 
      return CMD_PREV_BANK; 
    }
  }
  return CMD_WAIT;
}


//========================================================
//  MOSTRAR TIEMPO DE GRABACION DISPONIBLE
//========================================================

float mostrar_tiempo_restante()
{
  /*
   * Funcion que calcula el tiempo restante de grabacion de nuestra eeprom
   * teniendo en cuenta el valor "INTERVALO_MUESTRAS" y no como una mera representacion
   * de la cantidad de eeprom libre.
   */
  uint8_t sesiones_grabadas;
  memory_chip.load(POS_MEM_SESION_NUMBER, sesiones_grabadas);
  uint16_t last_pointer;
  memory_chip.load(POS_MEM_SESION_FAT_START+(2*(sesiones_grabadas-1)), last_pointer);

  float recTime = (EEPROM_SIZE-last_pointer)/USER_DATA_SIZE; 
  recTime *= INTERVALO_MUESTRAS;
  recTime=recTime/3600000.0;
  Serial.print(F("Tiempo de grabacion disponible: "));
  Serial.print(recTime,2);
  Serial.println(F(" Horas\n"));
  return recTime;
}

  
/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    CONTROL DE LA TIRA DE LEDS (ireccionable) PARA CREAR LOS EFECTOS DE COLOR
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

//========================================================
//  MOSTRAR UN COLOR (tira completa)
//========================================================

void mostrarColor(byte red, byte green, byte blue)
{
  /*
   * Mostrar un color en una tira de led rgb con led direccionables.
   * En nuestro caso usamos un unico led (en lugar de una tira de leds) 
   * 'Manias' de Inopya (yo) que uso este tipo de leds de forma individual como led RGB
   * de modo que solo ocupo una linea de datos y ademas el consumo de dichos leds
   * no tiene que ser soportado por los pines de datos, si no que se hace 
   * directamente desde Vcc.
   */
  for (int pixel=0; pixel< tiraLEDS.numPixels(); pixel++){
    tiraLEDS.setPixelColor(pixel, tiraLEDS.Color(red, green, blue)); //color unico 
  } 
  tiraLEDS.show();
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//    CONTROL DEL MPU6050 SIN LIBRERIA
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void init_MPU6050()
{
  /*
   * Inicializar el MPU6050 configurando manualmente sus regitros. 
   * Consultar las hojas de caracteristicas del fabricante para mas informacion.
  */
  
  /* Iniciar el MPU6050 conectado al bus i2c */
  Wire.begin();
  /* buscar el ID del MPU6050 para saber si esta conectado */  
  readFromWire(MPU6050_ADDR, WHO_AM_I, 1, _buff);
  _buff[0] = _buff[0]>>1;
  _buff[0]&=B00111111;
  
  //Serial.print(F("ID acelerometro: "));Serial.println(_buff[0]);
  
  if( _buff[0]==0x34 ){ 
    FLAG_MPU6050_presente = true;
    Serial.println(F("MPU6050 ok"));

    /* activar fuente de reloj */
    readFromWire(MPU6050_ADDR, PWR_MGMT, 1, _buff);
    _buff[0]&=B11111000;
    _buff[0]|=B00000001;
    writeToWire(MPU6050_ADDR, PWR_MGMT, _buff[0]); 
  
    /* establecer la sensibilidad del acelerometro bits 3 y 4 */  
    readFromWire(MPU6050_ADDR, ACEL_SENS, 1, _buff);
    _buff[0]&= B11100111;   //poner a cero los dos bits en los que se guarda la sensibilidad del acelerometro
    _buff[0]|= B00000000;   //Poner sensibilidad en (Binario):  (00)+-2G, (01)+-4G,(10)+-8G, (11)+-16G,
    writeToWire(MPU6050_ADDR, ACEL_SENS, _buff[0]); 

    /* establecer la sensibilidad del giroscopio bits 3 y 4 */
    readFromWire(MPU6050_ADDR, GIRO_SENS, 1, _buff);
    _buff[0]&= B11100111;   //poner a cero los dos bits en los que se guarda la sensibilidad del giroscopio
    _buff[0]|= B00000000;   //Poner sensibilidad en:  (00)+-250%, (01)+-500%,(02)+-1000%, (03)+-2000%,
    writeToWire(MPU6050_ADDR, GIRO_SENS, _buff[0]);
     
    /* Activar el MPU6050 forzando el modo sleep a false bit6=0 */
    readFromWire(MPU6050_ADDR, PWR_MGMT, 1, _buff);
    _buff[0]&=B10111111;
    writeToWire(MPU6050_ADDR, PWR_MGMT, _buff[0]); 

    /* leer la temperatura desde el MPU (chorrada) */ 
    readFromWire(MPU6050_ADDR, DATA_TEMP_H, 2, _buff);
    float temperatura = _buff[0]| _buff[1];
    temperatura = temperatura/340.0 + 36.53;
    //chorrada para pruebas: este MPU contiene internamente un sensor de temperatura
    Serial.print(F("Temperatura: ")); Serial.println(temperatura-5);  //este sensor parece dar unos 5 grados de mas
  }
  else{
    Serial.println(F("MPU6050 No presente!"));
    FLAG_MPU6050_presente = false;  //para forzar que se grabe gps a un no teniendo constania de movimiento
  }  
}



//========================================================
//  DETECTAR MOVIMIENTO CON EL ACELEROMETRO
//========================================================
 
bool read_aceleracion() 
{
  /*
   * Lectura de los datos de aceleracion del MPU6050 accediendo a directamente a sus registros
   * sin usar librerias. Consultar las hojas de caracteristicas del fabricante para mas detalles.
   * Trata de estimar si hay movimineto y devuelve un valor booleano Si/No
   */
  
  uint16_t UMBRAL = 700; // cantidad de aceleracion necesaria para detectar movimiento
  
  static int old_x;
  static int old_y;
  static int old_z;
  
  bool FLAG_movimiento = false;
  
  /* Leer los valores del registros del acelerometro */
  uint8_t numBytesToRead = 6;
  //Serial.println(F(" -- ACELEROMETRO --"));
  readFromWire(MPU6050_ADDR, DATA_AX_H, numBytesToRead, _buff);

  /* Leer los valores del registros del giroscopio */
  //Serial.print(F("GIROSCOPIO: "));
  //readFromWire(MPU6050_ADDR, DATA_GX_H, numBytesToRead, _buff);
 
  /* Convertir a sus correspondientes ejes (Cada eje tiene 16 bits, en 2 Bytes MSB-LSB) */
  int x = (((int)_buff[0]) << 8) | _buff[1];   
  int y = (((int)_buff[2]) << 8) | _buff[3];
  int z = (((int)_buff[4]) << 8) | _buff[5];

  /* Realizar calculos para decidr si se ha efectuado desplazamiento */
  if( abs(old_x-x)>UMBRAL || abs(old_y-y)>UMBRAL || abs(old_z-z)>UMBRAL ){ FLAG_movimiento = true; }
  

  //  ---->>  INICIO print DEBUG
  if(DEBUG_MODE_ACELETOMETRO){
      //Serial.print("x: "); Serial.print( x );
      //Serial.print(" y: "); Serial.print( y );
      //Serial.print(" z: "); Serial.println( z );
      //Serial.println();
    
      Serial.print("Dx: "); Serial.print( abs(old_x-x) );
      Serial.print(" Dy: "); Serial.print( abs(old_y-y) );
      Serial.print(" Dz: "); Serial.println( abs(old_z-z) );
    
      uint16_t Aceleracion_MAX = abs(old_x-x);
      if( abs(old_y-y) > Aceleracion_MAX ) { Aceleracion_MAX=abs(old_y-y); }
      if( abs(old_z-z) > Aceleracion_MAX ) { Aceleracion_MAX=abs(old_z-z); }
        
      Serial.print(Aceleracion_MAX);
      if( FLAG_movimiento == true ) { Serial.println(F("   >> Movimiento")); }
      else{ Serial.println(); }
  }
  //  <<----  FIN print DEBUG

  /* Reservar los valores para la proxima comparacion */
  old_x=x;
  old_y=y;
  old_z=z;
  
  return FLAG_movimiento;
  
}


//========================================================
//  Funcion auxiliar de escritura en el bus i2c
//========================================================

void writeToWire(int device, byte address, byte val) 
{
 /*
  * Funcion de "abstraccion" para escritura en el bus I2C
  */
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission(); 
}



//========================================================
//  Funcion auxiliar de lectura en el bus i2c
//========================================================

void readFromWire(int device, byte address, int num, byte _buff[]) 
{
 /*
  * Funcion de "abstraccion" para lectura de informacion en el bus I2C
  */
  
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

void read_all_sesion()
{
  /*
  * Funcion que itera sobre todas las sesiones de datos para mostrarlas
  */
  uint8_t numero_sesiones;
  uint8_t contador=1;
  
  memory_chip.load(POS_MEM_SESION_NUMBER, numero_sesiones); 
  
  if(numero_sesiones==0){
    Serial.println(F("\n\nNO HAY SESIONES, EEPROM VACIA\n\n"));
    return;
  }
  else{
    while( contador <= numero_sesiones ){ 
      read_one_sesion( contador );
      contador++;
    }
  }
}


//========================================================
//  RECUPERAR UNA SESION EN CONCRETO
//========================================================

void read_one_sesion( uint8_t n_sesion )
{
 /*
  * localiza el espacio ocupado por una sesion de datos en concreto 
  * y entrega dicha informacion a la funcion de listado de datos.
  */

  /* localiza los punteros y el tamaño de una sesion para pasarselos a la funcion de listado */
  uint16_t prt_inicio;
  uint16_t ptr_final;
  uint16_t sesion_size;
  uint8_t numero_sesiones;
  
  memory_chip.load(POS_MEM_SESION_NUMBER, numero_sesiones); 
  
  if( numero_sesiones==0 ){
    Serial.println(F("NO EXISTEN SESIONES\n\n"));
    return;
  }

  if( n_sesion>numero_sesiones ){
    Serial.println(F("\n\nSESION NO VALIDA (AUN NO EXISTE)\n\n"));
    return; 
  }
  
  Serial.println(F("\n***************************"));
  Serial.print(F(" Sesion: "));Serial.println(n_sesion);

  if(n_sesion==1 ){
    memory_chip.load(POS_MEM_SESION_FAT_START, prt_inicio);  // uso de un puntero temporal
    sesion_size = prt_inicio-POS_MEM_USER_DATA_START;  
    sesion_size = sesion_size / USER_DATA_SIZE; 
    if(sesion_size==0){ print_error_size(); return; }
    listar_datos_de_sesion(POS_MEM_USER_DATA_START, sesion_size); //listar_datos_de_sesion(puntero_inicio, tamaño)
    return;
  }
  /* para el resto de sesiones... */
  else{
  	/* el final de la sesion que nos piden esta indicado en esta posicion: */
  	ptr_final= memory_chip.load( 2*(n_sesion-1)+POS_MEM_SESION_FAT_START, ptr_final ); 
  
  	/* para saber el tamaño, debemos saber donde empieza esta sesion = donde acaba la sesion anterior: */
  	prt_inicio= memory_chip.load( 2*(n_sesion-2)+POS_MEM_SESION_FAT_START, prt_inicio ); 
    
  	/* la diferencia entre esos dos punteros será el tamaño de la sesion: */
    sesion_size = ptr_final-prt_inicio; 
    sesion_size = sesion_size / USER_DATA_SIZE; 
    if(sesion_size==0){ print_error_size(); return; }
    listar_datos_de_sesion(prt_inicio, sesion_size); //listar_datos_de_sesion(puntero_inicio, tamaño)
    return;
  }
}


//========================================================
//  MOSTAR ERROR SESION NO VALIDA
//========================================================
void print_error_size()
{ 
 /*
  * Mostar mensaje de sesion no valida cuando algun problema en sus punteros 
  * haga que el tamaño a mostrar no sea un valor logico.
  */	
  Serial.println(F("\nDEBUG >> ESTA SESION NO ES VALIDA, la saltamos\n")); 
}


//========================================================
//  LISTAR UNA DE LAS SESIONES ( MULTISESION )
//========================================================

void listar_datos_de_sesion(uint16_t _puntero_lectura, uint16_t _size)
{
 /*
  * Recibe la informacion que define la localizacion en eeprom de una sesion de datos
  * y lista dicha informacion interpretando la informacion de fecha/hora y puntos de recorrido
  */  
  Serial.print(F(" Puntero: "));Serial.println(_puntero_lectura);
  if( FLAG_info_extra==true){
    Serial.print(F(" Contiene: "));
    Serial.print(_size);
    Serial.println(F(" registros"));
    Serial.print(F(" Fecha: "));
  }
  else{ 
    Serial.print(F(" Mostrando "));
    Serial.print(_size);
    Serial.println(F(" registros...\n"));
  }
  /* Leer los datos de sesion (Fecha/Hora) */
  load_moment(_puntero_lectura);
  
  if( FLAG_info_extra==true){ return; } 
  
  _puntero_lectura+=USER_DATA_SIZE;  //incrementamos el puntero tras leer e interpretar la fecha/hora


  /* recuperar los puntos del recorrido */
  uint16_t contador=0;

  while( contador < (_size-1) ){  //ya que el primero sera el de fecha y hora
    struct_datos_gps dato_leido;  
    memory_chip.load(_puntero_lectura, dato_leido);  //recuperar un punto gps
    _puntero_lectura+=USER_DATA_SIZE;                //aumenter el puntero

    /* Mostrar Contador de muestras */
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


//========================================================
//  LISTAR RAW
//========================================================

void mostrar_informacion_raw()
{
 /*
  * DEBUG: Ante posibles fallos de la FAT, permite desde el menu Serial
  * el acceso y listado en modo RAW de los datos de la zona de usuarios
  */
  Serial.print(F("\n\n >> MOSTRANDO INFORMACION RAW <<\n\n"));
  /* recuperar los puntos del recorrido */
  uint16_t puntero_lectura=POS_MEM_USER_DATA_START;

  while( puntero_lectura <= EEPROM_SIZE ){ 
    struct_datos_gps dato_leido;  
    memory_chip.load(puntero_lectura, dato_leido);  //recuperar un punto gps
                   

    /* Mostrar Contador de muestras */
    if(puntero_lectura < 10){ Serial.print(F("0")); }
    if(puntero_lectura < 100){ Serial.print(F("0")); }
    if(puntero_lectura < 1000){ Serial.print(F("0")); }
    if(puntero_lectura < 10000){ Serial.print(F("0")); }
    Serial.print(puntero_lectura);
    Serial.print(F(","));
    
    Serial.print(dato_leido.latitud,6); 
    Serial.print(F(","));
    Serial.println(dato_leido.longitud,6);
    puntero_lectura+=USER_DATA_SIZE;  //aumentar el puntero
  }
}

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   BORRADO PARCIAL DE EEPROM LA FECHA (SOLO INFORMACION DE SESIONES)
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void soft_erase_eeprom()
{
 /*
  * Borrado del espacio de registros FAT de la eeprom para liberarla
  * sin tener que reescribirla por completo
  */
  
  /* borramos solo los registros de asignacion de sesiones */
  for(uint16_t reg=0; reg< POS_MEM_USER_DATA_START-1 ;reg++){
    memory_chip.write( reg, 0 );
  }
}



/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   BORRADO COMPLETO DE EEPROM 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void full_erase_eeprom()
{
 /*
  * Borrado completo de la eeprom para liberarla. 
  * Digamos que es un formateo, que escribe 0 entodas y cada una de las posiciones,
  * tanto de la FAT como de la zona de datos de usuario
  */
  
  /* borramos toda le eeprom */
  for(uint16_t reg=0; reg< EEPROM_SIZE+10 ;reg++){
    memory_chip.write( reg, 0 );
    if( reg%1000==0 ) { Serial.print(F("WRITE posicion: "));Serial.println(reg); }
  }
}

/*mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
//   CHEQUEO DE EEPROM FULL ERASE 
//mmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm*/

void check_eeprom()
{
 /*
  * Comprobacion de la eeprom tras un borrado con full_erase()
  */
  uint16_t error = 0;
  for(uint16_t reg=0; reg< EEPROM_SIZE+10 ;reg++){
    if( reg%1000==0 ) { Serial.print(F("READ posicion: "));Serial.println(reg); }
    if(memory_chip.read(reg)!=0){
      error++;
     Serial.print(F("ERROR en Posicion: "));Serial.println(reg);
    }
  }
  if(error==0) { Serial.println(F("MEMORIA ok")); }
  else { 
    Serial.print(F("La Memoria presenta errores en "));
    Serial.print(error);
    Serial.println(F(" celdas.\nPrueba a modificar el tiempo de escritura")); 
  }
}



//*******************************************************
//                    FIN DE PROGRAMA
//*******************************************************
