# GPS_data_logger_eeprom_multisesion
Data logger para el GPS de "@Norma, la perra", 
Graba datos en eeprom externa de hasta 64Kb y dispone de grabacion en modo multisesion
Es decir puede acumualr sesiones sucesivas de manera que no sobreescribe los datos anteriores

Version actual,  v1.1
Dispone de un acelerometro para detectar que Norma se ha quedado parada y no grabar datos.
Es bien sabido que los modulos GPS NEO6/7/8 tienen un cierto error de deriva lo genera "una nube de puntos" aun estando en una poscion fija.


TODO: 
* subir el esquema de fritzzing y alguno ficheros de datos recopilados.
* "Condicionar" el mostrar algunas lineas print que corresponden a mensajes de ayuda para debug
