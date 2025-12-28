# Trabajo Práctico Integrador: Internet de las cosas y conectividad de sistemas embebidos
## 2022

### Objetivo:
Aplicar conceptos adquiridos en clase por medio del uso de la placa STM32F401RE.
Metodología: Hay 2 etapas del desarrollo llevadas a cabo por 2 grupos de trabajo distintos: 'Sensor Side' y 'Lora Side'

Sensor Side: Comienza con la recepción por UART de una trama con datos de un sensor, se extraen sus datos y se arma una nueva trama que se completa con la **dirección de origen** y **destino** 
del mensaje, **tamaño en bytes** de los datos, los **datos de medición** del sensor en sí y finalmente el **CRC**.
Con la trama armada y utilizando módulos MAX-RS485 se habilita transmisión de datos hacia la segunda parte: Lora Side, por medio del protocolo **RS485**.

Lora Side: Se recibe la trama completa y utilizando máquina de estados se extraen los datos según si la dirección del mensaje sea coincidente con la seteada, se calcula nuevamente el CRC,
se la compara con el recibido y se lo envía al Sensor Side un mensaje de confirmación 'OK' si el CRC recibido coincide con el calculado o 'ERROR' en caso contrario.

Por último utilizando comandos AT, se envía información recibida a un módulo emisor LORA.


