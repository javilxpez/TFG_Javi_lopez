El ESP32 tiene periféricos UART de silicio dedicados. Cuando el servo responde por RS485, el hardware UART recibe los bytes en su propia FIFO con interrupciones, sin tocar el CPU. Cuando ModbusMaster llama a readBytes(), los bytes ya están esperando en el buffer — la espera es prácticamente cero (~1-3ms total por operación).


CPU: enviamos frame → hacemos otra cosa → bytes ya están → leemos y seguimos
                       ↑ hardware lo hace solo
Raspberry Pi Pico 2: UART por PIO (SerialPIO)

El Pico no tiene pines UART en GP13/GP14 (los pines que necesitamos). Por eso usamos SerialPIO — una máquina de estado programable (PIO) que emula el UART en software.

El problema es que SerialPIO hace el polling de otra manera, y parece que el setTimeout(20) no funciona igual que en un UART hardware. Lo que ocurría en la práctica:


Core0: enviamos frame → esperamos respuesta → TIMEOUT (20ms o más) → seguimos
                          ↑ esto bloqueaba todo el Core0
Diez writes × 20ms = 200ms bloqueado. Más las lecturas de monitorización. Todo acumulado daba los 3-4 segundos.

Por qué el timeout no funcionaba

ModbusMaster tiene su propio bucle interno esperando respuesta:


while (millis() - start < timeout) {
    if (_serial->available()) { // lee bytes }
}
En ESP32, available() devuelve true casi inmediatamente (hardware). En SerialPIO, el PIO puede tener más latencia en reportar bytes listos, o el timeout interno no se aplicaba correctamente.

Por qué el dual-core lo soluciona

No cambia nada del comportamiento de SerialPIO. Lo que cambia es que ahora ese bloqueo ocurre en Core1, que no tiene nada más que hacer. Core0 corre libre sin esperar nunca.
