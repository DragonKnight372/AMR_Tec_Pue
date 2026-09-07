# Subsistema — Intérprete

**Firmware actual:** `interprete_can` v2.4

## Descripción

El intérprete es el nodo maestro del bus CAN. Recibe comandos del control remoto RC
vía protocolo SBUS (Serial1, pin 1, 100 kbaud invertido) y los traduce en mensajes
CAN hacia los subsistemas de dirección, freno, tren motriz y relay.

También monitorea pérdida de señal (failsafe) y aplica frenado de emergencia
automáticamente. Muestra estado en pantalla OLED y recibe feedback de posición
de la dirección por CAN.

## Modos de firmware

| Entorno PlatformIO    | Uso                              |
|-----------------------|----------------------------------|
| `interprete_can`      | Integración completa por CAN bus |
| `interprete_serial`   | Calibración y pruebas por Serial |

## Hardware

| Componente                  | Pin / Interfaz              | Notas                                              |
|-----------------------------|-----------------------------|----------------------------------------------------|
| Adafruit Feather RP2040 CAN | —                           | Microcontrolador principal, CAN integrado (MCP2515)|
| Receptor SBUS               | Serial1 — PIN 1 (RX)        | 100 kbaud, 8E2, señal invertida por GPIO           |
| OLED SSD1306 128×64         | I2C — SDA/SCL (0x3C)        | Dashboard de estado en tiempo real                 |

## Canales SBUS del control remoto

| Canal SBUS | Índice | Función            | Rango RAW   | Descripción                                              |
|------------|--------|--------------------|-------------|----------------------------------------------------------|
| CH1        | 0      | Throttle / Freno   | 172 – 1811  | Palanca principal: arriba = acelera, abajo = frena       |
| CH2        | 1      | Dirección          | 172 – 1811  | Palanca lateral: izquierda/derecha                       |
| CH3        | 2      | Emergencia A       | —           | > 1500 RAW = activa emergencia                           |
| CH5        | 4      | Enable             | —           | > 1500 RAW = sistema habilitado (LIVE)                   |
| CH7        | 6      | Marcha / Reversa   | —           | > 1300 = REVERSA · < 700 = MARCHA ALTA · resto = F-LO   |

## Zonas de la palanca de throttle (CH1)

| Zona        | RAW            | Acción                                                  |
|-------------|----------------|---------------------------------------------------------|
| FRENA       | 172 – 910      | `valorFreno = map(throttle, 172, 910, 100, 1)`          |
| MUERTA      | 911 – 1100     | `valorVel = 0`, `valorFreno = 0`                        |
| ACELERA     | 1101 – 1811    | `valorVel = map(throttle, 1101, 1811, 50, VEL_MAX)`     |

- En marcha alta delantera: `VEL_MAX = 200`
- En marcha baja o reversa: `VEL_MAX = 120`

## IDs CAN que genera (TX)

| ID (hex) | Descripción                       | Payload                                         |
|----------|-----------------------------------|-------------------------------------------------|
| 0x100    | Comando dirección                 | 1 byte — 0-100 % (0=IZQ, 100=DER)              |
| 0x200    | Comando freno PROPORCIONAL        | 1 byte — 0-100 % (0=libre, 100=máximo)         |
| 0x210    | Paro de emergencia (broadcast)    | 1 byte: `1` = emergencia, `0` = reset           |
| 0x300    | Comando velocidad                 | 1 byte — 0-100 % (mapeado desde valorVel)       |
| 0x310    | Enable relay                      | 1 byte: `1` = habilitado, `0` = deshabilitado   |
| 0x320    | Reversa                           | 1 byte: `1` = reversa, `0` = adelante           |
| 0x330    | Marcha alta (informativo)         | 1 byte: `1` = marcha alta, `0` = marcha baja    |

## IDs CAN que escucha (RX)

| ID (hex) | Origen    | Descripción              | Payload          |
|----------|-----------|--------------------------|------------------|
| 0x101    | Dirección | Feedback posición actual | 1 byte — 0-100 % |

El feedback de dirección se muestra en el OLED (barra con marcador CMD y FB)
y se usa para calcular el error de seguimiento.

## Lógica de arbitraje / emergencia

### Condiciones de emergencia

El intérprete activa emergencia (`valorFreno = 100`, `valorVel = 0`, `valorEnable = 0`) cuando:

- CH3 > 1500 RAW (switch de emergencia del control)
- CH5 ≤ 1500 RAW (enable apagado — sistema en SAFE)
- `failsafe = true` (señal SBUS perdida)
- `millis() - last_update > 500 ms` (timeout de trama SBUS)

### Interlock freno-motor (v2.4)

Si `valorFreno > 7 %` (umbral `UMBRAL_FRENO_BLOQUEO`), el motor se fuerza a 0
aunque CH1 esté en zona de aceleración. El OLED muestra la etiqueta `LOCK`.

Esto evita acelerar mientras el freno está aplicado.

### Throttle y freno son mutuamente excluyentes

- Zona FRENA → `valorVel = 0` siempre
- Zona ACELERA → `valorFreno = 0` siempre
- Zona MUERTA → ambos en 0

## Display OLED

| Fila | Contenido                                                     |
|------|---------------------------------------------------------------|
| 1    | Header `AMR1` · estado (NO SIGNAL / SAFE / LIVE) · CAN link  |
| 2    | `VEL` barra + porcentaje                                      |
| 3    | `BRK` barra + porcentaje                                      |
| 4    | `DIR` barra bidireccional con marcador CMD (triángulo) y FB   |
| 5    | `FB:xx% ERR:±xx` (si dirección online) o `FB: sin link`       |
| 6    | Tags: `EN` · `REV/F-HI/F-LO` · `---/BRK/LOCK`               |

## Calibración (modo Serial)

> El firmware `interprete_serial` no requiere calibración de parámetros físicos.
> Se usa principalmente para inspeccionar los valores RAW de cada canal SBUS
> y verificar que el control remoto esté bien configurado.

## Notas

- La señal SBUS llega **invertida** a nivel lógico. Se corrige con `gpio_set_inover(SBUS_PIN_RX, 1)`.
- El bucle de SBUS corre en el **Core 1** del RP2040; el CAN y OLED corren en Core 0.
- La trama SBUS tiene 25 bytes: inicia con `0x0F` y termina con `0x00`.
- El relé queda activo incluso con freno para una liberación más rápida del freno.
