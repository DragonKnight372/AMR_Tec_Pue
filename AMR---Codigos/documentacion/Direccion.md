# Subsistema — Dirección

## Descripción

El subsistema de dirección controla el ángulo de giro de las ruedas delanteras del AMR1.
Un motor MY1016Z3 mueve la cadena de dirección; un potenciómetro acoplado al mecanismo
proporciona retroalimentación continua de posición. El controlador cierra el lazo de posición
en el Feather RP2040 y recibe comandos de posición objetivo (0-100 %) desde el intérprete
a través del CAN bus.

## Modos de firmware

| Entorno PlatformIO | Uso                              |
|--------------------|----------------------------------|
| `direccion_can`    | Integración completa por CAN bus |
| `direccion_serial` | Calibración y pruebas por Serial |

## Hardware

| Componente                        | Pin / Interfaz       | Notas                                               |
|-----------------------------------|----------------------|-----------------------------------------------------|
| Adafruit Feather RP2040 CAN       | —                    | Microcontrolador principal, CAN integrado (MCP2515) |
| Motor MY1016Z3                    | —                    | Motor DC que mueve la cadena de dirección           |
| Puente H BTS7960                  | —                    | Driver de potencia para el motor                    |
| └ R_EN + L_EN (juntos)            | PIN 5                | Enable del puente H                                 |
| └ RPWM                            | PIN 6                | PWM canal A (avance)                                |
| └ LPWM                            | PIN 9                | PWM canal B (retroceso)                             |
| Potenciómetro multivuelta bobinado| PIN A0 (ADC 12 bits) | Feedback de posición; rango útil: RAW 1200 – 2740   |
| OLED SSD1306 128×64               | I2C — SDA/SCL (0x3C) | Muestra posición objetivo, actual, error y PWM      |

## IDs CAN

| ID (hex) | Dirección | Descripción              | Payload          |
|----------|-----------|--------------------------|------------------|
| 0x100    | → RX      | Posición objetivo        | 1 byte — 0-100 % |
| 0x101    | ← TX      | Feedback posición actual | 1 byte — 0-100 % |

El nodo publica feedback cada 50 ms y también de forma inmediata al recibir un comando.

## Lógica de control

El firmware implementa un controlador proporcional de posición con zona muerta.

### Flujo CAN

```
Master (Intérprete)
  │
  │  0x100 — objetivo [0-100 %]
  ▼
Feather RP2040 (direccion_can)
  │  lee potenciómetro → convierte RAW a %
  │  error = objetivo - actual
  │  |error| ≤ TOLERANCIA → motor stop
  │  |error| > TOLERANCIA → PWM proporcional (135-195)
  │
  │  0x101 — posición actual [0-100 %]
  ▼
Master (Intérprete)
```

### Límites de posición (RAW ADC, 12 bits)

| Posición   | RAW medido | % equivalente | Descripción                        |
|------------|------------|---------------|------------------------------------|
| Izquierda  | ~1200      | 0 %           | Antes del final de carrera IZQ     |
| Medio      | ~2006      | ~54 %         | Ruedas mirando al frente           |
| Derecha    | ~2740      | 100 %         | Antes del final de carrera DER     |

> Los valores RAW son los que entrega el ADC de 12 bits (0-4095) del pin A0.
> Siempre dejar margen antes de los finales de carrera físicos.

### Parámetros de control

| Parámetro    | Valor  | Descripción                              |
|--------------|--------|------------------------------------------|
| `RAW_MIN`    | 1200   | RAW correspondiente a 0 % (izquierda)    |
| `RAW_MAX`    | 2740   | RAW correspondiente a 100 % (derecha)    |
| `TOLERANCIA` | 1 %    | Zona muerta; dentro de ella motor stop   |
| PWM mínimo   | 135    | PWM mínimo con el que el actuador se mueve |
| PWM máximo   | 195    | PWM máximo (error = 100 %)               |

## Calibración (modo Serial)

### Objetivo

Encontrar los valores RAW del potenciómetro que corresponden a los tres puntos clave:
izquierda, medio y derecha, sin llegar a los finales de carrera mecánicos.

### Requisitos

- Placa cargada con el entorno `direccion_serial`
- Monitor serial abierto a 115200 baud
- **Precaución:** mover la cadena de dirección despacio y con atención.
  Un movimiento brusco o ir más allá del final de carrera puede dañar el mecanismo.

### Procedimiento

#### 1. Arranque

Al conectar, el terminal muestra la calibración actual cargada desde EEPROM y la lectura
continua del potenciómetro:

```
RAW=2453 | L=1200 M=2006 R=2740 | POS=54% | MED
```

#### 2. Encontrar IZQUIERDA

1. Enviar `s` + Enter para asegurarse de que el motor está parado.
2. Mover **manualmente y con cuidado** la cadena de dirección hacia la izquierda,
   observando el valor `RAW` en el terminal.
3. Detenerse **antes** de escuchar o sentir el final de carrera físico.
4. Cuando el valor RAW se estabilice en el punto deseado, enviar:
   ```
   SL
   ```
   El sistema responde: `>> [GUARDADO] IZQ = RAW XXXX`

#### 3. Encontrar DERECHA

1. Mover la cadena hacia la derecha del mismo modo, con precaución.
2. Detenerse **antes** del final de carrera físico del lado derecho.
3. Enviar:
   ```
   SR
   ```
   El sistema responde: `>> [GUARDADO] DER = RAW XXXX`

#### 4. Encontrar MEDIO

1. Enviar `M` para ir al punto medio calculado automáticamente, o mover manualmente.
2. Ajustar hasta que **ambas ruedas queden mirando hacia adelante** (rectas).
3. Enviar:
   ```
   SM
   ```
   El sistema responde: `>> [GUARDADO] MED = RAW XXXX`

#### 5. Verificación

Usar los comandos `L`, `M`, `R` para que el actuador se mueva a cada punto guardado
y confirmar que las posiciones son correctas visualmente.

```
L   → ruedas a la izquierda (antes del tope)
M   → ruedas al frente
R   → ruedas a la derecha (antes del tope)
```

#### 6. Actualizar el firmware CAN

Copiar los valores RAW obtenidos en `src/direccion/can/main.cpp`:

```cpp
const int RAW_MIN = <valor IZQ>;   // IZQUIERDA
const int RAW_MAX = <valor DER>;   // DERECHA
```

Y en `src/direccion/serial/main.cpp`:

```cpp
int rawLeft  = <valor IZQ>;
int rawMid   = <valor MED>;
int rawRight = <valor DER>;
```

### Referencia de comandos Serial

| Comando | Acción                                      |
|---------|---------------------------------------------|
| `0-100` | Mover a ese porcentaje entre IZQ y DER      |
| `L`     | Ir a la posición IZQ guardada               |
| `M`     | Ir a la posición MED guardada               |
| `R`     | Ir a la posición DER guardada               |
| `SL`    | Guardar posición actual como IZQ            |
| `SM`    | Guardar posición actual como MED            |
| `SR`    | Guardar posición actual como DER            |
| `s`     | Parar motor                                 |
| `?`     | Imprimir calibración actual                 |

## Notas

>Cuidado con la cadena, se intento cambiar pero no termino funcioando probocando un lijero accidente. Por lo que se volvio a colocar la cadena anterior oxidada.
