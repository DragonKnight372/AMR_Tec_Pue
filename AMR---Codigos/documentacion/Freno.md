# Subsistema — Freno

**Firmware actual:** `freno_can` v2.6 · `freno_serial` (sin cambios)

## Descripción

El subsistema de freno controla un actuador lineal que extiende o retrae un pistón de
frenado. La posición se mide con un potenciómetro en el pin A0. El rango útil del ADC
es muy estrecho (RAW 815-868) porque el recorrido físico del actuador es pequeño.

La escala es **invertida**: mayor valor ADC = pistón más extendido = freno activo = 0 %.
El estado normal de reposo es pistón retraído (100 %).

Desde v2.6 el freno es **proporcional**: el comando CAN 0x200 acepta valores de 0 a 100 %
y el actuador se posiciona en `100 - cmd` %, permitiendo frenada gradual en lugar de
solo posiciones extremas.

El driver de potencia es un **Pololu** con señal de falla ACT_FLT: cuando el botón rojo
de paro físico corta la señal al driver, ACT_FLT baja a LOW y el firmware lo detecta
como paro físico (no como error eléctrico).

## Modos de firmware

| Entorno PlatformIO | Uso                              |
|--------------------|----------------------------------|
| `freno_can`        | Integración completa por CAN bus |
| `freno_serial`     | Calibración y pruebas por Serial |

## Hardware

| Componente                  | Pin / Interfaz       | Notas                                            |
|-----------------------------|----------------------|--------------------------------------------------|
| Adafruit Feather RP2040 CAN | —                    | Microcontrolador principal, CAN integrado        |
| Driver Pololu               | —                    | Driver de potencia para el actuador lineal       |
| └ PWM                       | PIN 6                | Señal PWM de velocidad al driver                 |
| └ DIR                       | PIN 9                | Dirección (HIGH = retraer, LOW = extender)       |
| └ ACT_FLT                   | PIN 5 (INPUT_PULLUP) | Fault del driver — LOW = falla / paro físico     |
| Potenciómetro               | PIN A0 (ADC 12 bits) | Feedback de posición; rango útil: RAW 815 – 868  |
| OLED SSD1306 128×64         | I2C — SDA/SCL (0x3C) | Muestra RAW, PCT, objetivo, estado motor y fault |

## IDs CAN

| ID (hex) | Dirección | Descripción              | Payload                                     |
|----------|-----------|--------------------------|--------------------------------------------|
| 0x200    | → RX      | Comando freno PROPORCIONAL | 1 byte: 0-100 % (0 = libre, 100 = máximo) |
| 0x201    | ← TX      | Feedback posición actual | 1 byte — 0-100 %                           |
| 0x210    | → RX      | Paro de emergencia       | 1 byte: `1` = emergencia, `0` = reset      |

El ID 0x210 es compartido con Tren Motriz; ambos nodos lo escuchan simultáneamente.

## Lógica de control

### Estados del sistema (CAN)

El actuador se posiciona en `posicionObjetivo = 100 − cmd`, donde `cmd` es el valor
recibido en 0x200 (0-100 %). Esto permite frenada gradual:

| cmd (0x200) | posicionObjetivo | Descripción             |
|-------------|-----------------|-------------------------|
| 0           | 100 %           | Normal — pistón retraído |
| 1-99        | 99-1 %          | Freno proporcional       |
| 100         | 0 %             | Freno máximo — extendido |

```
               ┌──────────────────────────────────────────┐
               │       NORMAL  (posición = 100%)          │  ← cmd=0
               └──────┬───────────────────────────────────┘
                      │ cmd = 1-100
               ┌──────▼────────────────────┐   ┌──────────────┐
               │  FRENO PROPORCIONAL       │   │  EMERGENCIA  │  ← 0x210=1
               │  posición = 100 - cmd     │   │  (pos = 0%)  │
               │  (gradual 99% → 0%)       │   └──────────────┘
               └───────────────────────────┘
```

Si `masterOffline` (watchdog 1500 ms sin CAN): `posicionObjetivo = POS_FRENO (0%)`.
Si `modoEmergencia` (0x210=1): `posicionObjetivo = POS_EMERGENCIA (0%)` — sobreescribe cualquier cmd.

### Detección de paro físico (botón rojo)

Cuando el operador pulsa el botón rojo de emergencia física:
1. El botón corta la señal al driver Pololu
2. `ACT_FLT` baja a LOW → firmware detecta `paroFisico = true`
3. Motor se detiene; **no** se marca error eléctrico
4. El sistema espera a que el operador libere el botón

### Modo recuperación

Si el potenciómetro queda fuera de rango (timeout 2 s), el firmware entra en modo
recuperación: retrae el actuador lentamente hasta que el ADC vuelve al rango válido.
Si durante la recuperación vuelve a detectarse paro físico, se detiene de inmediato.

### Parámetros de control

| Parámetro              | Valor | Descripción                                      |
|------------------------|-------|--------------------------------------------------|
| `RAW_MIN`              | 815   | ADC en posición retraída (normal / 100 %)        |
| `RAW_MAX`              | 868   | ADC en posición extendida (freno activo / 0 %)   |
| `TOLERANCIA`           | 8 %   | Zona muerta; dentro de ella motor stop           |
| `TIMEOUT_FUERA_RANGO`  | 2 s   | Tiempo antes de entrar en modo recuperación      |
| `POS_NORMAL`           | 100 % | Posición de reposo (pistón retraído)             |
| `POS_FRENO`            | 0 %   | Posición de frenado (pistón extendido)           |
| PWM mínimo             | 140   | PWM mínimo con el que el actuador se mueve       |
| PWM máximo             | 255   | PWM máximo (error = 100 %)                       |

> La escala está **invertida**: `map(raw, RAW_MAX, RAW_MIN, 0, 100)`.
> Mayor ADC → más extendido → menos %.

## Calibración (modo Serial)

### Objetivo

Encontrar los valores RAW del potenciómetro correspondientes a los dos extremos físicos:
pistón totalmente retraído (`RAW_MIN`) y pistón totalmente extendido (`RAW_MAX`).

### Requisitos

- Placa cargada con el entorno `freno_serial`
- Monitor serial abierto a 115200 baud
- **Precaución:** el recorrido útil es muy corto (RAW 815-868).
  Ir despacio y observar el valor RAW en todo momento.

### Procedimiento

#### 1. Arranque

Al conectar aparece el menú y la calibración actual:

```
=== CALIBRACION FRENOS ===
  Valores compilados:  MIN=815  MAX=868
  --------------------------
  0-100  Mover a posicion (%)
  s      Stop motor
  m      Marcar RAW_MIN con lectura actual (retraido)
  M      Marcar RAW_MAX con lectura actual (extendido)
  p      Imprimir calibracion para copiar al codigo CAN
  h      Ayuda
```

Salida continua (200 ms):
```
RAW:831      PCT:87%  OBJ:100%  RETR  PWM:185  FLT:OK
```

#### 2. Encontrar RAW_MIN (pistón retraído — posición normal)

1. Enviar `100` → el actuador se retrae completamente.
2. Esperar a que se detenga (PCT ≈ 100 %, motor STOP).
3. Observar el valor `RAW` estabilizado en el terminal.
4. Enviar `m` → guarda ese RAW como `RAW_MIN`.
   ```
   >> RAW_MIN actualizado = 815
   ```

#### 3. Encontrar RAW_MAX (pistón extendido — freno activo)

1. Enviar `0` → el actuador se extiende (frena).
2. Esperar a que se detenga (PCT ≈ 0 %, motor STOP).
3. Observar el valor `RAW` estabilizado.
4. Enviar `M` → guarda ese RAW como `RAW_MAX`.
   ```
   >> RAW_MAX actualizado = 868
   ```

#### 4. Imprimir y copiar

Enviar `p` → el sistema imprime las líneas listas para pegar:

```
  -- Copiar en freno/can/main.cpp --
    const int RAW_MIN = 815;
    const int RAW_MAX = 868;
  -- Y actualizar en freno/serial/main.cpp --
    const int RAW_MIN = 815;
    const int RAW_MAX = 868;
```

#### 5. Verificación

- Enviar `0` → debe frenar y detenerse solo.
- Enviar `100` → debe liberar y detenerse solo.
- Si el motor no para o PCT nunca llega al objetivo: ajustar `TOLERANCIA`.

### Referencia de comandos Serial

| Comando | Acción                                              |
|---------|-----------------------------------------------------|
| `0-100` | Mover a ese % (0 = freno, 100 = normal)             |
| `s`     | Stop motor                                          |
| `m`     | Guardar RAW actual como RAW_MIN (retraído)          |
| `M`     | Guardar RAW actual como RAW_MAX (extendido)         |
| `p`     | Imprimir calibración lista para copiar al código    |
| `h`     | Mostrar ayuda                                       |

## Notas

- El rango ADC útil es muy estrecho (solo ~53 cuentas entre MIN y MAX). Pequeñas
  variaciones mecánicas pueden desplazar los valores; recalibrar si el freno no
  responde correctamente.
- El pin ACT_FLT usa `INPUT_PULLUP`; LOW = fault del driver (paro físico o falla eléctrica).
- Los valores `RAW_MIN` y `RAW_MAX` deben actualizarse en **ambos** archivos
  (`freno/can/main.cpp` y `freno/serial/main.cpp`) después de cada calibración.
