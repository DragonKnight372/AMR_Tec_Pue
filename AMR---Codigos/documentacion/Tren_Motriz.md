# Subsistema — Tren Motriz

## Descripción

> _Función del subsistema dentro del AMR1._

## Modos de firmware

| Entorno PlatformIO    | Uso                              |
|-----------------------|----------------------------------|
| `tren_motriz_can`     | Integración completa por CAN bus |
| `tren_motriz_serial`  | Calibración y pruebas por Serial |

## Hardware

| Componente      | Pin / Interfaz | Notas                        |
|-----------------|----------------|------------------------------|
| Motor PWM       | A1             |                              |
| Relé habilitador| 4              |                              |
| Dirección REV   | 11             |                              |
| CONFIG L1       | 12             | Transistor ON permanente     |
| CONFIG L2       | 13             | Transistor OFF permanente    |
| CAN CS          | PIN_CAN_CS     | Definido por la placa        |

## IDs CAN

| ID (hex) | Dirección | Descripción       |
|----------|-----------|-------------------|
| 0x300    | Entrada   | Comando PWM (%)   |
| 0x301    | Salida    | Feedback PWM      |
| 0x310    | Entrada   | Comando relé      |
| 0x311    | Salida    | Feedback relé     |
| 0x320    | Entrada   | Comando dirección |
| 0x321    | Salida    | Feedback dirección|
| 0x210    | Entrada   | Paro emergencia   |

## Lógica de control

### Flujo CAN

```
Master (Intérprete)
  │
  │  0x310 — relay enable (1/0)
  │  0x320 — dirección (0=FWD, 1=REV)
  │  0x300 — velocidad [0-100 %]
  │  0x210 — emergencia (1/0)
  ▼
Feather RP2040 (tren_motriz_can)
  │  velocidad → map(0-100%, 0-255) → PWM
  │  relay: HIGH = habilitado
  │  dirección: MOTOR_DIR_REV LOW=FWD, HIGH=REV (cableado invertido)
  │
  │  0x301 — feedback velocidad
  │  0x311 — feedback relé
  │  0x321 — feedback dirección
  ▼
Master (Intérprete)
```

### Cambio de dirección

Antes de invertir la dirección el firmware para completamente el motor (PWM=0)
y espera 150 ms para evitar picos de corriente al revertir con el motor girando.

### Control del relé

Antes de abrir el relé (desactivar) el firmware pone PWM=0 para evitar arco
eléctrico en los contactos.

### Watchdog

Si no se recibe ningún ID relevante (0x300, 0x310, 0x320, 0x210) durante 500 ms,
el nodo entra en modo seguro: PWM=0 y relé abierto. Se reactiva al recibir el primer
mensaje CAN del master.

### Marchas (hardcoded)

| Pin        | Estado | Descripción                |
|------------|--------|----------------------------|
| CONFIG_L1  | HIGH   | Marcha L1 siempre activa   |
| CONFIG_L2  | LOW    | Marcha L2 siempre inactiva |

### Emergencia (0x210=1)

Motor a 0, relé abierto. Se resetea al recibir 0x210=0 del master.

## Calibración (modo Serial)

> _Procedimiento de calibración: comandos, valores esperados, cómo verificar._

## Notas

> _Observaciones de hardware, bugs conocidos, revisiones de PCB, etc._
