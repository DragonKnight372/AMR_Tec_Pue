# AMR1 — Documentación General

## Descripción del proyecto

> _Breve descripción del robot / vehículo AMR1 y su propósito._

## Arquitectura del sistema

> _Diagrama o descripción general de cómo se conectan los subsistemas entre sí._

```
[ Intérprete ] ──CAN──┬── [ Tren Motriz ]
                      ├── [ Freno ]
                      └── [ Dirección ]
```

## Subsistemas

| Subsistema   | Archivo de documentación          |
|--------------|-----------------------------------|
| Tren Motriz  | [Tren_Motriz.md](Tren_Motriz.md)  |
| Freno        | [Freno.md](Freno.md)              |
| Dirección    | [Direccion.md](Direccion.md)      |
| Intérprete   | [Interprete.md](Interprete.md)    |

## Bus CAN — Tabla de IDs

| ID (hex) | Subsistema    | Dirección  | Descripción                   | Payload            |
|----------|---------------|------------|-------------------------------|--------------------|
| 0x100    | Dirección     | → RX       | Posición objetivo             | 1 byte — 0-100 %   |
| 0x101    | Dirección     | ← TX       | Feedback posición actual      | 1 byte — 0-100 %   |
| 0x200    | Freno         | → RX       | Comando freno                 | 1 byte             |
| 0x201    | Freno         | ← TX       | Feedback estado freno         | 1 byte             |
| 0x210    | Freno / Tren  | Broadcast  | Paro de emergencia            | 1 byte             |
| 0x300    | Tren Motriz   | → RX       | Comando velocidad (PWM)       | 1 byte — 0-100 %   |
| 0x301    | Tren Motriz   | ← TX       | Feedback velocidad actual     | 1 byte — 0-100 %   |
| 0x310    | Tren Motriz   | → RX       | Comando relé (enable/disable) | 1 byte             |
| 0x311    | Tren Motriz   | ← TX       | Feedback estado relé          | 1 byte             |
| 0x320    | Tren Motriz   | → RX       | Comando dirección (fwd/rev)   | 1 byte             |
| 0x321    | Tren Motriz   | ← TX       | Feedback dirección actual     | 1 byte             |

## Hardware común

- **Microcontrolador:** Adafruit Feather RP2040 CAN
- **CAN transceiver:** MCP2515 (integrado en la placa)
- **Pantalla:** OLED SSD1306 128×64 (I2C, 0x3C)
- **Velocidad CAN:** 250 kbps
- **Velocidad Serial:** 115200 baud

## Upload al Feather RP2040 (Windows)

El Feather RP2040 usa **picotool** para cargar firmware. En Windows requiere instalar el driver WinUSB una sola vez.

### Pasos para subir código

1. **Doble tap en RESET** — la placa entra en modo bootloader y aparece como unidad USB `RPI-RP2` en el Explorador de archivos.
2. En PlatformIO seleccionar el entorno deseado y hacer Upload (o `pio run -e <env> -t upload`).

### Instalación del driver (una sola vez por PC)

Si picotool falla con `No accessible RP2040 devices in BOOTSEL mode`:

1. Descargar **Zadig** desde `zadig.akeo.ie`
2. Poner la placa en modo bootloader (doble tap RESET → aparece `RPI-RP2`)
3. Abrir Zadig → **Options → List All Devices**
4. Seleccionar **`RP2 Boot (Interface 1)`** (USB ID: `2E8A 0003`)
5. Driver destino: **WinUSB**
6. Click **Install Driver**

> **No tocar** `Pico Serial (Interface 0)` — ese es el puerto serial normal (Serial Monitor). Reemplazarlo con WinUSB rompe la comunicación serial.

## Herramientas y dependencias

- PlatformIO + framework Arduino
- Plataforma: `maxgerhardt/platform-raspberrypi`
- Librerías: Adafruit MCP2515, Adafruit SSD1306, Adafruit GFX, Adafruit BusIO

## Notas generales

> _Decisiones de diseño, restricciones conocidas, versiones de hardware, etc._
