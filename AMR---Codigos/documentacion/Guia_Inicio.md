# Guía de inicio — AMR1
### Para personas que vienen de Arduino IDE

---

## 1. Qué necesitas instalar

| Programa | Para qué sirve | Descarga |
|----------|---------------|----------|
| **VS Code** | El editor de código (reemplaza al Arduino IDE) | code.visualstudio.com |
| **PlatformIO** | Extensión de VS Code que compila y sube el código al microcontrolador | Se instala desde dentro de VS Code |
| **Git** | **Obligatorio** — PlatformIO lo usa para descargar la plataforma del Feather RP2040 desde GitHub. Sin Git, el proyecto no compila aunque tengas el código. | git-scm.com |

> **Python** también es necesario. PlatformIO te avisa si no lo tienes y te da el link.

> **¿Por qué Git aunque no uses GitHub?** Este proyecto usa `platform = https://github.com/maxgerhardt/platform-raspberrypi.git` en `platformio.ini`. Cuando PlatformIO compila por primera vez, descarga esa plataforma usando Git. Si Git no está instalado, la compilación falla con un error de "platform not found" o similar.

### Instalar PlatformIO en VS Code

1. Abrir VS Code
2. Click en el ícono de extensiones en la barra izquierda (o `Ctrl+Shift+X`)
3. Buscar **"PlatformIO IDE"**
4. Click en **Install**
5. Reiniciar VS Code cuando lo pida

---

## 2. Obtener el código

Hay dos formas de bajar el proyecto. **Elegí una sola.**

---

### Opción A — Descargar como ZIP 

1. Ir al repositorio en GitHub (pedile el link a quien te sumó al proyecto)
2. Click en el botón verde **`< > Code`**
3. Click en **`Download ZIP`**
4. Descomprimir el ZIP en una carpeta de tu computadora, por ejemplo:
   ```
   C:\Proyectos\AMR - Codigos\
   ```
5. Listo — ya podés ir al paso 3 (Abrir el proyecto)

> **Desventaja:** cada vez que el código se actualice vas a tener que bajar un ZIP nuevo a mano.

---

### Opción B — Clonar con Git *(recomendado)*

#### Instalar Git

1. Ir a **git-scm.com** → Download → Windows
2. Instalar con todas las opciones por defecto (next, next, finish)
3. Para verificar que quedó instalado: abrir una terminal (`Win + R` → escribir `cmd` → Enter) y escribir:
   ```
   git --version
   ```
   Tiene que aparecer algo como `git version 2.x.x`

#### Clonar el repositorio

1. Ir al repositorio en GitHub
2. Click en el botón verde **`< > Code`**
3. Copiá la URL que aparece (empieza con `https://github.com/...`)
4. Abrir una terminal (`Win + R` → `cmd` → Enter)
5. Navegar a la carpeta donde querés guardar el proyecto:
   ```
   cd C:\Proyectos
   ```
6. Escribir el comando de clonado con la URL que copiaste:
   ```
   git clone https://github.com/...
   ```
7. Git descarga todo el proyecto automáticamente en una carpeta nueva

#### Actualizar el código cuando haya cambios

Cada vez que alguien del equipo suba código nuevo, para traerte los cambios:

1. Abrir una terminal dentro de la carpeta del proyecto
2. Escribir:
   ```
   git pull
   ```

> Si VS Code ya está abierto, también podés hacer `git pull` desde la terminal integrada de VS Code (`Ctrl + ñ` o `Ver → Terminal`).

---

## 3. Abrir el proyecto

1. En VS Code: `Archivo → Abrir Carpeta`
2. Seleccionar la carpeta **`AMR - Codigos`**
3. VS Code la reconoce automáticamente como proyecto PlatformIO

> La primera vez que abres el proyecto, PlatformIO descarga las librerías y la plataforma automáticamente. Puede tardar unos minutos. Ver la barra inferior — cuando termine desaparece el ícono giratorio.

---

## 3. Diferencias con Arduino IDE

Si venís de Arduino IDE, estas son las cosas que cambian:

### El botón "Verificar" y "Subir"

En Arduino IDE usabas los botones de arriba. En PlatformIO los encontrás en la **barra inferior de VS Code**:

```
✓  →  Compilar (equivalente a "Verificar" en Arduino)
→  →  Subir al microcontrolador (equivalente a "Subir" en Arduino)
🔌 →  Monitor Serial
```

También podés usar el panel de PlatformIO: click en el ícono del alienígena 👾 en la barra izquierda.

### El archivo `main.cpp` en lugar de `.ino`

En Arduino el archivo principal era `Nombre.ino`. Acá se llama `main.cpp` y tiene una línea extra al principio:

```cpp
#include <Arduino.h>   // ← esta línea es obligatoria en PlatformIO
```

Todo lo demás (`setup()`, `loop()`, etc.) es exactamente igual.

### Las librerías se declaran en `platformio.ini`, no se instalan a mano

En Arduino ibas a `Herramientas → Administrar librerías` y las instalabas una por una. Acá están declaradas en el archivo `platformio.ini` y PlatformIO las descarga sola:

```ini
lib_deps =
    adafruit/Adafruit MCP2515
    adafruit/Adafruit SSD1306
```

No toques esa sección a menos que necesites agregar una librería nueva.

---

## 4. Cómo está organizado este proyecto

Este proyecto tiene varios firmwares para el mismo microcontrolador (Feather RP2040), uno por subsistema y modo:

```
src/
├── tren_motriz/
│   ├── can/main.cpp      ← integración estable
│   ├── serial/main.cpp   ← calibración
│   └── test/main.cpp     ← sandbox de experimentos
├── freno/
│   ├── can/main.cpp
│   ├── serial/main.cpp
│   └── test/main.cpp
├── direccion/
│   ├── can/main.cpp
│   ├── serial/main.cpp
│   └── test/main.cpp
├── interprete/
│   ├── can/main.cpp
│   ├── serial/main.cpp
│   └── test/main.cpp
└── sensores/
    ├── can/main.cpp
    ├── serial/main.cpp
    └── test/main.cpp
```

**`can`** = versión estable de integración, se comunica por el bus CAN con el resto del robot.
**`serial`** = versión de calibración, muestra datos por el monitor serial y no usa CAN.
**`test`** = sandbox para probar cambios. Si falla, volver a flashear `_can`. No editar `can` ni `serial` mientras se experimenta.

Cuando vayas a subir código al microcontrolador, primero tenés que **seleccionar cuál firmware querés usar**.

---

## 5. Cómo seleccionar y subir el firmware correcto

### Desde la barra inferior de VS Code

En la parte de abajo de VS Code vas a ver algo así:

```
   tren_motriz_can  ✓  →  🔌
```

Click en el nombre del entorno (ej. `tren_motriz_can`) para cambiar cuál está activo. Aparece una lista con los entornos:

```
freno_can / freno_serial / freno_test
direccion_can / direccion_serial / direccion_test
tren_motriz_can / tren_motriz_serial / tren_motriz_test
interprete_can / interprete_serial / interprete_test
sensores_can / sensores_serial / sensores_test
```

Seleccioná el que corresponde al microcontrolador que tenés conectado, y recién ahí apretá el botón de subir `→`.

### Desde el panel de PlatformIO 

```
PROJECT TASKS
└── tren_motriz_can
    ├── Build       ← compilar
    ├── Upload      ← subir
    └── Monitor     ← monitor serial
```

---

## 6. Qué versión subir a cada placa

| Microcontrolador | Versión estable | Calibración | Experimentos |
|------------------|-----------------|-------------|--------------|
| Freno            | `freno_can` | `freno_serial` | `freno_test` |
| Dirección        | `direccion_can` | `direccion_serial` | `direccion_test` |
| Tren Motriz      | `tren_motriz_can` | `tren_motriz_serial` | `tren_motriz_test` |
| Intérprete       | `interprete_can` | `interprete_serial` | `interprete_test` |
| Sensores         | `sensores_can` | `sensores_serial` | `sensores_test` |

> **Regla general:** usá `_serial` solo para calibrar o diagnosticar. Para que el robot funcione completo, todas las placas tienen que tener `_can`. Usá `_test` solo para probar cambios; si algo falla, volvé a subir `_can`. La OLED de test dice `TEST` al arrancar para no confundirla con la versión estable.

---

## 7. Monitor Serial

Para ver los mensajes que manda el microcontrolador:

1. Conectar la placa por USB
2. Seleccionar el entorno correcto
3. Click en el ícono 🔌 de la barra inferior (o `Upload and Monitor`)

La velocidad ya está configurada en **115200 baud** — no hace falta cambiar nada.

> Si el monitor muestra caracteres raros o basura, verificar que la velocidad sea 115200.

---

## 8. Driver USB para subir código (Windows) — Zadig

El Feather RP2040 usa **picotool** para cargar el firmware. En Windows, la primera vez
que conectás la placa en modo bootloader, el sistema no tiene el driver correcto y
la carga falla. Hay que instalarlo una sola vez con **Zadig**.

> Este paso es **obligatorio en Windows** la primera vez que usás una placa nueva.
> Solo se hace una vez por computadora.

### Paso a paso

1. Descargar **Zadig** desde `zadig.akeo.ie` y abrirlo (no requiere instalación).

2. Poner la placa en **modo bootloader**: hacer doble tap rápido en el botón RESET.
   - La placa aparece como unidad USB llamada **`RPI-RP2`** en el Explorador de archivos.
   - Si no aparece, intentar de nuevo — el doble tap tiene que ser rápido.

3. En Zadig: **Options → List All Devices**

4. En el menú desplegable buscar **`RP2 Boot (Interface 1)`**
   - USB ID: `2E8A 0003`
   - El driver actual dice `(NONE)`

5. Asegurarse de que el driver de destino (flecha verde →) sea **`WinUSB`**

6. Click en **Install Driver** y esperar a que termine.

7. Listo. Hacer doble tap RESET de nuevo y subir el código desde PlatformIO.

> **No tocar** `Pico Serial (Interface 0)` — ese es el puerto serial normal usado por
> el Monitor Serial. Reemplazarlo con WinUSB rompe la comunicación serial.

### Cómo subir el código después de instalar el driver

Cada vez que quieras subir código:

1. Doble tap RESET → aparece la unidad `RPI-RP2`
2. En PlatformIO: seleccionar el entorno correcto y click en `→` (Upload)
3. picotool detecta la placa automáticamente y carga el firmware

---

## 9. Errores comunes al venir de Arduino

| Error | Qué significa | Solución |
|-------|--------------|----------|
| `'setup' was not declared` | Falta el `#include <Arduino.h>` | Agregar esa línea al principio del `.cpp` |
| `No such file or directory` | Librería no instalada | Verificar `lib_deps` en `platformio.ini` |
| `COM port not found` | Puerto serie no detectado | Desconectar y reconectar USB, o instalar drivers |
| El monitor serial muestra basura | Velocidad incorrecta | Verificar que sea 115200 baud |
| Sube código al firmware equivocado | Entorno incorrecto seleccionado | Verificar el entorno en la barra inferior |

---

## 9. Flujo de trabajo típico

```
1. Abrir VS Code con la carpeta AMR - Codigos
2. Conectar el microcontrolador por USB
3. Seleccionar el entorno correcto en la barra inferior
4. Editar el código en src/<subsistema>/<modo>/main.cpp
5. Click en ✓ para compilar y verificar que no haya errores
6. Click en → para subir al microcontrolador
7. Click en 🔌 para abrir el monitor serial si necesitás ver datos
```

---

## 10. Documentación de cada subsistema

Para entender qué hace cada firmware, qué pines usa y qué mensajes CAN maneja:

- [Tren Motriz](Tren_Motriz.md)
- [Freno](Freno.md)
- [Dirección](Direccion.md)
- [Intérprete](Interprete.md)
- [Proyecto general y tabla de IDs CAN](AMR1_Proyecto.md)
