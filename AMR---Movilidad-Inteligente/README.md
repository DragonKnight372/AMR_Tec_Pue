# Movilidad Inteligente 2025 - AMR1
**Códigos para Arquitectura de Control y Comunicación**

Este repositorio contiene el conjunto de códigos utilizados en **AMR1**, una plataforma de movil.
El objetivo es documentar los módulos principales, su comunicación mediante **CAN** y **Serial**.

---

## Estructura del Repositorio

El repositorio se divide en dos carpetas principales:
```
AMR---Movilidad-Inteligente/
│
├── CAN/ 
│ ├── "AMR - Comunicacion CAN" 
│ ├── "AMR - Direccion CAN"  
│ ├── "AMR - Frenos CAN" 
│ └── "AMR - Tren_Motriz CAN."  
│
└── Serial/
├── "AMR - Direccion Serial" 
├── "AMR - Frenos Serial" 
└── "AMR - Tren_Motriz Serial" 
```

---

## Módulos del AMR1

| Módulo | Microcontrolador | Descripción |
|--------|------------------|-------------|
| **Dirección** | Pro Micro | Controla el motor de 24 V encargado del giro del vehículo. |
| **Frenos** | Pro Micro | Activa el pistón actuador de freno de 12 V. |
| **Comunicación / Control** | ESP32 | Recibe comandos vía Bluetooth y Serial/CAN, coordina y reenvía mensajes a los demás módulos. |
| **Tren Motriz** | ESP32 | Controla el motor de tracción de 48 V. |

---

## Cómo usar / pruebas rápidas

1. **Pruebas unitarias (Serial)**  
   - Abrir la carpeta `Serial/` y cargar el sketch correspondiente al módulo que quieras probar:  
     - `AMR - Direccion Serial`  
     - `AMR - Frenos Serial`   
     - `AMR - Tren_Motriz Serial`  
   - Conectar el microcontrolador al PC y abrir el monitor serial para hacer pruebas y ajustes.

2. **Integración final (CAN)**  
   - Subir los sketches de la carpeta `CAN/` a cada placa según su función.  
   - Asegurarse de las terminaciones y IDs del bus CAN y de la alimentación correcta de motores/actuadores.  
   - Verificar que el módulo `AMR - Comunicacion CAN` (ESP32) esté recibiendo comandos Bluetooth y retransmitiendo por CAN.

> Antes de conectar actuadores: revisar alimentaciones y la conexión a tierra.

---
## Mejoras a Futuro Implementar

El control del AMR tiene 2 defectos: 
- El control bluethoot parece ser que se tiene que mover rapidamente dos veces para que funcione la palanca de direccion
- En modo serial se tiene que mandar dos veces la velocidad para que pueda leerla correctamente.
---
## Autores

- Franco Abraham Díez
- Yonathan Romero Amador
- Abraham Moro Hernandez
- Mariana Manjarrez Lima
- Iván Valdéz del Toro
- Pedro García Millán
