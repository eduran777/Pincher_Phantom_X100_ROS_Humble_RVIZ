# Laboratorio No. 05 Pincher Phantom X100 - ROS Humble - RVIZ

<p align="center">
<img src="Imagenes/logo_3.png" alt="UNAL" width="600"/>
</p>


### Autores:  
Esteban Durán Jiménez  
Ana María Orozco Reyes  

**FACULTAD DE INGENIERÍA**  
**ROBÓTICA**  
**2025-II**
---

## Descripción detallada de la solución planteada:

## Diagrama de flujo:

### Diagrama general
<p align="center">
<img src="Imagenes/dgeneral.png" alt="UNAL" width="850"/>
</p>
El diagrama general muestra el ciclo de vida completo de la aplicación. El proceso inicia con la configuración de ROS2 y la creación del nodo controlador del brazo robótico, pasando por la configuración y comprobación de los motores Dynamixel. Si todo es exitoso, se lanza el hilo de ROS2 y la interfaz gráfica de usuario (GUI), permitiendo al usuario interactuar con el robot mediante diversas opciones, incluida la visualización en RViz. Todo el flujo está pensado para la seguridad, el monitoreo de estados y la capacidad de cerrar la aplicación de manera controlada, incluso ante errores de conexión o cierre inesperado.

### Inicialización del nodo y motores

<p align="center">
<img src="Imagenes/dnodoymotores.png" alt="UNAL" width="500"/>
</p>
Durante la inicialización, el sistema realiza una serie de comprobaciones críticas. Tras arrancar ROS2, se instancia el nodo controlador, que se encarga de obtener los parámetros de configuración desde archivos o argumentos, y de abrir y configurar el puerto de comunicación con los motores Dynamixel. Si cualquier etapa falla, el sistema muestra el error y apaga todo de forma segura. Si el puerto y los parámetros son correctos, se procede a configurar cada motor: habilitar torque, establecer velocidad y posición inicial (HOME), y preparar los publicadores para enviar estados articulares al entorno ROS2.

### Lógica principal y acciones de usuario
<p align="center">
<img src="Imagenes/dusuario.png" alt="UNAL" width="700"/>
</p>
En el diagrama de lógica principal, el usuario puede interactuar con la aplicación mediante la GUI, seleccionando diferentes acciones como mover motores individualmente o todos a la vez, cambiar la velocidad, ejecutar rutinas de movimientos predefinidos ("poses"), enviar a HOME, lanzar o detener la visualización en RViz y activar la parada de emergencia. Cada acción se valida y ejecuta, tras lo cual el sistema retorna al estado de espera de nuevas instrucciones, proporcionando una operación segura y fluida.

### Diagrama vertical de parada de emergencia y cierre seguro
<p align="center">
<img src="Imagenes/dparada.png" alt="UNAL" width="500"/>
</p>
El diagrama de parada de emergencia y cierre seguro describe el flujo que debe seguir el sistema cuando se detecta una situación anómala o cuando el usuario solicita cerrar la aplicación. Al activar la parada de emergencia, el sistema desactiva el torque de todos los motores, marca el estado de emergencia y bloquea los controles de la interfaz hasta nueva orden. Si el usuario decide cerrar la aplicación, el sistema primero verifica si RViz está activo para detenerlo, luego apaga todos los motores, cierra el puerto de comunicación, y finalmente apaga ROS2 y libera todos los recursos, garantizando un cierre seguro y evitando daños en el hardware.

---
## Plano de planta de la ubicación de cada uno de los elementos:
---
## Descripción de las funciones utilizadas:
---
## Código del script utilizado para el desarrollo de la práctica:
---
## Vídeo del brazo alcanzando cada posición solicitada:
---
## Vídeo demostración de uso de la interfaz de usuario:
---
## Gráfica digital de las poses comparándola con la fotografáa del brazo real en la misma configuración:
---
