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


La solución presentada integra tres componentes principales:

1. un controlador del robot basado en ROS2,
2. una ventana gráfica para manipular el brazo de manera intuitiva,
3. herramientas para visualizar en RViz y monitorear la posición real del robot.


---

### Comunicación con los motores Dynamixel

El sistema abre el puerto serial y establece la comunicación con cada motor del brazo. Durante el arranque, se activa su capacidad de movimiento, se configura la velocidad y se envía una posición inicial estable. Si algún motor no responde, el sistema lo reporta sin detener toda la aplicación.

Se emplea una conversión interna entre ángulos que ve el usuario (en grados) y valores que entienden los motores, para que el control sea intuitivo. Esto hace que el usuario pueda mover articulaciones con valores humanos, mientras el sistema convierte esos datos al formato correcto.

---

### Publicación del estado del robot en ROS2

Mientras la aplicación está en funcionamiento, el robot publica constantemente su estado articular. Esto permite que el modelo 3D en RViz se actualice en tiempo real y se pueda observar la postura del brazo sin necesidad de verlo físicamente.

La publicación incluye los nombres de las articulaciones y los valores de cada una, expresados en radianes, siguiendo los estándares de ROS.

---

### Cálculo de la posición del efector final (TCP)

El programa calcula de forma continua el TCP.
Para ello se usan los parámetros geométricos del robot y el ángulo actual de cada articulación. El resultado es una coordenada en el espacio (X, Y, Z) que representa dónde se encuentra la punta del brazo.

Estos valores se actualizan y se muestran en la interfaz gráfica, lo cual ayuda a verificar que la postura del robot coincide con el modelo digital o con la aplicación que se esté desarrollando.

---

### Interfaz gráfica (GUI)

La ventana principal está organizada en varias pestañas, cada una diseñada para un propósito:

* **Información inicial:** Presenta datos generales del proyecto y del equipo desarrollador.

* **Control con barras deslizantes:** Permite mover cada articulación manualmente. Cuando el usuario ajusta un valor, el robot se desplaza a ese ángulo y la interfaz muestra la lectura actualizada.

* **Control por entrada numérica:** Es posible escribir valores exactos para cada articulación. El sistema los valida y los envía al robot de forma segura.

* **Visualización en RViz:** Desde la propia ventana se puede abrir RViz con el modelo del robot listo para ver. También es posible detenerlo cuando ya no se necesite.

* **Poses predefinidas:** Incluye movimientos organizados en secuencias. Cada postura se ejecuta paso a paso, sin detener la interfaz, lo que facilita demostraciones o rutinas repetibles.

* **Cálculo del TCP:** Muestra la posición actual del efector final usando los valores obtenidos por la cinemática directa.

---

### Manejo seguro y parada de emergencia

El sistema incorpora una parada de emergencia que desactiva inmediatamente la capacidad de moverse.
Mientras está activada, ningún comando se envía al robot, lo que evita movimientos accidentales.

La interfaz permite volver a activar el torque cuando sea seguro continuar.
También, al cerrar la aplicación, se ofrece confirmación para evitar dejar motores encendidos sin supervisión.

---

### Integración con ROS2 y RViz

Esta solución tiene la capacidad de trabajar directamente con el ecosistema de ROS2.
Desde la interfaz:

* se puede abrir RViz con el modelo del robot,
* se observa la postura del brazo en tiempo real,
* se puede detener el proceso sin tener que abrir terminales adicionales.

Además, el controlador tiene parámetros configurables, como el puerto, el baudrate o los identificadores de los motores, lo cual permite adaptar el sistema sin modificar el código.



---
## Diagrama de flujo:

### Diagrama general
<p align="center">
<img src="Imagenes/dgeneral.png" alt="UNAL" width="900"/>
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
<img src="Imagenes/dparada.png" alt="UNAL" width="400"/>
</p>
El diagrama de parada de emergencia y cierre seguro describe el flujo que debe seguir el sistema cuando se detecta una situación anómala o cuando el usuario solicita cerrar la aplicación. Al activar la parada de emergencia, el sistema desactiva el torque de todos los motores, marca el estado de emergencia y bloquea los controles de la interfaz hasta nueva orden. Si el usuario decide cerrar la aplicación, el sistema primero verifica si RViz está activo para detenerlo, luego apaga todos los motores, cierra el puerto de comunicación, y finalmente apaga ROS2 y libera todos los recursos, garantizando un cierre seguro y evitando daños en el hardware.

---
## Plano de planta de la ubicación de cada uno de los elementos:
---



## Descripción de las funciones utilizadas:

### 1. Funciones Auxiliares (Comunicación con Dynamixel)

#### **1.1 write_goal_position(packet, port, dxl_id, position)**

Envía un comando al motor Dynamixel para moverse a una posición especificada en ticks. Usa el protocolo de escritura adecuado (1.0 o 2.0) y devuelve el resultado de la operación.

#### **1.2 write_moving_speed(packet, port, dxl_id, speed)**

Configura la velocidad del motor escribiendo en el registro de velocidad correspondiente, usando 2 o 4 bytes según el modelo del motor.

#### **1.3 read_present_position(packet, port, dxl_id)**

Lee la posición actual de un motor Dynamixel desde su registro interno y devuelve el valor en ticks.

---

### 2. Clase PincherController (Nodo ROS2)

#### **2.1 **init**()**

Inicializa el nodo ROS2, carga parámetros, crea la comunicación con Dynamixel, configura los publicadores, asigna estructuras internas de articulaciones, inicializa los motores y calcula la posición del TCP.

#### **2.2 dh_transform(a, alpha, d, theta)**

Genera una matriz de transformación homogénea de 4×4 utilizando los parámetros Denavit–Hartenberg estándar.

#### **2.3 update_tcp_position()**

Aplica la cinemática directa multiplicando las matrices DH de cada articulación. Actualiza la posición final del TCP (x, y, z).

#### **2.4 dxl_to_radians(dxl_value)**

Convierte un valor de ticks Dynamixel al ángulo equivalente en radianes.

#### **2.5 radians_to_dxl(radians)**

Convierte un ángulo en radianes a su valor en ticks para el motor Dynamixel.

#### **2.6 dxl_to_degrees(dxl_value)**

Convierte ticks Dynamixel a grados, usando la conversión previa a radianes.

#### **2.7 publish_joint_states()**

Publica un mensaje `JointState` con las posiciones actuales de las articulaciones para su visualización en RViz.

#### **2.8 initialize_motors(goal_positions, moving_speed, torque_limit)**

Configura todos los motores habilitando torque, estableciendo su velocidad, moviéndolos a la posición HOME y sincronizando los valores internos con RViz.

#### **2.9 move_motor(motor_id, position_ticks)**

Envía un comando para mover un motor a una posición específica en ticks, actualizando además la posición interna de la articulación.

#### **2.10 update_speed_single_motor(motor_id, speed)**

Actualiza la velocidad de un motor individual escribiendo en su registro correspondiente.

#### **2.11 update_speed(speed)**

Cambia la velocidad de todos los motores simultáneamente, siempre que el sistema no esté en estado de emergencia.

#### **2.12 home_all_motors()**

Lleva todos los motores a la posición HOME (DEFAULT_GOAL). Si había una parada de emergencia, reactiva primero el torque.

#### **2.13 r2_all_motors(list_q)**

Mueve los motores 1–4 a posiciones dadas en radianes, convirtiéndolas primero a ticks Dynamixel.

#### **2.14 emergency_stop()**

Desactiva inmediatamente el torque de todos los motores y habilita la bandera de “parada de emergencia” para bloquear futuros movimientos.

#### **2.15 reactivate_torque()**

Reactiva el torque de los motores luego de una parada de emergencia.

#### **2.16 close()**

Desactiva los motores y cierra el puerto serial antes de finalizar el nodo.

---

### 3. Clase PincherGUI (Interfaz Gráfica)

#### **3.1 **init**(controller)**

Construye la ventana principal con pestañas, sliders, controles RViz, botones comunes y une la GUI con el controlador ROS2.

---

### 4. Pestañas de la GUI

#### **4.1 setup_intro_tab()**

Configura la pestaña introductoria con el título del laboratorio, integrantes y logos ASCII.

#### **4.2 setup_tab1()**

Crea la pestaña con sliders que permiten mover cada motor en tiempo real usando ángulos en grados.

#### **4.3 setup_tab2()**

Pestaña para ingresar valores numéricos de ángulos manualmente, con función de mover individual o mover todos secuencialmente.

#### **4.4 setup_tab3()**

Incluye los botones para lanzar o detener RViz y muestra en tiempo real los valores articulares publicados por el nodo.

#### **4.5 setup_tab4()**

Contiene botones para ejecutar poses preestablecidas y muestra los valores reales de cada articulación en grados.

#### **4.6 setup_tab5()**

Muestra el TCP del robot calculado mediante cinemática directa, actualizándose en tiempo real.

---

### 5. Timers y Actualizaciones Automáticas

#### **5.1 update_tcp_labels()**

Actualiza continuamente las etiquetas del TCP (x,y,z) en milímetros dentro de la pestaña 5.

#### **5.2 update_joints_timer()**

Actualiza el valor de cada articulación en todas las pestañas que lo requieren (por ejemplo RViz y Poses).

---

### 6. Eventos y Manejo de la Interfaz

#### **6.1 on_motor_slider_change(motor_id)**

Se activa al mover un slider. Convierte el valor a ticks, mueve el motor y actualiza la interfaz si no hay emergencia.

#### **6.2 on_speed_slider_change(value)**

Cambia la velocidad general del robot, sincroniza sliders entre pestañas y actualiza el estado.

#### **6.3 move_single_motor_from_entry(motor_id)**

Mueve un motor según el valor ingresado manualmente en un campo de texto, validando rango y tipo de dato.

#### **6.4 move_all_motors_from_entries()**

Lee todos los valores ingresados y mueve cada motor en secuencia con un delay de 800 ms para evitar congelamiento de la GUI.

#### **6.5 run_routine_with_delay(list_q_rad)**

Ejecuta una pose articulada (lista q1–q4) moviendo cada articulación de forma escalonada.

#### **6.6 _move_all_sequence_step(...)**

Función recursiva que implementa el movimiento secuencial con `after(800 ms)` para mantener fluida la GUI.

---

### 7. Control del Sistema y RViz

#### **7.1 home_all()**

Manda todos los motores a la posición HOME y actualiza la interfaz. Maneja el caso de emergencia previa.

#### **7.2 launch_rviz()**

Inicia RViz + robot_state_publisher en un proceso separado mediante un archivo launch.

#### **7.3 stop_rviz()**

Detiene el proceso RViz si está en ejecución.

#### **7.4 on_rviz_closed()**

Actualiza la interfaz cuando el proceso de RViz se cierra (manualmente o de forma externa).

---

### 8. Cierre del Programa

#### **8.1 on_close()**

Cierra RViz, desactiva motores, apaga ROS2 y destruye la ventana de forma segura cuando el usuario intenta salir.

#### **8.2 run()**

Inicia el ciclo principal de Tkinter (`mainloop`) y mantiene la interfaz activa hasta que sea cerrada.

---
## Código del script utilizado para el desarrollo de la práctica:

El código utilizado para implementar la solución se encuentra en la siguiente ubicación del repositorio:
[codigo.py](../Codigo/codigo.py)

---
## Vídeo del brazo alcanzando cada posición solicitada y de la interfaz de usuario desarrollada:

Link del video:
[https://drive.google.com/file/d/1SspAiQ4TFuPGkakJVEnGYX0Zsdlxu_ms/view?usp=sharing](https://drive.google.com/file/d/1SspAiQ4TFuPGkakJVEnGYX0Zsdlxu_ms/view?usp=sharing)

---
## Gráfica digital de las poses comparándola con la fotografáa del brazo real en la misma configuración:
---
