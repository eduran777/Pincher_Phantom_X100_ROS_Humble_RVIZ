import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import subprocess
import os
import math
import numpy as np

# ============================================================
#  CONFIGURACIÓN: ¿QUÉ MOTORES ESTÁS USANDO?
# ============================================================
USE_XL430 = False

# Rango de ángulos para la GUI (en grados)
ANGLE_MIN = -150.0
ANGLE_MAX = 150.0

# ------------------------------------------------------------
# Direcciones de registro y parámetros según el tipo de motor
# ------------------------------------------------------------
if USE_XL430:
    PROTOCOL_VERSION = 2.0
    ADDR_TORQUE_ENABLE    = 64
    ADDR_GOAL_POSITION    = 116
    ADDR_MOVING_SPEED     = 112  # Profile Velocity
    ADDR_TORQUE_LIMIT     = 38
    ADDR_PRESENT_POSITION = 132
    DEFAULT_GOAL = 2048
    MAX_SPEED = 1023  # Velocidad máxima para XL430
else:
    PROTOCOL_VERSION = 1.0
    ADDR_TORQUE_ENABLE    = 24
    ADDR_GOAL_POSITION    = 30
    ADDR_MOVING_SPEED     = 32
    ADDR_TORQUE_LIMIT     = 34
    ADDR_PRESENT_POSITION = 36
    DEFAULT_GOAL = 512
    MAX_SPEED = 1023  # Velocidad máxima para otros modelos

# ============================================================
#  FUNCIONES AUXILIARES
# ============================================================

def write_goal_position(packet, port, dxl_id, position):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))

def write_moving_speed(packet, port, dxl_id, speed):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))

def read_present_position(packet, port, dxl_id):
    if USE_XL430:
        return packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    else:
        return packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)

# ============================================================
#  NODO ROS2 CON PUBLICACIÓN PARA RViz
# ============================================================

class PincherController(Node):
    def __init__(self):
        super().__init__('pincher_controller')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('goal_positions', [DEFAULT_GOAL] * 5)
        self.declare_parameter('moving_speed', 100)
        self.declare_parameter('torque_limit', 800)

        # Obtener parámetros
        port_name = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.dxl_ids = self.get_parameter('dxl_ids').value
        goal_positions = self.get_parameter('goal_positions').value
        moving_speed = int(self.get_parameter('moving_speed').value)
        torque_limit = int(self.get_parameter('torque_limit').value)

        # Inicializar comunicación
        self.port = PortHandler(port_name)
        if not self.port.openPort():
            self.get_logger().error(f'No se pudo abrir el puerto {port_name}')
            rclpy.shutdown()
            return

        if not self.port.setBaudRate(baudrate):
            self.get_logger().error(f'No se pudo configurar baudrate={baudrate}')
            self.port.closePort()
            rclpy.shutdown()
            return

        self.packet = PacketHandler(PROTOCOL_VERSION)
        
        # Estado de emergencia
        self.emergency_stop_activated = False
        
        # Publicador para Joint States (para RViz)
        from sensor_msgs.msg import JointState
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer para publicar joint states
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        
        # Posiciones actuales de las articulaciones (en radianes / metros)
        self.current_joint_positions = [0.0] * 5  # Para 5 articulaciones / ejes
        
        # Mapeo de IDs de motor a nombres de articulaciones del URDF
        self.joint_names = [
            'phantomx_pincher_arm_shoulder_pan_joint',
            'phantomx_pincher_arm_shoulder_lift_joint',
            'phantomx_pincher_arm_elbow_flex_joint',
            'phantomx_pincher_arm_wrist_flex_joint',
            'phantomx_pincher_gripper_finger1_joint',  # usamos el dedo 1 como gripper
        ]

        self.joint_sign = {
            1:  1,   
            2:  1,   
            3:  1,   
            4:  1,   
            5:  1,   
        }

        self.L1 = 0.0415
        self.L2 = 0.107
        self.L3 = 0.107

        self.tcp_position = (0.0, 0.0, 0.0)

        # Configuración inicial de los motores
        self.initialize_motors(goal_positions, moving_speed, torque_limit)
        self.update_tcp_position()

    # ===================== CINEMÁTICA DIRECTA =====================

    def dh_transform(self, a, alpha, d, theta):
        """Matriz de transformación homogénea usando parámetros DH estándar."""
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        ct = math.cos(theta)
        st = math.sin(theta)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,    sa,       ca,       d],
            [0.0,   0.0,      0.0,     1.0]
        ], dtype=float)

    def update_tcp_position(self):
        """
        Actualiza la posición del TCP (frame 4) usando la tabla DH:
        i   θ_i   d_i   a_i   α_i
        1   q1    L1    0    -pi/2
        2   q2     0    L2    0
        3   q3     0    L3    0
        4   q4     0    0    -pi/2
        """
        try:
            q1, q2, q3, q4 = self.current_joint_positions[:4]
        except Exception as e:
            self.get_logger().error(f'No se pudo leer las articulaciones para FK: {e}')
            return self.tcp_position

        T1 = self.dh_transform(0.0, -math.pi/2, self.L1, q1)
        T2 = self.dh_transform(self.L2, 0.0, 0.0, q2 - math.pi/2)
        T3 = self.dh_transform(self.L3, 0.0, 0.0, q3)
        T4 = self.dh_transform(0.0, -math.pi/2, 0.0, q4)

        self.get_logger().debug('Calculando TCP mediante FK')

        T = T1 @ T2 @ T3 @ T4

        x = float(T[0, 3])
        y = float(T[1, 3])
        z = float(T[2, 3])

        self.tcp_position = (x, y, z)

        return self.tcp_position

    # ===================== CONVERSIÓN DXL <-> RAD/DEG =====================

    def dxl_to_radians(self, dxl_value):
        """Convierte valor Dynamixel a radianes (-2.618 a 2.618 aprox.)."""
        if USE_XL430:
            center = 2048.0
            scale = 2.618 / 2048.0
        else:
            center = 512.0
            scale = 2.618 / 512.0
        return (dxl_value - center) * scale

    def radians_to_dxl(self, radians):
        """Convierte radianes a valor Dynamixel."""
        if USE_XL430:
            center = 2048.0
            inv_scale = 2048.0 / 2.618
        else:
            center = 512.0
            inv_scale = 512.0 / 2.618
        return int(radians * inv_scale + center)

    def dxl_to_degrees(self, dxl_value):
        """Convierte ticks Dynamixel a grados."""
        return self.dxl_to_radians(dxl_value) * 180.0 / math.pi

    # ===================== PUBLICACIÓN JOINT STATES =====================

    def publish_joint_states(self):
        """Publica el estado de las articulaciones para RViz"""
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Header
        
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "base_link"
        
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        
        self.joint_state_pub.publish(joint_state)

    # ===================== CONTROL DE MOTORES =====================

    def initialize_motors(self, goal_positions, moving_speed, torque_limit):
        """Configuración inicial de todos los motores"""
        for dxl_id, goal in zip(self.dxl_ids, goal_positions):
            try:
                # Habilitar torque
                result, error = self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                if result != 0:
                    self.get_logger().error(f'Error habilitando torque en motor {dxl_id}: {error}')
                    continue
                
                # Configurar velocidad
                self.update_speed_single_motor(dxl_id, moving_speed)
                
                # Mover a posición inicial
                write_goal_position(self.packet, self.port, dxl_id, goal)
                
                # Actualizar posición de articulación para RViz
                joint_index = self.dxl_ids.index(dxl_id)
                angle = self.dxl_to_radians(goal)
                angle *= self.joint_sign.get(dxl_id, 1)
                self.current_joint_positions[joint_index] = angle
                
                self.get_logger().info(f'Motor {dxl_id} configurado correctamente')
                
            except Exception as e:
                self.get_logger().error(f'Error configurando motor {dxl_id}: {str(e)}')

    def move_motor(self, motor_id, position_ticks):
        """Mueve un motor a la posición especificada en ticks"""
        if self.emergency_stop_activated:
            self.get_logger().warning(f'No se puede mover motor {motor_id}: Parada de emergencia activada')
            return
            
        try:
            result, error = write_goal_position(self.packet, self.port, motor_id, position_ticks)
            if result == 0:
                self.get_logger().info(f'[Motor {motor_id}] Moviendo a {position_ticks} ticks')
                
                # Actualizar posición de articulación para RViz
                joint_index = self.dxl_ids.index(motor_id)
                angle = self.dxl_to_radians(position_ticks)
                angle *= self.joint_sign.get(motor_id, 1)
                self.current_joint_positions[joint_index] = angle
                
            else:
                self.get_logger().error(f'Error moviendo motor {motor_id}: {error}')
        except Exception as e:
            self.get_logger().error(f'Excepción moviendo motor {motor_id}: {str(e)}')

    def update_speed_single_motor(self, motor_id, speed):
        """Actualiza la velocidad de un motor individual"""
        try:
            result, error = write_moving_speed(self.packet, self.port, motor_id, speed)
            return result == 0
        except Exception as e:
            self.get_logger().error(f'Error actualizando velocidad motor {motor_id}: {str(e)}')
            return False

    def update_speed(self, speed):
        """Actualiza la velocidad de movimiento en todos los motores"""
        if self.emergency_stop_activated:
            self.get_logger().warning('No se puede actualizar velocidad: Parada de emergencia activada')
            return
            
        success_count = 0
        for motor_id in self.dxl_ids:
            if self.update_speed_single_motor(motor_id, speed):
                success_count += 1
        
        if success_count == len(self.dxl_ids):
            self.get_logger().info(f'Velocidad actualizada a {speed} en todos los motores')
        else:
            self.get_logger().warning(f'Velocidad actualizada a {speed} en {success_count}/{len(self.dxl_ids)} motores')

    def home_all_motors(self):
        """Mueve todos los motores a la posición home (DEFAULT_GOAL)"""
        if self.emergency_stop_activated:
            # Reactivar torque si hay emergencia
            self.reactivate_torque()
            
        for motor_id in self.dxl_ids:
            self.move_motor(motor_id, DEFAULT_GOAL)
        self.get_logger().info('Todos los motores movidos a posición HOME')

    def r2_all_motors(self, list_q):
        """
        Mueve q1..q4 en radianes (lista de longitud >= 4).
        Ignora el motor 5 (pinza) para esta rutina.
        """
        if self.emergency_stop_activated:
            self.reactivate_torque()

        # Convertir radianes a Dynamixel
        point_q = [self.radians_to_dxl(q) for q in list_q]

        # Enviar cada posición al motor correspondiente
        for index, motor_id in enumerate(self.dxl_ids):
            if motor_id == 5:      # no mover la pinza
                break

            self.move_motor(motor_id, point_q[index])

        self.get_logger().info('Todos los motores han sido movidos a las posiciones')

    # ===================== EMERGENCIA / CIERRE =====================

    def emergency_stop(self):
        """Parada de emergencia - desactiva el torque de todos los motores"""
        self.emergency_stop_activated = True
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.get_logger().warning(f'Torque desactivado en motor {dxl_id} (EMERGENCY STOP)')
            except Exception as e:
                self.get_logger().error(f'Error en parada de emergencia motor {dxl_id}: {str(e)}')

    def reactivate_torque(self):
        """Reactivar el torque después de una parada de emergencia"""
        self.emergency_stop_activated = False
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                self.get_logger().info(f'Torque reactivado en motor {dxl_id}')
            except Exception as e:
                self.get_logger().error(f'Error reactivando torque en motor {dxl_id}: {str(e)}')

    def close(self):
        """Apaga el torque y cierra el puerto"""
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            except:
                pass
        self.port.closePort()

# ============================================================
#  INTERFAZ GRÁFICA CON PESTAÑAS (INCLUYENDO RViz)
# ============================================================

class PincherGUI:
    def __init__(self, controller):
        self.controller = controller
        self.window = tk.Tk()
        self.window.title("Control Pincher - Interfaz Completa")
        self.window.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # Variables para control de actualización
        self.last_motor_update = {motor_id: 0 for motor_id in controller.dxl_ids}
        self.last_speed_update = 0
        self.update_interval = 0.05  # 50ms entre actualizaciones
        
        # Proceso de RViz
        self.rviz_process = None
        
        # Crear notebook (pestañas)
        self.notebook = ttk.Notebook(self.window)
        self.notebook.pack(fill='both', expand=True, padx=10, pady=10)
        
        # 🔹 NUEVA PESTAÑA INTRODUCTORIA (antes de las demás)
        self.tab_intro = ttk.Frame(self.notebook)
        self.tab1 = ttk.Frame(self.notebook)
        self.tab2 = ttk.Frame(self.notebook)
        self.tab3 = ttk.Frame(self.notebook)  # Nueva pestaña para RViz
        self.tab4 = ttk.Frame(self.notebook)  # Nueva pestaña TCP/FK
        self.tab5 = ttk.Frame(self.notebook)  # Nueva pestaña TCP/FK

        # Añadir primero la pestaña introductoria
        self.notebook.add(self.tab_intro, text='Pestaña introductoria')
        self.notebook.add(self.tab1, text='Control por Sliders')
        self.notebook.add(self.tab2, text='Control por Valores')
        self.notebook.add(self.tab3, text='Visualización RViz')
        self.notebook.add(self.tab4, text='Poses')
        self.notebook.add(self.tab5, text='Cinematica Directa')
        
        # Configurar las pestañas
        self.setup_intro_tab()   # 🔹 nueva
        self.setup_tab1()
        self.setup_tab2()
        self.setup_tab3()
        self.setup_tab4()
        self.setup_tab5()
        
        # Barra de botones comunes en la parte inferior
        self.setup_common_buttons()

    # ===================== NUEVA PESTAÑA INTRODUCTORIA =====================

    def setup_intro_tab(self):
        """Pestaña introductoria con nombres del grupo"""

        # ---------- TÍTULO PRINCIPAL ----------
        title_label = tk.Label(
            self.tab_intro,
            text="Laboratorio No. 05\nCinemática Directa - Pincher Phantom X100",
            font=("Arial", 18, "bold"),
            pady=10
        )
        title_label.pack()

        # ---------- SUBTÍTULO ----------
        subtitle_label = tk.Label(
            self.tab_intro,
            text="Robótica 2025-II  |  Universidad Nacional de Colombia",
            font=("Arial", 12, "italic")
        )
        subtitle_label.pack(pady=(0, 20))

        # ---------- CONTENEDOR ----------
        main_frame = tk.Frame(self.tab_intro)
        main_frame.pack(pady=20)

        # ---------- INFO DE INTEGRANTES ----------
        info_frame = tk.Frame(main_frame)
        info_frame.pack(side="left", padx=30)

        tk.Label(
            info_frame,
            text="Integrantes:",
            font=("Arial", 14, "bold"),
            anchor="w"
        ).pack(anchor="w")

        integrantes = [
            "Esteban Durán Jiménez",
            "Ana María Orozco Reyes"
        ]

        for name in integrantes:
            tk.Label(
                info_frame,
                text=name,
                font=("Arial", 12),
                anchor="w"
            ).pack(anchor="w", pady=4)

        # ---------- LOGOS ASCII (PARTE INFERIOR) ----------
        logos_frame = tk.Frame(self.tab_intro)
        logos_frame.pack(pady=40)

        tk.Label(
            logos_frame,
            text="Logos del Proyecto:",
            font=("Arial", 14, "bold"),
            anchor="center"
        ).pack()

        # =============== ASCII ART LOGOS ===============
        logo_robot = r"""
        [::]
        |  o o |
        |   >  |
        \ -- /
        |  |
        --|  |--
            --
        """

        logo_unal = r"""
        _________
        |  UNAL  |
        |========|
        |   ♣    |
        |________|
        """

        # Mostrar logos en Tkinter
        tk.Label(
            logos_frame,
            text=logo_robot,
            font=("Courier", 10),
            justify="center"
        ).pack(side="left", padx=10)

        tk.Label(
            logos_frame,
            text=logo_unal,
            font=("Courier", 10),
            justify="center"
        ).pack(side="left", padx=10)



    # ===================== TAB 1 =====================

    def setup_tab1(self):
        """Configura la pestaña 1: Control por Sliders en Tiempo Real (Ángulos)"""
        title_label = tk.Label(self.tab1, text="Control por Sliders - TIEMPO REAL (Ángulos en grados)", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=10)

        motors_frame = tk.Frame(self.tab1)
        motors_frame.pack(fill='x', padx=20, pady=10)
        
        self.sliders = {}
        self.labels = {}
        
        # Ángulo HOME en grados calculado desde DEFAULT_GOAL
        home_deg = self.controller.dxl_to_degrees(DEFAULT_GOAL)
        
        # Sliders para cada motor
        for i, motor_id in enumerate(self.controller.dxl_ids):
            motor_frame = tk.Frame(motors_frame)
            motor_frame.pack(fill='x', pady=5)
            
            # Label del motor
            motor_label = tk.Label(motor_frame, text=f'Motor {motor_id}', 
                                  font=("Arial", 10, "bold"), width=8)
            motor_label.pack(side='left', padx=5)
            
            # Slider en grados
            slider = tk.Scale(
                motor_frame,
                from_=ANGLE_MIN,
                to=ANGLE_MAX,
                orient=tk.HORIZONTAL, 
                length=400,
                showvalue=True,
                resolution=0.1,
                command=lambda value, mid=motor_id: self.on_motor_slider_change(mid)
            )
            slider.set(home_deg)
            slider.pack(side='left', fill='x', expand=True, padx=5)
            
            # Etiqueta de ángulo
            label = tk.Label(motor_frame, text=f'Ángulo: {home_deg:.1f}°', 
                            font=("Arial", 9), width=16)
            label.pack(side='right', padx=5)
            
            self.sliders[motor_id] = slider
            self.labels[motor_id] = label
        
        # Separador
        ttk.Separator(self.tab1, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        # Frame para control de velocidad
        speed_frame = tk.Frame(self.tab1)
        speed_frame.pack(fill='x', padx=20, pady=10)
        
        speed_label = tk.Label(speed_frame, text="Velocidad de Movimiento:", 
                              font=("Arial", 10, "bold"))
        speed_label.pack(anchor='w')
        
        speed_control_frame = tk.Frame(speed_frame)
        speed_control_frame.pack(fill='x', pady=5)
        
        self.speed_slider_tab1 = tk.Scale(speed_control_frame, from_=0, to=MAX_SPEED, 
                                         orient=tk.HORIZONTAL, length=400,
                                         showvalue=True, resolution=1,
                                         command=self.on_speed_slider_change)
        self.speed_slider_tab1.set(100)
        self.speed_slider_tab1.pack(side='left', fill='x', expand=True)
        
        self.speed_value_label_tab1 = tk.Label(speed_control_frame, text="100", 
                                              font=("Arial", 10, "bold"), width=5)
        self.speed_value_label_tab1.pack(side='right', padx=10)
        
        # Nota sobre velocidad 0
        speed_note = tk.Label(speed_frame, text="Nota: Velocidad 0 = No movimiento", 
                             font=("Arial", 8), fg="gray")
        speed_note.pack(anchor='w')

    # ===================== TAB 2 =====================

    def setup_tab2(self):
        """Configura la pestaña 2: Control por Valores (Ángulos)"""
        title_label = tk.Label(self.tab2, text="Control por Valores Manuales (Ángulos en grados)", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # Frame para control de velocidad
        speed_frame = tk.Frame(self.tab2)
        speed_frame.pack(fill='x', padx=20, pady=10)
        
        speed_label = tk.Label(speed_frame, text="Velocidad de Movimiento:", 
                              font=("Arial", 10, "bold"))
        speed_label.pack(anchor='w')
        
        speed_control_frame = tk.Frame(speed_frame)
        speed_control_frame.pack(fill='x', pady=5)
        
        self.speed_slider_tab2 = tk.Scale(speed_control_frame, from_=0, to=MAX_SPEED, 
                                         orient=tk.HORIZONTAL, length=400,
                                         showvalue=True, resolution=1,
                                         command=self.on_speed_slider_change)
        self.speed_slider_tab2.set(100)
        self.speed_slider_tab2.pack(side='left', fill='x', expand=True)
        
        self.speed_value_label_tab2 = tk.Label(speed_control_frame, text="100", 
                                              font=("Arial", 10, "bold"), width=5)
        self.speed_value_label_tab2.pack(side='right', padx=10)
        
        # Nota sobre rango de ángulos
        speed_note = tk.Label(
            speed_frame,
            text=f"Nota: Rango válido de ángulo: {ANGLE_MIN:.0f}° a {ANGLE_MAX:.0f}°", 
            font=("Arial", 8), fg="gray"
        )
        speed_note.pack(anchor='w')
        
        # Separador
        ttk.Separator(self.tab2, orient='horizontal').pack(fill='x', padx=20, pady=10)
        
        # Frame para control individual de motores
        motors_frame = tk.Frame(self.tab2)
        motors_frame.pack(fill='both', expand=True, padx=20, pady=10)
        
        self.entries = {}
        self.entry_labels = {}
        
        home_deg = self.controller.dxl_to_degrees(DEFAULT_GOAL)
        
        # Crear filas para cada motor
        for i, motor_id in enumerate(self.controller.dxl_ids):
            motor_frame = tk.Frame(motors_frame)
            motor_frame.pack(fill='x', pady=8)
            
            # Label del motor
            motor_label = tk.Label(motor_frame, text=f'Motor {motor_id}', 
                                  font=("Arial", 10, "bold"), width=8)
            motor_label.pack(side='left', padx=5)
            
            # Entry para valor en ángulos
            entry_label = tk.Label(motor_frame, text="Ángulo [°]:", 
                                  font=("Arial", 9))
            entry_label.pack(side='left', padx=5)
            
            entry = tk.Entry(motor_frame, width=8, font=("Arial", 10))
            entry.insert(0, f"{home_deg:.1f}")
            entry.pack(side='left', padx=5)
            
            # Botón para mover motor individual
            move_btn = tk.Button(motor_frame, text="Mover Motor", 
                                font=("Arial", 9),
                                command=lambda mid=motor_id: self.move_single_motor_from_entry(mid))
            move_btn.pack(side='left', padx=5)
            
            # Etiqueta de estado
            status_label = tk.Label(motor_frame, text="Listo", 
                                   font=("Arial", 9), fg="green", width=10)
            status_label.pack(side='right', padx=5)
            
            self.entries[motor_id] = entry
            self.entry_labels[motor_id] = status_label
            
            # Bind Enter key para mover automáticamente
            entry.bind('<Return>', lambda event, mid=motor_id: self.move_single_motor_from_entry(mid))
        
        # Botón para mover todos los motores
        move_all_frame = tk.Frame(motors_frame)
        move_all_frame.pack(fill='x', pady=15)
        
        move_all_btn = tk.Button(move_all_frame, text="MOVER TODOS LOS MOTORES", 
                                font=("Arial", 10, "bold"), bg="#4CAF50", fg="white",
                                command=self.move_all_motors_from_entries)
        move_all_btn.pack(pady=10)

    # ===================== TAB 3 (RViz) =====================

    def setup_tab3(self):
        """Configura la pestaña 3: Visualización en RViz"""
        title_label = tk.Label(self.tab3, text="Visualización en RViz - PhantomX Pincher X100", 
                              font=("Arial", 14, "bold"))
        title_label.pack(pady=10)
        
        # Frame de información
        info_frame = tk.Frame(self.tab3)
        info_frame.pack(fill='x', padx=20, pady=10)
        
        info_text = """
Esta pestaña permite visualizar el robot PhantomX Pincher X100 en RViz.

Características:
• Modelo 3D del PhantomX Pincher X100
• Sincronización en tiempo real con el robot físico
• Visualización de todas las articulaciones
• Feedback visual de los movimientos
        """
        info_label = tk.Label(info_frame, text=info_text, justify=tk.LEFT, 
                             font=("Arial", 10), bg="#f0f0f0", relief="solid", padx=10, pady=10)
        info_label.pack(fill='x')
        
        # Frame de controles RViz
        controls_frame = tk.Frame(self.tab3)
        controls_frame.pack(fill='x', padx=20, pady=20)
        
        # Botón para lanzar RViz
        self.rviz_btn = tk.Button(controls_frame, text="LANZAR RViz", 
                                 font=("Arial", 12, "bold"), bg="#2196F3", fg="white",
                                 command=self.launch_rviz, height=2)
        self.rviz_btn.pack(fill='x', pady=5)
        
        # Botón para detener RViz
        self.stop_rviz_btn = tk.Button(controls_frame, text="DETENER RViz", 
                                      font=("Arial", 10), bg="#f44336", fg="white",
                                      command=self.stop_rviz, state=tk.DISABLED)
        self.stop_rviz_btn.pack(fill='x', pady=5)
        
        # Estado de RViz
        self.rviz_status_label = tk.Label(controls_frame, text="RViz no iniciado", 
                                         font=("Arial", 10), fg="red")
        self.rviz_status_label.pack(pady=10)
        
        # Frame de información de articulaciones
        joints_frame = tk.Frame(self.tab3)
        joints_frame.pack(fill='x', padx=20, pady=10)
        
        joints_label = tk.Label(joints_frame, text="Posiciones de Articulaciones (radianes):", 
                               font=("Arial", 10, "bold"))
        joints_label.pack(anchor='w')
        
        # Etiquetas para mostrar posiciones actuales
        self.joint_labels = {}
        for i, joint_name in enumerate(self.controller.joint_names):
            joint_frame = tk.Frame(joints_frame)
            joint_frame.pack(fill='x', pady=2)
            
            label = tk.Label(joint_frame, text=f"{joint_name}:", 
                            font=("Arial", 9), width=30, anchor='w')
            label.pack(side='left')
            
            value_label = tk.Label(joint_frame, text="0.000", 
                                  font=("Arial", 9), width=10)
            value_label.pack(side='left')
            
            self.joint_labels[joint_name] = value_label
        
        # Timer para actualizar las posiciones de las articulaciones
        self.update_joints_timer()

    # ===================== TAB 4 (Posiciones - Valores articulares reales) =====================
    def setup_tab4(self):
        "Configura 5 Poses de prueba y valores articulars reales"
        title_label = tk.Label(self.tab4, text="Poses - Valores Reales Articulares",
                               font=("Arial", 20, "bold"))
        title_label.pack(pady=10)

        info_frame = tk.Frame(self.tab4)
        info_frame.pack(fill='x', padx=20, pady=10)


        # ====== CONTENEDOR CENTRAL: columna de botones + columna de articulaciones ======
        main_frame = tk.Frame(self.tab4)
        main_frame.pack(fill='both', expand=True, padx=20, pady=20)

        # -------- Columna izquierda: botones de rutinas (vertical) --------
        buttons_frame = tk.Frame(main_frame)
        buttons_frame.pack(side='left', fill='y', padx=(0, 30))  # separación con la otra columna

        button_test1 = tk.Button(buttons_frame, text="Pose1", width=12, command=self.home_all, bg="#2ECC71", fg="white", activebackground="#27AE60", activeforeground="white")
        button_test1.pack(side='top', fill='x', pady=10)

        # AHORA LAS POSICIONES run_routine_with_delay (con after y delay)
        button_test2 = tk.Button(
            buttons_frame,
            text="Pose 2",
            width=12,
            command=lambda: self.run_routine_with_delay([0.44, 0.44, 0.35, -0.35, 0]),
            bg="#2ECC71", fg="white", activebackground="#27AE60", activeforeground="white"
        
        )
        button_test2.pack(side='top', fill='x', pady=5)

        button_test3 = tk.Button(
            buttons_frame,
            text="Pose 3",
            width=12,
            command=lambda: self.run_routine_with_delay([-0.61, 0.61, -0.52, 0.52, 0]),
            bg="#2ECC71", fg="white", activebackground="#27AE60", activeforeground="white"
        )
        button_test3.pack(side='top', fill='x', pady=5)

        button_test4 = tk.Button(
            buttons_frame,
            text="Pose 4",
            width=12,
            command=lambda: self.run_routine_with_delay([1.48, 0.35, 0.96, 0.44, 0]),
            bg="#2ECC71", fg="white", activebackground="#27AE60", activeforeground="white"
        )
        button_test4.pack(side='top', fill='x', pady=5)

        button_test5 = tk.Button(
            buttons_frame,
            text="Pose 5",
            width=12,
            command=lambda: self.run_routine_with_delay([1.40, -0.61, 0.96, -0.78, 0]),
            bg="#2ECC71", fg="white", activebackground="#27AE60", activeforeground="white"
        )
        button_test5.pack(side='top', fill='x', pady=5)

        # -------- Columna derecha: info de articulaciones --------
        joints_frame = tk.Frame(main_frame)
        joints_frame.pack(side='left', fill='both', expand=True)

        joints_label = tk.Label(
            joints_frame,
            text="Valores Articulares Reales (grados):",
            font=("Arial", 15, "bold"),
            anchor='w',
            justify='left'
        )
        joints_label.pack(anchor='w', pady=(0, 5),padx=30)

        # Etiquetas para mostrar posiciones actuales
        self.joint_labels = {}
        for i, joint_name in enumerate(self.controller.joint_names):
            joint_row = tk.Frame(joints_frame)
            joint_row.pack(fill='x', pady=5,padx=30)

            label = tk.Label(joint_row, text=f"Articulacion "+str(i), 
                             font=("Arial", 12), width=15, anchor='w', justify='left')
            label.pack(side='left')

            value_label = tk.Label(joint_row, text="0.000", 
                                   font=("Arial", 9), width=10, anchor='w')
            value_label.pack(side='left')

            self.joint_labels[joint_name] = value_label

        # Timer para actualizar las posiciones de las articulaciones
        self.update_joints_timer()

        # ===================== Tab 5 =====================
    def setup_tab5(self):
        """
        Cinemática Directa 
        """
        title_label = tk.Label(
            self.tab5,
            text="Cinemática Directa",
            font=("Arial", 30, "bold")
        )
        title_label.pack(pady=20)

        subtitle_label = tk.Label(
            self.tab5,
            text="A continuación se observa el TCP obtenido con la cinemática directa",
            font=("Arial", 15)
        )
        subtitle_label.pack(pady=10)

        self.controller.update_tcp_position()

        # ======= CONTENEDOR HORIZONTAL (valores + imagen) =======
        main_frame = tk.Frame(self.tab5)
        main_frame.pack(fill='x', padx=30, pady=40)

        # ---------------- VALORES TCP (lado izquierdo) ----------------
        values_frame = tk.Frame(main_frame)
        values_frame.pack(side='left', fill='y', padx=(50, 50))  # separa la imagen

        self.tcp_labels = {}

        col = ["red","green","blue"]

        for i,coord in enumerate(["X", "Y", "Z"]):
            row = tk.Frame(values_frame)
            row.pack(fill='x', pady=5)

            lbl = tk.Label(row, text=f"{coord} [mm]:",
                        font=("Arial", 20, "bold"), width=10, anchor='w',fg=col[i])
            lbl.pack(side='left')

            value_lbl = tk.Label(row, text="0.0",
                                font=("Arial", 15), width=12, anchor='w',fg=col[i])
            value_lbl.pack(side='left')

            self.tcp_labels[coord.lower()] = value_lbl
       

        # Actualización periódica de los valores TCP
        self.update_tcp_labels()



    # ===================== TIMERS =====================

    def update_tcp_labels(self):
        """Actualiza periódicamente las etiquetas del TCP en la pestaña 4"""
        self.controller.update_tcp_position()

        x, y, z = self.controller.tcp_position
        # Convertir a mm para visualización
        self.tcp_labels['x'].config(text=f"{x * 1000.0:.1f}")
        self.tcp_labels['y'].config(text=f"{y * 1000.0:.1f}")
        self.tcp_labels['z'].config(text=f"{z * 1000.0:.1f}")

        # Programar siguiente actualización
        self.window.after(50, self.update_tcp_labels)

    def update_joints_timer(self):
        """Actualiza periódicamente las posiciones de las articulaciones en la interfaz"""

        for i, joint_name in enumerate(self.controller.joint_names):
            if i < len(self.controller.current_joint_positions):
                pos_rad = self.controller.current_joint_positions[i]
                pos_deg = math.degrees(pos_rad)   # conversión rad -> deg

                if joint_name in self.joint_labels:
                    self.joint_labels[joint_name].config(text=f"{pos_deg:.1f}")
        
        # Programar siguiente actualización
        self.window.after(100, self.update_joints_timer)

    # ===================== EVENTOS GUI =====================

    def on_motor_slider_change(self, motor_id):
        """Se ejecuta CADA VEZ que se mueve el slider del motor (Pestaña 1)"""
        current_time = time.time()
        
        # Control de frecuencia de actualización para no saturar
        if current_time - self.last_motor_update[motor_id] >= self.update_interval:
            angle_deg = float(self.sliders[motor_id].get())
            angle_rad = math.radians(angle_deg)
            ticks = self.controller.radians_to_dxl(angle_rad)
            
            # Solo mover si la velocidad no es 0 y no hay emergencia
            speed = self.speed_slider_tab1.get()
            if speed > 0 and not self.controller.emergency_stop_activated:
                self.controller.move_motor(motor_id, ticks)
                self.labels[motor_id].config(text=f'Ángulo: {angle_deg:.1f}°')
                self.last_motor_update[motor_id] = current_time
                
                # Actualizar estado brevemente
                self.status_label.config(text=f"Motor {motor_id} moviéndose a {angle_deg:.1f}°", fg="green")
                self.window.after(2000, lambda: self.status_label.config(text="Sistema Listo", fg="green"))
            elif self.controller.emergency_stop_activated:
                self.status_label.config(text=f"EMERGENCIA: No se puede mover motor {motor_id}", fg="red")
            else:
                self.status_label.config(text=f"Velocidad 0: Motor {motor_id} no se moverá", fg="orange")

    def on_speed_slider_change(self, value):
        """Se ejecuta CADA VEZ que se mueve el slider de velocidad"""
        current_time = time.time()
        
        # Control de frecuencia de actualización
        if current_time - self.last_speed_update >= self.update_interval:
            speed = int(value)
            
            # Solo actualizar velocidad si no hay emergencia
            if not self.controller.emergency_stop_activated:
                self.controller.update_speed(speed)
                
                # Actualizar ambas etiquetas de velocidad
                self.speed_value_label_tab1.config(text=str(speed))
                self.speed_value_label_tab2.config(text=str(speed))
                
                # Sincronizar sliders de velocidad entre pestañas
                self.speed_slider_tab1.set(speed)
                self.speed_slider_tab2.set(speed)
                
                self.last_speed_update = current_time
                
                # Actualizar estado brevemente
                if speed == 0:
                    self.status_label.config(text=f"Velocidad 0: Los motores no se moverán", fg="orange")
                else:
                    self.status_label.config(text=f"Velocidad actualizada: {speed}", fg="green")
                
                self.window.after(2000, lambda: self.status_label.config(text="Sistema Listo", fg="green"))
            else:
                self.status_label.config(text="EMERGENCIA: No se puede cambiar velocidad", fg="red")

    def move_single_motor_from_entry(self, motor_id):
        """Mueve un motor individual basado en el valor del entry (Pestaña 2)"""
        try:
            value = self.entries[motor_id].get()
            angle_deg = float(value)

            if ANGLE_MIN <= angle_deg <= ANGLE_MAX:
                # Verificar velocidad y estado de emergencia
                speed = self.speed_slider_tab2.get()
                angle_rad = math.radians(angle_deg)
                ticks = self.controller.radians_to_dxl(angle_rad)

                if speed == 0:
                    self.entry_labels[motor_id].config(text="Velocidad 0", fg="orange")
                    self.status_label.config(text=f"Velocidad 0: Motor {motor_id} no se moverá", fg="orange")
                elif self.controller.emergency_stop_activated:
                    self.entry_labels[motor_id].config(text="EMERGENCIA", fg="red")
                    self.status_label.config(text="EMERGENCIA: No se puede mover motores", fg="red")
                else:
                    self.controller.move_motor(motor_id, ticks)
                    self.entry_labels[motor_id].config(text="Enviado", fg="blue")
                    self.status_label.config(text=f"Motor {motor_id} moviéndose a {angle_deg:.1f}°", fg="green")
                    
                    # Actualizar slider en la pestaña 1 si existe
                    if motor_id in self.sliders:
                        self.sliders[motor_id].set(angle_deg)
                    
                    # Resetear etiqueta después de 2 segundos
                    self.window.after(2000, lambda: self.entry_labels[motor_id].config(text="Listo", fg="green"))
                    self.window.after(2000, lambda: self.status_label.config(text="Sistema Listo", fg="green"))
            else:
                self.entry_labels[motor_id].config(
                    text=f"Error: {ANGLE_MIN:.0f}° a {ANGLE_MAX:.0f}°", fg="red"
                )
                
        except ValueError:
            self.entry_labels[motor_id].config(text="Error: Número", fg="red")

    # ========== SECUENCIA CON DELAY DESDE ENTRIES (TAB 2) ==========

    def move_all_motors_from_entries(self):
        """
        Inicia la secuencia para mover TODOS los motores usando los valores
        de los entries (en grados), con delay de 800 ms entre cada motor.
        """
        # Verificar condiciones
        speed = self.speed_slider_tab2.get()
        if speed == 0:
            self.status_label.config(text="Velocidad 0: Los motores no se moverán", fg="orange")
            return
            
        if self.controller.emergency_stop_activated:
            self.status_label.config(text="EMERGENCIA: No se puede mover motores", fg="red")
            return

        motor_ids_seq = []
        angles_deg_seq = []
        ticks_seq = []

        # Leer y validar todos los valores primero
        for motor_id in self.controller.dxl_ids:
            try:
                value = self.entries[motor_id].get()
                angle_deg = float(value)
            except ValueError:
                self.entry_labels[motor_id].config(text="Error: Número", fg="red")
                continue

            if not (ANGLE_MIN <= angle_deg <= ANGLE_MAX):
                self.entry_labels[motor_id].config(
                    text=f"Error: {ANGLE_MIN:.0f}° a {ANGLE_MAX:.0f}°", fg="red"
                )
                continue

            angle_rad = math.radians(angle_deg)
            ticks = self.controller.radians_to_dxl(angle_rad)

            motor_ids_seq.append(motor_id)
            angles_deg_seq.append(angle_deg)
            ticks_seq.append(ticks)

        if len(motor_ids_seq) == 0:
            self.status_label.config(text="No hay valores válidos para mover", fg="red")
            return

        self.status_label.config(
            text=f"Iniciando secuencia para {len(motor_ids_seq)}/{len(self.controller.dxl_ids)} motores...",
            fg="blue"
        )

        # Arrancar la secuencia recursiva con after
        self._move_all_sequence_step(motor_ids_seq, angles_deg_seq, ticks_seq, idx=0)

    # ========== SECUENCIA CON DELAY PARA RUTINAS (TAB 4) ==========

    def run_routine_with_delay(self, list_q_rad):
        """
        Ejecuta una rutina de q en radianes (list_q_rad) para q1..q4
        con delay de 800 ms entre motores, usando el mismo helper
        _move_all_sequence_step.
        """
        # Verificar parada de emergencia
        if self.controller.emergency_stop_activated:
            self.status_label.config(text="EMERGENCIA: No se puede mover motores", fg="red")
            return

        # Puedes usar la misma velocidad de la pestaña 2 o definir otra
        speed = self.speed_slider_tab2.get()
        if speed == 0:
            self.status_label.config(text="Velocidad 0: No se ejecutará la rutina", fg="orange")
            return

        motor_ids_seq = []
        angles_deg_seq = []
        ticks_seq = []

        # Asociar q1..q4 a los motores 1..4 (ignoramos 5)
        for idx, motor_id in enumerate(self.controller.dxl_ids):
            if motor_id == 5:
                break
            if idx >= len(list_q_rad):
                break

            q_rad = list_q_rad[idx]
            angle_deg = math.degrees(q_rad)
            ticks = self.controller.radians_to_dxl(q_rad)

            motor_ids_seq.append(motor_id)
            angles_deg_seq.append(angle_deg)
            ticks_seq.append(ticks)

        if len(motor_ids_seq) == 0:
            self.status_label.config(text="Rutina vacía o inválida", fg="red")
            return

        self.status_label.config(
            text=f"Iniciando rutina ({len(motor_ids_seq)} motores) con delay...",
            fg="blue"
        )

        self._move_all_sequence_step(motor_ids_seq, angles_deg_seq, ticks_seq, idx=0)

    # ========== HELPER GENÉRICO DE SECUENCIA ==========

    def _move_all_sequence_step(self, motor_ids_seq, angles_deg_seq, ticks_seq, idx):
        """
        Paso recursivo: mueve un motor y programa el siguiente después de 800 ms.
        Esto evita bloquear la GUI.
        """
        if idx >= len(motor_ids_seq):
            # Secuencia terminada
            self.status_label.config(text="Secuencia completada", fg="green")
            # Reset de labels a "Listo" después de 3 s
            self.window.after(
                3000,
                lambda: [
                    label.config(text="Listo", fg="green")
                    for label in self.entry_labels.values()
                ]
            )
            self.window.after(3000, lambda: self.status_label.config(text="Sistema Listo", fg="green"))
            return

        motor_id = motor_ids_seq[idx]
        angle_deg = angles_deg_seq[idx]
        ticks = ticks_seq[idx]

        # Mover el motor actual
        self.controller.move_motor(motor_id, ticks)

        # Actualizar etiquetas y sliders
        if motor_id in self.entry_labels:
            self.entry_labels[motor_id].config(text="Enviado", fg="blue")
        if motor_id in self.sliders:
            self.sliders[motor_id].set(angle_deg)
        if motor_id in self.labels:
            self.labels[motor_id].config(text=f'Ángulo: {angle_deg:.1f}°')

        self.status_label.config(
            text=f"Motor {motor_id} moviéndose a {angle_deg:.1f}° (paso {idx+1}/{len(motor_ids_seq)})",
            fg="blue"
        )

        # Programar siguiente motor en 800 ms
        self.window.after(
            800,
            lambda: self._move_all_sequence_step(motor_ids_seq, angles_deg_seq, ticks_seq, idx + 1)
        )

    # ===================== HOME / RVIZ / EMERGENCIA / CIERRE =====================

    def home_all(self):
        """Mueve todos los motores a la posición HOME"""
        # Si hay parada de emergencia activada, mostrar confirmación para reactivar
        if self.controller.emergency_stop_activated:
            if messagebox.askokcancel("Reactivar Sistema", 
                                     "La parada de emergencia está activada.\n\n¿Desea reactivar el sistema y mover los motores a HOME?"):
                self.controller.home_all_motors()
                self.status_label.config(text="Sistema reactivado y motores movidos a HOME", fg="blue")
            else:
                return
        else:
            self.controller.home_all_motors()
            self.status_label.config(text="Todos los motores movidos a HOME", fg="blue")
        
        # Actualizar interfaz en grados
        home_deg = self.controller.dxl_to_degrees(DEFAULT_GOAL)

        for motor_id in self.controller.dxl_ids:
            if motor_id in self.sliders:
                self.sliders[motor_id].set(home_deg)
                self.labels[motor_id].config(text=f'Ángulo: {home_deg:.1f}°')

            if motor_id in self.entries:
                self.entries[motor_id].delete(0, tk.END)
                self.entries[motor_id].insert(0, f"{home_deg:.1f}")
                self.entry_labels[motor_id].config(text="HOME", fg="green")

        self.window.after(3000, lambda: self.status_label.config(text="Sistema Listo", fg="green"))

    def launch_rviz(self):
        """Lanza robot_state_publisher + RViz usando ros2 launch"""
        try:
            # Lanzar el launch file: pincher_description/display.launch.py
            cmd = ["ros2", "launch", "phantomx_pincher_description", "display.launch.py"]

            def run_rviz():
                # Este proceso levanta robot_state_publisher + rviz2
                self.rviz_process = subprocess.Popen(cmd)
                self.rviz_process.wait()
                # Cuando se cierra, actualizar la interfaz
                self.window.after(0, self.on_rviz_closed)

            thread = threading.Thread(target=run_rviz, daemon=True)
            thread.start()

            # Actualizar interfaz
            self.rviz_btn.config(state=tk.DISABLED, bg="#cccccc")
            self.stop_rviz_btn.config(state=tk.NORMAL, bg="#f44336")
            self.rviz_status_label.config(text="RViz + robot_state_publisher ejecutándose", fg="green")
            self.status_label.config(text="Lanzado display.launch.py (RViz + modelo)", fg="green")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo lanzar RViz: {str(e)}")
            self.rviz_status_label.config(text=f"Error: {str(e)}", fg="red")


    def stop_rviz(self):
        """Detiene el proceso de RViz"""
        if self.rviz_process:
            try:
                self.rviz_process.terminate()
                self.rviz_process = None
            except:
                pass
        
        self.on_rviz_closed()

    def on_rviz_closed(self):
        """Actualiza la interfaz cuando RViz se cierra"""
        self.rviz_btn.config(state=tk.NORMAL, bg="#2196F3")
        self.stop_rviz_btn.config(state=tk.DISABLED, bg="#cccccc")
        self.rviz_status_label.config(text="RViz no iniciado", fg="red")
        self.status_label.config(text="RViz cerrado", fg="green")

    def setup_common_buttons(self):
        """Configura los botones comunes en la parte inferior"""
        # Frame para botones comunes
        common_buttons_frame = tk.Frame(self.window)
        common_buttons_frame.pack(fill='x', padx=20, pady=10)
        
        # Botón HOME
        home_btn = tk.Button(common_buttons_frame, text="HOME", 
                            font=("Arial", 10, "bold"), bg="#2196F3", fg="white",
                            command=self.home_all)
        home_btn.pack(side='left', padx=10)

        
        # Botón PARADA DE EMERGENCIA
        emergency_btn = tk.Button(common_buttons_frame, text="PARADA DE EMERGENCIA", 
                                 font=("Arial", 10, "bold"), bg="#f44336", fg="white",
                                 command=self.emergency_stop)
        emergency_btn.pack(side='right', padx=10)
        
        # Etiqueta de estado global
        self.status_label = tk.Label(common_buttons_frame, text="Sistema Listo", 
                                    font=("Arial", 9), fg="green")
        self.status_label.pack(side='bottom', pady=5)

    def emergency_stop(self):
        """Ejecuta parada de emergencia SIN confirmación"""
        self.controller.emergency_stop()
        self.status_label.config(text="PARADA DE EMERGENCIA ACTIVADA", fg="red")
        
        # Actualizar interfaz
        for label in self.entry_labels.values():
            label.config(text="EMERGENCIA", fg="red")

    def on_close(self):
        """Maneja el cierre de la ventana"""
        # Detener RViz si está ejecutándose
        if self.rviz_process:
            self.stop_rviz()
            
        if messagebox.askokcancel("Salir", "¿Estás seguro de que quieres salir?\nSe desactivará el torque de los motores."):
            self.controller.close()
            self.window.destroy()
            rclpy.shutdown()

    def run(self):
        """Ejecuta la interfaz"""
        try:
            self.window.mainloop()
        except KeyboardInterrupt:
            self.on_close()

# ============================================================
#  MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)

    # Crear el nodo controlador
    controller = PincherController()

    # Lanzar el spin de ROS2 en un hilo en segundo plano
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(controller,),
        daemon=True
    )
    spin_thread.start()

    try:
        # Lanzar la GUI en el hilo principal
        gui = PincherGUI(controller)
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Cerrar todo ordenadamente
        controller.close()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

