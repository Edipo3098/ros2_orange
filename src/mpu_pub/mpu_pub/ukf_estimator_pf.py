import math
import numpy as np
import rclpy
from time import perf_counter
from rclpy.node import Node
from robot_interfaces.msg import Mpu, COGframe
from message_filters import Subscriber, ApproximateTimeSynchronizer

DEG2RAD = math.pi/180.0
G2MS2   = 9.80665          # 1 g → 9.81 m/s²
CLAMP_DT = (1e-4, 0.05)    # 0.1 ms – 50 ms  (20–10000 Hz)
class ImuFusionNode(Node):
    def __init__(self):
        super().__init__('imu_fusion_node')
        # Suscripción a dos IMUs con sincronización aproximada por timestamp
        self.sub_imu1 = Subscriber(self, Mpu, 'mpu_data_2')
        #self.sub_imu2 = Subscriber(self, Mpu, 'mpu_data_2')
        self.sync = ApproximateTimeSynchronizer([self.sub_imu1], queue_size=10, slop=0.02,allow_headerless=True)
        self.sync.registerCallback(self.cb)
        # Publicador del marco COG fusionado
        self.pub_cog = self.create_publisher(COGframe, 'kalman_cog_frame_3', 10)

        # Inicialización del estado del filtro Kalman
        # Estado orientación como cuaternión (w,x,y,z) y bias giroscópico (bx,by,bz)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])   # cuaternión unitario inicial
        self.b_gyro = np.array([0.0, 0.0, 0.0])   # bias giroscopio inicial (rad/s)
        # Covarianza de estado inicial
        self.P = np.eye(7) * 0.0
        self.P[0:4, 0:4] *= 0.01    # incertidumbre inicial pequeña en orientación
        self.P[4:7, 4:7] *= 0.01    # incertidumbre inicial en bias giroscopio

        # Matrices de ruido (asumidos diagonales para simplicidad)
        self.g = 9.80665  # gravedad (m/s^2)
        self.R_meas = np.eye(3) * (0.5 ** 2)  # varianza del acelerómetro (~0.5 m/s^2 desviación estándar)
        self.Q_gyro = np.eye(3) * (0.02 ** 2)  # varianza proceso para ruido giros (rad/s)
        self.Q_bias = np.eye(3) * (0.001 ** 2) # varianza proceso para deriva lenta de bias

        # Vectores de posición relativa de cada IMU respecto al COG (en marco del cuerpo)
        # Estos deben ajustarse según la ubicación física de las IMUs en el robot.
        self.r1 = np.array([0.0, 0.0, 0.0])  # Offset IMU1 desde COG (ejemplo: en metros)
        self.r2 = np.array([0.0, 0.0, 0.0])  # Offset IMU2 desde COG (ejemplo: colocar valores reales)

        # Variables para integración de posición
        self.velocity = np.zeros(3)  # [vx, vy, vz] en marco mundial (m/s)
        self.position = np.zeros(3)  # [x, y, z] en marco mundial (m)
        self.last_time = None

        # Fase de calibración inicial
        self.calibrating = True
        self.calib_samples = []
        self.prev_time = perf_counter()
        self.last_t = None
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.q   = np.array([1,0,0,0], float)   # cuaternión
        self.bias = np.zeros(3)                 # bias gyro   (rad/s)

        #  --- ruidos ---
        self.Qw = (0.02*DEG2RAD)**2 * np.eye(3)  # var gyro
        self.Qb = (0.001*DEG2RAD)**2 * np.eye(3) # var bias
        self.Ra = (0.5*G2MS2)**2   * np.eye(3)   # var accel

        self.get_logger().info("Calibrando biases de las IMUs... espere unos segundos sin mover el dispositivo.")
    # ---------- callback sincronizado ----------
    def cb(self, m1: Mpu):
        now = self.get_clock().now()
        t   = now.nanoseconds*1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = max(CLAMP_DT[0], min(CLAMP_DT[1], t - self.last_t))
        self.last_t = t

        # 1. UNIDADES CORRECTAS ------------------------------------------------
        a1 = np.array([m1.acx, m1.acy, m1.acz]) * G2MS2
        w1 = np.array([m1.gx,  m1.gy,  m1.gz ]) * DEG2RAD
        a2 = np.array([m1.acx2, m1.acy2, m1.acz2]) * G2MS2
        w2 = np.array([m1.gx2,  m1.gy2,  m1.gz2 ]) * DEG2RAD

        a  = 0.5*(a1 + a2)
        w  = 0.5*(w1 + w2) - self.bias       # gyro sin bias

        # 2. PREDICCIÓN ORIENTACIÓN (integrar giro) ----------------------------
        dq = self._rv2quat(w*dt)             # rot vec → quat
        self.q = self._qmult(self.q, dq)
        self.q = self.q/np.linalg.norm(self.q)

        # 3. CORRECCIÓN CON GRAVEDAD (filtro complementario sencillo) ----------
        g_body = self._q_conj_rotate(self.q, np.array([0,0,-G2MS2]))
        err = np.cross(a, g_body)            # error ángulo pequeño
        self.bias += 0.0001*err              # adapta bias   (rad/s)
        self.q = self._qmult(self.q, self._rv2quat(0.01*err))

        # 4. ELIMINAR GRAVEDAD, INTEGRAR V y P ---------------------------------
        a_lin = a - g_body                   # acel lineal (m/s²)
        self.vel += a_lin*dt
        self.pos += self.vel*dt

        # 5. PUBLICAR -----------------------------------------------------------
        roll, pitch, yaw = self._quat2euler(self.q)
        out = COGframe()
        out.pos_x, out.pos_z, out.pos_z = self.pos
        out.roll  = math.degrees(roll)
        out.pitch = math.degrees(pitch)
        out.yaw   = math.degrees(yaw)
        self.pub_cog.publish(out)
        
    # ---------- utilidades ----------
    def _rv2quat(self, rv):
        th = np.linalg.norm(rv)
        if th < 1e-6:
            return np.array([1, *(0.5*rv)])
        ax = rv/th
        s  = math.sin(th/2)
        return np.array([math.cos(th/2), *(ax*s)])

    def _qmult(self, q1, q2):
        w0,x0,y0,z0 = q1; w1,x1,y1,z1 = q2
        return np.array([
            w0*w1 - x0*x1 - y0*y1 - z0*z1,
            w0*x1 + x0*w1 + y0*z1 - z0*y1,
            w0*y1 - x0*z1 + y0*w1 + z0*x1,
            w0*z1 + x0*y1 - y0*x1 + z0*w1])

    def _q_conj_rotate(self, q, v):
        # rota v (3d) con cuaternión q
        qv = np.array([0, *v])
        return self._qmult(self._qmult(q, qv), self._qconj(q))[1:]

    def _qconj(self, q): return np.array([q[0], -q[1], -q[2], -q[3]])

    def _quat2euler(self, q):
        w,x,y,z = q
        roll  = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(max(-1,min(1, 2*(w*y-z*x))))
        yaw   = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        return roll, pitch, yaw
    def imu_callback(self, imu1_msg: Mpu):
        # Sincronizado: ambos mensajes corresponden aproximadamente al mismo instante
        # Extraer lecturas de acelerómetro y giroscopio de cada Mpu (asumiendo campos ax, ay, az, gx, gy, gz)
        a1 = np.array([imu1_msg.acx, imu1_msg.acy, imu1_msg.acz], dtype=float)
        w1 = np.array([imu1_msg.gx, imu1_msg.gy, imu1_msg.gz], dtype=float)
        a2 = np.array([imu1_msg.acx2, imu1_msg.acy2, imu1_msg.acz2], dtype=float)
        w2 = np.array([imu1_msg.gx2, imu1_msg.gy2, imu1_msg.gz2], dtype=float)

        # Convertir unidades a SI (si no lo estuvieran ya). Suponemos que:
        # - aceleración viene en m/s^2
        # - velocidad angular viene en rad/s (de no ser así, convertir grados/s -> rad/s)
        # (Ajustar según la definición real de robot_interfaces/Mpu)

        # Acumular datos para calibración inicial de bias (asume dispositivo quieto al iniciar)
        if self.calibrating:
            self.calib_samples.append((w1, w2, a1, a2))
            # Calibrar por ~100 muestras (~1 seg a 100 Hz) antes de empezar filtro
            if len(self.calib_samples) < 100:
                return
            # Calculo de biases promedio tras recopilar suficientes muestras
            w1_avg = np.mean([s[0] for s in self.calib_samples], axis=0)
            w2_avg = np.mean([s[1] for s in self.calib_samples], axis=0)
            # Bias de giroscopio inicial para cada IMU (promedio estático)
            b1 = w1_avg
            b2 = w2_avg
            # Usamos el promedio de ambos biases como bias inicial global (asumiendo similares)
            self.b_gyro = (b1 + b2) / 2.0
            # Orientación inicial estimada a partir del acelerómetro promedio (asumiendo sin movimiento)
            a1_avg = np.mean([s[2] for s in self.calib_samples], axis=0)
            a2_avg = np.mean([s[3] for s in self.calib_samples], axis=0)
            a_avg = (a1_avg + a2_avg) / 2.0
            # Determinar roll y pitch inicial a partir de la dirección de la gravedad medida
            # Suponemos ejes del cuerpo: X adelante, Y izquierda, Z arriba (ENU típico)
            ax, ay, az = a_avg
            # Remover componente de gravedad en magnitud para calcular ángulos
            # (Normalizamos vector gravedad medido)
            norm_a = math.sqrt(ax*ax + ay*ay + az*az)
            if norm_a > 1e-6:
                ax_n, ay_n, az_n = ax/norm_a, ay/norm_a, az/norm_a
            else:
                ax_n, ay_n, az_n = ax, ay, az
            # Calcular ángulos (Tait-Bryan Z-Y-X intrínsecos: yaw, pitch, roll)
            pitch0 = math.asin(-ax_n)  # inclinación adelante/atrás
            roll0  = math.atan2(ay_n, az_n)  # inclinación lateral
            yaw0   = 0.0  # desconocido sin magnetómetro, se inicializa a 0
            # Convertir ángulos iniciales a cuaternión
            cy = math.cos(yaw0 * 0.5);    sy = math.sin(yaw0 * 0.5)
            cp = math.cos(pitch0 * 0.5); sp = math.sin(pitch0 * 0.5)
            cr = math.cos(roll0 * 0.5);  sr = math.sin(roll0 * 0.5)
            qw = cr*cp*cy + sr*sp*sy
            qx = sr*cp*cy - cr*sp*sy
            qy = cr*sp*cy + sr*cp*sy
            qz = cr*cp*sy - sr*sp*cy
            self.q = np.array([qw, qx, qy, qz], dtype=float)
            self.q = self._normalize_quaternion(self.q)
            self.calibrating = False
            self.get_logger().info("Calibración completada. Bias giroscopio estimado: {}".format(self.b_gyro))
            return

        # Si ya calibrado, continuamos con el filtro
        # Obtener timestamp actual y calcular dt
        current_time = perf_counter()
        dt = current_time - self.prev_time
        if dt < 1e-6:
            dt = 1e-6  # Set a minimum time step threshold
         # Apply low-pass filter to the raw accelerometer data

        # Paso de **predicción** del EKF (propagación de orientación y bias)
        # Combinar lecturas de giroscopio: asumimos el cuerpo es rígido, por lo que la velocidad angular es común.
        # Tomamos el promedio de ambas IMUs para reducir ruido.
        w_mean = 0.5 * (w1 + w2)
        # Restar bias de giroscopio actual del estado
        w_body = w_mean - self.b_gyro
        # Integrar pequeña rotación en dt para actualizar el cuaternión de orientación
        dq = self._rotation_vector_to_quaternion(w_body * dt)
        # Actualizar cuaternión: q_new = q_old * dq  (composición de rotación)
        self.q = self._quat_multiply(self.q, dq)
        self.q = self._normalize_quaternion(self.q)
        # Bias giroscopio se modela constante (sin cambio en predicción)
        # Actualizar covarianza de estado P = F * P * F^T + Q
        # Para simplificar, aproximamos F linealmente y usamos adición de ruido proceso:
        # Actualizar submatrices covarianza:
        # - Orientación: incremento de incertidumbre por ruido de gyro
        G = self._gyro_noise_jacobian(self.q, dt)   # matriz Jacobiana de entrada gyro
        # Ensamblar matriz de covarianza proceso 7x7
        Q_process = np.zeros((7,7))
        # Ruido del giro (convertido a varianza de cuaternión)
        Q_theta = self.Q_gyro * dt**2  # varianza de integración de giro en este paso
        # Proyectar Q_theta en espacio cuaternión via jacobiano G
        Q_quat = G.dot(Q_theta).dot(G.T)
        Q_process[0:4, 0:4] = Q_quat
        # Ruido de bias (random walk)
        Q_process[4:7, 4:7] = self.Q_bias * dt
        # Jacobiano de transición F (aproximado):
        F = np.eye(7)
        # (Orientación-bias están débilmente acoplados; ignoramos términos pequeños off-diagonal para F)
        # Actualizar P
        self.P = F.dot(self.P).dot(F.T) + Q_process

        # Paso de **corrección** del EKF usando acelerómetros de ambas IMUs
        # Convertir cuaternión a matriz de rotación del cuerpo al mundo
        R_body_to_world = self._quat_to_rot_matrix(self.q)
        # Primera medición (IMU1):
        self._update_orientation(a1, R_body_to_world)
        # Segunda medición (IMU2):
        self._update_orientation(a2, R_body_to_world)
        # (Después de estas dos actualizaciones secuenciales, la orientación self.q y bias self.b_gyro quedan corregidos)

        # **Cálculo de aceleración lineal del COG** (compensando rotación):
        # Expresar ω (velocidad ang) y α (aceleración ang) en marco del cuerpo
        omega = w_body  # velocidad angular cuerpo (rad/s) tras bias
        # Estimar aceleración angular α = Δω/dt (diferencia de vel ang entre iteraciones)
        # Guardar ω para la próxima iteración:
        if not hasattr(self, 'omega_prev'):
            self.omega_prev = omega
        alpha = (omega - self.omega_prev) / dt
        self.omega_prev = omega
        # Remover componente de gravedad de cada acelerómetro (pasar a aceleración propia en marco cuerpo)
        # La gravedad en marco del cuerpo es la inversa de rotación: g_body = R_world_to_body * [0,0,-g]
        R_world_to_body = R_body_to_world.T
        gravity_body = R_world_to_body.dot(np.array([0, 0, -self.g]))
        a1_body = a1 - gravity_body  # aceleración lineal medida por IMU1 en coords cuerpo
        a2_body = a2 - gravity_body  # aceleración lineal medida por IMU2 en coords cuerpo
        # Calcular aceleración del COG estimada por cada sensor: a_cog = a_sensor - (α × r + ω × (ω × r))
        a_cog1_body = a1_body - (np.cross(alpha, self.r1) + np.cross(omega, np.cross(omega, self.r1)))
        a_cog2_body = a2_body - (np.cross(alpha, self.r2) + np.cross(omega, np.cross(omega, self.r2)))
        # Promediar ambas estimaciones para reducir ruido
        a_cog_body = 0.5 * (a_cog1_body + a_cog2_body)
        # Transformar aceleración del COG al marco mundial
        a_cog_world = R_body_to_world.dot(a_cog_body)

        # **Integración de velocidad y posición**
        # Si se detecta reposo (aceleración ~ gravedad, sin rotación), aplicar Zero-Velocity Update
        if np.linalg.norm(a_cog_body) < 0.1 and np.linalg.norm(omega) < math.radians(5):
            # Consideramos que está prácticamente quieto
            self.velocity[:] = 0.0
        else:
            # Integrar velocidad
            self.velocity += a_cog_world * dt
        # Integrar posición
        self.position += self.velocity * dt

        # Publicar el mensaje de salida COGframe
        cog_msg = COGframe()
        # Rellenar campos de orientación en grados y posición en metros
        roll, pitch, yaw = self._quat_to_euler(self.q)  # en radianes
        cog_msg.roll = math.degrees(roll)
        cog_msg.pitch = math.degrees(pitch)
        cog_msg.yaw = math.degrees(yaw)
        cog_msg.pos_x = float(self.position[0])
        cog_msg.pos_y = float(self.position[1])
        cog_msg.pos_z = float(self.position[2])
        self.pub_cog.publish(cog_msg)

    # Funciones auxiliares de cálculo:

    def _normalize_quaternion(self, q):
        # Normaliza un cuaternión numpy array de tamaño 4
        norm = math.sqrt(np.dot(q, q))
        if norm == 0:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm

    def _quat_multiply(self, q1, q2):
        # Multiplica cuaterniones q1*q2 (cada uno array [w,x,y,z])
        w0, x0, y0, z0 = q1
        w1, x1, y1, z1 = q2
        return np.array([
            w0*w1 - x0*x1 - y0*y1 - z0*z1,
            w0*x1 + x0*w1 + y0*z1 - z0*y1,
            w0*y1 - x0*z1 + y0*w1 + z0*x1,
            w0*z1 + x0*y1 - y0*x1 + z0*w1
        ], dtype=float)

    def _rotation_vector_to_quaternion(self, rv):
        # Convierte un vector de rotación (rad) en un cuaternión [w,x,y,z]
        # (rv = ω*dt con ω en rad/s, pequeño ángulo de rotación en este intervalo)
        theta = np.linalg.norm(rv)
        if theta < 1e-8:
            # Rotación muy pequeña: aproximar cuaternión (w≈1, vector ≈ mitad del ángulo)
            return np.array([1.0, 0.5*rv[0], 0.5*rv[1], 0.5*rv[2]], dtype=float)
        axis = rv / theta
        half = theta / 2.0
        w = math.cos(half)
        sin_half = math.sin(half)
        x, y, z = axis * sin_half
        return np.array([w, x, y, z], dtype=float)

    def _quat_to_rot_matrix(self, q):
        # Convierte cuaternión en matriz de rotación 3x3 (de marco cuerpo a marco mundo)
        qw, qx, qy, qz = q
        # Fórmulas de rotación (convención Hamilton: [w,x,y,z])
        # Rotación activa: v_world = R * v_body
        xx = qx*qx; yy = qy*qy; zz = qz*qz
        xy = qx*qy; xz = qx*qz; yz = qy*qz
        wx = qw*qx; wy = qw*qy; wz = qw*qz
        return np.array([
            [1 - 2*(yy+zz),  2*(xy - wz),    2*(xz + wy)],
            [2*(xy + wz),    1 - 2*(xx+zz),  2*(yz - wx)],
            [2*(xz - wy),    2*(yz + wx),    1 - 2*(xx+yy)]
        ], dtype=float)

    def _quat_to_euler(self, q):
        # Convierte cuaternión a ángulos Euler (roll, pitch, yaw)
        qw, qx, qy, qz = q
        # Usamos convención Z-Y-X intrínseca (yaw-pitch-roll)
        # roll (X)
        sinr_cosp = 2 * (qw*qx + qy*qz)
        cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # pitch (Y)
        sinp = 2 * (qw*qy - qz*qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi/2, sinp)  # uso de 90° si fuera fuera de rango
        else:
            pitch = math.asin(sinp)
        # yaw (Z)
        siny_cosp = 2 * (qw*qz + qx*qy)
        cosy_cosp = 1 - 2 * (qy*qy + qz*qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _gyro_noise_jacobian(self, q, dt):
        # Calcula jacobiano de entrada (W) para propagación de incertidumbre del giro sobre el cuaternión:contentReference[oaicite:9]{index=9}
        # (Aproximamos linealmente para pequeñas rotaciones)
        qw, qx, qy, qz = q
        return (dt/2.0) * np.array([
            [-qx, -qy, -qz],
            [ qw, -qz,  qy],
            [ qz,  qw, -qx],
            [-qy,  qx,  qw]
        ], dtype=float)

    def _update_orientation(self, acc_meas, R_body_to_world):
        # Actualización EKF de la orientación con una medición de acelerómetro (de una IMU)
        # Normalizar aceleración medida para obtener dirección de gravedad observada
        acc = acc_meas.copy()
        norm = np.linalg.norm(acc)
        if norm < 1e-6:
            return  # medición inválida o muy pequeña
        acc_norm = acc / norm
        # Vector de gravedad esperado en marco del cuerpo según el estado (orientación actual)
        # En el marco mundial, gravedad = [0, 0, -1] (dirección normalizada hacia abajo)
        g_world_unit = np.array([0.0, 0.0, -1.0])
        # Gravedad predicha en cuerpo = R^T * g_world (usando R del cuerpo a mundo)
        R_world_to_body = R_body_to_world.T
        pred_acc_body = R_world_to_body.dot(g_world_unit)
        # Residuo (innovación) entre medido (dirección) y predicho
        y = acc_norm - pred_acc_body
        # Jacobiano de observación H (derivada de pred_acc_body respecto a estado [quat,bias])
        # Como la observación solo depende de la orientación (no directamente del bias de gyro en este instante),
        # H = [∂(g_body)/∂(quat), 0_{3x3}] 
        # Usamos derivadas aproximadas:contentReference[oaicite:10]{index=10}:
        qw, qx, qy, qz = self.q
        H_quat = 2 * self.g * np.array([
            [ qy, -qz,  qw, -qx],
            [-qx, -qw, -qz, -qy],
            [-qw,  qx,  qy, -qz]
        ], dtype=float)
        # Adaptar H_quat para la dirección normalizada (g simplificada a 1g):
        H_quat = 2 * np.array([
            [ qy, -qz,  qw, -qx],
            [-qx, -qw, -qz, -qy],
            [-qw,  qx,  qy, -qz]
        ], dtype=float)
        H = np.zeros((3, 7))
        H[:, 0:4] = H_quat
        # Innovación covarianza S = H * P * H^T + R
        S = H.dot(self.P).dot(H.T) + self.R_meas
        # Ganancia de Kalman K = P * H^T * S^{-1}
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        # Actualizar estado: x = x + K*y
        delta_x = K.dot(y)
        # Aplicar actualización al cuaternión y bias:
        dq = np.array([1.0, 0.5*delta_x[0], 0.5*delta_x[1], 0.5*delta_x[2]])  # cuaternión pequeño a partir de delta rotación
        self.q = self._quat_multiply(self.q, dq)
        self.q = self._normalize_quaternion(self.q)
        # Actualizar bias giroscopio (los últimos 3 elementos del delta_x)
        self.b_gyro += delta_x[4:7]
        # Actualizar covarianza: P = (I - K*H) * P
        I = np.eye(7)
        self.P = (I - K.dot(H)).dot(self.P)

def main(args=None):
    rclpy.init(args=args)
    node = ImuFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

