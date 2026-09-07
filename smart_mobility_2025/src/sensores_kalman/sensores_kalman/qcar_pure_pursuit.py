#!/usr/bin/env python3
"""
=======================================================================
 Pure Pursuit Controller for Physical QCar using ROS2
 Author: Marmanja
 Co-author: Iván Valdez del Toro
-----------------------------------------------------------------------
 Implements Ackermann-based Pure Pursuit tracking for the physical QCar,
 integrating ROS2 communication, obstacle-stop logic, dynamic lookahead,
 full trajectory analysis, extended plotting, and CSV export.
=======================================================================
"""



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool

import math
import csv
from pathlib import Path
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt


class PPState:
    """Estado mínimo del vehículo para Pure Pursuit."""
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.xr = x
        self.yr = y
        self.theta = yaw
        self.v = v

    def calc_distance(self, px, py):
        dx = self.xr - px
        dy = self.yr - py
        return math.hypot(dx, dy)


class TargetCourse:
    """Trayectoria + búsqueda del índice objetivo para Pure Pursuit."""
    def __init__(self, path_points, k_gain, base_lookahead):
        self.cx = [p[0] for p in path_points]
        self.cy = [p[1] for p in path_points]
        self.k = k_gain
        self.Lfc = base_lookahead
        self.old_nearest_point_index = None

    def search_target_index(self, state: PPState):
        """
        1) Busca punto más cercano
        2) Desde ahí avanza mientras la distancia disminuya
        3) Calcula Lf dinámico = k * v + Lfc (realmente no se usa dinámico por falta de retroalimentación de velocidad real)
        4) Busca el punto al menos a Lf de distancia
        """
        if self.old_nearest_point_index is None:
            dists = [
                math.hypot(state.xr - cx_i, state.yr - cy_i)
                for cx_i, cy_i in zip(self.cx, self.cy)
            ]
            ind = int(np.argmin(dists))
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this = state.calc_distance(self.cx[ind], self.cy[ind])

            while ind + 1 < len(self.cx):
                distance_next = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                if distance_this < distance_next:
                    break
                ind += 1
                distance_this = distance_next

            self.old_nearest_point_index = ind

        v = max(state.v, 0.0)
        Lf = self.k * v + self.Lfc

        target_ind = ind
        while target_ind < len(self.cx) - 1:
            dist = state.calc_distance(self.cx[target_ind], self.cy[target_ind])
            if dist >= Lf:
                break
            target_ind += 1

        return target_ind, Lf


# ==========================
# NODO PURE PURSUIT ROS2
# ==========================

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # ----------------- PARÁMETROS -----------------
        self.declare_parameter('path_csv', '')
        self.declare_parameter('circle_radius', 0.8)
        self.declare_parameter('circle_points', 300)
        self.declare_parameter('lookahead', 0.1)
        self.declare_parameter('k_gain', 0.8)
        self.declare_parameter('v_ref', 0.065)
        self.declare_parameter('wheelbase', 0.256)

        path_csv = self.get_parameter('path_csv').get_parameter_value().string_value
        self.lookahead = self.get_parameter('lookahead').get_parameter_value().double_value
        self.v_ref = self.get_parameter('v_ref').get_parameter_value().double_value
        self.k_gain = self.get_parameter('k_gain').get_parameter_value().double_value
        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value

        self.max_steer_cmd = 0.5
        self.paro = False

        if path_csv.strip():
            self.path = self.load_path(path_csv)
        else:
            self.path = self.generate_circle_path()

        if not self.path:
            self.get_logger().error('Path vacío, revisa el CSV o los parámetros del círculo')
        else:
            self.get_logger().info(f'Path cargado/generado con {len(self.path)} puntos')
            self.get_logger().info(f"Primeros 5 puntos path: {self.path[:5]}")

        if self.path:
            self.target_course = TargetCourse(self.path, self.k_gain, self.lookahead)
        else:
            self.target_course = None

        self.current_pose = None
        self.state = PPState()
        self.target_ind = 0
        self.generated_trajectory = []
        self.last_objective = None

        self.log_t = []
        self.log_x = []
        self.log_y = []
        self.log_yaw = []
        self.log_v = []

        self.start_time = self.get_clock().now()

        self.last_pose_time = self.get_clock().now()

        self.pose_sub = self.create_subscription(
            Vector3Stamped,
            '/qcar/pose',
            self.pose_callback,
            10
        )

        self.pub = self.create_publisher(
            Vector3Stamped,
            '/qcar/user_command',
            10
        )

        self.paro_sub = self.create_subscription(
            Bool,
            '/qcar/obstacle_alert',
            self.paro_callback,
            10
        )

        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("PurePursuitNode físico iniciado.")

    def load_path(self, csv_file):
        path_points = []
        if not csv_file:
            self.get_logger().warn('No se especificó path_csv')
            return path_points

        p = Path(csv_file)
        if not p.exists():
            self.get_logger().error(f'CSV no encontrado: {csv_file}')
            return path_points

        with p.open('r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    x = float(row['x'])
                    y = float(row['y'])
                    path_points.append((x, y))
                except (KeyError, ValueError):
                    continue

        self.get_logger().info(f'Path simple cargado con {len(path_points)} puntos')
        return path_points

    def generate_circle_path(self):
        R = self.get_parameter('circle_radius').get_parameter_value().double_value
        N = int(self.get_parameter('circle_points').get_parameter_value().integer_value)
        cx = 0.0
        cy = R

        if N < 3:
            self.get_logger().warn("circle_points < 3, usando N=3 por seguridad.")
            N = 3

        path_points = []
        for i in range(N):
            theta = 2.0 * math.pi * i / N
            x = cx + R * math.cos(theta)
            y = cy + R * math.sin(theta)
            path_points.append((x, y))

        self.get_logger().info(
            f"Trayectoria circular generada: R={R:.3f} m, centro=({cx:.3f}, {cy:.3f}), N={N}"
        )
        return path_points

    def pose_callback(self, msg: Vector3Stamped):
        self.current_pose = msg
        self.last_pose_time = self.get_clock().now()

    def paro_callback(self, msg: Bool):
        old = self.paro
        self.paro = bool(msg.data)

        if self.paro != old:
            if self.paro:
                self.get_logger().warn("Obstáculo detectado por LiDAR: activando PARO.")
            else:
                self.get_logger().info("Obstáculo despejado: reanudando movimiento.")

    def control_loop(self):

        now = self.get_clock().now()

        if self.current_pose is None or not self.path or self.target_course is None:
            return

        if self.paro:
            self.stop_qcar()
            return

        x_r = self.current_pose.vector.x
        y_r = self.current_pose.vector.y
        yaw = self.current_pose.vector.z

        self.state.xr = x_r
        self.state.yr = y_r
        self.state.theta = yaw

        self.state.v = abs(self.v_ref)
        delta = self.compute_pure_pursuit_delta()

        steer_cmd = delta
        if steer_cmd > self.max_steer_cmd:
            steer_cmd = self.max_steer_cmd
        elif steer_cmd < -self.max_steer_cmd:
            steer_cmd = -self.max_steer_cmd

        cmd = Vector3Stamped()
        cmd.header.stamp = now.to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.vector.x = float(self.v_ref)
        cmd.vector.y = float(-steer_cmd)
        cmd.vector.z = 0.0
        self.pub.publish(cmd)

        self.generated_trajectory.append((x_r, y_r))

        t = (now.nanoseconds - self.start_time.nanoseconds) * 1e-9
        self.log_t.append(t)
        self.log_x.append(x_r)
        self.log_y.append(y_r)
        self.log_yaw.append(yaw)
        self.log_v.append(self.v_ref)

    def compute_pure_pursuit_delta(self):

        ind, Lf = self.target_course.search_target_index(self.state)

        if self.target_ind >= ind:
            ind = self.target_ind

        self.target_ind = ind

        tx = self.target_course.cx[ind]
        ty = self.target_course.cy[ind]

        dist_to_target = self.state.calc_distance(tx, ty)
        if self.last_objective is None or ind != self.last_objective:
            self.last_objective = ind
            self.get_logger().info(
                f"Nuevo punto objetivo={ind} | ref=({tx:.3f}, {ty:.3f}) | "
                f"distancia={dist_to_target:.3f} m | Lf={Lf:.3f} m"
            )

        alpha = math.atan2(ty - self.state.yr, tx - self.state.xr) - self.state.theta

        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        Lf = max(Lf, 1e-3)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), Lf)
        return delta

    def stop_qcar(self):
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.vector.x = 0.0
        cmd.vector.y = 0.0
        cmd.vector.z = 0.0
        self.pub.publish(cmd)

    def find_closest_point(self, point, traj):
        min_dist = float('inf')
        closest = None
        for ref in traj:
            d = math.hypot(point[0] - ref[0], point[1] - ref[1])
            if d < min_dist:
                min_dist = d
                closest = ref
        return min_dist, closest

    def calculate_tracking_error(self):
        if not self.generated_trajectory or not self.path:
            return 0.0, 0.0, []

        errors = []
        for gen_point in self.generated_trajectory:
            min_dist, _ = self.find_closest_point(gen_point, self.path)
            errors.append(min_dist)

        return float(np.mean(errors)), float(np.max(errors)), errors

    def calculate_similarity_percentage(self, errors, tolerance=0.05):
        if not errors:
            return 0.0
        return sum(1 for e in errors if e <= tolerance) / len(errors) * 100.0

    def report_performance(self, avg_error, max_error, errors):

        tol1 = 0.05
        tol2 = 0.10

        sim05 = self.calculate_similarity_percentage(errors, tol1)
        sim10 = self.calculate_similarity_percentage(errors, tol2)
        std = float(np.std(errors))

        if avg_error < 0.05:
            assessment = "EXCELENTE"
            color = "\033[92m"
        elif avg_error < 0.10:
            assessment = "BUENO"
            color = "\033[96m"
        elif avg_error < 0.20:
            assessment = "ACEPTABLE"
            color = "\033[93m"
        else:
            assessment = "DEFICIENTE"
            color = "\033[91m"

        reset = "\033[0m"
        bold = "\033[1m"

        print("\n========== EVALUACIÓN DE TRAJECTORIA ==========")
        print(f"Error promedio:            {avg_error:.3f} m")
        print(f"Error máximo:              {max_error:.3f} m")
        print(f"Desviación estándar:       {std:.3f} m")
        print(f"% dentro de 5cm:           {sim05:.1f}%")
        print(f"% dentro de 10cm:          {sim10:.1f}%")
        print(f"Trayectoria referencia:    {len(self.path)} puntos")
        print(f"Trayectoria real:          {len(self.generated_trajectory)} puntos")
        print(f"\n{bold}Evalución general: {reset} {color}{assessment}{reset}")
        print("================================================\n")

    def save_trajectory_csv(self, errors, out_dir, timestamp, tag):

        csv_path = out_dir / f"trayectoria_{tag}_{timestamp}.csv"

        with csv_path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index', 't', 'x_real', 'y_real', 'yaw', 'v_cmd',
                             'x_ref', 'y_ref', 'error'])

            for i, (xr, yr) in enumerate(self.generated_trajectory):
                t = self.log_t[i] if i < len(self.log_t) else ''
                yaw = self.log_yaw[i] if i < len(self.log_yaw) else ''
                v_cmd = self.log_v[i] if i < len(self.log_v) else ''
                err = errors[i] if i < len(errors) else ''

                _, closest = self.find_closest_point((xr, yr), self.path)
                if closest:
                    xc, yc = closest
                else:
                    xc, yc = '', ''
                writer.writerow([i, t, xr, yr, yaw, v_cmd, xc, yc, err])

        print(f"CSV guardado en: {csv_path}")

    def plot_results(self):

        if not self.path or not self.generated_trajectory:
            print("No hay datos para graficar.")
            return

        exp_x = [p[0] for p in self.path]
        exp_y = [p[1] for p in self.path]
        gen_x = [p[0] for p in self.generated_trajectory]
        gen_y = [p[1] for p in self.generated_trajectory]

        avg_error, max_error, errors = self.calculate_tracking_error()
        if errors:
            self.report_performance(avg_error, max_error, errors)

        out_dir = Path.home() / 'Workspaces' / 'smart_mobility_2025' / 'resultados' / 'pure_pursuit_qcar'
        out_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = f"L{self.lookahead}_V{self.v_ref}_k{self.k_gain}"

        if self.log_t:
            fig2, axes = plt.subplots(2, 2, figsize=(12, 10))

            axes[0, 0].plot(exp_x, exp_y, '-r', label='Referencia', linewidth=2)
            axes[0, 0].plot(gen_x, gen_y, '-b', label='Real', linewidth=1.5)
            if len(gen_x) > 1:
                axes[0, 0].scatter(gen_x[0], gen_y[0], s=120, c='green', marker='o', label='Inicio real')
                axes[0, 0].scatter(gen_x[-1], gen_y[-1], s=120, c='red', marker='x', label='Final real')
            axes[0, 0].set_xlabel('X [m]')
            axes[0, 0].set_ylabel('Y [m]')
            axes[0, 0].set_title('Comparación de trayectorias')
            axes[0, 0].legend()
            axes[0, 0].grid(True)
            axes[0, 0].axis('equal')

            if errors:
                axes[0, 1].plot(self.log_t[:len(errors)], errors, '-g', linewidth=1.5)
                axes[0, 1].axhline(y=avg_error, linestyle='--',
                                   label=f'Promedio: {avg_error:.3f} m')
                axes[0, 1].set_xlabel('Tiempo [s]')
                axes[0, 1].set_ylabel('Error [m]')
                axes[0, 1].set_title('Error de seguimiento vs tiempo')
                axes[0, 1].legend()
                axes[0, 1].grid(True)

            axes[1, 0].plot(self.log_t, [v * 3.6 for v in self.log_v], '-m', linewidth=1.5)
            axes[1, 0].set_xlabel('Tiempo [s]')
            axes[1, 0].set_ylabel('Velocidad [km/h]')
            axes[1, 0].set_title('Perfil de velocidad (comando)')
            axes[1, 0].grid(True)

            axes[1, 1].plot(self.log_t, [math.degrees(th) for th in self.log_yaw],
                            '-c', linewidth=1.5)
            axes[1, 1].set_xlabel('Tiempo [s]')
            axes[1, 1].set_ylabel('Orientación [°]')
            axes[1, 1].set_title('Orientación del vehículo')
            axes[1, 1].grid(True)

            plt.tight_layout()
            fig_path2 = out_dir / f"analisis_trayectoria_{timestamp}_{tag}.png"
            fig2.savefig(fig_path2, dpi=300, bbox_inches='tight')
            plt.close(fig2)
            print(f"Gráficas de análisis guardadas en: {fig_path2}")

        self.save_trajectory_csv(errors, out_dir, timestamp, tag)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
