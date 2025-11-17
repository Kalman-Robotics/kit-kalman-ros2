#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
import threading
import signal
from kalman_interfaces.msg import ControlStatus

class PointSubscriber(Node):
    def __init__(self):
        super().__init__('point_subscriber')
        self.subscription = self.create_subscription(ControlStatus, 'control_status', self.listener_callback, 10)
        self.time = []
        self.motorL_data = {
            'velocidad': [],
            'control': [],
            'error': [],
            'setpoint': [],
            'velocidad_filtered': []
        }
        self.motorR_data = {
            'velocidad': [],
            'control': [],
            'error': [],
            'setpoint': [],
            'velocidad_filtered': []
        }
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def listener_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.time.append(current_time)

        # Motor Izquierdo (Left)
        self.motorL_data['velocidad'].append(msg.l_current_speed)
        self.motorL_data['control'].append(msg.l_current_control)
        self.motorL_data['error'].append(msg.l_current_error)
        self.motorL_data['setpoint'].append(msg.l_setpoint)
        self.motorL_data['velocidad_filtered'].append(msg.l_current_speed_filtered)

        # Motor Derecho (Right)
        self.motorR_data['velocidad'].append(msg.r_current_speed)
        self.motorR_data['control'].append(msg.r_current_control)
        self.motorR_data['error'].append(msg.r_current_error)
        self.motorR_data['setpoint'].append(msg.r_setpoint)
        self.motorR_data['velocidad_filtered'].append(msg.r_current_speed_filtered)

        self.get_logger().info(f'Motor Izq: SetPoint={msg.l_setpoint:.2f}, VelocidadFL={msg.l_current_speed:.2f}, ControlFL={msg.l_current_control:.2f}, ErrorFL={msg.l_current_error:.2f}')
        self.get_logger().info(f'Motor Der: SetPoint={msg.r_setpoint:.2f}, VelocidadFR={msg.r_current_speed:.2f}, ControlFR={msg.r_current_control:.2f}, ErrorFR={msg.r_current_error:.2f}')


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.motor_publisher = self.create_publisher(Bool, 'set_switch_control', 10)


    def publish_motor_status(self, status):
        msg = Bool()
        msg.data = status
        self.motor_publisher.publish(msg)
        self.get_logger().info(f'Motor status publicado: {status}')


def update_plot(node, ax, motor_data, motor_name, canvas):
    if not rclpy.ok() or not node.time:
        return

    ax.clear()

    current_time = node.time[-1]
    time_range = 5
    filtered_indices = [i for i, t in enumerate(node.time) if current_time - t <= time_range]

    filtered_time = [node.time[i] for i in filtered_indices]

    # Línea de referencia en 0 (horizontal)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.7, linewidth=0.8)

    # Graficar velocidad, setpoint y error
    for key, color, label, style in zip(['velocidad', 'setpoint', 'error'],
                                         ['blue', 'green', 'red'],
                                         ['Velocidad', 'Setpoint', 'Error'],
                                         ['-', '--', '-']):
        if motor_data[key]:
            ax.plot(filtered_time, [motor_data[key][i] for i in filtered_indices],
                   label=label, color=color, linestyle=style, linewidth=2.5, marker='', markersize=4)

    ax.set_title(f'Motor {motor_name}', fontsize=13, fontweight='bold', pad=15)
    ax.legend(loc='upper left', fontsize=11, framealpha=0.95, edgecolor='black')
    ax.set_xlabel('Tiempo (s)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Valor (rad/s)', fontsize=11, fontweight='bold')
    ax.set_ylim(-250, 250)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.tick_params(labelsize=10)

    # Configurar límites del eje X para que avance con el tiempo
    if filtered_time:
        ax.set_xlim(max(0, filtered_time[0]), filtered_time[-1] + 0.1)

    canvas.draw()
    canvas.get_tk_widget().after(200, update_plot, node, ax, motor_data, motor_name, canvas)

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = PointSubscriber()
    motor_node = MotorController()

    root = tk.Tk()
    root.title("Interfaz de Control y Gráficos")
    root.geometry("1600x700")

    fig, axes = plt.subplots(1, 2, figsize=(15, 6.5))
    fig.suptitle("Monitoreo de Control de Motores", fontsize=14, fontweight='bold')
    fig.subplots_adjust(left=0.08, right=0.95, top=0.92, bottom=0.12, wspace=0.25)

    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    def on_closing():
        root.quit()
        subscriber_node.destroy_node()
        motor_node.destroy_node()
        rclpy.shutdown()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    def handle_signal(sig, frame):
        on_closing()

    signal.signal(signal.SIGINT, handle_signal)

    ros_thread = threading.Thread(target=lambda: rclpy.spin(subscriber_node), daemon=True)
    ros_thread.start()

    # Gráficas de los motores
    canvas_widget.after(200, update_plot, subscriber_node, axes[0], subscriber_node.motorL_data, "Izquierdo (L)", canvas)
    canvas_widget.after(200, update_plot, subscriber_node, axes[1], subscriber_node.motorR_data, "Derecho (R)", canvas)

    root.mainloop()
    ros_thread.join()

if __name__ == '__main__':
    main()
