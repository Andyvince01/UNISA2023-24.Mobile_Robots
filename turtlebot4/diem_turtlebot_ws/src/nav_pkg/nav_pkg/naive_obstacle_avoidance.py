# Importazione delle librerie necessarie
import rclpy
# Importazione delle classi necessarie per la gestione dei nodi e dei messaggi in ROS2
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
# Importazione di metodi generici
from random import randint

class Handler(Node):
    """
    Classe per gestire il movimento di un robot attraverso comandi Twist.

    Questa classe crea un nodo ROS2 per controllare il movimento del robot
    pubblicando messaggi di tipo Twist sul topic "/cmd_vel".

    Attributes:
        - cmd_pub (Publisher): Pubblicatore per i messaggi di movimento.
        - ok (bool): Flag per indicare se è consentito pubblicare comandi di movimento.
        - msg (Twist): Messaggio di movimento corrente da pubblicare.
    """
    __slots__ = ['cmd_pub', 'scan_sub', 'ok', 'msg']

    LINEAR_SPEED = 0.31         # Velocità lineare predefinita in m/s (massima)
    ANGULAR_SPEED = 1.0         # Velocità angolare predefinita in rad/s
    TH_FRONT = TH_SIDE = 1.5    # Soglia di distanza minima del robot in metri da un ostacolo

    def __init__(self):
        """
        Inizializza il nodo e i suoi attributi.
        """
        try:
            # Inizializzazione del nodo ROS
            super().__init__("nav_step_node")
            self.get_logger().set_level(LoggingSeverity.DEBUG)
            # Creazione del publisher
            self.cmd_pub = self.create_publisher(msg_type=Twist, topic="/cmd_vel", qos_profile=10)             
            # Creazione del subscriber    
            self.scan_sub = self.create_subscription(msg_type=LaserScan, topic="/scan", callback=self.callback_scan, qos_profile=10)
        except Exception as e:
            self.get_logger().error(f"Errore durante l'inizializzazione del nodo ROS: {e}")

    def callback_scan(self, msg: LaserScan) -> None:
        front = msg.ranges[157]                    # Frontal lidar data
        right = msg.ranges[0]                      # Right lidar data
        left = msg.ranges[314]                     # Left lidar data
        self.get_logger().debug(f"front: {front}\tright: {right}\tleft: {left}")
        # If the robot encounters an obstacle, then stops and rotates until it has no more obstacle on its movement.
        if front > self.TH_FRONT and right > self.TH_SIDE and left > self.TH_SIDE:
            self.get_logger().debug(f"Moving...\n")
            self.go()
        else:
            self.get_logger().debug(f"Rotating...\n")
            self.rotate()

    def rotate(self, clockwise: bool = True) -> None:
        """
        Ruota il robot in senso orario o antiorario.

        Args:
            - clockwise (bool, optional): True per ruotare in senso orario, False per antiorario.
        """
        try:
            factor = -1 if clockwise else 1
            msg = Twist()
            msg.angular.z = self.ANGULAR_SPEED * factor     # Velocità angolare in rad/s
            self.cmd_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Errore durante la rotazione del robot: {e}")

    def go(self) -> None:
        """
        Fa avanzare il robot in avanti per un certo periodo di tempo.
        """
        try:
            msg = Twist()
            msg.linear.x = self.LINEAR_SPEED # Velocità lineare in m/s
            self.cmd_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Errore durante l'avanzamento del robot {e}")

    def stop(self) -> None:
        """
        Ferma il movimento del robot.
        """
        try:
            msg = Twist()
            msg.linear.x = 0.0                  # Arresto del movimento lineare
            msg.angular.z = 0.0                 # Arresto del movimento angolare
            self.cmd_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Errore durante lo stop del robot {e}")

def main():
    """
    Funzione principale per avviare il nodo e gestire l'esecuzione.
    """
    rclpy.init()
    executor = SingleThreadedExecutor()
    handller = Handler()
    # Impostazione del parametro del nodo per la simulazione. 
    # Questo forza il nodo a utilizzare l'orologio ROS anziché l'orologio di sistema.
    # Necessario per sincronizzare con l'orologio di Gazebo.
    param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)    # Creazione del parametro del nodo 'use_sim_time'
    handller.set_parameters([param])                                            # Impostazione del parametro del nodo 'use_sim_time'
    executor.add_node(handller)                                                 # Aggiunta del nodo all'esecutore
    try:
        executor.spin()                                                         # Running loop - bocking call
    except KeyboardInterrupt:
        pass

    handller.stop()
    handller.destroy_node()                                                     
    rclpy.shutdown()

if __name__ == "__main__":
    main()
