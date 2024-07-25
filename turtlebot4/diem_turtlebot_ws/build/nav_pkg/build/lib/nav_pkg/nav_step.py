import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.clock import Duration
from geometry_msgs.msg import Twist

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

    LINEAR_SPEED = 0.31     # Velocità lineare predefinita in m/s
    ANGULAR_SPEED = 1.5     # Velocità angolare predefinita in rad/s
    MOVEMENT_DURATION = 4   # Durata predefinita del movimento in secondi
    ROTATION_DURATION = 1   # Durata predefinita della rotazione in secondi

    def __init__(self):
        """
        Inizializza il nodo e i suoi attributi.
        """
        try:
            super().__init__("nav_step_node")                                           # Inizializzazione del nodo ROS
            self.get_logger().set_level(LoggingSeverity.DEBUG)
            self.create_timer(0.1, self.move_callback)                                  # Creazione di un timer per il callback di movimento
            self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)                 # Creazione del publisher
            self.ok = False
        except Exception as e:
            self.get_logger().error(f"Errore durante l'inizializzazione del robot: {e}")


    def move_callback(self):
        """
        Callback chiamato dal timer per pubblicare il messaggio di movimento.
        """
        if self.ok:
            self.cmd_pub.publish(self.msg)

    def rotate(self, clockwise=True):
        """
        Ruota il robot in senso orario o antiorario.

        Args:
            - clockwise (bool, optional): True per ruotare in senso orario, False per antiorario.
        """
        try:
            factor = -1 if clockwise else 1
            msg = Twist()
            msg.angular.z = self.ANGULAR_SPEED * factor # Velocità angolare in rad/s
            self.msg = msg
            self.ok = True
            self.sleep(1)
            self.ok = False
        except Exception as e:
            self.get_logger().error(f"Errore durante la rotazione del robot: {e}")

    def go(self):
        """
        Fa avanzare il robot in avanti per un certo periodo di tempo.
        """
        try:
            msg = Twist()
            msg.linear.x = self.LINEAR_SPEED # Velocità lineare in m/s
            self.msg = msg
            self.ok = True
            self.sleep(self.MOVEMENT_DURATION)
            self.ok = False
        except Exception as e:
            self.get_logger().error(f"Errore durante l'avanzamento del robot {e}")

    def stop(self):
        """
        Ferma il movimento del robot.
        """
        try:
            msg = Twist()
            msg.linear.x = 0.0 # Arresto del movimento lineare
            self.msg = msg
            self.ok = True
            self.sleep(self.ROTATION_DURATION)
            self.ok = False
        except Exception as e:
            self.get_logger().error(f"Errore durante lo stop del robot {e}")

    def sleep(self, time_seconds):
        """
        Mette il nodo in pausa per un certo periodo di tempo

        Args:
            - time_seconds (float): Numero di secondi di pausa.
        """
        self.get_clock().sleep_for(Duration(seconds=time_seconds))                          # Pausa per <time_seconds> secondi
    
    def loop(self):
        """
        Ciclo principale del nodo per eseguire sequenzialmente le operazioni di movimento.
        """
        # Inizializzazione del nodo
        self.get_logger().info("Node starting...")
        self.sleep(1)
        # Avanzamento per 4 secondi
        self.get_logger().info("Moving Forward...")
        self.go()
        # Rotazione per 1 secondo
        self.get_logger().info("Rotate...")
        self.rotate()
        # Avanzamento per 4 secondi
        self.get_logger().info("Moving Forward...")
        self.go()
        # Stop del robot
        self.get_logger().info("Stop...")
        self.stop()
        self.get_logger().info("The End!")

def main():
    """
    Funzione principale per avviare il nodo e gestire l'esecuzione.
    """
    rclpy.init()
    # Executor utilizza più thread anziché un singolo thread
    executor = MultiThreadedExecutor()
    handller = Handler()
    # Impostazione del parametro del nodo per la simulazione. 
    # Questo forza il nodo a utilizzare l'orologio ROS anziché l'orologio di sistema.
    # Necessario per sincronizzare con l'orologio di Gazebo.
    param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)    # Creazione del parametro del nodo 'use_sim_time'
    handller.set_parameters([param])                                            # Impostazione del parametro del nodo 'use_sim_time'
    executor.add_node(handller)                                                 # Aggiunta del nodo all'esecutore
    executor.create_task(handller.loop)                                         # Creazione di un task con il callable fornito come input
    try:
        executor.spin()                                                         # Esecuzione del loop - chiamata bloccante
    except KeyboardInterrupt:
        pass

    handller.destroy_node()                                                     # Buona norma distruggere il nodo

if __name__ == "__main__":
    main()
