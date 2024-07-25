# Importazione delle librerie necessarie
import rclpy
# Importazione delle classi necessarie per la gestione dei nodi e dei messaggi in ROS2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

class Handler(Node):
    """
    Classe per gestire i messaggi relativi all'odometria e ai comandi di velocità.

    Estende la classe Node del pacchetto rclpy.

    Attributi:
        sub_odom: Sottoscrizione al topic "/odom" per ricevere i messaggi di odometria.
        sub_vel: Sottoscrizione al topic "/cmd_vel" per ricevere i messaggi di comando di velocità.

    Metodi:
        - __init__: Inizializza la classe Handler e le sottoscrizioni ai topic.
        - odom_callback: Callback per i messaggi di odometria.
        - vel_callback: Callback per i messaggi di comando di velocità.
        - _log_info: Metodo privato per registrare le informazioni nei log.
    """
    __slots__ = ['sub_odom', 'sub_vel']

    def __init__(self) -> None:
        """
        Inizializza la classe Handler.

        Crea le sottoscrizioni ai topic "/odom" e "/cmd_vel".
        """
        super().__init__("odom_node")  # Inizializza il nodo
        self.sub_odom = self.create_subscription(msg_type=Odometry, topic="/odom", callback=self.odom_callback, qos_profile=10)
        self.sub_vel = self.create_subscription(msg_type=Twist, topic="/cmd_vel", callback=self.vel_callback, qos_profile=10)

    def odom_callback(self, msg: Odometry) -> None:
        """
        Callback per i messaggi di odometria.

        Registra le informazioni sull'odometria e sulla covarianza nei log.

        Args:
            - msg: Il messaggio di odometria ricevuto.
        """
        self._log_info("ODOMETRIA: Lin, Ang")
        lin = msg.twist.twist.linear.x
        ang = msg.twist.twist.angular.z
        text = "Lineare: {}\t Angolare: {}".format(lin, ang)
        self._log_info(text)
        self._log_info("COVARIANZA")
        text = "covarianza: {}".format(msg.twist.covariance)
        self._log_info(text)
        text = "Posizione: x: {}, y: {}, z: {}".format(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        self._log_info(text)
    
    def vel_callback(self, msg: Twist) -> None:
        """
        Callback per i messaggi di comando di velocità.

        Registra le informazioni sui comandi di velocità nei log.

        Args:
            - msg: Il messaggio di comando di velocità ricevuto.
        """
        self._log_info("COMANDO: Lin, Ang")
        lin = msg.linear.x
        ang = msg.angular.z
        text = "Lineare: {}\t Angolare: {}".format(lin, ang)
        self._log_info(text)
    
    def _log_info(self, text: str) -> None:
        """
        Registra le informazioni nei log.

        Args:
            text: Il testo da registrare nei log.
        """
        self.get_logger().info(text)

def main() -> None:
    """
    Funzione principale per eseguire il nodo Handler.

    Inizializza il sistema ROS 2, crea un'istanza della classe Handler,
    quindi avvia il ciclo di esecuzione ROS 2.
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
