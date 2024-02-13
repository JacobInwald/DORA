from sensors.camera import Camera
from sensors.lidar import LiDAR
from sensors.gps import GPS
from actuators.move_pub import MovePublisher, MoveType
import rclpy
import threading
import asyncio
import rclpy
from rclpy.node import Node

async def spin_node(node: Node):
    cancel = node.create_guard_condition(lambda: None)
    def _spin(node: Node,
              future: asyncio.Future,
              event_loop: asyncio.AbstractEventLoop):
        while not future.cancelled():
            rclpy.spin_once(node)
        if not future.cancelled():
            event_loop.call_soon_threadsafe(future.set_result, None)
    event_loop = asyncio.get_event_loop()
    spin_task = event_loop.create_future()
    spin_thread = threading.Thread(target=_spin, args=(node, spin_task, event_loop))
    spin_thread.start()
    try:
        await spin_task
    except asyncio.CancelledError:
        cancel.trigger()
    spin_thread.join()
    node.destroy_guard_condition(cancel)

class ROSInterface:
    
    def __init__(self) -> None:
        self.gps = GPS()
        self.lidar = LiDAR()
        self.camera = Camera()
        self.move_publisher = MovePublisher()
        self.move_dist = 0.05

    # ! METHODS
    # TODO: Implement these methods. They are the basic movement and sensor methods.
    # TODO: Use the simulator class as a base for how the output (if any) should look like.
    # TODO: Most of these methods can't really be implemented without an idea of what method of hardware control we are using.
    # TODO: For now, just implement whatever you can and research what the best approach is
    # TODO: (Wanting)

    def forward(self, move_dist) -> None:
        """
        Moves the robot in a given direction at a given speed.
        """
        self.move_publisher.publish(MoveType.FORWARD, move_dist)
        rclpy.spin_once(self.move_publisher)
        
    def backward(self, move_dist) -> None:
        """
        Moves the robot in a given direction at a given speed.
        """
        self.move_publisher.publish(MoveType.BACKWARD, -move_dist)
        rclpy.spin_once(self.move_publisher)
        
    def turn(self, angle) -> None:
        """
        Turns the robot by a given angle.
        """
        self.move_publisher.publish(MoveType.TURN, angle)
        rclpy.spin_once(self.move_publisher)

    async def spin(self) -> None:
        """
        Spins the robot.
        """
        print("Spinning")
        for i in range(10000):
            print(f"Spinning {i}")
            for n in [self.lidar, self.move_publisher]:
                task = asyncio.get_event_loop().create_task(self.run(n))
                await asyncio.wait([task], return_when=asyncio.FIRST_COMPLETED)
                task.cancel()
        rclpy.shutdown()
        
    async def run(self, node) -> None:
        """
        """    
        spin_task = asyncio.get_event_loop().create_task(spin_node(node))
        sleep_task = asyncio.get_event_loop().create_task(asyncio.sleep(1.0))

        # concurrently execute both tasks
        await asyncio.wait([spin_task, sleep_task], return_when=asyncio.FIRST_COMPLETED)

        # cancel tasks
        if spin_task.cancel():
            await spin_task
        if sleep_task.cancel():
            await sleep_task
    
    def update_sensors(self) -> None:
        """
        Spins the robot once.
        """
        rclpy.spin_once(self.lidar)
        rclpy.spin_once(self.gps)
        rclpy.spin_once(self.move_publisher)
    
    def destroy(self) -> None:
        """
        Destroys the robot.
        """
        self.move_publisher.destroy()
        self.lidar.destroy()
        # self.gps.destroy()
        # self.camera.destroy()
        rclpy.shutdown()
    
    def __str__(self) -> str:
        return f"DORA at {self.gps.pos}, with angle of {self.gps.pos}"


def spin():
    rclpy.init()
    interface = ROSInterface()
    interface.spin()
    interface.destroy()
    print("Done")   
    