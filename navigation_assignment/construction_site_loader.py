import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import math

class ConstructionSiteLoader(Node):
    def __init__(self):
        super().__init__('construction_site_loader')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo spawn service...')
            
        self.get_logger().info('Spawning Inspection Circuit Obstacles...')
        self.spawn_site()

    def spawn_box(self, name, x, y, size_x, size_y, size_z, r, g, b):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = f"""
        <?xml version="1.0" ?>
        <sdf version="1.6">
            <model name='{name}'>
                <static>true</static>
                <link name='link'>
                    <collision name='collision'>
                        <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
                    </collision>
                    <visual name='visual'>
                        <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
                        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
                    </visual>
                </link>
                <pose>{x} {y} {size_z/2.0} 0 0 0</pose>
            </model>
        </sdf>
        """
        self.client.call_async(request)

    def spawn_cylinder(self, name, x, y, radius, length, r, g, b):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = f"""
        <?xml version="1.0" ?>
        <sdf version="1.6">
            <model name='{name}'>
                <static>true</static>
                <link name='link'>
                    <collision name='collision'>
                        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
                    </collision>
                    <visual name='visual'>
                        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
                        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
                    </visual>
                </link>
                <pose>{x} {y} {length/2.0} 0 0 0</pose>
            </model>
        </sdf>
        """
        self.client.call_async(request)

    def spawn_site(self):
        RED = (0.8, 0.1, 0.1); ORANGE = (1.0, 0.5, 0.0); GREY = (0.3, 0.3, 0.3)

        # 1. THE "CHICANE" (Tight Walls at Start)
        self.spawn_box("wall_L1", 3.0, 1.2, 2.0, 0.2, 0.8, *RED)
        self.spawn_box("wall_R1", 3.0, -1.2, 2.0, 0.2, 0.8, *RED)

        # 2. THE "ISLANDS" (Obstacles inside the loop)
        self.spawn_cylinder("barrel_center", 5.0, 0.0, 0.4, 0.9, *ORANGE)
        self.spawn_cylinder("barrel_turn", 7.5, 1.0, 0.4, 0.9, *ORANGE)

        # 3. THE "TURNAROUND" (Dumpster at the far end)
        self.spawn_box("dumpster", 9.5, 0.0, 1.0, 2.5, 1.2, *GREY)
        
        # 4. RETURN LEG OBSTACLES
        self.spawn_box("wall_return", 5.0, -2.5, 3.0, 0.2, 0.8, *RED)

        self.get_logger().info("Circuit Generated!")

def main(args=None):
    rclpy.init(args=args)
    node = ConstructionSiteLoader()
    rclpy.shutdown()