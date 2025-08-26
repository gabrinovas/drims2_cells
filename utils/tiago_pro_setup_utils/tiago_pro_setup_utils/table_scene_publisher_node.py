import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject, ObjectColor
from shape_msgs.msg import SolidPrimitive


class AddTableNode(Node):
    def __init__(self):
        super().__init__('table_scene_publisher_node')

        # Declare parameters
        self.declare_parameter('table_length', 1.5)
        self.declare_parameter('table_width', 1.0)
        self.declare_parameter('table_thickness', 0.05)
        self.declare_parameter('leg_thickness', 0.05)
        self.declare_parameter('leg_height', 0.7)
        self.declare_parameter('table_pos_x', 1.3)
        self.declare_parameter('table_pos_y', 0.0)
        self.declare_parameter('table_pos_z', 0.0)

        # Get parameters
        L = self.get_parameter('table_length').value
        W = self.get_parameter('table_width').value
        T = self.get_parameter('table_thickness').value
        leg_T = self.get_parameter('leg_thickness').value
        leg_H = self.get_parameter('leg_height').value
        pos_x = self.get_parameter('table_pos_x').value
        pos_y = self.get_parameter('table_pos_y').value
        pos_z = self.get_parameter('table_pos_z').value

        scene = PlanningScene()
        scene.is_diff = True

        # -------------------------------
        # Table top (brown)
        table_top = CollisionObject()
        table_top.id = "table_top"
        table_top.header.frame_id = "base_footprint"

        top_primitive = SolidPrimitive()
        top_primitive.type = SolidPrimitive.BOX
        top_primitive.dimensions = [L, W, T]

        top_pose = PoseStamped()
        top_pose.header.frame_id = "base_footprint"
        top_pose.pose.position.x = pos_x
        top_pose.pose.position.y = pos_y
        top_pose.pose.position.z = pos_z + leg_H + T / 2.0
        top_pose.pose.orientation.w = 1.0

        table_top.primitives.append(top_primitive)
        table_top.primitive_poses.append(top_pose.pose)
        table_top.operation = CollisionObject.ADD
        scene.world.collision_objects.append(table_top)

        top_color = ObjectColor()
        top_color.id = "table_top"
        top_color.color.r = 0.55  # brown
        top_color.color.g = 0.27
        top_color.color.b = 0.07
        top_color.color.a = 1.0
        scene.object_colors.append(top_color)

        # -------------------------------
        # Camera base
        camera_base = CollisionObject()
        camera_base.id = "camera_base"
        camera_base.header.frame_id = "table_top"

        camera_base_primitive = SolidPrimitive()
        camera_base_primitive.type = SolidPrimitive.BOX
        camera_base_primitive.dimensions = [L, W/10, T]

        camera_base_pose = PoseStamped()
        camera_base_pose.header.frame_id = "table_top"
        camera_base_pose.pose.position.x = 0.0
        camera_base_pose.pose.position.y = -W/2 + W/20
        camera_base_pose.pose.position.z = T
        camera_base_pose.pose.orientation.w = 1.0

        camera_base.primitives.append(camera_base_primitive)
        camera_base.primitive_poses.append(camera_base_pose.pose)
        camera_base.operation = CollisionObject.ADD
        scene.world.collision_objects.append(camera_base)

        camera_base_color = ObjectColor()
        camera_base_color.id = "camera_base"
        camera_base_color.color.r = 0.5  # grey
        camera_base_color.color.g = 0.5
        camera_base_color.color.b = 0.5
        camera_base_color.color.a = 1.0
        scene.object_colors.append(camera_base_color)

        # Camera vertical
        camera_vertical_L = W/10
        camera_vertical_H = 0.5
        camera_vertical = CollisionObject()
        camera_vertical.id = "camera_vertical"
        camera_vertical.header.frame_id = "camera_base"

        camera_vertical_primitive = SolidPrimitive()
        camera_vertical_primitive.type = SolidPrimitive.BOX
        camera_vertical_primitive.dimensions = [camera_vertical_L, camera_vertical_L, camera_vertical_H]

        camera_vertical_pose = PoseStamped()
        camera_vertical_pose.header.frame_id = "camera_base"
        camera_vertical_pose.pose.position.x = -0.2
        camera_vertical_pose.pose.position.y = 0.0
        camera_vertical_pose.pose.position.z = T/2 + camera_vertical_H / 2.0
        camera_vertical_pose.pose.orientation.w = 1.0

        camera_vertical.primitives.append(camera_vertical_primitive)
        camera_vertical.primitive_poses.append(camera_vertical_pose.pose)
        camera_vertical.operation = CollisionObject.ADD
        scene.world.collision_objects.append(camera_vertical)

        camera_vertical_color = ObjectColor()
        camera_vertical_color.id = "camera_vertical"
        camera_vertical_color.color.r = 0.5  # grey
        camera_vertical_color.color.g = 0.5
        camera_vertical_color.color.b = 0.5
        camera_vertical_color.color.a = 1.0
        scene.object_colors.append(camera_vertical_color)


        # Camera horizontal
        camera_horizontal_L = W/10
        camera_horizontal_H = 0.5
        camera_horizontal = CollisionObject()
        camera_horizontal.id = "camera_horizontal"
        camera_horizontal.header.frame_id = "camera_vertical"

        camera_horizontal_primitive = SolidPrimitive()
        camera_horizontal_primitive.type = SolidPrimitive.BOX
        camera_horizontal_primitive.dimensions = [camera_horizontal_L, camera_horizontal_H, camera_horizontal_L]

        camera_horizontal_pose = PoseStamped()
        camera_horizontal_pose.header.frame_id = "camera_vertical"
        camera_horizontal_pose.pose.position.x = 0.0
        camera_horizontal_pose.pose.position.y = camera_horizontal_H/2 - camera_vertical_L/2
        camera_horizontal_pose.pose.position.z = camera_vertical_H/2 + camera_horizontal_L/2
        camera_horizontal_pose.pose.orientation.w = 1.0

        camera_horizontal.primitives.append(camera_horizontal_primitive)
        camera_horizontal.primitive_poses.append(camera_horizontal_pose.pose)
        camera_horizontal.operation = CollisionObject.ADD
        scene.world.collision_objects.append(camera_horizontal)

        camera_horizontal_color = ObjectColor()
        camera_horizontal_color.id = "camera_horizontal"
        camera_horizontal_color.color.r = 0.5  # grey
        camera_horizontal_color.color.g = 0.5
        camera_horizontal_color.color.b = 0.5
        camera_horizontal_color.color.a = 1.0
        scene.object_colors.append(camera_horizontal_color)

        # -------------------------------
        # Table legs (gray)
        leg_offsets = [
            ( L/2 - leg_T/2,  W/2 - leg_T/2),
            ( L/2 - leg_T/2, -W/2 + leg_T/2),
            (-L/2 + leg_T/2,  W/2 - leg_T/2),
            (-L/2 + leg_T/2, -W/2 + leg_T/2),
        ]

        for i, (dx, dy) in enumerate(leg_offsets):
            leg = CollisionObject()
            leg.id = f"table_leg_{i+1}"
            leg.header.frame_id = "base_footprint"

            leg_primitive = SolidPrimitive()
            leg_primitive.type = SolidPrimitive.BOX
            leg_primitive.dimensions = [leg_T, leg_T, leg_H]

            leg_pose = PoseStamped()
            leg_pose.header.frame_id = "base_footprint"
            leg_pose.pose.position.x = pos_x + dx
            leg_pose.pose.position.y = pos_y + dy
            leg_pose.pose.position.z = pos_z + leg_H / 2.0
            leg_pose.pose.orientation.w = 1.0

            leg.primitives.append(leg_primitive)
            leg.primitive_poses.append(leg_pose.pose)
            leg.operation = CollisionObject.ADD
            scene.world.collision_objects.append(leg)

            leg_color = ObjectColor()
            leg_color.id = leg.id
            leg_color.color.r = 0.5  # gray
            leg_color.color.g = 0.5
            leg_color.color.b = 0.5
            leg_color.color.a = 1.0
            scene.object_colors.append(leg_color)

        # -------------------------------
        # Safety barriers (yellow, semi-transparent)
        # barrier_thickness = 0.02  # 2 cm
        # barrier_height = 1.0      # 1 m

        # barrier_offsets = [
        #     +W/2 + barrier_thickness/2,  # positive Y side
        #     -W/2 - barrier_thickness/2   # negative Y side
        # ]

        # for i, y_offset in enumerate(barrier_offsets):
        #     barrier = CollisionObject()
        #     barrier.id = f"safety_barrier_{i+1}"
        #     barrier.header.frame_id = "base_footprint"

        #     barrier_primitive = SolidPrimitive()
        #     barrier_primitive.type = SolidPrimitive.BOX
        #     barrier_primitive.dimensions = [L, barrier_thickness, barrier_height]

        #     barrier_pose = PoseStamped()
        #     barrier_pose.header.frame_id = "base_footprint"
        #     barrier_pose.pose.position.x = pos_x
        #     barrier_pose.pose.position.y = pos_y + y_offset
        #     barrier_pose.pose.position.z = pos_z + leg_H + T + barrier_height / 2.0
        #     barrier_pose.pose.orientation.w = 1.0

        #     barrier.primitives.append(barrier_primitive)
        #     barrier.primitive_poses.append(barrier_pose.pose)
        #     barrier.operation = CollisionObject.ADD
        #     scene.world.collision_objects.append(barrier)

        #     barrier_color = ObjectColor()
        #     barrier_color.id = barrier.id
        #     barrier_color.color.r = 1.0  # yellow
        #     barrier_color.color.g = 1.0
        #     barrier_color.color.b = 0.0
        #     barrier_color.color.a = 0.0  # semi-transparent
        #     scene.object_colors.append(barrier_color)

        # -------------------------------
        # Apply scene via service
        self.cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_logger().info("Waiting for /apply_planning_scene service...")
        self.cli.wait_for_service()
        self.get_logger().info("/apply_planning_scene service available")

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.cli.call_async(req)
        future.add_done_callback(self.callback_done)

    def callback_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Table and safety barriers added to planning scene.")
            else:
                self.get_logger().error("Failed to apply planning scene.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = AddTableNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
