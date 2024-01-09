import xml.etree.ElementTree as ET
import xml.dom.minidom
import random

MAP_SIZE = 7.5

def generate_launch_file(num_robots, level_range):
    # Loop to generate launch configurations for each robot and level
    for level in range(*level_range):
        # Create the root launch element
        root = ET.Element("launch")

        # Create arguments for the robot model and position
        model_arg = ET.SubElement(root, "arg", name="model",
                                  default="$(env TURTLEBOT3_MODEL)",
                                  doc="model type [burger, waffle, waffle_pi]")

        for robot_index in range(num_robots):
            robot_name = f"robot{robot_index}"
            x_pos_arg = ET.SubElement(root, "arg", name=f"{robot_name}_x_pos")
            y_pos_arg = ET.SubElement(root, "arg", name=f"{robot_name}_y_pos")
            ET.SubElement(root, "arg", name=f"{robot_name}_z_pos", default="0.0")
            ET.SubElement(root, "arg", name=f"{robot_name}_gravity", default="false")

            # Generate random values for x_pos and y_pos
            x_pos_value = round(random.uniform(0, -1 * MAP_SIZE), 1)
            y_pos_value = round(random.uniform(0, MAP_SIZE), 1)

            # Set values for x_pos and y_pos
            x_pos_arg.set("default", str(x_pos_value))
            y_pos_arg.set("default", str(y_pos_value))

        # Create a new include element for each robot and level
        include = ET.SubElement(root, "include")
        include.set("file", "$(find gazebo_ros)/launch/empty_world.launch")

        # Set values for the world_name and other empty_world parameters
        world_name_arg = ET.SubElement(include, "arg", name="world_name",
                                       value=f"$(find turtlebot3_gazebo)/worlds/turtlebot3_level{level}.world")
        paused_arg = ET.SubElement(include, "arg", name="paused", value="false")
        use_sim_time_arg = ET.SubElement(include, "arg", name="use_sim_time", value="true")
        gui_arg = ET.SubElement(include, "arg", name="gui", value="true")
        headless_arg = ET.SubElement(include, "arg", name="headless", value="false")
        debug_arg = ET.SubElement(include, "arg", name="debug", value="false")

        for robot_index in range(num_robots):
            robot_name = f"robot{robot_index}"
            group = ET.SubElement(root, "group", ns=robot_name)

            # Create parameter for robot description
            param = ET.SubElement(group, "param", name="robot_description")
            param.set("command", "$(find xacro)/xacro --inorder "
                                 "$(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro")

            # Create node for publishinng the robot
            publisher_node = ET.SubElement(group, "node", name="robot_state_publisher", pkg="robot_state_publisher",
                                           type="robot_state_publisher", output="screen")
            ET.SubElement(publisher_node, "param", name="publish_frequency", type="double", value="50.0")
            ET.SubElement(publisher_node, "param", name="tf_prefix", value=robot_name)

            # Create node for spawning the robot
            spawn_node = ET.SubElement(group, "node", name="spawn_urdf", pkg="gazebo_ros", type="spawn_model")
            spawn_node.set("args", f"-urdf -model {robot_name} -x $(arg {robot_name}_x_pos) "
                                   f"-y $(arg {robot_name}_y_pos) -z $(arg {robot_name}_z_pos) "
                                   f"-param robot_description")

        # Convert the ElementTree to a string with indentation
        xml_str = ET.tostring(root, encoding="unicode")
        dom = xml.dom.minidom.parseString(xml_str)
        pretty_xml_str = dom.toprettyxml()

        # Write the formatted XML to a file
        with open(f"level{level}/turtlebot3_level{level}_robot{num_robots}.launch", "w") as file:
            file.write(pretty_xml_str)

if __name__ == "__main__":
    # Example usage: generate a launch file for 2 robots, levels 4 to 6
    generate_launch_file(num_robots=2, level_range=(3, 7))
