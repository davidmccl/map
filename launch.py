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
        for robot_index in range(num_robots):
            model_arg = ET.SubElement(root, "arg", name="model", default="$(env TURTLEBOT3_MODEL)")
            x_pos_arg = ET.SubElement(root, "arg", name="x_pos")
            y_pos_arg = ET.SubElement(root, "arg", name="y_pos")
            ET.SubElement(root, "arg", name="z_pos", default="0.0")
            ET.SubElement(root, "arg", name="gravity", default="false")

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

        # Create parameter for robot description
        param = ET.SubElement(root, "param", name="robot_description")
        param.set("command", "$(find xacro)/xacro --inorder "
                             "$(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro")

        # Create node for spawning the robot
        spawn_node = ET.SubElement(root, "node", name="spawn_urdf", pkg="gazebo_ros", type="spawn_model")
        spawn_node.set("args", "-urdf -model turtlebot3 -x $(arg x_pos) "
                               "-y $(arg y_pos) -z $(arg z_pos) -param robot_description")

        # Convert the ElementTree to a string with indentation
        xml_str = ET.tostring(root, encoding="unicode")
        dom = xml.dom.minidom.parseString(xml_str)
        pretty_xml_str = dom.toprettyxml()

        # Write the formatted XML to a file
        with open(f"level{level}/turtlebot3_level{level}.launch", "w") as file:
            file.write(pretty_xml_str)

if __name__ == "__main__":
    # Example usage: generate a launch file for 2 robots, levels 4 to 6
    generate_launch_file(num_robots=2, level_range=(4, 7))
