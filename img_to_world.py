import xml.etree.ElementTree as ET
import xml.dom.minidom
import cv2, argparse
from PIL import Image

def img_pre_process(file_name, file_type, scale=200):
    file_name = 'level' + file_name + '/level' + file_name
    origin_img = cv2.imread(file_name + '.' + file_type, cv2.IMREAD_GRAYSCALE)
    proc_img = cv2.resize(origin_img, (scale, scale), interpolation=cv2.INTER_NEAREST)
    proc_img = cv2.cvtColor(proc_img, cv2.COLOR_GRAY2BGR)
    proc_filename = file_name + "_proc." + file_type
    cv2.imwrite(proc_filename, proc_img)

def find_obstacles(file_name, file_type):
    obstacle_list = [] # pattern: [left, right, up, down]

    file_name = 'level' + file_name + '/level' + file_name
    img = Image.open(file_name + "_proc." + file_type)
    img = img.convert("L")

    width, height = img.size

    for y in range(height):
        has_wall = False
        for x in range(width):
            pixel_value = img.getpixel((x, y))
            # New Obstacle
            if not has_wall and pixel_value <= BLACK:
                has_wall = True
                obstacle_list.append([x, x, y, y])
            # Continue Obstacle: Pass
            # End Obstacle
            elif has_wall and pixel_value >= WHITE:
                has_wall = False
                obstacle_list[-1][1] = x-1
            elif has_wall and x == width-1:
                has_wall = False
                obstacle_list[-1][1] = x

    return obstacle_list

def obstacle_link(left, right, up, down, model):
    length = 2.0 if right - left < 2.0 else right - left
    width = 2.0 if down - up < 2.0 else down - up
    x = (left + right) / 2.0
    y = (up + down) / 2.0
    wall_name = "Wall_({0},{1})".format(x, y)

    def transform_tags(parent):
        geometry = ET.SubElement(parent, "geometry")
        box = ET.SubElement(geometry, "box")
        size = ET.SubElement(box, "size")
        size.text = "{0} {1} {2}".format(length * CELL_SCALE, width * CELL_SCALE, CELL_HEIGHT)
        pose = ET.SubElement(parent, "pose", frame='')
        pose.text = "-{0} {1} {2} 0 -0 0".format(x * CELL_SCALE, y * CELL_SCALE, CELL_HEIGHT/2.0)
    
    link = ET.SubElement(model, "link", name=wall_name)
    # collision
    collision = ET.SubElement(link, "collision", name=wall_name+"_Collision")
    transform_tags(collision)
    # inertial
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass").text = "0.5"
    ET.SubElement(inertial, "pose").text = "0 0.01 0 0 0 0"
    inertia = ET.SubElement(inertial, "inertia")
    ET.SubElement(inertia, "ixx").text = "0.004"
    ET.SubElement(inertia, "ixy").text = "0.001"
    ET.SubElement(inertia, "ixz").text = "0"
    ET.SubElement(inertia, "iyy").text = "0.006"
    ET.SubElement(inertia, "iyz").text = "0"
    ET.SubElement(inertia, "izz").text = "0.007"
    # visualization
    visual = ET.SubElement(link, "visual", name=wall_name+"_Visual")
    transform_tags(visual)
    # material
    material = ET.SubElement(visual, "material")
    material_script = ET.SubElement(material, "script")
    ET.SubElement(material_script, "uri").text = "file://media/materials/scripts/gazebo.material"
    ET.SubElement(material_script, "name").text = "Gazebo/Grey"
    ET.SubElement(material, "ambient").text = "1 1 1 1"
    # meta
    meta = ET.SubElement(visual, "meta")
    ET.SubElement(meta, "layer").text = "0"
    # position
    pose = ET.SubElement(link, "pose", frame='')
    pose.text = "0 0 0 0 -0 0"

def default_scenes(world):
    # light
    light = ET.SubElement(world, "include")
    light_uri = ET.SubElement(light, "uri")
    light_uri.text = "model://sun"

    # ground plane
    plane = ET.SubElement(world, "include")
    plane_uri = ET.SubElement(plane, "uri")
    plane_uri.text = "model://ground_plane"

    physics = ET.SubElement(world, "physics", type="ode")
    ET.SubElement(physics, "real_time_update_rate").text = "1000.0"
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_factor").text = "1"

    ode = ET.SubElement(physics, "ode")
    solver = ET.SubElement(ode, "solver")
    ET.SubElement(solver, "type").text = "quick"
    ET.SubElement(solver, "iters").text = "150"
    ET.SubElement(solver, "precon_iters").text = "0"
    ET.SubElement(solver, "sor").text = "1.4"
    ET.SubElement(solver, "use_dynamic_moi_rescaling").text = "1"

    constraints = ET.SubElement(ode, "constraints")
    ET.SubElement(constraints, "cfm").text = "0.00001"
    ET.SubElement(constraints, "erp").text = "0.2"
    ET.SubElement(constraints, "contact_max_correcting_vel").text = "2000.000000"
    ET.SubElement(constraints, "contact_surface_layer").text = "0.01000"

def parse_jpg_to_world(file_name, obstacle_list):
    # Create the root element for the SDF file
    sdf_root = ET.Element("sdf", version="1.6")
    world = ET.SubElement(sdf_root, "world", name="GridMap")

    default_scenes(world)

    model = ET.SubElement(world, "model", name="GridMap")
    pose = ET.SubElement(model, "pose", frame='')
    pose.text = "{0} {1} {2} 0 -0 0".format(0, 0, 0)

    # Loop through the image pixels
    for obstacle in obstacle_list:
        obstacle_link(obstacle[0], obstacle[1], obstacle[2], obstacle[3], model)

    ET.SubElement(model, "static").text = "1"

    # Convert the ElementTree to a string with indentation
    xml_str = ET.tostring(sdf_root, encoding="unicode")
    dom = xml.dom.minidom.parseString(xml_str)
    pretty_xml_str = dom.toprettyxml()

    # Write the formatted XML to a file
    with open(f"level{file_name}/turtlebot3_level{file_name}.world", "w") as file:
        file.write(pretty_xml_str)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", type=str, help="path to the input file", required=True)
    parser.add_argument("--type", type=str, help="input file type", default="jpg", required=False)
    parser.add_argument("--size", type=int, help="image scale", default=75, required=False)
    parser.add_argument("--scale", type=float, help="cell scale", default=0.1, required=False)
    parser.add_argument("--height", type=float, help="wall height", default=0.5, required=False)

    args = parser.parse_args()

    CELL_HEIGHT = args.height
    CELL_SCALE = args.scale

    BLACK = 5
    WHITE = 250

    img_pre_process(args.name, args.type, scale=args.size)
    obstacle_list = find_obstacles(args.name, args.type)
    parse_jpg_to_world(args.name, obstacle_list)
