import numpy as np
import trimesh
from os import listdir
from os.path import isfile, join, basename, splitext
import xml.etree.ElementTree as ET
import argparse

RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
BLUE = '\033[34m'
ENDC = '\033[m'

parser = argparse.ArgumentParser(prog='URDF Collision Simplifier',
                                 description='This script will simplify the collision shapes of URDF models.\U0001F916',
                                 epilog='Text at the bottom of help')
parser.add_argument('package_name', type=str,
                    help='Package name of the robot description (e.g. \"robot_description\").')
parser.add_argument('package_path', type=str,
                    help='Path to the source directory of your package (e.g. \"./\").')
parser.add_argument('--simplification', type=str, choices=['smallest_primitive', 'box', 'cylinder', 'sphere', 'convex_mesh'], default='smallest_primitive',
                    help='Which type of simplification should be performed. You can either chose one of the URDF primitive shapes, a convex mesh or automatically chose the smallest primitive.')
parser.add_argument('--urdf-folder', type=str, default='urdf/',
                    help='Name of the subfolder where the URDF file is.')
parser.add_argument('--urdf-name', type=str,
                    default='robot.urdf.xacro', help='Name of the URDF file.')
parser.add_argument('--mesh-folder', type=str, default='meshes/collision/',
                    help='Name of the subfolder where the collision mesh files are.')
args = parser.parse_args()

# parse URDF
package_name = args.package_name
package_path = args.package_path + package_name + '/'
urdf_path = package_path + args.urdf_folder + args.urdf_name
ET.register_namespace('xacro', "http://www.ros.org/wiki/xacro")
urdf_tree = ET.parse(urdf_path, ET.XMLParser(
    target=ET.TreeBuilder(insert_comments=True)))
urdf_root = urdf_tree.getroot()
print(f"{BLUE}Loaded URDF at {urdf_path}{ENDC}")

# find mesh files
mesh_path = package_path + args.mesh_folder
print(f"Will search for collision meshes in {mesh_path}")
mesh_files = [f for f in listdir(mesh_path) if isfile(join(mesh_path, f))]

for mesh_file_name in mesh_files:
    if '_simple' in mesh_file_name:
        print(
            f"{BLUE}Ignoring file {mesh_file_name}, since it is propably an already simplified mesh.{ENDC}")
        continue
    mesh_file_path = mesh_path+mesh_file_name
    # find URDF collision entries that use this mesh
    print(f'Processing mesh {mesh_file_name}')
    corresponding_collision_shape_entries = urdf_root.findall(
        ".//collision/geometry/mesh[@filename='package://" + package_name + "/meshes/collision/" + mesh_file_name + "']/../..")
    if len(corresponding_collision_shape_entries) == 0:
        print(
            f"  {YELLOW}Warning: Could not find an URDF collision entry for mesh {mesh_file_name}{ENDC}")
    else:
        # load mesh
        mesh = trimesh.load(mesh_file_path)
        if not mesh.is_watertight:
            print(f"  {YELLOW}Warnning: The original mesh is not watertight{ENDC}")
        # chose which simplification will be used
        simplification = args.simplification
        if simplification == 'smallest_primitive':
            # automatically find best representation for this mesh
            volumes = [('box', mesh.bounding_box_oriented), ('cylinder', mesh.bounding_cylinder),
                       ('sphere', mesh.bounding_sphere)]
            smallest_volume_amount = np.inf
            smallest_volume_index = 0
            i = 0
            for vol in volumes:
                if vol[1].volume < smallest_volume_amount:
                    smallest_volume_amount = vol[1].volume
                    smallest_volume_index = i
                i += 1
            smallest_volume = volumes[smallest_volume_index]
            print(
                f"  Smallest volumne is {smallest_volume[0]} with {smallest_volume[1].volume:.6f}.\n"
                f"  This is {smallest_volume[1].volume/mesh.volume:.6f} times the original value.")
            simplification = smallest_volume[0]

        for corresponding_collision_shape_entry in corresponding_collision_shape_entries:
            # remove mesh entry
            mesh_entry = corresponding_collision_shape_entry.findall(
                ".//mesh")
            if len(mesh_entry) != 1:
                print(
                    f"  {RED}Error: There is something wrong with the collsion entry. \
                        There should be exactly one collision mesh, but there are {len(mesh_entry)} \
                        in:\n{ET.tostring(corresponding_collision_shape_entry)}{ENDC}")
                exit(1)
            origin_entry = corresponding_collision_shape_entry.findall(
                "./origin")
            if len(origin_entry) != 1:
                print(
                    f"  {RED}Error: There is something wrong with the collsion entry. There should be \
                        exactly one origin, but there are {len(origin_entry)} \
                        in:\n {ET.tostring(corresponding_collision_shape_entry)}{ENDC}")
                exit(1)
            # remove old mesh collision object
            geometry_entry = corresponding_collision_shape_entry.findall(
                "./geometry")[0]
            geometry_entry.remove(mesh_entry[0])

            if simplification == 'convex_mesh':
                new_collision_mesh = mesh.convex_hull
                old_mesh_filename_without_extendsion = splitext(
                    basename(mesh_file_name))[0]
                trimesh.exchange.export.export_mesh(
                    new_collision_mesh, mesh_path + old_mesh_filename_without_extendsion + '_simple.STL', 'stl')
                new_mesh_entry = ET.Element('mesh')
                new_mesh_entry.set(
                    'filename', f'package://{package_name}/{args.mesh_folder}{old_mesh_filename_without_extendsion}_simple.STL')
                geometry_entry.append(new_mesh_entry)
            else:
                # replace mesh with primitive
                if simplification == 'box':
                    # get necessary information about the new collision shape
                    new_collision_mesh = mesh.bounding_box_oriented
                    size = new_collision_mesh.primitive.extents
                    size_str = f"{size[0]} {size[1]} {size[2]}"
                    # add primitive to URDF
                    primitve = ET.Element('box')
                    primitve.set("size", size_str)
                    geometry_entry.append(primitve)
                elif simplification == 'cylinder':
                    new_collision_mesh = mesh.bounding_cylinder
                    radius = new_collision_mesh.primitive.radius
                    length = new_collision_mesh.primitive.height
                    primitve = ET.Element('cylinder')
                    primitve.set("radius", str(radius))
                    primitve.set("length", str(length))
                    geometry_entry.append(primitve)
                    # radius length
                elif simplification == 'sphere':
                    new_collision_mesh = mesh.bounding_sphere
                    radius = new_collision_mesh.primitive.radius
                    primitve = ET.Element('sphere')
                    primitve.set("radius", str(radius))
                    geometry_entry.append(primitve)
                else:
                    print(
                        f"  {RED}Error: Simplification \'{simplification}\' not known.")
                    exit(1)

                # correct origin entry
                xyz = trimesh.transformations.translation_from_matrix(
                    new_collision_mesh.primitive.transform)
                rpy = trimesh.transformations.euler_from_matrix(
                    new_collision_mesh.primitive.transform)
                xyz_str = f"{xyz[0]} {xyz[1]} {xyz[2]}"
                rpy_str = f"{rpy[0]} {rpy[1]} {rpy[2]}"
                origin_entry[0].set('xyz', xyz_str)
                origin_entry[0].set('rpy', rpy_str)

# save modified URDF
urdf_tree.write(urdf_path)
print(f"{GREEN}Simplification completed!{ENDC}")
