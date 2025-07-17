from urdfpy import URDF
import numpy as np
import xml.etree.ElementTree as ET

def patch_visuals_with_dummy_geometry(input_path, output_path):
    tree = ET.parse(input_path)
    root = tree.getroot()

    for visual in root.findall(".//visual"):
        geometry = visual.find("geometry")
        mesh = geometry.find("mesh")
        if mesh is not None:
            geometry.remove(mesh)
            box = ET.SubElement(geometry, "box")
            box.set("size", "0.01 0.01 0.01")  # Small dummy cube

    for collision in root.findall(".//collision"):
        geometry = collision.find("geometry")
        mesh = geometry.find("mesh")
        if mesh is not None:
            geometry.remove(mesh)
            box = ET.SubElement(geometry, "box")
            box.set("size", "0.01 0.01 0.01")

    tree.write(output_path)
    print(f"✅ Dummy geometry inserted and saved to {output_path}")

def main():
    input_file = 'patched_arm_urdf.urdf'
    cleaned_file = 'dummy_geom_urdf.urdf'
    patch_visuals_with_dummy_geometry(input_file, cleaned_file)

    robot = URDF.load(cleaned_file)

    print("\nLink Lengths (from joint origins):")
    for joint in robot.joints:
        if joint.origin is not None:
            xyz = joint.origin[:3, 3]
            length = np.linalg.norm(xyz)
            print(f"{joint.name}: {length:.3f} m")

if __name__ == "__main__":
    main()
