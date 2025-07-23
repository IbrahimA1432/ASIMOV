import os

def patch_urdf(input_path, output_path):
    with open(input_path, 'r') as f:
        content = f.read()

    # Replace the package:// style URI with relative path to meshes/
    content = content.replace('package://arm_urdf/meshes/', 'meshes/')

    with open(output_path, 'w') as f:
        f.write(content)

    print(f"✅ Patched URDF saved to {output_path}")

if __name__ == "__main__":
    patch_urdf('arm_urdf.urdf', 'patched_arm_urdf.urdf')
