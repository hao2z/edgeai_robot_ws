#!/usr/bin/env python3
import os
import cv2

def generate_marker_model(marker_id: int, base_dir: str, side_pixels: int = 500, border_bits: int = 50):
    model_name = f'aruco_marker_{marker_id}'
    model_dir = os.path.join(base_dir, model_name)
    texture_dir = os.path.join(model_dir, 'materials', 'textures')
    os.makedirs(texture_dir, exist_ok=True)

    # 1. Generate Marker Image with White Border
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    raw_marker = cv2.aruco.generateImageMarker(dictionary, marker_id, side_pixels)
    bordered_marker = cv2.copyMakeBorder(
        raw_marker,
        border_bits, border_bits, border_bits, border_bits,
        cv2.BORDER_CONSTANT,
        value=255
    )
    png_path = os.path.join(texture_dir, f'marker_{marker_id}.png')
    cv2.imwrite(png_path, bordered_marker)

    # 2. Generate model.config
    config_content = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Hao</name>
  </author>
  <description>ArUco Marker ID {marker_id}</description>
</model>
"""
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write(config_content)

    # 3. Generate model.sdf (relative texture path)
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>0.02 0.20 0.20</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>materials/textures/marker_{marker_id}.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.02 0.20 0.20</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write(sdf_content)

    print(f"Created complete Gazebo model: {model_name}")

def main():
    models_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'models'))
    for marker_id in [0, 1, 2]:
        generate_marker_model(marker_id, models_dir)

if __name__ == '__main__':
    main()