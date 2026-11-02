#!/usr/bin/env python3
import os
import cv2

def generate_marker_model(marker_id: int, base_dir: str, side_pixels: int = 400, border_pixels: int = 50):
    """
    Generates an ArUco marker model with exact dimensions per Midterm specification:
    - Dictionary: DICT_4X4_50
    - Marker pattern size: 0.16m x 0.16m (side_pixels = 400)
    - White border: 0.02m margin on each side (border_pixels = 50)
    - Total plate size: 0.20m x 0.20m (total 500x500 pixels)
    """
    model_name = f'aruco_marker_{marker_id}'
    model_dir = os.path.join(base_dir, model_name)
    texture_dir = os.path.join(model_dir, 'materials', 'textures')
    os.makedirs(texture_dir, exist_ok=True)

    # 1. Generate Marker Image with White Border
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    raw_marker = cv2.aruco.generateImageMarker(dictionary, marker_id, side_pixels)
    bordered_marker = cv2.copyMakeBorder(
        raw_marker,
        border_pixels, border_pixels, border_pixels, border_pixels,
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
  <description>ArUco Marker ID {marker_id} (0.16m x 0.16m with 0.02m margin)</description>
</model>
"""
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write(config_content)

    # 3. Generate model.sdf (freestanding stand: base + pole + marker plate at z=0.35m)
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <!-- Base on floor (radius 0.12m, thickness 0.02m) -->
      <visual name="base_visual">
        <pose>0 0 0.01 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.12</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
      <collision name="base_collision">
        <pose>0 0 0.01 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.12</radius>
            <length>0.02</length>
          </cylinder>
        </geometry>
      </collision>

      <!-- Vertical stand pole (center at z=0.18m, height 0.34m) -->
      <visual name="pole_visual">
        <pose>0 0 0.18 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.015</radius>
            <length>0.34</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.6 0.6 0.6 1</diffuse>
        </material>
      </visual>
      <collision name="pole_collision">
        <pose>0 0 0.18 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.015</radius>
            <length>0.34</length>
          </cylinder>
        </geometry>
      </collision>

      <!-- Marker backing board centered at z=0.35m (0.01m thick, 0.20m x 0.20m) -->
      <visual name="marker_visual">
        <pose>0 0 0.35 0 0 0</pose>
        <geometry>
          <box>
            <size>0.01 0.20 0.20</size>
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
      <collision name="marker_collision">
        <pose>0 0 0.35 0 0 0</pose>
        <geometry>
          <box>
            <size>0.01 0.20 0.20</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write(sdf_content)

    print(f"Created complete freestanding Gazebo target stand: {model_name}")

def main():
    models_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'models'))
    # S1 is ID 1; C1 is ID 2
    for marker_id in [1, 2]:
        generate_marker_model(marker_id, models_dir)

if __name__ == '__main__':
    main()
