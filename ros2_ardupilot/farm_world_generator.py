#!/usr/bin/env python3

"""
Farm World Generator Script to create world with defined number of fields and drones.
"""

import os
import math

NUM_FIELDS_X = 4
NUM_FIELDS_Y = 4
FIELD_SIZE = 30
FIELD_SPACING = 20
FIELD_HEIGHT = 0.05
DRONE_HEIGHT = 5.0
NUM_DRONES = 4
DRONE_SPACING = 5

FIELD_TYPES = [
    ("corn", "0.4 0.7 0.2 1"),
    ("soybean", "0.2 0.6 0.3 1"),
    ("wheat", "0.9 0.8 0.2 1"),
    ("rice", "0.7 0.7 0.5 1"),
    ("canola", "0.9 0.9 0.0 1"),
    ("barley", "0.8 0.7 0.3 1"),
    ("cotton", "0.9 0.9 0.9 1"),
    ("sunflower", "0.9 0.6 0.0 1"),
    ("alfalfa", "0.3 0.8 0.3 1"),
    ("potato", "0.6 0.5 0.3 1"),
    ("sugarcane", "0.5 0.8 0.5 1"),
    ("oats", "0.8 0.8 0.6 1"),
    ("lentil", "0.6 0.4 0.3 1"),
    ("flax", "0.4 0.5 0.8 1"),
    ("pea", "0.5 0.8 0.4 1"),
    ("mustard", "0.9 0.8 0.0 1")
]


def generate_header():
    return """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="multi_field_farm_world">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>
    <plugin filename="gz-sim-navsat-system"
      name="gz::sim::systems::NavSat">
    </plugin>

    <scene>
      <ambient>0.6 0.8 0.4</ambient>
      <background>0.7 0.9 1.0</background>
      <sky></sky>
    </scene>

    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.7 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>2000 2000</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>2000 2000</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.5 0.2 1</ambient>
            <diffuse>0.3 0.5 0.2 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def generate_field(field_id, field_type, color, x, y):
    return f"""    <model name="field_{field_id}">
      <static>true</static>
      <pose>{x} {y} {FIELD_HEIGHT} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{FIELD_SIZE} {FIELD_SIZE} 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{FIELD_SIZE} {FIELD_SIZE} 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>{color}</ambient>
            <diffuse>{color}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def generate_drone(drone_id, x, y):
    model_id = (drone_id % 3) + 1

    return f"""    <include>
      <uri>model://drone{model_id}</uri>
      <n>drone_{drone_id}</n>
      <pose degrees="true">{x} {y} {DRONE_HEIGHT} 0 0 0</pose>
      <link name="base_link">
        <sensor name="gps" type="gps">
          <always_on>1</always_on>
          <update_rate>10</update_rate>
        </sensor>
      </link>
    </include>
"""


def generate_drones():
    center_x = ((NUM_FIELDS_X - 1) * (FIELD_SIZE + FIELD_SPACING)) / 2
    center_y = ((NUM_FIELDS_Y - 1) * (FIELD_SIZE + FIELD_SPACING)) / 2

    drones_sdf = ""

    if NUM_DRONES <= 1:
        drones_sdf += generate_drone(1, center_x, center_y)
    else:
        formation_width = (NUM_DRONES - 1) * DRONE_SPACING

        for i in range(NUM_DRONES):
            x_pos = center_x - (formation_width / 2) + (i * DRONE_SPACING)
            drones_sdf += generate_drone(i + 1, x_pos, center_y)

    return drones_sdf


def generate_farm_world():
    sdf = generate_header()

    field_id = 1
    for row in range(NUM_FIELDS_Y):
        for col in range(NUM_FIELDS_X):
            x = col * (FIELD_SIZE + FIELD_SPACING)
            y = row * (FIELD_SIZE + FIELD_SPACING)

            index = (field_id - 1) % len(FIELD_TYPES)
            field_type, color = FIELD_TYPES[index]

            sdf += generate_field(field_id, field_type, color, x, y)
            field_id += 1

    sdf += generate_drones()
    sdf += "  </world>\n</sdf>"

    return sdf


def main():
    sdf_content = generate_farm_world()

    with open("farm_world.sdf", "w") as f:
        f.write(sdf_content)

    print(f"Generated farm world with {NUM_FIELDS_X}x{NUM_FIELDS_Y} fields and {NUM_DRONES} drones")
    print(f"File saved as farm_world.sdf")


if __name__ == "__main__":
    main()