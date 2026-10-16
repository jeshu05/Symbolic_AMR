"""Generates the 36m x 36m multi-room Research Facility Gazebo Harmonic SDF world."""
import os

def generate_world():
    walls = []
    
    def add_box(name, x, y, z, sx, sy, sz, r=0.7, g=0.7, b=0.75):
        walls.append(f"""
        <!-- {name} -->
        <collision name="{name}_col">
          <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
          <geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>
        </collision>
        <visual name="{name}_vis">
          <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
          <geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>
          <material><diffuse>{r} {g} {b} 1</diffuse></material>
        </visual>""")

    # 1. Outer Perimeter (36m x 36m, height 1.8m, thickness 0.3m)
    # North: y = +18
    add_box("outer_wall_n", 0, 18, 0.9, 36.3, 0.3, 1.8, 0.5, 0.5, 0.55)
    # South: y = -18
    add_box("outer_wall_s", 0, -18, 0.9, 36.3, 0.3, 1.8, 0.5, 0.5, 0.55)
    # East: x = +18
    add_box("outer_wall_e", 18, 0, 0.9, 0.3, 36.3, 1.8, 0.5, 0.5, 0.55)
    # West: x = -18
    add_box("outer_wall_w", -18, 0, 0.9, 0.3, 36.3, 1.8, 0.5, 0.5, 0.55)

    # 2. Central Base Station Hub (x: [-3.5, 3.5], y: [-3.5, 3.5])
    # With 4 wide exits (1.4m wide) toward North, South, East, West corridors
    add_box("hub_nw_h", -2.3, 3.5, 0.9, 2.4, 0.25, 1.8)
    add_box("hub_ne_h", 2.3, 3.5, 0.9, 2.4, 0.25, 1.8)
    add_box("hub_sw_h", -2.3, -3.5, 0.9, 2.4, 0.25, 1.8)
    add_box("hub_se_h", 2.3, -3.5, 0.9, 2.4, 0.25, 1.8)

    add_box("hub_wn_v", -3.5, 2.3, 0.9, 0.25, 2.4, 1.8)
    add_box("hub_ws_v", -3.5, -2.3, 0.9, 0.25, 2.4, 1.8)
    add_box("hub_en_v", 3.5, 2.3, 0.9, 0.25, 2.4, 1.8)
    add_box("hub_es_v", 3.5, -2.3, 0.9, 0.25, 2.4, 1.8)

    # 3. Room 2: Medical / First-Aid Station (x: [7, 15], y: [7, 15])
    # Center (11, 11). Doorway at x=8.5, y=7
    add_box("med_wall_n", 11, 15, 0.9, 8.0, 0.25, 1.8, 0.3, 0.8, 0.5)
    add_box("med_wall_e", 15, 11, 0.9, 0.25, 8.0, 1.8, 0.3, 0.8, 0.5)
    add_box("med_wall_s1", 12.5, 7, 0.9, 5.0, 0.25, 1.8, 0.3, 0.8, 0.5)
    add_box("med_wall_w", 7, 11, 0.9, 0.25, 8.0, 1.8, 0.3, 0.8, 0.5)
    # Medical cross visual landmark at (11, 11)
    add_box("med_cross_v", 11, 11, 0.6, 0.3, 1.2, 1.2, 0.1, 0.8, 0.2)
    add_box("med_cross_h", 11, 11, 0.6, 1.2, 0.3, 1.2, 0.1, 0.8, 0.2)

    # 4. Room 3: Restricted Keep-Out Zone Alpha (x: [7, 15], y: [-3.5, 3.5])
    # Marked with bright red warning boundary pillars and central hazard marker
    add_box("restr_wall_e", 15, 0, 0.9, 0.25, 7.0, 1.8, 0.9, 0.1, 0.1)
    add_box("restr_wall_n", 11, 3.5, 0.9, 8.0, 0.25, 1.8, 0.9, 0.1, 0.1)
    add_box("restr_wall_s", 11, -3.5, 0.9, 8.0, 0.25, 1.8, 0.9, 0.1, 0.1)
    # Red warning barrier at doorway x=7
    add_box("restr_barrier", 7, 0, 0.5, 0.3, 4.0, 1.0, 0.9, 0.1, 0.1)

    # 5. Room 4: Hazardous Chemical Storage (x: [7, 15], y: [-15, -7])
    # Center (11, -11). Doorway at x=8.5, y=-7
    add_box("chem_wall_e", 15, -11, 0.9, 0.25, 8.0, 1.8, 0.8, 0.6, 0.1)
    add_box("chem_wall_s", 11, -15, 0.9, 8.0, 0.25, 1.8, 0.8, 0.6, 0.1)
    add_box("chem_wall_n1", 12.5, -7, 0.9, 5.0, 0.25, 1.8, 0.8, 0.6, 0.1)
    add_box("chem_wall_w", 7, -11, 0.9, 0.25, 8.0, 1.8, 0.8, 0.6, 0.1)
    # Chemical drum obstacles
    add_box("chem_drum1", 11, -11, 0.5, 0.8, 0.8, 1.0, 0.8, 0.6, 0.1)
    add_box("chem_drum2", 13, -13, 0.5, 0.8, 0.8, 1.0, 0.8, 0.6, 0.1)

    # 6. Room 5: Server Room & IT Equipment (x: [-15, -7], y: [7, 15])
    # Center (-11, 11). Doorway at x=-8.5, y=7
    add_box("srv_wall_w", -15, 11, 0.9, 0.25, 8.0, 1.8, 0.2, 0.4, 0.8)
    add_box("srv_wall_n", -11, 15, 0.9, 8.0, 0.25, 1.8, 0.2, 0.4, 0.8)
    add_box("srv_wall_s1", -12.5, 7, 0.9, 5.0, 0.25, 1.8, 0.2, 0.4, 0.8)
    add_box("srv_wall_e", -7, 11, 0.9, 0.25, 8.0, 1.8, 0.2, 0.4, 0.8)
    # Server rack obstacles forming narrow aisles
    add_box("srv_rack1", -11, 11, 0.7, 0.8, 3.5, 1.4, 0.2, 0.2, 0.25)
    add_box("srv_rack2", -13, 11, 0.7, 0.8, 3.5, 1.4, 0.2, 0.2, 0.25)

    # 7. Room 6: Robotics Workshop & Staging (x: [-15, -7], y: [-3.5, 3.5])
    # Center (-11, 0). Doorway at x=-7, y=1.0
    add_box("wrk_wall_w", -15, 0, 0.9, 0.25, 7.0, 1.8)
    add_box("wrk_wall_n", -11, 3.5, 0.9, 8.0, 0.25, 1.8)
    add_box("wrk_wall_s", -11, -3.5, 0.9, 8.0, 0.25, 1.8)
    add_box("wrk_wall_e1", -7, -2.0, 0.9, 0.25, 3.0, 1.8)
    add_box("wrk_wall_e2", -7, 2.5, 0.9, 0.25, 2.0, 1.8)
    # Workbench table obstacle
    add_box("wrk_bench", -11, 0, 0.45, 2.0, 1.2, 0.9, 0.6, 0.4, 0.2)

    # 8. Room 7: Logistics & Warehouse (x: [-15, -7], y: [-15, -7])
    # Center (-11, -11). Doorway at x=-8.5, y=-7
    add_box("wh_wall_w", -15, -11, 0.9, 0.25, 8.0, 1.8)
    add_box("wh_wall_s", -11, -15, 0.9, 8.0, 0.25, 1.8)
    add_box("wh_wall_n1", -12.5, -7, 0.9, 5.0, 0.25, 1.8)
    add_box("wh_wall_e", -7, -11, 0.9, 0.25, 8.0, 1.8)
    # Storage shelves
    add_box("wh_shelf1", -12, -11, 0.8, 1.0, 4.0, 1.6, 0.4, 0.4, 0.45)
    add_box("wh_shelf2", -10, -11, 0.8, 1.0, 4.0, 1.6, 0.4, 0.4, 0.45)

    # 9. Room 8: Executive Conference Room (x: [-3, 3], y: [9, 15])
    # Center (0, 12). Doorway at x=0, y=9
    add_box("conf_wall_n", 0, 15, 0.9, 6.0, 0.25, 1.8)
    add_box("conf_wall_w", -3, 12, 0.9, 0.25, 6.0, 1.8)
    add_box("conf_wall_e", 3, 12, 0.9, 0.25, 6.0, 1.8)
    add_box("conf_wall_sw", -2.0, 9, 0.9, 2.0, 0.25, 1.8)
    add_box("conf_wall_se", 2.0, 9, 0.9, 2.0, 0.25, 1.8)
    # Conference table
    add_box("conf_table", 0, 12, 0.4, 1.6, 2.4, 0.8, 0.5, 0.3, 0.2)

    # 10. Room 9: Maintenance & Utility Hub (x: [-3, 3], y: [-15, -9])
    # Center (0, -12). Doorway at x=0, y=-9
    add_box("maint_wall_s", 0, -15, 0.9, 6.0, 0.25, 1.8)
    add_box("maint_wall_w", -3, -12, 0.9, 0.25, 6.0, 1.8)
    add_box("maint_wall_e", 3, -12, 0.9, 0.25, 6.0, 1.8)
    add_box("maint_wall_nw", -2.0, -9, 0.9, 2.0, 0.25, 1.8)
    add_box("maint_wall_ne", 2.0, -9, 0.9, 2.0, 0.25, 1.8)

    # 11. Intermediate Rooms & Bypass Corridors (Breakroom, Security, Archives, Lab)
    # Breakroom at (5, 5)
    add_box("break_w", 3.5, 5.5, 0.9, 0.25, 3.5, 1.8)
    add_box("break_n", 5.5, 7.0, 0.9, 3.5, 0.25, 1.8)

    # Security at (-5, 5)
    add_box("sec_e", -3.5, 5.5, 0.9, 0.25, 3.5, 1.8)
    add_box("sec_n", -5.5, 7.0, 0.9, 3.5, 0.25, 1.8)

    # Archive at (5, -5)
    add_box("arch_w", 3.5, -5.5, 0.9, 0.25, 3.5, 1.8)
    add_box("arch_s", 5.5, -7.0, 0.9, 3.5, 0.25, 1.8)

    # Lab at (-5, -5)
    add_box("lab_e", -3.5, -5.5, 0.9, 0.25, 3.5, 1.8)
    add_box("lab_s", -5.5, -7.0, 0.9, 3.5, 0.25, 1.8)

    # Corridor Obstacles / Pillars for interesting navigation dynamics
    add_box("corr_pil_n", 0, 6.0, 0.5, 0.5, 0.5, 1.0, 0.4, 0.4, 0.45)
    add_box("corr_pil_s", 0, -6.0, 0.5, 0.5, 0.5, 1.0, 0.4, 0.4, 0.45)
    add_box("corr_pil_e", 5.5, 0, 0.5, 0.5, 0.5, 1.0, 0.4, 0.4, 0.45)
    add_box("corr_pil_w", -5.5, 0, 0.5, 0.5, 0.5, 1.0, 0.4, 0.4, 0.45)

    sdf_content = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="research_facility_world">
    <physics name="1ms" type="ode">
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material>
            <ambient>0.82 0.82 0.82 1</ambient>
            <diffuse>0.82 0.82 0.82 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="research_facility">
      <static>true</static>
      <link name="walls">
        {''.join(walls)}
      </link>
    </model>
  </world>
</sdf>
"""
    return sdf_content

with open('/home/jesvanth/symbolic_explorer_ws/src/explorer_sim/worlds/research_facility_world.sdf', 'w') as f:
    f.write(generate_world())

print("Successfully generated 36m x 36m research_facility_world.sdf!")