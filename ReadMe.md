** 

<link name="radar_1">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.04 0.04 0.02"/>
    </geometry>
    <material name="red">
        <color rgba="1 0 0 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
       <box size="0.04 0.04 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<link name="radar_2">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.04 0.04 0.02"/>
    </geometry>
    <material name="red">
        <color rgba="1 0 0 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
       <box size="0.04 0.04 0.02"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="radar_1_joint" type="fixed">
  <origin xyz="0.0 -0.035 0.10714" rpy="0 0 -0.5236"/>
  <parent link="base_link"/>
  <child link="radar_1"/>
</joint>

<joint name="radar_2_joint" type="fixed">
  <origin xyz="0.0 0.035 0.10714" rpy="0 0 0.5236"/>
  <parent link="base_link"/>
  <child link="radar_2"/>
</joint>