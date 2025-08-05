### drims2_cells

This repository contains the robot descriptions, MoveIt configurations, and controller settings for the UR5e and ABB YuMi robots used in the DRIMS2 Summer School 2025.

To launch the robotic cells, use the following commands.

For the UR5e robot:

```bash
ros2 launch drims2_description ur5e_start.launch.py fake:=true
```

For the ABB YuMi robot:
```bash
ros2 launch drims2_description yumi_start.launch.py fake:=true
```

For the Tiago Pro robot:
```bash
ros2 launch drims2_description tiago_pro_start.launch.py fake:=true
```

Use `fake:=true` to run in simulation mode, and `fake:=false` to connect to the real hardware.

