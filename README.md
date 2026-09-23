# UR3e Cartesian Path Drawing with ROS 2 Humble

ROS 2 package sử dụng MoveIt 2 để điều khiển robot UR3e mô phỏng trong Gazebo thực hiện:

- Quỹ đạo chữ **H**.
- Quỹ đạo **hình tròn**.
- Hiển thị quỹ đạo dự kiến bằng Marker trong RViz.

## 1. Môi trường

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- Gazebo
- Universal Robots ROS 2 GZ Simulation
- Robot UR3e

## 2. Cấu trúc package

```text
ur3e_letter_drawer/
├── config/
│   └── ur3e_drawer.rviz
├── launch/
│   ├── draw_circle.launch.py
│   └── draw_letter.launch.py
├── src/
│   ├── circle_drawer.cpp
│   └── letter_drawer.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 3. Các node chính

### `letter_drawer_node`

- Kết nối tới planning group `ur_manipulator`.
- Đưa đầu công tác tới điểm bắt đầu.
- Tạo waypoint cho ba nét của chữ H.
- Nâng đầu công tác khi di chuyển giữa các nét.
- Gọi `computeCartesianPath()` để tính quỹ đạo.
- Chỉ thực thi khi tỷ lệ quỹ đạo đạt ít nhất 95%.
- Publish Marker màu đỏ trên topic `/letter_marker`.

### `circle_drawer_node`

- Tạo 72 đoạn nhỏ trên đường tròn.
- Bán kính đường tròn là 0.06 m.
- Đường kính quỹ đạo là 0.12 m.
- Tính tọa độ bằng hàm sin và cos.
- Công bố Marker màu xanh trên topic `/letter_marker`.

## 4. Cài đặt

Tạo workspace:

```bash
mkdir -p ~/workspaces/ur_gz_humble/src
cd ~/workspaces/ur_gz_humble
```

Nạp ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

Clone Universal Robots simulation:

```bash
git clone -b humble \
  https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git \
  src/ur_simulation_gz
```

Clone package bài tập:

```bash
git clone https://github.com/dhung68/UR3e-Cartesian-path-drawing-using-ROS-2-Humble-and-MoveIt-2\
  src/ur3e_letter_drawer
```

Cài dependency:

```bash
rosdep update

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  -r -y
```

Build workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Chạy quỹ đạo chữ H

```bash
ros2 launch ur3e_letter_drawer draw_letter.launch.py
```

Launch file sẽ:

1. Khởi động UR3e trong Gazebo.
2. Khởi động MoveIt và RViz.
3. Đợi hệ thống sẵn sàng.
4. Tự động chạy `letter_drawer_node`.

## 6. Chạy quỹ đạo hình tròn

```bash
ros2 launch ur3e_letter_drawer draw_circle.launch.py
```

## 7. Hiển thị Marker trong RViz

Trong RViz:

1. Chọn **Add**.
2. Chọn **Marker**.
3. Đặt topic thành:

```text
/letter_marker
```

Marker biểu diễn quỹ đạo Cartesian dự kiến của đầu công tác.

## 8.Phương pháp tạo quỹ đạo

### Chữ H

Chữ H gồm ba nét:

1. Nét dọc bên trái.
2. Nét dọc bên phải.
3. Nét ngang ở giữa.

Kích thước chữ:

- Chiều cao: 0.12 m.
- Chiều rộng: 0.12 m.
- Độ nâng đầu công tác: 0.04 m.

### Hình tròn

Các waypoint được tạo theo:

```text
x = center_x + radius × cos(angle)
y = center_y + radius × sin(angle)
z = constant
```

Đường tròn được chia thành 72 đoạn để tạo chuyển động mượt.

## 9.Kết quả

- Cartesian path chữ H: 100%.
- Cartesian path hình tròn: 100%.
- Robot thực thi thành công trong Gazebo.
- Joint state và trajectory controller hoạt động bình thường.


