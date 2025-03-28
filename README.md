=== README ===
1. 디렉토리 및 파일 내용
Qt_GUI_with_Conveyor.zip

    Qt_GUI_with_Conveyor/
        main.py
        ui_design.ui
        conveyor_logic.py
        resources/
        config.yaml
        기타 여러 파일들

digital_twin.zip

    digital_twin/
        models/
        worlds/
        launch/
        config/
        CMakeLists.txt
        package.xml
        기타 ROS2 관련 파일들

turtlebot3_ws.zip

    turtlebot3_ws/
        src/
            turtlebot3/
            turtlebot3_simulations/
            turtlebot3_msgs/
        CMakeLists.txt
        package.xml
        
2. 수정 사항
SRDF JOINT 값 조정
YOLO PARAMETER 조정

3. 실행 순서
-Turtlebot
1. ros2 launch turtlebot3_manipulation_bringup hardware.launch.py
2. ros2 launch aruco_yolo aruco_yolo.launch.py-PC
3. ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py
4. ros2 run turtlebot_moveit turtlebot_arm_controller
5. ros2 run conveyor con
6. ros2 run conveyor gui
7. python3 filtered_rosbridge.py 브릿지 서버(생략 가능)
8. 유니티 엔진(생략 가능)
9. python3 test5.py (turtlebot_moveit/scripts/폴더안에서 실행)


