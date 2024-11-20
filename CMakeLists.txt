cmake_minimum_required(VERSION 3.22)
project(gen3_6dof_dslr_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY description DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scene DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/precompute_sequence_with_collisions.py
  scripts/apply_plan_scene.py
  scripts/mock_servo_input.py
  scripts/joy_to_servo_input.py
  DESTINATION lib/${PROJECT_NAME}
)