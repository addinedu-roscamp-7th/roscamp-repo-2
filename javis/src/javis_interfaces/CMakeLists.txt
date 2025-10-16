cmake_minimum_required(VERSION 3.8)
project(javis_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeArmPose.srv"
"srv/ChangeTrackingMode.srv"
"srv/CheckStorageBox.srv"
"srv/DetectBook.srv"
"srv/DetectTrash.srv"
"srv/DriveControlCommand.srv"
"srv/IdentifyBookShelf.srv"
"srv/RegisterPerson.srv"
"srv/VerifyBookPosition.srv"
"action/CleanDesk.action"
"action/CleanSeat.action"
"action/CollectReturnedBooks.action"
"action/CollectTrash.action"
"action/DisposeTrash.action"
"action/GuideNavigation.action"
"action/GuidePerson.action"
"action/MoveToTarget.action"
"action/PerformTask.action"
"action/PickBook.action"
"action/PickupBook.action"
"action/PlaceBook.action"
"action/RearrangeBook.action"
"action/ReshelvingBook.action"
"msg/BatteryStatus.msg"
"msg/CurrentPose.msg"
"msg/DetectedObjects.msg"
"msg/DobbyState.msg"
"msg/Obstacles.msg"
"msg/TrackingStatus.msg"

  DEPENDENCIES 
  std_msgs
  builtin_interfaces
  geometry_msgs
  sensor_msgs

)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
