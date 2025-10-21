cmake_minimum_required(VERSION 3.8)
project(javis_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(action_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/main/BatteryStatus.msg"
  "msg/main/CurrentPose.msg"
  "msg/main/DobbyState.msg"
  "msg/ai/DetectedObjects.msg"
  "msg/ai/Obstacles.msg"
  "msg/ai/TrackingStatus.msg"
  "srv/ai/DetectTrash.srv"
  "srv/arm/ChangeArmPose.srv"
  "srv/drive/ChangeTrackingMode.srv"
  "srv/drive/CheckStorageBox.srv"
  "srv/drive/DetectBook.srv"
  "srv/drive/DriveControlCommand.srv"
  "srv/drive/IdentifyBookShelf.srv"
  "srv/drive/RegisterPerson.srv"
  "srv/drive/VerifyBookPosition.srv"
  "action/arm/CleanDesk.action"
  "action/arm/CollectReturnedBooks.action"
  "action/arm/CollectTrash.action"
  "action/arm/DisposeTrash.action"
  "action/arm/PickBook.action"
  "action/arm/PlaceBook.action"
  "action/arm/RearrangeBook.action"
  "action/drive/GuideNavigation.action"
  "action/drive/MoveToTarget.action"
  "action/main/CleanSeat.action"
  "action/main/GuidePerson.action"
  "action/main/PerformTask.action"
  "action/main/PickupBook.action"
  "action/main/ReshelvingBook.action"
  "srv/test/MyJson.srv"
  "msg/main/BookInfo.msg"
  "msg/main/Location.msg"
  "msg/main/StorageInfo.msg"
  "msg/main/BookPicupJson.msg"
  
  DEPENDENCIES 
  std_msgs
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  unique_identifier_msgs
  action_msgs

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
