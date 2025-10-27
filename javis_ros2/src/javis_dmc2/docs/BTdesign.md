핵심 구현 노트 (Best Practice)
이 py_trees 코드가 설계대로 완벽하게 동작하려면, 사용자 정의 노드 구현 시 다음 두 가지를 반드시 지켜야 합니다.

비동기 (Asynchronous) 구현:

MoveTo, PickObject, RunListeningSequence, ScanUser 등 시간이 1틱 이상 걸리는 모든 노드는 절대 time.sleep() 등으로 메인 스레드를 블로킹(Blocking)하면 안 됩니다.

이 노드들은 tick() 메서드에서 ROS2 액션/서비스를 비동기적으로 호출하고, 즉시 **py_trees.common.Status.RUNNING**을 반환해야 합니다.

py_trees가 다음 틱에 다시 tick()을 호출하면, 노드는 ROS2 액션의 현재 상태(e.g., feedback)를 확인하여 RUNNING을 유지하거나, SUCCESS 또는 FAILURE를 반환해야 합니다.

이것을 지키지 않으면 EStop (P1)이 MoveTo (P6)가 끝날 때까지 30초간 반응하지 않는 심각한 오류가 발생합니다.

Halt (선점) 시그널 처리:

P3 LISTENING이 P4 TaskExecution을 선점할 때, py_trees는 RunTaskSubTree 노드의 terminate(self, new_status) 메서드를 자동으로 호출합니다.

사용자는 RunTaskSubTree의 terminate() 메서드 내부에 하위 트리(e.g., CleaningDesk_BT)에 stop()을 전파하는 코드를 작성해야 합니다.

마찬가지로, MoveTo나 PickObject 같은 노드의 terminate() 메서드 내부에 **self.ros_action_client.cancel_goal_async()**와 같은 실제 ROS2 액션 취소 코드를 반드시 구현해야 합니다.

이것을 지키지 않으면, "도비야"라고 불렀을 때 LISTENING 상태가 되지만 로봇팔은 CLEANING_DESK 동작을 멈추지 않는 오류가 발생합니다.