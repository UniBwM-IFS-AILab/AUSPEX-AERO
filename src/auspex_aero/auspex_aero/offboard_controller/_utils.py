# Some utility classes for managing the state of the action sequence execution and the state of low-level actions (f.e. flight_manager, camera_controller, etc.)

from enum import Enum
import threading
from typing import List

from auspex_utils.action_conversion import create_action_from_msg, Action, BaseAction

# ----- for managing the state of the action sequence ----- #

class ExecutionStatus(Enum):
    IDLE = 0
    EXECUTING = 1
    PAUSED = 2
    CANCELLING = 3
    FINISHED = 4

class ExecutionFinishType(Enum):
    SUCCESS = 0
    CANCELLED = 1
    FAILED = 2

class ActionSequence:
    def __init__(self, actions: List[Action]):
        self._lock = threading.Lock()
        
        self._action_sequence = []
        for action in actions:
            self._action_sequence.append(create_action_from_msg(action))
        self._current_index = 0
    
    @property
    def current_action(self) -> BaseAction:
        with self._lock:
            return self._action_sequence[self._current_index]
    
    @property
    def is_finished(self) -> bool:
        with self._lock:
            return self._current_index >= len(self._action_sequence)
    
    def action_completed(self):
        with self._lock:
            self._current_index += 1
    
    def __str__(self) -> str:
        with self._lock:
            return f"ActionSequence({[action.name for action in self._action_sequence]}, current index {self._current_index} / {len(self._action_sequence)})"

class ExecutionState:
    def __init__(self, sequence: ActionSequence):
        self._lock = threading.Lock()
        self.sequence = sequence
        
        self._status = ExecutionStatus.IDLE
        self._result_type = None
        self._result_msg = None
    
    @property
    def status(self):
        with self._lock:
            return self._status
    
    def start(self):
        with self._lock:
            self._status = ExecutionStatus.EXECUTING
    
    def pause(self):
        with self._lock:
            self._status = ExecutionStatus.PAUSED
    
    def resume(self):
        with self._lock:
            self._status = ExecutionStatus.EXECUTING
    
    def cancel(self):
        with self._lock:
            self._status = ExecutionStatus.CANCELLING
    
    def mark_successful(self, result = None):
        with self._lock:
            self._status = ExecutionStatus.FINISHED
            self._result_type = ExecutionFinishType.SUCCESS
            self._result_msg = result
    
    def mark_cancelled(self, result = None):
        with self._lock:
            self._status = ExecutionStatus.FINISHED
            self._result_type = ExecutionFinishType.CANCELLED
            self._result_msg = result

    def mark_failed(self, result = None):
        with self._lock:
            self._status = ExecutionStatus.FINISHED
            self._result_type = ExecutionFinishType.FAILED
            self._result_msg = result
    
    @property
    def result(self):
        with self._lock:
            return self._result_type, self._result_msg

# ----- for managing the state of the child actions ----- #

class ActionExecutionStatus(Enum):
    NOT_YET_SENT = 0
    PENDING = 1
    EXECUTING = 2
    CANCELING = 3
    FINISHED = 4

class ActionFinishType(Enum):
    GOAL_NOT_ACCEPTED = 0
    SUCCESS = 1
    CANCELED = 2
    FAILED = 3

class ChildAction:
    def __init__(self, client, goal_msg, feedback_callback = None):
        self._goal_msg = goal_msg
        self._client = client
        self._feedback_callback = feedback_callback

        self._lock = threading.Lock()
        self._state = ActionExecutionStatus.NOT_YET_SENT
        self._finish_type = ActionFinishType.GOAL_NOT_ACCEPTED
        self._result_msg = None
        self._goal_handle = None
    
    def start(self):
        self._client.wait_for_server()
        with self._lock:
            self._state = ActionExecutionStatus.PENDING
        if self._feedback_callback is not None:
            send_goal_future = self._client.send_goal_async(self._goal_msg, feedback_callback=self._feedback_callback)
        else:
            send_goal_future = self._client.send_goal_async(self._goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)
    
    def cancel(self) -> bool:
        if self._goal_handle is None:
            return False
        
        with self._lock:
            if self._state in [ActionExecutionStatus.CANCELING, ActionExecutionStatus.FINISHED]:
                return True # Action already finished or canceling
            
            cancel_future = self._goal_handle.cancel_goal_async()
            # There are two possible outcomes of the cancel request:
            # 1. The cancel gets rejected, which means the action has to finish executing (e.g. takeoff)
            # 2. The cancel gets accepted, which means the action will be canceled
            # In both cases the action will be finishing as soon as possible
            
            self._state = ActionExecutionStatus.CANCELING
            return True
    
    @property
    def state(self) -> ActionExecutionStatus:
        with self._lock:
            return self._state
    
    @property
    def finish_type(self) -> ActionFinishType:
        with self._lock:
            return self._finish_type
    
    # --- Callbacks for the action client --- #
    
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            with self._lock:
                self._state = ActionExecutionStatus.FINISHED
                self._finish_type = ActionFinishType.GOAL_NOT_ACCEPTED
            return
        
        with self._lock:
            self._state = ActionExecutionStatus.EXECUTING
            self._goal_handle = goal_handle
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)
    
    def _get_result_callback(self, future):
        response = future.result()
        status = response.status
        result = response.result
        
        with self._lock:
            self._result_msg = result
            match status:
                case 4:  # STATUS_SUCCEEDED
                    self._state = ActionExecutionStatus.FINISHED
                    self._finish_type = ActionFinishType.SUCCESS
                case 5:  # STATUS_CANCELED
                    self._state = ActionExecutionStatus.FINISHED
                    self._finish_type = ActionFinishType.CANCELED
                case _:
                    self._state = ActionExecutionStatus.FINISHED
                    self._finish_type = ActionFinishType.FAILED
    