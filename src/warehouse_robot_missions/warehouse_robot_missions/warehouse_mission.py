#!/usr/bin/env python3

import os
import sys
import time
import yaml
import threading

from enum import Enum
from datetime import datetime

import rclpy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Empty

from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult
)


POINTS_FILE = os.path.expanduser(
    '~/warehouse_ws/mission_points.yaml'
)

MISSIONS_DIR = os.path.expanduser(
    '~/warehouse_ws/missions'
)

LOG_DIR = os.path.expanduser(
    '~/warehouse_ws/mission_logs'
)


# ============================================================
# MISSION STATES
# ============================================================

class MissionState(Enum):
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    PAUSED = 'PAUSED'
    AT_STATION = 'AT_STATION'
    COMPLETED = 'COMPLETED'
    CANCELED = 'CANCELED'
    FAILED = 'FAILED'


# ============================================================
# YAML
# ============================================================

def load_yaml(path):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f'File not found: {path}'
        )

    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    if not data:
        raise RuntimeError(
            f'YAML file is empty: {path}'
        )

    return data


def get_mission_file():
    if len(sys.argv) < 2:
        print()
        print('ERROR: No mission selected.')
        print()
        print('Usage:')
        print(
            'ros2 run warehouse_robot_missions '
            'warehouse_mission <mission>'
        )
        print()

        if os.path.exists(MISSIONS_DIR):
            missions = sorted([
                f[:-5]
                for f in os.listdir(MISSIONS_DIR)
                if f.endswith('.yaml')
            ])

            if missions:
                print('Available missions:')

                for mission in missions:
                    print(f'  - {mission}')

                print()

        return None

    mission_name = sys.argv[1]

    if mission_name.endswith('.yaml'):
        filename = mission_name
    else:
        filename = mission_name + '.yaml'

    return os.path.join(
        MISSIONS_DIR,
        filename
    )


# ============================================================
# LOGGER
# ============================================================

class MissionLogger:

    def __init__(self, mission_name):
        os.makedirs(
            LOG_DIR,
            exist_ok=True
        )

        timestamp = datetime.now().strftime(
            '%Y%m%d_%H%M%S'
        )

        self.log_file = os.path.join(
            LOG_DIR,
            f'{mission_name}_{timestamp}.log'
        )

    def log(self, message):
        now = datetime.now().strftime(
            '%Y-%m-%d %H:%M:%S'
        )

        line = f'[{now}] {message}'

        print(line)

        with open(
            self.log_file,
            'a'
        ) as f:
            f.write(line + '\n')


# ============================================================
# STATUS PUBLISHER
# ============================================================

class MissionStatusPublisher:

    def __init__(self, navigator):
        self.publisher = navigator.create_publisher(
            String,
            '/mission_status',
            10
        )

    def publish(
        self,
        mission_name,
        state,
        station,
        next_station,
        current_index,
        total_stations,
        current_loop,
        total_loops
    ):
        msg = String()

        mission_text = (
            mission_name
            if mission_name is not None
            else 'NONE'
        )

        station_text = (
            station
            if station is not None
            else 'NONE'
        )

        next_station_text = (
            next_station
            if next_station is not None
            else 'NONE'
        )

        if total_stations > 0:
            percent = int(
                round(
                    (
                        current_index
                        / total_stations
                    )
                    * 100
                )
            )
        else:
            percent = 0

        percent = max(
            0,
            min(100, percent)
        )

        msg.data = (
            f'mission={mission_text} '
            f'state={state} '
            f'station={station_text} '
            f'next={next_station_text} '
            f'progress={current_index}/{total_stations} '
            f'percent={percent} '
            f'loop={current_loop}/{total_loops}'
        )

        self.publisher.publish(msg)


# ============================================================
# STATE MANAGER
# ============================================================

class MissionStateManager:

    def __init__(
        self,
        logger,
        status_publisher,
        mission_name
    ):
        self.logger = logger
        self.status_publisher = status_publisher

        self.lock = threading.Lock()

        self.mission_name = mission_name

        self.state = MissionState.IDLE

        self.current_station = None
        self.next_station = None

        self.current_index = 0
        self.total_stations = 0

        self.current_loop = 0
        self.total_loops = 0

        self.publish_status()

    def publish_status(self):
        with self.lock:
            mission_name = self.mission_name
            state = self.state.value

            station = self.current_station
            next_station = self.next_station

            current_index = self.current_index
            total_stations = self.total_stations

            current_loop = self.current_loop
            total_loops = self.total_loops

        self.status_publisher.publish(
            mission_name=mission_name,
            state=state,
            station=station,
            next_station=next_station,
            current_index=current_index,
            total_stations=total_stations,
            current_loop=current_loop,
            total_loops=total_loops
        )

    def set_state(self, new_state):
        with self.lock:
            old_state = self.state

            if old_state == new_state:
                return

            self.state = new_state

        self.logger.log(
            f'STATE: '
            f'{old_state.value} -> '
            f'{new_state.value}'
        )

        self.publish_status()

    def get_state(self):
        with self.lock:
            return self.state

    def set_station(self, station_name):
        with self.lock:
            self.current_station = station_name

        self.publish_status()

    def set_loop(
        self,
        current_loop,
        total_loops
    ):
        with self.lock:
            self.current_loop = current_loop
            self.total_loops = total_loops

        self.publish_status()

    def set_progress(
        self,
        current_index,
        total_stations,
        current_station,
        next_station
    ):
        with self.lock:
            self.current_index = current_index
            self.total_stations = total_stations
            self.current_station = current_station
            self.next_station = next_station

        self.publish_status()

    def set_completed(self):
        with self.lock:
            self.current_index = self.total_stations
            self.next_station = None

        self.publish_status()

    def get_status(self):
        with self.lock:
            return {
                'mission': self.mission_name,
                'state': self.state.value,
                'station': self.current_station,
                'next_station': self.next_station,
                'progress': self.current_index,
                'total_stations': self.total_stations,
                'loop': self.current_loop,
                'total_loops': self.total_loops
            }


# ============================================================
# MISSION CONTROL
# ============================================================

class MissionControl:

    def __init__(
        self,
        navigator,
        logger,
        state_manager
    ):
        self.navigator = navigator
        self.logger = logger
        self.state_manager = state_manager

        self.paused = False
        self.cancel_requested = False

        self.lock = threading.Lock()

        self.pause_subscription = (
            navigator.create_subscription(
                Empty,
                '/mission_pause',
                self.pause_callback,
                10
            )
        )

        self.resume_subscription = (
            navigator.create_subscription(
                Empty,
                '/mission_resume',
                self.resume_callback,
                10
            )
        )

        self.cancel_subscription = (
            navigator.create_subscription(
                Empty,
                '/mission_cancel',
                self.cancel_callback,
                10
            )
        )

        self.thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )

        self.thread.start()

    def request_pause(self, source='UNKNOWN'):
        with self.lock:
            if self.cancel_requested:
                return

            if self.paused:
                return

            self.paused = True

        self.logger.log(
            f'PAUSE requested via {source}'
        )

    def request_resume(self, source='UNKNOWN'):
        with self.lock:
            if self.cancel_requested:
                return

            if not self.paused:
                return

            self.paused = False

        self.logger.log(
            f'RESUME requested via {source}'
        )

    def request_cancel(self, source='UNKNOWN'):
        with self.lock:
            if self.cancel_requested:
                return

            self.cancel_requested = True

        self.logger.log(
            f'CANCEL requested via {source}'
        )

    def pause_callback(self, msg):
        self.request_pause(
            source='ROS_TOPIC'
        )

    def resume_callback(self, msg):
        self.request_resume(
            source='ROS_TOPIC'
        )

    def cancel_callback(self, msg):
        self.request_cancel(
            source='ROS_TOPIC'
        )

    def keyboard_loop(self):
        while True:
            try:
                command = input().strip().lower()

            except EOFError:
                return

            if command == 'p':
                self.request_pause(
                    source='KEYBOARD'
                )

            elif command == 'r':
                self.request_resume(
                    source='KEYBOARD'
                )

            elif command == 'c':
                self.request_cancel(
                    source='KEYBOARD'
                )

            elif command == 'status':
                status = (
                    self.state_manager
                    .get_status()
                )

                print()
                print(
                    '========================================'
                )
                print(
                    'MISSION STATUS'
                )
                print(
                    '========================================'
                )

                print(
                    f'Mission: '
                    f'{status["mission"]}'
                )

                print(
                    f'State: '
                    f'{status["state"]}'
                )

                if status['station'] is not None:
                    print(
                        f'Current station: '
                        f'{status["station"]}'
                    )

                if status['next_station'] is not None:
                    print(
                        f'Next station: '
                        f'{status["next_station"]}'
                    )

                if status['total_stations'] > 0:
                    print(
                        f'Progress: '
                        f'{status["progress"]}/'
                        f'{status["total_stations"]}'
                    )

                if status['total_loops'] > 0:
                    print(
                        f'Loop: '
                        f'{status["loop"]}/'
                        f'{status["total_loops"]}'
                    )

                print(
                    f'Paused: '
                    f'{self.is_paused()}'
                )

                print(
                    f'Cancel requested: '
                    f'{self.is_cancel_requested()}'
                )

                print(
                    '========================================'
                )
                print()

    def is_paused(self):
        with self.lock:
            return self.paused

    def is_cancel_requested(self):
        with self.lock:
            return self.cancel_requested


# ============================================================
# ROS CALLBACK PROCESSING
# ============================================================

def process_ros_callbacks(navigator):
    rclpy.spin_once(
        navigator,
        timeout_sec=0.0
    )


# ============================================================
# POSE
# ============================================================

def create_pose(
    navigator,
    point
):
    pose = PoseStamped()

    pose.header.frame_id = 'map'

    pose.header.stamp = (
        navigator.get_clock()
        .now()
        .to_msg()
    )

    pose.pose.position.x = float(
        point['x']
    )

    pose.pose.position.y = float(
        point['y']
    )

    pose.pose.position.z = 0.0

    orientation = point['orientation']

    pose.pose.orientation.x = float(
        orientation['x']
    )

    pose.pose.orientation.y = float(
        orientation['y']
    )

    pose.pose.orientation.z = float(
        orientation['z']
    )

    pose.pose.orientation.w = float(
        orientation['w']
    )

    return pose


# ============================================================
# NAVIGATION
# ============================================================

def navigate_once(
    navigator,
    logger,
    control,
    state_manager,
    station_name,
    point
):
    pose = create_pose(
        navigator,
        point
    )

    logger.log(
        f'Navigating to {station_name} '
        f'(x={point["x"]:.3f}, '
        f'y={point["y"]:.3f}, '
        f'yaw={point["yaw_deg"]:.2f})'
    )

    state_manager.set_station(
        station_name
    )

    state_manager.set_state(
        MissionState.NAVIGATING
    )

    navigator.goToPose(
        pose
    )

    task_active = True

    while True:
        process_ros_callbacks(
            navigator
        )

        if control.is_cancel_requested():
            if task_active:
                logger.log(
                    'Canceling active Nav2 task'
                )

                navigator.cancelTask()

                while not navigator.isTaskComplete():
                    process_ros_callbacks(
                        navigator
                    )

                    time.sleep(0.1)

            state_manager.set_state(
                MissionState.CANCELED
            )

            return 'MISSION_CANCEL'

        if control.is_paused():
            if task_active:
                logger.log(
                    f'Pausing navigation to '
                    f'{station_name}'
                )

                navigator.cancelTask()

                while not navigator.isTaskComplete():
                    process_ros_callbacks(
                        navigator
                    )

                    time.sleep(0.1)

                task_active = False

                state_manager.set_state(
                    MissionState.PAUSED
                )

                logger.log(
                    'Robot paused'
                )

                print()
                print(
                    'MISSION PAUSED'
                )
                print(
                    'Resume with:'
                )
                print(
                    '  r + Enter'
                )
                print(
                    'or publish /mission_resume'
                )
                print()

            while control.is_paused():
                process_ros_callbacks(
                    navigator
                )

                if control.is_cancel_requested():
                    state_manager.set_state(
                        MissionState.CANCELED
                    )

                    return 'MISSION_CANCEL'

                time.sleep(0.1)

            if control.is_cancel_requested():
                state_manager.set_state(
                    MissionState.CANCELED
                )

                return 'MISSION_CANCEL'

            pose.header.stamp = (
                navigator.get_clock()
                .now()
                .to_msg()
            )

            logger.log(
                f'Resuming navigation to '
                f'{station_name}'
            )

            state_manager.set_state(
                MissionState.NAVIGATING
            )

            navigator.goToPose(
                pose
            )

            task_active = True

        if (
            task_active
            and navigator.isTaskComplete()
        ):
            return navigator.getResult()

        if task_active:
            feedback = (
                navigator.getFeedback()
            )

            if feedback is not None:
                try:
                    distance = (
                        feedback.distance_remaining
                    )

                    print(
                        f'\r{station_name}: '
                        f'{distance:.2f} m remaining',
                        end='',
                        flush=True
                    )

                except AttributeError:
                    pass

        time.sleep(0.2)


def navigate_with_retry(
    navigator,
    logger,
    control,
    state_manager,
    station_name,
    point,
    max_retries,
    retry_wait
):
    total_attempts = (
        max_retries + 1
    )

    for attempt in range(
        1,
        total_attempts + 1
    ):
        logger.log(
            f'{station_name}: '
            f'attempt {attempt}/'
            f'{total_attempts}'
        )

        result = navigate_once(
            navigator=navigator,
            logger=logger,
            control=control,
            state_manager=state_manager,
            station_name=station_name,
            point=point
        )

        print()

        if result == 'MISSION_CANCEL':
            return 'MISSION_CANCEL'

        if result == TaskResult.SUCCEEDED:
            state_manager.set_state(
                MissionState.AT_STATION
            )

            logger.log(
                f'{station_name}: '
                'reached successfully'
            )

            return True

        if result == TaskResult.CANCELED:
            logger.log(
                f'{station_name}: '
                'navigation canceled'
            )

            return False

        logger.log(
            f'{station_name}: '
            'navigation failed'
        )

        if attempt < total_attempts:
            logger.log(
                'Clearing costmaps '
                'before retry'
            )

            try:
                navigator.clearAllCostmaps()

                logger.log(
                    'Costmaps cleared successfully'
                )

            except Exception as e:
                logger.log(
                    f'WARNING: '
                    f'costmap clear failed: {e}'
                )

            logger.log(
                f'Waiting {retry_wait:.1f} s '
                'before retry'
            )

            retry_start = time.monotonic()

            while (
                time.monotonic()
                - retry_start
                < retry_wait
            ):
                process_ros_callbacks(
                    navigator
                )

                if control.is_cancel_requested():
                    state_manager.set_state(
                        MissionState.CANCELED
                    )

                    return 'MISSION_CANCEL'

                time.sleep(0.1)

    state_manager.set_state(
        MissionState.FAILED
    )

    logger.log(
        f'{station_name}: '
        'maximum retries reached'
    )

    return False


# ============================================================
# INTERRUPTIBLE WAIT
# ============================================================

def interruptible_wait(
    seconds,
    navigator,
    control,
    state_manager
):
    elapsed = 0.0
    interval = 0.1

    while elapsed < seconds:
        process_ros_callbacks(
            navigator
        )

        if control.is_cancel_requested():
            state_manager.set_state(
                MissionState.CANCELED
            )

            return False

        if control.is_paused():
            state_manager.set_state(
                MissionState.PAUSED
            )

            while control.is_paused():
                process_ros_callbacks(
                    navigator
                )

                if control.is_cancel_requested():
                    state_manager.set_state(
                        MissionState.CANCELED
                    )

                    return False

                time.sleep(interval)

            state_manager.set_state(
                MissionState.AT_STATION
            )

        time.sleep(interval)
        elapsed += interval

    return True


# ============================================================
# STATION ACTION
# ============================================================

def perform_station_action(
    navigator,
    logger,
    control,
    state_manager,
    station_name,
    action,
    wait_time
):
    state_manager.set_state(
        MissionState.AT_STATION
    )

    action = action.lower()

    logger.log(
        f'{station_name}: '
        f'executing action "{action}"'
    )

    if action == 'pickup':
        print(
            '>> PICKUP operation'
        )

    elif action == 'inspect':
        print(
            '>> INSPECTION operation'
        )

    elif action == 'dropoff':
        print(
            '>> DROP-OFF operation'
        )

    elif action == 'finish':
        print(
            '>> Robot at HOME'
        )

    elif action == 'wait':
        print(
            '>> WAIT operation'
        )

    elif action == 'none':
        print(
            '>> No station action'
        )

    else:
        logger.log(
            f'WARNING: Unknown action '
            f'"{action}"'
        )

    if wait_time > 0:
        logger.log(
            f'{station_name}: '
            f'waiting {wait_time:.1f} s'
        )

        if not interruptible_wait(
            seconds=wait_time,
            navigator=navigator,
            control=control,
            state_manager=state_manager
        ):
            return False

    logger.log(
        f'{station_name}: '
        f'action "{action}" completed'
    )

    return True


# ============================================================
# SHUTDOWN
# ============================================================

def shutdown_navigator(
    navigator
):
    navigator.destroyNode()

    if rclpy.ok():
        rclpy.shutdown()


# ============================================================
# MAIN
# ============================================================

def main(args=None):
    mission_file = get_mission_file()

    if mission_file is None:
        return

    rclpy.init(args=args)

    navigator = BasicNavigator()

    try:
        points = load_yaml(
            POINTS_FILE
        )

        mission = load_yaml(
            mission_file
        )

    except Exception as e:
        print()
        print(
            f'[ERROR] {e}'
        )
        print()

        shutdown_navigator(
            navigator
        )

        return

    mission_name = mission.get(
        'mission_name',
        os.path.basename(
            mission_file
        )
    )

    logger = MissionLogger(
        mission_name
    )

    status_publisher = (
        MissionStatusPublisher(
            navigator
        )
    )

    state_manager = (
        MissionStateManager(
            logger=logger,
            status_publisher=status_publisher,
            mission_name=mission_name
        )
    )

    control = MissionControl(
        navigator=navigator,
        logger=logger,
        state_manager=state_manager
    )

    max_retries = int(
        mission.get(
            'max_retries',
            2
        )
    )

    retry_wait = float(
        mission.get(
            'wait_between_retries',
            2.0
        )
    )

    loops = int(
        mission.get(
            'loops',
            1
        )
    )

    stations = mission.get(
        'stations',
        []
    )

    if not stations:
        logger.log(
            'ERROR: mission contains '
            'no stations'
        )

        state_manager.set_state(
            MissionState.FAILED
        )

        shutdown_navigator(
            navigator
        )

        return

    if loops < 1:
        logger.log(
            'ERROR: loops must be >= 1'
        )

        state_manager.set_state(
            MissionState.FAILED
        )

        shutdown_navigator(
            navigator
        )

        return

    missing_points = []

    for station in stations:
        station_name = (
            station['name']
            .upper()
        )

        if station_name not in points:
            missing_points.append(
                station_name
            )

    if missing_points:
        logger.log(
            'ERROR: missing mission points: '
            + ', '.join(missing_points)
        )

        state_manager.set_state(
            MissionState.FAILED
        )

        shutdown_navigator(
            navigator
        )

        return

    state_manager.set_loop(
        0,
        loops
    )

    state_manager.set_progress(
        current_index=0,
        total_stations=len(stations),
        current_station=None,
        next_station=stations[0]['name'].upper()
    )

    print()
    print(
        '========================================'
    )
    print(
        '       WAREHOUSE MISSION MANAGER'
    )
    print(
        '========================================'
    )
    print()

    logger.log(
        f'Mission "{mission_name}" loaded'
    )

    print()
    print(
        f'Loops: {loops}'
    )

    print()
    print('Route:')

    for station in stations:
        print(
            f'  -> {station["name"]} '
            f'[{station.get("action", "none")}]'
        )

    print()
    print('Keyboard controls:')
    print(
        '  p + Enter  = Pause'
    )
    print(
        '  r + Enter  = Resume'
    )
    print(
        '  c + Enter  = Cancel'
    )
    print(
        '  status     = Mission status'
    )

    print()
    print('ROS 2 control topics:')
    print(
        '  /mission_pause'
    )
    print(
        '  /mission_resume'
    )
    print(
        '  /mission_cancel'
    )

    print()
    print('Waiting for Nav2...')

    navigator.waitUntilNav2Active()

    logger.log(
        'Nav2 active'
    )

    mission_start = (
        time.monotonic()
    )

    logger.log(
        'Mission started'
    )

    for loop_number in range(
        1,
        loops + 1
    ):
        state_manager.set_loop(
            loop_number,
            loops
        )

        logger.log(
            f'LOOP {loop_number}/{loops} '
            'started'
        )

        for index, station in enumerate(
            stations,
            start=1
        ):
            process_ros_callbacks(
                navigator
            )

            if control.is_cancel_requested():
                state_manager.set_state(
                    MissionState.CANCELED
                )

                logger.log(
                    'MISSION CANCELED BY USER'
                )

                shutdown_navigator(
                    navigator
                )

                return

            station_name = (
                station['name']
                .upper()
            )

            if index < len(stations):
                next_station = (
                    stations[index]['name']
                    .upper()
                )

            elif loop_number < loops:
                next_station = (
                    stations[0]['name']
                    .upper()
                )

            else:
                next_station = None

            state_manager.set_progress(
                current_index=index,
                total_stations=len(stations),
                current_station=station_name,
                next_station=next_station
            )

            action = station.get(
                'action',
                'none'
            )

            wait_after_arrival = float(
                station.get(
                    'wait_after_arrival',
                    0.0
                )
            )

            logger.log(
                f'Loop {loop_number}/{loops} - '
                f'Station {index}/'
                f'{len(stations)}: '
                f'{station_name}'
            )

            success = navigate_with_retry(
                navigator=navigator,
                logger=logger,
                control=control,
                state_manager=state_manager,
                station_name=station_name,
                point=points[station_name],
                max_retries=max_retries,
                retry_wait=retry_wait
            )

            if success == 'MISSION_CANCEL':
                state_manager.set_state(
                    MissionState.CANCELED
                )

                logger.log(
                    'MISSION CANCELED BY USER'
                )

                shutdown_navigator(
                    navigator
                )

                return

            if not success:
                state_manager.set_state(
                    MissionState.FAILED
                )

                logger.log(
                    f'MISSION ABORTED '
                    f'at {station_name}'
                )

                shutdown_navigator(
                    navigator
                )

                return

            action_success = (
                perform_station_action(
                    navigator=navigator,
                    logger=logger,
                    control=control,
                    state_manager=state_manager,
                    station_name=station_name,
                    action=action,
                    wait_time=wait_after_arrival
                )
            )

            if not action_success:
                state_manager.set_state(
                    MissionState.CANCELED
                )

                logger.log(
                    'MISSION CANCELED BY USER'
                )

                shutdown_navigator(
                    navigator
                )

                return

        logger.log(
            f'LOOP {loop_number}/{loops} '
            'completed'
        )

    mission_time = (
        time.monotonic()
        - mission_start
    )

    state_manager.set_completed()

    state_manager.set_state(
        MissionState.COMPLETED
    )

    logger.log(
        f'MISSION COMPLETED SUCCESSFULLY '
        f'in {mission_time:.1f} seconds'
    )

    print()
    print(
        '========================================'
    )
    print(
        'MISSION COMPLETED SUCCESSFULLY'
    )
    print(
        f'Loops completed: {loops}'
    )
    print(
        f'Total mission time: '
        f'{mission_time:.1f} seconds'
    )
    print(
        f'Final state: '
        f'{state_manager.get_state().value}'
    )
    print(
        f'Log file: '
        f'{logger.log_file}'
    )
    print(
        '========================================'
    )

    shutdown_navigator(
        navigator
    )


if __name__ == '__main__':
    main()
