#!/usr/bin/env python3

import os
import subprocess
import time
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty
from lifecycle_msgs.srv import GetState


MISSIONS_DIR = os.path.expanduser(
    '~/warehouse_ws/missions'
)


class MissionDashboard(Node):

    def __init__(self):
        super().__init__('mission_dashboard')

        # ====================================================
        # ROS 2 - MISSION
        # ====================================================

        self.status_subscription = self.create_subscription(
            String,
            '/mission_status',
            self.status_callback,
            10
        )

        self.pause_publisher = self.create_publisher(
            Empty,
            '/mission_pause',
            10
        )

        self.resume_publisher = self.create_publisher(
            Empty,
            '/mission_resume',
            10
        )

        self.cancel_publisher = self.create_publisher(
            Empty,
            '/mission_cancel',
            10
        )

        # ====================================================
        # NAV2 HEALTH
        # ====================================================

        self.nav2_nodes = [
            'controller_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'amcl',
            'map_server'
        ]

        self.lifecycle_clients = {}
        self.lifecycle_futures = {}

        self.lifecycle_states = {
            node_name: 'UNKNOWN'
            for node_name in self.nav2_nodes
        }

        for node_name in self.nav2_nodes:
            service_name = (
                f'/{node_name}/get_state'
            )

            self.lifecycle_clients[
                node_name
            ] = self.create_client(
                GetState,
                service_name
            )

        # ====================================================
        # MISSION PROCESS
        # ====================================================

        self.mission_process = None
        self.cancel_sent_time = None

        # ====================================================
        # CURRENT STATE
        # ====================================================

        self.current_mission = 'NONE'
        self.current_state = 'IDLE'
        self.current_station = 'NONE'
        self.next_station = 'NONE'
        self.current_progress = '0/0'
        self.current_percent = 0
        self.current_loop = '0/0'

        self.mission_start_time = None

        # ====================================================
        # WINDOW
        # ====================================================

        self.root = tk.Tk()

        self.root.title(
            'Warehouse Robot Mission Control'
        )

        self.root.geometry(
            '760x860'
        )

        self.root.resizable(
            False,
            False
        )

        self.root.protocol(
            'WM_DELETE_WINDOW',
            self.close_window
        )

        # ====================================================
        # VARIABLES
        # ====================================================

        self.mission_var = tk.StringVar(
            value='NONE'
        )

        self.state_var = tk.StringVar(
            value='IDLE'
        )

        self.station_var = tk.StringVar(
            value='NONE'
        )

        self.next_station_var = tk.StringVar(
            value='NONE'
        )

        self.progress_var = tk.StringVar(
            value='0/0'
        )

        self.percent_var = tk.StringVar(
            value='0%'
        )

        self.loop_var = tk.StringVar(
            value='0/0'
        )

        self.elapsed_var = tk.StringVar(
            value='00:00'
        )

        self.nav2_var = tk.StringVar(
            value='CHECKING...'
        )

        self.selected_mission = tk.StringVar()

        # ====================================================
        # TITLE
        # ====================================================

        title = ttk.Label(
            self.root,
            text='WAREHOUSE ROBOT',
            font=('Arial', 24, 'bold')
        )

        title.pack(
            pady=(18, 2)
        )

        subtitle = ttk.Label(
            self.root,
            text='Mission Control Dashboard',
            font=('Arial', 12)
        )

        subtitle.pack(
            pady=(0, 12)
        )

        # ====================================================
        # SYSTEM STATUS
        # ====================================================

        system_frame = ttk.LabelFrame(
            self.root,
            text='System Status'
        )

        system_frame.pack(
            fill='x',
            padx=25,
            pady=6
        )

        ttk.Label(
            system_frame,
            text='Nav2:',
            font=('Arial', 11, 'bold')
        ).grid(
            row=0,
            column=0,
            sticky='w',
            padx=15,
            pady=10
        )

        self.nav2_label = tk.Label(
            system_frame,
            textvariable=self.nav2_var,
            font=('Arial', 11, 'bold')
        )

        self.nav2_label.grid(
            row=0,
            column=1,
            sticky='w',
            padx=15,
            pady=10
        )

        # ====================================================
        # MISSION SELECTOR
        # ====================================================

        mission_frame = ttk.LabelFrame(
            self.root,
            text='Mission Selection'
        )

        mission_frame.pack(
            fill='x',
            padx=25,
            pady=6
        )

        self.mission_combo = ttk.Combobox(
            mission_frame,
            textvariable=self.selected_mission,
            state='readonly',
            font=('Arial', 11)
        )

        self.mission_combo.pack(
            fill='x',
            padx=15,
            pady=12
        )

        self.load_missions()

        # ====================================================
        # START
        # ====================================================

        self.start_button = ttk.Button(
            self.root,
            text='START MISSION',
            command=self.start_mission
        )

        self.start_button.pack(
            fill='x',
            padx=25,
            pady=7,
            ipady=7
        )

        # ====================================================
        # CONTROLS
        # ====================================================

        control_frame = ttk.LabelFrame(
            self.root,
            text='Mission Controls'
        )

        control_frame.pack(
            fill='x',
            padx=25,
            pady=6
        )

        button_frame = ttk.Frame(
            control_frame
        )

        button_frame.pack(
            fill='x',
            padx=10,
            pady=12
        )

        self.pause_button = ttk.Button(
            button_frame,
            text='PAUSE',
            command=self.pause_mission
        )

        self.pause_button.pack(
            side='left',
            expand=True,
            fill='x',
            padx=5
        )

        self.resume_button = ttk.Button(
            button_frame,
            text='RESUME',
            command=self.resume_mission
        )

        self.resume_button.pack(
            side='left',
            expand=True,
            fill='x',
            padx=5
        )

        self.cancel_button = ttk.Button(
            button_frame,
            text='CANCEL',
            command=self.cancel_mission
        )

        self.cancel_button.pack(
            side='left',
            expand=True,
            fill='x',
            padx=5
        )

        # ====================================================
        # LIVE STATUS
        # ====================================================

        status_frame = ttk.LabelFrame(
            self.root,
            text='Live Mission Status'
        )

        status_frame.pack(
            fill='x',
            padx=25,
            pady=6
        )

        self.create_status_row(
            status_frame,
            'Mission:',
            self.mission_var,
            0
        )

        self.create_status_row(
            status_frame,
            'State:',
            self.state_var,
            1
        )

        self.create_status_row(
            status_frame,
            'Current Station:',
            self.station_var,
            2
        )

        self.create_status_row(
            status_frame,
            'Next Station:',
            self.next_station_var,
            3
        )

        self.create_status_row(
            status_frame,
            'Progress:',
            self.progress_var,
            4
        )

        self.create_status_row(
            status_frame,
            'Completion:',
            self.percent_var,
            5
        )

        self.create_status_row(
            status_frame,
            'Loop:',
            self.loop_var,
            6
        )

        self.create_status_row(
            status_frame,
            'Elapsed:',
            self.elapsed_var,
            7
        )

        # ====================================================
        # PROGRESS BAR
        # ====================================================

        progress_frame = ttk.Frame(
            status_frame
        )

        progress_frame.grid(
            row=8,
            column=0,
            columnspan=2,
            sticky='ew',
            padx=20,
            pady=(5, 15)
        )

        status_frame.columnconfigure(
            1,
            weight=1
        )

        self.progress_bar = ttk.Progressbar(
            progress_frame,
            orient='horizontal',
            mode='determinate',
            maximum=100,
            value=0
        )

        self.progress_bar.pack(
            fill='x'
        )

        # ====================================================
        # EVENT LOG
        # ====================================================

        log_frame = ttk.LabelFrame(
            self.root,
            text='Event Log'
        )

        log_frame.pack(
            fill='both',
            expand=True,
            padx=25,
            pady=(6, 18)
        )

        self.log_text = tk.Text(
            log_frame,
            height=10,
            state='disabled',
            wrap='word',
            font=('Consolas', 10)
        )

        self.log_text.pack(
            fill='both',
            expand=True,
            padx=10,
            pady=10
        )

        # ====================================================
        # INITIAL STATE
        # ====================================================

        self.update_buttons()

        self.add_log(
            'Dashboard started'
        )

        # ====================================================
        # PERIODIC TASKS
        # ====================================================

        self.root.after(
            50,
            self.update_ros
        )

        self.root.after(
            300,
            self.check_mission_process
        )

        self.root.after(
            500,
            self.update_elapsed_time
        )

        self.root.after(
            500,
            self.request_nav2_health
        )

    # ========================================================
    # GUI HELPERS
    # ========================================================

    def create_status_row(
        self,
        parent,
        label_text,
        variable,
        row
    ):
        ttk.Label(
            parent,
            text=label_text,
            font=('Arial', 11, 'bold')
        ).grid(
            row=row,
            column=0,
            sticky='w',
            padx=20,
            pady=5
        )

        ttk.Label(
            parent,
            textvariable=variable,
            font=('Arial', 11)
        ).grid(
            row=row,
            column=1,
            sticky='w',
            padx=20,
            pady=5
        )

    def add_log(
        self,
        message
    ):
        now = time.strftime(
            '%H:%M:%S'
        )

        self.log_text.configure(
            state='normal'
        )

        self.log_text.insert(
            'end',
            f'[{now}] {message}\n'
        )

        self.log_text.see(
            'end'
        )

        self.log_text.configure(
            state='disabled'
        )

    # ========================================================
    # MISSIONS
    # ========================================================

    def load_missions(self):
        if not os.path.exists(
            MISSIONS_DIR
        ):
            return

        missions = sorted([
            filename[:-5]
            for filename in os.listdir(
                MISSIONS_DIR
            )
            if filename.endswith('.yaml')
        ])

        self.mission_combo[
            'values'
        ] = missions

        if missions:
            self.selected_mission.set(
                missions[0]
            )

    # ========================================================
    # START
    # ========================================================

    def start_mission(self):
        self.cleanup_finished_process()

        if self.mission_process is not None:
            messagebox.showwarning(
                'Mission',
                'Previous mission process is '
                'still shutting down.'
            )

            return

        mission = (
            self.selected_mission
            .get()
            .strip()
        )

        if not mission:
            messagebox.showwarning(
                'Mission',
                'Select a mission first.'
            )

            return

        if not self.nav2_is_healthy():
            messagebox.showwarning(
                'Nav2 Not Ready',
                'Nav2 is not fully ACTIVE.'
            )

            return

        command = [
            'ros2',
            'run',
            'warehouse_robot_missions',
            'warehouse_mission',
            mission
        ]

        try:
            self.current_mission = mission
            self.current_state = 'IDLE'
            self.current_station = 'NONE'
            self.next_station = 'NONE'
            self.current_progress = '0/0'
            self.current_percent = 0
            self.current_loop = '0/0'

            self.mission_var.set(
                mission
            )

            self.state_var.set(
                'IDLE'
            )

            self.station_var.set(
                'NONE'
            )

            self.next_station_var.set(
                'NONE'
            )

            self.progress_var.set(
                '0/0'
            )

            self.percent_var.set(
                '0%'
            )

            self.progress_bar[
                'value'
            ] = 0

            self.loop_var.set(
                '0/0'
            )

            self.mission_process = (
                subprocess.Popen(
                    command,
                    stdin=subprocess.DEVNULL
                )
            )

            self.mission_start_time = (
                time.monotonic()
            )

            self.cancel_sent_time = None

            self.elapsed_var.set(
                '00:00'
            )

            self.add_log(
                f'Started mission: {mission}'
            )

            self.update_buttons()

        except Exception as error:
            messagebox.showerror(
                'Mission Start Error',
                str(error)
            )

            self.add_log(
                f'ERROR starting mission: {error}'
            )

    # ========================================================
    # CONTROLS
    # ========================================================

    def pause_mission(self):
        self.pause_publisher.publish(
            Empty()
        )

        self.add_log(
            'Pause command sent'
        )

    def resume_mission(self):
        self.resume_publisher.publish(
            Empty()
        )

        self.add_log(
            'Resume command sent'
        )

    def cancel_mission(self):
        if self.mission_process is None:
            return

        self.cancel_publisher.publish(
            Empty()
        )

        self.cancel_sent_time = (
            time.monotonic()
        )

        self.add_log(
            'Cancel command sent'
        )

        self.cancel_button.configure(
            state='disabled'
        )

    # ========================================================
    # MISSION STATUS
    # ========================================================

    def status_callback(
        self,
        msg
    ):
        values = {}

        for item in msg.data.split():
            if '=' not in item:
                continue

            key, value = item.split(
                '=',
                1
            )

            values[key] = value

        new_mission = values.get(
            'mission',
            'NONE'
        )

        new_state = values.get(
            'state',
            'UNKNOWN'
        )

        new_station = values.get(
            'station',
            'NONE'
        )

        new_next_station = values.get(
            'next',
            'NONE'
        )

        new_progress = values.get(
            'progress',
            '0/0'
        )

        new_loop = values.get(
            'loop',
            '0/0'
        )

        try:
            new_percent = int(
                values.get(
                    'percent',
                    '0'
                )
            )

        except ValueError:
            new_percent = 0

        new_percent = max(
            0,
            min(100, new_percent)
        )

        if (
            new_mission != self.current_mission
            and new_mission != 'NONE'
        ):
            self.add_log(
                f'Mission: {new_mission}'
            )

        if new_state != self.current_state:
            self.add_log(
                f'State: '
                f'{self.current_state} '
                f'-> {new_state}'
            )

        if new_station != self.current_station:
            self.add_log(
                f'Current station: '
                f'{new_station}'
            )

        if (
            new_next_station
            != self.next_station
        ):
            self.add_log(
                f'Next station: '
                f'{new_next_station}'
            )

        self.current_mission = new_mission
        self.current_state = new_state
        self.current_station = new_station
        self.next_station = new_next_station
        self.current_progress = new_progress
        self.current_percent = new_percent
        self.current_loop = new_loop

        self.mission_var.set(
            new_mission
        )

        self.state_var.set(
            new_state
        )

        self.station_var.set(
            new_station
        )

        self.next_station_var.set(
            new_next_station
        )

        self.progress_var.set(
            new_progress
        )

        self.percent_var.set(
            f'{new_percent}%'
        )

        self.progress_bar[
            'value'
        ] = new_percent

        self.loop_var.set(
            new_loop
        )

        if new_state == 'COMPLETED':
            self.add_log(
                'Mission completed successfully'
            )

        elif new_state == 'FAILED':
            self.add_log(
                'Mission failed'
            )

        elif new_state == 'CANCELED':
            self.add_log(
                'Mission canceled'
            )

        self.update_buttons()

    # ========================================================
    # PROCESS CLEANUP
    # ========================================================

    def cleanup_finished_process(self):
        if self.mission_process is None:
            return

        return_code = (
            self.mission_process.poll()
        )

        if return_code is None:
            return

        self.add_log(
            'Mission process finished '
            f'with code {return_code}'
        )

        self.mission_process = None
        self.cancel_sent_time = None
        self.mission_start_time = None

        self.update_buttons()

    def check_mission_process(self):
        self.cleanup_finished_process()

        if (
            self.mission_process is not None
            and self.cancel_sent_time is not None
        ):
            elapsed = (
                time.monotonic()
                - self.cancel_sent_time
            )

            if (
                elapsed > 3.0
                and self.mission_process.poll()
                is None
            ):
                self.add_log(
                    'Mission did not exit after '
                    'cancel - terminating process'
                )

                self.mission_process.terminate()

                self.cancel_sent_time = None

        self.root.after(
            300,
            self.check_mission_process
        )

    # ========================================================
    # BUTTON LOGIC
    # ========================================================

    def update_buttons(self):
        mission_running = (
            self.mission_process is not None
            and self.mission_process.poll()
            is None
        )

        if not mission_running:
            if self.nav2_is_healthy():
                self.start_button.configure(
                    state='normal'
                )
            else:
                self.start_button.configure(
                    state='disabled'
                )

            self.mission_combo.configure(
                state='readonly'
            )

            self.pause_button.configure(
                state='disabled'
            )

            self.resume_button.configure(
                state='disabled'
            )

            self.cancel_button.configure(
                state='disabled'
            )

            return

        self.start_button.configure(
            state='disabled'
        )

        self.mission_combo.configure(
            state='disabled'
        )

        if self.current_state == 'NAVIGATING':
            self.pause_button.configure(
                state='normal'
            )

            self.resume_button.configure(
                state='disabled'
            )

            self.cancel_button.configure(
                state='normal'
            )

        elif self.current_state == 'PAUSED':
            self.pause_button.configure(
                state='disabled'
            )

            self.resume_button.configure(
                state='normal'
            )

            self.cancel_button.configure(
                state='normal'
            )

        elif self.current_state == 'AT_STATION':
            self.pause_button.configure(
                state='normal'
            )

            self.resume_button.configure(
                state='disabled'
            )

            self.cancel_button.configure(
                state='normal'
            )

        else:
            self.pause_button.configure(
                state='disabled'
            )

            self.resume_button.configure(
                state='disabled'
            )

            self.cancel_button.configure(
                state='normal'
            )

    # ========================================================
    # ELAPSED
    # ========================================================

    def update_elapsed_time(self):
        if (
            self.mission_start_time is not None
            and self.mission_process is not None
            and self.mission_process.poll()
            is None
        ):
            elapsed = int(
                time.monotonic()
                - self.mission_start_time
            )

            minutes = (
                elapsed // 60
            )

            seconds = (
                elapsed % 60
            )

            self.elapsed_var.set(
                f'{minutes:02d}:'
                f'{seconds:02d}'
            )

        self.root.after(
            500,
            self.update_elapsed_time
        )

    # ========================================================
    # NAV2 HEALTH
    # ========================================================

    def request_nav2_health(self):
        for node_name, client in (
            self.lifecycle_clients.items()
        ):
            old_future = (
                self.lifecycle_futures.get(
                    node_name
                )
            )

            if (
                old_future is not None
                and not old_future.done()
            ):
                continue

            if not client.service_is_ready():
                self.lifecycle_states[
                    node_name
                ] = 'UNAVAILABLE'

                continue

            request = GetState.Request()

            future = client.call_async(
                request
            )

            self.lifecycle_futures[
                node_name
            ] = future

            future.add_done_callback(
                lambda future,
                node=node_name:
                self.lifecycle_result(
                    node,
                    future
                )
            )

        self.update_nav2_indicator()

        self.root.after(
            2000,
            self.request_nav2_health
        )

    def lifecycle_result(
        self,
        node_name,
        future
    ):
        try:
            response = future.result()

            state = (
                response
                .current_state
                .label
                .upper()
            )

            self.lifecycle_states[
                node_name
            ] = state

        except Exception:
            self.lifecycle_states[
                node_name
            ] = 'ERROR'

        self.update_nav2_indicator()

    def nav2_is_healthy(self):
        return all(
            state == 'ACTIVE'
            for state
            in self.lifecycle_states.values()
        )

    def update_nav2_indicator(self):
        states = list(
            self.lifecycle_states.values()
        )

        if self.nav2_is_healthy():
            self.nav2_var.set(
                'ONLINE - ALL CORE NODES ACTIVE'
            )

            self.nav2_label.configure(
                fg='green'
            )

        elif all(
            state in (
                'UNKNOWN',
                'UNAVAILABLE'
            )
            for state in states
        ):
            self.nav2_var.set(
                'OFFLINE / NOT READY'
            )

            self.nav2_label.configure(
                fg='red'
            )

        else:
            not_active = [
                f'{node}={state}'
                for node, state
                in self.lifecycle_states.items()
                if state != 'ACTIVE'
            ]

            self.nav2_var.set(
                'NOT READY: '
                + ', '.join(not_active)
            )

            self.nav2_label.configure(
                fg='orange'
            )

        self.update_buttons()

    # ========================================================
    # ROS
    # ========================================================

    def update_ros(self):
        if not rclpy.ok():
            return

        rclpy.spin_once(
            self,
            timeout_sec=0.0
        )

        self.root.after(
            50,
            self.update_ros
        )

    # ========================================================
    # CLOSE
    # ========================================================

    def close_window(self):
        if (
            self.mission_process is not None
            and self.mission_process.poll()
            is None
        ):
            answer = messagebox.askyesno(
                'Mission Running',
                'A mission is still running.\n'
                'Cancel it and close dashboard?'
            )

            if not answer:
                return

            self.cancel_publisher.publish(
                Empty()
            )

            time.sleep(
                0.3
            )

            if (
                self.mission_process.poll()
                is None
            ):
                self.mission_process.terminate()

        self.root.destroy()

    # ========================================================
    # RUN
    # ========================================================

    def run(self):
        self.root.mainloop()


def main(args=None):

    rclpy.init(
        args=args
    )

    dashboard = MissionDashboard()

    try:
        dashboard.run()

    except KeyboardInterrupt:
        pass

    finally:
        dashboard.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
