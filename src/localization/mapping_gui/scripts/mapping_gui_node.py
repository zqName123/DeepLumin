#!/usr/bin/env python3
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

import rospy

from deeplumin_msgs.msg import MappingEnvironmentStatus, MappingRecordStatus, MappingStatus
from deeplumin_msgs.srv import (
    CheckMappingEnvironment,
    CheckMappingEnvironmentRequest,
    GetMappingStatus,
    GetMappingStatusRequest,
    StartBagRecording,
    StartBagRecordingRequest,
    StartMapping,
    StartMappingRequest,
    StopBagRecording,
    StopBagRecordingRequest,
    StopMapping,
    StopMappingRequest,
)


class MappingGuiNode:
    def __init__(self):
        rospy.init_node("mapping_gui", anonymous=False)

        self.status_topic = rospy.get_param("~status_topic", "/mapping/status")
        self.record_status_topic = rospy.get_param("~record_status_topic", "/mapping/record_status")
        self.environment_topic = rospy.get_param("~environment_topic", "/mapping/environment_last_result")

        self.check_env_service = rospy.get_param("~check_environment_service", "/mapping/check_environment")
        self.get_status_service = rospy.get_param("~get_status_service", "/mapping/get_status")
        self.start_mapping_service = rospy.get_param("~start_mapping_service", "/mapping/start")
        self.stop_mapping_service = rospy.get_param("~stop_mapping_service", "/mapping/stop")
        self.start_record_service = rospy.get_param("~start_record_service", "/mapping/start_bag_recording")
        self.stop_record_service = rospy.get_param("~stop_record_service", "/mapping/stop_bag_recording")

        self._lock = threading.Lock()
        self._status = MappingStatus()
        self._record_status = MappingRecordStatus()
        self._environment_status = MappingEnvironmentStatus()
        self._last_log_lines = []

        self.root = tk.Tk()
        self.root.title("DeepLumin 建图控制台")
        self.root.geometry("1180x820")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._build_vars()
        self._build_layout()
        self._wire_ros()
        self._refresh_ui()

        self.log("界面已初始化，正在等待 mapping_manager 接口。")

    def _build_vars(self):
        self.task_name_var = tk.StringVar(value="建图任务")
        self.config_path_var = tk.StringVar(value="")
        self.output_root_var = tk.StringVar(value="/tmp/deeplumin_mapping")
        self.enable_loop_var = tk.BooleanVar(value=True)
        self.enable_rviz_var = tk.BooleanVar(value=True)
        self.auto_record_var = tk.BooleanVar(value=False)

        self.bag_name_var = tk.StringVar(value="mapping")
        self.bag_output_dir_var = tk.StringVar(value="/tmp/deeplumin_mapping/bag")
        self.bag_topics_var = tk.StringVar(
            value="/localization/points,/ouster/imu,/mapping/status,/mapping/odom,/mapping/map_points,/mapping/loop_markers"
        )

        self.state_var = tk.StringVar(value="未知")
        self.task_id_var = tk.StringVar(value="-")
        self.task_output_dir_var = tk.StringVar(value="-")
        self.task_message_var = tk.StringVar(value="-")
        self.record_state_var = tk.StringVar(value="未知")
        self.record_bag_path_var = tk.StringVar(value="-")
        self.result_map_path_var = tk.StringVar(value="-")
        self.result_traj_path_var = tk.StringVar(value="-")
        self.result_keyframe_dir_var = tk.StringVar(value="-")
        self.result_keyframe_pose_path_var = tk.StringVar(value="-")

    def _build_layout(self):
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        left = ttk.Frame(self.root, padding=10)
        left.grid(row=0, column=0, sticky="ns")
        right = ttk.Frame(self.root, padding=10)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(2, weight=1)
        right.rowconfigure(3, weight=1)

        self._build_control_panel(left)
        self._build_status_panel(right)
        self._build_environment_panel(right)
        self._build_results_panel(right)
        self._build_log_panel(right)

    def _build_control_panel(self, parent):
        task_box = ttk.LabelFrame(parent, text="建图任务", padding=10)
        task_box.grid(row=0, column=0, sticky="ew", pady=(0, 10))

        self._add_labeled_entry(task_box, 0, "任务名称", self.task_name_var)
        self._add_labeled_entry(task_box, 1, "配置文件路径", self.config_path_var, width=36)
        self._add_labeled_entry(task_box, 2, "输出根目录", self.output_root_var, width=36)

        ttk.Checkbutton(task_box, text="启用回环检测", variable=self.enable_loop_var).grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(8, 0)
        )
        ttk.Checkbutton(task_box, text="由运行时启动 RViz", variable=self.enable_rviz_var).grid(
            row=4, column=0, columnspan=2, sticky="w"
        )
        ttk.Checkbutton(task_box, text="启动建图时自动开始录包", variable=self.auto_record_var).grid(
            row=5, column=0, columnspan=2, sticky="w"
        )

        action_box = ttk.LabelFrame(parent, text="操作区", padding=10)
        action_box.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        action_box.columnconfigure(0, weight=1)
        action_box.columnconfigure(1, weight=1)

        ttk.Button(action_box, text="检测环境", command=self.on_check_environment).grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=4)
        ttk.Button(action_box, text="刷新状态", command=self.on_refresh_status).grid(row=0, column=1, sticky="ew", padx=(4, 0), pady=4)
        ttk.Button(action_box, text="开始建图", command=self.on_start_mapping).grid(row=1, column=0, sticky="ew", padx=(0, 4), pady=4)
        ttk.Button(action_box, text="结束建图", command=self.on_stop_mapping).grid(row=1, column=1, sticky="ew", padx=(4, 0), pady=4)

        record_box = ttk.LabelFrame(parent, text="Bag 录制", padding=10)
        record_box.grid(row=2, column=0, sticky="ew")
        self._add_labeled_entry(record_box, 0, "Bag 名称", self.bag_name_var)
        self._add_labeled_entry(record_box, 1, "Bag 输出目录", self.bag_output_dir_var, width=36)
        self._add_labeled_entry(record_box, 2, "录制话题（逗号分隔）", self.bag_topics_var, width=36)
        ttk.Button(record_box, text="开始录包", command=self.on_start_recording).grid(row=3, column=0, sticky="ew", padx=(0, 4), pady=(8, 0))
        ttk.Button(record_box, text="停止录包", command=self.on_stop_recording).grid(row=3, column=1, sticky="ew", padx=(4, 0), pady=(8, 0))

    def _build_status_panel(self, parent):
        box = ttk.LabelFrame(parent, text="运行状态", padding=10)
        box.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        box.columnconfigure(1, weight=1)

        self._add_readonly_row(box, 0, "建图状态", self.state_var)
        self._add_readonly_row(box, 1, "任务 ID", self.task_id_var)
        self._add_readonly_row(box, 2, "输出目录", self.task_output_dir_var)
        self._add_readonly_row(box, 3, "状态说明", self.task_message_var)
        self._add_readonly_row(box, 4, "录包状态", self.record_state_var)
        self._add_readonly_row(box, 5, "Bag 路径", self.record_bag_path_var)

    def _build_environment_panel(self, parent):
        box = ttk.LabelFrame(parent, text="环境检测 / 接口约定", padding=10)
        box.grid(row=1, column=0, sticky="nsew", pady=(0, 10))
        box.columnconfigure(0, weight=1)
        box.rowconfigure(1, weight=1)

        contract = (
            "GUI 不负责点云三维显示，RViz 由建图运行时负责启动。\n"
            "控制面：使用 /mapping/* 下的服务接口。\n"
            "状态面：订阅 /mapping/status、/mapping/record_status、/mapping/environment_last_result。\n"
            "可视化面：由 RViz 订阅 /mapping/map_points、/mapping/current_scan、/mapping/path、/mapping/loop_markers。"
        )
        ttk.Label(box, text=contract, justify="left").grid(row=0, column=0, sticky="ew", pady=(0, 8))
        self.env_text = ScrolledText(box, height=12, wrap=tk.WORD)
        self.env_text.grid(row=1, column=0, sticky="nsew")
        self.env_text.configure(state=tk.DISABLED)

    def _build_results_panel(self, parent):
        box = ttk.LabelFrame(parent, text="最新结果路径", padding=10)
        box.grid(row=2, column=0, sticky="nsew", pady=(0, 10))
        box.columnconfigure(1, weight=1)

        self._add_readonly_row(box, 0, "地图 PCD", self.result_map_path_var)
        self._add_readonly_row(box, 1, "轨迹文件", self.result_traj_path_var)
        self._add_readonly_row(box, 2, "关键帧目录", self.result_keyframe_dir_var)
        self._add_readonly_row(box, 3, "关键帧位姿", self.result_keyframe_pose_path_var)

    def _build_log_panel(self, parent):
        box = ttk.LabelFrame(parent, text="事件日志", padding=10)
        box.grid(row=3, column=0, sticky="nsew")
        box.columnconfigure(0, weight=1)
        box.rowconfigure(0, weight=1)
        self.log_text = ScrolledText(box, height=16, wrap=tk.WORD)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        self.log_text.configure(state=tk.DISABLED)

    def _add_labeled_entry(self, parent, row, label, variable, width=24):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        ttk.Entry(parent, textvariable=variable, width=width).grid(row=row, column=1, sticky="ew", pady=2)

    def _add_readonly_row(self, parent, row, label, variable):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="nw", pady=2)
        ttk.Label(parent, textvariable=variable, wraplength=760, justify="left").grid(row=row, column=1, sticky="ew", pady=2)

    def _wire_ros(self):
        rospy.Subscriber(self.status_topic, MappingStatus, self._on_status)
        rospy.Subscriber(self.record_status_topic, MappingRecordStatus, self._on_record_status)
        rospy.Subscriber(self.environment_topic, MappingEnvironmentStatus, self._on_environment_status)

    def _on_status(self, msg):
        with self._lock:
            self._status = msg

    def _on_record_status(self, msg):
        with self._lock:
            self._record_status = msg

    def _on_environment_status(self, msg):
        with self._lock:
            self._environment_status = msg

    def _call_service_async(self, description, target):
        threading.Thread(target=self._service_thread, args=(description, target), daemon=True).start()

    def _service_thread(self, description, target):
        try:
            target()
        except Exception as exc:  # pylint: disable=broad-except
            error_text = f"{description}失败：\n{exc}"
            self.log(f"{description}失败: {exc}")
            self.root.after(0, lambda text=error_text: messagebox.showerror("服务调用错误", text))

    def _make_service_proxy(self, service_name, service_type, timeout=2.0):
        rospy.wait_for_service(service_name, timeout=timeout)
        return rospy.ServiceProxy(service_name, service_type)

    def on_check_environment(self):
        self._call_service_async("检测环境", self._do_check_environment)

    def _do_check_environment(self):
        proxy = self._make_service_proxy(self.check_env_service, CheckMappingEnvironment)
        req = CheckMappingEnvironmentRequest()
        req.include_runtime_checks = True
        req.config_path = self.config_path_var.get().strip()
        res = proxy(req)
        self.log(f"环境检测完成: overall_ok={res.overall_ok} summary={res.summary}")

    def on_refresh_status(self):
        self._call_service_async("刷新状态", self._do_refresh_status)

    def _do_refresh_status(self):
        proxy = self._make_service_proxy(self.get_status_service, GetMappingStatus)
        req = GetMappingStatusRequest()
        req.task_id = self.task_id_var.get().strip()
        res = proxy(req)
        with self._lock:
            self._status = res.status
            self._record_status = res.record_status
            self._environment_status = res.environment_status
        self.log("已通过服务刷新建图状态。")

    def on_start_mapping(self):
        self._call_service_async("开始建图", self._do_start_mapping)

    def _do_start_mapping(self):
        proxy = self._make_service_proxy(self.start_mapping_service, StartMapping)
        req = StartMappingRequest()
        req.task_name = self.task_name_var.get().strip()
        req.config_path = self.config_path_var.get().strip()
        req.output_root = self.output_root_var.get().strip()
        req.enable_loop_closure = bool(self.enable_loop_var.get())
        req.enable_visualization = bool(self.enable_rviz_var.get())
        req.auto_start_recording = bool(self.auto_record_var.get())
        res = proxy(req)
        self.log(f"开始建图请求结果: accepted={res.accepted} task_id={res.task_id} state={res.state} reason={res.reason}")
        if not res.accepted:
            self.root.after(0, lambda: messagebox.showwarning("开始建图被拒绝", res.reason or "请求被拒绝"))

    def on_stop_mapping(self):
        with self._lock:
            status = self._status
        save_map = True
        save_keyframes = True
        save_keyframe_poses = True
        save_trajectory = True
        if status.frame_count == 0 or status.keyframe_count == 0:
            only_stop = messagebox.askyesno(
                "无关键帧提示",
                "当前还没有有效关键帧，保存地图大概率会失败。\n是否仅停止建图而不保存？"
            )
            if only_stop:
                save_map = False
                save_keyframes = False
                save_keyframe_poses = False
                save_trajectory = False
        self._call_service_async(
            "结束建图",
            lambda: self._do_stop_mapping(save_map, save_keyframes, save_keyframe_poses, save_trajectory)
        )

    def _do_stop_mapping(self, save_map=True, save_keyframes=True, save_keyframe_poses=True, save_trajectory=True):
        proxy = self._make_service_proxy(self.stop_mapping_service, StopMapping)
        req = StopMappingRequest()
        req.task_id = self.task_id_var.get().strip()
        req.save_map = save_map
        req.save_keyframes = save_keyframes
        req.save_keyframe_poses = save_keyframe_poses
        req.save_trajectory = save_trajectory
        res = proxy(req)
        self.log(f"结束建图请求结果: accepted={res.accepted} state={res.state} reason={res.reason}")
        if not res.accepted:
            self.root.after(0, lambda reason=res.reason: messagebox.showwarning("结束建图被拒绝", reason or "请求被拒绝"))

    def on_start_recording(self):
        self._call_service_async("开始录包", self._do_start_recording)

    def _do_start_recording(self):
        proxy = self._make_service_proxy(self.start_record_service, StartBagRecording)
        req = StartBagRecordingRequest()
        req.task_id = self.task_id_var.get().strip()
        req.bag_name = self.bag_name_var.get().strip()
        req.output_dir = self.bag_output_dir_var.get().strip()
        topics = [topic.strip() for topic in self.bag_topics_var.get().split(",") if topic.strip()]
        req.topics = topics
        res = proxy(req)
        self.log(f"开始录包请求结果: accepted={res.accepted} state={res.state} bag_path={res.bag_path} reason={res.reason}")
        if not res.accepted:
            self.root.after(0, lambda: messagebox.showwarning("开始录包被拒绝", res.reason or "请求被拒绝"))

    def on_stop_recording(self):
        self._call_service_async("停止录包", self._do_stop_recording)

    def _do_stop_recording(self):
        proxy = self._make_service_proxy(self.stop_record_service, StopBagRecording)
        req = StopBagRecordingRequest()
        req.task_id = self.task_id_var.get().strip()
        req.save_now = True
        res = proxy(req)
        self.log(f"停止录包请求结果: accepted={res.accepted} state={res.state} bag_path={res.bag_path} reason={res.reason}")
        if not res.accepted:
            self.root.after(0, lambda: messagebox.showwarning("停止录包被拒绝", res.reason or "请求被拒绝"))

    def _refresh_ui(self):
        with self._lock:
            status = self._status
            record = self._record_status
            environment = self._environment_status
            log_lines = list(self._last_log_lines)

        self.state_var.set(status.state or "未知")
        self.task_id_var.set(status.task_id or "-")
        self.task_output_dir_var.set(status.output_dir or "-")
        self.task_message_var.set(status.status_message or "-")
        self.record_state_var.set(record.state or "未知")
        self.record_bag_path_var.set(record.bag_path or "-")
        self.result_map_path_var.set(status.map_path or "-")
        self.result_traj_path_var.set(status.trajectory_path or "-")
        self.result_keyframe_dir_var.set(status.keyframe_dir or "-")
        self.result_keyframe_pose_path_var.set(status.keyframe_pose_path or "-")

        env_lines = []
        summary = environment.summary or "当前还没有环境检测结果。"
        env_lines.append(f"总体结果: {'通过' if environment.overall_ok else '未知/失败'}")
        env_lines.append(f"摘要: {summary}")
        if environment.items:
            env_lines.append("")
            for item in environment.items:
                env_lines.append(f"[{item.level}] {item.name}: {'通过' if item.ok else '失败'}")
                if item.detail:
                    env_lines.append(f"  {item.detail}")
        self._replace_text(self.env_text, "\n".join(env_lines))
        self._replace_text(self.log_text, "\n".join(log_lines))

        self.root.after(500, self._refresh_ui)

    def _replace_text(self, widget, content):
        widget.configure(state=tk.NORMAL)
        widget.delete("1.0", tk.END)
        widget.insert(tk.END, content)
        widget.configure(state=tk.DISABLED)

    def log(self, line):
        stamp = time.strftime("%H:%M:%S")
        formatted = f"[{stamp}] {line}"
        with self._lock:
            self._last_log_lines.append(formatted)
            self._last_log_lines = self._last_log_lines[-300:]
        rospy.loginfo(formatted)

    def _on_close(self):
        try:
            rospy.signal_shutdown("GUI closed")
        finally:
            self.root.destroy()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    gui = MappingGuiNode()
    gui.run()
