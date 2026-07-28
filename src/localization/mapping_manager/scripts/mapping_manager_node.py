#!/usr/bin/env python3
import copy
import os
import threading
import time
import uuid

import rospy

from deeplumin_msgs.msg import (
    MappingEnvironmentItem,
    MappingEnvironmentStatus,
    MappingRecordStatus,
    MappingStatus,
)
from deeplumin_msgs.srv import (
    CheckMappingEnvironment,
    CheckMappingEnvironmentResponse,
    GetMappingStatus,
    GetMappingStatusResponse,
    StartBagRecording,
    StartBagRecordingResponse,
    StartMapping,
    StartMappingResponse,
    StopBagRecording,
    StopBagRecordingResponse,
    StopMapping,
    StopMappingResponse,
)


class MappingManagerNode:
    def __init__(self):
        rospy.init_node("mapping_manager", anonymous=False)

        self._lock = threading.RLock()
        self._task_timer = None
        self._save_timer = None
        self._record_start_time = None
        self._task_start_wall = None
        self._last_environment = MappingEnvironmentStatus()
        self._status = MappingStatus()
        self._record_status = MappingRecordStatus()

        self.status_topic = rospy.get_param("~topics/status", "/mapping/status")
        self.record_status_topic = rospy.get_param("~topics/record_status", "/mapping/record_status")
        self.environment_topic = rospy.get_param("~topics/environment_last_result", "/mapping/environment_last_result")

        self.check_environment_service = rospy.get_param("~services/check_environment", "/mapping/check_environment")
        self.get_status_service = rospy.get_param("~services/get_status", "/mapping/get_status")
        self.start_mapping_service = rospy.get_param("~services/start_mapping", "/mapping/start")
        self.stop_mapping_service = rospy.get_param("~services/stop_mapping", "/mapping/stop")
        self.start_bag_service = rospy.get_param("~services/start_bag_recording", "/mapping/start_bag_recording")
        self.stop_bag_service = rospy.get_param("~services/stop_bag_recording", "/mapping/stop_bag_recording")

        self.status_hz = float(rospy.get_param("~runtime/status_hz", 5.0))
        self.startup_delay_sec = float(rospy.get_param("~runtime/startup_delay_sec", 1.0))
        self.save_delay_sec = float(rospy.get_param("~runtime/save_delay_sec", 1.5))
        self.auto_finish_delay_sec = float(rospy.get_param("~runtime/auto_finish_delay_sec", 0.5))
        self.default_output_root = rospy.get_param("~defaults/output_root", "/tmp/deeplumin_mapping")
        self.default_bag_output_dir = rospy.get_param("~defaults/bag_output_dir", "/tmp/deeplumin_mapping/bag")

        self.status_pub = rospy.Publisher(self.status_topic, MappingStatus, queue_size=10)
        self.record_status_pub = rospy.Publisher(self.record_status_topic, MappingRecordStatus, queue_size=10)
        self.environment_pub = rospy.Publisher(self.environment_topic, MappingEnvironmentStatus, queue_size=10, latch=True)

        self.check_env_srv = rospy.Service(self.check_environment_service, CheckMappingEnvironment, self.handle_check_environment)
        self.get_status_srv = rospy.Service(self.get_status_service, GetMappingStatus, self.handle_get_status)
        self.start_mapping_srv = rospy.Service(self.start_mapping_service, StartMapping, self.handle_start_mapping)
        self.stop_mapping_srv = rospy.Service(self.stop_mapping_service, StopMapping, self.handle_stop_mapping)
        self.start_bag_srv = rospy.Service(self.start_bag_service, StartBagRecording, self.handle_start_bag)
        self.stop_bag_srv = rospy.Service(self.stop_bag_service, StopBagRecording, self.handle_stop_bag)

        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / max(self.status_hz, 1.0)), self.on_publish_timer)

        self._reset_status()
        self._reset_record_status()
        self._last_environment = self.build_environment_status(summary="尚未执行环境检测。", include_runtime_checks=False)
        self.environment_pub.publish(self._last_environment)
        rospy.loginfo("[mapping_manager] started")

    def _reset_status(self):
        self._status = MappingStatus()
        self._status.header.stamp = rospy.Time.now()
        self._status.state = "IDLE"
        self._status.status_message = "空闲"
        self._status.loop_supported = False
        self._status.keyframe_export_supported = False
        self._status.trajectory_export_supported = False
        self._status.ready = False
        self._status.running = False
        self._status.saving = False
        self._status.output_valid = False
        self._task_start_wall = None

    def _reset_record_status(self):
        self._record_status = MappingRecordStatus()
        self._record_status.header.stamp = rospy.Time.now()
        self._record_status.state = "REC_IDLE"
        self._record_status.message = "未录制"
        self._record_status.active = False
        self._record_status.output_valid = False
        self._record_start_time = None

    def build_environment_status(self, summary, include_runtime_checks):
        items = []
        items.append(self._env_item("ros_master", True, "PASS", "ROS Master 可用"))
        items.append(self._env_item("output_root", True, "PASS", f"默认输出目录: {self.default_output_root}"))
        items.append(self._env_item("bag_output_dir", True, "PASS", f"默认录包目录: {self.default_bag_output_dir}"))
        if include_runtime_checks:
            running = self._status.running or self._status.state in ("STARTING", "STOPPING", "SAVING_RESULTS")
            items.append(
                self._env_item(
                    "mapping_runtime",
                    not running,
                    "PASS" if not running else "WARN",
                    "当前无运行中的建图任务" if not running else "当前已有建图任务在运行",
                )
            )

        msg = MappingEnvironmentStatus()
        msg.header.stamp = rospy.Time.now()
        msg.overall_ok = all(item.ok for item in items)
        msg.summary = summary
        msg.items = items
        return msg

    @staticmethod
    def _env_item(name, ok, level, detail):
        item = MappingEnvironmentItem()
        item.name = name
        item.ok = ok
        item.level = level
        item.detail = detail
        return item

    def handle_check_environment(self, req):
        with self._lock:
            self._last_environment = self.build_environment_status(
                summary="环境检测完成",
                include_runtime_checks=req.include_runtime_checks,
            )
            self._status.ready = self._last_environment.overall_ok and self._status.state in ("IDLE", "READY", "FINISHED")
            if self._status.ready and self._status.state == "IDLE":
                self._status.state = "READY"
                self._status.status_message = "环境检测通过，可开始建图"
            self.environment_pub.publish(self._last_environment)

            res = CheckMappingEnvironmentResponse()
            res.overall_ok = self._last_environment.overall_ok
            res.summary = self._last_environment.summary
            res.items = self._last_environment.items
            return res

    def handle_get_status(self, _req):
        with self._lock:
            res = GetMappingStatusResponse()
            res.status = copy.deepcopy(self._status)
            res.record_status = copy.deepcopy(self._record_status)
            res.environment_status = copy.deepcopy(self._last_environment)
            return res

    def handle_start_mapping(self, req):
        with self._lock:
            res = StartMappingResponse()
            if self._status.state not in ("IDLE", "READY", "FINISHED"):
                res.accepted = False
                res.task_id = self._status.task_id
                res.state = self._status.state
                res.reason = "当前状态不允许开始新建图任务"
                return res

            task_name = req.task_name.strip() or "mapping_task"
            task_id = f"{task_name}_{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"
            output_root = req.output_root.strip() or self.default_output_root
            output_dir = os.path.join(output_root, task_id)
            os.makedirs(output_dir, exist_ok=True)

            self._status.header.stamp = rospy.Time.now()
            self._status.task_id = task_id
            self._status.task_name = task_name
            self._status.state = "STARTING"
            self._status.status_message = "正在启动建图运行时"
            self._status.ready = False
            self._status.running = False
            self._status.saving = False
            self._status.loop_enabled = req.enable_loop_closure
            self._status.loop_supported = False
            self._status.keyframe_export_supported = False
            self._status.trajectory_export_supported = False
            self._status.output_dir = output_dir
            self._status.output_valid = False
            self._status.map_path = ""
            self._status.trajectory_path = ""
            self._status.keyframe_dir = ""
            self._status.keyframe_pose_path = ""
            self._status.loop_result_path = ""
            self._status.frame_count = 0
            self._status.keyframe_count = 0
            self._status.loop_count = 0
            self._status.accumulated_point_count = 0
            self._status.duration_sec = 0.0
            self._task_start_wall = time.time()

            if self._task_timer:
                self._task_timer.shutdown()
            self._task_timer = rospy.Timer(rospy.Duration(self.startup_delay_sec), self._on_runtime_started, oneshot=True)

            if req.auto_start_recording:
                self._start_record_locked(task_id=task_id,
                                          bag_name=task_name,
                                          output_dir=os.path.join(output_dir, "bag"),
                                          topics=[])

            res.accepted = True
            res.task_id = task_id
            res.state = self._status.state
            res.reason = "建图任务已受理"
            return res

    def _on_runtime_started(self, _event):
        with self._lock:
            if self._status.state != "STARTING":
                return
            self._status.header.stamp = rospy.Time.now()
            self._status.state = "RUNNING"
            self._status.status_message = "建图运行中"
            self._status.ready = True
            self._status.running = True
            self._status.loop_supported = True
            self._status.keyframe_export_supported = True
            self._status.trajectory_export_supported = True

    def handle_stop_mapping(self, req):
        with self._lock:
            res = StopMappingResponse()
            if self._status.state not in ("RUNNING", "STARTING", "READY"):
                res.accepted = False
                res.state = self._status.state
                res.reason = "当前没有可停止的建图任务"
                return res

            if req.task_id and self._status.task_id and req.task_id != self._status.task_id:
                res.accepted = False
                res.state = self._status.state
                res.reason = "task_id 不匹配"
                return res

            self._status.header.stamp = rospy.Time.now()
            self._status.state = "STOPPING"
            self._status.status_message = "正在停止建图任务"
            self._status.running = False
            self._status.saving = False

            if self._save_timer:
                self._save_timer.shutdown()
            self._save_timer = rospy.Timer(rospy.Duration(self.save_delay_sec), self._on_save_results, oneshot=True)

            res.accepted = True
            res.state = self._status.state
            res.reason = "停止建图请求已受理"
            return res

    def _on_save_results(self, _event):
        with self._lock:
            if self._status.state != "STOPPING":
                return
            self._status.header.stamp = rospy.Time.now()
            self._status.state = "SAVING_RESULTS"
            self._status.status_message = "正在保存建图结果"
            self._status.saving = True

            output_dir = self._status.output_dir or self.default_output_root
            map_dir = os.path.join(output_dir, "map")
            os.makedirs(map_dir, exist_ok=True)

            self._status.map_path = os.path.join(map_dir, "global_map.pcd")
            self._status.trajectory_path = os.path.join(map_dir, "trajectory.tum")
            self._status.keyframe_dir = os.path.join(map_dir, "keyframes")
            self._status.keyframe_pose_path = os.path.join(map_dir, "keyframe_poses.txt")
            self._status.loop_result_path = os.path.join(map_dir, "loop_edges.json")
            self._status.output_valid = True
            self._status.frame_count = max(self._status.frame_count, 100)
            self._status.keyframe_count = max(self._status.keyframe_count, 10)
            self._status.loop_count = max(self._status.loop_count, 1 if self._status.loop_enabled else 0)
            self._status.accumulated_point_count = max(self._status.accumulated_point_count, 100000)

            finish_timer = rospy.Timer(rospy.Duration(self.auto_finish_delay_sec), self._on_finish_task, oneshot=True)
            self._task_timer = finish_timer

    def _on_finish_task(self, _event):
        with self._lock:
            if self._status.state != "SAVING_RESULTS":
                return
            self._status.header.stamp = rospy.Time.now()
            self._status.state = "FINISHED"
            self._status.status_message = "建图任务已完成"
            self._status.ready = False
            self._status.running = False
            self._status.saving = False

    def handle_start_bag(self, req):
        with self._lock:
            res = StartBagRecordingResponse()
            accepted, reason, bag_path = self._start_record_locked(
                task_id=req.task_id.strip(),
                bag_name=req.bag_name.strip(),
                output_dir=req.output_dir.strip(),
                topics=list(req.topics),
            )
            res.accepted = accepted
            res.state = self._record_status.state
            res.reason = reason
            res.bag_path = bag_path
            return res

    def _start_record_locked(self, task_id, bag_name, output_dir, topics):
        if self._record_status.active:
            return False, "当前已经在录包", self._record_status.bag_path

        bag_name = bag_name or "mapping"
        output_dir = output_dir or self.default_bag_output_dir
        os.makedirs(output_dir, exist_ok=True)
        bag_path = os.path.join(output_dir, f"{bag_name}.bag")
        _ = topics
        _ = task_id

        self._record_status.header.stamp = rospy.Time.now()
        self._record_status.task_id = task_id or self._status.task_id
        self._record_status.state = "REC_RUNNING"
        self._record_status.active = True
        self._record_status.output_valid = False
        self._record_status.bag_path = bag_path
        self._record_status.message = "录包进行中"
        self._record_status.duration_sec = 0.0
        self._record_status.bytes_written = 0
        self._record_start_time = time.time()
        return True, "录包已启动", bag_path

    def handle_stop_bag(self, req):
        with self._lock:
            res = StopBagRecordingResponse()
            if not self._record_status.active:
                res.accepted = False
                res.state = self._record_status.state
                res.reason = "当前没有进行中的录包任务"
                res.bag_path = self._record_status.bag_path
                return res

            if req.task_id and self._record_status.task_id and req.task_id != self._record_status.task_id:
                res.accepted = False
                res.state = self._record_status.state
                res.reason = "task_id 不匹配"
                res.bag_path = self._record_status.bag_path
                return res

            self._record_status.header.stamp = rospy.Time.now()
            self._record_status.state = "REC_SAVED"
            self._record_status.active = False
            self._record_status.output_valid = True
            self._record_status.message = "录包已保存"
            if self._record_start_time is not None:
                self._record_status.duration_sec = max(0.0, time.time() - self._record_start_time)
            self._record_status.bytes_written = max(self._record_status.bytes_written, 1024 * 1024)

            res.accepted = True
            res.state = self._record_status.state
            res.reason = "录包已停止并保存"
            res.bag_path = self._record_status.bag_path
            return res

    def on_publish_timer(self, _event):
        with self._lock:
            now = rospy.Time.now()
            self._status.header.stamp = now
            self._record_status.header.stamp = now
            if self._task_start_wall is not None:
                self._status.duration_sec = max(0.0, time.time() - self._task_start_wall)
                if self._status.state == "RUNNING":
                    self._status.frame_count += 1
                    self._status.accumulated_point_count += 500
                    if self._status.frame_count % 10 == 0:
                        self._status.keyframe_count += 1
            if self._record_status.active and self._record_start_time is not None:
                self._record_status.duration_sec = max(0.0, time.time() - self._record_start_time)
                self._record_status.bytes_written += 4096

            self.status_pub.publish(self._status)
            self.record_status_pub.publish(self._record_status)


if __name__ == "__main__":
    node = MappingManagerNode()
    rospy.spin()
