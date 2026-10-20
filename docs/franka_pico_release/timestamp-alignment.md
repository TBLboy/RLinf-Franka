# Timestamped collection and LeRobot conversion

Use this page to collect timestamped PKL episodes and export training data without changing the source files.

## Business contract

Collection writes the raw Pico episode to PKL without resampling or destructive
alignment. Every retained stream carries its own timestamp and data together.
The PKL remains the source record, including Pico controls, raw events, and
diagnostic fields that are not needed by training.

PKL-to-LeRobot conversion is the only alignment stage. It creates an equally
spaced 30 Hz grid and independently selects the nearest raw sample for each
required stream. A match is valid only when its distance from the grid point is
no greater than half that stream's sampling period. Offline conversion may use
the nearest sample before or after the grid point.

Required streams are:

- global and wrist camera images;
- measured robot state, including TCP, gripper, and Franka joint state;
- the actual post-guard robot action.

If any required stream has no valid match at any grid point, conversion fails
with an explicit diagnostic. It does not fill values with zeros, a previous
sample, or extrapolation, and it does not modify or delete the PKL. Raw samples
outside the aligned output interval remain in the PKL.

LeRobot output contains training fields only: camera images, robot state,
gripper state, actual action, task, success/end markers, and segment metadata.
Pico controls, Pico timestamps, raw events, and diagnostics remain PKL-only.

## Timestamp semantics

All host timestamps use one monotonic clock domain where possible:

- cameras: host frame-receive time, not an exposure timestamp;
- robot state: host time when the measured state is read;
- action: host time when the final post-guard command is sent;
- Pico: vendor timestamp plus host receive time;
- episode/reset events: host monotonic event time.

The converter must use host-monotonic timestamps for cross-stream matching and
must preserve the original timestamp semantics in the PKL metadata.

## State and Action Frames

`observations.raw_state` preserves the measured robot fields before observation wrappers: TCP position in metres and quaternion in xyzw order in the robot base frame. `training_state` preserves named fields after the configured wrappers; `states` concatenates these fields in sorted-key order. With relative-frame and Euler wrappers enabled, training TCP pose is reset-relative xyz plus Euler angles, and velocity uses the end-effector frame. Force, torque and joint fields retain the backend convention; the relative wrapper does not rotate force or torque.

Raw robot samples retain both `robot_state_fields` and training `state_fields`. Action samples retain the base-frame post-guard TCP target, `action_base`, the transformed training `action`, and `action_frame`. The relative action transform uses the pre-step end-effector transform, matching the live action interface; it is not a measured velocity or evidence that the robot reached the target.

Camera periods come from each camera's configured FPS. Robot, gripper-read and command periods come from the environment cadence, not the controller's internal servo rate. These are requested periods, not measured achieved rates. Unchanged cached gripper timestamps do not create new samples. The output grid is anchored at recording start and restricted to shared stream coverage; endpoint loss exceeding one source period is rejected, as are missing matches, non-increasing timestamps and reported overflow.

## Memory and Failure Handling

Each camera archives only its selected monocular training image, using the same crop, resize and channel order as the live observation, with the original capture timestamp. The current training shape is 128×128×3 uint8. Full stereo frames are not archived. Images have one archival owner in `raw_streams`, rather than additional copies in saved `observations` or `infos`; live observations returned to the runner are unchanged. At two cameras and 30 FPS, image payload is about 2.95 MB/s, or 177 MB/minute before other metadata (about 31 times smaller than full 1344×376 stereo frames). No lossy compression is applied. The converter still accepts older stereo payloads.

One asynchronous episode write may be pending; the next submission waits rather than growing an unbounded queue. Each active episode still resides in RAM. `RLINF_RAW_EPISODE_MAX_BYTES` limits raw array payloads to 1073741824 bytes by default; this is not a whole-process RSS limit. Exceeding it raises an error and the active episode is not saved. Do not raise this limit without a host-memory budget.

PKL writes use a temporary file, flush/fsync and atomic rename. Writer failures propagate on the next collection call or close. Unexpected episode endings are archived with `missing_terminal_event` and rejected by conversion. A process crash can leave a `.partial` file; it is not a completed episode.

The converter reads one source PKL at a time, validates all episodes, then writes to a sibling `.lerobot-partial-*` directory. Only a successful finalization publishes the final directory. Existing destinations are rejected, and failed exports leave source PKLs unchanged. Abrupt process termination may leave the explicitly partial directory for manual cleanup.

## Short Operator Acceptance Run

From the Release root, clear the workspace, keep the emergency stop reachable, and run:

```bash
bash scripts/run_pico_session.sh --check
bash scripts/run_pico_session.sh
```

Start with an unloaded, small-range recording of 2–3 seconds. Use the configured Pico left-stick click to start, hold right grip for teleoperation, and Y to finish successfully. Confirm both cameras and gripper changes in the PKL, then stop the session normally so pending writes drain. Stop immediately on unexpected motion, vibration, force or Desk warnings.

```bash
.venv/bin/python scripts/check_collection_run.py --run-dir <run-directory>
.venv/bin/python scripts/post_process_to_lerobot.py --run-dir <run-directory> --repo-id local/franka-pico-check --dry-run
.venv/bin/python scripts/post_process_to_lerobot.py --run-dir <run-directory> --repo-id local/franka-pico-check
```

The first command checks collection quality; the second validates strict alignment; the third creates the dataset. Use a new repo ID for each export. Separately test discard and a bounded timeout run. An alignment failure is a failed acceptance result, not permission to fill missing samples or relax robot protections.

## 中文验收摘要

原始 `raw_state` 在包装前保存，训练用 `training_state` 在包装后保存；post-guard 动作同时保留基座坐标和训练坐标版本。相机周期取各自 FPS，机器人/夹爪读取和动作周期取环境频率，这些是配置周期而非实测频率。缺流、重复时间戳、溢出或缺少结束事件会阻止导出。

先按上述命令前检，再由操作者空载、小范围录制 2–3 秒：左摇杆按下开始、握住右 grip 遥操作、Y 成功结束。正常退出后依次做质量检查、dry-run 和实际导出；每次用新 repo ID。另行验收丢弃和超时。异常运动、振动、受力或 Desk 告警立即停止。

每台相机只保存实际训练使用的单目图像，裁剪、缩放和通道顺序与实时观测相同，当前为 128×128×3 uint8，并保留采集时间戳。不保存完整双目帧，不在 observations/infos 重复存图，也不引入有损压缩。两台相机 30 FPS 的图像载荷约 2.95 MB/s、177 MB/分钟，约为原来的 1/31；其他状态及元数据另计。转换器仍兼容旧双目载荷。

训练图像仍按 episode 驻留内存，默认数组预算为 1 GiB，超过预算报错且当前 episode 不保存；这不是进程总内存上限。异步写盘限制为一个待写 episode，PKL 原子发布，导出逐条读取并仅在完成后发布最终目录；失败不修改源 PKL。当前图像预处理一致性及短数据导出/回读已做离线检查，最终真实硬件完整链路与峰值 RSS 仍需现场验收，不能视为已验证稳定版本。
