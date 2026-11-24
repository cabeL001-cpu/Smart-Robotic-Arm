# 总控流程（task.py）

一条口令跑完整场：等唤醒词 → 听口令 → 南侧路线到板南 → 闭环入格 → 播报后退出。
代码入口是 `src/task.py`，全部 Python 都在树莓派上跑。

## 一、流程

```
启动 python -m task
  │  打开 Uno 连接（STOP 握手）→ AVOID OFF
  │  ★ 只开语音识别：不做 Apriltag 检测，不动车、不动臂
  ▼
等待“小伙小伙” ──(每轮等开口 8 s，循环)──→ 听到唤醒词
  │
  ├─ 播报：您好，有什么事，很高兴为您服务
  ├─ 等 0.6 s 再开录（避免把播报声录进口令）
  ▼
逐句监听口令（每轮最多等 7 s 开口）
  │
  ├─ 只听到声音、没识别出文字 ──→ 直接跳过，继续听（不播报、不打断）
  ├─ 识别出文字 ──→ 立即交 DeepSeek 解析（不再攒 7 s 窗口）
  ├─ 解析不出可用格号（只收 0/3/6/7/8）──→ 播报“没有听到指令”，同一次唤醒里继续听（不运动）
  ▼
播报：开始移动
IMU 静止标定 2 s → 东行 73 cm → 左转 85.5° → 北行 41 cm（move.south_route）
  ▼
播报：已抵达地点
vision.place 闭环入格（method=auto：0/3/6/7/8 走臂闭环；语音口令也只放行这五格）
  │
  ├─ 到位 ──→ 播报：已抵达{x}号位 ──→ 恢复 AVOID ON，退出码 0
  └─ 未到位/异常 ──→ 播报失败原因 ──→ 恢复 AVOID ON，退出码 1
```

退出码：`0` 成功；`1` 未到位或链路/IMU/相机异常；`2` 没设 `DEEPSEEK_API_KEY`；
`130` 用户 `Ctrl-C`。

## 二、上车前准备

场地与摆放（与 [南侧路线](move/SOUTH_ROUTE.md) 同一套坐标）：

- 车心放在 **(0.17, 0.17) m**，车头朝 **+X（东）**，启动后保持静止，等 IMU 标定完。
- 九宫格中心 **(0.90, 0.90) m**，边长 30 cm，南边在 `y = 0.75 m`。
- 路线结束车心 (0.90, 0.58) m，车头前沿贴住九宫格南边。
- 顶摄用 C270（通常是 `/dev/video2`），`TOP_CAM=2` 或 `--top-source 2`。

电源与串口：

- HC-05 在 **D0/D1** 上；烧录固件时必须拔下 HC-05，运行时不插 USB。
- `sh bt_link.sh bind sjtu013` 只在树莓派重启或 HC-05 断电后需要（脚本默认已是 sjtu013；sjtu006 的 TXD 坏，绑错会出现“Uno 未确认停车”）。
- IMU 在 `/dev/ttyUSB0`；行驶中 CH340 掉线会让任务中止，见第七节。

## 三、环境

```bash
cd ~/Smart-Robotic-Arm/src
source ~/Smart-Robotic-Arm/.venv/bin/activate
python -m pip install -r listen_speak/requirements.txt   # vosk numpy requests edge-tts piper-tts

export UNO_PORT=/dev/rfcomm0
export TOP_CAM=2
export DEEPSEEK_API_KEY=sk-...        # 不要写进仓库里的任何文件
# export DEEPSEEK_MODEL=deepseek-chat # 官方接口若报 Model Not Exist，用这行换模型
# export DEEPSEEK_BASE_URL=...        # 只有用第三方兼容网关时才需要
```

- `piper-tts` 不装也能跑：`tts.py` 会回退到 espeak-ng；Piper 模型要到
  `listen_speak/assets/piper/` 才生效（见 [语音模块](listen_speak/README.md)）。
- Vosk 模型路径固定为 `~/Desktop/mxpidemo/sound/vosk-model-small-cn-0.22`。
- 麦克风设备 `plughw:CARD=WEBCAM,DEV=0`，音箱走 3.5 mm 耳机孔。

## 四、运行

```bash
# 先干跑：只打印参数，不碰硬件、不需要相机和串口
python -m task --dry-run

# 正式运行
python -m task
```

常用参数（完整列表见 [API.md](API.md#7-总控)）：

```bash
python -m task --window-s 7 --wake-timeout 8          # 语音时序
python -m task --east-ms 4760 --north-ms 2830 --turn-deg 85.5   # 路线实测值
python -m task --top-source 2 --config mount.json     # 顶摄与安装参数
python -m task --method auto                          # 缺省；0/3/6/7/8 走臂闭环
```

`--dry-run` 的输出应显示唤醒词、7 s 窗口、路线三段时间、相机与 Key 状态；
Key 显示“未设置”时正式运行会在碰硬件之前返回 2。

## 五、为什么用 vision.place 而不是 arm

`arm.arm()` 需要 `~/.config/smart-robotic-arm/arm.json` 里的 `angle_at_max_deg` 等实测参数，
这台车的 `arm.json` 里为 `null`，开环解算会直接抛错。`vision.place` 用顶摄闭环：
`CELL_SEEDS` 给臂一个起点，剩下的按顶摄测到的指尖位置迭代，够不到的格交给
`vision.drive` 用车体微调。`method=auto` 就是这套分流。

## 六、代码地图

| 文件 | 作用 |
| --- | --- |
| `task.py` | 总控：参数、唤醒/口令循环、路线、入格、播报、退出码 |
| `listen_speak/__init__.py` | `wait_for_wake` / `collect` / `understand` / `listen` / 播报 |
| `listen_speak/config.py` | 唤醒词、7 s 窗口、播报模板、`DEEPSEEK_API_KEY` 读取 |
| `listen_speak/llm.py` | DeepSeek 分类：提示词与代码都只放行 0/3/6/7/8 格；“零”的同音误识别（林/林浩）在送模型前修正 |
| `move/south_route.py` | `drive_to_board()` 函数 + 原命令行入口 |
| `vision/place.py` | `place_cells()` 闭环入格总入口 |
| `move/heading.py`、`move/nav_sensors.py` | IMU 航向保持与传感器线程 |

路线本身只负责运动：`drive_to_board(imu=..., link=...)` 复用总控打开的串口和 IMU，
返回最终航向偏差，不再自己开关设备。

## 七、常见故障

| 现象 | 原因与处理 |
| --- | --- |
| 一直等不到唤醒词 | Vosk 小模型常把“小伙小伙”识别成同音词。先 `python -m listen_speak.asr` 看实际文本，把实际写法加进 `config.WAKE_WORDS`，或直接 `export WAKE_WORDS=小伙小伙,小伙,小活`。 |
| 一直只出现 `识别：（有声音，但没识别出文字）` | 环境噪声或说话太轻/太远，Vosk 没出字。靠近麦克风、放慢语速；纯噪声轮次会被跳过，不会打断流程。 |
| 识别出文字但没交给大模型 | 不应该发生；看终端是否有 `交给大模型解析：…`。若只有 `识别：…`，确认跑的是最新 `task.py` 并重启程序。 |
| 出现 `大模型未处理口令：…` | 按冒号后的原因处理：未配置 → 启动前 `export DEEPSEEK_API_KEY`；**API Key 无效 → 到平台重新生成 Key，确认是 api.deepseek.com 的 Key、账号有余额**；HTTP 400/模型不存在 → `export DEEPSEEK_MODEL=deepseek-chat`；请求失败/网络 → 查热点或代理。 |
| `大模型未处理口令：JSON 解析失败…` | 模型输出被截断或夹了额外文字。代码已把 `max_tokens` 提到 256，并会从被截断的原文里抢救格号；仍失败时看错误末尾的“原文”，必要时换 `DEEPSEEK_MODEL`。 |
| `大模型未处理口令：格数不可达：'4'` | 正常现象：本任务只接受 0/3/6/7/8 号格，1/2/4/5 按识别失败处理。在同一次唤醒里重说这五格之一即可。 |
| 说“0 号格”终端却识别成“林号格 / 林浩格” | Vosk 小模型词表里没有“零”。`llm._repair_zero()` 会把它改回 `0号格` 并打印 `格号修正：…`；没打印却仍失败时，把实际文本补进 `ZERO_HOMOPHONES` 的同音字表。 |
| 唤醒后立刻又提示没听到 | 播报声被麦克风录进去了；调大 `--echo-guard-s`，或把音箱音量调低。 |
| 播报“网络离线” | 树莓派没网或 DNS 不通，DeepSeek 不可用；检查热点/代理后重说口令。 |
| 启动即返回 2 | 没 `export DEEPSEEK_API_KEY`，或变量在当前 shell 里不存在。 |
| 路线中途 `IMU 读取失败` | CH340 整口掉线重枚举。看 `sudo dmesg | tail`；换 USB 口/线、查供电，然后从起点重跑整条路线。 |
| `ERR OBSTACLE` / 底盘不动 | 避障被固件恢复为 ON（上电会恢复），或探头缺失被判为受阻。总控在启动时会 `AVOID OFF`；单独调试时可 `python -m move --avoid off`。 |
| 入格中途停住 | 顶摄看不到角标/末端，或车体调试时避障拦下。检查相机分辨率、YUYV（`--no-raw` 反而是 MJPG）、现场光照。 |
| 车体闭环方向反了 | `vision.drive` 的 `probe_sign`/雅可比退化；先确认车头与九宫格南边平行，再重试。 |

## 八、验收清单

1. `python -m task --dry-run` 参数正确，Key 显示“已设置”。
2. 车放好、保持静止：说“小伙小伙”，听到问候语。
3. 说“去三号格”（0/3/6/7/8 之一；说 1/2/4/5 会被要求重说），听到“开始移动”，车走东—左转—北，停在板南。
4. 听到“已抵达地点”，顶摄臂闭环把末端送到 3 号格。
5. 听到“已抵达3号位”，程序退出，退出码 0。
6. 检查 `AVOID` 已恢复 ON（再跑一次 `python -m move --avoid on` 不会报错，或看固件行为）。

## 九、已知限制

1. **还没有 Apriltag 微调**：到达板南后直接入格，没有先用车头/板边微调姿态。
2. **唤醒词是整词匹配**：识别成同音词就不会启动；现场需要把实际写法补进 `WAKE_WORDS`。
3. **口令窗口内没有二次确认**：解析出格号就直接运动；演示时先确认场地清空。
4. **车体闭环会动车**：绕开语音、直接调用 `vision.place`（或 `--method drive`）测试 1/2/4/5 号格时，车体闭环会在“已抵达地点”之后小幅移动车轮，周围别站人。语音口令已不接收这四格。
5. **避障全程关闭**：路线和车体闭环都依赖这条路；这是演示流程的取舍，正式加避障前要先重标路线。
6. **A–D 停靠不在此流程内**：总控只接受 0/3/6/7/8 号格。
