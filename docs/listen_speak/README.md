# listen_speak 语音模块说明

## 一、模块职责

把一句话变成一条结构化指令，并负责机器人的语音播报。

- **唤醒监听**：听到“小伙小伙”之前只做语音识别，不触发任何检测或运动
- **语音识别**：麦克风录音 → 能量 VAD 断句 → Vosk 离线识别
- **指令解析**：识别文本 → DeepSeek Flash → 校验 → `(格数, 动作)`
- **语音播报**优先播放缓存。需要合成时，在线使用 edge-tts，离线使用 Piper，Piper 失败后使用 espeak-ng。

一句话总结：`听懂 → 返回 (格数, "arrive") → 播报状态`。

## 二、对外接口

```python
from listen_speak import listen, speak, say_event

result = listen()
```

`listen()` 返回值：

| 情况 | 返回值 |
|---|---|
| 正常识别 | `("3", "arrive")` |
| 空指令（用户说不用了、待机等） | `(None, None)` |
| 网络离线 | `(None, None)`（已播报"网络离线，机器暂时无法使用"） |
| API Key 未配置 | `(None, None)`（已播报"大模型未配置"） |
| 没听清 / 解析失败 / 格号不可达 | `(None, None)`（已播报"没听清，请再说一遍"） |

- **格数**：字符串，只接受 `"0"`、`"3"`、`"6"`、`"7"`、`"8"`（这几格臂能闭环到位）；
  说 `1/2/4/5` 会按识别失败处理，让用户重说
- **动作**：固定字符串 `"arrive"`
- A-D 停靠可由上层调用 `MHS.dispatch(("A", "arrive"), navigator=nav)`。本模块的语音分类目前只产生上述五个格号。[系统调用示例](../API.md)

总控（`python -m task`）唤醒后逐句处理，不等静音窗口攒句子：

```python
from listen_speak import wait_for_wake, recognize, understand

wait_for_wake()        # 阻塞，直到听到 WAKE_WORDS（默认“小伙小伙”）；每轮等开口 LISTEN_TIMEOUT 秒
text = recognize(7.0)  # 录一句话：最多等 7 s 开口，说完静音 1 s 断句；没听清返回 ""
cell, action = understand(text)   # 只调用大模型并校验，不录音、不播报；失败返回 (None, None)
```

- 总控只要 `recognize()` 返回非空文字就立刻调用 `understand()`；返回空（纯噪声或没听到人声）就跳过这一轮继续听，不播报、不要求重新说唤醒词。
- `collect()` 仍保留为独立接口：它把连续多句攒到 7 s 静音窗口结束再返回，适合需要整段文本的场景；总控已不再使用。
- `understand()` 对“离线/未配置/格式错/接口报错”都返回 `(None, None)`，但会先在终端打印 `大模型未处理口令：<原因>`；总控自己用 `is_online()` 决定播报“网络离线”还是“没有听到指令”。

播报接口：

```python
say_event("start", number="3")          # 开始执行任务，目标3号格
say_event("arrive", number="3")         # 已到达3号位置
say_event("complete")                   # 任务完成
say_event("fail", reason="未到位")       # 任务失败，未到位
say_event("offline", offline=True)      # 网络离线，机器暂时无法使用
say_event("wake")                       # 您好，有什么事，很高兴为您服务
say_event("moving")                     # 开始移动
say_event("arrived")                    # 已抵达地点
say_event("cell", number="3")           # 已抵达3号位
say_event("no_command")                 # 没有听到指令，请再说一遍
```

## 三、目录结构

```
listen_speak/
├── __init__.py        # 对外接口 listen / wait_for_wake / collect / understand / speak / say_event
├── config.py          # 所有配置（API Key 读环境变量 DEEPSEEK_API_KEY）
├── asr.py             # 语音识别：arecord + VAD + Vosk
├── llm.py             # 大模型调用与结果校验（含“零”同音字修正、可达格号白名单）
├── test_llm.py        # llm 的离线单元测试，不联网
├── tts.py             # 语音合成与播放
├── requirements.txt   # Python 依赖
├── README.md          # 本文件
├── assets/tts/        # 播报缓存，运行时生成，不入库
└── assets/piper/      # Piper 离线语音模型，需下载，不入库
```

## 四、工作流程

```
listen()
  │
  ├─ API Key 为空 ──────────────→ 播报"大模型未配置" → (None, None)
  │
  ├─ 网络探测失败 ──────────────→ 播报"网络离线…"   → (None, None)
  │
  ├─ arecord 录音 → 能量 VAD → Vosk 离线识别
  │     │
  │     └─ 没听到人声 ──────────→ 播报"没听清…"     → (None, None)
  │
  ├─ 文本 → 同音字修正（“林浩格”→“0号格”）→ DeepSeek Flash（JSON 模式）
  │
  ├─ 校验 action / number
  │     ├─ 非法或格号不可达 ────→ 播报"没听清…"     → (None, None)
  │     └─ action = null ───────→ 播报"好的"        → (None, None)
  │
  └─ 返回 ("3", "arrive")
```

## 五、三个子模块

### 5.1 asr.py —— 离线语音识别

- 用 `arecord` 子进程直接采集，**不使用 PyAudio**：这台树莓派上 PortAudio 枚举不到 C270 麦克风，现有 PyAudio 脚本会报 `Invalid input device`
- 用平均幅值判断人声起止，参数在 `config.py` 中：
  - `ENERGY_THRESHOLD`：人声阈值
  - `SPEECH_START_CHUNKS`：连续几个响亮 chunk（0.25 s/个）才算开口，默认 2
  - `SILENCE_TIMEOUT`：静音多久算说完
  - `LISTEN_TIMEOUT`：最多等多久开口
- arecord 每次启动的第一个 chunk 常有爆音（实测均值超过阈值），所以要求连续多个响亮 chunk 才开录，这些 chunk 会一起送进 Vosk，不会丢掉字头
- Vosk 使用小模型 `vosk-model-small-cn-0.22`（66MB），适合树莓派实时识别
- 每收录到一段人声都会在终端打印 `识别：<文本>`；只有噪声、没识别出文字时打印 `识别：（有声音，但没识别出文字）`，方便现场核对唤醒词和口令

### 5.2 llm.py —— 大模型解析

调用 DeepSeek 的 OpenAI 兼容接口，提示词要求模型只输出 JSON：

```json
{"action": "arrive", "number": "3", "confidence": 0.95}
```

约束分三层：

1. **提示词**：固定动作集合、规定字段类型、给出正反例，并写明本次只接受 0/3/6/7/8 号格
2. **接口参数**：`temperature=0`、`max_tokens=256`、`response_format={"type":"json_object"}`
3. **代码校验**：
   - 去掉 Markdown 代码块、提取 JSON 对象
   - `action` 只能是 `"arrive"` 或 `null`
   - `number` 只能是 `"0"`~`"8"`，中文数字会自动转换；`1/2/4/5` 不在可达集合内，按解析失败处理（`LLMParseError`），上层继续听
   - “零”被 Vosk 听成“林/林浩/林号”等（如“林浩格”）时，`_repair_zero()` 在送大模型前改回 `0号格`；`CN_NUMBERS` 也兜底接受模型原样抄回的“林/林浩/O”
   - 任一校验失败都返回 `(None, None)`，不执行动作

失败时抛出 `LLMOffline` / `LLMNotConfigured` / `LLMParseError`，由 `__init__.py` 统一转成播报和 `(None, None)`。

`LLM_MAX_TOKENS` 为 256，给 flash 模型留出“先说明再输出 JSON”的余量；如果返回仍被截断，`_parse()` 会从原文里抢救 `action`/`number` 两个白名单字段，抢救不到才报 `LLMParseError`（错误信息附带原文前 120 字）。

### 5.3 tts.py —— 语音合成与播放

| 情况 | 行为 |
|---|---|
| 在线合成成功 | edge-tts 合成 → ffmpeg 转 WAV → 缓存 → `pw-play` 播放 |
| 同一句话已缓存 | 直接播放缓存，不再联网 |
| 离线或合成失败 | Piper `huayan-medium` 合成 → `pw-play` 播放；Piper 失败再退回 espeak-ng |

- 缓存目录：`assets/tts/`，按文本内容哈希命名
- 播放用 `pw-play`（实测可用）；没有时退回 `aplay -D plughw:CARD=Headphones,DEV=0`
- `prewarm()` 可以在有网时预生成固定播报，之后断网也能播放自然语音
- 离线兜底用 **Piper 神经网络合成**（`assets/piper/zh_CN-huayan-medium.onnx`，约 61MB），音质远好于 espeak-ng 的机械音；模型需按下方命令单独下载，不入库

## 六、配置说明（config.py）

| 配置项 | 默认值 | 说明 |
|---|---|---|
| `LLM_API_KEY` | `os.environ.get("DEEPSEEK_API_KEY", "")` | **不要把密钥写进仓库**；运行时 `export DEEPSEEK_API_KEY=...` |
| `WAKE_WORDS` | `("小伙小伙", "小伙", "小虎小虎", "小虎")` | 唤醒词；默认收 Vosk 常听成的单“小伙”和“小虎”。可用环境变量 `WAKE_WORDS=词1,词2` 覆盖，不必改代码 |
| `COMMAND_WINDOW_S` | `7.0` | 唤醒后收集口令的静音窗口（秒） |
| `LLM_BASE_URL` | `https://api.deepseek.com` | 接口地址；用第三方兼容网关时 `export DEEPSEEK_BASE_URL=...` 覆盖 |
| `LLM_MODEL` | `deepseek-flash` | 模型名；官方接口用 `deepseek-chat`/`deepseek-reasoner`，可用 `DEEPSEEK_MODEL` 覆盖 |
| `LLM_TIMEOUT` | `3.0` | 单次请求超时（秒） |
| `VOSK_MODEL_DIR` | `~/Desktop/mxpidemo/sound/vosk-model-small-cn-0.22` | Vosk 模型路径 |
| `MIC_DEVICE` | `plughw:CARD=WEBCAM,DEV=0` | C270 麦克风设备 |
| `ENERGY_THRESHOLD` | `1500` | 人声检测阈值（实测环境噪声约 650） |
| `SPEECH_START_CHUNKS` | `2` | 连续几个响亮 chunk 才算开口，用于剔除 arecord 启动爆音 |
| `SILENCE_TIMEOUT` | `1.0` | 静音断句时间（秒） |
| `LISTEN_TIMEOUT` | `8.0` | 等待开口时间（秒） |
| `TTS_VOICE` | `zh-CN-XiaoxiaoNeural` | edge-tts 音色 |
| `ESPEAK_VOICE` | `cmn` | espeak-ng 中文音色 |
| `PIPER_MODEL` | `assets/piper/zh_CN-huayan-medium.onnx` | Piper 离线音色模型路径 |
| `PLAY_DEVICE` | `plughw:CARD=Headphones,DEV=0` | aplay 兜底设备 |

## 七、运行环境与依赖

在树莓派的 `src` 目录安装依赖。下面使用部署指南创建的虚拟环境，已有 `~/mx` 环境时也可激活该环境。

```bash
cd ~/Smart-Robotic-Arm/src
source ~/Smart-Robotic-Arm/.venv/bin/activate
python -m pip install -r listen_speak/requirements.txt
```

大模型密钥只放环境变量，仓库里的 `config.py` 保持为空：

```bash
export DEEPSEEK_API_KEY=sk-...
```

系统依赖（已安装）：`espeak-ng`、`ffmpeg`、`pipewire`（提供 `pw-play`）。

离线语音模型（Piper，下载到 `assets/piper/`，不入库）：

```bash
mkdir -p ~/Smart-Robotic-Arm/src/listen_speak/assets/piper
cd ~/Smart-Robotic-Arm/src/listen_speak/assets/piper
# 国内镜像（hf-mirror）直连；huggingface.co 直连超时
curl -L -o zh_CN-huayan-medium.onnx \
  'https://hf-mirror.com/rhasspy/piper-voices/resolve/main/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx'
curl -L -o zh_CN-huayan-medium.onnx.json \
  'https://hf-mirror.com/rhasspy/piper-voices/resolve/main/zh/zh_CN/huayan/medium/zh_CN-huayan-medium.onnx.json'
```

## 八、自测命令

安装模型后切回 `src/`，在同一虚拟环境执行。完整链路需要麦克风、扬声器与可用的模型服务。[部署、模型位置与密钥输入](../DEPLOY_RASPBERRY_PI.md#8-可选语音部署)

```bash
# 1. 预生成固定播报（需要联网和 edge-tts）
python -m listen_speak.tts --prewarm

# 2. 播放离线提示（验证音箱和离线合成）
python -m listen_speak.tts

# 3. 只测语音识别（对麦克风说一句话）
python -m listen_speak.asr

# 4. 只测大模型解析（先 export DEEPSEEK_API_KEY；“4号”会按不可达报错，换 3 号）
python -m listen_speak.llm "请到九宫格的3号位置"
python -m listen_speak.llm "去林浩格"      # 验证“零”的同音字修正

# 5. 完整链路（先 export DEEPSEEK_API_KEY）
python -m listen_speak

# 6. 只测唤醒 + 窗口收集，不驱动车
python -c 'from listen_speak import wait_for_wake, collect; print(wait_for_wake()); print(collect())'
```

## 九、离线与异常行为

| 情况 | 行为 | 返回 |
|---|---|---|
| 网络不可用 | 播报"网络离线，机器暂时无法使用" | `(None, None)` |
| API Key 为空 | 播报"大模型未配置" | `(None, None)` |
| 用户没说话 | 播报"没听清，请再说一遍" | `(None, None)` |
| 大模型返回非法格式 | 播报"没听清，请再说一遍" | `(None, None)` |
| 用户说"不用了" | 播报"好的" | `(None, None)` |
| edge-tts 合成失败 | 自动改用 Piper | 正常播报 |

## 十、已知限制与待确认

1. **不处理 A-D**：口令里出现"去 A 点"时返回 `(None, None)`，A-D 停靠由其他模块决定
2. **系统集成**：本模块和项目约定都使用 `"arrive"`。当前底盘独立接受方向和时长，格号与停靠点属于上层目标。[系统集成说明](../README.md)
3. **播报是同步的**：`say_event()` 会等语音播完才返回；如果执行模块需要并行，可以后续加线程
4. **音箱**：默认输出是 3.5mm 耳机孔，需要确认现场插了音箱
5. **模型名**：`deepseek-flash` 和 `LLM_BASE_URL` 需要按实际接入平台确认
6. **唤醒词是整词匹配**：Vosk 小模型常把“小伙小伙”识别成别的同音词；现场多试几次，把实际识别到的写法补进 `WAKE_WORDS` 即可
7. **总控唤醒一次后会持续重听口令**：口令为空或解析不出格号都只播报“没有听到指令”，不要求重新说唤醒词；要退出监听用 `Ctrl-C`。环境嘈杂时注意周围人声，避免被解析成格号
