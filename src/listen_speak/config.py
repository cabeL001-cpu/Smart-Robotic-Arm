"""listen_speak 模块配置。

API Key 从环境变量 DEEPSEEK_API_KEY 读取，不要把密钥写进仓库；
其余的麦克风、模型、播放设备参数按现场情况修改。
"""

import os
from pathlib import Path

# ---------------- 大模型 ----------------
# 优先读环境变量，避免把密钥写进仓库；未设置时回退到下面的常量（仓库里保持为空）。
LLM_API_KEY = os.environ.get("DEEPSEEK_API_KEY", "").strip()
# 使用第三方兼容网关时，用 DEEPSEEK_BASE_URL 覆盖成对方给的地址。
LLM_BASE_URL = os.environ.get("DEEPSEEK_BASE_URL", "https://api.deepseek.com").rstrip("/")
# 官方接口的模型名是 deepseek-chat / deepseek-reasoner；
# 如果使用兼容网关提供的 flash 模型，可用环境变量 DEEPSEEK_MODEL 覆盖。
LLM_MODEL = os.environ.get("DEEPSEEK_MODEL", "deepseek-flash")
LLM_TIMEOUT = 3.0
LLM_TEMPERATURE = 0.0
LLM_MAX_TOKENS = 256       # 留足余量：部分 flash 模型会先输出一段说明再给 JSON

# ---------------- 语音识别 ----------------
VOSK_MODEL_DIR = str(
    Path.home() / "Desktop" / "mxpidemo" / "sound" / "vosk-model-small-cn-0.22"
)
MIC_DEVICE = "plughw:CARD=WEBCAM,DEV=0"
SAMPLE_RATE = 16000
CHUNK_BYTES = 8000          # 16000Hz * 16bit * 单声道 * 0.25 秒
ENERGY_THRESHOLD = 1500     # 判断是否有人声的平均幅值
SPEECH_START_CHUNKS = 2     # 连续几个响亮 chunk 才算开口（剔除 arecord 启动爆音）
LISTEN_TIMEOUT = 8.0        # 等待用户开口的最长时间（秒）
SILENCE_TIMEOUT = 1.0       # 说完后连续静音多久算结束（秒）
MAX_RECORD_SECONDS = 12.0   # 单次录音上限（秒）

# ---------------- 总控唤醒与口令收集 ----------------
# 唤醒词，逗号分隔，可用环境变量 WAKE_WORDS 覆盖。
# Vosk 小模型常把“小伙小伙”吞成“小伙”，现场也可以把实际识别到的同音词补进来。
WAKE_WORDS = tuple(
    word.strip()
    for word in os.environ.get("WAKE_WORDS", "小伙小伙,小伙,小虎小虎,小虎").split(",")
    if word.strip()
)
COMMAND_WINDOW_S = 7.0      # 唤醒后收集口令的静音窗口；说到新句子就重新计时

# ---------------- 语音合成 ----------------
TTS_VOICE = "zh-CN-XiaoxiaoNeural"
TTS_CACHE_DIR = str(Path(__file__).parent / "assets" / "tts")
ESPEAK_VOICE = "cmn"
PIPER_MODEL = str(
    Path(__file__).parent / "assets" / "piper" / "zh_CN-huayan-medium.onnx"
)
PLAY_DEVICE = "plughw:CARD=Headphones,DEV=0"  # pw-play 不可用时的 aplay 兜底

# ---------------- 播报模板 ----------------
SAY_TEMPLATES = {
    "start": "开始执行任务，目标{number}号格",
    "arrive": "已到达{number}号位置",
    "complete": "任务完成",
    "fail": "任务失败，{reason}",
    "empty": "好的",
    "offline": "网络离线，机器暂时无法使用",
    # 总控流程的固定播报
    "wake": "您好，有什么事，很高兴为您服务",
    "moving": "开始移动",
    "arrived": "已抵达地点",
    "cell": "已抵达{number}号位",
    "no_command": "没有听到指令，请再说一遍",
    # 南侧路线阶段播报与任务结束
    "turn": "距离符合误差，开始左转",
    "dock": "到达停靠区，符合误差",
    "end": "任务结束",
}
