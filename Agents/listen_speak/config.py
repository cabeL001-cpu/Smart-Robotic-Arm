"""listen_speak 模块配置。

使用前只需要在 LLM_API_KEY 中填入 DeepSeek 的 API Key，
其余的麦克风、模型、播放设备参数按现场情况修改。
"""

from pathlib import Path

# ---------------- 大模型 ----------------
LLM_API_KEY = ""  # TODO: 填入你的 DeepSeek API Key
LLM_BASE_URL = "https://api.deepseek.com"
LLM_MODEL = "deepseek-flash"
LLM_TIMEOUT = 3.0
LLM_TEMPERATURE = 0.0
LLM_MAX_TOKENS = 64

# ---------------- 语音识别 ----------------
VOSK_MODEL_DIR = str(
    Path.home() / "Desktop" / "mxpidemo" / "sound" / "vosk-model-small-cn-0.22"
)
MIC_DEVICE = "plughw:CARD=WEBCAM,DEV=0"
SAMPLE_RATE = 16000
CHUNK_BYTES = 8000          # 16000Hz * 16bit * 单声道 * 0.25 秒
ENERGY_THRESHOLD = 1500     # 判断是否有人声的平均幅值
LISTEN_TIMEOUT = 8.0        # 等待用户开口的最长时间（秒）
SILENCE_TIMEOUT = 1.0       # 说完后连续静音多久算结束（秒）
MAX_RECORD_SECONDS = 12.0   # 单次录音上限（秒）

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
}
