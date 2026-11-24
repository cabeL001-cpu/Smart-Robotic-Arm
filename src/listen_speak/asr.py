"""离线语音识别：arecord 采集 + 能量 VAD + Vosk 识别。

完全不依赖网络，也不使用 PyAudio
（这台树莓派上 PyAudio/PortAudio 枚举不到 C270 的麦克风）。
"""

import json
import subprocess
import time

import numpy as np
from vosk import KaldiRecognizer, Model, SetLogLevel

from . import config

SetLogLevel(-1)

_model = None


def _get_model():
    """Vosk 模型加载较慢，全局只加载一次。"""
    global _model
    if _model is None:
        _model = Model(config.VOSK_MODEL_DIR)
    return _model


def _energy(chunk):
    """返回这一小段音频的平均幅值，用来判断有没有人声。"""
    samples = np.frombuffer(chunk, dtype=np.int16)
    return float(np.mean(np.abs(samples)))


def listen(timeout=None):
    """录一句话并返回识别文本；没有检测到人声时返回空字符串。"""
    timeout = config.LISTEN_TIMEOUT if timeout is None else timeout
    recognizer = KaldiRecognizer(_get_model(), config.SAMPLE_RATE)

    proc = subprocess.Popen(
        [
            "arecord", "-q", "-D", config.MIC_DEVICE,
            "-f", "S16_LE", "-r", str(config.SAMPLE_RATE),
            "-c", "1", "-t", "raw",
        ],
        stdout=subprocess.PIPE,
    )

    start = time.monotonic()
    speech_start = None
    silence_start = None
    text = ""

    try:
        while True:
            chunk = proc.stdout.read(config.CHUNK_BYTES)
            if not chunk:
                break

            now = time.monotonic()
            loud = _energy(chunk) >= config.ENERGY_THRESHOLD

            # 还没听到人声：超时就退出
            if speech_start is None:
                if loud:
                    speech_start = now
                elif now - start >= timeout:
                    break
                else:
                    continue

            # 识别当前这一小段
            if recognizer.AcceptWaveform(chunk):
                text += json.loads(recognizer.Result()).get("text", "")

            # 连续静音足够久：认为说完了
            if loud:
                silence_start = None
            else:
                if silence_start is None:
                    silence_start = now
                if now - silence_start >= config.SILENCE_TIMEOUT:
                    break

            # 防止一直说下去
            if now - speech_start >= config.MAX_RECORD_SECONDS:
                break
    finally:
        proc.terminate()
        proc.wait(timeout=2)

    if speech_start is None:
        return ""

    text += json.loads(recognizer.FinalResult()).get("text", "")
    return "".join(text.split())


if __name__ == "__main__":
    print("请说话……")
    print("识别结果：", listen())
