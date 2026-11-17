"""语音播报：edge-tts 在线合成，espeak-ng 离线兜底，pw-play 播放。

在线合成成功的语音会缓存成 WAV，之后断网也能播放同一句话。
"""

import asyncio
import hashlib
import shutil
import subprocess
from pathlib import Path

from . import config


def _cache_path(text):
    """同一句话、同一个音色对应同一个缓存文件。"""
    key = hashlib.md5((config.TTS_VOICE + text).encode("utf-8")).hexdigest()
    return Path(config.TTS_CACHE_DIR) / (key + ".wav")


def _synthesize_edge(text, wav_path):
    """用 edge-tts 合成，再用 ffmpeg 转成 WAV。"""
    import edge_tts

    wav_path.parent.mkdir(parents=True, exist_ok=True)
    mp3_path = wav_path.with_suffix(".mp3")
    asyncio.run(edge_tts.Communicate(text, config.TTS_VOICE).save(str(mp3_path)))
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "quiet", "-i", str(mp3_path), str(wav_path)],
        check=True,
    )
    mp3_path.unlink(missing_ok=True)


def _synthesize_espeak(text, wav_path):
    """离线合成，中文音色 cmn。"""
    subprocess.run(
        ["espeak-ng", "-v", config.ESPEAK_VOICE, "-w", str(wav_path), text],
        check=True,
    )


def _play(wav_path):
    """优先用 PipeWire 播放（实测可用），没有就退回 aplay 直连耳机孔。"""
    if shutil.which("pw-play"):
        subprocess.run(["pw-play", str(wav_path)], check=True)
    else:
        subprocess.run(
            ["aplay", "-q", "-D", config.PLAY_DEVICE, str(wav_path)],
            check=True,
        )


def speak(text, offline=False):
    """播放一段语音。offline=True 时只使用缓存或 espeak-ng。"""
    wav_path = _cache_path(text)
    wav_path.parent.mkdir(parents=True, exist_ok=True)

    if wav_path.exists():
        _play(wav_path)
        return True

    if not offline:
        try:
            _synthesize_edge(text, wav_path)
            _play(wav_path)
            return True
        except Exception:
            pass  # 在线合成失败，退回离线引擎

    offline_path = wav_path.with_name(wav_path.stem + "-espeak.wav")
    _synthesize_espeak(text, offline_path)
    _play(offline_path)
    return True


def say_event(event, offline=False, **params):
    """按模板播报事件：start / arrive / complete / fail / empty / offline。"""
    template = config.SAY_TEMPLATES[event]
    try:
        return speak(template.format(**params), offline=offline)
    except Exception:
        return False


def prewarm():
    """在线时预生成固定播报，之后断网也能用自然语音播放。"""
    texts = [config.SAY_TEMPLATES["offline"], config.SAY_TEMPLATES["complete"]]
    for number in range(9):
        texts.append(config.SAY_TEMPLATES["start"].format(number=number))
        texts.append(config.SAY_TEMPLATES["arrive"].format(number=number))

    for text in texts:
        wav_path = _cache_path(text)
        if not wav_path.exists():
            _synthesize_edge(text, wav_path)
            print("已生成", wav_path.name, text)


if __name__ == "__main__":
    import sys

    if "--prewarm" in sys.argv:
        prewarm()
    else:
        say_event("offline", offline=True)
