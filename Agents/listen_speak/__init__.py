"""语音模块入口。

对外只暴露 listen()、speak()、say_event()。

listen() 返回 (格数, 动作)：
    ("4", "arrive")   前往 4 号格
    (None, None)      空指令 / 离线 / 未配置 / 没听清
"""

from . import config
from .asr import listen as recognize
from .llm import (
    LLMNotConfigured,
    LLMOffline,
    LLMParseError,
    classify,
    is_online,
)
from .tts import say_event, speak

EMPTY_RESULT = (None, None)


def listen(timeout=None):
    """完整链路：在线检查 → 录音识别 → 大模型分类 → (格数, 动作)。"""
    if not config.LLM_API_KEY:
        say_event("fail", reason="大模型未配置", offline=True)
        return EMPTY_RESULT

    # 离线时按约定直接提示，不录音、不调大模型
    if not is_online():
        say_event("offline", offline=True)
        return EMPTY_RESULT

    text = recognize(timeout)
    if not text:
        say_event("fail", reason="没有听清，请再说一遍", offline=True)
        return EMPTY_RESULT

    try:
        result = classify(text)
    except LLMOffline:
        say_event("offline", offline=True)
        return EMPTY_RESULT
    except LLMNotConfigured:
        say_event("fail", reason="大模型未配置", offline=True)
        return EMPTY_RESULT
    except LLMParseError:
        say_event("fail", reason="没有听清，请再说一遍", offline=True)
        return EMPTY_RESULT

    if result == EMPTY_RESULT:
        say_event("empty")
    return result


if __name__ == "__main__":
    print(listen())
