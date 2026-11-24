"""语音模块入口。

对外暴露：
    listen()         一次完整链路，返回 (格数, 动作)
    wait_for_wake()  阻塞等待唤醒词
    collect()        唤醒后按静音窗口收集连续口令
    understand()     只做文字 -> (格数, 动作)，不录音、不播报
    speak()、say_event()  语音播报

listen() / understand() 返回 (格数, 动作)：
    ("3", "arrive")   前往 3 号格（只接受 0/3/6/7/8 号格）
    (None, None)      空指令 / 离线 / 未配置 / 没听清 / 格号不可达
"""

from . import config
from .asr import listen as recognize
from .llm import (
    LLMError,
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


def wake_heard(text):
    """文本里是否包含唤醒词。去掉空格再匹配，兼容“小伙 小伙”这类分词结果。"""
    compact = "".join((text or "").split())
    return any(word in compact for word in config.WAKE_WORDS)


def wait_for_wake(timeout=None):
    """反复录音，直到听到唤醒词为止；每轮等用户开口 timeout 秒。"""
    while True:
        if wake_heard(recognize(timeout)):
            return True


def collect(timeout=None):
    """唤醒后收集口令：timeout 秒内没听到人声就结束，听到一句就重新计时。"""
    timeout = config.COMMAND_WINDOW_S if timeout is None else timeout
    parts = []
    while True:
        text = recognize(timeout)
        if not text:
            break
        parts.append(text)
    return "".join(parts)


def understand(text):
    """把收集到的文字交给大模型解析；失败时在终端打印原因并返回空指令。"""
    if not (text or "").strip():
        return EMPTY_RESULT
    try:
        return classify(text)
    except LLMError as error:
        # 总控只关心有没有格号，但失败原因必须让人看见，
        # 否则“大模型没被调用”和“调用了但报错”在终端上看起来一样。
        print(f"大模型未处理口令：{error}", flush=True)
        return EMPTY_RESULT


if __name__ == "__main__":
    print(listen())
