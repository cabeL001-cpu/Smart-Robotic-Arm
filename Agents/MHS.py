"""树莓派系统入口。open_uno 可供连续的底盘与机械臂动作共用。"""

if __package__:
    from .arm import arm
    from .move import move
    from .move.navigation import go_to, open_navigation
    from .uno import open_uno
else:
    from arm import arm
    from move import move
    from move.navigation import go_to, open_navigation
    from uno import open_uno

__all__ = ["arm", "move", "open_uno", "go_to", "open_navigation", "dispatch", "listen", "say"]


def listen(timeout=None):
    """调用现有语音识别接口。只在实际使用麦克风时加载语音依赖。"""
    if __package__:
        from .listen_speak import listen as listen_command
    else:
        from listen_speak import listen as listen_command
    return listen_command(timeout=timeout)


def say(text, offline=False):
    """使用现有播报接口播放文本，返回播放结果。"""
    if __package__:
        from .listen_speak import speak
    else:
        from listen_speak import speak
    return speak(text, offline=offline)


def dispatch(command, *, navigator=None, arm_config=None, link=None):
    """分派 (目标, 动作)。停靠点走导航，格号在已标定的机械臂工位执行。"""
    target, action = command
    if target is None and action is None:
        return None
    if action != "arrive":
        raise ValueError(f"未知动作 {action!r}")
    name = str(target).upper()
    if name in ("A", "B", "C", "D"):
        if navigator is None:
            raise ValueError("停靠点任务需要已打开的导航会话")
        return navigator.go_to_dock(name)
    if name not in tuple("012345678") or arm_config is None:
        raise ValueError("对格任务需要 0–8 的格号与实测 arm_config")
    if navigator is not None:
        with navigator.shared_link() as shared:
            if link is not None and link is not shared:
                raise ValueError("同一会话应复用同一 Uno 连接")
            return arm(name, config=arm_config, link=shared)
    return arm(name, config=arm_config, link=link)
