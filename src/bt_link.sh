#!/bin/sh
# 把已配对的 HC-05 绑定成 /dev/rfcomm0。树莓派每次重启或模块重连后都要重跑 bind。
#
# 用法：
#   sh bt_link.sh bind [MAC|名称]   省略时按 BT_NAME 自动查找
#   sh bt_link.sh release
#   sh bt_link.sh status
#
# 可用环境变量覆盖：BT_NAME（默认 sjtu006）、BT_DEV（默认 rfcomm0）、BT_CHANNEL（默认 1）。
set -eu

NAME=${BT_NAME:-sjtu006}
DEV=${BT_DEV:-rfcomm0}
CHANNEL=${BT_CHANNEL:-1}
PORT=/dev/$DEV

find_mac() {
    bluetoothctl devices | awk -v name="$NAME" '$1 == "Device" && $3 == name { print $2; exit }'
}

case ${1:-bind} in
    bind)
        target=${2:-}
        case $target in
            "") mac=$(find_mac) ;;
            *:*:*:*:*:*) mac=$target ;;
            *) NAME=$target; mac=$(find_mac) ;;
        esac
        if [ -z "$mac" ]; then
            echo "找不到名为 $NAME 的已配对设备。先运行 bluetoothctl 配对，或把 MAC 作为第二个参数传入。" >&2
            exit 1
        fi
        # 旧绑定可能停留在半断开状态，先释放再绑定。
        sudo rfcomm release "$DEV" 2>/dev/null || true
        sudo rfcomm bind "$DEV" "$mac" "$CHANNEL"
        [ -r "$PORT" ] || echo "设备已创建，但当前用户无权读取；执行 sudo chown $USER $PORT 或确认在 dialout 组。" >&2
        ls -l "$PORT"
        ;;
    release)
        sudo rfcomm release "$DEV"
        ;;
    status)
        rfcomm show "$DEV" || true
        ls -l "$PORT" 2>/dev/null || echo "$PORT 不存在"
        ;;
    *)
        echo "用法：$0 bind [MAC] | release | status" >&2
        exit 2
        ;;
esac
