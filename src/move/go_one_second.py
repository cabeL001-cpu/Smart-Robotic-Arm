"""在 src 目录运行 python3 -m move.go_one_second，前进一秒后停车。"""

from . import move


if __name__ == "__main__":
    move("go", 1000)
