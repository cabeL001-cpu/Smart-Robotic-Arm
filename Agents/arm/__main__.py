"""机械臂独立点动与格心规划入口。"""

import argparse

from . import ArmConfig, DEFAULT_PORT, cell_center, joint_command, set_joints


def main():
    parser = argparse.ArgumentParser(description="两自由度机械臂")
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--cell", type=int, choices=range(9), help="按图纸对齐后的目标格")
    target.add_argument("--joints", type=float, nargs=2, metavar=("肩角", "肘角"))
    target.add_argument("--export-limits", metavar="头文件", help="导出实测脉宽限位，供 Uno 编译")
    parser.add_argument("--config", help="舵机及机械臂几何标定 JSON")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--dry-run", action="store_true", help="仅计算角度和脉宽")
    args = parser.parse_args()
    try:
        config = ArmConfig.load(args.config) if args.config else ArmConfig()
        if args.export_limits:
            config.write_firmware_limits(args.export_limits)
            print(f"已写入 {args.export_limits}")
            return
        angles = args.joints if args.joints is not None else config.solve(*cell_center(args.cell))
        command = joint_command(angles, config)
        print(f"肩角 {angles[0]:.2f}°，肘角 {angles[1]:.2f}°")
        print(command.decode().strip())
        if not args.dry_run:
            set_joints(angles, config=config, port=args.port)
            print("Uno 已接受关节指令。")
    except (OSError, ValueError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
