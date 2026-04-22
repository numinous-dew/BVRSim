from bvrsim import bvrsim, DroneInfo, SendData
import numpy as np


class mySim(bvrsim):
    def redstrategy(self, info: DroneInfo, step_num: int) -> SendData:
        """红方策略（需返回SendData指令）"""
        cmd = SendData()
        cmd.CmdSpd = 1.5
        cmd.CmdAlt = 12000
        cmd.CmdHeadingDeg = 180
        # 示例：每200步锁定敌机，再过100步发射导弹
        if step_num % 200 == 0:
            cmd.engage = -1  # 火控锁定
        # 保证发射仰角，发射导弹
        elif step_num % 200 == 100 and 1 < np.rad2deg(info.Pitch) < 5:
            cmd.engage = 1
        cmd.EnemyID = 3  # 攻击/锁定ID=3的敌机
        return cmd

    def bluestrategy(self, info: DroneInfo, step_num: int) -> SendData:
        """蓝方策略（需返回SendData指令）"""
        cmd = SendData()
        cmd.CmdSpd = 1.2
        cmd.CmdAlt = 10000
        cmd.CmdHeadingDeg = 0
        if step_num % 150 == 0:
            cmd.engage = -1
        elif step_num % 150 == 75 and 1 < np.rad2deg(info.Pitch) < 5:
            cmd.engage = 1
        cmd.EnemyID = 1
        return cmd


# 自定义战场范围（纬度、经度、高度）
field = ((23.0, 26.0), (118.0, 120.0), (2000, 15000))
# 自定义威胁区（红方飞机额外禁区）
threat = ((24.5, 119.0, 0, 50000),)

if __name__ == "__main__":  # 多进程必须
    sim = mySim(field=field, threat=threat)

    # 覆盖红蓝方默认初始兵力参数（纬度、经度、高度、航向、马赫、导弹数、燃油）
    sim.red = [
        dict(lat=24.2, lon=118.2, alt=10000, head=180, mach=0.8, num=4, fuel=5000),
        dict(lat=24.2, lon=118.4, alt=10000, head=180, mach=0.8, num=4, fuel=5000),
    ]
    sim.blue = [
        dict(lat=23.2, lon=118.2, alt=10000, head=0, mach=0.8, num=6, fuel=5000),
        dict(lat=23.2, lon=118.4, alt=10000, head=0, mach=0.8, num=6, fuel=5000),
    ]
    sim.main(time=10)  # 仿真 10 分钟（游戏内时间）
