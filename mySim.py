from bvrsim import bvrsim, DroneInfo, SendData
import numpy as np


class mySim(bvrsim):
    def redstrategy(self, info: DroneInfo, step_num: int) -> SendData:
        """红方策略（需返回SendData指令）"""
        cmd = SendData()
        cmd.CmdSpd = 2
        cmd.CmdAlt = 12000
        cmd.CmdHeadingDeg = 180
        cmd.EnemyID = 3  # 攻击/锁定ID=3的敌机
        # 目标信息
        enemy = next((x for x in info.FoundEnemyList if x.EnemyID == cmd.EnemyID), None)
        if not step_num % 200:
            cmd.engage = -1
        elif (
            step_num % 200 == 100
            and enemy
            and enemy.TargetDis < 5e4  # 保证发射距离
            and 1 < np.rad2deg(info.Pitch) < 5  # 保证发射仰角
        ):
            cmd.engage = 1
        return cmd

    def bluestrategy(self, info: DroneInfo, step_num: int) -> SendData:
        """蓝方策略（需返回SendData指令）"""
        cmd = SendData()
        cmd.CmdSpd = 1.2
        cmd.CmdAlt = 11000
        cmd.CmdHeadingDeg = 0
        cmd.EnemyID = 1
        enemy = next((x for x in info.FoundEnemyList if x.EnemyID == cmd.EnemyID), None)
        if not step_num % 100:
            cmd.engage = -1
        elif (
            step_num % 100 == 50
            and enemy
            and enemy.TargetDis < 6e4
            and 1 < np.rad2deg(info.Pitch) < 5
        ):
            cmd.engage = 1
        return cmd


# 战场空间：((纬度范围 deg), (经度范围 deg), (高度范围 m))
field = ((23.0, 26.0), (118.0, 120.0), (2000, 15000))
# 红方防空威胁区：(中心纬度 deg, 中心经度 deg, 高度 m, 半径 m)
threat = ((24.5, 119.0, 0, 50000),)

if __name__ == "__main__":  # 多进程必须
    sim = mySim(field=field, threat=threat)

    # 覆盖双方默认初始兵力参数（纬度、经度、高度、航向、马赫、导弹数、燃油）
    sim.red = [
        dict(lat=24.2, lon=118.2, alt=10000, head=180, mach=0.8, num=4, fuel=5000),
        dict(lat=24.2, lon=118.4, alt=10000, head=180, mach=0.8, num=4, fuel=5000),
    ]
    sim.blue = [
        dict(lat=23.2, lon=118.2, alt=10000, head=0, mach=0.8, num=6, fuel=5000),
        dict(lat=23.2, lon=118.4, alt=10000, head=0, mach=0.8, num=6, fuel=5000),
    ]
    sim.main(time=10)  # 仿真 10 分钟（游戏内时间）
