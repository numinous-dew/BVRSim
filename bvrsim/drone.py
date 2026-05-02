from . import DroneInfo, SendData
from .missile import *

table = (
    (np.linspace(0, 18e3, 7), (0.35, 0.5, 0.65, 0.8, 1, 1.15, 1.25)),  # 高度(m)
    (np.linspace(0.5, 1.5, 5), (0.7, 0.9, 1, 1.15, 1.25)),  # 马赫
    (np.linspace(0, np.pi, 7), (1.3, 1.2, 1, 0.8, 0.6, 0.45, 0.35)),  # 相对方位角(rad)
)
MissileTrackList = defaultdict(list[missile])  # 已射导弹列表


class drone(model):
    def __init__(self, root):
        super().__init__(root)
        self.radarR = self.radarAngle = 0.0
        self.AlarmList = []  # 告警列表(辐射源,相对方位,类型)
        self.FoundEnemyList = []  # 发现敌机列表(EnemyInfo)
        self.MissileNowNum = 0  # 当前导弹数量
        self.engage = 0  # -1=火控锁定,1=发射导弹,从0变化有效
        self.roll2ail = self.z2pitch = self.pitch2ele = self.mach2thr = pid(*4 * [0])

    def attack(self, engage, EnemyID, misl):
        # misl:missile子类
        enemy = next((x for x in self.FoundEnemyList if x.EnemyID == EnemyID), None)
        if not self.engage and enemy:
            if -1 == engage:  # 火控锁定
                enemy.isNTS = True
            elif (
                1 == engage  # 发射导弹
                and 0 < self.MissileNowNum  # 有导弹
                and enemy.TargetDis <= enemy.MissileMaxDis  # 射程内
            ):
                MissileTrackList[self.ID].append(misl(self, enemy))
                self.MissileNowNum -= 1
        self.engage = engage

    def step(self, cmd: SendData, misl=aim120c):
        # cmd:飞机控制指令
        self.attack(cmd.engage, cmd.EnemyID, misl)
        # 航向误差
        dhead = const(np.deg2rad(cmd.CmdHeadingDeg) - self.Yaw)
        if np.sign(cmd.TurnDirection * dhead) < 0:
            dhead = cmd.TurnDirection * (2 * np.pi - abs(dhead))  # 就远转
        # 1:1航向变化率,滚转角限幅
        roll = np.deg2rad(min(90, abs(cmd.CmdPhi)))
        roll = np.clip(np.arctan2(self.Mach_M * Vs * dhead, g), -roll, roll)
        # 副翼
        aileron = self.roll2ail.update(roll - self.Roll)
        # 俯仰角限幅
        pitch = np.deg2rad(min(90, abs(cmd.CmdPitchDeg)))
        self.z2pitch.limit = -pitch, pitch
        # 高度差转体轴
        dz = (self.rotate().T @ (0, 0, self.Altitude - cmd.CmdAlt))[2]
        # 级联PID
        pitch = self.z2pitch.update(-dz)
        elevator = self.pitch2ele.update(pitch - self.Pitch)
        # 油门限幅
        self.mach2thr.limit = 0.1, np.clip(cmd.ThrustLimit / 129, 0.1, 1)
        # 油门
        throttle = self.mach2thr.update(cmd.CmdSpd - self.Mach_M)
        super().step(aileron, elevator, throttle)

    def alarm(self, AlarmID, MisAzi, AlarmType):
        """
        AlarmID:int,辐射源ID
        MisAzi:float,相对方位角(rad)
        AlarmType:str,告警类型
        """
        self.AlarmList.append((AlarmID, MisAzi, AlarmType))

    def find(self, info: EnemyInfo, pos, std, isNTS):
        info.isNTS = isNTS
        for (xp, fp), x in zip(table[:2], (self.Altitude, self.Mach_M)):
            x = np.interp(x, xp, fp)
            info.MissilePowerfulDis *= x
            info.MissileMaxDis *= x
        # 马赫反比于NEZ
        info.MissilePowerfulDis *= max(0.5, 1 - info.TargetMach_M / 10)
        # 攻角影响射程
        info.MissileMaxDis *= np.interp(abs(info.TargetYaw), *table[2])
        self.FoundEnemyList.append(info)
        # 导弹数据链
        for misl in MissileTrackList[self.ID]:
            if info.EnemyID == misl.TargetID and misl.state < 2:
                # 载机到导弹的相对坐标
                rel = super().geo2NED(misl.Latitude, misl.Longitude, misl.Altitude)
                # info只要vNED
                misl.find(info, pos - rel, std)

    def getinfo(self):
        info = DroneInfo(self)
        info.strike = [x.TargetID for x in MissileTrackList[self.ID] if x.state == 2]
        return info


class f16(drone):
    def __init__(self, lat, lon, alt, mach, num, head, pitch=0.0, roll=0.0, fuel=5e3):
        super().__init__(None)
        self.fdm.load_model("f16")
        self.radarR = 14e4
        self.radarAngle = np.deg2rad(60)
        self.MissileNowNum = num
        # 滚转角->副翼
        self.roll2ail = pid(self.fdm.get_delta_t(), 3, 0, 0.1)
        # 体轴高度->俯仰角->升降舵
        self.z2pitch = pid(self.fdm.get_delta_t(), 0.05, 0.005, 0.05)
        self.pitch2ele = pid(self.fdm.get_delta_t(), 3, 0.02, 0.25)
        # 马赫->油门
        self.mach2thr = pid(self.fdm.get_delta_t(), 1.8, 0.08, 0.08)
        super().ready(lat, lon, alt, mach, head, pitch, roll, fuel)
