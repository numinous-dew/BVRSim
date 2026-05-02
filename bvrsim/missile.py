from .utils import *
from pathlib import Path

level = 0  # 调试等级


class missile(model):  # 导弹基类
    def __init__(self, drone: model, info: EnemyInfo):
        """
        drone:载机实例
        info:目标敌机信息
        """
        super().__init__(str(Path(__file__).parent.parent))
        self.state = 0  # 0=中制导,1=末制导,2=已击中
        self.radarR = self.radarAngle = 0.0  # 雷达距离(m),角度(rad)
        self.roll2ail = self.az2ele = pid(*4 * [0])
        self.TargetID = info.EnemyID
        # 发射时目标经纬高(原点)
        self.geo = drone.los2geo(info.TargetYaw, info.TargetPitch, info.TargetDis)
        self.data = ()  # 目标最新绝对信息
        self.kalman = Kalman(self.fdm.get_delta_t(), sigma(info.TargetDis))
        self.kalman.X[3:6] = info.vNED  # 速度初始化

    def find(self, info: EnemyInfo, pos, std):
        # pos:np.ndarray,目标相对坐标向量
        self.data = pos - super().geo2NED(*self.geo), info.vNED, std

    def guide(self):
        pos, vNED, aNED = self.kalman.get()
        # 相对坐标 = 原点->目标 + 导弹->原点
        pos += super().geo2NED(*self.geo)
        dis = float(np.linalg.norm(pos)) or 1
        if not self.state and dis <= self.radarR:  # 进入末制导
            self.state = 1
        vNED -= self.vNED  # 绝对速度转相对速度
        los = pos / dis
        tgo = max(0.5, dis / max(1e-3, np.dot(-vNED, los)))  # 避免末段奇异
        N = 4  # 比例基准值
        acmd = N * (vNED - np.dot(vNED, los) * los) / tgo  # PN中制导
        if self.state:
            acmd += N / 2 * min(1, tgo / 2) * (aNED - np.dot(aNED, los) * los)  # an补偿
        acmd[2] -= g  # 抵消g
        return acmd * min(1, 40 * g / float(np.linalg.norm(acmd)))  # 过载限制

    def step(self):
        self.kalman.seta(0.1 + (not self.data))  # 目标丢失,加速度衰减增大
        self.kalman.predict()
        if self.data:
            self.kalman.update(*self.data)
            self.data = ()
        acmd = self.rotate().T @ self.guide()  # 转体轴加速度
        # 期望滚转角
        roll = np.arctan2(acmd[1], -acmd[2])
        # 副翼
        aileron = self.roll2ail.update(roll - self.Roll)
        # 升降舵
        elevator = self.az2ele.update(self.axyz[2] - acmd[2])
        super().step(aileron, elevator)


class aim120c(missile):
    def __init__(self, drone, info):
        super().__init__(drone, info)
        global level
        self.fdm.set_debug_level(level)
        self.fdm.load_model("aim120c", False)
        if level:
            input(type(self).__name__ + "调试,按Enter继续")
            level = 0
        self.roll2ail = pid(self.fdm.get_delta_t(), 5, 0, 0.03)  # 滚转角->副翼
        # -az->升降舵
        self.az2ele = pid(self.fdm.get_delta_t(), 0.35, 0.02, 0.008)
        self.radarR = 2e4
        self.radarAngle = np.deg2rad(20)
        lat, lon, alt = drone.los2geo(0, 0, 20)  # 前方20m,避免炸到载机
        # 继承载机状态
        lat, lon, head, pitch, roll = np.rad2deg(
            (lat, lon, drone.Yaw, drone.Pitch, drone.Roll)
        )  # 小仰角模拟高抛弹道
        super().ready(lat, lon, alt, drone.Mach_M, head, min(90, 5 + pitch), roll, 113)
