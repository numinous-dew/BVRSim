import numpy as np
from . import EnemyInfo
from jsbsim import FGFDMExec
from collections import defaultdict

g = 9.80665  # 重力加速度
Vs = 303.77  # 9km声速(m/s)
m2ft = lambda x: x / 0.3048  # 米转英尺
ft2m = lambda x: 0.3048 * x  # 英尺转米
const = lambda x, y=np.pi: np.mod(x + y, 2 * y) - y  # 把x约束到[-y,y]
# 距离,速度标准差正比于距离平方
sigma = lambda x: np.clip((x / 2e4) ** 2, 0.004, 3) * np.array((100, 7))
# 系数分母
k = 1, 2, 28 / 3, 84, 1680, 30240, 1209600, 3024e4, 3024e5, 108864e4


def expm(A):  # 9阶Pade近似求矩阵指数
    j = np.linalg.norm(A, 1)  # 1范数
    j = max(0, 2 + int(np.floor(np.log2(j)))) if 0 < j else 0
    A /= 2**j
    E = np.eye(len(A))
    N = D = 0
    for i, x in enumerate(k):
        N += E / x
        D += (-1) ** i * E / x
        E @= A
    E = np.linalg.solve(D, N)
    for _ in range(j):
        E @= E
    return E


def psd(P):  # 确保P半正定
    val, vec = np.linalg.eigh((P + P.T) / 2)
    return vec @ np.diag(np.maximum(1e-12, val)) @ vec.T


class Kalman:
    H = np.hstack((np.eye(6), np.zeros((6, 3))))  # 观测矩阵

    def seta(self, a):
        # a:float,机动频率(1/s)
        if self.a != a:  # Singer模型单轴连续状态矩阵
            A = np.array(((0, 1, 0), (0, 0, 1), (0, 0, -a)))
            Q = np.zeros((6, 6))
            Q[2, 4] = self.q
            Q[:3, :3] = -A
            Q[3:, 3:] = A.T
            A, Q = map(expm, (A * self.dt, Q * self.dt))
            Q = A @ Q[:3, 3:] @ A.T
            Q = (Q + Q.T) / 2
            self.a = a
            self.F = np.eye(9)  # 状态转移矩阵
            self.Q = np.zeros((9, 9))  # 过程噪声协方差矩阵
            for i in range(0, 7, 3):
                self.F[i : 3 + i, i : 3 + i] = A
                self.Q[i : 3 + i, i : 3 + i] = Q

    def __init__(self, dt, std, q=5):
        """
        dt:float,滤波周期(s)
        std:np.ndarray,距离,速度标准差
        q:float,连续过程噪声功率谱密度(m^2/s^5)
        """
        self.dt = dt
        self.q = q
        self.X = np.zeros(9)  # 目标NED系绝对坐标(m),速度(m/s),加速度(m/s^2)
        # 状态估计误差协方差矩阵
        self.P = np.diag(np.repeat(np.append(std, 20) ** 2, 3))
        self.a = None
        self.seta(0.1)

    def predict(self):  # 绝对状态预测,不依赖导弹运动
        # 状态预测
        self.X = self.F @ self.X
        # 协方差预测
        self.P = psd(self.F @ self.P @ self.F.T + self.Q)

    def update(self, pos, vNED, std):
        """
        pos:np.ndarray,目标绝对坐标观测
        vNED:np.ndarray,目标绝对速度观测
        """
        R = np.diag(np.repeat(std**2, 3))  # 测量噪声协方差矩阵
        # 卡尔曼增益
        K = np.linalg.solve((self.H @ self.P @ self.H.T + R).T, self.H @ self.P.T).T
        # 状态更新
        self.X += K @ (np.hstack((pos, vNED)) - self.H @ self.X)
        # Joseph形式协方差更新
        I_KH = np.eye(9) - K @ self.H
        self.P = psd(I_KH @ self.P @ I_KH.T + K @ R @ K.T)

    def get(self):
        return self.X[:3].copy(), self.X[3:6].copy(), self.X[6:].copy()


class spatialgrid:  # 均匀网格优化邻近点查找
    def __init__(self, size):
        self.size = size
        self.grid = defaultdict(list)

    def clear(self):
        self.grid.clear()

    def hash(self, pos: np.ndarray):
        return tuple((pos // self.size).astype(int))

    def add(self, ID, pos):
        self.grid[self.hash(pos)].append(ID)

    def getnear(self, pos):
        key = self.hash(pos)
        near = []
        for i in range(key[0] - 1, key[0] + 2):
            for j in range(key[1] - 1, key[1] + 2):
                for k in range(key[2] - 1, key[2] + 2):
                    near.extend(self.grid[(i, j, k)])
        return near


class pid:  # 离散PID控制
    def __init__(self, dt, kp, ki, kd, limit=(-1, 1)):
        """
        dt:float,时间步长
        kp:float,比例增益,纠正强度
        ki:float,积分增益,惩罚累积误差
        kd:float,微分增益,提前刹车
        limit:tuple,输出限幅
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.limit = limit
        self.error = self.summa = self.diff = 0

    def update(self, error):
        # error:float,当前误差,期望为0
        a = self.dt / (0.1 + self.dt)  # 1阶低通滤波
        diff = (1 - a) * self.diff + a * (error - self.error) / self.dt
        self.error = error
        out = self.kp * error + self.ki * self.summa + self.kd * diff
        clip = np.clip(out, *self.limit)
        # 未饱和/误差试图退出饱和
        if np.sign(out - clip) != np.sign(error):
            if self.ki:
                self.summa += error * self.dt
                m, M = self.limit
                self.summa = np.clip(self.summa, m / self.ki, M / self.ki)  # 积分抗饱和
            self.diff = diff  # 微分抗饱和
        return clip


class model:
    ID = 1

    def __init__(self, root):
        # root:str,模型根目录,None则jsbsim
        base = type(self).mro()[-2]  # 避免子类ID副本
        self.ID = base.ID
        base.ID += 1
        self.fdm = FGFDMExec(root)
        self.fdm.set_debug_level(0)
        self.Longitude = self.Latitude = 0.0  # 经纬度(rad)
        self.Altitude = 0.0  # 高度(m)
        self.Yaw = self.Pitch = self.Roll = 0.0  # 航向,俯仰,滚转角(rad)
        self.p = self.q = self.r = 0.0  # 滚转,俯仰,偏航角速度(rad/s)
        self.vNED = np.zeros(3)  # NED系速度向量(m/s)
        self.aNED = np.zeros(3)  # NED系运动加速度向量(m/s^2)
        self.axyz = np.zeros(3)  # 体轴加速度向量(m/s^2)
        self.RM = np.zeros((3, 3))  # 体轴转NED的旋转矩阵
        self.Mach_M = 0.0  # 马赫
        self.fuel = 0.0  # 燃油(lbs)

    def rotate(self, flush=False):
        if flush:
            sy, sp, sr = map(np.sin, (self.Yaw, self.Pitch, self.Roll))
            cy, cp, cr = map(np.cos, (self.Yaw, self.Pitch, self.Roll))
            # 绕z轴转Yaw
            Rz = np.array(((cy, sy, 0), (-sy, cy, 0), (0, 0, 1)))
            # 绕y轴转Pitch
            Ry = np.array(((cp, 0, -sp), (0, 1, 0), (sp, 0, cp)))
            # 绕x轴转Roll
            Rx = np.array(((1, 0, 0), (0, cr, sr), (0, -sr, cr)))
            self.RM = Rz @ Ry @ Rx
        return self.RM

    def update(self):
        # 位置
        self.Latitude = self.fdm["position/lat-gc-rad"]
        self.Longitude = self.fdm["position/long-gc-rad"]
        self.Altitude = ft2m(self.fdm["position/h-sl-ft"])
        # 姿态
        self.Yaw = const(self.fdm["attitude/psi-rad"])
        self.Pitch = self.fdm["attitude/theta-rad"]
        self.Roll = self.fdm["attitude/phi-rad"]
        self.p = self.fdm["velocities/p-rad_sec"]
        self.q = self.fdm["velocities/q-rad_sec"]
        self.r = self.fdm["velocities/r-rad_sec"]
        # 速度
        self.vNED[0] = ft2m(self.fdm["velocities/v-north-fps"])
        self.vNED[1] = ft2m(self.fdm["velocities/v-east-fps"])
        self.vNED[2] = ft2m(self.fdm["velocities/v-down-fps"])
        self.Mach_M = self.fdm["velocities/mach"]
        self.fuel = self.fdm["propulsion/total-fuel-lbs"]
        # 加速度
        self.axyz[0] = ft2m(self.fdm["accelerations/a-pilot-x-ft_sec2"])
        self.axyz[1] = ft2m(self.fdm["accelerations/a-pilot-y-ft_sec2"])
        self.axyz[2] = ft2m(self.fdm["accelerations/a-pilot-z-ft_sec2"])
        self.aNED = self.rotate(True) @ self.axyz

    def ready(self, lat, lon, alt, mach, head, pitch, roll, fuel):
        """
        lat:float,初始纬度(deg)
        lon:初始经度
        alt:float,初始高度(m)
        mach:float,初始马赫
        head:float,初始方位角(deg)
        pitch:初始俯仰角
        roll:初始滚转角
        fuel:float,初始燃油(lbs)
        """
        if not self.fdm.get_model_name():
            raise AttributeError(f"{type(self)}.__init__未加载模型")
        self.fdm.set_dt(0.01)
        self.fdm["ic/lat-gc-deg"] = lat
        self.fdm["ic/long-gc-deg"] = lon
        self.fdm["ic/h-sl-ft"] = m2ft(alt)
        self.fdm["ic/mach"] = mach
        self.fdm["ic/psi-true-deg"] = head
        self.fdm["ic/theta-deg"] = pitch
        self.fdm["ic/phi-deg"] = roll
        self.fdm["propulsion/tank/contents-lbs"] = fuel
        self.fdm.run_ic()
        self.fdm["gear/gear-cmd-norm"] = 0  # 收起起落架
        self.fdm["propulsion/engine/set-running"] = 1  # 启动引擎
        self.update()

    def step(self, aileron, elevator, throttle=1):
        """
        aileron:float,归一化副翼(-1~1)
        elevator:归一化升降舵,+上-下
        throttle:float,归一化油门(0~1)
        """
        self.fdm["fcs/aileron-cmd-norm"] = np.clip(aileron, -1, 1)
        self.fdm["fcs/elevator-cmd-norm"] = np.clip(-elevator, -1, 1)  # 实则+下-上
        self.fdm["fcs/throttle-cmd-norm"] = np.clip(throttle, 0, 1)
        self.fdm["fcs/rudder-cmd-norm"] = 0
        # 自动起落架,导弹无效
        gear = self.Altitude < ft2m(1500) and self.Mach_M < 0.5
        if gear != round(self.fdm["gear/gear-pos-norm"]):
            self.fdm["gear/gear-cmd-norm"] = gear
        self.fdm.run()
        self.update()

    def geo2NED(self, lat, lon, alt):  # 经纬高转NED向量(m)
        R = 6371e3 + self.Altitude
        r = R * np.cos(self.Latitude)
        lat -= self.Latitude
        lon -= self.Longitude
        D = self.Altitude - alt
        return np.array((R * lat, r * lon, D))

    def NED2geo(self, vector):  # NED向量转经纬高
        R = 6371e3 + self.Altitude
        r = R * np.cos(self.Latitude)
        lat = self.Latitude + vector[0] / R
        lon = self.Longitude + vector[1] / r
        alt = self.Altitude - vector[2]
        return lat, lon, alt

    def los2NED(self, yaw, pitch, dis):  # 视线角转NED向量
        yaw += self.Yaw
        pitch += self.Pitch
        N = np.cos(pitch) * np.cos(yaw)
        E = np.cos(pitch) * np.sin(yaw)
        D = -np.sin(pitch)
        return dis * np.array((N, E, D))

    def NED2los(self, vector):  # NED向量转视线角(rad)
        dis = float(np.linalg.norm(vector))
        if dis < 1:  # 避免近距离角度突变
            return 0, 0, 1
        yaw = const(np.arctan2(vector[1], vector[0]) - self.Yaw)
        pitch = np.arcsin(-vector[2] / dis) - self.Pitch
        return yaw, pitch, dis

    def los2geo(self, yaw, pitch, dis):
        return self.NED2geo(self.los2NED(yaw, pitch, dis))

    def geo2los(self, lat, lon, alt):
        return self.NED2los(self.geo2NED(lat, lon, alt))

    def find(self, target, pos=np.zeros(3)):
        """
        target:model,目标实例
        pos:np.ndarray,相对位置NED向量
        """
        if not pos.any():
            pos = self.geo2NED(target.Latitude, target.Longitude, target.Altitude)
        # 距离,速度标准差
        std = sigma(np.linalg.norm(pos))
        # 带噪距离
        pos = np.random.normal(pos, std[0])
        info = EnemyInfo(target.ID)
        # 带噪速度
        info.vNED = np.random.normal(target.vNED, std[1])
        # 带噪声计算其他
        info.TargetYaw, info.TargetPitch, info.TargetDis = self.NED2los(pos)  # 极坐标
        # 径向相对速度
        info.DisRate = np.dot(info.vNED - self.vNED, pos) / info.TargetDis
        # 测量马赫
        info.TargetMach_M = float(np.linalg.norm(info.vNED)) / Vs
        return info, pos, std
