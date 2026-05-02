import multiprocessing as mu
from .drone import *
from .tacview import tacview


def inradar(entity: drone | missile, info: EnemyInfo):
    return (  # 角度条件
        np.cos(entity.radarAngle) < np.cos(info.TargetYaw) * np.cos(info.TargetPitch)
        and info.TargetDis < entity.radarR  # 距离条件
    )


class bvrsim:
    def __init__(
        self,
        field=((23, 26), (118, 120), (2e3, 15e3)),  # tuple,战场纬,经,高度范围(deg,m)
        threat=((24.5, 119, 0, 5e4),),  # tuple,红方威胁区中心纬,经度(deg),高度,半径(m)
    ):
        self.field = np.vstack((np.deg2rad(field[:2]), field[2]))
        # 战场中心(原点)
        self.center = np.mean(self.field, 1)
        self.threat = [np.append(np.deg2rad(x[:3]), x[3]) for x in threat]
        self.red = [
            dict(lat=25.8, lon=118.2, alt=1e4, head=180, mach=0.8, num=4, fuel=5e3),
            dict(lat=25.8, lon=118.4, alt=1e4, head=180, mach=0.8, num=4, fuel=5e3),
            dict(lat=25.8, lon=118.6, alt=1e4, head=180, mach=0.8, num=4, fuel=5e3),
            dict(lat=25.8, lon=118.8, alt=1e4, head=180, mach=0.8, num=4, fuel=5e3),
        ]  # 红方初始条件
        self.blue = [
            dict(lat=23.2, lon=118.2, alt=1e4, head=0, mach=0.8, num=6, fuel=5e3),
            dict(lat=23.2, lon=118.4, alt=1e4, head=0, mach=0.8, num=6, fuel=5e3),
            dict(lat=23.2, lon=118.6, alt=1e4, head=0, mach=0.8, num=6, fuel=5e3),
            dict(lat=23.2, lon=118.8, alt=1e4, head=0, mach=0.8, num=6, fuel=5e3),
        ]  # 蓝方初始条件

    def redstrategy(self, info: DroneInfo, step_num):  # 红方策略函数
        cmd = SendData()
        cmd.CmdSpd = 2
        cmd.CmdAlt = 13e3
        expect = 23.5, 119
        if 24 < info.Latitude:
            if info.DroneID == 1:
                expect = [24, 118.15]
            elif info.DroneID == 2:
                expect = [24, 118.35]
            elif info.DroneID == 3:
                expect = [24, 119.65]
            else:
                expect = [24, 119.85]
            expect[0] += 25 < info.Latitude
        R = 6371e3 + info.Altitude
        r = R * np.cos(info.Latitude)
        expect = np.deg2rad(expect) - (info.Latitude, info.Longitude)
        expect *= (R, r)
        cmd.CmdHeadingDeg = np.rad2deg(np.arctan2(expect[1], expect[0]))
        cmd.EnemyID = info.DroneID + 4
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

    def bluestrategy(self, info: DroneInfo, step_num):  # 蓝方策略函数
        cmd = SendData()
        cmd.CmdSpd = 1.2
        cmd.CmdAlt = 11e3
        cmd.EnemyID = info.DroneID - 4
        enemy = next((x for x in info.FoundEnemyList if x.EnemyID == cmd.EnemyID), None)
        if not step_num % 100:
            cmd.engage = -1
        elif (
            step_num % 100 == 50
            and enemy
            and enemy.TargetDis < 5e4  # 保证发射距离
            and 1 < np.rad2deg(info.Pitch) < 5  # 保证发射仰角
        ):
            cmd.engage = 1
        return cmd

    def redconsumer(self, get: mu.Queue, put: mu.Queue):  # 红方消费者
        while True:
            data = get.get()
            if not data:
                break
            put.put((data[0].DroneID, self.redstrategy(*data)))

    def blueconsumer(self, get: mu.Queue, put: mu.Queue):  # 蓝方消费者
        while True:
            data = get.get()
            if not data:
                break
            put.put((data[0].DroneID, self.bluestrategy(*data)))

    def restrict(self):  # 检查是否有效
        dead = set()
        for ID, (entity, color) in self.entity.items():
            if isinstance(entity, drone) and not entity.fuel:  # 飞机无燃料
                dead.add(ID)
                continue
            for (m, M), x in zip(
                self.field, (entity.Latitude, entity.Longitude, entity.Altitude)
            ):
                if x < m or M < x:  # 飞出战场
                    dead.add(ID)
                    break
            if "Red" == color and not ID in dead:
                for lat, lon, alt, r in self.threat:
                    # 是否飞入威胁区
                    if np.linalg.norm(entity.geo2NED(lat, lon, alt)) < r:
                        dead.add(ID)
                        break
        return dead

    def strike(self):  # 检查是否碰撞
        grid = spatialgrid(15)
        pos = {}  # 实体绝对坐标
        for ID, (entity, _) in self.entity.items():
            pos[ID] = -model.geo2NED(entity, *self.center)
            grid.add(ID, pos[ID])
        near = {}  # 邻近点缓存
        dis = defaultdict(dict)
        dead = set()
        for ID, (entity, _) in self.entity.items():
            key = grid.hash(pos[ID])
            if not key in near:
                near[key] = grid.getnear(pos[ID])
            for x in near[key]:
                if ID != x:
                    if not x in dis[ID]:
                        dis[ID][x] = dis[x][ID] = np.linalg.norm(pos[x] - pos[ID])
                    if dis[ID][x] < grid.size:
                        dead.add(x)
                        dead.add(ID)
                        if isinstance(entity, missile):  # 可能误截获
                            entity.TargetID = x
        return dead, pos

    def main(self, time=30, exist_ok=False):  # 仿真主函数
        """
        time:float,仿真最大分钟
        exist_ok:bool,是否覆盖最近acmi文件
        """
        queue = [mu.Queue() for _ in range(3)]
        # 红方沙盒
        mu.Process(target=self.redconsumer, args=queue[::2], daemon=True).start()
        # 蓝方沙盒
        mu.Process(target=self.blueconsumer, args=queue[1:], daemon=True).start()
        log = tacview("tacview", exist_ok)
        log.logtime(0)
        exist = {"Red": len(self.red), "Blue": len(self.blue)}
        self.entity = {}
        # 初始化红方飞机
        for entity in self.red:
            entity = f16(**entity)
            log.loginit(entity, "Red")
            self.entity[entity.ID] = entity, "Red"
        # 初始化蓝方飞机
        for entity in self.blue:
            entity = f16(**entity)
            entity.radarR *= 4 / 3  # 蓝方雷达距离优势
            log.loginit(entity, "Blue")
            self.entity[entity.ID] = entity, "Blue"
        # 仿真步长
        dt = next(iter(self.entity.values()))[0].fdm.get_delta_t()
        # 主循环
        for step_num, time in enumerate(np.arange(0, 60 * time, dt)):
            log.logtime(time)
            for ID in self.restrict():
                entity, color = self.entity.pop(ID)
                print(time, ":", vars(entity), "飞入禁区")
                if color in exist:
                    exist[color] -= 1
                else:
                    entity.TargetID = None  # 导弹失效
                log.logdestroy(ID)
            dead, pos = self.strike()
            for ID in dead:
                entity, color = self.entity.pop(ID)
                print(time, ":", vars(entity), "受撞")
                if color in exist:
                    exist[color] -= 1
                else:
                    entity.state = 2  # 导弹击中
                log.logdestroy(ID)
            if not all(exist.values()):
                break
            if not step_num % 3:  # 雷达延迟
                for ID1, (entity, _) in self.entity.items():
                    if isinstance(entity, drone):
                        isNTS = {x.EnemyID: x.isNTS for x in entity.FoundEnemyList}
                        entity.FoundEnemyList.clear()
                        entity.AlarmList.clear()
                        for ID2, (target, _) in self.entity.items():
                            if ID1 != ID2:
                                info, rel, std = model.find(
                                    entity, target, pos[ID2] - pos[ID1]
                                )
                                if info.TargetDis < 2e4:  # 告警
                                    entity.alarm(
                                        ID2, info.TargetYaw, type(target).__name__
                                    )
                                if isinstance(target, drone) and inradar(entity, info):
                                    # 雷达发现敌机
                                    entity.find(info, rel, std, isNTS.get(ID2, False))
                    elif entity.TargetID in self.entity:
                        ID2 = entity.TargetID
                        target = self.entity[ID2][0]
                        info, rel, std = model.find(entity, target, pos[ID2] - pos[ID1])
                        if inradar(entity, info):
                            entity.find(info, rel, std)
                            log.logNTS(ID1, ID2)
            for entity, color in self.entity.values():  # 无人机信息发送
                if isinstance(entity, drone):
                    i = next(i for i, x in enumerate(exist) if x == color)
                    queue[i].put((entity.getinfo(), step_num))
            mislist = []  # 新发射导弹列表
            for entity, _ in self.entity.values():
                if isinstance(entity, missile):
                    old = entity.state
                    entity.step()
                    if not old and entity.state:
                        log.loginit(entity, "White", "AIM-120C")  # 主动雷达开启
                else:
                    ID, cmd = queue[2].get()
                    entity = self.entity[ID][0]  # 实际飞机
                    entity.step(cmd)
                    for enemy in entity.FoundEnemyList:
                        if enemy.isNTS:
                            log.logNTS(ID, enemy.EnemyID)  # 持续显示锁定
                    if 1 == cmd.engage and MissileTrackList[ID]:
                        # 最新发射导弹
                        M = MissileTrackList[ID][-1]
                        if not M.ID in self.entity and M.TargetID and M.state < 2:
                            mislist.append(M)
                            print(time, ":", vars(entity), "发射", vars(M))
                            log.loginit(M, "White", "AIM-120C", 0)
                log.logstep(entity)
            for M in mislist:
                self.entity[M.ID] = M, "White"
        for q in queue[:-1]:
            q.put(None)
        print("仿真秒数:", time, "\n无人机剩余:", exist, "\n数据文件:", log.file.name)
