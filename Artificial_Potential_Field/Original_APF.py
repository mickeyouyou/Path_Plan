"""
人工势场寻路算法实现
最基本的人工势场，存在目标点不可达及局部最小点问题
"""
import signal
import math
import random
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
import time
import numpy as np

LANE_WIDTH = 0.4  # meter
VEHICLE_WIDTH = 0.21  # meter


class Vector2d():
    """
    2维向量, 支持加减, 支持常量乘法(右乘)
    """

    def __init__(self, x, y):
        self.deltaX = x
        self.deltaY = y
        self.length = -1
        self.direction = [0, 0]
        self.vector2d_share()

    def vector2d_share(self):
        if type(self.deltaX) == type(list()) and type(self.deltaY) == type(list()):
            deltaX, deltaY = self.deltaX, self.deltaY
            self.deltaX = deltaY[0] - deltaX[0]
            self.deltaY = deltaY[1] - deltaX[1]
        self.length = math.sqrt(self.deltaX ** 2 + self.deltaY ** 2) * 1.0
        if self.length > 0:
            self.direction = [self.deltaX /
                              self.length, self.deltaY / self.length]
        else:
            self.direction = None

    def __add__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX += other.deltaX
        vec.deltaY += other.deltaY
        vec.vector2d_share()
        return vec

    def __sub__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX -= other.deltaX
        vec.deltaY -= other.deltaY
        vec.vector2d_share()
        return vec

    def __mul__(self, other):
        vec = Vector2d(self.deltaX, self.deltaY)
        vec.deltaX *= other
        vec.deltaY *= other
        vec.vector2d_share()
        return vec

    def __truediv__(self, other):
        return self.__mul__(1.0 / other)

    def __repr__(self):
        return 'Vector deltaX:{}, deltaY:{}, length:{}, direction:{}'.format(self.deltaX, self.deltaY, self.length,
                                                                             self.direction)


class APF():
    """
    人工势场寻路
    """

    def __init__(self, start: (), goal: (), obstacles: [], k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int, goal_threshold: float, is_plot=False):
        """
        :param start: 起点
        :param goal: 终点
        :param obstacles: 障碍物列表，每个元素为Vector2d对象
        :param k_att: 引力系数
        :param k_rep: 斥力系数
        :param rr: 斥力作用范围
        :param step_size: 步长
        :param max_iters: 最大迭代次数
        :param goal_threshold: 离目标点小于此值即认为到达目标点
        :param is_plot: 是否绘图
        """
        self.start = Vector2d(start[0], start[1])
        self.current_pos = Vector2d(start[0], start[1])
        self.goal = Vector2d(goal[0], goal[1])
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr  # 斥力作用范围
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.goal_threashold = goal_threshold
        self.path = list()
        self.is_path_plan_success = False
        self.is_plot = is_plot
        self.delta_t = 0.001

    def in_bound(self):

        # 构建车道的边界，直道给出四个点，圆弧TODO
        if(self.current_pos.deltaY > 1.7 or self.current_pos.deltaY < 1.3 or self.current_pos.deltaX < 1.0 or self.current_pos.deltaX > 3.0):
            return False

        return True

    def point_in_bound(self, point):
        if(point.deltaY > 1.7 or point.deltaY < 1.3 or point.deltaX < 1.0 or point.deltaX > 3.0):
            return False
        return True

    def attractive(self):
        """
        引力计算
        :return: 引力
        """
        # 圆锥计算方式 attr = eq * k_attr
        att = (self.goal - self.current_pos) * self.k_att  # 方向由机器人指向目标点
        # 抛物面结算方式 attr = 0.5 * eq * eq * k_att
        # diff = (self.goal - self.current_pos)
        # value = diff.length * diff.length
        # att = value * self.k_att * 0.5
        return att

    def repulsion(self):
        """
        斥力计算
        :return: 斥力大小
        """
        rep = Vector2d(0, 0)  # 所有障碍物总斥力
        for obstacle in self.obstacles:
            t_vec = self.current_pos - obstacle
            # print('current_pos:{} to obstacle: {} vector:{}'.format(
            # self.current_pos, obstacle, t_vec))
            # if self.point_in_bound(t_vec):
            # 超出障碍物斥力影响范围
            if (t_vec.length > self.rr):
                pass
            else:
                # 方向由障碍物指向机器人
                obstacle_potential = Vector2d(t_vec.direction[0], t_vec.direction[1]) * 0.5 * (
                    1.0 / t_vec.length - 1.0 / self.rr) / (t_vec.length ** 2)
                print('obstacle potential is {}'.format(obstacle_potential))
                rep += obstacle_potential

        # 道路边界作为斥力加入计算
        # road = [[1.0, 1.3], [3.0, 1.7], [1.0, 1.7], [3.0, 1.7]]
        # for i in np.arange(1.0, 3.0, self.step_size):
        #     # print('i is :{}'.format(i))
        #     # TODO 撒点排查当前点
        #     left_bound = Vector2d(i, 1.7)
        #     right_bound = Vector2d(i, 1.3)

        #     lb_vector = left_bound - self.current_pos
        #     rb_vector = right_bound - self.current_pos

        #     left_potential = Vector2d(lb_vector.direction[0], lb_vector.direction[1]) * \
        #         0.5 * (1 / (lb_vector.length - VEHICLE_WIDTH / 2)) ** 2
        #     right_potential = Vector2d(rb_vector.direction[0], rb_vector.direction[1]) * \
        #         0.5 * (1 / (rb_vector.length - VEHICLE_WIDTH / 2)) ** 2

        #     # print('left point : {} ,left_potential: {}'.format(i, left_potential))
        #     # print('right point :{} right_potential:{}'.format(i, right_potential))

        #     rep += left_potential
        #     rep += right_potential

        # else:
        #     # 方向指向沿目标点的道路方向
        #     rep += Vector2d(new_vec.direction[0], new_vec.direction[1]) * self.k_rep * (
        #         1.0 / new_vec.length - 1.0 / self.rr) / (new_vec.length ** 2)

        # 道路边界斥力势场数学模型
        # if self.in_bound():
            # rep += Vector2d(diff_vec.direction[0], diff_vec.direction[1]) * \
            # self.k_rep * (1 / (diff_vec.length - 0.21 / 2)) ** 2

        return rep

    def path_plan(self):
        while (self.iters < self.max_iters and (self.current_pos - self.goal).length > self.goal_threashold):
            attractive = self.attractive()
            print('attractive : {}'.format(attractive))
            repulsion = self.repulsion()
            print('repulsion: {}'.format(repulsion))
            f_vec = self.attractive() + self.repulsion()
            print('combined force:{}'.format(f_vec))
            # 下一个点的选择
            self.current_pos += Vector2d(
                f_vec.direction[0], f_vec.direction[1]) * self.step_size
            print('current_position:{}'.format(self.current_pos))
            # exit(0)
            self.iters += 1
            self.path.append(
                [self.current_pos.deltaX, self.current_pos.deltaY])
            if self.is_plot:
                plt.plot(self.current_pos.deltaX,
                         self.current_pos.deltaY, '.r')
                plt.pause(self.delta_t)
        if (self.current_pos - self.goal).length <= self.goal_threashold:
            self.is_path_plan_success = True

    def plot(self):
        fig = plt.figure()
        ax = Axes3D(fig)
        X = np.arange(1.0, 3.0, self.step_size)
        Y = np.arange(1.3, 1.7, self.step_size)
        X, Y = np.meshgrid(X, Y)
        # R = np.sqrt(X**2 + Y**2)
        # Z = np.arange(1000, -1000)
        Z = np.sqrt(X**2 + Y**2)

        # 具体函数方法可用 help(function) 查看，如：help(ax.plot_surface)
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='rainbow')
        plt.show()


def handler(signum, frame):
    print('You choose to stop me.')


signal.signal(signal.SIGINT, handler)
signal.signal(signal.SIGTERM, handler)


if __name__ == '__main__':
    # 相关参数设置
    k_att, k_rep = 1.0, 100.0
    rr = 0.1
    step_size, max_iters, goal_threashold = 0.05, 100, 0.05
    # 步长0.5寻路1000次用时4.37s, 步长0.1寻路1000次用时21s
    step_size_ = 2

    # 设置、绘制起点终点
    start, goal = (1, 1.5), (3.0, 1.5)
    print("start point is : 1, 1,5")
    print("goal point is 3.0, 1.5")
    is_plot = True
    if is_plot:
        fig = plt.figure(figsize=(8, 6))
        subplot = fig.add_subplot(111)
        subplot.set_xlabel('X-distance: m')
        subplot.set_ylabel('Y-distance: m')
        subplot.plot(start[0], start[1], '*y')
        subplot.plot(goal[0], goal[1], '*r')

    # 车道绘制
    stright_path_length = 2
    path_width = 0.4
    subplot.add_patch(
        patches.Rectangle(
            (1, 1.3),   # (x,y)
            stright_path_length,
            path_width,
        )
    )
    subplot.add_patch(
        patches.Rectangle(
            (1, 2.3),   # (x,y)
            stright_path_length,
            path_width,
        )
    )

    left_wedge = patches.Wedge((1, 2), .7, 90, -90, width=path_width)
    right_wedge = patches.Wedge((3, 2), .7, -90, 90, width=path_width)
    # subplot.add_patch(left_wedge)
    subplot.add_patch(right_wedge)

    subplot.grid(True, linestyle="-.", color="r", linewidth=0.1)

    # 障碍物设置及绘制
    obs = [[1.5, 1.5]]
    print('obstacles: {0}'.format(obs))

    if is_plot:
        for OB in obs:
            circle = Circle(xy=(OB[0], OB[1]), radius=rr,
                            alpha=0.3, color='coral')
            subplot.add_patch(circle)
            subplot.plot(OB[0], OB[1], 'xk', color='coral')
    # t1 = time.time()
    # for i in range(1000):
    # plt.show()

    # path plan
    apf = APF(start, goal, obs, k_att, k_rep, rr, step_size,
              max_iters, goal_threashold, is_plot)
    apf.path_plan()
    apf.plot()
    # plt.show()
    if apf.is_path_plan_success:
        path = apf.path
        # path_ = []
        # i = int(step_size_ / step_size)
        # while (i < len(path)):
        #     path_.append(path[i])
        #     i += int(step_size_ / step_size)

        # if path_[-1] != path[-1]:
        #     path_.append(path[-1])
        # print('planed path points:{}'.format(path_))
        print('path plan success')
        if is_plot:
            px, py = [K[0] for K in path], [K[1] for K in path]
            subplot.plot(px, py, '>b')
            plt.show()
    else:
        print('path plan failed')
    # t2 = time.time()
    # print('寻路1000次所用时间:{}, 寻路1次所用时间:{}'.format(t2-t1, (t2-t1)/1000))
