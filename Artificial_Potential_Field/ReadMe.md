# 人工势场寻路算法

## Original Artificial Potential Field Method


最基本的人工势场方法，会出现局部最小值问题导致寻路失败，此外还存在不可达问题（如下图）
![](./Original-APF-Path-Plan.png)

加了道路边界斥力约束，改进了斥力系数方程，算是理论上可用。

![](./basedonroad.png.png)
