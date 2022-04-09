# 基于多智能体协调的车路协同关键技术研究
## 搭载有多智能体算法的一个城市规模交通信号控制仿真平台
### 设计思路:
1. 搭建模拟平台：在现有 CityFlow 项目的基础上，优化代码，使用 C++语言模拟车辆的移动、停止、转弯、遵守交通信号灯等一系列车辆在现实世界的
交通场景中可能会出现的动作，为用户提供方便的获取交通信息接口,控制信号灯API接口。
2. 测试模拟平台：调用已搭建平台的接口，模拟当前主流的交通控制算法，如max-pressure算法，同时与前端协作，实时渲染模拟2D车辆运动，
为用户提供直观的车流感受和精准的车流信息。
3. 构建算法模型：对现有的多Agent算法进行总结优化，提出更优的调度算法并搭载在了我们的系统中。  

### 目前存在问题
1. 重叠问题： 当多条lane进入同一条lane时，可能会因为拥堵产生重叠问题。  
解决思路：
    - 后端优化碰撞检测逻辑(完成）
    - 后端增加waitingBuffer功能， 当车道车辆已满时， 将新增车辆存入waitingBuffer中(完成)
    - 后端与前端优化车辆渲染与车辆坐标问题。(待完成)
![重叠问题](https://github.com/wyyadd/SEUTraffic/blob/main/img/overlap.png)
2. 跨道问题： 在十字路口处， 若laneLink与lane重叠且信号灯能同时放行的话， 直行的车与拐弯的车会发生重叠现象，不会触发碰撞检测。
解决思路：  
    - 后端增加Drivable重叠检测判断(已解决)
    
![跨道问题解决1](https://github.com/wyyadd/SEUTraffic/blob/main/img/cross_solve1.png)
![跨道问题解决2](https://github.com/wyyadd/SEUTraffic/blob/main/img/cross_solve2.png)

### 效果展示
![效果展示](https://github.com/wyyadd/SEUTraffic/blob/main/img/demo1.gif)
![数据展示](https://github.com/wyyadd/SEUTraffic/blob/main/img/demo1.png)
