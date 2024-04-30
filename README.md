# 能量机关识别方案
- [能量机关识别方案](#能量机关识别方案)
  - [灯条分类](#灯条分类)
  - [确定未激活扇叶的短边直线的角点组合](#确定未激活扇叶的短边直线的角点组合)
  - [判断圆形灯条是R标还是靶标内环](#判断圆形灯条是r标还是靶标内环)
    - [1 过一个点做一条直线的垂线](#1-过一个点做一条直线的垂线)
    - [2 一条直线经过一条线段](#2-一条直线经过一条线段)
  - [确定待激活扇叶上的打击位置](#确定待激活扇叶上的打击位置)
    - [1 仿射变换](#1-仿射变换)
    - [2 开闭运算，得到中心](#2-开闭运算得到中心)
- [能量机关预测方案](#能量机关预测方案)
  - [数据采集](#数据采集)
  - [预测目标](#预测目标)
    - [方案1：拟合速度函数](#方案1拟合速度函数)
    - [方案2：拟合位置函数](#方案2拟合位置函数)


## 灯条分类
按形状可以初步分为以下几种灯条类型
1. `圆形灯条` 
   - 种类：激活的靶标内环、R标
   - 特征：长宽比<1.1
2. `大型灯条`
   - 种类：激活准度高的扇叶根部
   - 特征：
     - 长宽比 = 1.25 &plusmn; 0.1
3. `小型灯条`
   - 种类：激活准度高的扇叶顶部
   - 特征：
     - 长宽比 = 2.7 &plusmn; 0.1
4. `巨型灯条`
   - 种类：待激活扇叶、激活准度低的扇叶
   - 特征：面积最大，长宽比在一定范围内
     <!-- - 短边 = 120 &plusmn; 10
     - 长边 = 230 &plusmn; 10
       - 缩放比例 = 1 &plusmn; 0.2 -->
     - 长宽比 = 1.85 &plusmn; 0.2

按照其功能可以进一步细分为以下几种灯条
1. `R标`
   - 用途：用于确定旋转圆心和进行PnP测距
   - `圆形灯条` 特征：参见[如何判断圆形灯条是R标还是靶标内环](#如何判断圆形灯条是r标还是靶标内环)
2. `未激活`
   - 用途：
   - `巨型灯条` 特征：轮廓与外接矩形面积比例<0.6
3. `已激活`
   - 用途：
   - `巨型灯条` 特征：轮廓与外接矩形面积比例>0.6
4. `靶标内环`
   - 用途：用于辅助确定靶标圆心位置
5. `不合法`

## 确定未激活扇叶的短边直线的角点组合
扇叶最终是利用 OpenCV 中的 cv::RotatedRect 数据结构进行描述。过获得旋转矩形的四个角点确定两两相邻的点属于长边还是短边。由于四个角点是按照逆时针连续编号的，我们可以得到以下结论：
- 0,1点间距离 > 1,2点间距离 : 短边1的组合是 1,2点，短边2的组合是 0,3点
- 0,1点间距离 < 1,2点间距离 : 短边1的组合是 0,1点，短边2的组合是 2,3点

## 判断圆形灯条是R标还是靶标内环
### 1 过一个点做一条直线的垂线
**已知：** 直线 $l_1$ 过两点 $(x_1,y_1)$ 和 $(x_2,y_2)$，其垂线 $l_0$ 经过的点为 $P(x_0,y_0)$

**设：** 直线 $l_1$ 的方向向量 $\overrightarrow{m}$；直线 $l_0$ 的方向向量 $\overrightarrow{n}$（即直线 $l_1$ 的法向向量）

易得

$$\overrightarrow{m}=(x_2-x_1,y_2-y_1)$$

由解析几何知识：直线 $Ax + By + C = 0$ 的方向向量为 $\overrightarrow{s} = \plusmn (B,-A)$

$$可得
\left \{
   \begin{array}{l}
      A_1 = -(y_2-y_1) \\
      B_1 = x_2-x_1 \\
      C_1 =  - (A_1x_1 + B_1y_1)
   \end{array}
\right .
$$

$$
l_1 : A_1x + B_1y + C_1 = 0
$$


由解析几何知识：直线 $Ax + By + C = 0$ 的法向量为 $\overrightarrow{s} = (A,B)$

$$可得\overrightarrow{n}=(-(y_2-y_1),x_2-x_1)$$

由解析几何知识同理得

$$
\left \{
   \begin{array}{l}
      A_0 = x_2-x_1 \\
      B_0 = y_2-y_1 \\
      C_0 =  - (A_0x_0 + B_0y_0)
   \end{array}
\right .
$$

$$
则 l_0 : A_0x + B_0y + C_0 = 0
$$

### 2 一条直线经过一条线段
**已知：** 线段 $l_2$ 有两顶点 $(x_3,y_3)$ 和 $(x_4,y_4)$，且分别位于直线 $l_0 ：A_0x + B_0y + C_0 = 0$ 两侧

令函数
$$
F(x,y)=A_0x + B_0y + C_0
$$

由直线的性质可得 $F(x_3,y_3) \cdot F(x_4,y_4)<0$

## 确定待激活扇叶上的打击位置
### 1 仿射变换
将二值图像中属于待击打扇叶的旋转矩形的区域做仿射变换，变换成 150x75 大小的图像。仿射变换选取 4 个角点，与变换后 4 点位置的对应如下图：

![warp](attachment/warp.png)

### 2 开闭运算，得到中心

在仿射变换后的图像，上下左右右侧添加空白，防止后续形态学操作越界而使中心偏离

- 首先进行闭运算，将扇叶的中心区域填充；

- 之后进行开运算，将扇臂区域过滤掉，只保留待击打区域。

<center>

<img src="attachment/ori_roi.png" width=30%>
<img src="attachment/after_close.png" width=30%>
<img src="attachment/after_open.png" width=30%>

从左到右依次为：仿射变换后的原始图像（添加空白后的）、闭运算后、开运算后


</center>

- 之后可以得到待击打区域的中心在仿射变换后的图像中的位置。

- 再然后对这个中心点做逆变换，得到该点在原图中的位置。

# 能量机关预测方案

## 数据采集
- 采集的数据：`时间戳`、`待打击点角位置`、`旋转速度`

- 优化点
  1. 优化滤波方式，不要每添加一个数据就对所有数据进行滤波，太消耗算力了

## 预测目标
### 方案1：拟合速度函数
1. 数学推导

   根据规则手册，速度目标函数为 $spd = a\cdot\sin(\omega\cdot(t+c))+b$

   其中 $a\in[0.780,1.045],b=2.09-a,c\in\R,\omega\in[1.884,2.000]$ 

   积分可得 $t_0$ 到 $t_1$ 时刻位置变化量为 $\Delta pos = (-\frac{a}{\omega}\cos(\omega\cdot(t+c))+bt)|_{t_0}^{t_1}$

2. 拟合函数

   当采集到150帧数据后开始拟合速度函数。

3. 预测位置

预测效果如下图：

![Predict](attachment/FittedSpeed.png)

![Predict](attachment/TargetPos.png)

可以看见曲线基本重合，准确度还是比较高的

### 方案2：拟合位置函数

TBD...