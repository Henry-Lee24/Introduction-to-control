# SJTU-AU2301控制导论

#### 前言

顺便记录一下第一次用GitHub上传，详细内容请见master分支。

该库主要基于sjtu自动化系专业课AU2301控制导论，该课程主要基于TORCS平台，完成包括巡线竞速、巡线精度、泊车实验，我们小组在此课程中取得了不错的成绩，故将此开源，其中：

*巡线竞速Rank 10； 巡线精度Rank 2； 泊车Rank 1.*

#### 实验平台

采用VS 2019 为编译环境，编程语言以C语言为主

库中分别有包含破解版和加密版的实验平台的文件夹：

```
CyberTORCS_2022Spring_V2.0_Publish
```

```
CyberTORCS_Key
```

注：加密版需要助教提供的硬盘锁解锁

**此平台由上海交通大学自动化系智能车实验室杨明老师及其助教团搭建，请勿盗用，违者必究**

#### 巡线竞速实验代码

```
driver_cruise\driver_cruise\driver_cuise1
```

#### 巡线精度实验代码

```
driver_cruise\driver_cruise\driver_cuise2
```

#### 泊车实验代码

```
driver_parking\driver_parking
```

#### 操作方法

在VS 2019 中用Realse模式运行，会在对应实验文件夹下的Release文件夹中生成dll文件，将此dll文件复制到实验平台的driver_cruise文件夹下即可。例如放在以下位置

```
CyberTORCS_Key\CyberTORCS_Key\cybercruise
```

**如有侵犯老师和助教的知识结晶，还望谅解，联系我后会立马删除相应部分**

