# eye to hand calibration
将标定板固定在机械臂末端，通过机械臂正运动学可以计算出机器人坐标系到机械臂末端的齐次变换矩阵rMe；通过摄像头可以计算出标定板的位姿cMo；移动机械臂末端，采取多组rMe、cMo；根据多组rMe、cMo可计算出机器人坐标系到相机坐标系的齐次变换矩阵rMc。

具体操作：
* Step 1: 搭建硬件环境
> 1. 将board文件夹下的marker制作出来，然后固定在机械臂末端
> 2. 将相机固定在工作台。

* Step 2：记录cMo，rMe

运行以下命令：
```bash
./detect_markers -c=camParam.yml -d=10 --ci=0 -l=0.16 -s
```
上述命令将读取camParam.yml中摄像头内参矩阵r_intin，畸变参数r_coeffs，识别字典类型为10，外框长16cm的aruco marker；每按一次s键就保存当前cMo到cMo.txt，同时应该人工记录当前机器人末端位姿rMe到rMe.txt。

* Step 3：计算rMc

修改cMo.txt、wMe.txt，在文件首行写入"type nums"，其中type = 0表示数据是齐次变换矩阵形式，type = 1是XYZRPY形式(单位为mm和度)；nums表示个数。
运行以下命令，结果将保存到hand_eye.yml:
```bash
./hand_eye
```

# eye in hand calibration
将标定板固定在地面不动，多次移动机械臂，记录机械臂末端姿态rMe和标定板在相机坐标系下的姿态cMo；根据多组rMe、cMo可计算出机械臂末端到相机坐标系的齐次变换矩阵eMc。

具体操作和eye to hand calibration类似，只需将最后一步改成
```bash
./hand_eye --eye_in_hand=true
```
