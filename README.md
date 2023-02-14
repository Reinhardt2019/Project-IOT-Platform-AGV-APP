# Project-IOT-Platform-AGV-APP： Auto-Move 

该分支在Project-IOT-Platform-AGV-APP/application/scripts/AGV_APP/templates/APP/上传了基于Axure的Demo搭建的一部分前端网页，目前仅制作了登录页及下单页的雷达图实时显示，后续开发过程中，需要按照Project-IOT-Platform-AGV-APP/application/scripts/AGV_APP/templates路径下的网页，将网页中的表单与flask默认的表单输入方式进行匹配，并把其他功能陆续实现。

## 雷达实时显示功能需要打开rosbridge并运行docker

#### ssh登录小车
```
ssh -Y wheeltec@10.20.240.247
```
#### 打开rosbridge
```
roslaunch rosbridge_server rosbridge_websocket.launch
```
### 第一次使用时需要运行webviz对应docker镜像

#### 运行docker（如果在虚拟机安装）
https://blog.csdn.net/weixin_43134049/article/details/124476759

```
sudo docker run -p 8080:8080 cruise/webviz
```

显示docker进程

```
sudo docker ps
```

关闭docker
```
sudo docker stop +CONTAINER ID
```
#### 运行docker（如果在win安装）
按照GitHub指南安装（https://github.com/cruise-automation/webviz）

## 参考资料 (很重要很重要很重要)

1.  [Flask Document](https://flask.palletsprojects.com/en/2.1.x/)
2.  [Flask页面蓝图](https://flask.palletsprojects.com/en/2.1.x/tutorial/views/)
3. 项目文件架构: [ROS架构](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem), [Flask Application Factory](https://flask.palletsprojects.com/en/2.1.x/patterns/appfactories/)
4. [Flask 菜鸟教程](https://www.cainiaojc.com/flask/flask-tutorial.html)
5. [Flask-Security](https://pythonhosted.org/Flask-Security/index.html)
6. 数据库：[SQLAlchemy](https://docs.sqlalchemy.org/en/20/index.html), [Flask-SQLAlchemy](https://flask-sqlalchemy.palletsprojects.com/en/2.x/)
7. 相关教程：[login](https://hackersandslackers.com/flask-login-user-authentication/)

## 运行APP
### 安装ROS 
参照ROS教程
### 创建workspace

```
mkdir -p Project-IOT-Platform-AGV-APP/src
cd Project-IOT-Platform-AGV-APP/src
git clone git@github.com:Reinhardt2019/Project-IOT-Platform-AGV-APP.git
```
### 安装Python依赖包
```
cd Project-IOT-Platform-AGV-APP/application/scripts
python3 setup.py build
python3 setup.py install
```
### 编译ROS（虚拟机端）
```
cd Project-IOT-Platform-AGV-APP
catkin_make
. devel/setup.bash
```
## 运行APP
### 运行Flask与ROS
运行APP时，先ssh登录小车，在AGV端输入如下指令打开AGV端服务器
```
roslaunch delivery_navigation delivery.launch 
```
再使用继续在虚拟机端运行
```
roslaunch application start_server.launch
```
### 单独运行Flask
若想在本地单独运行Flask，建议使用Ubuntu中的pycharm
需要注释程序中与ROS相关的代码，如下：

#### __init__.py:

line 11：
```
from application.srv import *
import rospy
import threading
from std_msgs.msg import String
```
line 30：
```
threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
service = rospy.ServiceProxy('delivery', ClientPose)
```
#### order.py:

line 8：
```
from . import order_datastore, merchandise_manager, service, position_manager中去掉service
```
运行wsgi.py
```
python3 run wsgi.py
```

 
## 整体软件框架
![structure](https://user-images.githubusercontent.com/49314691/161005721-c77ac9d7-cbef-4246-b065-da58469952b2.PNG)

## 数据库设计
文件运行后会自动连接AGV_APP/db.sqlite3数据库，若在该路径下无此文件，会自动创建，路径配置在config.py文件。

由于时间有限，目前配置为当前虚拟机的绝对路径，若用其他虚拟机运行需要更改路径，后续开发过程中可以尝试改成相对路径

运行程序会对其初始化，初始化详细内容见models.py

需要注册账号：
[lucidchart](https://lucid.app/lucidchart/d2323f50-abc5-4ffb-a262-2baed325204b/edit?invitationId=inv_c0acda29-5079-4a02-9d8d-55d0ff28b3d4)

## [软件设计文档](https://siemens.sharepoint.com/:w:/r/teams/IOTPlatformAGVRobot/Shared%20Documents/01%20Docs%EF%BC%88All%EF%BC%89/Phase%2002-Output%2001-AGV%20APP%20(All)/01%20Implement%20Docs%20(All)/03%20Design/02%20SWD/20220208%20QiaoGuanlun%20SunQingyi%20SWD.docx?d=w196f71135a534d619e2721d5624b0532&csf=1&web=1&e=bGq8bd)

## 软件功能
### 已开发功能
#### class UML
![UML](https://user-images.githubusercontent.com/49314691/161005683-c9bf8b03-e9e2-42f0-af02-a79801b80e3c.PNG)

相关文件集合在utils文件夹中，代码注释中已标注功能及参数类型。开发以上utils类的目的是为程序提供数据库接口，避免在app代码中重复使用底层函数，如`model.query.filter_by().first()`等。在后续开发中，如遇到类似场景，应考虑开发新的类集成datastore，或在已有的类中添加新的函数，避免代码重复。
### 未开发功能

![future plan](https://user-images.githubusercontent.com/49314691/161005735-6fc1f120-2b35-4626-bf0e-faac8243a999.PNG)
及SWD中提及的其他功能

### 注意
1.小车刚开机时需要一段时间初始化并连接网络，出现ssh: connect to host 10.20.240.247 port 22: No route to host属于正常现象，等待即可

2.只有在数据库order表中订单completed后，才可下达下一个订单
