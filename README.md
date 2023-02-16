# Project-IOT-Platform-AGV-APP： Auto-Move 

该分支上传了基于Axure的Demo（详见网盘资料）搭建的一部分前端网页，目前仅制作了登录页及下单页的雷达图实时显示。目前前端是基于HTML/CSS搭建，可以使用vs code等软件打开文件夹并运行HTML。后续开发过程中，需要按照Project-IOT-Platform-AGV-APP/application/scripts/AGV_APP/templates路径下的网页，将网页中的表单与flask默认的表单输入方式进行匹配，并且与后端程序连接，把其他功能陆续实现。

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
[相关教程](https://blog.csdn.net/weixin_43134049/article/details/124476759)

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
[按照GitHub指南安装](https://github.com/cruise-automation/webviz)



## 参考资料 (很重要很重要很重要)
导航可视化: [webviz](https://webviz.io/)

