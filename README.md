# WUWU库使用介绍

## 结构说明
```
├── WuwuSama_Icar_Project/           # 应用层开源库  
    ├── cross_lib/  
        ├── json                     # jsoncpp动态库  
        ├── ncnn                     # ncnn静态库  
        ├── opencv                   # opencv动态库  
    ├── smartCar/  
        ├── code                     # 用户代码添加在此处  
        ├── include                  # 开源库头文件路径  
        ├── wuwu_library             # 开源库源文件路径  
        main.cc                      # 程序入口.cpp文件  
    CMakeLists.txt                   # 项目CMake文件  
```


## 如何使用
- 在 __WuwuSama_Icar_Project__ 路径下输入
    >`cmake -B build`  
- 进入 __build__ 目录
    > `cd build`
- 多核编译 -jN _根据虚拟机分配的CPU数量填写_ 
    > `make -j12`
- 如何使用 __根据启动方式__
    1. (可选)网络启动
        > `cp main ~/linux/nfs/lsrootfs/root/`
    2. (可选)EMMC启动 _192.168.x.x_ 填写久久派IP
        > `scp main root@192.168.x.x:/root`  
---

## WUWU库资源

- __ww_brushless.cc/.h__ 无刷电调驱动  

- __ww_buzzer.cc/.h__ 无源蜂鸣器驱动  
    > 参考../example/buzzer_main.cc

- __ww_icm42688.cc/.h__ ICM42688六轴姿态传感器驱动
    > 参考../example/icm42688_main.cc

- __ww_key.cc/.h__ 按键驱动

- __ww_lcd.cc/.h__ 显示屏驱动
    > 参考../example/Lcd_main.cc

- __ww_motor.cc/.h__ 电机驱动
    > 参考../example/motor_main.cc  
    > 参考../example/encoder_main.cc

- __ww_timerThread.cc/.h__ 定时器线程
    > 参考../example/buzzer_main.cc
    > 参考../example/icm42688_main.cc

- __ww_vl53l0x.cc/.h__ 激光测距驱动

- __ww_camera_server.cc/.h__ tcp图传
    > 参考../example/camera_server_main.cc

- __ww_tcp_client.cc/.h__ tcp客户端
    > 参考../example/tcp_client_main.cc

- __ww_camera.cc/.h__ 摄像头零拷贝读取
    > 参考../example/camera_zeroCopy_main.cc  
    > 参考../example/camera_save_video_main.cc

