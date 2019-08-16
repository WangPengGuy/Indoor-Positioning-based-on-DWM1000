# Indoor-Positioning-based-on-DWM1000

## 1. 整体描述

​        这是一个**室内定位+健康监护平台**项目，硬件采用 *爱尔兰Decawave* 公司的DWM1000芯片来进行室内定位，采用进口或国产的生物传感器芯片进行人体生理参数测量，主控芯片采用 *意法半导体公司* 的STM32芯片。软件部分全部采用C语言编写（包括驱动和应用）。

​        该项目的使用场景是**医院**或**大型商场**等场景。

​        暂时只有室内定位部分代码与原理。

## 2. 原理描述

### 室内定位

+ 一个房间内的网络结构：

  ![一个房间内定位网络](F:\研究生科研项目\UWB-IP&WHMN_Github\assets\一个房间内定位网络.jpg)

+ 多个房间内的网络结构

  ![1565940210447](C:\Users\Mr_wang\AppData\Roaming\Typora\typora-user-images\1565940210447.png)



+ 消息发送过程示意图：使用串口的DMA打印消息的话，**测距约20ms/次，发送消息--返回应答约10ms/次。**

  ![POLL ](file:///C:/Users/Mr_wang/AppData/Local/Temp/msohtmlclip1/01/clip_image001.png)

## 3. 文件夹说明

- `UWB_IP&WHMN_Hardware` 包含了室内定位的硬件设计。
- `UWB_IP&WHMN_Software` 包含了 STM32 的 keil 软件程序。