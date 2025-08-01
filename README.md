# FZX6180MT_FBoot_Debug

1、功能说明

    	1.SPI 主模式测试2401

2、使用环境

        硬件环境：
            1.基于FZX6180MT_V1.0开发板
            2.MCU：FZX6180MT

```
    软件开发环境：KEIL MDK-ARM V5.31.0.0
	编译说明：
```

3、使用说明        

    	描述相关模块配置方法；例如:时钟，I/O等
        SystemClock：64MHz
        GPIO：（主模式 DEMO 板）SPI1: CS--PA0、SCK--PA1、MISO--PA3、MOSI--PA2
    	日志打印：DEMO 板 PB6(TX)，波特率：115200


    	描述Demo的测试步骤和现象：
            1.编译后下载程序复位运行；
            2.接好串口打印工具，上电，查看打印测试写、读0XA0成功；

4、注意事项