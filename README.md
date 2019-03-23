# arduino-nrf2401-bootloader

#### 介绍
这是一个专门为arduino UNO/Nano/Pro Mini打造的无线下载bootloader,可通过NRF24L01+模块或串口烧写程序.2 Kb Flash.兼容STK500V2协议,支持avrdude,支持arduino IDE,支持跳频传输,附带编程器端实现代码.     
#### 特点:     
##### 1.  兼容STK500V2下载协议    
实现了协议中的通用命令以及ISP命令子集:  Flash读+写 / EEPROM读+写 / 熔丝位只读. 
通过无线编程器和avrdude通信,完成程序下载.
所谓的无线编程器仅仅是数据透传,相当于无线串口,作用是从PC串口接收一帧命令,不做解析,通过NRF24L01+模块分包发送给bootloader.
bootloader执行命令完毕后,再将命令的ACK帧分包发送给无线编程器,再由串口送达avrdude.
精力有限,只使用avrdude5.10 和 avrdud6.3测试过,不保证其他版本没问题.
5.10是winAVR2010内置avrdude版本;
6.3是当前官方最新版本.
##### 2. 支持两种下载接口: nrf24L01P-on-SPI 或者 串口 
bootloader默认优先使用无线模块接收数据,当检测不到无线模块时,启用串口(115200bps).
只在复位后检测一次,中途不再切换.
##### 3. 支持简易的跳频通信
复位后bootloader在 [默认空中波特率+默认地址+默认频道]上监听握手信号,programmer端将 [新的空中波特率+随机频道+随机地址] 打包进握手信号,握手成功后双方一起修改配置,转移到新的频道继续通信.
跳频命令是基于STK500V2的协议格式实现的,在bootloader看来和其他命令没有任何区别,所以理论上双方可以多次跳频.
#### 软件架构
软件架构说明


#### 安装教程

1. xxxx
2. xxxx
3. xxxx

#### 使用说明

1. xxxx
2. xxxx
3. xxxx

#### 参与贡献

1. Fork 本仓库
2. 新建 Feat_xxx 分支
3. 提交代码
4. 新建 Pull Request


#### 码云特技

1. 使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2. 码云官方博客 [blog.gitee.com](https://blog.gitee.com)
3. 你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解码云上的优秀开源项目
4. [GVP](https://gitee.com/gvp) 全称是码云最有价值开源项目，是码云综合评定出的优秀开源项目
5. 码云官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6. 码云封面人物是一档用来展示码云会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)