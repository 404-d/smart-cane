# 树莓派安装 WiringPi(GPIO)

1. 将树莓派连接到互联网，打开终端
2. 更新软件列表：  
   `sudo apt-get update`
3. 安装 Git：  
   `sudo apt-get install git`
4. 克隆 WiringPi 库：  
   `sudo git clone https://github.com/WiringPi/WiringPi.git`
5. 进入 WiringPi 目录：  
   `cd WiringPi/`
6. 构建和安装 WiringPi：  
   `sudo ./build`
7. 安装完成，查看 WiringPi 版本：  
   `gpio -v`  
   测试：  
   `gpio readall`
