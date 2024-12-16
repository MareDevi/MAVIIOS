## 本项目用于BUPT公选课期末报告

# 简介
项目使用rust与slintui进行开发，界面部分参考并魔改了slintui官方的printerdemo与orbiterdemo

本项目只是一次在rp2040上使用rust与slintui的一次尝试~~就是写着玩的~~

# 环境搭建

1. 确保已经安装 rust 嵌入式开发工具链。
2. 克隆该仓库：
3. 在桌面上运行（模拟器）
    ```
    cargo run --features simulator
    ```
4. 在设备上运行

   a. 安装cargo扩展以创建RP2040 USB引导加载程序的UF2镜像
      ```
      cargo install elf2uf2-rs --locked
      ```

   b. 在设备上运行
      ```
      # If you're on Linux: mount the device
      udisksctl mount -b /dev/sda1
      cargo run --target=thumbv6m-none-eabi --features=pico --release
      ```
# 运行效果
![2024-12-16-135515_hyprshot.png](https://s2.loli.net/2024/12/16/jxXQkr8PycWqAwK.png) ![2024-12-16-135537_hyprshot.png](https://s2.loli.net/2024/12/16/wqZf6rXIP1msbCl.png) ![2024-12-16-135447_hyprshot.png](https://s2.loli.net/2024/12/16/6z1bU4kPQMnNaTG.png)