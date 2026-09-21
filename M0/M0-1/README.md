# M0-1 从零搭建开发环境

## 1. 任务目标

本任务完成智能车后续开发所需的基础环境搭建与验证，包括：

- Ubuntu 22.04
- ROS2 Humble
- Python 虚拟环境
- C/C++ 工具链
- VS Code 与开发插件
- Git / GitHub
- SSH 远程登录
- scp / rsync 文件传输
- SSH Key 免密认证

同时记录真实安装过程、验证方法和实际遇到的问题。

---

## 2. 本机环境

本机采用 Windows + Ubuntu 双系统，实际开发在 Ubuntu 下进行。

系统信息：

```text
Distributor ID: Ubuntu
Description: Ubuntu 22.04.5 LTS
Release: 22.04
Codename: jammy
```

已验证的主要工具：

| 工具 | 版本 / 状态 |
|---|---|
| Ubuntu | 22.04.5 LTS |
| ROS2 | Humble |
| Python | 3.10.12 |
| uv | 0.12.17 |
| gcc | 11.4.0 |
| g++ | 11.4.0 |
| GNU Make | 4.3 |
| CMake | 3.22.1 |
| Git | 2.34.1 |
| OpenSSH | 8.9p1 |
| VS Code | 1.138.0 x64 |

### 为什么选择双系统 Ubuntu

ROS2 Humble 官方支持 Ubuntu 22.04。相比 WSL2 或虚拟机，双系统 Ubuntu 直接运行在真实 Linux 环境中，后续连接串口、USB、CAN、摄像头等硬件时通常更直接，也更接近开发板上的实际运行环境。

缺点是 Windows 和 Ubuntu 之间切换需要重启，并且安装双系统时需要处理磁盘分区和启动项。

WSL2 的优点是和 Windows 共存方便，但部分硬件直通、网络配置可能需要额外处理；虚拟机隔离性好、方便做快照，但会有一定性能开销，并且 USB / 串口等硬件通常需要额外配置直通。

---

## 3. 基础开发工具检查

使用以下命令检查本机已有开发工具：

```bash
git --version
gcc --version
g++ --version
make --version
cmake --version
python3 --version
ssh -V
```

本机 gcc、g++、make、cmake、Git、Python 和 OpenSSH 均已可以正常使用，因此没有重复安装。

Git 用户信息配置为：

```bash
git config --global user.name "adtyuhd"
git config --global user.email "leilei_8@foxmail.com"
```

验证：

```bash
git config --global user.name
git config --global user.email
```

---

## 4. ROS2 Humble

### 4.1 配置软件源并安装

Ubuntu 22.04 对应 ROS2 Humble。

启用 Universe：

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

安装必要工具：

```bash
sudo apt install curl
sudo apt install gnupg2
```

之后安装官方 `ros2-apt-source` 软件源包，并执行：

```bash
sudo apt update
```

更新时可以看到 ROS2 软件源：

```text
http://packages.ros.org/ros2/ubuntu jammy
```

安装 ROS2 Humble Desktop：

```bash
sudo apt install ros-humble-desktop
```

### 4.2 加载环境

```bash
source /opt/ros/humble/setup.bash
```

验证：

```bash
echo $ROS_DISTRO
which ros2
```

输出：

```text
humble
/opt/ros/humble/bin/ros2
```

为了让新终端自动加载 ROS2，将下面一行加入 `~/.bashrc`：

```bash
source /opt/ros/humble/setup.bash
```

重新打开终端后再次验证：

```bash
echo $ROS_DISTRO
which ros2
```

仍然输出 Humble 和正确的 ros2 路径。

### 4.3 Talker / Listener 通信验证

终端 1：

```bash
ros2 run demo_nodes_cpp talker
```

终端 2：

```bash
ros2 run demo_nodes_py listener
```

Talker 持续发布 `Hello World`，Listener 可以接收到对应消息，说明 ROS2 基础通信链路正常。

---

## 5. Python 虚拟环境：uv

本项目选择 `uv` 管理 Python 虚拟环境。

选择原因：Ubuntu 22.04 和 ROS2 Humble 已经使用系统 Python / apt 管理 ROS2 相关环境，目前自己的 Python 代码只需要轻量级依赖隔离，因此使用 `uv + .venv` 比较直接。如果后续项目出现复杂科学计算、CUDA 或 Conda 专用包，再考虑 Conda。

安装：

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

验证：

```bash
uv --version
```

输出：

```text
uv 0.12.17
```

创建项目虚拟环境：

```bash
cd ~/22SmartCar
uv venv .venv
```

创建结果：

```text
Using CPython 3.10.12 interpreter at: /usr/bin/python3
Creating virtual environment at: .venv
Activate with: source .venv/bin/activate
```

激活：

```bash
source .venv/bin/activate
```

验证：

```bash
which python
python --version
uv --version
```

其中 Python 路径为：

```text
/home/adtyuhd/22SmartCar/.venv/bin/python
```

说明当前使用的是项目独立虚拟环境。

---

## 6. VS Code

VS Code 使用 Snap 安装：

```bash
sudo snap install code --classic
```

验证：

```bash
code --version
```

输出：

```text
1.138.0
7debcd0e2acdea1c52de81bf9ee1620444407dda
x64
```

安装的插件：

- Python（Microsoft）
- C/C++（Microsoft）
- Remote Development（Microsoft）

最终通过 VS Code 图形界面的 Extensions 页面安装成功。

打开项目：

```bash
cd ~/22SmartCar
code .
```

---

## 7. Git 项目与 GitHub

项目结构：

```text
22SmartCar/
├── M0/
│   ├── M0-1/
│   ├── M0-2/
│   ├── M0-3/
│   └── M0-4/
└── README.md
```

### 7.1 初始化本地仓库

最开始直接执行：

```bash
git status
```

出现：

```text
fatal: not a git repository (or any of the parent directories): .git
```

原因是当前目录还没有初始化 Git 仓库。

执行：

```bash
git init
```

之后 `git status` 可以正常工作。

### 7.2 .gitignore

项目中使用：

```gitignore
# Python virtual environment
.venv/

# Python cache
__pycache__/
*.pyc

# ROS2 / colcon
build/
install/
log/

# VS Code local settings
.vscode/
```

避免把虚拟环境、Python 缓存、ROS2 构建产物和 VS Code 本地配置提交到仓库。

### 7.3 第一次提交

```bash
git add .
git commit -m "chore: initialize project structure"
```

第一次提交：

```text
09320f1 chore: initialize project structure
```

### 7.4 GitHub 远程仓库

生成 SSH Key：

```bash
ssh-keygen -t ed25519 -C "leilei_8@foxmail.com"
```

生成：

```text
~/.ssh/id_ed25519
~/.ssh/id_ed25519.pub
```

其中：

- `id_ed25519`：私钥，只保存在本机
- `id_ed25519.pub`：公钥，可以添加到 GitHub 或远程服务器

将公钥添加到 GitHub 后验证：

```bash
ssh -T git@github.com
```

第一次连接确认 GitHub 主机指纹后，输出：

```text
Hi adtyuhd! You've successfully authenticated, but GitHub does not provide shell access.
```


添加远程仓库并推送：

```bash
git remote add origin git@github.com:adtyuhd/22SmartCar.git
git branch -M main
git push -u origin main
```

本地 `main` 分支成功关联 `origin/main`。

---

## 8. SSH 远程登录、scp 与 rsync

### 8.1 基本概念

三种常用工具：

```text
ssh    -> 登录并操作远程电脑
scp    -> 复制文件
rsync  -> 同步文件和目录
```

本机检查结果：

```bash
command -v ssh
command -v scp
command -v rsync
```

输出：

```text
/usr/bin/ssh
/usr/bin/scp
/usr/bin/rsync
```

假设未来开发板信息为：

```text
用户名：ubuntu
IP：192.168.1.50
```

登录：

```bash
ssh ubuntu@192.168.1.50
```

登录成功后，终端中运行的命令实际由远程开发板执行。

退出：

```bash
exit
```

### 8.2 SSH Key 免密登录

将本机公钥复制到远程服务器：

```bash
ssh-copy-id ubuntu@192.168.1.50
```

本质上是把本机：

```text
~/.ssh/id_ed25519.pub
```

加入远程用户的：

```text
~/.ssh/authorized_keys
```

之后远程服务器可以通过公钥验证客户端是否持有对应私钥，从而不需要每次输入账户密码。

私钥 `id_ed25519` 不能发送给其他人。

### 8.3 authorized_keys 权限

常见安全权限：

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/authorized_keys
```

如果 `authorized_keys` 可以被其他用户随意修改，SSH 可能认为该文件不安全并拒绝使用它进行认证。

当公钥认证没有成功时，SSH 还可能继续尝试密码认证，因此会重新出现：

```text
ubuntu@192.168.1.50's password:
```

排查认证过程：

```bash
ssh -v ubuntu@192.168.1.50
```

### 8.4 scp 示例

把本机文件传到开发板：

```bash
scp ~/22SmartCar/test.py ubuntu@192.168.1.50:/home/ubuntu/
```

`scp` 是复制，本地文件不会消失。

### 8.5 rsync 示例

同步整个项目：

```bash
rsync -av ~/22SmartCar/ ubuntu@192.168.1.50:/home/ubuntu/22SmartCar/
```

`rsync` 会比较源目录和目标目录，后续再次同步时主要传输发生变化的内容，适合开发过程中频繁同步代码。

---

## 9. SSH 实际验证

目前手边没有真实开发板或服务器，因此使用本机运行的 OpenSSH Server 完成了 SSH / scp / rsync 的完整流程练习。

检查 SSH Server：

```bash
systemctl status ssh
```

状态：

```text
Active: active (running)
```

第一次密码登录：

```bash
ssh localhost
```

成功进入本机 SSH Server。

配置公钥：

```bash
ssh-copy-id adtyuhd@localhost
```

输出：

```text
Number of key(s) added: 1
```

再次登录：

```bash
ssh adtyuhd@localhost
```

不再要求输入账户密码，说明 SSH Key 认证成功。

### scp 实际测试

创建测试文件：

```bash
echo "M0-1 scp test" > M0/M0-1/scp_test.txt
```

传输：

```bash
scp M0/M0-1/scp_test.txt adtyuhd@localhost:/tmp/
```

验证：

```bash
ssh adtyuhd@localhost 'cat /tmp/scp_test.txt'
```

输出：

```text
M0-1 scp test
```

说明 scp 文件传输成功。

### rsync 实际测试

```bash
rsync -av M0/M0-1/ adtyuhd@localhost:/tmp/M0-1-rsync/
```

输出：

```text
sending incremental file list
created directory /tmp/M0-1-rsync
./
README.md
scp_test.txt

sent 210 bytes  received 95 bytes  610.00 bytes/sec
total size is 14  speedup is 0.05
```

说明 rsync 工作正常。


---

## 10. 实际踩坑与排查

### 10.1 Git 仓库未初始化

现象：

```bash
git status
```

报错：

```text
fatal: not a git repository (or any of the parent directories): .git
```

原因：当前目录还没有执行 `git init`。

解决：

```bash
git init
```


### 10.2 Python 命令多输入字符

曾误输入：

```bash
python3 --version~
```

出现：

```text
unknown option --version~
usage: python3 [option] ... [-c cmd | -m mod | file | -] [arg] ...
Try `python -h' for more information.
```

实际原因不是 Python 安装错误，而是命令末尾多输入了 `~`。

正确命令：

```bash
python3 --version
```

说明排查环境问题时，应先检查命令本身是否输入正确。

### 10.3 VS Code CLI 安装插件出现 BAD_DECRYPT

使用：

```bash
code --install-extension ms-python.python
```

安装 Python 插件时出现：

```text
OPENSSL_internal:BAD_DECRYPT
```

随后使用：

```bash
code --install-extension ms-vscode.cpptools --verbose
```

测试 C/C++ 插件，仍然在下载 VSIX 包阶段出现：

```text
Cipher functions:OPENSSL_internal:BAD_DECRYPT
```

因此问题并不只发生在 Python 插件。

进一步检查：

```bash
which code
snap list code
openssl version
```

结果表明当前使用 Snap 版 VS Code，系统 OpenSSL 为 3.0.2。

同时检查环境变量发现本机配置了代理：

```text
http_proxy=http://127.0.0.1:7897/
https_proxy=http://127.0.0.1:7897/
ALL_PROXY=socks://127.0.0.1:7897/
```

详细日志中 Marketplace 元数据请求可以返回 HTTP 200，但下载扩展 VSIX 时反复出现 `BAD_DECRYPT`。

由于当时没有足够证据把故障完全归因于代理、Snap 或 OpenSSL，因此没有直接修改系统 OpenSSL，也没有立即重装 VS Code。

最后通过 VS Code 图形界面的 Extensions 页面安装：

- Python
- C/C++
- Remote Development

均成功。

这次排查过程采用了：

```text
一个插件失败
    ↓
换另一个插件做对照
    ↓
使用 --verbose 确认失败阶段
    ↓
检查 VS Code 安装方式、OpenSSL、网络和代理
    ↓
避免在证据不足时直接修改系统环境
```

---

## 11. 环境验证

Ubuntu：

```bash
lsb_release -a
```

ROS2：

```bash
echo $ROS_DISTRO
which ros2
```

Python / uv：

```bash
python3 --version
uv --version
```

虚拟环境：

```bash
cd ~/22SmartCar
source .venv/bin/activate
which python
```

C/C++：

```bash
gcc --version
g++ --version
make --version
cmake --version
```

Git：

```bash
git --version
git config --global user.name
git config --global user.email
git remote -v
```

SSH：

```bash
ssh -V
command -v scp
command -v rsync
```

VS Code：

```bash
code --version
```

---

## 12. 参考资料

- ROS2 Humble Ubuntu 安装：
  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- ROS apt source：
  https://github.com/ros-infrastructure/ros-apt-source
- uv：
  https://docs.astral.sh/uv/
- VS Code Linux：
  https://code.visualstudio.com/docs/setup/linux
- Git：
  https://git-scm.com/docs
- OpenSSH：
  https://www.openssh.com/
