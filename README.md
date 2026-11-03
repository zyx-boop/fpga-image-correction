# FPGA图像增强与动态畸变矫正系统

本项目面向紫光同创盘古开发板及Logos2 PG2L100H FPGA，实现OV5640图像采集、外部DDR3帧缓存、图像增强、几何矫正与HDMI输出，并配套Windows串口控制上位机。

源码包含Bayer图像处理、颜色校准与白平衡、自适应Gamma、边缘保持降噪、定点畸变坐标映射、RGB565双线性重采样、JY61姿态接收与动态重投影、帧缓存管理和云台控制模块。具体连接与启用配置以顶层源码和约束文件为准。

## 目录

| 路径 | 内容 |
|---|---|
| `ControlPanel/` | Windows串口控制上位机 |
| `ImageCorrection/ControlPanel/` | 随原工程保存的另一份上位机文件 |
| `ImageCorrection/ImageCorrection/source/` | RTL、参数头文件及引脚/时钟约束 |
| `ImageCorrection/ImageCorrection/data/` | Gamma、亮部保护及行范围初始化表 |
| `ImageCorrection/ImageCorrection/implementation/work/ImageCorrection.pds` | PDS工程入口 |
| `ImageCorrection/ImageCorrection/ipcore/vendor_ip_parameters.json` | 四个厂商IP实例的配置参数与端口记录 |
| `docs/DEPENDENCIES.md` | 厂商IP恢复及工程构建说明 |
| `docs/PUBLICATION_MANIFEST.json` | 公开文件的SHA256及来源记录 |

## 硬件和工具

- FPGA型号为Logos2 PG2L100H，封装FBG676，速度等级-6。
- FPGA工具为Pango Design Suite 2022.2-SP6.4，工程顶层为`top`。
- 相机为OV5640，显示链路为1920×1080图像通路。
- 上位机依赖Python、PySide6-Essentials和pyserial，版本范围见`ControlPanel/requirements.txt`。

## 打开FPGA工程

先按照`docs/DEPENDENCIES.md`在获得授权的PDS环境中恢复PLL、DDR3和帧缓冲RAM四个厂商IP实例，再打开`ImageCorrection.pds`。工程文件保留原目录中的相对路径。完成依赖恢复后，应在本机重新执行Compile、Synthesize、Map、PNR和时序检查，生成对应实现数据库。

工程的管脚分配与接口时钟见`source/top.fdc`。该配置对应特定开发板及其接线，移植时需要结合实际硬件核对。

## 启动上位机

在Windows终端进入`ControlPanel`目录，执行下面的命令。

```powershell
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe -B -X utf8 app.py
```

应用提供串口选择、云台控制、姿态显示和校准参数管理。控制范围及协议以本仓库的`protocol.py`和FPGA控制模块为准。`calibration.json`与`home_calibration.json`保存的是云台参数，相机模型参数位于`source/coordinate/`。

## 评估口径

本仓库发布所提供工程目录中的源码与配置。端到端新帧率、延迟、清晰度、颜色及完整接口时序的验收，应结合对应位流和实际测试条件记录。HDMI时序模块的名称及1920×1080输出格式不能单独代替新帧吞吐的测量。

## 许可证和厂商依赖

项目自研代码与本仓库编写的文档采用MIT许可证，见`LICENSE`。第三方组件的权利归原作者。Pango厂商IP实现受其原许可约束，使用者需在获得授权的工具环境中生成或取得这些依赖，说明见`THIRD_PARTY_NOTICES.md`。
