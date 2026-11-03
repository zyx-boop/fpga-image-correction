# 厂商IP依赖与工程恢复

PDS工程入口为`ImageCorrection/ImageCorrection/implementation/work/ImageCorrection.pds`，顶层为`top`，器件为Logos2 PG2L100H-6FBG676，工具版本为2022.2-SP6.4。

## IP实例

| 实例名称 | PDS IP类别 | 恢复目录 |
|---|---|---|
| `pll` | PLL | `ImageCorrection/ImageCorrection/ipcore/pll/` |
| `ddr3_test` | DDR3 Interface / Logos2 HMIC_S | `ImageCorrection/ImageCorrection/ipcore/ddr3_test/` |
| `rd_fram_buf` | DRM Based Simple Dual Port RAM | `ImageCorrection/ImageCorrection/ipcore/rd_fram_buf/` |
| `wr_fram_buf` | DRM Based Simple Dual Port RAM | `ImageCorrection/ImageCorrection/ipcore/wr_fram_buf/` |

`vendor_ip_parameters.json`记录了原四个IDF配置中的参数键、值、器件信息和端口信息。使用获得授权的PDS IP Compiler，按照这些记录配置同名实例，并把生成结果放在上表目录。参数记录中的RAM配置头部包含原工程的封装元数据，恢复时需要同时核对主工程的FBG676器件设置与实际端口配置。

PLL、DDR3控制器、厂商RAM封装、加密Verilog以及厂商仿真模型需要由使用者从获授权的PDS安装或开发板资料中取得。工程文件中的相应路径在依赖恢复后解析。

## 构建

1. 安装并配置获授权的Pango Design Suite 2022.2-SP6.4。
2. 在IP Compiler中恢复四个同名实例，核对参数、DDR管脚和端口。
3. 打开工程入口，核对顶层`top`、全部源码以及`source/top.fdc`。
4. 从Compile开始重新执行实现流程，检查各阶段日志与时序结果。
5. 使用与实际输入对应的实现数据库生成位流，并在目标硬件上验证。

公开文件保留了原PDS工程内容。原工程中记录的历史阶段状态需要结合本机重新构建的真实产物判断。构建依赖恢复和下载后的硬件表现需要在使用者环境中验证。

## 公开内容和本地内容

公开内容包括自研RTL、上位机、约束、查找表、原PDS工程和IP配置参数记录。厂商IP实现、仿真模型、示例工程、派生网表、实现数据库、构建缓存和位流按厂商依赖处理。本次发布的来源范围与文件校验值保存在公开清单中。
