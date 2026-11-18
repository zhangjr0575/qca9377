# ath10Wlan — QCA9377 原生 WiFi 驱动（黑苹果 / OpenCore）

让 Qualcomm Atheros **QCA9377**（`168c:0042`，subsystem `144d:4133`）在 macOS 上
通过 **原生 WiFi 栈**（`IO80211Controller` + net80211）工作，而不是"以太网 + 配套 App"方案。

## 当前状态

| 层 | 状态 |
| --- | --- |
| 硬件层（PCIe / CE / DMA / 复位） | ✅ 已打通 |
| 固件层（board / OTP / WLAN 固件 / HTC READY） | ✅ 已打通 |
| HTC 服务层（HTT / WMI 端点协商） | ✅ 已打通 |
| WMI 层（bootstrap / INIT / vdev） | ✅ 已验证（真实 MAC 已生效） |
| net80211 协议层（扫描 / 连接 / 密钥） | 🔄 驱动侧已能扫到真实 SSID；待接入 net80211 |

**当前 `ath10k-hardware-status` 已验证到 `wmi-ready`**：固件完成 WMI INIT、拿到真实 MAC
`3c:a0:67:c7:91:1f`、创建 STA vdev、建好 HTT RX 环，并且**已扫描到真实 SSID**
（`ChinaNet-F301-5G`、`4F_5G`）。下一步是 M1-c3：把扫描结果接进 net80211，让系统 Wi-Fi 菜单显示网络列表。

## 目录

| 路径 | 说明 |
| --- | --- |
| `itlwm/` | 上游 itlwm 项目 + 本项目新增的 `itlwm/itlwm/hal_ath10k/` |
| `Ath10kMac/` | 参考实现（以太网方案），本项目复用其硬件抽象头文件 |
| `docs/` | 全部设计与验证文档，**从 `docs/09` 开始读** |
| `scripts/` | 构建验证脚本（只读） |
| `build/` | 各里程碑构建产物 |

## 快速开始

**编译**

```sh
cd itlwm
xcodebuild -project itlwm.xcodeproj -scheme AirportItlwm-Sequoia \
  -configuration Release ARCHS=x86_64 CODE_SIGNING_ALLOWED=NO \
  -derivedDataPath /tmp/itlwm-dd build
```

**离线自测（不需要真机）**

```sh
clang++ -std=c++14 -O0 -I itlwm/itlwm/hal_ath10k/hw \
        tests/wmi-tlv-selftest.cpp -framework IOKit -o /tmp/wmi-self && /tmp/wmi-self
```

**验证（真机，需先部署到 EFI）**

```sh
sleep 45 && sh scripts/verify-m1b2.sh   # M1-b2 + M1-c1（WMI INIT / vdev / MAC / 扫描）
sleep 30 && sh scripts/verify-m0.1c.sh  # M0.1c 全量基线
```

## 必读文档

1. [`docs/09-M1-后续计划与工程交接.md`](docs/09-M1-后续计划与工程交接.md) —— **接手必读**：现状、架构、约束、后续计划
2. [`docs/07-M0.1c-移植规格.md`](docs/07-M0.1c-移植规格.md) —— 19 个阶段的行号级移植规格
3. [`docs/08-M0.1c-完成报告.md`](docs/08-M0.1c-完成报告.md) —— 固件成功启动的验收数据
4. [`docs/10-M1-b2-实现说明.md`](docs/10-M1-b2-实现说明.md) —— WMI INIT / host memory / vdev create 的帧格式与失败定位
5. [`docs/11-M1-c1-WMI扫描通路.md`](docs/11-M1-c1-WMI扫描通路.md) —— 扫描命令帧、信道策略与扫描结果捕获
6. [`docs/01-调研报告.md`](docs/01-调研报告.md) —— 最初的可行性调研与路线对比

## 关键约束（血泪教训）

- 日志必须用 `IOLog`，不能用 itlwm 的 `XYLog`（后者不进 unified log）
- `hi_board_data_initialized` 是 **host 主动写入**的，不是等 ROM 置位
- `SOC_CHIP_ID` 必须**冷复位后**才可信
- 复用 Ath10kMac 头文件时注意宏污染：`hz` 在 itlwm 里被 `#define hz 100` 污染
- `BMI_DONE` 不可逆，发送前必须关闭 BMI

## 许可证

- itlwm：GPL-2.0
- Ath10kMac：GPL-3.0
- QCA9377 固件/board data 受 Qualcomm Atheros 独立条款约束（见 `Ath10kMac/firmware/LICENSE.QualcommAtheros_ath10k`）

合并产物按 GPL-3.0 分发。