# M1-c1 实现说明：WMI 扫描通路

> 状态：**M1-c1 已完成**（23:30 真机拿到真实 SSID）；扫描命令 + HTT RX + 管理帧解析全链路验证通过
> 最后更新：2026-11-16 23:05
> 代码：`hw/Ath10kWmiScan.hpp`（新增）、`Ath10kHal.cpp`
> 验证：`sh scripts/verify-m1b2.sh`（第 9 段）、`tests/wmi-tlv-selftest.cpp`

---

## 一、目标与范围

M1-c 拆成两步：

| 步骤 | 内容 | 可见成果 |
| --- | --- | --- |
| **M1-c1（本文件）** | PDEV 参数 → SCAN_CHAN_LIST → START_SCAN → 解析扫描事件与 MGMT_RX | `ath10k-scan-ssids` 里出现真实 SSID |
| M1-c2 | 把扫描结果接进 net80211（`ic_scan` / node cache）+ `ath_newstate` 的 SCAN 分支 | 系统 Wi-Fi 菜单出现网络列表 |

M1-c1 只碰 WMI 层，不动 net80211，因此风险低、验证快。

---

## 二、命令序列

在 M1-b2 之后（vdev 已创建）：

```
PDEV_SET_PARAM(param=1 TX chain mask, value=1)   → 28 字节
PDEV_SET_PARAM(param=2 RX chain mask, value=1)   → 28 字节
SCAN_CHAN_LIST（38 个信道）                       → 24 + 28*38 = 1088 字节
START_SCAN（22 个非 DFS 信道）                    → 140 + 4*22 = 228 字节
```

每条命令后都会走 `wmiSubmitAndReclaim()`：提交 → 等 CE3 消费 → 最多 2 s 收事件
把信用要回来（沿用 M1-b2 的 CE1 trailer 机制）。信用不足时状态串会写
`no-credit` / `uncertain-previous`，不会静默失败。

### 2.1 PDEV_SET_PARAM

```
0  endpoint(2)   1  NEED_CREDIT_UPDATE   2-3  len=20   5  seq
8-11  0x4003     12-15 TLV(len=12, tag=82)
16-19 pdev_id=0  20-23 param   24-27 value
```

### 2.2 SCAN_CHAN_LIST

```
8-11  0x3003   12-15 TLV(len=4, tag=79)   16-19 count(38)
20-23 TLV(len=28*38, tag=18)
24+   每个信道 28 字节：
      0-3  TLV(len=24, tag=80)
      4-7  freq          8-11 band_center_freq1   12-15 band_center_freq2=0
      16-19 flags（0x80/0x81 = PASSIVE | 11A/11G）
      20-23 reg_info0    24-27 reg_info1（半 dBm，上限 10 dBm）
```

### 2.3 START_SCAN

```
8-11  0x3001   12-15 TLV(len=100, tag=77)
16-75 common[15] = {scan_id=0xa036, request_id=0xa001, vdev=0, priority=1,
                    event_mask=0x1ff, probe_delay=50, dwell=200, rest=50,
                    max_rest=500, 0,0,0, firmware_ms=30000, 5, flags=0x31}
76-79 burst_duration=0        80-83 num_channels
84-87 num_bssids=1            88-95 num_ssids/ie_len=0
96-99 num_probes=0            100-115 mac/mask=0
116-119 TLV(len=4n, tag=16)   120+ 频率数组
tail    TLV(len=0, tag=19) 空 SSID 数组
        TLV(len=8, tag=19) + 广播 BSSID
        TLV(len=0, tag=17) 空 IE 数组
```

`flags=0x31` = `PASSIVE(0x01) | HIGH_PRIORITY(0x10) | FAST_SCAN(0x20)`：
被动扫描（不主动发 probe request），dwell 200 ms，22 个信道约 4.4 s。

**信道策略**：固件侧信道表给全量 38 个（含 DFS）；START_SCAN 首轮只访问
13 个 2.4G + 9 个非 DFS 5G。DFS 需要被动驻留 500 ms 以上且当前阶段连不上，
留到 M1-c2 之后再按需补。

---

## 三、事件解析

| 事件 | ID | 用途 |
| --- | --- | --- |
| `WMI_TLV_SCAN_EVENTID` | `0x3001` | type=1 开始、type=2 完成（reason=0 成功） |
| `WMI_TLV_MGMT_RX_EVENTID` | `0x7001` | 带回 beacon/probe response 原始帧 |

MGMT_RX 结构：TLV(40, 44) 给出 channel / snr / buf_len / status，
后面的 `ARRAY_BYTE(tag=17)` 是 802.11 帧本体。

从帧里抽：

- BSSID：帧偏移 16
- SSID：偏移 36 起的 IE 里 id=0（长度 ≤ 32）
- 信道：IE id=3（DS Parameter）

RSSI 用 `snr - 95`（与参考实现一致），发布时给出原始 snr。

结果按 BSSID 去重，最多记 16 条。

---

## 四、新增 ioreg 属性

| 属性 | 含义 |
| --- | --- |
| `ath10k-pdev-setup-ok` / `-status` | chain mask 是否下发成功 |
| `ath10k-scan-setup-ok` | 信道表是否下发成功 |
| `ath10k-scan-ok` / `-status` | 扫描是否完成且 reason=0 |
| `ath10k-scan-events` / `-mgmt` | 扫描事件数 / 管理帧数 |
| `ath10k-scan-started` / `-completed` / `-reason` | 扫描进度与完成原因 |
| `ath10k-scan-last-freq` | 最近一次扫描事件的信道频率 |
| `ath10k-scan-wait-ms` | 收集耗时 |
| `ath10k-scan-bss-count` | 去重后的 BSS 数 |
| `ath10k-scan-ssids` | `ssid\|bssid\|freq\|snr`，最多 8 条 |

`ath10k-hardware-status` 仍然是 `wmi-ready`：扫描属于上层功能，失败不会把
硬件基线判死，只会体现在 `ath10k-scan-status`。

---

## 四之二、真机第一轮实测（2026-11-16 23:01，M1c1 构建）

```
pdev-setup-ok    = 1     pdev-setup-status = "chains-1x1"
scan-setup-ok    = 1
scan-ok          = 1     scan-status = "complete"
scan-started     = 1     scan-completed = 1     scan-reason = 0
scan-events      = 47    scan-last-freq = 5825  scan-wait-ms = 3783
scan-mgmt        = 0     scan-bss-count = 0     scan-ssids = "(none)"
```

**结论：WMI 扫描命令链路完全打通**——PDEV 参数、信道表、START_SCAN 全部被固件接受，
扫描在 3.8 s 内跑完 22 个信道并正常回报 STARTED/COMPLETED（reason=0）。

**但一帧管理帧都没收到**（`scan-mgmt = 0`）。这说明固件没有把 beacon/probe response
通过 `WMI_TLV_MGMT_RX_EVENTID(0x7001)` 交给主机。

### 为什么以及下一步

参考实现（Ath10kMac README）在同一台机器上明确表示"能扫描、能连 WPA2、能跑流量"，
它的流程比我们多一步：**INIT 之后、vdev 之前先做 HTT RX setup**（在 CE1 上配置
HTT RX ring、投递 RX buffer）。Linux ath10k 同样在 `ath10k_core_start()` 里先
`ath10k_htt_setup()` 再建 vdev。

据此判断：本固件的接收路径（包括扫描期间的管理帧）被固件内部挂在 HTT RX 环上；
主机没投递 RX ring，固件就不会把收到的帧交上来。这一条也正好是 M1-d（数据面）的前置。

因此本轮先做 A/B 与诊断（M1c1b 构建），确认这个判断后再上 HTT RX：

| 新增诊断 | 判断依据 |
| --- | --- |
| `ath10k-scan-ev-bss-chan` | >0 表示固件在信道上**确实发现了 BSS**（扫描无线电正常） |
| `ath10k-scan-ev-foreign-chan` / `-other` | 扫描事件类型分布 |
| `ath10k-ce1-frames` / `-last-endpoint` / `-last-bytes` | CE1 上是否出现 HTT（endpoint 1）帧 |

同时把两处配置改成更接近 Linux 的做法：

1. **信道表不再把所有信道标 PASSIVE**：只有 DFS 标被动，非 DFS 允许主动扫描
   （否则固件会把整张表当被动信道，START_SCAN 的 PASSIVE 位就失去意义）；
2. **START_SCAN 改成主动扫描**（flags=0x30，发 probe request），比被动驻留更容易
   拿到帧，也便于区分"没帧"与"被动没收到"。

判读：

- `scan-ev-bss-chan > 0` 且 `ce1-frames = 0`、`scan-mgmt = 0`
  → 固件看见了 BSS 但不交帧，**必须实现 HTT RX**（M1-c2 的主体工作）；
- `ce1-frames > 0`（endpoint=1）
  → 帧其实在 CE1 上，HTT 分发是缺口，同样是 M1-c2；
- `scan-ev-bss-chan = 0`
  → 扫描本身没看到任何 BSS，先查信道/功率配置与射频状态。

---

## 四之三、真机第二轮（2026-11-16 23:11，M1c1b 构建）：确认缺 HTT RX

```
scan-ev-bss-chan     = 23      ← 固件在 23 个信道上发现了 BSS
scan-ev-foreign-chan = 22      ← 22 个信道都扫到了
scan-mgmt            = 0
ce1-frames           = 0       ← CE1 上也没有帧
```

主动扫描 + 非被动信道表没有改变结果，说明问题不在扫描本身，而在**接收通路上没有
主机侧的 HTT RX 环**。固件把扫描结果留在自己的缓存里，没有把管理帧交给主机。

### 已实现（M1c2 构建）

按参考实现在同一颗芯片上验证过的顺序移植了 HTT RX setup（`hw/Ath10kHtt.hpp` +
`runHTTRxSetup()`），放在 **PDEV 参数之后、vdev 之前**（与 Linux
`ath10k_core_start()` 的顺序一致）：

| 步骤 | 消息 | 长度 | 内容 |
| --- | --- | --- | --- |
| 1 | `VERSION_REQ` | 12 B | 等 `VERSION_CONF`（major ∈ {2,3}） |
| 2 | `RX_RING_CFG` | 48 B | index/ring 地址、2048 项、2048 B/buffer、投递 1023、10 个描述符偏移 |
| 3 | `AGGR_CFG` | 11 B | AMPDU 64 / AMSDU 3 |

内存布局：`index`(4 B 写索引) + `ring`(2048×LE32，指向 pool) + `pool`(1023×2048 B ≈ 2 MB)。
HTT 命令走 **CE4**（不是 CE0/CE3），描述符 `flags = endpoint << 2`，HTT 服务已关闭信用流
所以不带 `NEED_CREDIT_UPDATE`。

新增属性：`ath10k-htt-setup-ok` / `-status` / `-stage` / `-version-major` / `-version-minor` /
`-messages` / `-last-type` / `-rx-posted` / `-rx-pool-bytes`。

### 判读

- `htt-setup-ok = 1` 且 `scan-mgmt > 0` → 管理帧恢复上报，`scan-ssids` 应出现真实 SSID；
- `htt-setup-ok = 1` 但 `scan-mgmt = 0`、`htt-messages > 0`
  → 帧开始走 CE1（HTT），下一步实现 `RX_IND` 的 MSDU 解析（`Ath10kRX.hpp` 那部分）；
- `htt-setup-status` = `htt-ringcfg-submit` / `htt-tx-timeout`
  → RX_RING_CFG 未被消费，需要检查 CE4 管道或消息字段；
- `htt-version-received = 0` → 固件没回 VERSION_CONF，`htt-setup-stage` 会停在 1。

---

## 四之四、真机第三轮（2026-11-16 23:22，M1c2 构建）：**瓶颈打通**

```
htt-setup-ok        = 1     htt-setup-stage = 3     htt-setup-status = "rx-ring-posted"
htt-rx-posted       = 1023  htt-rx-pool-bytes = 2095104
htt-messages        = 1     ce1-last-endpoint = 1   ce1-last-bytes = 12（VERSION_CONF）
scan-mgmt           = 4     ← 管理帧开始上报！
scan-bss-count      = 4     ← 记到 4 个 BSS
scan-ok             = 1     scan-reason = 0        scan-ev-bss-chan = 23
```

**HTT RX 环是唯一缺口，这个判断被证实**：环一建好，固件立刻开始通过
`WMI_TLV_MGMT_RX_EVENTID` 上报扫描到的管理帧，不需要额外配置。

遗留两个小问题（M1c2b 已修）：

1. `ath10k-scan-ssids` 打印为空白 —— SSID 是二进制字段，含控制字符时会把
   ioreg/终端输出搞乱。现在按可打印 ASCII 消毒（其余替换为 `.`），并**逐条**
   发布 `ath10k-scan-bss0..bss7`，汇总串只作参考。
2. `htt-version-received = 0` 但确实收到了 12 字节 type=0 回复 —— 解析条件过严。
   现发布 `ath10k-htt-last-prefix`（16 字节十六进制）用于对齐固件 ABI，
   版本回复不作为阻塞条件。
3. 扫描完成后的收集窗口从 600 ms 放宽到 2 s，让固件把缓存的 BSS 全部推完。

判读（M1c2b 起）：

- `scan-bss0` 等属性出现 `ssid|bssid|freq|snr` → 扫描结果落地，M1-c1 完成；
- `htt-last-prefix` 里能看到 `01 00 04 00 xx 00 00 00 00 00 <major> <minor>` 形状
  （第 9 字节起是 type/major/minor），可据此修正版本解析。

---

## 四之五、真机第四轮（2026-11-16 23:30）：SSID 落地，M1-c1 完成

```
scan-ok = 1   scan-status = "complete"   scan-reason = 0
scan-mgmt = 2   scan-bss-count = 2
ath10k-scan-bss0 = "ChinaNet-F301-5G|f4:2e:48:e1:e6:08|36|15"
ath10k-scan-bss1 = "4F_5G|d4:84:09:4a:65:e6|153|16"
```

两条都是 5 GHz 网络（信道 36 / 153），SNR 15/16（信号偏弱但正确解码）。
扫描期间固件上报了 23 个 BSS_CHANNEL 事件，但只有 2 帧真正通过 MGMT_RX 到达，
说明**管理帧的上报是部分/延迟的**：值得在 M1-c3 里补时长与深度（见下）。

### 顺带确认的一个小问题

`ath10k-htt-last-prefix = "01 00 04 00 00 0a 00 00 00 38 03 00"`
→ 第 9 字节（type）=0 是 VERSION_CONF，第 11 字节 major=3；第 10 字节 0x38 不是
合理的 minor，说明固件这个字段的含义与我们的假设不同。当前代码只用它做诊断，
并把 minor 的合理性检查当作拒绝条件，所以 `htt-version-received` 一直是 0 ——
**纯诊断问题，不影响功能**（HTT 环已配置成功、帧已正常上报）。若后续要用版本号，
把 `parseHTTVersionReply` 的 `minor > 16` 检查去掉即可。

### 扫描覆盖率的改进方向（M1-c3 顺带）

- 目前只扫 22 个非 DFS 信道、dwell 200 ms；23 个 BSS_CHANNEL 只换来 2 帧，
  下一步可把 dwell 提到 300–500 ms，或在扫描结束后多留 2–3 s 收尾；
- 5 GHz 覆盖到 165，2.4 GHz 覆盖 1–13，暂不含 DFS。

---

## 五、验收

```sh
cd /Users/zhangjr/Desktop/vibe-coding/ath10Wlan
sleep 45 && sh scripts/verify-m1b2.sh      # 看第 9 段
```

期望：

```
ath10k-pdev-setup-ok   = 1
ath10k-scan-setup-ok   = 1
ath10k-scan-ok         = 1
ath10k-scan-status     = "complete"
ath10k-scan-started    = 1
ath10k-scan-completed  = 1
ath10k-scan-reason     = 0
ath10k-scan-mgmt       > 0
ath10k-scan-bss-count  > 0
ath10k-scan-ssids      = 真实 SSID 列表
```

---

## 六、失败定位

| 现象 | 原因 | 查什么 |
| --- | --- | --- |
| `pdev-setup-status` = `no-credit` / `uncertain-previous` | 信用或 CE3 状态异常 | `ath10k-wmi-tx-fail`、`credits-*` |
| `scan-setup-ok` = 0 且 status=`build-failed` | 信道表帧构造失败 | 离线自测第 8 组 |
| `scan-status` = `timeout` | 固件没有回 COMPLETED | `scan-started`、`scan-events`、`firmware-crash` |
| `scan-status` = `complete-with-reason` | 固件拒绝或中断扫描 | `scan-reason`（非 0） |
| `scan-completed` = 1 但 `bss-count` = 0 | 没收到 MGMT_RX，或解析失败 | `scan-mgmt`；`wmi-parse-error` 是否 `scan-*` |
| `scan-status` = `scan-event-invalid` | 事件 TLV 结构不符 | `wmi-parse-error` |

日志：

```sh
sudo log show --last 5m --predicate 'eventMessage CONTAINS "Ath10kHal"' --style compact \
  | grep -E "SCAN|PDEV"
```

---

## 七、下一步（M1-c2）

按优先级：

1. **HTT RX setup（关键路径）**：在 CE1 上配置 HTT RX ring 并投递 RX buffer。
   参考 `Ath10kHw.hpp`/`Ath10kRxSetup.hpp` 的 HTT 消息定义：
   - `HTT_T2H_MSG_TYPE_RX_RING_CFG`（或 `HTT_MSG_TYPE_RX_SETUP`）告知固件 ring 地址/深度；
   - 每个 RX buffer 需在 ring 里挂描述符，固件回填后由 `HTT_T2H_MSG_TYPE_RX_IND` 通知；
   - 先把 RX 帧头解析出来（`RX_MSDU_START` / `RX_MSDU_END` 的 128 字节 TLV 头），
     管理帧走 `ieee80211_rx_irqsafe` 等价路径进 net80211。
2. 把扫描/接收到的 BSS 灌进 net80211 node cache，供 `AirportItlwm` 的
   `SCAN_RESULT` 读取；
3. `ath_newstate` 实现 `IEEE80211_S_SCAN`，把扫描请求映射到 M1-c1 的 WMI 序列；
4. 补 `PDEV_SET_REGDOMAIN`（用 `eeprom-rd` / `low2` / `high2`）。