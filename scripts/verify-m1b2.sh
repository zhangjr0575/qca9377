#!/bin/sh
#
# verify-m1b2.sh — M1-b2 验证：WMI INIT + host memory + vdev create + 真实 MAC
#
# 用法：sh scripts/verify-m1b2.sh
# 只读脚本，不修改任何东西。bring-up 是异步的（含扫描，最长约 30 秒），
# 开机后建议等 45 秒再跑。

set -u

echo "=================================================================="
echo " M1-b2 验证：WMI INIT / host memory / vdev create / target MAC"
echo " 时间: $(date)"
echo "=================================================================="
echo ""

echo "---- 1. kext / HAL ----"
kmutil showloaded 2>/dev/null | grep -iE "airport|itlwm" || echo "  ✗ 未加载"
ioreg -l -w0 2>/dev/null | tr ',' '\n' | grep -E '"(Ath10kHal|ItlIwm|ItlIwx|ItlIwn)"=' | head -5
echo ""

echo "---- 2. 总状态（关键）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(hardware-status|bringup-done|bringup-running)"'
echo ""

echo "---- 3. 基线：芯片 / CE / 热复位（必须保持通过）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(chip-id-hex|chip-revision|ce-rings-ok|ce-configured|ce-verified|warm-reset-ok|ce-pipe-passes|chip-stable)"'
echo ""

echo "---- 4. 固件 / HTC（M0.1c ~ M1-a 基线）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(fw-download-ok|fw-readback-ok|bmi-done-sent|htc-ready-ok|htc-credits|htc-credit-size|htc-endpoints|htc-services-ok|wmi-endpoint|wmi-max-message|htt-endpoint|htc-ctrl-tx)"'
echo ""

echo "---- 5. WMI bootstrap（M1-b1，注意 SERVICE_AVAILABLE=3 先到，SERVICE_READY=1 后到）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(setup-complete-sent|wmi-service-available|wmi-service-ready|wmi-status|wmi-event-status|wmi-parse-error|wmi-event-id|wmi-rx-bytes|wmi-wait-ms|wmi-service-bits|wmi-bitmap[0-3])"'
echo ""

echo "---- 6. SERVICE_READY 内容（WMI INIT 的输入）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-wmi-(fw-build|abi0|abi2|abi3|rf-chains|memory-requests|svc-bitmap-present|svc-word[0-3]|reg-caps-present|eeprom-rd|reg-cap1|low2|high2)"'
echo ""

echo "---- 7. WMI INIT + host memory（M1-b2 核心）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-wmi-(init-ok|init-status|init-tx-bytes|init-wait-ms|host-memory-chunks|host-memory-bytes|ready-event|ready-status|ready-status30|credits-total|credits-available|credits-outstanding|credit-size|seq)"'
echo ""
echo "  --- INIT 事件原始前缀（解析失败时唯一依据）---"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-wmi-(evt-prefix|evt-prefix-bytes)"'
echo ""

echo "---- 8. vdev create + 真实 MAC（M1-b2 可见成果）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(vdev-ok|vdev-status|vdev-tx-consumed|vdev-tx-bytes|vdev-wait-ms|target-mac|target-mac-valid)"'
echo ""
echo "  --- 失败定位辅助（CE3 环指针 / 信用 / 固件心跳 / CE1）---"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(wmi-tx-fail|wmi-tx-src-write|wmi-tx-src-read|fw-indicator|fw-indicator-baseline|firmware-crash|ce1-runtime-armed|wmi-seq)"'
echo ""
echo "  --- 接口 MAC（应等于 target-mac）---"
ifconfig 2>/dev/null | grep -A2 -E "^en[0-9]+:" | grep -E "^en[0-9]+:|ether" | head -20
echo ""

echo "---- 8b. HTT RX 环（M1-c2：没有这个环，固件不交任何接收帧）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(htt-setup-ok|htt-setup-status|htt-setup-stage|htt-version-received|htt-version-major|htt-version-minor|htt-messages|htt-last-type|htt-last-prefix|htt-last-prefix-bytes|htt-rx-posted|htt-rx-pool-bytes|ce1-frames|ce1-last-endpoint)"'
echo ""

echo "---- 9. M1-c1 扫描（PDEV / 信道表 / START_SCAN / SSID）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(pdev-setup-ok|pdev-setup-status|scan-setup-ok|scan-ok|scan-status|scan-events|scan-mgmt|scan-started|scan-completed|scan-reason|scan-last-freq|scan-wait-ms|scan-bss-count|scan-ev-bss-chan|scan-ev-foreign-chan|scan-ev-other|ce1-frames|ce1-last-endpoint|ce1-last-bytes)"'
echo ""
echo "  --- 扫到的 SSID（ssid|bssid|freq|snr，逐条 + 汇总）---"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E '"ath10k-scan-(bss[0-7]|ssids)"'
echo ""

echo "---- 10. 稳定性 ----"
uptime
ls -lt /Library/Logs/DiagnosticReports/*.panic 2>/dev/null | head -2 || echo "  (无 panic)"
echo ""

cat <<'EOF'
==================================================================
 判读标准（M1-b2）
==================================================================
必须保持的基线：
  ath10k-hardware-status      = "wmi-ready"（M1-b2 通过后的终态）
  ath10k-chip-id-hex          = "0x003821ff"
  ath10k-ce-rings-ok          = 1
  ath10k-warm-reset-ok        = 1
  ath10k-htc-ready-ok         = 1
  ath10k-htc-services-ok      = 1

M1-b1（WMI 事件顺序）：
  ath10k-wmi-service-available = 1     （事件 3，服务可用性位图）
  ath10k-wmi-service-ready     = 1     （事件 1，固件 ABI + RF 链数 + host memory 请求）
  ath10k-wmi-rf-chains         = 1     （QCA9377 是 1x1）
  ath10k-wmi-memory-requests   = 事件 1 里的 num_host_mem_reqs

M1-b2（核心验收）：
  ath10k-wmi-init-ok          = 1
  ath10k-wmi-init-status      = "ready" 或 "ready-credit-pending"
  ath10k-wmi-host-memory-chunks = 与 memory-requests 相同
  ath10k-wmi-ready-event      = 1
  ath10k-wmi-ready-status     = 0
  ath10k-vdev-ok              = 1
  ath10k-vdev-tx-consumed     = 1
  ath10k-target-mac-valid     = 1
  ath10k-target-mac           = 真实 MAC（不再是 02:11:22:33:44:55）
  ath10k-firmware-crash       = 0
  接口 en4 的 ether 应等于 target-mac

M1-c1（扫描命令链路）：
  ath10k-pdev-setup-ok   = 1
  ath10k-scan-setup-ok   = 1
  ath10k-scan-ok         = 1
  ath10k-scan-status     = "complete"
  ath10k-scan-reason     = 0
  ath10k-scan-events     > 0
  ath10k-scan-ssids      = 真实 SSID 列表（若为 "(none)" 见下方判读）

扫描结果为空时的判读（M1-c1b 起提供）：
  scan-ev-bss-chan > 0 且 scan-mgmt = 0
      => 固件看见了 BSS 但不交帧，需要 HTT RX setup（M1-c2）；
         再看 ce1-frames 是否 > 0（帧其实在 CE1 上）
  scan-ev-bss-chan = 0
      => 扫描没发现任何 BSS，先查信道/功率配置与射频状态

vdev 失败时先看这四项：
  ath10k-wmi-tx-fail        失败分支（uncertain-previous / ce3-not-idle / no-credit /
                            timeout / doorbell / tx-sync / src-read）
  ath10k-wmi-tx-src-write   与 -tx-src-read 不等 => CE3 环上还有未消费命令
  ath10k-firmware-crash     = 1 => 固件启动后崩溃
  ath10k-wmi-credits-*      信用是否归还（outstanding 应回 0）

失败定位：
  wmi-status = "wmi-service-ready-event-missing"
      → 只收到事件 3，事件 1 没来。看 wmi-event-id / wmi-evt-prefix。
  wmi-init-status = "wmi-init-abi-mismatch"
      → 固件 ABI 与 Linux host ABI 不符，看 wmi-abi0/abi2/abi3。
  wmi-init-status = "wmi-init-no-credit" / vdev-status = "vdev-no-credit"
      → WMI 信用不足；看 credits-total / credits-available / credit-size。
  wmi-init-status = "wmi-ready-timeout"
      → INIT 已发出但没有 READY(2)。看 wmi-init-tx-bytes、credits-outstanding、
        wmi-evt-prefix（是否有 READY 但解析失败，此时 wmi-parse-error 非 none）。
  wmi-init-status = "wmi-ready-firmware-rejected"
      → READY 状态字非 0，看 wmi-ready-status / wmi-ready-status30。
  wmi-init-status = "wmi-memory-request-*" / "wmi-host-memory-*"
      → host memory 请求解析或 DMA 分配失败。
==================================================================
EOF