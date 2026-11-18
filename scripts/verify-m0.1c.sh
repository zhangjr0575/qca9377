#!/bin/sh
#
# verify-m0.1c.sh — M0.1c 验证：芯片识别 + CE ring + 热复位 + CE 管道
#
# 用法：sh scripts/verify-m0.1c.sh
# 只读脚本，不修改任何东西。

set -u

echo "=================================================================="
echo " M0.1c 验证：CE ring / 热复位 / CE 管道编程"
echo " 时间: $(date)"
echo "=================================================================="
echo ""

echo "---- 1. kext / HAL ----"
kmutil showloaded 2>/dev/null | grep -iE "airport|itlwm" || echo "  ✗ 未加载"
ioreg -l -w0 2>/dev/null | tr ',' '\n' | grep -E '"(Ath10kHal|ItlIwm|ItlIwx|ItlIwn)"=' | head -5
echo ""

echo "---- 2. 芯片识别（基线，必须仍为 ok）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(hardware-status|chip-id-hex|chip-revision|target-revision)"'
echo ""

echo "---- 3. CE ring 分配 ----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-ce-(rings-ok|rings-failure|configured|verified|pipe-passes)"'
echo ""
echo "  --- 各 CE 的 DMA 地址与表项数 ---"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-ce[0-9]-(src|dst)-(entries|dma)"'
echo ""

echo "---- 4. 热复位（M0.1c-2 核心）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(warm-reset-ok|warm-control-before|warm-control-after|warm-init1-ms|warm-init2-ms|warm-indicator1|warm-indicator2|post-warm-chip-id|chip-stable)"'
echo ""

echo "---- 4b. CE7 诊断通道（M0.1c-3）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(ce7-ok|ce7-transactions|ce7-core-ctrl|pcie-state-addr|pipe-cfg-addr|svc-map-addr|config-flags)"'
echo ""

echo "---- 4c. init_config（M0.1c-4）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-init-(config-ok|config-banks|flags-orig|flags-new|early-alloc-orig|early-alloc-new|flag2-orig|flag2-new)"'
echo ""

echo "---- 4d. CPU 唤醒 + BMI_GET_TARGET_INFO（M0.1c-5）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(cpu-woken|core-ctrl-before-wake|core-ctrl-after-wake|bmi-ok|bmi-length|bmi-type|bmi-version|bmi-uncertain|bmi-timed-out)"'
echo ""

echo "---- 4e. HI 前置 / PLL / Boot HI（M0.1c-6）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(bmi-prereq-ok|hi-board-data|hi-board-init|hi-early-alloc|hi-interconnect|pll-ok|pll-status|pll-step|boot-hi-ok|boot-hi-option-before|boot-hi-option-applied|boot-hi-words)"'
echo ""

echo "---- 4f. payload + board data 上传（M0.1c-7）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(payload-selected|payload-key|payload-selection|payload-sha256|board-write-ok|board-verify-ok|board-bytes|hi-board-init)"'
echo ""

echo "---- 4g. OTP 下载与执行（M0.1c-8）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(otp-download-ok|otp-execute-ok|otp-result-accepted|otp-result|otp-wire-bytes|otp-lz-commands|otp-status|bringup-running)"'
echo ""

echo "---- 4h. WLAN 固件下载（M0.1c-9）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(fw-download-ok|fw-readback-ok|fw-readback-bytes|fw-segments|fw-failed-segment|fw-wire-bytes|fw-lz-commands|fw-post-version|fw-status|bringup-done)"'
echo ""

echo "---- 4i. BMI_DONE + HTC READY（M0.1c-10）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(uart-disabled|pre-done-verified|hi-option-after-download|bmi-closed|bmi-done-sent|htc-rx-posted|htc-ready-ok|htc-rx-bytes|htc-wait-ms|htc-fw-indicator|htc-credits|htc-credit-size|htc-endpoints|htc-version|htc-status|htc-parse-error)"'
echo ""

echo "---- 4j. HTC 服务建立（M1-a）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(htc-services-ok|htc-service-status|wmi-endpoint|wmi-max-message|htt-endpoint|htt-max-message|htc-ctrl-tx|htc-ctrl-rx)"'
echo ""

echo "---- 4k. WMI bootstrap（M1-b1：事件 3 = SERVICE_AVAILABLE，事件 1 = SERVICE_READY）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(setup-complete-sent|wmi-service-available|wmi-service-ready|wmi-status|wmi-event-status|wmi-parse-error|wmi-event-id|wmi-rx-bytes|wmi-wait-ms|wmi-service-bits|wmi-bitmap[0-3]|wmi-evt-prefix|wmi-evt-prefix-bytes|wmi-rf-chains|wmi-memory-requests)"'
echo ""

echo "---- 4l. WMI INIT + host memory + vdev（M1-b2）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(wmi-init-ok|wmi-init-status|wmi-init-tx-bytes|wmi-init-wait-ms|wmi-host-memory-chunks|wmi-host-memory-bytes|wmi-ready-event|wmi-ready-status|wmi-credits-total|wmi-credits-available|wmi-credits-outstanding|wmi-credit-size|vdev-ok|vdev-status|vdev-tx-consumed|vdev-wait-ms|target-mac|target-mac-valid|wmi-seq|wmi-tx-fail|wmi-tx-src-write|wmi-tx-src-read|fw-indicator|firmware-crash|ce1-runtime-armed)"'
echo ""

echo "---- 4m. 扫描命令通路（M1-c1）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(pdev-setup-ok|pdev-setup-status|scan-setup-ok|scan-ok|scan-status|scan-events|scan-mgmt|scan-completed|scan-reason|scan-bss-count|scan-ssids)"'
echo ""

echo "---- 5. 稳定性 ----"
uptime
ls -lt /Library/Logs/DiagnosticReports/*.panic 2>/dev/null | head -2 || echo "  (无 panic)"
echo ""

cat <<'EOF'
==================================================================
 判读标准
==================================================================
基线（必须保持）：
  ath10k-hardware-status   = "ok"
  ath10k-chip-revision     = 1  （hw1.1）
  ath10k-chip-id-hex       = "0x003821ff"

M0.1c-1（CE ring）：
  ath10k-ce-rings-ok       = 1
  ath10k-ce-configured     = 6      （CE0/1/2/3/4/7 共 6 个有 ring）
  CE0 src 16, CE1 dst 512, CE2 dst 128, CE3 src 32, CE4 src 8192, CE7 src/dst 2
  所有 *-dma 必须是 0x00xxxxxx 且在 4GB 以下

M0.1c-2（热复位）：
  ath10k-warm-reset-ok     = 1      ← 核心指标
  ath10k-ce-pipe-passes    = 2      （热复位内部两遍 + 复位后一遍）
  ath10k-warm-indicator2   = 非 0xffffffff，且 bit1(0x2 initialized) 置位
  ath10k-chip-stable       = 1      （热复位后芯片版本未变）

失败定位：
  ce-ring-failed          → 看 ath10k-ce-rings-failure
  warm-reset-failed       → 看 ath10k-warm-init2-ms / warm-indicator2
                            indicator 仍是 0xffffffff → CE 管道或 CPU 复位未生效
  post-warm-wake-failed   → 热复位后无法重新唤醒
  ce-pipe-program-failed  → 复位后 CE 寄存器回读不一致
==================================================================
EOF