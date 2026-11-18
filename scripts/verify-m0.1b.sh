#!/bin/sh
#
# verify-m0.1b.sh — M0.1b 唤醒+复位+芯片识别验证（只读，无需 sudo）
#
# 用法：
#   sh scripts/verify-m0.1b.sh
#
# 安全性：只执行 ioreg / kmutil / ifconfig 等读取命令。

set -u

echo "=================================================================="
echo " M0.1b 唤醒 / 冷复位 / 芯片识别 验证"
echo " 时间: $(date)"
echo "=================================================================="
echo ""

echo "---- 1. kext 加载 ----"
kmutil showloaded 2>/dev/null | grep -iE "airport|itlwm" || echo "  ✗ 未加载"
echo ""

echo "---- 2. HAL 分支 ----"
ioreg -l -w0 2>/dev/null | tr ',' '\n' | grep -E '"(Ath10kHal|ItlIwm|ItlIwx|ItlIwn)"=' | head -5
echo ""

echo "---- 3. 芯片识别结果（核心）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(hardware-status|chip-id|chip-id-hex|chip-revision|chip-name|target-revision|firmware-dir|chip-id-attempts)"'
echo ""

echo "---- 4. 唤醒 / 复位诊断（M0.1b 新增）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(aspm-disabled|wake-ok|wake-delay-us|rtc-state-before|rtc-state-after|cold-reset-done|global-reset-before|global-reset-after|pcie-link-control-before|pcie-link-control-after)"'
echo ""

echo "---- 5. 控制器 / 接口 ----"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E '"IOModel"|"IOMACAddress"'
networksetup -listallhardwareports 2>/dev/null | grep -A2 "Wi-Fi"
echo ""

echo "---- 6. 稳定性 ----"
uptime
ls -lt /Library/Logs/DiagnosticReports/*.panic 2>/dev/null | head -2 || echo "  (无 panic)"
echo ""

cat <<'EOF'
==================================================================
 判读标准
==================================================================
1) ath10k-hardware-status = "ok"
   → 芯片唤醒并识别成功，可直接进入 M0.1c（BMI 固件下载）

2) ath10k-chip-revision = 1，ath10k-target-revision = hw1.1
   → 与三星 NP800G5M-X06CN 的已知配置一致

3) 若仍是 chip-id-invalid：
   - 看 ath10k-wake-ok        （0 表示 SOC_WAKE 唤醒失败）
   - 看 ath10k-cold-reset-done（0 表示冷复位失败）
   - 看 ath10k-rtc-state-after（期望 0x00000003，即 RTC_STATE_V_ON）
   - 看 ath10k-global-reset-after（期望最低位为 0）

4) 若 wake-failed / rewake-failed：
   → 说明复位窗口内链路仍未就绪，需补 PCIe bring-up parity（MSI 配置等）
==================================================================
EOF