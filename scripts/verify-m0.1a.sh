#!/bin/sh
#
# verify-m0.1a.sh — M0.1a 硬件探测验证（只读，无需 sudo）
#
# 用法：
#   sh scripts/verify-m0.1a.sh
#
# 验证内容：
#   1. kext 是否加载
#   2. Ath10kHal 是否实例化（且 Intel 分支未触发）
#   3. 硬件探测属性是否正确发布（chip id / revision / BAR0 / 状态）
#   4. 原生 Wi-Fi 接口是否存在
#
# 安全性：只执行 ioreg / kmutil / ifconfig 等读取命令，不修改任何东西。

set -u

echo "=================================================================="
echo " M0.1a 硬件探测验证"
echo " 时间: $(date)"
echo "=================================================================="
echo ""

echo "---- 1. kext 加载状态 ----"
kmutil showloaded 2>/dev/null | grep -iE "airport|itlwm|ath10k" || echo "  ✗ 未加载 AirportItlwm"
echo ""

echo "---- 2. HAL 分支选择（Ath10kHal 应为 1，Intel 应为 0）----"
ioreg -l -w0 2>/dev/null | tr ',' '\n' \
    | grep -E '"(Ath10kHal|ItlIwm|ItlIwx|ItlIwn)"=' | head -6
echo ""

echo "---- 3. M0.1a 硬件探测属性（关键）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(hardware-status|chip-id|chip-revision|bar0-length|hw-probed|chip-name|target-revision|firmware-dir)"' \
    || echo "  ✗ 未找到 ath10k-* 探测属性（当前很可能仍是无硬件探测的旧版 kext）"
echo ""

echo "---- 4. 控制器基本信息 ----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -iE "IOModel|IOMACAddress|IONetworkRootType|IOMatchedAtBoot" || echo "  (未找到)"
echo ""

echo "---- 5. 原生 Wi-Fi 接口 ----"
networksetup -listallhardwareports 2>/dev/null | grep -A2 "Wi-Fi" || echo "  (未找到 Wi-Fi 端口)"
echo ""

echo "---- 6. 本次启动后是否新增 panic ----"
uptime
echo ""
ls -lt /Library/Logs/DiagnosticReports/*.panic 2>/dev/null | head -3 || echo "  (无 panic 记录)"
echo ""

echo "=================================================================="
echo " 判读标准："
echo "   - 第 2 节：Ath10kHal=1，ItlIwm/ItlIwx/ItlIwn 均为 0"
echo "   - 第 3 节：ath10k-hardware-status 出现，理想值为 ok"
echo "   - ath10k-chip-id：理想值 0x00000000 系列的 0x000000c1 之类"
echo "     （若为 0xffffffff 表示芯片休眠，需 M0.1b 唤醒）"
echo "=================================================================="