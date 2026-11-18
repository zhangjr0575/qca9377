#!/bin/sh
#
# collect-m0-logs.sh — M0 骨架验证日志采集（只读脚本，不做任何修改）
#
# 用法：
#   sudo sh scripts/collect-m0-logs.sh
#
# 输出：workspace/logs/m0-logs-<时间戳>.txt
# 作用：采集内核日志、kext 加载状态、AirPort 接口信息、panic 记录，
#       用于确认 Ath10kHal 是否正确附着到 QCA9377。
#
# 安全性说明：本脚本只执行读取类命令（log show / ioreg / kmutil / ifconfig /
#             system_profiler / networksetup），不修改、不删除任何文件或配置。

set -u

# 输出目录（脚本所在目录的上一级的 logs/）
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
OUT_DIR="${SCRIPT_DIR}/../logs"
mkdir -p "$OUT_DIR"
STAMP=$(date +%Y%m%d-%H%M%S)
OUT="${OUT_DIR}/m0-logs-${STAMP}.txt"

# 所有输出同时写文件与屏幕
exec > "$OUT" 2>&1

echo "=================================================================="
echo " M0 骨架验证日志采集"
echo " 时间: $(date)"
echo "=================================================================="
echo ""

echo "==================== 1. 系统信息 ===================="
sw_vers
echo "内核: $(uname -a)"
echo "运行时间: $(uptime)"
echo "启动参数:"
nvram -p 2>/dev/null | grep -i "boot-args" || echo "  (读取失败或为空)"
echo ""

echo "==================== 2. 已加载的 Wi-Fi 相关 kext ===================="
kmutil showloaded 2>/dev/null | grep -iE "airport|itlwm|ath10k" || echo "  (未匹配到)"
echo ""

echo "==================== 3. 期望的关键日志: Ath10kHal 附着 ===================="
echo "--- 查找 'M0 walking skeleton attached' ---"
log show --last 120m --predicate 'eventMessage CONTAINS "walking skeleton"' --style compact 2>/dev/null \
    | tail -20 || echo "  (未找到该日志)"
echo ""

echo "==================== 4. Ath10kHal / AirportItlwm 全部内核日志 ===================="
echo "--- 最近 120 分钟 ---"
log show --last 120m --predicate '
    eventMessage CONTAINS[c] "Ath10k"
    OR eventMessage CONTAINS[c] "AirportItlwm"
    OR eventMessage CONTAINS[c] "ieee80211_ifattach"
    OR eventMessage CONTAINS[c] "itlwm"
' --style compact 2>/dev/null | tail -200 || echo "  (未取到)"
echo ""

echo "==================== 5. 内核报错 / 崩溃痕迹 ===================="
echo "--- 最近 120 分钟的 error/fault 日志 ---"
log show --last 120m --predicate '
    process == "kernel"
    AND (messageType == error OR eventMessage CONTAINS[c] "panic")
' --style compact 2>/dev/null | tail -80 || echo "  (未取到)"
echo ""

echo "==================== 6. AirportItlwm 设备树 ===================="
echo "--- IO80211Controller ---"
ioreg -rc IO80211Controller -w0 2>/dev/null | head -40 || echo "  (未找到)"
echo ""
echo "--- AirportItlwm 关键属性 ---"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -iE "IOModel|IOMACAddress|IONetworkRootType|IOMatchedAtBoot|IOFeatures" || echo "  (未找到)"
echo ""

echo "==================== 7. 类实例统计（确认 HAL 分支）===================="
echo "--- Ath10kHal / Itl* 实例数 ---"
ioreg -l -w0 2>/dev/null | tr ',' '\n' | grep -E '"(Ath10kHal|ItlIwm|ItlIwx|ItlIwn|IOPCIEDeviceWrapper|AirportItlwm)"=' | head -10 \
    || echo "  (未取到)"
echo ""

echo "==================== 8. Wi-Fi 接口 ===================="
networksetup -listallhardwareports 2>/dev/null
echo ""
echo "--- en4 详细状态 ---"
ifconfig en4 2>/dev/null || echo "  (en4 不存在)"
echo ""
echo "--- en3 残留状态（旧 Ath10kMac 接口）---"
ifconfig en3 2>/dev/null || echo "  (en3 不存在，正常)"
echo ""

echo "==================== 9. AirPort 软件信息 ===================="
system_profiler SPAirPortDataType 2>/dev/null | head -60 || echo "  (未取到)"
echo ""

echo "==================== 10. 网络服务顺序（含残留项）===================="
networksetup -listnetworkserviceorder 2>/dev/null | head -30
echo ""

echo "==================== 11. panic 记录列表 ===================="
ls -lt /Library/Logs/DiagnosticReports/*.panic 2>/dev/null | head -5 || echo "  (无 panic 记录)"
echo ""

echo "=================================================================="
echo " 采集完成"
echo " 结果文件: $OUT"
echo "=================================================================="