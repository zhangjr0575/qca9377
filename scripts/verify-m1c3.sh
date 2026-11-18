#!/bin/sh
#
# verify-m1c3.sh — M1-c3 验证：扫描结果接入 net80211（系统 Wi-Fi 列表可见）
#
# 用法：sh scripts/verify-m1c3.sh
# 只读脚本，不修改任何东西。
#
# 时间预期：bring-up（固件下载）约 15 秒，第一轮全信道扫描再约 15 秒，
# 开机后建议等 60 秒再跑；要观察第二轮扫描时可过 30 秒再跑一次。

set -u

AIRPORT="/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport"

echo "=================================================================="
echo " M1-c3 验证：扫描 → net80211 节点 → 系统 Wi-Fi 列表"
echo " 时间: $(date)"
echo "=================================================================="
echo ""

echo "---- 1. kext / HAL ----"
kmutil showloaded 2>/dev/null | grep -iE "airport|itlwm" || echo "  ✗ 未加载"
ioreg -l -w0 2>/dev/null | tr ',' '\n' | grep -E '"(Ath10kHal|ItlIwx|ItlIwm)"=' | head -3
echo ""

echo "---- 2. 总状态 ----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(hardware-status|bringup-done|bringup-running)"'
echo ""

echo "---- 3. M1-c3 核心：net80211 侧状态 + 扫描节流（r1）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(net80211-state|net80211-opmode|net80211-nodes|net80211-event-handler)"'
echo ""
echo "  --- 喂帧 / 收尾统计 ---"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-scan-(rounds|feed-attempts|feed-ok|feed-skipped|feed-status|stage-overflow|nodes|done-events)"'
echo ""
echo "  --- r1 节流/熔断（防自旋）：disabled / self-fused / grace 必须看 ---"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-scan-(disabled|self-fused|tick|grace-ticks|next-allowed-tick|fail-streak|request-seen|request-rounds|round-start-tick)"'
echo ""

echo "---- 3b. 自旋检测（关键）：5 秒后再取一次 scan-rounds，必须不变 ----"
R1=$(ioreg -rc AirportItlwm -w0 2>/dev/null | sed -n 's/.*"ath10k-scan-rounds" = \([0-9]*\).*/\1/p')
sleep 5
R2=$(ioreg -rc AirportItlwm -w0 2>/dev/null | sed -n 's/.*"ath10k-scan-rounds" = \([0-9]*\).*/\1/p')
echo "  scan-rounds 5s 前=$R1 现在=$R2 （未打开 Wi-Fi 菜单时应相等）"
if [ -n "$R1" ] && [ -n "$R2" ] && [ "$R1" = "$R2" ]; then
    echo "  ✓ 无自旋"
else
    echo "  ⚠ 轮数在增长：如果此时没有打开 Wi-Fi 菜单，说明还有路径在重排扫描"
fi
echo "  提示：r1 开机有 60 s 静默期（scan-rounds 应保持 0、feed-status=waiting-grace-window），"
echo "        静默期后也只有系统发起扫描请求才扫（1~2 轮，间隔 >=10 s）。"
echo ""

echo "---- 4. 固件侧扫描基线（M1-c1，必须保持通过）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(pdev-setup-ok|pdev-setup-status|scan-setup-ok|scan-ok|scan-status|scan-reason|scan-events|scan-mgmt|scan-bss-count|scan-wait-ms|htt-setup-ok)"'
echo ""
echo "  --- 驱动侧 SSID 摘要（ssid|bssid|freq|snr）---"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E '"ath10k-scan-(bss[0-7]|ssids)"'
echo ""

echo "---- 5. WMI / vdev / MAC 基线（M1-b2）----"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(wmi-init-ok|wmi-init-status|vdev-ok|target-mac|target-mac-valid|firmware-crash|scan-feed-status)"'
echo ""
echo "  --- CE3 提交诊断（r2 起每轮刷新；ce3-not-idle/resync 看这里）---"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(wmi-tx-fail|wmi-seq|wmi-tx-src-write|wmi-tx-src-read|wmi-tx-src-write-raw|wmi-tx-src-read-raw|ce3-resyncs)"'
echo ""
echo "  --- 运行期唤醒状态（r3：SOC_WAKE 必须一直断言）---"
ioreg -rc AirportItlwm -w0 2>/dev/null \
    | grep -E '"ath10k-(wake-ok|wake-asserted|target-awake|wake-retries|last-rtc-state|last-rtc-readable|aspm-disabled)"'
echo ""

echo "---- 6. 接口状态（IFF_RUNNING 是 WCL 放行扫描请求的前提）----"
ifconfig 2>/dev/null | grep -B1 -A3 -E "^en4:" | head -8
echo ""
echo "  --- networksetup 里的 Wi-Fi 端口 ---"
networksetup -listallhardwareports 2>/dev/null | grep -A2 -i "Wi-Fi" | head -6
echo ""

echo "---- 7. 系统侧扫描结果（最终的可见成果）----"
echo "  ★ 唯一判据：菜单栏 Wi-Fi / 系统设置 > Wi-Fi 里是否出现邻居网络。"
echo "    airport 在 macOS 15 已废弃，对任何网卡都只打印一句警告后返回空，"
echo "    所以下面这段即使为空也**不算失败**（M1-c3 验收时就踩过这个陷阱）。"
if [ -x "$AIRPORT" ]; then
    echo "  \$ airport -s   （仅供参考，空输出属正常）"
    "$AIRPORT" -s 2>&1 | grep -v "WARNING\|For diagnosing" | head -20
fi
echo ""
echo "  --- 需要 root 的辅助命令（自行执行）---"
echo "    sudo wdutil info | head -40"
echo "    networksetup -getairportnetwork en4"
echo ""

echo "---- 8. SPAirPortDataType 前 40 行 ----"
system_profiler SPAirPortDataType 2>/dev/null | head -40
echo ""

echo "---- 9. 稳定性 ----"
uptime
ls -lt /Library/Logs/DiagnosticReports/*.panic 2>/dev/null | head -2 || echo "  (无 panic)"
echo ""

cat <<'EOF'
==================================================================
 判读标准（M1-c3）
==================================================================
驱动侧（必须全绿）：

  ath10k-net80211-opmode   = 1                  （IEEE80211_M_STA）
  ath10k-net80211-state    = 1                  （停在 SCAN 属正常，未连接）
  ath10k-scan-rounds       >= 1                 （r4 真机实测 11 轮；
                                                 开机 60 s 内仍应为 0）
  ath10k-scan-disabled     = 0                  （1 = boot-arg 关闭或自熔断）
  ath10k-scan-self-fused   = 0                  （1 = 连续失败后已自行停用）
  ath10k-scan-fail-streak  = 0                  （>0 说明某轮失败过）
  ath10k-scan-feed-ok      > 0                  （原始帧真的进了 ieee80211_input）
  ath10k-net80211-nodes    > 0                  （ic_tree 里真的有了节点）
  ath10k-scan-done-events  >= 1                 （WCL 能收到 SCAN_DONE；实测 11）
  ath10k-net80211-event-handler = 1             （WCL 事件回调已挂上；r4 修复项）
  ath10k-hardware-status   = "wmi-ready"        （M1-b2 基线不回归）
  ath10k-scan-feed-status  = "fed"              （或 round-complete-no-frames）

稳定性判据（r1 重点）：
  1) 开机 60 s 内：scan-rounds = 0、feed-status = "waiting-grace-window"；
  2) 不开 Wi-Fi 菜单静置 3~5 分钟：scan-rounds 不得增长；
  3) 打开 Wi-Fi 菜单/airport -s 后：scan-rounds +1~2 后再次停住；
  4) 任何时候 scan-self-fused = 1 都意味着驱动已自行停用扫描，
     看 scan-feed-status 里的原因（ce3-not-idle / no-credit / timeout ...）。

系统侧（唯一判据）：

  菜单栏 Wi-Fi 列表 / 系统设置 > Wi-Fi 出现邻居网络   ← 2026-11-17 23:20 已达成
  （airport -s 在 macOS 15 已废弃，空输出不代表失败；辅助可用 sudo wdutil info）

==================================================================
 失败模式速查
==================================================================
  feed-ok = 0 且 feed-status = "skipped-not-scan-state"
      → 收帧时 ic_state 不是 SCAN。多为 setPOWER 关过网卡（disable 清掉了
        IFF_RUNNING），先确认 Wi-Fi 电源是打开的。

  feed-ok = 0 且 feed-status = "round-complete-no-frames"
      → 固件没交管理帧。看 ath10k-scan-mgmt / ath10k-ce1-frames；
        M1-c2 的 HTT RX 环若 htt-setup-ok = 0 则一定为 0。

  feed-ok > 0 但 net80211-nodes = 0
      → 帧进了 net80211 却被丢弃。检查是否缺 Rates IE（不该发生）或
        DS 信道与 rxi_chan 不一致（内核日志里 ath10k: SCAN BSS 仍在打）。

  nodes > 0 但 scan-done-events = 0
      → 要么 state 已不是 SCAN，要么 ic_event_handler 为空。
        r4 修正：AirportItlwm::start() 在 attach() 之前写 ic_event_handler，
        而 attach() 里 bzero(&ic) 会把它清掉（r3 真机就是这样：驱动侧 174 帧、
        15 节点全对，但 airport -s 空、scan-done-events = 0）。
        现在看 ath10k-net80211-event-handler 必须 = 1。

  rounds 不增长
      → 上一轮 worker 还卡在轮询里，或 queueScanWork 失败
        （看 ath10k-scan-feed-status = thread-call-unavailable / not-attached）。

  scan-feed-status = ce3-not-idle / ce3-structure / ce3-snapshot / ce3-byteswap
      → WMI TX（CE3）提交前的环状态检查没过：
        ce3-not-idle   = 写/读索引（取模后）不一致，且有在途命令（真忙）
        ce3-structure  = 环基址/长度与驱动记录不符
        ce3-snapshot   = 寄存器读失败
        ce3-byteswap   = CE3 被打开了字节交换
        ce3-asleep     = 读到 0xDEADBEEF/0xFFFFFFFF（SOC 睡了）
        对照 ath10k-wmi-tx-src-*-raw（原始自由计数值）与 ath10k-ce3-resyncs。
        r2 起若账本确认无在途命令，会按设备读索引重同步（ce3-resyncs 递增）。

  target-awake = 0 / last-rtc-state = 0xdeadbeef
      → 扫描时 SOC 已经睡了（r3 之前就是这样：bring-up 一结束就 releaseTargetWake）。
        r3 起 bring-up 成功后保持 SOC_WAKE 断言，并在扫描前用 ensureTargetAwake()
        再检查/重唤醒（wake-retries 会 +1）。
EOF