#!/bin/sh
# M1-d 连接流程验证（只读，不修改任何状态）
#
# 用法：
#   sh scripts/verify-m1d.sh            # 驱动侧状态
#   （连接结果本身要看菜单栏：连上后菜单里会出现 ✓）
#
# 前提：kext 已部署并重启，且已点选一个网络尝试连接。
set -u

echo "=================================================================="
echo " M1-d 验证：连接流程（VDEV/PEER/AUTH/ASSOC/RUN）"
echo " 时间: $(date)"
echo "=================================================================="
echo ""

echo "---- 1. 连接总状态 ----"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E \
 '"ath10k-(join-enabled|join-status|join-stage|join-commands|join-command-fails)"'
echo ""
echo "  判读：join-stage 0=空闲 1=VDEV_START 2=PEER_CREATE 3=PEER_ASSOC"
echo "        5=AUTHORIZE 6=VDEV_UP ；join-status 是最后一次动作的定性串"
echo ""

echo "---- 2. 管理帧发送（HTT TX / CE4）----"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E \
 '"ath10k-(mgmt-tx-submitted|mgmt-tx-completed|mgmt-tx-failed|mgmt-tx-dropped)"'
echo ""
echo "  submitted 必须 > 0（至少发出 AUTH 请求）；completed 应跟随 submitted"
echo "  failed 常见值：mgmt-tx-timeout / mgmt-tx-status / ce4-* / mgmt-tx-target-asleep"
echo ""

echo "---- 3. 运行期 RX 泵 ----"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E \
 '"ath10k-(rx-pump-running|rx-pump-events|rx-pump-errors|join-feed-ok)"'
echo ""
echo "  rx-pump-running=1 表示正在收（连接期间才运行）；join-feed-ok 是喂进"
echo "  net80211 的 auth/assoc 帧数 —— 它 > 0 说明 AUTH 响应真的到了"
echo ""

echo "---- 0. 构建标记（先确认跑的是哪一版）----"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E '"ath10k-build"'
echo ""
echo "---- 3b. 系统请求计数（系统到底有没有叫我们）----"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E \
 '"wcl-(scan|assoc|leave|join-abort)-reqs|wcl-join-aborts|wcl-legacy-assoc|wcl-directed-scans|wcl-last-scan-(type|ssid-len|ssid)"'
echo "  wcl-directed-scans 增加 → 点击确实到了驱动（定向扫描），但系统没紧跟 JOIN"
echo "  wcl-assoc-reqs / wcl-legacy-assoc = 0 → 点击没有变成任何驱动请求（UI 层问题）"
echo "  wcl-assoc-reqs > 0  → 系统发了 WCL 连接请求，再看下面的 join 判读"
echo ""
echo "---- 3c. WCL 连接入口（点了网络却什么也没发生就看这里）----"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E \
 '"wcl-join-(attempts|last-result)"'
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E \
 '"ath10k-(wcl-join-requested|wcl-scan-requested|wcl-mac-reconfig|des-esslen|if-running|if-flags|ic-flags)"'
echo ""
echo "  wcl-join-attempts=0            → 点击根本没到达驱动的连接入口"
echo "  last-result=unsupported        → 该网络的认证类型被 classifyLegacyAuth 判为不支持"
echo "  last-result=no-running         → IFF_RUNNING 没置位（enable() 没跑或又被 disable 了）"
echo "  last-result=scan-pending       → 有扫描请求还没收尾（SCAN_DONE 没送到）"
echo "  last-result=join-pending       → 上一次连接还没结束"
echo "  last-result=accepted           → 入口通过了，问题在连接命令链/RX 泵（看第 1~3 节）"
echo ""

echo "---- 4. net80211 状态机 ----"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E \
 '"ath10k-(net80211-state|net80211-opmode|net80211-nodes|net80211-event-handler)"'
echo ""
echo "  net80211-state：1=SCAN 2=AUTH 3=ASSOC 4=RUN（4 就是关联成功）"
echo ""

echo "---- 5. 固件基线（不能回归）----"
ioreg -rc AirportItlwm -w0 2>/dev/null | grep -E \
 '"ath10k-(hardware-status|wmi-init-status|vdev-ok|firmware-crash|scan-feed-status|wmi-tx-fail)"'
echo ""

echo "---- 6. 接口 ----"
ifconfig en4 2>/dev/null | head -4
echo ""

echo "---- 7. 内核日志（最近 30 分钟与本项目相关）----"
echo "  需要 root，请自行执行："
echo "    sudo log show --last 30m --predicate 'eventMessage CONTAINS \"Ath10kHal\"' --style compact | tail -60"
echo "    # 只看连接相关的关键行："
echo "    sudo log show --last 10m --predicate 'eventMessage CONTAINS \"Ath10kHal\"' --style compact | grep -E \"WCL JOIN|JOIN |end_scan|newstate|mgmt\" | tail -40"
echo ""

echo "=================================================================="
echo " 判读（M1-d 阶段目标：走到 RUN）"
echo "=================================================================="
echo ""
echo "  join-stage=3 + mgmt-tx-submitted>=1 + rx-pump-running=1   → 正在认证"
echo "  join-feed-ok>0                                             → 收到 AUTH/ASSOC 响应"
echo "  join-stage=6 + net80211-state=4                            → ★ 关联成功（RUN）"
echo ""
echo "  失败速查："
echo "    join-commands 不增长且 join-status=join-no-bss"
echo "        → end_scan 没选到 AP（ic_bss 为空）。看 WCL 是否设了 ic_des_essid，"
echo "          以及目标 SSID 是否在本轮扫描结果里。"
echo "    join-status=join-bad-channel"
echo "        → 目标 AP 的信道不在我们的扫描表内（DFS 或 6GHz）。"
echo "    mgmt-tx-failed 增长且 join-status=mgmt-tx-timeout"
echo "        → 帧发出去了但固件没回 MGMT_TX_COMPLETION：看 ce4/HTT 通路。"
echo "    mgmt-tx-submitted=0"
echo "        → ath_start 没被调用（net80211 没发管理帧），或 join-enabled=0。"
echo "    net80211-state 回到 1 且反复"
echo "        → 认证/关联被拒或超时，看第 7 节日志里的 AUTH/ASSOC 记录。"
echo ""