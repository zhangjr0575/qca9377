#!/bin/sh
#
# deploy-kext.sh — 把 AirportItlwm 构建产物替换进 EFI/OC/Kexts
#
# 用法：
#   sh scripts/deploy-kext.sh <新kext路径>              # 交互确认（推荐）
#   sh scripts/deploy-kext.sh <新kext路径> --yes        # 跳过确认
#   sh scripts/deploy-kext.sh <新kext路径> --dry-run    # 只做检查，不动 EFI
#
# 行为：
#   1. 校验新 kext 是有效 bundle，且确实包含 M1-b2 符号
#   2. 把现有 kext 备份到 ~/Desktop（带时间戳）
#   3. 删除 EFI 上的旧 kext，复制新 kext
#   4. 用 md5 比对部署结果，校验 Info.plist，sync
#
# 注意：第 3 步会删除 EFI 分区上的旧目录；脚本会先备份并要求确认。
# 替换完成后必须重启才生效。

set -eu

SRC=${1:-}
MODE=${2:-}
DST="/Volumes/ESP/EFI/OC/Kexts/AirportItlwm-Sequoia.kext"
STAMP=$(date +%Y%m%d-%H%M%S)
BACKUP="$HOME/Desktop/AirportItlwm-Sequoia.backup-$STAMP.kext"

fail() { echo "✗ $1" >&2; exit 1; }
step() { echo ""; echo "==> $1"; }

# ---------------------------------------------------------------- 参数与检查
[ -n "$SRC" ] || fail "用法: sh scripts/deploy-kext.sh <新kext路径> [--yes|--dry-run]"
[ -d "$SRC" ] || fail "找不到 kext 目录: $SRC"
[ -f "$SRC/Contents/Info.plist" ] || fail "不是有效 kext（缺 Contents/Info.plist）: $SRC"
[ -f "$SRC/Contents/MacOS/AirportItlwm" ] || fail "缺可执行文件: $SRC/Contents/MacOS/AirportItlwm"

# 防止把误建的嵌套目录当产物（历史上踩过）
[ ! -d "$SRC/AirportItlwm.kext" ] || fail "该目录里嵌套了 AirportItlwm.kext，看起来是误拷贝: $SRC"

BUNDLE_ID=$(/usr/libexec/PlistBuddy -c "Print :CFBundleIdentifier" "$SRC/Contents/Info.plist" 2>/dev/null || true)
[ "$BUNDLE_ID" = "com.zxystd.AirportItlwm" ] || fail "Bundle ID 不符: ${BUNDLE_ID:-<空>}"

if ! nm -C "$SRC/Contents/MacOS/AirportItlwm" 2>/dev/null | grep -q "runWMIVdevCreate"; then
    echo "⚠ 警告：产物里没有 runWMIVdevCreate 符号，可能不是 M1-b2 构建"
fi

# 未定义的本项目符号 = kext 加载时必然失败（内核里没有这些符号）。
# 典型的成因：声明了成员函数但忘了写定义，链接器对 kext 不报错，只有重启后
# 才发现驱动根本没加载。
UNDEF=$(nm -u -C "$SRC/Contents/MacOS/AirportItlwm" 2>/dev/null | grep -E "Ath10kHal|AirportItlwm::" | head -5)
if [ -n "$UNDEF" ]; then
    echo "✗ 产物里有未定义的本项目符号（kext 会加载失败）：" >&2
    echo "$UNDEF" >&2
    exit 1
fi
echo "✓ 无未定义的本项目符号"

if [ ! -d "/Volumes/ESP/EFI/OC/Kexts" ]; then
    fail "ESP 未挂载（缺 /Volumes/ESP/EFI/OC/Kexts）。请先执行：
      diskutil list                                   # 找到 EFI 分区，例如 disk0s1
      sudo diskutil mount disk0s1                     # 挂载后应出现 /Volumes/ESP
    挂载好以后重新运行本脚本。"
fi

SRC_MD5=$(md5 -q "$SRC/Contents/MacOS/AirportItlwm")
echo ""
echo "源 kext : $SRC"
echo "目标    : $DST"
echo "二进制  : md5=$SRC_MD5"
[ -d "$DST" ] && echo "现有    : 存在（将备份后替换）" || echo "现有    : 不存在（首次部署）"

# ---------------------------------------------------------------- 执行
if [ "$MODE" = "--dry-run" ]; then
    step "dry-run：检查通过，未修改 EFI"
    echo "实际执行请去掉 --dry-run"
    exit 0
fi

if [ "$MODE" != "--yes" ]; then
    echo ""
    echo "即将执行："
    [ -d "$DST" ] && echo "  1) 备份 $DST"
    echo "  2) sudo rm -rf $DST"
    echo "  3) sudo cp -R $SRC $DST"
    echo ""
    printf "确认替换？输入 yes 继续: "
    read -r answer
    [ "$answer" = "yes" ] || fail "已取消，未做任何修改"
fi

if [ -d "$DST" ]; then
    step "备份现有 kext"
    cp -R "$DST" "$BACKUP"
    echo "✓ 备份到 $BACKUP"
    OLD_MD5=$([ -f "$BACKUP/Contents/MacOS/AirportItlwm" ] && md5 -q "$BACKUP/Contents/MacOS/AirportItlwm" || echo "<无二进制>")
    echo "  旧二进制 md5=$OLD_MD5"
fi

step "替换 kext"
sudo rm -rf "$DST"
sudo cp -R "$SRC" "$DST"

step "校验部署结果"
[ -f "$DST/Contents/MacOS/AirportItlwm" ] || fail "复制后缺可执行文件"
DST_MD5=$(md5 -q "$DST/Contents/MacOS/AirportItlwm")
[ "$DST_MD5" = "$SRC_MD5" ] || fail "md5 不一致: 源=$SRC_MD5 目标=$DST_MD5"
echo "✓ md5 一致: $DST_MD5"

/usr/libexec/PlistBuddy -c "Print :CFBundleIdentifier" "$DST/Contents/Info.plist" >/dev/null \
    || fail "部署后的 Info.plist 无法解析"
echo "✓ Info.plist 可解析（$BUNDLE_ID）"

if /usr/libexec/PlistBuddy -c "Print :IOKitPersonalities:NetworkController:IOPCIMatch" \
        "$DST/Contents/Info.plist" 2>/dev/null | grep -qi "0042168c"; then
    echo "✓ IOPCIMatch 含 0x0042168c（QCA9377）"
else
    echo "⚠ 警告：IOPCIMatch 里没找到 0x0042168c，设备可能不会被匹配"
fi

sync

step "完成"
cat <<EOF
kext 已替换并校验通过。现在重启：

  sudo shutdown -r now

重启后等 30 秒，执行：

  cd /Users/zhangjr/Desktop/vibe-coding/ath10Wlan && sh scripts/verify-m1b2.sh
EOF