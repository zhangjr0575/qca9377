#pragma once

#include <IOKit/IOTypes.h>

#include "Ath10kHTC.hpp"

// 精简版 WMI 扫描 ABI（Linux v6.12 wmi-tlv.h + Ath10kMac 同板卡实现）。
//
// 本文件只做两件事：
//   1. 构造扫描所需的 WMI 命令帧（PDEV_SET_PARAM / SCAN_CHAN_LIST / START_SCAN / STOP_SCAN）；
//   2. 从 WMI 事件里抽出扫描进度与扫描到的 BSS（SSID/BSSID/信道/SNR）。
//
// 信道表与功率编码与参考实现保持一致：保守 10 dBm 上限，DFS 只收不发。
namespace Ath10k {

// ---- 命令 ID ----
static constexpr UInt32 kWmiScanStartCmdId   = 0x3001; // WMI_TLV_START_SCAN_CMDID
static constexpr UInt32 kWmiScanStopCmdId    = 0x3002; // WMI_TLV_STOP_SCAN_CMDID
static constexpr UInt32 kWmiScanChanListCmdId = 0x3003;
static constexpr UInt32 kWmiPdevSetParamCmdId = 0x4003;

// ---- 命令内 TLV tag ----
static constexpr UInt32 kWmiTagStartScanCmd  = 77;
static constexpr UInt32 kWmiTagStopScanCmd   = 78;
static constexpr UInt32 kWmiTagScanChanList  = 79;
static constexpr UInt32 kWmiTagChannel       = 80;
static constexpr UInt32 kWmiTagPdevSetParam  = 82;

// ---- 事件 ID ----
static constexpr UInt32 kWmiScanEventId   = 0x3001; // WMI_TLV_SCAN_EVENTID
static constexpr UInt32 kWmiMgmtRxEventId = 0x7001; // WMI_TLV_MGMT_RX_EVENTID

// ---- 事件内 TLV tag ----
static constexpr UInt32 kWmiTagScanEvent = 36;
static constexpr UInt32 kWmiTagMgmtRx    = 44;

// 扫描参数（与参考实现一致）
static constexpr UInt32 kWmiScanId        = 0xa036;
static constexpr UInt32 kWmiScanRequestId = 0xa001;
static constexpr UInt32 kWmiScanFastDwellMs = 200;
static constexpr UInt32 kWmiScanDwellMs     = 500;
static constexpr UInt32 kWmiScanFirmwareMs  = 30000;
// HAL 侧收集窗口：扫描完成后再多收一小段，避免漏掉尾部 MGMT_RX
// 实测一轮扫描约 1 s 完成；窗口缩短可减少单轮的最坏暴露时间
// （M1-c3 真机卡死复盘：失败路径必须有界，见 docs/13）
static constexpr UInt32 kWmiScanCollectMs = 8000;
static constexpr UInt32 kWmiScanDrainMs   = 1500;
// 单帧管理帧上限（WMI 接收缓冲 2048 B，HTC 头 12 B）
static constexpr UInt32 kWmiScanMaxFrameBytes = 2048;

// 信道基准功率：半 dBm，10 dBm
static constexpr UInt8 kWmiScanPowerHalfDbm = 20;

struct WMIScanChannel {
    UInt32 channel;
    UInt32 frequency;
};

// QCA9377 支持的 2.4G + 5G 信道（不含 6 GHz）
static constexpr WMIScanChannel kWMIScanPlan[] = {
    {1, 2412}, {2, 2417}, {3, 2422}, {4, 2427}, {5, 2432}, {6, 2437},
    {7, 2442}, {8, 2447}, {9, 2452}, {10, 2457}, {11, 2462}, {12, 2467},
    {13, 2472},
    {36, 5180}, {40, 5200}, {44, 5220}, {48, 5240},
    {52, 5260}, {56, 5280}, {60, 5300}, {64, 5320},
    {100, 5500}, {104, 5520}, {108, 5540}, {112, 5560}, {116, 5580},
    {120, 5600}, {124, 5620}, {128, 5640}, {132, 5660}, {136, 5680},
    {140, 5700}, {144, 5720},
    {149, 5745}, {153, 5765}, {157, 5785}, {161, 5805}, {165, 5825}
};

static constexpr UInt32 kWmiScanChannelCount =
    sizeof(kWMIScanPlan) / sizeof(kWMIScanPlan[0]);

inline bool wmiScanChannelAllowed(UInt32 channel)
{
    for (UInt32 i = 0; i < kWmiScanChannelCount; ++i)
        if (kWMIScanPlan[i].channel == channel)
            return true;
    return false;
}

inline bool wmiChannelNeedsDFS(UInt32 channel)
{
    return channel >= 52 && channel <= 144 && wmiScanChannelAllowed(channel);
}

// 频率（MHz）→ IEEE 信道号。M1-c3 把 MGMT_RX 里的频率交给 net80211 时要用：
// rxi_chan 必须是信道号，net80211 用它与 DS 参数、ic_bss->ni_chan 比对。
inline UInt32 wmiFreqToChannel(UInt32 freq)
{
    if (freq == 2484)
        return 14;
    if (freq >= 2412 && freq < 2484)
        return (freq - 2407) / 5;
    if (freq >= 5000 && freq < 5925)
        return (freq - 5000) / 5;
    return 0;
}

// 2.4G 用 MODE_11G(1)，5G 用 MODE_11A(0)。
// 只有 DFS 信道标 PASSIVE；非 DFS 保持可主动扫描——否则固件会把整张表当
// 被动信道，START_SCAN 即便不带 PASSIVE 位也只会被动驻留。
inline UInt32 wmiChannelListFlags(UInt32 channel)
{
    const UInt32 mode = channel <= 14 ? 0x1u : 0x0u;
    return mode | (wmiChannelNeedsDFS(channel) ? 0x80u : 0x0u);
}

// wmi_channel 的 reg_info 打包：低 16 位 max，高 16 位 min（半 dBm）
inline void writeChannelPower(UInt8 *reg0, UInt8 *reg1, UInt8 halfDbm)
{
    htcWriteLE32(reg0, (UInt32(halfDbm) << 8) | (UInt32(halfDbm) << 16));
    htcWriteLE32(reg1, UInt32(halfDbm) << 8);
}

// ---------------------------------------------------------------------------
// PDEV_SET_PARAM：param 1 = TX chain mask，2 = RX chain mask
// ---------------------------------------------------------------------------
static constexpr UInt32 kWmiPdevSetParamFrameBytes = 28;
inline bool buildWMIPdevSetParam(UInt8 *f, UInt32 capacity, UInt8 endpoint,
                                 UInt32 param, UInt32 value, UInt8 seq)
{
    if (!f || capacity < kWmiPdevSetParamFrameBytes || !endpoint || endpoint == 0xff)
        return false;
    htcZero(f, kWmiPdevSetParamFrameBytes);
    f[0] = endpoint;
    f[1] = 1;               // NEED_CREDIT_UPDATE
    f[5] = seq;
    htcWriteLE16(f + 2, 20);
    htcWriteLE32(f + 8, kWmiPdevSetParamCmdId);
    htcWriteLE16(f + 12, 12);
    htcWriteLE16(f + 14, static_cast<UInt16>(kWmiTagPdevSetParam));
    htcWriteLE32(f + 16, 0);        // pdev_id
    htcWriteLE32(f + 20, param);
    htcWriteLE32(f + 24, value);
    return true;
}

// ---------------------------------------------------------------------------
// SCAN_CHAN_LIST：把完整信道表交给固件
// ---------------------------------------------------------------------------
inline bool buildWMIScanChanList(UInt8 *f, UInt32 capacity, UInt8 endpoint, UInt8 seq)
{
    const UInt32 count = kWmiScanChannelCount;
    const UInt32 frameBytes = 24 + 28 * count;
    if (!f || capacity < frameBytes || !endpoint || endpoint == 0xff)
        return false;

    htcZero(f, frameBytes);
    f[0] = endpoint;
    f[1] = 1;
    f[5] = seq;
    htcWriteLE16(f + 2, static_cast<UInt16>(frameBytes - 8));
    htcWriteLE32(f + 8, kWmiScanChanListCmdId);
    htcWriteLE16(f + 12, 4);
    htcWriteLE16(f + 14, static_cast<UInt16>(kWmiTagScanChanList));
    htcWriteLE32(f + 16, count);
    htcWriteLE16(f + 20, static_cast<UInt16>(28 * count));
    htcWriteLE16(f + 22, 18);       // ARRAY_STRUCT

    for (UInt32 i = 0; i < count; ++i) {
        UInt8 *ch = f + 24 + 28 * i;
        htcWriteLE16(ch, 24);
        htcWriteLE16(ch + 2, static_cast<UInt16>(kWmiTagChannel));
        htcWriteLE32(ch + 4, kWMIScanPlan[i].frequency);   // freq
        htcWriteLE32(ch + 8, kWMIScanPlan[i].frequency);   // band_center_freq1
        htcWriteLE32(ch + 12, 0);                          // band_center_freq2
        htcWriteLE32(ch + 16, wmiChannelListFlags(kWMIScanPlan[i].channel));
        writeChannelPower(ch + 20, ch + 24,
                          wmiScanChannelAllowed(kWMIScanPlan[i].channel)
                              ? kWmiScanPowerHalfDbm : 0);
    }
    return true;
}

// ---------------------------------------------------------------------------
// START_SCAN（140 + 4 * 信道数）
//
// 布局（与参考实现一致）：
//   0..11    HTC 头
//   12..15   TLV(100, 77) —— start_scan_cmd
//   16..75   common[15]
//   76..79   burst_duration
//   80..83   num_channels
//   84..87   num_bssids
//   88..91   num_ssids
//   92..95   ie_len
//   96..99   num_probes
//   100..115 mac / mask
//   116..119 TLV(4*n, 16) 频道频率数组
//   120..    频率数组
//   tail     SSID 数组 / BSSID 数组 / IE 数组
// ---------------------------------------------------------------------------
inline bool buildWMIStartScan(UInt8 *f, UInt32 capacity, UInt8 endpoint, UInt8 seq,
                              bool passive, bool fast)
{
    // 首轮扫描只访问非 DFS 信道：DFS 需要被动驻留，代价高且当前阶段
    // 还连不上；固件侧的信道表（SCAN_CHAN_LIST）仍然给全量。
    UInt32 count = 0;
    for (UInt32 i = 0; i < kWmiScanChannelCount; ++i)
        if (kWMIScanPlan[i].channel <= 14 || !wmiChannelNeedsDFS(kWMIScanPlan[i].channel))
            ++count;
    const UInt32 frameBytes = 140 + 4 * count;
    if (!f || capacity < frameBytes || !endpoint || endpoint == 0xff)
        return false;

    htcZero(f, frameBytes);
    f[0] = endpoint;
    f[1] = 1;
    f[5] = seq;
    htcWriteLE16(f + 2, static_cast<UInt16>(frameBytes - 8));
    htcWriteLE32(f + 8, kWmiScanStartCmdId);
    htcWriteLE16(f + 12, 100);
    htcWriteLE16(f + 14, static_cast<UInt16>(kWmiTagStartScanCmd));

    const UInt32 dwell = fast ? kWmiScanFastDwellMs : kWmiScanDwellMs;
    // 事件掩码 0x1ff；flags bit0=PASSIVE，0x10=HIGH_PRIORITY，0x20=FAST_SCAN
    const UInt32 flags = 0x30u | (passive ? 0x01u : 0x00u);
    const UInt32 common[15] = {
        kWmiScanId, kWmiScanRequestId, 0 /*vdev*/, 1 /*priority*/, 0x1ff,
        50 /*probe delay*/, dwell, 50 /*rest*/, 500 /*max rest*/, 0, 0, 0,
        kWmiScanFirmwareMs, 5, flags
    };
    for (UInt32 i = 0; i < 15; ++i)
        htcWriteLE32(f + 16 + 4 * i, common[i]);

    htcWriteLE32(f + 76, 0);            // burst_duration
    htcWriteLE32(f + 80, count);        // num_channels
    htcWriteLE32(f + 84, 1);            // num_bssids（广播）
    htcWriteLE32(f + 88, 0);            // num_ssids
    htcWriteLE32(f + 92, 0);            // ie_len
    htcWriteLE32(f + 96, 0);            // num_probes

    htcWriteLE16(f + 116, static_cast<UInt16>(4 * count));
    htcWriteLE16(f + 118, 16);          // ARRAY_UINT32
    UInt32 slot = 0;
    for (UInt32 i = 0; i < kWmiScanChannelCount; ++i) {
        if (kWMIScanPlan[i].channel > 14 && wmiChannelNeedsDFS(kWMIScanPlan[i].channel))
            continue;
        htcWriteLE32(f + 120 + 4 * slot++, kWMIScanPlan[i].frequency);
    }

    const UInt32 tail = 120 + 4 * count;
    htcWriteLE16(f + tail + 2, 19);     // 空 SSID 数组
    htcWriteLE16(f + tail + 4, 8);
    htcWriteLE16(f + tail + 6, 19);     // BSSID 数组（广播）
    for (UInt32 i = 0; i < 6; ++i)
        f[tail + 8 + i] = 0xff;
    htcWriteLE16(f + tail + 18, 17);    // 空 IE 数组
    return true;
}

inline bool buildWMIStopScan(UInt8 *f, UInt32 capacity, UInt8 endpoint, UInt8 seq)
{
    if (!f || capacity < 32 || !endpoint || endpoint == 0xff)
        return false;
    htcZero(f, 32);
    f[0] = endpoint;
    f[1] = 1;
    f[5] = seq;
    htcWriteLE16(f + 2, 24);
    htcWriteLE32(f + 8, kWmiScanStopCmdId);
    htcWriteLE16(f + 12, 16);
    htcWriteLE16(f + 14, static_cast<UInt16>(kWmiTagStopScanCmd));
    htcWriteLE32(f + 16, kWmiScanRequestId);
    htcWriteLE32(f + 20, kWmiScanId);
    htcWriteLE32(f + 24, 0);            // req_type
    htcWriteLE32(f + 28, 0);            // vdev_id
    return true;
}

// ---------------------------------------------------------------------------
// 扫描事件 / 管理帧解析结果
// ---------------------------------------------------------------------------
struct WMIScanInfo {
    // SCAN_EVENT(0x3001)
    bool   scanEvent = false;
    UInt32 scanType = 0;      // 1=STARTED, 2=COMPLETED, ...
    UInt32 scanReason = 0;
    UInt32 scanFreq = 0;
    UInt32 scanId = 0;
    UInt32 scanRequestId = 0;

    // MGMT_RX(0x7001)
    bool   mgmtRx = false;
    UInt32 mgmtDeclaredFrameBytes = 0;
    UInt32 mgmtFreq = 0;
    UInt32 mgmtSnr = 0;
    UInt32 mgmtStatus = 0xffffffffu;
    UInt32 mgmtFrameBytes = 0;

    // 原始 802.11 管理帧（M1-c3）：指向 HTC/WMI 接收缓冲内部，
    // 只在本次解析回调内有效；调用方必须在同一轮里把它拷走。
    const UInt8 *mgmtFrame = nullptr;
    UInt32       mgmtFrameLength = 0;

    // 从 beacon / probe response 里抽出的 BSS 信息
    bool   bssValid = false;
    UInt8  bssMac[6] = {};
    UInt8  bssSsid[32] = {};
    UInt32 bssSsidLen = 0;
    UInt32 bssChannel = 0;
};

// 从 802.11 管理帧里取 SSID / DS 信道
inline void parseBeaconIEs(const UInt8 *frame, UInt32 bytes, WMIScanInfo &out)
{
    out.bssSsidLen = 0;
    out.bssChannel = 0;
    if (!frame || bytes < 36)
        return;
    UInt32 p = 36;   // 24 字节 MAC 头 + 12 字节固定参数
    while (p + 2 <= bytes) {
        const UInt32 id = frame[p];
        const UInt32 len = frame[p + 1];
        p += 2;
        if (len > bytes - p)
            break;
        if (id == 0 && len <= 32) {
            out.bssSsidLen = len;
            for (UInt32 i = 0; i < len; ++i)
                out.bssSsid[i] = frame[p + i];
        } else if (id == 3 && len == 1) {
            out.bssChannel = frame[p];
        }
        p += len;
    }
}

// 解析扫描相关事件；其它事件返回 true 且 out.scanEvent=false
inline const char *parseWMIScanEvent(const UInt8 *f, UInt32 bytes,
                                     UInt32 eventId, WMIScanInfo &out)
{
    out = WMIScanInfo();
    if (!f || bytes < 12)
        return "scan-frame-size";

    const UInt32 trailer = (f[1] & kHTCTrailerPresent) ? f[4] : 0;
    const UInt32 payload = htcReadLE16(f + 2);
    const UInt32 declared = kHTCHeaderBytes + payload;
    if (declared > bytes || trailer > payload)
        return "scan-frame-length";
    const UInt32 end = declared - trailer;

    const UInt8 *frame = nullptr;
    UInt32 frameBytes = 0;

    for (UInt32 p = 12; p < end; ) {
        if (end - p < 4)
            return "scan-tlv-header";
        const UInt32 len = htcReadLE16(f + p);
        const UInt32 tag = htcReadLE16(f + p + 2);
        p += 4;
        if (len > end - p)
            return "scan-tlv-size";

        if (eventId == kWmiScanEventId && tag == kWmiTagScanEvent) {
            if (len < 24)
                return "scan-event-size";
            if (out.scanEvent)
                return "scan-event-duplicate";
            out.scanType = wmiReadLE32(f + p);
            out.scanReason = wmiReadLE32(f + p + 4);
            out.scanFreq = wmiReadLE32(f + p + 8);
            out.scanRequestId = wmiReadLE32(f + p + 12);
            out.scanId = wmiReadLE32(f + p + 16);
            out.scanEvent = true;
        } else if (eventId == kWmiMgmtRxEventId && tag == kWmiTagMgmtRx) {
            if (len < 40)
                return "mgmt-rx-size";
            out.mgmtFreq = wmiReadLE32(f + p);
            out.mgmtSnr = wmiReadLE32(f + p + 4);
            out.mgmtDeclaredFrameBytes = wmiReadLE32(f + p + 16);
            out.mgmtStatus = wmiReadLE32(f + p + 20);
            out.mgmtRx = true;
        } else if (eventId == kWmiMgmtRxEventId && tag == 17) {
            if (frame)
                return "mgmt-rx-duplicate-array";
            frame = f + p;
            frameBytes = len;
        }
        p += len;
    }

    if (out.mgmtRx && frame && frameBytes >= 24) {
        out.mgmtFrameBytes = frameBytes;
        // M1-c3：把原始帧一起交出去，交给 net80211 建节点（字段由协议栈自己填）
        out.mgmtFrame = frame;
        out.mgmtFrameLength = frameBytes;
        if (out.mgmtStatus == 0) {
            for (UInt32 i = 0; i < 6; ++i)
                out.bssMac[i] = frame[16 + i];
            UInt8 nonzero = 0;
            for (UInt32 i = 0; i < 6; ++i)
                nonzero |= out.bssMac[i];
            out.bssValid = nonzero != 0 && (out.bssMac[0] & 1) == 0;
            if (out.bssValid)
                parseBeaconIEs(frame, frameBytes, out);
        }
    }
    return nullptr;
}

// START_SCAN 实际访问的信道数（首轮跳过 DFS）
inline UInt32 wmiStartScanChannelCount()
{
    UInt32 count = 0;
    for (UInt32 i = 0; i < kWmiScanChannelCount; ++i)
        if (kWMIScanPlan[i].channel <= 14 || !wmiChannelNeedsDFS(kWMIScanPlan[i].channel))
            ++count;
    return count;
}

inline UInt32 wmiStartScanFrameBytes()
{
    return 140 + 4 * wmiStartScanChannelCount();
}

} // namespace Ath10k