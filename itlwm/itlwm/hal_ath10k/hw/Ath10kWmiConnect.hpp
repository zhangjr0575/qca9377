#pragma once

#include <IOKit/IOTypes.h>

// htcWriteLE16/32、htcZero、writeChannelPower
#include "Ath10kWmiScan.hpp"

// M1-d：连接（Association）所需的 WMI 命令 + HTT TX 描述符。
//
// 移植自 Ath10kMac（同板卡已验证实现）：`Ath10kMac/src/kext/Ath10kConnection.hpp`。
// 只换命名与注释，**字节布局必须逐字保持一致**——这些偏移是真机试出来的。
//
// 关键结论（M1-d 设计依据）：
//   1. QCA9377 固件**不做 SME**：认证/关联帧必须由主机构造并通过 HTT TX（CE4）发出，
//      固件只负责空口收发；响应从 WMI MGMT_RX 上来（M1-c1 已在解析）。
//   2. 连接命令序列（全部走 CE3 / WMI）：
//        VDEV_START(0x5003) → PEER_CREATE(0x6001) → PEER_ASSOC(0x6005)
//        → [HTT TX 发 AUTH / ASSOC 帧]
//        → PEER_SET_AUTHORIZE(0x6004) → VDEV_UP(0x5005)
//   3. 数据面（含 EAPOL）同样走 HTT TX，描述符由 buildHTTPacket 生成。
namespace Ath10k {

// ---- 连接相关命令 ID ----
static constexpr UInt32 kWmiVdevStartCmdId      = 0x5003;
static constexpr UInt32 kWmiVdevUpCmdId         = 0x5005;
static constexpr UInt32 kWmiPeerCreateCmdId     = 0x6001;
static constexpr UInt32 kWmiPeerAuthorizeCmdId  = 0x6004;
static constexpr UInt32 kWmiPeerAssocCmdId      = 0x6005;

// ---- 命令内 TLV tag ----
static constexpr UInt32 kWmiTagVdevStartParams  = 88;
static constexpr UInt32 kWmiTagVdevUpParams     = 92;
static constexpr UInt32 kWmiTagPeerCreateParams = 97;
static constexpr UInt32 kWmiTagPeerSetParam     = 100;
static constexpr UInt32 kWmiTagPeerAssocParams  = 101;
static constexpr UInt32 kWmiTagPeerAssocRates   = 17;
static constexpr UInt32 kWmiTagPeerAssocVht     = 102;

// ---- HTT TX ----
static constexpr UInt32 kWmiHttTxHtcBytes       = 256;   // HTC 头 + HTT TX 描述符
static constexpr UInt32 kWmiHttTxPrefetchMax    = 50;    // 描述符里内联的帧前缀上限
static constexpr UInt16 kWmiHttTxIdModulus      = 1056;  // 固件配置的 TX desc ID 数量
static constexpr UInt32 kWmiEthernetFrameMax    = 1600;

// HTT TX 的 frame_type：>=3 的固件统一用 TX_FRM（管理帧与以太网帧同一个值），
// 具体行为由第 9 字节的 flags 区分。
static constexpr UInt8 kWmiHttTxFlagsMgmt       = 0x65;
static constexpr UInt8 kWmiHttTxFlagsDataSecured = 0x42;
static constexpr UInt8 kWmiHttTxFlagsDataOpen   = 0x46;

// 管理帧固定用 TID 17（固件约定），数据帧按 AC 映射
static constexpr UInt8 kWmiHttTxMgmtTid         = 17;

// ---------------------------------------------------------------------------
// 连接参数：从 net80211 的 node 里抽出来的最小集合
// ---------------------------------------------------------------------------
struct WmiConnectParams {
    UInt8  bssid[6] = {};
    UInt8  localMac[6] = {};
    UInt8  ssid[32] = {};
    UInt32 ssidLen = 0;

    UInt32 channel = 0;          // 主信道（1~165）
    UInt32 centerChannel = 0;    // 中心信道（20MHz 时等于 channel）
    UInt32 beaconInterval = 100;
    UInt32 dtimPeriod = 1;
    UInt32 txPowerHalfDbm = 20;  // 10 dBm

    bool   htEnabled = false;
    bool   vhtEnabled = false;
    bool   secure = false;       // RSN/WPA
    bool   wmmPresent = false;

    UInt32 aid = 0;
    UInt32 peerCaps = 0;         // 关联响应的 capability 字段
    UInt32 peerHtCaps = 0;       // AP 的 HT Capability Info
    UInt32 peerVhtCaps = 0;
    UInt8  nss = 1;
    UInt8  width = 20;

    UInt8  rateCount = 0;        // 支持的 legacy rate（已 &0x7f）
    UInt8  rates[16] = {};
};

// phy mode 编码（与 Linux ath10k / 参考实现一致）
inline UInt32 connectPhyMode(UInt32 channel, bool ht, bool vht, UInt8 width)
{
    if (vht)
        return width == 80 ? 10 : (width == 40 ? 9 : 8);
    if (ht) {
        if (channel > 14)
            return width == 40 ? 6 : 4;
        return width == 40 ? 7 : 5;
    }
    return channel > 14 ? 0 : 1;
}

inline UInt32 connectFreq(UInt32 channel)
{
    if (channel == 0)
        return 0;
    return channel <= 14 ? 2407 + 5 * channel : 5000 + 5 * channel;
}

// TLV 头：{len, tag}（命令体从 f+12 开始）
inline void connectTLV(UInt8 *p, UInt16 tag, UInt16 bytes)
{
    htcWriteLE16(p, bytes);
    htcWriteLE16(p + 2, tag);
}

// 命令封尾：HTC 头 + 命令 ID
inline void connectFinish(UInt8 *f, UInt8 endpoint, UInt8 seq, UInt32 bytes, UInt32 cmd)
{
    f[0] = endpoint;
    f[1] = 1;                       // NEED_CREDIT_UPDATE
    f[5] = seq;
    htcWriteLE16(f + 2, static_cast<UInt16>(bytes - 8));
    htcWriteLE32(f + 8, cmd);
}

// ---------------------------------------------------------------------------
// VDEV_START（120 字节）
//   12..15  TLV(88, 72)  vdev_start_params
//   16..87  beacon_int / dtim / ssid_len / ssid[32]
//   88..115 TLV(80, 24)  channel
//   116..119 TLV(18, 0)  空数组
// ---------------------------------------------------------------------------
static constexpr UInt32 kWmiVdevStartBytes = 120;
inline bool buildWMIVdevStart(UInt8 *f, UInt32 capacity, UInt8 endpoint, UInt8 seq,
                              const WmiConnectParams &c)
{
    if (!f || capacity < kWmiVdevStartBytes || !endpoint || endpoint == 0xff ||
        c.ssidLen == 0 || c.ssidLen > 32 || c.channel == 0)
        return false;

    htcZero(f, kWmiVdevStartBytes);
    connectTLV(f + 12, static_cast<UInt16>(kWmiTagVdevStartParams), 72);
    UInt8 *p = f + 16;
    htcWriteLE32(p + 8, c.beaconInterval);
    htcWriteLE32(p + 12, c.dtimPeriod ? c.dtimPeriod : 1);
    htcWriteLE32(p + 20, c.ssidLen);
    for (UInt32 i = 0; i < c.ssidLen; ++i)
        p[24 + i] = c.ssid[i];

    connectTLV(f + 88, static_cast<UInt16>(kWmiTagChannel), 24);
    htcWriteLE32(f + 92, connectFreq(c.channel));
    htcWriteLE32(f + 96, connectFreq(c.centerChannel ? c.centerChannel : c.channel));
    htcWriteLE32(f + 100, 0);                                  // band_center_freq2
    htcWriteLE32(f + 104, connectPhyMode(c.channel, c.htEnabled, c.vhtEnabled, c.width) |
                            (c.htEnabled ? (1u << 11) : 0u) |
                            (c.vhtEnabled ? (1u << 12) : 0u));
    writeChannelPower(f + 108, f + 112, static_cast<UInt8>(c.txPowerHalfDbm));
    connectTLV(f + 116, 18, 0);

    connectFinish(f, endpoint, seq, kWmiVdevStartBytes, kWmiVdevStartCmdId);
    return true;
}

// ---------------------------------------------------------------------------
// PEER_CREATE（32 字节）：TLV(97,16)，payload 偏移 4 起是 peer 地址
// ---------------------------------------------------------------------------
static constexpr UInt32 kWmiPeerCreateBytes = 32;
inline bool buildWMIPeerCreate(UInt8 *f, UInt32 capacity, UInt8 endpoint, UInt8 seq,
                               const UInt8 *peer)
{
    if (!f || capacity < kWmiPeerCreateBytes || !endpoint || endpoint == 0xff || !peer)
        return false;
    htcZero(f, kWmiPeerCreateBytes);
    connectTLV(f + 12, static_cast<UInt16>(kWmiTagPeerCreateParams), 16);
    for (UInt32 i = 0; i < 6; ++i)
        f[16 + 4 + i] = peer[i];
    connectFinish(f, endpoint, seq, kWmiPeerCreateBytes, kWmiPeerCreateCmdId);
    return true;
}

// ---------------------------------------------------------------------------
// PEER_ASSOC（120 + rates + htRates 字节）：TLV(101,76) + 两个 rate 数组
// ---------------------------------------------------------------------------
inline UInt32 wmiPeerAssocBytes(const WmiConnectParams &c)
{
    const UInt32 rates = (c.rateCount + 3u) & ~3u;
    const UInt32 htRates = c.htEnabled ? 4u : 0u;   // 1x1：一个 HT MCS 集
    return 120 + rates + htRates;
}

inline bool buildWMIPeerAssoc(UInt8 *f, UInt32 capacity, UInt8 endpoint, UInt8 seq,
                              const WmiConnectParams &c)
{
    const UInt32 rates = (c.rateCount + 3u) & ~3u;
    const UInt32 htRates = c.htEnabled ? 4u : 0u;
    const UInt32 bytes = wmiPeerAssocBytes(c);
    if (!f || capacity < bytes || !endpoint || endpoint == 0xff ||
        c.rateCount == 0 || c.rateCount > 16)
        return false;

    htcZero(f, bytes);
    connectTLV(f + 12, static_cast<UInt16>(kWmiTagPeerAssocParams), 76);
    UInt8 *p = f + 16;

    for (UInt32 i = 0; i < 6; ++i)
        p[i] = c.bssid[i];                       // peer_addr
    htcWriteLE32(p + 12, 1);                     // peer_num
    htcWriteLE32(p + 16, c.aid);
    UInt32 flags = (c.secure ? (5u | 16u) : 1u); // 5 = associated | authenticated
    if (c.wmmPresent)
        flags |= 2u;
    if (c.htEnabled)
        flags |= 0x1000u;
    if (c.vhtEnabled)
        flags |= 0x02000000u;
    if (c.width == 40)
        flags |= 0x2000u;
    if (c.width == 80)
        flags |= 0x04000000u;
    // 1x1 设备固定 SMPS static（与参考实现的 1x1 分支一致）
    flags |= 0x40000u;
    htcWriteLE32(p + 20, flags);
    htcWriteLE32(p + 24, c.peerCaps);
    htcWriteLE32(p + 28, 1);                     // peer_assoc_num
    htcWriteLE32(p + 48, c.nss);
    htcWriteLE32(p + 56, connectPhyMode(c.channel, c.htEnabled, c.vhtEnabled, c.width));
    if (c.htEnabled) {
        htcWriteLE32(p + 32, c.peerHtCaps);
        htcWriteLE32(p + 36, (1u << (13 + 3)) - 1);  // 最大 A-MPDU（64 KiB 指数 3）
        htcWriteLE32(p + 40, 6);                     // MPDU density 8us
        htcWriteLE32(p + 44, 8u | (c.width >= 40 ? 2u : 0u) | 1u);
        htcWriteLE32(p + 72, 1);                     // HT MCS 集数量
        if (c.vhtEnabled)
            htcWriteLE32(p + 52, c.peerVhtCaps);
    }
    htcWriteLE32(p + 68, c.rateCount);

    connectTLV(f + 92, static_cast<UInt16>(kWmiTagPeerAssocRates),
               static_cast<UInt16>(rates));
    for (UInt32 i = 0; i < c.rateCount; ++i)
        f[96 + i] = c.rates[i] & 0x7f;

    if (c.htEnabled) {
        UInt8 *ht = f + 96 + rates;
        connectTLV(ht, static_cast<UInt16>(kWmiTagPeerAssocRates),
                   static_cast<UInt16>(htRates));
        ht[4] = 0xff;                                // MCS0-7
        if (c.vhtEnabled) {
            UInt8 *v = ht + 4 + htRates;
            connectTLV(v, static_cast<UInt16>(kWmiTagPeerAssocVht), 16);
            // rx/tx MCS map：1x1 只支持 MCS0-7（其余位填 0xffff 表示不支持）
            htcWriteLE32(v + 8, 0xfffeu);
            htcWriteLE32(v + 16, 0xfffeu);
        }
    }

    connectFinish(f, endpoint, seq, bytes, kWmiPeerAssocCmdId);
    return true;
}

// ---------------------------------------------------------------------------
// PEER_SET_AUTHORIZE（36 字节）：TLV(100,20)，param 3 = AUTHORIZE
// ---------------------------------------------------------------------------
static constexpr UInt32 kWmiPeerAuthorizeBytes = 36;
inline bool buildWMIPeerAuthorize(UInt8 *f, UInt32 capacity, UInt8 endpoint, UInt8 seq,
                                  const UInt8 *peer, bool authorized)
{
    if (!f || capacity < kWmiPeerAuthorizeBytes || !endpoint || endpoint == 0xff || !peer)
        return false;
    htcZero(f, kWmiPeerAuthorizeBytes);
    connectTLV(f + 12, static_cast<UInt16>(kWmiTagPeerSetParam), 20);
    for (UInt32 i = 0; i < 6; ++i)
        f[20 + i] = peer[i];                 // payload +4
    htcWriteLE32(f + 28, 3);                 // param_id = AUTHORIZE
    htcWriteLE32(f + 32, authorized ? 1u : 0u);
    connectFinish(f, endpoint, seq, kWmiPeerAuthorizeBytes, kWmiPeerAuthorizeCmdId);
    return true;
}

// ---------------------------------------------------------------------------
// VDEV_UP（32 字节）：TLV(92,16)，payload = AID + BSSID
// ---------------------------------------------------------------------------
static constexpr UInt32 kWmiVdevUpBytes = 32;
inline bool buildWMIVdevUp(UInt8 *f, UInt32 capacity, UInt8 endpoint, UInt8 seq,
                           const WmiConnectParams &c)
{
    if (!f || capacity < kWmiVdevUpBytes || !endpoint || endpoint == 0xff)
        return false;
    htcZero(f, kWmiVdevUpBytes);
    connectTLV(f + 12, static_cast<UInt16>(kWmiTagVdevUpParams), 16);
    htcWriteLE32(f + 20, c.aid);             // payload +4
    for (UInt32 i = 0; i < 6; ++i)
        f[24 + i] = c.bssid[i];              // payload +8
    connectFinish(f, endpoint, seq, kWmiVdevUpBytes, kWmiVdevUpCmdId);
    return true;
}

// ---------------------------------------------------------------------------
// HTT TX 描述符（HTC 头 + HTT 头 + 内联帧前缀）
//
// 布局（参考实现 buildHTTPacket）：
//   0      HTC endpoint
//   1      1 = need credit update
//   2..3   payload 长度
//   5      HTC 序号
//   8      HTT 消息类型 = 1（TX_FRM）
//   9      flags（管理帧 0x65 / 数据 0x42 加密 / 0x46 明文）
//   10..11 0x800 | (tid << 6)   POSTPONED
//   12..13 帧长度
//   14..15 TX descriptor id（用于完成事件匹配，模 1056 轮转）
//   16..19 管理帧/单 buffer：帧的 DMA 地址；否则 fragment 数组地址
//   20..21 0xffff（EXT_TXDESC 缺省）
//   24..   帧前缀（最长 50 字节）
// ---------------------------------------------------------------------------
inline UInt32 buildHTTPacket(UInt8 *f, UInt32 capacity, const UInt8 *packet, UInt32 length,
                             UInt32 packetAddress, UInt32 fragAddress,
                             UInt8 endpoint, UInt8 seq, UInt16 txId, bool management)
{
    if (!f || !packet || capacity < kWmiHttTxHtcBytes || !endpoint || endpoint == 0xff ||
        length < 14 || length > kWmiEthernetFrameMax)
        return 0;
    if (!packetAddress && !fragAddress)
        return 0;

    htcZero(f, kWmiHttTxHtcBytes);
    f[0] = endpoint;
    f[5] = seq;

    UInt32 prefetch = length < kWmiHttTxPrefetchMax ? length : kWmiHttTxPrefetchMax;
    const UInt32 bytes = 24 + ((prefetch + 3u) & ~3u);

    f[8] = 1;                                            // HTT_TX_FRM
    f[9] = management ? kWmiHttTxFlagsMgmt : kWmiHttTxFlagsDataOpen;
    const UInt8 tid = management ? kWmiHttTxMgmtTid : 0;  // 数据面后续按 AC 映射
    htcWriteLE16(f + 10, static_cast<UInt16>(0x800u | (static_cast<UInt16>(tid) << 6)));
    htcWriteLE16(f + 12, static_cast<UInt16>(length));
    htcWriteLE16(f + 14, txId);
    htcWriteLE32(f + 16, management ? packetAddress : fragAddress);
    htcWriteLE32(f + 20, 0xffff);
    for (UInt32 i = 0; i < prefetch; ++i)
        f[24 + i] = packet[i];

    htcWriteLE16(f + 2, static_cast<UInt16>(bytes - 8));
    return bytes;
}

// ---------------------------------------------------------------------------
// AUTH / ASSOC 请求帧（HTT TX 的 payload）
//   Auth：30 字节（fixed=alg 1 + seq 1）
//   Assoc：28 + IEs（SSID / rates / RSN ...）
// 只提供"最小可用"版本：SSID + 速率 + （可选）RSN 由调用方追加。
// ---------------------------------------------------------------------------
inline UInt32 buildAuthRequest(UInt8 *f, UInt32 capacity, const WmiConnectParams &c,
                               UInt16 sequence, UInt16 authAlg = 1, UInt16 authSeq = 1)
{
    if (!f || capacity < 30)
        return 0;
    htcZero(f, 30);
    f[0] = 0xb0;                                  // auth
    f[1] = 0x00;
    for (UInt32 i = 0; i < 6; ++i) {
        f[4 + i] = c.bssid[i];                    // addr1
        f[10 + i] = c.localMac[i];                // addr2
        f[16 + i] = c.bssid[i];                   // addr3
    }
    htcWriteLE16(f + 22, static_cast<UInt16>((sequence & 0xfff) << 4));
    htcWriteLE16(f + 24, authAlg);
    htcWriteLE16(f + 26, authSeq);
    return 30;
}

inline UInt32 buildAssocRequest(UInt8 *f, UInt32 capacity, const WmiConnectParams &c,
                                UInt16 sequence, const UInt8 *rsnIe, UInt32 rsnIeLen)
{
    if (!f || c.ssidLen == 0 || c.ssidLen > 32 || c.rateCount == 0)
        return 0;
    const UInt32 tail = 2 + c.ssidLen + 2 + c.rateCount + (rsnIe && rsnIeLen ? rsnIeLen : 0);
    if (capacity < 28 + tail)
        return 0;

    htcZero(f, 28 + tail);
    f[0] = 0x00;                                  // assoc req
    f[1] = 0x00;
    for (UInt32 i = 0; i < 6; ++i) {
        f[4 + i] = c.bssid[i];
        f[10 + i] = c.localMac[i];
        f[16 + i] = c.bssid[i];
    }
    htcWriteLE16(f + 22, static_cast<UInt16>((sequence & 0xfff) << 4));
    UInt16 cap = 1;                               // ESS
    if (c.wmmPresent)
        cap |= 0x20;                              // short slot
    if (c.secure)
        cap |= 0x10;                              // privacy
    htcWriteLE16(f + 24, cap);
    htcWriteLE16(f + 26, 1);                      // listen interval

    UInt32 n = 28;
    f[n++] = 0;                                   // SSID
    f[n++] = static_cast<UInt8>(c.ssidLen);
    for (UInt32 i = 0; i < c.ssidLen; ++i)
        f[n++] = c.ssid[i];

    f[n++] = 1;                                   // Rates
    f[n++] = static_cast<UInt8>(c.rateCount);
    for (UInt32 i = 0; i < c.rateCount; ++i)
        f[n++] = c.rates[i] & 0x7f;

    if (rsnIe && rsnIeLen) {
        for (UInt32 i = 0; i < rsnIeLen; ++i)
            f[n++] = rsnIe[i];
    }
    return n;
}

} // namespace Ath10k