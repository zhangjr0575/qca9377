#pragma once

#include <IOKit/IOTypes.h>

#include "Ath10kHTC.hpp"

// HTT 接收环 setup（QCA6174/QCA9377 PCIe，ring32 ABI）。
//
// 这一步之前，固件虽然能扫到 BSS，但不会把收到的帧交给主机：
// 扫描期间的管理帧与后续数据帧都需要主机先把 HTT RX 环交出去。
//
// 内存模型（与 Linux htt_rx_desc_v1 / 参考实现一致）：
//   index ring : 4 字节写索引，主机初始写入"已投递的 buffer 数"
//   ring       : kHTTRingEntries × LE32，每项一个 RX buffer 的 DMA 地址
//   pool       : kHTTRxPosted × kHTTRxBufferBytes，接收缓冲区
//
// 消息顺序：VERSION_REQ(0) → RX_RING_CFG(2) → AGGR_CFG(5)
namespace Ath10k {

static constexpr UInt32 kHTTRingEntries   = 2048;
static constexpr UInt32 kHTTRxPosted      = 1023;
static constexpr UInt32 kHTTRxBufferBytes = 2048;
static constexpr UInt32 kHTTAMPDULimit    = 64;
static constexpr UInt32 kHTTAMSDULimit    = 3;

static constexpr UInt32 kHTTVersionReqBytes = 12;
static constexpr UInt32 kHTTRxRingCfgBytes  = 48;
static constexpr UInt32 kHTTAggrCfgBytes    = 11;

// H2T 消息类型（字节 8）
static constexpr UInt8 kHTTMsgVersionReq = 0;
static constexpr UInt8 kHTTMsgRxRingCfg  = 2;
static constexpr UInt8 kHTTMsgAggrCfg    = 5;

// RX descriptor v1 在 buffer 内的字段偏移（单位：字），来自 Linux rx_desc.h
static constexpr UInt16 kHTTRxDescOffsets[10] = {59, 75, 21, 31, 3, 20, 6, 10, 1, 2};

inline bool dmaSpan32(UInt64 address, UInt64 length)
{
    return address && length && address <= 0xffffffffULL &&
           length - 1 <= 0xffffffffULL - address;
}

// HTT 消息的 HTC 头：HTT 服务关闭了信用流，所以不带 NEED_CREDIT_UPDATE
inline void httHeader(UInt8 *f, UInt8 endpoint, UInt32 bytes, UInt8 seq)
{
    htcZero(f, bytes);
    f[0] = endpoint;
    f[1] = 0;
    f[5] = seq;
    htcWriteLE16(f + 2, static_cast<UInt16>(bytes - 8));
}

inline UInt32 buildHTTVersionReq(UInt8 *f, UInt32 capacity, UInt8 endpoint, UInt8 seq)
{
    if (!f || capacity < kHTTVersionReqBytes || !endpoint || endpoint == 0xff)
        return 0;
    httHeader(f, endpoint, kHTTVersionReqBytes, seq);
    f[8] = kHTTMsgVersionReq;
    return kHTTVersionReqBytes;
}

inline UInt32 buildHTTRxRingCfg(UInt8 *f, UInt32 capacity, UInt8 endpoint, UInt8 seq,
                                UInt64 indexAddress, UInt64 ringAddress)
{
    if (!f || capacity < kHTTRxRingCfgBytes || !endpoint || endpoint == 0xff)
        return 0;
    if (!dmaSpan32(indexAddress, 4) || !dmaSpan32(ringAddress, kHTTRingEntries * 4) ||
        (indexAddress & 3) || (ringAddress & 3))
        return 0;

    httHeader(f, endpoint, kHTTRxRingCfgBytes, seq);
    f[8] = kHTTMsgRxRingCfg;
    f[9] = 1;                                  // 单环
    htcWriteLE32(f + 12, static_cast<UInt32>(indexAddress));
    htcWriteLE32(f + 16, static_cast<UInt32>(ringAddress));
    htcWriteLE16(f + 20, static_cast<UInt16>(kHTTRingEntries));
    htcWriteLE16(f + 22, static_cast<UInt16>(kHTTRxBufferBytes));
    htcWriteLE16(f + 24, 0xffff);
    htcWriteLE16(f + 26, static_cast<UInt16>(kHTTRxPosted));
    for (UInt32 i = 0; i < 10; ++i)
        htcWriteLE16(f + 28 + 2 * i, kHTTRxDescOffsets[i]);
    return kHTTRxRingCfgBytes;
}

inline UInt32 buildHTTAggrCfg(UInt8 *f, UInt32 capacity, UInt8 endpoint, UInt8 seq)
{
    if (!f || capacity < kHTTAggrCfgBytes || !endpoint || endpoint == 0xff)
        return 0;
    httHeader(f, endpoint, kHTTAggrCfgBytes, seq);
    f[8] = kHTTMsgAggrCfg;
    f[9] = static_cast<UInt8>(kHTTAMPDULimit);
    f[10] = static_cast<UInt8>(kHTTAMSDULimit);
    return kHTTAggrCfgBytes;
}

// 解析 VERSION_CONF 回复：type=0，12 字节，major 在字节 10，minor 在字节 9
inline const char *parseHTTVersionReply(const UInt8 *f, UInt32 bytes, UInt8 endpoint,
                                        UInt32 &major, UInt32 &minor, UInt32 &type)
{
    major = minor = 0;
    type = 0xff;
    if (!f || bytes < 8 || bytes > 256)
        return "htt-reply-size";
    if (f[0] != endpoint)
        return "htt-reply-endpoint";
    if (f[1] & ~(kHTCTrailerPresent | kHTCMoreBlock))
        return "htt-reply-flags";
    const UInt32 payload = htcReadLE16(f + 2);
    const UInt32 declared = kHTCHeaderBytes + payload;
    if (declared > bytes || (bytes != declared && bytes != ((declared + 3u) & ~3u)))
        return "htt-reply-length";
    const UInt32 trailer = (f[1] & kHTCTrailerPresent) ? f[4] : 0;
    if (trailer > payload)
        return "htt-reply-trailer";
    const UInt32 end = declared - trailer;
    if (end < 9)
        return "htt-reply-empty";
    type = f[8];
    if (type != kHTTMsgVersionReq)
        return "htt-reply-not-version";
    if (end != 12)
        return "htt-version-length";
    major = f[10];
    minor = f[9];
    if ((major != 2 && major != 3) || minor > 16)
        return "htt-version-unsupported";
    return nullptr;
}

} // namespace Ath10k