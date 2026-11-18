#pragma once

#include <IOKit/IOTypes.h>

#include "Ath10kHTC.hpp"

// 精简版 WMI TLV ABI（对齐 Linux v6.12 drivers/net/wireless/ath/ath10k/wmi-tlv.h）。
//
// Ath10kMac 的 Ath10kWMI.hpp 依赖 Ath10kConnection.hpp（整个自研 802.11 状态机），
// 本驱动按 net80211 架构实现，不引入那份状态机，因此只抽取 bootstrap / INIT /
// vdev create 三个阶段真正需要的帧定义与构造、解析工具。
//
// 关键 ABI 事实（来自 Linux 枚举，已与 M1-b1 真机报文核对）：
//   - SERVICE_AVAILABLE(0x3) 紧接在 SERVICE_READY(0x1) 之前发出；
//   - SERVICE_READY 携带固件 ABI、RF 链数、host memory 请求与服务位图，
//     是 WMI INIT 的前置条件；
//   - INIT 之后固件用 READY(0x2) 事件回应，里面才有 target MAC。
namespace Ath10k {

static constexpr UInt32 kWMIRxBytes = 2048;
static constexpr UInt32 kWMICommandBufferBytes = 2048;
static constexpr UInt32 kWMIServiceReadyTimeoutMs = 5000;
static constexpr UInt32 kWMIServiceReadyEventMax = 24;
static constexpr UInt32 kWMIInitTimeoutMs = 5000;
// READY 已到但信用未归还时，最多再等这么久，避免拖慢 bring-up
static constexpr UInt32 kWMIInitCreditWaitMs = 1500;
static constexpr UInt32 kWMIVdevTimeoutMs = 2000;
static constexpr UInt32 kWMIEventPollMs = 1;

// ---- 事件 ID（Linux enum wmi_tlv_event_id）----
static constexpr UInt32 kWMIServiceReadyEventId     = 0x1;
static constexpr UInt32 kWMIReadyEventId            = 0x2;
static constexpr UInt32 kWMIServiceAvailableEventId = 0x3;

// ---- TLV tag（Linux enum wmi_tlv_tag）----
static constexpr UInt32 kWmiTagArrayUint32       = 16;
static constexpr UInt32 kWmiTagArrayStruct       = 18;
static constexpr UInt32 kWmiTagServiceReadyEvent = 32;
static constexpr UInt32 kWmiTagHalRegCapabilities = 33;
static constexpr UInt32 kWmiTagHostMemoryRequest = 34;
static constexpr UInt32 kWmiTagReadyEvent        = 35;
// SERVICE_AVAILABLE 的固定 tag（本机固件实测 0x022f，20 字节 TLV）
static constexpr UInt32 kWmiTagServiceAvailable  = 0x22f;

// 命令 ID / 命令内 TLV tag
static constexpr UInt32 kWMIInitCmdId        = 0x00000001;
static constexpr UInt32 kWmiTagInitCmd       = 74;
static constexpr UInt32 kWmiTagResourceConfig = 75;
static constexpr UInt32 kWmiTagHostMemoryChunk = 76;
static constexpr UInt32 kWMIVdevCreateCmdId  = 0x5001; // WMI_TLV_CMD(GRP_VDEV=5)
static constexpr UInt32 kWmiTagVdevCreateCmd = 86;

// 结构最小长度
static constexpr UInt32 kWmiServiceReadyMinBytes = 104; // struct wmi_tlv_svc_rdy_ev
static constexpr UInt32 kWmiRegCapabilitiesMinBytes = 9 * 4;
static constexpr UInt32 kWmiReadyEventMinBytes = 36;    // abi 24 + mac 8 + status 4
static constexpr UInt32 kWMIInitFrameBytes = 228;
static constexpr UInt32 kWMIVdevCreateFrameBytes = 36;

// WMI INIT 的 host ABI（与 Linux ath10k 一致；host 侧固定值，不取自固件）
static constexpr UInt32 kWMIHostABI[6] = {0x01000000, 53, 0x5f414351, 0x4c4d, 0, 0};

// host memory 请求上限（与 Ath10kMac / Linux 一致）
static constexpr UInt32 kWMIHostMemoryMax        = 16;
static constexpr UInt32 kWMIHostMemoryPerRequest = 2 * 1024 * 1024;
static constexpr UInt32 kWMIHostMemoryTotal      = 8 * 1024 * 1024;

inline UInt32 wmiReadLE32(const UInt8 *p)
{
    return static_cast<UInt32>(p[0]) |
           (static_cast<UInt32>(p[1]) << 8) |
           (static_cast<UInt32>(p[2]) << 16) |
           (static_cast<UInt32>(p[3]) << 24);
}

// TLV 头：先长度后 tag
inline void wmiConnTLV(UInt8 *p, UInt16 tag, UInt16 bytes)
{
    htcWriteLE16(p, bytes);
    htcWriteLE16(p + 2, tag);
}

inline bool wmiABICompatible(const UInt32 *abi)
{
    // Linux 刻意不要求固件 ABI 字 1 等于 53
    return abi[0] == kWMIHostABI[0] && abi[2] == kWMIHostABI[2] &&
           abi[3] == kWMIHostABI[3] && abi[4] == 0 && abi[5] == 0;
}

// ---------------------------------------------------------------------------
// HTC 信用账本（WMI 端点）
//
// WMI 命令走 CE3，每帧按 creditSize 折算信用数；信用由固件在 CE2 事件的
// HTC trailer（记录 id=1）里归还。没信用就不许再发命令。
// ---------------------------------------------------------------------------
struct WMICreditState {
    UInt32 total = 0;
    UInt32 available = 0;
    UInt32 outstanding = 0;
    UInt32 size = 0;
    UInt8  endpoint = 0xff;
    bool   initialized = false;

    bool initialize(UInt32 count, UInt32 bytes, UInt8 ep)
    {
        if (!count || !bytes || ep == 0 || ep == 0xff)
            return false;
        total = available = count;
        outstanding = 0;
        size = bytes;
        endpoint = ep;
        initialized = true;
        return true;
    }

    UInt32 cost(UInt32 frameBytes) const
    {
        if (!size || !frameBytes)
            return 0;
        return frameBytes / size + ((frameBytes % size) != 0 ? 1u : 0u);
    }

    bool reserve(UInt32 frameBytes)
    {
        const UInt32 units = cost(frameBytes);
        if (!units || units > available)
            return false;
        available -= units;
        outstanding += units;
        return true;
    }

    bool giveBack(UInt8 ep, UInt32 count)
    {
        if (!initialized || !size || ep != endpoint || count > outstanding)
            return false;
        available += count;
        outstanding -= count;
        return true;
    }
};

// ---------------------------------------------------------------------------
// host memory 请求 / 已分配块
// ---------------------------------------------------------------------------
struct WMIHostMemoryRequest {
    UInt32 id = 0;
    UInt32 unitBytes = 0;
    UInt32 flags = 0;
    UInt32 units = 0;
};

struct WMIHostMemoryChunk {
    UInt32 id = 0;
    UInt32 address = 0;
    UInt32 bytes = 0;
};

// 单个请求需要的主机内存字节数。
// 与 Ath10kMac hostMemoryRequestBytes 等价：flags 低位决定实际块数。
inline UInt32 hostMemoryRequestBytes(const WMIHostMemoryRequest &r)
{
    if (!r.unitBytes || r.unitBytes > kWMIHostMemoryPerRequest || (r.flags & ~7u))
        return 0;
    // flags bit1/2：33 个 peer + 4 个 vdev（含 self）；bit0：5 个 vdev
    const UInt32 units = (r.flags & 6) ? 34u : ((r.flags & 1) ? 5u : r.units);
    const UInt32 aligned = (r.unitBytes + 3) & ~3u;
    if (!units || units > kWMIHostMemoryPerRequest / aligned)
        return 0;
    return units * aligned;
}

// ---------------------------------------------------------------------------
// 一条 WMI 事件帧的解析结果
// ---------------------------------------------------------------------------
struct WMIEventInfo {
    UInt32 id = 0;
    UInt32 bytes = 0;
    UInt32 prefixBytes = 0;
    UInt8  prefix[48] = {};
    bool   informational = false;
    bool   trailerOnly = false;   // 只有 HTC trailer（信用回报），没有 WMI 事件头
    bool   creditsReturned = false;

    // ---- SERVICE_READY(1) ----
    bool   serviceReady = false;
    UInt32 fwBuild = 0;
    UInt32 abi[6] = {};
    bool   abiPresent = false;
    UInt32 rfChains = 0;
    UInt32 htCapability = 0;
    UInt32 vhtCapability = 0;
    UInt32 vhtMcs = 0;
    UInt32 hwMinPower = 0;
    UInt32 hwMaxPower = 0;
    UInt32 memoryRequests = 0;
    bool   memoryArrayPresent = false;
    bool   memoryArrayEmpty = false;
    UInt32 hostMemoryCount = 0;
    UInt32 hostMemoryBytes = 0;
    WMIHostMemoryRequest hostMemory[kWMIHostMemoryMax] = {};
    bool   serviceBitmapPresent = false;
    UInt32 serviceBitmap[32] = {};
    bool   regCapsPresent = false;
    UInt32 regCapsBytes = 0;
    UInt32 regCaps[9] = {};

    // ---- SERVICE_AVAILABLE(3) ----
    bool   serviceAvailable = false;
    UInt32 availableBits = 0;
    UInt32 availableBitmap[4] = {};

    // ---- READY(2) ----
    bool   ready = false;
    UInt32 readyStatus = 0xffffffffu;
    UInt8  mac[6] = {};
    bool   macValid = false;

    void reset()
    {
        *this = WMIEventInfo();
    }
};

// ---------------------------------------------------------------------------
// HTC trailer 信用回报
//
// 固件在它发出的 HTC 帧尾部附加 trailer，记录 id=1 的信用归还条目，
// 每条 4 字节 {endpoint, credits, 0, 0}。这类回报既可能挂在 WMI 事件后面，
// 也可能单独成包（CE1 控制端点或 CE2 无事件头的空帧）。
// 只有整段 trailer 校验通过后才入账；credits 未初始化时只验证结构。
// ---------------------------------------------------------------------------
inline const char *applyHTCCreditTrailer(const UInt8 *f, UInt32 bytes,
                                         WMICreditState &credits, bool &applied)
{
    applied = false;
    if (!f || bytes < kHTCHeaderBytes)
        return "htc-frame-size";
    if (!(f[1] & kHTCTrailerPresent))
        return nullptr;

    const UInt32 payload = htcReadLE16(f + 2);
    const UInt32 declared = kHTCHeaderBytes + payload;
    if (declared > bytes)
        return "htc-frame-length";
    const UInt32 trailer = f[4];
    if (!trailer || trailer < 4 || trailer > payload)
        return "htc-trailer-length";

    WMICreditState next = credits;
    for (UInt32 p = declared - trailer; p < declared; ) {
        if (declared - p < 4)
            return "htc-trailer-header";
        const UInt32 id = f[p];
        const UInt32 len = f[p + 1];
        p += 4;
        if (len > declared - p)
            return "htc-trailer-record";
        if (id == 1) {
            if (!len || (len & 3u))
                return "htc-credit-record-size";
            if (next.initialized) {
                for (UInt32 i = 0; i < len; i += 4)
                    if (!next.giveBack(f[p + i], f[p + i + 1]))
                        return "htc-credit-report-invalid";
                applied = true;
            }
        }
        p += len;
    }
    credits = next;
    return nullptr;
}

// ---------------------------------------------------------------------------
// 事件解析
//
// 返回值：nullptr 表示帧结构与本文关心的事件内容都合法（结果写入 out）；
//         非空字符串为失败原因（可直接作为 ioreg 属性发布）。
// 信用账本只在整帧校验通过后一次性提交。
// ---------------------------------------------------------------------------
inline const char *parseWMIEvent(const UInt8 *f, UInt32 bytes, UInt8 endpoint,
                                 WMICreditState &credits, WMIEventInfo &out)
{
    out.reset();
    out.bytes = bytes;
    if (!f || bytes < kHTCHeaderBytes || bytes > kWMIRxBytes)
        return "wmi-frame-size";
    if (f[0] != endpoint)
        return "wmi-endpoint-mismatch";
    if (f[1] & ~(kHTCTrailerPresent | kHTCMoreBlock))
        return "wmi-rx-flags";

    const UInt32 payload = htcReadLE16(f + 2);
    const UInt32 declared = kHTCHeaderBytes + payload;
    if (declared > bytes || (bytes != declared && bytes != ((declared + 3u) & ~3u)))
        return "wmi-frame-length";

    const UInt32 trailer = (f[1] & kHTCTrailerPresent) ? f[4] : 0;
    if (trailer > payload || ((f[1] & kHTCTrailerPresent) && trailer < 4))
        return "wmi-trailer-length";
    const UInt32 end = declared - trailer;
    if (end < kHTCHeaderBytes)
        return "wmi-frame-length";

    out.prefixBytes = bytes < sizeof(out.prefix) ? bytes : static_cast<UInt32>(sizeof(out.prefix));
    for (UInt32 i = 0; i < out.prefixBytes; ++i)
        out.prefix[i] = f[i];

    // ---- 纯 trailer 帧：只有信用回报，没有 WMI 事件头 ----
    if (end < 12) {
        if (!trailer)
            return "wmi-empty-frame";
        out.trailerOnly = true;
        bool applied = false;
        const char *error = applyHTCCreditTrailer(f, bytes, credits, applied);
        if (error)
            return error;
        out.creditsReturned = applied;
        return nullptr;
    }

    out.id = wmiReadLE32(f + 8) & 0x00ffffffu;

    const bool available = out.id == kWMIServiceAvailableEventId;
    const bool serviceReady = out.id == kWMIServiceReadyEventId;
    const bool ready = out.id == kWMIReadyEventId;
    if (!available && !serviceReady && !ready)
        out.informational = true; // 诊断/温度等事件：结构合法即可忽略

    // ---- TLV 扫描 ----
    for (UInt32 p = 12; p < end; ) {
        if (end - p < 4)
            return "wmi-truncated-tlv-header";
        const UInt32 len = htcReadLE16(f + p);
        const UInt32 tag = htcReadLE16(f + p + 2);
        p += 4;
        if (len > end - p)
            return "wmi-truncated-tlv";

        if (available) {
            if (tag == kWmiTagServiceAvailable) {
                if (out.serviceAvailable)
                    return "wmi-service-available-duplicate";
                if (len < 4)
                    return "wmi-service-available-size";
                const UInt32 bits = wmiReadLE32(f + p);
                if (!bits || bits > 128 || len < 4 + ((bits + 31) / 32) * 4)
                    return "wmi-service-available-bitmap";
                out.availableBits = bits;
                const UInt32 words = (bits + 31) / 32;
                for (UInt32 i = 0; i < words && i < 4; ++i)
                    out.availableBitmap[i] = wmiReadLE32(f + p + 4 + i * 4);
                out.serviceAvailable = true;
            }
        } else if (serviceReady) {
            if (tag == kWmiTagServiceReadyEvent) {
                if (out.serviceReady)
                    return "wmi-service-ready-duplicate";
                if (len < kWmiServiceReadyMinBytes)
                    return "wmi-service-ready-size";
                out.fwBuild = wmiReadLE32(f + p);
                for (UInt32 i = 0; i < 6; ++i)
                    out.abi[i] = wmiReadLE32(f + p + 4 + 4 * i);
                out.abiPresent = true;
                out.rfChains = wmiReadLE32(f + p + 36);
                out.htCapability = wmiReadLE32(f + p + 40);
                out.vhtCapability = wmiReadLE32(f + p + 44);
                out.vhtMcs = wmiReadLE32(f + p + 48);
                out.hwMinPower = wmiReadLE32(f + p + 52);
                out.hwMaxPower = wmiReadLE32(f + p + 56);
                out.memoryRequests = wmiReadLE32(f + p + 72);
                out.serviceReady = true;
            } else if (tag == kWmiTagArrayUint32) {
                if (out.serviceBitmapPresent)
                    return "wmi-service-bitmap-duplicate";
                if (len != 128)
                    return "wmi-service-bitmap-size";
                for (UInt32 i = 0; i < 32; ++i)
                    out.serviceBitmap[i] = wmiReadLE32(f + p + 4 * i);
                out.serviceBitmapPresent = true;
            } else if (tag == kWmiTagArrayStruct) {
                if (out.memoryArrayPresent)
                    return "wmi-memory-array-duplicate";
                out.memoryArrayPresent = true;
                out.memoryArrayEmpty = (len == 0);
                for (UInt32 m = 0; m < len; ) {
                    if (len - m < 20 ||
                        htcReadLE16(f + p + m) != 16 ||
                        htcReadLE16(f + p + m + 2) != kWmiTagHostMemoryRequest ||
                        out.hostMemoryCount >= kWMIHostMemoryMax)
                        return "wmi-memory-request-layout";
                    const UInt8 *q = f + p + m + 4;
                    WMIHostMemoryRequest &r = out.hostMemory[out.hostMemoryCount];
                    r.id = wmiReadLE32(q);
                    r.unitBytes = wmiReadLE32(q + 4);
                    r.flags = wmiReadLE32(q + 8);
                    r.units = wmiReadLE32(q + 12);
                    for (UInt32 i = 0; i < out.hostMemoryCount; ++i)
                        if (out.hostMemory[i].id == r.id)
                            return "wmi-memory-request-duplicate";
                    const UInt32 requested = hostMemoryRequestBytes(r);
                    if (!requested || requested > kWMIHostMemoryTotal - out.hostMemoryBytes)
                        return "wmi-memory-request-size";
                    out.hostMemoryBytes += requested;
                    ++out.hostMemoryCount;
                    m += 20;
                }
            } else if (tag == kWmiTagHalRegCapabilities) {
                if (out.regCapsPresent)
                    return "wmi-reg-capabilities-duplicate";
                if (len < kWmiRegCapabilitiesMinBytes)
                    return "wmi-reg-capabilities-size";
                for (UInt32 i = 0; i < 9; ++i)
                    out.regCaps[i] = wmiReadLE32(f + p + 4 * i);
                out.regCapsBytes = len;
                out.regCapsPresent = true;
            }
        } else if (ready) {
            if (tag == kWmiTagReadyEvent) {
                if (out.ready)
                    return "wmi-ready-duplicate";
                if (len < kWmiReadyEventMinBytes)
                    return "wmi-ready-size";
                for (UInt32 i = 0; i < 6; ++i)
                    out.abi[i] = wmiReadLE32(f + p + 4 * i);
                out.abiPresent = true;
                UInt8 nonzero = 0;
                for (UInt32 i = 0; i < 6; ++i) {
                    out.mac[i] = f[p + 24 + i];
                    nonzero |= out.mac[i];
                }
                out.macValid = nonzero != 0 && (out.mac[0] & 1) == 0;
                out.readyStatus = wmiReadLE32(f + p + 32);
                out.ready = true;
            }
        }
        p += len;
    }

    // ---- 事件完整性 ----
    if (available && !out.serviceAvailable)
        return "wmi-service-available-tlv-missing";
    if (serviceReady) {
        if (!out.serviceReady)
            return "wmi-service-ready-tlv-missing";
        if (!out.abiPresent)
            return "wmi-service-ready-abi-missing";
        if (out.memoryRequests != out.hostMemoryCount ||
            out.memoryRequests > kWMIHostMemoryMax)
            return "wmi-memory-request-count";
        if (out.rfChains > 3)
            return "wmi-service-ready-chains-invalid";
    }
    if (ready && !out.ready)
        return "wmi-ready-tlv-missing";

    // ---- HTC trailer：信用归还 ----
    bool applied = false;
    const char *trailerError = applyHTCCreditTrailer(f, bytes, credits, applied);
    if (trailerError)
        return trailerError;
    out.creditsReturned = applied;
    return nullptr;
}

// ---------------------------------------------------------------------------
// WMI INIT 帧构造（CE3 发送）
//
// 布局与 Ath10kMac buildWMIInit 一致（已按 Linux wmi-tlv.c 核对）：
//   0      HTC endpoint
//   1      NEED_CREDIT_UPDATE
//   2-3    HTC payload 长度（总长 - 8）
//   5      HTC 序号
//   8-11   WMI_TLV_INIT_CMDID(0x1)
//   12-15  TLV: len=28, tag=74（init cmd）
//   16-39  host ABI 6 字
//   40-43  host memory 块数
//   44-47  TLV: len=176, tag=75（resource config，44 字）
//   48-223 resource config 默认值
//   224-227 TLV: len=20*n, tag=18（host memory chunk 数组）
//   228+   每块 20 字节：TLV(16,76) + id + 地址 + 字节数
// ---------------------------------------------------------------------------
inline const char *buildWMIInit(UInt8 *f, UInt32 capacity, const WMIEventInfo &boot,
                                UInt8 endpoint, UInt32 hardwareChains,
                                const WMIHostMemoryChunk *chunks, UInt32 count,
                                UInt8 sequence)
{
    if (!hardwareChains || hardwareChains > 3)
        return "wmi-hardware-chains-invalid";
    if (!f || capacity < kWMIInitFrameBytes || !endpoint || endpoint == 0xff)
        return "wmi-init-prerequisite";
    if (!boot.serviceReady)
        return "wmi-init-service-ready-missing";
    if (!wmiABICompatible(boot.abi))
        return "wmi-init-abi-mismatch";
    if (count > kWMIHostMemoryMax || count != boot.memoryRequests ||
        count != boot.hostMemoryCount || (count && !chunks))
        return "wmi-init-memory-requests-unsupported";

    const UInt32 frameBytes = kWMIInitFrameBytes + 20 * count;
    if (capacity < frameBytes)
        return "wmi-init-capacity";

    for (UInt32 i = 0; i < count; ++i) {
        if (chunks[i].id != boot.hostMemory[i].id || !chunks[i].address ||
            (chunks[i].address & 3u) ||
            chunks[i].bytes != hostMemoryRequestBytes(boot.hostMemory[i]) ||
            static_cast<UInt64>(chunks[i].address) + chunks[i].bytes > 0x100000000ULL)
            return "wmi-init-memory-chunk-invalid";
        for (UInt32 j = 0; j < i; ++j)
            if (static_cast<UInt64>(chunks[i].address) <
                    static_cast<UInt64>(chunks[j].address) + chunks[j].bytes &&
                static_cast<UInt64>(chunks[j].address) <
                    static_cast<UInt64>(chunks[i].address) + chunks[i].bytes)
                return "wmi-init-memory-chunk-overlap";
    }

    htcZero(f, frameBytes);
    f[0] = endpoint;
    f[1] = 1;   // NEED_CREDIT_UPDATE
    f[5] = sequence;
    htcWriteLE16(f + 2, static_cast<UInt16>(frameBytes - 8));
    htcWriteLE32(f + 8, kWMIInitCmdId);
    wmiConnTLV(f + 12, static_cast<UInt16>(kWmiTagInitCmd), 28);
    for (UInt32 i = 0; i < 6; ++i)
        htcWriteLE32(f + 16 + 4 * i, kWMIHostABI[i]);
    htcWriteLE32(f + 40, count);
    wmiConnTLV(f + 44, static_cast<UInt16>(kWmiTagResourceConfig), 176);

    // Linux TLV 的 QCA6174/QCA9377 默认资源表；这里不会真的建 vdev/peer
    UInt32 cfg[44] = {4, 33, 0, 0, 2, 66, 16, 7, 7, 100, 100, 100, 40, 1, 4, 4, 4, 8,
                      0, 0, 0, 1024, 32, 0, 0, 0, 0, 2, 1056, 2, 1, 32, 2, 5, 22, 6, 0,
                      1, 1, 0, 0, 0, 0, 0};
    // 固件上报 RF 链数；Linux 用它推导 TX/RX chain mask = (1 << chains) - 1。
    // 本机是 QCA9377（1x1），hardwareChains 传入 1。rfChains 为 0 时按 1 处理。
    UInt32 chains = boot.rfChains ? boot.rfChains : 1u;
    if (chains > hardwareChains)
        chains = hardwareChains;
    cfg[7] = cfg[8] = (1u << chains) - 1u;
    // Linux 在 WMI_SERVICE_4ADDR 置位时才启用 4 地址（service 65）
    if (boot.serviceBitmap[65 / 32] & (1u << (65 % 32)))
        cfg[2] = cfg[3] = 4;
    for (UInt32 i = 0; i < 44; ++i)
        htcWriteLE32(f + 48 + 4 * i, cfg[i]);

    wmiConnTLV(f + 224, kWmiTagArrayStruct, static_cast<UInt16>(20 * count));
    for (UInt32 i = 0; i < count; ++i) {
        UInt8 *q = f + 228 + 20 * i;
        wmiConnTLV(q, static_cast<UInt16>(kWmiTagHostMemoryChunk), 16);
        htcWriteLE32(q + 4, chunks[i].id);
        htcWriteLE32(q + 8, chunks[i].address);
        htcWriteLE32(q + 12, chunks[i].bytes);
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// vdev create（STA 模式）
// ---------------------------------------------------------------------------
inline const char *buildWMIVdevCreate(UInt8 *f, UInt32 capacity, UInt8 endpoint,
                                      const UInt8 *mac, UInt8 sequence)
{
    if (!f || capacity < kWMIVdevCreateFrameBytes || !endpoint || endpoint == 0xff)
        return "vdev-create-prerequisite";
    if (!mac || (mac[0] & 1))
        return "vdev-create-mac-invalid";
    UInt8 nonzero = 0;
    for (UInt32 i = 0; i < 6; ++i)
        nonzero |= mac[i];
    if (!nonzero)
        return "vdev-create-mac-invalid";

    htcZero(f, kWMIVdevCreateFrameBytes);
    f[0] = endpoint;
    f[1] = 1;   // NEED_CREDIT_UPDATE
    f[5] = sequence;
    htcWriteLE16(f + 2, 28);
    htcWriteLE32(f + 8, kWMIVdevCreateCmdId);
    wmiConnTLV(f + 12, kWmiTagVdevCreateCmd, 20);
    // 偏移 16：vdev_id=0；20：type=STA(2)；24：subtype=NONE(0)；28：MAC（8 字节 union）
    htcWriteLE32(f + 16, 0);
    htcWriteLE32(f + 20, 2);
    htcWriteLE32(f + 24, 0);
    for (UInt32 i = 0; i < 6; ++i)
        f[28 + i] = mac[i];
    return nullptr;
}

} // namespace Ath10k