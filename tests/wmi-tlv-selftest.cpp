// WMI TLV 帧构造/解析的离线自测（不接触硬件、不需要真机）
//
// 目的：M1-b2 的帧格式（WMI INIT / VDEV_CREATE）与事件解析
//       （SERVICE_READY / READY / SERVICE_AVAILABLE）一旦被改动，
//       先在主机上跑这个自测，避免为了验证一个字段偏移反复重启真机。
//
// 编译运行：
//   clang++ -std=c++14 -O0 -I itlwm/itlwm/hal_ath10k/hw \
//           tests/wmi-tlv-selftest.cpp -framework IOKit -o /tmp/wmi-self //     && /tmp/wmi-self
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "Ath10kWmiTlv.hpp"
#include "Ath10kWmiScan.hpp"
#include "Ath10kWmiConnect.hpp"
#include "Ath10kHtt.hpp"

using namespace Ath10k;

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); ++failures; } } while (0)
#define CHECKSTR(a, b) do { if (std::strcmp((a),(b)) != 0) { printf("FAIL %s:%d got=%s want=%s\n", __FILE__, __LINE__, (a), (b)); ++failures; } } while (0)

static void put16(UInt8 *p, UInt16 v) { p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; }
static void put32(UInt8 *p, UInt32 v) { p[0]=v&0xff; p[1]=(v>>8)&0xff; p[2]=(v>>16)&0xff; p[3]=(v>>24)&0xff; }

int main()
{
    // ---------- 1. buildWMIInit ----------
    WMIEventInfo boot;
    boot.serviceReady = true;
    for (int i = 0; i < 6; ++i) boot.abi[i] = kWMIHostABI[i];
    boot.rfChains = 1;
    boot.memoryRequests = 2;
    boot.hostMemoryCount = 2;
    boot.hostMemory[0] = WMIHostMemoryRequest{7, 4096, 2, 0};
    boot.hostMemory[1] = WMIHostMemoryRequest{9, 2048, 1, 0};
    boot.serviceBitmap[65 / 32] |= (1u << (65 % 32));
    CHECK(hostMemoryRequestBytes(boot.hostMemory[0]) == 34u * 4096u);
    CHECK(hostMemoryRequestBytes(boot.hostMemory[1]) == 5u * 2048u);

    WMIHostMemoryChunk chunks[2];
    chunks[0] = WMIHostMemoryChunk{7, 0x10000000u, hostMemoryRequestBytes(boot.hostMemory[0])};
    chunks[1] = WMIHostMemoryChunk{9, 0x10220000u, hostMemoryRequestBytes(boot.hostMemory[1])};

    UInt8 frame[2048];
    const char *err = buildWMIInit(frame, sizeof(frame), boot, 2, 1, chunks, 2, 0);
    if (err) { printf("FAIL buildWMIInit: %s\n", err); ++failures; }
    CHECK(frame[0] == 2);
    CHECK(frame[1] == 1);
    CHECK(htcReadLE16(frame + 2) == (UInt16)(228 + 40 - 8));
    CHECK(frame[5] == 0);
    CHECK(wmiReadLE32(frame + 8) == 1);
    CHECK(htcReadLE16(frame + 12) == 28 && htcReadLE16(frame + 14) == 74);
    CHECK(wmiReadLE32(frame + 16) == 0x01000000u);
    CHECK(wmiReadLE32(frame + 20) == 53);
    CHECK(wmiReadLE32(frame + 40) == 2);
    CHECK(htcReadLE16(frame + 44) == 176 && htcReadLE16(frame + 46) == 75);
    CHECK(wmiReadLE32(frame + 48 + 4 * 7) == 1);   // tx chain mask
    CHECK(wmiReadLE32(frame + 48 + 4 * 8) == 1);   // rx chain mask
    CHECK(wmiReadLE32(frame + 48 + 4 * 2) == 4);   // 4addr -> offload peers
    CHECK(wmiReadLE32(frame + 48 + 4 * 3) == 4);
    CHECK(htcReadLE16(frame + 224) == 40 && htcReadLE16(frame + 226) == 18);
    CHECK(htcReadLE16(frame + 228) == 16 && htcReadLE16(frame + 230) == 76);
    CHECK(wmiReadLE32(frame + 232) == 7);
    CHECK(wmiReadLE32(frame + 236) == 0x10000000u);
    CHECK(wmiReadLE32(frame + 240) == 34u * 4096u);

    // rfChains 缺失时按 1 处理
    WMIEventInfo boot0 = boot;
    boot0.rfChains = 0;
    err = buildWMIInit(frame, sizeof(frame), boot0, 2, 1, chunks, 2, 0);
    CHECK(err == nullptr);
    CHECK(wmiReadLE32(frame + 48 + 4 * 7) == 1);

    // ABI 不匹配必须拒绝
    WMIEventInfo badA = boot;
    badA.abi[2] = 0xdeadbeef;
    CHECKSTR(buildWMIInit(frame, sizeof(frame), badA, 2, 1, chunks, 2, 0), "wmi-init-abi-mismatch");

    // chunk 尺寸/重叠必须拒绝
    WMIHostMemoryChunk badChunks[2] = {chunks[0],
        WMIHostMemoryChunk{9, 0x10000000u, hostMemoryRequestBytes(boot.hostMemory[1])}};
    CHECKSTR(buildWMIInit(frame, sizeof(frame), boot, 2, 1, badChunks, 2, 0), "wmi-init-memory-chunk-overlap");

    // ---------- 2. buildWMIVdevCreate ----------
    UInt8 mac[6] = {0x34, 0x02, 0x86, 0x12, 0x34, 0x56};
    err = buildWMIVdevCreate(frame, sizeof(frame), 2, mac, 1);
    CHECK(err == nullptr);
    CHECK(frame[0] == 2 && frame[1] == 1 && frame[5] == 1);
    CHECK(htcReadLE16(frame + 2) == 28);
    CHECK(wmiReadLE32(frame + 8) == 0x5001);
    CHECK(htcReadLE16(frame + 12) == 20 && htcReadLE16(frame + 14) == 86);
    CHECK(wmiReadLE32(frame + 16) == 0);
    CHECK(wmiReadLE32(frame + 20) == 2);
    CHECK(wmiReadLE32(frame + 24) == 0);
    CHECK(std::memcmp(frame + 28, mac, 6) == 0);
    UInt8 mcast[6] = {0x01, 0, 0, 0, 0, 0};
    CHECKSTR(buildWMIVdevCreate(frame, sizeof(frame), 2, mcast, 1), "vdev-create-mac-invalid");

    // ---------- 3. parseWMIEvent：SERVICE_READY(1) ----------
    {
        UInt8 f[1024];
        std::memset(f, 0, sizeof(f));
        UInt32 p = 12;
        // tag 32：service ready struct（104 字节）
        put16(f + p, 104); put16(f + p + 2, 32); p += 4;
        put32(f + p + 0, 0x00300001u);            // fw build
        for (int i = 0; i < 6; ++i) put32(f + p + 4 + 4 * i, kWMIHostABI[i]);
        put32(f + p + 36, 1);                     // rf chains
        put32(f + p + 72, 1);                     // num_mem_reqs
        p += 104;
        // tag 16：服务位图 128 字节
        put16(f + p, 128); put16(f + p + 2, 16); p += 4;
        put32(f + p + 4 * 2, 0x00000002u);        // bit 65
        p += 128;
        // tag 18：host memory 请求（一条 20 字节记录）
        put16(f + p, 20); put16(f + p + 2, 18); p += 4;
        put16(f + p, 16); put16(f + p + 2, 34);
        put32(f + p + 4, 7); put32(f + p + 8, 4096);
        put32(f + p + 12, 2); put32(f + p + 16, 0);
        p += 20;
        // tag 33：reg caps 36 字节
        put16(f + p, 36); put16(f + p + 2, 33); p += 4;
        for (int i = 0; i < 9; ++i) put32(f + p + 4 * i, 0x100 + i);
        p += 36;
        const UInt32 end = p;

        f[0] = 2; f[1] = 0; put16(f + 2, end - 8);
        put32(f + 8, 1);

        WMICreditState credits;
        CHECK(credits.initialize(2, 256, 2));
        WMIEventInfo info;
        const char *e = parseWMIEvent(f, end, 2, credits, info);
        if (e) { printf("FAIL parse service-ready: %s\n", e); ++failures; }
        CHECK(info.serviceReady);
        CHECK(info.fwBuild == 0x00300001u);
        CHECK(info.rfChains == 1);
        CHECK(info.memoryRequests == 1);
        CHECK(info.hostMemoryCount == 1);
        CHECK(info.hostMemory[0].id == 7);
        CHECK(info.hostMemoryBytes == 34u * 4096u);
        CHECK(info.serviceBitmapPresent);
        CHECK((info.serviceBitmap[65 / 32] & (1u << (65 % 32))) != 0);
        CHECK(info.regCapsPresent && info.regCaps[0] == 0x100);
        CHECK(credits.available == 2 && credits.outstanding == 0);
    }

    // ---------- 4. parseWMIEvent：READY(2) + 信用归还 ----------
    {
        UInt8 f[256];
        std::memset(f, 0, sizeof(f));
        UInt32 p = 12;
        put16(f + p, 36); put16(f + p + 2, 35); p += 4;
        for (int i = 0; i < 6; ++i) put32(f + p + 4 * i, kWMIHostABI[i]);
        const UInt8 mac2[6] = {0x34, 0x02, 0x86, 0xaa, 0xbb, 0xcc};
        std::memcpy(f + p + 24, mac2, 6);
        put32(f + p + 32, 0);
        p += 36;
        const UInt32 end = p;
        const UInt32 trailer = 8;
        f[0] = 2; f[1] = 2 /*trailer*/; f[4] = (UInt8)trailer;
        put16(f + 2, end - 8 + trailer);
        put32(f + 8, 2);
        // trailer: 4 字节头 + 4 字节信用记录
        put16(f + end, 1); f[end + 1] = 4; // id=1 len=4
        f[end + 4] = 2; f[end + 5] = 1;    // ep=2 count=1
        const UInt32 bytes = end + 8;

        WMICreditState credits;
        CHECK(credits.initialize(2, 256, 2));
        CHECK(credits.reserve(228));
        CHECK(credits.available == 1 && credits.outstanding == 1);
        WMIEventInfo info;
        const char *e = parseWMIEvent(f, bytes, 2, credits, info);
        if (e) { printf("FAIL parse ready: %s\n", e); ++failures; }
        CHECK(info.ready);
        CHECK(info.readyStatus == 0);
        CHECK(info.macValid);
        CHECK(std::memcmp(info.mac, mac2, 6) == 0);
        CHECK(info.creditsReturned);
        CHECK(credits.available == 2 && credits.outstanding == 0);
    }

    // ---------- 5. parseWMIEvent：SERVICE_AVAILABLE(3) ----------
    {
        UInt8 f[128];
        std::memset(f, 0, sizeof(f));
        UInt32 p = 12;
        put16(f + p, 20); put16(f + p + 2, 559); p += 4;
        put32(f + p, 128);
        put32(f + p + 4, 0x08000000u);
        p += 20;
        f[0] = 2; f[1] = 0; put16(f + 2, p - 8);
        put32(f + 8, 3);
        WMICreditState credits;   // 未初始化：只验证结构
        WMIEventInfo info;
        const char *e = parseWMIEvent(f, p, 2, credits, info);
        if (e) { printf("FAIL parse available: %s\n", e); ++failures; }
        CHECK(info.serviceAvailable);
        CHECK(info.availableBits == 128);
        CHECK(info.availableBitmap[0] == 0x08000000u);
    }

    // ---------- 7. trailer-only 帧（信用回报，无事件头）----------
    {
        UInt8 f[64];
        std::memset(f, 0, sizeof(f));
        const UInt32 end = 8;        // 只有 HTC 头
        const UInt32 trailer = 8;    // 4 字节记录头 + 4 字节信用
        f[0] = 2; f[1] = 2 /*TRAILER*/; f[4] = (UInt8)trailer;
        put16(f + 2, end - 8 + trailer);
        f[end + 0] = 1; f[end + 1] = 4; f[end + 2] = 0; f[end + 3] = 0;
        f[end + 4] = 2; f[end + 5] = 1;
        const UInt32 bytes = end + trailer;

        WMICreditState credits;
        CHECK(credits.initialize(2, 256, 2));
        CHECK(credits.reserve(228));
        CHECK(credits.available == 1 && credits.outstanding == 1);
        WMIEventInfo info;
        const char *e = parseWMIEvent(f, bytes, 2, credits, info);
        if (e) { printf("FAIL parse trailer-only: %s\n", e); ++failures; }
        CHECK(info.trailerOnly);
        CHECK(info.creditsReturned);
        CHECK(credits.available == 2 && credits.outstanding == 0);

        // 直接调用 trailer 解析：不认识的端点必须被拒绝
        WMICreditState c2;
        CHECK(c2.initialize(2, 256, 2));
        CHECK(c2.reserve(228));
        bool applied = false;
        f[end + 4] = 5;         // endpoint 5 不在账本里
        CHECKSTR(applyHTCCreditTrailer(f, bytes, c2, applied), "htc-credit-report-invalid");
        CHECK(c2.outstanding == 1);

        // 没有 trailer、也没有事件头的空帧必须报错
        f[1] = 0;
        put16(f + 2, 0);          // payload 0 -> declared 8
        CHECKSTR(parseWMIEvent(f, 8, 2, credits, info), "wmi-empty-frame");
    }

    // ---------- 6. 坏帧必须被拒绝 ----------
    {
        UInt8 f[64];
        std::memset(f, 0, sizeof(f));
        f[0] = 2; f[1] = 0; put16(f + 2, 12);
        put32(f + 8, 3);
        WMICreditState credits;
        WMIEventInfo info;
        f[0] = 3;
        CHECKSTR(parseWMIEvent(f, 20, 2, credits, info), "wmi-endpoint-mismatch");
        f[0] = 2;
        CHECKSTR(parseWMIEvent(f, 20, 2, credits, info), "wmi-service-available-tlv-missing");
        put16(f + 2, 1000);
        CHECKSTR(parseWMIEvent(f, 20, 2, credits, info), "wmi-frame-length");
        put16(f + 2, 100);
        CHECKSTR(parseWMIEvent(f, 20, 2, credits, info), "wmi-frame-length");
        put16(f + 2, 12); f[0] = 9;
        CHECKSTR(parseWMIEvent(f, 20, 2, credits, info), "wmi-endpoint-mismatch");
    }

    // ---------- 8. 扫描帧构造 ----------
    {
        UInt8 f[2048];
        // PDEV_SET_PARAM
        CHECK(buildWMIPdevSetParam(f, sizeof(f), 2, 1, 1, 5));
        CHECK(htcReadLE16(f + 2) == 20);
        CHECK(wmiReadLE32(f + 8) == 0x4003);
        CHECK(htcReadLE16(f + 12) == 12 && htcReadLE16(f + 14) == 82);
        CHECK(wmiReadLE32(f + 20) == 1 && wmiReadLE32(f + 24) == 1);
        CHECK(f[5] == 5);

        // SCAN_CHAN_LIST
        CHECK(buildWMIScanChanList(f, sizeof(f), 2, 6));
        const UInt32 listBytes = 24 + 28 * kWmiScanChannelCount;
        CHECK(htcReadLE16(f + 2) == listBytes - 8);
        CHECK(wmiReadLE32(f + 8) == 0x3003);
        CHECK(htcReadLE16(f + 12) == 4 && htcReadLE16(f + 14) == 79);
        CHECK(wmiReadLE32(f + 16) == kWmiScanChannelCount);
        CHECK(htcReadLE16(f + 20) == 28 * kWmiScanChannelCount);
        CHECK(htcReadLE16(f + 22) == 18);
        CHECK(htcReadLE16(f + 24) == 24 && htcReadLE16(f + 26) == 80);
        CHECK(wmiReadLE32(f + 28) == 2412);
        CHECK(wmiReadLE32(f + 40) == 0x01);            // 11g（非 DFS，可主动）
        CHECK(wmiReadLE32(f + 44) == ((20u << 8) | (20u << 16)));
        // 最后一个信道是 5825
        const UInt8 *last = f + 24 + 28 * (kWmiScanChannelCount - 1);
        CHECK(wmiReadLE32(last + 4) == 5825);

        // START_SCAN
        CHECK(buildWMIStartScan(f, sizeof(f), 2, 7, true, true));
        const UInt32 scanBytes = wmiStartScanFrameBytes();
        const UInt32 scanChans = wmiStartScanChannelCount();
        CHECK(scanChans < kWmiScanChannelCount);       // 已跳过 DFS
        CHECK(htcReadLE16(f + 2) == scanBytes - 8);
        CHECK(wmiReadLE32(f + 8) == 0x3001);
        CHECK(htcReadLE16(f + 12) == 100 && htcReadLE16(f + 14) == 77);
        CHECK(wmiReadLE32(f + 16) == 0xa036);          // scan id
        CHECK(wmiReadLE32(f + 20) == 0xa001);          // request id
        CHECK(wmiReadLE32(f + 16 + 4 * 6) == 200);     // fast dwell
        CHECK(wmiReadLE32(f + 16 + 4 * 14) == 0x31);   // PASSIVE|HIGH|FAST
        CHECK(wmiReadLE32(f + 80) == scanChans);
        CHECK(wmiReadLE32(f + 84) == 1);
        CHECK(htcReadLE16(f + 116) == 4 * scanChans);
        CHECK(htcReadLE16(f + 118) == 16);
        CHECK(wmiReadLE32(f + 120) == 2412);
        const UInt32 tail = 120 + 4 * scanChans;
        CHECK(htcReadLE16(f + tail + 4) == 8 && htcReadLE16(f + tail + 6) == 19);
        CHECK(f[tail + 8] == 0xff && f[tail + 13] == 0xff);
        CHECK(htcReadLE16(f + tail + 18) == 17);
    }

    // ---------- 9. 扫描事件 / MGMT_RX 解析 ----------
    {
        // SCAN event：type=2(COMPLETED) reason=0 freq=2412
        UInt8 f[128];
        std::memset(f, 0, sizeof(f));
        UInt32 p = 12;
        put16(f + p, 24); put16(f + p + 2, 36); p += 4;
        put32(f + p, 2); put32(f + p + 4, 0); put32(f + p + 8, 2412);
        put32(f + p + 12, 0xa001); put32(f + p + 16, 0xa036);
        p += 24;
        f[0] = 2; f[1] = 0; put16(f + 2, p - 8);
        put32(f + 8, 0x3001);
        WMIScanInfo info;
        const char *e = parseWMIScanEvent(f, p, 0x3001, info);
        if (e) { printf("FAIL scan event: %s\n", e); ++failures; }
        CHECK(info.scanEvent && !info.mgmtRx);
        CHECK(info.scanType == 2 && info.scanReason == 0 && info.scanFreq == 2412);

        // MGMT_RX：构造一个 36 字节头 + SSID IE + Rates IE + DS IE 的 beacon
        UInt8 frame[128];
        std::memset(frame, 0, sizeof(frame));
        frame[0] = 0x80;                                                 // beacon
        frame[1] = 0x00;
        for (int i = 0; i < 6; ++i) frame[16 + i] = (UInt8)(0x10 + i);  // BSSID
        UInt32 fp = 36;
        frame[fp++] = 0; frame[fp++] = 8;
        std::memcpy(frame + fp, "TestWLAN", 8); fp += 8;
        frame[fp++] = 1; frame[fp++] = 4;          // rates IE（net80211 必需）
        frame[fp++] = 0x82; frame[fp++] = 0x84; frame[fp++] = 0x8b; frame[fp++] = 0x96;
        frame[fp++] = 3; frame[fp++] = 1; frame[fp++] = 6;   // DS 参数

        std::memset(f, 0, sizeof(f));
        p = 12;
        put16(f + p, 40); put16(f + p + 2, 44); p += 4;
        put32(f + p, 2437); put32(f + p + 4, 55); put32(f + p + 8, 0);
        put32(f + p + 16, fp); put32(f + p + 20, 0);
        p += 40;
        const UInt32 frameOffset = p + 4;
        put16(f + p, fp); put16(f + p + 2, 17); p += 4;
        std::memcpy(f + p, frame, fp); p += fp;
        f[0] = 2; f[1] = 0; put16(f + 2, p - 8);
        put32(f + 8, 0x7001);
        e = parseWMIScanEvent(f, p, 0x7001, info);
        if (e) { printf("FAIL mgmt rx: %s\n", e); ++failures; }
        CHECK(info.mgmtRx && !info.scanEvent);
        CHECK(info.mgmtFreq == 2437 && info.mgmtSnr == 55);
        CHECK(info.bssValid);
        CHECK(info.bssSsidLen == 8 && std::memcmp(info.bssSsid, "TestWLAN", 8) == 0);
        CHECK(info.bssChannel == 6);

        // M1-c3：原始帧必须原样暴露给 net80211（路径 A 的输入）
        CHECK(info.mgmtFrame == f + frameOffset);
        CHECK(info.mgmtFrameLength == fp);
        CHECK(info.mgmtFrame[0] == 0x80);            // beacon subtype
        CHECK(std::memcmp(info.mgmtFrame + 16, frame + 16, 6) == 0);

        // 管理帧数组里的 IE 必须能被 net80211 从第 36 字节开始读到
        CHECK(info.mgmtFrame[36] == 0 && info.mgmtFrame[37] == 8);   // SSID
        CHECK(info.mgmtFrame[46] == 1 && info.mgmtFrame[47] == 4);   // Rates
    }

    // ---------- 9b. 频率 → 信道（M1-c3 的 rxi_chan 换算） ----------
    {
        CHECK(wmiFreqToChannel(2412) == 1);
        CHECK(wmiFreqToChannel(2437) == 6);
        CHECK(wmiFreqToChannel(2484) == 14);
        CHECK(wmiFreqToChannel(5180) == 36);
        CHECK(wmiFreqToChannel(5765) == 153);
        CHECK(wmiFreqToChannel(5825) == 165);
        CHECK(wmiFreqToChannel(0) == 0);
        CHECK(wmiFreqToChannel(6000) == 0);
    }

    // ---------- 9c. M1-d 连接命令与 HTT TX 描述符（字节布局对齐 Ath10kMac） ----------
    {
        WmiConnectParams c;
        c.ssidLen = 9;
        std::memcpy(c.ssid, "TestNet-5", 9);
        c.channel = 36;
        c.centerChannel = 36;
        c.beaconInterval = 100;
        c.dtimPeriod = 3;
        c.txPowerHalfDbm = 20;
        c.htEnabled = true;
        c.width = 20;
        c.nss = 1;
        c.secure = true;
        c.wmmPresent = true;
        c.rateCount = 8;
        for (int i = 0; i < 8; ++i) c.rates[i] = (UInt8)(0x82 + i * 2);
        for (int i = 0; i < 6; ++i) c.bssid[i] = (UInt8)(0x40 + i);
        for (int i = 0; i < 6; ++i) c.localMac[i] = (UInt8)(0x50 + i);
        c.aid = 7;
        c.peerCaps = 0x431;
        c.peerHtCaps = 0x0102;

        UInt8 f[512];

        // VDEV_START
        CHECK(buildWMIVdevStart(f, sizeof(f), 2, 9, c));
        CHECK(htcReadLE16(f + 2) == kWmiVdevStartBytes - 8);
        CHECK(wmiReadLE32(f + 8) == 0x5003);
        CHECK(htcReadLE16(f + 12) == 72 && htcReadLE16(f + 14) == 88);
        CHECK(wmiReadLE32(f + 16 + 8) == 100);        // beacon interval
        CHECK(wmiReadLE32(f + 16 + 12) == 3);         // dtim
        CHECK(wmiReadLE32(f + 16 + 20) == 9);         // ssid len
        CHECK(std::memcmp(f + 16 + 24, "TestNet-5", 9) == 0);
        CHECK(htcReadLE16(f + 88) == 24 && htcReadLE16(f + 90) == 80);
        CHECK(wmiReadLE32(f + 92) == 5180);           // freq
        CHECK(wmiReadLE32(f + 96) == 5180);           // center
        // phy mode：5GHz + HT + 20MHz => 4，再带 HT/VHT 标志位
        CHECK(wmiReadLE32(f + 104) == (4u | (1u << 11)));
        CHECK(htcReadLE16(f + 116) == 0 && htcReadLE16(f + 118) == 18);

        // PEER_CREATE
        CHECK(buildWMIPeerCreate(f, sizeof(f), 2, 10, c.bssid));
        CHECK(htcReadLE16(f + 2) == kWmiPeerCreateBytes - 8);
        CHECK(wmiReadLE32(f + 8) == 0x6001);
        CHECK(htcReadLE16(f + 12) == 16 && htcReadLE16(f + 14) == 97);
        CHECK(std::memcmp(f + 20, c.bssid, 6) == 0);

        // PEER_ASSOC（120 + 8 aligned + 4 HT）
        CHECK(wmiPeerAssocBytes(c) == 132);
        CHECK(buildWMIPeerAssoc(f, sizeof(f), 2, 11, c));
        CHECK(htcReadLE16(f + 2) == 132 - 8);
        CHECK(wmiReadLE32(f + 8) == 0x6005);
        CHECK(htcReadLE16(f + 12) == 76 && htcReadLE16(f + 14) == 101);
        CHECK(std::memcmp(f + 16, c.bssid, 6) == 0);
        CHECK(wmiReadLE32(f + 16 + 16) == 7);                 // aid
        CHECK((wmiReadLE32(f + 16 + 20) & 0x1000u) != 0);     // HT flag
        CHECK((wmiReadLE32(f + 16 + 20) & 1u) != 0);          // associated
        CHECK(wmiReadLE32(f + 16 + 24) == 0x431);             // peer caps
        CHECK(wmiReadLE32(f + 16 + 32) == 0x0102);            // HT caps
        CHECK(wmiReadLE32(f + 16 + 68) == 8);                 // legacy rate count
        CHECK(htcReadLE16(f + 92) == 8 && htcReadLE16(f + 94) == 17);
        CHECK(f[96] == (c.rates[0] & 0x7f));
        CHECK(htcReadLE16(f + 96 + 8) == 4 && htcReadLE16(f + 96 + 10) == 17);
        CHECK(f[96 + 8 + 4] == 0xff);                          // MCS0-7

        // PEER_AUTHORIZE
        CHECK(buildWMIPeerAuthorize(f, sizeof(f), 2, 12, c.bssid, true));
        CHECK(wmiReadLE32(f + 8) == 0x6004);
        CHECK(htcReadLE16(f + 12) == 20 && htcReadLE16(f + 14) == 100);
        CHECK(std::memcmp(f + 20, c.bssid, 6) == 0);
        CHECK(wmiReadLE32(f + 28) == 3 && wmiReadLE32(f + 32) == 1);

        // VDEV_UP
        CHECK(buildWMIVdevUp(f, sizeof(f), 2, 13, c));
        CHECK(wmiReadLE32(f + 8) == 0x5005);
        CHECK(htcReadLE16(f + 12) == 16 && htcReadLE16(f + 14) == 92);
        CHECK(wmiReadLE32(f + 20) == 7);
        CHECK(std::memcmp(f + 24, c.bssid, 6) == 0);

        // HTT TX 描述符
        UInt8 frame[64];
        std::memset(frame, 0xAA, sizeof(frame));
        const UInt32 httBytes = buildHTTPacket(f, sizeof(f), frame, 40, 0x20000000u, 0,
                                               4, 7, 33, true);
        CHECK(httBytes == 24 + 40);
        CHECK(htcReadLE16(f + 2) == httBytes - 8);
        CHECK(f[0] == 4 && f[5] == 7);
        CHECK(f[8] == 1);
        CHECK(f[9] == 0x65);                                  // mgmt flags
        CHECK(htcReadLE16(f + 10) == (0x800 | (17 << 6)));    // POSTPONED + TID 17
        CHECK(htcReadLE16(f + 12) == 40);
        CHECK(htcReadLE16(f + 14) == 33);
        CHECK(wmiReadLE32(f + 16) == 0x20000000u);
        CHECK(f[24] == 0xAA);
        // 大帧只内联前 50 字节（对齐到 52）
        UInt8 bigFrame[256];
        std::memset(bigFrame, 0x55, sizeof(bigFrame));
        const UInt32 big = buildHTTPacket(f, sizeof(f), bigFrame, 100,
                                          0x20000000u, 0, 4, 8, 34, true);
        CHECK(big == 24 + 52);
        CHECK(htcReadLE16(f + 12) == 100);
        CHECK(f[24] == 0x55 && f[24 + 49] == 0x55);

        // AUTH / ASSOC 帧
        const UInt32 auth = buildAuthRequest(frame, sizeof(frame), c, 5);
        CHECK(auth == 30);
        CHECK(frame[0] == 0xb0);
        CHECK(std::memcmp(frame + 4, c.bssid, 6) == 0);
        CHECK(std::memcmp(frame + 10, c.localMac, 6) == 0);
        CHECK(htcReadLE16(frame + 22) == (5 << 4));
        CHECK(htcReadLE16(frame + 24) == 1 && htcReadLE16(frame + 26) == 1);

        const UInt8 rsn[] = {48, 2, 0x01, 0x00};
        const UInt32 assoc = buildAssocRequest(frame, sizeof(frame), c, 6, rsn, sizeof(rsn));
        CHECK(assoc == 28 + (2 + 9) + (2 + 8) + sizeof(rsn));
        CHECK(frame[0] == 0x00);
        CHECK(htcReadLE16(frame + 24) == (1 | 0x20 | 0x10));  // ESS|shortslot|privacy
        CHECK(htcReadLE16(frame + 26) == 1);
        CHECK(frame[28] == 0 && frame[29] == 9);              // SSID IE
        CHECK(std::memcmp(frame + 30, "TestNet-5", 9) == 0);
        CHECK(frame[39] == 1 && frame[40] == 8);              // Rates IE
        CHECK(frame[49] == rsn[0]);                           // RSN IE
    }

    // ---------- 10. HTT RX 环消息 ----------
    {
        UInt8 f[64];
        CHECK(buildHTTVersionReq(f, sizeof(f), 1, 0) == 12);
        CHECK(f[0] == 1 && f[1] == 0 && f[5] == 0);
        CHECK(htcReadLE16(f + 2) == 4);
        CHECK(f[8] == 0);

        CHECK(buildHTTRxRingCfg(f, sizeof(f), 1, 1, 0x10000000u, 0x10001000u) == 48);
        CHECK(f[8] == 2 && f[9] == 1);
        CHECK(htcReadLE16(f + 2) == 40);
        CHECK(wmiReadLE32(f + 12) == 0x10000000u);
        CHECK(wmiReadLE32(f + 16) == 0x10001000u);
        CHECK(htcReadLE16(f + 20) == 2048);
        CHECK(htcReadLE16(f + 22) == 2048);
        CHECK(htcReadLE16(f + 24) == 0xffff);
        CHECK(htcReadLE16(f + 26) == 1023);
        CHECK(htcReadLE16(f + 28) == 59 && htcReadLE16(f + 28 + 18) == 2);
        // 地址不对齐必须拒绝
        CHECK(buildHTTRxRingCfg(f, sizeof(f), 1, 1, 0x10000001u, 0x10001000u) == 0);

        CHECK(buildHTTAggrCfg(f, sizeof(f), 1, 2) == 11);
        CHECK(f[8] == 5 && f[9] == 64 && f[10] == 3);
        CHECK(htcReadLE16(f + 2) == 3);

        // VERSION_CONF 回复：type 0，major=2，minor=1
        UInt8 r[16];
        std::memset(r, 0, sizeof(r));
        r[0] = 1; r[1] = 0;
        put16(r + 2, 4);
        r[8] = 0; r[9] = 1; r[10] = 2;
        UInt32 major = 0, minor = 0, type = 0xff;
        CHECK(parseHTTVersionReply(r, 12, 1, major, minor, type) == nullptr);
        CHECK(major == 2 && minor == 1 && type == 0);
    }

    if (failures == 0)
        printf("ALL WMI TLV TESTS PASSED\n");
    else
        printf("%d TEST(S) FAILED\n", failures);
    return failures == 0 ? 0 : 1;
}
