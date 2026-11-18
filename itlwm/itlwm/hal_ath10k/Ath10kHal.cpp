/*
 * Ath10kHal.cpp
 *
 * QCA9377 (ath10k) net80211 HAL。
 *
 * 当前进度：
 *   M0    ✅ 走通骨架（net80211 附着 + 原生 Wi-Fi 接口可见）
 *   M0.1a ✅ 硬件探测（BAR0 映射成功，MMIO 可读；实测读到 0xDEADBEEF 说明芯片未唤醒）
 *   M0.1b ✅ 目标唤醒 + 冷复位 + 重新读取芯片 ID
 *
 * M0.1b 序列（对齐 Linux ath10k / Ath10kMac 的 probeTarget 前段）：
 *   1. 关闭 PCIe ASPM（复位窗口内链路抖动会导致唤醒失败）
 *   2. 先直接读一次 SOC_CHIP_ID
 *   3. 若为哨兵值：断言 SOC_WAKE → 轮询 RTC_STATE 直到 ON
 *   4. 冷复位：拉高 SOC_GLOBAL_RESET 20ms → 拉低 20ms
 *   5. 再次唤醒 → 重新读 SOC_CHIP_ID
 *   6. 收尾：释放 SOC_WAKE、恢复 ASPM
 *
 * 安全性：只操作唤醒/复位/时钟相关的 PCIe 与 RTC 寄存器，
 *         不配置 CE ring、不启停 DMA、不下载固件（那些属 M0.1c）。
 */

#include "Ath10kHal.hpp"
// 14 MB 的 board/firmware 二进制数组：只在本 TU 编译一次
#include "payload/Ath10kPayload.hpp"

#include <stdarg.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSString.h>
#include <IOKit/IOTimerEventSource.h>
#include <pexpert/pexpert.h>

#define super ItlHalService
OSDefineMetaClassAndStructors(Ath10kHal, ItlHalService)

// PCI 配置空间常量（避免依赖不同 SDK 的符号名）
static constexpr UInt8  kPCICapExpress       = 0x10;  // PCI Express Capability
static constexpr UInt8  kPCIeLinkControlOff  = 0x10;  // Link Control (相对 PCIe cap)
static constexpr UInt16 kPCIeLinkASPMBars    = 0x0003;// L0s | L1
static constexpr UInt16 kPCICfgSubsystemVen  = 0x2c;
static constexpr UInt16 kPCICfgSubsystemId   = 0x2e;

// ---------------------------------------------------------------------------
// 属性发布工具
// ---------------------------------------------------------------------------
static void publishNumber(IOService *service, const char *key, UInt64 value)
{
    if (!service || !key)
        return;
    // 显式走 64 位无符号重载，避免 32 位值被按有符号扩展（M0.1a 踩过的坑）
    OSNumber *number = OSNumber::withNumber((unsigned long long)value, 64);
    if (!number)
        return;
    service->setProperty(key, number);
    number->release();
}

static void publishString(IOService *service, const char *key, const char *value)
{
    if (!service || !key || !value)
        return;
    OSString *string = OSString::withCString(value);
    if (!string)
        return;
    service->setProperty(key, string);
    string->release();
}

static void publishHex32(IOService *service, const char *key, UInt32 value)
{
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "0x%08x", value);
    publishString(service, key, buffer);
}

// 极简字符串相等（kext 里不引入 strcmp 依赖）
static bool ath10kStrEq(const char *a, const char *b)
{
    if (!a || !b)
        return a == b;
    while (*a && *a == *b) {
        ++a;
        ++b;
    }
    return *a == *b;
}

// 扫描失败分级：
//   软失败（固件完成但 reason!=0 / 收集超时）→ 退避后重试；
//   其余（build/rx-arm/CE3/credit/doorbell/前置命令）→ 说明 WMI 通路已经不可用，
//   立刻熔断，绝不再发命令（r0 就是在这种状态里反复重试把系统拖死的）。
static bool ath10kScanFailureIsFatal(const char *status)
{
    if (!status)
        return true;
    if (ath10kStrEq(status, "timeout"))
        return false;
    if (ath10kStrEq(status, "complete-with-reason"))
        return false;
    return true;
}

// SSID 是二进制字段：只保留可打印 ASCII，其余替换为 '.'，
// 避免控制字符把 ioreg / 终端输出搞乱。
static void sanitizeSsid(const UInt8 *src, UInt32 len, char *out, UInt32 capacity)
{
    if (!out || capacity == 0)
        return;
    UInt32 n = 0;
    for (UInt32 i = 0; src && i < len && n + 1 < capacity; ++i) {
        const UInt8 c = src[i];
        out[n++] = (c >= 0x20 && c < 0x7f) ? static_cast<char>(c) : '.';
    }
    out[n] = 0;
}

// ---------------------------------------------------------------------------
// 板卡匹配
// ---------------------------------------------------------------------------
bool Ath10kHal::
ath10k_match(IOPCIDevice *device)
{
    if (!device)
        return false;
    const UInt16 vendor = device->configRead16(kIOPCIConfigVendorID);
    const UInt16 dev    = device->configRead16(kIOPCIConfigDeviceID);
    return Ath10k::isSupportedDevice(vendor, dev);
}

// ---------------------------------------------------------------------------
// M0.1b：PCIe ASPM
// ---------------------------------------------------------------------------
bool Ath10kHal::
disableASPM()
{
    aspmDisabled = false;
    pcieLinkControlBefore = 0xffff;
    pcieLinkControlAfter  = 0xffff;
    pcieCapOffset = 0;

    if (!pciNub)
        return false;

    UInt8 cap = 0;
    if (!pciNub->findPCICapability(kPCICapExpress, &cap) || cap == 0) {
        IOLog("Ath10kHal: ASPM skip (no PCIe capability)\n");
        return false;
    }
    pcieCapOffset = cap;

    const UInt8 offset = static_cast<UInt8>(cap + kPCIeLinkControlOff);
    pcieLinkControlBefore = pciNub->configRead16(offset);
    const UInt16 desired  = static_cast<UInt16>(pcieLinkControlBefore & ~kPCIeLinkASPMBars);
    pciNub->configWrite16(offset, desired);
    pcieLinkControlAfter = pciNub->configRead16(offset);

    aspmDisabled = ((pcieLinkControlAfter & kPCIeLinkASPMBars) == 0);
    IOLog("Ath10kHal: PCIe LinkControl before=0x%04x after=0x%04x aspm=%s\n",
          pcieLinkControlBefore, pcieLinkControlAfter,
          aspmDisabled ? "disabled" : "still-enabled");
    return aspmDisabled;
}

void Ath10kHal::
restoreASPM()
{
    if (!pciNub || pcieCapOffset == 0)
        return;
    const UInt8 offset = static_cast<UInt8>(pcieCapOffset + kPCIeLinkControlOff);
    const UInt16 current = pciNub->configRead16(offset);
    const UInt16 restored = static_cast<UInt16>(
        (current & ~kPCIeLinkASPMBars) | (pcieLinkControlBefore & kPCIeLinkASPMBars));
    pciNub->configWrite16(offset, restored);
    IOLog("Ath10kHal: PCIe LinkControl restored=0x%04x\n", pciNub->configRead16(offset));
}

// ---------------------------------------------------------------------------
// M0.1b：目标唤醒
// ---------------------------------------------------------------------------
bool Ath10kHal::
wakeTarget()
{
    wakeSucceeded = false;
    wakeDelayUs   = 0;
    rtcStateAfter = 0xffffffffu;

    if (!regs.attached())
        return false;

    const auto &layout = Ath10k::kQca6174RegisterLayout;
    const UInt32 rtcOffset  = Ath10k::rtcStateOffset(layout);
    const UInt32 wakeOffset = Ath10k::socWakeOffset(layout);

    if (!regs.read32(rtcOffset, rtcStateBefore)) {
        IOLog("Ath10kHal: wake ERROR RTC_STATE unreadable offset=0x%08x\n", rtcOffset);
        return false;
    }
    IOLog("Ath10kHal: wake RTC_STATE before=0x%08x state=%u\n",
          rtcStateBefore, Ath10k::rtcStateValue(rtcStateBefore));

    // Linux __ath10k_pci_wake() 即使已醒也会断言 SOC_WAKE
    if (!regs.write32(wakeOffset, Ath10k::kPcieSocWakeMask)) {
        IOLog("Ath10kHal: wake ERROR cannot write SOC_WAKE\n");
        return false;
    }
    wakeAsserted = true;

    UInt32 total = 0;
    UInt32 step  = 5;
    UInt32 value = 0xffffffffu;

    while (total < Ath10k::kWakeTimeoutUs) {
        if (!regs.read32(rtcOffset, value)) {
            IOLog("Ath10kHal: wake ERROR RTC_STATE became unreadable\n");
            break;
        }
        if (Ath10k::rtcStateValue(value) == layout.rtcStateValueOn) {
            rtcStateAfter = value;
            wakeDelayUs   = total;
            wakeSucceeded = true;
            IOLog("Ath10kHal: wake OK RTC_STATE=0x%08x delay=%u us\n", value, total);
            return true;
        }
        IODelay(step);
        total += step;
        if (step < 50)
            step += 5;
    }

    rtcStateAfter = value;
    wakeDelayUs   = total;
    IOLog("Ath10kHal: wake TIMEOUT RTC_STATE=0x%08x after=%u us\n", value, total);
    return false;
}

void Ath10kHal::
releaseTargetWake()
{
    if (!wakeAsserted)
        return;
    const UInt32 wakeOffset = Ath10k::socWakeOffset(Ath10k::kQca6174RegisterLayout);
    if (regs.write32(wakeOffset, Ath10k::kPcieSocWakeReset))
        IOLog("Ath10kHal: wake released SOC_WAKE=0\n");
    else
        IOLog("Ath10kHal: wake release ERROR\n");
    wakeAsserted = false;
}

bool Ath10kHal::
ensureTargetAwake(const char *stage)
{
    if (!regs.attached())
        return false;

    const auto &layout = Ath10k::kQca6174RegisterLayout;
    UInt32 rtc = 0xffffffffu;
    const bool readable = regs.read32(Ath10k::rtcStateOffset(layout), rtc);
    const bool on = readable && Ath10k::rtcStateValue(rtc) == layout.rtcStateValueOn;
    lastRtcState = rtc;
    lastRtcReadable = readable;

    if (on) {
        targetAwake = true;
        // Linux 在 power_up 之后一直保持 SOC_WAKE 断言；我们同样保持，
        // 否则 SOC 会在空闲后自行睡下去，后续 CE 寄存器读取会变成 0xDEADBEEF。
        if (!wakeAsserted) {
            const UInt32 wakeOffset = Ath10k::socWakeOffset(layout);
            if (regs.write32(wakeOffset, Ath10k::kPcieSocWakeMask))
                wakeAsserted = true;
        }
        return true;
    }

    targetAwake = false;
    ++wakeRetries;
    IOLog("Ath10kHal: target asleep before %s (RTC_STATE=0x%08x readable=%d), re-waking #%u\n",
          stage ? stage : "?", rtc, (int)readable, wakeRetries);

    // ASPM 若被恢复过，唤醒窗口里链路不稳会失败
    if (!aspmDisabled)
        disableASPM();
    return wakeTarget();
}

// ---------------------------------------------------------------------------
// M0.1b：冷复位
// ---------------------------------------------------------------------------
bool Ath10kHal::
coldResetTarget()
{
    coldResetDone = false;
    globalResetBefore = 0xffffffffu;
    globalResetAfter  = 0xffffffffu;

    if (!regs.attached())
        return false;

    const UInt32 offset = Ath10k::globalResetOffset(Ath10k::kQca6174RegisterLayout);
    if (!regs.read32(offset, globalResetBefore) || globalResetBefore == 0xffffffffu) {
        IOLog("Ath10kHal: cold reset ERROR read SOC_GLOBAL_RESET=0x%08x\n", globalResetBefore);
        return false;
    }

    const UInt32 asserted   = globalResetBefore | Ath10k::kSocGlobalResetMask;
    const UInt32 deasserted = globalResetBefore & ~Ath10k::kSocGlobalResetMask;

    if (!regs.write32(offset, asserted)) {
        IOLog("Ath10kHal: cold reset ERROR assert failed\n");
        return false;
    }
    IOSleep(Ath10k::kColdResetDelayMs);

    if (!regs.write32(offset, deasserted)) {
        IOLog("Ath10kHal: cold reset ERROR deassert failed\n");
        return false;
    }
    IOSleep(Ath10k::kColdResetDelayMs);

    if (!regs.read32(offset, globalResetAfter)) {
        IOLog("Ath10kHal: cold reset ERROR readback failed\n");
        return false;
    }

    coldResetDone = ((globalResetAfter & Ath10k::kSocGlobalResetMask) == 0);
    IOLog("Ath10kHal: cold reset before=0x%08x after=0x%08x %s\n",
          globalResetBefore, globalResetAfter, coldResetDone ? "PASS" : "FAIL");
    return coldResetDone;
}

// ---------------------------------------------------------------------------
// 芯片 ID
// ---------------------------------------------------------------------------
bool Ath10kHal::
readChipIdValue(UInt32 &value)
{
    value = 0xffffffffu;
    if (!regs.attached())
        return false;
    return regs.read32(Ath10k::chipIdOffset(Ath10k::kQca6174RegisterLayout), value);
}

bool Ath10kHal::
isChipIdSentinel(UInt32 value)
{
    // 芯片未上电/未唤醒时常见的非法值
    if (value == 0xffffffffu || value == 0x00000000u || value == 0xdeadbeefu)
        return true;
    return false;
}

// ---------------------------------------------------------------------------
// M0.1c-1：CE ring 的 DMA 分配（纯主机侧）
//
// 按 kCEHostConfig[] 为每个 CE 分配非零的 src/dst ring。
// 目的是提前验证 Ath10kMac README 记录过的关键坑：
// IODMACommand 多段拼接、32 位地址约束、对齐。
// CE4 的 8192 项（64 KiB）ring 正是当年只拿到 4 KiB 段的那一个。
// ---------------------------------------------------------------------------
bool Ath10kHal::
allocateCERings()
{
    ringsAllocated     = false;
    ringsFailure       = "not-run";
    ceConfiguredCount  = 0;
    ceVerifiedCount    = 0;

    if (!regs.attached()) {
        ringsFailure = "no-bar0";
        IOLog("Ath10kHal: CE ring allocate skipped (BAR0 not mapped)\n");
        return false;
    }

    for (UInt32 ceId = 0; ceId < Ath10k::kCECount; ++ceId) {
        const auto &cfg = Ath10k::kCEHostConfig[ceId];
        if (cfg.srcEntries == 0 && cfg.dstEntries == 0)
            continue;
        ++ceConfiguredCount;

        if (cfg.srcEntries != 0) {
            if (!ceSrcRings[ceId].allocate(ceId, Ath10kCERingDirection::Source,
                                           cfg.srcEntries, nullptr)) {
                ringsFailure = ceSrcRings[ceId].allocationFailure();
                IOLog("Ath10kHal: CE%u src ring FAILED reason=%s bytes=%llu align=%llu\n",
                      ceId, ringsFailure,
                      (unsigned long long)ceSrcRings[ceId].allocationFailureRequestedBytes(),
                      (unsigned long long)ceSrcRings[ceId].allocationFailureAlignment());
                return false;
            }
            IOLog("Ath10kHal: CE%u src ok entries=%u bytes=%llu dma=0x%08x role=%s\n",
                  ceId, ceSrcRings[ceId].entries(),
                  (unsigned long long)ceSrcRings[ceId].bytes(),
                  ceSrcRings[ceId].dmaAddress32(), cfg.role);
        }

        if (cfg.dstEntries != 0) {
            if (!ceDstRings[ceId].allocate(ceId, Ath10kCERingDirection::Destination,
                                           cfg.dstEntries, nullptr)) {
                ringsFailure = ceDstRings[ceId].allocationFailure();
                IOLog("Ath10kHal: CE%u dst ring FAILED reason=%s bytes=%llu align=%llu\n",
                      ceId, ringsFailure,
                      (unsigned long long)ceDstRings[ceId].allocationFailureRequestedBytes(),
                      (unsigned long long)ceDstRings[ceId].allocationFailureAlignment());
                return false;
            }
            IOLog("Ath10kHal: CE%u dst ok entries=%u bytes=%llu dma=0x%08x role=%s\n",
                  ceId, ceDstRings[ceId].entries(),
                  (unsigned long long)ceDstRings[ceId].bytes(),
                  ceDstRings[ceId].dmaAddress32(), cfg.role);
        }
    }

    ringsAllocated = true;
    ringsFailure   = "none";
    IOLog("Ath10kHal: CE rings allocated for %u CEs\n", ceConfiguredCount);
    return true;
}

void Ath10kHal::
releaseCERings()
{
    for (UInt32 ceId = 0; ceId < Ath10k::kCECount; ++ceId) {
        ceSrcRings[ceId].release();
        ceDstRings[ceId].release();
    }
    ringsAllocated = false;
    ringsFailure   = "released";
    cePipesProgrammed = false;
}

// ---------------------------------------------------------------------------
// CE 管道编程与校验
// ---------------------------------------------------------------------------
bool Ath10kHal::
verifyCopyEnginePipe(UInt32 ceId, const Ath10k::CESnapshot &snap) const
{
    if (ceId >= Ath10k::kCECount || !snap.valid)
        return false;

    const auto &cfg = Ath10k::kCEHostConfig[ceId];
    bool ok = true;

    if (!cfg.poll) {
        ok &= (snap.hostIE & Ath10k::kCEHostInterruptMask) == 0;
        ok &= (snap.miscIE & Ath10k::kCEMiscErrorMask) == 0;
    }

    if (cfg.srcEntries != 0) {
        ok &= ceSrcRings[ceId].allocated();
        ok &= snap.srcBase == ceSrcRings[ceId].dmaAddress32();
        ok &= snap.srcSize == cfg.srcEntries;
        ok &= (snap.ctrl1 & Ath10k::kCECtrl1DmaxMask) ==
              (cfg.srcSizeMax & Ath10k::kCECtrl1DmaxMask);
        ok &= (snap.ctrl1 & Ath10k::kCECtrl1SrcByteSwapMask) == 0;
        ok &= snap.srcWatermark == Ath10k::watermarkValue(0, cfg.srcEntries);
    }

    if (cfg.dstEntries != 0) {
        ok &= ceDstRings[ceId].allocated();
        ok &= snap.dstBase == ceDstRings[ceId].dmaAddress32();
        ok &= snap.dstSize == cfg.dstEntries;
        ok &= (snap.ctrl1 & Ath10k::kCECtrl1DstByteSwapMask) == 0;
        ok &= snap.dstWatermark == Ath10k::watermarkValue(0, cfg.dstEntries);
    }

    return ok;
}

bool Ath10kHal::
initOneCopyEnginePipe(UInt32 ceId, const char *stage)
{
    if (ceId >= Ath10k::kCECount)
        return false;

    const auto &cfg = Ath10k::kCEHostConfig[ceId];
    if (cfg.srcEntries == 0 && cfg.dstEntries == 0)
        return true;

    const UInt32 base = Ath10k::ceBaseAddress(ceId);
    bool ok = true;

    if (cfg.srcEntries != 0) {
        if (!ceSrcRings[ceId].allocated())
            return false;
        bzero(ceSrcRings[ceId].virtualAddress(),
              static_cast<size_t>(ceSrcRings[ceId].bytes()));
        OSSynchronizeIO();
        if (!ceSrcRings[ceId].syncForDevice()) {
            IOLog("Ath10kHal: CE%u init %s ERROR src syncForDevice\n", ceId, stage);
            return false;
        }

        UInt32 ctrl1 = 0xffffffffu;
        ok &= regs.read32(base + Ath10k::kCECtrl1, ctrl1);
        ctrl1 &= ~(Ath10k::kCECtrl1DmaxMask | Ath10k::kCECtrl1SrcByteSwapMask);
        ctrl1 |= (cfg.srcSizeMax & Ath10k::kCECtrl1DmaxMask);
        ok &= regs.write32(base + Ath10k::kCESrcRingBaseLo, ceSrcRings[ceId].dmaAddress32());
        ok &= regs.write32(base + Ath10k::kCESrcRingSize, cfg.srcEntries);
        ok &= regs.write32(base + Ath10k::kCECtrl1, ctrl1);
        ok &= regs.write32(base + Ath10k::kCESrcWatermark,
                           Ath10k::watermarkValue(0, cfg.srcEntries));
    }

    if (cfg.dstEntries != 0) {
        if (!ceDstRings[ceId].allocated())
            return false;
        bzero(ceDstRings[ceId].virtualAddress(),
              static_cast<size_t>(ceDstRings[ceId].bytes()));
        OSSynchronizeIO();
        if (!ceDstRings[ceId].syncForDevice()) {
            IOLog("Ath10kHal: CE%u init %s ERROR dst syncForDevice\n", ceId, stage);
            return false;
        }

        UInt32 ctrl1 = 0xffffffffu;
        ok &= regs.read32(base + Ath10k::kCECtrl1, ctrl1);
        ctrl1 &= ~Ath10k::kCECtrl1DstByteSwapMask;
        ok &= regs.write32(base + Ath10k::kCEDstRingBaseLo, ceDstRings[ceId].dmaAddress32());
        ok &= regs.write32(base + Ath10k::kCEDstRingSize, cfg.dstEntries);
        ok &= regs.write32(base + Ath10k::kCECtrl1, ctrl1);
        ok &= regs.write32(base + Ath10k::kCEDstWatermark,
                           Ath10k::watermarkValue(0, cfg.dstEntries));
    }

    Ath10k::CESnapshot snap{};
    const bool readOk = Ath10k::readCESnapshot(regs, ceId, snap);
    const bool verified = ok && readOk && verifyCopyEnginePipe(ceId, snap);

    IOLog("Ath10kHal: CE%u init %s readback=%s src=%08x/%u dst=%08x/%u ctrl1=%08x idx=%u/%u,%u/%u role=%s\n",
          ceId, stage, verified ? "PASS" : "FAIL",
          snap.srcBase, snap.srcSize, snap.dstBase, snap.dstSize, snap.ctrl1,
          snap.srcWriteIndex, snap.currentSrcRead,
          snap.dstWriteIndex, snap.currentDstRead, cfg.role);
    return verified;
}

bool Ath10kHal::
initCopyEnginePipes(const char *stage)
{
    ceVerifiedCount = 0;
    UInt32 configured = 0;

    IOLog("Ath10kHal: CE init_pipes begin stage=%s transfers=OFF\n", stage);

    for (UInt32 ceId = 0; ceId < Ath10k::kCECount; ++ceId) {
        const auto &cfg = Ath10k::kCEHostConfig[ceId];
        if (cfg.srcEntries == 0 && cfg.dstEntries == 0)
            continue;
        ++configured;

        if (!initOneCopyEnginePipe(ceId, stage)) {
            IOLog("Ath10kHal: CE init_pipes FAILED stage=%s ce=%u; deinit\n", stage, ceId);
            deinitCopyEnginePipes("init-failure-cleanup");
            return false;
        }
        ++ceVerifiedCount;
    }

    cePipesProgrammed = true;
    const bool ok = (ceVerifiedCount == configured);
    if (ok)
        ++cePipeInitPassCount;

    IOLog("Ath10kHal: CE init_pipes %s stage=%s verified=%u/%u pass-count=%u\n",
          ok ? "READY" : "FAILED", stage, ceVerifiedCount, configured,
          cePipeInitPassCount);
    return ok;
}

bool Ath10kHal::
deinitOneCopyEnginePipe(UInt32 ceId)
{
    if (ceId >= Ath10k::kCECount)
        return false;

    const auto &cfg = Ath10k::kCEHostConfig[ceId];
    if (cfg.srcEntries == 0 && cfg.dstEntries == 0)
        return true;

    const UInt32 base = Ath10k::ceBaseAddress(ceId);
    bool ok = true;

    if (cfg.srcEntries != 0) {
        UInt32 ctrl1 = 0xffffffffu;
        ok &= regs.read32(base + Ath10k::kCECtrl1, ctrl1);
        ctrl1 &= ~Ath10k::kCECtrl1DmaxMask;
        ok &= regs.write32(base + Ath10k::kCESrcRingBaseLo, 0);
        ok &= regs.write32(base + Ath10k::kCESrcRingSize, 0);
        ok &= regs.write32(base + Ath10k::kCECtrl1, ctrl1);
        ok &= regs.write32(base + Ath10k::kCESrcWatermark, 0);
    }
    if (cfg.dstEntries != 0) {
        ok &= regs.write32(base + Ath10k::kCEDstRingBaseLo, 0);
        ok &= regs.write32(base + Ath10k::kCEDstRingSize, 0);
        ok &= regs.write32(base + Ath10k::kCEDstWatermark, 0);
    }

    Ath10k::CESnapshot snap{};
    const bool readOk = Ath10k::readCESnapshot(regs, ceId, snap);
    bool verified = ok && readOk;

    if (cfg.srcEntries != 0) {
        verified &= snap.srcBase == 0 && snap.srcSize == 0;
        verified &= (snap.ctrl1 & Ath10k::kCECtrl1DmaxMask) == 0;
        verified &= (snap.srcWatermark & Ath10k::kCEWatermarkHighMask) == 0;
    }
    if (cfg.dstEntries != 0) {
        verified &= snap.dstBase == 0 && snap.dstSize == 0;
        verified &= (snap.dstWatermark & Ath10k::kCEWatermarkHighMask) == 0;
    }
    return verified;
}

bool Ath10kHal::
deinitCopyEnginePipes(const char *stage)
{
    UInt32 verified = 0;
    UInt32 configured = 0;
    bool allOk = true;

    for (UInt32 ceId = 0; ceId < Ath10k::kCECount; ++ceId) {
        const auto &cfg = Ath10k::kCEHostConfig[ceId];
        if (cfg.srcEntries == 0 && cfg.dstEntries == 0)
            continue;
        ++configured;
        if (deinitOneCopyEnginePipe(ceId))
            ++verified;
        else
            allOk = false;
    }

    const bool ok = allOk && verified == configured;
    if (ok)
        cePipesProgrammed = false;

    IOLog("Ath10kHal: CE deinit_pipes %s stage=%s verified=%u/%u\n",
          ok ? "READY" : "FAILED", stage, verified, configured);
    return ok;
}

// ---------------------------------------------------------------------------
// M0.1c-3：CE7 诊断通道
//
// 这是本项目第一次真正的 CE DMA 事务：通过 CE7 诊断窗口直接读写 target 内存。
// 移植自 Ath10kMac ce7DiagnosticRead()（3683–3944），保留其全部同步纪律：
//   - 描述符/doorbell 写入后必须 syncForDevice
//   - 每次轮询迭代都要 syncFromDevice，否则 IODMACommand 的内部 bounce
//     会让 CPU 读到过期数据（Ath10kMac 踩过的坑）
//   - 提交顺序：先挂 dst，再提交 src
// ---------------------------------------------------------------------------
bool Ath10kHal::
targetCpuToCEAddress(UInt32 targetAddress, UInt32 &ceAddress)
{
    const UInt32 coreCtrlOffset =
        Ath10k::socCoreOffset(Ath10k::kQca6174RegisterLayout, Ath10k::kCoreCtrlAddress);
    UInt32 coreCtrl = 0xffffffffu;
    if (!regs.read32(coreCtrlOffset, coreCtrl) || coreCtrl == 0xffffffffu)
        return false;

    ceDiagCoreCtrl = coreCtrl;

    const UInt32 region      = targetAddress & 0x000fffffu;
    const UInt32 window      = (coreCtrl & 0x000007ffu) << 21;
    const UInt32 upperRegion = (targetAddress >= 0x00100000u) ? 0x00100000u : 0u;
    ceAddress = window | upperRegion | region;
    return true;
}

bool Ath10kHal::
ce7DiagnosticRead(UInt32 targetAddress, void *outData, UInt32 nbytes)
{
    static constexpr UInt32 kDiagCE = 7;

    if (!outData || nbytes == 0 || nbytes > Ath10k::kCEDiagTransferLimit ||
        (targetAddress & 0x3u) != 0 || (nbytes & 0x3u) != 0) {
        IOLog("Ath10kHal: CE7 read ERROR invalid request target=0x%08x bytes=%u\n",
              targetAddress, nbytes);
        return false;
    }

    if (!cePipesProgrammed ||
        !ceSrcRings[kDiagCE].allocated() || !ceDstRings[kDiagCE].allocated() ||
        !ceDiagBounce.allocated() || ceDiagBounce.length() < nbytes) {
        IOLog("Ath10kHal: CE7 read ERROR rings/bounce not ready\n");
        return false;
    }

    Ath10k::CESnapshot before{};
    if (!Ath10k::readCESnapshot(regs, kDiagCE, before) ||
        !verifyCopyEnginePipe(kDiagCE, before)) {
        IOLog("Ath10kHal: CE7 read ERROR pipe readback invalid\n");
        return false;
    }

    const UInt32 srcEntries = Ath10k::kCEHostConfig[kDiagCE].srcEntries;
    const UInt32 dstEntries = Ath10k::kCEHostConfig[kDiagCE].dstEntries;
    if (before.srcWriteIndex >= srcEntries || before.currentSrcRead >= srcEntries ||
        before.dstWriteIndex >= dstEntries || before.currentDstRead >= dstEntries ||
        before.srcWriteIndex != before.currentSrcRead ||
        before.dstWriteIndex != before.currentDstRead) {
        IOLog("Ath10kHal: CE7 read ERROR rings not idle src=%u/%u dst=%u/%u\n",
              before.srcWriteIndex, before.currentSrcRead,
              before.dstWriteIndex, before.currentDstRead);
        return false;
    }

    UInt32 ceAddress = 0xffffffffu;
    if (!targetCpuToCEAddress(targetAddress, ceAddress)) {
        IOLog("Ath10kHal: CE7 read ERROR address conversion target=0x%08x\n", targetAddress);
        return false;
    }

    bzero(ceDiagBounce.virtualAddress(), static_cast<size_t>(nbytes));

    auto *srcRing = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceSrcRings[kDiagCE].virtualAddress());
    auto *dstRing = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kDiagCE].virtualAddress());

    const UInt32 srcSlot = before.srcWriteIndex;
    const UInt32 dstSlot = before.dstWriteIndex;
    const UInt32 srcNext = Ath10k::ceRingNext(srcSlot, srcEntries);
    const UInt32 dstNext = Ath10k::ceRingNext(dstSlot, dstEntries);

    auto *srcDesc = &srcRing[srcSlot];
    auto *dstDesc = &dstRing[dstSlot];

    bzero(srcDesc, sizeof(*srcDesc));
    bzero(dstDesc, sizeof(*dstDesc));

    // 与 Linux ath10k_pci_diag_read_mem() 同序：先挂主机目的缓冲，再提交源描述符
    dstDesc->address = static_cast<UInt32>(ceDiagBounce.dmaAddress());
    dstDesc->nbytes  = 0;
    dstDesc->flags   = 0;

    srcDesc->address = ceAddress;
    srcDesc->nbytes  = static_cast<UInt16>(nbytes);
    srcDesc->flags   = 0;
    OSSynchronizeIO();

    if (!ceDiagBounce.syncForDevice() || !ceDstRings[kDiagCE].syncForDevice()) {
        IOLog("Ath10kHal: CE7 read ERROR syncForDevice before submit\n");
        return false;
    }

    const UInt32 base = Ath10k::ceBaseAddress(kDiagCE);
    IOLog("Ath10kHal: CE7 READ begin target=0x%08x ce=0x%08x bytes=%u core-ctrl=0x%08x bounce=0x%08x\n",
          targetAddress, ceAddress, nbytes, ceDiagCoreCtrl,
          static_cast<UInt32>(ceDiagBounce.dmaAddress()));

    if (!regs.write32(base + Ath10k::kCEDstWriteIndex, dstNext)) {
        IOLog("Ath10kHal: CE7 read ERROR cannot post destination index\n");
        return false;
    }
    OSSynchronizeIO();
    UInt32 dstIdxReadback = 0xffffffffu;
    if (!regs.read32(base + Ath10k::kCEDstWriteIndex, dstIdxReadback) ||
        dstIdxReadback != dstNext) {
        IOLog("Ath10kHal: CE7 read ERROR dst write-index readback=%u expected=%u\n",
              dstIdxReadback, dstNext);
        return false;
    }

    // 目的已挂好后才发布源描述符
    if (!ceSrcRings[kDiagCE].syncForDevice()) {
        IOLog("Ath10kHal: CE7 read ERROR src ring syncForDevice\n");
        return false;
    }

    if (!regs.write32(base + Ath10k::kCESrcWriteIndex, srcNext)) {
        IOLog("Ath10kHal: CE7 read ERROR cannot submit source index\n");
        return false;
    }
    OSSynchronizeIO();

    UInt32 srcIdxReadback = 0xffffffffu;
    if (!regs.read32(base + Ath10k::kCESrcWriteIndex, srcIdxReadback) ||
        srcIdxReadback != srcNext) {
        IOLog("Ath10kHal: CE7 read ERROR src write-index did not latch\n");
        return false;
    }

    // 轮询完成
    bool sourceDone = false;
    bool destinationDone = false;
    UInt16 completedBytes = 0;
    UInt32 waitedUs = 0;
    UInt32 srcRead = 0xffffffffu;
    UInt32 dstRead = 0xffffffffu;

    for (;;) {
        OSSynchronizeIO();

        if (!regs.read32(base + Ath10k::kCECurrentSrcRead, srcRead) ||
            !regs.read32(base + Ath10k::kCECurrentDstRead, dstRead)) {
            IOLog("Ath10kHal: CE7 read ERROR cannot poll read indices\n");
            return false;
        }

        // 每次迭代都必须把设备写回的数据同步进 CPU 视图
        if (!ceDstRings[kDiagCE].syncFromDevice()) {
            IOLog("Ath10kHal: CE7 read ERROR dst ring syncFromDevice\n");
            return false;
        }

        completedBytes  = dstDesc->nbytes;
        sourceDone      = ((srcRead & (srcEntries - 1u)) == srcNext);
        destinationDone = (completedBytes != 0);

        if (sourceDone && destinationDone)
            break;
        if (waitedUs >= Ath10k::kCEDiagAccessTimeoutUs)
            break;

        IODelay(Ath10k::kCEDiagAccessWaitUs);
        waitedUs += Ath10k::kCEDiagAccessWaitUs;
    }

    if (!sourceDone || !destinationDone) {
        IOLog("Ath10kHal: CE7 read TIMEOUT target=0x%08x waited=%u us src-w=%u src-r=%u dst-r=%u completed=%u\n",
              targetAddress, waitedUs, srcNext, srcRead, dstRead, completedBytes);
        return false;
    }

    if (completedBytes != nbytes ||
        dstDesc->address != static_cast<UInt32>(ceDiagBounce.dmaAddress())) {
        IOLog("Ath10kHal: CE7 read ERROR completion mismatch completed=%u expected=%u\n",
              completedBytes, nbytes);
        return false;
    }

    if (!ceDiagBounce.syncFromDevice()) {
        IOLog("Ath10kHal: CE7 read ERROR bounce syncFromDevice\n");
        return false;
    }

    bcopy(ceDiagBounce.virtualAddress(), outData, static_cast<size_t>(nbytes));

    // 清掉描述符完成元数据并重新发布，保证双描述符 ring 绕回时状态确定
    srcDesc->nbytes = 0;
    dstDesc->nbytes = 0;
    OSSynchronizeIO();
    if (!ceSrcRings[kDiagCE].syncForDevice() || !ceDstRings[kDiagCE].syncForDevice()) {
        IOLog("Ath10kHal: CE7 read ERROR descriptor cleanup sync\n");
        return false;
    }

    ++ceDiagTransactions;

    UInt32 value = 0;
    if (nbytes >= sizeof(UInt32))
        bcopy(outData, &value, sizeof(value));

    IOLog("Ath10kHal: CE7 READ ok target=0x%08x ce=0x%08x bytes=%u value0=0x%08x waited=%u us\n",
          targetAddress, ceAddress, completedBytes, value, waitedUs);
    return true;
}

bool Ath10kHal::
runCE7DiagnosticRead()
{
    ceDiagOk = false;
    ceDiagPCIeStateAddress  = 0xffffffffu;
    ceDiagPipeCfgAddress    = 0xffffffffu;
    ceDiagServiceMapAddress = 0xffffffffu;
    ceDiagConfigFlags       = 0xffffffffu;

    if (!warmResetDone || !cePipesProgrammed) {
        IOLog("Ath10kHal: CE7 discovery ERROR warm reset/pipes not ready\n");
        return false;
    }

    // 复用一个 2048 字节 bounce 缓冲（对齐 4）
    if (!ceDiagBounce.allocate(Ath10k::kCEDiagTransferLimit, 4, nullptr)) {
        IOLog("Ath10kHal: CE7 discovery ERROR bounce alloc failed reason=%s\n",
              ceDiagBounce.allocationFailure());
        return false;
    }

    IOLog("Ath10kHal: CE7 discovery begin hi-interconnect=0x%08x timeout=%u us\n",
          Ath10k::kHostInterestInterconnectAddress, Ath10k::kCEDiagAccessTimeoutUs);

    // 1) hi_interconnect_state → pcie_state 地址
    if (!ce7DiagnosticRead(Ath10k::kHostInterestInterconnectAddress,
                           &ceDiagPCIeStateAddress, sizeof(UInt32))) {
        IOLog("Ath10kHal: CE7 discovery ERROR hi_interconnect_state read failed\n");
        return false;
    }
    if (ceDiagPCIeStateAddress == 0 || ceDiagPCIeStateAddress == 0xffffffffu) {
        IOLog("Ath10kHal: CE7 discovery ERROR invalid pcie_state=0x%08x\n",
              ceDiagPCIeStateAddress);
        return false;
    }

    // 2) pipe_cfg 地址
    if (!ce7DiagnosticRead(ceDiagPCIeStateAddress + Ath10k::kPCIeStatePipeCfgOffset,
                           &ceDiagPipeCfgAddress, sizeof(UInt32)) ||
        ceDiagPipeCfgAddress == 0 || ceDiagPipeCfgAddress == 0xffffffffu) {
        IOLog("Ath10kHal: CE7 discovery ERROR pipe_cfg addr=0x%08x\n", ceDiagPipeCfgAddress);
        return false;
    }

    // 3) svc_to_pipe_map 地址
    if (!ce7DiagnosticRead(ceDiagPCIeStateAddress + Ath10k::kPCIeStateServiceMapOffset,
                           &ceDiagServiceMapAddress, sizeof(UInt32)) ||
        ceDiagServiceMapAddress == 0 || ceDiagServiceMapAddress == 0xffffffffu) {
        IOLog("Ath10kHal: CE7 discovery ERROR svc_map addr=0x%08x\n", ceDiagServiceMapAddress);
        return false;
    }

    // 4) config_flags
    if (!ce7DiagnosticRead(ceDiagPCIeStateAddress + Ath10k::kPCIeStateConfigFlagsOffset,
                           &ceDiagConfigFlags, sizeof(UInt32))) {
        IOLog("Ath10kHal: CE7 discovery ERROR config_flags read failed\n");
        return false;
    }

    ceDiagOk = true;
    IOLog("Ath10kHal: CE7 discovery READY transactions=%u pcie-state=0x%08x pipe-cfg=0x%08x svc-map=0x%08x flags=0x%08x\n",
          ceDiagTransactions, ceDiagPCIeStateAddress, ceDiagPipeCfgAddress,
          ceDiagServiceMapAddress, ceDiagConfigFlags);
    return true;
}

// ---------------------------------------------------------------------------
// M0.1c-4：CE7 写通道
//
// 与读对称：src 描述符指向主机 bounce 缓冲，dst 指向 target CE 地址。
// 顺序仍是「先挂 dst，再提交 src」，且 CPU 写完 bounce 必须先 syncForDevice。
// ---------------------------------------------------------------------------
bool Ath10kHal::
ce7DiagnosticWrite(UInt32 targetAddress, const void *data, UInt32 nbytes)
{
    static constexpr UInt32 kDiagCE = 7;

    if (!data || nbytes == 0 || nbytes > Ath10k::kCEDiagTransferLimit ||
        (targetAddress & 0x3u) != 0 || (nbytes & 0x3u) != 0) {
        IOLog("Ath10kHal: CE7 write ERROR invalid request target=0x%08x bytes=%u\n",
              targetAddress, nbytes);
        return false;
    }

    if (!cePipesProgrammed ||
        !ceSrcRings[kDiagCE].allocated() || !ceDstRings[kDiagCE].allocated() ||
        !ceDiagBounce.allocated() || ceDiagBounce.length() < nbytes) {
        IOLog("Ath10kHal: CE7 write ERROR rings/bounce not ready\n");
        return false;
    }

    Ath10k::CESnapshot before{};
    if (!Ath10k::readCESnapshot(regs, kDiagCE, before) ||
        !verifyCopyEnginePipe(kDiagCE, before)) {
        IOLog("Ath10kHal: CE7 write ERROR pipe readback invalid\n");
        return false;
    }

    const UInt32 srcEntries = Ath10k::kCEHostConfig[kDiagCE].srcEntries;
    const UInt32 dstEntries = Ath10k::kCEHostConfig[kDiagCE].dstEntries;
    if (before.srcWriteIndex >= srcEntries || before.currentSrcRead >= srcEntries ||
        before.dstWriteIndex >= dstEntries || before.currentDstRead >= dstEntries ||
        before.srcWriteIndex != before.currentSrcRead ||
        before.dstWriteIndex != before.currentDstRead) {
        IOLog("Ath10kHal: CE7 write ERROR rings not idle\n");
        return false;
    }

    UInt32 ceAddress = 0xffffffffu;
    if (!targetCpuToCEAddress(targetAddress, ceAddress)) {
        IOLog("Ath10kHal: CE7 write ERROR address conversion target=0x%08x\n", targetAddress);
        return false;
    }

    // 数据进 bounce，并立即发布给设备
    bcopy(data, ceDiagBounce.virtualAddress(), static_cast<size_t>(nbytes));
    OSSynchronizeIO();
    if (!ceDiagBounce.syncForDevice()) {
        IOLog("Ath10kHal: CE7 write ERROR bounce syncForDevice\n");
        return false;
    }

    auto *srcRing = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceSrcRings[kDiagCE].virtualAddress());
    auto *dstRing = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kDiagCE].virtualAddress());

    const UInt32 srcSlot = before.srcWriteIndex;
    const UInt32 dstSlot = before.dstWriteIndex;
    const UInt32 srcNext = Ath10k::ceRingNext(srcSlot, srcEntries);
    const UInt32 dstNext = Ath10k::ceRingNext(dstSlot, dstEntries);

    auto *srcDesc = &srcRing[srcSlot];
    auto *dstDesc = &dstRing[dstSlot];
    bzero(srcDesc, sizeof(*srcDesc));
    bzero(dstDesc, sizeof(*dstDesc));

    // Linux ath10k_pci_diag_write_mem()：先挂 target 目的，再发主机源
    dstDesc->address = ceAddress;
    dstDesc->nbytes  = 0;
    dstDesc->flags   = 0;

    srcDesc->address = static_cast<UInt32>(ceDiagBounce.dmaAddress());
    srcDesc->nbytes  = static_cast<UInt16>(nbytes);
    srcDesc->flags   = 0;
    OSSynchronizeIO();

    if (!ceDstRings[kDiagCE].syncForDevice()) {
        IOLog("Ath10kHal: CE7 write ERROR dst ring syncForDevice\n");
        return false;
    }

    const UInt32 base = Ath10k::ceBaseAddress(kDiagCE);
    UInt32 value0 = 0;
    if (nbytes >= sizeof(UInt32))
        bcopy(data, &value0, sizeof(value0));
    IOLog("Ath10kHal: CE7 WRITE begin target=0x%08x ce=0x%08x bytes=%u value0=0x%08x\n",
          targetAddress, ceAddress, nbytes, value0);

    if (!regs.write32(base + Ath10k::kCEDstWriteIndex, dstNext)) {
        IOLog("Ath10kHal: CE7 write ERROR cannot post destination index\n");
        return false;
    }
    OSSynchronizeIO();
    UInt32 dstIdxReadback = 0xffffffffu;
    if (!regs.read32(base + Ath10k::kCEDstWriteIndex, dstIdxReadback) ||
        dstIdxReadback != dstNext) {
        IOLog("Ath10kHal: CE7 write ERROR dst write-index readback=%u expected=%u\n",
              dstIdxReadback, dstNext);
        return false;
    }

    if (!ceSrcRings[kDiagCE].syncForDevice()) {
        IOLog("Ath10kHal: CE7 write ERROR src ring syncForDevice\n");
        return false;
    }

    if (!regs.write32(base + Ath10k::kCESrcWriteIndex, srcNext)) {
        IOLog("Ath10kHal: CE7 write ERROR cannot submit source index\n");
        return false;
    }
    OSSynchronizeIO();

    UInt32 srcIdxReadback = 0xffffffffu;
    if (!regs.read32(base + Ath10k::kCESrcWriteIndex, srcIdxReadback) ||
        srcIdxReadback != srcNext) {
        IOLog("Ath10kHal: CE7 write ERROR src write-index did not latch\n");
        return false;
    }

    bool sourceDone = false;
    bool destinationDone = false;
    UInt16 completedBytes = 0;
    UInt32 waitedUs = 0;
    UInt32 srcRead = 0xffffffffu;
    UInt32 dstRead = 0xffffffffu;

    for (;;) {
        OSSynchronizeIO();
        if (!regs.read32(base + Ath10k::kCECurrentSrcRead, srcRead) ||
            !regs.read32(base + Ath10k::kCECurrentDstRead, dstRead)) {
            IOLog("Ath10kHal: CE7 write ERROR cannot poll read indices\n");
            return false;
        }
        if (!ceDstRings[kDiagCE].syncFromDevice()) {
            IOLog("Ath10kHal: CE7 write ERROR dst ring syncFromDevice\n");
            return false;
        }
        completedBytes  = dstDesc->nbytes;
        sourceDone      = ((srcRead & (srcEntries - 1u)) == srcNext);
        destinationDone = (completedBytes != 0);

        if (sourceDone && destinationDone)
            break;
        if (waitedUs >= Ath10k::kCEDiagAccessTimeoutUs)
            break;

        IODelay(Ath10k::kCEDiagAccessWaitUs);
        waitedUs += Ath10k::kCEDiagAccessWaitUs;
    }

    if (!sourceDone || !destinationDone) {
        IOLog("Ath10kHal: CE7 write TIMEOUT target=0x%08x waited=%u us src-r=%u dst-r=%u completed=%u\n",
              targetAddress, waitedUs, srcRead, dstRead, completedBytes);
        return false;
    }

    if (completedBytes != nbytes || dstDesc->address != ceAddress) {
        IOLog("Ath10kHal: CE7 write ERROR completion mismatch completed=%u expected=%u dst-addr=0x%08x expected=0x%08x\n",
              completedBytes, nbytes, dstDesc->address, ceAddress);
        return false;
    }

    srcDesc->nbytes = 0;
    dstDesc->nbytes = 0;
    OSSynchronizeIO();
    if (!ceSrcRings[kDiagCE].syncForDevice() || !ceDstRings[kDiagCE].syncForDevice()) {
        IOLog("Ath10kHal: CE7 write ERROR descriptor cleanup sync\n");
        return false;
    }

    ++ceDiagTransactions;
    IOLog("Ath10kHal: CE7 WRITE ok target=0x%08x ce=0x%08x bytes=%u waited=%u us\n",
          targetAddress, ceAddress, completedBytes, waitedUs);
    return true;
}

// ---------------------------------------------------------------------------
// M0.1c-4：init_config 持久写入
//
// 移植自 Ath10kMac runCE7InitConfigPersistentTest()（4266–4578）。要点：
//   - 先用 no-op 写/回读证明 target 写通道可用（不改语义，无需回滚）
//   - 写前快照全部区域
//   - 每个区域写完立即回读校验，任一失败即停
//   - early_alloc 必须带 magic 0x6d8a 与真实 bank 数，否则 target 会拒绝
// ---------------------------------------------------------------------------
bool Ath10kHal::
runCE7InitConfig()
{
    initConfigOk = false;
    initConfigOrigFlags = initConfigNewFlags = 0xffffffffu;
    initConfigOrigEarlyAlloc = initConfigNewEarlyAlloc = 0xffffffffu;
    initConfigOrigFlag2 = initConfigNewFlag2 = 0xffffffffu;
    initConfigBanks = 0;

    if (!ceDiagOk || !cePipesProgrammed || !ceDiagBounce.allocated() ||
        ceDiagPCIeStateAddress == 0 || ceDiagPCIeStateAddress == 0xffffffffu ||
        ceDiagPipeCfgAddress == 0 || ceDiagPipeCfgAddress == 0xffffffffu ||
        ceDiagServiceMapAddress == 0 || ceDiagServiceMapAddress == 0xffffffffu) {
        IOLog("Ath10kHal: init_config ERROR discovery/pipes/bounce not ready\n");
        return false;
    }

    initConfigBanks = Ath10k::targetIRAMBanks(pciDeviceId, chipRevision);
    if (initConfigBanks == 0) {
        IOLog("Ath10kHal: init_config ERROR unknown IRAM banks device=%04x rev=%u\n",
              pciDeviceId, chipRevision);
        return false;
    }

    const UInt32 configFlagsAddress =
        ceDiagPCIeStateAddress + Ath10k::kPCIeStateConfigFlagsOffset;

    IOLog("Ath10kHal: init_config begin pipe-cfg=0x%08x(%u) svc-map=0x%08x(%u) banks=%u\n",
          ceDiagPipeCfgAddress, Ath10k::kQCA6174TargetCEConfigBytes,
          ceDiagServiceMapAddress, Ath10k::kQCA6174ServiceMapBytes,
          initConfigBanks);

    // 1) no-op 写/回读：证明 target 写通道可用
    if (!ce7DiagnosticRead(configFlagsAddress, &initConfigOrigFlags, sizeof(UInt32))) {
        IOLog("Ath10kHal: init_config ERROR snapshot config_flags\n");
        return false;
    }
    if (!ce7DiagnosticWrite(configFlagsAddress, &initConfigOrigFlags, sizeof(UInt32))) {
        IOLog("Ath10kHal: init_config ERROR no-op write\n");
        return false;
    }
    UInt32 noOp = 0xffffffffu;
    if (!ce7DiagnosticRead(configFlagsAddress, &noOp, sizeof(UInt32)) ||
        noOp != initConfigOrigFlags) {
        IOLog("Ath10kHal: init_config ERROR no-op readback=0x%08x expected=0x%08x\n",
              noOp, initConfigOrigFlags);
        return false;
    }
    IOLog("Ath10kHal: init_config no-op WRITE PASS config-flags=0x%08x\n", initConfigOrigFlags);

    // 2) 快照其余区域
    if (!ce7DiagnosticRead(Ath10k::kHostInterestEarlyAllocAddress,
                           &initConfigOrigEarlyAlloc, sizeof(UInt32)) ||
        !ce7DiagnosticRead(Ath10k::kHostInterestOptionFlag2Address,
                           &initConfigOrigFlag2, sizeof(UInt32))) {
        IOLog("Ath10kHal: init_config ERROR snapshot HI words\n");
        return false;
    }

    // 3) 计算应用值
    initConfigNewFlags = initConfigOrigFlags & ~Ath10k::kPCIeConfigFlagEnableL1;
    initConfigNewEarlyAlloc = initConfigOrigEarlyAlloc |
        ((Ath10k::kHIEarlyAllocMagic << Ath10k::kHIEarlyAllocMagicShift) &
         Ath10k::kHIEarlyAllocMagicMask) |
        ((initConfigBanks << Ath10k::kHIEarlyAllocIRAMBanksShift) &
         Ath10k::kHIEarlyAllocIRAMBanksMask);
    initConfigNewFlag2 = initConfigOrigFlag2 | Ath10k::kHIOptionEarlyCfgDone;

    IOLog("Ath10kHal: init_config values flags 0x%08x->0x%08x early-alloc 0x%08x->0x%08x flag2 0x%08x->0x%08x\n",
          initConfigOrigFlags, initConfigNewFlags,
          initConfigOrigEarlyAlloc, initConfigNewEarlyAlloc,
          initConfigOrigFlag2, initConfigNewFlag2);

    // 4) pipe_cfg（168 B）+ 回读
    if (!ce7DiagnosticWrite(ceDiagPipeCfgAddress, Ath10k::kQCA6174TargetCEConfig,
                            Ath10k::kQCA6174TargetCEConfigBytes)) {
        IOLog("Ath10kHal: init_config ERROR write pipe_cfg\n");
        return false;
    }
    {
        UInt8 verify[Ath10k::kQCA6174TargetCEConfigBytes];
        bzero(verify, sizeof(verify));
        if (!ce7DiagnosticRead(ceDiagPipeCfgAddress, verify,
                               Ath10k::kQCA6174TargetCEConfigBytes) ||
            bcmp(verify, Ath10k::kQCA6174TargetCEConfig,
                 Ath10k::kQCA6174TargetCEConfigBytes) != 0) {
            IOLog("Ath10kHal: init_config ERROR pipe_cfg readback mismatch\n");
            return false;
        }
    }
    IOLog("Ath10kHal: init_config pipe_cfg readback PASS (%u pipes)\n",
          Ath10k::kQCA6174TargetCEConfigCount);

    // 5) svc_map（204 B）+ 回读
    if (!ce7DiagnosticWrite(ceDiagServiceMapAddress, Ath10k::kQCA6174ServiceMap,
                            Ath10k::kQCA6174ServiceMapBytes)) {
        IOLog("Ath10kHal: init_config ERROR write svc_map\n");
        return false;
    }
    {
        UInt8 verify[Ath10k::kQCA6174ServiceMapBytes];
        bzero(verify, sizeof(verify));
        if (!ce7DiagnosticRead(ceDiagServiceMapAddress, verify,
                               Ath10k::kQCA6174ServiceMapBytes) ||
            bcmp(verify, Ath10k::kQCA6174ServiceMap,
                 Ath10k::kQCA6174ServiceMapBytes) != 0) {
            IOLog("Ath10kHal: init_config ERROR svc_map readback mismatch\n");
            return false;
        }
    }
    IOLog("Ath10kHal: init_config svc_map readback PASS (%u entries)\n",
          Ath10k::kQCA6174ServiceMapCount);

    // 6) config_flags / early_alloc / option_flag2
    const UInt32 earlyAlloc = initConfigNewEarlyAlloc;
    const UInt32 flag2      = initConfigNewFlag2;
    const UInt32 flags      = initConfigNewFlags;

    if (!ce7DiagnosticWrite(configFlagsAddress, &flags, sizeof(UInt32))) {
        IOLog("Ath10kHal: init_config ERROR write config_flags\n");
        return false;
    }
    if (!ce7DiagnosticWrite(Ath10k::kHostInterestEarlyAllocAddress, &earlyAlloc, sizeof(UInt32)) ||
        !ce7DiagnosticWrite(Ath10k::kHostInterestOptionFlag2Address, &flag2, sizeof(UInt32))) {
        IOLog("Ath10kHal: init_config ERROR write HI words\n");
        return false;
    }

    // 7) 终检
    UInt32 rbFlags = 0xffffffffu, rbEarly = 0xffffffffu, rbFlag2 = 0xffffffffu;
    if (!ce7DiagnosticRead(configFlagsAddress, &rbFlags, sizeof(UInt32)) ||
        !ce7DiagnosticRead(Ath10k::kHostInterestEarlyAllocAddress, &rbEarly, sizeof(UInt32)) ||
        !ce7DiagnosticRead(Ath10k::kHostInterestOptionFlag2Address, &rbFlag2, sizeof(UInt32)) ||
        rbFlags != flags || rbEarly != earlyAlloc || rbFlag2 != flag2) {
        IOLog("Ath10kHal: init_config ERROR final readback flags=0x%08x/0x%08x early=0x%08x/0x%08x flag2=0x%08x/0x%08x\n",
              rbFlags, flags, rbEarly, earlyAlloc, rbFlag2, flag2);
        return false;
    }

    initConfigOk = true;
    IOLog("Ath10kHal: init_config READY flags=0x%08x early-alloc=0x%08x flag2=0x%08x transactions=%u\n",
          rbFlags, rbEarly, rbFlag2, ceDiagTransactions);
    return true;
}

// ---------------------------------------------------------------------------
// M0.1c-5：CPU 唤醒与 BMI 传输
//
// 移植自 Ath10kMac wakeTargetCPUForBMI(4581) / bmiExchange(4651) /
// runBMIGetTargetInfoTest(5128)。核心不变量：
//   「提交过 DMA 但完成状态未知」= 污染，禁止重试，只能复位。
// ---------------------------------------------------------------------------
bool Ath10kHal::
wakeTargetCPUForBMI()
{
    cpuWoken = false;
    coreCtrlBeforeWake = coreCtrlAfterWake = 0xffffffffu;

    const UInt32 addr = Ath10k::socCoreOffset(Ath10k::kQca6174RegisterLayout,
                                             Ath10k::kCoreCtrlAddress);
    if (!regs.read32(addr, coreCtrlBeforeWake)) {
        IOLog("Ath10kHal: wake CPU ERROR cannot read CORE_CTRL\n");
        return false;
    }

    const UInt32 requested = coreCtrlBeforeWake | Ath10k::kCoreCtrlCPUIntrMask;
    if (!regs.write32(addr, requested)) {
        IOLog("Ath10kHal: wake CPU ERROR cannot write CORE_CTRL\n");
        return false;
    }

    // 触发器可能自清，回读仅用于记录
    regs.read32(addr, coreCtrlAfterWake);

    cpuWoken = true;
    IOLog("Ath10kHal: wake CPU ok CORE_CTRL 0x%08x -> 0x%08x (mask 0x%08x, readback 0x%08x)\n",
          coreCtrlBeforeWake, requested, Ath10k::kCoreCtrlCPUIntrMask, coreCtrlAfterWake);
    return true;
}

bool Ath10kHal::
validateBMIPipeStructure(UInt32 ceId, const Ath10k::CESnapshot &snap) const
{
    if (ceId >= Ath10k::kCECount || !snap.valid)
        return false;

    const auto &cfg = Ath10k::kCEHostConfig[ceId];
    bool ok = true;

    if (cfg.srcEntries != 0) {
        ok &= snap.srcBase == ceSrcRings[ceId].dmaAddress32();
        ok &= snap.srcSize == cfg.srcEntries;
        ok &= (snap.ctrl1 & Ath10k::kCECtrl1DmaxMask) ==
              (cfg.srcSizeMax & Ath10k::kCECtrl1DmaxMask);
        ok &= (snap.ctrl1 & Ath10k::kCECtrl1SrcByteSwapMask) == 0;
    }
    if (cfg.dstEntries != 0) {
        ok &= snap.dstBase == ceDstRings[ceId].dmaAddress32();
        ok &= snap.dstSize == cfg.dstEntries;
        ok &= (snap.ctrl1 & Ath10k::kCECtrl1DstByteSwapMask) == 0;
    }
    return ok;
}

bool Ath10kHal::
bmiExchange(const void *request, UInt32 requestLength,
            void *response, UInt32 responseLength)
{
    static constexpr UInt32 kTxCE = Ath10k::kBMICENumToTarget; // CE0
    static constexpr UInt32 kRxCE = Ath10k::kBMICENumToHost;   // CE1

    if (!request || requestLength == 0 || !response || responseLength == 0) {
        IOLog("Ath10kHal: BMI ERROR invalid request/response\n");
        return false;
    }
    if (!cePipesProgrammed || !cpuWoken) {
        IOLog("Ath10kHal: BMI ERROR pipes/cpu not ready\n");
        return false;
    }
    if (bmiTransferUncertain || bmiTimedOut) {
        IOLog("Ath10kHal: BMI refused: transfer state uncertain; reset required\n");
        return false;
    }

    if (!bmiRequestBuffer.allocated() &&
        !bmiRequestBuffer.allocate(Ath10k::kBMIResponseBufferSize, 4, nullptr)) {
        IOLog("Ath10kHal: BMI ERROR request buffer alloc reason=%s\n",
              bmiRequestBuffer.allocationFailure());
        return false;
    }
    if (!bmiResponseBuffer.allocated() &&
        !bmiResponseBuffer.allocate(Ath10k::kBMIResponseBufferSize, 4, nullptr)) {
        IOLog("Ath10kHal: BMI ERROR response buffer alloc reason=%s\n",
              bmiResponseBuffer.allocationFailure());
        return false;
    }

    // 结构 + 空闲校验
    Ath10k::CESnapshot txSnap{}, rxSnap{};
    if (!Ath10k::readCESnapshot(regs, kTxCE, txSnap) ||
        !validateBMIPipeStructure(kTxCE, txSnap) ||
        !Ath10k::readCESnapshot(regs, kRxCE, rxSnap) ||
        !validateBMIPipeStructure(kRxCE, rxSnap)) {
        IOLog("Ath10kHal: BMI ERROR pipe structure invalid\n");
        return false;
    }

    const UInt32 txEntries = Ath10k::kCEHostConfig[kTxCE].srcEntries;
    const UInt32 rxEntries = Ath10k::kCEHostConfig[kRxCE].dstEntries;
    if (txSnap.srcWriteIndex != txSnap.currentSrcRead ||
        rxSnap.dstWriteIndex != rxSnap.currentDstRead) {
        IOLog("Ath10kHal: BMI ERROR rings busy tx=%u/%u rx=%u/%u\n",
              txSnap.srcWriteIndex, txSnap.currentSrcRead,
              rxSnap.dstWriteIndex, rxSnap.currentDstRead);
        return false;
    }

    bcopy(request, bmiRequestBuffer.virtualAddress(), requestLength);
    bzero(bmiResponseBuffer.virtualAddress(),
          static_cast<size_t>(bmiResponseBuffer.length()));
    OSSynchronizeIO();
    if (!bmiRequestBuffer.syncForDevice() || !bmiResponseBuffer.syncForDevice()) {
        IOLog("Ath10kHal: BMI ERROR buffer syncForDevice\n");
        return false;
    }

    auto *txRing = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceSrcRings[kTxCE].virtualAddress());
    auto *rxRing = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kRxCE].virtualAddress());

    const UInt32 txNext = Ath10k::ceRingNext(txSnap.srcWriteIndex, txEntries);
    const UInt32 rxNext = Ath10k::ceRingNext(rxSnap.dstWriteIndex, rxEntries);

    auto *txDesc = &txRing[txSnap.srcWriteIndex];
    auto *rxDesc = &rxRing[rxSnap.dstWriteIndex];
    bzero(txDesc, sizeof(*txDesc));
    bzero(rxDesc, sizeof(*rxDesc));

    const UInt32 txBase = Ath10k::ceBaseAddress(kTxCE);
    const UInt32 rxBase = Ath10k::ceBaseAddress(kRxCE);

    // ---- 1) 先挂 RX（CE1 目的）----
    rxDesc->address = static_cast<UInt32>(bmiResponseBuffer.dmaAddress());
    rxDesc->nbytes  = 0;
    rxDesc->flags   = 0;
    OSSynchronizeIO();

    if (!ceDstRings[kRxCE].syncForDevice()) {
        IOLog("Ath10kHal: BMI ERROR rx ring syncForDevice\n");
        return false;
    }
    if (!regs.write32(rxBase + Ath10k::kCEDstWriteIndex, rxNext)) {
        IOLog("Ath10kHal: BMI ERROR cannot post rx index\n");
        return false;
    }
    OSSynchronizeIO();
    {
        UInt32 rb = 0xffffffffu;
        if (!regs.read32(rxBase + Ath10k::kCEDstWriteIndex, rb) || rb != rxNext) {
            IOLog("Ath10kHal: BMI ERROR rx index readback=%u expected=%u\n", rb, rxNext);
            return false;
        }
    }

    // ---- 2) 再提交 TX（CE0 源）----
    txDesc->address = static_cast<UInt32>(bmiRequestBuffer.dmaAddress());
    txDesc->nbytes  = static_cast<UInt16>(requestLength);
    txDesc->flags   = Ath10k::kQCA6174CEDescriptorMetadataAnonymous;
    OSSynchronizeIO();

    if (!ceSrcRings[kTxCE].syncForDevice()) {
        IOLog("Ath10kHal: BMI ERROR tx ring syncForDevice\n");
        return false;
    }

    // 从这里开始 DMA 已提交；任何失败都视为状态不确定
    bmiTransferUncertain = true;

    if (!regs.write32(txBase + Ath10k::kCESrcWriteIndex, txNext)) {
        IOLog("Ath10kHal: BMI ERROR cannot submit tx index; DMA UNCERTAIN\n");
        return false;
    }
    OSSynchronizeIO();
    {
        UInt32 sb = 0xffffffffu;
        if (!regs.read32(txBase + Ath10k::kCESrcWriteIndex, sb) || sb != txNext) {
            IOLog("Ath10kHal: BMI ERROR tx index did not latch; DMA UNCERTAIN\n");
            return false;
        }
    }

    // ---- 3) 轮询完成 ----
    UInt32 waitedUs = 0;
    bool txDone = false;
    bool rxDone = false;
    UInt16 completedBytes = 0;
    UInt32 srcRead = 0xffffffffu;
    UInt32 dstRead = 0xffffffffu;

    for (;;) {
        OSSynchronizeIO();
        if (!regs.read32(txBase + Ath10k::kCECurrentSrcRead, srcRead) ||
            !regs.read32(rxBase + Ath10k::kCECurrentDstRead, dstRead)) {
            IOLog("Ath10kHal: BMI ERROR cannot poll indices; DMA UNCERTAIN\n");
            return false;
        }

        if (!ceDstRings[kRxCE].syncFromDevice()) {
            IOLog("Ath10kHal: BMI ERROR rx ring syncFromDevice; DMA UNCERTAIN\n");
            return false;
        }

        // 只看描述符 nbytes 判 rx 完成（对齐 Ath10kMac，避免 CURRENT_DST_READ 假阴性）
        txDone = ((srcRead & (txEntries - 1u)) == txNext);
        completedBytes = rxDesc->nbytes;
        rxDone = (completedBytes != 0);

        if (txDone && rxDone)
            break;

        if (waitedUs >= Ath10k::kBMICommunicationTimeoutUs) {
            bmiTimedOut = true;
            IOLog("Ath10kHal: BMI TIMEOUT waited=%u us tx=%u/%u rx=%u bytes=%u; state remains UNCERTAIN\n",
                  waitedUs, srcRead, txNext, dstRead, completedBytes);
            return false;
        }

        IOSleep(1);
        waitedUs += 1000;
    }

    bmiTransferUncertain = false;

    if (completedBytes > responseLength) {
        IOLog("Ath10kHal: BMI ERROR response %u > buffer %u\n", completedBytes, responseLength);
        return false;
    }

    if (!bmiResponseBuffer.syncFromDevice()) {
        IOLog("Ath10kHal: BMI ERROR response syncFromDevice\n");
        return false;
    }
    bcopy(bmiResponseBuffer.virtualAddress(), response, completedBytes);

    // 清理描述符并重新发布，保证 ring 绕回时状态确定
    txDesc->nbytes = 0;
    rxDesc->nbytes = 0;
    OSSynchronizeIO();
    if (!ceSrcRings[kTxCE].syncForDevice() || !ceDstRings[kRxCE].syncForDevice()) {
        IOLog("Ath10kHal: BMI ERROR descriptor cleanup sync\n");
        return false;
    }

    IOLog("Ath10kHal: BMI exchange ok req=%u resp=%u waited=%u us\n",
          requestLength, completedBytes, waitedUs);
    return true;
}

bool Ath10kHal::
runBMIGetTargetInfo()
{
    bmiOk = false;
    bmiTargetLength = bmiTargetVersion = bmiTargetType = 0;

    const UInt32 request = Ath10k::kBMIGetTargetInfo;
    Ath10k::BMIGetTargetInfoResponse response{};

    IOLog("Ath10kHal: BMI_GET_TARGET_INFO begin\n");
    if (!bmiExchange(&request, sizeof(request), &response, sizeof(response))) {
        IOLog("Ath10kHal: BMI_GET_TARGET_INFO ERROR transport failed\n");
        return false;
    }

    bmiTargetLength  = response.length;
    bmiTargetVersion = response.version;
    bmiTargetType    = response.type;

    IOLog("Ath10kHal: BMI_GET_TARGET_INFO reply length=%u version=0x%08x type=%u\n",
          bmiTargetLength, bmiTargetVersion, bmiTargetType);

    if (bmiTargetLength != 12 || bmiTargetType != 8) {
        IOLog("Ath10kHal: BMI_GET_TARGET_INFO ERROR unexpected ABI length=%u type=%u\n",
              bmiTargetLength, bmiTargetType);
        return false;
    }
    // 版本号的权威期望值来自 payload Profile（QCA9377 hw1.1 = 0x05020001）。
    // payload 尚未集成到本 kext，这里只校验 ABI 不变量（length/type），
    // 版本比对留到 M0.1c-6 集成 payload 时执行。
    bmiOk = true;
    IOLog("Ath10kHal: BMI_GET_TARGET_INFO READY version=0x%08x (profile check deferred)\n",
          bmiTargetVersion);
    return true;
}

// ---------------------------------------------------------------------------
// M0.1c-6：BMI 内存访问 / PLL / Boot Host Interest
//
// 移植自 Ath10kMac bmiSendOnly(5301) / bmiReadMemory(5246) /
// bmiWriteSOCRegister(5206) / bmiWritePLLWord(5212) /
// configureBootHostInterest(5952) / writeBootHIWord(5928)。
// ---------------------------------------------------------------------------
bool Ath10kHal::
bmiSendOnly(const void *request, UInt32 requestLength)
{
    static constexpr UInt32 kTxCE = Ath10k::kBMICENumToTarget;

    if (!request || requestLength < sizeof(UInt32) ||
        requestLength > Ath10k::kBMIMaxDataSize) {
        IOLog("Ath10kHal: BMI TX-only ERROR invalid request\n");
        return false;
    }
    if (!bmiOk || !bmiPrereqOk || !cePipesProgrammed || !cpuWoken) {
        IOLog("Ath10kHal: BMI TX-only ERROR not ready\n");
        return false;
    }
    if (bmiTransferUncertain || bmiTimedOut) {
        IOLog("Ath10kHal: BMI TX-only refused: uncertain state\n");
        return false;
    }

    // BMI_DONE 关闭 BMI 之后，只允许 DONE 本身通过（它就是在关闭后提交的）
    UInt32 opcode = 0;
    bcopy(request, &opcode, sizeof(opcode));
    const bool isDone = (opcode == Ath10k::kBMIDone);
    if (bmiClosed && !isDone) {
        IOLog("Ath10kHal: BMI TX-only refused: BMI already closed (opcode=%u)\n", opcode);
        return false;
    }
    if (!bmiRequestBuffer.allocated() || !ceSrcRings[kTxCE].allocated()) {
        IOLog("Ath10kHal: BMI TX-only ERROR buffers/rings missing\n");
        return false;
    }

    Ath10k::CESnapshot before{};
    const UInt32 entries = Ath10k::kCEHostConfig[kTxCE].srcEntries;
    if (!Ath10k::readCESnapshot(regs, kTxCE, before) ||
        !validateBMIPipeStructure(kTxCE, before) ||
        before.srcWriteIndex >= entries || before.currentSrcRead >= entries ||
        before.srcWriteIndex != before.currentSrcRead) {
        IOLog("Ath10kHal: BMI TX-only ERROR CE0 structure/idle invalid\n");
        return false;
    }

    bzero(bmiRequestBuffer.virtualAddress(),
          static_cast<size_t>(bmiRequestBuffer.length()));
    bcopy(request, bmiRequestBuffer.virtualAddress(), requestLength);
    if (!bmiRequestBuffer.syncForDevice()) {
        IOLog("Ath10kHal: BMI TX-only ERROR request syncForDevice\n");
        return false;
    }

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceSrcRings[kTxCE].virtualAddress());
    auto *desc = &ring[before.srcWriteIndex];
    bzero(desc, sizeof(*desc));
    desc->address = static_cast<UInt32>(bmiRequestBuffer.dmaAddress());
    desc->nbytes  = static_cast<UInt16>(requestLength);
    desc->flags   = Ath10k::kQCA6174CEDescriptorMetadataAnonymous;
    OSSynchronizeIO();

    if (!ceSrcRings[kTxCE].syncForDevice()) {
        IOLog("Ath10kHal: BMI TX-only ERROR ring syncForDevice\n");
        return false;
    }

    const UInt32 next = Ath10k::ceRingNext(before.srcWriteIndex, entries);
    const UInt32 base = Ath10k::ceBaseAddress(kTxCE);

    bmiTransferUncertain = true;
    if (!regs.write32(base + Ath10k::kCESrcWriteIndex, next)) {
        IOLog("Ath10kHal: BMI TX-only ERROR submit failed; UNCERTAIN\n");
        return false;
    }
    OSSynchronizeIO();
    {
        UInt32 sb = 0xffffffffu;
        if (!regs.read32(base + Ath10k::kCESrcWriteIndex, sb) || sb != next) {
            IOLog("Ath10kHal: BMI TX-only ERROR index not latched; UNCERTAIN\n");
            return false;
        }
    }

    UInt32 waited = 0;
    for (;;) {
        OSSynchronizeIO();
        UInt32 srcRead = 0xffffffffu;
        if (!regs.read32(base + Ath10k::kCECurrentSrcRead, srcRead))
            return false;
        if (srcRead >= entries)
            return false;
        if (srcRead == next)
            break;
        if (waited >= Ath10k::kBMICommunicationTimeoutUs) {
            bmiTimedOut = true;
            IOLog("Ath10kHal: BMI TX-only TIMEOUT src=%u/%u; reset required\n", srcRead, next);
            return false;
        }
        IOSleep(1);
        waited += 1000;
    }

    bmiTransferUncertain = false;
    desc->nbytes = 0;
    OSSynchronizeIO();
    if (!ceSrcRings[kTxCE].syncForDevice())
        return false;

    return true;
}

bool Ath10kHal::
bmiReadMemory(UInt32 targetAddress, void *outData, UInt32 length)
{
    if (!outData || !Ath10k::bmiMemoryRangeValid(targetAddress, length) || !bmiOk) {
        IOLog("Ath10kHal: BMI_READ_MEMORY ERROR invalid addr=0x%08x len=%u\n",
              targetAddress, length);
        return false;
    }

    auto *out = reinterpret_cast<UInt8 *>(outData);
    UInt32 remaining = length;
    UInt32 address = targetAddress;

    while (remaining != 0) {
        const UInt32 chunk = remaining > Ath10k::kBMIMaxDataSize
            ? Ath10k::kBMIMaxDataSize : remaining;

        Ath10k::BMIReadMemoryRequest request{};
        request.id = Ath10k::kBMIReadMemory;
        request.address = address;
        request.length = chunk;

        UInt8 response[Ath10k::kBMIMaxDataSize]{};
        UInt32 responseLength = chunk;

        if (!bmiExchange(&request, sizeof(request), response, responseLength)) {
            IOLog("Ath10kHal: BMI_READ_MEMORY ERROR transport addr=0x%08x bytes=%u\n",
                  address, chunk);
            return false;
        }
        if (responseLength != chunk) {
            IOLog("Ath10kHal: BMI_READ_MEMORY ERROR length addr=0x%08x got=%u want=%u\n",
                  address, responseLength, chunk);
            return false;
        }

        bcopy(response, out, chunk);
        address   += chunk;
        out       += chunk;
        remaining -= chunk;
    }
    return true;
}

bool Ath10kHal::
bmiWriteMemory(UInt32 targetAddress, const void *data, UInt32 length)
{
    if (!data || (targetAddress & 3u) ||
        !Ath10k::bmiMemoryRangeValid(targetAddress, length)) {
        IOLog("Ath10kHal: BMI_WRITE_MEMORY ERROR invalid addr=0x%08x len=%u\n",
              targetAddress, length);
        return false;
    }

    const auto *bytes = reinterpret_cast<const UInt8 *>(data);
    UInt32 remaining = length;
    UInt32 address = targetAddress;

    while (remaining != 0) {
        Ath10k::BMIWriteMemoryRequest request{};
        UInt32 consumed = 0;
        const UInt32 wireBytes = Ath10k::buildBMIWriteRequest(
            request, address, bytes, remaining, consumed);
        if (wireBytes == 0) {
            IOLog("Ath10kHal: BMI_WRITE_MEMORY ERROR build addr=0x%08x\n", address);
            return false;
        }
        if (!bmiSendOnly(&request, wireBytes)) {
            IOLog("Ath10kHal: BMI_WRITE_MEMORY ERROR transport addr=0x%08x\n", address);
            return false;
        }
        address   += consumed;
        bytes     += consumed;
        remaining -= consumed;
    }
    return true;
}

bool Ath10kHal::
bmiReadSOCRegister(UInt32 address, UInt32 &value)
{
    value = 0;
    if (!bmiOk || (address & 3u))
        return false;

    const Ath10k::BMIReadSOCRequest request{Ath10k::kBMIReadSOCRegister, address};
    UInt32 bytes = sizeof(value);
    return bmiExchange(&request, sizeof(request), &value, bytes) &&
           bytes == sizeof(value);
}

bool Ath10kHal::
bmiWriteSOCRegister(UInt32 address, UInt32 value)
{
    if (!bmiOk || !bmiPrereqOk || (address & 3u))
        return false;
    const Ath10k::BMIWriteSOCRequest request{Ath10k::kBMIWriteSOCRegister, address, value};
    return bmiSendOnly(&request, sizeof(request));
}

bool Ath10kHal::
bmiWritePLLWord(UInt32 address, UInt32 value)
{
    // PLL 只允许三步固定参数：0x404028=1、0x404020=1、0x404024=176MHz
    const bool parameter =
        (pll.step == 5  && address == 0x404028u && value == 1u) ||
        (pll.step == 12 && address == 0x404020u && value == 1u) ||
        (pll.step == 13 && address == 0x404024u &&
         hwCandidate && value == hwCandidate->targetCpuFrequencyHz);
    if (!parameter || !bmiOk || !bmiPrereqOk || !pll.attempted || pll.complete)
        return false;

    Ath10k::BMIWriteMemoryRequest request{};
    UInt32 consumed = 0;
    const UInt32 bytes = Ath10k::buildBMIWriteRequest(
        request, address, reinterpret_cast<const UInt8 *>(&value), sizeof(value), consumed);
    if (bytes != 16 || consumed != sizeof(value) || !bmiSendOnly(&request, bytes))
        return false;

    IOLog("Ath10kHal: PLL BMI_WRITE_MEMORY ok addr=0x%08x value=0x%08x step=%u\n",
          address, value, pll.step);
    return true;
}

bool Ath10kHal::
configureTargetPLL()
{
    pllOk = false;
    if (!hwCandidate || !bmiOk || !bmiPrereqOk)
        return false;

    const bool ok = Ath10k::configurePLL(
        pll, hwCandidate->targetCpuFrequencyHz,
        [this](UInt32 a, UInt32 &v) { return bmiReadSOCRegister(a, v); },
        [this](UInt32 a, UInt32 v) { return bmiWriteSOCRegister(a, v); },
        [this](UInt32 a, UInt32 v) { return bmiWritePLLWord(a, v); },
        [](UInt32 us) { IODelay(us); },
        []() -> UInt64 {
            uint64_t t = 0, n = 0;
            clock_get_uptime(&t);
            absolutetime_to_nanoseconds(t, &n);
            return n / 1000;
        });

    pllOk = ok;
    IOLog("Ath10kHal: PLL %s ref=%u ref-hz=%u step=%u polls=%u\n",
          pll.status, pll.reference, pll.referenceHz, pll.step, pll.polls);
    return ok;
}

bool Ath10kHal::
runBMIBoardPrerequisiteProbe()
{
    bmiPrereqOk = false;
    hiBoardData = hiBoardDataInit = hiEarlyAlloc = 0xffffffffu;
    hiInterconnect = hiOptionFlag2 = hiCalData = 0xffffffffu;

    const struct { const char *name; UInt32 address; UInt32 *slot; } words[] = {
        {"hi_board_data",             Ath10k::kHostInterestBoardDataAddress,     &hiBoardData},
        {"hi_board_data_initialized", Ath10k::kHostInterestBoardDataInitAddress, &hiBoardDataInit},
        {"hi_early_alloc",            Ath10k::kHostInterestEarlyAllocAddress,    &hiEarlyAlloc},
        {"hi_interconnect_state",     Ath10k::kHostInterestInterconnectAddress,  &hiInterconnect},
        {"hi_option_flag2",           Ath10k::kHostInterestOptionFlag2Address,   &hiOptionFlag2},
        {"hi_cal_data",               Ath10k::kHostInterestCalDataAddress,       &hiCalData},
    };

    for (const auto &w : words) {
        if (!bmiReadMemory(w.address, w.slot, sizeof(UInt32))) {
            IOLog("Ath10kHal: BMI prereq ERROR read %s (0x%08x)\n", w.name, w.address);
            return false;
        }
        IOLog("Ath10kHal: BMI prereq %-28s = 0x%08x\n", w.name, *w.slot);
    }

    // board data 指针必须落在保留的 board RAM 区间内
    if (hiBoardData == 0 || hiBoardData == 0xffffffffu ||
        hiBoardData < 0x401000u || hiBoardData >= 0x404000u) {
        IOLog("Ath10kHal: BMI prereq ERROR board pointer 0x%08x outside board RAM\n",
              hiBoardData);
        return false;
    }

    bmiPrereqOk = true;
    IOLog("Ath10kHal: BMI prereq READY board=0x%08x init=%u interconnect=0x%08x\n",
          hiBoardData, hiBoardDataInit, hiInterconnect);
    return true;
}

bool Ath10kHal::
verifyBootHIWord(UInt32 address, UInt32 value)
{
    if (!Ath10k::isBootHIWord(address))
        return false;

    UInt32 observed = 0xffffffffu;
    if (!bmiReadMemory(address, &observed, sizeof(UInt32)) || observed != value) {
        IOLog("Ath10kHal: BOOT HI VERIFY FAILED addr=0x%08x expected=0x%08x observed=0x%08x\n",
              address, value, observed);
        return false;
    }
    return true;
}

bool Ath10kHal::
writeBootHIWord(UInt32 address, UInt32 value)
{
    if (!bmiPrereqOk || !Ath10k::isBootHIWord(address))
        return false;

    Ath10k::BMIWriteMemoryRequest request{};
    UInt32 consumed = 0;
    const UInt32 wireBytes = Ath10k::buildBMIWriteRequest(
        request, address, reinterpret_cast<const UInt8 *>(&value), sizeof(value), consumed);
    if (wireBytes != 16 || consumed != 4 || !bmiSendOnly(&request, wireBytes))
        return false;

    ++hiWordsWritten;
    if (!verifyBootHIWord(address, value))
        return false;

    IOLog("Ath10kHal: BOOT HI VERIFIED addr=0x%08x value=0x%08x\n", address, value);
    return true;
}

bool Ath10kHal::
configureBootHostInterest()
{
    bootHIOk = false;
    bootHIOptionBefore = bootHIOptionApplied = 0xffffffffu;
    hiWordsWritten = 0;

    if (!bmiOk || !bmiPrereqOk)
        return false;

    // 与 ath10k_init_configure_target() 同序
    if (!writeBootHIWord(Ath10k::kBootHIApp, Ath10k::kBootHTCProtocolVersion))
        return false;
    if (!bmiReadMemory(Ath10k::kBootHIOption, &bootHIOptionBefore, sizeof(UInt32)) ||
        bootHIOptionBefore == 0xffffffffu)
        return false;

    bootHIOptionApplied = bootHIOptionBefore | Ath10k::kBootHIOptionBits;

    if (!writeBootHIWord(Ath10k::kBootHIOption, bootHIOptionApplied) ||
        !writeBootHIWord(Ath10k::kBootHIHostBE, 0) ||
        !writeBootHIWord(Ath10k::kBootHIFWSwap, 0) ||
        !writeBootHIWord(Ath10k::kBootHIPCIId, pciDeviceId))
        return false;

    bootHIOk = true;
    IOLog("Ath10kHal: BOOT HI CONFIGURED protocol=%u option=0x%08x->0x%08x pci-id=0x%04x\n",
          Ath10k::kBootHTCProtocolVersion, bootHIOptionBefore,
          bootHIOptionApplied, pciDeviceId);
    return true;
}

// ---------------------------------------------------------------------------
// M0.1c-7：board data 上传
//
// 移植自 Ath10kMac runBMIBoardUploadTest(5426)。这是规格里标注的
// 「非破坏性里程碑」：board 8124 B 写入 target ROM 提供的地址后逐字节回读，
// 全部一致则证明 BMI 大块传输可靠，且不动固件执行区。
// ---------------------------------------------------------------------------
bool Ath10kHal::
runBMIBoardUpload()
{
    boardWriteOk = boardVerifyOk = false;
    boardBytes = 0;

    if (!payloadSelected || !payloadProfile || !payloadProfile->available) {
        IOLog("Ath10kHal: board upload ERROR no payload for this card\n");
        return false;
    }
    if (!bmiOk || !bmiPrereqOk || !bootHIOk) {
        IOLog("Ath10kHal: board upload ERROR BMI/Boot HI not ready\n");
        return false;
    }
    if (!payloadProfile->boardAddressValid(hiBoardData)) {
        IOLog("Ath10kHal: board upload ERROR board addr 0x%08x invalid for %u bytes\n",
              hiBoardData, payloadProfile->kBoardSize);
        return false;
    }

    boardBytes = payloadProfile->kBoardSize;
    IOLog("Ath10kHal: board upload begin addr=0x%08x bytes=%u key=%s\n",
          hiBoardData, boardBytes, payloadProfile->kBoardKey);

    // 1) 写入 board data
    if (!bmiWriteMemory(hiBoardData, payloadProfile->kBoard, boardBytes)) {
        IOLog("Ath10kHal: board upload ERROR write failed\n");
        return false;
    }
    boardWriteOk = true;

    // 2) 分块回读并逐字节比对（避免在内核栈上放 8 KB）
    UInt8 verify[Ath10k::kBMIMaxDataSize];
    UInt32 offset = 0;
    while (offset < boardBytes) {
        UInt32 chunk = boardBytes - offset;
        if (chunk > Ath10k::kBMIMaxDataSize)
            chunk = Ath10k::kBMIMaxDataSize;

        if (!bmiReadMemory(hiBoardData + offset, verify, chunk)) {
            IOLog("Ath10kHal: board upload ERROR readback at 0x%08x\n", hiBoardData + offset);
            return false;
        }
        if (bcmp(verify, payloadProfile->kBoard + offset, chunk) != 0) {
            IOLog("Ath10kHal: board upload ERROR mismatch at 0x%08x (%u bytes)\n",
                  hiBoardData + offset, chunk);
            return false;
        }
        offset += chunk;
    }
    boardVerifyOk = true;

    // 3) 主动置位 hi_board_data_initialized 并回读验证
    //    注意：这个标志由 host 写入，不是等 ROM 自己置位
    //    （Ath10kMac runBMIBoardUploadTest 的做法，真机验证过）
    const UInt32 initialized = 1;
    UInt32 observed = 0xffffffffu;
    if (!bmiWriteMemory(Ath10k::kHostInterestBoardDataInitAddress,
                        &initialized, sizeof(initialized))) {
        IOLog("Ath10kHal: board upload ERROR cannot write init flag\n");
        return false;
    }
    if (!bmiReadMemory(Ath10k::kHostInterestBoardDataInitAddress,
                       &observed, sizeof(observed)) ||
        observed != 1) {
        IOLog("Ath10kHal: board upload ERROR init flag readback=%u expected=1\n", observed);
        return false;
    }
    hiBoardDataInit = observed;

    IOLog("Ath10kHal: board upload VERIFIED bytes=%u init=1 (written by host); OTP stage may follow\n",
          boardBytes);
    return true;
}

// ---------------------------------------------------------------------------
// M0.1c-8：OTP 下载与执行
//
// 移植自 Ath10kMac bmiLZStreamStart(5589) / bmiLZData(5615) / runBMIOtpTest(5642)。
// 流程：LZ_STREAM_START(0x1234) → LZ_DATA × N → LZ_STREAM_START(0) → BMI_EXECUTE
// 对齐 Linux ath10k_bmi_fast_download：4 字节对齐的头部 + 独立的零填充尾部。
// ---------------------------------------------------------------------------
bool Ath10kHal::
bmiLZStreamStart(UInt32 address)
{
    if (!otpAttempted || !boardUploadOk || otpStreamClosed || otpExecuteAttempted)
        return false;

    if (address == Ath10k::kQCA6174OTPLoadAddress) {
        if (otpStreamStarted)
            return false;             // 只能开始一次
    } else if (address == 0) {
        // 收尾时必须已发送全部对齐后的字节
        if (!otpStreamStarted || !payloadSelected ||
            otpWireBytesSent != ((payloadProfile->kOTPSize + 3u) & ~3u))
            return false;
    } else {
        return false;
    }

    const Ath10k::BMILZStreamStartRequest request{Ath10k::kBMILZStreamStart, address};
    if (!bmiSendOnly(&request, sizeof(request)))
        return false;

    if (address == 0)
        otpStreamClosed = true;
    else
        otpStreamStarted = true;

    IOLog("Ath10kHal: OTP LZ_STREAM_START ok address=0x%08x\n", address);
    return true;
}

bool Ath10kHal::
bmiLZData(const UInt8 *data, UInt32 length)
{
    if (!payloadSelected)
        return false;

    const UInt32 paddedSize = (payloadProfile->kOTPSize + 3u) & ~3u;
    if (!data || length == 0 || (length & 3u) || !otpStreamStarted ||
        otpStreamClosed || otpExecuteAttempted || otpWireBytesSent > paddedSize ||
        length > paddedSize - otpWireBytesSent)
        return false;

    UInt32 offset = 0;
    while (offset < length) {
        Ath10k::BMILZDataRequest request{};
        UInt32 consumed = 0;
        const UInt32 wireSize = Ath10k::buildBMILZDataRequest(
            request, data + offset, length - offset, consumed);
        if (!wireSize || !bmiSendOnly(&request, wireSize)) {
            IOLog("Ath10kHal: OTP LZ_DATA FAILED sent=%u command=%u\n",
                  otpWireBytesSent, otpLZDataCommands + 1);
            return false;
        }
        offset           += consumed;
        otpWireBytesSent += consumed;
        ++otpLZDataCommands;
    }
    return true;
}

bool Ath10kHal::
runBMIOtp()
{
    otpDownloadOk = otpExecuteOk = otpResultAccepted = false;
    otpResult = 0xffffffffu;
    otpWireBytesSent = 0;
    otpLZDataCommands = 0;
    otpStreamStarted = otpStreamClosed = otpExecuteAttempted = false;
    otpStatus = "prerequisite-mismatch";

    if (otpAttempted)
        return false;                 // 同一次上电内不允许重来
    if (!boardUploadOk || !bmiOk || !bmiPrereqOk || !payloadSelected ||
        payloadProfile->kOTPSize == 0) {
        IOLog("Ath10kHal: OTP BLOCKED: verified board/payload required\n");
        return false;
    }

    otpAttempted = true;
    otpStatus = "lz-start-failed";
    IOLog("Ath10kHal: OTP BEGIN bytes=%u sha=%s load=0x%08x ignore-result=%s\n",
          payloadProfile->kOTPSize, payloadProfile->kOTPSHA256,
          Ath10k::kQCA6174OTPLoadAddress,
          payloadProfile->kOTPIgnoreResult ? "yes" : "no");

    if (!bmiLZStreamStart(Ath10k::kQCA6174OTPLoadAddress))
        return false;

    const UInt32 head = payloadProfile->kOTPSize & ~3u;
    const UInt32 tail = payloadProfile->kOTPSize - head;

    otpStatus = "lz-data-failed";
    if (head && !bmiLZData(payloadProfile->kOTP, head))
        return false;
    if (tail) {
        UInt8 trailer[4]{};
        bcopy(payloadProfile->kOTP + head, trailer, tail);
        if (!bmiLZData(trailer, sizeof(trailer)))
            return false;
    }
    otpStatus = "lz-flush-failed";
    if (!bmiLZStreamStart(0))
        return false;

    otpDownloadOk = true;
    IOLog("Ath10kHal: OTP LZ download ok wire-bytes=%u commands=%u\n",
          otpWireBytesSent, otpLZDataCommands);

    // BMI_EXECUTE 执行 OTP
    const Ath10k::BMIExecuteRequest request{Ath10k::kBMIExecute,
                                            Ath10k::kQCA6174OTPLoadAddress,
                                            Ath10k::kQCA6174OTPExecuteParam};
    UInt32 result = 0xffffffffu;
    UInt32 responseBytes = sizeof(result);
    otpExecuteAttempted = true;
    otpStatus = "execute-transport-failed";

    if (!bmiExchange(&request, sizeof(request), &result, responseBytes))
        return false;
    if (responseBytes != sizeof(result)) {
        otpStatus = "execute-response-size-invalid";
        IOLog("Ath10kHal: OTP EXECUTE invalid response bytes=%u\n", responseBytes);
        return false;
    }

    otpExecuteOk = true;
    otpResult = result;
    otpResultAccepted = Ath10k::otpResultAccepted(result, payloadProfile->kOTPIgnoreResult);
    otpStatus = !otpResultAccepted ? "otp-result-rejected"
               : (result != 0 ? "returned-nonzero-accepted" : "returned-zero");

    IOLog("Ath10kHal: OTP EXECUTE result=0x%08x accepted=%s status=%s\n",
          result, otpResultAccepted ? "yes" : "no", otpStatus);
    return otpResultAccepted;
}

// ---------------------------------------------------------------------------
// M0.1c-9：WLAN 固件下载
//
// 与 OTP 同一套 BMI LZ 机制，但量大 30 倍（727 KB / 2932 条命令）。
// 移植自 Ath10kMac bmiFirmwareLZStart(5712) / bmiFirmwareLZData(5732) /
// verifyFirmwareSegments(5764) / runBMIFirmwareDownloadTest(5811)。
// 注意：本阶段**不发 BMI_DONE**，DONE 属 M0.1c-10。
// ---------------------------------------------------------------------------
bool Ath10kHal::
bmiFirmwareLZStart(UInt32 address)
{
    // 前置：固件阶段已开始、OTP 已完成且结果被接受、流尚未关闭
    if (!fwAttempted || !otpDownloadOk || !otpExecuteOk || !otpResultAccepted ||
        fwStreamClosed) {
        IOLog("Ath10kHal: WLAN LZ start blocked attempted=%d otp-dl=%d otp-exec=%d otp-accepted=%d closed=%d\n",
              (int)fwAttempted, (int)otpDownloadOk, (int)otpExecuteOk,
              (int)otpResultAccepted, (int)fwStreamClosed);
        return false;
    }

    if (address == Ath10k::kQCA6174FirmwareLoadAddress) {
        if (fwStreamStarted)
            return false;
    } else if (address == 0) {
        if (!fwStreamStarted || !payloadSelected ||
            fwWireBytesSent != ((payloadProfile->kFirmwareSize + 3u) & ~3u))
            return false;
    } else {
        return false;
    }

    const Ath10k::BMILZStreamStartRequest request{Ath10k::kBMILZStreamStart, address};
    if (!bmiSendOnly(&request, sizeof(request)))
        return false;

    if (address == 0)
        fwStreamClosed = true;
    else
        fwStreamStarted = true;

    IOLog("Ath10kHal: WLAN LZ_STREAM_START ok address=0x%08x\n", address);
    return true;
}

bool Ath10kHal::
bmiFirmwareLZData(const UInt8 *data, UInt32 length)
{
    if (!payloadSelected)
        return false;

    const UInt32 paddedSize = (payloadProfile->kFirmwareSize + 3u) & ~3u;
    if (!data || length == 0 || (length & 3u) || !fwStreamStarted ||
        fwStreamClosed || fwWireBytesSent > paddedSize ||
        length > paddedSize - fwWireBytesSent)
        return false;

    UInt32 offset = 0;
    while (offset < length) {
        Ath10k::BMILZDataRequest request{};
        UInt32 consumed = 0;
        const UInt32 wireSize = Ath10k::buildBMILZDataRequest(
            request, data + offset, length - offset, consumed);
        if (!wireSize || !bmiSendOnly(&request, wireSize)) {
            IOLog("Ath10kHal: WLAN LZ_DATA FAILED sent=%u command=%u\n",
                  fwWireBytesSent, fwLZDataCommands + 1);
            return false;
        }
        offset           += consumed;
        fwWireBytesSent  += consumed;
        ++fwLZDataCommands;

        if ((fwLZDataCommands % 512u) == 0)
            IOLog("Ath10kHal: WLAN LZ progress %u/%u bytes, %u commands\n",
                  fwWireBytesSent, paddedSize, fwLZDataCommands);
    }
    return true;
}

bool Ath10kHal::
verifyFirmwareSegments()
{
    fwReadbackBytes = 0;
    fwVerifiedSegments = 0;
    fwFailedSegment = 0xffffffffu;
    fwReadbackOk = false;

    if (!payloadSelected)
        return false;

    for (UInt32 index = 0; index < payloadProfile->kFirmwareSegmentCount; ++index) {
        const auto &segment = payloadProfile->kFirmwareSegments[index];
        fwFailedSegment = index;

        if (!segment.length || (segment.address & 3u) ||
            segment.offset > payloadProfile->kFirmwareSize ||
            segment.length > payloadProfile->kFirmwareSize - segment.offset) {
            IOLog("Ath10kHal: WLAN readback ERROR segment %u invalid\n", index);
            return false;
        }

        for (UInt32 offset = 0; offset < segment.length; ) {
            UInt8 buffer[Ath10k::kBMIMaxDataSize]{};
            UInt32 count = segment.length - offset;
            if (count > sizeof(buffer))
                count = sizeof(buffer);

            // 按整字读，只比较实际段内字节：末段长度为奇数，RAM 填充值未定义
            const UInt32 readSize = (count + 3u) & ~3u;
            if (!bmiReadMemory(segment.address + offset, buffer, readSize)) {
                IOLog("Ath10kHal: WLAN readback ERROR segment=%u addr=0x%08x\n",
                      index, segment.address + offset);
                return false;
            }

            const UInt8 *expected = payloadProfile->kFirmware + segment.offset + offset;
            if (bcmp(buffer, expected, count) != 0) {
                UInt32 differing = 0;
                while (differing < count && buffer[differing] == expected[differing])
                    ++differing;
                IOLog("Ath10kHal: WLAN readback MISMATCH segment=%u addr=0x%08x expected=%02x actual=%02x\n",
                      index, segment.address + offset + differing,
                      expected[differing], buffer[differing]);
                return false;
            }
            offset           += count;
            fwReadbackBytes  += count;
        }
        ++fwVerifiedSegments;
        IOLog("Ath10kHal: WLAN readback VERIFIED segment=%u bytes=%u total=%u\n",
              index, segment.length, fwReadbackBytes);
    }

    fwFailedSegment = 0xffffffffu;
    fwReadbackOk = (fwVerifiedSegments == payloadProfile->kFirmwareSegmentCount) &&
                   (fwReadbackBytes == payloadProfile->kFirmwareSegmentDataBytes);
    return fwReadbackOk;
}

bool Ath10kHal::
runBMIFirmwareDownload()
{
    fwDownloadOk = fwReadbackOk = false;
    fwWireBytesSent = 0;
    fwLZDataCommands = 0;
    fwReadbackBytes = 0;
    fwVerifiedSegments = 0;
    fwFailedSegment = 0xffffffffu;
    fwPostVersion = fwPostType = 0xffffffffu;
    fwStreamStarted = fwStreamClosed = false;
    fwStatus = "prerequisite-mismatch";

    if (fwAttempted)
        return false;                 // 不允许重试已部分下载的镜像
    if (!boardUploadOk || !otpDownloadOk || !otpExecuteOk || !otpResultAccepted ||
        !bmiOk || !payloadSelected) {
        IOLog("Ath10kHal: WLAN download BLOCKED: board/OTP prerequisites\n");
        return false;
    }

    fwAttempted = true;
    fwStatus = "lz-start-failed";
    IOLog("Ath10kHal: WLAN BEGIN version=%s bytes=%u sha=%s otp-result=0x%08x\n",
          payloadProfile->kFirmwareVersion, payloadProfile->kFirmwareSize,
          payloadProfile->kFirmwareSHA256, otpResult);

    if (!bmiFirmwareLZStart(Ath10k::kQCA6174FirmwareLoadAddress))
        return false;

    const UInt32 head = payloadProfile->kFirmwareSize & ~3u;
    const UInt32 tail = payloadProfile->kFirmwareSize - head;

    fwStatus = "lz-data-failed";
    if (head && !bmiFirmwareLZData(payloadProfile->kFirmware, head))
        return false;
    if (tail) {
        UInt8 trailer[4]{};
        bcopy(payloadProfile->kFirmware + head, trailer, tail);
        if (!bmiFirmwareLZData(trailer, sizeof(trailer)))
            return false;
    }

    fwStatus = "lz-flush-failed";
    if (!bmiFirmwareLZStart(0))
        return false;

    fwDownloadOk = true;
    IOLog("Ath10kHal: WLAN LZ download ok wire-bytes=%u commands=%u\n",
          fwWireBytesSent, fwLZDataCommands);

    // 下载后复读 GET_TARGET_INFO，确认 ABI 未变
    fwStatus = "post-download-bmi-failed";
    const UInt32 command = Ath10k::kBMIGetTargetInfo;
    Ath10k::BMIGetTargetInfoResponse response{};
    UInt32 responseLength = sizeof(response);
    if (!bmiExchange(&command, sizeof(command), &response, responseLength))
        return false;

    fwPostVersion = response.version;
    fwPostType    = response.type;

    if (responseLength != sizeof(response) ||
        response.length != sizeof(response) ||
        response.version != bmiTargetVersion ||
        response.type != bmiTargetType) {
        IOLog("Ath10kHal: WLAN post-download GET_TARGET_INFO mismatch len=%u ver=0x%08x type=%u\n",
              responseLength, response.version, response.type);
        return false;
    }
    IOLog("Ath10kHal: WLAN post-download GET_TARGET_INFO stable ver=0x%08x\n", response.version);

    // 6 段回读校验
    fwStatus = "readback-failed";
    if (!verifyFirmwareSegments())
        return false;

    fwStatus = "download-verified";
    IOLog("Ath10kHal: WLAN DOWNLOAD VERIFIED image=%u data=%u segments=%u entry=0x%08x; BMI_DONE not sent yet\n",
          payloadProfile->kFirmwareSize, fwReadbackBytes, fwVerifiedSegments,
          payloadProfile->kFirmwareEntryAddress);
    return true;
}

// ---------------------------------------------------------------------------
// M0.1c-10：Boot HI 收尾 / BMI_DONE / HTC READY
//
// 移植自 Ath10kMac finishBootHostInterest(5999) / prepareHTCReadyReceive(6026) /
// waitForHTCReady(6081)。这一步是单向的：
//   - BMI_DONE 一旦提交，CE1 永久归固件所有，不能再发任何 BMI 命令
//   - bmiClosed 在 doorbell 之前置位，即使 MMIO 失败也不允许重试
// ---------------------------------------------------------------------------
bool Ath10kHal::
verifyBootHIOptionsAfterDownload()
{
    bootHIOptionAfterDownload = 0xffffffffu;
    if (!bmiReadMemory(Ath10k::kBootHIOption, &bootHIOptionAfterDownload, sizeof(UInt32))) {
        IOLog("Ath10kHal: BOOT HI OPTION read failed\n");
        return false;
    }

    // 允许固件追加 HI_OPTION_BMI_CRED_LIMIT(bit1)，其余位必须保持
    if (!Ath10k::bootHIOptionMatchesAfterDownload(bootHIOptionApplied,
                                                  bootHIOptionAfterDownload)) {
        IOLog("Ath10kHal: BOOT HI OPTION verify FAILED applied=0x%08x observed=0x%08x\n",
              bootHIOptionApplied, bootHIOptionAfterDownload);
        return false;
    }

    const UInt32 changed = bootHIOptionApplied ^ bootHIOptionAfterDownload;
    IOLog("Ath10kHal: BOOT HI OPTION verified applied=0x%08x observed=0x%08x changed=0x%08x credit-limit-added=%s\n",
          bootHIOptionApplied, bootHIOptionAfterDownload, changed,
          changed == Ath10k::kBootHIOptionBMICreditLimit ? "yes" : "no");
    return true;
}

bool Ath10kHal::
finishBootHostInterest()
{
    uartDisabled = false;
    preDoneHIVerified = false;

    if (!bootHIOk || !fwReadbackOk || bmiClosed || htcRxOwned) {
        IOLog("Ath10kHal: BOOT HI finish BLOCKED boot-hi=%d fw-readback=%d bmi-closed=%d rx-owned=%d\n",
              (int)bootHIOk, (int)fwReadbackOk, (int)bmiClosed, (int)htcRxOwned);
        return false;
    }

    // ath10k_init_uart()：固件下载后关闭 UART 输出
    if (!writeBootHIWord(Ath10k::kBootHISerial, 0))
        return false;
    uartDisabled = true;

    if (!verifyBootHIWord(Ath10k::kBootHIApp, Ath10k::kBootHTCProtocolVersion) ||
        !verifyBootHIOptionsAfterDownload() ||
        !verifyBootHIWord(Ath10k::kBootHIHostBE, 0) ||
        !verifyBootHIWord(Ath10k::kBootHIFWSwap, 0) ||
        !verifyBootHIWord(Ath10k::kBootHIPCIId, pciDeviceId)) {
        IOLog("Ath10kHal: BOOT HI final verify FAILED\n");
        return false;
    }

    UInt32 board = 0, initialized = 0;
    if (!bmiReadMemory(Ath10k::kHostInterestBoardDataAddress, &board, sizeof(board)) ||
        !bmiReadMemory(Ath10k::kHostInterestBoardDataInitAddress, &initialized, sizeof(initialized)) ||
        board != hiBoardData || initialized != 1) {
        IOLog("Ath10kHal: BOOT HI final check board=0x%08x init=%u FAILED\n", board, initialized);
        return false;
    }

    preDoneHIVerified = true;
    IOLog("Ath10kHal: BOOT HI FINAL VERIFIED writes=%u UART=off board=0x%08x init=%u\n",
          hiWordsWritten, board, initialized);
    return true;
}

bool Ath10kHal::
prepareHTCReadyReceive()
{
    static constexpr UInt32 kRxCE = Ath10k::kBMICENumToHost;   // CE1

    htcRxPosted = htcRxOwned = htcRxCompleted = false;
    htcRxBytes = 0;
    htcPrefixBytes = 0;
    htcFwIndicator = 0xffffffffu;
    htcStatus = "not-run";

    const UInt32 entries = Ath10k::kCEHostConfig[kRxCE].dstEntries;
    if (!preDoneHIVerified || !uartDisabled || htcRxOwned || bmiClosed ||
        bmiTransferUncertain || !cePipesProgrammed) {
        IOLog("Ath10kHal: HTC RX arm BLOCKED pre-done=%d uart=%d rx-owned=%d bmi-closed=%d\n",
              (int)preDoneHIVerified, (int)uartDisabled, (int)htcRxOwned, (int)bmiClosed);
        return false;
    }

    Ath10k::CESnapshot before{};
    if (!Ath10k::readCESnapshot(regs, kRxCE, before) ||
        !validateBMIPipeStructure(kRxCE, before) ||
        (before.ctrl1 & Ath10k::kCECtrl1DmaxMask) != Ath10k::kHTCReadyBufferBytes ||
        before.dstWriteIndex >= entries || before.currentDstRead >= entries ||
        before.dstWriteIndex != before.currentDstRead) {
        IOLog("Ath10kHal: HTC RX arm ERROR CE1 structure/idle invalid dmax=0x%x w/r=%u/%u\n",
              before.ctrl1 & Ath10k::kCECtrl1DmaxMask, before.dstWriteIndex,
              before.currentDstRead);
        return false;
    }

    // 固件崩溃指示：bit0
    if (!regs.read32(Ath10k::kQca6174RegisterLayout.fwIndicatorAddress, htcFwIndicator) ||
        htcFwIndicator == 0xffffffffu) {
        IOLog("Ath10kHal: HTC RX arm ERROR cannot read FW_INDICATOR\n");
        return false;
    }
    if (htcFwIndicator & Ath10k::kFwIndicatorEventPending) {
        htcStatus = "firmware-crash-indicated";
        IOLog("Ath10kHal: HTC RX arm ERROR firmware crash indicated 0x%08x\n", htcFwIndicator);
        return false;
    }

    if (!htcReadyBuffer.allocated() &&
        !htcReadyBuffer.allocate(Ath10k::kHTCReadyBufferBytes,
                                 Ath10k::kCEDescriptorRingAlign, nullptr)) {
        IOLog("Ath10kHal: HTC RX arm ERROR buffer alloc reason=%s\n",
              htcReadyBuffer.allocationFailure());
        return false;
    }
    if (htcReadyBuffer.length() != Ath10k::kHTCReadyBufferBytes)
        return false;

    bzero(htcReadyBuffer.virtualAddress(), static_cast<size_t>(htcReadyBuffer.length()));
    if (!htcReadyBuffer.syncForDevice())
        return false;

    htcRxSlot = before.dstWriteIndex;
    htcRxNext = Ath10k::ceRingNext(htcRxSlot, entries);
    htcRxRead = before.currentDstRead;

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kRxCE].virtualAddress());
    auto *desc = &ring[htcRxSlot];
    bzero(desc, sizeof(*desc));
    desc->address = static_cast<UInt32>(htcReadyBuffer.dmaAddress());
    OSSynchronizeIO();

    if (!ceDstRings[kRxCE].syncForDevice())
        return false;

    // CE1 从此归固件所有；CE0 还剩唯一一条命令 BMI_DONE
    htcRxOwned = true;

    const UInt32 doorbell = Ath10k::ceBaseAddress(kRxCE) + Ath10k::kCEDstWriteIndex;
    if (!regs.write32(doorbell, htcRxNext) ||
        !regs.read32(doorbell, htcRxWrite) || htcRxWrite != htcRxNext) {
        IOLog("Ath10kHal: HTC RX arm ERROR doorbell readback=%u expected=%u\n",
              htcRxWrite, htcRxNext);
        return false;
    }

    htcRxPosted = true;
    htcStatus = "rx-armed";
    IOLog("Ath10kHal: HTC RX ARMED CE1 slot=%u next=%u bytes=%u dma=0x%08x fw-indicator=0x%08x\n",
          htcRxSlot, htcRxNext, Ath10k::kHTCReadyBufferBytes,
          static_cast<UInt32>(htcReadyBuffer.dmaAddress()), htcFwIndicator);
    return true;
}

bool Ath10kHal::
bmiSendDone()
{
    if (bmiClosed || doneAttempted)
        return false;
    if (!htcRxPosted || !htcRxOwned || !preDoneHIVerified) {
        IOLog("Ath10kHal: BMI_DONE BLOCKED rx-posted=%d rx-owned=%d pre-done=%d\n",
              (int)htcRxPosted, (int)htcRxOwned, (int)preDoneHIVerified);
        return false;
    }

    // 在 doorbell 之前永久关闭 BMI：即使后面 MMIO 失败也不允许再发任何命令
    bmiClosed = true;
    doneAttempted = true;

    const UInt32 command = Ath10k::kBMIDone;
    IOLog("Ath10kHal: BMI_DONE submit (one-way; BMI closed)\n");
    if (!bmiSendOnly(&command, sizeof(command))) {
        IOLog("Ath10kHal: BMI_DONE transport FAILED; bring-up cannot continue\n");
        return false;
    }

    doneSent = true;
    IOLog("Ath10kHal: BMI_DONE consumed by CE0; waiting for HTC READY\n");
    return true;
}

bool Ath10kHal::
waitForHTCReady()
{
    static constexpr UInt32 kRxCE = Ath10k::kBMICENumToHost;   // CE1

    htcReadyOk = false;
    htcWaitMs = 0;
    htcParseError = "none";
    runtimeRxSlot = runtimeRxNext = 0xffffffffu;
    htcCtrlTxCount = htcCtrlRxCount = 0;
    htcCtrlLastWaitMs = 0;
    htcServicesOk = false;
    htcServiceStatus = "not-run";
    wmiEndpoint = httEndpoint = 0xff;
    wmiMaxMessageSize = httMaxMessageSize = 0;
    wmiRxSlot = wmiRxNext = 0xffffffffu;
    wmiRxBytes = wmiEventId = wmiWaitMs = 0;
    setupCompleteSent = wmiServiceReady = false;
    wmiServiceBits = 0;
    for (UInt32 i = 0; i < 4; ++i) wmiServiceBitmap[i] = 0;
    for (UInt32 i = 0; i < 6; ++i) wmiAbi[i] = 0;
    wmiAbiPresent = false;
    wmiEvtPrefixBytes = 0;
    bzero(wmiEvtPrefix, sizeof(wmiEvtPrefix));
    wmiStatus = "not-run";
    wmiParseError = "none";
    htcStatus = "not-run";

    const UInt32 entries = Ath10k::kCEHostConfig[kRxCE].dstEntries;
    if (!doneSent || !bmiClosed || !htcRxPosted || !htcRxOwned ||
        htcRxCompleted || htcRxSlot >= entries || !htcReadyBuffer.allocated())
        return false;

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kRxCE].virtualAddress());
    const auto *desc = &ring[htcRxSlot];

    for (;;) {
        htcStatus = "htc-mmio-failed";
        OSSynchronizeIO();

        if (!regs.read32(Ath10k::ceBaseAddress(kRxCE) + Ath10k::kCECurrentDstRead, htcRxRead) ||
            htcRxRead >= entries ||
            (htcRxRead != htcRxSlot && htcRxRead != htcRxNext) ||
            !regs.read32(Ath10k::kQca6174RegisterLayout.fwIndicatorAddress, htcFwIndicator) ||
            htcFwIndicator == 0xffffffffu)
            return false;

        if (htcFwIndicator & Ath10k::kFwIndicatorEventPending) {
            htcStatus = "firmware-crash-indicated";
            IOLog("Ath10kHal: HTC READY firmware crash indicated 0x%08x\n", htcFwIndicator);
            return false;
        }

        htcStatus = "htc-ring-sync-failed";
        if (!ceDstRings[kRxCE].syncFromDevice())
            return false;

        htcRxBytes = desc->nbytes;
        if (htcRxBytes != 0)
            break;   // 描述符完成即为权威，即使 DRRI 还没走

        if (htcWaitMs >= Ath10k::kHTCReadyTimeoutMs) {
            htcStatus = "htc-ready-timeout";
            IOLog("Ath10kHal: HTC READY TIMEOUT waited=%u ms CE1 w/r=%u/%u slot=%u bytes=%u fw=0x%08x\n",
                  htcWaitMs, htcRxWrite, htcRxRead, htcRxSlot, htcRxBytes, htcFwIndicator);
            return false;
        }
        IOSleep(Ath10k::kHTCReadyPollMs);
        htcWaitMs += Ath10k::kHTCReadyPollMs;
    }

    htcRxCompleted = true;
    htcStatus = "htc-descriptor-invalid";

    if (desc->address != static_cast<UInt32>(htcReadyBuffer.dmaAddress()) ||
        htcRxBytes > htcReadyBuffer.length()) {
        IOLog("Ath10kHal: HTC READY descriptor invalid addr=0x%08x bytes=%u\n",
              desc->address, htcRxBytes);
        return false;
    }

    htcStatus = "htc-buffer-sync-failed";
    if (!htcReadyBuffer.syncFromDevice())
        return false;

    const auto *bytes = reinterpret_cast<const UInt8 *>(htcReadyBuffer.virtualAddress());
    htcPrefixBytes = htcRxBytes < sizeof(htcPrefix) ? htcRxBytes : sizeof(htcPrefix);
    bcopy(bytes, htcPrefix, htcPrefixBytes);

    char hex[sizeof(htcPrefix) * 3 + 1]{};
    static const char digits[] = "0123456789abcdef";
    for (UInt32 i = 0; i < htcPrefixBytes; ++i) {
        hex[i * 3]     = digits[bytes[i] >> 4];
        hex[i * 3 + 1] = digits[bytes[i] & 15];
        hex[i * 3 + 2] = ' ';
    }
    IOLog("Ath10kHal: HTC READY RX bytes=%u waited=%u ms CE1 w/r=%u/%u prefix=%s\n",
          htcRxBytes, htcWaitMs, htcRxWrite, htcRxRead, hex);

    const char *error = Ath10k::parseHTCReady(bytes, htcRxBytes, htcReady);
    if (error) {
        htcParseError = error;
        htcStatus = "htc-ready-invalid";
        IOLog("Ath10kHal: HTC READY INVALID reason=%s\n", error);
        return false;
    }

    htcReadyOk = true;
    htcStatus = "htc-ready";
    IOLog("Ath10kHal: HTC READY VERIFIED credits=%u credit-size=%u endpoints=%u extended=%s version=%u bundle-max=%u alt-credit=%u\n",
          htcReady.credits, htcReady.creditSize, htcReady.maxEndpoints,
          htcReady.extended ? "yes" : "no", htcReady.version,
          htcReady.maxBundleMessages, htcReady.altDataCreditSize);
    return true;
}

// ---------------------------------------------------------------------------
// M1-a：HTC 服务建立
//
// 移植自 Ath10kMac htcSubmitControlFrame(1377) / runHTCServiceSetup(1588)。
// HTC READY 之后 CE1 已收到一包，必须先重挂描述符才能继续接收。
// 服务顺序：HTT（关闭 credit flow）→ WMI（把 credits 放进 flags 高字节）。
// ---------------------------------------------------------------------------
bool Ath10kHal::
armRuntimeRx()
{
    static constexpr UInt32 kRxCE = Ath10k::kBMICENumToHost;

    runtimeRxArmed = false;
    if (!cePipesProgrammed || bmiTransferUncertain)
        return false;

    const UInt32 entries = Ath10k::kCEHostConfig[kRxCE].dstEntries;

    if (!htcCtrlRxBuffer.allocated() &&
        !htcCtrlRxBuffer.allocate(Ath10k::kHTCReadyBufferBytes,
                                  Ath10k::kCEDescriptorRingAlign, nullptr)) {
        IOLog("Ath10kHal: runtime RX alloc failed reason=%s\n",
              htcCtrlRxBuffer.allocationFailure());
        return false;
    }

    Ath10k::CESnapshot snap{};
    if (!Ath10k::readCESnapshot(regs, kRxCE, snap) ||
        !validateBMIPipeStructure(kRxCE, snap)) {
        IOLog("Ath10kHal: runtime RX ERROR CE1 structure invalid\n");
        return false;
    }
    if (snap.dstWriteIndex >= entries || snap.currentDstRead >= entries ||
        snap.dstWriteIndex != snap.currentDstRead) {
        IOLog("Ath10kHal: runtime RX ERROR CE1 not idle w/r=%u/%u\n",
              snap.dstWriteIndex, snap.currentDstRead);
        return false;
    }

    bzero(htcCtrlRxBuffer.virtualAddress(),
          static_cast<size_t>(htcCtrlRxBuffer.length()));
    if (!htcCtrlRxBuffer.syncForDevice())
        return false;

    runtimeRxSlot = snap.dstWriteIndex;
    runtimeRxNext = Ath10k::ceRingNext(runtimeRxSlot, entries);

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kRxCE].virtualAddress());
    auto *desc = &ring[runtimeRxSlot];
    bzero(desc, sizeof(*desc));
    desc->address = static_cast<UInt32>(htcCtrlRxBuffer.dmaAddress());
    OSSynchronizeIO();

    if (!ceDstRings[kRxCE].syncForDevice())
        return false;

    const UInt32 doorbell = Ath10k::ceBaseAddress(kRxCE) + Ath10k::kCEDstWriteIndex;
    UInt32 readback = 0xffffffffu;
    if (!regs.write32(doorbell, runtimeRxNext) ||
        !regs.read32(doorbell, readback) || readback != runtimeRxNext) {
        IOLog("Ath10kHal: runtime RX ERROR doorbell readback=%u expected=%u\n",
              readback, runtimeRxNext);
        return false;
    }
    runtimeRxArmed = true;
    return true;
}

// CE1 收到一包后把描述符推进到下一槽位（参考文献 htcRepostRuntimeReceive）。
bool Ath10kHal::
htcRuntimeRepostRx()
{
    static constexpr UInt32 kRxCE = Ath10k::kBMICENumToHost;

    const UInt32 entries = Ath10k::kCEHostConfig[kRxCE].dstEntries;
    if (!cePipesProgrammed || entries == 0 || !htcCtrlRxBuffer.allocated() ||
        !ceDstRings[kRxCE].allocated() ||
        runtimeRxSlot >= entries || runtimeRxNext >= entries)
        return false;

    Ath10k::CESnapshot snap{};
    if (!Ath10k::readCESnapshot(regs, kRxCE, snap) ||
        !validateBMIPipeStructure(kRxCE, snap) ||
        snap.dstWriteIndex != runtimeRxNext || snap.currentDstRead >= entries ||
        !ceDstRings[kRxCE].syncFromDevice()) {
        IOLog("Ath10kHal: runtime RX repost ERROR CE1 state w/r=%u/%u expected-next=%u\n",
              snap.dstWriteIndex, snap.currentDstRead, runtimeRxNext);
        return false;
    }

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kRxCE].virtualAddress());
    auto *old = &ring[runtimeRxSlot];
    if (!old->nbytes || old->address != static_cast<UInt32>(htcCtrlRxBuffer.dmaAddress())) {
        IOLog("Ath10kHal: runtime RX repost ERROR old descriptor nbytes=%u addr=0x%08x\n",
              old->nbytes, old->address);
        return false;
    }

    const UInt32 slot = runtimeRxNext;
    const UInt32 next = Ath10k::ceRingNext(slot, entries);
    bzero(old, sizeof(*old));
    auto *desc = &ring[slot];
    bzero(desc, sizeof(*desc));
    desc->address = static_cast<UInt32>(htcCtrlRxBuffer.dmaAddress());
    bzero(htcCtrlRxBuffer.virtualAddress(),
          static_cast<size_t>(htcCtrlRxBuffer.length()));
    OSSynchronizeIO();

    if (!htcCtrlRxBuffer.syncForDevice() || !ceDstRings[kRxCE].syncForDevice())
        return false;

    const UInt32 doorbell = Ath10k::ceBaseAddress(kRxCE) + Ath10k::kCEDstWriteIndex;
    UInt32 readback = 0xffffffffu;
    if (!regs.write32(doorbell, next) ||
        !regs.read32(doorbell, readback) || readback != next)
        return false;

    runtimeRxSlot = slot;
    runtimeRxNext = next;
    runtimeRxArmed = true;
    return true;
}

bool Ath10kHal::
htcEnsureRuntimeRx()
{
    if (runtimeRxArmed)
        return true;
    if (runtimeRxSlot == 0xffffffffu || runtimeRxNext == 0xffffffffu)
        return armRuntimeRx();
    return htcRuntimeRepostRx();
}

// 非阻塞消费 CE1 上的一个包。
// 当前只处理 HTC trailer 的信用回报；HTT/其它 HTC 消息在 M1-c 实现分发时再处理。
bool Ath10kHal::
htcRuntimePump(bool &hasPacket)
{
    static constexpr UInt32 kRxCE = Ath10k::kBMICENumToHost;

    hasPacket = false;
    const UInt32 entries = Ath10k::kCEHostConfig[kRxCE].dstEntries;
    if (!cePipesProgrammed || entries == 0 || !htcCtrlRxBuffer.allocated() ||
        !ceDstRings[kRxCE].allocated())
        return true;

    if (!htcEnsureRuntimeRx()) {
        wmiEventStatus = "ce1-ensure-rx-failed";
        return false;
    }

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kRxCE].virtualAddress());
    auto *desc = &ring[runtimeRxSlot];
    if (!ceDstRings[kRxCE].syncFromDevice()) {
        wmiEventStatus = "ce1-sync-failed";
        return false;
    }
    const UInt32 bytes = desc->nbytes;
    if (!bytes)
        return true;

    if (desc->address != static_cast<UInt32>(htcCtrlRxBuffer.dmaAddress()) ||
        bytes > htcCtrlRxBuffer.length()) {
        wmiEventStatus = "ce1-descriptor-invalid";
        return false;
    }
    if (!htcCtrlRxBuffer.syncFromDevice()) {
        wmiEventStatus = "ce1-buffer-sync-failed";
        return false;
    }
    hasPacket = true;
    runtimeRxArmed = false;

    const auto *f = reinterpret_cast<const UInt8 *>(htcCtrlRxBuffer.virtualAddress());
    bool applied = false;
    const char *error = Ath10k::applyHTCCreditTrailer(f, bytes, wmiCredits, applied);
    if (error) {
        wmiParseError = error;
        wmiEventStatus = "ce1-credit-invalid";
        IOLog("Ath10kHal: CE1 trailer INVALID reason=%s bytes=%u endpoint=%u\n",
              error, bytes, f[0]);
        return false;
    }
    if (f[0] != 0 || !applied) {
        // 非控制端点或非纯信用帧：当前阶段只计数与识别 HTT 类型
        ++ce1Frames;
        ce1LastEndpoint = f[0];
        ce1LastBytes = bytes;
        if (httEndpoint != 0xff && f[0] == httEndpoint && bytes >= 9) {
            ++httMessages;
            httLastType = f[8];
            handleHTTMessage(f, bytes);   // M1-d：TX 完成事件
            httLastPrefixBytes = bytes < sizeof(httLastPrefix) ? bytes
                                                              : static_cast<UInt32>(sizeof(httLastPrefix));
            for (UInt32 i = 0; i < httLastPrefixBytes; ++i)
                httLastPrefix[i] = f[i];
            UInt32 major = 0, minor = 0, type = 0xff;
            if (bytes >= 12 &&
                Ath10k::parseHTTVersionReply(f, bytes, httEndpoint, major, minor, type) == nullptr) {
                httVersionReceived = true;
                httVersionMajor = major;
                httVersionMinor = minor;
                IOLog("Ath10kHal: HTT VERSION reply %u.%u\n", major, minor);
            }
        }
        IOLog("Ath10kHal: CE1 frame endpoint=%u bytes=%u flags=0x%02x type=%u（暂不解析）\n",
              f[0], bytes, f[1], bytes >= 9 ? f[8] : 0xffu);
    }
    IOLog("Ath10kHal: CE1 frame endpoint=%u bytes=%u credit-returned=%s credits=%u/%u outstanding=%u\n",
          f[0], bytes, applied ? "yes" : "no",
          wmiCredits.available, wmiCredits.total, wmiCredits.outstanding);

    if (!htcRuntimeRepostRx()) {
        wmiEventStatus = "ce1-repost-failed";
        return false;
    }
    return true;
}

bool Ath10kHal::
htcSubmitControlFrame(const UInt8 *request, UInt32 requestBytes,
                      UInt16 expectedMessageId,
                      UInt8 *response, UInt32 &responseBytes)
{
    static constexpr UInt32 kTxCE = Ath10k::kBMICENumToTarget;
    static constexpr UInt32 kRxCE = Ath10k::kBMICENumToHost;

    const UInt32 responseCapacity = responseBytes;
    responseBytes = 0;
    htcCtrlLastWaitMs = 0;

    if (!request || requestBytes < Ath10k::kHTCHeaderBytes ||
        requestBytes > Ath10k::kHTCControlMaxBytes ||
        !response || responseCapacity == 0 ||
        responseCapacity > Ath10k::kHTCControlMaxBytes ||
        !cePipesProgrammed || !htcReadyOk) {
        IOLog("Ath10kHal: HTC ctrl ERROR invalid arguments\n");
        return false;
    }

    if (!htcControlBuffer.allocated() &&
        !htcControlBuffer.allocate(Ath10k::kHTCControlMaxBytes,
                                   Ath10k::kCEDescriptorRingAlign, nullptr)) {
        IOLog("Ath10kHal: HTC ctrl ERROR buffer alloc\n");
        return false;
    }

    // 每次交换前确保 CE1 已挂描述符
    if (!htcEnsureRuntimeRx()) {
        htcServiceStatus = "runtime-rx-arm-failed";
        return false;
    }

    const UInt32 txEntries = Ath10k::kCEHostConfig[kTxCE].srcEntries;
    const UInt32 rxEntries = Ath10k::kCEHostConfig[kRxCE].dstEntries;

    Ath10k::CESnapshot txBefore{};
    Ath10k::CESnapshot rxBefore{};
    if (!Ath10k::readCESnapshot(regs, kTxCE, txBefore) ||
        !Ath10k::readCESnapshot(regs, kRxCE, rxBefore) ||
        !validateBMIPipeStructure(kTxCE, txBefore) ||
        !validateBMIPipeStructure(kRxCE, rxBefore) ||
        txBefore.srcWriteIndex >= txEntries || txBefore.currentSrcRead >= txEntries ||
        txBefore.srcWriteIndex != txBefore.currentSrcRead ||
        rxBefore.dstWriteIndex != runtimeRxNext) {
        IOLog("Ath10kHal: HTC ctrl ERROR ring state invalid\n");
        return false;
    }

    auto *txRing = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceSrcRings[kTxCE].virtualAddress());
    auto *rxRing = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kRxCE].virtualAddress());

    if (!ceDstRings[kRxCE].syncFromDevice() ||
        rxRing[runtimeRxSlot].nbytes != 0 ||
        rxRing[runtimeRxSlot].address != static_cast<UInt32>(htcCtrlRxBuffer.dmaAddress())) {
        IOLog("Ath10kHal: HTC ctrl ERROR RX descriptor not clean\n");
        return false;
    }

    bzero(htcControlBuffer.virtualAddress(),
          static_cast<size_t>(htcControlBuffer.length()));
    bcopy(request, htcControlBuffer.virtualAddress(), requestBytes);
    // endpoint 0 的序号从 0 开始，每次交换递增
    reinterpret_cast<UInt8 *>(htcControlBuffer.virtualAddress())[5] =
        static_cast<UInt8>(htcCtrlTxCount);
    if (!htcControlBuffer.syncForDevice())
        return false;

    const UInt32 txSlot = txBefore.srcWriteIndex;
    const UInt32 txNext = Ath10k::ceRingNext(txSlot, txEntries);
    auto *txDesc = &txRing[txSlot];
    bzero(txDesc, sizeof(*txDesc));
    txDesc->address = static_cast<UInt32>(htcControlBuffer.dmaAddress());
    txDesc->nbytes  = static_cast<UInt16>(requestBytes);
    txDesc->flags   = 0;   // HTC 的 transfer_id 是 endpoint 0，不是 BMI 匿名元数据
    OSSynchronizeIO();

    if (!ceSrcRings[kTxCE].syncForDevice())
        return false;

    const UInt32 txBase = Ath10k::ceBaseAddress(kTxCE);
    const UInt32 rxBase = Ath10k::ceBaseAddress(kRxCE);

    UInt32 txWrite = 0xffffffffu;
    if (!regs.write32(txBase + Ath10k::kCESrcWriteIndex, txNext) ||
        !regs.read32(txBase + Ath10k::kCESrcWriteIndex, txWrite) || txWrite != txNext) {
        htcServiceStatus = "control-submit-failed";
        return false;
    }

    UInt32 waitedMs = 0;
    UInt32 srcRead = 0xffffffffu;
    UInt32 dstRead = 0xffffffffu;
    UInt16 completedBytes = 0;
    bool txDone = false;
    bool rxDone = false;

    for (;;) {
        htcCtrlLastWaitMs = waitedMs;
        OSSynchronizeIO();

        if (!regs.read32(txBase + Ath10k::kCECurrentSrcRead, srcRead) ||
            !regs.read32(rxBase + Ath10k::kCECurrentDstRead, dstRead) ||
            srcRead >= txEntries || dstRead >= rxEntries) {
            htcServiceStatus = "control-mmio-failed";
            return false;
        }
        if (!ceDstRings[kRxCE].syncFromDevice()) {
            htcServiceStatus = "control-rx-sync-failed";
            return false;
        }

        txDone = ((srcRead & (txEntries - 1u)) == txNext);
        completedBytes = rxRing[runtimeRxSlot].nbytes;
        rxDone = (completedBytes != 0);

        if (txDone && rxDone)
            break;

        if (waitedMs >= Ath10k::kHTCServiceTimeoutMs) {
            htcServiceStatus = "control-timeout";
            IOLog("Ath10kHal: HTC ctrl TIMEOUT waited=%u ms tx=%u/%u rx=%u bytes=%u\n",
                  waitedMs, srcRead, txNext, dstRead, completedBytes);
            return false;
        }
        IOSleep(Ath10k::kHTCServicePollMs);
        waitedMs += Ath10k::kHTCServicePollMs;
    }

    ++htcCtrlTxCount;
    ++htcCtrlRxCount;
    runtimeRxArmed = false;   // 本槽位的包已被消费，下一次由 repost 推进

    if (!htcCtrlRxBuffer.syncFromDevice())
        return false;

    const auto *bytes = reinterpret_cast<const UInt8 *>(htcCtrlRxBuffer.virtualAddress());
    if (completedBytes > responseCapacity) {
        htcServiceStatus = "control-response-too-large";
        return false;
    }
    bcopy(bytes, response, completedBytes);
    responseBytes = completedBytes;

    // 校验响应消息号
    if (completedBytes < Ath10k::kHTCHeaderBytes + 2 ||
        Ath10k::htcReadLE16(bytes + Ath10k::kHTCHeaderBytes) != expectedMessageId) {
        htcServiceStatus = "control-unexpected-message";
        IOLog("Ath10kHal: HTC ctrl unexpected message id (wanted %u)\n", expectedMessageId);
        return false;
    }

    IOLog("Ath10kHal: HTC ctrl exchange ok tx=%u rx=%u waited=%u ms\n",
          requestBytes, completedBytes, htcCtrlLastWaitMs);
    return true;
}

bool Ath10kHal::
runHTCServiceSetup()
{
    htcServicesOk = false;
    htcServiceStatus = "not-run";
    wmiEndpoint = httEndpoint = 0xff;
    wmiMaxMessageSize = httMaxMessageSize = 0;
    htcCtrlTxCount = htcCtrlRxCount = 0;
    runtimeRxSlot = runtimeRxNext = 0xffffffffu;

    if (!htcReadyOk || !htcReady.maxEndpoints || !htcReady.credits ||
        htcReady.credits > 255 || !cePipesProgrammed) {
        htcServiceStatus = "prerequisite-mismatch";
        IOLog("Ath10kHal: HTC setup BLOCKED ready=%d endpoints=%u credits=%u\n",
              (int)htcReadyOk, htcReady.maxEndpoints, htcReady.credits);
        return false;
    }

    UInt8 frame[Ath10k::kHTCControlMaxBytes]{};
    UInt8 response[Ath10k::kHTCControlMaxBytes]{};

    // ---- 1) HTT 服务（关闭 credit flow）----
    htcServiceStatus = "htt-connect-failed";
    const UInt16 httService = static_cast<UInt16>(Ath10k::kSvcHTTData);
    UInt32 reqBytes = Ath10k::buildHTCConnectServiceFrame(
        frame, sizeof(frame), httService,
        static_cast<UInt16>(Ath10k::kHTCConnFlagDisableCreditFlow));
    UInt32 respBytes = sizeof(response);
    if (!reqBytes ||
        !htcSubmitControlFrame(frame, reqBytes, Ath10k::kHTCMessageConnectServiceResponse,
                               response, respBytes)) {
        IOLog("Ath10kHal: HTC service HTT connect FAILED\n");
        return false;
    }

    Ath10k::HTCServiceInfo httInfo{};
    const char *httError = Ath10k::parseHTCConnectServiceResponse(
        response, respBytes, httService, htcReady.maxEndpoints, httInfo);
    if (httError) {
        htcServiceStatus = httError;
        IOLog("Ath10kHal: HTC service HTT INVALID reason=%s status=%u endpoint=%u\n",
              httError, httInfo.status, httInfo.endpoint);
        return false;
    }
    httEndpoint = httInfo.endpoint;
    httMaxMessageSize = httInfo.maxMessageSize;
    IOLog("Ath10kHal: HTC service HTT READY endpoint=%u max=%u\n",
          httEndpoint, httMaxMessageSize);

    // ---- 2) WMI 服务（credits 放在 flags 高字节）----
    htcServiceStatus = "wmi-connect-failed";
    const UInt16 wmiService = static_cast<UInt16>(Ath10k::kSvcWMIControl);
    reqBytes = Ath10k::buildHTCConnectServiceFrame(
        frame, sizeof(frame), wmiService,
        static_cast<UInt16>(htcReady.credits << 8));
    respBytes = sizeof(response);
    if (!reqBytes ||
        !htcSubmitControlFrame(frame, reqBytes, Ath10k::kHTCMessageConnectServiceResponse,
                               response, respBytes)) {
        IOLog("Ath10kHal: HTC service WMI connect FAILED\n");
        return false;
    }

    Ath10k::HTCServiceInfo wmiInfo{};
    const char *wmiError = Ath10k::parseHTCConnectServiceResponse(
        response, respBytes, wmiService, htcReady.maxEndpoints, wmiInfo);
    if (wmiError) {
        htcServiceStatus = wmiError;
        IOLog("Ath10kHal: HTC service WMI INVALID reason=%s status=%u endpoint=%u\n",
              wmiError, wmiInfo.status, wmiInfo.endpoint);
        return false;
    }
    wmiEndpoint = wmiInfo.endpoint;
    wmiMaxMessageSize = wmiInfo.maxMessageSize;

    if (wmiEndpoint == httEndpoint) {
        htcServiceStatus = "duplicate-service-endpoint";
        IOLog("Ath10kHal: HTC service ERROR duplicate endpoint %u\n", wmiEndpoint);
        return false;
    }

    htcServicesOk = true;
    htcServiceStatus = "services-connected";
    IOLog("Ath10kHal: HTC SERVICES READY wmi-endpoint=%u max=%u htt-endpoint=%u max=%u tx=%u rx=%u\n",
          wmiEndpoint, wmiMaxMessageSize, httEndpoint, httMaxMessageSize,
          htcCtrlTxCount, htcCtrlRxCount);
    return true;
}

// ---------------------------------------------------------------------------
// M1-b1：WMI bootstrap（CE2 接收 + SETUP_COMPLETE + SERVICE_READY）
//
// Ath10kMac 的 Ath10kWMI.hpp 依赖 Ath10kConnection.hpp（整个自研 802.11 状态机），
// 我们不引入它，只按 Linux WMI TLV ABI 抽取 bootstrap 必需部分。
// ---------------------------------------------------------------------------
bool Ath10kHal::
armWmiRx()
{
    static constexpr UInt32 kRxCE = 2;   // WMI T2H

    wmiRxArmed = false;
    const UInt32 entries = Ath10k::kCEHostConfig[kRxCE].dstEntries;
    if (!cePipesProgrammed || entries == 0 || !ceDstRings[kRxCE].allocated())
        return false;

    if (!wmiRxBuffer.allocated() &&
        !wmiRxBuffer.allocate(Ath10k::kWMIRxBytes, Ath10k::kCEDescriptorRingAlign,
                              nullptr)) {
        IOLog("Ath10kHal: WMI RX alloc failed reason=%s\n", wmiRxBuffer.allocationFailure());
        return false;
    }

    Ath10k::CESnapshot snap{};
    if (!Ath10k::readCESnapshot(regs, kRxCE, snap) ||
        !validateBMIPipeStructure(kRxCE, snap) ||
        snap.dstWriteIndex >= entries || snap.currentDstRead >= entries ||
        snap.dstWriteIndex != snap.currentDstRead) {
        IOLog("Ath10kHal: WMI RX ERROR CE2 not idle w/r=%u/%u\n",
              snap.dstWriteIndex, snap.currentDstRead);
        return false;
    }

    bzero(wmiRxBuffer.virtualAddress(), static_cast<size_t>(wmiRxBuffer.length()));
    if (!wmiRxBuffer.syncForDevice())
        return false;

    wmiRxSlot = snap.dstWriteIndex;
    wmiRxNext = Ath10k::ceRingNext(wmiRxSlot, entries);

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kRxCE].virtualAddress());
    auto *desc = &ring[wmiRxSlot];
    bzero(desc, sizeof(*desc));
    desc->address = static_cast<UInt32>(wmiRxBuffer.dmaAddress());
    OSSynchronizeIO();

    if (!ceDstRings[kRxCE].syncForDevice())
        return false;

    const UInt32 doorbell = Ath10k::ceBaseAddress(kRxCE) + Ath10k::kCEDstWriteIndex;
    UInt32 readback = 0xffffffffu;
    if (!regs.write32(doorbell, wmiRxNext) ||
        !regs.read32(doorbell, readback) || readback != wmiRxNext) {
        IOLog("Ath10kHal: WMI RX ERROR doorbell readback=%u expected=%u\n",
              readback, wmiRxNext);
        return false;
    }
    IOLog("Ath10kHal: WMI RX ARMED CE2 slot=%u next=%u bytes=%u dma=0x%08x\n",
          wmiRxSlot, wmiRxNext, Ath10k::kWMIRxBytes,
          static_cast<UInt32>(wmiRxBuffer.dmaAddress()));
    wmiRxArmed = true;
    return true;
}

// 收到一包后把 CE2 描述符推进到下一个槽位。
// 与首次 arm 的区别：此时 dstWriteIndex 已经等于上一次 doorbell 值，
// 旧描述符带 nbytes，必须清掉再复用。
bool Ath10kHal::
wmiRepostRx()
{
    static constexpr UInt32 kRxCE = 2;

    const UInt32 entries = Ath10k::kCEHostConfig[kRxCE].dstEntries;
    if (!cePipesProgrammed || entries == 0 || !wmiRxBuffer.allocated() ||
        wmiRxSlot >= entries || wmiRxNext >= entries)
        return false;

    Ath10k::CESnapshot snap{};
    if (!Ath10k::readCESnapshot(regs, kRxCE, snap) ||
        !validateBMIPipeStructure(kRxCE, snap) ||
        snap.dstWriteIndex != wmiRxNext || snap.currentDstRead >= entries ||
        !ceDstRings[kRxCE].syncFromDevice()) {
        IOLog("Ath10kHal: WMI RX repost ERROR CE2 state w/r=%u/%u expected-next=%u\n",
              snap.dstWriteIndex, snap.currentDstRead, wmiRxNext);
        return false;
    }

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kRxCE].virtualAddress());
    auto *old = &ring[wmiRxSlot];
    if (!old->nbytes || old->address != static_cast<UInt32>(wmiRxBuffer.dmaAddress())) {
        IOLog("Ath10kHal: WMI RX repost ERROR old descriptor nbytes=%u addr=0x%08x\n",
              old->nbytes, old->address);
        return false;
    }

    const UInt32 slot = wmiRxNext;
    const UInt32 next = Ath10k::ceRingNext(slot, entries);
    bzero(old, sizeof(*old));
    auto *desc = &ring[slot];
    bzero(desc, sizeof(*desc));
    desc->address = static_cast<UInt32>(wmiRxBuffer.dmaAddress());
    bzero(wmiRxBuffer.virtualAddress(), static_cast<size_t>(wmiRxBuffer.length()));
    OSSynchronizeIO();

    if (!wmiRxBuffer.syncForDevice() || !ceDstRings[kRxCE].syncForDevice())
        return false;

    const UInt32 doorbell = Ath10k::ceBaseAddress(kRxCE) + Ath10k::kCEDstWriteIndex;
    UInt32 readback = 0xffffffffu;
    if (!regs.write32(doorbell, next) ||
        !regs.read32(doorbell, readback) || readback != next) {
        IOLog("Ath10kHal: WMI RX repost ERROR doorbell readback=%u expected=%u\n",
              readback, next);
        return false;
    }
    wmiRxSlot = slot;
    wmiRxNext = next;
    wmiRxArmed = true;
    return true;
}

// 保证 CE2 已挂描述符：首次 arm，收到事件后 repost。
bool Ath10kHal::
wmiEnsureRxArmed()
{
    if (wmiRxArmed)
        return true;
    if (wmiRxSlot == 0xffffffffu || wmiRxNext == 0xffffffffu)
        return armWmiRx();
    return wmiRepostRx();
}

bool Ath10kHal::
htcSendControlFrame(const UInt8 *frame, UInt32 bytes)
{
    static constexpr UInt32 kTxCE = Ath10k::kBMICENumToTarget;

    if (!frame || bytes < Ath10k::kHTCHeaderBytes ||
        bytes > Ath10k::kHTCControlMaxBytes || !cePipesProgrammed)
        return false;

    if (!htcControlBuffer.allocated() &&
        !htcControlBuffer.allocate(Ath10k::kHTCControlMaxBytes,
                                   Ath10k::kCEDescriptorRingAlign, nullptr))
        return false;

    const UInt32 entries = Ath10k::kCEHostConfig[kTxCE].srcEntries;
    Ath10k::CESnapshot snap{};
    if (!Ath10k::readCESnapshot(regs, kTxCE, snap) ||
        !validateBMIPipeStructure(kTxCE, snap) ||
        snap.srcWriteIndex >= entries || snap.currentSrcRead >= entries ||
        snap.srcWriteIndex != snap.currentSrcRead)
        return false;

    bzero(htcControlBuffer.virtualAddress(),
          static_cast<size_t>(htcControlBuffer.length()));
    bcopy(frame, htcControlBuffer.virtualAddress(), bytes);
    reinterpret_cast<UInt8 *>(htcControlBuffer.virtualAddress())[5] =
        static_cast<UInt8>(htcCtrlTxCount);
    if (!htcControlBuffer.syncForDevice())
        return false;

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceSrcRings[kTxCE].virtualAddress());
    const UInt32 slot = snap.srcWriteIndex;
    const UInt32 next = Ath10k::ceRingNext(slot, entries);
    auto *desc = &ring[slot];
    bzero(desc, sizeof(*desc));
    desc->address = static_cast<UInt32>(htcControlBuffer.dmaAddress());
    desc->nbytes  = static_cast<UInt16>(bytes);
    desc->flags   = 0;
    OSSynchronizeIO();

    if (!ceSrcRings[kTxCE].syncForDevice())
        return false;

    const UInt32 base = Ath10k::ceBaseAddress(kTxCE);
    UInt32 write = 0xffffffffu;
    if (!regs.write32(base + Ath10k::kCESrcWriteIndex, next) ||
        !regs.read32(base + Ath10k::kCESrcWriteIndex, write) || write != next)
        return false;

    ++htcCtrlTxCount;

    // 只等 CE0 消费完，不等响应（事件会从 CE2 来）
    UInt32 waited = 0;
    for (;;) {
        OSSynchronizeIO();
        UInt32 srcRead = 0xffffffffu;
        if (!regs.read32(base + Ath10k::kCECurrentSrcRead, srcRead))
            return false;
        if ((srcRead & (entries - 1u)) == next)
            return true;
        if (waited >= Ath10k::kHTCServiceTimeoutMs) {
            IOLog("Ath10kHal: HTC TX-only TIMEOUT src=%u next=%u\n", srcRead, next);
            return false;
        }
        IOSleep(Ath10k::kHTCServicePollMs);
        waited += Ath10k::kHTCServicePollMs;
    }
}

// 读并发布 FW_INDICATOR。EVENT_PENDING 置位表示固件已崩溃/断言。
bool Ath10kHal::
sampleFirmwareIndicator()
{
    UInt32 value = 0xffffffffu;
    if (!regs.read32(Ath10k::kQca6174RegisterLayout.fwIndicatorAddress, value)) {
        wmiEventStatus = "fw-indicator-read-failed";
        return false;
    }
    fwIndicator = value;
    const bool pending = (value & Ath10k::kFwIndicatorEventPending) != 0;

    // 首次采样作为基线：若启动时该位就是 1，更可能是上一轮的陈旧值，
    // 只记录不中断；只有"启动后才翻转"才判定为固件崩溃。
    if (fwIndicatorBaseline == 0xffffffffu) {
        fwIndicatorBaseline = value;
        if (pending)
            IOLog("Ath10kHal: FW_INDICATOR 在基线中已带 EVENT_PENDING=0x%08x（继续观察）\n",
                  value);
        return true;
    }
    if (pending && !(fwIndicatorBaseline & Ath10k::kFwIndicatorEventPending)) {
        firmwareCrash = true;
        wmiEventStatus = "firmware-crash";
        IOLog("Ath10kHal: FIRMWARE CRASH indicated fw-indicator=0x%08x (baseline 0x%08x)\n",
              value, fwIndicatorBaseline);
        return false;
    }
    return true;
}

// 非阻塞推进一次 WMI 管线：
//   - CE1：先收信用回报（固件常在控制端点上归还 WMI 信用）
//   - CE2：有一个 WMI 事件就解析并合并状态
//   - 同时检查 FW_INDICATOR，固件崩溃立即失败
// 返回 false 表示不可恢复错误（wmiEventStatus / wmiParseError 已写入原因）。
bool Ath10kHal::
wmiPumpOnce(bool &gotEvent)
{
    static constexpr UInt32 kRxCE = 2;

    gotEvent = false;
    if (!sampleFirmwareIndicator())
        return false;

    // CE1 可能积压多包 trailer，最多取两包，避免饿死 CE2
    for (UInt32 i = 0; i < 2; ++i) {
        bool hasPacket = false;
        if (!htcRuntimePump(hasPacket))
            return false;
        if (!hasPacket)
            break;
    }

    if (!wmiEnsureRxArmed()) {
        wmiEventStatus = "wmi-rx-arm-failed";
        return false;
    }

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceDstRings[kRxCE].virtualAddress());
    auto *desc = &ring[wmiRxSlot];
    if (!ceDstRings[kRxCE].syncFromDevice()) {
        wmiEventStatus = "wmi-rx-sync-failed";
        return false;
    }
    const UInt32 bytes = desc->nbytes;
    if (!bytes)
        return true;   // 尚无事件

    if (desc->address != static_cast<UInt32>(wmiRxBuffer.dmaAddress()) ||
        bytes > wmiRxBuffer.length()) {
        wmiEventStatus = "wmi-descriptor-invalid";
        return false;
    }
    if (!wmiRxBuffer.syncFromDevice()) {
        wmiEventStatus = "wmi-rx-buffer-sync-failed";
        return false;
    }
    wmiRxArmed = false;
    wmiRxBytes = bytes;

    const auto *f = reinterpret_cast<const UInt8 *>(wmiRxBuffer.virtualAddress());
    const char *error = Ath10k::parseWMIEvent(f, bytes, wmiEndpoint, wmiCredits, wmiEvent);
    if (error) {
        wmiParseError = error;
        wmiEventStatus = "wmi-event-invalid";
        UInt32 id = bytes >= 12 ? (Ath10k::wmiReadLE32(f + 8) & 0x00ffffffu) : 0;
        IOLog("Ath10kHal: WMI event PARSE ERROR reason=%s bytes=%u id=%u\n",
              error, bytes, id);
        return false;
    }

    if (wmiEvent.trailerOnly) {
        IOLog("Ath10kHal: WMI credit-only frame bytes=%u credits=%u/%u outstanding=%u\n",
              bytes, wmiCredits.available, wmiCredits.total, wmiCredits.outstanding);
        return true;   // 纯信用回报，不算事件
    }

    // 扫描事件与管理帧由 parseWMIScanEvent 二次解析（parseWMIEvent 只做信封校验）
    if (wmiEvent.id == Ath10k::kWmiScanEventId || wmiEvent.id == Ath10k::kWmiMgmtRxEventId) {
        const char *scanError = Ath10k::parseWMIScanEvent(f, bytes, wmiEvent.id, scanInfo);
        if (scanError) {
            wmiParseError = scanError;
            wmiEventStatus = "scan-event-invalid";
            IOLog("Ath10kHal: SCAN event PARSE ERROR reason=%s bytes=%u id=%u\n",
                  scanError, bytes, wmiEvent.id);
            return false;
        }
        applyScanInfo();
    }

    gotEvent = true;
    wmiApplyEvent();
    return true;
}


// 把解析结果合并进 HAL 状态（跨事件累积，不在每条事件上清零）。
void Ath10kHal::
wmiApplyEvent()
{
    const Ath10k::WMIEventInfo &e = wmiEvent;

    wmiEventId = e.id;
    wmiEvtPrefixBytes = e.prefixBytes < sizeof(wmiEvtPrefix) ? e.prefixBytes
                                                             : static_cast<UInt32>(sizeof(wmiEvtPrefix));
    bcopy(e.prefix, wmiEvtPrefix, wmiEvtPrefixBytes);

    if (e.serviceAvailable) {
        wmiServiceAvailable = true;
        wmiServiceBits = e.availableBits;
        for (UInt32 i = 0; i < 4; ++i)
            wmiServiceBitmap[i] = e.availableBitmap[i];
        IOLog("Ath10kHal: WMI SERVICE_AVAILABLE bits=%u bitmap=%08x %08x %08x %08x\n",
              e.availableBits, e.availableBitmap[0], e.availableBitmap[1],
              e.availableBitmap[2], e.availableBitmap[3]);
    }

    if (e.serviceReady) {
        wmiServiceReady = true;
        wmiBoot = e;                       // INIT 的输入快照
        wmiFwBuild = e.fwBuild;
        wmiAbiPresent = e.abiPresent;
        for (UInt32 i = 0; i < 6; ++i)
            wmiAbi[i] = e.abi[i];
        wmiRfChains = e.rfChains;
        wmiMemoryRequests = e.memoryRequests;
        wmiServiceBitmapPresent = e.serviceBitmapPresent;
        for (UInt32 i = 0; i < 32; ++i)
            wmiServiceBitmapFull[i] = e.serviceBitmap[i];
        wmiRegCapsPresent = e.regCapsPresent;
        for (UInt32 i = 0; i < 9; ++i)
            wmiRegCaps[i] = e.regCaps[i];
        IOLog("Ath10kHal: WMI SERVICE_READY build=0x%08x abi=0x%08x chains=%u mem-reqs=%u reg-caps=%s bytes=%u\n",
              e.fwBuild, e.abi[0], e.rfChains, e.memoryRequests,
              e.regCapsPresent ? "yes" : "no", e.bytes);
    }

    if (e.ready) {
        wmiReadyEventReceived = true;
        wmiReadyStatus = e.readyStatus;
        if (e.macValid) {
            for (UInt32 i = 0; i < 6; ++i)
                targetMac[i] = e.mac[i];
            targetMacValid = true;
        }
        IOLog("Ath10kHal: WMI READY status=%u mac=%02x:%02x:%02x:%02x:%02x:%02x valid=%s\n",
              e.readyStatus, e.mac[0], e.mac[1], e.mac[2], e.mac[3], e.mac[4], e.mac[5],
              e.macValid ? "yes" : "no");
    }

    if (e.informational) {
        IOLog("Ath10kHal: WMI skipping event id=%u bytes=%u credits-returned=%s\n",
              e.id, e.bytes, e.creditsReturned ? "yes" : "no");
    }
    if (e.creditsReturned) {
        IOLog("Ath10kHal: WMI credits total=%u available=%u outstanding=%u\n",
              wmiCredits.total, wmiCredits.available, wmiCredits.outstanding);
    }
}

bool Ath10kHal::
runWMIBootstrap()
{
    setupCompleteSent = false;
    wmiStatus = "not-run";
    wmiEventStatus = "not-run";
    wmiServiceAvailable = false;
    wmiServiceReady = false;
    targetMacValid = false;
    wmiReadyEventReceived = false;
    wmiReadyStatus = 0xffffffffu;
    wmiRxArmed = false;
    runtimeRxArmed = false;
    firmwareCrash = false;
    fwIndicator = 0xffffffffu;
    fwIndicatorBaseline = 0xffffffffu;
    wmiTxFail = "none";

    if (!htcServicesOk || wmiEndpoint == 0xff || !cePipesProgrammed) {
        wmiStatus = "prerequisite-mismatch";
        IOLog("Ath10kHal: WMI bootstrap BLOCKED services=%d endpoint=%u ce=%d\n",
              (int)htcServicesOk, wmiEndpoint, (int)cePipesProgrammed);
        return false;
    }

    // 信用账本必须在任何 WMI 事件到达前建立，否则 trailer 里的归还无处入账
    if (!wmiInitCredits()) {
        wmiStatus = "wmi-credits-invalid";
        IOLog("Ath10kHal: WMI credits INVALID credits=%u size=%u endpoint=%u\n",
              htcReady.credits, htcReady.creditSize, wmiEndpoint);
        return false;
    }

    if (!armWmiRx()) {
        wmiStatus = "wmi-rx-arm-failed";
        return false;
    }

    // SETUP_COMPLETE：无响应，作用是让固件开始发 WMI 事件
    UInt8 frame[Ath10k::kHTCControlMaxBytes]{};
    const UInt32 bytes = Ath10k::buildHTCSetupCompleteExFrame(frame, sizeof(frame),
                                                              htcReady.maxBundleMessages);
    if (!bytes) {
        wmiStatus = "setup-complete-build-failed";
        return false;
    }
    if (!htcSendControlFrame(frame, bytes)) {
        wmiStatus = "setup-complete-send-failed";
        IOLog("Ath10kHal: SETUP_COMPLETE send FAILED\n");
        return false;
    }
    setupCompleteSent = true;
    IOLog("Ath10kHal: SETUP_COMPLETE sent (%u bytes); waiting for WMI events\n", bytes);

    // 固件先发 SERVICE_AVAILABLE(3)，紧接着发 SERVICE_READY(1)。
    // 中间也可能夹着诊断类事件，全部按结构校验后跳过。
    // 每收一包都要重新挂 CE2 描述符，否则下一包无处落地。
    UInt32 elapsedMs = 0;
    UInt32 eventCount = 0;
    while (elapsedMs < Ath10k::kWMIServiceReadyTimeoutMs) {
        bool gotEvent = false;
        if (!wmiPumpOnce(gotEvent)) {
            wmiStatus = wmiServiceAvailable ? "wmi-service-ready-event-missing"
                                            : ((wmiEventStatus && wmiEventStatus[0]) ? wmiEventStatus
                                                                                    : "wmi-event-failed");
            IOLog("Ath10kHal: WMI bootstrap pump FAILED status=%s event-status=%s elapsed=%u ms last-id=%u\n",
                  wmiStatus, wmiEventStatus ? wmiEventStatus : "?", elapsedMs, wmiEventId);
            return false;
        }
        if (wmiServiceReady)
            break;
        if (gotEvent) {
            if (++eventCount >= Ath10k::kWMIServiceReadyEventMax) {
                wmiStatus = "wmi-event-limit";
                break;
            }
        } else {
            IOSleep(Ath10k::kWMIEventPollMs);
            elapsedMs += Ath10k::kWMIEventPollMs;
        }
    }
    wmiWaitMs = elapsedMs;

    if (!wmiServiceReady) {
        wmiStatus = "unexpected-wmi-event";
        IOLog("Ath10kHal: WMI bootstrap never saw SERVICE_READY (last id=%u available=%d)\n",
              wmiEventId, (int)wmiServiceAvailable);
        return false;
    }

    wmiStatus = "service-ready";
    IOLog("Ath10kHal: WMI SERVICE_READY ok build=0x%08x abi=0x%08x chains=%u mem=%u bits=%u waited=%u ms\n",
          wmiFwBuild, wmiAbi[0], wmiRfChains, wmiMemoryRequests, wmiServiceBits, elapsedMs);
    return true;
}

// ---------------------------------------------------------------------------
// M1-b2：WMI INIT + host memory + vdev create
//
// 与 Linux ath10k 的 TLV 路径一致：
//   SERVICE_READY(1) 给出固件 ABI / RF 链数 / host memory 请求 →
//   CE3 发 WMI INIT(0x1) → CE2 等 READY(2)（带 target MAC）→
//   CE3 发 VDEV_CREATE(0x5001, STA)。
// WMI 命令不能走 CE0：CE0/CE1 是 HTC 控制端点 0 专用。
// ---------------------------------------------------------------------------
bool Ath10kHal::
wmiInitCredits()
{
    wmiCreditsReady = false;
    if (wmiEndpoint == 0 || wmiEndpoint == 0xff)
        return false;
    if (!wmiCredits.initialize(htcReady.credits, htcReady.creditSize, wmiEndpoint))
        return false;
    wmiCreditsReady = true;
    IOLog("Ath10kHal: WMI credits ready total=%u size=%u endpoint=%u\n",
          wmiCredits.total, wmiCredits.size, wmiCredits.endpoint);
    return true;
}

bool Ath10kHal::
wmiSubmitCommand(UInt32 bytes, UInt32 &waitedMs)
{
    static constexpr UInt32 kTxCE = 3;   // WMI H2T

    waitedMs = 0;
    wmiTxFail = "none";

    if (!cePipesProgrammed || !wmiCreditsReady || wmiEndpoint == 0xff ||
        bytes < Ath10k::kHTCHeaderBytes || bytes > Ath10k::kWMICommandBufferBytes ||
        !wmiTxBuffer.allocated() || !ceSrcRings[kTxCE].allocated()) {
        wmiTxFail = "prerequisite";
        return false;
    }

    // 上一条命令若已提交但完成状态未知，CE3 视为污染：禁止再发。
    // 注意：这个判断必须在清 wmiTxConsumed 之前，否则第二次提交会被自己拦住
    // （M1-b2 首轮真机就踩在这里：vdev 命令根本没发出去）。
    if (wmiTxOwned && !wmiTxConsumed) {
        wmiTxFail = "uncertain-previous";
        IOLog("Ath10kHal: WMI TX BLOCKED uncertain previous submit (slot=%u next=%u)\n",
              wmiTxSlot, wmiTxNext);
        return false;
    }

    const UInt32 entries = Ath10k::kCEHostConfig[kTxCE].srcEntries;
    Ath10k::CESnapshot snap{};
    if (!Ath10k::readCESnapshot(regs, kTxCE, snap)) {
        wmiTxFail = "ce3-snapshot";
        IOLog("Ath10kHal: WMI TX ERROR CE3 snapshot read failed\n");
        return false;
    }
    // 寄存器里的索引是**自由计数**（Linux 侧同样按 nentries 取模），
    // 而且固件在 WMI INIT / vdev 之后会自己改写 currentSrcRead。
    // M1-c3 真机就是卡在这里：扫描从 bring-up 内联改成 45 s 后异步执行，
    // CE3 的读索引早已不是我们上次写下的值 → 预检判"not idle"→ 提交失败。
    // 0xDEADBEEF / 0xFFFFFFFF = SOC 没上电时的哨兵值，不是真索引
    if (snap.srcWriteIndex == 0xdeadbeefu || snap.currentSrcRead == 0xdeadbeefu ||
        snap.ctrl1 == 0xdeadbeefu || snap.srcWriteIndex == 0xffffffffu) {
        wmiTxFail = "ce3-asleep";
        IOLog("Ath10kHal: WMI TX ERROR CE3 registers read as sentinel "
              "(w=0x%08x r=0x%08x ctrl1=0x%08x) -> target asleep\n",
              snap.srcWriteIndex, snap.currentSrcRead, snap.ctrl1);
        return false;
    }

    const UInt32 mask     = entries - 1u;
    const UInt32 writeIdx = snap.srcWriteIndex & mask;
    const UInt32 readIdx  = snap.currentSrcRead & mask;
    wmiTxSrcWrite    = snap.srcWriteIndex;
    wmiTxSrcRead     = snap.currentSrcRead;
    wmiTxSrcWriteRaw = snap.srcWriteIndex;
    wmiTxSrcReadRaw  = snap.currentSrcRead;

    // 结构校验只对"环基址/长度/字节序"这些会导致数据错乱的项硬失败；
    // dmax（突发长度）是性能参数，固件在 WMI INIT 之后可能自己改，只记录不拦。
    const auto &ce3cfg = Ath10k::kCEHostConfig[kTxCE];
    if (snap.srcBase != ceSrcRings[kTxCE].dmaAddress32() || snap.srcSize != entries) {
        wmiTxFail = "ce3-structure";
        IOLog("Ath10kHal: WMI TX ERROR CE3 ring mismatch base=0x%08x expect=0x%08x "
              "size=%u expect=%u\n",
              snap.srcBase, ceSrcRings[kTxCE].dmaAddress32(), snap.srcSize, entries);
        return false;
    }
    if ((snap.ctrl1 & Ath10k::kCECtrl1SrcByteSwapMask) != 0) {
        wmiTxFail = "ce3-byteswap";
        IOLog("Ath10kHal: WMI TX ERROR CE3 byte-swap enabled ctrl1=0x%08x\n", snap.ctrl1);
        return false;
    }
    if ((snap.ctrl1 & Ath10k::kCECtrl1DmaxMask) !=
        (ce3cfg.srcSizeMax & Ath10k::kCECtrl1DmaxMask)) {
        IOLog("Ath10kHal: CE3 dmax changed ctrl1=0x%08x (expected dmax=0x%08x), continue\n",
              snap.ctrl1, ce3cfg.srcSizeMax & Ath10k::kCECtrl1DmaxMask);
    }

    UInt32 slot;
    if (writeIdx != readIdx) {
        // 只有在我们自己的账本确认"没有在途命令"时才允许重同步：
        // 设备读索引就是环的权威尾部，把它当作新的写位置即可继续。
        if (wmiTxOwned && !wmiTxConsumed) {
            wmiTxFail = "ce3-not-idle";
            IOLog("Ath10kHal: WMI TX ERROR CE3 busy raw w/r=%u/%u masked=%u/%u entries=%u\n",
                  snap.srcWriteIndex, snap.currentSrcRead, writeIdx, readIdx, entries);
            return false;
        }
        slot = readIdx;
        ++ce3Resyncs;
        wmiTxFail = "ce3-resync";
        IOLog("Ath10kHal: CE3 RESYNC raw w/r=%u/%u masked=%u/%u -> slot=%u (resync #%u)\n",
              snap.srcWriteIndex, snap.currentSrcRead, writeIdx, readIdx,
              slot, ce3Resyncs);
    } else {
        slot = writeIdx;
    }

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceSrcRings[kTxCE].virtualAddress());
    const UInt32 next = Ath10k::ceRingNext(slot, entries);
    auto *desc = &ring[slot];
    bzero(desc, sizeof(*desc));
    desc->address = static_cast<UInt32>(wmiTxBuffer.dmaAddress());
    desc->nbytes  = static_cast<UInt16>(bytes);
    // QCA6174/QCA9377 的 CE 源描述符把 HTC 端点编码在 flags 的 bit2 起
    desc->flags   = static_cast<UInt16>(wmiEndpoint << 2);
    OSSynchronizeIO();

    if (!wmiTxBuffer.syncForDevice() || !ceSrcRings[kTxCE].syncForDevice()) {
        wmiTxFail = "tx-sync";
        return false;
    }

    if (!wmiCredits.reserve(bytes)) {
        wmiTxFail = "no-credit";
        IOLog("Ath10kHal: WMI TX NO CREDIT bytes=%u cost=%u available=%u outstanding=%u\n",
              bytes, wmiCredits.cost(bytes), wmiCredits.available, wmiCredits.outstanding);
        return false;
    }

    wmiTxSlot = slot;
    wmiTxNext = next;
    // 从这一刻起本次提交接管 CE3：完成状态未确认前禁止再发
    wmiTxConsumed = false;
    wmiTxOwned = true;   // 门铃可能已生效，此后任何失败都不允许重试

    const UInt32 base = Ath10k::ceBaseAddress(kTxCE);
    UInt32 readback = 0xffffffffu;
    if (!regs.write32(base + Ath10k::kCESrcWriteIndex, next) ||
        !regs.read32(base + Ath10k::kCESrcWriteIndex, readback) || readback != next) {
        wmiTxFail = "doorbell";
        IOLog("Ath10kHal: WMI TX doorbell FAILED readback=%u expected=%u\n",
              readback, next);
        return false;
    }
    wmiTxSrcWrite = next;   // 门铃已推进，读写指针应以本次 next 为基准
    IOLog("Ath10kHal: WMI TX submitted CE3 slot=%u next=%u bytes=%u flags=0x%04x credits=%u/%u\n",
          slot, next, bytes, (unsigned)(wmiEndpoint << 2),
          wmiCredits.available, wmiCredits.total);

    // 等 CE3 消费（只等发送完成，响应从 CE2 来）
    for (;;) {
        OSSynchronizeIO();
        UInt32 srcRead = 0xffffffffu;
        if (!regs.read32(base + Ath10k::kCECurrentSrcRead, srcRead)) {
            wmiTxFail = "src-read";
            return false;
        }
        wmiTxSrcRead = srcRead;
        if ((srcRead & (entries - 1u)) == next) {
            wmiTxConsumed = true;
            IOLog("Ath10kHal: WMI TX consumed waited=%u ms\n", waitedMs);
            return true;
        }
        if (waitedMs >= Ath10k::kWMIInitTimeoutMs) {
            wmiTxFail = "timeout";
            IOLog("Ath10kHal: WMI TX TIMEOUT waited=%u ms srcRead=%u expected=%u\n",
                  waitedMs, srcRead, next);
            return false;
        }
        IOSleep(Ath10k::kWMIEventPollMs);
        waitedMs += Ath10k::kWMIEventPollMs;
    }
}

bool Ath10kHal::
runWMIInit()
{
    wmiInitOk = false;
    wmiInitStatus = "not-run";
    wmiInitTxBytes = 0;
    wmiInitWaitMs = 0;
    wmiHostMemoryCount = 0;
    wmiHostMemoryBytes = 0;

    if (!wmiServiceReady || !wmiCreditsReady || !cePipesProgrammed) {
        wmiInitStatus = "wmi-init-prerequisite";
        IOLog("Ath10kHal: WMI INIT BLOCKED service-ready=%d credits=%d ce=%d\n",
              (int)wmiServiceReady, (int)wmiCreditsReady, (int)cePipesProgrammed);
        return false;
    }
    if (!wmiBoot.serviceReady) {
        wmiInitStatus = "wmi-service-ready-snapshot-missing";
        return false;
    }
    if (!Ath10k::wmiABICompatible(wmiBoot.abi)) {
        wmiInitStatus = "wmi-init-abi-mismatch";
        IOLog("Ath10kHal: WMI INIT ABI MISMATCH fw=%08x %08x %08x %08x %08x %08x\n",
              wmiBoot.abi[0], wmiBoot.abi[1], wmiBoot.abi[2],
              wmiBoot.abi[3], wmiBoot.abi[4], wmiBoot.abi[5]);
        return false;
    }
    if (wmiMaxMessageSize &&
        wmiMaxMessageSize < Ath10k::kWMIInitFrameBytes - Ath10k::kHTCHeaderBytes) {
        wmiInitStatus = "wmi-init-message-too-small";
        IOLog("Ath10kHal: WMI INIT frame too large for endpoint max=%u need=%u\n",
              wmiMaxMessageSize, Ath10k::kWMIInitFrameBytes - Ath10k::kHTCHeaderBytes);
        return false;
    }

    // ---- 1) host memory：按 SERVICE_READY 的请求逐块分配 ----
    Ath10k::WMIHostMemoryChunk chunks[Ath10k::kWMIHostMemoryMax]{};
    const UInt32 count = wmiBoot.hostMemoryCount;
    if (count > Ath10k::kWMIHostMemoryMax ||
        count != wmiBoot.memoryRequests) {
        wmiInitStatus = "wmi-host-memory-count";
        return false;
    }
    for (UInt32 i = 0; i < count; ++i) {
        const Ath10k::WMIHostMemoryRequest &request = wmiBoot.hostMemory[i];
        const UInt32 bytes = Ath10k::hostMemoryRequestBytes(request);
        if (!bytes) {
            wmiInitStatus = "wmi-host-memory-size";
            IOLog("Ath10kHal: WMI host memory request %u invalid unit=%u flags=%u units=%u\n",
                  i, request.unitBytes, request.flags, request.units);
            return false;
        }
        Ath10kDMABuffer &buffer = wmiHostMemory[i];
        if (buffer.allocated() && buffer.length() != bytes) {
            wmiInitStatus = "wmi-host-memory-mismatch";
            return false;
        }
        if (!buffer.allocated() && !buffer.allocate(bytes, 128, nullptr)) {
            wmiInitStatus = buffer.allocationFailure();
            IOLog("Ath10kHal: WMI host memory %u alloc %u bytes FAILED reason=%s\n",
                  i, bytes, buffer.allocationFailure());
            return false;
        }
        bzero(buffer.virtualAddress(), static_cast<size_t>(bytes));
        if (!buffer.syncForDevice()) {
            wmiInitStatus = "wmi-host-memory-sync";
            return false;
        }
        chunks[i].id      = request.id;
        chunks[i].address = static_cast<UInt32>(buffer.dmaAddress());
        chunks[i].bytes   = bytes;
        wmiHostMemoryBytes += bytes;
    }
    wmiHostMemoryCount = count;

    if (wmiHostMemoryBytes > Ath10k::kWMIHostMemoryTotal) {
        wmiInitStatus = "wmi-host-memory-total";
        return false;
    }

    // ---- 2) 构造 INIT ----
    if (!wmiTxBuffer.allocated() &&
        !wmiTxBuffer.allocate(Ath10k::kWMICommandBufferBytes,
                              Ath10k::kCEDescriptorRingAlign, nullptr)) {
        wmiInitStatus = wmiTxBuffer.allocationFailure();
        IOLog("Ath10kHal: WMI TX buffer alloc FAILED reason=%s\n",
              wmiTxBuffer.allocationFailure());
        return false;
    }

    UInt8 *frame = static_cast<UInt8 *>(wmiTxBuffer.virtualAddress());
    const char *error = Ath10k::buildWMIInit(
        frame, Ath10k::kWMICommandBufferBytes, wmiBoot, wmiEndpoint,
        1 /* QCA9377 单 RF 链 */, chunks, count, wmiSeq);
    if (error) {
        wmiInitStatus = error;
        IOLog("Ath10kHal: WMI INIT build FAILED reason=%s chunks=%u chains=%u\n",
              error, count, wmiRfChains);
        return false;
    }
    wmiInitTxBytes = Ath10k::kWMIInitFrameBytes + 20 * count;
    if (wmiMaxMessageSize &&
        wmiInitTxBytes - Ath10k::kHTCHeaderBytes > wmiMaxMessageSize) {
        wmiInitStatus = "wmi-init-message-too-small";
        IOLog("Ath10kHal: WMI INIT frame %u exceeds endpoint max=%u chunks=%u\n",
              wmiInitTxBytes - Ath10k::kHTCHeaderBytes, wmiMaxMessageSize, count);
        return false;
    }
    if (wmiCredits.cost(wmiInitTxBytes) > wmiCredits.available) {
        wmiInitStatus = "wmi-init-no-credit";
        IOLog("Ath10kHal: WMI INIT no credit cost=%u available=%u total=%u\n",
              wmiCredits.cost(wmiInitTxBytes), wmiCredits.available, wmiCredits.total);
        return false;
    }

    if (!wmiEnsureRxArmed()) {
        wmiInitStatus = "wmi-rx-arm-failed";
        return false;
    }

    // ---- 3) 提交 INIT 并等 READY(2) ----
    UInt32 waited = 0;
    if (!wmiSubmitCommand(wmiInitTxBytes, waited)) {
        wmiInitStatus = (wmiTxFail && wmiTxFail[0]) ? wmiTxFail : "wmi-init-submit-failed";
        IOLog("Ath10kHal: WMI INIT submit FAILED reason=%s consumed=%d\n",
              wmiInitStatus, (int)wmiTxConsumed);
        return false;
    }
    ++wmiSeq;
    wmiInitWaitMs = waited;

    UInt32 elapsed = 0;
    while (elapsed < Ath10k::kWMIInitTimeoutMs) {
        if (wmiReadyEventReceived &&
            (wmiCredits.outstanding == 0 || elapsed >= Ath10k::kWMIInitCreditWaitMs))
            break;
        bool gotEvent = false;
        if (!wmiPumpOnce(gotEvent)) {
            // READY 已到但之后出错（例如信用帧异常）时不把 INIT 判死
            if (!wmiReadyEventReceived) {
                wmiInitStatus = wmiEventStatus;
                IOLog("Ath10kHal: WMI INIT pump FAILED status=%s elapsed=%u ms\n",
                      wmiEventStatus ? wmiEventStatus : "?", elapsed);
                return false;
            }
            break;
        }
        if (wmiReadyEventReceived && wmiReadyStatus != 0) {
            wmiInitStatus = "wmi-ready-firmware-rejected";
            return false;
        }
        if (!gotEvent) {
            IOSleep(Ath10k::kWMIEventPollMs);
            ++elapsed;
        }
    }

    if (!wmiReadyEventReceived) {
        wmiInitStatus = elapsed >= Ath10k::kWMIInitTimeoutMs ? "wmi-ready-timeout"
                                                            : "wmi-ready-missing";
        IOLog("Ath10kHal: WMI INIT never saw READY waited=%u ms last-id=%u\n",
              elapsed, wmiEventId);
        return false;
    }
    if (wmiReadyStatus != 0) {
        wmiInitStatus = "wmi-ready-firmware-rejected";
        return false;
    }

    wmiInitOk = true;
    wmiInitWaitMs = elapsed;
    wmiInitStatus = (wmiCredits.outstanding == 0) ? "ready" : "ready-credit-pending";
    applyTargetMac();

    IOLog("Ath10kHal: WMI INIT PASS status=0 chunks=%u mem=%u bytes=%u mac=%02x:%02x:%02x:%02x:%02x:%02x valid=%s waited=%u ms\n",
          wmiHostMemoryCount, wmiHostMemoryBytes, wmiInitTxBytes,
          targetMac[0], targetMac[1], targetMac[2], targetMac[3], targetMac[4], targetMac[5],
          targetMacValid ? "yes" : "no", wmiInitWaitMs);
    return true;
}

void Ath10kHal::
applyTargetMac()
{
    if (!targetMacValid)
        return;

    IEEE80211_ADDR_COPY(ic.ic_myaddr, targetMac);

    struct _ifnet *ifp = &ic.ic_ac.ac_if;
    // 兼容层：更新 arpcom 里的以太网地址
    if_setlladdr(ifp, targetMac);

    // 接口已经发布时，把 BSD 侧 lladdr 也同步过去（等价于控制器
    // setHardwareAddress 里做的事），否则 ifconfig 仍会显示占位 MAC。
    if (attached && ifp->iface) {
        ifnet_t bsd = ifp->iface->getIfnet();
        if (bsd) {
            const errno_t error = ifnet_set_lladdr(bsd, targetMac, IEEE80211_ADDR_LEN);
            ifp->iface->setProperty(kIOMACAddress, targetMac, IEEE80211_ADDR_LEN);
            IOLog("Ath10kHal: ifnet_set_lladdr %s (err=%d)\n",
                  error == 0 ? "ok" : "failed", (int)error);
        }
    }

    IOLog("Ath10kHal: net80211 MAC set to %02x:%02x:%02x:%02x:%02x:%02x\n",
          targetMac[0], targetMac[1], targetMac[2], targetMac[3], targetMac[4], targetMac[5]);
}

bool Ath10kHal::
runWMIVdevCreate()
{
    wmiVdevOk = false;
    wmiVdevTxConsumed = false;
    wmiVdevStatus = "not-run";
    wmiVdevTxBytes = 0;
    wmiVdevWaitMs = 0;

    if (!wmiInitOk || !wmiTxBuffer.allocated()) {
        wmiVdevStatus = "vdev-create-prerequisite";
        IOLog("Ath10kHal: VDEV CREATE BLOCKED init=%d tx-buffer=%d\n",
              (int)wmiInitOk, (int)wmiTxBuffer.allocated());
        return false;
    }

    // 优先用固件上报的 MAC；无效时退回本地管理地址（避免组播地址）
    UInt8 mac[6]{};
    if (targetMacValid) {
        for (UInt32 i = 0; i < 6; ++i)
            mac[i] = targetMac[i];
    } else {
        for (UInt32 i = 0; i < 6; ++i)
            mac[i] = placeholderMac[i];
    }

    UInt8 *frame = static_cast<UInt8 *>(wmiTxBuffer.virtualAddress());
    const char *error = Ath10k::buildWMIVdevCreate(
        frame, Ath10k::kWMICommandBufferBytes, wmiEndpoint, mac, wmiSeq);
    if (error) {
        wmiVdevStatus = error;
        IOLog("Ath10kHal: VDEV CREATE build FAILED reason=%s mac=%02x:%02x:%02x:%02x:%02x:%02x\n",
              error, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return false;
    }
    wmiVdevTxBytes = Ath10k::kWMIVdevCreateFrameBytes;
    if (wmiCredits.cost(wmiVdevTxBytes) > wmiCredits.available) {
        wmiVdevStatus = "vdev-no-credit";
        IOLog("Ath10kHal: VDEV CREATE no credit cost=%u available=%u total=%u\n",
              wmiCredits.cost(wmiVdevTxBytes), wmiCredits.available, wmiCredits.total);
        return false;
    }

    if (!wmiEnsureRxArmed()) {
        wmiVdevStatus = "vdev-rx-arm-failed";
        return false;
    }

    UInt32 waited = 0;
    if (!wmiSubmitCommand(wmiVdevTxBytes, waited)) {
        wmiVdevStatus = (wmiTxFail && wmiTxFail[0]) ? wmiTxFail : "vdev-submit-failed";
        IOLog("Ath10kHal: VDEV CREATE submit FAILED reason=%s consumed=%d ce3-w/r=%u/%u\n",
              wmiVdevStatus, (int)wmiTxConsumed, wmiTxSrcWrite, wmiTxSrcRead);
        return false;
    }
    ++wmiSeq;
    wmiVdevTxConsumed = true;
    wmiVdevWaitMs = waited;

    // vdev create 在本固件上没有语义响应，只有传输完成；
    // 顺手把可能到达的事件收掉以归还信用，超时不算失败。
    UInt32 elapsed = 0;
    while (elapsed < Ath10k::kWMIVdevTimeoutMs && wmiCredits.outstanding != 0) {
        bool gotEvent = false;
        if (!wmiPumpOnce(gotEvent))
            break;
        if (!gotEvent) {
            IOSleep(Ath10k::kWMIEventPollMs);
            ++elapsed;
        }
    }
    wmiVdevWaitMs += elapsed;

    wmiVdevOk = true;
    wmiVdevStatus = (wmiCredits.outstanding == 0) ? "transport-complete"
                                                  : "transport-complete-credit-pending";
    IOLog("Ath10kHal: VDEV CREATE PASS id=0 type=STA mac=%02x:%02x:%02x:%02x:%02x:%02x seq=%u waited=%u ms credits=%u/%u\n",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
          static_cast<UInt32>(wmiSeq - 1), wmiVdevWaitMs,
          wmiCredits.available, wmiCredits.total);
    return true;
}

// ---------------------------------------------------------------------------
// M1-c2：HTT RX 环
//
// 真机（23:11）确认：扫描能发现 23 个 BSS 的事件，但固件不交任何帧
// （scan-mgmt=0、ce1-frames=0）。原因是主机从未投递 HTT RX 环。
// 这里按参考实现在同一颗芯片上验证过的顺序移植：
//   VERSION_REQ(12B) → RX_RING_CFG(48B) → AGGR_CFG(11B)
// 内存：index ring(4B 写索引) + ring(2048×LE32，指向 pool) + pool(1023×2048B)。
// ---------------------------------------------------------------------------
bool Ath10kHal::
httSubmitCommand(UInt32 bytes, UInt32 timeoutMs)
{
    static constexpr UInt32 kTxCE = 4;   // HTT H2T

    httTxConsumed = false;
    const UInt32 entries = Ath10k::kCEHostConfig[kTxCE].srcEntries;
    if (!cePipesProgrammed || !httEndpoint || httEndpoint == 0xff ||
        !httTxBuffer.allocated() || !ceSrcRings[kTxCE].allocated() ||
        bytes < Ath10k::kHTCHeaderBytes || bytes > 64) {
        httSetupStatus = "htt-tx-prerequisite";
        return false;
    }

    Ath10k::CESnapshot snap{};
    if (!Ath10k::readCESnapshot(regs, kTxCE, snap) ||
        !validateBMIPipeStructure(kTxCE, snap) ||
        snap.srcWriteIndex >= entries || snap.currentSrcRead >= entries ||
        snap.srcWriteIndex != snap.currentSrcRead) {
        httSetupStatus = "htt-ce4-not-idle";
        IOLog("Ath10kHal: HTT TX CE4 not idle w/r=%u/%u\n",
              snap.srcWriteIndex, snap.currentSrcRead);
        return false;
    }

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceSrcRings[kTxCE].virtualAddress());
    const UInt32 slot = snap.srcWriteIndex;
    const UInt32 next = Ath10k::ceRingNext(slot, entries);
    auto *desc = &ring[slot];
    bzero(desc, sizeof(*desc));
    desc->address = static_cast<UInt32>(httTxBuffer.dmaAddress());
    desc->nbytes  = static_cast<UInt16>(bytes);
    desc->flags   = static_cast<UInt16>(httEndpoint << 2);
    OSSynchronizeIO();

    if (!httTxBuffer.syncForDevice() || !ceSrcRings[kTxCE].syncForDevice()) {
        httSetupStatus = "htt-tx-sync";
        return false;
    }

    httTxSlot = slot;
    httTxNext = next;
    const UInt32 base = Ath10k::ceBaseAddress(kTxCE);
    UInt32 back = 0xffffffffu;
    if (!regs.write32(base + Ath10k::kCESrcWriteIndex, next) ||
        !regs.read32(base + Ath10k::kCESrcWriteIndex, back) || back != next) {
        httSetupStatus = "htt-doorbell";
        IOLog("Ath10kHal: HTT TX doorbell FAILED readback=%u expected=%u\n", back, next);
        return false;
    }
    const auto *frameBytes = reinterpret_cast<const UInt8 *>(httTxBuffer.virtualAddress());
    IOLog("Ath10kHal: HTT TX submitted CE4 slot=%u next=%u bytes=%u type=%u\n",
          slot, next, bytes, bytes > 8 ? frameBytes[8] : 0xffu);

    UInt32 elapsed = 0;
    while (elapsed < timeoutMs) {
        bool gotEvent = false;
        if (!wmiPumpOnce(gotEvent)) {
            httSetupStatus = wmiEventStatus;
            return false;
        }
        UInt32 srcRead = 0xffffffffu;
        if (!regs.read32(base + Ath10k::kCECurrentSrcRead, srcRead)) {
            httSetupStatus = "htt-src-read";
            return false;
        }
        if ((srcRead & (entries - 1u)) == next) {
            httTxConsumed = true;
            return true;
        }
        IOSleep(Ath10k::kWMIEventPollMs);
        ++elapsed;
    }
    httSetupStatus = "htt-tx-timeout";
    IOLog("Ath10kHal: HTT TX TIMEOUT waited=%u ms\n", elapsed);
    return false;
}

bool Ath10kHal::
httWaitVersionReply(UInt32 timeoutMs)
{
    UInt32 elapsed = 0;
    while (elapsed < timeoutMs && !httVersionReceived) {
        bool gotEvent = false;
        if (!wmiPumpOnce(gotEvent))
            return false;
        if (!gotEvent) {
            IOSleep(Ath10k::kWMIEventPollMs);
            ++elapsed;
        }
    }
    return httVersionReceived;
}

bool Ath10kHal::
runHTTRxSetup()
{
    httSetupOk = false;
    httSetupStage = 0;
    httSetupStatus = "not-run";
    httVersionReceived = false;
    httVersionMajor = httVersionMinor = 0;
    httMessages = 0;
    httLastType = 0xffu;
    httLastPrefixBytes = 0;
    bzero(httLastPrefix, sizeof(httLastPrefix));
    httRxRingPosted = 0;
    httSeq = 0;
    httTxSlot = httTxNext = 0xffffffffu;
    httTxConsumed = false;

    if (!wmiInitOk || !htcServicesOk || httEndpoint == 0xff || !cePipesProgrammed) {
        httSetupStatus = "prerequisite";
        IOLog("Ath10kHal: HTT RX setup BLOCKED init=%d services=%d endpoint=%u\n",
              (int)wmiInitOk, (int)htcServicesOk, httEndpoint);
        return false;
    }

    const UInt64 poolBytes = static_cast<UInt64>(Ath10k::kHTTRxPosted) *
                             Ath10k::kHTTRxBufferBytes;
    if (!httTxBuffer.allocated() &&
        !httTxBuffer.allocate(64, Ath10k::kCEDescriptorRingAlign, nullptr)) {
        httSetupStatus = "htt-tx-allocation";
        return false;
    }
    if (!httRxIndex.allocated() &&
        !httRxIndex.allocate(8, Ath10k::kCEDescriptorRingAlign, nullptr)) {
        httSetupStatus = "htt-index-allocation";
        return false;
    }
    if (!httRxRing.allocated() &&
        !httRxRing.allocate(Ath10k::kHTTRingEntries * 4,
                            Ath10k::kCEDescriptorRingAlign, nullptr)) {
        httSetupStatus = "htt-ring-allocation";
        return false;
    }
    if (!httRxPool.allocated() &&
        !httRxPool.allocate(poolBytes, 128, nullptr, Ath10kDMABuffer::Role::Payload)) {
        httSetupStatus = httRxPool.allocationFailure();
        IOLog("Ath10kHal: HTT RX pool %llu bytes alloc FAILED reason=%s\n",
              poolBytes, httRxPool.allocationFailure());
        return false;
    }
    if (!Ath10k::dmaSpan32(httRxPool.dmaAddress(), poolBytes) ||
        !Ath10k::dmaSpan32(httRxRing.dmaAddress(), httRxRing.length()) ||
        !Ath10k::dmaSpan32(httRxIndex.dmaAddress(), httRxIndex.length())) {
        httSetupStatus = "htt-rx-address";
        return false;
    }

    // 环与索引：ring[i] 指向 pool 里第 i 个 buffer；索引初值 = 已投递数
    auto *ring = static_cast<UInt8 *>(httRxRing.virtualAddress());
    auto *pool = static_cast<UInt8 *>(httRxPool.virtualAddress());
    bzero(ring, static_cast<size_t>(httRxRing.length()));
    bzero(pool, static_cast<size_t>(poolBytes));
    for (UInt32 i = 0; i < Ath10k::kHTTRxPosted; ++i)
        Ath10k::htcWriteLE32(ring + 4 * i,
                     static_cast<UInt32>(httRxPool.dmaAddress() +
                                         static_cast<UInt64>(i) * Ath10k::kHTTRxBufferBytes));
    Ath10k::htcWriteLE32(static_cast<UInt8 *>(httRxIndex.virtualAddress()), Ath10k::kHTTRxPosted);
    if (!httRxPool.syncForDevice() || !httRxRing.syncForDevice() ||
        !httRxIndex.syncForDevice()) {
        httSetupStatus = "htt-rx-sync";
        return false;
    }
    httRxRingPosted = Ath10k::kHTTRxPosted;

    UInt8 *frame = static_cast<UInt8 *>(httTxBuffer.virtualAddress());

    // 1) VERSION_REQ
    UInt32 bytes = Ath10k::buildHTTVersionReq(frame, 64, httEndpoint, httSeq++);
    if (!bytes || !httSubmitCommand(bytes, 800)) {
        httSetupStatus = httSetupStatus ? httSetupStatus : "htt-version-submit";
        return false;
    }
    httSetupStage = 1;
    if (!httWaitVersionReply(700))
        IOLog("Ath10kHal: HTT version reply missing; continuing to RX ring cfg\n");
    else
        IOLog("Ath10kHal: HTT version %u.%u\n", httVersionMajor, httVersionMinor);

    // 2) RX_RING_CFG
    bytes = Ath10k::buildHTTRxRingCfg(frame, 64, httEndpoint, httSeq++,
                                      httRxIndex.dmaAddress(), httRxRing.dmaAddress());
    if (!bytes) {
        httSetupStatus = "htt-ringcfg-build";
        return false;
    }
    if (!httSubmitCommand(bytes, 1200)) {
        httSetupStatus = "htt-ringcfg-submit";
        return false;
    }
    httSetupStage = 2;

    // 3) AGGR_CFG
    bytes = Ath10k::buildHTTAggrCfg(frame, 64, httEndpoint, httSeq++);
    if (!bytes || !httSubmitCommand(bytes, 1200)) {
        httSetupStatus = !bytes ? "htt-aggr-build" : "htt-aggr-submit";
        return false;
    }
    httSetupStage = 3;

    httSetupOk = true;
    httSetupStatus = "rx-ring-posted";
    IOLog("Ath10kHal: HTT RX READY posted=%u pool=%llu bytes version=%u.%u msgs=%u\n",
          httRxRingPosted, poolBytes, httVersionMajor, httVersionMinor, httMessages);
    return true;
}

// ---------------------------------------------------------------------------
// M1-c1：扫描命令通路（PDEV 参数 → SCAN_CHAN_LIST → START_SCAN → 收集结果）
//
// 先只把 WMI 层打通并发布到 ioreg（SSID 列表），net80211 的扫描结果对接
// 放到 M1-c2。扫描用被动模式（PASSIVE），首轮跳过 DFS 信道。
// ---------------------------------------------------------------------------
bool Ath10kHal::
wmiSubmitAndReclaim(UInt32 bytes, UInt32 reclaimTimeoutMs)
{
    if (!wmiEnsureRxArmed()) {
        wmiTxFail = "rx-arm";
        return false;
    }

    UInt32 waited = 0;
    if (!wmiSubmitCommand(bytes, waited))
        return false;
    ++wmiSeq;

    UInt32 elapsed = 0;
    while (elapsed < reclaimTimeoutMs && wmiCredits.outstanding != 0) {
        bool gotEvent = false;
        if (!wmiPumpOnce(gotEvent))
            return false;
        if (!gotEvent) {
            IOSleep(Ath10k::kWMIEventPollMs);
            ++elapsed;
        }
    }
    return true;
}

bool Ath10kHal::
runWMIPdevSetup()
{
    pdevSetupOk = false;
    pdevSetupStatus = "not-run";

    if (!wmiInitOk || !wmiTxBuffer.allocated()) {
        pdevSetupStatus = "prerequisite";
        return false;
    }

    UInt8 *frame = static_cast<UInt8 *>(wmiTxBuffer.virtualAddress());
    for (UInt32 param = 1; param <= 2; ++param) {
        if (!Ath10k::buildWMIPdevSetParam(frame, Ath10k::kWMICommandBufferBytes,
                                          wmiEndpoint, param, 1, wmiSeq)) {
            pdevSetupStatus = "build-failed";
            return false;
        }
        if (!wmiSubmitAndReclaim(Ath10k::kWmiPdevSetParamFrameBytes, 2000)) {
            pdevSetupStatus = wmiTxFail;
            IOLog("Ath10kHal: PDEV_SET_PARAM param=%u FAILED reason=%s\n",
                  param, pdevSetupStatus ? pdevSetupStatus : "?");
            return false;
        }
        IOLog("Ath10kHal: PDEV_SET_PARAM param=%u value=1 ok credits=%u/%u\n",
              param, wmiCredits.available, wmiCredits.total);
    }

    pdevSetupOk = true;
    pdevSetupStatus = "chains-1x1";
    return true;
}

bool Ath10kHal::
runWMIScanSetup()
{
    scanSetupOk = false;
    scanStatus = "not-run";

    if (!pdevSetupOk || !wmiVdevOk || !wmiTxBuffer.allocated()) {
        scanStatus = "prerequisite";
        IOLog("Ath10kHal: SCAN setup BLOCKED pdev=%d vdev=%d buffer=%d\n",
              (int)pdevSetupOk, (int)wmiVdevOk, (int)wmiTxBuffer.allocated());
        return false;
    }

    UInt8 *frame = static_cast<UInt8 *>(wmiTxBuffer.virtualAddress());
    if (!Ath10k::buildWMIScanChanList(frame, Ath10k::kWMICommandBufferBytes,
                                      wmiEndpoint, wmiSeq)) {
        scanStatus = "build-failed";
        return false;
    }
    const UInt32 bytes = 24 + 28 * Ath10k::kWmiScanChannelCount;
    if (!wmiSubmitAndReclaim(bytes, 2000)) {
        scanStatus = wmiTxFail;
        IOLog("Ath10kHal: SCAN_CHAN_LIST FAILED reason=%s\n", scanStatus ? scanStatus : "?");
        return false;
    }

    scanSetupOk = true;
    scanStatus = "channel-list";
    IOLog("Ath10kHal: SCAN_CHAN_LIST ok channels=%u bytes=%u credits=%u/%u\n",
          Ath10k::kWmiScanChannelCount, bytes, wmiCredits.available, wmiCredits.total);
    return true;
}

bool Ath10kHal::
runWMIScan()
{
    scanOk = false;
    scanStarted = false;
    scanCompleted = false;
    scanEvents = 0;
    scanMgmtFrames = 0;
    scanBssCount = 0;
    scanReason = 0xffffffffu;
    scanLastFreq = 0;
    scanWaitMs = 0;
    scanEvStarted = scanEvCompleted = scanEvBssChan = 0;
    scanEvForeignChan = scanEvOther = 0;
    scanStatus = "not-run";

    if (!scanSetupOk) {
        scanStatus = "prerequisite";
        return false;
    }

    UInt8 *frame = static_cast<UInt8 *>(wmiTxBuffer.virtualAddress());
    const UInt32 bytes = Ath10k::wmiStartScanFrameBytes();
    // 主动扫描：非 DFS 信道上发 probe request，比被动扫描更容易拿到帧；
    // 若固件确实不把扫描结果交给 WMI，这里仍会是 scan-mgmt=0。
    if (!Ath10k::buildWMIStartScan(frame, Ath10k::kWMICommandBufferBytes,
                                   wmiEndpoint, wmiSeq, false, true)) {
        scanStatus = "build-failed";
        return false;
    }
    if (!wmiEnsureRxArmed()) {
        scanStatus = "rx-arm-failed";
        return false;
    }

    UInt32 waited = 0;
    bool submitted = wmiSubmitCommand(bytes, waited);
    if (!submitted && ath10kStrEq(wmiTxFail, "ce3-asleep") &&
        ensureTargetAwake("scan-retry")) {
        // 只有"读到哨兵值"这一种失败可以重试：提交根本没发生，CE3 未被污染
        IOLog("Ath10kHal: START_SCAN retry after re-wake\n");
        submitted = wmiSubmitCommand(bytes, waited);
    }
    if (!submitted) {
        scanStatus = wmiTxFail;
        IOLog("Ath10kHal: START_SCAN submit FAILED reason=%s ce3-w/r=%u/%u\n",
              scanStatus ? scanStatus : "?", wmiTxSrcWrite, wmiTxSrcRead);
        return false;
    }
    ++wmiSeq;
    IOLog("Ath10kHal: WMI START_SCAN submitted bytes=%u channels=%u\n",
          bytes, Ath10k::wmiStartScanChannelCount());

    bool pumpFailed = false;
    UInt32 elapsed = 0;
    UInt32 drain = 0;
    while (elapsed < Ath10k::kWmiScanCollectMs) {
        // detach 可能与本轮扫描并发：一旦开始拆除就立刻退出轮询，
        // 否则 DMA 缓冲被释放后我们还会继续读它。
        if (shuttingDown) {
            scanStatus = "aborted-detach";
            IOLog("Ath10kHal: SCAN aborted (detaching) waited=%u ms\n", elapsed);
            return false;
        }
        bool gotEvent = false;
        if (!wmiPumpOnce(gotEvent)) {
            pumpFailed = true;
            break;
        }
        if (scanCompleted) {
            if (!gotEvent) {
                IOSleep(Ath10k::kWMIEventPollMs);
                ++drain;
            }
            if (drain >= Ath10k::kWmiScanDrainMs)
                break;
            continue;
        }
        if (!gotEvent) {
            IOSleep(Ath10k::kWMIEventPollMs);
            ++elapsed;
        }
    }
    scanWaitMs = elapsed;

    if (pumpFailed) {
        scanStatus = wmiEventStatus;
        IOLog("Ath10kHal: SCAN pump FAILED status=%s events=%u mgmt=%u\n",
              wmiEventStatus ? wmiEventStatus : "?", scanEvents, scanMgmtFrames);
        return false;
    }
    if (!scanCompleted) {
        scanStatus = "timeout";
        IOLog("Ath10kHal: SCAN TIMEOUT waited=%u ms events=%u mgmt=%u started=%d\n",
              elapsed, scanEvents, scanMgmtFrames, (int)scanStarted);
        return false;
    }

    scanOk = (scanReason == 0);
    scanStatus = scanOk ? "complete" : "complete-with-reason";
    IOLog("Ath10kHal: SCAN %s reason=%u events=%u mgmt=%u bss=%u waited=%u ms\n",
          scanOk ? "PASS" : "FAILED", scanReason, scanEvents, scanMgmtFrames,
          scanBssCount, elapsed);
    return scanOk;
}

// ic_tree 里是否已有本次连接请求的目标 AP（按 des_bssid / des_essid 匹配）
bool Ath10kHal::
hasJoinTarget()
{
    if (!(ic.ic_wcl_join_requested || ic.ic_des_esslen != 0))
        return false;
    const bool wantBssid = (ic.ic_flags & IEEE80211_F_DESBSSID) != 0;
    for (struct ieee80211_node *ni = RB_MIN(ieee80211_tree, &ic.ic_tree);
         ni != nullptr; ni = RB_NEXT(ieee80211_tree, &ic.ic_tree, ni)) {
        if (wantBssid) {
            if (IEEE80211_ADDR_EQ(ni->ni_bssid, ic.ic_des_bssid))
                return true;
            continue;
        }
        if (ic.ic_des_esslen != 0 && ni->ni_esslen == (u_int)ic.ic_des_esslen &&
            memcmp(ni->ni_essid, ic.ic_des_essid, ic.ic_des_esslen) == 0)
            return true;
    }
    return false;
}

void Ath10kHal::
applyScanInfo()
{
    const Ath10k::WMIScanInfo &info = scanInfo;

    if (info.scanEvent) {
        ++scanEvents;
        scanLastFreq = info.scanFreq;
        if (info.scanType == 1) {
            scanStarted = true;
            ++scanEvStarted;
        } else if (info.scanType == 2) {
            scanCompleted = true;
            scanReason = info.scanReason;
            ++scanEvCompleted;
        } else if (info.scanType == 4) {
            ++scanEvBssChan;      // 固件在信道上发现了 BSS
        } else if (info.scanType == 8) {
            ++scanEvForeignChan;
        } else {
            ++scanEvOther;
        }
        // 明细只在第 1 轮打印：每轮几十行 IOLog 会明显拖慢系统（r1 降噪）
        if (scanRounds <= 1) {
            IOLog("Ath10kHal: SCAN event type=%u reason=%u freq=%u\n",
                  info.scanType, info.scanReason, info.scanFreq);
        }
    }

    if (info.mgmtRx) {
        ++scanMgmtFrames;
        // M1-c3：原始帧先暂存 —— 这是 ic_tree 建节点的唯一入口。
        // 必须在下面的 return 分支之前做，否则 SSID 相同/BSSID 非法的帧会被漏掉。
        if (info.mgmtFrame && info.mgmtFrameLength >= 24 && info.mgmtStatus == 0)
            stageScanFrame(info);
        if (!info.bssValid)
            return;
        // 同一 BSSID 只记一条，后到的更新 SNR/信道
        for (UInt32 i = 0; i < scanBssCount; ++i) {
            if (bcmp(scanBss[i].mac, info.bssMac, 6) != 0)
                continue;
            scanBss[i].freq = info.mgmtFreq;
            scanBss[i].snr = info.mgmtSnr;
            if (info.bssSsidLen && !scanBss[i].ssidLen) {
                scanBss[i].ssidLen = info.bssSsidLen;
                for (UInt32 k = 0; k < info.bssSsidLen && k < 32; ++k)
                    scanBss[i].ssid[k] = info.bssSsid[k];
            }
            return;
        }
        if (scanBssCount >= 16)
            return;
        ScanBssEntry &entry = scanBss[scanBssCount];
        for (UInt32 i = 0; i < 6; ++i)
            entry.mac[i] = info.bssMac[i];
        entry.ssidLen = info.bssSsidLen <= 32 ? info.bssSsidLen : 32;
        for (UInt32 i = 0; i < entry.ssidLen; ++i)
            entry.ssid[i] = info.bssSsid[i];
        entry.freq = info.mgmtFreq;
        entry.snr = info.mgmtSnr;
        ++scanBssCount;
        if (scanRounds <= 1) {
            IOLog("Ath10kHal: SCAN BSS ssid=%u.%s mac=%02x:%02x:%02x:%02x:%02x:%02x freq=%u snr=%u\n",
                  entry.ssidLen, entry.ssid, entry.mac[0], entry.mac[1], entry.mac[2],
                  entry.mac[3], entry.mac[4], entry.mac[5], entry.freq, entry.snr);
        }
    }
}

// ---------------------------------------------------------------------------
// M1-c3：把扫描结果交给 net80211（ic_tree 节点 + WCL 事件）
//
// 数据流：
//   WMI MGMT_RX（beacon / probe response 原始帧）
//     → stageScanFrame()：扫描线程里只做定长拷贝（缓冲随时会被下一个 CE2 包覆盖）
//     → runScanCompletion()：切到 workloop，feedStagedScanFrames()
//     → mbuf + ieee80211_find_rxnode + ieee80211_input
//     → net80211 的 ieee80211_recv_probe_resp 自行建/更新 ic_tree 节点，
//       并在 AIRPORT_WCL 下发出 IEEE80211_EVT_BEACON
//     → AirportWCL::publishWCLBeacon → 系统 Wi-Fi 列表
//
// 必须走 workloop：WCL 的 BEACON 回调里会直接 postMessage / 查 rfcomm 状态，
// 那些都属于 RX 上下文（iwx 也是在 workloop 的 task 里调 ieee80211_inputm）。
//
// 采用 docs/09 的路径 A（喂原始帧）而不是手工填节点：rate/RSN/HT/capinfo
// 全部由协议栈按帧内容填好，后面做连接时可以直接复用同一批节点。
// ---------------------------------------------------------------------------

// M1-c3 r1 的扫描节流参数（tick = 1 s，由 scanTimer 驱动）
// 开机后无条件扫描的轮数。**故意为 0**：r0 卡死后，先让新版本在"完全不扫描"
// 的状态下证明驱动空闲时是稳定的，再由系统扫描请求（打开 Wi-Fi 菜单）触发扫描。
// 这样一次重启就能把"驱动加载/空闲"和"扫描/喂帧"两件事分开定性。见 docs/13。
static constexpr UInt32 kScanInitialRounds    = 0;
static constexpr UInt32 kScanGapTicks         = 10;  // 成功一轮后到下一轮的最小间隔
static constexpr UInt32 kScanMaxRoundsPerReq  = 2;   // 同一个系统扫描请求最多触发几轮
static constexpr UInt32 kScanBackoffBaseTicks = 5;   // 失败退避起点
static constexpr UInt32 kScanBackoffMaxTicks  = 35;  // 失败退避上限
static constexpr UInt32 kScanRoundHangTicks   = 40;  // 单轮超过这么多秒视为卡死
// 开机静默期：这段时间内不响应任何扫描请求，给"驱动加载+空闲"留出可观察窗口。
// 目的是把"r0 卡死"定性成驱动装载问题还是扫描/喂帧问题（见 docs/13）。
static constexpr UInt32 kScanGraceTicks       = 60;

// 构建标记：真机验证时先看 ath10k-build，避免"到底跑的是哪一版"的扯皮
static constexpr const char *kAth10kBuildTag = "M1d7";

// M1-d 连接相关常量
static constexpr UInt32 kMgmtTxBufferBytes   = 2048;   // 单帧负载上限
static constexpr UInt32 kMgmtTxFragmentBytes = 16;     // [addr][len]
static constexpr UInt32 kMgmtTxCommandBytes  = 256;    // HTC + HTT TX 描述符
static constexpr UInt32 kMgmtTxCeTimeoutMs   = 100;    // CE4 消费等待
static constexpr UInt32 kMgmtTxCompTimeoutMs = 300;    // HTT 完成事件等待
static constexpr UInt32 kRxPumpIdleSleepUs   = 2000;   // 泵空闲时的休眠（2 ms）
static constexpr UInt32 kRxPumpMaxIdleLoops  = 30000;  // 最长约 60 秒

// net80211 / Airport 的 RSSI 刻度：0 = IWM_MIN_DBM(-100 dBm)，即 rssi = dBm + 100。
// 固件 MGMT_RX 给的是 SNR（相对底噪），底噪用 getBSSNoise()（-96 dBm）。
static int ath10kRssiFromSnr(UInt32 snr, int noiseDbm)
{
    int rssi = (noiseDbm + (int)snr) + 100;
    return rssi < 0 ? 0 : rssi;
}

// 只收 beacon(0x80) / probe response(0x50)：其它管理帧在 SCAN 状态下
// 也会被 net80211 丢掉，没必要占暂存区。
static bool ath10kIsBeaconOrProbeResp(const UInt8 *frame, UInt32 length)
{
    if (!frame || length < 24)
        return false;
    if ((frame[0] & 0x0c) != 0)          // type 必须是 management
        return false;
    const UInt8 subtype = frame[0] & 0xf0;
    return subtype == 0x80 || subtype == 0x50;
}

// M1-d：连接期间需要喂给 net80211 的管理帧
//   0xb0 auth / 0x10 assoc resp / 0x30 reassoc resp / 0xa0 disassoc
//   0xc0 deauth / 0xd0 action
static bool ath10kIsJoinMgmtFrame(const UInt8 *frame, UInt32 length)
{
    if (!frame || length < 24)
        return false;
    if ((frame[0] & 0x0c) != 0)
        return false;
    const UInt8 subtype = frame[0] & 0xf0;
    return subtype == 0xb0 || subtype == 0x10 || subtype == 0x30 ||
           subtype == 0xa0 || subtype == 0xc0 || subtype == 0xd0;
}

void Ath10kHal::
stageScanFrame(const Ath10k::WMIScanInfo &info)
{
    if (ic.ic_state <= IEEE80211_S_INIT) {
        ++scanFeedSkipped;
        scanFeedStatus = "skipped-init-state";
        return;
    }
    const bool scannable = ath10kIsBeaconOrProbeResp(info.mgmtFrame, info.mgmtFrameLength);
    const bool joinable = joinEnabled &&
        ath10kIsJoinMgmtFrame(info.mgmtFrame, info.mgmtFrameLength);
    if (!scannable && !joinable) {
        scanFeedStatus = "not-supported-mgmt";
        return;
    }
    if (info.mgmtFrameLength > kScanStageFrameBytes ||
        scanStageCount >= kScanStageMaxFrames) {
        // 超长帧（带一堆 VHT/HE IE）或本轮帧太多：丢掉但留计数，便于判读
        ++scanStageOverflow;
        scanFeedStatus = "stage-overflow";
        return;
    }

    ScanStageFrame &slot = scanStage[scanStageCount++];
    slot.length = (UInt16)info.mgmtFrameLength;
    slot.freq = info.mgmtFreq;
    slot.snr = info.mgmtSnr;
    bcopy(info.mgmtFrame, slot.data, info.mgmtFrameLength);
}

bool Ath10kHal::
feedScanFrameTo80211(const UInt8 *frame, UInt32 length, UInt32 freq, UInt32 snr)
{
    ++scanFeedAttempts;

    if (ic.ic_state <= IEEE80211_S_INIT) {
        ++scanFeedSkipped;
        scanFeedStatus = "skipped-init-state";
        return false;
    }
    const bool scannable = ath10kIsBeaconOrProbeResp(frame, length);
    const bool joinable = joinEnabled && ath10kIsJoinMgmtFrame(frame, length);
    if (!scannable && !joinable) {
        scanFeedStatus = "not-supported-mgmt";
        return false;
    }
    if (joinable && !scannable)
        ++joinFeedOk;

    unsigned int maxChunks = 1;
    mbuf_t m = nullptr;
    if (mbuf_allocpacket(MBUF_DONTWAIT, length, &maxChunks, &m) != 0 || m == nullptr) {
        scanFeedStatus = "mbuf-alloc-failed";
        return false;
    }
    mbuf_setlen(m, length);
    mbuf_pkthdr_setlen(m, length);
    void *dst = mtod(m, void *);
    if (!dst) {
        mbuf_freem(m);
        scanFeedStatus = "mbuf-mtod-failed";
        return false;
    }
    bcopy(frame, dst, length);

    const UInt32 channel = Ath10k::wmiFreqToChannel(freq);
    struct ieee80211_rxinfo rxi;
    bzero(&rxi, sizeof(rxi));
    // rxi_chan 参与 net80211 的信道一致性检查，必须给真实信道号
    rxi.rxi_chan = channel <= 255 ? (uint8_t)channel : 0;
    rxi.rxi_rssi = ath10kRssiFromSnr(snr, getBSSNoise());
    if (ic.ic_max_rssi && rxi.rxi_rssi > (int)ic.ic_max_rssi)
        rxi.rxi_rssi = ic.ic_max_rssi;

    struct ieee80211_frame *wh = mtod(m, struct ieee80211_frame *);
    struct ieee80211_node *ni = ieee80211_find_rxnode(&ic, wh);
    if (!ni) {
        mbuf_freem(m);
        scanFeedStatus = "find-rxnode-failed";
        return false;
    }

    // ieee80211_input 会接管并释放 mbuf；ni 的引用计数由我们还。
    // 对 beacon/probe response，net80211 内部会再 alloc/find 真正的 BSS 节点。
    ieee80211_input(&ic.ic_ac.ac_if, m, ni, &rxi);
    ieee80211_release_node(&ic, ni);

    ++scanFeedOk;
    scanFeedStatus = "fed";
    return true;
}

void Ath10kHal::
feedStagedScanFrames()
{
    // 重入保护：net80211 喂帧过程中可能回调 if_start 发管理帧（auth→assoc），
    // 那条路径会间接再次触发泵与暂存；这里不允许递归喂。
    if (feedingFrames)
        return;
    feedingFrames = true;
    for (UInt32 i = 0; i < scanStageCount; ++i) {
        ScanStageFrame &slot = scanStage[i];
        // 每喂一帧都可能追加新的暂存项，scanStageCount 会变，用 i 而不是缓存值
        if (i >= scanStageCount)
            break;
        feedScanFrameTo80211(slot.data, slot.length, slot.freq, slot.snr);
    }
    scanStageCount = 0;
    feedingFrames = false;
}

// 一轮扫描的收尾。注意：这里**故意不调用 ieee80211_end_scan()**。
// end_scan 的 notfound 分支会 next_scan → newstate(SCAN) → 立刻重排下一轮，
// 而 runWMIScan() 在 CE3 提交失败时会立即返回，于是形成毫秒级自旋：
// 每圈都做 free_allnodes + ~150 次 setProperty + 若干 IOLog + thread_call enter，
// 几十秒就能把整机拖死（r0 真机卡死的原因）。r1 只发 SCAN_DONE 事件。
// 1 Hz 节拍里检查连接请求：目标 AP 已进 ic_tree 就排一次 worker 执行 end_scan
// （不在 workloop 上做重活：WMI 提交与发帧都是阻塞的）
void Ath10kHal::
checkJoinTrigger()
{
    if (!joinEnabled || shuttingDown || !attached)
        return;
    if (joinTriggered || joinTriggerPending)
        return;
    if (ic.ic_state != IEEE80211_S_SCAN)
        return;
    if (!(ic.ic_wcl_join_requested || ic.ic_des_esslen != 0))
        return;
    if (scanTickCount < joinNextAllowedTick)
        return;                        // 失败重试也要 ≥10 s 间隔，避免重演 r0 自旋
    if (!hasJoinTarget())
        return;                        // 还没看到目标 AP，继续等扫描结果
    joinNextAllowedTick = scanTickCount + 10;
    joinTriggerPending = true;
    joinStatus = "join-trigger-queued";
    queueScanWork();
}

void Ath10kHal::
runScanCompletion()
{
    // 1) 清掉上一轮的邻居节点（保留 ic_bss）
    ieee80211_free_allnodes(&ic, 0);

    // 2) 兜底保证 ic_bss->ni_chan 合法（喂帧时用 rxi_chan，这里只防 nullptr）
    struct ieee80211_channel *chan = firstActiveScanChannel();
    if (ic.ic_bss && chan &&
        (ic.ic_bss->ni_chan == IEEE80211_CHAN_ANYC ||
         ic.ic_bss->ni_chan == nullptr ||
         ic.ic_bss->ni_chan->ic_flags == 0)) {
        ic.ic_bss->ni_chan = chan;
    }

    // 3) 喂帧：不持 workloop 锁（if_input 内部会自己取 _fCommandGate；
    //    与 iwx 的 RX task 上下文一致）
    feedStagedScanFrames();

    // 4) 收尾分两种：
    //   a) 有连接请求（用户选了网络）→ 交给 net80211 的 end_scan 去 choose_bss，
    //      成功后 node_join_bss → 进 AUTH，由连接命令链接住；
    //   b) 没有连接请求 → 只发 SCAN_DONE，**不调用 end_scan**，
    //      避免 r0 那种"失败→立刻重扫"的无退避自旋。
    if (ic.ic_state == IEEE80211_S_SCAN) {
        const bool wantJoin = joinEnabled &&
            (ic.ic_wcl_join_requested || ic.ic_des_esslen != 0);
        if (wantJoin) {
            joinStatus = "scan-complete-join-requested";
            ieee80211_end_scan(&ic.ic_ac.ac_if);
        } else if (ic.ic_event_handler) {
            ++scanDoneEvents;
            ic.ic_event_handler(&ic, IEEE80211_EVT_SCAN_DONE, NULL);
        }
    }
}

struct ieee80211_channel *Ath10kHal::
firstActiveScanChannel()
{
    for (UInt32 i = 0; i <= IEEE80211_CHAN_MAX; ++i) {
        if (!ic.ic_channels[i].ic_flags)
            continue;
        if ((ic.ic_chan_active[i >> 3] & (1u << (i & 7))) == 0)
            continue;
        return &ic.ic_channels[i];
    }
    return nullptr;
}

bool Ath10kHal::
queueScanWork()
{
    if (shuttingDown || !attached) {
        scanFeedStatus = "not-attached";
        return false;
    }
    if (scanDisabled) {
        scanFeedStatus = "disabled-by-boot-arg";
        return false;
    }
    if (scanWorkRunning || scanWorkQueued) {
        // end_scan → next_scan 会在当前 worker 里重新请求一次扫描
        scanWorkRerun = true;
        return true;
    }
    if (!scanCall) {
        // ONCE：如果在自己回调里再次 thread_call_enter，重排会推迟到本次返回之后，
        // 避免同一个 thread call 并行两次（两次并发扫描会抢 CE3，直接坏事）。
        scanCall = thread_call_allocate_with_options(scanWorkEntry, this,
                                                     THREAD_CALL_PRIORITY_USER,
                                                     THREAD_CALL_OPTIONS_ONCE);
        if (!scanCall) {
            scanFeedStatus = "thread-call-unavailable";
            return false;
        }
    }
    scanWorkQueued = true;
    thread_call_enter(scanCall);
    return true;
}

void Ath10kHal::
scanWorkEntry(thread_call_param_t param0, thread_call_param_t)
{
    auto *self = static_cast<Ath10kHal *>(param0);
    if (self)
        self->runScanWork();
}

// 一轮完整扫描：前置命令 → WMI START_SCAN（帧在此期间暂存）→ 喂帧 → SCAN_DONE。
// 全程跑在 thread_call 线程上，不占用 workloop；失败只退避，绝不重入自旋。
void Ath10kHal::
runScanWork()
{
    // 双保险：即使 thread call 被并行重排，也绝不允许两轮扫描同时在跑
    if (scanWorkRunning) {
        scanWorkRerun = true;
        return;
    }
    scanWorkRunning = true;
    scanWorkQueued = false;
    scanRoundStartTick = scanTickCount;

    if (shuttingDown || !attached) {
        scanFeedStatus = "aborted-detaching";
    } else if (scanDisabled) {
        scanFeedStatus = scanSelfFused ? "disabled-self-fused" : "disabled-by-boot-arg";
    } else if (ic.ic_state != IEEE80211_S_SCAN) {
        scanFeedStatus = "not-scan-state";
    } else {
        const UInt32 feedBefore = scanFeedOk;

        // -1) 连接触发：用户选了网络且 ic_tree 里已有目标 AP 时，
        //     直接走 net80211 的 end_scan 去选 BSS → AUTH。
        //     不能只依赖"某一轮扫描结束时"触发：点选网络的时刻可能没有扫描在跑。
        if (joinTriggerPending) {
            joinTriggerPending = false;
            if (ic.ic_state == IEEE80211_S_SCAN) {
                joinTriggered = true;
                joinStatus = "end-scan-triggered";
                IOLog("Ath10kHal: JOIN 触发 end_scan（选 BSS → AUTH）\n");
                ieee80211_end_scan(&ic.ic_ac.ac_if);
                return;
            }
        }

        // 0) 运行期先确认 target 还醒着（SOC 睡下去后所有 CE 寄存器读会变 0xDEADBEEF）
        bool ready = true;
        if (!ensureTargetAwake("scan")) {
            scanFeedStatus = "target-asleep";
            ready = false;
        }

        // 1) 固件前置：chain mask + 信道表（bring-up 已下发，失败时才补）
        if (!ready) {
            // 已在上面定性，跳过后续命令
        } else if (!pdevSetupOk && !runWMIPdevSetup()) {
            scanFeedStatus = "pdev-setup-failed";
            ready = false;
        } else if (!scanSetupOk && !runWMIScanSetup()) {
            scanFeedStatus = "chan-list-failed";
            ready = false;
        }

        // 2) 固件扫描；MGMT_RX 在 applyScanInfo 里只做暂存
        scanStageCount = 0;
        bool ok = false;
        if (ready) {
            ++scanRounds;
            ok = runWMIScan();
            if (!ok)
                scanFeedStatus = scanStatus ? scanStatus : "scan-failed";
            else if (scanStageCount == 0)
                scanFeedStatus = "round-complete-no-frames";
        }

        // 3) 收尾：清节点 → 喂帧 → SCAN_DONE
        runScanCompletion();
        scanNodes = ic.ic_nnodes;

        // 4) 退避：成功后 ≥10 s 才允许下一轮；失败 5 s 起按连续失败次数线性退避（≤35 s）
        if (ok) {
            scanFailStreak = 0;
            scanNextAllowedTick = scanTickCount + kScanGapTicks;
        } else {
            if (scanFailStreak < 6)
                ++scanFailStreak;
            scanNextAllowedTick = scanTickCount + kScanBackoffBaseTicks + scanFailStreak * 5;
            if (scanNextAllowedTick > scanTickCount + kScanBackoffMaxTicks)
                scanNextAllowedTick = scanTickCount + kScanBackoffMaxTicks;

            // 通路级故障（CE3/WMI/前置命令）：立刻熔断，不赌重试
            if (ath10kScanFailureIsFatal(scanFeedStatus) && !scanDisabled) {
                scanDisabled = true;
                scanSelfFused = true;
                IOLog("Ath10kHal: M1-c3 scan self-fused (fatal status=%s)\n",
                      scanFeedStatus ? scanFeedStatus : "?");
            }
            // 软失败（超时等）连续 3 轮也熔断：反复超时同样说明通路不正常
            else if (scanFailStreak >= 3 && !scanDisabled) {
                scanDisabled = true;
                scanSelfFused = true;
                IOLog("Ath10kHal: M1-c3 scan self-fused after %u failed rounds (status=%s)\n",
                      scanFailStreak, scanFeedStatus ? scanFeedStatus : "?");
            }
        }

        if (ic.ic_wcl_scan_requested)
            ++scanRequestRounds;
        IOLog("Ath10kHal: M1-c3 scan round=%u ok=%d fed=%u status=%s nodes=%u fail-streak=%u req-rounds=%u\n",
              scanRounds, (int)ok, scanFeedOk - feedBefore,
              scanFeedStatus ? scanFeedStatus : "?", scanNodes, scanFailStreak,
              scanRequestRounds);
    }

    publishScanResultsProperties();
    emitDriverReady();          // handler 晚装时在这里补发（只发一次）

    scanWorkRunning = false;
    if (scanWorkRerun) {
        scanWorkRerun = false;
        queueScanWork();
    }
}

// 1 Hz 定时器回调（workloop 线程，只做调度判断，绝不阻塞、不碰硬件）
void Ath10kHal::
scanTimerAction(OSObject *owner, IOTimerEventSource *timer)
{
    auto *self = static_cast<Ath10kHal *>(owner);
    if (!self)
        return;
    ++self->scanTickCount;
    if (!self->shuttingDown && timer)
        timer->setTimeoutMS(1000);
    self->scanTick();
}

void Ath10kHal::
scanTick()
{
    if (shuttingDown || !attached)
        return;

    // M1-d：WCL 在 setHardwareAddress() 里置 ic_wcl_mac_reconfig，并等着驱动补一条
    // DRIVER_READY 来确认（consumeMacReconfiguration）。只发一次的话这个标志永远
    // 卡在 1，WCL 会把后续的连接请求"延后"、连扫描请求都不再下发 ——
    // 真机表现就是"点了网络毫无反应、驱动侧计数全 0"。
    if (ic.ic_wcl_mac_reconfig) {
        if (scanTickCount >= macReconfigNextTick && macReconfigAcks < 8) {
            ++macReconfigAcks;
            macReconfigNextTick = scanTickCount + 2;
            IOLog("Ath10kHal: MAC reconfig pending, re-emit DRIVER_READY #%u\n",
                  macReconfigAcks);
            emitDriverReady(true);
        }
    } else if (macReconfigAcks) {
        IOLog("Ath10kHal: MAC reconfig confirmed after %u DRIVER_READY\n", macReconfigAcks);
        macReconfigAcks = 0;
    }

    if (scanDisabled) {
        if (scanRounds == 0)
            scanFeedStatus = scanSelfFused ? "disabled-self-fused" : "disabled-by-boot-arg";
        return;
    }
    if (ic.ic_state != IEEE80211_S_SCAN)
        return;

    // M1-d：连接请求优先，且不受开机静默期限制（用户明确点了连接）。
    checkJoinTrigger();
    if (joinTriggerPending || joinTriggered)
        return;

    // 开机静默期：先让系统稳定运行一分钟（仅用于自动扫描）
    if (scanTickCount < kScanGraceTicks)
        return;

    if (scanWorkRunning) {
        // 卡轮看门狗：一轮正常 ≤12 s，超过 40 s 说明扫描线程卡在硬件访问里。
        // 这时停掉调度并留下痕迹（线程本身可能已经不可恢复）。
        if (scanRoundStartTick != 0 &&
            (scanTickCount - scanRoundStartTick) > kScanRoundHangTicks &&
            !scanDisabled) {
            scanDisabled = true;
            scanSelfFused = true;
            IOLog("Ath10kHal: M1-c3 scan round exceeded %u s, scanning disabled\n",
                  kScanRoundHangTicks);
        }
        return;
    }
    if (scanWorkQueued)
        return;
    if (scanTickCount < scanNextAllowedTick)
        return;

    // 按请求扫描：开机只跑 kScanInitialRounds 轮（证明通路 + 列表有内容），
    // 之后只有系统真的在请求扫描（WCL 的 beginWCLScan 会把
    // ic_wcl_scan_requested 置位）时才开新一轮。
    const bool requested = ic.ic_wcl_scan_requested;
    if (requested) {
        if (!scanRequestSeen) {          // 上升沿：新的一次系统扫描请求
            scanRequestSeen = true;
            scanRequestRounds = 0;
        }
        if (scanRequestRounds >= kScanMaxRoundsPerReq)
            return;                       // 本次请求已经扫够，等 WCL 清标志
    } else {
        scanRequestSeen = false;
        if (scanRounds >= kScanInitialRounds)
            return;
    }

    queueScanWork();
}

void Ath10kHal::
startScanTimer()
{
    if (scanTimer || !attached)
        return;
    IOWorkLoop *workLoop = getMainWorkLoop();
    if (!workLoop) {
        IOLog("Ath10kHal: scan timer skipped (no workloop)\n");
        return;
    }
    scanTimer = IOTimerEventSource::timerEventSource(this, scanTimerAction);
    if (!scanTimer) {
        IOLog("Ath10kHal: scan timer allocation failed\n");
        return;
    }
    if (workLoop->addEventSource(scanTimer) != kIOReturnSuccess) {
        IOLog("Ath10kHal: scan timer addEventSource failed\n");
        scanTimer->release();
        scanTimer = nullptr;
        return;
    }
    scanTimer->setTimeoutMS(1000);
    IOLog("Ath10kHal: scan poll timer started (1 Hz, request-driven)\n");
}

void Ath10kHal::
stopScanTimer()
{
    if (!scanTimer)
        return;
    scanTimer->cancelTimeout();
    if (scanTimer->getWorkLoop())
        scanTimer->getWorkLoop()->removeEventSource(scanTimer);
    scanTimer->release();
    scanTimer = nullptr;
}

// ---------------------------------------------------------------------------
// M1-d：连接命令链
//
//   AUTH 之前：VDEV_START → PEER_CREATE → PEER_ASSOC
//   RUN  之前：PEER_AUTHORIZE → VDEV_UP
//
// 参数全部取自 net80211 的 ic_bss（end_scan 选好 AP 后它已被填好）。
// 任何一步失败都只记录 + 让 net80211 自己超时回退，**绝不重试**。
// ---------------------------------------------------------------------------
bool Ath10kHal::
buildConnectParams(const char *stage, Ath10k::WmiConnectParams &c)
{
    struct ieee80211_node *ni = ic.ic_bss;
    if (!ni || !ni->ni_chan || ni->ni_chan == IEEE80211_CHAN_ANYC) {
        joinStatus = "join-no-bss";
        if (stage) IOLog("Ath10kHal: JOIN %s 缺少 ic_bss\n", stage);
        return false;
    }
    const UInt32 channel = ieee80211_chan2ieee(&ic, ni->ni_chan);
    if (!Ath10k::wmiScanChannelAllowed(channel)) {
        joinStatus = "join-bad-channel";
        IOLog("Ath10kHal: JOIN %s 信道 %u 不在扫描表内\n", stage, channel);
        return false;
    }

    c = Ath10k::WmiConnectParams();
    for (UInt32 i = 0; i < 6; ++i) {
        c.bssid[i] = ni->ni_bssid[i];
        c.localMac[i] = ic.ic_myaddr[i];
    }
    c.ssidLen = ni->ni_esslen <= 32 ? ni->ni_esslen : 32;
    for (UInt32 i = 0; i < c.ssidLen; ++i)
        c.ssid[i] = ni->ni_essid[i];
    c.channel = channel;
    c.centerChannel = channel;
    c.beaconInterval = ni->ni_intval ? ni->ni_intval : 100;
    c.dtimPeriod = ni->ni_dtimperiod ? ni->ni_dtimperiod : 1;
    c.htEnabled = (ni->ni_flags & IEEE80211_NODE_HT) != 0;
    c.vhtEnabled = (ni->ni_flags & IEEE80211_NODE_VHT) != 0;
    c.secure = (ni->ni_rsnprotos != IEEE80211_PROTO_NONE) ||
               (ni->ni_capinfo & IEEE80211_CAPINFO_PRIVACY) != 0;
    c.wmmPresent = (ni->ni_flags & IEEE80211_NODE_QOS) != 0;
    c.aid = ni->ni_associd & 0x3fffu;
    c.peerCaps = ni->ni_capinfo;
    c.peerHtCaps = ni->ni_htcaps;
    c.peerVhtCaps = ni->ni_vhtcaps;
    c.nss = 1;
    c.width = 20;
    if (ni->ni_flags & IEEE80211_NODE_HT) {
        const UInt8 chw = ni->ni_htop0 & IEEE80211_HTOP0_CHW;
        if (chw && (ic.ic_htcaps & IEEE80211_HTCAP_CBW20_40))
            c.width = 40;
    }

    c.rateCount = 0;
    const UInt32 nr = ni->ni_rates.rs_nrates;
    for (UInt32 i = 0; i < nr && c.rateCount < 16; ++i) {
        const UInt8 r = ni->ni_rates.rs_rates[i] & 0x7f;
        if (r == 0)
            continue;
        c.rates[c.rateCount++] = r;
    }
    if (!c.rateCount) {
        joinStatus = "join-no-rate";
        IOLog("Ath10kHal: JOIN %s 没有可用速率\n", stage);
        return false;
    }
    return true;
}

bool Ath10kHal::
submitConnectCommand(const UInt8 *frame, UInt32 bytes, const char *stage)
{
    if (!wmiTxBuffer.allocated()) {
        joinStatus = "join-no-wmi-buffer";
        return false;
    }
    bcopy(frame, wmiTxBuffer.virtualAddress(), bytes);
    ++joinCommands;
    if (!wmiSubmitAndReclaim(bytes, 2000)) {
        ++joinCommandFails;
        joinStatus = wmiTxFail ? wmiTxFail : "join-submit-failed";
        IOLog("Ath10kHal: JOIN %s 失败 reason=%s\n", stage, joinStatus);
        return false;
    }
    IOLog("Ath10kHal: JOIN %s ok bytes=%u seq=%u\n", stage, bytes, wmiSeq);
    return true;
}

bool Ath10kHal::
runJoinPrepare()
{
    if (!joinEnabled)
        return false;
    if (joinStage >= 3)          // 已经准备过
        return true;

    Ath10k::WmiConnectParams c;
    if (!buildConnectParams("prepare", c))
        return false;

    UInt8 *buf = static_cast<UInt8 *>(wmiTxBuffer.virtualAddress());

    if (!Ath10k::buildWMIVdevStart(buf, Ath10k::kWMICommandBufferBytes,
                                   wmiEndpoint, static_cast<UInt8>(wmiSeq), c)) {
        joinStatus = "join-build-vdev-start";
        return false;
    }
    if (!submitConnectCommand(buf, Ath10k::kWmiVdevStartBytes, "VDEV_START"))
        return false;
    joinStage = 1;

    if (!Ath10k::buildWMIPeerCreate(buf, Ath10k::kWMICommandBufferBytes,
                                    wmiEndpoint, static_cast<UInt8>(wmiSeq), c.bssid)) {
        joinStatus = "join-build-peer";
        return false;
    }
    if (!submitConnectCommand(buf, Ath10k::kWmiPeerCreateBytes, "PEER_CREATE"))
        return false;
    joinStage = 2;

    if (!Ath10k::buildWMIPeerAssoc(buf, Ath10k::kWMICommandBufferBytes,
                                   wmiEndpoint, static_cast<UInt8>(wmiSeq), c)) {
        joinStatus = "join-build-peer-assoc";
        return false;
    }
    if (!submitConnectCommand(buf, Ath10k::wmiPeerAssocBytes(c), "PEER_ASSOC"))
        return false;
    joinStage = 3;
    joinStatus = "join-prepared";
    return true;
}

bool Ath10kHal::
runJoinComplete()
{
    if (!joinEnabled || joinStage < 3 || joinStage >= 7)
        return false;

    Ath10k::WmiConnectParams c;
    if (!buildConnectParams("complete", c))
        return false;

    UInt8 *buf = static_cast<UInt8 *>(wmiTxBuffer.virtualAddress());

    if (joinStage < 5) {
        if (!Ath10k::buildWMIPeerAuthorize(buf, Ath10k::kWMICommandBufferBytes,
                                           wmiEndpoint, static_cast<UInt8>(wmiSeq),
                                           c.bssid, true)) {
            joinStatus = "join-build-authorize";
            return false;
        }
        if (!submitConnectCommand(buf, Ath10k::kWmiPeerAuthorizeBytes, "PEER_AUTHORIZE"))
            return false;
        joinStage = 5;
    }

    if (!Ath10k::buildWMIVdevUp(buf, Ath10k::kWMICommandBufferBytes,
                                wmiEndpoint, static_cast<UInt8>(wmiSeq), c)) {
        joinStatus = "join-build-vdev-up";
        return false;
    }
    if (!submitConnectCommand(buf, Ath10k::kWmiVdevUpBytes, "VDEV_UP"))
        return false;
    joinStage = 6;
    joinStatus = "join-complete";
    return true;
}

// 扫描结果相关的轻量属性：每轮扫描结束后刷新，避免整组 publishHardwareProperties
// （约 150 次 setProperty）被高频重复执行。
void Ath10kHal::
publishScanResultsProperties()
{
    IOService *service = OSDynamicCast(IOService, getController());
    if (!service)
        return;

    publishNumber(service, "ath10k-pdev-setup-ok", pdevSetupOk ? 1 : 0);
    publishString(service, "ath10k-pdev-setup-status",
                  pdevSetupStatus ? pdevSetupStatus : "unknown");
    publishNumber(service, "ath10k-scan-setup-ok", scanSetupOk ? 1 : 0);
    publishNumber(service, "ath10k-scan-ok", scanOk ? 1 : 0);
    publishString(service, "ath10k-scan-status", scanStatus ? scanStatus : "unknown");
    publishNumber(service, "ath10k-scan-events", scanEvents);
    publishNumber(service, "ath10k-scan-mgmt", scanMgmtFrames);
    publishNumber(service, "ath10k-scan-started", scanStarted ? 1 : 0);
    publishNumber(service, "ath10k-scan-completed", scanCompleted ? 1 : 0);
    publishNumber(service, "ath10k-scan-reason", scanReason);
    publishNumber(service, "ath10k-scan-last-freq", scanLastFreq);
    publishNumber(service, "ath10k-scan-wait-ms", scanWaitMs);
    publishNumber(service, "ath10k-scan-bss-count", scanBssCount);
    publishNumber(service, "ath10k-scan-ev-started", scanEvStarted);
    publishNumber(service, "ath10k-scan-ev-completed", scanEvCompleted);
    publishNumber(service, "ath10k-scan-ev-bss-chan", scanEvBssChan);
    publishNumber(service, "ath10k-scan-ev-foreign-chan", scanEvForeignChan);
    publishNumber(service, "ath10k-scan-ev-other", scanEvOther);
    publishNumber(service, "ath10k-ce1-frames", ce1Frames);
    publishNumber(service, "ath10k-ce1-last-endpoint", ce1LastEndpoint);
    publishNumber(service, "ath10k-ce1-last-bytes", ce1LastBytes);

    // WMI TX 诊断：必须每轮刷新，否则看到的是 bring-up 时的旧值
    // （M1-c3 r1 真机判读时就被 wmi-tx-fail="none" 误导过一次）
    publishString(service, "ath10k-wmi-tx-fail", wmiTxFail ? wmiTxFail : "none");
    publishNumber(service, "ath10k-wmi-seq", wmiSeq);
    publishNumber(service, "ath10k-wmi-tx-src-write", wmiTxSrcWrite);
    publishNumber(service, "ath10k-wmi-tx-src-read", wmiTxSrcRead);
    publishNumber(service, "ath10k-wmi-tx-src-write-raw", wmiTxSrcWriteRaw);
    publishNumber(service, "ath10k-wmi-tx-src-read-raw", wmiTxSrcReadRaw);
    publishNumber(service, "ath10k-ce3-resyncs", ce3Resyncs);
    publishNumber(service, "ath10k-wake-asserted", wakeAsserted ? 1 : 0);
    publishNumber(service, "ath10k-target-awake", targetAwake ? 1 : 0);
    publishNumber(service, "ath10k-wake-retries", wakeRetries);
    publishHex32(service, "ath10k-last-rtc-state", lastRtcState);
    publishNumber(service, "ath10k-last-rtc-readable", lastRtcReadable ? 1 : 0);
    publishNumber(service, "ath10k-net80211-event-handler", ic.ic_event_handler ? 1 : 0);

    // M1-d 连接诊断（每轮刷新，避免再被旧值误导）
    publishString(service, "ath10k-join-status", joinStatus ? joinStatus : "not-run");
    publishString(service, "ath10k-build", kAth10kBuildTag);
    publishNumber(service, "ath10k-join-enabled", joinEnabled ? 1 : 0);
    publishNumber(service, "ath10k-join-stage", joinStage);
    publishNumber(service, "ath10k-join-commands", joinCommands);
    publishNumber(service, "ath10k-join-command-fails", joinCommandFails);
    publishNumber(service, "ath10k-mgmt-tx-submitted", mgmtTxSubmitted);
    publishNumber(service, "ath10k-mgmt-tx-completed", mgmtTxCompleted);
    publishNumber(service, "ath10k-mgmt-tx-failed", mgmtTxFailed);
    publishNumber(service, "ath10k-mgmt-tx-dropped", mgmtTxDropped);
    publishNumber(service, "ath10k-join-feed-ok", joinFeedOk);
    publishNumber(service, "ath10k-rx-pump-running", rxPumpRunning ? 1 : 0);
    publishNumber(service, "ath10k-rx-pump-events", rxPumpEvents);
    publishNumber(service, "ath10k-rx-pump-errors", rxPumpErrors);
    // WCL / net80211 侧的连接意图（判断"点了网络但驱动没看到"用）
    publishNumber(service, "ath10k-wcl-join-requested", ic.ic_wcl_join_requested ? 1 : 0);
    publishNumber(service, "ath10k-wcl-scan-requested", ic.ic_wcl_scan_requested ? 1 : 0);
    publishNumber(service, "ath10k-wcl-mac-reconfig", ic.ic_wcl_mac_reconfig ? 1 : 0);
    publishNumber(service, "ath10k-mac-reconfig-acks", macReconfigAcks);
    publishNumber(service, "ath10k-des-esslen", static_cast<UInt32>(ic.ic_des_esslen));
    publishNumber(service, "ath10k-if-flags", static_cast<UInt32>(ic.ic_if.if_flags));
    publishNumber(service, "ath10k-if-running",
                  (ic.ic_if.if_flags & IFF_RUNNING) ? 1 : 0);
    publishNumber(service, "ath10k-ic-flags", static_cast<UInt32>(ic.ic_flags));
    {
        char list[1024];
        UInt32 pos = 0;
        list[0] = 0;
        for (UInt32 i = 0; i < scanBssCount && i < 8; ++i) {
            const ScanBssEntry &e = scanBss[i];
            char ssid[36];
            sanitizeSsid(e.ssid, e.ssidLen, ssid, sizeof(ssid));
            char one[96];
            snprintf(one, sizeof(one), "%s|%02x:%02x:%02x:%02x:%02x:%02x|%u|%u",
                     ssid[0] ? ssid : "<hidden>",
                     e.mac[0], e.mac[1], e.mac[2], e.mac[3], e.mac[4], e.mac[5],
                     e.freq, e.snr);
            /* 每条单独发布，避免汇总串被特殊字符破坏后无法判读 */
            char key[32];
            snprintf(key, sizeof(key), "ath10k-scan-bss%u", i);
            publishString(service, key, one);

            const int written = snprintf(list + pos, sizeof(list) - pos, "%s%s",
                                         i ? " ; " : "", one);
            if (written <= 0 || (UInt32)written >= sizeof(list) - pos)
                break;
            pos += (UInt32)written;
        }
        publishString(service, "ath10k-scan-ssids", pos ? list : "(none)");
    }

    publishScan80211Properties();
}

void Ath10kHal::
emitDriverReady(bool force)
{
    if (driverReadySent && !force)
        return;
    if (!ic.ic_event_handler) {
        // AirportItlwm::start() 才装 handler；bring-up 有可能先跑完，
        // 这时先不发，等第一轮扫描结束时再补（两处都是 thread_call 上下文）。
        IOLog("Ath10kHal: DRIVER_READY postponed (no event handler yet)\n");
        return;
    }
    driverReadySent = true;
    // 生命周期事件必须在 workloop 之外发：WCL 自己会通过 command gate 派发
    ic.ic_event_handler(&ic, IEEE80211_EVT_DRIVER_READY, NULL);
    IOLog("Ath10kHal: IEEE80211_EVT_DRIVER_READY emitted (force=%d)\n", (int)force);
}

void Ath10kHal::
publishScan80211Properties()
{
    IOService *service = OSDynamicCast(IOService, getController());
    if (!service)
        return;

    publishNumber(service, "ath10k-net80211-state", ic.ic_state);
    publishNumber(service, "ath10k-net80211-opmode", ic.ic_opmode);
    publishNumber(service, "ath10k-net80211-nodes", ic.ic_nnodes);
    publishNumber(service, "ath10k-scan-rounds", scanRounds);
    publishNumber(service, "ath10k-scan-feed-attempts", scanFeedAttempts);
    publishNumber(service, "ath10k-scan-feed-ok", scanFeedOk);
    publishNumber(service, "ath10k-scan-feed-skipped", scanFeedSkipped);
    publishNumber(service, "ath10k-scan-stage-overflow", scanStageOverflow);
    publishNumber(service, "ath10k-scan-nodes", scanNodes);
    publishNumber(service, "ath10k-scan-done-events", scanDoneEvents);
    publishNumber(service, "ath10k-scan-disabled", scanDisabled ? 1 : 0);
    publishNumber(service, "ath10k-scan-self-fused", scanSelfFused ? 1 : 0);
    publishNumber(service, "ath10k-scan-tick", scanTickCount);
    publishNumber(service, "ath10k-scan-next-allowed-tick", scanNextAllowedTick);
    publishNumber(service, "ath10k-scan-fail-streak", scanFailStreak);
    publishNumber(service, "ath10k-scan-request-seen", scanRequestSeen ? 1 : 0);
    publishNumber(service, "ath10k-scan-request-rounds", scanRequestRounds);
    publishNumber(service, "ath10k-scan-round-start-tick", scanRoundStartTick);
    publishNumber(service, "ath10k-scan-grace-ticks", kScanGraceTicks);
    publishString(service, "ath10k-scan-feed-status",
                  scanFeedStatus ? scanFeedStatus : "unknown");
}

// ---------------------------------------------------------------------------
// 异步 bring-up：把整条固件链路移出 attach() 的同步上下文
// ---------------------------------------------------------------------------
void Ath10kHal::
bringUpCallEntry(thread_call_param_t param0, thread_call_param_t)
{
    auto *self = static_cast<Ath10kHal *>(param0);
    if (self)
        self->runBringUp();
}

void Ath10kHal::
runBringUp()
{
    bringUpRunning = true;
    IOLog("Ath10kHal: bring-up thread begin\n");

    // WCL 生命周期第一步：原生驱动（iwn/iwm/iwx）在 init 一开始都会发这条，
    // WCL 据此向系统 post ResetInterface(0) = resetInterfacesBegin。
    // 我们原先只发 DRIVER_READY，系统侧很可能因此不认为接口完成过一次复位，
    // 表现为：菜单能列出网络，但点选后**根本不下发连接请求**（M1-d 真机现象）。
    if (ic.ic_event_handler) {
        ic.ic_event_handler(&ic, IEEE80211_EVT_DRIVER_RESET_BEGIN, NULL);
        IOLog("Ath10kHal: IEEE80211_EVT_DRIVER_RESET_BEGIN emitted\n");
    }

    if (!shuttingDown) {
        probeHardware();
        publishHardwareProperties();
    }

    bringUpRunning = false;

    // publishHardwareProperties 是在 bring-up 运行期间调用的，
    // 那时 bringUpRunning 必然为 1；这里单独发布最终完成标志。
    IOEthernetController *ctrl = getController();
    if (ctrl) {
        OSNumber *done = OSNumber::withNumber((unsigned long long)1, 64);
        if (done) {
            ctrl->setProperty("ath10k-bringup-done", done);
            done->release();
        }
        OSNumber *running = OSNumber::withNumber((unsigned long long)0, 64);
        if (running) {
            ctrl->setProperty("ath10k-bringup-running", running);
            running->release();
        }
    }

    IOLog("Ath10kHal: bring-up thread end status=%s\n",
          hardwareStatus ? hardwareStatus : "unknown");
}

// ---------------------------------------------------------------------------
// M0.1c-2：热复位
// ---------------------------------------------------------------------------
bool Ath10kHal::
maskTargetIRQsForWarmReset()
{
    bool ok = true;
    for (UInt32 ceId = 0; ceId < Ath10k::kCECount; ++ceId) {
        const auto &cfg = Ath10k::kCEHostConfig[ceId];
        if (cfg.poll)
            continue; // Linux ath10k_ce_disable_interrupt() 跳过 polling CE

        const UInt32 base = Ath10k::ceBaseAddress(ceId);
        UInt32 hostIE = 0xffffffffu;
        UInt32 miscIE = 0xffffffffu;
        if (!regs.read32(base + Ath10k::kCEHostIE, hostIE) ||
            !regs.read32(base + Ath10k::kCEMiscIE, miscIE)) {
            ok = false;
            continue;
        }
        ok &= regs.write32(base + Ath10k::kCEHostIE,
                           hostIE & ~Ath10k::kCEHostInterruptMask);
        ok &= regs.write32(base + Ath10k::kCEMiscIE,
                           miscIE & ~Ath10k::kCEMiscErrorMask);
    }
    IOLog("Ath10kHal: warm reset IRQ mask %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

bool Ath10kHal::
warmResetSI0()
{
    const UInt32 offset = Ath10k::socResetControlOffset(Ath10k::kQca6174RegisterLayout);
    UInt32 val = 0xffffffffu;
    if (!regs.read32(offset, val))
        return false;

    bool ok = true;
    ok &= regs.write32(offset, val | Ath10k::kSocResetControlSI0ResetMask);
    UInt32 flush = 0xffffffffu;
    ok &= regs.read32(offset, flush);
    IOSleep(Ath10k::kWarmResetDelayMs);
    ok &= regs.read32(offset, val);
    ok &= regs.write32(offset, val & ~Ath10k::kSocResetControlSI0ResetMask);
    ok &= regs.read32(offset, flush);
    IOSleep(Ath10k::kWarmResetDelayMs);

    IOLog("Ath10kHal: warm reset SI0 %s control=0x%08x\n", ok ? "PASS" : "FAIL", flush);
    return ok;
}

bool Ath10kHal::
warmResetCPU(const char *stage)
{
    const auto &layout = Ath10k::kQca6174RegisterLayout;
    const UInt32 resetOffset = Ath10k::socResetControlOffset(layout);
    UInt32 val = 0xffffffffu;

    if (!regs.write32(layout.fwIndicatorAddress, 0) || !regs.read32(resetOffset, val)) {
        IOLog("Ath10kHal: warm reset CPU %s ERROR prepare\n", stage);
        return false;
    }

    const UInt32 requested = val | Ath10k::kSocResetControlCPUWarmMask;
    const bool ok = regs.write32(resetOffset, requested);
    IOLog("Ath10kHal: warm reset CPU %s %s control 0x%08x -> 0x%08x\n",
          stage, ok ? "asserted" : "FAIL", val, requested);
    return ok;
}

bool Ath10kHal::
warmResetClearLF()
{
    const UInt32 offset = Ath10k::socLFTimerControl0Offset(Ath10k::kQca6174RegisterLayout);
    UInt32 before = 0xffffffffu;
    UInt32 after  = 0xffffffffu;

    bool ok = regs.read32(offset, before);
    if (ok)
        ok &= regs.write32(offset, before & ~Ath10k::kSocLFTimerEnableMask);
    if (ok)
        ok &= regs.read32(offset, after);

    const bool cleared = ok && (after & Ath10k::kSocLFTimerEnableMask) == 0;
    IOLog("Ath10kHal: warm reset LF timer %s 0x%08x -> 0x%08x\n",
          cleared ? "PASS" : "FAIL", before, after);
    return cleared;
}

bool Ath10kHal::
warmResetCE()
{
    const UInt32 offset = Ath10k::socResetControlOffset(Ath10k::kQca6174RegisterLayout);
    UInt32 val = 0xffffffffu;
    if (!regs.read32(offset, val))
        return false;

    if (!regs.write32(offset, val | Ath10k::kSocResetControlCEResetMask))
        return false;
    IOSleep(Ath10k::kWarmResetDelayMs);
    if (!regs.write32(offset, val & ~Ath10k::kSocResetControlCEResetMask))
        return false;

    IOLog("Ath10kHal: warm reset CE done value=0x%08x\n",
          val & ~Ath10k::kSocResetControlCEResetMask);
    return true;
}

bool Ath10kHal::
waitForTargetInit(UInt32 &elapsedMs, UInt32 &indicator)
{
    const UInt32 offset = Ath10k::kQca6174RegisterLayout.fwIndicatorAddress;
    UInt32 value = 0xffffffffu;
    UInt32 elapsed = 0;

    elapsedMs = 0;
    indicator = 0xffffffffu;

    if (!regs.read32(offset, value)) {
        IOLog("Ath10kHal: target-init ERROR FW_INDICATOR unreadable\n");
        return false;
    }
    IOLog("Ath10kHal: target-init begin FW_INDICATOR=0x%08x timeout=%u ms\n",
          value, Ath10k::kTargetInitTimeoutMs);

    bool ready = false;
    while (elapsed <= Ath10k::kTargetInitTimeoutMs) {
        if (!regs.read32(offset, value)) {
            IOLog("Ath10kHal: target-init ERROR MMIO read failed\n");
            break;
        }
        indicator = value;

        if (value != 0xffffffffu) {
            if (value & Ath10k::kFwIndicatorEventPending) {
                IOLog("Ath10kHal: target-init ERROR event-pending 0x%08x elapsed=%u ms\n",
                      value, elapsed);
                break;
            }
            if (value & Ath10k::kFwIndicatorInitialized) {
                ready = true;
                break;
            }
        }

        if (elapsed == Ath10k::kTargetInitTimeoutMs)
            break;

        // Linux 的 INTx target/BAR 初始化竞争规避：bootstrap 轮询期间保持中断屏蔽
        if (value != 0xffffffffu) {
            const UInt32 mask = Ath10k::kPCIeIntrFirmwareMask | Ath10k::kPCIeIntrCEMaskAll;
            const UInt32 enable = Ath10k::socCoreOffset(Ath10k::kQca6174RegisterLayout,
                                                       Ath10k::kPCIeIntrEnableAddress);
            UInt32 flush = 0;
            if (!regs.write32(enable, mask) || !regs.read32(enable, flush) || flush != mask)
                break;
        }

        IOSleep(Ath10k::kTargetInitPollMs);
        elapsed += Ath10k::kTargetInitPollMs;
        if (elapsed > Ath10k::kTargetInitTimeoutMs)
            elapsed = Ath10k::kTargetInitTimeoutMs;
    }

    // 离开时保持中断屏蔽并清除挂起位
    {
        UInt32 flush = 0;
        const auto &l = Ath10k::kQca6174RegisterLayout;
        regs.write32(Ath10k::socCoreOffset(l, Ath10k::kPCIeIntrEnableAddress), 0);
        regs.write32(Ath10k::socCoreOffset(l, Ath10k::kPCIeIntrClearAddress),
                     Ath10k::kPCIeIntrFirmwareMask | Ath10k::kPCIeIntrCEMaskAll);
        regs.read32(Ath10k::socCoreOffset(l, Ath10k::kPCIeIntrEnableAddress), flush);
    }

    elapsedMs = elapsed;
    IOLog("Ath10kHal: target-init %s indicator=0x%08x elapsed=%u ms\n",
          ready ? "READY" : "TIMEOUT", indicator, elapsed);
    return ready;
}

bool Ath10kHal::
warmResetTarget()
{
    warmResetDone = false;
    warmSI0Ok = warmCPU1Ok = warmCPU2Ok = warmLFok = warmCEok = false;
    warmInit1Ms = warmInit2Ms = 0;
    warmIndicator1 = warmIndicator2 = 0xffffffffu;
    postWarmChipId = 0xffffffffu;
    chipStableAfterWarm = false;
    cePipeInitPassCount = 0;

    const UInt32 resetOffset = Ath10k::socResetControlOffset(Ath10k::kQca6174RegisterLayout);
    regs.read32(resetOffset, warmResetControlBefore);
    IOLog("Ath10kHal: warm reset begin control=0x%08x\n", warmResetControlBefore);

    if (!maskTargetIRQsForWarmReset())
        return false;

    warmSI0Ok = warmResetSI0();
    if (!warmSI0Ok)
        return false;

    warmCPU1Ok = warmResetCPU("pass1");
    if (!warmCPU1Ok)
        return false;

    if (!initCopyEnginePipes("warm-pass1"))
        return false;

    // 第一次 target-init 仅记录，Linux 同样忽略其返回值
    const bool init1 = waitForTargetInit(warmInit1Ms, warmIndicator1);
    IOLog("Ath10kHal: warm reset first target-init %s indicator=0x%08x elapsed=%u ms\n",
          init1 ? "READY" : "FAILED", warmIndicator1, warmInit1Ms);

    warmLFok = warmResetClearLF();
    if (!warmLFok) {
        deinitCopyEnginePipes("warm-lf-failure");
        return false;
    }

    warmCEok = warmResetCE();
    if (!warmCEok) {
        deinitCopyEnginePipes("warm-ce-failure");
        return false;
    }

    warmCPU2Ok = warmResetCPU("pass2");
    if (!warmCPU2Ok) {
        deinitCopyEnginePipes("warm-cpu2-failure");
        return false;
    }

    if (!initCopyEnginePipes("warm-pass2"))
        return false;

    const bool init2 = waitForTargetInit(warmInit2Ms, warmIndicator2);

    regs.read32(resetOffset, warmResetControlAfter);
    regs.read32(Ath10k::chipIdOffset(Ath10k::kQca6174RegisterLayout), postWarmChipId);

    const UInt8 rev = Ath10k::chipRevision(postWarmChipId);
    chipStableAfterWarm = (hwCandidate != nullptr) &&
                          postWarmChipId != 0xffffffffu &&
                          rev == hwCandidate->chipRevision;

    warmResetDone = init2 && chipStableAfterWarm && cePipeInitPassCount >= 2;

    IOLog("Ath10kHal: warm reset %s second-init=%s indicator=0x%08x elapsed=%u ms post-chip=0x%08x rev=%u ce-passes=%u\n",
          warmResetDone ? "COMPLETE" : "FAILED",
          init2 ? "READY" : "FAILED", warmIndicator2, warmInit2Ms,
          postWarmChipId, rev, cePipeInitPassCount);
    return warmResetDone;
}

// ---------------------------------------------------------------------------
// 硬件探测主流程（M0.1a + M0.1b + M0.1c-1/2）
//
// 完整顺序：
//   1) PCI 标识 → 2) BAR0 映射 → 3) 关 ASPM
//   4) 读 chip-id（预期哨兵）→ 5) wake → 6) 冷复位 → 7) wake → 8) 读 chip-id
//   9) 匹配硬件候选 → 10) 分配全部 CE ring
//   11) 热复位（内部两遍 CE 管道编程）
//   12) 复位后重新唤醒 → 13) CE 管道最终编程与校验
//   14) 收尾：释放 SOC_WAKE、恢复 ASPM
// ---------------------------------------------------------------------------
bool Ath10kHal::
probeHardware()
{
    hardwareProbed  = false;
    hardwareStatus  = "not-started";
    chipId          = 0xffffffffu;
    chipRevision    = 0xff;
    hwCandidate     = nullptr;
    chipIdAttempts  = 0;
    pciDeviceId     = 0;

    if (!pciNub) {
        hardwareStatus = "no-pci-device";
        IOLog("Ath10kHal: probeHardware ERROR no PCI device\n");
        return false;
    }

    // 1) PCI 标识
    const UInt16 vendor    = pciNub->configRead16(kIOPCIConfigVendorID);
    const UInt16 device    = pciNub->configRead16(kIOPCIConfigDeviceID);
    const UInt16 subsysVen = pciNub->configRead16(kPCICfgSubsystemVen);
    const UInt16 subsysDev = pciNub->configRead16(kPCICfgSubsystemId);
    pciDeviceId    = device;
    pciSubVendorId = subsysVen;
    pciSubDeviceId = subsysDev;
    IOLog("Ath10kHal: PCI %04x:%04x subsystem %04x:%04x\n",
          vendor, device, subsysVen, subsysDev);

    if (!Ath10k::isSupportedDevice(vendor, device)) {
        hardwareStatus = "unsupported-pci-device";
        IOLog("Ath10kHal: probeHardware ERROR unsupported PCI device\n");
        return false;
    }

    // 2) BAR0 映射
    bar0Map = pciNub->mapDeviceMemoryWithRegister(0x10 /* BAR0 */);
    if (!bar0Map) {
        hardwareStatus = "bar0-map-failed";
        IOLog("Ath10kHal: probeHardware ERROR BAR0 map failed\n");
        return false;
    }

    const UInt64 barLength = bar0Map->getLength();
    regs.attach(reinterpret_cast<volatile UInt8 *>(bar0Map->getVirtualAddress()),
                barLength);
    IOLog("Ath10kHal: BAR0 mapped length=0x%llx\n", (unsigned long long)barLength);

    if (!regs.attached()) {
        hardwareStatus = "bar0-too-small";
        IOLog("Ath10kHal: probeHardware ERROR BAR0 too small\n");
        return false;
    }

    // 3) 关闭 ASPM，稳定后续复位与传输窗口
    disableASPM();

    // 4) 第一次读芯片 ID（复位前，通常为哨兵）
    UInt32 id = 0xffffffffu;
    bool readOk = readChipIdValue(id);
    chipIdAttempts++;
    IOLog("Ath10kHal: chip-id attempt#%u raw=0x%08x read=%s\n",
          chipIdAttempts, id, readOk ? "ok" : "fail");

    if (readOk && !isChipIdSentinel(id)) {
        chipId = id;
    } else {
        // 5~8) 唤醒 → 冷复位 → 再唤醒 → 重读
        if (!wakeTarget()) {
            hardwareStatus = "wake-failed";
            IOLog("Ath10kHal: probeHardware wake failed\n");
            goto finish;
        }
        if (!coldResetTarget()) {
            hardwareStatus = "cold-reset-failed";
            IOLog("Ath10kHal: probeHardware cold reset failed\n");
            goto finish;
        }
        if (!wakeTarget()) {
            hardwareStatus = "rewake-failed";
            IOLog("Ath10kHal: probeHardware re-wake failed\n");
            goto finish;
        }

        readOk = readChipIdValue(id);
        chipIdAttempts++;
        IOLog("Ath10kHal: chip-id attempt#%u (post-reset) raw=0x%08x read=%s\n",
              chipIdAttempts, id, readOk ? "ok" : "fail");

        if (readOk && !isChipIdSentinel(id))
            chipId = id;
    }

    if (!readOk) {
        hardwareStatus = "chip-id-read-failed";
        IOLog("Ath10kHal: probeHardware ERROR chip-id unreadable\n");
        goto finish;
    }
    if (isChipIdSentinel(chipId)) {
        hardwareStatus = "chip-id-invalid";
        IOLog("Ath10kHal: probeHardware chip-id still invalid (0x%08x)\n", chipId);
        goto finish;
    }

    // 9) 匹配硬件候选
    chipRevision = Ath10k::chipRevision(chipId);
    hwCandidate  = Ath10k::findCandidate(device, chipRevision);
    IOLog("Ath10kHal: SOC_CHIP_ID=0x%08x chip-rev=%u\n", chipId, chipRevision);

    if (!hwCandidate) {
        hardwareStatus = "unsupported-chip-revision";
        IOLog("Ath10kHal: chip revision %u not supported for device %04x\n",
              chipRevision, device);
        goto finish;
    }

    IOLog("Ath10kHal: matched candidate target=%s firmware=%s cpu=%u\n",
          hwCandidate->targetRevision, hwCandidate->firmwareDirectory,
          hwCandidate->targetCpuFrequencyHz);

    // 10) 分配全部 CE ring
    if (!allocateCERings()) {
        hardwareStatus = "ce-ring-failed";
        goto finish;
    }

    // 11) 热复位（内部含两遍 CE 管道编程）
    if (!warmResetTarget()) {
        hardwareStatus = "warm-reset-failed";
        goto finish;
    }

    // 12) 热复位会清掉 SOC_WAKE，需要重新唤醒
    if (!wakeTarget()) {
        hardwareStatus = "post-warm-wake-failed";
        goto finish;
    }

    // 13) 复位后 CE 管道最终编程与校验
    if (!initCopyEnginePipes("post-chip-reset")) {
        hardwareStatus = "ce-pipe-program-failed";
        deinitCopyEnginePipes("post-chip-reset-failure");
        goto finish;
    }

    // 14) CE7 诊断通道：只读发现 target 的 CE 配置地址
    if (!runCE7DiagnosticRead()) {
        hardwareStatus = "ce7-discovery-failed";
        goto finish;
    }

    // 15) init_config：把 CE 管道表与 service map 写入 target
    if (!runCE7InitConfig()) {
        hardwareStatus = "init-config-failed";
        goto finish;
    }

    // 16) 唤醒 target CPU，进入 BMI 阶段
    if (!wakeTargetCPUForBMI()) {
        hardwareStatus = "cpu-wake-failed";
        goto finish;
    }

    // 17) BMI_GET_TARGET_INFO —— M0.1c-1 的终点
    if (!runBMIGetTargetInfo()) {
        hardwareStatus = "bmi-get-target-info-failed";
        goto finish;
    }

    // 18) 前置探测：读 Host Interest 关键字段
    if (!runBMIBoardPrerequisiteProbe()) {
        hardwareStatus = "bmi-prereq-failed";
        goto finish;
    }

    // 19) 176 MHz PLL（仅 hw1.1 档）
    if (!configureTargetPLL()) {
        hardwareStatus = "pll-failed";
        goto finish;
    }

    // 20) Boot Host Interest 五字写入
    if (!configureBootHostInterest()) {
        hardwareStatus = "boot-hi-failed";
        goto finish;
    }

    // 21) 选择 board/firmware payload（按精确 subsystem ID + chip revision）
    payloadProfile = &Ath10kPayload::selectProfile(Ath10k::kAtherosVendor,
                                                   pciDeviceId,
                                                   pciSubVendorId,
                                                   pciSubDeviceId,
                                                   chipRevision);
    payloadSelected = (payloadProfile != nullptr) && payloadProfile->available;
    IOLog("Ath10kHal: payload select %04x:%04x sub %04x:%04x chip %u -> %s\n",
          Ath10k::kAtherosVendor, pciDeviceId, pciSubVendorId, pciSubDeviceId,
          chipRevision, payloadSelected ? payloadProfile->kBoardKey : "unavailable");

    if (!payloadSelected) {
        hardwareStatus = "payload-unavailable";
        goto finish;
    }

    // 22) board data 上传 + 回读校验（非破坏性里程碑）
    if (!runBMIBoardUpload()) {
        hardwareStatus = "board-upload-failed";
        goto finish;
    }
    boardUploadOk = true;

    // 23) OTP 下载 + BMI_EXECUTE（校准数据，OTP 必须早于固件）
    if (!runBMIOtp()) {
        hardwareStatus = "otp-failed";
        goto finish;
    }

    // 24) WLAN 固件下载 + 6 段回读校验
    if (!runBMIFirmwareDownload()) {
        hardwareStatus = "firmware-download-failed";
        goto finish;
    }

    // 25) Boot HI 收尾（关 UART + 终检）
    if (!finishBootHostInterest()) {
        hardwareStatus = "boot-hi-finish-failed";
        goto finish;
    }

    // 26) CE1 挂 2048 B RX 描述符，准备接收固件的第一个 HTC 包
    if (!prepareHTCReadyReceive()) {
        hardwareStatus = "htc-rx-arm-failed";
        goto finish;
    }

    // 27) BMI_DONE —— 单向不可逆，之后 BMI 永久关闭
    if (!bmiSendDone()) {
        hardwareStatus = "bmi-done-failed";
        goto finish;
    }

    // 28) 等待并校验 HTC READY
    if (!waitForHTCReady()) {
        hardwareStatus = "htc-ready-failed";
        goto finish;
    }

    // 29) HTC 服务建立（HTT + WMI）
    if (!runHTCServiceSetup()) {
        hardwareStatus = "htc-service-failed";
        goto finish;
    }

    // 30) WMI bootstrap：SETUP_COMPLETE → SERVICE_AVAILABLE(3) → SERVICE_READY(1)
    if (!runWMIBootstrap()) {
        hardwareStatus = "wmi-bootstrap-failed";
        goto finish;
    }

    // 31) WMI INIT + host memory → READY(2)（带真实 MAC）
    if (!runWMIInit()) {
        hardwareStatus = "wmi-init-failed";
        goto finish;
    }

    // 32) HTT RX 环：不建这个环，固件不会把任何接收帧交给主机
    if (!runHTTRxSetup())
        IOLog("Ath10kHal: HTT RX setup not completed status=%s（继续尝试后续阶段）\n",
              httSetupStatus ? httSetupStatus : "unknown");

    // 33) vdev create（STA）
    if (!runWMIVdevCreate()) {
        hardwareStatus = "wmi-vdev-failed";
        goto finish;
    }

    hardwareStatus = "wmi-ready";
    IOLog("Ath10kHal: M1-b2 complete — firmware running chip=0x%08x rev=%u ce-passes=%u htc-credits=%u endpoints=%u wmi-credits=%u/%u wmi-seq=%u\n",
          postWarmChipId, chipRevision, cePipeInitPassCount,
          htcReady.credits, htcReady.maxEndpoints,
          wmiCredits.available, wmiCredits.total, wmiSeq);

    // 34) M1-c3：PDEV 参数 + 信道表是扫描前置；真正的 START_SCAN 交给
    //     net80211 的 SCAN 状态（ath_newstate → thread_call），
    //     这样系统每次请求扫描都会重新走一遍，而不是只有开机那一次。
    if (!runWMIPdevSetup() || !runWMIScanSetup()) {
        IOLog("Ath10kHal: M1-c3 scan setup incomplete pdev=%s scan=%s\n",
              pdevSetupStatus ? pdevSetupStatus : "unknown",
              scanStatus ? scanStatus : "unknown");
    }

    // 固件已就绪：net80211 侧的 IFF_RUNNING 语义等同"驱动可用"
    // （iwx 也在 iwx_init 结尾置位）。WCL 的 beginWCLScan 第一道门就是它。
    ic.ic_ac.ac_if.if_flags |= IFF_RUNNING;

    // 让 net80211 进入 SCAN 状态：内部只做状态切换并排队扫描
    if (ic.ic_newstate) {
        const int stateError = ic.ic_newstate(&ic, IEEE80211_S_SCAN, -1);
        if (stateError)
            IOLog("Ath10kHal: M1-c3 SCAN state request failed err=%d\n", stateError);
    }

    // 告诉 WCL：驱动已就绪（iwx 在 iwx_init 结束时做同样的事）。
    // 没有这一条，系统侧不会认为网卡可用，也就不会发起扫描请求。
    emitDriverReady();

finish:
    // 固件已跑起来时**不能**释放 SOC_WAKE / 恢复 ASPM：
    // Linux ath10k 是 power_up 断言后一直保持到 power_down。
    // 提前释放会让 SOC 空闲后睡眠，之后 CE 寄存器全部读回 0xDEADBEEF
    // （M1-c3 r2 真机就是这样失败的；见 docs/13）。
    if (wmiVdevOk) {
        IOLog("Ath10kHal: keeping SOC_WAKE asserted + ASPM disabled (firmware running)\n");
        targetAwake = true;
    } else {
        releaseTargetWake();
        restoreASPM();
        targetAwake = false;
    }
    hardwareProbed = true;
    // 整条链路的结果必须在这里统一发布一次：只靠进入 bring-up 时的那次发布，
    // ioreg 里会一直是 "not-started" 和空值（M1-c3 排查时发现的老问题）。
    publishHardwareProperties();
    return true;
}

void Ath10kHal::
publishHardwareProperties()
{
    IOService *service = OSDynamicCast(IOService, getController());
    if (!service) {
        IOLog("Ath10kHal: publishHardwareProperties skipped (no controller)\n");
        return;
    }

    publishString(service, "ath10k-hardware-status",
                  hardwareStatus ? hardwareStatus : "unknown");
    publishNumber(service, "ath10k-chip-id", chipId);
    publishHex32(service, "ath10k-chip-id-hex", chipId);
    publishNumber(service, "ath10k-chip-revision", chipRevision);
    publishNumber(service, "ath10k-bar0-length", bar0Map ? bar0Map->getLength() : 0);
    publishNumber(service, "ath10k-hw-probed", hardwareProbed ? 1 : 0);
    publishNumber(service, "ath10k-chip-id-attempts", chipIdAttempts);

    // M0.1b 唤醒/复位诊断
    publishNumber(service, "ath10k-aspm-disabled", aspmDisabled ? 1 : 0);
    publishNumber(service, "ath10k-wake-ok", wakeSucceeded ? 1 : 0);
    publishNumber(service, "ath10k-wake-asserted", wakeAsserted ? 1 : 0);
    publishNumber(service, "ath10k-target-awake", targetAwake ? 1 : 0);
    publishNumber(service, "ath10k-wake-retries", wakeRetries);
    publishHex32(service, "ath10k-last-rtc-state", lastRtcState);
    publishNumber(service, "ath10k-last-rtc-readable", lastRtcReadable ? 1 : 0);
    publishNumber(service, "ath10k-wake-delay-us", wakeDelayUs);
    publishHex32(service, "ath10k-rtc-state-before", rtcStateBefore);
    publishHex32(service, "ath10k-rtc-state-after", rtcStateAfter);
    publishNumber(service, "ath10k-cold-reset-done", coldResetDone ? 1 : 0);
    publishHex32(service, "ath10k-global-reset-before", globalResetBefore);
    publishHex32(service, "ath10k-global-reset-after", globalResetAfter);
    publishHex32(service, "ath10k-pcie-link-control-before", pcieLinkControlBefore);
    publishHex32(service, "ath10k-pcie-link-control-after", pcieLinkControlAfter);

    // M0.1c-1 CE ring 状态
    publishNumber(service, "ath10k-ce-rings-ok", ringsAllocated ? 1 : 0);
    publishString(service, "ath10k-ce-rings-failure", ringsFailure ? ringsFailure : "unknown");
    publishNumber(service, "ath10k-ce-configured", ceConfiguredCount);
    publishNumber(service, "ath10k-ce-verified", ceVerifiedCount);
    publishNumber(service, "ath10k-ce-pipe-passes", cePipeInitPassCount);

    for (UInt32 ceId = 0; ceId < Ath10k::kCECount; ++ceId) {
        if (!ceSrcRings[ceId].allocated() && !ceDstRings[ceId].allocated())
            continue;
        char key[48];
        snprintf(key, sizeof(key), "ath10k-ce%u-src-entries", ceId);
        publishNumber(service, key, ceSrcRings[ceId].entries());
        snprintf(key, sizeof(key), "ath10k-ce%u-src-dma", ceId);
        publishHex32(service, key, ceSrcRings[ceId].dmaAddress32());
        snprintf(key, sizeof(key), "ath10k-ce%u-dst-entries", ceId);
        publishNumber(service, key, ceDstRings[ceId].entries());
        snprintf(key, sizeof(key), "ath10k-ce%u-dst-dma", ceId);
        publishHex32(service, key, ceDstRings[ceId].dmaAddress32());
    }

    // M0.1c-2 热复位诊断
    publishNumber(service, "ath10k-warm-reset-ok", warmResetDone ? 1 : 0);
    publishHex32(service, "ath10k-warm-control-before", warmResetControlBefore);
    publishHex32(service, "ath10k-warm-control-after", warmResetControlAfter);
    publishNumber(service, "ath10k-warm-init1-ms", warmInit1Ms);
    publishNumber(service, "ath10k-warm-init2-ms", warmInit2Ms);
    publishHex32(service, "ath10k-warm-indicator1", warmIndicator1);
    publishHex32(service, "ath10k-warm-indicator2", warmIndicator2);
    publishHex32(service, "ath10k-post-warm-chip-id", postWarmChipId);
    publishNumber(service, "ath10k-chip-stable", chipStableAfterWarm ? 1 : 0);

    // M0.1c-3 CE7 诊断通道
    publishNumber(service, "ath10k-ce7-ok", ceDiagOk ? 1 : 0);
    publishNumber(service, "ath10k-ce7-transactions", ceDiagTransactions);
    publishHex32(service, "ath10k-ce7-core-ctrl", ceDiagCoreCtrl);
    publishHex32(service, "ath10k-pcie-state-addr", ceDiagPCIeStateAddress);
    publishHex32(service, "ath10k-pipe-cfg-addr", ceDiagPipeCfgAddress);
    publishHex32(service, "ath10k-svc-map-addr", ceDiagServiceMapAddress);
    publishHex32(service, "ath10k-config-flags", ceDiagConfigFlags);

    // M0.1c-4 init_config
    publishNumber(service, "ath10k-init-config-ok", initConfigOk ? 1 : 0);
    publishNumber(service, "ath10k-init-config-banks", initConfigBanks);
    publishHex32(service, "ath10k-init-flags-orig", initConfigOrigFlags);
    publishHex32(service, "ath10k-init-flags-new", initConfigNewFlags);
    publishHex32(service, "ath10k-init-early-alloc-orig", initConfigOrigEarlyAlloc);
    publishHex32(service, "ath10k-init-early-alloc-new", initConfigNewEarlyAlloc);
    publishHex32(service, "ath10k-init-flag2-orig", initConfigOrigFlag2);
    publishHex32(service, "ath10k-init-flag2-new", initConfigNewFlag2);

    // M0.1c-5 CPU 唤醒与 BMI
    publishNumber(service, "ath10k-cpu-woken", cpuWoken ? 1 : 0);
    publishHex32(service, "ath10k-core-ctrl-before-wake", coreCtrlBeforeWake);
    publishHex32(service, "ath10k-core-ctrl-after-wake", coreCtrlAfterWake);
    publishNumber(service, "ath10k-bmi-ok", bmiOk ? 1 : 0);
    publishNumber(service, "ath10k-bmi-length", bmiTargetLength);
    publishNumber(service, "ath10k-bmi-type", bmiTargetType);
    publishHex32(service, "ath10k-bmi-version", bmiTargetVersion);
    publishNumber(service, "ath10k-bmi-uncertain", bmiTransferUncertain ? 1 : 0);
    publishNumber(service, "ath10k-bmi-timed-out", bmiTimedOut ? 1 : 0);

    // M0.1c-6 前置探测 / PLL / Boot HI
    publishNumber(service, "ath10k-bmi-prereq-ok", bmiPrereqOk ? 1 : 0);
    publishHex32(service, "ath10k-hi-board-data", hiBoardData);
    publishNumber(service, "ath10k-hi-board-init", hiBoardDataInit);
    publishHex32(service, "ath10k-hi-early-alloc", hiEarlyAlloc);
    publishHex32(service, "ath10k-hi-interconnect", hiInterconnect);
    publishHex32(service, "ath10k-hi-cald-data", hiCalData);
    publishNumber(service, "ath10k-pll-ok", pllOk ? 1 : 0);
    publishString(service, "ath10k-pll-status", pll.status ? pll.status : "unknown");
    publishNumber(service, "ath10k-pll-step", pll.step);
    publishNumber(service, "ath10k-boot-hi-ok", bootHIOk ? 1 : 0);
    publishHex32(service, "ath10k-boot-hi-option-before", bootHIOptionBefore);
    publishHex32(service, "ath10k-boot-hi-option-applied", bootHIOptionApplied);
    publishNumber(service, "ath10k-boot-hi-words", hiWordsWritten);

    // M0.1c-7 payload / board
    publishNumber(service, "ath10k-payload-selected", payloadSelected ? 1 : 0);
    publishString(service, "ath10k-payload-key",
                  payloadSelected ? payloadProfile->kBoardKey : "unavailable");
    publishString(service, "ath10k-payload-selection",
                  payloadSelected ? payloadProfile->kBoardSelection : "unavailable");
    publishString(service, "ath10k-payload-sha256",
                  payloadSelected ? payloadProfile->kBoardSHA256 : "unavailable");
    publishNumber(service, "ath10k-board-write-ok", boardWriteOk ? 1 : 0);
    publishNumber(service, "ath10k-board-verify-ok", boardVerifyOk ? 1 : 0);
    publishNumber(service, "ath10k-board-bytes", boardBytes);

    // M0.1c-8 OTP
    publishNumber(service, "ath10k-otp-download-ok", otpDownloadOk ? 1 : 0);
    publishNumber(service, "ath10k-otp-execute-ok", otpExecuteOk ? 1 : 0);
    publishNumber(service, "ath10k-otp-result-accepted", otpResultAccepted ? 1 : 0);
    publishHex32(service, "ath10k-otp-result", otpResult);
    publishNumber(service, "ath10k-otp-wire-bytes", otpWireBytesSent);
    publishNumber(service, "ath10k-otp-lz-commands", otpLZDataCommands);
    publishString(service, "ath10k-otp-status", otpStatus ? otpStatus : "unknown");
    publishNumber(service, "ath10k-bringup-running", bringUpRunning ? 1 : 0);

    // M0.1c-9 固件
    publishNumber(service, "ath10k-fw-download-ok", fwDownloadOk ? 1 : 0);
    publishNumber(service, "ath10k-fw-readback-ok", fwReadbackOk ? 1 : 0);
    publishNumber(service, "ath10k-fw-readback-bytes", fwReadbackBytes);
    publishNumber(service, "ath10k-fw-segments", fwVerifiedSegments);
    publishHex32(service, "ath10k-fw-failed-segment", fwFailedSegment);
    publishNumber(service, "ath10k-fw-wire-bytes", fwWireBytesSent);
    publishNumber(service, "ath10k-fw-lz-commands", fwLZDataCommands);
    publishHex32(service, "ath10k-fw-post-version", fwPostVersion);
    publishString(service, "ath10k-fw-status", fwStatus ? fwStatus : "unknown");

    // M0.1c-10 Boot HI 收尾 / BMI_DONE / HTC READY
    publishNumber(service, "ath10k-uart-disabled", uartDisabled ? 1 : 0);
    publishNumber(service, "ath10k-pre-done-verified", preDoneHIVerified ? 1 : 0);
    publishHex32(service, "ath10k-hi-option-after-download", bootHIOptionAfterDownload);
    publishNumber(service, "ath10k-bmi-closed", bmiClosed ? 1 : 0);
    publishNumber(service, "ath10k-bmi-done-sent", doneSent ? 1 : 0);
    publishNumber(service, "ath10k-htc-rx-posted", htcRxPosted ? 1 : 0);
    publishNumber(service, "ath10k-htc-ready-ok", htcReadyOk ? 1 : 0);
    publishNumber(service, "ath10k-htc-rx-bytes", htcRxBytes);
    publishNumber(service, "ath10k-htc-wait-ms", htcWaitMs);
    publishHex32(service, "ath10k-htc-fw-indicator", htcFwIndicator);
    publishNumber(service, "ath10k-htc-credits", htcReady.credits);
    publishNumber(service, "ath10k-htc-credit-size", htcReady.creditSize);
    publishNumber(service, "ath10k-htc-endpoints", htcReady.maxEndpoints);
    publishNumber(service, "ath10k-htc-version", htcReady.version);
    publishString(service, "ath10k-htc-status", htcStatus ? htcStatus : "unknown");
    publishString(service, "ath10k-htc-parse-error", htcParseError ? htcParseError : "none");

    // M1-a HTC 服务
    publishNumber(service, "ath10k-htc-services-ok", htcServicesOk ? 1 : 0);
    publishString(service, "ath10k-htc-service-status",
                  htcServiceStatus ? htcServiceStatus : "unknown");
    publishNumber(service, "ath10k-wmi-endpoint", wmiEndpoint);
    publishNumber(service, "ath10k-wmi-max-message", wmiMaxMessageSize);
    publishNumber(service, "ath10k-htt-endpoint", httEndpoint);
    publishNumber(service, "ath10k-htt-max-message", httMaxMessageSize);
    publishNumber(service, "ath10k-htc-ctrl-tx", htcCtrlTxCount);
    publishNumber(service, "ath10k-htc-ctrl-rx", htcCtrlRxCount);

    // M1-b1 WMI bootstrap
    publishNumber(service, "ath10k-setup-complete-sent", setupCompleteSent ? 1 : 0);
    publishNumber(service, "ath10k-wmi-service-ready", wmiServiceReady ? 1 : 0);
    publishString(service, "ath10k-wmi-status", wmiStatus ? wmiStatus : "unknown");
    publishString(service, "ath10k-wmi-parse-error", wmiParseError ? wmiParseError : "none");
    publishNumber(service, "ath10k-wmi-event-id", wmiEventId);
    publishNumber(service, "ath10k-wmi-rx-bytes", wmiRxBytes);
    publishNumber(service, "ath10k-wmi-wait-ms", wmiWaitMs);
    publishNumber(service, "ath10k-wmi-service-bits", wmiServiceBits);
    publishHex32(service, "ath10k-wmi-bitmap0", wmiServiceBitmap[0]);
    publishHex32(service, "ath10k-wmi-bitmap1", wmiServiceBitmap[1]);
    publishHex32(service, "ath10k-wmi-bitmap2", wmiServiceBitmap[2]);
    publishHex32(service, "ath10k-wmi-bitmap3", wmiServiceBitmap[3]);
    publishNumber(service, "ath10k-wmi-service-available", wmiServiceAvailable ? 1 : 0);
    publishNumber(service, "ath10k-wmi-svc-bitmap-present", wmiServiceBitmapPresent ? 1 : 0);
    publishHex32(service, "ath10k-wmi-svc-word0", wmiServiceBitmapFull[0]);
    publishHex32(service, "ath10k-wmi-svc-word1", wmiServiceBitmapFull[1]);
    publishHex32(service, "ath10k-wmi-svc-word2", wmiServiceBitmapFull[2]);
    publishHex32(service, "ath10k-wmi-svc-word3", wmiServiceBitmapFull[3]);
    publishHex32(service, "ath10k-wmi-fw-build", wmiFwBuild);
    publishHex32(service, "ath10k-wmi-abi0", wmiAbi[0]);
    publishHex32(service, "ath10k-wmi-abi2", wmiAbi[2]);
    publishHex32(service, "ath10k-wmi-abi3", wmiAbi[3]);
    publishNumber(service, "ath10k-wmi-rf-chains", wmiRfChains);
    publishNumber(service, "ath10k-wmi-memory-requests", wmiMemoryRequests);
    publishNumber(service, "ath10k-wmi-reg-caps-present", wmiRegCapsPresent ? 1 : 0);
    publishHex32(service, "ath10k-wmi-eeprom-rd", wmiRegCaps[0]);
    publishHex32(service, "ath10k-wmi-reg-cap1", wmiRegCaps[2]);
    publishHex32(service, "ath10k-wmi-low2", wmiRegCaps[5]);
    publishHex32(service, "ath10k-wmi-high2", wmiRegCaps[6]);

    // M1-b2 WMI INIT / vdev
    publishString(service, "ath10k-wmi-event-status", wmiEventStatus ? wmiEventStatus : "unknown");
    publishNumber(service, "ath10k-wmi-credits-total", wmiCredits.total);
    publishNumber(service, "ath10k-wmi-credits-available", wmiCredits.available);
    publishNumber(service, "ath10k-wmi-credits-outstanding", wmiCredits.outstanding);
    publishNumber(service, "ath10k-wmi-credit-size", wmiCredits.size);
    publishNumber(service, "ath10k-wmi-init-ok", wmiInitOk ? 1 : 0);
    publishString(service, "ath10k-wmi-init-status", wmiInitStatus ? wmiInitStatus : "unknown");
    publishNumber(service, "ath10k-wmi-init-tx-bytes", wmiInitTxBytes);
    publishNumber(service, "ath10k-wmi-init-wait-ms", wmiInitWaitMs);
    publishNumber(service, "ath10k-wmi-host-memory-chunks", wmiHostMemoryCount);
    publishNumber(service, "ath10k-wmi-host-memory-bytes", wmiHostMemoryBytes);
    publishNumber(service, "ath10k-wmi-ready-event", wmiReadyEventReceived ? 1 : 0);
    publishNumber(service, "ath10k-wmi-ready-status", wmiReadyStatus);
    publishNumber(service, "ath10k-vdev-ok", wmiVdevOk ? 1 : 0);
    publishString(service, "ath10k-vdev-status", wmiVdevStatus ? wmiVdevStatus : "unknown");
    publishNumber(service, "ath10k-vdev-tx-consumed", wmiVdevTxConsumed ? 1 : 0);
    publishNumber(service, "ath10k-vdev-tx-bytes", wmiVdevTxBytes);
    publishNumber(service, "ath10k-vdev-wait-ms", wmiVdevWaitMs);
    publishNumber(service, "ath10k-wmi-seq", wmiSeq);
    publishNumber(service, "ath10k-target-mac-valid", targetMacValid ? 1 : 0);
    {
        char macText[24];
        snprintf(macText, sizeof(macText), "%02x:%02x:%02x:%02x:%02x:%02x",
                 targetMac[0], targetMac[1], targetMac[2],
                 targetMac[3], targetMac[4], targetMac[5]);
        publishString(service, "ath10k-target-mac", macText);
    }
    // M1-c2 HTT RX
    publishNumber(service, "ath10k-htt-setup-ok", httSetupOk ? 1 : 0);
    publishString(service, "ath10k-htt-setup-status",
                  httSetupStatus ? httSetupStatus : "unknown");
    publishNumber(service, "ath10k-htt-setup-stage", httSetupStage);
    publishNumber(service, "ath10k-htt-version-received", httVersionReceived ? 1 : 0);
    publishNumber(service, "ath10k-htt-version-major", httVersionMajor);
    publishNumber(service, "ath10k-htt-version-minor", httVersionMinor);
    publishNumber(service, "ath10k-htt-messages", httMessages);
    publishNumber(service, "ath10k-htt-last-type", httLastType);
    publishNumber(service, "ath10k-htt-rx-posted", httRxRingPosted);
    publishNumber(service, "ath10k-htt-rx-pool-bytes",
                  static_cast<UInt64>(Ath10k::kHTTRxPosted) * Ath10k::kHTTRxBufferBytes);

    // M1-c1 / M1-c3 扫描结果（搬进独立函数：每轮扫描结束要单独刷新）
    publishScanResultsProperties();

    {
        char hex[sizeof(httLastPrefix) * 3 + 1];
        static const char digits[] = "0123456789abcdef";
        UInt32 pos = 0;
        for (UInt32 i = 0; i < httLastPrefixBytes && pos + 3 < sizeof(hex); ++i) {
            hex[pos++] = digits[httLastPrefix[i] >> 4];
            hex[pos++] = digits[httLastPrefix[i] & 15];
            hex[pos++] = ' ';
        }
        hex[pos] = 0;
        publishString(service, "ath10k-htt-last-prefix", pos ? hex : "(empty)");
        publishNumber(service, "ath10k-htt-last-prefix-bytes", httLastPrefixBytes);
    }

    publishString(service, "ath10k-wmi-tx-fail", wmiTxFail ? wmiTxFail : "none");
    publishNumber(service, "ath10k-wmi-tx-src-write", wmiTxSrcWrite);
    publishNumber(service, "ath10k-wmi-tx-src-read", wmiTxSrcRead);
    publishHex32(service, "ath10k-fw-indicator", fwIndicator);
    publishHex32(service, "ath10k-fw-indicator-baseline", fwIndicatorBaseline);
    publishNumber(service, "ath10k-firmware-crash", firmwareCrash ? 1 : 0);
    publishNumber(service, "ath10k-ce1-runtime-armed", runtimeRxArmed ? 1 : 0);

    // WMI 事件原始前缀（最多 48 字节的十六进制串）——解析失败时唯一的调试依据
    {
        char hex[sizeof(wmiEvtPrefix) * 3 + 1];
        static const char digits[] = "0123456789abcdef";
        UInt32 pos = 0;
        for (UInt32 i = 0; i < wmiEvtPrefixBytes && pos + 3 < sizeof(hex); ++i) {
            hex[pos++] = digits[wmiEvtPrefix[i] >> 4];
            hex[pos++] = digits[wmiEvtPrefix[i] & 15];
            hex[pos++] = ' ';
        }
        hex[pos] = 0;
        publishString(service, "ath10k-wmi-evt-prefix", pos ? hex : "(empty)");
        publishNumber(service, "ath10k-wmi-evt-prefix-bytes", wmiEvtPrefixBytes);
    }

    if (hwCandidate) {
        publishString(service, "ath10k-chip-name", hwCandidate->chipName);
        publishString(service, "ath10k-target-revision", hwCandidate->targetRevision);
        publishString(service, "ath10k-firmware-dir", hwCandidate->firmwareDirectory);
    } else {
        publishString(service, "ath10k-chip-name", "unmatched");
    }
}

// ---------------------------------------------------------------------------
// 信道表（M0 静态标准信道；M1 后由固件/board data 精化）
// ---------------------------------------------------------------------------
void Ath10kHal::
initChannelMap()
{
    struct ieee80211_channel *c;

    for (u_int ch = 1; ch <= 14; ch++) {
        c = &ic.ic_channels[ch];
        c->ic_freq  = ieee80211_ieee2mhz(ch, IEEE80211_CHAN_2GHZ);
        c->ic_flags = IEEE80211_CHAN_CCK | IEEE80211_CHAN_OFDM |
                      IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ |
                      IEEE80211_CHAN_HT20;
    }

    static const u_int ch5g[] = {
        36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112,
        116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165
    };
    for (u_int i = 0; i < (u_int)(sizeof(ch5g) / sizeof(ch5g[0])); i++) {
        const u_int ch = ch5g[i];
        c = &ic.ic_channels[ch];
        c->ic_freq  = ieee80211_ieee2mhz(ch, IEEE80211_CHAN_5GHZ);
        c->ic_flags = IEEE80211_CHAN_A | IEEE80211_CHAN_5GHZ |
                      IEEE80211_CHAN_HT20 | IEEE80211_CHAN_VHT80;
        if ((ch >= 52 && ch <= 64) || (ch >= 100 && ch <= 144))
            c->ic_flags |= IEEE80211_CHAN_DFS | IEEE80211_CHAN_PASSIVE;
    }
}

// ---------------------------------------------------------------------------
// net80211 回调（M1-c3 起接管 SCAN；其余状态仍交回协议栈）
// ---------------------------------------------------------------------------
int Ath10kHal::
ath_newstate(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
    struct _ifnet *ifp = &ic->ic_ac.ac_if;
    auto *self = static_cast<Ath10kHal *>(ifp->if_softc);
    if (!self) {
        IOLog("Ath10kHal: newstate=%d without softc（忽略）\n", (int)nstate);
        return 0;
    }
    return self->handleNewState(ic, nstate, arg);
}

int Ath10kHal::
handleNewState(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
    if (nstate == IEEE80211_S_SCAN) {
        // 全信道扫描由固件一次完成，这里只切状态并把重活排到 thread_call。
        // 调用方（WCL / net80211 的 next_scan）通常持有 workloop，不能阻塞。
        const enum ieee80211_state ostate = ic->ic_state;
        ic->ic_state = nstate;
        ieee80211_set_link_state(ic, LINK_STATE_DOWN);
        IOLog("Ath10kHal: newstate %d -> SCAN\n", (int)ostate);
        if (scanDisabled) {
            scanFeedStatus = scanSelfFused ? "disabled-self-fused" : "disabled-by-boot-arg";
            return 0;
        }
        // 开机静默期内不排队：状态已经切到 SCAN，静默期结束后由 1 Hz 定时器
        // 按"系统是否在请求扫描"决定是否开新一轮（见 scanTick）。
        if (scanTickCount < kScanGraceTicks) {
            scanFeedStatus = "waiting-grace-window";
            return 0;
        }
        if (!queueScanWork())
            return EIO;
        return 0;
    }

    // ---- M1-d：连接流程 ----
    // AUTH 之前先让固件建好 vdev/peer 并协商速率，否则认证帧发不出去；
    // RUN 之前再授权 peer + vdev up。全部有界、失败只记录不重试。
    if (nstate == IEEE80211_S_SCAN && ic->ic_state != IEEE80211_S_SCAN)
        joinTriggered = false;         // 本次连接请求结束，允许下一次触发

    if (nstate == IEEE80211_S_AUTH) {
        if (joinEnabled) {
            if (!runJoinPrepare())
                IOLog("Ath10kHal: JOIN prepare FAILED status=%s\n",
                      joinStatus ? joinStatus : "?");
            else
                joinStage = 3;
            startRxPump();          // auth/assoc 响应要靠它收
        }
    } else if (nstate == IEEE80211_S_RUN) {
        if (joinEnabled && !runJoinComplete())
            IOLog("Ath10kHal: JOIN complete FAILED status=%s\n",
                  joinStatus ? joinStatus : "?");
    }

    // INIT / ASSOC 交回 net80211 原状态机。
    IOLog("Ath10kHal: newstate -> %d（转交 net80211）\n", (int)nstate);
    if (defaultNewState)
        return defaultNewState(ic, nstate, arg);
    return 0;
}

int Ath10kHal::
ath_set_key(struct ieee80211com *ic, struct ieee80211_node *ni,
            struct ieee80211_key *k)
{
    return 0;
}

void Ath10kHal::
ath_delete_key(struct ieee80211com *ic, struct ieee80211_node *ni,
               struct ieee80211_key *k)
{
}

void Ath10kHal::
ath_start(struct _ifnet *ifp)
{
    ifp->if_flags &= ~IFF_OACTIVE;
    // net80211 的 IEEE80211_SEND_MGMT 会把帧放进 ic_mgtq 再调这里
    auto *self = static_cast<Ath10kHal *>(ifp->if_softc);
    if (self)
        self->serviceMgmtQueue();
}

// ---------------------------------------------------------------------------
// M1-d：管理帧发送（HTT TX / CE4）
//
// 认证/关联帧由 net80211 的 ieee80211_send_mgmt() 构造好放进 ic->ic_mgtq，
// 这里只负责搬运 + 发出去 + 等发送完成（HTT MGMT_TX_COMPLETION = 0x0e）。
// 同一时刻只允许一条在途帧：缓冲区是一条，且管理帧本来就少。
// ---------------------------------------------------------------------------
void Ath10kHal::
serviceMgmtQueue()
{
    if (!joinEnabled || shuttingDown || !attached)
        return;
    if (httTxOwned) {
        ++mgmtTxDropped;          // 上一条还没发完，丢掉；net80211 会重试
        joinStatus = "mgmt-tx-busy";
        return;
    }
    if (!txFrameBuffer.allocated() || !txFragBuffer.allocated() ||
        !txPacketBuffer.allocated())
        return;

    mbuf_t m = mq_dequeue(&ic.ic_mgtq);
    if (!m)
        return;
    const UInt32 length = static_cast<UInt32>(mbuf_len(m));
    if (length < 24 || length > kMgmtTxBufferBytes) {
        ++mgmtTxDropped;
        joinStatus = "mgmt-tx-length";
        mbuf_freem(m);
        return;
    }
    bcopy(mtod(m, void *), txFrameBuffer.virtualAddress(), length);
    mbuf_freem(m);
    submitMgmtFrame(static_cast<const UInt8 *>(txFrameBuffer.virtualAddress()), length);
}

bool Ath10kHal::
submitMgmtFrame(const UInt8 *frame, UInt32 length)
{
    if (!joinEnabled || !httEndpoint || httEndpoint == 0xff)
        return false;
    if (!ensureTargetAwake("mgmt-tx")) {
        ++mgmtTxFailed;
        joinStatus = "mgmt-tx-target-asleep";
        return false;
    }

    // fragment 数组：[u32 addr][u32 len]
    UInt8 *frag = static_cast<UInt8 *>(txFragBuffer.virtualAddress());
    bzero(frag, kMgmtTxFragmentBytes);
    Ath10k::htcWriteLE32(frag, static_cast<UInt32>(txFrameBuffer.dmaAddress()));
    Ath10k::htcWriteLE32(frag + 4, length);
    if (!txFrameBuffer.syncForDevice() || !txFragBuffer.syncForDevice()) {
        ++mgmtTxFailed;
        joinStatus = "mgmt-tx-sync";
        return false;
    }

    httTxIdCurrent = static_cast<UInt16>((httTxId + 1) % Ath10k::kWmiHttTxIdModulus);
    httTxId = httTxIdCurrent;
    httTxComplete = false;
    httTxStatus = 0;
    httTxOwned = true;

    UInt8 *cmd = static_cast<UInt8 *>(txPacketBuffer.virtualAddress());
    const UInt32 bytes = Ath10k::buildHTTPacket(
        cmd, kMgmtTxCommandBytes, frame, length,
        static_cast<UInt32>(txFrameBuffer.dmaAddress()),
        static_cast<UInt32>(txFragBuffer.dmaAddress()),
        httEndpoint, mgmtTxSeq++, httTxIdCurrent, true);
    if (!bytes) {
        httTxOwned = false;
        ++mgmtTxFailed;
        joinStatus = "mgmt-tx-build";
        return false;
    }

    const char *reason = nullptr;
    if (!ce4Submit(txPacketBuffer, bytes, &reason)) {
        httTxOwned = false;
        ++mgmtTxFailed;
        joinStatus = reason ? reason : "mgmt-tx-submit";
        return false;
    }
    ++mgmtTxSubmitted;

    // 等 HTT 完成事件（有界）
    UInt32 waited = 0;
    while (!httTxComplete && waited < kMgmtTxCompTimeoutMs && !shuttingDown) {
        bool gotEvent = false;
        if (!wmiPumpOnce(gotEvent))
            break;
        if (!gotEvent) {
            IOSleep(1);
            ++waited;
        }
    }
    httTxOwned = false;
    if (!httTxComplete) {
        ++mgmtTxFailed;
        joinStatus = "mgmt-tx-timeout";
        return false;
    }
    ++mgmtTxCompleted;
    if (httTxStatus) {
        ++mgmtTxFailed;
        joinStatus = "mgmt-tx-status";
        return false;
    }
    joinStatus = "mgmt-tx-ok";
    return true;
}

// CE4 提交：与 wmiSubmitCommand 同款"取模比较 + 账本空闲才重同步"，但有界。
bool Ath10kHal::
ce4Submit(Ath10kDMABuffer &command, UInt32 bytes, const char **reason)
{
    static constexpr UInt32 kTxCE = 4;
    if (reason)
        *reason = nullptr;
    const UInt32 entries = Ath10k::kCEHostConfig[kTxCE].srcEntries;
    if (!cePipesProgrammed || !httEndpoint || httEndpoint == 0xff ||
        !command.allocated() || !ceSrcRings[kTxCE].allocated() ||
        bytes < Ath10k::kHTCHeaderBytes || bytes > command.length()) {
        if (reason) *reason = "ce4-prerequisite";
        return false;
    }

    Ath10k::CESnapshot snap{};
    if (!Ath10k::readCESnapshot(regs, kTxCE, snap)) {
        if (reason) *reason = "ce4-snapshot";
        return false;
    }
    if (snap.srcWriteIndex == 0xdeadbeefu || snap.ctrl1 == 0xdeadbeefu) {
        if (reason) *reason = "ce4-asleep";
        return false;
    }
    if (!validateBMIPipeStructure(kTxCE, snap)) {
        if (reason) *reason = "ce4-structure";
        return false;
    }

    const UInt32 mask = entries - 1u;
    const UInt32 writeIdx = snap.srcWriteIndex & mask;
    const UInt32 readIdx = snap.currentSrcRead & mask;
    UInt32 slot;
    if (writeIdx != readIdx) {
        // CE4 只有这一条在途路径（管理帧），写≠读时按设备读索引重同步
        slot = readIdx;
        IOLog("Ath10kHal: CE4 resync raw w/r=%u/%u masked=%u/%u\n",
              snap.srcWriteIndex, snap.currentSrcRead, writeIdx, readIdx);
    } else {
        slot = writeIdx;
    }
    const UInt32 next = Ath10k::ceRingNext(slot, entries);

    auto *ring = reinterpret_cast<Ath10k::CEDescriptor32 *>(
        ceSrcRings[kTxCE].virtualAddress());
    auto *desc = &ring[slot];
    bzero(desc, sizeof(*desc));
    desc->address = static_cast<UInt32>(command.dmaAddress());
    desc->nbytes  = static_cast<UInt16>(bytes);
    desc->flags   = static_cast<UInt16>(httEndpoint << 2);
    OSSynchronizeIO();

    if (!command.syncForDevice() || !ceSrcRings[kTxCE].syncForDevice()) {
        if (reason) *reason = "ce4-sync";
        return false;
    }

    const UInt32 base = Ath10k::ceBaseAddress(kTxCE);
    UInt32 back = 0xffffffffu;
    if (!regs.write32(base + Ath10k::kCESrcWriteIndex, next) ||
        !regs.read32(base + Ath10k::kCESrcWriteIndex, back) || back != next) {
        if (reason) *reason = "ce4-doorbell";
        return false;
    }

    UInt32 waited = 0;
    for (;;) {
        OSSynchronizeIO();
        UInt32 srcRead = 0xffffffffu;
        if (!regs.read32(base + Ath10k::kCECurrentSrcRead, srcRead)) {
            if (reason) *reason = "ce4-src-read";
            return false;
        }
        if ((srcRead & mask) == next)
            return true;
        if (waited >= kMgmtTxCeTimeoutMs) {
            if (reason) *reason = "ce4-timeout";
            return false;
        }
        IOSleep(1);
        ++waited;
    }
}

// CE1 上到达的 HTT 消息
void Ath10kHal::
handleHTTMessage(const UInt8 *f, UInt32 bytes)
{
    if (!f || bytes < 12)
        return;
    const UInt8 type = f[8];
    if (type == 0x0e) {                      // MGMT_TX_COMPLETION
        if (bytes < 20)
            return;
        const UInt32 id = Ath10k::wmiReadLE32(f + 12);
        if (httTxOwned && id == static_cast<UInt32>(httTxIdCurrent)) {
            httTxStatus = Ath10k::wmiReadLE32(f + 16);
            httTxComplete = true;
        }
        return;
    }
    if (type == 0x0b) {                      // TX_COMPL_IND（数据面，M1-d 后续用）
        ++rxPumpEvents;
        return;
    }
}

// ---------------------------------------------------------------------------
// 运行期 RX 泵：只在连接/关联期间工作（ic_state >= AUTH），有界、可随时退出
// ---------------------------------------------------------------------------
void Ath10kHal::
rxPumpEntry(thread_call_param_t param0, thread_call_param_t)
{
    auto *self = static_cast<Ath10kHal *>(param0);
    if (self)
        self->runRxPump();
}

void Ath10kHal::
startRxPump()
{
    if (!joinEnabled || shuttingDown || !attached)
        return;
    if (rxPumpRunning)
        return;
    if (!rxCall) {
        rxCall = thread_call_allocate_with_options(rxPumpEntry, this,
                                                   THREAD_CALL_PRIORITY_USER,
                                                   THREAD_CALL_OPTIONS_ONCE);
        if (!rxCall) {
            joinStatus = "rx-pump-thread-unavailable";
            return;
        }
    }
    thread_call_enter(rxCall);
}

void Ath10kHal::
runRxPump()
{
    if (rxPumpRunning) {
        return;
    }
    rxPumpRunning = true;
    UInt32 idleLoops = 0;
    while (!shuttingDown && ic.ic_state >= IEEE80211_S_AUTH) {
        bool gotEvent = false;
        if (!wmiPumpOnce(gotEvent)) {
            ++rxPumpErrors;
            joinStatus = wmiEventStatus ? wmiEventStatus : "rx-pump-error";
            break;
        }
        if (gotEvent) {
            ++rxPumpEvents;
            idleLoops = 0;
        } else {
            if (++idleLoops > kRxPumpMaxIdleLoops)
                break;                        // 有界：最多约 60 秒空闲
            // 连接中（AUTH/ASSOC）要低延迟；已进 RUN 后降频，避免 500 Hz 空转
            IOSleep(ic.ic_state >= IEEE80211_S_RUN ? 20000 : kRxPumpIdleSleepUs);
        }
        // 把这一轮收到的管理帧（auth/assoc/deauth）喂进 net80211
        feedStagedScanFrames();
    }
    rxPumpRunning = false;
}

void Ath10kHal::
ath_watchdog(struct _ifnet *ifp)
{
}

// ---------------------------------------------------------------------------
// ItlHalService 接口
// ---------------------------------------------------------------------------
bool Ath10kHal::
attach(IOPCIDevice *device)
{
    pciNub         = device;
    attached       = false;
    bar0Map        = nullptr;
    hardwareProbed = false;
    hardwareStatus = "not-started";
    chipId         = 0xffffffffu;
    chipRevision   = 0xff;
    hwCandidate    = nullptr;
    chipIdAttempts = 0;
    pciDeviceId    = 0;
    pcieCapOffset  = 0;
    pcieLinkControlBefore = 0xffff;
    pcieLinkControlAfter  = 0xffff;
    aspmDisabled   = false;
    wakeAsserted   = false;
    rtcStateBefore = 0xffffffffu;
    rtcStateAfter  = 0xffffffffu;
    wakeDelayUs    = 0;
    wakeSucceeded  = false;
    wakeRetries    = 0;
    targetAwake    = false;
    lastRtcState   = 0xffffffffu;
    lastRtcReadable = false;
    globalResetBefore = 0xffffffffu;
    globalResetAfter  = 0xffffffffu;
    coldResetDone  = false;
    ringsAllocated = false;
    ringsFailure   = "not-run";
    ceConfiguredCount  = 0;
    ceVerifiedCount    = 0;
    cePipeInitPassCount = 0;
    cePipesProgrammed   = false;
    warmResetDone = false;
    warmResetControlBefore = 0xffffffffu;
    warmResetControlAfter  = 0xffffffffu;
    warmSI0Ok = warmCPU1Ok = warmCPU2Ok = warmLFok = warmCEok = false;
    warmInit1Ms = warmInit2Ms = 0;
    warmIndicator1 = warmIndicator2 = 0xffffffffu;
    postWarmChipId = 0xffffffffu;
    chipStableAfterWarm = false;
    ceDiagOk = false;
    ceDiagCoreCtrl = 0xffffffffu;
    ceDiagPCIeStateAddress  = 0xffffffffu;
    ceDiagPipeCfgAddress    = 0xffffffffu;
    ceDiagServiceMapAddress = 0xffffffffu;
    ceDiagConfigFlags       = 0xffffffffu;
    ceDiagTransactions      = 0;
    initConfigOk = false;
    initConfigOrigFlags = initConfigNewFlags = 0xffffffffu;
    initConfigOrigEarlyAlloc = initConfigNewEarlyAlloc = 0xffffffffu;
    initConfigOrigFlag2 = initConfigNewFlag2 = 0xffffffffu;
    initConfigBanks = 0;
    cpuWoken = false;
    coreCtrlBeforeWake = coreCtrlAfterWake = 0xffffffffu;
    bmiTransferUncertain = false;
    bmiTimedOut = false;
    bmiOk = false;
    bmiTargetLength = bmiTargetVersion = bmiTargetType = 0;
    bmiPrereqOk = false;
    hiBoardData = hiBoardDataInit = hiEarlyAlloc = 0xffffffffu;
    hiInterconnect = hiOptionFlag2 = hiCalData = 0xffffffffu;
    pllOk = false;
    bootHIOk = false;
    bootHIOptionBefore = bootHIOptionApplied = 0xffffffffu;
    hiWordsWritten = 0;
    payloadProfile = nullptr;
    pciSubVendorId = pciSubDeviceId = 0;
    payloadSelected = false;
    boardWriteOk = boardVerifyOk = false;
    boardBytes = 0;
    boardUploadOk = false;
    otpAttempted = otpStreamStarted = otpStreamClosed = otpExecuteAttempted = false;
    otpWireBytesSent = otpLZDataCommands = 0;
    otpDownloadOk = otpExecuteOk = otpResultAccepted = false;
    otpResult = 0xffffffffu;
    otpStatus = "not-run";
    fwAttempted = fwStreamStarted = fwStreamClosed = false;
    fwWireBytesSent = fwLZDataCommands = 0;
    fwDownloadOk = fwReadbackOk = false;
    fwReadbackBytes = fwVerifiedSegments = 0;
    fwFailedSegment = 0xffffffffu;
    fwPostVersion = fwPostType = 0xffffffffu;
    fwStatus = "not-run";
    uartDisabled = preDoneHIVerified = false;
    bootHIOptionAfterDownload = 0xffffffffu;
    bmiClosed = doneAttempted = doneSent = false;
    htcRxPosted = htcRxOwned = htcRxCompleted = htcReadyOk = false;
    htcRxSlot = htcRxNext = htcRxRead = htcRxWrite = 0xffffffffu;
    htcRxBytes = htcWaitMs = 0;
    htcFwIndicator = 0xffffffffu;
    htcPrefixBytes = 0;
    bzero(htcPrefix, sizeof(htcPrefix));
    bzero(&htcReady, sizeof(htcReady));
    htcStatus = "not-run";
    htcParseError = "none";

    // M1-a / M1-b1 / M1-b2 状态：探测前必须全部干净，
    // 否则 ioreg 会把上一次（或未执行阶段）的残值当成结果。
    htcServicesOk = false;
    htcServiceStatus = "not-run";
    wmiEndpoint = httEndpoint = 0xff;
    wmiMaxMessageSize = httMaxMessageSize = 0;
    htcCtrlTxCount = htcCtrlRxCount = 0;
    htcCtrlLastWaitMs = 0;
    runtimeRxSlot = runtimeRxNext = 0xffffffffu;
    wmiRxSlot = wmiRxNext = 0xffffffffu;
    wmiRxArmed = false;
    runtimeRxArmed = false;
    wmiRxBytes = wmiEventId = wmiWaitMs = 0;
    setupCompleteSent = false;
    wmiServiceAvailable = false;
    wmiServiceReady = false;
    wmiServiceBits = 0;
    bzero(wmiServiceBitmap, sizeof(wmiServiceBitmap));
    bzero(wmiServiceBitmapFull, sizeof(wmiServiceBitmapFull));
    wmiServiceBitmapPresent = false;
    wmiFwBuild = 0;
    bzero(wmiAbi, sizeof(wmiAbi));
    wmiAbiPresent = false;
    wmiRfChains = 0;
    wmiMemoryRequests = 0;
    bzero(wmiRegCaps, sizeof(wmiRegCaps));
    wmiRegCapsPresent = false;
    wmiEvtPrefixBytes = 0;
    bzero(wmiEvtPrefix, sizeof(wmiEvtPrefix));
    wmiStatus = "not-run";
    wmiParseError = "none";
    wmiEventStatus = "not-run";
    wmiCreditsReady = false;
    wmiCredits = Ath10k::WMICreditState();
    wmiEvent.reset();
    wmiBoot.reset();
    wmiInitOk = false;
    wmiReadyEventReceived = false;
    wmiReadyStatus = 0xffffffffu;
    wmiInitTxBytes = wmiInitWaitMs = 0;
    wmiHostMemoryCount = wmiHostMemoryBytes = 0;
    wmiVdevOk = false;
    wmiVdevTxConsumed = false;
    wmiVdevTxBytes = wmiVdevWaitMs = 0;
    wmiSeq = 0;
    wmiTxSlot = wmiTxNext = 0xffffffffu;
    wmiTxOwned = false;
    wmiTxConsumed = false;
    wmiTxSrcWrite = wmiTxSrcRead = 0xffffffffu;
    wmiTxSrcWriteRaw = wmiTxSrcReadRaw = 0xffffffffu;
    ce3Resyncs = 0;
    wmiTxFail = "none";
    fwIndicator = 0xffffffffu;
    fwIndicatorBaseline = 0xffffffffu;
    firmwareCrash = false;
    bzero(targetMac, sizeof(targetMac));
    targetMacValid = false;
    wmiInitStatus = "not-run";
    wmiVdevStatus = "not-run";

    // M1-c2 HTT RX
    httSetupOk = false;
    httSetupStage = 0;
    httSetupStatus = "not-run";
    httVersionReceived = false;
    httVersionMajor = httVersionMinor = 0;
    httMessages = 0;
    httLastType = 0xffu;
    httLastPrefixBytes = 0;
    bzero(httLastPrefix, sizeof(httLastPrefix));
    httRxRingPosted = 0;
    httSeq = 0;
    httTxSlot = httTxNext = 0xffffffffu;
    httTxConsumed = false;

    // M1-c1 扫描
    pdevSetupOk = false;
    scanSetupOk = false;
    scanOk = false;
    scanStarted = false;
    scanCompleted = false;
    scanEvents = scanMgmtFrames = 0;
    scanReason = 0xffffffffu;
    scanLastFreq = 0;
    scanWaitMs = 0;
    scanBssCount = 0;
    pdevSetupStatus = "not-run";
    scanStatus = "not-run";
    scanInfo = Ath10k::WMIScanInfo();
    bzero(scanBss, sizeof(scanBss));
    scanEvStarted = scanEvCompleted = scanEvBssChan = 0;
    scanEvForeignChan = scanEvOther = 0;
    ce1Frames = ce1LastEndpoint = ce1LastBytes = 0;

    // M1-c3 扫描 → net80211
    defaultNewState = nullptr;
    scanCall = nullptr;
    scanTimer = nullptr;
    scanWorkQueued = false;
    scanWorkRunning = false;
    scanWorkRerun = false;
    scanDisabled = false;
    scanSelfFused = false;
    scanTickCount = 0;
    scanNextAllowedTick = 0;
    scanFailStreak = 0;
    scanRoundStartTick = 0;
    scanRequestSeen = false;
    scanRequestRounds = 0;
    scanRounds = 0;
    scanFeedAttempts = 0;
    scanFeedOk = 0;
    scanFeedSkipped = 0;
    scanNodes = 0;
    scanDoneEvents = 0;
    scanFeedStatus = "not-run";
    scanStageCount = 0;
    scanStageOverflow = 0;
    bzero(scanStage, sizeof(scanStage));
    driverReadySent = false;

    // M1-d 连接
    joinEnabled = true;
    joinStatus = "not-run";
    joinCommands = 0;
    joinCommandFails = 0;
    joinStage = 0;
    mgmtTxSubmitted = mgmtTxCompleted = mgmtTxFailed = mgmtTxDropped = 0;
    joinFeedOk = joinFeedSkipped = 0;
    httTxId = 0;
    httTxIdCurrent = 0;
    httTxOwned = httTxComplete = false;
    httTxStatus = 0;
    httSeq = 0;
    rxPumpRunning = false;
    feedingFrames = false;
    joinTriggerPending = false;
    joinTriggered = false;
    joinNextAllowedTick = 0;
    macReconfigAcks = 0;
    macReconfigNextTick = 0;
    rxCall = nullptr;
    rxPumpEvents = rxPumpErrors = 0;

    bringUpCall = nullptr;
    bringUpRunning = false;
    shuttingDown = false;

    // AirportItlwm::start() 在 attach() 之前就写好了这两个字段，
    // 而下面要整体 bzero(ic)——必须先备份再恢复，否则：
    //   ic_event_handler 被清空 → WCL 永远收不到 BEACON/SCAN_DONE →
    //   系统 Wi-Fi 列表永远为空（M1-c3 r3 真机就是这样：驱动侧 174 帧、
    //   15 个节点全对，但 airport -s 什么都没有、scan-done-events = 0）。
    void (*presetEventHandler)(struct ieee80211com *, int, void *) = ic.ic_event_handler;
    const UInt32 presetUserFlags = ic.ic_userflags;

    bzero(&ic, sizeof(ic));
    ic.ic_event_handler = presetEventHandler;
    ic.ic_userflags = presetUserFlags;

    // M0 占位 MAC（M0.1c 固件启动后改为 OTP 回读值）
    placeholderMac[0] = 0x02;
    placeholderMac[1] = 0x11;
    placeholderMac[2] = 0x22;
    placeholderMac[3] = 0x33;
    placeholderMac[4] = 0x44;
    placeholderMac[5] = 0x55;
    IEEE80211_ADDR_COPY(ic.ic_myaddr, placeholderMac);

    // bzero 后 ic_opmode == 0，在 IEEE80211_STA_ONLY 构建里那是个非法值
    // （IBSS 枚举被编译掉了），必须显式声明为基础设施站点模式。
    ic.ic_opmode = IEEE80211_M_STA;
    ic.ic_caps = IEEE80211_C_WEP | IEEE80211_C_RSN |
                 IEEE80211_C_SCANALL | IEEE80211_C_SCANALLBAND |
                 IEEE80211_C_SHSLOT | IEEE80211_C_SHPREAMBLE |
                 IEEE80211_C_QOS | IEEE80211_C_TX_AMPDU;
    ic.ic_htcaps = IEEE80211_HTCAP_SGI20 |
                   (IEEE80211_HTCAP_SMPS_DIS << IEEE80211_HTCAP_SMPS_SHIFT);
    ic.ic_sup_rates[IEEE80211_MODE_11A] = ieee80211_std_rateset_11a;
    ic.ic_sup_rates[IEEE80211_MODE_11B] = ieee80211_std_rateset_11b;
    ic.ic_sup_rates[IEEE80211_MODE_11G] = ieee80211_std_rateset_11g;
    ic.ic_max_rssi   = 90;
    ic.ic_ibss_chan  = &ic.ic_channels[1];
    ic.ic_curmode    = IEEE80211_MODE_AUTO;

    initChannelMap();

    struct _ifnet *ifp = &ic.ic_ac.ac_if;
    ifp->if_softc    = this;
    ifp->if_flags    = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
    ifp->if_ioctl    = ieee80211_ioctl;
    ifp->if_start    = ath_start;
    ifp->if_watchdog = ath_watchdog;
    memcpy(ifp->if_xname, "ath0", sizeof("ath0"));

    // NB: ic_newstate 要等 ieee80211_ifattach 之后再装（proto_attach 会覆盖），
    //     见本函数末尾的 defaultNewState 保存处。
    ic.ic_set_key    = ath_set_key;
    ic.ic_delete_key = ath_delete_key;

    if_attach(ifp);
    ieee80211_ifattach(ifp, getController());
    ieee80211_media_init(ifp);

    // ieee80211_proto_attach（在 ifattach 里）会把 ic_newstate 换成 net80211
    // 自己的状态机，所以 ath_newstate 必须在这之后重新装上，否则永远不会
    // 被调用（M1-c3 排查时发现的老问题）。
    defaultNewState  = ic.ic_newstate;
    ic.ic_newstate   = ath_newstate;

    attached = true;

    // 熔断开关：boot-arg "ath10k_scan=0" 可整体关闭 M1-c3 扫描，
    // 用于在不重新编译的情况下排除扫描/喂帧路径（见 docs/13）。
    {
        int scanArg = 1;
        if (PE_parse_boot_argn("ath10k_scan", &scanArg, sizeof(scanArg)))
            scanDisabled = (scanArg == 0);
        if (scanDisabled)
            IOLog("Ath10kHal: M1-c3 scanning disabled by boot-arg ath10k_scan=0\n");
    }

    // 1 Hz 扫描轮询（按请求 + 退避，见 runScanWork 注释）
    startScanTimer();

    // M1-d：连接用的 DMA 缓冲 + 开关（boot-arg ath10k_join=0 可整体关闭）
    {
        int joinArg = 1;
        if (PE_parse_boot_argn("ath10k_join", &joinArg, sizeof(joinArg)))
            joinEnabled = (joinArg != 0);
        if (!joinEnabled)
            IOLog("Ath10kHal: M1-d join disabled by boot-arg ath10k_join=0\n");
        if (joinEnabled) {
            const bool ok =
                txFrameBuffer.allocate(kMgmtTxBufferBytes, Ath10k::kCEDescriptorRingAlign, nullptr) &&
                txFragBuffer.allocate(kMgmtTxFragmentBytes, Ath10k::kCEDescriptorRingAlign, nullptr) &&
                txPacketBuffer.allocate(kMgmtTxCommandBytes, Ath10k::kCEDescriptorRingAlign, nullptr);
            if (!ok) {
                joinEnabled = false;
                joinStatus = "join-buffer-alloc-failed";
                IOLog("Ath10kHal: M1-d TX buffers alloc FAILED, join disabled\n");
            } else {
                IOLog("Ath10kHal: M1-d TX buffers ready (frame=%u frag=%u cmd=%u)\n",
                      kMgmtTxBufferBytes, kMgmtTxFragmentBytes, kMgmtTxCommandBytes);
            }
        }
    }

    // ===== 异步 bring-up =====
    // 固件阶段（OTP + 727 KB 固件 + 回读校验）耗时 10 s 量级，
    // 不能阻塞 attach() —— 那会直接拖住 IOConfigThread 导致开机卡顿。
    bringUpCall = thread_call_allocate_with_priority(bringUpCallEntry, this,
                                                     THREAD_CALL_PRIORITY_USER);
    if (bringUpCall) {
        thread_call_enter(bringUpCall);
        IOLog("Ath10kHal: attach done; bring-up queued asynchronously\n");
    } else {
        IOLog("Ath10kHal: thread_call unavailable; bring-up runs synchronously\n");
        probeHardware();
        publishHardwareProperties();
    }
    return true;
}

void Ath10kHal::
detach(IOPCIDevice *device)
{
    shuttingDown = true;
    if (bringUpCall) {
        thread_call_cancel(bringUpCall);
        thread_call_free(bringUpCall);
        bringUpCall = nullptr;
    }
    stopScanTimer();

    if (rxCall) {
        thread_call_cancel(rxCall);
        thread_call_free(rxCall);
        rxCall = nullptr;
    }

    if (scanCall) {
        // 先取消再等待：worker 每轮都会检查 shuttingDown，这里保证它真的退出后
        // 才继续 ifdetach / releaseAll（否则会释放它正在读的 DMA 缓冲）。
        thread_call_cancel_wait(scanCall);
        thread_call_free(scanCall);
        scanCall = nullptr;
    }
    for (UInt32 i = 0; i < 500 && scanWorkRunning; ++i)
        IOSleep(10);

    // 拆除前释放唤醒、恢复 ASPM（保持断言的那份由这里收尾）
    if (ic.ic_event_handler && driverReadySent && !shuttingDown)
        ic.ic_event_handler(&ic, IEEE80211_EVT_DRIVER_STOPPED, NULL);
    releaseTargetWake();
    restoreASPM();

    if (attached) {
        ieee80211_ifdetach(&ic.ic_ac.ac_if);
        attached = false;
    }
    releaseAll();
}

IOReturn Ath10kHal::
enable(IONetworkInterface *netif)
{
    struct _ifnet *ifp = &ic.ic_ac.ac_if;
    if (ifp->if_flags & IFF_UP)
        return kIOReturnSuccess;
    // IFF_RUNNING 必须一起置位：WCL 的扫描请求入口
    // （AirportItlwmSkywalkInterface::beginWCLScan）第一道门就检查它，
    // 只置 IFF_UP 会让所有扫描请求直接返回 kIOReturnNotReady。
    ifp->if_flags |= (IFF_UP | IFF_RUNNING);
    IOLog("Ath10kHal: enable() -> IFF_UP|IFF_RUNNING\n");
    return kIOReturnSuccess;
}

IOReturn Ath10kHal::
disable(IONetworkInterface *netif)
{
    struct _ifnet *ifp = &ic.ic_ac.ac_if;
    ifp->if_flags &= ~(IFF_UP | IFF_RUNNING);
    // 注意：这里**故意不发** IEEE80211_EVT_DRIVER_STOPPED。
    // disable() 在系统关机时也会被 setPOWER 调用，而 WCL 收到该事件会走
    // finishWCLScan/finishWCLJoin 并向用户态 postMessage；在关机竞态里投递
    // 可能阻塞，进而让关机不干净（真机 2026-11-18 00:43 那次开机卡在
    // AppleNvme Assert 就怀疑是这条链）。停机通知只在 detach() 里发。
    return kIOReturnSuccess;
}

struct ieee80211com *Ath10kHal::
get80211Controller()
{
    return &ic;
}

ItlDriverInfo *Ath10kHal::
getDriverInfo()
{
    return this;
}

ItlDriverController *Ath10kHal::
getDriverController()
{
    return this;
}

void Ath10kHal::
free()
{
    IOLog("%s\n", __FUNCTION__);
    super::free();
}

void Ath10kHal::
releaseAll()
{
    httRxPool.release();
    httRxRing.release();
    httRxIndex.release();
    httTxBuffer.release();
    for (UInt32 i = 0; i < Ath10k::kWMIHostMemoryMax; ++i)
        wmiHostMemory[i].release();
    wmiTxBuffer.release();
    wmiRxBuffer.release();
    htcControlBuffer.release();
    htcCtrlRxBuffer.release();
    htcReadyBuffer.release();
    bmiRequestBuffer.release();
    bmiResponseBuffer.release();
    ceDiagBounce.release();
    releaseCERings();
    regs.detach();
    if (bar0Map) {
        bar0Map->release();
        bar0Map = nullptr;
    }
    pciNub = nullptr;
}

// ---------------------------------------------------------------------------
// ItlDriverInfo 接口
// ---------------------------------------------------------------------------
const char *Ath10kHal::
getFirmwareVersion()
{
    return "hw-probe";
}

int16_t Ath10kHal::
getBSSNoise()
{
    return -96;
}

bool Ath10kHal::
is5GBandSupport()
{
    return true;
}

int Ath10kHal::
getTxNSS()
{
    return 1;
}

const char *Ath10kHal::
getFirmwareName()
{
    if (hwCandidate && hwCandidate->chipName)
        return hwCandidate->chipName;
    return "Qualcomm Atheros QCA9377";
}

UInt32 Ath10kHal::
supportedFeatures()
{
    return kIONetworkFeatureMultiPages;
}

const char *Ath10kHal::
getFirmwareCountryCode()
{
    return "US";
}

uint32_t Ath10kHal::
getTxQueueSize()
{
    return 256;
}

// ---------------------------------------------------------------------------
// ItlDriverController 接口
// ---------------------------------------------------------------------------
void Ath10kHal::
clearScanningFlags()
{
}

IOReturn Ath10kHal::
setMulticastList(IOEthernetAddress *addr, int count)
{
    return kIOReturnSuccess;
}