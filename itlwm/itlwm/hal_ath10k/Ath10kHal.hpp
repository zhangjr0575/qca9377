/*
 * Ath10kHal.hpp
 *
 * QCA9377 (ath10k) 原生 WiFi 的 net80211 HAL。
 *
 * 里程碑进度：
 *   - M0   ✅ 走通骨架：net80211 附着 + 原生 Wi-Fi 接口可见
 *   - M0.1a ✅ 硬件探测：BAR0 映射 + 芯片 ID/版本识别（只读，无固件操作）
 *   - M0.1b ✅ 目标唤醒 + 冷复位 + 芯片 ID 复读
 *   - M0.1c ✅ CE ring / 热复位 / BMI（board + OTP + 固件）/ HTC READY
 *   - M1-a  ✅ HTC 服务建立（HTT + WMI 端点）
 *   - M1-b1 ✅ WMI bootstrap（SERVICE_AVAILABLE + SERVICE_READY）
 *   - M1-b2 🔄 WMI INIT + host memory + vdev create + 真实 MAC
 *   - M1-c~M4 ⏳ 扫描 / 连接 / Apple RSN / 打磨
 *
 * 设计约束：
 *   - 硬件层复用 Ath10kMac 已验证的独立工具类（hw/ 目录，零改动）；
 *   - 所有日志使用 IOLog（而非 itlwm 的 XYLog/kprintf），确保能进 unified log；
 *   - 关键状态通过 setProperty 发布到 ioreg，便于无日志时验证；
 *   - M0.1a 阶段只做只读探测，绝不写硬件寄存器，避免 panic。
 */

#ifndef Ath10kHal_hpp
#define Ath10kHal_hpp

#include <compat.h>

#include <HAL/ItlHalService.hpp>
#include <HAL/ItlDriverInfo.hpp>
#include <HAL/ItlDriverController.hpp>

// 复用的 Ath10kMac 硬件层（自包含，仅依赖 IOKit）
#include "hw/Ath10kHw.hpp"
#include "hw/Ath10kTarget.hpp"
#include "hw/Ath10kRegisterIO.hpp"
// M0.1c 预备：CE / DMA 复用层（本里程碑开始接入）
#include "hw/Ath10kCE.hpp"
#include "hw/Ath10kCERing.hpp"
#include "hw/Ath10kBMI.hpp"
#include "hw/Ath10kPLL.hpp"
#include "hw/Ath10kBoot.hpp"
#include "hw/Ath10kWmiTlv.hpp"
#include "hw/Ath10kWmiScan.hpp"
#include "hw/Ath10kWmiConnect.hpp"   // M1-d：连接命令 + HTT TX 描述符
#include "hw/Ath10kHtt.hpp"
// payload 声明（仅类型；14 MB 的二进制数组只在 Ath10kHal.cpp 中编译一次）
#include "payload/Ath10kPayloadProfile.hpp"

class Ath10kHal : public ItlHalService, ItlDriverInfo, ItlDriverController {
    OSDeclareDefaultStructors(Ath10kHal)

public:
    // ---- ItlHalService ----
    virtual bool attach(IOPCIDevice *device) override;
    virtual void detach(IOPCIDevice *device) override;
    virtual IOReturn enable(IONetworkInterface *netif) override;
    virtual IOReturn disable(IONetworkInterface *netif) override;
    virtual struct ieee80211com *get80211Controller() override;
    virtual ItlDriverInfo *getDriverInfo() override;
    virtual ItlDriverController *getDriverController() override;
    void free() override;

    // ---- 板卡匹配（QCA9377: PCI 168c:0042）----
    static bool ath10k_match(IOPCIDevice *device);

    // ---- ItlDriverInfo ----
    virtual const char *getFirmwareVersion() override;
    virtual int16_t getBSSNoise() override;
    virtual bool is5GBandSupport() override;
    virtual int getTxNSS() override;
    virtual const char *getFirmwareName() override;
    virtual UInt32 supportedFeatures() override;
    virtual const char *getFirmwareCountryCode() override;
    virtual uint32_t getTxQueueSize() override;

    // ---- ItlDriverController ----
    virtual void clearScanningFlags() override;
    virtual IOReturn setMulticastList(IOEthernetAddress *addr, int count) override;

    void releaseAll();

private:
    // 用静态信道表填充 ic_channels（M1 后由固件/board data 精化）
    void initChannelMap();

    // ===== M0.1a 硬件探测（只读）=====
    // 映射 BAR0 并读取芯片标识。返回 false 不影响接口发布，仅记录原因。
    bool probeHardware();
    // 把探测结果写入控制器属性，便于 ioreg 验证
    void publishHardwareProperties();

    // ===== M0.1b 目标唤醒与冷复位 =====
    // 关闭 PCIe ASPM（复位窗口内链路不稳会导致唤醒失败）
    bool disableASPM();
    void restoreASPM();
    // 断言 SOC_WAKE 并轮询 RTC_STATE 直到 RTC_STATE_V_ON
    bool wakeTarget();
    // 运行期"确保 target 醒着"：RTC_STATE 不是 ON 就重新断言 SOC_WAKE。
    // M1-c3 真机教训：bring-up 结束就 releaseTargetWake() 会让 SOC 睡下去，
    // 之后任何 CE 寄存器读都返回 0xDEADBEEF（Linux 是 power_up 后一直保持断言）。
    bool ensureTargetAwake(const char *stage);
    void releaseTargetWake();
    // 冷复位：拉高/拉低 SOC_GLOBAL_RESET
    bool coldResetTarget();
    // 读取 SOC_CHIP_ID；返回 false 表示不可读
    bool readChipIdValue(UInt32 &value);
    // 判断读到的是否为哨兵值（芯片未就绪）
    static bool isChipIdSentinel(UInt32 value);

    // ===== M0.1c-1 CE ring DMA 分配与管道编程 =====
    bool allocateCERings();
    void releaseCERings();
    bool initOneCopyEnginePipe(UInt32 ceId, const char *stage);
    bool initCopyEnginePipes(const char *stage);
    bool verifyCopyEnginePipe(UInt32 ceId, const Ath10k::CESnapshot &snap) const;
    bool deinitOneCopyEnginePipe(UInt32 ceId);
    bool deinitCopyEnginePipes(const char *stage);

    // ===== M0.1c-3 CE7 诊断通道（向 target 读写内存）=====
    // target 地址 → CE7 诊断窗口地址（依赖 CORE_CTRL 的 window 字段）
    bool targetCpuToCEAddress(UInt32 targetAddress, UInt32 &ceAddress);
    // 经 CE7 从 target 内存读 nbytes 字节（4 字节对齐，<= 2048）
    bool ce7DiagnosticRead(UInt32 targetAddress, void *outData, UInt32 nbytes);
    // 只读发现：hi_interconnect_state → pcie_state → pipe_cfg / svc_map / config_flags
    bool runCE7DiagnosticRead();
    // 经 CE7 向 target 内存写 nbytes 字节
    bool ce7DiagnosticWrite(UInt32 targetAddress, const void *data, UInt32 nbytes);
    // init_config：写 pipe_cfg / svc_map / config_flags / early_alloc / option_flag2
    bool runCE7InitConfig();

    // ===== M0.1c-5 CPU 唤醒与 BMI 传输 =====
    // CORE_CTRL |= 0x2000 唤醒 target CPU
    bool wakeTargetCPUForBMI();
    // BMI 管道结构校验（base/size/dmax/byteswap，不查 IE/watermark）
    bool validateBMIPipeStructure(UInt32 ceId, const Ath10k::CESnapshot &snap) const;
    // 发送一条 BMI 命令并等待响应（CE0 发送 / CE1 接收）
    bool bmiExchange(const void *request, UInt32 requestLength,
                     void *response, UInt32 responseLength);
    // BMI_GET_TARGET_INFO，校验 version/type/length
    bool runBMIGetTargetInfo();

    // ===== M0.1c-6 BMI 内存访问 / PLL / Boot Host Interest =====
    // 只发不等的 BMI 命令（WRITE_MEMORY / LZ_* / DONE）
    bool bmiSendOnly(const void *request, UInt32 requestLength);
    // BMI 读写 target 物理内存（按 256 B 分块）
    bool bmiReadMemory(UInt32 targetAddress, void *outData, UInt32 length);
    bool bmiWriteMemory(UInt32 targetAddress, const void *data, UInt32 length);
    // BMI 读写 SOC 寄存器（PLL 用）
    bool bmiReadSOCRegister(UInt32 address, UInt32 &value);
    bool bmiWriteSOCRegister(UInt32 address, UInt32 value);
    bool bmiWritePLLWord(UInt32 address, UInt32 value);
    // 前置探测：读 Host Interest 若干字并校验
    bool runBMIBoardPrerequisiteProbe();
    // 176 MHz PLL（hw1.1）
    bool configureTargetPLL();
    // Boot Host Interest 五字写入 + 立即回读
    bool writeBootHIWord(UInt32 address, UInt32 value);
    bool verifyBootHIWord(UInt32 address, UInt32 value);
    bool configureBootHostInterest();

    // ===== M0.1c-7 board data 上传 =====
    bool runBMIBoardUpload();

    // ===== M0.1c-8 OTP 下载与执行 =====
    bool bmiLZStreamStart(UInt32 address);
    bool bmiLZData(const UInt8 *data, UInt32 length);
    bool runBMIOtp();

    // ===== M0.1c-9 WLAN 固件下载 =====
    bool bmiFirmwareLZStart(UInt32 address);
    bool bmiFirmwareLZData(const UInt8 *data, UInt32 length);
    bool verifyFirmwareSegments();
    bool runBMIFirmwareDownload();

    // ===== M0.1c-10 Boot HI 收尾 / BMI_DONE / HTC READY =====
    bool finishBootHostInterest();
    bool verifyBootHIOptionsAfterDownload();
    bool prepareHTCReadyReceive();
    bool bmiSendDone();
    bool waitForHTCReady();

    // ===== M1-a HTC 服务建立 =====
    // 在 CE1 当前 dstWriteIndex 挂运行时 RX 描述符（仅首次）
    bool armRuntimeRx();
    // 收到一包后把 CE1 描述符推进到下一槽位
    bool htcRuntimeRepostRx();
    // 保证 CE1 已挂描述符（首次 arm / 后续 repost）
    bool htcEnsureRuntimeRx();
    // 非阻塞消费 CE1 上的包，只处理 HTC trailer 的信用回报
    bool htcRuntimePump(bool &hasPacket);
    // 经 CE0/CE1 提交一条 HTC 控制帧并等待响应
    bool htcSubmitControlFrame(const UInt8 *request, UInt32 requestBytes,
                               UInt16 expectedMessageId,
                               UInt8 *response, UInt32 &responseBytes);
    // 连接 HTT / WMI 服务
    bool runHTCServiceSetup();

    // ===== M1-b1 WMI bootstrap（SERVICE_READY）=====
    // CE2 首次挂 WMI 接收描述符
    bool armWmiRx();
    // 收到事件后把 CE2 描述符推进到下一个槽位
    bool wmiRepostRx();
    // 保证 CE2 已挂描述符（未挂则 arm 或 repost）
    bool wmiEnsureRxArmed();
    // 只发送不等待的 HTC 控制帧（SETUP_COMPLETE 走这条）
    bool htcSendControlFrame(const UInt8 *frame, UInt32 bytes);
    // 非阻塞推进一次 WMI 管线：CE2 事件 + CE1 信用回报 + 固件心跳
    bool wmiPumpOnce(bool &gotEvent);
    // 读并发布 FW_INDICATOR（EVENT_PENDING 表示固件崩溃）
    bool sampleFirmwareIndicator();
    // 把 wmiEvent 合并进 HAL 状态
    void wmiApplyEvent();
    // 发送 SETUP_COMPLETE 并等待 SERVICE_READY(id=1)
    bool runWMIBootstrap();

    // ===== M1-b2 WMI INIT + vdev create =====
    // 初始化 WMI 端点信用账本（用 HTC READY 协商值）
    bool wmiInitCredits();
    // 经 CE3 提交 wmiTxBuffer 里的命令并等 CE3 消费
    bool wmiSubmitCommand(UInt32 bytes, UInt32 &waitedMs);
    // host memory 分配 + WMI INIT + 等 READY(2)
    bool runWMIInit();
    // vdev create（STA 模式）
    bool runWMIVdevCreate();
    // 把固件上报的 target MAC 应用到 net80211
    void applyTargetMac();

    // ===== M1-c2 HTT RX 环 =====
    // VERSION_REQ → RX_RING_CFG → AGGR_CFG，内存：index/ring/pool
    bool runHTTRxSetup();
    // 经 CE4 提交一条 HTT 消息并等 CE4 消费（期间继续收 CE1/CE2 事件）
    bool httSubmitCommand(UInt32 bytes, UInt32 timeoutMs);
    // 等固件的 VERSION_CONF 回复
    bool httWaitVersionReply(UInt32 timeoutMs);

    // ===== M1-c1 扫描命令通路 =====
    // PDEV chain mask（1x1）
    bool runWMIPdevSetup();
    // 把完整信道表交给固件
    bool runWMIScanSetup();
    // START_SCAN + 收集扫描事件与 MGMT_RX
    bool runWMIScan();
    // 提交一条 WMI 命令并等信用归还（顺带收事件）
    bool wmiSubmitAndReclaim(UInt32 bytes, UInt32 reclaimTimeoutMs);
    // 合并一条扫描事件 / 管理帧
    void applyScanInfo();

    // ===== M1-c3 扫描结果接入 net80211 =====
    // ath_newstate 的实体（静态回调里取 if_softc 后转进来）
    int  handleNewState(struct ieee80211com *, enum ieee80211_state, int);
    // 排一次异步扫描（thread_call，绝不在调用方上下文里跑）
    bool queueScanWork();
    // 异步扫描主体：前置命令 → WMI 扫描 → 喂帧 → 发 SCAN_DONE（不做 end_scan）
    void runScanWork();
    static void scanWorkEntry(thread_call_param_t param0, thread_call_param_t param1);
    // 1 Hz 轮询：只有系统真的在请求扫描、且过了退避窗口时才排新一轮。
    // r0 用 end_scan → next_scan 自驱重扫，失败时会毫秒级自旋拖死整机，
    // r1 改成这种"按请求 + 退避"的驱动方式。
    void scanTick();
    static void scanTimerAction(OSObject *owner, IOTimerEventSource *timer);
    // 创建/销毁扫描轮询定时器（挂在主 workloop 上）
    void startScanTimer();
    void stopScanTimer();
    // 把一帧 beacon/probe response 暂存下来（扫描线程里只做拷贝）
    void stageScanFrame(const Ath10k::WMIScanInfo &info);
    // 喂暂存帧（在扫描线程直接调用，与 iwx 的 RX task 上下文一致）
    void feedStagedScanFrames();
    // 一轮扫描的收尾：清节点 → 喂帧 → 发 SCAN_DONE → 更新退避
    void runScanCompletion();
    // 单帧喂入实现（不持 workloop 锁；内部 if_input 会自行取 _fCommandGate）
    bool feedScanFrameTo80211(const UInt8 *frame, UInt32 length, UInt32 freq, UInt32 snr);
    // 选一个合法信道指针（net80211 的 next_scan 依赖 ic_bss->ni_chan）
    struct ieee80211_channel *firstActiveScanChannel();
    // M1-c3 轻量属性（每轮刷新，不碰整组 publishHardwareProperties）
    void publishScan80211Properties();
    void publishScanResultsProperties();
    // 通过 ic_event_handler 告诉 WCL「驱动已就绪」（只发一次）
    void emitDriverReady(bool force = false);

    // ===== M1-d 连接（Association）=====
    // 只在"用户选中某个网络"后（ic_state 进到 AUTH 及以后）才激活，
    // 默认扫描/菜单路径完全不受影响。boot-arg ath10k_join=0 可整体关闭。
    // 认证/关联帧由 net80211 放进 ic->ic_mgtq，驱动用 HTT TX(CE4) 发出。
    void serviceMgmtQueue();
    bool submitMgmtFrame(const UInt8 *frame, UInt32 length);
    // CE4 单命令提交（HTT 描述符），有界等待发送完成
    bool ce4Submit(Ath10kDMABuffer &command, UInt32 bytes, const char **reason);
    // CE1 上的 HTT 消息（0x0e = MGMT_TX_COMPLETION，0x0b = TX_COMPL_IND）
    void handleHTTMessage(const UInt8 *f, UInt32 bytes);
    // 运行期 RX 泵：连接/关联期间持续收 WMI 事件与 HTT 消息
    void startRxPump();
    void runRxPump();
    static void rxPumpEntry(thread_call_param_t param0, thread_call_param_t param1);
    // 连接命令链：VDEV_START → PEER_CREATE → PEER_ASSOC（进 AUTH 时）
    bool runJoinPrepare();
    // 连接收尾：PEER_AUTHORIZE → VDEV_UP（进 RUN 时）
    bool runJoinComplete();
    bool submitConnectCommand(const UInt8 *frame, UInt32 bytes, const char *stage);
    bool buildConnectParams(const char *stage, Ath10k::WmiConnectParams &c);

    // ===== 异步 bring-up =====
    // 固件阶段整体耗时 10s 量级，必须离开 attach() 的同步上下文
    void runBringUp();
    static void bringUpCallEntry(thread_call_param_t param0, thread_call_param_t param1);

    // ===== M0.1c-2 热复位 =====
    bool maskTargetIRQsForWarmReset();
    bool warmResetSI0();
    bool warmResetCPU(const char *stage);
    bool warmResetClearLF();
    bool warmResetCE();
    // 轮询 FW_INDICATOR 直到已初始化；输出耗时与最终值
    bool waitForTargetInit(UInt32 &elapsedMs, UInt32 &indicator);
    bool warmResetTarget();

    // net80211 回调占位（M1~M4 逐步替换）
    static int  ath_newstate(struct ieee80211com *, enum ieee80211_state, int);
    static int  ath_set_key(struct ieee80211com *, struct ieee80211_node *, struct ieee80211_key *);
    static void ath_delete_key(struct ieee80211com *, struct ieee80211_node *, struct ieee80211_key *);
    static void ath_start(struct _ifnet *);
    static void ath_watchdog(struct _ifnet *);

public:
    IOPCIDevice *pciNub;       // AirportItlwm::start() 已使能的 PCI provider
    struct ieee80211com ic;    // 嵌入的 net80211 控制块
    bool attached;             // net80211 是否已完成附着

    // ---- M0.1a 硬件状态 ----
    IOMemoryMap *bar0Map;                  // BAR0 映射
    Ath10kRegisterIO regs;                 // MMIO 访问抽象（带边界/对齐检查）
    UInt32 chipId;                         // SOC_CHIP_ID 原始值
    UInt8  chipRevision;                   // 芯片版本字段
    const Ath10k::HwCandidate *hwCandidate;// 匹配到的硬件候选（可能为 null）
    bool   hardwareProbed;                 // 是否完成探测
    const char *hardwareStatus;            // 探测结果简述（写入属性）

    // ---- M0.1b 唤醒/复位诊断 ----
    UInt16 pciDeviceId;                    // PCI device ID（用于匹配硬件候选）
    UInt8  pcieCapOffset;                  // PCIe capability 偏移
    UInt16 pcieLinkControlBefore;          // 原始 Link Control
    UInt16 pcieLinkControlAfter;           // ASPM 禁用后 Link Control
    bool   aspmDisabled;                   // ASPM 是否成功禁用
    bool   wakeAsserted;                   // 当前是否持有 SOC_WAKE
    UInt32 rtcStateBefore;                 // 唤醒前 RTC_STATE
    UInt32 rtcStateAfter;                  // 唤醒后 RTC_STATE
    UInt32 wakeDelayUs;                    // 唤醒耗时（微秒）
    bool   wakeSucceeded;                  // 唤醒是否成功
    UInt32 wakeRetries;                    // 运行期重新唤醒次数（诊断）
    bool   targetAwake;                    // 最近一次检查时 RTC_STATE 是否 ON
    UInt32 lastRtcState;                   // 最近一次读到的 RTC_STATE 原始值
    bool   lastRtcReadable;                // 该次读取是否成功
    UInt32 globalResetBefore;              // 复位前 SOC_GLOBAL_RESET
    UInt32 globalResetAfter;               // 复位后 SOC_GLOBAL_RESET
    bool   coldResetDone;                  // 是否执行过冷复位
    UInt32 chipIdAttempts;                 // 芯片 ID 读取尝试次数

    // ---- M0.1c CE ring ----
    Ath10kCERing ceSrcRings[Ath10k::kCECount]; // 各 CE 的源 ring
    Ath10kCERing ceDstRings[Ath10k::kCECount]; // 各 CE 的目的 ring
    bool   ringsAllocated;                 // ring 是否全部分配成功
    const char *ringsFailure;              // 分配失败原因
    UInt32 ceConfiguredCount;              // 需要编程的 CE 数量
    UInt32 ceVerifiedCount;                // 最近一次编程后校验通过的 CE 数量
    UInt32 cePipeInitPassCount;            // CE 编程成功次数（热复位要求 >=2）
    bool   cePipesProgrammed;              // CE 管道当前是否处于已编程状态

    // ---- M0.1c-2 热复位诊断 ----
    bool   warmResetDone;                  // 热复位是否整体成功
    UInt32 warmResetControlBefore;         // 复位前 SOC_RESET_CONTROL
    UInt32 warmResetControlAfter;          // 复位后 SOC_RESET_CONTROL
    bool   warmSI0Ok;                      // SI0 复位成功
    bool   warmCPU1Ok;                     // 第一遍 CPU 复位成功
    bool   warmCPU2Ok;                     // 第二遍 CPU 复位成功
    bool   warmLFok;                       // LF 定时器清零成功
    bool   warmCEok;                       // CE 复位成功
    UInt32 warmInit1Ms;                    // 第一次 target-init 耗时
    UInt32 warmInit2Ms;                    // 第二次 target-init 耗时
    UInt32 warmIndicator1;                 // 第一次 FW_INDICATOR 终值
    UInt32 warmIndicator2;                 // 第二次 FW_INDICATOR 终值
    UInt32 postWarmChipId;                 // 热复位后芯片 ID（应保持不变）
    bool   chipStableAfterWarm;            // 热复位后芯片版本是否一致

    // ---- M0.1c-3 CE7 诊断通道 ----
    Ath10kDMABuffer ceDiagBounce;          // CE7 传输用的主机 bounce 缓冲
    UInt32 ceDiagCoreCtrl;                 // CORE_CTRL（决定诊断窗口基址）
    UInt32 ceDiagPCIeStateAddress;         // target 的 pcie_state 地址
    UInt32 ceDiagPipeCfgAddress;           // target 的 pipe_cfg 地址
    UInt32 ceDiagServiceMapAddress;        // target 的 svc_to_pipe_map 地址
    UInt32 ceDiagConfigFlags;              // target 的 config_flags 值
    UInt32 ceDiagTransactions;             // 成功完成的 CE7 事务计数
    bool   ceDiagOk;                       // 只读发现是否成功

    // ---- M0.1c-4 CE7 init_config ----
    UInt32 initConfigOrigFlags;            // 原始 config_flags
    UInt32 initConfigNewFlags;             // 应用后 config_flags
    UInt32 initConfigOrigEarlyAlloc;       // 原始 hi_early_alloc
    UInt32 initConfigNewEarlyAlloc;        // 应用后 hi_early_alloc
    UInt32 initConfigOrigFlag2;            // 原始 hi_option_flag2
    UInt32 initConfigNewFlag2;             // 应用后 hi_option_flag2
    UInt32 initConfigBanks;                // target IRAM bank 数
    bool   initConfigOk;                   // init_config 是否全部成功

    // ---- M0.1c-5 BMI ----
    Ath10kDMABuffer bmiRequestBuffer;      // CE0 发送缓冲（256 B）
    Ath10kDMABuffer bmiResponseBuffer;     // CE1 接收缓冲（256 B）
    bool   cpuWoken;                       // CORE_CTRL |= 0x2000 是否成功
    UInt32 coreCtrlBeforeWake;             // 唤醒前 CORE_CTRL
    UInt32 coreCtrlAfterWake;              // 唤醒后 CORE_CTRL
    bool   bmiTransferUncertain;           // 提交过 DMA 但完成状态未知（禁止重试）
    bool   bmiTimedOut;                    // BMI 是否超时
    bool   bmiOk;                          // GET_TARGET_INFO 是否成功
    UInt32 bmiTargetLength;                // 响应 length 字段
    UInt32 bmiTargetVersion;               // 响应 version 字段
    UInt32 bmiTargetType;                  // 响应 type 字段

    // ---- M0.1c-6 ----
    Ath10k::PLLState pll;                  // PLL 配置状态
    bool   bmiPrereqOk;                    // HI 前置探测是否成功
    UInt32 hiBoardData;                    // hi_board_data
    UInt32 hiBoardDataInit;                // hi_board_data_initialized
    UInt32 hiEarlyAlloc;                   // hi_early_alloc
    UInt32 hiInterconnect;                 // hi_interconnect_state
    UInt32 hiOptionFlag2;                  // hi_option_flag2
    UInt32 hiCalData;                      // hi_cal_data
    bool   pllOk;                          // PLL 是否配置成功
    bool   bootHIOk;                       // Boot HI 是否配置成功
    UInt32 bootHIOptionBefore;             // 写入前 hi_option
    UInt32 bootHIOptionApplied;            // 写入后 hi_option
    UInt32 hiWordsWritten;                 // Boot HI 写入计数

    // ---- M0.1c-7 payload / board ----
    const Ath10kPayload::Profile *payloadProfile; // 选中的 board/firmware payload
    UInt16 pciSubVendorId;                 // PCI subsystem vendor
    UInt16 pciSubDeviceId;                 // PCI subsystem device
    bool   payloadSelected;                // payload 是否匹配到本机板卡
    bool   boardWriteOk;                   // board 写入完成
    bool   boardVerifyOk;                  // board 回读逐字节一致
    UInt32 boardBytes;                     // board data 字节数
    bool   boardUploadOk;                  // board 写入+回读+init 全部通过

    // ---- M0.1c-8 OTP ----
    bool   otpAttempted;                   // 同一次上电内只允许一次
    bool   otpStreamStarted;               // LZ 流已开始
    bool   otpStreamClosed;                // LZ 流已收尾
    bool   otpExecuteAttempted;
    UInt32 otpWireBytesSent;               // 已发送的 wire 字节数
    UInt32 otpLZDataCommands;              // 已发送的 LZ_DATA 命令数
    bool   otpDownloadOk;                  // OTP 数据下载完成
    bool   otpExecuteOk;                   // BMI_EXECUTE 传输/ABI 完成
    UInt32 otpResult;                      // EXECUTE 返回值
    bool   otpResultAccepted;              // 结果被接受（可能忽略非零）
    const char *otpStatus;                 // OTP 阶段状态

    // ---- M0.1c-9 固件 ----
    bool   fwAttempted;
    bool   fwStreamStarted;
    bool   fwStreamClosed;
    UInt32 fwWireBytesSent;
    UInt32 fwLZDataCommands;
    bool   fwDownloadOk;                   // LZ 传输完成
    bool   fwReadbackOk;                   // 6 段回读全部一致
    UInt32 fwReadbackBytes;                // 已校验字节总数
    UInt32 fwVerifiedSegments;             // 已校验段数
    UInt32 fwFailedSegment;                // 失败段号
    UInt32 fwPostVersion;                  // 下载后复读的 target version
    UInt32 fwPostType;
    const char *fwStatus;                  // 固件阶段状态

    // ---- M0.1c-10 ----
    bool   uartDisabled;                   // HI_SERIAL 已清零（关 UART）
    bool   preDoneHIVerified;              // BMI_DONE 前的 HI 终检
    UInt32 bootHIOptionAfterDownload;      // 下载后复读的 hi_option
    bool   bmiClosed;                      // BMI 已永久关闭
    bool   doneAttempted;                  // BMI_DONE 已尝试
    bool   doneSent;                       // BMI_DONE 已被 CE0 消费
    Ath10kDMABuffer htcReadyBuffer;        // CE1 的 2048B HTC 首包缓冲
    bool   htcRxPosted;                    // CE1 RX 描述符已挂
    bool   htcRxOwned;                     // CE1 已让渡给固件
    bool   htcRxCompleted;                 // CE1 收到首包
    bool   htcReadyOk;                     // HTC READY 解析成功
    UInt32 htcRxSlot;
    UInt32 htcRxNext;
    UInt32 htcRxRead;
    UInt32 htcRxWrite;
    UInt32 htcRxBytes;
    UInt32 htcWaitMs;
    UInt32 htcFwIndicator;
    UInt8  htcPrefix[32];
    UInt32 htcPrefixBytes;
    Ath10k::HTCReadyInfo htcReady;         // HTC READY 协商结果
    const char *htcStatus;                 // HTC 阶段状态
    const char *htcParseError;             // HTC READY 解析错误

    // ---- M1-a HTC 服务 ----
    Ath10kDMABuffer htcControlBuffer;      // HTC 控制帧 TX（256 B）
    Ath10kDMABuffer htcCtrlRxBuffer;       // HTC 运行时 RX（2048 B）
    UInt32 runtimeRxSlot;                  // 运行时 RX 描述符槽位
    UInt32 runtimeRxNext;                  // 运行时 RX doorbell 值
    bool   runtimeRxArmed;                 // CE1 描述符当前是否已挂
    UInt32 htcCtrlTxCount;                 // 控制帧发送计数
    UInt32 htcCtrlRxCount;                 // 控制帧接收计数
    UInt32 htcCtrlLastWaitMs;              // 最近一次控制交换耗时
    bool   htcServicesOk;                  // 服务是否全部连接
    const char *htcServiceStatus;          // 服务建立状态
    UInt8  wmiEndpoint;                    // WMI 服务端点
    UInt16 wmiMaxMessageSize;              // WMI 单条消息上限
    UInt8  httEndpoint;                    // HTT 服务端点
    UInt16 httMaxMessageSize;              // HTT 单条消息上限

    // ---- M1-b1 WMI bootstrap ----
    Ath10kDMABuffer wmiRxBuffer;           // CE2 WMI 接收缓冲（2048 B）
    UInt32 wmiRxSlot;                      // WMI 接收描述符槽位
    UInt32 wmiRxNext;                      // WMI 接收 doorbell 值
    bool   wmiRxArmed;                     // CE2 描述符当前是否为已挂状态
    UInt32 wmiRxBytes;                     // 本次收到的字节数
    UInt32 wmiEventId;                     // 最近一条事件 ID
    UInt32 wmiWaitMs;                      // 等待耗时
    bool   setupCompleteSent;              // SETUP_COMPLETE 已发送
    bool   wmiServiceAvailable;            // 已收到 SERVICE_AVAILABLE(3)
    bool   wmiServiceReady;                // 已收到 SERVICE_READY(1)
    UInt32 wmiServiceBits;                 // SERVICE_AVAILABLE 位图位数
    UInt32 wmiServiceBitmap[4];            // SERVICE_AVAILABLE 位图（最多 128 位）
    UInt32 wmiServiceBitmapFull[32];       // SERVICE_READY 完整服务位图（1024 位）
    bool   wmiServiceBitmapPresent;
    UInt32 wmiFwBuild;                     // 固件 build 号（SERVICE_READY）
    UInt32 wmiAbi[6];                      // firmware ABI 字
    bool   wmiAbiPresent;
    UInt32 wmiRfChains;                    // 固件上报 RF 链数
    UInt32 wmiMemoryRequests;              // host memory 请求数
    UInt32 wmiRegCaps[9];                  // hal_reg_capabilities 前 9 字
    bool   wmiRegCapsPresent;
    UInt8  wmiEvtPrefix[48];               // 事件原始前缀（诊断）
    UInt32 wmiEvtPrefixBytes;
    const char *wmiStatus;                 // WMI bootstrap 阶段状态
    const char *wmiParseError;             // WMI 事件解析错误
    const char *wmiEventStatus;            // WMI 事件等待状态

    // ---- M1-b2 WMI INIT / vdev ----
    Ath10kDMABuffer wmiTxBuffer;           // CE3 WMI 命令缓冲（2048 B）
    Ath10kDMABuffer wmiHostMemory[Ath10k::kWMIHostMemoryMax];
    Ath10k::WMICreditState wmiCredits;     // WMI 端点信用账本
    bool   wmiCreditsReady;
    Ath10k::WMIEventInfo wmiEvent;         // 最近一条事件的解析结果
    Ath10k::WMIEventInfo wmiBoot;          // SERVICE_READY 快照（INIT 的输入）
    bool   wmiInitOk;                      // WMI INIT 完成（READY 且状态为 0）
    bool   wmiReadyEventReceived;          // 收到 READY(2)
    UInt32 wmiReadyStatus;                 // READY 事件里的固件状态字
    UInt32 wmiInitTxBytes;                 // INIT 帧字节数
    UInt32 wmiInitWaitMs;                  // INIT 等待耗时
    UInt32 wmiHostMemoryCount;             // 已分配的 host memory 块数
    UInt32 wmiHostMemoryBytes;             // 已分配字节数
    bool   wmiVdevOk;                      // vdev create 已被 CE3 消费
    bool   wmiVdevTxConsumed;
    UInt32 wmiVdevTxBytes;
    UInt32 wmiVdevWaitMs;
    UInt8  wmiSeq;                         // WMI 端点序号（每条命令递增）
    UInt32 wmiTxSlot;                      // CE3 当前描述符槽位
    UInt32 wmiTxNext;                      // CE3 当前 doorbell 值
    bool   wmiTxOwned;                     // 已提交过命令（不确定时禁止重发）
    bool   wmiTxConsumed;                  // 最近一条命令已被 CE3 消费
    UInt32 wmiTxSrcWrite;                  // 最近一次 CE3 srcWriteIndex 快照
    UInt32 wmiTxSrcRead;                   // 最近一次 CE3 currentSrcRead 快照
    UInt32 wmiTxSrcWriteRaw;               // CE3 srcWriteIndex 原始寄存器值（自由计数）
    UInt32 wmiTxSrcReadRaw;                // CE3 currentSrcRead 原始寄存器值
    UInt32 ce3Resyncs;                     // CE3 读写索引重同步次数（诊断）
    const char *wmiTxFail;                 // WMI TX 失败分支（自解释标记）
    UInt32 fwIndicator;                    // FW_INDICATOR 原始值
    UInt32 fwIndicatorBaseline;            // 首次采样值（用于区分陈旧崩溃位）
    bool   firmwareCrash;                  // 固件报告 EVENT_PENDING
    UInt8  targetMac[IEEE80211_ADDR_LEN];  // 固件上报的 MAC
    bool   targetMacValid;
    const char *wmiInitStatus;
    const char *wmiVdevStatus;

    // ---- M1-c2 HTT RX ----
    Ath10kDMABuffer httTxBuffer;           // HTT 命令缓冲（64 B）
    Ath10kDMABuffer httRxIndex;            // 写索引环（8 B）
    Ath10kDMABuffer httRxRing;             // 描述符环（2048 × 4 B）
    Ath10kDMABuffer httRxPool;             // 接收缓冲池（1023 × 2048 B）
    bool   httSetupOk;
    UInt32 httSetupStage;                  // 0 未开始，1 VERSION_REQ，2 RING_CFG，3 AGGR_CFG
    const char *httSetupStatus;
    bool   httVersionReceived;
    UInt32 httVersionMajor;
    UInt32 httVersionMinor;
    UInt32 httMessages;                    // CE1 上收到的 HTT 消息数
    UInt32 httLastType;
    UInt8  httLastPrefix[16];               // 最近一条 HTT 消息前缀（诊断）
    UInt32 httLastPrefixBytes;
    UInt32 httRxRingPosted;               // 已投递的 RX buffer 数
    UInt8  httSeq;
    UInt32 httTxSlot;
    UInt32 httTxNext;
    bool   httTxConsumed;

    // ---- M1-c1 扫描 ----
    bool   pdevSetupOk;                    // chain mask 已下发
    bool   scanSetupOk;                    // 信道表已下发
    bool   scanOk;                         // 扫描完成且 reason=0
    bool   scanStarted;                    // 收到 SCAN_STARTED
    bool   scanCompleted;                  // 收到 SCAN_COMPLETED
    UInt32 scanEvents;                     // 扫描事件数
    UInt32 scanMgmtFrames;                 // MGMT_RX 帧数
    UInt32 scanReason;                     // 扫描完成原因（0 为成功）
    UInt32 scanLastFreq;                   // 最近一次扫描事件的信道频率
    UInt32 scanWaitMs;                     // 扫描收集耗时
    UInt32 scanBssCount;                   // 去重后的 BSS 数
    const char *pdevSetupStatus;
    const char *scanStatus;
    Ath10k::WMIScanInfo scanInfo;          // 最近一条扫描事件解析结果
    UInt32 scanEvStarted;                  // type=1 STARTED
    UInt32 scanEvCompleted;                // type=2 COMPLETED
    UInt32 scanEvBssChan;                  // type=4 BSS_CHANNEL（固件发现 BSS）
    UInt32 scanEvForeignChan;              // type=8 FOREIGN_CHANNEL
    UInt32 scanEvOther;                    // 其它扫描事件
    UInt32 ce1Frames;                      // CE1 上收到的非 trailer 帧（HTT/控制）
    UInt32 ce1LastEndpoint;
    UInt32 ce1LastBytes;
    struct ScanBssEntry {
        UInt8  mac[6];
        UInt8  ssid[32];
        UInt32 ssidLen;
        UInt32 freq;
        UInt32 snr;
    };
    ScanBssEntry scanBss[16];              // 最多记录 16 个 BSS

    // ---- M1-c3 扫描 → net80211 ----
    int (*defaultNewState)(struct ieee80211com *, enum ieee80211_state, int); // net80211 原状态机
    thread_call_t scanCall;                // 异步扫描用 thread call
    IOTimerEventSource *scanTimer;         // 1 Hz 轮询（按请求 + 退避）
    volatile bool scanWorkQueued;          // 已排队但尚未开始
    volatile bool scanWorkRunning;         // 正在执行
    volatile bool scanWorkRerun;           // 执行期间又被请求
    bool   scanDisabled;                   // boot-arg ath10k_scan=0 或自熔断
    bool   scanSelfFused;                  // 连续失败过多而自行停用扫描
    UInt32 scanTickCount;                  // 定时器 tick 计数（1 Hz）
    UInt32 scanNextAllowedTick;             // 早于该 tick 不允许开新一轮
    UInt32 scanFailStreak;                 // 连续失败轮数（退避用）
    UInt32 scanRoundStartTick;             // 当前轮开始时的 tick（卡轮看门狗）

    // ---- M1-d 连接 ----
    bool   joinEnabled;                    // false = boot-arg ath10k_join=0
    const char *joinStatus;                // 连接阶段状态串（写入 ioreg）
    UInt32 joinCommands;                   // 已提交的连接命令数
    UInt32 joinCommandFails;               // 连接命令失败数
    UInt32 joinStage;                      // 0=idle 1=start 2=peer 3=assoc 4=auth-wait 5=authorize 6=vdev-up 7=done 8=failed
    UInt32 mgmtTxSubmitted, mgmtTxCompleted, mgmtTxFailed, mgmtTxDropped;
    UInt32 joinFeedOk, joinFeedSkipped;    // 连接期间喂给 net80211 的管理帧
    UInt16 httTxId;                        // 轮转的 TX descriptor id
    UInt16 httTxIdCurrent;                 // 在途 TX id
    bool   httTxOwned, httTxComplete;
    UInt32 httTxStatus;
    UInt8  mgmtTxSeq;                      // 管理帧 HTT 描述符序号
    bool   rxPumpRunning;
    bool   feedingFrames;                   // feedStagedScanFrames 重入保护
    bool   joinTriggerPending;              // 待触发的 end_scan（选 BSS → AUTH）
    bool   joinTriggered;                   // 本次连接请求已经触发过 end_scan
    UInt32 joinNextAllowedTick;             // 下次允许触发连接的 tick（≥10 s 间隔）
    UInt32 macReconfigAcks;                 // 已为 MAC 重配置补发的 DRIVER_READY 次数
    UInt32 macReconfigNextTick;             // 下次允许补发的 tick
    bool   hasJoinTarget();                 // ic_tree 里是否已有目标 AP
    void   checkJoinTrigger();              // 1 Hz 检查是否需要触发连接
    thread_call_t rxCall;
    UInt32 rxPumpEvents, rxPumpErrors;
    Ath10kDMABuffer txFrameBuffer;         // 帧负载（2048）
    Ath10kDMABuffer txFragBuffer;          // fragment 数组（16）
    Ath10kDMABuffer txPacketBuffer;        // HTT TX 描述符（256）
    bool   scanRequestSeen;                // 是否已看到本次系统扫描请求（上升沿检测）
    UInt32 scanRequestRounds;              // 本次系统请求已消耗的扫描轮数
    UInt32 scanRounds;                     // 已完成的 WMI 扫描轮数
    UInt32 scanFeedAttempts;               // 尝试喂给 net80211 的管理帧
    UInt32 scanFeedOk;                     // 成功进入 ieee80211_input 的帧
    UInt32 scanFeedSkipped;                // 因状态不是 SCAN 而跳过的帧
    UInt32 scanNodes;                      // 最近一轮结束时的 ic_tree 节点数
    UInt32 scanDoneEvents;                 // 已发出的 SCAN_DONE 事件数
    const char *scanFeedStatus;            // M1-c3 状态串（写入属性）
    bool   driverReadySent;                // DRIVER_READY 是否已发给 WCL

    // 扫描线程只负责把帧拷进这个暂存区；真正喂给 net80211 的动作
    // 统一在 workloop 上执行（WCL 的 BEACON / SCAN_DONE 回调假设 RX 上下文）。
    static constexpr UInt32 kScanStageMaxFrames  = 32;
    static constexpr UInt32 kScanStageFrameBytes = 1024;
    struct ScanStageFrame {
        UInt16 length;
        UInt32 freq;
        UInt32 snr;
        UInt8  data[kScanStageFrameBytes];
    };
    ScanStageFrame scanStage[kScanStageMaxFrames];
    UInt32 scanStageCount;                 // 本轮暂存的帧数
    UInt32 scanStageOverflow;              // 因容量/长度被丢弃的帧数

    // ---- 异步 bring-up ----
    thread_call_t bringUpCall;             // 延迟执行 bring-up 的 thread call
    volatile bool bringUpRunning;          // bring-up 是否正在执行
    volatile bool shuttingDown;            // 正在拆除，bring-up 应尽快退出

    // 占位 MAC：M1-b2 拿到 WMI READY 的 target MAC 后替换
    uint8_t placeholderMac[IEEE80211_ADDR_LEN];
};

#endif /* Ath10kHal_hpp */