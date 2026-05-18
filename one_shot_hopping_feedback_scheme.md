# USRP X310 + MATLAB 单次传输  跳频传输系统方案

本文档给出一个可以直接按模块编码的传输方案。目标不是做传统 ARQ 可靠传输，而是做演示型“一次发射、可观测受干扰效果”的文件传输系统：发射端在进入业务数据阶段后，每个业务数据/校验包只发送一次，不做 UDP 式循环广播，也不根据缺包请求重传。接收端通过单次接收结果、外层前向纠错和质量指标来展示另一个干扰系统造成的影响。

## 1. 设计目标与边界

### 1.1 必须满足

1. 支持文本、图片、视频文件传输。
2. 主链路支持跳频传输。
3. 存在反向链路，接收端向发射端回传 SNR 等状态信息，并预留后续扩展字段。
4. 业务数据只发送一次：不循环广播全文件，不按缺包重发，不做选择性重传。
5. 考虑发射端和接收端不能同时启动的情况。
6. 接收端输出能反映传输状态和干扰效果的指标。
7. 系统偏演示用途，优先保证可观测、启动稳定、传输速度合理，而不是追求部署级协议完备性。

### 1.2 关键原则

1. 启动同步和控制帧可以重复，业务数据帧不能重复。
2. 反向链路只做状态遥测和下一次会话建议，不触发本次会话重传。
3. 本次会话的跳频表、调制方式、FEC 参数在业务数据开始前锁定，避免数据阶段边收边改参数导致两端状态不一致。
4. 可靠性主要靠三层保护：
   - 物理层 CRC32 + LDPC；
   - 业务层分组序号和文件 CRC；
   - 包级外层 FEC，用冗余校验包恢复丢包。

## 2. 总体架构

系统分为四条逻辑链路：

1. 前向控制链路：TX 在固定锚点频率发送 BEACON、START、END 等控制帧。该链路允许重复，因为它不承载业务文件内容。
2. 前向业务链路：TX 按跳频表发送文件数据帧和外层 FEC 校验帧。进入该阶段后只扫一遍。
3. 反向反馈链路：RX 在固定 1.45 GHz 发送 READY、TELEMETRY、RESULT 等反馈帧。
4. UI/日志链路：MATLAB 通过本地 HTTP 或文件日志把状态送给 PyQt/Flask UI。

推荐频率：

| 用途 | 频率 |
|---|---:|
| 前向控制锚点 | 2.0 GHz 或当前 Carrier_set(1) |
| 前向跳频集合 | 2.0:0.5:4.0 GHz |
| 反向反馈链路 | 1.45 GHz |

推荐采样率沿用当前项目：

```matlab
samp_rate = 200e6 / 512;    % 390.625 kS/s
sps = 4;
```

## 3. 启动不同步处理

严格来说，如果发射端已经开始发送业务数据，而接收端还没有启动，接收端不可能恢复错过的数据。因此协议必须让 TX 在发送业务数据前等待 RX_READY。这样既满足“数据只发一次”，又能处理两端启动先后顺序。

### 3.1 TX 先启动

1. TX 初始化 SDR，预处理待发送文件，生成业务帧缓存。
2. TX 不发送业务数据，只在锚点频率周期性发送 TX_BEACON。
3. RX 后启动，扫描锚点频率，收到 TX_BEACON 后发送 RX_READY。
4. TX 收到 RX_READY 后发送 TX_START 倒计时控制帧，例如 3 个控制槽。
5. 倒计时结束后，TX 进入业务数据阶段，一次性发送全部 DATA/PARITY 帧。

### 3.2 RX 先启动

1. RX 初始化后进入 WAIT_BEACON，同时在反向链路低占空比发送 RX_READY_DISCOVERY。
2. TX 后启动，先监听反向链路并发送 TX_BEACON。
3. 两端握手成功后进入 START 倒计时，再开始单次业务传输。

### 3.3 业务阶段迟到

如果 RX 在 TX 已经进入 DATA 阶段后才启动，本次会话应标记为 `late_join=1`，只做频谱/SNR/捕获到的包统计，不要求完整恢复。TX 不为迟到接收端重新发送业务数据。

## 4. 会话状态机

### 4.1 发射端状态机

| 状态 | 行为 | 退出条件 |
|---|---|---|
| INIT | 读取文件、压缩/封装、生成 FEC、预生成波形 | 初始化完成 |
| WAIT_READY | 锚点频率重复发送 TX_BEACON；监听反馈链路 | 收到 RX_READY |
| START_COUNTDOWN | 重复发送 TX_START，给出 session_id、hop_seed、slot_len、total_slots | 倒计时结束 |
| DATA_ONCE | 按跳频表发送所有业务槽，每个 DATA/PARITY 帧一次 | 所有槽发送完 |
| END_LISTEN | 发送 TX_END 控制帧，可监听 RX_RESULT | 超时或收到 RESULT |
| DONE | 停止业务发送，保留 UI 显示 | 用户下一次发送 |

DATA_ONCE 状态不处理缺包请求，不回退到 REPAIR。

### 4.2 接收端状态机

| 状态 | 行为 | 退出条件 |
|---|---|---|
| WAIT_BEACON | 扫描锚点频率，必要时发送 READY_DISCOVERY | 收到 BEACON |
| READY_SENT | 发送 RX_READY，等待 START | 收到 START |
| FOLLOW_HOP | 按跳频表接收业务槽，解析 DATA/PARITY | 收到 END 或超时 |
| FEC_REBUILD | 对每个 FEC 组做包级恢复，重组文件 | 恢复完成 |
| RESULT_REPORT | 反向发送 RESULT/TELEMETRY，UI 展示指标 | 用户结束或下一会话 |

## 5. 前向业务帧设计

现有代码中一个 LDPC 输入块是 486 bit，其中 CRC32 之前最多可承载 454 bit 信息。为了加入跳频和 FEC 字段，建议把每个物理业务帧的净载荷从 44 Byte 调整为 40 Byte。

### 5.1 DATA/PARITY 帧信息位布局

总信息位 454 bit，CRC32 后补齐到 486 bit，再 LDPC 编码到 648 bit。

| 字段 | bit | 说明 |
|---|---:|---|
| FrameHead | 8 | 沿用 `defs.frame_head` |
| UserID | 8 | 沿用 `defs.user_id` |
| FrameType | 8 | `20=DATA`，`21=PARITY`，`22=META` |
| ProtoVer | 4 | 建议固定为 2 |
| Flags | 4 | bit0=last_file_frame，bit1=encrypted/reserved，其他保留 |
| SessionID | 16 | 会话号 |
| TotalFrameNum | 16 | 本会话业务物理帧总数，含 META、DATA、PARITY |
| FrameID | 16 | 从 1 开始的物理帧序号 |
| PayloadBytes | 6 | 本帧有效字节数，0-40 |
| FileType | 3 | 0=text，1=image，2=video，3=binary，4=video_frame_container |
| StreamID | 3 | 预留多流，当前填 0 |
| IsParity | 1 | 0=源数据，1=FEC 校验 |
| LastInSession | 1 | 是否最后一个业务物理帧 |
| HopSlotID | 12 | 当前跳频槽号，可循环 |
| HopIndex | 4 | 频点索引，最多 16 个频点 |
| FecGroupID | 12 | 外层 FEC 组号 |
| FecIndex | 6 | 组内序号，0 到 N-1 |
| FecK | 6 | 组内源包数 K，推荐 16/24/32 |
| Payload | 320 | 40 Byte 业务或校验载荷 |

编码流程沿用现有 `Data_trans_sig_Gen`：

```matlab
frame_bits = [header_bits; payload_bits_320];
coded_in = [crcGenerator(frame_bits); zeros(486 - 32 - length(frame_bits), 1)];
scr_bits = scramble_bits(coded_in, defs.scr_seq);
enc_bits = ldpcEncode(scr_bits, cfgLDPCEnc);
inter_bits = interleave_36x18(enc_bits);
mod_sig = qpsk_or_spread_bpsk(inter_bits, mode);
```

### 5.2 文件元数据 META

META 不要依赖文件扩展名放在第一个普通 payload 字节这种隐式格式，建议明确 TLV 化。META 也进入外层 FEC 保护。

推荐 META TLV：

| 字段 | 长度 | 说明 |
|---|---:|---|
| magic | 4 Byte | `OSFH`，One Shot FH |
| proto_ver | 1 | 2 |
| file_type | 1 | text/image/video/binary |
| fec_k | 1 | K |
| fec_r | 1 | R=N-K |
| file_size | 4 | 原始业务字节数 |
| source_packet_num | 4 | 源数据包数 |
| parity_packet_num | 4 | 校验包数 |
| total_group_num | 2 | FEC 组数 |
| file_crc32 | 4 | 原始业务字节 CRC32 |
| hop_seed | 4 | 跳频随机种子 |
| slot_len_samples | 4 | 每跳槽长度 |
| codewords_per_slot | 2 | 每槽承载业务码字数 |
| name_len | 1 | 文件名长度 |
| ext_len | 1 | 扩展名长度 |
| name/ext | 可变 | UTF-8，必要时截断 |

META 可以分成若干 40B 源包并作为 FEC group 0，用更强冗余，例如 `K=4,R=8`。这不是业务数据重复，而是元数据的前向纠错冗余。

## 6. 外层包级 FEC

因为不能重传，必须用前向纠错处理丢包。建议使用系统型 Reed-Solomon erasure coding：

1. 将文件字节流切成 40B 源包。
2. 每 K 个源包组成一个 FEC 组。
3. 对每个字节位置独立做 RS 编码，生成 R 个 40B 校验包。
4. TX 发送 K 个源包和 R 个校验包，每个只发一次。
5. RX 对 CRC 通过的物理帧建表；每组收到不少于 K 个包即可恢复该组所有源包。

推荐参数：

| 场景 | K | R | 冗余率 | 用途 |
|---|---:|---:|---:|---|
| 快速演示 | 24 | 4 | 16.7% | 信道较好，突出速度 |
| 默认演示 | 24 | 8 | 33.3% | 建议默认 |
| 强干扰演示 | 16 | 8 | 50% | 更容易展示抗干扰效果 |
| 极强干扰 | 12 | 12 | 100% | 牺牲速度换恢复概率 |

MATLAB 实现建议：

```matlab
% group_src: K x 40 uint8
% 对每一列做 RS(N,K) 编码，得到 N x 40
N = K + R;
encoded = zeros(N, 40, 'uint8');
for col = 1:40
    msg = gf(group_src(:, col).', 8);      % 1 x K
    code = rsenc(msg, N, K);               % 1 x N
    encoded(:, col) = uint8(code.x(:));
end
```

接收端解码时，把缺失位置标成 erasure。若 MATLAB 的 RS 解码接口不方便处理二维 erasure，可先实现简化版 GF(256) Vandermonde erasure decoder；或者退一步实现 XOR 分层校验，但 RS 的恢复能力明显更稳定。

## 7. 跳频方案

### 7.1 频点集合

沿用项目当前集合：

```matlab
Carrier_set = 2e9 : 0.5e9 : 4e9;   % 5 个频点
```

### 7.2 跳频序列

每个会话由 `hop_seed` 生成伪随机序列：

```matlab
rng(hop_seed, 'twister');
hop_seq = randi(numel(Carrier_set), total_slots, 1);
```

为避免连续两个槽在同一频点，可加约束：

```matlab
for i = 2:total_slots
    if hop_seq(i) == hop_seq(i-1)
        hop_seq(i) = mod(hop_seq(i), numel(Carrier_set)) + 1;
    end
end
```

### 7.3 跳频槽结构

推荐从“每个物理包一个长帧头”升级为“每个跳频槽一个长同步头，后面连续多个 LDPC 码字”。这样速度比当前 10 包/槽循环广播明显提高。

一个业务槽：

```text
GuardPre
SlotPreamble(head_data, 511 symbols)
SlotHeader(LDPC codeword, robust QPSK or BPSK)
DATA/PARITY codeword 1
DATA/PARITY codeword 2
...
DATA/PARITY codeword M
GuardPost
```

推荐初始参数：

| 参数 | 推荐值 |
|---|---:|
| slot_len_samples | 160000 |
| guard_pre_samples | 12000 |
| guard_post_samples | 12000 |
| retune_guard_ms | 30-80 ms，按实测调整 |
| codewords_per_slot | 60-90，先从 60 调试 |
| slot_preamble | 现有 `defs.head_data` |

如果先做低风险版本，可以保留现有每包 preamble 格式，只把 `BURST_PKTS` 从 10 改成自动填满槽。长期建议做槽级 preamble，否则视频传输速度太慢。

### 7.4 接收端跟跳

RX 在 START 阶段获得 `hop_seed,total_slots,slot_len_samples,codewords_per_slot` 后，按同样函数生成 `hop_seq`。每个槽：

1. 提前设置 `radio_rx.CenterFrequency = Carrier_set(hop_seq(slot_id))`。
2. 丢弃调谐稳定保护样本。
3. 读取一个槽长度样本。
4. 用 SlotPreamble 相关峰定位。
5. 解 SlotHeader，确认 slot_id/hop_index/session_id。
6. 按固定码字长度切分后续 DATA/PARITY 码字。
7. 每个码字独立 LDPC+CRC，CRC 过才入缓存。

如果某槽同步失败，则整槽记为 lost slot。不要请求重发，只进入指标统计和 FEC 恢复。

## 8. 反向链路设计

反向链路固定 1.45 GHz，使用更鲁棒的 BPSK + 15 倍扩频，沿用现有 `head_fb`、`pn_fb`、LDPC、CRC 结构。当前反馈信息位只用 96 bit 且没有保留空间，建议升级到 256 bit 信息位：

```text
256 bit feedback_info + CRC32 + 198 bit zero padding = 486 bit LDPC input
```

### 8.1 反馈帧类型

| FrameType | 名称 | 用途 |
|---:|---|---|
| 30 | RX_READY | 接收端已准备好，可开始会话 |
| 31 | RX_TELEMETRY | 数据阶段周期遥测 |
| 32 | RX_RESULT | 本次会话最终结果 |
| 33 | RX_ABORT | 接收端异常退出 |
| 34 | RX_NEXT_ADVICE | 给下一次会话的频点/模式/增益建议 |

### 8.2 反馈信息位布局

| 字段 | bit | 说明 |
|---|---:|---|
| FrameHead | 8 | `defs.frame_head` |
| UserID | 8 | `defs.user_id` |
| FrameType | 8 | 30-34 |
| ProtoVer | 4 | 2 |
| HeaderLen | 4 | 当前填 32 Byte |
| SessionID | 16 | 会话号 |
| FeedbackSeq | 16 | 反馈序号 |
| RxState | 4 | WAIT/READY/FOLLOW/FEC/RESULT |
| FileType | 4 | 当前文件类型 |
| SNR_q8 | 8 | SNR dB 量化，`round((snr_db+20)*4)`，范围约 -20 到 43.75 dB |
| RSSI_q8 | 8 | 接收功率量化 |
| Noise_q8 | 8 | 噪声底量化 |
| EVM_q8 | 8 | EVM 百分比或 dB 量化 |
| CFO_i16 | 16 | 频偏估计，单位可设 10 Hz |
| SyncMetric_q8 | 8 | 同步峰归一化指标 |
| HopIndex | 4 | 当前频点 |
| Mode | 4 | 当前调制/扩频模式 |
| TotalFrameNum | 16 | 总业务帧数 |
| RxCrcOkNum | 16 | CRC 通过帧数 |
| RxLostNum | 16 | 当前缺失帧数 |
| FecRecoveredNum | 16 | FEC 恢复帧数 |
| PreFecPER_q16 | 16 | `round(PER*65535)` |
| PostFecPER_q16 | 16 | FEC 后未恢复比例 |
| GoodputKbps_q16 | 16 | 有效吞吐，kbps |
| ResultCode | 8 | 0=unknown，1=complete，2=partial，3=failed |
| Advice | 8 | 给下一会话的模式/功率建议编码 |
| Reserved | 64 | 后续扩展 |

反向链路在 DATA 阶段建议每 0.5-1 s 发一次 TELEMETRY。反馈可以重复 2-4 次以提高显示稳定性，但 TX 只显示，不触发本次 DATA 重传。

## 9. 业务文件封装

### 9.1 文本

1. 统一转 UTF-8。
2. META 中 `file_type=0`，扩展名 `.txt`。
3. 接收端按 UTF-8 解码，失败字节用替换符并统计文本可读率。

### 9.2 图片

1. 支持 `.jpg/.jpeg/.png/.bmp`。
2. 演示建议统一转 JPEG，控制质量和大小，例如长边 640、质量 70-85。
3. 接收端完整恢复后写出文件，并计算 MSE/PSNR/SSIM。

### 9.3 视频

有两种模式：

1. 普通文件模式：把 `.mp4` 当二进制文件传输。优点是实现简单；缺点是如果 FEC 后仍缺关键字节，视频可能整体不可播放。
2. 演示视频容器模式：发送端把视频拆成低分辨率 JPEG 帧或 I-frame 图片序列，再封装为 `video_frame_container`。每帧独立恢复，丢帧时接收端可以用上一帧冻结或灰帧补齐，最后重新编码成 MP4。该模式最适合展示干扰效果。

推荐演示视频预处理：

| 参数 | 推荐值 |
|---|---:|
| 分辨率 | 320x180 或 426x240 |
| 帧率 | 8-12 fps |
| 时长 | 5-15 s |
| 单帧 JPEG 质量 | 45-65 |
| FEC | K=16,R=8 或 K=24,R=8 |

## 10. 接收端指标设计

接收端 UI 和日志至少输出以下指标。

### 10.1 物理层指标

1. SNR：建议用前导空闲段估噪声、数据段估信号功率，输出 dB。
2. RSSI/接收功率：`10*log10(mean(abs(rx_sig).^2)+eps)`。
3. 同步峰值：相关峰最大值、峰均比、同步成功/失败槽数。
4. CFO：由前导相位斜率估计频偏。
5. EVM：解调星座到理想星座的均方误差。
6. 每跳频点的 SNR/PER，用于展示干扰系统在哪些频点上影响更强。

### 10.2 链路层指标

1. `rx_crc_ok_num / total_frame_num`：物理帧 CRC 成功率。
2. Pre-FEC PER：`1 - crc_ok_frames / total_frames`。
3. FEC 恢复帧数：由校验包恢复出的源包数。
4. Post-FEC PER：FEC 后仍缺的源包比例。
5. 丢包位置图：按 FrameID 或 FecGroupID 显示缺失分布。
6. 有效吞吐 Goodput：最终恢复业务字节数 / DATA_ONCE 持续时间。
7. 单次传输完成时间：从 START 结束到 END 的时间。
8. 迟到标志：是否 late_join。

### 10.3 比特/文件层指标

1. 文件 CRC32 是否匹配。
2. 字节恢复率：恢复字节数 / 原始字节数。
3. 参考 BER：如果接收端有原文件或演示时允许读取原文件，直接按 bit 比较。
4. 估计 BER：没有原文件时，用 EVM/SNR 或 LDPC 软信息给出估计值，并标明是估计值。

### 10.4 图片质量指标

完整恢复且有原图时：

1. MSE。
2. PSNR。
3. SSIM。
4. 文件 CRC 是否一致。

无原图时：

1. 是否可被 `imread/imfinfo` 正常解析。
2. 图像尺寸是否符合 META。
3. NIQE/BRISQUE，如果 MATLAB 版本和工具箱支持。

### 10.5 视频质量指标

普通 MP4 文件模式：

1. 是否可播放。
2. 可解码帧数 / 期望帧数。
3. 可播放时长 / 原始时长。
4. 如果有原视频：逐帧 PSNR/SSIM 平均值。

演示视频容器模式：

1. 帧恢复率。
2. 丢帧率。
3. 冻结帧率。
4. 平均 PSNR/SSIM。
5. 最终重建视频是否可播放。

## 11. MATLAB 模块拆分建议

### 11.1 公共模块

| 文件 | 作用 |
|---|---|
| `link_phy_defs.m` | 增加协议版本、帧类型、频点集合、FEC 默认参数 |
| `pack_bits.m` / `unpack_bits.m` | 支持非 8/16 bit 字段打包，避免大量手写索引 |
| `crc32_bytes.m` | 文件级 CRC32 |
| `build_hop_sequence.m` | 根据 hop_seed 生成跳频序列 |
| `fec_rs_encode_groups.m` | 外层 RS 编码 |
| `fec_rs_decode_groups.m` | 外层 RS 擦除恢复 |

### 11.2 发射端模块

| 文件 | 作用 |
|---|---|
| `build_file_container.m` | 文本/图片/视频统一封装，生成 META 和业务字节 |
| `build_forward_frames_v2.m` | 按 40B payload 生成 META/DATA/PARITY 帧结构 |
| `forward_frame_modulate_v2.m` | 单个 LDPC codeword 调制 |
| `build_hop_slot_waveform.m` | 一个跳频槽内放 preamble、slot header、多个 codeword |
| `Transmitter_one_shot_fh.m` | 新发射端主程序 |

### 11.3 接收端模块

| 文件 | 作用 |
|---|---|
| `detect_hop_slot.m` | 前导相关、槽同步、相位/频偏估计 |
| `decode_forward_codewords_v2.m` | 解调连续 codewords，输出 CRC 通过帧 |
| `rx_frame_cache_update.m` | 按 SessionID/FecGroupID/FecIndex 建表 |
| `rebuild_file_from_fec.m` | FEC 恢复并重组文件 |
| `compute_rx_metrics.m` | 计算 SNR/PER/吞吐/质量指标 |
| `Receiver_one_shot_fh.m` | 新接收端主程序 |

### 11.4 反馈模块

| 文件 | 作用 |
|---|---|
| `feedback_frame_pack_v2.m` | 打包 256 bit 反馈信息 |
| `feedback_frame_modulate_v2.m` | 反馈帧 BPSK+扩频调制 |
| `feedback_frame_decode_v2.m` | 发射端解反馈 |

## 12. 主程序伪代码

### 12.1 发射端

```matlab
init_usrp_txrx();
cfg = load_link_config();
session_id = next_session_id();

container = build_file_container(input_path_or_text, cfg.demo_video_mode);
[src_pkts, meta] = split_to_40B_packets(container);
[frames, fec_info] = build_forward_frames_v2(src_pkts, meta, cfg.fec);
hop_seq = build_hop_sequence(meta.hop_seed, total_slots, cfg.Carrier_set);
slot_cache = prebuild_hop_slots(frames, hop_seq, cfg);

state = WAIT_READY;
while state ~= DONE
    switch state
        case WAIT_READY
            tx_beacon_on_anchor(meta);
            fb = try_receive_feedback();
            if is_ready(fb, session_id)
                state = START_COUNTDOWN;
            end

        case START_COUNTDOWN
            for k = 3:-1:1
                tx_start_control(meta, k);
                pause_control_slot();
            end
            state = DATA_ONCE;

        case DATA_ONCE
            t0 = tic;
            for slot_id = 1:total_slots
                radio_tx.CenterFrequency = cfg.Carrier_set(hop_seq(slot_id));
                settle_radio();
                radio_tx(slot_cache{slot_id});
                fb = try_receive_feedback();  % 只显示，不触发重传
                update_tx_ui(fb, slot_id);
            end
            tx_duration = toc(t0);
            state = END_LISTEN;

        case END_LISTEN
            tx_end_control(session_id);
            fb = listen_result_for_timeout();
            log_result(fb, tx_duration);
            state = DONE;
    end
end
```

### 12.2 接收端

```matlab
init_usrp_txrx();
state = WAIT_BEACON;

while state ~= DONE
    switch state
        case WAIT_BEACON
            radio_rx.CenterFrequency = cfg.anchor_freq;
            sig = radio_rx();
            beacon = decode_control(sig);
            send_ready_discovery_if_needed();
            if is_valid_beacon(beacon)
                session = beacon.session;
                send_rx_ready(session);
                state = READY_SENT;
            end

        case READY_SENT
            sig = radio_rx();
            start_info = decode_start(sig);
            send_rx_ready(session);
            if start_info.valid
                hop_seq = build_hop_sequence(start_info.hop_seed, ...
                    start_info.total_slots, cfg.Carrier_set);
                state = FOLLOW_HOP;
            end

        case FOLLOW_HOP
            for slot_id = 1:total_slots
                radio_rx.CenterFrequency = cfg.Carrier_set(hop_seq(slot_id));
                settle_radio();
                sig = radio_rx();
                [frames, phy_metrics] = decode_hop_slot(sig, slot_id);
                update_frame_cache(frames);
                metrics = compute_live_metrics(phy_metrics, frame_cache);
                tx_feedback_telemetry(metrics);
                update_rx_ui(metrics);
            end
            state = FEC_REBUILD;

        case FEC_REBUILD
            [file_bytes, rebuild_info] = rebuild_file_from_fec(frame_cache);
            metrics = compute_final_metrics(file_bytes, rebuild_info);
            write_recovered_file(file_bytes, meta);
            state = RESULT_REPORT;

        case RESULT_REPORT
            for i = 1:10
                tx_feedback_result(metrics);
            end
            update_rx_ui(metrics);
            state = DONE;
    end
end
```

## 13. 速度估算与演示建议

当前代码每槽只发 10 包，且每个包都有独立长前导，视频会很慢。建议分两步优化：

1. 第一阶段低风险改造：保留当前每包前导，取消循环广播和重传，加入启动握手、跳频、反向遥测、外层 FEC，并把每槽包数改为自动填满。
2. 第二阶段速度改造：升级为槽级 preamble + 连续 LDPC codewords，显著减少同步头开销。

推荐演示文件大小：

| 类型 | 建议大小 |
|---|---:|
| 文本 | 1-20 KB |
| 图片 | 20-150 KB |
| 视频普通文件 | 100-500 KB |
| 视频帧容器 | 5-15 s，低分辨率，按 JPEG 帧发送 |

演示时可以准备三组 FEC/跳频配置：

1. 快速模式：QPSK，K=24,R=4，适合无干扰或弱干扰。
2. 默认模式：QPSK，K=24,R=8，适合常规展示。
3. 强鲁棒模式：BPSK 扩频，K=16,R=8，适合强干扰对比，但速度慢。

## 14. 和现有代码的迁移关系

现有可复用部分：

1. `link_phy_defs.m` 中的 LDPC 矩阵、CRC 多项式、扰码序列、前导序列。
2. `Data_trans_sig_Gen.m` 的 CRC、扰码、LDPC、交织、QPSK/BPSK 扩频调制流程。
3. `Data_Rece_sig_Gen.m` 的同步、相位估计、软解调、LDPC、CRC 检测流程。
4. `Par_trans_sig_Gen.m` 和 `Par_Rece_sig_Gen.m` 的反馈物理层流程。
5. UI 中频谱、时域、状态显示逻辑。

必须删除或禁用的旧逻辑：

1. `Transmitter_Main_UDP*.m` 中 `round_count` 循环全包广播。
2. `Transmitter_Main.m` 中 `STATE_REPAIR` 定点补包。
3. `Receiver_main_0416.m` 中 REQUEST_MISSING 反馈逻辑。
4. COMPLETE 持续反馈可以保留为 RESULT 的一种，但不能驱动 TX 重新发业务数据。

建议新增主程序，不直接覆盖旧文件：

1. `发射机/Transmitter_one_shot_fh.m`
2. `接收机/Receiver_one_shot_fh.m`

这样旧的可靠传输版本可以保留作对照，新方案用于老师要求的干扰效果展示。

## 15. 最小可行实现顺序

1. 先实现启动握手：BEACON、READY、START、END，确保两端不用同时启动。
2. 把 TX 主循环改成 DATA_ONCE，只发送一遍现有 44B 包，不重传、不循环。
3. 加入反向 TELEMETRY v2，先回传 SNR、CRC 成功数、缺包数、吞吐。
4. 加入跳频：先每个 burst 一个频点，RX 根据 START 中的 hop_seed 跟跳。
5. 加入外层 FEC：先 K=24,R=8，恢复丢包。
6. 改 40B V2 业务帧，把 hop/fec 字段写进物理帧。
7. 做槽级 preamble + 连续 codeword，提高视频演示速度。
8. 加入图片/视频质量指标和 UI 展示。

## 16. 验收标准

1. TX 先启动、RX 后启动时，TX 不会提前发送业务数据；RX_READY 后才发送一次。
2. RX 先启动、TX 后启动时，可以自动进入同一会话。
3. 日志中每个业务 FrameID 最多发送一次。
4. 干扰关闭时，文本和图片可完整恢复，文件 CRC32 匹配。
5. 开启干扰时，UI 能显示 SNR 下降、同步失败、Pre-FEC PER 上升、Post-FEC PER 和文件质量变化。
6. 跳频开启后，UI 能显示每个频点的 SNR/PER 差异。
7. 反向链路能周期回传 SNR 等信息，且反馈帧中保留字段不影响当前解析。
8. 视频演示模式下，即使部分帧丢失，也能输出可播放的重建视频，并显示帧恢复率和冻结帧率。
