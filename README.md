# f401_l8_2 TOF 串口协议说明

本工程用于 STM32F401 + VL53L8CX 多传感器测距。

当前支持两种输出模式：

- 二进制帧模式（默认，高速可靠）
- 文本调试模式（便于联调）

可通过串口命令切换：

- `o`：切换输出模式
- `r`：切换 4x4 / 8x8 分辨率
- `s`：切换 signal/ambient 输出配置
- `c`：清屏提示

## 1. 二进制帧格式（小端）

每帧表示“一个传感器的全部 zone（target index = 0）”。

固定头字段总长度为 16 字节，随后是变长 payload，再跟 2 字节 CRC。

### 1.1 字段定义与偏移

| 偏移 | 长度 | 字段 | 类型 | 说明 |
|---|---:|---|---|---|
| 0 | 1 | sof0 | u8 | 固定 `0xAA` |
| 1 | 1 | sof1 | u8 | 固定 `0x55` |
| 2 | 1 | version | u8 | 协议版本，当前 `0x01` |
| 3 | 1 | frame_type | u8 | 帧类型：`0x01=测距帧`，`0x02=错误帧` |
| 4 | 2 | seq | u16 LE | 发送序号，发送成功后递增 |
| 6 | 4 | timestamp_ms | u32 LE | `HAL_GetTick()` 毫秒时间戳 |
| 10 | 1 | sensor_id | u8 | 传感器索引（0,1,2...） |
| 11 | 1 | resolution_code | u8 | `1=4x4(16 zone)`，`2=8x8(64 zone)`，`0=未知` |
| 12 | 1 | zone_count | u8 | 本帧 zone 数 |
| 13 | 1 | target_index | u8 | 当前固定为 `0` |
| 14 | 2 | payload_len | u16 LE | payload 字节数 |
| 16 | N | payload | bytes | zone 数据区，顺序即 zone 索引 |
| 16+N | 2 | crc16 | u16 LE | CRC16-CCITT-FALSE |

### 1.2 payload（zone 记录）

每个 zone 固定 3 字节，连续排列：

- `dist_mm`：u16 LE，毫米
- `status`：u8

因此：

- `payload_len = zone_count * 3`
- 第 `k` 个 zone 的记录起始偏移：`16 + 3*k`

### 1.2.1 错误帧 payload（frame_type=0x02）

错误帧用于事件触发时即时上报，当前 payload 固定 5 字节：

| payload 偏移 | 长度 | 字段 | 类型 | 说明 |
|---|---:|---|---|---|
| 0 | 1 | error_code | u8 | 错误类型 |
| 1 | 4 | error_value | u32 LE | 错误相关值 |

当前约定：

- `error_code=0x01`：传感器离线/读数失败（`error_value` 为对应状态码）
- `error_code=0x02`：UART 发送失败计数增长（`error_value` 为累计失败次数）

错误帧头里的 `sensor_id` 表示关联传感器；`resolution_code/zone_count/target_index` 在错误帧中不使用（置 0）。

### 1.3 zone 编号规则

帧内不单独发送 zone_id，采用顺序隐含：

- 第 0 条记录对应 zone 0
- 第 1 条记录对应 zone 1
- ...

上位机请使用 `zone_count + resolution_code` 确认当前布局（4x4/8x8）。

### 1.4 status 与无目标编码

- `status = 0`：有效测距
- `status = 255`：no update / no target
- 其它值：设备状态码（透传）

无目标时：

- `dist_mm = 65535 (0xFFFF)`
- `status = 255`

## 2. CRC16 算法

采用 CRC16-CCITT-FALSE：

- 多项式：`0x1021`
- 初值：`0xFFFF`
- 不反射输入、不反射输出
- 无最终异或
- 计算范围：从 `version`（偏移 2）到 payload 末尾（不含 SOF 和 CRC 字段）

伪代码：

```text
crc = 0xFFFF
for byte in frame[2 : 16 + payload_len]:
	crc ^= (byte << 8)
	repeat 8 times:
		if (crc & 0x8000) != 0:
			crc = (crc << 1) ^ 0x1021
		else:
			crc = crc << 1
	crc &= 0xFFFF
```

## 3. 示例帧

假设：

- sensor_id = 1
- 4x4 模式（zone_count=16, resolution_code=1）
- seq = 0x1234
- timestamp_ms = 0x000186A0
- 仅演示前 3 个 zone：
  - zone0: dist=500, status=0
  - zone1: dist=65535, status=255（no target）
  - zone2: dist=820, status=0

则 payload 前 9 字节为：

- zone0: `F4 01 00`
- zone1: `FF FF FF`
- zone2: `34 03 00`

帧头（不含 CRC）示意：

```text
AA 55 01 01 34 12 A0 86 01 00 01 01 10 00 30 00 ...payload...
```

说明：

- `0x30 0x00` 是 payload_len=48（16*3）
- CRC 需要按完整 payload 重新计算后填入末尾 2 字节

## 4. 文本调试模式格式

切换到文本模式后，每次输出：

1. 一行头：`[TOF][TXT] sensor=<id> zones=<count>`
2. 每个 zone 一行：`<zone>,<dist>,<status>`

无目标示例：`7,65535,255`

## 5. 上位机解析建议

1. 按 SOF `AA 55` 做重同步。
2. 读取固定头 16 字节，取出 `payload_len`。
3. 读取 payload 与尾部 CRC。
4. 按同算法验 CRC，失败则丢弃并继续从下一个 SOF 重同步。
5. 使用 `seq` 统计丢帧，使用 `sensor_id` 区分多传感器。
6. 业务层可按需对 `status=255` 做“显示保持”，但原始数据层不应改写为旧值。