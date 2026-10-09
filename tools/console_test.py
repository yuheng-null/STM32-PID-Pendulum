"""端到端验证串口调参协议（app/console）。

用法: python console_test.py [COM口]

逐条发命令、读响应，并**按预期格式做断言**。
只回 "OK" 不算通过 —— 必须字段名和格式都对得上，
否则 agent 解析不了，等于协议没实现。
"""
import re
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
BAUD = 115200

fails = []


def send(sp, cmd):
    """发一条命令，读一行响应。"""
    sp.reset_input_buffer()
    sp.write((cmd + "\n").encode())
    line = sp.readline().decode("ascii", errors="replace").strip()
    print(f"  > {cmd:<18} < {line}")
    return line


def expect(label, line, pattern):
    if re.match(pattern, line):
        print(f"    [OK] {label}")
    else:
        print(f"    [!!] {label}: 期望 {pattern}")
        fails.append(label)


def main():
    print(f"open {PORT} @ {BAUD} 8N1")
    with serial.Serial(PORT, BAUD, timeout=1.5) as sp:
        # dtr/rts 拉低：CH340 上这两个信号没实际用途，
        # 但打开串口时若被拉高，某些板子会因此复位。
        sp.dtr = False
        sp.rts = False
        time.sleep(0.3)

        print("\n=== 0. 静态检查（不该有自发输出）===")
        sp.reset_input_buffer()
        time.sleep(0.6)
        noise = sp.read(4096)
        print(f"  自发字节数 = {len(noise)}  (期望 0)")
        if noise:
            print(f"  [!!] 收到意外数据: {noise[:80]!r}")
            fails.append("no-spontaneous-output")

        print("\n=== 1. HELP（自描述）===")
        line = send(sp, "HELP")
        expect("HELP", line, r"^OK CMD=.*SET,GET,STAT,RUN,SWING,STOP,ZERO,TARGET,STREAM,HELP.*")
        expect("HELP-PARAM", line, r".*PARAM=AKP,AKI,AKD,PKP,PKI,PKD,CENTER,RANGE,START,OFFSET,SWP,SWT.*")

        print("\n=== 2. GET ALL（一次读全）===")
        line = send(sp, "GET ALL")
        expect("GETALL", line, r"^OK AKP=[-\d.]+ AKI=[-\d.]+ AKD=[-\d.]+ "
                               r"PKP=[-\d.]+ PKI=[-\d.]+ PKD=[-\d.]+ "
                               r"CENTER=[-\d.]+ RANGE=[-\d.]+ START=[-\d.]+ "
                               r"OFFSET=[-\d.]+ SWP=[-\d.]+ SWT=[-\d.]+ TARGET=-?\d+$")
        defaults = dict(re.findall(r"(\w+)=(-?[\d.]+)", line))
        print(f"    默认值: {defaults}")
        assert float(defaults["AKP"]) == 4.5, "AKP 默认值不对"
        assert float(defaults["CENTER"]) == 2086.0, "CENTER 默认值不对"

        print("\n=== 3. SET / GET 往返 ===")
        line = send(sp, "SET AKP 5.25")
        expect("SET", line, r"^OK AKP=5\.250$")
        line = send(sp, "GET AKP")
        expect("GET", line, r"^OK AKP=5\.250$")
        line = send(sp, "set akp 4.5")          # 小写命令 + 小写参数名
        expect("SET-lowercase", line, r"^OK AKP=4\.500$")

        print("\n=== 4. 浮点负值与格式 ===")
        line = send(sp, "SET PKD -82.08")
        expect("SET-negative", line, r"^OK PKD=-82\.080$")
        line = send(sp, "SET PKD 82.08")
        expect("SET-restore", line, r"^OK PKD=82\.080$")

        print("\n=== 5. STAT（实时状态）===")
        line = send(sp, "STAT")
        expect("STAT", line, r"^OK ANGLE=\d+ POS=-?\d+ SPD=-?\d+ "
                             r"ATAR=[-\d.]+ AOUT=[-\d.]+ POUT=[-\d.]+ PWM=-?\d+ "
                             r"RUN=[01] ST=[0-2] SWR=[0-3]$")

        print("\n=== 6. TARGET / ZERO ===")
        line = send(sp, "TARGET 408")
        expect("TARGET", line, r"^OK TARGET=408$")
        line = send(sp, "GET ALL")
        expect("TARGET-persist", line, r".*TARGET=408$")
        line = send(sp, "TARGET 9999")           # 应该被拒
        expect("TARGET-oor", line, r"^ERR OUT_OF_RANGE 9999$")
        line = send(sp, "ZERO")
        expect("ZERO", line, r"^OK POS=0 TARGET=0$")

        print("\n=== 7. RUN 的拒绝路径 ===")
        # 7a：**这是两窗口设计要挡住的那一种**。
        #     CENTER 保持默认 2086，摆杆若正自由下垂（读数约 1883）：
        #       CENTER ± RANGE(500) = [1586,2586] → 1883 在里面（会放行！）
        #       CENTER ± START(150) = [1936,2236] → 1883 在外面（应拒绝）
        #     所以这一条同时验证了「START 确实比 RANGE 严，且确实生效」。
        line = send(sp, "STAT")
        cur = int(re.search(r"ANGLE=(\d+)", line).group(1))
        if 1586 <= cur <= 2586 and not (1936 <= cur <= 2236):
            line = send(sp, "RUN")
            expect("hang-refused", line, r"^ERR NOT_READY ANGLE_OUT_OF_WINDOW$")
        else:
            print(f"      跳过 7a：当前 ANGLE={cur} 不在「RANGE 内但 START 外」这个区间，"
                  f"没法验证两窗口的差别（摆杆可能被扶着或停在别处）")

        # 7b：纯逻辑 —— 把 CENTER 推到远离当前角度的地方，也应当拒绝。
        send(sp, f"SET CENTER {2086 - 900 if cur > 2086 else 2086 + 900}")
        line = send(sp, "RUN")
        expect("far-refused", line, r"^ERR NOT_READY ANGLE_OUT_OF_WINDOW$")
        line = send(sp, "STAT")
        expect("still-stopped", line, r".*RUN=0 ST=0 SWR=")
        send(sp, "SET CENTER 2086")

        print("\n=== 8. RUN 的受理 + 倒下自动保护（**电机保证不动**）===")
        # ⚠️ 安全设计：把两个环的增益和静摩擦补偿**全部设成 0**。
        #    这样控制环照常跑（分频、poll/trigger、倒下判定全都执行），
        #    但角度环输出恒为 0、offset 也是 0 ⟹ 写到电机的占空比恒为 0。
        #    于是可以完整验证「受理 → 运行 → 自动停机」这条路，
        #    而电机一点都不会动。
        #    （第一版没这么做，直接 RUN，电机以 ~66% 占空比转了起来。）
        for c in ("SET AKP 0", "SET AKI 0", "SET AKD 0", "SET OFFSET 0"):
            send(sp, c)

        st = send(sp, "STAT")
        cur = int(re.search(r"ANGLE=(\d+)", st).group(1))
        send(sp, f"SET CENTER {cur}")        # 窗口对准当前角度
        # START 也显式设回来：这一组要测的是"RUN 的受理路径"，前提是准入窗口
        # 正常。**不能假设进来时环境是干净的** —— 2026-10-10 踩到过：上一次
        # 运行把 START 留在 1，快照/回滚又忠实地把它保存下来，于是从那次起
        # 每次跑都在这一组失败（"第一次过、第二次挂"）。
        send(sp, "SET START 150")
        line = send(sp, "RUN")
        expect("run-accepted", line, r"^OK RUN=1$")

        line = send(sp, "STAT")
        expect("running", line, r".*RUN=1 ST=1 SWR=")
        print(f"      （运行中，检查 PWM 是不是 0）: {line}")
        pwm = int(re.search(r"PWM=(-?\d+)", line).group(1))
        if pwm != 0:
            print(f"      [!!] PWM={pwm}，增益归零后不该有输出！")
            fails.append("zero-gain-still-outputs")

        # 把窗口推走 → 倒下保护应当立刻停机
        send(sp, f"SET CENTER {cur + 800}")
        time.sleep(0.05)
        line = send(sp, "STAT")
        expect("auto-stop", line, r".*RUN=0 ST=0 SWR=")
        send(sp, "STOP")

        # 恢复参数：**用开头那次 GET ALL 读到的真值**，不要写死。
        #
        # ⚠️ 这里原先是一串写死的常量，2026-10-10 踩到过：位置环的默认值
        #    从 ×18 改回官方原值之后，这串常量没跟着改，于是**每跑一次回归
        #    就把运行中的参数改回错的那一组**。现象极具迷惑性 ——
        #    「调完参测一次，反而更抖了」，而且 GET ALL 也不容易看出问题
        #    （数字变了，但没人记得住默认值该是多少）。
        #
        #    改成快照/回滚之后，测试就不再是"另一个事实来源"了。
        for name, val in defaults.items():
            if name == "TARGET":            # 这不是参数，是位置目标
                continue
            send(sp, f"SET {name} {val}")

        print("\n=== 8b. SWING：自动启摆的入口（**电机保证不动**）===")
        # ⚠️ 安全设计：把启摆脉冲占空比设成 0。
        #    启摆状态机照常跑（分频、判定、相位切换、超时计时全都执行），
        #    但每一个脉冲的占空比都是 0 ⟹ 电机一点都不会动。
        #    于是可以安全验证「SWING → 进启摆态 → STOP 退出」这条路。
        #    （用非零 SWP 会真的把摆杆荡起来，那是要人在场看着做的事，
        #      不能放在自动回归里。）
        send(sp, "SET SWP 0")
        # 另外把 START 压到 1，让"交棒"条件不可能成立。
        #
        # ⚠️ 不这么做的话这一组**不确定**：如果跑测试时摆杆恰好是立着的，
        #    SWING 进去 5 ms 内就被交棒接走（ST 变 1、SWR 记 1），
        #    后面的 STOP 就不是"打断启摆"而是"停掉双环"，SWR 也就不是 3。
        #    2026-10-10 实测碰上过。窗口压到 ±1 之后，启摆一定停在 SWING_UP，
        #    这一组才真的可复现。
        send(sp, "SET START 1")

        line = send(sp, "SWING")
        # SWING 是**无条件**启摆：不检查角度、不猜姿态，所以恒为 ST=2。
        # （2026-10-10 去掉了原先"角度在窗口内就直接进双环"的捷径 ——
        #   摆杆还在摆动时读数会路过窗口，那条捷径会误判。）
        expect("swing-accepted", line, r"^OK ST=2$")

        line = send(sp, "STAT")
        print(f"      （SWING 之后）: {line}")
        pwm = int(re.search(r"PWM=(-?\d+)", line).group(1))
        if pwm != 0:
            print(f"      [!!] SWP=0 时 PWM={pwm}，不该有输出！")
            fails.append("swing-zero-pwm-still-outputs")

        line = send(sp, "STOP")
        expect("swing-stop", line, r"^OK RUN=0$")
        line = send(sp, "STAT")
        # 被 STOP 打断的启摆应记成 SWR=3（区分于超时的 2）
        expect("swing-aborted", line, r".*ST=0 SWR=3$")

        send(sp, "SET SWP 630")
        # START 也要还原：它刚才被压到 1 过。**漏了这一步会让下一次运行
        # 在第 8 组就失败**（RUN 的准入窗口变成 ±1 码，必然被拒）——
        # 2026-10-10 实测踩到，现象是"同一个测试第一次过、第二次挂"。
        send(sp, f"SET START {defaults['START']}")

        print("\n=== 9. 错误路径（agent 靠这些自我纠正）===")
        line = send(sp, "FOO")
        expect("unknown-cmd", line, r"^ERR UNKNOWN_CMD FOO$")
        line = send(sp, "SET AKQ 1.0")
        expect("unknown-param", line, r"^ERR UNKNOWN_PARAM AKQ$")
        line = send(sp, "SET AKP 1.2.3")
        expect("bad-value", line, r"^ERR BAD_VALUE 1\.2\.3$")
        line = send(sp, "SET RANGE 0")           # 合法数值、非法范围
        expect("range-0", line, r"^ERR OUT_OF_RANGE RANGE$")
        line = send(sp, "GET")                   # 缺参数
        expect("usage", line, r"^ERR USAGE GET <NAME>\|ALL$")
        line = send(sp, "SET CENTER 99999")
        expect("center-oor", line, r"^ERR OUT_OF_RANGE CENTER$")

        print("\n=== 10. 超长行 ===")
        line = send(sp, "A" * 100)
        expect("too-long", line, r"^ERR LINE_TOO_LONG$")

        print("\n=== 11. 空行不算错误（应当静默）===")
        sp.reset_input_buffer()
        sp.write(b"\r\n\r\n\n")
        time.sleep(0.4)
        extra = sp.read(4096)
        print(f"  空行后的回复字节数 = {len(extra)}  (期望 0)")
        if extra:
            print(f"  [!!] {extra!r}")
            fails.append("empty-line-quiet")

        print("\n=== 12. STREAM 数据流 ===")
        line = send(sp, "STREAM 1")
        expect("STREAM-on", line, r"^OK STREAM=1$")

        # 用**墙钟窗口**统计速率，不能像早先那样 "read(8192) 然后除以 1 秒" ——
        # read(n) 会一直阻塞到凑满 n 字节或超时，实际耗时远大于 sleep 的时间，
        # 除出来会得到一个虚高的速率（我第一版就是这么错的：算出 213 行/秒）。
        sp.reset_input_buffer()
        rows = []
        t_start = time.time()
        while time.time() - t_start < 2.0:
            chunk = sp.read(4096)
            if chunk:
                rows.extend(l for l in chunk.decode("ascii", "replace").splitlines() if l.strip())
        elapsed = time.time() - t_start
        rate = len(rows) / elapsed
        print(f"  {elapsed:.2f} 秒收到 {len(rows)} 行 CSV  →  {rate:.1f} 行/秒 (期望 ~50)")
        for r in rows[:4]:
            print(f"    > {r}")
        if rows:
            expect("stream-csv", rows[0], r"^\d+,-?\d+,-?\d+,[-\d.]+,[-\d.]+,[-\d.]+,-?\d+,[01]$")
            if not (35.0 <= rate <= 65.0):
                print(f"    [!!] 速率偏离 20ms 周期太多")
                fails.append("stream-rate")
        else:
            fails.append("stream-silent")

        # 关流。⚠️ 响应前面可能还排着一行已经进了发送缓冲的 CSV，
        # 所以不能只 readline() 一次就断言 —— 要一直读到 OK/ERR 前缀那行为止。
        sp.reset_input_buffer()
        sp.write(b"STREAM 0\n")
        resp, deadline = "", time.time() + 2.0
        while time.time() < deadline:
            l = sp.readline().decode("ascii", "replace").strip()
            if l:
                resp = l
                if l.startswith("OK") or l.startswith("ERR"):
                    break
        print(f"  > STREAM 0           < {resp}")
        expect("STREAM-off", resp, r"^OK STREAM=0$")

        # 关流之后必须**彻底静默**，否则上位机永远收不到干净的结尾
        time.sleep(0.4)
        tail = sp.read(4096)
        print(f"  关流后残留字节数 = {len(tail)}  (期望 0)")
        if tail:
            print(f"  [!!] {tail[:80]!r}")
            fails.append("stream-after-off")

    print("\n" + "=" * 46)
    if fails:
        print(f"失败 {len(fails)} 项: {fails}")
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
