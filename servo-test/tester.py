import json
import struct
from serial import Serial
import time
# import matplotlib.pyplot as plt


PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5626036531-if00"
BAUD = 2_000_000


def esp32_reset(serial: Serial):
    print("Resetting...")
    serial.dtr = False
    serial.rts = False
    time.sleep(0.1)
    serial.rts = False
    serial.dtr = False
    serial.open()
    serial.read_until(b"start\r\n")
    time.sleep(0.1)


def convert(x):
    x = x & ~(1 << 15)
    if x & 0x4000: x -= 0x8000
    return x
    # return 360 / (2**15) * x


def main(config):
    serial = Serial(baudrate=BAUD, timeout=1, exclusive=True)
    serial.port = PORT  # prevents pyserial from automatically opening
    esp32_reset(serial)

    cfg = " ".join(str(int(x)) for x in config.values()) + "\n"
    serial.write(cfg.encode())
    serial.read_until(b"end\r\n")

    data = {"angle": [], "speed": [], "time": []}
    angle_sum = 0
    while True:
        line = serial.readline()
        if not line: break
        angle, speed, time = map(lambda x: int(x, 16), line.decode().split())
        angle, speed = convert(angle), convert(speed)
        angle_sum += angle
        data["angle"].append(angle)
        data["speed"].append(speed)
        data["time"].append(time)
    zero_angle = int(angle_sum / len(data["angle"]))
    data["angle"] = [x - zero_angle for x in data["angle"]]
    serial.close()

    name = "_".join(k for k, v in config.items() if v)
    if not name: name = "default"
    plt.title(name)
    plt.tight_layout()
    plt.plot(data["time"], data["angle"], "b", label="angle (int)")
    plt.plot(data["time"], data["speed"], "r", label="speed (int)")
    plt.xlabel("time (ms)")
    plt.legend()
    plt.gcf().set_size_inches(20, 5)
    plt.savefig(f"plots/{name}.png", bbox_inches='tight')
    plt.clf()


def main2(name):
    serial = Serial(baudrate=BAUD, timeout=1, exclusive=True)
    serial.port = PORT  # prevents pyserial from automatically opening
    esp32_reset(serial)
    serial.read_until(b"end\r\n")
    print("start")

    data = {"angle": [], "speed": [], "time": []}
    while True:
        line = serial.read_until("\n\n", size=14)[:-2]
        if not line: break
        # print(len(line), line)
        angle, speed, time = struct.unpack("ffI", line)
        # print(angle, speed, time)

        # break
        # angle, speed, time = line.decode().split(", ")
        data["angle"].append(float(angle))
        data["speed"].append(float(speed))
        data["time"].append(int(time))

        if data["time"][-1] > 60000: break

    with open(f"{name}.json", "w") as file:
        json.dump(data, file)


def main3(name):
    with open(f"{name}.json") as file:
        data = json.load(file)

    import cartpole.log as log
    from cartpole.common import State

    mcap = log.MCAPLogger('test.mcap')
    print("writing")

    start_ts = data["time"][0]
    for i in range(len(data["angle"])):
        ts = (data["time"][i] - start_ts) / 1000
        angle, speed = data["angle"][i], data["speed"][i]

        state = State(
            pole_angle=angle,
            pole_angular_velocity=speed,
            stamp=ts,
        )
        mcap.publish("/cartpole/state", state, ts)
    mcap.close()


def main4(name):
    import mcaplog
    from pydantic import BaseModel

    class ServoMsg(BaseModel):
        target_angle: float
        encoder_angle: float
        adc_value: float

    mcap = mcaplog.MCAPLogger(f"data/{name}.mcap")
    serial = Serial(baudrate=BAUD, timeout=2.0, exclusive=True)
    serial.port = PORT  # prevents pyserial from automatically opening
    esp32_reset(serial)
    serial.read_until(b"end\r\n")
    print("start")
    
    prev_ang = -1
    while True:
        line = serial.read_until(b"\n").decode().strip()
        if not line or line == "stop": break
        assert line.startswith("ang")
        ang, adc, enc = [float(x.split(" ")[1]) for x in line.split(", ")]
        msg = ServoMsg(target_angle=ang, encoder_angle=enc, adc_value=adc)
        mcap.publish("/cartpole/state", msg, time.perf_counter())
        if ang != prev_ang:
            msg.target_angle = prev_ang
            mcap.publish("/kek", msg, time.perf_counter())
            prev_ang = ang
    print("stop")
    mcap.close()


if __name__ == "__main__":
    # configs = [
    #     {"slow_upd_rate": 0, "half_precision": 0, "prediction": 0},
    #     {"slow_upd_rate": 1, "half_precision": 0, "prediction": 0},
    #     {"slow_upd_rate": 1, "half_precision": 0, "prediction": 1},
    #     {"slow_upd_rate": 1, "half_precision": 1, "prediction": 0},
    #     {"slow_upd_rate": 1, "half_precision": 1, "prediction": 1},
    # ]
    # for c in configs:
    #     main(c)

    name = "servo_new_v3"
    # main2(name)
    # main3(name)
    main4(name)
