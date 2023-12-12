from statistics import mean
from scapy.all import *
import matplotlib.pyplot as plt


def hex_convert(bytes: bytes):
    x = 0
    res = 0
    for byte in bytes:
        res += 16 ** (2 * x) * byte
        x += 1
    return res


def ts_convert(ts):
    ts /= 1e6
    time_array = time.localtime(ts - 8 * 3600)
    res = time.strftime("%Y-%m-%d %H:%M:%S:", time_array) + str('{:-016f}'.format(ts))[-6:]
    return res


def read_packet(pacp_file: str):
    file = rdpcap(pacp_file)
    ts_frame_start = []
    points = []
    points_tmp = 0
    start_flag = 0
    frame_count = 0
    for data in file:
        if 'UDP' in data:
            if data['UDP'].dport == 8010:
                raw_load = data[Raw].load
                magic_num = (raw_load[0:2])
                if magic_num == b'\x6A\x17':
                    item = struct.unpack('I', raw_load[38:42])[0]
                    item_raw = '{:032b}'.format(item)
                    item_type = int(item_raw[24:], 2)
                    if item_type != 3 and item_type != 2:
                        timestamp = struct.unpack('d', raw_load[16:24])[0]
                        frame_idx = hex_convert(raw_load[26:34])
                        sub_idx = hex_convert(raw_load[34:36])
                        item_number = int(item_raw[:24], 2)
                        # print(frame_idx)
                        if start_flag != 0:
                            points_tmp += item_number

                        if sub_idx == 0:
                            print(frame_idx)
                            frame_count += 1
                            start_flag = 1
                            ts_frame_start.append(timestamp)
                            points.append(points_tmp)
                            points_tmp = 0

    points.pop(0)
    points.pop(-1)
    print(frame_count)
    return ts_frame_start, points


def frame_rate(args: []):
    ts_list = args[0]
    points = args[1]
    frame_num = len(ts_list)
    diff = []
    for i in range(frame_num - 1):
        ts_left = ts_list[i]
        ts_right = ts_list[i + 1]
        diff.append((ts_right - ts_left) / 1e4)
    print(diff)
    plot(diff, points)


def plot(diff, points):
    plt.figure(figsize=(16, 9))
    # Frame rate
    x = list(range(0, len(diff)))
    y = diff
    ax1 = plt.subplot(2, 1, 1)
    ax1.set_title("Frame rate")
    plt.plot(x, y, color="red", label='frame rate', linestyle='--', linewidth=1.0)
    plt.axhline(y=mean(diff), color='green', label='mean=%s' % mean(diff), linewidth=1.0)
    plt.legend(loc='upper right')

    ax1 = plt.subplot(2, 1, 2)
    ax1.set_title("Points per frame")
    x = list(range(0, len(points)))
    y = points
    plt.plot(x, y, color="red", label='points per frame', linestyle='--', linewidth=1.0)
    plt.axhline(y=mean(points), color='green', label='mean=%s' % mean(points), linewidth=1.0)
    plt.legend(loc='upper right')
    plt.show()


if __name__ == "__main__":
    pcap_file = "/home/demo/Downloads/A00963_Deepway_Collect_orin-1_20230919173500_.pcap"
    frame_rate(read_packet(pcap_file))
