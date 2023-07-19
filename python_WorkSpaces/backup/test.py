# global int a = 1
import numpy as np
a = np.array([0, 0, 0])
b = 0

def p1():
    global a, b
    a = np.array([2, 4, 6])
    b = a[0]

def p2():
    global a, b
    print('global', a)
    a = np.array([0, 1, 0])
    b = a[1]

# print(a, b)
# p1()
# print(a, b)
# p2()
# print(a, b)

# import pyrealsense2 as rs 

# ctx = rs.context()
# devices = ctx.query_devices()
# print(len(devices))
# print(devices[0])
# print(devices[1])

flag = 1 # 仮の受信データとして100を使用

send_data = flag << 4

print(send_data)

