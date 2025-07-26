# 视觉题目脱机调整阈值，并实现红绿光点检测和矩形识别，以及与单片机通信

import time, os, sys, ujson, math
from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA, Pin, TOUCH, UART

# 全局变量定义
lcd_width = 640
lcd_height = 480
current_display_width = lcd_width
current_display_height = lcd_height

# 硬件初始化
fpioa = FPIOA()

# UART 初始化
fpioa.set_function(3, FPIOA.UART1_TXD)
fpioa.set_function(4, FPIOA.UART1_RXD)
uart = UART(UART.UART1, 9600)

# 按键初始化
fpioa.set_function(21, FPIOA.GPIO21)
KEY = Pin(21, Pin.IN, Pin.PULL_UP)

# 触摸屏初始化
tp = TOUCH(0)

# 阈值文件路径
THRESHOLD_FILE = '/sdcard/thresholds.txt'

# 默认阈值
def default_thresholds():
    return {
        'gray': [80, 200],
        'red': [27, 89, 23, 104, -2, 67],
        'green': [30, 100, -64, -9, -32, 32]
    }

# 加载阈值
def load_thresholds():
    try:
        with open(THRESHOLD_FILE, 'r') as f:
            lines = f.readlines()
            thresholds = {}
            for line in lines:
                key, value = line.strip().split(':')
                thresholds[key] = [int(x) for x in value.split(',')]
            return thresholds
    except Exception as e:
        print(f"加载阈值失败: {e}, 使用默认阈值")
        return default_thresholds()

# 保存阈值
def save_thresholds(thresholds):
    try:
        with open(THRESHOLD_FILE, 'w') as f:
            for key, value in thresholds.items():
                f.write(f"{key}:{','.join(map(str, value))}\n")
        print("阈值保存成功")
    except Exception as e:
        print(f"保存阈值失败: {e}")

threshold_dict = load_thresholds()

# 摄像头初始化
sensor = Sensor(width=1280, height=960)
sensor.reset()
sensor.set_framesize(width=640, height=480)  # 统一使用640x480
sensor.set_pixformat(Sensor.RGB565)

# 显示屏初始化
Display.init(Display.ST7701, width=lcd_width, height=lcd_height, to_ide=True)
MediaManager.init()
sensor.run()
clock = time.clock()

# 调整模式标志
in_threshold_mode = False
threshold_modes = ['gray', 'red', 'green']
threshold_mode = 'gray'  # 当前调整模式

# 颜色阈值定义
RED_LASER_THRESHOLD = (27, 89, 23, 104, -2, 67)
GREEN_LASER_THRESHOLDS = (30, 100, -64, -9, -32, 32)

# 矩形识别标志
trigger_rect = 0
rect_coords = []

def sort_corners_clockwise(points):
    """将四个点按顺时针顺序排序（左上开始）"""
    if len(points) != 4:
        return points

    # 计算中心点
    center_x = sum(p[0] for p in points) / 4
    center_y = sum(p[1] for p in points) / 4

    # 计算极角并排序
    angles = []
    for p in points:
        dx = p[0] - center_x
        dy = p[1] - center_y
        angles.append(math.atan2(dy, dx))

    # 逆时针排序后反转得到顺时针
    sorted_points = [p for _, p in sorted(zip(angles, points), key=lambda x: x[0], reverse=False)]

    # 重新排列起点为左上附近
    x_sorted = sorted(sorted_points, key=lambda p: p[0])
    left_points = x_sorted[:2]
    left_sorted = sorted(left_points, key=lambda p: p[1])

    start_index = sorted_points.index(left_sorted[0])
    return sorted_points[start_index:] + sorted_points[:start_index]

def pack_data(data, flag):
    """数据打包函数"""
    header = 0xFF
    footer = 0xFE
    buffer = bytearray([header, flag])
    for num in data:
        num = max(0, min(num, 320))
        buffer.append((num >> 8) & 0xFF)
        buffer.append(num & 0xFF)
    buffer.append(footer)
    buffer.append(footer)
    return buffer

def draw_threshold_ui(img, mode, value):
    # 无需声明global，直接使用全局变量
    # 顶部
    img.draw_rectangle(0, 0, lcd_width, 60, color=(200,200,200), thickness=2, fill=True)
    img.draw_string_advanced(10, 10, 30, f"模式:{mode.upper()}", color=(0,0,0), bg_color=(255,255,255))

    button_width, button_height = 60, 40  # 墛大按钮尺寸
    x_start = 20  # 墛大起始位置
    y_start = 80  # 墛大起始位置
    x_offset = button_width + 10  # 墛大偏移量
    y_offset = button_height + 50  # 墛大偏移量

    if mode == 'gray':
        # 灰度模式：两个阈值
        # 实时值显示
        img.draw_string_advanced(200, 10, 30, f"Gray Min: {value[0]} Max: {value[1]}", color=(0,0,0), bg_color=(255,255,255))

        # Min 值显示
        img.draw_string_advanced(x_start, y_start, 30, f"Min: {value[0]}", color=(0,0,0), bg_color=(255,255,255))

        # Min按钮布局 - 与触摸检测位置对应
        img.draw_rectangle(x_start + 100, y_start, button_width, button_height, color=(180,180,180), thickness=2, fill=True)
        img.draw_string_advanced(x_start + 120, y_start + 5, 30, "-", color=(0,0,0), bg_color=(255,255,255))

        img.draw_rectangle(x_start + 100 + x_offset, y_start, button_width, button_height, color=(180,180,180), thickness=2, fill=True)
        img.draw_string_advanced(x_start + 120 + x_offset, y_start + 5, 30, "+", color=(0,0,0), bg_color=(255,255,255))

        # Max 值显示
        x_start_max = 10
        img.draw_string_advanced(x_start_max, y_start+100, 30, f"Max: {value[1]}", color=(0,0,0), bg_color=(255,255,255))

        # Max按钮布局 - 与触摸检测位置对应
        img.draw_rectangle(x_start + 100, y_start + y_offset, button_width, button_height, color=(180,180,180), thickness=2, fill=True)
        img.draw_string_advanced(x_start + 120, y_start + y_offset + 5, 30, "-", color=(0,0,0), bg_color=(255,255,255))

        img.draw_rectangle(x_start + 100 + x_offset, y_start + y_offset, button_width, button_height, color=(180,180,180), thickness=2, fill=True)
        img.draw_string_advanced(x_start + 120 + x_offset, y_start + y_offset + 5, 30, "+", color=(0,0,0), bg_color=(255,255,255))

    else:
        # 红/绿模式：L, A, B 三个值
        labels = ['L_Min', 'A_Min', 'B_Min', 'L_Max', 'A_Max', 'B_Max']
        min_values = [value[0], value[2], value[4]]
        max_values = [value[1], value[3], value[5]]

        # 实时值显示
        img.draw_string_advanced(180, 10, 25, f"{mode.upper()} ", color=(0,0,0), bg_color=(255,255,255))
        img.draw_string_advanced(180, 30, 25, f"L:{min_values[0]},{max_values[0]} A:{min_values[1]},{max_values[1]} B:{min_values[2]},{max_values[2]}", color=(0,0,0), bg_color=(255,255,255))

        x_min_start = 20
        x_max_start = 400

        for i in range(3):
            # Min values
            img.draw_string_advanced(x_min_start, y_start + i * y_offset, 20, f"{labels[i]}: {min_values[i]}", color=(0,0,0), bg_color=(255,255,255))
            # -
            img.draw_rectangle(x_min_start + 80, y_start + i * y_offset, button_width, button_height, color=(180,180,180), thickness=2, fill=True)
            img.draw_string_advanced(x_min_start + 80 + 20, y_start + i * y_offset + 5, 30, "-", color=(0,0,0), bg_color=(255,255,255))
            # +
            img.draw_rectangle(x_min_start + 80 + x_offset, y_start + i * y_offset, button_width, button_height, color=(180,180,180), thickness=2, fill=True)
            img.draw_string_advanced(x_min_start + 80 + x_offset + 20, y_start + i * y_offset + 5, 30, "+", color=(0,0,0), bg_color=(255,255,255))

            # Max values
            img.draw_string_advanced(x_max_start, y_start + i * y_offset, 20, f"{labels[i+3]}: {max_values[i]}", color=(0,0,0), bg_color=(255,255,255))
            # -
            img.draw_rectangle(x_max_start + 80, y_start + i * y_offset, button_width, button_height, color=(180,180,180), thickness=2, fill=True)
            img.draw_string_advanced(x_max_start + 80 + 20, y_start + i * y_offset + 5, 30, "-", color=(0,0,0), bg_color=(255,255,255))
            # +
            img.draw_rectangle(x_max_start + 80 + x_offset, y_start + i * y_offset, button_width, button_height, color=(180,180,180), thickness=2, fill=True)
            img.draw_string_advanced(x_max_start + 80 + x_offset + 20, y_start + i * y_offset + 5, 30, "+", color=(0,0,0), bg_color=(255,255,255))

    # 返回按钮（使用lcd_height）
    img.draw_rectangle(0, lcd_height-90, 120, 80, color=(180,180,180), thickness=2, fill=True)
    img.draw_string_advanced(20, lcd_height-80, 40, "返回", color=(0,0,0), bg_color=(255,255,255))

    # 保存按钮（使用lcd_width和lcd_height）
    img.draw_rectangle(lcd_width-120, lcd_height-90, 120, 80, color=(100,255,100), thickness=2, fill=True)
    img.draw_string_advanced(lcd_width-100, lcd_height-80, 40, "保存", color=(0,0,0), bg_color=(255,255,255))

    # 切换按钮（使用lcd_width和lcd_height）
    img.draw_rectangle((lcd_width-120)//2, lcd_height-90, 120, 80, color=(180,180,180), thickness=2, fill=True)
    img.draw_string_advanced((lcd_width-100)//2, lcd_height-80, 40, "切换", color=(0,0,0), bg_color=(255,255,255))

def threshold_preview(img, mode, value):
    if mode == 'gray':
        img_gray = img.to_grayscale(copy=True)
        img_bin = img_gray.binary([value])
        return img_bin.to_rgb565()
    else:
        # 红/绿阈值预览：高亮find_blobs区域
        blobs = img.find_blobs([tuple(value)], pixels_threshold=10, area_threshold=10, merge=True)
        for b in blobs:
            img.draw_rectangle(b.x(), b.y(), b.w(), b.h(), color=(255,0,0) if mode=='red' else (0,255,0), thickness=3)

        # 黑白样式显示阈值调整变化
        img_copy = img.copy()
        img_bin = img_copy.binary([value])
        return img_bin.to_rgb565()

# 时间控制参数
last_laser_send = time.ticks_ms()
send_red_next = True

while True:
    clock.tick()
    try:
        img = sensor.snapshot()
    except Exception as e:
        print(f"获取图像出错: {e}")
        time.sleep_ms(50)  # 降低帧率
        continue
    points = tp.read()
    now = time.ticks_ms()

    # 长按触摸进入阈值调整模式
    if not in_threshold_mode:
        if len(points) > 0:
            if last_touch_time == 0:
                last_touch_time = now
            elif time.ticks_diff(now, last_touch_time) > 3000:
                in_threshold_mode = True
                last_touch_time = 0
        else:
            last_touch_time = 0

        # 默认红绿激光点识别（640x480）
        red_blobs = img.find_blobs([tuple(threshold_dict['red'])],  # 修改为元组
                                 pixels_threshold=1,
                                 area_threshold=1,
                                 merge=True,
                                 margin=5)
        green_blobs = img.find_blobs([tuple(threshold_dict['green'])],  # 修改为元组
                                      pixels_threshold=15,
                                      area_threshold=15,
                                      merge=True,
                                      margin=5)

        red_coords = None
        green_coords = None

        if red_blobs:
            max_red = max(red_blobs, key=lambda b: (b.area(), -b.y()))
            red_coords = (max_red.cx(), max_red.cy())
            img.draw_circle(red_coords[0], red_coords[1], 8, color=(255,0,0))
            img.draw_string_advanced(red_coords[0]+10, red_coords[1]+10, 20, "R:({},{})".format(*red_coords), color=(255,0,0), bg_color=(0,0,0))

        if green_blobs:
            max_green = max(green_blobs, key=lambda b: (b.area(), -b.y()))
            green_coords = (max_green.cx(), max_green.cy())
            img.draw_circle(green_coords[0], green_coords[1], 8, color=(0,255,0))
            img.draw_string_advanced(green_coords[0]+10, green_coords[1]+10, 20, "G:({},{})".format(*green_coords), color=(0,255,0), bg_color=(0,0,0))

        # 数据交替发送逻辑
        current_time = time.ticks_ms()
        if current_time - last_laser_send >= 200:
            if send_red_next:
                if red_blobs and red_coords:
                    uart.write(pack_data(red_coords, 0x02))
                    print("send red_coords",red_coords)
                    send_red_next = False
                    last_laser_send = current_time
                elif green_blobs and green_coords:
                    uart.write(pack_data(green_coords, 0x03))
                    print("send green_coords",green_coords)
                    send_red_next = True
                    last_laser_send = current_time
            else:
                if green_blobs and green_coords:
                    uart.write(pack_data(green_coords, 0x03))
                    print("send green_coords",green_coords)
                    send_red_next = True
                    last_laser_send = current_time
                elif red_blobs and red_coords:
                    uart.write(pack_data(red_coords, 0x02))
                    print("send red_coords",red_coords)
                    send_red_next = False
                    last_laser_send = current_time

        # 显示图像（直接全屏显示）
        Display.show_image(img, x=0, y=0)

    else:
        # 调整模式下
        img_full = img
        preview = threshold_preview(img_full.copy(), threshold_mode, threshold_dict[threshold_mode])
        draw_threshold_ui(preview, threshold_mode, threshold_dict[threshold_mode])
        Display.show_image(preview, x=0, y=0)
        points = tp.read()
        if len(points) > 0:
            x, y = points[0].x, points[0].y
            button_width, button_height = 80, 60  # 墛大按钮尺寸
            x_start = 20  # 墛大起始位置
            y_start = 80  # 墛大起始位置
            x_offset = button_width + 20  # 墛大偏移量
            y_offset = button_height + 20  # 墛大偏移量

            if threshold_mode == 'gray':
                # 灰度模式
                if x_start + 100 <= x < x_start + 100 + button_width and y_start <= y < y_start + button_height:
                    # Min -
                    threshold_dict['gray'][0] = max(0, threshold_dict['gray'][0]-1)
                elif x_start + 100 + x_offset <= x < x_start + 100 + x_offset + button_width and y_start <= y < y_start + button_height:
                    # Min +
                    threshold_dict['gray'][0] = min(255, threshold_dict['gray'][0]+1)
                elif x_start + 100 <= x < x_start + 100 + button_width and y_start + y_offset <= y < y_start + y_offset + button_height:
                    # Max -
                    threshold_dict['gray'][1] = max(0, threshold_dict['gray'][1]-1)
                elif x_start + 100 + x_offset <= x < x_start + 100 + x_offset + button_width and y_start + y_offset <= y < y_start + y_offset + button_height:
                    # Max +
                    threshold_dict['gray'][1] = min(255, threshold_dict['gray'][1]+1)

            else:
                # 红/绿模式
                x_min_start = 20
                x_max_start = 400
                for i in range(3):
                    # Min
                    if x_min_start + 80 <= x < x_min_start + 80 + button_width and y_start + i * y_offset <= y < y_start + i * y_offset + button_height:
                        # L/A/B Min -
                        threshold_dict[threshold_mode][i*2] = max(-128, threshold_dict[threshold_mode][i*2]-1)
                    elif x_min_start + 80 + x_offset <= x < x_min_start + 80 + x_offset + button_width and y_start + i * y_offset <= y < y_start + i * y_offset + button_height:
                        # L/A/B Min +
                        threshold_dict[threshold_mode][i*2] = min(127, threshold_dict[threshold_mode][i*2]+1)

                    # Max
                    elif x_max_start + 80 <= x < x_max_start + 80 + button_width and y_start + i * y_offset <= y < y_start + i * y_offset + button_height:
                        # L/A/B Max -
                        threshold_dict[threshold_mode][i*2+1] = max(-128, threshold_dict[threshold_mode][i*2+1]-1)
                    elif x_max_start + 80 + x_offset <= x < x_max_start + 80 + x_offset + button_width and y_start + i * y_offset <= y < y_start + i * y_offset + button_height:
                        # L/A/B Max +
                        threshold_dict[threshold_mode][i*2+1] = min(127, threshold_dict[threshold_mode][i*2+1]+1)

            # 返回
            if 0 <= x < 120 and current_display_height-90 <= y < current_display_height:
                in_threshold_mode = False
            # 保存
            elif current_display_width-120 <= x < current_display_width and current_display_height-90 <= y < current_display_height:
                old = load_thresholds()
                save_thresholds(threshold_dict)
                print(f"保存阈值: 新值={threshold_dict}, 旧值={old}")
                in_threshold_mode = False
            # 切换
            elif (current_display_width-120)//2 <= x < (current_display_width-120)//2+120 and current_display_height-90 <= y < current_display_height:
                idx = (threshold_modes.index(threshold_mode)+1)%3
                threshold_mode = threshold_modes[idx]
        time.sleep_ms(120)
        continue

    # 按键检测（消抖）
    if KEY.value() == 0:
        time.sleep_ms(10)
        if KEY.value() == 0:
            trigger_rect = 1
            print('按键触发 Trigger_rect 1')
            while KEY.value() == 0:
                pass

    # UART协议解析状态机
    def parse_uart():
        """K230专用协议解析函数"""
        global trigger_rect
        byte = uart.read(3)
        if byte:
            if byte[0] == 0xff and byte[-1] == 0xfe:
                if trigger_rect == 0:
                    trigger_rect = 1
                    print('set Trigger_rect 1')
                if trigger_rect == 2 and len(byte) >= 3 and 0 <= int(byte[1]) <= 3:
                     # 发送坐标时补充flag参数
                        uart.write(pack_data(rect_coords[int(byte[1])], 0x01))
                        print('send:', rect_coords[int(byte[1])])

    # 矩形检测逻辑（trigger_rect==1时进入）
    parse_uart()

    if trigger_rect == 1:
        # 灰度二值化+find_rects
        img_gray = img.to_grayscale(copy=True)
        img_bin = img_gray.binary([threshold_dict['gray']]) # 修改为列表
        rects = list(img_bin.find_rects(threshold=15000))
        rects_sorted = sorted(rects, key=lambda r: (r.rect()[2] * r.rect()[3]), reverse=True)

        if len(rects_sorted) >= 2:
            # 处理外边框
            outer = rects_sorted[0]
            outer_ordered = sort_corners_clockwise(outer.corners())
            # 处理内边框
            inner = rects_sorted[1]
            inner_ordered = sort_corners_clockwise(inner.corners())
            # 绘制外框连线和标签
            for i in range(4):
                start = outer_ordered[i]
                end = outer_ordered[(i+1)%4]
                img.draw_line(start[0], start[1], end[0], end[1], color=(255,0,0), thickness=2)
                img.draw_string_advanced(start[0]+5, start[1]+5, 20, f"x{i+1}", color=(255,0,0))
                img.draw_circle(start[0], start[1], 5, color=(255,0,0))
            # 绘制内框连线和标签
            for i in range(4):
                start = inner_ordered[i]
                end = inner_ordered[(i+1)%4]
                img.draw_line(start[0], start[1], end[0], end[1], color=(0,255,0), thickness=2)
                img.draw_string_advanced(start[0]+5, start[1]+5, 20, f"x{i+5}", color=(0,255,0))
                img.draw_circle(start[0], start[1], 5, color=(0,255,0))
            # 计算平均坐标
            all_points = outer_ordered + inner_ordered
            averaged_points = []
            for i in range(4):
                outer_p = all_points[i]
                inner_p = all_points[i+4]
                avg_x = (outer_p[0] + inner_p[0]) // 2
                avg_y = (outer_p[1] + inner_p[1]) // 2
                averaged_points.append((avg_x, avg_y))
            rect_coords = averaged_points
            # 打印坐标
            print("\n==== Coordinates ====")
            for idx, p in enumerate(all_points[:4], 1):
                print(f"Outer x{idx}: ({p[0]:.1f}, {p[1]:.1f})")
            for idx, p in enumerate(all_points[4:], 5):
                print(f"Inner x{idx}: ({p[0]:.1f}, {p[1]:.1f})")
            print("\n==== Averaged Points ====")
            for idx, (x, y) in enumerate(averaged_points, 1):
                print(f"Avg x{idx}: ({x:.1f}, {y:.1f})")
            trigger_rect = 2
            print('set Trigger_rect 2')

    # 激光检测逻辑（来自K230）
    if trigger_rect == 0 or trigger_rect == 2:
        # 红色激光检测 - 修改参数使识别更严格
        red_blobs = img.find_blobs([tuple(threshold_dict['red'])],
                                 pixels_threshold=3,     # 增加像素阈值
                                 area_threshold=3,       # 增加面积阈值
                                 merge=False,            # 禁用区域合并
                                 margin=2)              # 减小边缘像素

        if red_blobs:
            # 选择最大面积且最亮的点
            max_red = max(red_blobs, key=lambda b: (b.pixels(), b.density(), -b.y()))
            red_coords = (max_red.cx(), max_red.cy())
            # 增加密度检查，确保是激光点
            if max_red.density() > 0.6:  # 密度阈值，激光点通常较为密集
                img.draw_circle(red_coords[0], red_coords[1], 8, color=(255,0,0))
                img.draw_string_advanced(red_coords[0]+10, red_coords[1]+10, 20,
                                      "R:({},{})".format(*red_coords),
                                      color=(255,0,0), bg_color=(0,0,0))
                # 打印调试信息
                print(f"Red point - pixels: {max_red.pixels()}, density: {max_red.density():.2f}")

        # 绿色激光检测（保持原有参数）
        green_blobs = img.find_blobs([tuple(threshold_dict['green'])],
                                      pixels_threshold=15,
                                      area_threshold=15,
                                      merge=True,
                                      margin=5)
        if green_blobs:
            max_green = max(green_blobs, key=lambda b: (b.area(), -b.y()))
            green_coords = (max_green.cx(), max_green.cy())
            img.draw_circle(green_coords[0], green_coords[1], 8, color=(0,255,0))
            img.draw_string_advanced(green_coords[0]+10, green_coords[1]+10, 20, "G:({},{})".format(*green_coords), color=(0,255,0), bg_color=(0,0,0))

        # 数据交替发送逻辑
        current_time = time.ticks_ms()
        if current_time - last_laser_send >= 200:
            if send_red_next:
                if red_blobs and red_coords:
                    uart.write(pack_data(red_coords, 0x02))
                    print("send red_coords",red_coords)
                    send_red_next = False
                    last_laser_send = current_time
                elif green_blobs and green_coords:
                    uart.write(pack_data(green_coords, 0x03))
                    print("send green_coords",green_coords)
                    send_red_next = True
                    last_laser_send = current_time
            else:
                if green_blobs and green_coords:
                    uart.write(pack_data(green_coords, 0x03))
                    print("send green_coords",green_coords)
                    send_red_next = True
                    last_laser_send = current_time
                elif red_blobs and red_coords:
                    uart.write(pack_data(red_coords, 0x02))
                    print("send red_coords",red_coords)
                    send_red_next = False
                    last_laser_send = current_time

    Display.show_image(img, x=0, y=0) # 修改为全屏显示


