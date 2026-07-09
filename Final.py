import cv2
import numpy as np
import pyrealsense2 as rs
import serial
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import time
from flask import Flask, Response
import threading

# ================= CONFIG =================
ENGINE_PATH = "yolov8n.engine"
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

CONF_THRES = 0.35
NMS_THRES = 0.45

DANGER_DISTANCE = 7.0
NEAR_STOP_DISTANCE = 4.0
DANGER_FRAMES = 2
SAFE_FRAMES = 5
DEPTH_SCAN_INTERVAL = 8

TARGET_CLASSES = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    5: "bus",
    7: "truck"
}

# ================= WEB STREAM =================
app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()

@app.route("/")
def index():
    return """
    <html>
    <head>
        <title>Jetson ROI</title>
    </head>
    <body style="margin:0;background:#111;text-align:center;">
        <h2 style="color:white;">Jetson Nano YOLOv8 Safety</h2>
        <img src="/video" style="width:95%;max-width:900px;border-radius:12px;">
    </body>
    </html>
    """

@app.route("/video")
def video():
    def generate():
        global latest_frame

        while True:
            with frame_lock:
                if latest_frame is None:
                    continue

                ok, jpeg = cv2.imencode(".jpg", latest_frame)

            if not ok:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                jpeg.tobytes() +
                b"\r\n"
            )

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

def start_video_server():
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)

# ================= SERIAL =================
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print("[INFO] ESP32 connected")
except Exception as e:
    print("[WARNING] Serial failed:", e)
    print("[INFO] Vision-only mode")

# ================= TENSORRT =================
TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

with open(ENGINE_PATH, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
    engine = runtime.deserialize_cuda_engine(f.read())

context = engine.create_execution_context()

inputs, outputs, bindings = [], [], []
stream = cuda.Stream()

for binding in engine:
    shape = engine.get_binding_shape(binding)
    size = trt.volume(shape) * engine.max_batch_size
    dtype = trt.nptype(engine.get_binding_dtype(binding))

    host_mem = cuda.pagelocked_empty(size, dtype)
    device_mem = cuda.mem_alloc(host_mem.nbytes)

    bindings.append(int(device_mem))

    item = {
        "host": host_mem,
        "device": device_mem,
        "shape": shape
    }

    if engine.binding_is_input(binding):
        inputs.append(item)
    else:
        outputs.append(item)

print("[INFO] TensorRT YOLOv8 engine ready")

# ================= REALSENSE =================
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, FRAME_WIDTH, FRAME_HEIGHT, rs.format.z16, 30)

profile = pipeline.start(config)
align = rs.align(rs.stream.color)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("[INFO] Depth scale:", depth_scale)

# ================= ROI HÌNH THANG =================
trap_top_width = 80
trap_bottom_width = 500
trap_height = 340
trap_y_offset = 90

center_x = FRAME_WIDTH // 2

pt1 = [center_x - trap_top_width // 2, trap_y_offset]
pt2 = [center_x + trap_top_width // 2, trap_y_offset]
pt3 = [center_x + trap_bottom_width // 2, trap_y_offset + trap_height]
pt4 = [center_x - trap_bottom_width // 2, trap_y_offset + trap_height]

roi_points = np.array([pt1, pt2, pt3, pt4], np.int32).reshape((-1, 1, 2))

# ================= FUNCTIONS =================
def preprocess(img, input_shape):
    input_h = input_shape[2]
    input_w = input_shape[3]

    h, w = img.shape[:2]
    r = min(input_w / w, input_h / h)

    new_w = int(w * r)
    new_h = int(h * r)

    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    pad_top = (input_h - new_h) // 2
    pad_bottom = input_h - new_h - pad_top
    pad_left = (input_w - new_w) // 2
    pad_right = input_w - new_w - pad_left

    padded = cv2.copyMakeBorder(
        resized,
        pad_top,
        pad_bottom,
        pad_left,
        pad_right,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114)
    )

    rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
    img_float = rgb.astype(np.float32) / 255.0
    chw = np.transpose(img_float, (2, 0, 1))

    return np.ascontiguousarray(chw), r, pad_left, pad_top

def nms(boxes, scores, iou_thres):
    if len(boxes) == 0:
        return []

    boxes = np.array(boxes)
    scores = np.array(scores)

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 0] + boxes[:, 2]
    y2 = boxes[:, 1] + boxes[:, 3]

    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]

    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)

        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(iou <= iou_thres)[0]
        order = order[inds + 1]

    return keep

def clamp_point(x, y):
    x = int(max(0, min(x, FRAME_WIDTH - 1)))
    y = int(max(0, min(y, FRAME_HEIGHT - 1)))
    return x, y

def get_distance(depth_frame, x, y):
    x, y = clamp_point(x, y)
    values = []

    for dy in range(-2, 3):
        for dx in range(-2, 3):
            px, py = clamp_point(x + dx, y + dy)
            d = depth_frame.get_distance(px, py)
            if d > 0:
                values.append(d)

    if len(values) == 0:
        return 0

    return float(np.median(values))
def get_object_distance(depth_frame, x1, y1, x2, y2):
    points = [
        ((x1 + x2) // 2, (y1 + y2) // 2), 
        ((x1 + x2) // 2, y2),              
        (x1, y2),
        (x2, y2)
    ]

    values = []

    for px, py in points:
        d = get_distance(depth_frame, px, py)
        if d > 0:
            values.append(d)

    if len(values) == 0:
        return 0

    return min(values)
def detect_large_obstacle(depth_frame, img):
    depth_raw = np.asanyarray(depth_frame.get_data())

    min_m = 0.4
    max_m = DANGER_DISTANCE

    min_raw = int(min_m / depth_scale)
    max_raw = int(max_m / depth_scale)

    mask = cv2.inRange(depth_raw, min_raw, max_raw)

    roi_mask = np.zeros_like(mask)
    cv2.fillPoly(roi_mask, [roi_points], 255)
    mask = cv2.bitwise_and(mask, roi_mask)

    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contour_data = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contour_data[0] if len(contour_data) == 2 else contour_data[1]

    for c in contours:
        area = cv2.contourArea(c)

        if area < 12000:
            continue

        x, y, w, h = cv2.boundingRect(c)

        if w < 60 or h < 60:
            continue

        cx = x + w // 2
        cy = y + h // 2
        distance = get_distance(depth_frame, cx, cy)

        if distance > 0 and distance < DANGER_DISTANCE:
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.putText(
                img,
                "LARGE OBSTACLE %.2fm" % distance,
                (x, max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 255),
                2
            )
            return True

    return False

def send_state(state):
    if ser is None:
        return

    if state == "STOP":
        ser.write(b"S\n")
    else:
        ser.write(b"R\n")

# ================= MAIN =================
stable_state = "RUN"
danger_count = 0
safe_count = 0
frame_id = 0

print("[INFO] System running with web stream...")
print("[INFO] Open browser: http://JETSON_IP:5000")

try:
    threading.Thread(target=start_video_server, daemon=True).start()

    last_send_time = 0

    while True:
        start_time = time.time()
        frame_id += 1

        raw_frames = pipeline.wait_for_frames()
        frames = align.process(raw_frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        img = np.asanyarray(color_frame.get_data())

        cv2.polylines(img, [roi_points], True, (255, 0, 0), 2)

        input_shape = inputs[0]["shape"]
        input_data, ratio, pad_x, pad_y = preprocess(img, input_shape)

        np.copyto(inputs[0]["host"], input_data.ravel())

        cuda.memcpy_htod_async(inputs[0]["device"], inputs[0]["host"], stream)

        context.execute_async_v2(
            bindings=bindings,
            stream_handle=stream.handle
        )

        cuda.memcpy_dtoh_async(outputs[0]["host"], outputs[0]["device"], stream)
        stream.synchronize()

        output = outputs[0]["host"].reshape(outputs[0]["shape"])

        preds = np.squeeze(output).T

        boxes = []
        scores = []
        labels = []

        for pred in preds:
            x, y, w, h = pred[:4]
            class_scores = pred[4:]

            cls_id = int(np.argmax(class_scores))
            conf = float(class_scores[cls_id])

            if conf < CONF_THRES:
                continue

            if cls_id not in TARGET_CLASSES:
                continue

            left = int((x - w / 2 - pad_x) / ratio)
            top = int((y - h / 2 - pad_y) / ratio)
            width = int(w / ratio)
            height = int(h / ratio)

            boxes.append([left, top, width, height])
            scores.append(conf)
            labels.append(cls_id)

        indices = nms(boxes, scores, NMS_THRES)

        danger = False

        for i in indices:
            x1, y1, w, h = boxes[i]

            x2 = x1 + w
            y2 = y1 + h

            x1 = max(0, min(x1, FRAME_WIDTH - 1))
            y1 = max(0, min(y1, FRAME_HEIGHT - 1))
            x2 = max(0, min(x2, FRAME_WIDTH - 1))
            y2 = max(0, min(y2, FRAME_HEIGHT - 1))

            cls_id = labels[i]
            name = TARGET_CLASSES[cls_id]

            cx = (x1 + x2) // 2
            cy = y2
            cx, cy = clamp_point(cx, cy)

            # Khoảng cách từ điểm kiểm tra đến ROI:
            # roi_dist >= 0  : điểm nằm trong ROI
            # roi_dist < 0   : điểm nằm ngoài ROI, giá trị càng âm là càng xa mép ROI
            roi_dist = cv2.pointPolygonTest(roi_points, (cx, cy), True)
            in_roi = roi_dist >= 0
            near_roi = roi_dist >= -35   # cho phép lệch mép ROI khoảng 35 pixel để dự phòng quán tính

            distance = get_object_distance(
                 depth_frame,
                 x1, y1,
                 x2, y2
            )

            color = (0, 255, 0)

            # Điều kiện STOP chính:
            # Vật nằm trong ROI và khoảng cách <= DANGER_DISTANCE thì STOP.
            if in_roi and distance > 0 and distance <= DANGER_DISTANCE:
                danger = True
                color = (0, 0, 255)

            # Điều kiện STOP dự phòng:
            # Vật rất gần <= NEAR_STOP_DISTANCE nhưng phải nằm trong ROI hoặc sát mép ROI.
            # Tránh trường hợp xe/vật bên hông dưới 4m làm STOP sai.
            if near_roi and distance > 0 and distance <= NEAR_STOP_DISTANCE:
                danger = True
                color = (0, 0, 255)
            
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.circle(img, (cx, cy), 5, color, -1)

            cv2.putText(
                img,
                "%s %.2f %.2fm" % (name, scores[i], distance),
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2
            )

        if frame_id % DEPTH_SCAN_INTERVAL == 0:
            if detect_large_obstacle(depth_frame, img):
                danger = True

        if danger:
            danger_count += 1
            safe_count = 0
        else:
            safe_count += 1
            danger_count = 0

        if danger_count >= DANGER_FRAMES:
            stable_state = "STOP"

        if safe_count >= SAFE_FRAMES:
            stable_state = "RUN"

        if time.time() - last_send_time > 0.1:
            send_state(stable_state)
            last_send_time = time.time()

        color_state = (0, 0, 255) if stable_state == "STOP" else (0, 255, 0)

        cv2.putText(
            img,
            stable_state,
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            color_state,
            3
        )

        fps = 1.0 / max(time.time() - start_time, 0.001)

        cv2.putText(
            img,
            "FPS: %.1f" % fps,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2
        )

        with frame_lock:
            latest_frame = img.copy()

except KeyboardInterrupt:
    print("[INFO] Stopped")

finally:
    print("[INFO] Cleaning up")
    pipeline.stop()

    if ser:
        ser.close()
