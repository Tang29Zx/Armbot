#!/usr/bin/env python3
"""Armbot Web 控制页: /map /odom 可视化 + 键盘 /cmd_vel 控制"""
import os, threading, json, math, time, subprocess
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, TransformStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

# map_server 的 /map 是 TRANSIENT_LOCAL 发布，必须用 TRANSIENT_LOCAL 订阅（VOLATILE 收不到）
MAP_QOS = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

LOCK_MAP, LOCK_ODOM, LOCK_CMD = threading.Lock(), threading.Lock(), threading.Lock()
MAP, MAP_STATIC, ODOM, CMD = None, None, {"x": 0, "y": 0, "theta": 0}, {"x": 0.0, "y": 0.0, "z": 0.0, "t": 0.0}
NODE = None
MAPPING_LOCK = threading.Lock()
MAPPING_LAST = [0.0]  # 上次触发开始建图的时间（防重复）
NAV_LOCK = threading.Lock()
NAV_LAST = [0.0]      # 上次触发启动导航的时间（防重复）
# map->odom 变换（tx, ty, theta）——把 odom 坐标变换到 map 系
TFOFF = [0.0, 0.0, 0.0]
# 纯里程计模式：用户设定初始位姿 + odom 参考点（无雷达/无 slam 时用）
POSE_REF = None


class RosNode(Node):
    def __init__(self):
        super().__init__("armbot_web")
        # /map: map_server 发布（TRANSIENT_LOCAL），须 TRANSIENT_LOCAL 订阅
        self.create_subscription(OccupancyGrid, "/map", self.cb_map, MAP_QOS)
        # 8-29 15:18: 导航模式下 /map 会被 slam 定位节点的局部图(尺寸可变)抢占/覆盖静态图
        # → 额外订阅 map_relay 转发的 /map_static（map_server 完整静态图），前端优先显示它
        self.create_subscription(OccupancyGrid, "/map_static", self.cb_map_static, MAP_QOS)
        self.create_subscription(Odometry, "/odom", self.cb_odom, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # 取消目标直接调 action cancel service（空 goal_id = 取消所有），不依赖内部句柄
        # 8-22: 急停必须连 recovery 的 spin/backup 一起取消（否则车继续转圈）
        self.cancel_cli = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel")
        self.cancel_spin = self.create_client(CancelGoal, "/spin/_action/cancel")
        self.cancel_backup = self.create_client(CancelGoal, "/backup/_action/cancel")
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_timer(0.1, self.timer_cb)
        # tf: 维护 map->odom（slam 发布的动态变换）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.tf_cb)

    def cb_map(self, msg):
        global MAP
        w, h = msg.info.width, msg.info.height
        # 只接受更大的图：map_server 完整图（272x285）最大，slam 局部图（尺寸会变）永远更小、不覆盖
        with LOCK_MAP:
            if MAP is not None and w * h < MAP["w"] * MAP["h"]:
                return
        step = max(1, math.ceil(w / 240))
        data = [[msg.data[j * w + i] for i in range(0, w, step)] for j in range(0, h, step)]
        # 8-29: 同时存全分辨率数据（保存地图用，显示用降采样 data）
        data_full = [[msg.data[j * w + i] for i in range(w)] for j in range(h)]
        with LOCK_MAP:
            MAP = {"w": w, "h": h, "res": msg.info.resolution,
                   "ox": msg.info.origin.position.x, "oy": msg.info.origin.position.y,
                   "step": step, "data": data, "data_full": data_full}

    def cb_map_static(self, msg):
        """/map_static：map_relay 转发的 map_server 完整静态图（导航模式优先显示）"""
        global MAP_STATIC
        w, h = msg.info.width, msg.info.height
        step = max(1, math.ceil(w / 240))
        data = [[msg.data[j * w + i] for i in range(0, w, step)] for j in range(0, h, step)]
        data_full = [[msg.data[j * w + i] for i in range(w)] for j in range(h)]
        with LOCK_MAP:
            MAP_STATIC = {"w": w, "h": h, "res": msg.info.resolution,
                          "ox": msg.info.origin.position.x, "oy": msg.info.origin.position.y,
                          "step": step, "data": data, "data_full": data_full}

    def cb_odom(self, msg):
        global ODOM, POSE_REF
        q = msg.pose.pose.orientation
        th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # odom 系坐标
        xo, yo = msg.pose.pose.position.x, msg.pose.pose.position.y
        with LOCK_ODOM:
            if POSE_REF is not None:
                # 纯里程计模式：map 位置 = 初始位姿 + odom 增量（无雷达/slam 依赖）
                # 不加 0.175 激光偏移——用户设哪显示哪（偏移仅 slam 地图用）
                dx = xo - POSE_REF["ox"]
                dy = yo - POSE_REF["oy"]
                # mth = map->odom 旋转角 = 设位姿时车头 map 朝向 - odom 朝向
                c0, s0 = math.cos(POSE_REF["mth"]), math.sin(POSE_REF["mth"])
                xm = POSE_REF["mx"] + c0 * dx - s0 * dy
                ym = POSE_REF["my"] + s0 * dx + c0 * dy
                thm = POSE_REF["mth"] + th   # map 朝向 = map->odom 旋转 + 当前 odom 朝向
            else:
                # 兼容模式：tf map->odom（slam/静态 tf）
                tx, ty, tth = TFOFF
                cos_t, sin_t = math.cos(tth), math.sin(tth)
                xm = cos_t * xo - sin_t * yo + tx
                ym = sin_t * xo + cos_t * yo + ty
                thm = th + tth
                # 显示激光（车头）位置：slam 地图以激光为中心建图，
                # 车图形中心放车头（base_link 前方 0.175m），避免车图形落在图外/白色区
                xm += 0.175 * math.cos(thm)
                ym += 0.175 * math.sin(thm)
            # 归一化角度
            thm = math.atan2(math.sin(thm), math.cos(thm))
            ODOM = {"x": xm, "y": ym, "theta": thm,
                    "ox": xo, "oy": yo, "oth": th}  # 保留原始 odom 值

    def tf_cb(self):
        global TFOFF
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time())
            q = t.transform.rotation
            th = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            with LOCK_ODOM:
                TFOFF = [t.transform.translation.x, t.transform.translation.y, th]
        except Exception:
            pass  # tf 未就绪时保持上次值

    def timer_cb(self):
        with LOCK_CMD:
            if time.time() - CMD["t"] > 0.8:
                CMD["x"] = CMD["y"] = CMD["z"] = 0.0
            x, y, z = CMD["x"], CMD["y"], CMD["z"]
        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = x, y, z
        self.pub.publish(tw)

    def send_goal(self, x, y, th):
        """发 Nav2 导航目标（map 系坐标，th 弧度）"""
        if not self.nav_client.wait_for_server(timeout_sec=8.0):  # 8-22 16:21: DDS SHM 竞争导致发现慢，2s 常超时
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(th / 2)
        goal.pose.pose.orientation.w = math.cos(th / 2)
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: setattr(self, "_gh", f.result()))
        return True

    def cancel_goal(self):
        """取消当前导航目标 + recovery（spin/backup）——急停三连取消"""
        gh = getattr(self, "_gh", None)
        if gh is not None:
            try:
                self.nav_client.cancel_goal(gh)  # Humble: 同步取消
            except Exception as e:
                print("cancel_goal(gh) error:", e)
        for cli in (self.cancel_cli, self.cancel_spin, self.cancel_backup):
            try:
                if not cli.service_is_ready():
                    cli.wait_for_service(timeout_sec=0.5)
                req = CancelGoal.Request()
                req.goal_info = GoalInfo()  # 空 goal_id = 取消该 action server 所有目标
                cli.call_async(req)
            except Exception as e:
                print("cancel_goal(srv) error:", e)
                return False
        return True

    def set_pose(self, x, y, th):
        """设初始位姿（map 系）：记录 odom 参考点，纯里程计模式由此推算位置"""
        global POSE_REF
        # 记录当前 odom 原值作为参考
        with LOCK_ODOM:
            ref_ox = ODOM.get("ox", 0.0)
            ref_oy = ODOM.get("oy", 0.0)
            ref_oth = ODOM.get("oth", 0.0)
        POSE_REF = {"mx": x, "my": y,
                    # mth = map->odom 旋转角（车头 map 朝向 - odom 朝向），
                    # 不是车头朝向本身！否则显示方向会横着飘
                    "mth": th - ref_oth,
                    "ox": ref_ox, "oy": ref_oy, "oth": ref_oth}
        # 同时发 /initialpose（兼容 slam 存在时的定位）
        p = PoseWithCovarianceStamped()
        p.header.frame_id = "map"
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.pose.position.x = x
        p.pose.pose.position.y = y
        p.pose.pose.orientation.z = math.sin(th / 2)
        p.pose.pose.orientation.w = math.cos(th / 2)
        self.pose_pub.publish(p)
        return True


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        global MAP, MAP_STATIC
        if self.path == "/api/map" or self.path.startswith("/api/map?"):
            with LOCK_MAP:
                # 导航模式优先显示 map_server 静态完整图；建图模式无 MAP_STATIC 用 slam 图
                cur = MAP_STATIC if MAP_STATIC else MAP
                body = json.dumps(cur if cur else {})
            self._send(body, "application/json")
        elif self.path.startswith("/api/odom"):
            with LOCK_ODOM:
                body = json.dumps(ODOM)
            self._send(body, "application/json")
        elif self.path.startswith("/api/goal"):
            from urllib.parse import urlparse, parse_qs
            q = parse_qs(urlparse(self.path).query)
            g = lambda k, d: float(q.get(k, [d])[0])
            x, y, th = g("x", 0), g("y", 0), g("th", 0)
            ok = bool(NODE and NODE.send_goal(x, y, th))
            self._send('{"ok":%s}' % ("true" if ok else "false"), "application/json")
        elif self.path.startswith("/api/pose"):
            from urllib.parse import urlparse, parse_qs
            q = parse_qs(urlparse(self.path).query)
            g = lambda k, d: float(q.get(k, [d])[0])
            x, y, th = g("x", 0), g("y", 0), g("th", 0)
            ok = bool(NODE and NODE.set_pose(x, y, th))
            self._send('{"ok":%s}' % ("true" if ok else "false"), "application/json")
        elif self.path.startswith("/api/cancel_goal"):
            ok = bool(NODE and NODE.cancel_goal())
            self._send('{"ok":%s}' % ("true" if ok else "false"), "application/json")
        elif self.path.startswith("/api/nav_maps"):
            # 列出 ~/ 下可用的地图（*.yaml → 名字列表，去掉扩展名）
            import glob as _glob
            maps = []
            for f in sorted(_glob.glob("/home/sunrise/*.yaml")):
                name = f.rsplit("/", 1)[-1][:-5]
                if name:
                    maps.append(name)
            self._send(json.dumps({"ok": True, "maps": maps}), "application/json")
        elif self.path.startswith("/api/start_nav"):
            # 一键启动导航：选地图 → 启动底盘+nav.launch+map_server+slam（Web 保持运行）
            from urllib.parse import urlparse, parse_qs
            q = parse_qs(urlparse(self.path).query)
            map_name = (q.get("map", ["map_verify"])[0]).strip()
            map_name = map_name[:-5] if map_name.endswith(".yaml") else map_name
            now = time.time()
            with NAV_LOCK:
                if now - NAV_LAST[0] < 90:
                    self._send('{"ok":true,"msg":"导航正在启动中（90秒防重）..."}', "application/json")
                    return
                NAV_LAST[0] = now
                try:
                    os.makedirs("/tmp/nav_run", exist_ok=True)
                    logf = open("/tmp/nav_run/start_nav_web.log", "a")
                    subprocess.Popen(["bash", "/home/sunrise/start_nav_web.sh", map_name],
                                     stdout=logf, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL)
                    self._send('{"ok":true,"msg":"导航启动中（地图: %s，约需 90 秒）..."}' % map_name, "application/json")
                except Exception as e:
                    self._send('{"ok":false,"msg":"%s"}' % e, "application/json")
        elif self.path.startswith("/api/start_mapping"):
            # 一键启动建图进程（雷达+底盘+tf+slam），Web 自身保持运行
            # 30 秒内重复点击忽略——防止并发执行多套进程互相踩踏
            now = time.time()
            with MAPPING_LOCK:
                if now - MAPPING_LAST[0] < 30:
                    self._send('{"ok":true,"msg":"建图已在启动中（30秒防重）..."}', "application/json")
                    return
                MAPPING_LAST[0] = now
                try:
                    # 8-29: 清空 Web 地图缓存，等待新 slam 出图（否则旧图不释放）
                    with LOCK_MAP:
                        MAP = None
                        MAP_STATIC = None
                    # RDK 重启后 /tmp/explore_run 不存在，先建目录再写日志
                    os.makedirs("/tmp/explore_run", exist_ok=True)
                    logf = open("/tmp/explore_run/start_mapping_web.log", "a")
                    subprocess.Popen(["bash", "/home/sunrise/start_mapping_only.sh"],
                                     stdout=logf, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL)
                    self._send('{"ok":true,"msg":"建图启动中..."}', "application/json")
                except Exception as e:
                    self._send('{"ok":false,"msg":"%s"}' % e, "application/json")
        elif self.path.startswith("/api/map_save"):
            # 8-29: 保存当前 /map 为 PGM+YAML（map_server 可直接加载）
            import json as _json
            with LOCK_MAP:
                m = dict(MAP) if MAP else None
            if not m or not m.get("data_full"):
                self._send('{"ok":false,"msg":"地图为空，先建图再保存"}', "application/json")
                return
            name = "/home/sunrise/map_save"
            w, h, res = m["w"], m["h"], m["res"]
            ox, oy = m["ox"], m["oy"]
            raw = bytearray()
            # 8-29: PGM 行序反转——map_server 加载 PGM 会垂直翻转（PGM顶部→地图底部），
            # 故世界底部(j=0)必须写在 PGM 底部（最后一行），否则导航加载后地图上下颠倒
            for j in range(h - 1, -1, -1):    # 世界顶部先写（PGM 顶部），世界底部最后
                row = m["data_full"][j]
                for i in range(w):
                    v = row[i]
                    if v >= 100:
                        raw.append(0)      # 障碍 → 黑
                    elif v == -1:
                        raw.append(205)    # 未知 → 灰
                    else:
                        raw.append(255)    # 空闲 → 白
            try:
                with open(name + ".pgm", "wb") as f:
                    f.write(b"P5\n%d %d\n255\n" % (w, h))
                    f.write(bytes(raw))
                with open(name + ".yaml", "w") as f:
                    # 8-29: 阈值改 0.9/0.1——PGM 未知写 205(occ=0.196)，默认阈值 0.65/0.25 会把 205 判成空闲
                    # 0.1 < 0.196 < 0.9 → 未知区域在导航加载后保留为 -1（map_server 标准）
                    f.write("image: map_save.pgm\nresolution: %.6f\norigin: [%.4f, %.4f, 0.0]\nnegate: 0\noccupied_thresh: 0.9\nfree_thresh: 0.1\n" % (res, ox, oy))
                self._send('{"ok":true,"msg":"已保存 %s (%dx%d)"}' % (name, w, h), "application/json")
            except Exception as e:
                self._send('{"ok":false,"msg":"保存失败: %s"}' % e, "application/json")
        elif self.path.startswith("/api/emergency_stop"):
            # 8-29 15:05 升级急停：只清 I2C 不够——底盘节点活着会按 Nav2 cmd_vel 50ms 内写回速度，
            # 急停必须从源头切断：杀底盘节点 + 杀 nav2/定位进程（Web 自身保留），再 I2C 清零
            estop_cmd = (
                "ps aux | grep -E 'chassis_control_node|nav2_|localization_slam|map_server|"
                "map_relay|ydlidar_node|scan_filter|static_transform' "
                "| grep -v grep | grep -v armbot_web | awk '{print $2}' | xargs -r kill -9 2>/dev/null; "
                "sleep 1; python3 /home/sunrise/stop.py"
            )
            try:
                subprocess.Popen(["bash", "-c", estop_cmd],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
            with LOCK_CMD:
                CMD["x"] = CMD["y"] = CMD["z"] = 0.0
            try:
                if NODE:
                    NODE.cancel_goal()
            except Exception:
                pass
            self._send('{"ok":true,"msg":"急停!（已切断底盘与导航）"}', "application/json")
        elif self.path.startswith("/api/cmd"):
            from urllib.parse import urlparse, parse_qs
            q = parse_qs(urlparse(self.path).query)
            g = lambda k, d: float(q.get(k, [d])[0])
            with LOCK_CMD:
                CMD["x"] = max(-0.2, min(0.2, g("x", 0)))
                CMD["y"] = max(-0.15, min(0.15, g("y", 0)))
                CMD["z"] = max(-0.5, min(0.5, g("z", 0)))
                CMD["t"] = time.time()
            self._send('{"ok":true}', "application/json")
        else:
            self._send(PAGE, "text/html; charset=utf-8")

    def _send(self, body, ctype):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        # 禁止缓存：避免浏览器拿到旧页面看不到新按钮/新功能
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.end_headers()
        self.wfile.write(body)


PAGE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Armbot 遥控 v8-22-1505</title>
<style>
html,body{height:100%}
body{background:#111;color:#eee;font-family:monospace;margin:0;display:flex;flex-direction:column;height:100vh}
/* 顶栏：关键操作永远可见（不依赖侧栏布局） */
#topbar{display:flex;align-items:center;gap:10px;padding:8px 12px;background:#222;border-bottom:1px solid #333;flex-wrap:wrap}
#topbar button{padding:6px 14px;color:#fff;border:none;border-radius:4px;cursor:pointer;font-family:monospace;font-size:13px}
#main{flex:1;display:flex;min-height:0}
#map{flex:1;image-rendering:pixelated;background:#222}
#panel{width:240px;padding:12px 16px;background:#1a1a1a;border-left:1px solid #333;overflow-y:auto}
/* 窄屏：面板移到地图下方 */
@media (max-width: 720px){
  #main{flex-direction:column}
  #panel{width:auto;border-left:none;border-top:1px solid #333;max-height:40vh}
}
.ver{font-size:10px;color:#555;margin-top:4px}
h1{font-size:16px;margin:0 0 12px}
h2{font-size:13px;color:#888;margin:14px 0 6px}
kbd{background:#333;padding:2px 8px;border-radius:4px;border:1px solid #555;font-size:13px}
#status{font-size:12px;color:#0f0}
table{font-size:12px;border-collapse:collapse}
td{padding:2px 8px;color:#aaa}
.tip{font-size:11px;color:#666;margin-top:8px}
</style>
</head>
<body>
<div id="topbar">
  <b style="font-size:14px">Armbot <span class="ver">v8-29-1603</span></b>
  <select id="mapSel" style="background:#222;color:#eee;border:1px solid #555;border-radius:3px;padding:3px 6px;font-size:12px"></select>
  <button id="btnNav" onclick="startNav(); this.blur();" style="background:#38c">启动导航</button>
  <button id="btnMap" onclick="startMapping(); this.blur();" style="background:#38c">重新建图</button>
  <button id="btnSave" onclick="saveMap(); this.blur();" style="background:#3a3">保存地图</button>
  <button id="btnEStop" onclick="emergencyStop(); this.blur();" style="background:#d22;font-weight:bold;padding:6px 20px">急停</button>
  <button id="btnStop" onclick="stopAll(); this.blur();" style="background:#c33">停止导航</button>
  <label style="font-size:12px;color:#aaa;display:flex;align-items:center;gap:4px"><input type="checkbox" id="chkRobot" checked onchange="show_robot=this.checked"> 显示小车</label>
  <label style="font-size:12px;color:#aaa;display:flex;align-items:center;gap:4px">初始朝向<input id="poseTh" type="number" value="0" step="15" style="width:52px;background:#222;color:#eee;border:1px solid #555;border-radius:3px;padding:2px 4px">°</label>
  <span id="status" style="margin-left:auto">连接中...</span>
</div>
<div id="main">
<canvas id="map"></canvas>
<div id="panel">
<h1>Armbot 导航控制 <span class="ver">v8-22-1505</span></h1>
<h2>键盘控制</h2>
<table>
<tr><td><kbd>W</kbd>/<kbd>&uarr;</kbd></td><td>前进</td><td><kbd>S</kbd>/<kbd>&darr;</kbd></td><td>后退</td></tr>
<tr><td><kbd>A</kbd></td><td>左转</td><td><kbd>D</kbd></td><td>右转</td></tr>
<tr><td><kbd>Q</kbd></td><td>左横移</td><td><kbd>E</kbd></td><td>右横移</td></tr>
<tr><td><kbd>空格</kbd></td><td colspan="3">急停</td></tr>
</table>
<div class="tip">先点击页面空白处获得键盘焦点，再按按键</div>
<div class="tip">左键点地图 = 发导航目标</div>
<div class="tip">右键点地图 = 设置当前位置（初始位姿）</div>
</div>
<script>
var cv = document.getElementById("map");
var ctx = cv.getContext("2d");
var map = null;
var odom = {x:0, y:0, theta:0};
var goal = null;
var show_robot = true;
var keys = {};

// 8-29: 急停——直接 I2C 清零（底盘崩了也能停），独立于普通停止
function emergencyStop(){
  var st = document.getElementById("status");
  var b = document.getElementById("btnEStop");
  st.textContent = "急停中...（切断底盘与导航）";
  if(b){ b.textContent = "急停中..."; }
  fetch("/api/emergency_stop").then(function(r){return r.json();}).then(function(d){
    st.textContent = d.ok ? "已急停 ✓ 车已停" : ("急停失败: " + d.msg);
    if(b){ setTimeout(function(){ b.textContent = "急停"; }, 3000); }
    setTimeout(function(){ st.textContent = ""; }, 4000);
  }).catch(function(){ st.textContent = "急停失败: 网络错误"; });
}

// 加载地图列表到下拉框（页面打开时调用）
function loadMaps(){
  fetch("/api/nav_maps").then(function(r){ return r.json(); }).then(function(d){
    var sel = document.getElementById("mapSel");
    if(!sel || !d.ok || !d.maps || !d.maps.length) return;
    sel.innerHTML = "";
    d.maps.forEach(function(m){
      var o = document.createElement("option");
      o.value = m; o.textContent = m;
      sel.appendChild(o);
    });
    // 8-29 16:03: map_save 强制优先（最新保存的地图）；localStorage 记忆仅在其他图时兜底
    var last = localStorage.getItem("armbotMap") || "";
    var prefer = d.maps.indexOf("map_save") >= 0 ? "map_save"
               : (d.maps.indexOf(last) >= 0 ? last : d.maps[0]);
    sel.value = prefer;
    // 选择变化时记忆
    sel.onchange = function(){ try{ localStorage.setItem("armbotMap", sel.value); }catch(e){} };
  }).catch(function(){});
}

// 启动导航：选地图 → 后端启动底盘+nav.launch（Web 保持运行）；90秒防重复
function startNav(){
  var b = document.getElementById("btnNav");
  var st = document.getElementById("status");
  var sel = document.getElementById("mapSel");
  if(b.disabled) return;
  var m = sel && sel.value ? sel.value : "map_verify";
  b.disabled = true; b.textContent = "启动中...";
  // 8-29: 与重新建图互斥（两套链路不能同时跑）
  var bm = document.getElementById("btnMap");
  if(bm) bm.disabled = true;
  st.textContent = "导航启动中（地图: " + m + "）...";
  fetch("/api/start_nav?map=" + encodeURIComponent(m)).then(function(r){ return r.json(); }).then(function(d){
    st.textContent = d.ok ? d.msg : ("启动失败: " + d.msg);
  }).catch(function(){ st.textContent = "启动失败: 网络错误"; }).then(function(){
    setTimeout(function(){
      b.disabled = false; b.textContent = "启动导航";
      if(bm) bm.disabled = false;
    }, 90000);
  });
}

// 开始建图：调后端一键启动雷达/底盘/slam（不重启 Web）；30秒防重复
function startMapping(){
  var b = document.getElementById("btnMap");
  var st = document.getElementById("status");
  if(b.disabled) return;
  b.disabled = true; b.textContent = "启动中...";
  // 8-29: 与启动导航互斥（两套链路不能同时跑）
  var bn = document.getElementById("btnNav");
  if(bn) bn.disabled = true;
  st.textContent = "建图启动中...";
  fetch("/api/start_mapping").then(function(r){return r.json();}).then(function(d){
    st.textContent = d.ok ? d.msg : ("启动失败: " + d.msg);
  }).catch(function(){ st.textContent = "启动失败: 网络错误"; }).then(function(){
    setTimeout(function(){
      b.disabled = false; b.textContent = "重新建图";
      if(bn) bn.disabled = false;
    }, 10000);
  });
}

// 8-29: 保存当前地图（后端写 ~/map_save.pgm + .yaml）
function saveMap(){
  var b = document.getElementById("btnSave");
  var st = document.getElementById("status");
  if(b.disabled) return;
  b.disabled = true; b.textContent = "保存中...";
  st.textContent = "保存地图中...";
  fetch("/api/map_save").then(function(r){return r.json();}).then(function(d){
    st.textContent = d.ok ? ("✓ " + d.msg) : ("保存失败: " + d.msg);
    setTimeout(function(){ st.textContent = ""; }, 6000);
  }).catch(function(){ st.textContent = "保存失败: 网络错误"; }).then(function(){
    b.disabled = false; b.textContent = "保存地图";
  });
}

// 按键监听放 window 级（防焦点丢失导致 keyup 漏掉）；页面失焦自动清键+急停
window.addEventListener("keydown", function(e){
  if(!e.repeat){ keys[e.code] = 1; }
  if(e.code === "Space"){ stopAll(); }  // 空格急停：取消导航 + 零速度双保险
  e.preventDefault();
});
window.addEventListener("keyup", function(e){
  delete keys[e.code];
  var s = speed();
  if(s.x===0 && s.y===0 && s.z===0){
    send(0,0,0);  // 全部松开：发一次零速度停住
  } else {
    update();
  }
});
// 页面失焦（切窗口/点其他软件）：清空按键 + 急停，防止车继续走
window.addEventListener("blur", function(){
  keys = {};
  send(0,0,0);
});

// 急停：零速度 + 取消导航目标（Nav2 会在取消后停止发 cmd_vel，防止覆盖急停）
function stopAll(){
  send(0,0,0);
  fetch("/api/cancel_goal").then(function(r){ return r.json(); }).then(function(d){
    var st = document.getElementById("status");
    if(st) st.textContent = d.ok ? "已停止（导航目标已取消）" : "已发零速度急停";
  }).catch(function(){});
}

function speed(){
  var x=0, y=0, z=0;
  if(keys["KeyW"]||keys["ArrowUp"]){ x += 0.15; }
  if(keys["KeyS"]||keys["ArrowDown"]){ x -= 0.15; }
  if(keys["KeyA"]){ z += 0.4; }
  if(keys["KeyD"]){ z -= 0.4; }
  if(keys["KeyQ"]){ y += 0.15; }
  if(keys["KeyE"]){ y -= 0.15; }
  return {x:x, y:y, z:z};
}
function update(){
  var s = speed();
  // 无按键时不再发零速度轮询——避免把 Nav2 的 cmd_vel 顶掉（导航/遥控冲突）
  if(s.x !== 0 || s.y !== 0 || s.z !== 0){
    fetch("/api/cmd?x=" + s.x + "&y=" + s.y + "&z=" + s.z);
  }
}
setInterval(update, 100);

function load(){
  loadMaps();  // 8-29: 加载地图下拉列表
  try{
    fetch("/api/map").then(function(r){ return r.json(); }).then(function(m){
      if(m && m.data){ map = m; } else { map = null; }  // 8-29: 地图被清空时也清空前端缓存（否则旧图残留）
      return fetch("/api/odom");
    }).then(function(r){ return r.json(); }).then(function(o){
      odom = o;
      var st = document.getElementById("status");
      var mw = map ? (map.width + "x" + map.height) : "-";
      st.textContent = "车: x=" + o.x.toFixed(2) + " y=" + o.y.toFixed(2) +
        " yaw=" + (o.theta*180/Math.PI).toFixed(0) + "deg  地图:" + mw;
      if(map){ draw(); } else { cv.width = cv.width; }  // 8-29: 地图清空时同步清空画布
    });
  }catch(e){
    document.getElementById("status").textContent = "等待ROS数据...";
  }
  setTimeout(load, 400);
}

function draw(){
  var d = map.data, H = d.length, W = d[0].length;
  cv.width = W; cv.height = H;
  var img = ctx.createImageData(W, H);
  for(var j=0; j<H; j++){
    for(var i=0; i<W; i++){
      var v = d[j][i];
      var p = img.data;
      var p4 = ((H-1-j)*W + i) * 4;
      if(v === -1){ p[p4]=130; p[p4+1]=130; p[p4+2]=130; }     // 未知 → 深灰
      else if(v >= 100){ p[p4]=0; p[p4+1]=0; p[p4+2]=0; }       // 障碍 → 黑
      else { p[p4]=255; p[p4+1]=255; p[p4+2]=255; }             // 空闲 → 白（8-29: 与未知区分）
      p[p4+3]=255;
    }
  }
  ctx.putImageData(img, 0, 0);

  // ── 车位置 + 正方向（车头朝向）指示 ──
  // 地图坐标系: X 右, Y 上; canvas: X 右, Y 下 → y 取反
  var cx = (odom.x - map.ox) / map.res / map.step;
  var cy = -((odom.y - map.oy) / map.res / map.step) + (H - 1);
  var th = odom.theta;

  // 小车图形（可通过"显示小车"开关隐藏）
  if(show_robot){
  // 车体轮廓（蓝矩形 35cm x 25cm，与实际比例一致，浅蓝填充更醒目）
  var hl = 0.175 / map.res / map.step;  // 半长 17.5cm
  var hw = 0.125 / map.res / map.step;  // 半宽 12.5cm
  var cos = Math.cos(th), sin = Math.sin(th);
  var corners = [[hl,hw],[hl,-hw],[-hl,-hw],[-hl,hw]];
  ctx.beginPath();
  ctx.moveTo(cx + corners[0][0]*cos - corners[0][1]*sin, cy - (corners[0][0]*sin + corners[0][1]*cos));
  for(var k=1;k<4;k++){
    ctx.lineTo(cx + corners[k][0]*cos - corners[k][1]*sin, cy - (corners[k][0]*sin + corners[k][1]*cos));
  }
  ctx.closePath();
  ctx.fillStyle = "rgba(0,170,255,0.22)";
  ctx.fill();
  ctx.strokeStyle = "#00aaff";
  ctx.lineWidth = 2;
  ctx.stroke();

  // 方向箭头（红色，从车心指向车头朝向）
  var L = Math.max(3, hl * 0.9);        // 箭头长度 = 车半长 90%，不超车身
  var hx = cx + L * Math.cos(th);
  var hy = cy - L * Math.sin(th);        // canvas Y 向下取反
  ctx.strokeStyle = "#ff0000";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(hx, hy);
  ctx.stroke();
  // 箭头尖（小三角形，与箭头成比例）
  var a1 = th + 2.6, a2 = th - 2.6;
  var s1 = Math.max(1.5, L * 0.6), s2 = Math.max(1.5, L * 0.6);
  ctx.fillStyle = "#ff0000";
  ctx.beginPath();
  ctx.moveTo(hx, hy);
  ctx.lineTo(hx - s1 * Math.cos(a1), hy + s1 * Math.sin(a1));
  ctx.lineTo(hx - s2 * Math.cos(a2), hy + s2 * Math.sin(a2));
  ctx.closePath();
  ctx.fill();

  // 中心红点（小）
  ctx.fillStyle = "#ff0000";
  ctx.beginPath();
  ctx.arc(cx, cy, Math.max(1, L * 0.3), 0, 7);
  ctx.fill();
  }

  // 导航目标点（绿圈）
  if(goal){
    var gpx = (goal.x - map.ox) / map.res / map.step;
    var gpy = -((goal.y - map.oy) / map.res / map.step) + (H - 1);
    ctx.strokeStyle = "#00ff00";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(gpx, gpy, 8, 0, 7);
    ctx.stroke();
    ctx.fillStyle = "#00ff00";
    ctx.beginPath();
    ctx.arc(gpx, gpy, 2.5, 0, 7);
    ctx.fill();
  }
}

// 左键点击 → 发导航目标；右键点击 → 设置当前位置（初始位姿，带当前朝向）
function clickToMap(e, api, th){
  if(!map) return;
  var r = cv.getBoundingClientRect();
  var px = (e.clientX - r.left) * (cv.width / r.width);
  var py = (e.clientY - r.top) * (cv.height / r.height);
  var gx = px * map.res * map.step + map.ox;
  var gy = -((py - (cv.height - 1)) * map.res * map.step) + map.oy;
  fetch("/api/" + api + "?x=" + gx.toFixed(2) + "&y=" + gy.toFixed(2) + "&th=" + (th || 0));
  return {x:gx, y:gy};
}
cv.addEventListener("click", function(e){
  var p = clickToMap(e, "goal", 0);
  if(p){ goal = p; document.getElementById("status").textContent = "导航目标: (" + p.x.toFixed(2) + ", " + p.y.toFixed(2) + ")"; }
});
var poseThUserSet = false;
document.getElementById("poseTh").addEventListener("input", function(){ poseThUserSet = true; });
cv.addEventListener("contextmenu", function(e){
  e.preventDefault();
  // 初始朝向：默认用底盘实测车头方向（odom.oth），用户手动改过"初始朝向"框才用输入值
  var thd;
  if(poseThUserSet){
    thd = parseFloat(document.getElementById("poseTh").value) || 0;
  }else{
    thd = (odom && typeof odom.oth === "number") ? odom.oth * 180 / Math.PI : 0;
  }
  var p = clickToMap(e, "pose", thd * Math.PI / 180);
  if(p){ goal = null; document.getElementById("status").textContent = "设置当前位置: (" + p.x.toFixed(2) + ", " + p.y.toFixed(2) + ") 朝向 " + thd.toFixed(0) + "deg"; }
});

load();
</script>
</body>
</html>"""


def main():
    global NODE
    rclpy.init()
    node = RosNode()
    NODE = node
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    srv = ThreadingHTTPServer(("0.0.0.0", 8080), Handler)
    print("Armbot Web on :8080", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
