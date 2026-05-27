import streamlit as st
import folium
from streamlit_folium import st_folium
from folium.plugins import Draw
import json
import os
from datetime import datetime
import pandas as pd
import math
import random
import time

# ====================== 页面配置 ======================
st.set_page_config(
    page_title="无人机航线规划与飞行监控系统",
    layout="wide",
    initial_sidebar_state="expanded"
)

st.title("🚁 无人机航线规划与飞行监控系统")

# ====================== 配置文件 ======================
CONFIG_FILE = "obstacle_config.json"

# ====================== 初始化 Session State ======================
if "start_point" not in st.session_state:
    st.session_state.start_point = (32.2345, 118.7492)
if "end_point" not in st.session_state:
    st.session_state.end_point = (32.2337, 118.7496)
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "flight_altitude" not in st.session_state:
    st.session_state.flight_altitude = 15.0
if "safety_radius" not in st.session_state:
    st.session_state.safety_radius = 15.0
if "current_route" not in st.session_state:
    st.session_state.current_route = []
if "map_center" not in st.session_state:
    st.session_state.map_center = [32.2341, 118.7494]
if "pending_polygon" not in st.session_state:
    st.session_state.pending_polygon = None
if "set_mode" not in st.session_state:
    st.session_state.set_mode = None
if "route_mode" not in st.session_state:
    st.session_state.route_mode = "best"
if "last_click" not in st.session_state:
    st.session_state.last_click = None

# ====================== 飞行监控状态 ======================
if "mission_active" not in st.session_state:
    st.session_state.mission_active = False
if "mission_paused" not in st.session_state:
    st.session_state.mission_paused = False
if "current_waypoint_index" not in st.session_state:
    st.session_state.current_waypoint_index = 0
if "mission_start_time" not in st.session_state:
    st.session_state.mission_start_time = None
if "flight_speed" not in st.session_state:
    st.session_state.flight_speed = 8.5
if "battery_level" not in st.session_state:
    st.session_state.battery_level = 100
if "flight_log" not in st.session_state:
    st.session_state.flight_log = []
if "current_position" not in st.session_state:
    st.session_state.current_position = None
if "simulation_running" not in st.session_state:
    st.session_state.simulation_running = False

# ====================== 通信链路状态 ======================
if "gcs_status" not in st.session_state:
    st.session_state.gcs_status = "在线"
if "obc_status" not in st.session_state:
    st.session_state.obc_status = "在线"
if "fcu_status" not in st.session_state:
    st.session_state.fcu_status = "在线"

# ====================== 心跳状态 ======================
if "heartbeat_history" not in st.session_state:
    st.session_state.heartbeat_history = []
if "heartbeat_running" not in st.session_state:
    st.session_state.heartbeat_running = False

# ====================== 保存/加载 ======================
def save_data():
    data = {
        "obstacles": st.session_state.obstacles,
        "start_point": st.session_state.start_point,
        "end_point": st.session_state.end_point,
        "flight_altitude": st.session_state.flight_altitude,
        "safety_radius": st.session_state.safety_radius,
        "route_mode": st.session_state.route_mode,
        "save_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_data():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        st.session_state.obstacles = data.get("obstacles", [])
        st.session_state.start_point = tuple(data.get("start_point", (32.2345, 118.7492)))
        st.session_state.end_point = tuple(data.get("end_point", (32.2337, 118.7496)))
        st.session_state.flight_altitude = data.get("flight_altitude", 15.0)
        st.session_state.safety_radius = data.get("safety_radius", 15.0)
        st.session_state.route_mode = data.get("route_mode", "best")

load_data()

# ====================== 几何计算函数 ======================
def calculate_distance(point1, point2):
    lat1, lng1 = point1
    lat2, lng2 = point2
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lng2 - lng1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def point_in_polygon(point, polygon):
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside

def segments_intersect(p1, p2, p3, p4):
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    
    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)
    
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False

def line_intersects_polygon(line_start, line_end, polygon):
    for i in range(len(polygon)):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % len(polygon)]
        if segments_intersect(line_start, line_end, p1, p2):
            return True
    if point_in_polygon(line_start, polygon) or point_in_polygon(line_end, polygon):
        return True
    return False

def get_polygon_bounds(polygon):
    lats = [p[0] for p in polygon]
    lngs = [p[1] for p in polygon]
    return min(lats), max(lats), min(lngs), max(lngs)

def get_polygon_center(polygon):
    lats = [p[0] for p in polygon]
    lngs = [p[1] for p in polygon]
    return sum(lats)/len(lats), sum(lngs)/len(lngs)

def is_path_safe(start, end, obstacles, flight_altitude):
    for obs in obstacles:
        if obs.get("height", 0) >= flight_altitude:
            polygon = obs.get("polygon", [])
            if polygon and line_intersects_polygon(start, end, polygon):
                return False
    return True

# ====================== 通用 Dijkstra 路径规划 ======================
def dijkstra_path(nodes, start, end, obstacles, flight_altitude):
    """在节点列表中找最短安全路径，返回路径点列表"""
    # 去重
    unique = []
    for n in nodes:
        if n not in unique:
            unique.append(n)
    
    n = len(unique)
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            if is_path_safe(unique[i], unique[j], obstacles, flight_altitude):
                dist = calculate_distance(unique[i], unique[j])
                adj[i].append((j, dist))
                adj[j].append((i, dist))
    
    start_idx = unique.index(start)
    end_idx = unique.index(end)
    INF = float('inf')
    dist = [INF] * n
    dist[start_idx] = 0
    prev = [-1] * n
    visited = [False] * n
    
    for _ in range(n):
        u = -1
        min_d = INF
        for i in range(n):
            if not visited[i] and dist[i] < min_d:
                min_d = dist[i]
                u = i
        if u == -1:
            break
        visited[u] = True
        for v, w in adj[u]:
            if not visited[v] and dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                prev[v] = u
    
    if dist[end_idx] == INF:
        return [start, end]  # 降级
    
    path = []
    cur = end_idx
    while cur != -1:
        path.insert(0, unique[cur])
        cur = prev[cur]
    return path

# ====================== 智能穿行（所有方向，从中间穿过） ======================
def plan_route_best(start, end, obstacles, flight_altitude, safety_radius):
    high_obstacles = [obs for obs in obstacles if obs.get("height", 0) >= flight_altitude]
    if not high_obstacles:
        return [start, end]
    if is_path_safe(start, end, high_obstacles, flight_altitude):
        return [start, end]
    
    # 候选点：起点、终点、每个障碍物周围8个方向
    nodes = [start, end]
    for obs in high_obstacles:
        bounds = get_polygon_bounds(obs["polygon"])
        min_lat, max_lat, min_lng, max_lng = bounds
        center_lat = (min_lat + max_lat) / 2
        center_lng = (min_lng + max_lng) / 2
        offset_meters = safety_radius * 2.5
        offset_lat = offset_meters / 111320
        offset_lng = offset_meters / (111320 * math.cos(math.radians(center_lat)))
        nodes.append((min_lat - offset_lat, center_lng))   # 上
        nodes.append((max_lat + offset_lat, center_lng))   # 下
        nodes.append((center_lat, min_lng - offset_lng))   # 左
        nodes.append((center_lat, max_lng + offset_lng))   # 右
        nodes.append((min_lat - offset_lat, min_lng - offset_lng))  # 左上
        nodes.append((min_lat - offset_lat, max_lng + offset_lng))  # 右上
        nodes.append((max_lat + offset_lat, min_lng - offset_lng))  # 左下
        nodes.append((max_lat + offset_lat, max_lng + offset_lng))  # 右下
    
    return dijkstra_path(nodes, start, end, high_obstacles, flight_altitude)

# ====================== 强制向左绕行（只使用左侧候选点） ======================
def plan_route_left(start, end, obstacles, flight_altitude, safety_radius):
    high_obstacles = [obs for obs in obstacles if obs.get("height", 0) >= flight_altitude]
    if not high_obstacles:
        return [start, end]
    if is_path_safe(start, end, high_obstacles, flight_altitude):
        return [start, end]
    
    nodes = [start, end]
    for obs in high_obstacles:
        bounds = get_polygon_bounds(obs["polygon"])
        min_lat, max_lat, min_lng, max_lng = bounds
        center_lat = (min_lat + max_lat) / 2
        center_lng = (min_lng + max_lng) / 2
        offset_meters = safety_radius * 2.5
        offset_lat = offset_meters / 111320
        offset_lng = offset_meters / (111320 * math.cos(math.radians(center_lat)))
        # 只添加左侧相关的点：左、左上、左下
        nodes.append((center_lat, min_lng - offset_lng))               # 左
        nodes.append((min_lat - offset_lat, min_lng - offset_lng))    # 左上
        nodes.append((max_lat + offset_lat, min_lng - offset_lng))    # 左下
        # 为了满足连接，也添加上下中心（但不加右侧）
        nodes.append((min_lat - offset_lat, center_lng))
        nodes.append((max_lat + offset_lat, center_lng))
    
    return dijkstra_path(nodes, start, end, high_obstacles, flight_altitude)

# ====================== 强制向右绕行（只使用右侧候选点） ======================
def plan_route_right(start, end, obstacles, flight_altitude, safety_radius):
    high_obstacles = [obs for obs in obstacles if obs.get("height", 0) >= flight_altitude]
    if not high_obstacles:
        return [start, end]
    if is_path_safe(start, end, high_obstacles, flight_altitude):
        return [start, end]
    
    nodes = [start, end]
    for obs in high_obstacles:
        bounds = get_polygon_bounds(obs["polygon"])
        min_lat, max_lat, min_lng, max_lng = bounds
        center_lat = (min_lat + max_lat) / 2
        center_lng = (min_lng + max_lng) / 2
        offset_meters = safety_radius * 2.5
        offset_lat = offset_meters / 111320
        offset_lng = offset_meters / (111320 * math.cos(math.radians(center_lat)))
        # 只添加右侧相关的点：右、右上、右下
        nodes.append((center_lat, max_lng + offset_lng))               # 右
        nodes.append((min_lat - offset_lat, max_lng + offset_lng))    # 右上
        nodes.append((max_lat + offset_lat, max_lng + offset_lng))    # 右下
        # 添加上下中心以便连接
        nodes.append((min_lat - offset_lat, center_lng))
        nodes.append((max_lat + offset_lat, center_lng))
    
    return dijkstra_path(nodes, start, end, high_obstacles, flight_altitude)

def plan_route():
    start = st.session_state.start_point
    end = st.session_state.end_point
    obstacles = st.session_state.obstacles
    altitude = st.session_state.flight_altitude
    safety_radius = st.session_state.safety_radius
    mode = st.session_state.route_mode
    
    if mode == "best":
        route = plan_route_best(start, end, obstacles, altitude, safety_radius)
    elif mode == "left":
        route = plan_route_left(start, end, obstacles, altitude, safety_radius)
    else:  # right
        route = plan_route_right(start, end, obstacles, altitude, safety_radius)
    
    st.session_state.current_route = route
    st.session_state.current_waypoint_index = 0
    st.session_state.current_position = start
    st.session_state.simulation_running = False
