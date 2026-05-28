import streamlit as st
import folium
from streamlit_folium import st_folium
from folium.plugins import Draw, AntPath
import json
import os
from datetime import datetime, timedelta
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import LineString, Polygon, Point
import time

# ====================== 页面全局配置 ======================
st.set_page_config(
    page_title="无人机航线规划与障碍物管理系统",
    layout="wide",
    initial_sidebar_state="expanded"
)

st.title("🚁 无人机航线规划与障碍物管理系统")

# ====================== 永久配置文件 ======================
CONFIG_FILE = "obstacle_config.json"

# ====================== 初始化 ======================
SCHOOL_CENTER = [32.2341, 118.7494]

if "start_point" not in st.session_state:
    st.session_state.start_point = (32.2345, 118.7492)
if "end_point" not in st.session_state:
    st.session_state.end_point = (32.2337, 118.7496)
if "obstacles" not in st.session_state:
    st.session_state.obstacles = []
if "deployed_obstacles" not in st.session_state:
    st.session_state.deployed_obstacles = []
if "flight_height" not in st.session_state:
    st.session_state.flight_height = 30.0
if "safe_radius" not in st.session_state:
    st.session_state.safe_radius = 5.0
if "route_strategy" not in st.session_state:
    st.session_state.route_strategy = "向左绕行"
if "temp_obstacle" not in st.session_state:
    st.session_state.temp_obstacle = None
if "temp_obstacle_height" not in st.session_state:
    st.session_state.temp_obstacle_height = 30.0
if "map_clicked_point" not in st.session_state:
    st.session_state.map_clicked_point = None

# 飞行监控状态
if "flight_running" not in st.session_state:
    st.session_state.flight_running = False
if "current_waypoint_idx" not in st.session_state:
    st.session_state.current_waypoint_idx = 0
if "flight_speed" not in st.session_state:
    st.session_state.flight_speed = 8.5
if "flight_start_time" not in st.session_state:
    st.session_state.flight_start_time = None
if "flight_path" not in st.session_state:
    st.session_state.flight_path = []
if "flight_pos" not in st.session_state:
    st.session_state.flight_pos = None

# 通信链路状态
if "gcs_online" not in st.session_state:
    st.session_state.gcs_online = True
if "obc_online" not in st.session_state:
    st.session_state.obc_online = True
if "fcu_online" not in st.session_state:
    st.session_state.fcu_online = True
if "latency" not in st.session_state:
    st.session_state.latency = 25
if "packet_loss" not in st.session_state:
    st.session_state.packet_loss = 0.2

# P2 通信日志状态
if "comm_log" not in st.session_state:
    st.session_state.comm_log = []
if "comm_log_last_wp" not in st.session_state:
    st.session_state.comm_log_last_wp = -1

# ====================== P2 通信日志工具函数 ======================
def comm_log_add(direction, from_node, to_node, event, detail=""):
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    st.session_state.comm_log.append({
        "time": ts,
        "direction": direction,
        "from": from_node,
        "to": to_node,
        "event": event,
        "detail": detail
    })

def comm_log_generate_mission_start(start_point, end_point, flight_height):
    st.session_state.comm_log = []
    st.session_state.comm_log_last_wp = -1
    detail = (f"起点: ({start_point[0]:.6f}, {start_point[1]:.6f}), "
              f"终点: ({end_point[0]:.6f}, {end_point[1]:.6f}), "
              f"目标高度: {flight_height}m")
    comm_log_add("GCS->OBC->FCU", "GCS", "OBC", "导航目标", detail)

def comm_log_generate_route_plan(waypoints, algorithm="A*", obstacle_count=0):
    route_len = 0
    if len(waypoints) >= 2:
        from shapely.geometry import LineString as LS
        route_len = LS(waypoints).length * 111139
    for i in range(min(3, max(1, obstacle_count))):
        alt_len = route_len * (1 + (i * 0.03))
        alt_pts = len(waypoints) + i
        comm_log_add("OBC_INTERNAL", "OBC", "OBC",
                     "航线规划",
                     f"航线规划完成 | 类型: horizontal | 航点数: {alt_pts} | 路径长度: {alt_len:.1f}m")
    comm_log_add("OBC_INTERNAL", "OBC", "OBC",
                 "航线规划",
                 f"开始航线规划 | 算法: {algorithm} | 障碍物数量: {obstacle_count}")

def comm_log_update_waypoint(wp_idx, total_wps, flight_pos):
    if wp_idx == st.session_state.comm_log_last_wp:
        return
    st.session_state.comm_log_last_wp = wp_idx
    if wp_idx == total_wps - 1:
        comm_log_add("FCU->OBC->GCS", "FCU", "OBC", "MISSION_COMPLETE", "")
        comm_log_add("FCU->OBC->GCS", "OBC", "GCS", "MISSION_COMPLETE", "")
    else:
        comm_log_add("FCU->OBC->GCS", "FCU", "OBC", f"WP_REACHED #{wp_idx}", "")
        comm_log_add("FCU->OBC->GCS", "OBC", "GCS", f"WP_REACHED #{wp_idx}", "")

def comm_log_ack():
    comm_log_add("FCU->OBC->GCS", "FCU", "OBC", "ACK", "Mode: AUTO")
    comm_log_add("FCU->OBC->GCS", "OBC", "GCS", "ACK", "Mode: AUTO")

# ====================== 保存/加载 ======================
def save_obstacles():
    data = {
        "obstacles": st.session_state.obstacles,
        "deployed_obstacles": st.session_state.deployed_obstacles,
        "start_point": st.session_state.start_point,
        "end_point": st.session_state.end_point,
        "flight_height": st.session_state.flight_height,
        "safe_radius": st.session_state.safe_radius,
        "save_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "version": "v23.0"
    }
    with open(CONFIG_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def load_obstacles():
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        st.session_state.obstacles = data.get("obstacles", [])
        st.session_state.deployed_obstacles = data.get("deployed_obstacles", [])
        st.session_state.start_point = tuple(data.get("start_point", (32.2345, 118.7492)))
        st.session_state.end_point = tuple(data.get("end_point", (32.2337, 118.7496)))
        st.session_state.flight_height = float(data.get("flight_height", 30.0))
        st.session_state.safe_radius = float(data.get("safe_radius", 5.0))
        # 兼容旧数据：为缺失 id 的障碍物自动分配 id
        for idx, obs in enumerate(st.session_state.obstacles):
            if 'id' not in obs:
                obs['id'] = idx + 1
        for obs in st.session_state.deployed_obstacles:
            if 'id' not in obs:
                found = None
                for o in st.session_state.obstacles:
                    if o.get('coords') == obs.get('coords'):
                        found = o.get('id')
                        break
                obs['id'] = found if found is not None else len(st.session_state.obstacles) + 1

load_obstacles()

# ====================== 工具函数 ======================
def meters_to_degrees(meters, lat):
    lat_deg = meters / 111139
    lon_deg = meters / (111139 * np.cos(np.radians(lat)))
    return lat_deg, lon_deg

def get_buffered_obstacle_coords(obstacle_coords, safe_radius_meters):
    if safe_radius_meters <= 0:
        return obstacle_coords
    poly = Polygon(obstacle_coords)
    lat = obstacle_coords[0][0]
    lat_deg, lon_deg = meters_to_degrees(safe_radius_meters, lat)
    buffer_deg = max(lat_deg, lon_deg)
    buffered_poly = poly.buffer(buffer_deg)
    return list(buffered_poly.exterior.coords)

def line_intersects_polygon(line_points, polygon_coords, safe_radius=0):
    if len(line_points) < 2 or len(polygon_coords) < 3:
        return False
    line = LineString(line_points)
    if safe_radius > 0:
        buffered_coords = get_buffered_obstacle_coords(polygon_coords, safe_radius)
        poly = Polygon(buffered_coords)
    else:
        poly = Polygon(polygon_coords)
    return line.intersects(poly) and not line.touches(poly)

def latlon_to_meters(p, origin):
    m_per_lat = 111139.0
    m_per_lng = 111139.0 * np.cos(np.radians(origin[0]))
    x = (p[1] - origin[1]) * m_per_lng
    y = (p[0] - origin[0]) * m_per_lat
    return (x, y)

def meters_to_latlon(xy, origin):
    m_per_lat = 111139.0
    m_per_lng = 111139.0 * np.cos(np.radians(origin[0]))
    lat = origin[0] + xy[1] / m_per_lat
    lng = origin[1] + xy[0] / m_per_lng
    return (lat, lng)

def make_meter_poly(coords_latlon, origin, safe_radius=0):
    xy = [latlon_to_meters(c, origin) for c in coords_latlon]
    poly = Polygon(xy)
    if not poly.is_valid:
        poly = poly.buffer(0)
    if safe_radius > 0:
        poly = poly.buffer(safe_radius)
    return poly

def seg_blocked_m(p1_m, p2_m, polys_m):
    line = LineString([p1_m, p2_m])
    for poly in polys_m:
        inter = line.intersection(poly)
        if not inter.is_empty and inter.length > 1e-6:
            return True
    return False

def walk_boundary(poly_m, entry_m, exit_m, side):
    exterior = list(poly_m.exterior.coords)
    n = len(exterior) - 1

    def nearest_idx(pt):
        best_i, best_d = 0, float('inf')
        for i in range(n):
            d = np.hypot(exterior[i][0] - pt[0], exterior[i][1] - pt[1])
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    i_entry = nearest_idx(entry_m)
    i_exit = nearest_idx(exit_m)

    if i_entry == i_exit:
        return []

    if side == 'left':
        if i_entry > i_exit:
            indices = list(range(i_entry, i_exit, -1))
        else:
            indices = list(range(i_entry, -1, -1)) + list(range(n - 1, i_exit, -1))
    else:
        if i_entry < i_exit:
            indices = list(range(i_entry, i_exit))
        else:
            indices = list(range(i_entry, n)) + list(range(0, i_exit))

    pts = [exterior[i] for i in indices[1:]]
    return pts

def route_total_length(waypoints):
    total = 0.0
    for i in range(len(waypoints) - 1):
        total += np.hypot(waypoints[i+1][0] - waypoints[i][0],
                          waypoints[i+1][1] - waypoints[i][1])
    return total

def find_route(A, B, obstacles, flight_height, side, safe_radius=0):
    high_obs = [obs for obs in obstacles if obs.get("height", 0) > flight_height]
    if not high_obs:
        return [A, B]

    origin = A
    polys_m = [make_meter_poly(obs["coords"], origin, safe_radius) for obs in high_obs]
    A_m = latlon_to_meters(A, origin)
    B_m = latlon_to_meters(B, origin)

    waypoints_m = [A_m]
    cur_m = A_m
    visited = set()

    for _ in range(20):
        hit_idx = None
        for i, poly in enumerate(polys_m):
            if seg_blocked_m(cur_m, B_m, [poly]):
                hit_idx = i
                break
        if hit_idx is None:
            break

        poly_hit = polys_m[hit_idx]
        line_m = LineString([cur_m, B_m])
        inter = line_m.intersection(poly_hit)

        if inter.is_empty:
            break

        inter_pts = []
        if inter.geom_type == 'Point':
            inter_pts = [(inter.x, inter.y)]
        elif inter.geom_type == 'MultiPoint':
            inter_pts = [(p.x, p.y) for p in inter.geoms]
        elif inter.geom_type == 'LineString':
            inter_pts = list(inter.coords)
        elif inter.geom_type == 'MultiLineString':
            for geom in inter.geoms:
                inter_pts += list(geom.coords)
        elif inter.geom_type == 'GeometryCollection':
            for geom in inter.geoms:
                if geom.geom_type == 'Point':
                    inter_pts.append((geom.x, geom.y))
                elif geom.geom_type == 'LineString':
                    inter_pts += list(geom.coords)

        if len(inter_pts) < 2:
            inter_pts = [cur_m, B_m]

        def proj(pt):
            dx, dy = B_m[0]-cur_m[0], B_m[1]-cur_m[1]
            return (pt[0]-cur_m[0])*dx + (pt[1]-cur_m[1])*dy
        inter_pts.sort(key=proj)
        entry_m = inter_pts[0]
        exit_m = inter_pts[-1]

        boundary_pts = walk_boundary(poly_hit, entry_m, exit_m, side)
        bypass = [entry_m] + boundary_pts + [exit_m]

        for pt in bypass:
            key = (round(pt[0], 2), round(pt[1], 2))
            if key not in visited:
                visited.add(key)
                if waypoints_m[-1] != pt:
                    waypoints_m.append(pt)

        cur_m = exit_m

    if waypoints_m[-1] != B_m:
        waypoints_m.append(B_m)

    return [meters_to_latlon(p, origin) for p in waypoints_m]

def find_route_with_step_by_step(A, B, obstacles, flight_height, strategy, safe_radius=0):
    if strategy == "向左绕行":
        return find_route(A, B, obstacles, flight_height, 'left', safe_radius)
    elif strategy == "向右绕行":
        return find_route(A, B, obstacles, flight_height, 'right', safe_radius)
    else:
        left = find_route(A, B, obstacles, flight_height, 'left', safe_radius)
        right = find_route(A, B, obstacles, flight_height, 'right', safe_radius)
        return left if route_total_length(left) <= route_total_length(right) else right

def compute_routes(A, B, obstacles, flight_height, safe_radius, strategy):
    high_obs = [obs for obs in obstacles if obs.get("height", 0) > flight_height]
    origin = A
    polys_m = [make_meter_poly(obs["coords"], origin, safe_radius) for obs in high_obs]
    A_m = latlon_to_meters(A, origin)
    B_m = latlon_to_meters(B, origin)
    if not seg_blocked_m(A_m, B_m, polys_m):
        return {"✈️ 直接飞跃": [A, B]}

    left_route = find_route(A, B, obstacles, flight_height, 'left', safe_radius)
    right_route = find_route(A, B, obstacles, flight_height, 'right', safe_radius)

    if strategy == "向左绕行":
        return {"⬅️ 向左绕行": left_route}
    elif strategy == "向右绕行":
        return {"➡️ 向右绕行": right_route}
    else:
        left_len = route_total_length(left_route)
        right_len = route_total_length(right_route)
        if left_len <= right_len:
            return {"⭐ 最佳航线（向左）": left_route}
        else:
            return {"⭐ 最佳航线（向右）": right_route}

def interpolate_for_display(waypoints, steps_per_segment=15):
    if len(waypoints) < 2:
        return waypoints
    path = []
    for i in range(len(waypoints) - 1):
        p1, p2 = waypoints[i], waypoints[i + 1]
        for s in range(steps_per_segment):
            t = s / steps_per_segment
            path.append((p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1])))
    path.append(waypoints[-1])
    return path

# ====================== 界面 ======================
tab1, tab2 = st.tabs(["🗺️ 地图与障碍物管理", "📡 飞行监控"])

with tab1:
    col1, col2 = st.columns([3, 1.2])
    with col1:
        st.subheader("🗺️ 卫星地图")
        m = folium.Map(
            location=SCHOOL_CENTER, zoom_start=18,
            tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
            attr="© 高德地图"
        )
        draw = Draw(
            draw_options={"polygon": {"shapeOptions": {"color": "red", "fillColor": "red", "fillOpacity": 0.5}}},
            edit_options={"edit": True, "remove": True}
        )
        draw.add_to(m)
        folium.Marker(location=st.session_state.start_point, popup="✈️ 起飞点 A",
                      icon=folium.Icon(color="red", icon="play")).add_to(m)
        folium.Marker(location=st.session_state.end_point, popup="🎯 目标点 B",
                      icon=folium.Icon(color="green", icon="flag")).add_to(m)

        for obs in st.session_state.deployed_obstacles:
            color = "red" if obs.get("height", 0) > st.session_state.flight_height else "orange"
            folium.Polygon(locations=obs["coords"], color=color, weight=3, fill=True, fill_opacity=0.3).add_to(m)
            if st.session_state.safe_radius > 0:
                buf_coords = get_buffered_obstacle_coords(obs["coords"], st.session_state.safe_radius)
                folium.Polygon(locations=buf_coords, color="blue", weight=1, fill=False).add_to(m)

        routes = compute_routes(
            st.session_state.start_point, st.session_state.end_point,
            st.session_state.deployed_obstacles,
            st.session_state.flight_height, st.session_state.safe_radius,
            st.session_state.route_strategy
        )
        route_colors = {
            "✈️ 直接飞跃": "blue",
            "⬅️ 向左绕行": "orange",
            "➡️ 向右绕行": "purple",
            "⭐ 最佳航线（向左）": "green",
            "⭐ 最佳航线（向右）": "green"
        }
        for name, pts in routes.items():
            display_pts = interpolate_for_display(pts, steps_per_segment=12)
            folium.PolyLine(locations=display_pts, color=route_colors[name], weight=4, opacity=0.9).add_to(m)
            for i, pt in enumerate(pts):
                if i == 0:
                    folium.CircleMarker(pt, radius=8, color="red", fill=True).add_to(m)
                elif i == len(pts)-1:
                    folium.CircleMarker(pt, radius=8, color="green", fill=True).add_to(m)
                else:
                    folium.CircleMarker(pt, radius=4, color=route_colors[name], fill=True, fill_opacity=0.8,
                                        popup=f"绕行航点 {i}").add_to(m)

        map_data = st_folium(m, width=800, height=650, key="main_map")
        if map_data and map_data.get("last_clicked"):
            clicked = map_data["last_clicked"]
            st.session_state.map_clicked_point = (clicked["lat"], clicked["lng"])
        if map_data and map_data.get("last_active_drawing"):
            drawing = map_data["last_active_drawing"]
            if drawing["geometry"]["type"] == "Polygon":
                coords = [(lat, lng) for lng, lat in drawing["geometry"]["coordinates"][0]]
                st.session_state.temp_obstacle = {
                    "coords": coords,
                    "timestamp": datetime.now().strftime("%H:%M:%S")
                }

    with col2:
        st.subheader("✈️ 航线规划参数")
        flight_h = st.number_input(
            "无人机飞行高度 (m)",
            min_value=0.0,
            max_value=200.0,
            value=float(st.session_state.flight_height),
            step=5.0
        )
        safe_r = st.number_input(
            "安全半径 (m)",
            min_value=0.0,
            max_value=50.0,
            value=float(st.session_state.safe_radius),
            step=1.0
        )
        st.subheader("绕行策略")
        strategy = st.radio("", ["向左绕行", "向右绕行", "最佳航线（自动选择）"],
                            format_func=lambda x: {
                                "向左绕行": "⬅️ 向左绕行",
                                "向右绕行": "➡️ 向右绕行",
                                "最佳航线（自动选择）": "⭐ 最佳航线"
                            }[x])
        if st.button("🚀 规划航线", type="primary", use_container_width=True):
            st.session_state.flight_height = flight_h
            st.session_state.safe_radius = safe_r
            st.session_state.route_strategy = strategy
            save_obstacles()
            st.session_state.flight_path = routes[list(routes.keys())[0]]
            st.session_state.current_waypoint_idx = 0
            st.session_state.flight_pos = st.session_state.flight_path[0]
            st.rerun()

        st.divider()
        total_obs = len(st.session_state.deployed_obstacles)
        high_obs = sum(1 for o in st.session_state.deployed_obstacles if o.get("height", 0) > st.session_state.flight_height)
        col_s1, col_s2 = st.columns(2)
        with col_s1: st.metric("📊 总障碍物", total_obs)
        with col_s2: st.metric("⚠️ 需绕行", high_obs)

        st.divider()
        st.subheader("📍 起飞点/目标点设置")
        if st.session_state.map_clicked_point:
            st.info(f"📍 点击位置: {st.session_state.map_clicked_point[0]:.6f}, {st.session_state.map_clicked_point[1]:.6f}")
        ca, cb = st.columns(2)
        with ca:
            if st.button("✈️ 设为A点", use_container_width=True):
                st.session_state.start_point = st.session_state.map_clicked_point
                save_obstacles()
                st.rerun()
        with cb:
            if st.button("🎯 设为B点", use_container_width=True):
                st.session_state.end_point = st.session_state.map_clicked_point
                save_obstacles()
                st.rerun()

        st.write("**起飞点 A**")
        a_lat = st.number_input("A纬度", value=st.session_state.start_point[0], format="%.6f", step=0.0001, label_visibility="collapsed")
        a_lng = st.number_input("A经度", value=st.session_state.start_point[1], format="%.6f", step=0.0001, label_visibility="collapsed")
        if st.button("✅ 确认A点", use_container_width=True):
            st.session_state.start_point = (a_lat, a_lng)
            save_obstacles()
            st.rerun()

        st.write("**目标点 B**")
        b_lat = st.number_input("B纬度", value=st.session_state.end_point[0], format="%.6f", step=0.0001, label_visibility="collapsed")
        b_lng = st.number_input("B经度", value=st.session_state.end_point[1], format="%.6f", step=0.0001, label_visibility="collapsed")
        if st.button("✅ 确认B点", use_container_width=True):
            st.session_state.end_point = (b_lat, b_lng)
            save_obstacles()
            st.rerun()

        st.divider()
        st.subheader("🚫 障碍物管理")
        if st.session_state.temp_obstacle:
            st.info(f"📐 新障碍物，顶点数: {len(st.session_state.temp_obstacle['coords'])}")
            temp_h = st.number_input(
                "高度 (m)",
                min_value=0.0,
                max_value=200.0,
                value=float(st.session_state.temp_obstacle_height),
                step=5.0
            )
            c_add, c_cancel = st.columns(2)
            with c_add:
                if st.button("✅ 添加", use_container_width=True):
                    new_id = len(st.session_state.obstacles) + 1
                    st.session_state.obstacles.append({
                        "id": new_id,
                        "coords": st.session_state.temp_obstacle["coords"],
                        "height": temp_h,
                        "create_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    })
                    st.session_state.temp_obstacle = None
                    save_obstacles()
                    st.rerun()
            with c_cancel:
                if st.button("❌ 取消", use_container_width=True):
                    st.session_state.temp_obstacle = None
                    st.rerun()

        st.write(f"**📋 障碍物列表 ({len(st.session_state.obstacles)}个)**")
        for i, obs in enumerate(st.session_state.obstacles):
            if 'id' not in obs:
                obs['id'] = i + 1
            c1, c2, c3 = st.columns([0.5, 1.5, 2])
            with c1:
                if st.button("🗑️", key=f"del_{obs['id']}"):
                    st.session_state.obstacles.pop(i)
                    st.session_state.deployed_obstacles = [o for o in st.session_state.deployed_obstacles if o.get('id') != obs['id']]
                    save_obstacles()
                    st.rerun()
            with c2:
                st.write(f"障碍物 {obs['id']}")
            with c3:
                new_h = st.number_input(
                    "m",
                    min_value=0.0,
                    max_value=200.0,
                    value=float(obs.get("height", 30.0)),
                    step=5.0,
                    key=f"h_{obs['id']}",
                    label_visibility="collapsed"
                )
                if new_h != obs.get("height", 30.0):
                    obs["height"] = new_h
                    for d in st.session_state.deployed_obstacles:
                        if d.get('id') == obs['id']:
                            d["height"] = new_h
                    save_obstacles()

        st.divider()
        st.subheader("⚙️ 持久化")
        cs, cl, cc, cd = st.columns(4)
        with cs:
            if st.button("💾 保存", use_container_width=True):
                save_obstacles()
                st.success("已保存")
        with cl:
            if st.button("📂 加载", use_container_width=True):
                load_obstacles()
                st.rerun()
        with cc:
            if st.button("🗑️ 清空", use_container_width=True):
                st.session_state.obstacles = []
                st.session_state.deployed_obstacles = []
                save_obstacles()
                st.rerun()
        with cd:
            if st.button("🚀 部署", use_container_width=True, type="primary"):
                st.session_state.deployed_obstacles = st.session_state.obstacles.copy()
                save_obstacles()
                st.rerun()

with tab2:
    st.header("🛸 飞行实时画面 - 任务执行监控")

    col_ctrl = st.columns(4)
    with col_ctrl[0]:
        if st.button("▶️ 开始任务", type="primary", use_container_width=True):
            if len(st.session_state.flight_path) > 0:
                st.session_state.flight_running = True
                st.session_state.flight_start_time = datetime.now()
                st.session_state.current_waypoint_idx = 0
                st.session_state.flight_pos = st.session_state.flight_path[0]
                comm_log_generate_mission_start(
                    st.session_state.start_point,
                    st.session_state.end_point,
                    st.session_state.flight_height
                )
                comm_log_generate_route_plan(
                    st.session_state.flight_path,
                    algorithm="A*",
                    obstacle_count=len(st.session_state.deployed_obstacles)
                )
                comm_log_ack()
    with col_ctrl[1]:
        if st.button("⏸️ 暂停", use_container_width=True):
            st.session_state.flight_running = False
    with col_ctrl[2]:
        if st.button("⏹️ 停止", use_container_width=True):
            st.session_state.flight_running = False
            st.session_state.current_waypoint_idx = 0
            st.session_state.flight_pos = st.session_state.flight_path[0] if len(st.session_state.flight_path) > 0 else None
    with col_ctrl[3]:
        if st.button("🔄 重置", use_container_width=True):
            st.session_state.flight_running = False
            st.session_state.current_waypoint_idx = 0
            st.session_state.flight_pos = st.session_state.flight_path[0] if len(st.session_state.flight_path) > 0 else None
            st.session_state.flight_start_time = None

    col_status = st.columns(6)
    total_waypoints = len(st.session_state.flight_path) if len(st.session_state.flight_path) > 0 else 0
    current_wp = st.session_state.current_waypoint_idx + 1 if st.session_state.flight_running else 0
    elapsed_time = (datetime.now() - st.session_state.flight_start_time).total_seconds() if st.session_state.flight_start_time else 0
    remaining_distance = 0
    if st.session_state.flight_running and st.session_state.flight_pos and st.session_state.current_waypoint_idx < total_waypoints-1:
        remaining_pts = st.session_state.flight_path[st.session_state.current_waypoint_idx:]
        line = LineString(remaining_pts)
        remaining_distance = line.length * 111139
    eta = remaining_distance / st.session_state.flight_speed if st.session_state.flight_speed > 0 else 0
    battery = max(0, 100 - elapsed_time * 0.1)
    with col_status[0]:
        st.metric("当前航点", f"{current_wp}/{total_waypoints}")
    with col_status[1]:
        st.metric("飞行速度", f"{st.session_state.flight_speed} m/s")
    with col_status[2]:
        st.metric("已用时间", str(timedelta(seconds=int(elapsed_time))))
    with col_status[3]:
        st.metric("剩余距离", f"{remaining_distance:.1f} m")
    with col_status[4]:
        st.metric("预计到达", str(timedelta(seconds=int(eta))))
    with col_status[5]:
        st.metric("电量模拟", f"{battery:.0f}%")

    if total_waypoints > 0:
        progress_val = st.session_state.current_waypoint_idx / max(total_waypoints - 1, 1)
        progress_pct = int(progress_val * 100)
        if not st.session_state.flight_running and st.session_state.current_waypoint_idx == 0:
            st.progress(0, text="⏹ 任务未开始")
        elif progress_pct >= 100:
            st.progress(1.0, text="✅ 任务完成！已到达目标点")
        else:
            st.progress(progress_val, text=f"🛸 飞行中... {progress_pct}%  （第 {st.session_state.current_waypoint_idx}/{total_waypoints - 1} 步）")
    else:
        st.progress(0, text="💡 请先规划航线")

    st.divider()
    col_flight_map, col_comm = st.columns([2, 1])
    with col_flight_map:
        st.subheader("🗺️ 实时飞行地图")
        if len(st.session_state.flight_path) > 0:
            m_flight = folium.Map(
                location=st.session_state.flight_path[0], zoom_start=18,
                tiles="https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
                attr="© 高德地图"
            )
            for obs in st.session_state.deployed_obstacles:
                color = "red" if obs.get("height", 0) > st.session_state.flight_height else "orange"
                folium.Polygon(locations=obs["coords"], color=color, weight=3, fill=True, fill_opacity=0.3).add_to(m_flight)

            display_path = interpolate_for_display(st.session_state.flight_path, steps_per_segment=12)
            folium.PolyLine(locations=display_path, color="green", weight=4, opacity=0.7).add_to(m_flight)

            if st.session_state.current_waypoint_idx > 0:
                flown_semantic = st.session_state.flight_path[:st.session_state.current_waypoint_idx + 1]
                flown_display = interpolate_for_display(flown_semantic, steps_per_segment=12)
                AntPath(locations=flown_display, color="blue", weight=5, delay=1000).add_to(m_flight)

            if st.session_state.flight_pos:
                folium.Marker(
                    location=st.session_state.flight_pos, popup="🛸 当前位置",
                    icon=folium.Icon(color="blue", icon="plane", prefix="fa")
                ).add_to(m_flight)
            st_folium(m_flight, width=600, height=450, key="flight_map")

            if st.session_state.flight_running and st.session_state.current_waypoint_idx < len(st.session_state.flight_path) - 1:
                time.sleep(1.0)
                st.session_state.current_waypoint_idx += 1
                st.session_state.flight_pos = st.session_state.flight_path[st.session_state.current_waypoint_idx]
                comm_log_update_waypoint(
                    st.session_state.current_waypoint_idx,
                    len(st.session_state.flight_path),
                    st.session_state.flight_pos
                )
                st.rerun()
            elif st.session_state.flight_running and st.session_state.current_waypoint_idx >= len(st.session_state.flight_path) - 1:
                st.session_state.flight_running = False
                st.success("✅ 任务完成！已到达目标点")
        else:
            st.info("💡 请先在「地图与障碍物管理」页面规划一条航线，再开始飞行监控")

    with col_comm:
        st.subheader("📡 通信链路拓扑与数据流")
        
        col_status_dev = st.columns(3)
        with col_status_dev[0]:
            status_color = "#4CAF50" if st.session_state.gcs_online else "#f44336"
            st.markdown(f"""
            <div style="text-align: center;">
                <span style="display: inline-block; width: 10px; height: 10px; background-color: {status_color}; border-radius: 50%; margin-right: 5px;"></span>
                <span>GCS 在线</span>
            </div>
            """, unsafe_allow_html=True)
        with col_status_dev[1]:
            status_color = "#4CAF50" if st.session_state.obc_online else "#f44336"
            st.markdown(f"""
            <div style="text-align: center;">
                <span style="display: inline-block; width: 10px; height: 10px; background-color: {status_color}; border-radius: 50%; margin-right: 5px;"></span>
                <span>OBC 在线</span>
            </div>
            """, unsafe_allow_html=True)
        with col_status_dev[2]:
            status_color = "#4CAF50" if st.session_state.fcu_online else "#f44336"
            st.markdown(f"""
            <div style="text-align: center;">
                <span style="display: inline-block; width: 10px; height: 10px; background-color: {status_color}; border-radius: 50%; margin-right: 5px;"></span>
                <span>FCU 在线</span>
            </div>
            """, unsafe_allow_html=True)
        
        st.markdown("---")
        
        col_nodes = st.columns(3)
        with col_nodes[0]:
            st.markdown("""
            <div style="background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 20px 10px; border-radius: 15px; text-align: center; color: white; box-shadow: 0 4px 15px rgba(0,0,0,0.1);">
                <div style="font-size: 2.5em;">🖥️</div>
                <h3 style="margin: 5px 0; color: white;">GCS</h3>
                <p style="margin: 5px 0; opacity: 0.9;">地面站</p>
                <p style="font-size: 0.7em; opacity: 0.7; margin-top: 8px;">192.168.1.100</p>
            </div>
            """, unsafe_allow_html=True)
        with col_nodes[1]:
            st.markdown("""
            <div style="background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%); padding: 20px 10px; border-radius: 15px; text-align: center; color: white; box-shadow: 0 4px 15px rgba(0,0,0,0.1);">
                <div style="font-size: 2.5em;">🧠</div>
                <h3 style="margin: 5px 0; color: white;">OBC</h3>
                <p style="margin: 5px 0; opacity: 0.9;">机载计算机</p>
                <p style="font-size: 0.7em; opacity: 0.7; margin-top: 8px;">Raspberry Pi 4</p>
            </div>
            """, unsafe_allow_html=True)
        with col_nodes[2]:
            st.markdown("""
            <div style="background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%); padding: 20px 10px; border-radius: 15px; text-align: center; color: white; box-shadow: 0 4px 15px rgba(0,0,0,0.1);">
                <div style="font-size: 2.5em;">⚙️</div>
                <h3 style="margin: 5px 0; color: white;">FCU</h3>
                <p style="margin: 5px 0; opacity: 0.9;">飞控</p>
                <p style="font-size: 0.7em; opacity: 0.7; margin-top: 8px;">Pixhawk</p>
            </div>
            """, unsafe_allow_html=True)
        
        st.markdown("""
        <div style="display: flex; justify-content: space-between; padding: 5px 40px; margin: 5px 0;">
            <span style="font-size: 0.8em; color: #666;">⬅️ GCS ↔ OBC ➡️</span>
            <span style="font-size: 0.8em; color: #666;">⬅️ OBC ↔ FCU ➡️</span>
        </div>
        """, unsafe_allow_html=True)
        
        st.markdown("---")
        
        st.markdown("""
        <div style="background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%); padding: 12px; border-radius: 10px; margin: 10px 0;">
            <div style="display: flex; justify-content: space-around; color: white;">
                <div style="text-align: center;">
                    <div style="font-size: 0.8em; opacity: 0.8;">GCS ↔ OBC</div>
                    <div style="font-weight: bold;">✅ 正常</div>
                </div>
                <div style="text-align: center;">
                    <div style="font-size: 0.8em; opacity: 0.8;">OBC ↔ FCU</div>
                    <div style="font-weight: bold;">✅ 正常</div>
                </div>
            </div>
        </div>
        """, unsafe_allow_html=True)
        
        col_stats = st.columns(2)
        with col_stats[0]:
            st.metric("📡 延迟", f"{st.session_state.latency} ms", delta=None)
        with col_stats[1]:
            st.metric("📉 丢包率", f"{st.session_state.packet_loss}%", delta=None)
        
        st.divider()
        st.subheader("📈 飞行状态曲线")
        if st.session_state.flight_running or st.session_state.flight_start_time:
            time_points = np.linspace(0, elapsed_time, 20)
            battery_points = 100 - time_points * 0.1
            speed_points = np.full_like(time_points, st.session_state.flight_speed)
            fig, ax = plt.subplots(figsize=(10, 3))
            ax.plot(time_points, battery_points, label="电量 (%)", color="#f39c12")
            ax.plot(time_points, speed_points, label="速度 (m/s)", color="#3498db")
            ax.set_title("飞行状态曲线")
            ax.set_xlabel("时间 (s)")
            ax.legend()
            ax.grid(True, alpha=0.3)
            st.pyplot(fig)
        st.markdown("---")

        st.subheader("📋 通信日志")
        log_tab1, log_tab2, log_tab3 = st.tabs(["📊 业务流程", "📤 GCS→OBC→FCU", "📥 FCU→OBC→GCS"])

        with log_tab1:
            if not st.session_state.comm_log:
                st.info("💡 点击「▶️ 开始任务」后将显示通信日志")
            else:
                all_logs = st.session_state.comm_log
                with st.container():
                    prev_dir = None
                    for entry in all_logs:
                        dir_ = entry['direction']
                        if dir_ == "GCS->OBC->FCU":
                            bg = "#e8f5e9"
                        elif dir_ == "FCU->OBC->GCS":
                            bg = "#fff8e1"
                        else:
                            bg = "#f3f4f6"
                        group_header = None
                        if dir_ != prev_dir:
                            if dir_ == "GCS->OBC->FCU":
                                group_header = "GCS → OBC"
                            elif dir_ == "FCU->OBC->GCS":
                                group_header = "FCU → OBC"
                            else:
                                group_header = "OBC 内部"
                            prev_dir = dir_
                        if group_header:
                            st.markdown(
                                f"<div style='font-size:0.75em;color:#777;margin:8px 0 2px 0;"
                                f"font-weight:bold;border-left:3px solid #aaa;padding-left:6px;'>"
                                f"🔗 {group_header}</div>",
                                unsafe_allow_html=True
                            )
                        detail_str = f"<br><span style='padding-left:10px;color:#444;'>{entry['detail']}</span>" if entry['detail'] else ""
                        st.markdown(
                            f"<div style='background:{bg};padding:4px 8px;border-radius:6px;"
                            f"margin:2px 0;font-size:0.78em;font-family:monospace;'>"
                            f"<span style='color:#888;'>[{entry['time']}]</span> "
                            f"<b style='color:#1a73e8;'>{entry['event']}</b>"
                            f"{detail_str}</div>",
                            unsafe_allow_html=True
                        )

        with log_tab2:
            gcs_logs = [e for e in st.session_state.comm_log if e['direction'] in ("GCS->OBC->FCU", "OBC_INTERNAL")]
            if not gcs_logs:
                st.info("💡 暂无 GCS→OBC→FCU 方向的日志")
            else:
                gcs_to_obc = [e for e in gcs_logs if e['from'] == 'GCS']
                if gcs_to_obc:
                    st.markdown(
                        "<div style='font-size:0.8em;font-weight:bold;color:#1a73e8;"
                        "margin:4px 0;'>🖥️ GCS → OBC</div>",
                        unsafe_allow_html=True
                    )
                    for entry in gcs_to_obc:
                        detail_str = f"<br><span style='padding-left:10px;color:#444;'>{entry['detail']}</span>" if entry['detail'] else ""
                        st.markdown(
                            f"<div style='background:#e8f5e9;padding:4px 8px;border-radius:6px;"
                            f"margin:2px 0;font-size:0.78em;font-family:monospace;'>"
                            f"<span style='color:#888;'>[{entry['time']}]</span> "
                            f"<b>GCS→OBC→FCU:</b> "
                            f"<span style='color:#1a73e8;'>{entry['event']}</span>"
                            f"{detail_str}</div>",
                            unsafe_allow_html=True
                        )
                obc_internal = [e for e in gcs_logs if e['direction'] == 'OBC_INTERNAL']
                if obc_internal:
                    st.markdown(
                        "<div style='font-size:0.8em;font-weight:bold;color:#e65100;"
                        "margin:8px 0 4px 0;'>🧠 OBC 内部</div>",
                        unsafe_allow_html=True
                    )
                    for entry in obc_internal:
                        detail_str = f"<br><span style='padding-left:10px;color:#444;'>{entry['detail']}</span>" if entry['detail'] else ""
                        st.markdown(
                            f"<div style='background:#fff3e0;padding:4px 8px;border-radius:6px;"
                            f"margin:2px 0;font-size:0.78em;font-family:monospace;'>"
                            f"<span style='color:#888;'>[{entry['time']}]</span> "
                            f"<b>🔵 {entry['event']}</b>"
                            f"{detail_str}</div>",
                            unsafe_allow_html=True
                        )

        with log_tab3:
            fcu_logs = [e for e in st.session_state.comm_log if e['direction'] == "FCU->OBC->GCS"]
            if not fcu_logs:
                st.info("💡 暂无 FCU→OBC→GCS 方向的日志")
            else:
                fcu_to_obc = [e for e in fcu_logs if e['from'] == 'FCU']
                if fcu_to_obc:
                    st.markdown(
                        "<div style='font-size:0.8em;font-weight:bold;color:#c62828;"
                        "margin:4px 0;'>⚙️ FCU → OBC</div>",
                        unsafe_allow_html=True
                    )
                    for entry in fcu_to_obc:
                        st.markdown(
                            f"<div style='background:#fce4ec;padding:4px 8px;border-radius:6px;"
                            f"margin:2px 0;font-size:0.78em;font-family:monospace;'>"
                            f"<span style='color:#888;'>[{entry['time']}]</span> "
                            f"FCU→OBC→GCS: <b>{entry['event']}</b></div>",
                            unsafe_allow_html=True
                        )
                obc_to_gcs = [e for e in fcu_logs if e['from'] == 'OBC']
                if obc_to_gcs:
                    st.markdown(
                        "<div style='font-size:0.8em;font-weight:bold;color:#1565c0;"
                        "margin:8px 0 4px 0;'>🧠 OBC → GCS</div>",
                        unsafe_allow_html=True
                    )
                    for entry in obc_to_gcs:
                        st.markdown(
                            f"<div style='background:#e3f2fd;padding:4px 8px;border-radius:6px;"
                            f"margin:2px 0;font-size:0.78em;font-family:monospace;'>"
                            f"<span style='color:#888;'>[{entry['time']}]</span> "
                            f"FCU→OBC→GCS: <b>{entry['event']}</b></div>",
                            unsafe_allow_html=True
                        )

        st.markdown("---")
        st.caption("🚁 无人机航线规划系统 v24.0 | P2 通信日志模块已添加")
