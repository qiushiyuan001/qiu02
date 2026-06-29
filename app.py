import streamlit as st
import folium
from streamlit_folium import folium_static
import json
import math
import time
import random
import pandas as pd
from datetime import datetime

st.set_page_config(page_title="无人机地面控制站", layout="wide")

PI = math.pi
AXIS = 6378245.0
OFFSET = 0.00669342162296594323

def transform_lat(x, y):
    ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(y * PI) + 40.0 * math.sin(y / 3.0 * PI)) * 2.0 / 3.0
    ret += (160.0 * math.sin(y / 12.0 * PI) + 320 * math.sin(y * PI / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(x, y):
    ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    ret += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    ret += (20.0 * math.sin(x * PI) + 40.0 * math.sin(x / 3.0 * PI)) * 2.0 / 3.0
    ret += (150.0 * math.sin(x / 12.0 * PI) + 300.0 * math.sin(x / 30.0 * PI)) * 2.0 / 3.0
    return ret

def out_of_china(lng, lat):
    return not (73.66 <= lng <= 135.05 and 3.86 <= lat <= 53.55)

def wgs84_to_gcj02(lng, lat):
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - OFFSET * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((AXIS * (1 - OFFSET)) / (magic * sqrtmagic) * PI)
    dlng = (dlng * 180.0) / (AXIS / sqrtmagic * math.cos(radlat) * PI)
    mglat = lat + dlat
    mglng = lng + dlng
    return mglng, mglat

def gcj02_to_wgs84(lng, lat):
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * PI
    magic = math.sin(radlat)
    magic = 1 - OFFSET * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((AXIS * (1 - OFFSET)) / (magic * sqrtmagic) * PI)
    dlng = (dlng * 180.0) / (AXIS / sqrtmagic * math.cos(radlat) * PI)
    mglat = lat + dlat
    mglng = lng + dlng
    return lng * 2 - mglng, lat * 2 - mglat

def gcj02_to_bd09(lng, lat):
    z = math.sqrt(lng * lng + lat * lat) + 0.00002 * math.sin(lat * PI * 3000.0 / 180.0)
    theta = math.atan2(lat, lng) + 0.000003 * math.cos(lng * PI * 3000.0 / 180.0)
    bd_lng = z * math.cos(theta) + 0.0065
    bd_lat = z * math.sin(theta) + 0.006
    return bd_lng, bd_lat

def bd09_to_gcj02(lng, lat):
    x = lng - 0.0065
    y = lat - 0.006
    z = math.sqrt(x * x + y * y) - 0.00002 * math.sin(y * PI * 3000.0 / 180.0)
    theta = math.atan2(y, x) - 0.000003 * math.cos(x * PI * 3000.0 / 180.0)
    gcj_lng = z * math.cos(theta)
    gcj_lat = z * math.sin(theta)
    return gcj_lng, gcj_lat

def wgs84_to_bd09(lng, lat):
    gcj_lng, gcj_lat = wgs84_to_gcj02(lng, lat)
    return gcj02_to_bd09(gcj_lng, gcj_lat)

def bd09_to_wgs84(lng, lat):
    gcj_lng, gcj_lat = bd09_to_gcj02(lng, lat)
    return gcj02_to_wgs84(gcj_lng, gcj_lat)

def calculate_distance(lat1, lng1, lat2, lng2):
    R = 6371000
    dlat = math.radians(lat2 - lat1)
    dlng = math.radians(lng2 - lng1)
    a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlng/2) * math.sin(dlng/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def generate_path_overfly(start, end, obstacles, fly_height=50):
    path = [(start['lat'], start['lng']), (end['lat'], end['lng'])]
    return path, '飞越'

def generate_path_around(start, end, obstacles, fly_height=50, safe_radius=30):
    path = [(start['lat'], start['lng'])]
    mid_lat = (start['lat'] + end['lat']) / 2
    mid_lng = (start['lng'] + end['lng']) / 2
    offset_lat = (end['lng'] - start['lng']) * 0.0001
    offset_lng = -(end['lat'] - start['lat']) * 0.0001
    path.append((mid_lat + offset_lat, mid_lng + offset_lng))
    path.append((end['lat'], end['lng']))
    return path, '绕飞'

def init_session_state():
    if 'obstacles' not in st.session_state:
        st.session_state.obstacles = [
            {'id': 1, 'name': '障碍物1', 'points': [(32.236479, 118.743786), (32.236779, 118.743786), (32.236779, 118.744086), (32.236479, 118.744086)], 'height': 30, 'color': '#e74c3c'},
            {'id': 2, 'name': '障碍物2', 'points': [(32.235979, 118.744286), (32.236279, 118.744286), (32.236279, 118.744586), (32.235979, 118.744586)], 'height': 25, 'color': '#f39c12'},
            {'id': 3, 'name': '障碍物3', 'points': [(32.235779, 118.743586), (32.236079, 118.743586), (32.236079, 118.743886), (32.235779, 118.743886)], 'height': 40, 'color': '#9b59b6'},
        ]
    if 'start_point' not in st.session_state:
        st.session_state.start_point = {'lat': 32.236179, 'lng': 118.744186}
    if 'end_point' not in st.session_state:
        st.session_state.end_point = {'lat': 32.235479, 'lng': 118.744786}
    if 'fly_height' not in st.session_state:
        st.session_state.fly_height = 50
    if 'safe_radius' not in st.session_state:
        st.session_state.safe_radius = 30
    if 'planned_path' not in st.session_state:
        st.session_state.planned_path = []
    if 'is_flying' not in st.session_state:
        st.session_state.is_flying = False
    if 'flight_data' not in st.session_state:
        st.session_state.flight_data = {
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'alt': 0.0, 'speed': 0.0, 'dist': 0.0,
            'lat': 32.236179, 'lng': 118.744186,
            'voltage': 24.0, 'current': 10.0, 'power': 100,
            'status': '待机'
        }
    if 'flight_logs' not in st.session_state:
        st.session_state.flight_logs = []
    if 'mavlink_messages' not in st.session_state:
        st.session_state.mavlink_messages = []
    if 'path_type' not in st.session_state:
        st.session_state.path_type = '飞越'
    if 'coord_input_mode' not in st.session_state:
        st.session_state.coord_input_mode = 'WGS-84'

init_session_state()

def add_log(message):
    st.session_state.flight_logs.insert(0, {
        'time': datetime.now().strftime('%H:%M:%S'),
        'message': message
    })
    if len(st.session_state.flight_logs) > 50:
        st.session_state.flight_logs.pop()

def add_mavlink_message(msg_type, content):
    st.session_state.mavlink_messages.insert(0, {
        'time': datetime.now().strftime('%H:%M:%S'),
        'type': msg_type,
        'content': content
    })
    if len(st.session_state.mavlink_messages) > 30:
        st.session_state.mavlink_messages.pop()

def update_flight_data():
    if not st.session_state.is_flying:
        return
    
    fd = st.session_state.flight_data
    
    if fd['alt'] < st.session_state.fly_height:
        fd['alt'] = min(fd['alt'] + 0.5, st.session_state.fly_height)
        fd['speed'] = 2.0
        fd['status'] = '爬升中'
    else:
        has_path = st.session_state.planned_path and len(st.session_state.planned_path) > 1
        
        if has_path:
            fd['speed'] = 10.0
            fd['status'] = '飞行中'
            
            current_idx = 0
            if 'path_index' in st.session_state:
                current_idx = st.session_state.path_index
            
            target_lat, target_lng = st.session_state.planned_path[current_idx]
            dist_to_target = calculate_distance(fd['lat'], fd['lng'], target_lat, target_lng)
            
            if dist_to_target < 5:
                if current_idx < len(st.session_state.planned_path) - 1:
                    st.session_state.path_index = current_idx + 1
                    target_lat, target_lng = st.session_state.planned_path[current_idx + 1]
                    add_log(f'📍 经过航点 {current_idx + 1}')
                else:
                    fd['speed'] = 0.0
                    st.session_state.is_flying = False
                    fd['status'] = '已到达'
                    add_log('✈️ 到达目的地')
                    add_mavlink_message('MISSION', 'Mission complete')
                    return
            
            step_size = 0.000045
            direction_lat = (target_lat - fd['lat']) / max(dist_to_target, 0.1) * step_size
            direction_lng = (target_lng - fd['lng']) / max(dist_to_target, 0.1) * step_size
            fd['lat'] += direction_lat
            fd['lng'] += direction_lng
            fd['dist'] += calculate_distance(
                fd['lat'] - direction_lat, fd['lng'] - direction_lng,
                fd['lat'], fd['lng']
            )
        else:
            fd['speed'] = 0.0
            fd['status'] = '悬停'
    
    fd['roll'] = random.uniform(-2, 2)
    fd['pitch'] = random.uniform(-1, 1)
    fd['yaw'] = (fd['yaw'] + 0.5) % 360
    fd['voltage'] = max(18.0, fd['voltage'] - 0.01)
    fd['current'] = random.uniform(8, 12)
    fd['power'] = max(0, fd['power'] - 0.1)
    
    add_mavlink_message('GPS', f"Lat: {fd['lat']:.6f}, Lng: {fd['lng']:.6f}, Alt: {fd['alt']:.1f}m")
    add_mavlink_message('ATTITUDE', f"Roll: {fd['roll']:.1f}°, Pitch: {fd['pitch']:.1f}°, Yaw: {fd['yaw']:.1f}°")
    add_mavlink_message('BATTERY', f"Voltage: {fd['voltage']:.1f}V, Current: {fd['current']:.1f}A")

tab1, tab2, tab3, tab4 = st.tabs(['📍 地图定位', '🚧 障碍物与航线规划', '🛫 飞行监控', '🔗 通信链路'])

with tab1:
    col1, col2 = st.columns([3, 1])
    
    with col2:
        st.subheader('坐标转换')
        
        wgs_lng = st.number_input('WGS-84 经度', value=118.744186, step=0.0001, format='%.6f')
        wgs_lat = st.number_input('WGS-84 纬度', value=32.236179, step=0.0001, format='%.6f')
        
        gcj_lng, gcj_lat = wgs84_to_gcj02(wgs_lng, wgs_lat)
        bd_lng, bd_lat = wgs84_to_bd09(wgs_lng, wgs_lat)
        
        st.write(f'**WGS-84 (GPS坐标):**')
        st.write(f'经度: {wgs_lng:.6f}')
        st.write(f'纬度: {wgs_lat:.6f}')
        
        st.write(f'**GCJ-02 (火星坐标):**')
        st.write(f'经度: {gcj_lng:.6f}')
        st.write(f'纬度: {gcj_lat:.6f}')
        
        st.write(f'**BD-09 (百度坐标):**')
        st.write(f'经度: {bd_lng:.6f}')
        st.write(f'纬度: {bd_lat:.6f}')
        
        st.subheader('地图信息')
        st.write(f'南京科技职业学院')
        st.write(f'校园正门（高德GCJ-02）')
        st.write(f'经度: 118°44′58″ E')
        st.write(f'纬度: 32°14′03″ N')
        st.write(f'坐标系: WGS-84')
    
    with col1:
        m = folium.Map(location=[wgs_lat, wgs_lng], zoom_start=18, tiles='CartoDB positron', attr='CartoDB', max_zoom=19)
        
        folium.Marker(
            [wgs_lat, wgs_lng],
            popup=f'WGS-84: {wgs_lat:.6f}, {wgs_lng:.6f}\nGCJ-02: {gcj_lat:.6f}, {gcj_lng:.6f}',
            icon=folium.Icon(color='blue', icon='info-sign')
        ).add_to(m)
        
        st.write(f'地图中心点 (WGS-84): {wgs_lat:.6f}, {wgs_lng:.6f}')
        folium_static(m)

with tab2:
    col1, col2 = st.columns([3, 1])
    
    with col2:
        st.subheader('参数设置')
        st.session_state.fly_height = st.number_input('飞行高度 (m)', value=50, min_value=10, max_value=200)
        st.session_state.safe_radius = st.number_input('安全半径 (m)', value=30, min_value=5, max_value=100)
        st.session_state.path_type = st.radio('路径规划方式', ['飞越', '绕飞'])
        
        st.subheader('起点/终点设置')
        
        input_mode = st.radio('坐标输入模式', ['WGS-84 (GPS)', '高德GCJ-02'], horizontal=True, key='input_mode')
        
        start_lat = st.session_state.start_point['lat']
        start_lng = st.session_state.start_point['lng']
        end_lat = st.session_state.end_point['lat']
        end_lng = st.session_state.end_point['lng']
        
        if input_mode == '高德GCJ-02':
            start_lng_disp, start_lat_disp = wgs84_to_gcj02(start_lng, start_lat)
            end_lng_disp, end_lat_disp = wgs84_to_gcj02(end_lng, end_lat)
        else:
            start_lat_disp, start_lng_disp = start_lat, start_lng
            end_lat_disp, end_lng_disp = end_lat, end_lng
        
        st.write('**起点**')
        col_s1, col_s2 = st.columns(2)
        with col_s1:
            new_s_lat = st.number_input('纬度', value=start_lat_disp, step=0.0001, format='%.6f', key='start_lat_input')
        with col_s2:
            new_s_lng = st.number_input('经度', value=start_lng_disp, step=0.0001, format='%.6f', key='start_lng_input')
        
        step_size = st.select_slider('微调步长', options=[1, 5, 10, 20, 50], value=5, format_func=lambda x: f'{x}米', key='step_size')
        step_deg = step_size / 111000.0
        
        col_up, col_down = st.columns(2)
        with col_up:
            if st.button('⬆️ 北移', use_container_width=True, key='start_north'):
                if input_mode == '高德GCJ-02':
                    wgs_s_lng, wgs_s_lat = gcj02_to_wgs84(new_s_lng, new_s_lat)
                else:
                    wgs_s_lng, wgs_s_lat = new_s_lng, new_s_lat
                st.session_state.start_point['lat'] = wgs_s_lat + step_deg
                st.session_state.start_point['lng'] = wgs_s_lng
                st.rerun()
            if st.button('⬇️ 南移', use_container_width=True, key='start_south'):
                if input_mode == '高德GCJ-02':
                    wgs_s_lng, wgs_s_lat = gcj02_to_wgs84(new_s_lng, new_s_lat)
                else:
                    wgs_s_lng, wgs_s_lat = new_s_lng, new_s_lat
                st.session_state.start_point['lat'] = wgs_s_lat - step_deg
                st.session_state.start_point['lng'] = wgs_s_lng
                st.rerun()
        
        with col_down:
            if st.button('➡️ 东移', use_container_width=True, key='start_east'):
                if input_mode == '高德GCJ-02':
                    wgs_s_lng, wgs_s_lat = gcj02_to_wgs84(new_s_lng, new_s_lat)
                else:
                    wgs_s_lng, wgs_s_lat = new_s_lng, new_s_lat
                lat_rad = math.radians(wgs_s_lat)
                lng_step = step_deg / math.cos(lat_rad)
                st.session_state.start_point['lat'] = wgs_s_lat
                st.session_state.start_point['lng'] = wgs_s_lng + lng_step
                st.rerun()
            if st.button('⬅️ 西移', use_container_width=True, key='start_west'):
                if input_mode == '高德GCJ-02':
                    wgs_s_lng, wgs_s_lat = gcj02_to_wgs84(new_s_lng, new_s_lat)
                else:
                    wgs_s_lng, wgs_s_lat = new_s_lng, new_s_lat
                lat_rad = math.radians(wgs_s_lat)
                lng_step = step_deg / math.cos(lat_rad)
                st.session_state.start_point['lat'] = wgs_s_lat
                st.session_state.start_point['lng'] = wgs_s_lng - lng_step
                st.rerun()
        
        if input_mode == '高德GCJ-02':
            wgs_s_lng, wgs_s_lat = gcj02_to_wgs84(new_s_lng, new_s_lat)
            st.session_state.start_point['lat'] = wgs_s_lat
            st.session_state.start_point['lng'] = wgs_s_lng
        else:
            st.session_state.start_point['lat'] = new_s_lat
            st.session_state.start_point['lng'] = new_s_lng
        
        st.write('**终点**')
        col_e1, col_e2 = st.columns(2)
        with col_e1:
            new_e_lat = st.number_input('纬度', value=end_lat_disp, step=0.0001, format='%.6f', key='end_lat_input')
        with col_e2:
            new_e_lng = st.number_input('经度', value=end_lng_disp, step=0.0001, format='%.6f', key='end_lng_input')
        
        col_eu, col_ed = st.columns(2)
        with col_eu:
            if st.button('⬆️ 北移', use_container_width=True, key='end_north'):
                if input_mode == '高德GCJ-02':
                    wgs_e_lng, wgs_e_lat = gcj02_to_wgs84(new_e_lng, new_e_lat)
                else:
                    wgs_e_lng, wgs_e_lat = new_e_lng, new_e_lat
                st.session_state.end_point['lat'] = wgs_e_lat + step_deg
                st.session_state.end_point['lng'] = wgs_e_lng
                st.rerun()
            if st.button('⬇️ 南移', use_container_width=True, key='end_south'):
                if input_mode == '高德GCJ-02':
                    wgs_e_lng, wgs_e_lat = gcj02_to_wgs84(new_e_lng, new_e_lat)
                else:
                    wgs_e_lng, wgs_e_lat = new_e_lng, new_e_lat
                st.session_state.end_point['lat'] = wgs_e_lat - step_deg
                st.session_state.end_point['lng'] = wgs_e_lng
                st.rerun()
        
        with col_ed:
            if st.button('➡️ 东移', use_container_width=True, key='end_east'):
                if input_mode == '高德GCJ-02':
                    wgs_e_lng, wgs_e_lat = gcj02_to_wgs84(new_e_lng, new_e_lat)
                else:
                    wgs_e_lng, wgs_e_lat = new_e_lng, new_e_lat
                lat_rad = math.radians(wgs_e_lat)
                lng_step = step_deg / math.cos(lat_rad)
                st.session_state.end_point['lat'] = wgs_e_lat
                st.session_state.end_point['lng'] = wgs_e_lng + lng_step
                st.rerun()
            if st.button('⬅️ 西移', use_container_width=True, key='end_west'):
                if input_mode == '高德GCJ-02':
                    wgs_e_lng, wgs_e_lat = gcj02_to_wgs84(new_e_lng, new_e_lat)
                else:
                    wgs_e_lng, wgs_e_lat = new_e_lng, new_e_lat
                lat_rad = math.radians(wgs_e_lat)
                lng_step = step_deg / math.cos(lat_rad)
                st.session_state.end_point['lat'] = wgs_e_lat
                st.session_state.end_point['lng'] = wgs_e_lng - lng_step
                st.rerun()
        
        if input_mode == '高德GCJ-02':
            wgs_e_lng, wgs_e_lat = gcj02_to_wgs84(new_e_lng, new_e_lat)
            st.session_state.end_point['lat'] = wgs_e_lat
            st.session_state.end_point['lng'] = wgs_e_lng
        else:
            st.session_state.end_point['lat'] = new_e_lat
            st.session_state.end_point['lng'] = new_e_lng
        
        dist_se = calculate_distance(
            st.session_state.start_point['lat'], st.session_state.start_point['lng'],
            st.session_state.end_point['lat'], st.session_state.end_point['lng']
        )
        st.info(f'📏 起点到终点距离: {dist_se:.1f} 米')
        
        if st.button('📐 生成路径', type='primary', use_container_width=True):
            if st.session_state.path_type == '飞越':
                path, type_name = generate_path_overfly(
                    st.session_state.start_point,
                    st.session_state.end_point,
                    st.session_state.obstacles,
                    st.session_state.fly_height
                )
            else:
                path, type_name = generate_path_around(
                    st.session_state.start_point,
                    st.session_state.end_point,
                    st.session_state.obstacles,
                    st.session_state.fly_height,
                    st.session_state.safe_radius
                )
            st.session_state.planned_path = path
            st.session_state.path_type = type_name
            st.success(f'路径规划完成: {type_name}模式')
            add_log(f'📐 生成{type_name}路径')
        
        st.subheader('障碍物管理')
        st.write(f'当前障碍物数量: {len(st.session_state.obstacles)}')
        for obs in st.session_state.obstacles:
            with st.expander(f"{obs['name']} (高度: {obs['height']}m)"):
                st.write(f"位置: {obs['points'][0]}")
                if st.button(f'删除 {obs["name"]}', key=f'del_obs_{obs["id"]}'):
                    st.session_state.obstacles = [o for o in st.session_state.obstacles if o['id'] != obs['id']]
                    st.rerun()
        
        json_data = json.dumps(st.session_state.obstacles, ensure_ascii=False, indent=2)
        st.download_button('📥 导出障碍物JSON', json_data, file_name='obstacles.json', mime='application/json')
    
    with col1:
        center_lat = st.session_state.start_point['lat']
        center_lng = st.session_state.start_point['lng']
        m = folium.Map(location=[center_lat, center_lng], zoom_start=18, tiles='CartoDB positron', attr='CartoDB', max_zoom=19)
        
        for obs in st.session_state.obstacles:
            folium.Polygon(
                locations=obs['points'],
                color=obs['color'],
                fill=True,
                fill_color=obs['color'],
                fill_opacity=0.5,
                popup=f"{obs['name']} - {obs['height']}m"
            ).add_to(m)
        
        folium.Marker(
            [st.session_state.start_point['lat'], st.session_state.start_point['lng']],
            popup='起点',
            icon=folium.Icon(color='green', icon='play')
        ).add_to(m)
        folium.Marker(
            [st.session_state.end_point['lat'], st.session_state.end_point['lng']],
            popup='终点',
            icon=folium.Icon(color='red', icon='flag')
        ).add_to(m)
        
        if st.session_state.planned_path:
            folium.PolyLine(
                locations=st.session_state.planned_path,
                color='#3498db',
                weight=3,
                popup=f'{st.session_state.path_type}路径'
            ).add_to(m)
            
            total_dist = sum(
                calculate_distance(
                    st.session_state.planned_path[i][0],
                    st.session_state.planned_path[i][1],
                    st.session_state.planned_path[i+1][0],
                    st.session_state.planned_path[i+1][1]
                ) for i in range(len(st.session_state.planned_path)-1)
            )
            st.write(f'**规划路径距离:** {total_dist:.1f}米')
        
        folium_static(m)

with tab3:
    col1, col2 = st.columns([2, 1])
    
    with col1:
        st.subheader('实时飞行地图')
        fd = st.session_state.flight_data
        m = folium.Map(location=[fd['lat'], fd['lng']], zoom_start=19, tiles='CartoDB positron', attr='CartoDB', max_zoom=19)
        
        if st.session_state.planned_path:
            folium.PolyLine(
                locations=st.session_state.planned_path,
                color='#3498db',
                weight=2,
                opacity=0.6,
                popup='规划路径'
            ).add_to(m)
        
        folium.Marker(
            [fd['lat'], fd['lng']],
            popup=f"高度: {fd['alt']:.1f}m\n速度: {fd['speed']:.1f}m/s\n状态: {fd['status']}",
            icon=folium.Icon(color='orange', icon='plane')
        ).add_to(m)
        
        folium.Circle(
            location=[fd['lat'], fd['lng']],
            radius=st.session_state.safe_radius,
            color='orange',
            fill=True,
            fill_opacity=0.1,
            popup=f'安全半径: {st.session_state.safe_radius}m'
        ).add_to(m)
        
        folium_static(m)
        
        col_a, col_b = st.columns(2)
        
        with col_a:
            st.subheader('姿态数据')
            roll = fd['roll']
            pitch = fd['pitch']
            yaw = fd['yaw']
            
            st.metric('横滚 (Roll)', f'{roll:.1f}°')
            st.metric('俯仰 (Pitch)', f'{pitch:.1f}°')
            st.metric('航向 (Yaw)', f'{yaw:.1f}°')
        
        with col_b:
            st.subheader('动力系统')
            voltage = fd['voltage']
            current = fd['current']
            power = fd['power']
            
            st.metric('电压', f'{voltage:.1f}V')
            st.metric('电流', f'{current:.1f}A')
            st.metric('电量', f'{power:.0f}%')
        
        st.subheader('位置与高度')
        col_pos1, col_pos2, col_pos3 = st.columns(3)
        
        with col_pos1:
            st.metric('纬度', f"{fd['lat']:.6f}")
        
        with col_pos2:
            st.metric('经度', f"{fd['lng']:.6f}")
        
        with col_pos3:
            st.metric('高度', f"{fd['alt']:.1f}m")
        
        st.subheader('飞行状态')
        col_status1, col_status2, col_status3 = st.columns(3)
        
        with col_status1:
            st.metric('飞行速度', f"{fd['speed']:.1f}m/s")
        
        with col_status2:
            st.metric('飞行距离', f"{fd['dist']:.1f}m")
        
        with col_status3:
            status_color = 'green' if fd['status'] == '飞行中' else 'blue'
            st.metric('状态', fd['status'], delta_color=status_color)
    
    with col2:
        st.subheader('飞行控制')
        col_btns = st.columns(2)
        
        with col_btns[0]:
            if st.button('▶ 启动', disabled=st.session_state.is_flying, use_container_width=True):
                st.session_state.is_flying = True
                st.session_state.flight_data['status'] = '飞行中'
                st.session_state.flight_data['path_index'] = 0
                add_log('▶ 飞行启动')
                st.rerun()
        
        with col_btns[1]:
            if st.button('⏸ 暂停', disabled=not st.session_state.is_flying, use_container_width=True):
                st.session_state.is_flying = False
                st.session_state.flight_data['status'] = '暂停'
                add_log('⏸ 飞行暂停')
                st.rerun()
        
        if st.button('🔄 重置飞行', use_container_width=True):
            st.session_state.is_flying = False
            st.session_state.flight_data = {
                'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                'alt': 0.0, 'speed': 0.0, 'dist': 0.0,
                'lat': st.session_state.start_point['lat'],
                'lng': st.session_state.start_point['lng'],
                'voltage': 24.0, 'current': 10.0, 'power': 100,
                'status': '待机'
            }
            st.session_state.path_index = 0
            add_log('🔄 飞行重置')
            st.rerun()
        
        st.subheader('飞行日志')
        log_container = st.container()
        
        with log_container:
            if not st.session_state.flight_logs:
                st.write('暂无日志')
            else:
                for log in st.session_state.flight_logs[:10]:
                    st.write(f"[{log['time']}] {log['message']}")

with tab4:
    col1, col2 = st.columns([1, 2])
    
    with col1:
        st.subheader('系统拓扑图')
        
        svg_html = '''
        <svg width="200" height="300" viewBox="0 0 200 300">
            <rect x="20" y="20" width="160" height="80" rx="10" fill="#3498db" stroke="#2980b9" stroke-width="2"/>
            <text x="100" y="65" text-anchor="middle" fill="white" font-size="14" font-weight="bold">GCS</text>
            <text x="100" y="85" text-anchor="middle" fill="white" font-size="10">地面控制站</text>
            
            <line x1="100" y1="100" x2="100" y2="140" stroke="#7f8c8d" stroke-width="2" stroke-dasharray="5,5"/>
            <rect x="155" y="110" width="20" height="20" rx="3" fill="#27ae60"/>
            <text x="165" y="125" text-anchor="middle" fill="white" font-size="8">4G</text>
            
            <rect x="20" y="160" width="160" height="80" rx="10" fill="#e67e22" stroke="#d35400" stroke-width="2"/>
            <text x="100" y="205" text-anchor="middle" fill="white" font-size="14" font-weight="bold">OBC</text>
            <text x="100" y="225" text-anchor="middle" fill="white" font-size="10">机载计算机</text>
            
            <line x1="100" y1="240" x2="100" y2="280" stroke="#7f8c8d" stroke-width="2"/>
            
            <rect x="20" y="280" width="160" height="60" rx="10" fill="#9b59b6" stroke="#8e44ad" stroke-width="2"/>
            <text x="100" y="315" text-anchor="middle" fill="white" font-size="14" font-weight="bold">FCU</text>
            <text x="100" y="335" text-anchor="middle" fill="white" font-size="10">飞控单元</text>
            
            <circle cx="60" cy="310" r="8" fill="#e74c3c"/>
            <text x="60" y="313" text-anchor="middle" fill="white" font-size="7">IMU</text>
            <circle cx="140" cy="310" r="8" fill="#3498db"/>
            <text x="140" y="313" text-anchor="middle" fill="white" font-size="7">GPS</text>
        </svg>
        '''
        st.write(svg_html, unsafe_allow_html=True)
        
        st.subheader('链路状态')
        st.write('✅ GCS ↔ OBC: 正常')
        st.write('✅ OBC ↔ FCU: 正常')
        st.write('📡 信号强度: -58 dBm')
        st.write('📶 丢包率: 0.2%')
        st.write('⚡ 延迟: 12ms')
    
    with col2:
        st.subheader('MAVLink 数据流')
        
        mavlink_container = st.container()
        
        with mavlink_container:
            if not st.session_state.mavlink_messages:
                st.write('暂无报文数据，启动飞行后将显示MAVLink数据流')
            else:
                for msg in st.session_state.mavlink_messages[:20]:
                    color = '#27ae60' if msg['type'] == 'GPS' else '#3498db' if msg['type'] == 'ATTITUDE' else '#e67e22'
                    st.markdown(f"<span style='color:{color}'>[{msg['time']}] **{msg['type']}**:</span> {msg['content']}", unsafe_allow_html=True)

if st.session_state.is_flying:
    update_flight_data()
    time.sleep(0.1)
    st.rerun()
