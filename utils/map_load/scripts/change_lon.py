#!/usr/bin/env python
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-12-19 22:10:22
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-12-19 22:23:15
FilePath: /undefined/home/cyun/Documents/panel_ws/src/map_load/scripts/change_lon.py
Description: 

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''

import rospy
import xml.etree.ElementTree as ET
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# 解析 OSM 文件
def parse_osm(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    
    nodes = {}
    ways = []
    relations = []

    # 解析节点
    for node in root.findall('node'):
        lat = float(node.attrib['lat'])
        lon = float(node.attrib['lon'])
        ele = float(node.find('tag[@k="ele"]').attrib['v']) if node.find('tag[@k="ele"]') is not None else 0.0
        nodes[node.attrib['id']] = (lat, lon, ele)

    # 解析路径
    for way in root.findall('way'):
        nd_refs = [nd.attrib['ref'] for nd in way.findall('nd')]
        tags = {tag.attrib['k']: tag.attrib['v'] for tag in way.findall('tag')}
        # 添加 id 到标签中，如果不存在则使用方式的 id 属性
        tags['id'] = way.attrib.get('id', str(len(ways)))
        ways.append({'nd_refs': nd_refs, 'tags': tags})

    # 解析关系
    for relation in root.findall('relation'):
        members = [{'type': member.attrib['type'], 'ref': member.attrib['ref'], 'role': member.attrib.get('role', '')} 
                   for member in relation.findall('member')]
        tags = {tag.attrib['k']: tag.attrib['v'] for tag in relation.findall('tag')}
        relations.append({'members': members, 'tags': tags})
    
    return nodes, ways, relations

# 创建节点标记
def create_node_marker(id, position, color=(1.0, 0.0, 0.0), scale=0.5):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "osm_nodes"
    marker.id = int(id)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position = Point(position[1], position[0], position[2])
    
    # 显式初始化四元数为单位四元数
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    return marker


# 创建路径标记
def create_way_marker(way, nodes, id_offset=0, color=(0.0, 1.0, 0.0), scale=0.1):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "osm_ways"
    marker.id = id_offset
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = scale
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    # 显式初始化四元数为单位四元数
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    points = []
    for ref in way['nd_refs']:
        if ref in nodes:
            lat, lon, ele = nodes[ref]
            point = Point(lon, lat, ele)
            points.append(point)

    # 确保有足够的点来形成 LINE_STRIP
    if len(points) < 2:
        rospy.logwarn_once(f"Way {way['tags'].get('id', 'unknown')} has fewer than 2 points and will not be visualized.")
        return None  # 返回 None 表示不创建此 Marker

    marker.points = points
    return marker

# 创建关系标记（示例：使用箭头表示转向方向）
def create_relation_marker(relation, ways, nodes, id_offset=0, color=(0.0, 0.0, 1.0)):
    markers = []
    id_counter = id_offset

    for member in relation['members']:
        if member['type'] == 'way':
            for way in ways:
                if way['tags'].get('id') == member['ref']:
                    marker = create_way_marker(way, nodes, id_counter, color)
                    markers.append(marker)
                    id_counter += 1
                    break

    # 添加文本标记表示关系属性（例如速度限制、单行道等）
    text_marker = Marker()
    text_marker.header.frame_id = "map"
    text_marker.header.stamp = rospy.Time.now()
    text_marker.ns = "osm_relations_text"
    text_marker.id = id_counter
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.scale.z = 1.0
    text_marker.color.a = 1.0
    text_marker.color.r = color[0]
    text_marker.color.g = color[1]
    text_marker.color.b = color[2]
    text_marker.text = f"Speed Limit: {relation['tags'].get('speed_limit', 'N/A')} km/h\nOne Way: {'Yes' if relation['tags'].get('one_way') == 'yes' else 'No'}"

    # 设置文本位置（这里简单地放在第一个节点的位置上）
    first_node_ref = ways[0]['nd_refs'][0] if ways else None
    if first_node_ref and first_node_ref in nodes:
        lat, lon, ele = nodes[first_node_ref]
        text_marker.pose.position = Point(lon, lat, ele + 2)  # 抬高一点以便看得清楚
        
        # 显式初始化四元数为单位四元数
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0

    markers.append(text_marker)
    return markers

def publish_markers(nodes, ways, relations):
    rospy.init_node('osm_marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('osm_markers', MarkerArray, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        marker_array = MarkerArray()

        # 发布节点标记
        for id, position in nodes.items():
            marker_array.markers.append(create_node_marker(id, position))

        # 发布路径标记
        for i, way in enumerate(ways):
            way_marker = create_way_marker(way, nodes, i * 10)
            if way_marker:  # 只添加非空的 Marker
                marker_array.markers.append(way_marker)

        # 发布关系标记
        id_offset = len(nodes) + len([w for w in ways if create_way_marker(w, nodes, 0)]) * 10
        for relation in relations:
            markers = create_relation_marker(relation, ways, nodes, id_offset)
            marker_array.markers.extend(markers)
            id_offset += len(markers)

        marker_pub.publish(marker_array)
        rate.sleep()

if __name__ == '__main__':
    try:
        osm_file_path = '/home/cyun/Documents/panel_ws/src/map_load/scripts/lanelet2_map.osm'  # 替换为实际路径
        nodes, ways, relations = parse_osm(osm_file_path)
        publish_markers(nodes, ways, relations)
    except rospy.ROSInterruptException:
        pass
