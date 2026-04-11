#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCD点云 → 2D占据栅格地图 转换工具

核心方案 (第十七次新增):
  1. 读取 FAST-LIO2 生成的全局 .pcd 点云
  2. Z轴直通滤波: 提取飞行高度 ±tolerance 的点云切片
  3. 投影到2D平面, 生成标准 ROS map_server 格式 (.pgm + .yaml)
  4. 可直接用于 A*/Dijkstra 2D路径规划

用法:
  rosrun mak4_sim pcd_to_2d_map.py                                # 默认参数
  rosrun mak4_sim pcd_to_2d_map.py --height 1.0 --tolerance 0.3  # 自定义高度
  rosrun mak4_sim pcd_to_2d_map.py --pcd /path/to/map.pcd         # 自定义PCD文件
"""

import argparse
import os
import sys
import struct
import numpy as np


def read_pcd(filepath):
    """读取PCD文件, 返回 Nx3 numpy数组 (x,y,z)"""
    print("[PCD→2D] 读取PCD文件: %s" % filepath)
    
    with open(filepath, 'rb') as f:
        header_lines = []
        fields = []
        n_points = 0
        data_type = 'ascii'
        
        while True:
            line = f.readline()
            if not line:
                break
            line_str = line.decode('ascii', errors='ignore').strip()
            header_lines.append(line_str)
            
            if line_str.startswith('FIELDS'):
                fields = line_str.split()[1:]
            elif line_str.startswith('POINTS'):
                n_points = int(line_str.split()[1])
            elif line_str.startswith('DATA'):
                data_type = line_str.split()[1].lower()
                break
        
        if n_points == 0:
            print("[PCD→2D] 错误: PCD文件中没有点!")
            return None
        
        # 找到 x,y,z 在fields中的索引
        try:
            xi = fields.index('x')
            yi = fields.index('y')
            zi = fields.index('z')
        except ValueError:
            print("[PCD→2D] 错误: PCD文件缺少x/y/z字段! fields=%s" % fields)
            return None
        
        print("[PCD→2D] 点数: %d, 数据格式: %s, 字段: %s" % (n_points, data_type, fields))
        
        if data_type == 'binary':
            # 二进制PCD - 假设每个字段4字节float32
            n_fields = len(fields)
            point_size = n_fields * 4
            raw = f.read(n_points * point_size)
            if len(raw) < n_points * point_size:
                # 可能有额外字段 - 尝试检测实际SIZE
                f.seek(-len(raw), 1)
                # 重新读取header找SIZE信息
                for hl in header_lines:
                    if hl.startswith('SIZE'):
                        sizes = [int(s) for s in hl.split()[1:]]
                        point_size = sum(sizes)
                        break
                raw = f.read(n_points * point_size)
            
            data = np.frombuffer(raw, dtype=np.uint8).reshape(n_points, point_size)
            x = np.frombuffer(np.ascontiguousarray(data[:, xi*4:(xi+1)*4]), dtype=np.float32)
            y = np.frombuffer(np.ascontiguousarray(data[:, yi*4:(yi+1)*4]), dtype=np.float32)
            z = np.frombuffer(np.ascontiguousarray(data[:, zi*4:(zi+1)*4]), dtype=np.float32)
            points = np.column_stack([x, y, z])
        
        elif data_type == 'binary_compressed':
            print("[PCD→2D] 暂不支持 binary_compressed 格式, 请用 pcl_convert_pcd_ascii_binary 转换")
            return None
        
        else:  # ascii
            points = np.zeros((n_points, 3), dtype=np.float32)
            for i in range(n_points):
                line = f.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    break
                vals = line.split()
                try:
                    points[i, 0] = float(vals[xi])
                    points[i, 1] = float(vals[yi])
                    points[i, 2] = float(vals[zi])
                except (IndexError, ValueError):
                    continue
    
    # 过滤NaN/Inf
    valid = np.isfinite(points).all(axis=1)
    points = points[valid]
    print("[PCD→2D] 有效点数: %d (过滤了 %d 个无效点)" % (len(points), np.sum(~valid)))
    
    return points


def passthrough_filter(points, z_min, z_max):
    """Z轴直通滤波: 只保留 z_min <= z <= z_max 的点"""
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    filtered = points[mask]
    print("[PCD→2D] Z轴直通滤波 [%.2f, %.2f]: %d → %d 个点" % (z_min, z_max, len(points), len(filtered)))
    return filtered


def points_to_gridmap(points_2d, resolution, inflate_radius=0.0):
    """
    2D点集 → 占据栅格地图
    
    参数:
      points_2d: Nx2 numpy数组 (x, y)
      resolution: 栅格分辨率 (m/pixel)
      inflate_radius: 障碍物膨胀半径 (m)
    
    返回:
      grid: 2D numpy数组 (0=free, 254=occupied, 205=unknown)
      origin: (origin_x, origin_y) 地图左下角世界坐标
    """
    if len(points_2d) == 0:
        print("[PCD→2D] 警告: 无点可投影!")
        return None, None
    
    x_min, x_max = points_2d[:, 0].min(), points_2d[:, 0].max()
    y_min, y_max = points_2d[:, 1].min(), points_2d[:, 1].max()
    
    # 留边距
    margin = 1.0  # 1m边距
    x_min -= margin
    x_max += margin
    y_min -= margin
    y_max += margin
    
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))
    
    print("[PCD→2D] 栅格地图大小: %d x %d (%.1fm x %.1fm), 分辨率=%.3fm" % 
          (width, height, x_max - x_min, y_max - y_min, resolution))
    
    # 初始化为自由空间 (白色=254)
    grid = np.full((height, width), 254, dtype=np.uint8)
    
    # 将点云投影到栅格坐标
    col = ((points_2d[:, 0] - x_min) / resolution).astype(int)
    row = ((points_2d[:, 1] - y_min) / resolution).astype(int)
    
    # 裁剪到范围内
    valid = (col >= 0) & (col < width) & (row >= 0) & (row < height)
    col = col[valid]
    row = row[valid]
    
    # 标记障碍物 (黑色=0)
    grid[row, col] = 0
    
    # 障碍物膨胀
    if inflate_radius > 0:
        inflate_cells = int(np.ceil(inflate_radius / resolution))
        print("[PCD→2D] 障碍物膨胀: %.2fm (%d cells)" % (inflate_radius, inflate_cells))
        inflated = grid.copy()
        occupied = np.argwhere(grid == 0)
        for oy, ox in occupied:
            for dy in range(-inflate_cells, inflate_cells + 1):
                for dx in range(-inflate_cells, inflate_cells + 1):
                    if dy*dy + dx*dx <= inflate_cells*inflate_cells:
                        ny, nx = oy + dy, ox + dx
                        if 0 <= ny < height and 0 <= nx < width:
                            inflated[ny, nx] = 0
        grid = inflated
    
    # 统计
    occupied_count = np.sum(grid == 0)
    free_count = np.sum(grid == 254)
    print("[PCD→2D] 占据: %d cells, 自由: %d cells" % (occupied_count, free_count))
    
    origin = (x_min, y_min)
    return grid, origin


def save_map(grid, origin, resolution, output_dir, map_name):
    """保存为 ROS map_server 标准格式 (.pgm + .yaml)"""
    os.makedirs(output_dir, exist_ok=True)
    
    pgm_path = os.path.join(output_dir, map_name + '.pgm')
    yaml_path = os.path.join(output_dir, map_name + '.yaml')
    
    # 保存 PGM (注意: ROS map_server 使用 y轴翻转, 从bottom-up存储)
    height, width = grid.shape
    # ROS map_server格式: 0=occupied(黑), 254=free(白), pgm中翻转y轴
    grid_flipped = np.flipud(grid)
    
    with open(pgm_path, 'wb') as f:
        header = "P5\n%d %d\n255\n" % (width, height)
        f.write(header.encode('ascii'))
        f.write(grid_flipped.tobytes())
    
    # 保存 YAML
    with open(yaml_path, 'w') as f:
        f.write("image: %s\n" % (map_name + '.pgm'))
        f.write("resolution: %.4f\n" % resolution)
        f.write("origin: [%.4f, %.4f, 0.0000]\n" % (origin[0], origin[1]))
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
    
    print("[PCD→2D] ✓ 地图已保存:")
    print("  PGM: %s (%d x %d)" % (pgm_path, width, height))
    print("  YAML: %s" % yaml_path)
    print("  加载方式: rosrun map_server map_server %s" % yaml_path)
    
    return pgm_path, yaml_path


def main():
    parser = argparse.ArgumentParser(description='PCD点云 → 2D占据栅格地图')
    parser.add_argument('--pcd', type=str, 
                        default=os.path.expanduser('~/catkin_ws/maps/greenhouse_map.pcd'),
                        help='PCD文件路径')
    parser.add_argument('--height', type=float, default=1.0,
                        help='飞行高度 (m), 用于Z轴切片中心')
    parser.add_argument('--tolerance', type=float, default=0.3,
                        help='Z轴切片半宽 (m), 实际范围=[height-tolerance, height+tolerance]')
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='栅格地图分辨率 (m/pixel)')
    parser.add_argument('--inflate', type=float, default=0.2,
                        help='障碍物膨胀半径 (m), 0=不膨胀')
    parser.add_argument('--output', type=str,
                        default=os.path.expanduser('~/catkin_ws/maps'),
                        help='输出目录')
    parser.add_argument('--name', type=str, default='greenhouse_2d',
                        help='输出地图名')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("  PCD → 2D 占据栅格地图转换工具")
    print("=" * 60)
    print("参数:")
    print("  PCD文件:  %s" % args.pcd)
    print("  飞行高度: %.2fm" % args.height)
    print("  切片范围: [%.2f, %.2f]m" % (args.height - args.tolerance, args.height + args.tolerance))
    print("  分辨率:   %.3fm" % args.resolution)
    print("  膨胀半径: %.2fm" % args.inflate)
    print("")
    
    # 检查文件
    if not os.path.exists(args.pcd):
        print("[PCD→2D] 错误: PCD文件不存在: %s" % args.pcd)
        print("[PCD→2D] 请先运行建图 (slam_mapping.launch) 并保存地图 (save_map.py)")
        sys.exit(1)
    
    file_size = os.path.getsize(args.pcd) / (1024 * 1024)
    print("[PCD→2D] PCD文件大小: %.1f MB" % file_size)
    
    # 1. 读取PCD
    points = read_pcd(args.pcd)
    if points is None or len(points) == 0:
        print("[PCD→2D] 错误: 读取PCD失败或无有效点")
        sys.exit(1)
    
    print("[PCD→2D] 点云范围: X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f]" % (
        points[:, 0].min(), points[:, 0].max(),
        points[:, 1].min(), points[:, 1].max(),
        points[:, 2].min(), points[:, 2].max()))
    
    # 2. Z轴直通滤波
    z_min = args.height - args.tolerance
    z_max = args.height + args.tolerance
    slice_points = passthrough_filter(points, z_min, z_max)
    
    if len(slice_points) == 0:
        print("[PCD→2D] 错误: 切片范围内没有点! 请检查 --height 参数")
        print("[PCD→2D] 建议: 点云Z范围=[%.2f, %.2f], 当前切片=[%.2f, %.2f]" % (
            points[:, 2].min(), points[:, 2].max(), z_min, z_max))
        sys.exit(1)
    
    # 3. 投影到2D (丢弃Z)
    points_2d = slice_points[:, :2]  # 只取x,y
    
    # 4. 生成栅格地图
    grid, origin = points_to_gridmap(points_2d, args.resolution, args.inflate)
    if grid is None:
        sys.exit(1)
    
    # 5. 保存
    save_map(grid, origin, args.resolution, args.output, args.name)
    
    print("")
    print("=" * 60)
    print("  ✓ 转换完成!")
    print("  下一步: roslaunch mak4_sim navigation_2d.launch")
    print("=" * 60)


if __name__ == '__main__':
    main()
