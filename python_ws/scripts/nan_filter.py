#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
点云NaN过滤 + 体素降采样节点
订阅: /cloud_registered_body (body帧) → 发布: /cloud_filtered

1. 过滤掉点云中包含NaN/Inf值的点
2. 体素网格降采样 (0.15m)
3. ★第十七次关键修复: 改用body帧点云
   - 之前用/cloud_registered (frame_id=camera_init, 全局帧)
   - OctoMap计算sensor origin = TF(camera_init→camera_init) = (0,0,0)
   - 所有射线从世界原点出发, 而非无人机位置 → 这是墙壁瓦解的真正根因!
   - 现在用/cloud_registered_body (frame_id=body)
   - OctoMap通过TF(body→camera_init)得到正确的无人机位置作为sensor origin
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2

# 体素降采样分辨率 (匹配OctoMap resolution)
VOXEL_SIZE = 0.15


class NanFilter:
    def __init__(self):
        rospy.init_node('nan_filter', anonymous=True)
        self.pub = rospy.Publisher('/cloud_filtered', PointCloud2, queue_size=2)
        # ★第17次关键修复: 订阅body帧点云, 让OctoMap获得正确的sensor origin
        rospy.Subscriber('/cloud_registered_body', PointCloud2, self.callback, queue_size=2,
                         buff_size=2**24)
        self.count = 0
        self.drop_count = 0
        rospy.loginfo("[NaN Filter] 已启动 - /cloud_registered_body(body帧) -> /cloud_filtered")

    def callback(self, msg):
        try:
            point_step = msg.point_step
            n_points = msg.width * msg.height
            if n_points == 0:
                self.pub.publish(msg)
                return

            # 找xyz字段偏移
            ox = oy = oz = -1
            for f in msg.fields:
                if f.name == 'x':
                    ox = f.offset
                elif f.name == 'y':
                    oy = f.offset
                elif f.name == 'z':
                    oz = f.offset

            if ox < 0 or oy < 0 or oz < 0:
                self.pub.publish(msg)
                return

            # ★ 向量化处理: 一次性将整个点云转为numpy数组
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            expected_size = n_points * point_step
            if raw.size != expected_size:
                rospy.logwarn_throttle(5, "[NaN Filter] 数据大小不匹配: %d != %d",
                                       raw.size, expected_size)
                return

            # 重塑为 (n_points, point_step) 的2D数组
            points_raw = raw.reshape(n_points, point_step)

            # ★ 使用np.ndarray提取xyz (无需逐点struct.unpack)
            # 先确保内存连续, 再用view转换
            x_bytes = np.ascontiguousarray(points_raw[:, ox:ox+4]).view(np.float32).flatten()
            y_bytes = np.ascontiguousarray(points_raw[:, oy:oy+4]).view(np.float32).flatten()
            z_bytes = np.ascontiguousarray(points_raw[:, oz:oz+4]).view(np.float32).flatten()

            # ★ 向量化过滤: 一次性计算所有点的有效性
            valid = (np.isfinite(x_bytes) & np.isfinite(y_bytes) & np.isfinite(z_bytes) &
                     (np.abs(x_bytes) < 1e6) & (np.abs(y_bytes) < 1e6) & (np.abs(z_bytes) < 1e6))

            valid_mask = valid
            valid_count_nan = int(np.sum(valid_mask))

            if valid_count_nan == 0:
                return

            # 取有效点的原始字节行
            valid_rows = points_raw[valid_mask]
            vx = x_bytes[valid_mask]
            vy = y_bytes[valid_mask]
            vz = z_bytes[valid_mask]

            # ★ 第十五次关键优化: 体素网格降采样
            # 同一OctoMap体素内的多个点只保留1个, 大幅减少OctoMap计算量
            voxel_idx = np.floor(np.column_stack([vx, vy, vz]) / VOXEL_SIZE).astype(np.int32)
            # 用结构化数组做unique, 比tuple快很多
            voxel_flat = voxel_idx[:, 0].astype(np.int64) * 1000000 + \
                         voxel_idx[:, 1].astype(np.int64) * 1000 + \
                         voxel_idx[:, 2].astype(np.int64)
            _, unique_idx = np.unique(voxel_flat, return_index=True)

            downsampled_rows = valid_rows[unique_idx]
            valid_count = len(unique_idx)

            # 发布降采样后的点云
            out_data = downsampled_rows.tobytes()
            out = PointCloud2()
            out.header = msg.header
            out.height = 1
            out.width = valid_count
            out.fields = msg.fields
            out.is_bigendian = msg.is_bigendian
            out.point_step = point_step
            out.row_step = point_step * valid_count
            out.data = out_data
            out.is_dense = True
            self.pub.publish(out)

            self.count += 1
            filtered = n_points - valid_count
            if filtered > 0:
                self.drop_count += filtered
            if self.count % 100 == 0:
                rospy.loginfo_throttle(15,
                    "[NaN Filter] 已处理 %d 帧, 本帧: %d->%d->%d (NaN过滤->体素降采样), 累计过滤 %d",
                    self.count, n_points, valid_count_nan, valid_count, self.drop_count)

        except Exception as e:
            rospy.logwarn_throttle(3, "[NaN Filter] 回调异常: %s", str(e))


if __name__ == '__main__':
    try:
        NanFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
