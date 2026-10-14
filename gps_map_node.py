#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import rasterio
import math
import os
import csv
import matplotlib.pyplot as plt

from sensor_msgs.msg import NavSatFix


class GpsSatelliteTracker:

    def __init__(self,
                 tif_path,
                 downsample_scale=1,
                 roi_size=(800, 800),
                 zoom=1):

        self.downsample_scale = downsample_scale
        self.roi_width, self.roi_height = roi_size
        self.zoom = zoom

        # 创建保存目录
        self.save_dir = os.path.expanduser("~/gps_eval_results")
        os.makedirs(self.save_dir, exist_ok=True)

        # 读取卫星图
        dataset = rasterio.open(tif_path)
        self.src_transform = dataset.transform

        raw_img = dataset.read([1, 2, 3])
        raw_img = np.transpose(raw_img, (1, 2, 0))
        raw_img = cv2.cvtColor(raw_img, cv2.COLOR_RGB2BGR)

        self.src_height, self.src_width = raw_img.shape[:2]

        dst_width = self.src_width // downsample_scale
        dst_height = self.src_height // downsample_scale

        self.map_img = cv2.resize(
            raw_img,
            (dst_width, dst_height),
            interpolation=cv2.INTER_AREA
        )

        scale_transform = rasterio.Affine(self.src_transform.a * downsample_scale,
                                          self.src_transform.b,
                                          self.src_transform.c,
                                          self.src_transform.d,
                                          self.src_transform.e * downsample_scale,
                                          self.src_transform.f)

        self.down_transform = scale_transform

        # 轨迹
        self.geovins_path_points = []
        self.mavros_path_points = []

        # 经纬度
        self.geovins_latest = None
        self.mavros_latest = None

        # 误差记录
        self.error_list = []
        self.time_list = []

        # CSV数据
        self.csv_data = []

        self.window_name = "Tracked Satellite ROI"

        rospy.Subscriber("/geovins_gps", NavSatFix, self.geovins_gps_callback)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.mavros_callback)

        rospy.on_shutdown(self.save_results)

    # -----------------------------------
    # GeoVINS GPS
    # -----------------------------------
    def geovins_gps_callback(self, msg):

        lat = msg.latitude
        lon = msg.longitude

        self.geovins_latest = (lat, lon)

        px, py = ~self.down_transform * (lon, lat)

        px = int(round(px))
        py = int(round(py))

        h, w = self.map_img.shape[:2]

        if 0 <= px < w and 0 <= py < h:
            self.geovins_path_points.append((px, py))

    # -----------------------------------
    # MAVROS GPS
    # -----------------------------------
    def mavros_callback(self, msg):

        lat = msg.latitude
        lon = msg.longitude

        self.mavros_latest = (lat, lon)

        px, py = ~self.down_transform * (lon, lat)

        px = int(round(px))
        py = int(round(py))

        h, w = self.map_img.shape[:2]

        if 0 <= px < w and 0 <= py < h:
            self.mavros_path_points.append((px, py))

    # -----------------------------------
    # Haversine
    # -----------------------------------
    def haversine_distance(self, lat1, lon1, lat2, lon2):

        R = 6371000.0

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)

        d_phi = math.radians(lat2 - lat1)
        d_lambda = math.radians(lon2 - lon1)

        a = math.sin(d_phi / 2.0) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * \
            math.sin(d_lambda / 2.0) ** 2

        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

    # -----------------------------------
    # 主循环
    # -----------------------------------
    def run(self):

        rate = rospy.Rate(10)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():

            display_img = self.map_img.copy()

            # geovins轨迹
            if len(self.geovins_path_points) > 1:
                for i in range(len(self.geovins_path_points) - 1):
                    cv2.line(display_img,
                             self.geovins_path_points[i],
                             self.geovins_path_points[i + 1],
                             (0, 0, 255), 2)

            if self.geovins_path_points:
                geovins_pt = self.geovins_path_points[-1]
                cv2.circle(display_img, geovins_pt, 5, (0, 255, 0), -1)

            # MAVROS轨迹
            if len(self.mavros_path_points) > 1:
                for i in range(len(self.mavros_path_points) - 1):
                    cv2.line(display_img,
                             self.mavros_path_points[i],
                             self.mavros_path_points[i + 1],
                             (255, 0, 0), 2)

            if self.mavros_path_points:
                mavros_pt = self.mavros_path_points[-1]
                cv2.circle(display_img, mavros_pt, 5, (0, 255, 255), -1)

            # ROI中心
            if self.geovins_path_points:
                center_pt = self.geovins_path_points[-1]
            else:
                center_pt = (self.map_img.shape[1] // 2,
                             self.map_img.shape[0] // 2)

            eff_roi_w = int(self.roi_width / self.zoom)
            eff_roi_h = int(self.roi_height / self.zoom)

            roi = cv2.getRectSubPix(display_img,
                                    (eff_roi_w, eff_roi_h),
                                    (float(center_pt[0]), float(center_pt[1])))

            roi_disp = cv2.resize(roi,
                                  (self.roi_width, self.roi_height),
                                  interpolation=cv2.INTER_LINEAR)

            # 计算误差
            if self.geovins_latest and self.mavros_latest:

                lat_g, lon_g = self.geovins_latest
                lat_m, lon_m = self.mavros_latest

                error = self.haversine_distance(lat_g, lon_g, lat_m, lon_m)

                t = rospy.Time.now().to_sec() - start_time

                self.error_list.append(error)
                self.time_list.append(t)

                self.csv_data.append([t, lat_g, lon_g, lat_m, lon_m, error])

                text = "Error: {:.2f} m".format(error)

                cv2.putText(roi_disp,
                            text,
                            (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (0, 255, 255),
                            2)

            cv2.imshow(self.window_name, roi_disp)
            cv2.waitKey(1)

            rate.sleep()

    # -----------------------------------
    # 保存结果
    # -----------------------------------
    def save_results(self):

        print("Saving evaluation results...")

        if len(self.error_list) == 0:
            return

        errors = np.array(self.error_list)

        rmse = np.sqrt(np.mean(errors ** 2))
        mean = np.mean(errors)
        max_err = np.max(errors)

        # 保存统计
        with open(self.save_dir + "/stats.txt", "w") as f:
            f.write("RMSE: {:.3f} m\n".format(rmse))
            f.write("Mean Error: {:.3f} m\n".format(mean))
            f.write("Max Error: {:.3f} m\n".format(max_err))

        # 保存CSV
        with open(self.save_dir + "/gps_error.csv", "w") as f:

            writer = csv.writer(f)

            writer.writerow([
                "time",
                "geovins_lat",
                "geovins_lon",
                "mavros_lat",
                "mavros_lon",
                "error_m"
            ])

            writer.writerows(self.csv_data)

        # 保存误差曲线
        plt.figure()
        plt.plot(self.time_list, self.error_list)
        plt.xlabel("Time (s)")
        plt.ylabel("Error (m)")
        plt.title("GPS Error")
        plt.grid()

        plt.savefig(self.save_dir + "/error_curve.png")

        # 保存轨迹图
        traj_img = self.map_img.copy()

        for i in range(len(self.geovins_path_points) - 1):
            cv2.line(traj_img,
                     self.geovins_path_points[i],
                     self.geovins_path_points[i + 1],
                     (0, 0, 255), 2)

        for i in range(len(self.mavros_path_points) - 1):
            cv2.line(traj_img,
                     self.mavros_path_points[i],
                     self.mavros_path_points[i + 1],
                     (255, 0, 0), 2)

        cv2.imwrite(self.save_dir + "/trajectory.png", traj_img)

        print("Results saved to:", self.save_dir)
        print("RMSE:", rmse, "m")
        print("Mean:", mean, "m")
        print("Max:", max_err, "m")


def main():

    rospy.init_node("gps_satellite_tracker")

    tif_path = "/home/yzq/Drivers/flight_ws/zhuozhou.tif"

    tracker = GpsSatelliteTracker(tif_path)

    tracker.run()


if __name__ == "__main__":
    main()
