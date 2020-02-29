

说明： 代码补充完成，但是计算结果一直有问题，排除中暂时还找不到问题点。。

![image-20200223195004398](/home/robot/tmp/github/hw/hw3/result.assets/image-20200223195004398.png)



![image-20200223195028669](/home/robot/tmp/github/hw/hw3/result.assets/image-20200223195028669.png)



===============================记录点



landmark   

```
createLandmarks  200

    std::normal_distribution<double> d_x{0.0, 4.0};
    std::normal_distribution<double> d_y{0.0, 10.0};
    std::normal_distribution<double> d_z{0.0, 10.0};
```





createCameraPose(v_Twc, Eigen::Vector3d(0,0,0));

```
    float x_offset = 20;
    float y_offset = 0;
    float z_offset = -5; 
```



```
detectFeatures(v_Twc[0], K, landmarks, features_last, features_matched_last);
detectFeatures(v_Twc[i], K, landmarks, features_curr, features_matched_cur);

features_curr ==> point_curr
        std::vector<cv::Point2f> point_curr;
        std::vector<cv::Point2f> point_last;
```




​    std::vector<Eigen::Vector3d> map_points;
​    map_points.resize(landmarks.size());

```
init_flag = createInitMap(point_curr, point_last, cv_K, T_curr_last, map_points, good_matches);
```

初始化F ， 三角化的map_points是带有尺度的

```
            double t_scale = 1.0;
            {
                Eigen::Matrix4d gt_Twc_last = Twc_last;
                Eigen::Matrix4d gt_Twc_curr = v_Twc[i];
                Eigen::Matrix4d gt_T_cur_last = gt_Twc_curr.inverse() * gt_Twc_last;
                t_scale = gt_T_cur_last.block(0, 3, 3, 1).norm();
            }
```

map_points恢复到世界坐标系

```
            for (size_t n = 0; n < map_points.size(); n++)
            {
                if (!good_matches[n]) continue;
                Eigen::Vector3d &mpt = map_points[n];
                mpt *= t_scale;
                mpt = Twc_last.block(0, 0, 3, 3) * mpt + Twc_last.block(0, 3, 3, 1);
            }
```

