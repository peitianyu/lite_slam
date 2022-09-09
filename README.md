# 纸上得来终觉浅, 绝知此事要躬行.

# 环境
    - Ubuntu18
    - Eigen
    - Opencv(仅用于显示)

# bag2txt
```
https://github.com/peitianyu/bag2txt
```

# 已完成
```
- scan_to_map定位(体素滤波加速)
- grid_map更新(降采样地图)
- scancontext回环检测
- 建图与重定位,纯定位完整流程
```

# 模块测试完成
```
- 位姿图优化 // FIXME: 思考能否降维
- 配置文件设置(ini文件) // FIXME: 不够优雅
- ndt,icp(完成测试)
```

# 待完成
```
- 逻辑优化
    - 借鉴loam思想实现实时定位
        - 简单版: 直接差分里程
        - 复杂版: 通过ndt得到粗pose,用于输出前端里程
    - scantomap工程优化
    - 使用scan_context回环检测用于位姿图优化更新位姿与概率图
- 概率地图data将MatrixXf格式改为一维数组,使之更为通用
- 加入mcl进行重定位
- 加入submap模块,并重新设计工程
- 加入反光板(作为land_mark)
- 加入imu预处理
- 加入scan畸变修正
```

# 注意
```
- 注意scan_context.h文件中,有一项需要配置
```
