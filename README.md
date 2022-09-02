# 纸上得来终觉浅, 绝知此事要躬行.

# 环境
    - Eigen
    - Opencv(仅用于显示)

# 建图

```
- scan_to_map定位
- scan_to_map获得匹配概率
- 更新grid_map
- 保存grid_map
```

# 纯定位

```
- 读取grid_map
- scan_context重定位
- 转到scan_to_map定位
```

# 分析

## grid_map

```
- 精炼代码(主要是Eigen::Vector2i Point)
```

## scan_to_map

```
- 优化时先用粗匹配优化到一定程度,在进行细匹配
```

## scan_context

```
- 注意scan_context.h文件中,有一项需要配置
```


## 更新地图
```
- 注意只有当迭代残差小于一定值时才更新坐标
```
