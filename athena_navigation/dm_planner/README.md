Distance Map planner
===================
  基于A*算法的全局规划器，与Navfn不同之处在于支持指定搜索区域，搜索区域有给定路径及搜索半径确定。
  
## Navigation2

### 规划器配置参数

```
planner_plugins: ["DMP"]
DMP:
    plugin: "dm_planner/DMPlanner"
    visualize: true
    free_radius: 0.5
    search_radius: 1.0
    tracking_distance: 1.0
```
- visualize: 发布搜索代价地图，默认为false
- free_radius: 目标点清空半径，在搜索地图中清空目标点周围指定半径区域。默认为0.0,此时不限制
- search_radius: 搜索半径，与指定路径结合限制算法搜索区域。默认为1.0,此时不限制
- tracking_distanse: 跟踪半径，默认为1.0
    
### nav2_behavior_tree
导航树默认规划器需替换为DMPlanner

```
<ComputePathToPose goal="{goal}" path="{path}" planner_id="DMP"/>
```
