^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nav2_costmap_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

(2021-08-05)
-------------------
* Fixed the master layer will be emptied after receiving one static map. In this case, if Costmap2DRos is too late to update the master layer, the planner will get an empty map to search in
* Fixed the master layer will be emptied after called clear costmap service

(2018-10-9)
-------------------
* Port nav2_costmap_2d from costmap_2d version 1.16.2 (2018-07-31) 
