
RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/simulator/pprzsim-launch -a ardrone2 -t nps'
RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/tools/gcs_launch.py '
RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/ground_segment/tmtc/server -n'
RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/ground_segment/tmtc/link -udp -udp_broadcast'
Simulating with dt of 0.000977
Current schedparam: policy 0, prio 0
setschedparam failed!: Operation not permitted
THX before no-fly-points 
start printing
Depth 0: Node 1 has no neighbors.
done 
x: -26088.000000 y: -25118.000000 rad: 759.000000 THX denom 2.414214  end 
x: 22295.000000 y: 27750.000000 rad: 1265.000000 THX denom 2.414214  end 
THX after no-fly-points 
start printing
Depth 0: Node 1 (0.0, 0.0) has 8 neighbors: 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 16(21030.0, 27226.0), 17(21030.0, 28274.0)
start printing
Depth 1: Node 3 (-25773.6, -24359.0) has 9 neighbors: 2(-26402.4, -24359.0), 4(-25329.0, -24803.6), 9(-26847.0, -24803.6), 1(0.0, 0.0), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 16(21030.0, 27226.0), 17(21030.0, 28274.0)
start printing
Depth 2: Node 2 (-26402.4, -24359.0) has 6 neighbors: 3(-25773.6, -24359.0), 8(-26847.0, -25432.4), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 17(21030.0, 28274.0)
start printing
Depth 3: Node 8 (-26847.0, -25432.4) has 2 neighbors: 7(-26402.4, -25877.0), 2(-26402.4, -24359.0)
start printing
Depth 4: Node 7 (-26402.4, -25877.0) has 2 neighbors: 6(-25773.6, -25877.0), 8(-26847.0, -25432.4)
start printing
Depth 5: Node 6 (-25773.6, -25877.0) has 2 neighbors: 5(-25329.0, -25432.4), 7(-26402.4, -25877.0)
start printing
Depth 6: Node 5 (-25329.0, -25432.4) has 8 neighbors: 4(-25329.0, -24803.6), 6(-25773.6, -25877.0), 1(0.0, 0.0), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 16(21030.0, 27226.0), 17(21030.0, 28274.0)
start printing
Depth 7: Node 4 (-25329.0, -24803.6) has 8 neighbors: 3(-25773.6, -24359.0), 5(-25329.0, -25432.4), 1(0.0, 0.0), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 16(21030.0, 27226.0), 17(21030.0, 28274.0)
start printing
Depth 8: Node 13 (23560.0, 27226.0) has 8 neighbors: 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 9(-26847.0, -24803.6), 12(23560.0, 28274.0), 14(22819.0, 26485.0), 1(0.0, 0.0)
start printing
Depth 9: Node 9 (-26847.0, -24803.6) has 6 neighbors: 3(-25773.6, -24359.0), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 16(21030.0, 27226.0), 17(21030.0, 28274.0)
start printing
Depth 10: Node 14 (22819.0, 26485.0) has 8 neighbors: 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 9(-26847.0, -24803.6), 13(23560.0, 27226.0), 15(21771.0, 26485.0), 1(0.0, 0.0)
start printing
Depth 11: Node 15 (21771.0, 26485.0) has 8 neighbors: 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 9(-26847.0, -24803.6), 14(22819.0, 26485.0), 16(21030.0, 27226.0), 1(0.0, 0.0)
start printing
Depth 12: Node 16 (21030.0, 27226.0) has 6 neighbors: 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 9(-26847.0, -24803.6), 15(21771.0, 26485.0), 1(0.0, 0.0)
start printing
Depth 10: Node 17 (21030.0, 28274.0) has 7 neighbors: 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 9(-26847.0, -24803.6), 11(22819.0, 29015.0), 1(0.0, 0.0)
start printing
Depth 11: Node 11 (22819.0, 29015.0) has 3 neighbors: 12(23560.0, 28274.0), 10(21771.0, 29015.0), 17(21030.0, 28274.0)
start printing
Depth 12: Node 12 (23560.0, 28274.0) has 2 neighbors: 11(22819.0, 29015.0), 13(23560.0, 27226.0)
start printing
Depth 12: Node 10 has one neighbor: 11(22819.0, 29015.0)
done 
THX after all 
start printing
Depth 0: Node 1 (0.0, 0.0) has 10 neighbors: 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 16(21030.0, 27226.0), 17(21030.0, 28274.0), 19(26088.0, 25118.0), 20(18503.0, 30384.0)
start printing
Depth 1: Node 3 (-25773.6, -24359.0) has 12 neighbors: 2(-26402.4, -24359.0), 4(-25329.0, -24803.6), 9(-26847.0, -24803.6), 1(0.0, 0.0), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 16(21030.0, 27226.0), 17(21030.0, 28274.0), 18(-26088.0, -25118.0), 19(26088.0, 25118.0), 20(18503.0, 30384.0)
start printing
Depth 2: Node 2 (-26402.4, -24359.0) has 9 neighbors: 3(-25773.6, -24359.0), 8(-26847.0, -25432.4), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 17(21030.0, 28274.0), 18(-26088.0, -25118.0), 19(26088.0, 25118.0), 20(18503.0, 30384.0)
start printing
Depth 3: Node 8 (-26847.0, -25432.4) has 3 neighbors: 7(-26402.4, -25877.0), 2(-26402.4, -24359.0), 18(-26088.0, -25118.0)
start printing
Depth 4: Node 7 (-26402.4, -25877.0) has 3 neighbors: 6(-25773.6, -25877.0), 8(-26847.0, -25432.4), 18(-26088.0, -25118.0)
start printing
Depth 5: Node 6 (-25773.6, -25877.0) has 4 neighbors: 5(-25329.0, -25432.4), 7(-26402.4, -25877.0), 18(-26088.0, -25118.0), 19(26088.0, 25118.0)
start printing
Depth 6: Node 5 (-25329.0, -25432.4) has 11 neighbors: 4(-25329.0, -24803.6), 6(-25773.6, -25877.0), 1(0.0, 0.0), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 16(21030.0, 27226.0), 17(21030.0, 28274.0), 18(-26088.0, -25118.0), 19(26088.0, 25118.0), 20(18503.0, 30384.0)
start printing
Depth 7: Node 4 (-25329.0, -24803.6) has 11 neighbors: 3(-25773.6, -24359.0), 5(-25329.0, -25432.4), 1(0.0, 0.0), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 16(21030.0, 27226.0), 17(21030.0, 28274.0), 18(-26088.0, -25118.0), 19(26088.0, 25118.0), 20(18503.0, 30384.0)
start printing
Depth 8: Node 13 (23560.0, 27226.0) has 9 neighbors: 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 9(-26847.0, -24803.6), 12(23560.0, 28274.0), 14(22819.0, 26485.0), 1(0.0, 0.0), 19(26088.0, 25118.0)
start printing
Depth 9: Node 9 (-26847.0, -24803.6) has 8 neighbors: 3(-25773.6, -24359.0), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 16(21030.0, 27226.0), 17(21030.0, 28274.0), 18(-26088.0, -25118.0), 20(18503.0, 30384.0)
start printing
Depth 10: Node 14 (22819.0, 26485.0) has 9 neighbors: 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 9(-26847.0, -24803.6), 13(23560.0, 27226.0), 15(21771.0, 26485.0), 1(0.0, 0.0), 19(26088.0, 25118.0)
start printing
Depth 11: Node 15 (21771.0, 26485.0) has 9 neighbors: 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 9(-26847.0, -24803.6), 14(22819.0, 26485.0), 16(21030.0, 27226.0), 1(0.0, 0.0), 19(26088.0, 25118.0)
start printing
Depth 12: Node 16 (21030.0, 27226.0) has 7 neighbors: 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 9(-26847.0, -24803.6), 15(21771.0, 26485.0), 1(0.0, 0.0), 20(18503.0, 30384.0)
start printing
Depth 13: Node 20 (18503.0, 30384.0) has 12 neighbors: 1(0.0, 0.0), 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 9(-26847.0, -24803.6), 10(21771.0, 29015.0), 11(22819.0, 29015.0), 16(21030.0, 27226.0), 17(21030.0, 28274.0), 18(-26088.0, -25118.0), 19(26088.0, 25118.0)
start printing
Depth 14: Node 10 (21771.0, 29015.0) has 2 neighbors: 11(22819.0, 29015.0), 20(18503.0, 30384.0)
start printing
Depth 15: Node 11 (22819.0, 29015.0) has 4 neighbors: 12(23560.0, 28274.0), 10(21771.0, 29015.0), 17(21030.0, 28274.0), 20(18503.0, 30384.0)
start printing
Depth 16: Node 12 (23560.0, 28274.0) has 3 neighbors: 11(22819.0, 29015.0), 13(23560.0, 27226.0), 19(26088.0, 25118.0)
start printing
Depth 17: Node 19 (26088.0, 25118.0) has 12 neighbors: 1(0.0, 0.0), 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 6(-25773.6, -25877.0), 12(23560.0, 28274.0), 13(23560.0, 27226.0), 14(22819.0, 26485.0), 15(21771.0, 26485.0), 18(-26088.0, -25118.0), 20(18503.0, 30384.0)
start printing
Depth 18: Node 18 (-26088.0, -25118.0) has 10 neighbors: 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 6(-25773.6, -25877.0), 7(-26402.4, -25877.0), 8(-26847.0, -25432.4), 9(-26847.0, -24803.6), 19(26088.0, 25118.0), 20(18503.0, 30384.0)
start printing
Depth 16: Node 17 (21030.0, 28274.0) has 8 neighbors: 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 9(-26847.0, -24803.6), 11(22819.0, 29015.0), 1(0.0, 0.0), 20(18503.0, 30384.0)
done Test2/26 
[video_thread_nps] Added bottom_camera to camera array.
[viewvideo] Added asynchronous video streamer listener for CAMERA1 at 5 FPS 
Time factor is 1.000000. (Press Ctrl-Z to change)
Gazebo directory: /home/thx/orangePaparazzi/paparazziNewGCS/conf/simulator/gazebo/
Broadcasting on network 127.255.255.255, port 2010
Add Paparazzi paths: /home/thx/orangePaparazzi/paparazziNewGCS/conf/simulator/gazebo/
Add TU Delft paths: /home/thx/orangePaparazzi/paparazziNewGCS/sw/ext/tudelft_gazebo_models/
Load vehicle: /home/thx/orangePaparazzi/paparazziNewGCS/conf/simulator/gazebo/models//ardrone/ardrone.sdf
Load world: /home/thx/orangePaparazzi/paparazziNewGCS/conf/simulator/gazebo/world//empty.world
Get pointer to aircraft: ardrone
Sensors initialized...
Gazebo initialized successfully!
Initializing cameras...
Setting up 'bottom_camera'... ok
here we cahnge the dest_x in the NavSegment.
Using A* to find the shortest path from (0.0, 0.0) to (0.0, 0.0)
printing the astar linked list:
(0.0, 0.0)->
astar_pop************
Reached the goal: (0.0, 0.0)
prepending (0.0, 0.0)
Finished calculating the path
Calculated path: 1(0.0, 0.0)
astar_free************
astar_free************
Using A* to find the shortest path from (0.0, 0.0) to (26088.0, 25118.0)
printing the astar linked list:
(0.0, 0.0)->
astar_pop************
Now considering (0.0, 0.0)
astar_insert************
after insert this node of that  (-25773.6, -24359.0)
astar_insert************
after insert this node of that  (-25329.0, -24803.6)
astar_insert************
after insert this node of that  (-25329.0, -25432.4)
astar_insert************
after insert this node of that  (23560.0, 27226.0)
astar_insert************
after insert this node of that  (22819.0, 26485.0)
astar_insert************
after insert this node of that  (21771.0, 26485.0)
astar_insert************
after insert this node of that  (21030.0, 27226.0)
astar_insert************
after insert this node of that  (21030.0, 28274.0)
astar_insert************
after insert this node of that  (26088.0, 25118.0)
astar_insert************
after insert this node of that  (18503.0, 30384.0)
astar_insert************
printing the astar linked list:
(26088.0, 25118.0)->(22819.0, 26485.0)->(21771.0, 26485.0)->(23560.0, 27226.0)->(21030.0, 27226.0)->(21030.0, 28274.0)->(18503.0, 30384.0)->(-25329.0, -24803.6)->(-25773.6, -24359.0)->(-25329.0, -25432.4)->
astar_pop************
Reached the goal: (26088.0, 25118.0)
prepending (26088.0, 25118.0)
prepending (0.0, 0.0)
Finished calculating the path
Calculated path: 1(0.0, 0.0) 19(26088.0, 25118.0)
astar_free************
astar_free************
above we cahnge the dest_x in the NavSegment.
*************APR25************************************************************************8
Before the dest_x 1, y -1335549777 
After the dest_x 0, y 0 
here we cahnge the dest_x in the NavSegment.
Using A* to find the shortest path from (26088.0, 25118.0) to (26088.0, 25118.0)
printing the astar linked list:
(26088.0, 25118.0)->
astar_pop************
Reached the goal: (26088.0, 25118.0)
prepending (26088.0, 25118.0)
Finished calculating the path
Calculated path: 19(26088.0, 25118.0)
astar_free************
astar_free************
Using A* to find the shortest path from (26088.0, 25118.0) to (18503.0, 30384.0)
printing the astar linked list:
(26088.0, 25118.0)->
astar_pop************
Now considering (26088.0, 25118.0)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (-26402.4, -24359.0)
astar_insert************
after insert this node of that  (-25773.6, -24359.0)
astar_insert************
after insert this node of that  (-25329.0, -24803.6)
astar_insert************
after insert this node of that  (-25329.0, -25432.4)
astar_insert************
after insert this node of that  (-25773.6, -25877.0)
astar_insert************
after insert this node of that  (23560.0, 28274.0)
astar_insert************
after insert this node of that  (23560.0, 27226.0)
astar_insert************
after insert this node of that  (22819.0, 26485.0)
astar_insert************
after insert this node of that  (21771.0, 26485.0)
astar_insert************
after insert this node of that  (-26088.0, -25118.0)
astar_insert************
after insert this node of that  (18503.0, 30384.0)
astar_insert************
printing the astar linked list:
(18503.0, 30384.0)->(23560.0, 27226.0)->(22819.0, 26485.0)->(23560.0, 28274.0)->(21771.0, 26485.0)->(0.0, 0.0)->(-25773.6, -24359.0)->(-25329.0, -24803.6)->(-26402.4, -24359.0)->(-25329.0, -25432.4)->(-26088.0, -25118.0)->(-25773.6, -25877.0)->
astar_pop************
Reached the goal: (18503.0, 30384.0)
prepending (18503.0, 30384.0)
prepending (26088.0, 25118.0)
Finished calculating the path
Calculated path: 19(26088.0, 25118.0) 20(18503.0, 30384.0)
astar_free************
astar_free************
above we cahnge the dest_x in the NavSegment.
*************APR25************************************************************************8
Before the dest_x 1, y -1335549777 
After the dest_x 0, y 0 
here we cahnge the dest_x in the NavSegment.
Using A* to find the shortest path from (18503.0, 30384.0) to (18503.0, 30384.0)
printing the astar linked list:
(18503.0, 30384.0)->
astar_pop************
Reached the goal: (18503.0, 30384.0)
prepending (18503.0, 30384.0)
Finished calculating the path
Calculated path: 20(18503.0, 30384.0)
astar_free************
astar_free************
Using A* to find the shortest path from (18503.0, 30384.0) to (0.0, 0.0)
printing the astar linked list:
(18503.0, 30384.0)->
astar_pop************
Now considering (18503.0, 30384.0)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (-26402.4, -24359.0)
astar_insert************
after insert this node of that  (-25773.6, -24359.0)
astar_insert************
after insert this node of that  (-25329.0, -24803.6)
astar_insert************
after insert this node of that  (-25329.0, -25432.4)
astar_insert************
after insert this node of that  (-26847.0, -24803.6)
astar_insert************
after insert this node of that  (21771.0, 29015.0)
astar_insert************
after insert this node of that  (22819.0, 29015.0)
astar_insert************
after insert this node of that  (21030.0, 27226.0)
astar_insert************
after insert this node of that  (21030.0, 28274.0)
astar_insert************
after insert this node of that  (-26088.0, -25118.0)
astar_insert************
after insert this node of that  (26088.0, 25118.0)
astar_insert************
printing the astar linked list:
(0.0, 0.0)->(21030.0, 27226.0)->(21030.0, 28274.0)->(21771.0, 29015.0)->(22819.0, 29015.0)->(26088.0, 25118.0)->(-25773.6, -24359.0)->(-25329.0, -24803.6)->(-26402.4, -24359.0)->(-25329.0, -25432.4)->(-26088.0, -25118.0)->(-26847.0, -24803.6)->
astar_pop************
Reached the goal: (0.0, 0.0)
prepending (0.0, 0.0)
prepending (18503.0, 30384.0)
Finished calculating the path
Calculated path: 20(18503.0, 30384.0) 1(0.0, 0.0)
astar_free************
astar_free************
above we cahnge the dest_x in the NavSegment.
*************APR25************************************************************************8
Before the dest_x 1, y -1335549777 
After the dest_x 0, y 0 
