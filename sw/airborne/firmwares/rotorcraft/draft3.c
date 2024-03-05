
RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/simulator/pprzsim-launch -a ardrone2 -t nps'
RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/tools/gcs_launch.py '
RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/ground_segment/tmtc/server -n'
RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/ground_segment/tmtc/link -udp -udp_broadcast'
Simulating with dt of 0.000977
THX before no-fly-points 
start printing
Current schedparam: policy 0, prio 0
Depth 0: Node 1 has no neighbors.
done 
x: -26088.000000 y: -25118.000000 rad: 759.000000 THX denom 2.414214  end 
setschedparam failed!: Operation not permitted
THX after no-fly-points 
start printing
Depth 0: Node 1 (0.0, 0.0) has 3 neighbors: 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4)
start printing
Depth 1: Node 3 (-25773.6, -24359.0) has 4 neighbors: 2(-26402.4, -24359.0), 4(-25329.0, -24803.6), 9(-26847.0, -24803.6), 1(0.0, 0.0)
start printing
Depth 2: Node 2 (-26402.4, -24359.0) has 2 neighbors: 3(-25773.6, -24359.0), 8(-26847.0, -25432.4)
start printing
Depth 3: Node 8 (-26847.0, -25432.4) has 2 neighbors: 7(-26402.4, -25877.0), 2(-26402.4, -24359.0)
start printing
Depth 4: Node 7 (-26402.4, -25877.0) has 2 neighbors: 6(-25773.6, -25877.0), 8(-26847.0, -25432.4)
start printing
Depth 5: Node 6 (-25773.6, -25877.0) has 2 neighbors: 5(-25329.0, -25432.4), 7(-26402.4, -25877.0)
start printing
Depth 6: Node 5 (-25329.0, -25432.4) has 3 neighbors: 4(-25329.0, -24803.6), 6(-25773.6, -25877.0), 1(0.0, 0.0)
start printing
Depth 7: Node 4 (-25329.0, -24803.6) has 3 neighbors: 3(-25773.6, -24359.0), 5(-25329.0, -25432.4), 1(0.0, 0.0)
start printing
Depth 2: Node 9 has one neighbor: 3(-25773.6, -24359.0)
done 
THX after all 
start printing
Depth 0: Node 1 (0.0, 0.0) has 4 neighbors: 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 11(26088.0, 25118.0)
start printing
Depth 1: Node 3 (-25773.6, -24359.0) has 6 neighbors: 2(-26402.4, -24359.0), 4(-25329.0, -24803.6), 9(-26847.0, -24803.6), 1(0.0, 0.0), 10(-26088.0, -25118.0), 11(26088.0, 25118.0)
start printing
Depth 2: Node 2 (-26402.4, -24359.0) has 4 neighbors: 3(-25773.6, -24359.0), 8(-26847.0, -25432.4), 10(-26088.0, -25118.0), 11(26088.0, 25118.0)
start printing
Depth 3: Node 8 (-26847.0, -25432.4) has 3 neighbors: 7(-26402.4, -25877.0), 2(-26402.4, -24359.0), 10(-26088.0, -25118.0)
start printing
Depth 4: Node 7 (-26402.4, -25877.0) has 3 neighbors: 6(-25773.6, -25877.0), 8(-26847.0, -25432.4), 10(-26088.0, -25118.0)
start printing
Depth 5: Node 6 (-25773.6, -25877.0) has 4 neighbors: 5(-25329.0, -25432.4), 7(-26402.4, -25877.0), 10(-26088.0, -25118.0), 11(26088.0, 25118.0)
start printing
Depth 6: Node 5 (-25329.0, -25432.4) has 5 neighbors: 4(-25329.0, -24803.6), 6(-25773.6, -25877.0), 1(0.0, 0.0), 10(-26088.0, -25118.0), 11(26088.0, 25118.0)
start printing
Depth 7: Node 4 (-25329.0, -24803.6) has 5 neighbors: 3(-25773.6, -24359.0), 5(-25329.0, -25432.4), 1(0.0, 0.0), 10(-26088.0, -25118.0), 11(26088.0, 25118.0)
start printing
Depth 8: Node 10 (-26088.0, -25118.0) has 9 neighbors: 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 6(-25773.6, -25877.0), 7(-26402.4, -25877.0), 8(-26847.0, -25432.4), 9(-26847.0, -24803.6), 11(26088.0, 25118.0)
start printing
Depth 9: Node 9 (-26847.0, -24803.6) has 2 neighbors: 3(-25773.6, -24359.0), 10(-26088.0, -25118.0)
start printing
Depth 9: Node 11 (26088.0, 25118.0) has 7 neighbors: 1(0.0, 0.0), 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 6(-25773.6, -25877.0), 10(-26088.0, -25118.0)
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
after insert this node of that  (26088.0, 25118.0)
astar_insert************
printing the astar linked list:
(26088.0, 25118.0)->(-25329.0, -24803.6)->(-25773.6, -24359.0)->(-25329.0, -25432.4)->
astar_pop************
Reached the goal: (26088.0, 25118.0)
prepending (26088.0, 25118.0)
prepending (0.0, 0.0)
Finished calculating the path
Calculated path: 1(0.0, 0.0) 11(26088.0, 25118.0)
astar_free************
astar_free************
*************APR25************************************************************************8
Before the dest_x 1, y -1695965009 
After the dest_x 0, y 0 
shoulb be 30.179688
noflypoint after detection:************************************* 30.179688
reconstruct the graph*********************** print befroe recon:
start printing
Depth 0: Node 1 (0.0, 0.0) has 4 neighbors: 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 11(26088.0, 25118.0)
start printing
Depth 1: Node 3 (-25773.6, -24359.0) has 6 neighbors: 2(-26402.4, -24359.0), 4(-25329.0, -24803.6), 9(-26847.0, -24803.6), 1(0.0, 0.0), 10(-26088.0, -25118.0), 11(26088.0, 25118.0)
start printing
Depth 2: Node 2 (-26402.4, -24359.0) has 4 neighbors: 3(-25773.6, -24359.0), 8(-26847.0, -25432.4), 10(-26088.0, -25118.0), 11(26088.0, 25118.0)
start printing
Depth 3: Node 8 (-26847.0, -25432.4) has 3 neighbors: 7(-26402.4, -25877.0), 2(-26402.4, -24359.0), 10(-26088.0, -25118.0)
start printing
Depth 4: Node 7 (-26402.4, -25877.0) has 3 neighbors: 6(-25773.6, -25877.0), 8(-26847.0, -25432.4), 10(-26088.0, -25118.0)
start printing
Depth 5: Node 6 (-25773.6, -25877.0) has 4 neighbors: 5(-25329.0, -25432.4), 7(-26402.4, -25877.0), 10(-26088.0, -25118.0), 11(26088.0, 25118.0)
start printing
Depth 6: Node 5 (-25329.0, -25432.4) has 5 neighbors: 4(-25329.0, -24803.6), 6(-25773.6, -25877.0), 1(0.0, 0.0), 10(-26088.0, -25118.0), 11(26088.0, 25118.0)
start printing
Depth 7: Node 4 (-25329.0, -24803.6) has 5 neighbors: 3(-25773.6, -24359.0), 5(-25329.0, -25432.4), 1(0.0, 0.0), 10(-26088.0, -25118.0), 11(26088.0, 25118.0)
start printing
Depth 8: Node 10 (-26088.0, -25118.0) has 9 neighbors: 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 6(-25773.6, -25877.0), 7(-26402.4, -25877.0), 8(-26847.0, -25432.4), 9(-26847.0, -24803.6), 11(26088.0, 25118.0)
start printing
Depth 9: Node 9 (-26847.0, -24803.6) has 2 neighbors: 3(-25773.6, -24359.0), 10(-26088.0, -25118.0)
start printing
Depth 9: Node 11 (26088.0, 25118.0) has 7 neighbors: 1(0.0, 0.0), 2(-26402.4, -24359.0), 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4), 6(-25773.6, -25877.0), 10(-26088.0, -25118.0)
THX before no-fly-points 
start printing
Depth 0: Node 1 has no neighbors.
done 
x: 7726.000000 y: 11437.000000 rad: 759.000000 THX denom 2.414214  end 
THX after no-fly-points 
start printing
Depth 0: Node 1 (0.0, 0.0) has 5 neighbors: 5(8485.0, 11122.6), 6(8040.4, 10678.0), 7(7411.6, 10678.0), 8(6967.0, 11122.6), 9(6967.0, 11751.4)
start printing
Depth 1: Node 5 (8485.0, 11122.6) has 3 neighbors: 4(8485.0, 11751.4), 6(8040.4, 10678.0), 1(0.0, 0.0)
start printing
Depth 2: Node 4 (8485.0, 11751.4) has 2 neighbors: 3(8040.4, 12196.0), 5(8485.0, 11122.6)
start printing
Depth 3: Node 3 (8040.4, 12196.0) has 3 neighbors: 2(7411.6, 12196.0), 4(8485.0, 11751.4), 9(6967.0, 11751.4)
start printing
Depth 4: Node 2 (7411.6, 12196.0) has 2 neighbors: 3(8040.4, 12196.0), 8(6967.0, 11122.6)
start printing
Depth 5: Node 8 (6967.0, 11122.6) has 3 neighbors: 7(7411.6, 10678.0), 2(7411.6, 12196.0), 1(0.0, 0.0)
start printing
Depth 6: Node 7 (7411.6, 10678.0) has 3 neighbors: 6(8040.4, 10678.0), 8(6967.0, 11122.6), 1(0.0, 0.0)
start printing
Depth 7: Node 6 (8040.4, 10678.0) has 3 neighbors: 5(8485.0, 11122.6), 7(7411.6, 10678.0), 1(0.0, 0.0)
start printing
Depth 4: Node 9 (6967.0, 11751.4) has 2 neighbors: 3(8040.4, 12196.0), 1(0.0, 0.0)
done 
THX before adding the start node 
start printing
Depth 0: Node 1 (0.0, 0.0) has 6 neighbors: 5(8485.0, 11122.6), 6(8040.4, 10678.0), 7(7411.6, 10678.0), 8(6967.0, 11122.6), 9(6967.0, 11751.4), 11(26088.0, 25118.0)
start printing
Depth 1: Node 5 (8485.0, 11122.6) has 5 neighbors: 4(8485.0, 11751.4), 6(8040.4, 10678.0), 1(0.0, 0.0), 10(7726.0, 11437.0), 11(26088.0, 25118.0)
start printing
Depth 2: Node 4 (8485.0, 11751.4) has 4 neighbors: 3(8040.4, 12196.0), 5(8485.0, 11122.6), 10(7726.0, 11437.0), 11(26088.0, 25118.0)
start printing
Depth 3: Node 3 (8040.4, 12196.0) has 5 neighbors: 2(7411.6, 12196.0), 4(8485.0, 11751.4), 9(6967.0, 11751.4), 10(7726.0, 11437.0), 11(26088.0, 25118.0)
start printing
Depth 4: Node 2 (7411.6, 12196.0) has 4 neighbors: 3(8040.4, 12196.0), 8(6967.0, 11122.6), 10(7726.0, 11437.0), 11(26088.0, 25118.0)
start printing
Depth 5: Node 8 (6967.0, 11122.6) has 4 neighbors: 7(7411.6, 10678.0), 2(7411.6, 12196.0), 1(0.0, 0.0), 10(7726.0, 11437.0)
start printing
Depth 6: Node 7 (7411.6, 10678.0) has 4 neighbors: 6(8040.4, 10678.0), 8(6967.0, 11122.6), 1(0.0, 0.0), 10(7726.0, 11437.0)
start printing
Depth 7: Node 6 (8040.4, 10678.0) has 5 neighbors: 5(8485.0, 11122.6), 7(7411.6, 10678.0), 1(0.0, 0.0), 10(7726.0, 11437.0), 11(26088.0, 25118.0)
start printing
Depth 8: Node 10 (7726.0, 11437.0) has 9 neighbors: 2(7411.6, 12196.0), 3(8040.4, 12196.0), 4(8485.0, 11751.4), 5(8485.0, 11122.6), 6(8040.4, 10678.0), 7(7411.6, 10678.0), 8(6967.0, 11122.6), 9(6967.0, 11751.4), 11(26088.0, 25118.0)
start printing
Depth 9: Node 9 (6967.0, 11751.4) has 3 neighbors: 3(8040.4, 12196.0), 1(0.0, 0.0), 10(7726.0, 11437.0)
start printing
Depth 9: Node 11 (26088.0, 25118.0) has 7 neighbors: 1(0.0, 0.0), 2(7411.6, 12196.0), 3(8040.4, 12196.0), 4(8485.0, 11751.4), 5(8485.0, 11122.6), 6(8040.4, 10678.0), 10(7726.0, 11437.0)
done 
after recon********* 
start printing
Depth 0: Node 1 (0.0, 0.0) has 7 neighbors: 5(8485.0, 11122.6), 6(8040.4, 10678.0), 7(7411.6, 10678.0), 8(6967.0, 11122.6), 9(6967.0, 11751.4), 11(26088.0, 25118.0), 12(8561.5, 8242.3)
start printing
Depth 1: Node 5 (8485.0, 11122.6) has 6 neighbors: 4(8485.0, 11751.4), 6(8040.4, 10678.0), 1(0.0, 0.0), 10(7726.0, 11437.0), 11(26088.0, 25118.0), 12(8561.5, 8242.3)
start printing
Depth 2: Node 4 (8485.0, 11751.4) has 5 neighbors: 3(8040.4, 12196.0), 5(8485.0, 11122.6), 10(7726.0, 11437.0), 11(26088.0, 25118.0), 12(8561.5, 8242.3)
start printing
Depth 3: Node 3 (8040.4, 12196.0) has 5 neighbors: 2(7411.6, 12196.0), 4(8485.0, 11751.4), 9(6967.0, 11751.4), 10(7726.0, 11437.0), 11(26088.0, 25118.0)
start printing
Depth 4: Node 2 (7411.6, 12196.0) has 4 neighbors: 3(8040.4, 12196.0), 8(6967.0, 11122.6), 10(7726.0, 11437.0), 11(26088.0, 25118.0)
start printing
Depth 5: Node 8 (6967.0, 11122.6) has 5 neighbors: 7(7411.6, 10678.0), 2(7411.6, 12196.0), 1(0.0, 0.0), 10(7726.0, 11437.0), 12(8561.5, 8242.3)
start printing
Depth 6: Node 7 (7411.6, 10678.0) has 5 neighbors: 6(8040.4, 10678.0), 8(6967.0, 11122.6), 1(0.0, 0.0), 10(7726.0, 11437.0), 12(8561.5, 8242.3)
start printing
Depth 7: Node 6 (8040.4, 10678.0) has 6 neighbors: 5(8485.0, 11122.6), 7(7411.6, 10678.0), 1(0.0, 0.0), 10(7726.0, 11437.0), 11(26088.0, 25118.0), 12(8561.5, 8242.3)
start printing
Depth 8: Node 10 (7726.0, 11437.0) has 9 neighbors: 2(7411.6, 12196.0), 3(8040.4, 12196.0), 4(8485.0, 11751.4), 5(8485.0, 11122.6), 6(8040.4, 10678.0), 7(7411.6, 10678.0), 8(6967.0, 11122.6), 9(6967.0, 11751.4), 11(26088.0, 25118.0)
start printing
Depth 9: Node 9 (6967.0, 11751.4) has 3 neighbors: 3(8040.4, 12196.0), 1(0.0, 0.0), 10(7726.0, 11437.0)
start printing
Depth 9: Node 11 (26088.0, 25118.0) has 7 neighbors: 1(0.0, 0.0), 2(7411.6, 12196.0), 3(8040.4, 12196.0), 4(8485.0, 11751.4), 5(8485.0, 11122.6), 6(8040.4, 10678.0), 10(7726.0, 11437.0)
start printing
Depth 8: Node 12 (8561.5, 8242.3) has 6 neighbors: 1(0.0, 0.0), 4(8485.0, 11751.4), 5(8485.0, 11122.6), 6(8040.4, 10678.0), 7(7411.6, 10678.0), 8(6967.0, 11122.6)
out of the reconstruct
start rebuild Path
here we are going to (0,0)?
the dest_x 1, y -1695965009 
Using A* to find the shortest path from (8561.5, 8242.3) to (26088.0, 25118.0)
printing the astar linked list:
(8561.5, 8242.3)->
astar_pop************
Now considering (8561.5, 8242.3)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (8485.0, 11751.4)
astar_insert************
after insert this node of that  (8485.0, 11122.6)
astar_insert************
after insert this node of that  (8040.4, 10678.0)
astar_insert************
after insert this node of that  (7411.6, 10678.0)
astar_insert************
after insert this node of that  (6967.0, 11122.6)
astar_insert************
printing the astar linked list:
(8485.0, 11122.6)->(8040.4, 10678.0)->(8485.0, 11751.4)->(7411.6, 10678.0)->(6967.0, 11122.6)->(0.0, 0.0)->
astar_pop************
Now considering (8485.0, 11122.6)
astar_insert************
after insert this node of that  (8485.0, 11751.4)
astar_insert************
after insert this node of that  (8040.4, 10678.0)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (7726.0, 11437.0)
astar_insert************
after insert this node of that  (26088.0, 25118.0)
astar_insert************
after insert this node of that  (8561.5, 8242.3)
astar_insert************
printing the astar linked list:
(26088.0, 25118.0)->(8040.4, 10678.0)->(8485.0, 11751.4)->(8485.0, 11751.4)->(7411.6, 10678.0)->(7726.0, 11437.0)->(8040.4, 10678.0)->(6967.0, 11122.6)->(8561.5, 8242.3)->(0.0, 0.0)->(0.0, 0.0)->
astar_pop************
Reached the goal: (26088.0, 25118.0)
prepending (26088.0, 25118.0)
prepending (8485.0, 11122.6)
prepending (8561.5, 8242.3)
Finished calculating the path
Calculated path: 12(8561.5, 8242.3) 5(8485.0, 11122.6) 11(26088.0, 25118.0)
astar_free************
astar_free************
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.332031
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.554688
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.523438
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.156250
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.750000
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.511719
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.375000
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.359375
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.328125
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.507812
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.488281
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.628906
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.703125
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.636719
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.609375
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.632812
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.707031
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.890625
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.906250
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.910156
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.863281
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.847656
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.859375
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.921875
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.976562
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 29.996094
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.003906
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.011719
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.027344
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.089844
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.093750
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.125000
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.152344
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.164062
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.160156
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.210938
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.261719
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.285156
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.312500
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.363281
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.398438
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.425781
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.472656
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.500000
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.546875
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.593750
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.613281
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.656250
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.722656
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.753906
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.796875
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.867188
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.910156
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 30.964844
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.003906
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.058594
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.132812
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.191406
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.296875
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.371094
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.488281
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.968750
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.921875
here we cahnge the dest_x in the NavSegment.
Using A* to find the shortest path from (0.0, 0.0) to (26088.0, 25118.0)
printing the astar linked list:
(0.0, 0.0)->
astar_pop************
Now considering (0.0, 0.0)
astar_insert************
after insert this node of that  (8485.0, 11122.6)
astar_insert************
after insert this node of that  (8040.4, 10678.0)
astar_insert************
after insert this node of that  (7411.6, 10678.0)
astar_insert************
after insert this node of that  (6967.0, 11122.6)
astar_insert************
after insert this node of that  (6967.0, 11751.4)
astar_insert************
after insert this node of that  (26088.0, 25118.0)
astar_insert************
after insert this node of that  (8561.5, 8242.3)
astar_insert************
printing the astar linked list:
(26088.0, 25118.0)->(8561.5, 8242.3)->(8485.0, 11122.6)->(8040.4, 10678.0)->(7411.6, 10678.0)->(6967.0, 11122.6)->(6967.0, 11751.4)->
astar_pop************
Reached the goal: (26088.0, 25118.0)
prepending (26088.0, 25118.0)
prepending (0.0, 0.0)
Finished calculating the path
Calculated path: 1(0.0, 0.0) 11(26088.0, 25118.0)
astar_free************
astar_free************
Using A* to find the shortest path from (26088.0, 25118.0) to (0.0, 0.0)
printing the astar linked list:
(26088.0, 25118.0)->
astar_pop************
Now considering (26088.0, 25118.0)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (7411.6, 12196.0)
astar_insert************
after insert this node of that  (8040.4, 12196.0)
astar_insert************
after insert this node of that  (8485.0, 11751.4)
astar_insert************
after insert this node of that  (8485.0, 11122.6)
astar_insert************
after insert this node of that  (8040.4, 10678.0)
astar_insert************
after insert this node of that  (7726.0, 11437.0)
astar_insert************
printing the astar linked list:
(0.0, 0.0)->(8485.0, 11122.6)->(8040.4, 10678.0)->(8485.0, 11751.4)->(7726.0, 11437.0)->(8040.4, 12196.0)->(7411.6, 12196.0)->
astar_pop************
Reached the goal: (0.0, 0.0)
prepending (0.0, 0.0)
prepending (26088.0, 25118.0)
Finished calculating the path
Calculated path: 11(26088.0, 25118.0) 1(0.0, 0.0)
astar_free************
astar_free************
*************APR25************************************************************************8
Before the dest_x 1, y -1695965009 
After the dest_x 0, y 0 
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.804688
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.667969
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.593750
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.496094
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.394531
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.332031
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.250000
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.187500
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.136719
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.085938
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.023438
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 32.000000
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.937500
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.906250
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.863281
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.773438
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.753906
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.703125
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.683594
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.644531
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.617188
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.609375
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.617188
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.554688
Judge no reconstructing, nofly_before_x 30.179688, nofly_after_x 31.535156