noflypoint : not move
without that code 

ONE 'make -C /home/thx/orangePaparazzi/paparazziNewGCS -f Makefile.ac AIRCRAFT=ardrone2 nps.compile'

RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/simulator/pprzsim-launch -a ardrone2 -t nps'
RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/tools/gcs_launch.py '
RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/ground_segment/tmtc/server -n'
RUN '/home/thx/orangePaparazzi/paparazziNewGCS/sw/ground_segment/tmtc/link -udp -udp_broadcast'
Simulating with dt of 0.000977
THX before no-fly-points 
start printing
Depth 0: Node 1 has no neighbors.
done 
x: -26088.000000 y: -25118.000000 rad: 759.000000 THX denom 2.414214  end 
Current schedparam: policy 0, prio 0
THX after no-fly-points 
start printing
Depth 0: Node 1 (0.0, 0.0) has 3 neighbors: 3(-25773.6, -24359.0), 4(-25329.0, -24803.6), 5(-25329.0, -25432.4)
start printing
Depth 1: Node 3 (-25773.6, -24359.0) has 4 neighbors: 2(-26402.4, -24359.0), setschedparam failed!: Operation not permitted
4(-25329.0, -24803.6), 9(-26847.0, -24803.6), 1(0.0, 0.0)
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
Before the dest_x 1, y -1020194641 
After the dest_x 0, y 0 
shoulb be 44.503906
noflypoint after detection:************************************* 44.503906
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
x: 11393.000000 y: 11473.000000 rad: 759.000000 THX denom 2.414214  end 
THX after no-fly-points 
start printing
Depth 0: Node 1 (0.0, 0.0) has 3 neighbors: 7(11078.6, 10714.0), 8(10634.0, 11158.6), 9(10634.0, 11787.4)
start printing
Depth 1: Node 7 (11078.6, 10714.0) has 3 neighbors: 6(11707.4, 10714.0), 8(10634.0, 11158.6), 1(0.0, 0.0)
start printing
Depth 2: Node 6 (11707.4, 10714.0) has 2 neighbors: 5(12152.0, 11158.6), 7(11078.6, 10714.0)
start printing
Depth 3: Node 5 (12152.0, 11158.6) has 2 neighbors: 4(12152.0, 11787.4), 6(11707.4, 10714.0)
start printing
Depth 4: Node 4 (12152.0, 11787.4) has 2 neighbors: 3(11707.4, 12232.0), 5(12152.0, 11158.6)
start printing
Depth 5: Node 3 (11707.4, 12232.0) has 3 neighbors: 2(11078.6, 12232.0), 4(12152.0, 11787.4), 9(10634.0, 11787.4)
start printing
Depth 6: Node 2 (11078.6, 12232.0) has 2 neighbors: 3(11707.4, 12232.0), 8(10634.0, 11158.6)
start printing
Depth 7: Node 8 (10634.0, 11158.6) has 3 neighbors: 7(11078.6, 10714.0), 2(11078.6, 12232.0), 1(0.0, 0.0)
start printing
Depth 6: Node 9 (10634.0, 11787.4) has 2 neighbors: 3(11707.4, 12232.0), 1(0.0, 0.0)
done 
THX before adding the start node 
start printing
Depth 0: Node 1 (0.0, 0.0) has 3 neighbors: 7(11078.6, 10714.0), 8(10634.0, 11158.6), 9(10634.0, 11787.4)
start printing
Depth 1: Node 7 (11078.6, 10714.0) has 4 neighbors: 6(11707.4, 10714.0), 8(10634.0, 11158.6), 1(0.0, 0.0), 10(11393.0, 11473.0)
start printing
Depth 2: Node 6 (11707.4, 10714.0) has 3 neighbors: 5(12152.0, 11158.6), 7(11078.6, 10714.0), 10(11393.0, 11473.0)
start printing
Depth 3: Node 5 (12152.0, 11158.6) has 4 neighbors: 4(12152.0, 11787.4), 6(11707.4, 10714.0), 10(11393.0, 11473.0), 11(26088.0, 25118.0)
start printing
Depth 4: Node 4 (12152.0, 11787.4) has 4 neighbors: 3(11707.4, 12232.0), 5(12152.0, 11158.6), 10(11393.0, 11473.0), 11(26088.0, 25118.0)
start printing
Depth 5: Node 3 (11707.4, 12232.0) has 5 neighbors: 2(11078.6, 12232.0), 4(12152.0, 11787.4), 9(10634.0, 11787.4), 10(11393.0, 11473.0), 11(26088.0, 25118.0)
start printing
Depth 6: Node 2 (11078.6, 12232.0) has 4 neighbors: 3(11707.4, 12232.0), 8(10634.0, 11158.6), 10(11393.0, 11473.0), 11(26088.0, 25118.0)
start printing
Depth 7: Node 8 (10634.0, 11158.6) has 4 neighbors: 7(11078.6, 10714.0), 2(11078.6, 12232.0), 1(0.0, 0.0), 10(11393.0, 11473.0)
start printing
Depth 8: Node 10 (11393.0, 11473.0) has 9 neighbors: 2(11078.6, 12232.0), 3(11707.4, 12232.0), 4(12152.0, 11787.4), 5(12152.0, 11158.6), 6(11707.4, 10714.0), 7(11078.6, 10714.0), 8(10634.0, 11158.6), 9(10634.0, 11787.4), 11(26088.0, 25118.0)
start printing
Depth 9: Node 9 (10634.0, 11787.4) has 3 neighbors: 3(11707.4, 12232.0), 1(0.0, 0.0), 10(11393.0, 11473.0)
start printing
Depth 9: Node 11 (26088.0, 25118.0) has 5 neighbors: 2(11078.6, 12232.0), 3(11707.4, 12232.0), 4(12152.0, 11787.4), 5(12152.0, 11158.6), 10(11393.0, 11473.0)
done 
after recon********* 
start printing
Depth 0: Node 1 (0.0, 0.0) has 4 neighbors: 7(11078.6, 10714.0), 8(10634.0, 11158.6), 9(10634.0, 11787.4), 12(9471.7, 9122.8)
start printing
Depth 1: Node 7 (11078.6, 10714.0) has 5 neighbors: 6(11707.4, 10714.0), 8(10634.0, 11158.6), 1(0.0, 0.0), 10(11393.0, 11473.0), 12(9471.7, 9122.8)
start printing
Depth 2: Node 6 (11707.4, 10714.0) has 4 neighbors: 5(12152.0, 11158.6), 7(11078.6, 10714.0), 10(11393.0, 11473.0), 12(9471.7, 9122.8)
start printing
Depth 3: Node 5 (12152.0, 11158.6) has 4 neighbors: 4(12152.0, 11787.4), 6(11707.4, 10714.0), 10(11393.0, 11473.0), 11(26088.0, 25118.0)
start printing
Depth 4: Node 4 (12152.0, 11787.4) has 4 neighbors: 3(11707.4, 12232.0), 5(12152.0, 11158.6), 10(11393.0, 11473.0), 11(26088.0, 25118.0)
start printing
Depth 5: Node 3 (11707.4, 12232.0) has 5 neighbors: 2(11078.6, 12232.0), 4(12152.0, 11787.4), 9(10634.0, 11787.4), 10(11393.0, 11473.0), 11(26088.0, 25118.0)
start printing
Depth 6: Node 2 (11078.6, 12232.0) has 4 neighbors: 3(11707.4, 12232.0), 8(10634.0, 11158.6), 10(11393.0, 11473.0), 11(26088.0, 25118.0)
start printing
Depth 7: Node 8 (10634.0, 11158.6) has 5 neighbors: 7(11078.6, 10714.0), 2(11078.6, 12232.0), 1(0.0, 0.0), 10(11393.0, 11473.0), 12(9471.7, 9122.8)
start printing
Depth 8: Node 10 (11393.0, 11473.0) has 9 neighbors: 2(11078.6, 12232.0), 3(11707.4, 12232.0), 4(12152.0, 11787.4), 5(12152.0, 11158.6), 6(11707.4, 10714.0), 7(11078.6, 10714.0), 8(10634.0, 11158.6), 9(10634.0, 11787.4), 11(26088.0, 25118.0)
start printing
Depth 9: Node 9 (10634.0, 11787.4) has 4 neighbors: 3(11707.4, 12232.0), 1(0.0, 0.0), 10(11393.0, 11473.0), 12(9471.7, 9122.8)
start printing
Depth 10: Node 12 (9471.7, 9122.8) has 5 neighbors: 1(0.0, 0.0), 6(11707.4, 10714.0), 7(11078.6, 10714.0), 8(10634.0, 11158.6), 9(10634.0, 11787.4)
start printing
Depth 9: Node 11 (26088.0, 25118.0) has 5 neighbors: 2(11078.6, 12232.0), 3(11707.4, 12232.0), 4(12152.0, 11787.4), 5(12152.0, 11158.6), 10(11393.0, 11473.0)
out of the reconstruct
start rebuild Path
here we are going to (0,0)?
the dest_x 1, y -1020194641 
Using A* to find the shortest path from (9471.7, 9122.8) to (26088.0, 25118.0)
printing the astar linked list:
(9471.7, 9122.8)->
astar_pop************
Now considering (9471.7, 9122.8)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (11707.4, 10714.0)
astar_insert************
after insert this node of that  (11078.6, 10714.0)
astar_insert************
after insert this node of that  (10634.0, 11158.6)
astar_insert************
after insert this node of that  (10634.0, 11787.4)
astar_insert************
printing the astar linked list:
(11078.6, 10714.0)->(11707.4, 10714.0)->(10634.0, 11158.6)->(10634.0, 11787.4)->(0.0, 0.0)->
astar_pop************
Now considering (11078.6, 10714.0)
astar_insert************
after insert this node of that  (11707.4, 10714.0)
astar_insert************
after insert this node of that  (10634.0, 11158.6)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (9471.7, 9122.8)
astar_insert************
printing the astar linked list:
(11707.4, 10714.0)->(11393.0, 11473.0)->(10634.0, 11158.6)->(11707.4, 10714.0)->(10634.0, 11787.4)->(10634.0, 11158.6)->(9471.7, 9122.8)->(0.0, 0.0)->(0.0, 0.0)->
astar_pop************
Now considering (11707.4, 10714.0)
astar_insert************
after insert this node of that  (12152.0, 11158.6)
astar_insert************
after insert this node of that  (11078.6, 10714.0)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (9471.7, 9122.8)
astar_insert************
printing the astar linked list:
(12152.0, 11158.6)->(11393.0, 11473.0)->(10634.0, 11158.6)->(11707.4, 10714.0)->(10634.0, 11787.4)->(11393.0, 11473.0)->(10634.0, 11158.6)->(11078.6, 10714.0)->(9471.7, 9122.8)->(9471.7, 9122.8)->(0.0, 0.0)->(0.0, 0.0)->
astar_pop************
Now considering (12152.0, 11158.6)
astar_insert************
after insert this node of that  (12152.0, 11787.4)
astar_insert************
after insert this node of that  (11707.4, 10714.0)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (26088.0, 25118.0)
astar_insert************
memory-leak********************************************
printing the astar linked list:
(26088.0, 25118.0)->(11393.0, 11473.0)->(10634.0, 11158.6)->(11707.4, 10714.0)->(12152.0, 11787.4)->(10634.0, 11787.4)->(11393.0, 11473.0)->(10634.0, 11158.6)->(11078.6, 10714.0)->(11393.0, 11473.0)->(11707.4, 10714.0)->(9471.7, 9122.8)->(9471.7, 9122.8)->(0.0, 0.0)->(0.0, 0.0)->
astar_pop************
Reached the goal: (26088.0, 25118.0)
prepending (26088.0, 25118.0)
prepending (12152.0, 11158.6)
prepending (11707.4, 10714.0)
prepending (9471.7, 9122.8)
Finished calculating the path
Calculated path: 12(9471.7, 9122.8) 6(11707.4, 10714.0) 5(12152.0, 11158.6) 11(26088.0, 25118.0)
astar_free************
astar_free************
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.589844
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.582031
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.496094
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.562500
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.839844
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.835938
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.914062
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.882812
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.730469
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.675781
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.542969
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.480469
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.609375
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.625000
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.691406
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.753906
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.703125
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.695312
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.687500
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.777344
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.761719
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.781250
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.789062
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.796875
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.792969
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.812500
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.855469
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.933594
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.925781
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.949219
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.960938
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.968750
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 44.964844
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.015625
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.046875
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.078125
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.070312
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.093750
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.113281
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.128906
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.148438
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.207031
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.183594
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.214844
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.253906
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.316406
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.308594
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.332031
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.367188
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.406250
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.421875
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.460938
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.484375
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.488281
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.523438
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.562500
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.640625
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.707031
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.761719
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.808594
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 45.890625
Judge no reconstructing, nofly_before_x 44.503906, nofly_after_x 46.125000
here we cahnge the dest_x in the NavSegment.
Using A* to find the shortest path from (0.0, 0.0) to (26088.0, 25118.0)
printing the astar linked list:
(0.0, 0.0)->
astar_pop************
Now considering (0.0, 0.0)
astar_insert************
after insert this node of that  (11078.6, 10714.0)
astar_insert************
after insert this node of that  (10634.0, 11158.6)
astar_insert************
after insert this node of that  (10634.0, 11787.4)
astar_insert************
after insert this node of that  (9471.7, 9122.8)
astar_insert************
printing the astar linked list:
(9471.7, 9122.8)->(11078.6, 10714.0)->(10634.0, 11158.6)->(10634.0, 11787.4)->
astar_pop************
Now considering (9471.7, 9122.8)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (11707.4, 10714.0)
astar_insert************
after insert this node of that  (11078.6, 10714.0)
astar_insert************
after insert this node of that  (10634.0, 11158.6)
astar_insert************
after insert this node of that  (10634.0, 11787.4)
astar_insert************
printing the astar linked list:
(11078.6, 10714.0)->(11078.6, 10714.0)->(10634.0, 11158.6)->(11707.4, 10714.0)->(10634.0, 11787.4)->(10634.0, 11158.6)->(10634.0, 11787.4)->(0.0, 0.0)->
astar_pop************
Now considering (11078.6, 10714.0)
astar_insert************
after insert this node of that  (11707.4, 10714.0)
astar_insert************
after insert this node of that  (10634.0, 11158.6)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (9471.7, 9122.8)
astar_insert************
printing the astar linked list:
(11078.6, 10714.0)->(10634.0, 11158.6)->(11707.4, 10714.0)->(10634.0, 11787.4)->(11393.0, 11473.0)->(10634.0, 11158.6)->(11707.4, 10714.0)->(10634.0, 11787.4)->(10634.0, 11158.6)->(9471.7, 9122.8)->(0.0, 0.0)->(0.0, 0.0)->
astar_pop************
Now considering (11078.6, 10714.0)
astar_insert************
after insert this node of that  (11707.4, 10714.0)
astar_insert************
after insert this node of that  (10634.0, 11158.6)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (9471.7, 9122.8)
astar_insert************
printing the astar linked list:
(10634.0, 11158.6)->(11707.4, 10714.0)->(10634.0, 11787.4)->(11393.0, 11473.0)->(11393.0, 11473.0)->(10634.0, 11158.6)->(11707.4, 10714.0)->(11707.4, 10714.0)->(10634.0, 11787.4)->(10634.0, 11158.6)->(10634.0, 11158.6)->(9471.7, 9122.8)->(9471.7, 9122.8)->(0.0, 0.0)->(0.0, 0.0)->(0.0, 0.0)->
astar_pop************
Now considering (10634.0, 11158.6)
astar_insert************
after insert this node of that  (11078.6, 10714.0)
astar_insert************
after insert this node of that  (11078.6, 12232.0)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (9471.7, 9122.8)
astar_insert************
printing the astar linked list:
(11707.4, 10714.0)->(10634.0, 11787.4)->(11393.0, 11473.0)->(11393.0, 11473.0)->(11393.0, 11473.0)->(10634.0, 11158.6)->(11078.6, 12232.0)->(11707.4, 10714.0)->(11707.4, 10714.0)->(10634.0, 11787.4)->(11078.6, 10714.0)->(10634.0, 11158.6)->(10634.0, 11158.6)->(9471.7, 9122.8)->(9471.7, 9122.8)->(9471.7, 9122.8)->(0.0, 0.0)->(0.0, 0.0)->(0.0, 0.0)->(0.0, 0.0)->
astar_pop************
Now considering (11707.4, 10714.0)
astar_insert************
after insert this node of that  (12152.0, 11158.6)
astar_insert************
after insert this node of that  (11078.6, 10714.0)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (9471.7, 9122.8)
astar_insert************
printing the astar linked list:
(12152.0, 11158.6)->(10634.0, 11787.4)->(11393.0, 11473.0)->(11393.0, 11473.0)->(11393.0, 11473.0)->(10634.0, 11158.6)->(11078.6, 12232.0)->(11707.4, 10714.0)->(11707.4, 10714.0)->(10634.0, 11787.4)->(11393.0, 11473.0)->(11078.6, 10714.0)->(10634.0, 11158.6)->(10634.0, 11158.6)->(11078.6, 10714.0)->(9471.7, 9122.8)->(9471.7, 9122.8)->(9471.7, 9122.8)->(9471.7, 9122.8)->(0.0, 0.0)->
astar_pop************
Now considering (12152.0, 11158.6)
astar_insert************
after insert this node of that  (12152.0, 11787.4)
astar_insert************
after insert this node of that  (11707.4, 10714.0)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (26088.0, 25118.0)
astar_insert************
printing the astar linked list:
(26088.0, 25118.0)->(10634.0, 11787.4)->(11393.0, 11473.0)->(11393.0, 11473.0)->(11393.0, 11473.0)->(10634.0, 11158.6)->(11078.6, 12232.0)->(11707.4, 10714.0)->(11707.4, 10714.0)->(12152.0, 11787.4)->(10634.0, 11787.4)->(11393.0, 11473.0)->(11078.6, 10714.0)->(10634.0, 11158.6)->(10634.0, 11158.6)->(11078.6, 10714.0)->(11393.0, 11473.0)->(11707.4, 10714.0)->(9471.7, 9122.8)->(9471.7, 9122.8)->
astar_pop************
Reached the goal: (26088.0, 25118.0)
prepending (26088.0, 25118.0)
prepending (12152.0, 11158.6)
prepending (11707.4, 10714.0)
prepending (9471.7, 9122.8)
prepending (0.0, 0.0)
Finished calculating the path
Calculated path: 1(0.0, 0.0) 12(9471.7, 9122.8) 6(11707.4, 10714.0) 5(12152.0, 11158.6) 11(26088.0, 25118.0)
astar_free************
astar_free************
Using A* to find the shortest path from (26088.0, 25118.0) to (0.0, 0.0)
printing the astar linked list:
(26088.0, 25118.0)->
astar_pop************
Now considering (26088.0, 25118.0)
astar_insert************
after insert this node of that  (11078.6, 12232.0)
astar_insert************
after insert this node of that  (11707.4, 12232.0)
astar_insert************
after insert this node of that  (12152.0, 11787.4)
astar_insert************
after insert this node of that  (12152.0, 11158.6)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
printing the astar linked list:
(12152.0, 11787.4)->(11393.0, 11473.0)->(12152.0, 11158.6)->(11707.4, 12232.0)->(11078.6, 12232.0)->
astar_pop************
Now considering (12152.0, 11787.4)
astar_insert************
after insert this node of that  (11707.4, 12232.0)
astar_insert************
after insert this node of that  (12152.0, 11158.6)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (26088.0, 25118.0)
astar_insert************
printing the astar linked list:
(11393.0, 11473.0)->(12152.0, 11158.6)->(11707.4, 12232.0)->(11393.0, 11473.0)->(11078.6, 12232.0)->(12152.0, 11158.6)->(11707.4, 12232.0)->(26088.0, 25118.0)->
astar_pop************
Now considering (11393.0, 11473.0)
astar_insert************
after insert this node of that  (11078.6, 12232.0)
astar_insert************
after insert this node of that  (11707.4, 12232.0)
astar_insert************
after insert this node of that  (12152.0, 11787.4)
astar_insert************
after insert this node of that  (12152.0, 11158.6)
astar_insert************
after insert this node of that  (11707.4, 10714.0)
astar_insert************
after insert this node of that  (11078.6, 10714.0)
astar_insert************
after insert this node of that  (10634.0, 11158.6)
astar_insert************
after insert this node of that  (10634.0, 11787.4)
astar_insert************
after insert this node of that  (26088.0, 25118.0)
astar_insert************
printing the astar linked list:
(12152.0, 11158.6)->(11707.4, 12232.0)->(11393.0, 11473.0)->(11078.6, 12232.0)->(11078.6, 10714.0)->(10634.0, 11158.6)->(12152.0, 11158.6)->(11707.4, 10714.0)->(10634.0, 11787.4)->(11707.4, 12232.0)->(12152.0, 11158.6)->(11078.6, 12232.0)->(12152.0, 11787.4)->(11707.4, 12232.0)->(26088.0, 25118.0)->(26088.0, 25118.0)->
astar_pop************
Now considering (12152.0, 11158.6)
astar_insert************
after insert this node of that  (12152.0, 11787.4)
astar_insert************
after insert this node of that  (11707.4, 10714.0)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (26088.0, 25118.0)
astar_insert************
printing the astar linked list:
(11707.4, 10714.0)->(11707.4, 12232.0)->(11393.0, 11473.0)->(11078.6, 12232.0)->(11078.6, 10714.0)->(10634.0, 11158.6)->(12152.0, 11158.6)->(11393.0, 11473.0)->(11707.4, 10714.0)->(10634.0, 11787.4)->(11707.4, 12232.0)->(12152.0, 11787.4)->(12152.0, 11158.6)->(11078.6, 12232.0)->(12152.0, 11787.4)->(11707.4, 12232.0)->(26088.0, 25118.0)->(26088.0, 25118.0)->(26088.0, 25118.0)->
astar_pop************
Now considering (11707.4, 10714.0)
astar_insert************
after insert this node of that  (12152.0, 11158.6)
astar_insert************
after insert this node of that  (11078.6, 10714.0)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (9471.7, 9122.8)
astar_insert************
printing the astar linked list:
(11707.4, 12232.0)->(9471.7, 9122.8)->(11393.0, 11473.0)->(11078.6, 12232.0)->(11078.6, 10714.0)->(10634.0, 11158.6)->(11078.6, 10714.0)->(12152.0, 11158.6)->(11393.0, 11473.0)->(11707.4, 10714.0)->(10634.0, 11787.4)->(11707.4, 12232.0)->(12152.0, 11787.4)->(11393.0, 11473.0)->(12152.0, 11158.6)->(11078.6, 12232.0)->(12152.0, 11158.6)->(12152.0, 11787.4)->(11707.4, 12232.0)->(26088.0, 25118.0)->
astar_pop************
Now considering (11707.4, 12232.0)
astar_insert************
after insert this node of that  (11078.6, 12232.0)
astar_insert************
after insert this node of that  (12152.0, 11787.4)
astar_insert************
after insert this node of that  (10634.0, 11787.4)
astar_insert************
after insert this node of that  (11393.0, 11473.0)
astar_insert************
after insert this node of that  (26088.0, 25118.0)
astar_insert************
printing the astar linked list:
(9471.7, 9122.8)->(11393.0, 11473.0)->(11078.6, 12232.0)->(11078.6, 10714.0)->(10634.0, 11158.6)->(11393.0, 11473.0)->(10634.0, 11787.4)->(11078.6, 10714.0)->(12152.0, 11158.6)->(11078.6, 12232.0)->(11393.0, 11473.0)->(11707.4, 10714.0)->(10634.0, 11787.4)->(11707.4, 12232.0)->(12152.0, 11787.4)->(12152.0, 11787.4)->(11393.0, 11473.0)->(12152.0, 11158.6)->(11078.6, 12232.0)->(12152.0, 11158.6)->
astar_pop************
Now considering (9471.7, 9122.8)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (11707.4, 10714.0)
astar_insert************
after insert this node of that  (11078.6, 10714.0)
astar_insert************
after insert this node of that  (10634.0, 11158.6)
astar_insert************
after insert this node of that  (10634.0, 11787.4)
astar_insert************
printing the astar linked list:
(0.0, 0.0)->(11393.0, 11473.0)->(11078.6, 12232.0)->(11078.6, 10714.0)->(10634.0, 11158.6)->(11393.0, 11473.0)->(10634.0, 11787.4)->(11078.6, 10714.0)->(12152.0, 11158.6)->(11078.6, 12232.0)->(11393.0, 11473.0)->(11707.4, 10714.0)->(10634.0, 11787.4)->(11707.4, 12232.0)->(12152.0, 11787.4)->(12152.0, 11787.4)->(11393.0, 11473.0)->(12152.0, 11158.6)->(11078.6, 12232.0)->(12152.0, 11158.6)->
astar_pop************
Reached the goal: (0.0, 0.0)
prepending (0.0, 0.0)
prepending (9471.7, 9122.8)
prepending (11707.4, 10714.0)
prepending (12152.0, 11158.6)
prepending (26088.0, 25118.0)
Finished calculating the path
Calculated path: 11(26088.0, 25118.0) 5(12152.0, 11158.6) 6(11707.4, 10714.0) 12(9471.7, 9122.8) 1(0.0, 0.0)
astar_free************
astar_free************
*************APR25************************************************************************8
Before the dest_x 1, y -1020194641 
After the dest_x 0, y 0 
