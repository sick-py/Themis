
DONE 'make -C /home/thx/orangePaparazzi/paparazziNewGCS -f Makefile.ac AIRCRAFT=ardrone2 nps.compile'

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
x: 11740.000000 y: 11302.000000 rad: 759.000000 THX denom 2.414214  end 
THX after no-fly-points 
start printing
Depth 0: Node 1 (0.0, 0.0) has 4 neighbors: 6(12054.4, 10543.0), 7(11425.6, 10543.0), 8(10981.0, 10987.6), 9(10981.0, 11616.4)
start printing
Depth 1: Node 6 (12054.4, 10543.0) has 3 neighbors: 5(12499.0, 10987.6), 7(11425.6, 10543.0), 1(0.0, 0.0)
start printing
Depth 2: Node 5 (12499.0, 10987.6) has 2 neighbors: 4(12499.0, 11616.4), 6(12054.4, 10543.0)
start printing
Depth 3: Node 4 (12499.0, 11616.4) has 2 neighbors: 3(12054.4, 12061.0), 5(12499.0, 10987.6)
start printing
Depth 4: Node 3 (12054.4, 12061.0) has 3 neighbors: 2(11425.6, 12061.0), 4(12499.0, 11616.4), 9(10981.0, 11616.4)
start printing
Depth 5: Node 2 (11425.6, 12061.0) has 2 neighbors: 3(12054.4, 12061.0), 8(10981.0, 10987.6)
start printing
Depth 6: Node 8 (10981.0, 10987.6) has 3 neighbors: 7(11425.6, 10543.0), 2(11425.6, 12061.0), 1(0.0, 0.0)
start printing
Depth 7: Node 7 (11425.6, 10543.0) has 3 neighbors: 6(12054.4, 10543.0), 8(10981.0, 10987.6), 1(0.0, 0.0)
start printing
Depth 5: Node 9 (10981.0, 11616.4) has 2 neighbors: 3(12054.4, 12061.0), 1(0.0, 0.0)
done 
THX after all 
start printing
Depth 0: Node 1 (0.0, 0.0) has 4 neighbors: 6(12054.4, 10543.0), 7(11425.6, 10543.0), 8(10981.0, 10987.6), 9(10981.0, 11616.4)
start printing
Depth 1: Node 6 (12054.4, 10543.0) has 4 neighbors: 5(12499.0, 10987.6), 7(11425.6, 10543.0), 1(0.0, 0.0), 10(11479.0, 11051.0)
start printing
Depth 2: Node 5 (12499.0, 10987.6) has 4 neighbors: 4(12499.0, 11616.4), 6(12054.4, 10543.0), 10(11479.0, 11051.0), 11(26088.0, 25118.0)
start printing
Depth 3: Node 4 (12499.0, 11616.4) has 4 neighbors: 3(12054.4, 12061.0), 5(12499.0, 10987.6), 10(11479.0, 11051.0), 11(26088.0, 25118.0)
start printing
Depth 4: Node 3 (12054.4, 12061.0) has 5 neighbors: 2(11425.6, 12061.0), 4(12499.0, 11616.4), 9(10981.0, 11616.4), 10(11479.0, 11051.0), 11(26088.0, 25118.0)
start printing
Depth 5: Node 2 (11425.6, 12061.0) has 4 neighbors: 3(12054.4, 12061.0), 8(10981.0, 10987.6), 10(11479.0, 11051.0), 11(26088.0, 25118.0)
start printing
Depth 6: Node 8 (10981.0, 10987.6) has 4 neighbors: 7(11425.6, 10543.0), 2(11425.6, 12061.0), 1(0.0, 0.0), 10(11479.0, 11051.0)
start printing
Depth 7: Node 7 (11425.6, 10543.0) has 4 neighbors: 6(12054.4, 10543.0), 8(10981.0, 10987.6), 1(0.0, 0.0), 10(11479.0, 11051.0)
start printing
Depth 8: Node 10 (11479.0, 11051.0) has 9 neighbors: 2(11425.6, 12061.0), 3(12054.4, 12061.0), 4(12499.0, 11616.4), 5(12499.0, 10987.6), 6(12054.4, 10543.0), 7(11425.6, 10543.0), 8(10981.0, 10987.6), 9(10981.0, 11616.4), 11(26088.0, 25118.0)
start printing
Depth 9: Node 9 (10981.0, 11616.4) has 3 neighbors: 3(12054.4, 12061.0), 1(0.0, 0.0), 10(11479.0, 11051.0)
start printing
Depth 9: Node 11 (26088.0, 25118.0) has 5 neighbors: 2(11425.6, 12061.0), 3(12054.4, 12061.0), 4(12499.0, 11616.4), 5(12499.0, 10987.6), 10(11479.0, 11051.0)
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
after insert this node of that  (12054.4, 10543.0)
astar_insert************
after insert this node of that  (11425.6, 10543.0)
astar_insert************
after insert this node of that  (10981.0, 10987.6)
astar_insert************
after insert this node of that  (10981.0, 11616.4)
astar_insert************
printing the astar linked list:
(10981.0, 10987.6)->(11425.6, 10543.0)->(10981.0, 11616.4)->(12054.4, 10543.0)->
astar_pop************
Now considering (10981.0, 10987.6)
astar_insert************
after insert this node of that  (11425.6, 10543.0)
astar_insert************
after insert this node of that  (11425.6, 12061.0)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (11479.0, 11051.0)
astar_insert************
printing the astar linked list:
(11425.6, 10543.0)->(10981.0, 11616.4)->(12054.4, 10543.0)->(11479.0, 11051.0)->(11425.6, 12061.0)->(11425.6, 10543.0)->(0.0, 0.0)->
astar_pop************
Now considering (11425.6, 10543.0)
astar_insert************
after insert this node of that  (12054.4, 10543.0)
astar_insert************
after insert this node of that  (10981.0, 10987.6)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (11479.0, 11051.0)
astar_insert************
printing the astar linked list:
(10981.0, 11616.4)->(12054.4, 10543.0)->(11479.0, 11051.0)->(11425.6, 12061.0)->(11479.0, 11051.0)->(12054.4, 10543.0)->(11425.6, 10543.0)->(10981.0, 10987.6)->(0.0, 0.0)->(0.0, 0.0)->
astar_pop************
Now considering (10981.0, 11616.4)
astar_insert************
after insert this node of that  (12054.4, 12061.0)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (11479.0, 11051.0)
astar_insert************
printing the astar linked list:
(12054.4, 10543.0)->(12054.4, 12061.0)->(11479.0, 11051.0)->(11425.6, 12061.0)->(11479.0, 11051.0)->(12054.4, 10543.0)->(11425.6, 10543.0)->(10981.0, 10987.6)->(11479.0, 11051.0)->(0.0, 0.0)->(0.0, 0.0)->(0.0, 0.0)->
astar_pop************
Now considering (12054.4, 10543.0)
astar_insert************
after insert this node of that  (12499.0, 10987.6)
astar_insert************
after insert this node of that  (11425.6, 10543.0)
astar_insert************
after insert this node of that  (0.0, 0.0)
astar_insert************
after insert this node of that  (11479.0, 11051.0)
astar_insert************
printing the astar linked list:
(12499.0, 10987.6)->(12054.4, 12061.0)->(11479.0, 11051.0)->(11425.6, 12061.0)->(11479.0, 11051.0)->(12054.4, 10543.0)->(11425.6, 10543.0)->(10981.0, 10987.6)->(11479.0, 11051.0)->(11479.0, 11051.0)->(11425.6, 10543.0)->(0.0, 0.0)->(0.0, 0.0)->(0.0, 0.0)->(0.0, 0.0)->
astar_pop************
Now considering (12499.0, 10987.6)
astar_insert************
after insert this node of that  (12499.0, 11616.4)
astar_insert************
after insert this node of that  (12054.4, 10543.0)
astar_insert************
after insert this node of that  (11479.0, 11051.0)
astar_insert************
after insert this node of that  (26088.0, 25118.0)
astar_insert************
printing the astar linked list:
(26088.0, 25118.0)->(12054.4, 12061.0)->(11479.0, 11051.0)->(11425.6, 12061.0)->(11479.0, 11051.0)->(12054.4, 10543.0)->(12499.0, 11616.4)->(11425.6, 10543.0)->(10981.0, 10987.6)->(11479.0, 11051.0)->(11479.0, 11051.0)->(11425.6, 10543.0)->(12054.4, 10543.0)->(11479.0, 11051.0)->(0.0, 0.0)->(0.0, 0.0)->(0.0, 0.0)->(0.0, 0.0)->
astar_pop************
Reached the goal: (26088.0, 25118.0)
prepending (26088.0, 25118.0)
prepending (12499.0, 10987.6)
prepending (12054.4, 10543.0)
prepending (0.0, 0.0)
Finished calculating the path
Calculated path: 1(0.0, 0.0) 6(12054.4, 10543.0) 5(12499.0, 10987.6) 11(26088.0, 25118.0)
astar_free************
astar_free************
*************APR25************************************************************************8
Before the dest_x 1, y -701423441 