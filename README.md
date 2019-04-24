# kobuki-catbot
Kobuki robot implementation based on ROS emulating Cat-like behavior

Implementation List:
- [x] Table edge avoidance
  - [x] Respond to events from Turtlebot cliff sensors
  - [x] Turn away from edge when edge is detected
  - [x] Use wheel encoders to turn precisely 180 degrees away from edge (consider different angles depending on which cliff sensor is activated)
- [ ] Behavioral finite state machine (FSM)
- [ ] Object detection and targeting
  - [ ] Interface to Kinect in code
  - [ ] Support getting Kinect video data into OpenCV-usable format
  - [ ] OpenCV object detection
  - [ ] Object targeting using Turtlebot drive commands
- [ ] Rotational object yeetance
  - [ ] Move so that object is in range of "tail"
  - [x] Full speed single/double rotation to ~remove~ yeet object from table
