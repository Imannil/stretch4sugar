    const angularSpeed = 0.5;

    // Movement intensity variables
    let cameraMovementIntensity = 0.8;
    let gripperAperture = 0.1;
    let liftHeight = 0.6;
    let baseMovementIntensity = 0.1;
    let armExtensionDistance = 0.0;
    let gripperRotationAmount = 0.2;  // New variable for gripper rotation
    let gripperPitchAmount = 0.2;     // New variable for gripper pitch

    // Update functions for sliders
    function updateCameraValue(value) {
      cameraMovementIntensity = parseFloat(value);
      document.getElementById('cameraValue').innerText = value;
    }

    function updateGripperValue(value) {
      gripperAperture = parseFloat(value);
      document.getElementById('gripperValue').innerText = value;
    }

    function updateLiftValue(value) {
      liftHeight = parseFloat(value);
      document.getElementById('liftValue').innerText = value;
    }

    function updateArmValue(value) {
      armExtensionDistance = parseFloat(value);
      document.getElementById('armValue').innerText = value;
    }

    function updateBaseMovementValue(value) {
      baseMovementIntensity = parseFloat(value);
      document.getElementById('baseMovementValue').innerText = value;
    }

    // New update functions for gripper orientation
    function updateGripperRotationValue(value) {
      gripperRotationAmount = parseFloat(value);
      document.getElementById('gripperRotationValue').innerText = value;
    }

    function updateGripperPitchValue(value) {
      gripperPitchAmount = parseFloat(value);
      document.getElementById('gripperPitchValue').innerText = value;
    }

    /**
     * Execute a joint trajectory
     * @param {Array} jointNames - Array of joint names
     * @param {Array} jointPositions - Array of joint positions
     */
    function executeFollowJointTrajectory(jointNames, jointPositions) {
      const goal = new ROSLIB.ActionGoal({
        trajectory: {
          header: { stamp: { secs: 0, nsecs: 0 } },
          joint_names: jointNames,
          points: [
            {
              positions: jointPositions,
              time_from_start: { secs: 1, nsecs: 0 },
            },
          ],
        },
      });
      trajectoryClient.createClient(goal);
    }

    // Robot control functions
    const RobotControl = {
      // Gripper controls
      openGripper: () => executeFollowJointTrajectory(['gripper_aperture'], [gripperAperture]),
      closeGripper: () => executeFollowJointTrajectory(['gripper_aperture'], [-gripperAperture]),
      
      // New gripper orientation controls
      rotateGripperCW: () => executeFollowJointTrajectory(['joint_wrist_yaw'], [gripperRotationAmount]),
      rotateGripperCCW: () => executeFollowJointTrajectory(['joint_wrist_yaw'], [-gripperRotationAmount]),
      pitchGripperUp: () => executeFollowJointTrajectory(['joint_wrist_pitch'], [gripperPitchAmount]),
      pitchGripperDown: () => executeFollowJointTrajectory(['joint_wrist_pitch'], [-gripperPitchAmount]),
      
      // Lift controls
      moveLiftToTop: () => executeFollowJointTrajectory(['joint_lift'], [1.1]),
      moveLiftToMiddle: () => executeFollowJointTrajectory(['joint_lift'], [liftHeight]),
      
      // Arm extension controls
      extendArm: () => executeFollowJointTrajectory(['joint_arm'], [armExtensionDistance]),
      retractArm: () => executeFollowJointTrajectory(['joint_arm'], [-armExtensionDistance]),

      // Base movement
      moveBaseForward: () => {
        executeFollowJointTrajectory(['translate_mobile_base'], [baseMovementIntensity]);
      },
      
      moveBaseBackward: () => {
        executeFollowJointTrajectory(['translate_mobile_base'], [-baseMovementIntensity]);
      },
      
      // Camera/Head movements with slider-controlled intensity
      lookLeft: () => executeFollowJointTrajectory(['joint_head_pan'], [cameraMovementIntensity]),
      lookRight: () => executeFollowJointTrajectory(['joint_head_pan'], [-cameraMovementIntensity]),
      lookUp: () => executeFollowJointTrajectory(['joint_head_tilt'], [cameraMovementIntensity/2]),
      lookDown: () => executeFollowJointTrajectory(['joint_head_tilt'], [-cameraMovementIntensity/2]),
      lookCenter: () => executeFollowJointTrajectory(['joint_head_pan', 'joint_head_tilt'], [0.0, 0.0]),
      
      // Rotation controls
      rotateLeft: () => RobotControl.sendTwist(angularSpeed),
      rotateRight: () => RobotControl.sendTwist(-angularSpeed),
      
      // Twist command for rotation
      sendTwist: (zSpeed) => {
        switchToNavigationMode((success) => {
          if (success) {
            const cmdVel = new ROSLIB.Topic({
              ros: ros,
              name: '/stretch/cmd_vel',
              messageType: 'geometry_msgs/msg/Twist'
            });
            
            const twist = new ROSLIB.Message({
              linear: { x: 0.0, y: 0.0, z: 0.0 },
              angular: { x: 0.0, y: 0.0, z: zSpeed }
            });
            
            cmdVel.publish(twist);
          }
        });
      },
      
      // Home robot position
      homeRobot: () => {
        const homeService = new ROSLIB.Service({
          ros: ros,
          name: '/home_the_robot',
          serviceType: 'std_srvs/srv/Trigger'
        });
        
        homeService.callService(new ROSLIB.ServiceRequest({}), (result) => {
          console.log("Home service response:", result);
        });
      },
      
      // Position the robot (placeholder for complex movement)
      positionRobot: () => {
        console.log("Position robot function - implement as needed");
      },
      
      // Pose management
      savePoseA: () => {
        const msg = new ROSLIB.Message({ data: 'Pose A,base_link' });
        new ROSLIB.Topic({
            ros: ros,
            name: '/save_pose',
            messageType: 'std_msgs/String'
        }).publish(msg);
        document.getElementById('poseAstatus').innerText = 'Status: saved';
      },  
      
      executePoseA: () => {
        const svc = new ROSLIB.Service({
            ros: ros,
            name: '/pose_player',
            serviceType: 'std_srvs/Trigger'
        });
        const req = new ROSLIB.ServiceRequest({});
        
        svc.callService(req, res => {
            document.getElementById('poseAstatus').innerText =
                res.success ? 'Status: done' : `Status: ${res.message}`;
            
            console.log("Service response:", res);
        });
      },
    };