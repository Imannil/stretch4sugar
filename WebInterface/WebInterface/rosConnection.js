/**
 * Handles ROS WebSocket connection setup and management
 */

// ROS connection configuration
const ROS_WS_URL = 'ws://rocky.hcrlab.cs.washington.edu:9090';

// Main ROS connection object
let ros = new ROSLIB.Ros({
  url: ROS_WS_URL
});

// Connection event handlers
ros.on('connection', () => {
  document.getElementById('connection').innerText = "Connected to Stretch";
  subscribeToCameraVideo();
  createTrajectoryClient();
});

ros.on('error', (error) => {
  document.getElementById('connection').innerText = "Connection error: check websocket";
  console.log(error);
});

ros.on('close', () => {
  document.getElementById('connection').innerText = "Disconnected";
});

// Video streaming
function subscribeToCameraVideo() {
  const cameraImage = document.getElementById("cameraImage");
  const topic = new ROSLIB.Topic({
    ros: ros,
    name: "/camera/color/image_raw/compressed",
    messageType: "sensor_msgs/CompressedImage",
  });
  
  topic.subscribe((message) => {
    cameraImage.src = "data:image/jpg;base64," + message.data;
  });
}

// Create trajectory action client
let trajectoryClient;

function createTrajectoryClient() {
  trajectoryClient = new ROSLIB.ActionHandle({
    ros: ros,
    name: "/stretch_controller/follow_joint_trajectory",
    actionType: "control_msgs/action/FollowJointTrajectory",
  });
}

// Mode switching functions
function switchToNavigationMode(callback) {
  const switchModeClient = new ROSLIB.Service({
    ros: ros,
    name: '/switch_to_navigation_mode',
    serviceType: 'std_srvs/srv/Trigger'
  });
  
  const request = new ROSLIB.ServiceRequest({});
  switchModeClient.callService(request, 
    function(result) {
      callback(result.success);
    }, 
    function(error) {
      console.error('Error calling switch_to_navigation_mode:', error);
      callback(false);
    }
  );
}

function switchToPositionMode(callback) {
  const switchModeClient = new ROSLIB.Service({
    ros: ros,
    name: '/switch_to_position_mode',
    serviceType: 'std_srvs/srv/Trigger'
  });
  
  const request = new ROSLIB.ServiceRequest({});
  switchModeClient.callService(request, 
    function(result) {
      if (result.success) {
        console.log("Switched to position mode.");
        callback(true);
      } else {
        console.error("Failed to switch to position mode:", result.message);
        callback(false);
      }
    }, 
    function(error) {
      console.error('Error calling switch_to_position_mode:', error);
      callback(false);
    }
  );
}