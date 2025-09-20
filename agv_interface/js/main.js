// Connection to rosbridge
var ros = new ROSLIB.Ros({
    url: 'ws://127.0.0.1:9090' // Replace with your robot's IP address
});

ros.on('connection', function() {
    document.getElementById('status').innerText = 'Connected';
    console.log('Connected to rosbridge.');
});

ros.on('error', function(error) {
    document.getElementById('status').innerText = 'Error';
    console.log('Error connecting to rosbridge: ', error);
});

ros.on('close', function() {
    document.getElementById('status').innerText = 'Disconnected';
    console.log('Connection to rosbridge closed.');
});

// Topic to publish commands to the robot
var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
});

// Function to send command messages
function sendCmd(linear_x, angular_z) {
    var twist_msg = new ROSLIB.Message({
        linear: { x: linear_x, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angular_z }
    });
    cmdVel.publish(twist_msg);
    console.log(`Sending command: linear=${linear_x}, angular=${angular_z}`);
}

// Subscribe to a topic for data display (optional)
var aruco_pose_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/aruco_pose',
    messageType: 'geometry_msgs/PoseArray'
});

aruco_pose_listener.subscribe(function(message) {
    // This is where you would process the data from the 'aruco_pose' topic
    // and display it on your web page.
    console.log('Received ArUco pose data:', message);
});