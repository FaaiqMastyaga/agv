// Connection to rosbridge
var ros = new ROSLIB.Ros({
    url: 'ws://127.0.0.1:9090' // Replace with your robot's IP address
});

// An element to display the connection status (add this to your HTML)
// <p id="status">Connecting...</p>
ros.on('connection', function() {
    console.log('Connected to rosbridge.');
    document.getElementById('status').innerText = 'Connected';
});

ros.on('error', function(error) {
    console.log('Error connecting to rosbridge: ', error);
    document.getElementById('status').innerText = 'Error';
});

ros.on('close', function() {
    console.log('Connection to rosbridge closed.');
    document.getElementById('status').innerText = 'Disconnected';
});

// Function to convert quaternion to Euler angles
function toEuler(q) {
    // roll (x-axis rotation)
    const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    const pitch = Math.atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    const sinp = 2 * (q.w * q.y - q.z * q.x);
    let yaw;
    if (Math.abs(sinp) >= 1) {
        yaw = Math.sign(sinp) * Math.PI / 2; // use 90 degrees if out of range
    } else {
        yaw = Math.asin(sinp);
    }

    // yaw (z-axis rotation)
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    const roll = Math.atan2(siny_cosp, cosy_cosp);

    return {
        roll: roll,
        pitch: pitch,
        yaw: yaw
    };
}

// Subscribe to the /robot/pose topic
var robot_pose_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/robot/pose',
    messageType: 'geometry_msgs/PoseArray'
});

robot_pose_listener.subscribe(function(message) {
    let outputHTML = '';
    
    if (message.poses.length > 0) {
        message.poses.forEach(pose => {
            const position = pose.position;
            const orientation = pose.orientation;
            
            // Convert quaternion to Euler angles
            const euler = toEuler(orientation);
            
            // Convert radians to degrees
            const yaw_deg = euler.roll * 180 / Math.PI;
            const pitch_deg = euler.pitch * 180 / Math.PI;
            const roll_deg = euler.yaw * 180 / Math.PI;
            
            // Generate HTML for each marker
            outputHTML += `
            <p><strong>Robot Position:</strong></p>
            <ul>
            <li>x: ${position.x.toFixed(4)} mm</li>
            <li>y: ${position.y.toFixed(4)} mm</li>
            <li>z: ${position.z.toFixed(4)} mm</li>
            </ul>
            <p><strong>Robot Orientation (Euler in Degrees):</strong></p>
            <ul>
            <li>Roll: ${roll_deg.toFixed(2)} degree</li>
            <li>Pitch: ${pitch_deg.toFixed(2)} degree</li>
            <li>Yaw: ${yaw_deg.toFixed(2)} degree</li>
            </ul>
            <hr>
            `;
        });
    } else {
        outputHTML = '<p>No ArUco markers detected.</p>';
    }
    
    // Update the HTML element with the generated data
    // document.getElementById('aruco_data').innerHTML = outputHTML;
    document.getElementById('robot_data').innerHTML = outputHTML;
});

// Subscribe to the /aruco/pose topic
var aruco_pose_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/aruco/pose',
    messageType: 'aruco_msgs/MarkerArray'
});

aruco_pose_listener.subscribe(function(message) {
        let outputHTML = '';
    
        if (message.markers.length > 0) {
        message.markers.forEach(marker => {
            const position = marker.pose.pose.position;
            const orientation = marker.pose.pose.orientation;
            
            // Convert quaternion to Euler angles
            const euler = toEuler(orientation);
            
            // Convert radians to degrees
            const roll_deg = euler.roll * 180 / Math.PI;
            const pitch_deg = euler.pitch * 180 / Math.PI;
            const yaw_deg = euler.yaw * 180 / Math.PI;

            // Generate HTML for each marker
            outputHTML += `
                <h3>Marker ID: ${marker.id}</h3>
                <p><strong>Position:</strong></p>
                <ul>
                    <li>x: ${position.x.toFixed(4)} mm</li>
                    <li>y: ${position.y.toFixed(4)} mm</li>
                    <li>z: ${position.z.toFixed(4)} mm</li>
                </ul>
                <p><strong>Orientation (Euler in Degrees):</strong></p>
                <ul>
                    <li>Roll: ${roll_deg.toFixed(2)} degree</li>
                    <li>Pitch: ${pitch_deg.toFixed(2)} degree</li>
                    <li>Yaw: ${yaw_deg.toFixed(2)} degree</li>
                </ul>
                <hr>
            `;
        });
    } else {
        outputHTML = '<p>No ArUco markers detected.</p>';
    }

    // Update the HTML element with the generated data
    // document.getElementById('aruco_data').innerHTML = outputHTML;
    document.getElementById('aruco_data').innerHTML = outputHTML;
});