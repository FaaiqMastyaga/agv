// --- GLOBAL CONFIGURATION ---
var ros = new ROSLIB.Ros({
    url: 'ws://127.0.0.1:9090' // Replace with your robot's IP address
});

// Map Visualization Constants (These can be set based on the largest possible map size)
const MAX_MAP_WIDTH_M = 6.0;   // Max real width of the map in meters
const MAX_MAP_HEIGHT_M = 4.0;  // Max real height of the map in meters
const PIXEL_PER_METER = 100;   // Scaling factor: 1 meter = 100 pixels

const SVG_WIDTH = MAX_MAP_WIDTH_M * PIXEL_PER_METER;
const SVG_HEIGHT = MAX_MAP_HEIGHT_M * PIXEL_PER_METER;

// --- ROS CONNECTION HANDLERS ---
ros.on('connection', function() {
    console.log('Connected to rosbridge.');
    document.getElementById('status').innerText = 'Status: Connected';
    // Initialize the default page
    showPage('setup_map');
});

ros.on('error', function(error) {
    console.log('Error connecting to rosbridge: ', error);
    document.getElementById('status').innerText = 'Status: Error - Check rosbridge connection';
});

ros.on('close', function() {
    console.log('Connection to rosbridge closed.');
    document.getElementById('status').innerText = 'Status: Disconnected';
});

// --- NAVIGATION LOGIC ---
function showPage(pageId) {
    // Hide all pages
    document.querySelectorAll('.page-content').forEach(page => {
        page.classList.remove('active');
    });
    // Remove active state from all buttons
    document.querySelectorAll('.nav-buttons button').forEach(button => {
        button.classList.remove('active');
    });

    // Show the selected page
    document.getElementById(pageId + '_page').classList.add('active');
    document.getElementById('btn_' + pageId).classList.add('active');

    // Special initialization for Operation page
    if (pageId === 'operation') {
        // Ensure map SVG is sized correctly and start listeners
        document.getElementById('operation-map-svg').setAttribute('viewBox', `0 0 ${SVG_WIDTH} ${SVG_HEIGHT}`);
        subscribeToPoseTopics();
        
        // Fetch the currently loaded map for visualization
        getAndDrawMap('operation-map-svg');
    }
}


// --- MAP VISUALIZATION FUNCTIONS (Core Logic) ---

/**
 * Converts quaternion (ROS orientation) to Yaw angle (Z-axis rotation) in degrees.
 */
function getYawFromQuaternion(q) {
    // Yaw (z-axis rotation)
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    
    // Result in radians
    const yaw_rad = Math.atan2(siny_cosp, cosy_cosp); 

    // Convert to degrees
    return yaw_rad * (180 / Math.PI);
}

/**
 * Draws the static markers onto a specified SVG element.
 * @param {string} svgId - The ID of the SVG element (e.g., 'setup-map-svg').
 * @param {Array} markers - An array of marker objects {id: number, pose: {pose: {position: {x: number, y: number, z: number}, orientation: {x: number, y: number, z: number, w: number}}}}
 */
function drawMarkers(svgId, markers) {
    const svg = document.getElementById(svgId);
    
    // Clear previous markers and the AGV indicator
    // This assumes the AGV is the last element added
    while (svg.lastChild && svg.lastChild.id !== 'agv-indicator') {
        svg.removeChild(svg.lastChild);
    }
    
    // Set viewbox based on maximum size
    svg.setAttribute('viewBox', `0 0 ${SVG_WIDTH} ${SVG_HEIGHT}`);

    markers.forEach(marker => {
        const x_m = marker.pose.pose.position.x;
        const y_m = marker.pose.pose.position.y;
        
        // 1. Convert meters to SVG pixels
        const x_px = x_m * PIXEL_PER_METER;
        // 2. Flip Y-axis: SVG Y = Total Height - Real Y (to make map origin bottom-left)
        const y_px = SVG_HEIGHT - (y_m * PIXEL_PER_METER); 

        // Draw Marker (e.g., a square)
        const rect = document.createElementNS("http://www.w3.org/2000/svg", 'rect');
        rect.setAttribute('x', x_px - 8); // Center the square (size 16x16)
        rect.setAttribute('y', y_px - 8);
        rect.setAttribute('width', 16);
        rect.setAttribute('height', 16);
        rect.setAttribute('fill', 'blue');
        rect.setAttribute('stroke', '#333');
        svg.appendChild(rect);

        // Draw ID text
        const text = document.createElementNS("http://www.w3.org/2000/svg", 'text');
        text.setAttribute('x', x_px);
        text.setAttribute('y', y_px + 5);
        text.setAttribute('text-anchor', 'middle');
        text.setAttribute('fill', 'white');
        text.setAttribute('font-size', '10px');
        text.textContent = marker.id;
        svg.appendChild(text);
    });

    // Ensure AGV indicator exists for operation page
    if (svgId === 'operation-map-svg' && !document.getElementById('agv-indicator')) {
        const agvElement = document.createElementNS("http://www.w3.org/2000/svg", 'polygon');
        agvElement.setAttribute('id', 'agv-indicator');
        agvElement.setAttribute('points', "0,-15 15,15 -15,15"); // Triangle shape, centered at (0,0), pointing up
        agvElement.setAttribute('fill', 'red');
        agvElement.setAttribute('stroke', 'black');
        agvElement.setAttribute('transform', 'translate(50, 50) rotate(0)'); // Initial position
        svg.appendChild(agvElement);
    }
}

/**
 * Updates the AGV position on the operation map.
 */
function updateAGVVisualization(x_m, y_m, yaw_deg) {
    const agv = document.getElementById('agv-indicator');
    if (!agv) return;

    const x_px = x_m * PIXEL_PER_METER;
    const y_px = SVG_HEIGHT - (y_m * PIXEL_PER_METER);

    // The yaw from ROS is typically 0 along the X-axis. 
    // Our SVG triangle points along the negative Y axis (up) by default.
    // A 90 degree offset is commonly needed to align the ROS frame with the SVG element.
    const rotation_compensation = 90; 

    agv.setAttribute('transform',
        `translate(${x_px}, ${y_px}) rotate(${yaw_deg + rotation_compensation})`
    );
}

// --- SETUP MAP PAGE LOGIC ---

// Local array to hold marker data for the form (not yet saved to ROS)
var mapDataForSave = [];

function addMarkerField() {
    const markerListDiv = document.getElementById('marker_list');
    const markerCount = document.querySelectorAll('.marker-entry').length;

    const div = document.createElement('div');
    div.classList.add('marker-entry');
    div.dataset.index = markerCount;

    div.innerHTML = `
        <strong>Marker ${markerCount + 1}</strong>
        <label>ID:</label> <input type="number" class="marker-id" value="${markerCount}"><br>
        <label>X (m):</label> <input type="number" step="0.01" class="marker-x" value="1.0"><br>
        <label>Y (m):</label> <input type="number" step="0.01" class="marker-y" value="1.0"><br>
        <label>Orientation (deg):</label> <input type="number" step="1" class="marker-ort" value="0"><br>
        <button onclick="removeMarkerField(${markerCount})" style="float: right;">Remove</button>
    `;

    markerListDiv.appendChild(div);
    updateMapVisualization(); // Update map immediately when a field is added
}

function removeMarkerField(index) {
    document.querySelector(`.marker-entry[data-index="${index}"]`).remove();
    updateMapVisualization();
}

function updateMapVisualization() {
    const markerData = [];
    document.querySelectorAll('.marker-entry').forEach(entry => {
        const id = parseInt(entry.querySelector('.marker-id').value);
        const x = parseFloat(entry.querySelector('.marker-x').value);
        const y = parseFloat(entry.querySelector('.marker-y').value);
        const ort_deg = parseFloat(entry.querySelector('.marker-ort').value);
        
        // Convert orientation degrees to ROS quaternion (only Z/Yaw needed)
        const yaw_rad = ort_deg * (Math.PI / 180);
        
        // This is a minimal representation needed for drawMarkers
        // We'll use this format for compatibility with the ROS Marker message structure
        markerData.push({
            id: id,
            pose: {
                pose: {
                    position: {x: x, y: y, z: 0},
                    orientation: {x: 0, y: 0, z: Math.sin(yaw_rad / 2), w: Math.cos(yaw_rad / 2)}
                }
            }
        });
    });

    mapDataForSave = markerData; // Store the data for saving
    drawMarkers('setup-map-svg', markerData);
}

// Add event listeners to update map visualization on input change
document.getElementById('marker_list').addEventListener('input', updateMapVisualization);


function saveMap() {
    const mapName = document.getElementById('map_name').value;
    const markerDict = document.getElementById('marker_dictionary').value;
    const markerSize = parseFloat(document.getElementById('marker_size').value);
    const saveStatusElement = document.getElementById('save_status');
    
    if (!mapName || !markerDict || isNaN(markerSize) || mapDataForSave.length === 0) {
        saveStatusElement.innerText = 'Error: Please fill all fields and add at least one marker.';
        return;
    }

    saveStatusElement.innerText = 'Saving map...';

    // 1. Convert the local mapDataForSave into the ROS MarkerArray message structure
    const markerArrayMsg = new ROSLIB.Message({
        markers: mapDataForSave.map(localMarker => {
            return {
                id: localMarker.id,
                pose: localMarker.pose // Already in correct format
            };
        })
    });

    // 2. Create the ROS Service Client
    const saveMapClient = new ROSLIB.Service({
        ros: ros,
        name: '/map_database/save_map',
        serviceType: 'map_database_interfaces/srv/SaveMap' // Assuming you created this srv file
    });

    // 3. Create the Service Request
    const request = new ROSLIB.ServiceRequest({
        map_name: mapName,
        marker_dictionary: markerDict,
        marker_size: markerSize,
        map_data: markerArrayMsg
    });

    // 4. Call the service
    saveMapClient.callService(request, function(result) {
        if (result.success) {
            saveStatusElement.innerText = `Success! Map '${mapName}' saved. ID: ${result.new_map_id}.`;
        } else {
            saveStatusElement.innerText = `Failed to save map: ${result.message}`;
            console.error('Save Map Error:', result.message);
        }
    }, function(error) {
        saveStatusElement.innerText = `Service communication error: ${error}`;
        console.error('Save Map Comm Error:', error);
    });
}

function loadMap() {
    const mapName = document.getElementById('load_map_name').value;
    const loadStatusElement = document.getElementById('load_status');

    if (!mapName) {
        loadStatusElement.innerText = 'Error: Please enter a map name.';
        return;
    }

    loadStatusElement.innerText = `Loading map '${mapName}'...`;

    // 1. Create the ROS Service Client for LoadMap
    const loadMapClient = new ROSLIB.Service({
        ros: ros,
        name: '/map_database/load_map',
        serviceType: 'map_database_interfaces/srv/LoadMap'
    });

    // 2. Create the Service Request
    const request = new ROSLIB.ServiceRequest({
        map_name: mapName
    });

    // 3. Call the service
    loadMapClient.callService(request, function(result) {
        if (result.success) {
            loadStatusElement.innerText = `Map '${mapName}' successfully loaded by ROS backend.`;
            // Trigger drawing the map on both setup and operation pages
            getAndDrawMap('setup-map-svg'); 
        } else {
            loadStatusElement.innerText = `Failed to load map: ${result.message}`;
            console.error('Load Map Error:', result.message);
        }
    }, function(error) {
        loadStatusElement.innerText = `Service communication error: ${error}`;
        console.error('Load Map Comm Error:', error);
    });
}

/**
 * Fetches the currently loaded map data from the ROS map_loader node
 * and draws it on the specified SVG.
 */
function getAndDrawMap(svgId) {
    // 1. Create the ROS Service Client for GetMap
    const getMapClient = new ROSLIB.Service({
        ros: ros,
        name: '/map_database/get_map',
        serviceType: 'map_database_interfaces/srv/GetMap'
    });

    // The GetMap service doesn't require a request body.
    const request = new ROSLIB.ServiceRequest({});

    // 2. Call the service
    getMapClient.callService(request, function(result) {
        if (result.success && result.map_data.markers.length > 0) {
            drawMarkers(svgId, result.map_data.markers);
            console.log(`Successfully retrieved and drew ${result.map_data.markers.length} markers.`);
        } else {
            console.warn('Get Map Warning: No map data received from ROS loader.');
            // Clear visualization if called on setup page
            if (svgId === 'setup-map-svg') {
                drawMarkers(svgId, []);
            }
        }
    }, function(error) {
        console.error('Get Map Comm Error:', error);
    });
}


// --- OPERATION PAGE LOGIC (Subscriptions) ---

function subscribeToPoseTopics() {
    // Subscribe to the /robot/pose topic (AGV Position)
    // NOTE: Using the first pose from the PoseArray for the single AGV location.
    const robot_pose_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/robot/pose',
        messageType: 'geometry_msgs/PoseArray'
    });

    robot_pose_listener.subscribe(function(message) {
        let outputHTML = '';
        const agv_data_div = document.getElementById('robot_data');
        const agv_map_svg_id = 'operation-map-svg';

        if (message.poses.length > 0) {
            const pose = message.poses[0]; // Assume first pose is the AGV
            const position = pose.position;
            const orientation = pose.orientation;
            
            const yaw_deg = getYawFromQuaternion(orientation);
            
            outputHTML += `
                <p><strong>AGV Global Position:</strong></p>
                <ul>
                    <li>x: ${position.x.toFixed(4)} cm</li>
                    <li>y: ${position.y.toFixed(4)} cm</li>
                    <li>z: ${position.z.toFixed(4)} cm</li>
                </ul>
                <p><strong>AGV Orientation (Yaw):</strong></p>
                <ul>
                    <li>Yaw: ${yaw_deg.toFixed(2)} degree</li>
                </ul>
                <hr>
            `;
            
            // 1. Update the Map Visualization
            updateAGVVisualization(position.x, position.y, yaw_deg);
            
        } else {
            outputHTML = '<p>AGV pose not available.</p>';
        }
        
        agv_data_div.innerHTML = outputHTML;
    });

    // Subscribe to the /aruco/pose topic (Detected Markers)
    const aruco_pose_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/aruco/pose',
        messageType: 'aruco_msgs/MarkerArray'
    });

    aruco_pose_listener.subscribe(function(message) {
        let outputHTML = '';
        const aruco_data_div = document.getElementById('aruco_data');
        
        if (message.markers.length > 0) {
            outputHTML += '<h3>Currently Detected Markers:</h3>';
            message.markers.forEach(marker => {
                const position = marker.pose.pose.position;
                const orientation = marker.pose.pose.orientation;
                const yaw_deg = getYawFromQuaternion(orientation);

                outputHTML += `
                    <h4>ID: ${marker.id} (Local to Camera Frame)</h4>
                    <ul>
                        <li>x: ${position.x.toFixed(4)} mm</li>
                        <li>y: ${position.y.toFixed(4)} mm</li>
                        <li>Yaw: ${yaw_deg.toFixed(2)} deg</li>
                    </ul>
                `;
            });
        } else {
            outputHTML = '<p>No ArUco markers currently detected by camera.</p>';
        }

        aruco_data_div.innerHTML = outputHTML;
    });
}


// --- INITIALIZATION ---
// Add the first marker field on page load for the Setup Map page
window.onload = function() {
    addMarkerField();
    
    // Set up the initial SVG viewbox (needed even if not displayed)
    document.getElementById('setup-map-svg').setAttribute('viewBox', `0 0 ${SVG_WIDTH} ${SVG_HEIGHT}`);
};