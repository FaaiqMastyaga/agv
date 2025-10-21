// ===================================================================
// === FINAL MERGED JAVASCRIPT: app.js (CM Units & UX Fixes) ===
// ===================================================================

// --- GLOBAL CONFIGURATION ---
const MAX_MAP_WIDTH_CM = 600.0;   // 6 meters = 600 cm
const MAX_MAP_HEIGHT_CM = 600.0; 
const PIXEL_PER_CM = 1;   

const mapCanvas = document.getElementById("mapCanvas");
const opMapCanvas = document.getElementById("opMapCanvas");
const ctx = mapCanvas ? mapCanvas.getContext("2d") : null;
const opCtx = opMapCanvas ? opMapCanvas.getContext("2d") : null;

let markers = []; 
let agvPose = { x: 0, y: 0, yaw: 0 };
let agvPoseHypotheses = [];

// --- DOM ELEMENTS ---
const rosStatus = document.getElementById("rosStatus");
const cameraFeed = document.getElementById("cameraFeed");
const tabs = document.querySelectorAll(".tab-btn");
const panes = document.querySelectorAll(".tab-pane");

let ros = null; 
const MJPEG_PORT = '8080'; 
const MJPEG_TOPIC = '/camera/image_marked';
const ROSBRIDGE_PORT = '9090';
const ROSBRIDGE_URL = `ws://localhost:${ROSBRIDGE_PORT}`;

function connectRosBridge(wsUrl) {
    ros = new ROSLIB.Ros({
        url: wsUrl
    });

    ros.on('connection', () => {
        rosStatus.textContent = "Status: Connected ✅";
        
        // --- ROS-RELATED LOGIC: M-JPEG STREAM ---
        const urlParts = new URL(wsUrl);
        const host = urlParts.hostname;
        
        if (cameraFeed) {
             const mjpegUrl = `http://${host}:${MJPEG_PORT}/stream?topic=${MJPEG_TOPIC}`;
             cameraFeed.src = mjpegUrl;
             cameraFeed.alt = `Live stream from ${MJPEG_TOPIC}`;
             console.log(`Video feed source set to: ${mjpegUrl}`);
        }
        
        subscribeToAGVPosition(); 
        subscribeToPoseTopics();  
        populateMapList(); 
    });

    ros.on('error', (error) => {
        rosStatus.textContent = "Status: Error ❌. Cek konsol dan URL.";
        if (cameraFeed) cameraFeed.src = "#";
        console.error('ROS-bridge Error: ', error);
        if (ros) ros.close();
        ros = null;
    });

    ros.on('close', () => {
        rosStatus.textContent = "Status: Disconnected";
        if (cameraFeed) cameraFeed.src = "#";
        ros = null;
    });
}


// === TAB SWITCHING ===
const tabs_nav = document.querySelectorAll(".tab-btn");
const panes_nav = document.querySelectorAll(".tab-pane");

tabs_nav.forEach(btn => {
  btn.addEventListener("click", () => {
    tabs_nav.forEach(b => b.classList.remove("active"));
    panes_nav.forEach(p => p.classList.remove("active"));
    btn.classList.add("active");
    document.getElementById(btn.dataset.tab).classList.add("active");
    
    if (btn.dataset.tab === "operation") {
        renderOperationMap();
        if (!ros || !ros.isConnected) {
            connectRosBridge(ROSBRIDGE_URL);
        }
    } else {
        renderMap();
    }
  });
});


// --- HELPER FUNCTION: Quaternion to Yaw ---
function quaternionToYaw(q) {
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return Math.atan2(siny_cosp, cosy_cosp) * 180 / Math.PI; 
}

// --- RENDERING FUNCTIONS (Canvas) ---
function getMapScale() {
    const width = parseFloat(document.getElementById("xLength").value) || MAX_MAP_WIDTH_CM;
    const height = parseFloat(document.getElementById("yLength").value) || MAX_MAP_HEIGHT_CM;
    
    const scaleX = mapCanvas.width / width;
    const scaleY = mapCanvas.height / height;
    return { width, height, scaleX, scaleY };
}

function drawGrid(canvas, context, width, height, scaleX, scaleY) {
    if (!context || !canvas) return;
    context.strokeStyle = "#e0e0e0";
    context.font = "10px Arial";
    context.fillStyle = "#777";

    for (let j = 0; j <= height; j += 100) { 
        const y = j * scaleY;
        context.beginPath();
        context.moveTo(0, y);
        context.lineTo(canvas.width, y);
        context.stroke();
        context.fillText((height - j).toFixed(0) + "cm", 5, y - 5);
    }
    for (let i = 0; i <= width; i += 100) { 
        const x = i * scaleX;
        context.beginPath();
        context.moveTo(x, 0);
        context.lineTo(x, canvas.height);
        context.stroke();
        context.fillText(i.toFixed(0) + "cm", x + 5, canvas.height - 5);
    }
}

function renderMap() {
    if (!ctx || !mapCanvas) return;
    
    // --- CRITICAL FIX: Use the globally updated window.markers ---
    const markersToDraw = window.markers || [];
    
    // 1. Get current dimensions/scale based on input fields
    const { width, height, scaleX, scaleY } = getMapScale();
    
    // 2. Clear and Draw Grid
    ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    drawGrid(mapCanvas, ctx, width, height, scaleX, scaleY);

    // --- Draw Static Markers ---
    markersToDraw.forEach(m => {
        // Calculate scaled coordinates
        ctx.fillStyle = "blue";
        const centerX = m.x * scaleX;
        // Flip Y-axis: Canvas Y = Canvas Height - (Marker Y * Scale Y)
        const centerY = mapCanvas.height - (m.y * scaleY); 

        // Draw Marker Body (Blue Circle)
        ctx.beginPath();
        ctx.arc(centerX, centerY, 6, 0, 2 * Math.PI);
        ctx.fill();
        
        // Draw Orientation Line (Red Line)
        ctx.save();
        ctx.translate(centerX, centerY);
        // Corrected orientation line: Yaw is 0 along X axis, so no compensation needed here
        ctx.rotate(-m.yaw * Math.PI / 180); 
        ctx.strokeStyle = "red";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(15, 0); 
        ctx.stroke();
        ctx.restore();
        
        // DRAW TEXT LABEL
        ctx.fillText(`ID ${m.id} (${m.yaw}°)`, centerX + 8, centerY - 8);
    });
    // The console log will now accurately reflect the number of markers drawn
    console.log(`Map Setup rendered with ${markersToDraw.length} markers.`);
}


function renderOperationMap() {
    if (!opCtx || !opMapCanvas) return;
    
    // Use the globally updated window.markers
    const markersToDraw = window.markers || [];

    const { width, height, scaleX, scaleY } = getMapScale();
    opCtx.clearRect(0, 0, opMapCanvas.width, opMapCanvas.height);
    drawGrid(opMapCanvas, opCtx, width, height, scaleX, scaleY);

    // Static Markers
    markersToDraw.forEach(m => {
        opCtx.fillStyle = "gray";
        const centerX = m.x * scaleX;
        const centerY = opMapCanvas.height - (m.y * scaleY); 
        
        opCtx.beginPath();
        opCtx.arc(centerX, centerY, 5, 0, 2 * Math.PI);
        opCtx.fill();
        opCtx.fillText(`ID ${m.id}`, centerX + 8, centerY - 8);
    });

    const HYPOTHESIS_SIZE = 5;
    const HYPOTHESIS_COLOR = 'rgba(0, 150, 255, 0.3)';
    
    // --- DEBUG LOGGING ---
    console.log(`DEBUG: Hypothesis count to draw: ${agvPoseHypotheses.length}`);
    if (agvPoseHypotheses.length > 0) {
        console.log(`DEBUG: First hypothesis coordinates (cm): X=${agvPoseHypotheses[0].x.toFixed(2)}, Y=${agvPoseHypotheses[0].y.toFixed(2)}`);
    }
    // --- END DEBUG LOGGING ---


    agvPoseHypotheses.forEach(pose => {
        const hyp_x_cm = pose.x;
        const hyp_y_cm = pose.y;

        const centerX = hyp_x_cm * scaleX; 
        const centerY = opMapCanvas.height - (hyp_y_cm * scaleY); 
        
        // Skip drawing if coordinates are invalid or outside a plausible range
        // This prevents the entire rendering function from crashing or drawing off-screen junk.
        if (isNaN(centerX) || isNaN(centerY) || centerX < -50 || centerX > opMapCanvas.width + 50 || centerY < -50 || centerY > opMapCanvas.height + 50) {
            console.warn(`WARNING: Hypothesis skipped (invalid coordinates): X=${hyp_x_cm.toFixed(2)}, Y=${hyp_y_cm.toFixed(2)}`);
            return; 
        }

        // Draw Hypothesis Body (Circle)
        opCtx.fillStyle = HYPOTHESIS_COLOR;
        opCtx.beginPath();
        opCtx.arc(centerX, centerY, HYPOTHESIS_SIZE, 0, 2 * Math.PI); 
        opCtx.fill();

        // Draw Hypothesis Orientation (Short Line)
        opCtx.save();
        opCtx.translate(centerX, centerY);
        opCtx.rotate(-pose.yaw * Math.PI / 180); 
        opCtx.strokeStyle = HYPOTHESIS_COLOR;
        opCtx.lineWidth = 1;
        opCtx.beginPath();
        opCtx.moveTo(0, 0);
        opCtx.lineTo(HYPOTHESIS_SIZE + 5, 0); 
        opCtx.stroke();
        opCtx.restore();
    });

    // AGV Position (Final Pose)
    const agv_x_cm = agvPose.x; 
    const agv_y_cm = agvPose.y; 

    const centerX = agv_x_cm * scaleX; 
    const centerY = opMapCanvas.height - (agv_y_cm * scaleY); 

    // Draw AGV body
    opCtx.fillStyle = "green";
    opCtx.beginPath();
    opCtx.arc(centerX, centerY, 5, 0, 2 * Math.PI); 
    opCtx.fill();

    // Draw AGV orientation
    opCtx.save();
    opCtx.translate(centerX, centerY);
    opCtx.rotate(-agvPose.yaw * Math.PI / 180); 
    opCtx.strokeStyle = "darkgreen";
    opCtx.lineWidth = 3;
    opCtx.beginPath();
    opCtx.moveTo(0, 0);
    opCtx.lineTo(20, 0); 
    opCtx.stroke();
    opCtx.restore();
    
    opCtx.fillText(`AGV`, centerX + 12, centerY);
}

// --- Marker Management and Local Map Persistence ---
let mapDataForSave = []; 

function updateMarkerTable() {
    // NOTE: Use the global markers array. If you are using 'window.markers' elsewhere, 
    // you must ensure consistency. 'markers' globally defined at the top is safer.
    const markersToRender = window.markers; 

    const tbody = document.querySelector("#markerTable tbody");
    if (!tbody) {
        console.error("DOM Error: Could not find tbody for #markerTable.");
        return;
    }
    
    // Check if the array is empty before continuing
    console.log(`--- DEBUG: Rendering Table ---`);
    console.log(`Markers array size for table rendering: ${markersToRender.length}`);

    tbody.innerHTML = "";
    
    // Ensure the markers array is not empty
    if (markersToRender.length === 0) {
        return; 
    }

    markersToRender.forEach((m, idx) => {
        // --- CRITICAL CHECK: Ensure m.id, m.x, m.y, and m.yaw exist and are numbers. ---
        if (m === undefined || isNaN(m.id) || isNaN(m.x) || isNaN(m.y) || isNaN(m.yaw)) {
            console.error(`Skipping invalid marker data at index ${idx}:`, m);
            return; 
        }

        const row = document.createElement("tr");
        row.innerHTML = `
            <td><input type="number" value="${m.id}" class="marker-id-input" onchange="updateMarker(${idx},'id',this.value)"></td>
            <td><input type="number" value="${m.x.toFixed(1)}" step="1" onchange="updateMarker(${idx},'x',this.value)"></td>
            <td><input type="number" value="${m.y.toFixed(1)}" step="1" onchange="updateMarker(${idx},'y',this.value)"></td>
            <td><input type="number" value="${m.yaw.toFixed(1)}" step="1" onchange="updateMarker(${idx},'yaw',this.value)"></td>
            <td><button onclick="deleteMarker(${idx})">Hapus</button></td>
        `; 
        tbody.appendChild(row);
    });
}

window.updateMarker = (idx, key, val) => {
    // Uses window.markers to update the array
    if (!window.markers) window.markers = [];

    if (key === 'id') {
        window.markers[idx][key] = parseInt(val);
    } else {
        window.markers[idx][key] = parseFloat(val);
    }
    renderMap();
};

window.deleteMarker = (idx) => {
    // Uses window.markers to splice the array
    if (!window.markers) return; 
    
    window.markers.splice(idx, 1);
    updateMarkerTable();
    renderMap();
};

if (document.getElementById("addMarkerBtn")) {
    document.getElementById("addMarkerBtn").addEventListener("click", () => {
        // Initialize window.markers if it's undefined (e.g., first run)
        if (!window.markers) window.markers = [];
        
        const markersToUse = window.markers; // Access the array that holds the map data
        
        // Find max ID from the active array
        const maxId = markersToUse.reduce((max, m) => (m.id > max ? m.id : max), -1);
        const defaultId = maxId + 1;
        
        const { width, height } = getMapScale();
        
        // ADD NEW MARKER TO window.markers
        markersToUse.push({ id: defaultId, x: width / 2, y: height / 2, yaw: 0 }); 
        
        updateMarkerTable();
        renderMap();
    });
}

// === Canvas Click ===
if (mapCanvas) {
    mapCanvas.addEventListener("click", e => {
        // Initialize window.markers if it's undefined
        if (!window.markers) window.markers = [];
        
        const markersToUse = window.markers; // Access the array that holds the map data
        
        const rect = mapCanvas.getBoundingClientRect();
        const xCanvas = e.clientX - rect.left;
        const yCanvas = e.clientY - rect.top;

        const { scaleX, scaleY, height } = getMapScale();
        
        // Find max ID from the active array
        const maxId = markersToUse.reduce((max, m) => (m.id > max ? m.id : max), -1);

        // ADD NEW MARKER TO window.markers
        markersToUse.push({
            id: maxId + 1,
            x: xCanvas / scaleX,
            y: height - (yCanvas / scaleY), 
            yaw: 0
        });
        updateMarkerTable();
        renderMap();
    });
}

// === Map Persistence (Local Storage) ===
if (document.getElementById("saveMapBtn")) {
    document.getElementById("saveMapBtn").addEventListener("click", () => {
        // FIX: Now calls the ROS service function
        saveMap();
    });
}

function loadMapData() {
    const savedData = localStorage.getItem("agv_map_data");
    if (savedData) {
        const mapData = JSON.parse(savedData);
        if (document.getElementById("mapName")) document.getElementById("mapName").value = mapData.name || "ruang_lab";
        if (document.getElementById("xLength")) document.getElementById("xLength").value = mapData.xLength || 600.0;
        if (document.getElementById("yLength")) document.getElementById("yLength").value = mapData.yLength || 600.0;
        if (document.getElementById("markerSize")) document.getElementById("markerSize").value = mapData.markerSize || 5.0;
        if (document.getElementById("markerDict")) document.getElementById("markerDict").value = mapData.dict || "DICT_4X4_50";
        markers = mapData.markers || [];
        updateMarkerTable();
        renderMap();
        renderOperationMap(); 
    }
}


// --- ROS DATABASE MANAGEMENT ---

/**
 * Converts local marker data structure (CM units) to ROS message format (CM numerical values).
 */
function updateMapVisualization() {
    const localMarkers = [];
    document.querySelectorAll('#markerTable tbody tr').forEach(row => {
        const inputs = row.querySelectorAll('input[type="number"]');
        
        // FIX: Get ID from the first input field
        const id = parseInt(inputs[0].value); 
        
        const x_ros = parseFloat(inputs[1].value);
        const y_ros = parseFloat(inputs[2].value);
        const ort_deg = parseFloat(inputs[3].value);
        
        const yaw_rad = ort_deg * (Math.PI / 180);
        
        localMarkers.push({
            id: id,
            pose: {
                pose: {
                    position: {x: x_ros, y: y_ros, z: 0}, 
                    orientation: {x: 0, y: 0, z: Math.sin(yaw_rad / 2), w: Math.cos(yaw_rad / 2)}
                }
            }
        });
    });
    window.mapDataForSave = localMarkers;
    renderMap(); 
}

function populateMapList() {
    if (!ros || !ros.isConnected) return;

    const mapSelect = document.getElementById('load_map_name');
    const loadStatusElement = document.getElementById('load_status');
    
    mapSelect.innerHTML = '<option value="" disabled selected>Loading Maps...</option>';
    loadStatusElement.innerText = 'Fetching map list...';

    const getAllMapsClient = new ROSLIB.Service({
        ros: ros,
        name: '/map_database/get_map_name',
        serviceType: 'map_database_interfaces/srv/GetMapName' 
    });

    const request = new ROSLIB.ServiceRequest({});

    getAllMapsClient.callService(request, function(result) {
        if (result.success && result.map_names && result.map_names.length > 0) {
            mapSelect.innerHTML = '<option value="" disabled selected>--- Select a Map ---</option>';
            
            result.map_names.forEach(mapName => {
                const option = document.createElement('option');
                option.value = mapName;
                option.textContent = mapName;
                mapSelect.appendChild(option);
            });

            loadStatusElement.innerText = `Found ${result.map_names.length} maps.`;
        } else {
            mapSelect.innerHTML = '<option value="" disabled selected>--- No Maps Found ---</option>';
            loadStatusElement.innerText = `No maps found or service failed: ${result.message || 'Unknown error'}`;
        }
    }, function(error) {
        mapSelect.innerHTML = '<option value="" disabled selected>--- Error Fetching Maps ---</option>';
        loadStatusElement.innerText = `Service communication error: ${error}`;
        console.error('Get All Maps Comm Error:', error);
    });
}

function saveMap() {
    if (!ros || !ros.isConnected) {
        alert("Must be connected to ROS to save the map to the database.");
        return;
    }
    
    // 1. Update the map visualization data from the HTML table (populates window.mapDataForSave)
    // This must be called immediately before validation.
    updateMapVisualization(); 

    // --- Gather Required Inputs ---
    const mapName = document.getElementById('mapName').value;
    const xLength = parseFloat(document.getElementById('xLength').value);
    const yLength = parseFloat(document.getElementById('yLength').value);
    const markerDict = document.getElementById('markerDict').value;
    const markerSize = parseFloat(document.getElementById('markerSize').value);
    const saveStatusElement = document.getElementById('save_status');
    
    // 2. Validation Check
    if (!mapName || !markerDict || isNaN(markerSize)) {
        saveStatusElement.innerText = 'Error: Please ensure Map Name, Dictionary, and Marker Size are filled correctly.';
        return;
    }

    saveStatusElement.innerText = 'Saving map...';

    // 3. Assemble the MarkerArray message (populated by updateMapVisualization)
    const markerArrayMsg = new ROSLIB.Message({
        markers: window.mapDataForSave.map(localMarker => {
            return {
                id: localMarker.id,
                pose: localMarker.pose
            }
        })
    });

    // 4. Set up Service Client and Request
    const saveMapClient = new ROSLIB.Service({
        ros: ros,
        name: '/map_database/save_map',
        serviceType: 'map_database_interfaces/srv/SaveMap' 
    });

    const request = new ROSLIB.ServiceRequest({
        map_name: mapName,
        x_length: xLength,
        y_length: yLength,
        marker_dictionary: markerDict,
        marker_size: markerSize,
        map_data: markerArrayMsg
    });

    // 5. Call Service and Handle Response
    saveMapClient.callService(request, function(result) {
        if (result.success) {
            saveStatusElement.innerText = `Success! Map '${mapName}' saved. ID: ${result.new_map_id}.`;
            // Refresh map list for the dropdown
            populateMapList(); 
        } else {
            saveStatusElement.innerText = `Failed to save map: ${result.message}`;
            // IMPORTANT: If your Python backend returns a clear error message, it will be displayed here.
            console.error('Save Map Error (Backend):', result.message);
        }
    }, function(error) {
        saveStatusElement.innerText = `Service communication error: ${error}`;
        console.error('Save Map Comm Error (Frontend):', error);
    });
}


/**
 * Implements the logic to load a map from the ROS Database and fetches the data in a chained call.
 */
function loadMap() {
    if (!ros || !ros.isConnected) {
        alert("Must be connected to ROS to load the map from the database.");
        return;
    }
    const mapName = document.getElementById('load_map_name').value;
    const loadStatusElement = document.getElementById('load_status');

    if (!mapName) {
        loadStatusElement.innerText = 'Error: Please select a map name.';
        return;
    }

    loadStatusElement.innerText = `Requesting map '${mapName}'...`; // Status before first call

    const loadMapClient = new ROSLIB.Service({
        ros: ros,
        name: '/map_database/load_map',
        serviceType: 'map_database_interfaces/srv/LoadMap' // Service returns only success/message
    });

    const request = new ROSLIB.ServiceRequest({
        map_name: mapName
    });

    // --- 1. Call LoadMap Service (Activates map on backend) ---
    loadMapClient.callService(request, function(result) {
        if (result.success) {
            loadStatusElement.innerText = `Map '${mapName}' loaded on backend. Fetching data...`;
            
            // --- 2. SUCCESS: Immediately call GetMap to fetch detailed data and update UI ---
            // Pass the mapName to ensure we update the UI's map name input field
            getAndDrawMap('mapCanvas', mapName); 
            
        } else {
            loadStatusElement.innerText = `Failed to load map on backend: ${result.message}`;
            console.error('Load Map Error:', result.message);
        }
    }, function(error) {
        loadStatusElement.innerText = `Service communication error: ${error}`;
        console.error('Load Map Comm Error:', error);
    });
}

function deleteMap() {

}

/**
 * Fetches the currently loaded map data (CM numerical values) using the GetMap service,
 * updates the UI configuration fields, and draws the map on both canvases.
 * @param {string} canvasId - The canvas ID to render to ('mapCanvas' or 'opMapCanvas').
 * @param {string} expectedMapName - The map name to use for updating the UI input field.
 */
function getAndDrawMap(canvasId, expectedMapName = null) {
    
    const processAndRender = (result) => {
        const markersArray = result.map_data.markers;
        
        console.log("--- DEBUG: GetMap Service Response ---");
        console.log("Markers received from ROS:", markersArray); // DEBUG 1

        // --- 1. Populate map configuration fields from GetMap response ---
        if (expectedMapName) {
            document.getElementById('mapName').value = expectedMapName;
        }
        
        if (result.x_length) {
            document.getElementById('xLength').value = result.x_length;
        }
        if (result.y_length) {
            document.getElementById('yLength').value = result.y_length;
        }
        
        if (result.marker_dictionary) {
            document.getElementById('markerDict').value = result.marker_dictionary;
        }
        if (result.marker_size) {
            document.getElementById('markerSize').value = result.marker_size; 
        }

        // --- 2. Process and Update Markers ---
        const tempMarkers = [];
        markersArray.forEach(marker => {
            const position = marker.pose.pose.position; 
            const orientation = marker.pose.pose.orientation;
            
            const x_cm = position.x; 
            const y_cm = position.y; 
            const yaw_deg = quaternionToYaw(orientation);
            
            tempMarkers.push({
                id: marker.id,
                x: x_cm,
                y: y_cm,
                yaw: yaw_deg
            });
        });
        window.markers = tempMarkers;
        
        // --- 3. Render on Both Pages (Fulfils the visualization requirement) ---
        updateMarkerTable();          // Calls the table renderer
        renderMap();                  // Updates Setup Map canvas
        renderOperationMap();         // Updates Operation Map canvas

        document.getElementById('load_status').innerText = `Map '${expectedMapName || 'Active Map'}' successfully loaded and displayed.`;
        console.log(`Visualization updated with ${markersArray.length} markers.`);
    };
    
    if (!ros || !ros.isConnected) return;
    const getMapClient = new ROSLIB.Service({
        ros: ros,
        name: '/map_database/get_map',
        serviceType: 'map_database_interfaces/srv/GetMap'
    });

    getMapClient.callService(new ROSLIB.ServiceRequest({}), function(result) {
        if (result.success && result.map_data && result.map_data.markers && result.map_data.markers.length > 0) {
            processAndRender(result);
        } else {
            console.warn('Get Map Warning: No active map data received or markers array is empty.');
            // Clear maps if the load failed
            window.markers = [];
            updateMarkerTable();
            renderMap();
            renderOperationMap();
            document.getElementById('load_status').innerText = `Error: Active map has no markers.`;
        }
    });
}

// --- ROS TOPIC SUBSCRIBERS ---

function subscribeToAGVPosition() {
    if (!ros) return;
    const agvPosTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/agv/pose', 
        messageType: 'geometry_msgs/Pose' 
    });

    const posXSpan = document.getElementById("posX");
    const posYSpan = document.getElementById("posY");
    const posYawSpan = document.getElementById("posYaw");

    agvPosTopic.subscribe(function(message) {
        const x = message.position.x;
        const y = message.position.y;
        const yaw = quaternionToYaw(message.orientation); 

        agvPose.x = x;
        agvPose.y = y;
        agvPose.yaw = yaw + 90;

        if (posXSpan) posXSpan.textContent = x.toFixed(1) + " cm";
        if (posYSpan) posYSpan.textContent = y.toFixed(1) + " cm";
        if (posYawSpan) posYawSpan.textContent = yaw.toFixed(1) + " °";
        
        renderOperationMap(); 
    });
}

function subscribeToPoseTopics() {
    if (!ros) return;
    
    // 1. /agv/pose_hypothesis (Hypotheses)
    const agv_pose_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/agv/pose_hypothesis',
        messageType: 'geometry_msgs/PoseArray'
    });

    agv_pose_listener.subscribe(function(message) {
        const detected_poses_div = document.getElementById('detected_poses');
        if (!detected_poses_div) return;

        agvPoseHypotheses = message.poses.map(pose => ({
            x: pose.position.x,
            y: pose.position.y,
            yaw: quaternionToYaw(pose.orientation),
        }));

        renderOperationMap();
        
        if (message.poses.length > 0) {
            let all_poses_html = '<h3>All Detected Pose Hypotheses:</h3>';
            
            message.poses.forEach((pose, index) => {
                const p = pose.position;
                const o = pose.orientation;
                const yaw = quaternionToYaw(o);
                
                all_poses_html += `
                    <p><strong>Hypothesis #${index + 1}</strong></p>
                    <ul>
                        <li>x: ${p.x.toFixed(2)} cm</li>
                        <li>y: ${p.y.toFixed(2)} cm</li>
                        <li>Yaw: ${yaw.toFixed(2)} deg</li>
                    </ul>
                    <hr style="margin: 5px 0;">
                `;
            });
            
            detected_poses_div.innerHTML = all_poses_html;
            
        } else { 
            detected_poses_div.innerHTML = '<p>No pose hypotheses received.</p>';
        }
    });

    // 2. /aruco/pose (Raw Detection)
    const aruco_pose_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/aruco/pose',
        messageType: 'aruco_msgs/MarkerArray'
    });

    aruco_pose_listener.subscribe(function(message) {
        let outputHTML = '';
        const aruco_data_div = document.getElementById('aruco_data');
        if (!aruco_data_div) return;
        
        if (message.markers.length > 0) {
            outputHTML += '<h3>Camera-Local Marker Poses:</h3>';
            message.markers.forEach(marker => {
                const position = marker.pose.pose.position;
                const orientation = marker.pose.pose.orientation;
                const yaw_deg = quaternionToYaw(orientation);

                outputHTML += `
                    <h4>ID: ${marker.id} (Local to Camera Frame)</h4>
                    <ul>
                        <li>x: ${position.x.toFixed(4)} cm</li>
                        <li>y: ${position.y.toFixed(4)} cm</li>
                        <li>Yaw: ${yaw_deg.toFixed(2)} deg</li>
                    </ul>
                `;1
            });
        } else {
            outputHTML = '<p>No ArUco markers currently detected by camera.</p>';
        }

        aruco_data_div.innerHTML = outputHTML;
    });
}


// --- INITIALIZATION ---
window.onload = function() {
    loadMapData(); 
    
    const { width, height } = getMapScale(); 
    if (mapCanvas) {
        mapCanvas.width = width * PIXEL_PER_CM;
        mapCanvas.height = height * PIXEL_PER_CM;
        renderMap();
    }
    if (opMapCanvas) {
        opMapCanvas.width = width * PIXEL_PER_CM;
        opMapCanvas.height = height * PIXEL_PER_CM;
    }

    connectRosBridge(ROSBRIDGE_URL);

    populateMapList();
};