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

// --- HOVER STATE MANAGEMENT ---
let hoveredMarkerSetup = null;
let hoveredMarkerOperation = null;
const mapTooltip = document.getElementById("mapTooltip"); // Ambil elemen tooltip

// --- ROS3D Globals ---
let map3dViewer = null;
let op3dViewer = null;
let tfClient = null; // Single TF client for all viewers

// --- DOM ELEMENTS ---
const rosStatus = document.getElementById("rosStatus");
const cameraFeed = document.getElementById("cameraFeed");
const tabs = document.querySelectorAll(".tab-btn");
const panes = document.querySelectorAll(".tab-pane");

let ros = null; 
const MJPEG_PORT = '8080'; 
const ROSBRIDGE_PORT = '9090';
const ROSBRIDGE_URL = `ws://localhost:${ROSBRIDGE_PORT}`;
const MJPEG_TOPIC = '/camera/image_marked';

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
             const mjpegUrl = `http://localhost:${MJPEG_PORT}/stream?topic=${MJPEG_TOPIC}`;
             cameraFeed.src = mjpegUrl;
             cameraFeed.alt = `Live stream from ${MJPEG_TOPIC}`;
             console.log(`Video feed source set to: ${mjpegUrl}`);
        }
        
        subscribeToAGVPosition(); 
        subscribeToPoseTopics();  
        populateMapList(); 
        initRos3D(ros);
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
    const markerSize = parseFloat(document.getElementById("markerSize").value) || 5.0; // Ambil markerSize
    
    const scaleX = mapCanvas.width / width;
    const scaleY = mapCanvas.height / height;
    return { width, height, scaleX, scaleY, markerSize }; // Sertakan markerSize
}

function drawAxisLabels(canvas, context, width, height, scaleX, scaleY) { 
    if (!context || !canvas) return;
    context.font = "10px Arial";
    context.fillStyle = "#777";

    // Draw Y-Axis Labels (Vertical Scale)
    for (let j = 0; j <= height; j += 100) { 
        const y = j * scaleY;
        context.fillText((height - j).toFixed(0) + "cm", 5, y - 5); // Label on the left edge
    }
    
    // Draw X-Axis Labels (Horizontal Scale)
    for (let i = 0; i <= width; i += 100) { 
        const x = i * scaleX;
        context.fillText(i.toFixed(0) + "cm", x + 5, canvas.height - 5); // Label on the bottom edge
    }
    
    // Add corner labels (Width and Height dimensions)
    context.fillStyle = "#333";
    context.font = "12px Arial bold";
    context.fillText(`X: ${width.toFixed(0)} cm`, canvas.width - 60, canvas.height - 10);
    context.fillText(`Y: ${height.toFixed(0)} cm`, 5, 15);
}

function renderMap() {
    if (!ctx || !mapCanvas) return;
    
    const markersToDraw = window.markers || [];
    // Dapatkan markerSize dari getMapScale
    const { width, height, scaleX, scaleY, markerSize } = getMapScale(); 
    
    ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    drawAxisLabels(mapCanvas, ctx, width, height, scaleX, scaleY); // Gunakan fungsi baru

    // --- Draw Static Markers (Hollow Square with Axis) ---
    markersToDraw.forEach(m => {
        const centerX = m.x * scaleX;
        const centerY = mapCanvas.height - (m.y * scaleY); 
        
        // Gunakan markerSize (dalam cm) yang diskalakan ke piksel
        const MARKER_DRAW_SIZE = markerSize * scaleX; 
        const AXIS_LENGTH = MARKER_DRAW_SIZE / 2 + 5; // Panjang sumbu orientasi

        const LINE_COLOR = "#3498db"; // Blue outline
        const AXIS_COLOR = "#e74c3c"; // Red for orientation

        // Draw Marker Body (Hollow Square)
        ctx.strokeStyle = LINE_COLOR; 
        ctx.lineWidth = 2;
        ctx.strokeRect(centerX - MARKER_DRAW_SIZE/2, centerY - MARKER_DRAW_SIZE/2, MARKER_DRAW_SIZE, MARKER_DRAW_SIZE);

        // Draw Center Dot
        ctx.fillStyle = LINE_COLOR;
        ctx.beginPath();
        ctx.arc(centerX, centerY, 2, 0, 2 * Math.PI);
        ctx.fill();
        
        // Draw Orientation Axis (Thick Line)
        ctx.save();
        ctx.translate(centerX, centerY);
        ctx.rotate(-m.yaw * Math.PI / 180); 
        ctx.strokeStyle = AXIS_COLOR; 
        ctx.lineWidth = 3; 
        
        ctx.beginPath();
        // Sumbu dimulai dari tengah
        ctx.moveTo(0, 0); 
        ctx.lineTo(AXIS_LENGTH, 0); 
        ctx.stroke();
        
        // Draw simple arrowhead (small triangle)
        ctx.fillStyle = AXIS_COLOR;
        ctx.beginPath();
        ctx.moveTo(AXIS_LENGTH, 0);
        ctx.lineTo(AXIS_LENGTH - 5, 3);
        ctx.lineTo(AXIS_LENGTH - 5, -3);
        ctx.closePath();
        ctx.fill();
        
        ctx.restore();
        
        // DRAW TEXT LABEL (Hanya ID yang selalu muncul)
        ctx.fillStyle = "#333";
        ctx.fillText(`ID ${m.id}`, centerX + AXIS_LENGTH + 2, centerY - 5);
        
    });
    // Kebutuhan hover diurus oleh setupCanvasHover()
    console.log(`Map Setup rendered with ${markersToDraw.length} markers.`);
}


function renderOperationMap() {
    if (!opCtx || !opMapCanvas) return;
    
    const markersToDraw = window.markers || [];
    // Dapatkan markerSize dari getMapScale
    const { width, height, scaleX, scaleY, markerSize } = getMapScale(); 
    opCtx.clearRect(0, 0, opMapCanvas.width, opMapCanvas.height);
    drawAxisLabels(opMapCanvas, opCtx, width, height, scaleX, scaleY); // Gunakan fungsi baru

    // Static Markers (Using a lighter, scaled hollow square)
    markersToDraw.forEach(m => {
        const centerX = m.x * scaleX;
        const centerY = opMapCanvas.height - (m.y * scaleY); 
        
        // Gunakan 80% dari ukuran marker sebenarnya untuk representasi visual yang lebih kecil di Operation Map
        const MARKER_DRAW_SIZE = markerSize * scaleX * 0.8; 
        const AXIS_LENGTH = MARKER_DRAW_SIZE / 2 + 5;
        
        // Draw Marker Body (Lighter Hollow Square)
        opCtx.strokeStyle = "#95a5a6"; // Gray outline
        opCtx.lineWidth = 1;
        opCtx.strokeRect(centerX - MARKER_DRAW_SIZE/2, centerY - MARKER_DRAW_SIZE/2, MARKER_DRAW_SIZE, MARKER_DRAW_SIZE);
        
        opCtx.fillStyle = "#333";
        // DRAW TEXT LABEL (Hanya ID yang selalu muncul)
        opCtx.fillText(`ID ${m.id}`, centerX + AXIS_LENGTH + 2, centerY - 5);
    });

    // --- Hypothesis Visualization (Small Cross/X remains) ---
    const HYPOTHESIS_SIZE = 5;
    const HYPOTHESIS_COLOR = 'rgba(0, 150, 255, 0.5)';
    const HYPOTHESIS_ORIENTATION_LENGTH = 10;
    
    agvPoseHypotheses.forEach(pose => {
        const hyp_x_cm = pose.x;
        const hyp_y_cm = pose.y;

        const centerX = hyp_x_cm * scaleX; 
        const centerY = opMapCanvas.height - (hyp_y_cm * scaleY); 
        
        if (isNaN(centerX) || isNaN(centerY) || centerX < -50 || centerX > opMapCanvas.width + 50 || centerY < -50 || centerY > opMapCanvas.height + 50) {
            console.warn(`WARNING: Hypothesis skipped (invalid coordinates): X=${hyp_x_cm.toFixed(2)}, Y=${hyp_y_cm.toFixed(2)}`);
            return; 
        }

        // Draw Hypothesis Body (Small Cross/X)
        opCtx.strokeStyle = HYPOTHESIS_COLOR;
        opCtx.lineWidth = 1;
        
        opCtx.beginPath();
        opCtx.moveTo(centerX - 3, centerY - 3);
        opCtx.lineTo(centerX + 3, centerY + 3);
        opCtx.moveTo(centerX + 3, centerY - 3);
        opCtx.lineTo(centerX - 3, centerY + 3);
        opCtx.stroke();
        
        // Draw Hypothesis Orientation (Short Line)
        opCtx.save();
        opCtx.translate(centerX, centerY);
        opCtx.rotate(-pose.yaw * Math.PI / 180); 
        opCtx.beginPath();
        opCtx.moveTo(0, 0);
        opCtx.lineTo(HYPOTHESIS_ORIENTATION_LENGTH, 0); 
        opCtx.stroke();
        opCtx.restore();
    });

    // --- AGV Position (Final Pose - Arrowhead) ---
    const AGV_BODY_LENGTH = 25; 
    const AGV_BODY_WIDTH = 15;
    
    const agv_x_cm = agvPose.x; 
    const agv_y_cm = agvPose.y; 

    const centerX = agv_x_cm * scaleX; 
    const centerY = opMapCanvas.height - (agv_y_cm * scaleY); 

    // Draw AGV body (Arrowhead/Triangle)
    opCtx.save();
    opCtx.translate(centerX, centerY);
    opCtx.rotate(-agvPose.yaw * Math.PI / 180); 

    opCtx.fillStyle = "#2ecc71"; // Primary green
    opCtx.strokeStyle = "#27ae60";
    opCtx.lineWidth = 1;
    
    opCtx.beginPath();
    opCtx.moveTo(AGV_BODY_LENGTH, 0); // Point
    opCtx.lineTo(-AGV_BODY_LENGTH/3, -AGV_BODY_WIDTH / 2);
    opCtx.lineTo(-AGV_BODY_LENGTH/3, AGV_BODY_WIDTH / 2);
    opCtx.closePath();
    opCtx.fill();
    opCtx.stroke();
    
    opCtx.restore();
    
    opCtx.fillStyle = "#333";
    opCtx.fillText(`AGV`, centerX + 12, centerY);
}

// Helper to calculate Euclidean distance (for collision check)
function getDistance(p1, p2) {
    return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
}

// Universal function to set up hover detection on Canvas
function setupCanvasHover(canvas, renderFunction, mapType) {
    if (!canvas || !mapTooltip) return;

    // targetX and targetY are now the screen coordinates of the marker center
    const setHoveredMarker = (marker, targetX, targetY) => { 
        let currentHoveredMarker = (mapType === 'setup') ? hoveredMarkerSetup : hoveredMarkerOperation;
        
        // Cek apakah ada perubahan status hover
        if (marker !== currentHoveredMarker) {
            if (mapType === 'setup') {
                hoveredMarkerSetup = marker;
            } else {
                hoveredMarkerOperation = marker;
            }
            renderFunction();
        }

        // Tampilkan atau sembunyikan tooltip
        if (marker) {
            const tooltipHTML = `
                <strong>ID: ${marker.id}</strong><br>
                X: ${marker.x.toFixed(2)} cm<br>
                Y: ${marker.y.toFixed(2)} cm<br>
                Yaw: ${marker.yaw.toFixed(2)} °
            `;
            mapTooltip.innerHTML = tooltipHTML;
            mapTooltip.style.display = 'block';

            // Posisikan tooltip relatif terhadap viewport (menggunakan targetX/Y Marker)
            const offsetX = 10; // Offset dari pusat Marker
            const offsetY = 10;
            
            // Posisi awal: Geser ke kanan bawah dari pusat marker
            let tooltipX = targetX + offsetX; 
            let tooltipY = targetY + offsetY;
            
            // Penyesuaian agar tidak keluar dari kanan layar (Edge check)
            if (tooltipX + mapTooltip.offsetWidth > window.innerWidth - 10) {
                // Posisikan ke kiri marker
                tooltipX = targetX - mapTooltip.offsetWidth - offsetX;
            }
            // Penyesuaian agar tidak keluar dari bawah layar (Edge check)
            if (tooltipY + mapTooltip.offsetHeight > window.innerHeight - 10) {
                 // Posisikan ke atas marker
                 tooltipY = targetY - mapTooltip.offsetHeight - offsetY; 
            }

            mapTooltip.style.left = `${tooltipX}px`;
            mapTooltip.style.top = `${tooltipY}px`;
            
            canvas.style.cursor = 'pointer';

        } else {
            mapTooltip.style.display = 'none';
            canvas.style.cursor = 'default';
        }
    };

    canvas.addEventListener('mousemove', (e) => {
        const rect = canvas.getBoundingClientRect();
        const xCanvas = e.clientX - rect.left;
        const yCanvas = e.clientY - rect.top;

        const { scaleX, scaleY, height, markerSize } = getMapScale();
        
        // Convert mouse position from pixels to map coordinates (cm)
        const x_cm = xCanvas / scaleX;
        const y_cm = height - (yCanvas / scaleY);
        
        // Radius deteksi dalam cm (Radius sekitar 75% dari sisi marker)
        const detectionRadiusCm = markerSize * 0.75; 

        let detectedMarker = null;

        // Check if cursor is over any marker
        for (const m of window.markers) {
            const distance = getDistance(
                { x: m.x, y: m.y }, 
                { x: x_cm, y: y_cm }
            );

            if (distance <= detectionRadiusCm) {
                detectedMarker = m;

                // Hitung posisi Marker di viewport (screen)
                const markerXCanvas = m.x * scaleX; 
                const markerYCanvas = canvas.height - (m.y * scaleY);
                const markerXViewport = markerXCanvas + rect.left;
                const markerYViewport = markerYCanvas + rect.top;

                setHoveredMarker(detectedMarker, markerXViewport, markerYViewport);
                return; // Stop checking once found
            }
        }
        
        // Jika tidak ada marker yang terdeteksi, sembunyikan tooltip
        setHoveredMarker(null, e.clientX, e.clientY);
    });
    
    canvas.addEventListener('mouseout', () => {
        setHoveredMarker(null, 0, 0); // Hide tooltip on mouse exit
    });
}


// --- Marker Management and Local Map Persistence (Code remains the same) ---
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
            <td><button onclick="deleteMarker(${idx})">Delete</button></td>
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
    
    mapSelect.innerHTML = '<option value="" disabled selected>Loading Maps...</option>';
    showInfoToast('Fetching map list from database...', 'Loading');

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

            showSuccessToast(`Found ${result.map_names.length} map(s) in database`, 'Maps Loaded');
        } else {
            mapSelect.innerHTML = '<option value="" disabled selected>--- No Maps Found ---</option>';
            showWarningToast(result.message || 'No maps available in database', 'No Maps Found');
        }
    }, function(error) {
        mapSelect.innerHTML = '<option value="" disabled selected>--- Error Fetching Maps ---</option>';
        showErrorToast(`Database communication error: ${error}`, 'Connection Failed');
        console.error('Get All Maps Comm Error:', error);
    });
}

function saveMap() {
    if (!ros || !ros.isConnected) {
        showErrorToast('Please connect to ROS bridge before saving', 'Not Connected');
        return;
    }
    
    updateMapVisualization();

    const mapName = document.getElementById('mapName').value;
    const xLength = parseFloat(document.getElementById('xLength').value);
    const yLength = parseFloat(document.getElementById('yLength').value);
    const markerDict = document.getElementById('markerDict').value;
    const markerSize = parseFloat(document.getElementById('markerSize').value);
    
    if (!mapName || !markerDict || isNaN(markerSize)) {
        showErrorToast('Please ensure Map Name, Dictionary, and Marker Size are filled correctly', 'Validation Error');
        return;
    }

    showInfoToast(`Saving map '${mapName}' to database...`, 'Saving');

    const markerArrayMsg = new ROSLIB.Message({
        markers: window.mapDataForSave.map(localMarker => {
            return {
                id: localMarker.id,
                pose: localMarker.pose
            }
        })
    });

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

    saveMapClient.callService(request, function(result) {
        if (result.success) {
            showSuccessToast(`Map '${mapName}' saved successfully with ID: ${result.new_map_id}`, 'Map Saved');
            populateMapList(); 
        } else {
            showErrorToast(result.message, 'Save Failed');
            console.error('Save Map Error (Backend):', result.message);
        }
    }, function(error) {
        showErrorToast(`Database communication error: ${error}`, 'Connection Failed');
        console.error('Save Map Comm Error (Frontend):', error);
    });
}


/**
 * Implements the logic to load a map from the ROS Database and fetches the data in a chained call.
 */
function loadMap() {
    if (!ros || !ros.isConnected) {
        showErrorToast('Please connect to ROS bridge before loading', 'Not Connected');
        return;
    }
    const mapName = document.getElementById('load_map_name').value;

    if (!mapName) {
        showWarningToast('Please select a map from the dropdown', 'No Map Selected');
        return;
    }

    showInfoToast(`Loading map '${mapName}' from database...`, 'Loading');

    const loadMapClient = new ROSLIB.Service({
        ros: ros,
        name: '/map_database/load_map',
        serviceType: 'map_database_interfaces/srv/LoadMap'
    });

    const request = new ROSLIB.ServiceRequest({
        map_name: mapName
    });

    loadMapClient.callService(request, function(result) {
        if (result.success) {
            showInfoToast(`Map '${mapName}' activated. Fetching details...`, 'Loading');
            getAndDrawMap('mapCanvas', mapName); 
        } else {
            showErrorToast(result.message, 'Load Failed');
            console.error('Load Map Error:', result.message);
        }
    }, function(error) {
        showErrorToast(`Database communication error: ${error}`, 'Connection Failed');
        console.error('Load Map Comm Error:', error);
    });
}

function deleteMap() {
    if (!ros || !ros.isConnected) {
        showErrorToast('Please connect to ROS bridge before deleting', 'Not Connected');
        return;
    }
    const mapName = document.getElementById('load_map_name').value;

    if (!mapName) {
        showWarningToast('Please select a map to delete', 'No Map Selected');
        return;
    }

    if (!confirm(`Are you sure you want to delete map '${mapName}'? This action cannot be undone.`)) {
        return;
    }

    showInfoToast(`Deleting map '${mapName}'...`, 'Deleting');

    const deleteMapClient = new ROSLIB.Service({
        ros: ros,
        name: '/map_database/delete_map',
        serviceType: 'map_database_interfaces/srv/DeleteMap' 
    });

    const request = new ROSLIB.ServiceRequest({
        map_name: mapName
    });

    deleteMapClient.callService(request, function(result) {
        if (result.success) {
            showSuccessToast(`Map '${mapName}' successfully deleted`, 'Map Deleted');
            populateMapList(); 
        } else {
            showErrorToast(result.message, 'Delete Failed');
            console.error('Delete Map Error:', result.message);
        }
    }, function(error) {
        showErrorToast(`Database communication error: ${error}`, 'Connection Failed');
        console.error('Delete Map Comm Error:', error);
    });
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

        showSuccessToast(`Map '${expectedMapName || 'Active Map'}' loaded with ${markersArray.length} marker(s)`, 'Map Loaded');
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
            showWarningToast('The loaded map contains no markers', 'Empty Map');
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
        setupCanvasHover(mapCanvas, renderMap, 'setup'); // Setup hover for Setup Map
        
        // --- NEW LISTENERS FOR CONFIGURATION CHANGES ---
        const inputsToMonitor = ["xLength", "yLength", "markerSize"];
        inputsToMonitor.forEach(id => {
            const input = document.getElementById(id);
            if (input) {
                // Use 'input' event for immediate visual feedback when typing/dragging number controls
                input.addEventListener("input", renderMap);
                // Also add 'change' event in case 'input' doesn't fire immediately
                input.addEventListener("change", renderMap); 
            }
        });
        // --- END NEW LISTENERS ---
    }
    if (opMapCanvas) {
        opMapCanvas.width = width * PIXEL_PER_CM;
        opMapCanvas.height = height * PIXEL_PER_CM;
        setupCanvasHover(opMapCanvas, renderOperationMap, 'operation'); // Setup hover for Operation Map
    }

    connectRosBridge(ROSBRIDGE_URL);

    populateMapList();
};

function initRos3D(rosInstance) {
    const viewerDiv = document.getElementById('ros3d_viewer');
    if (!viewerDiv) {
        console.warn('ROS3D Viewer DIV not found. Skipping 3D initialization.');
        return;
    }
    
    // 1. Initialize the 3D Viewer
    // NOTE: Dimensions are fixed to 600x400 as set in HTML
    var ros3d_viewer = new ROS3D.Viewer({
        divID: 'ros3d_viewer',
        width: viewerDiv.offsetWidth, 
        height: viewerDiv.offsetHeight,
        cameraPosition : {x: 15.0, y: 15.0, z: 15.0}, // Zoom way out (15 meters)
        background: '#333333',
        antialias: true,
    });
    
    // 2. Add the TFClient to listen for /tf
    var tfClient = new ROSLIB.TFClient({
        ros: rosInstance,
        angularThres: 0.01,
        transThres: 0.01,
        rate: 10.0,
        fixedFrame: 'world' // CRITICAL: Must match the world frame used in your Python code
    });

    // 3. Add a visualization stream for your Aruco Markers (MarkerArray)
    markerArrayClient = new ROS3D.MarkerArrayClient({
        ros: rosInstance,
        tfClient: tfClient,
        topic: '/aruco/visual', // Topic from your Python code
        rootObject: ros3d_viewer.scene,
        // Optional: Use a large lifetime so markers stay visible until the map is reloaded
        lifetime: 0 
    });
    
    // 4. Add a visualization stream for the final AGV Pose (as an Arrow)
    // NOTE: We don't need a separate PoseClient for /agv/pose_hypothesis unless we want
    // to visualize the individual hypothesis poses differently than the markers.
// 5. Visualize the AGV/Camera frame using an Axis object
    new ROS3D.Arrow({
        ros : rosInstance,
        tfClient : tfClient,
        rootObject : ros3d_viewer.scene,
        frameID : 'camera', // Attach the arrow to the 'camera' frame
        // Arrow geometry settings:
        length: 0.2, // 20cm
        headLength: 0.05,
        shaftDiameter: 0.02,
        headDiameter: 0.04,
        color: 0xFF0000 // Red arrow to represent AGV/Camera
    });
    
    console.log('ROS3D Viewer initialized and subscribing to /tf and /aruco/visual.');
}