// === TAB SWITCHING ===
const tabs = document.querySelectorAll(".tab-btn");
const panes = document.querySelectorAll(".tab-pane");
tabs.forEach(btn => {
  btn.addEventListener("click", () => {
    tabs.forEach(b => b.classList.remove("active"));
    panes.forEach(p => p.classList.remove("active"));
    btn.classList.add("active");
    document.getElementById(btn.dataset.tab).classList.add("active");
    
    // Perbarui peta saat pindah tab
    if (btn.dataset.tab === "operation") {
        renderOperationMap();
    } else {
        renderMap();
    }
  });
});

// === SETUP MAP CANVAS ===
const mapCanvas = document.getElementById("mapCanvas");
const ctx = mapCanvas.getContext("2d");
let markers = [];

// Variabel untuk Operation Map
const opMapCanvas = document.getElementById("opMapCanvas");
const opCtx = opMapCanvas.getContext("2d");
let agvPose = { x: 0, y: 0, yaw: 0 }; // Posisi AGV saat ini

// --- Fungsi Render ---
function getMapScale() {
    const width = parseFloat(document.getElementById("mapWidth").value) || 6;
    const height = parseFloat(document.getElementById("mapHeight").value) || 6;
    const scaleX = mapCanvas.width / width;
    const scaleY = mapCanvas.height / height;
    return { width, height, scaleX, scaleY };
}

function drawGrid(canvas, context, width, height, scaleX, scaleY) {
    context.strokeStyle = "#e0e0e0";
    for (let i = 0; i <= width; i++) {
        const x = i * scaleX;
        context.beginPath();
        context.moveTo(x, 0);
        context.lineTo(x, canvas.height);
        context.stroke();
    }
    for (let j = 0; j <= height; j++) {
        const y = j * scaleY;
        context.beginPath();
        context.moveTo(0, y);
        context.lineTo(canvas.width, y);
        context.stroke();
    }
}

function renderMap() {
    const { width, height, scaleX, scaleY } = getMapScale();
    ctx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
    drawGrid(mapCanvas, ctx, width, height, scaleX, scaleY);

    // Marker
    markers.forEach(m => {
        ctx.fillStyle = "blue";
        const centerX = m.x * scaleX;
        const centerY = m.y * scaleY;

        ctx.beginPath();
        ctx.arc(centerX, centerY, 6, 0, 2 * Math.PI);
        ctx.fill();
        ctx.fillText(`ID ${m.id} (${m.yaw}°)`, centerX + 8, centerY - 8);
        
        // Orientasi (Panah kecil)
        ctx.save();
        ctx.translate(centerX, centerY);
        ctx.rotate(-m.yaw * Math.PI / 180); // Putar berdasarkan yaw (negatif karena Y-axis canvas terbalik)
        ctx.strokeStyle = "red";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(15, 0); // Panah ke arah X positif (0 deg)
        ctx.stroke();
        ctx.restore();
    });
}

function renderOperationMap() {
    const { width, height, scaleX, scaleY } = getMapScale();
    opCtx.clearRect(0, 0, opMapCanvas.width, opMapCanvas.height);
    drawGrid(opMapCanvas, opCtx, width, height, scaleX, scaleY);

    // Gambar Marker yang sudah di-setup
    markers.forEach(m => {
        opCtx.fillStyle = "gray";
        opCtx.beginPath();
        opCtx.arc(m.x * scaleX, m.y * scaleY, 5, 0, 2 * Math.PI);
        opCtx.fill();
    });

    // Gambar Posisi AGV (Diperoleh dari ROS)
    const centerX = agvPose.x * scaleX;
    const centerY = agvPose.y * scaleY;

    opCtx.fillStyle = "green";
    opCtx.beginPath();
    opCtx.arc(centerX, centerY, 10, 0, 2 * Math.PI); // Lingkaran besar untuk AGV
    opCtx.fill();

    // Orientasi AGV
    opCtx.save();
    opCtx.translate(centerX, centerY);
    opCtx.rotate(-agvPose.yaw * Math.PI / 180); 
    opCtx.strokeStyle = "darkgreen";
    opCtx.lineWidth = 3;
    opCtx.beginPath();
    opCtx.moveTo(0, 0);
    opCtx.lineTo(20, 0); // Panah ke arah orientasi AGV
    opCtx.stroke();
    opCtx.restore();
    
    opCtx.fillText(`AGV`, centerX + 12, centerY);
}

// Event listener untuk perubahan ukuran map
document.getElementById("mapWidth").addEventListener("change", renderMap);
document.getElementById("mapHeight").addEventListener("change", renderMap);


// === Tabel marker (Manual Input) ===
function updateMarkerTable() {
    const tbody = document.querySelector("#markerTable tbody");
    tbody.innerHTML = "";
    markers.forEach((m, idx) => {
        const row = document.createElement("tr");
        row.innerHTML = `
            <td>${m.id}</td>
            <td><input type="number" value="${m.x.toFixed(2)}" step="0.1" onchange="updateMarker(${idx},'x',this.value)"></td>
            <td><input type="number" value="${m.y.toFixed(2)}" step="0.1" onchange="updateMarker(${idx},'y',this.value)"></td>
            <td><input type="number" value="${m.yaw.toFixed(1)}" step="1" onchange="updateMarker(${idx},'yaw',this.value)"></td>
            <td><button onclick="deleteMarker(${idx})">Hapus</button></td>
        `;
        tbody.appendChild(row);
    });
}

window.updateMarker = (idx, key, val) => {
    markers[idx][key] = parseFloat(val);
    renderMap();
};

window.deleteMarker = (idx) => {
    markers.splice(idx, 1);
    // Perbarui ID setelah penghapusan
    markers.forEach((m, i) => m.id = i);
    updateMarkerTable();
    renderMap();
};

document.getElementById("addMarkerBtn").addEventListener("click", () => {
    const maxId = markers.reduce((max, m) => (m.id > max ? m.id : max), -1);
    markers.push({ id: maxId + 1, x: 0, y: 0, yaw: 0 });
    updateMarkerTable();
    renderMap();
});

// === Klik canvas untuk menambahkan marker posisi ===
mapCanvas.addEventListener("click", e => {
    const rect = mapCanvas.getBoundingClientRect();
    const xCanvas = e.clientX - rect.left;
    const yCanvas = e.clientY - rect.top;

    const { scaleX, scaleY } = getMapScale();
    const maxId = markers.reduce((max, m) => (m.id > max ? m.id : max), -1);

    markers.push({
        id: maxId + 1,
        x: xCanvas / scaleX,
        y: yCanvas / scaleY,
        yaw: 0
    });
    updateMarkerTable();
    renderMap();
});

// === Simpan Map & Muat Map dari Local Storage ===
document.getElementById("saveMapBtn").addEventListener("click", () => {
    const mapData = {
        name: document.getElementById("mapName").value,
        width: document.getElementById("mapWidth").value,
        height: document.getElementById("mapHeight").value,
        markerSize: document.getElementById("markerSize").value,
        dict: document.getElementById("markerDict").value,
        markers
    };
    localStorage.setItem("agv_map_data", JSON.stringify(mapData));
    alert("Map disimpan!");
});

function loadMapData() {
    const savedData = localStorage.getItem("agv_map_data");
    if (savedData) {
        const mapData = JSON.parse(savedData);
        document.getElementById("mapName").value = mapData.name || "ruang_lab";
        document.getElementById("mapWidth").value = mapData.width || 6.0;
        document.getElementById("mapHeight").value = mapData.height || 6.0;
        document.getElementById("markerSize").value = mapData.markerSize || 0.05;
        document.getElementById("markerDict").value = mapData.dict || "DICT_4X4_50";
        markers = mapData.markers || [];
        updateMarkerTable();
        renderMap();
        alert("Map berhasil dimuat dari penyimpanan lokal.");
    }
}


// ------------------------------------------------------------------
// === FITUR BARU: UPLOAD & PROSES MARKER ===
// ------------------------------------------------------------------
document.getElementById("processMarkerImageBtn").addEventListener("click", () => {
    const fileInput = document.getElementById("markerImageUpload");
    if (fileInput.files.length === 0) {
        alert("Pilih gambar marker terlebih dahulu!");
        return;
    }

    // --- LOGIKA MOCKUP UNTUK DEMONSTRASI ---
    // Deteksi Aruco memerlukan Backend Server (Python/OpenCV).
    // Kode ini hanya untuk mendemonstrasikan penambahan marker setelah proses.
    const mapWidth = parseFloat(document.getElementById("mapWidth").value);
    const mapHeight = parseFloat(document.getElementById("mapHeight").value);

    // Contoh data hasil deteksi dari server (Random untuk Mockup)
    const mockNewMarkers = [
        { id: 1, x: parseFloat((Math.random() * mapWidth).toFixed(2)), y: parseFloat((Math.random() * mapHeight).toFixed(2)), yaw: 0 },
        { id: 2, x: parseFloat((Math.random() * mapWidth).toFixed(2)), y: parseFloat((Math.random() * mapHeight).toFixed(2)), yaw: 90 },
        { id: 3, x: parseFloat((Math.random() * mapWidth).toFixed(2)), y: parseFloat((Math.random() * mapHeight).toFixed(2)), yaw: 180 }
    ];

    // Ganti/Gabungkan markers yang sudah ada
    markers = mockNewMarkers; 
    updateMarkerTable();
    renderMap();

    alert(`Mockup: Marker terdeteksi (${mockNewMarkers.length} marker) dan ditambahkan ke list. Dalam implementasi nyata, kirim file ke server.`);
});


// ------------------------------------------------------------------
// === FITUR BARU: ROS BRIDGE & OPERASI KAMERA ===
// ------------------------------------------------------------------
const connectRosBtn = document.getElementById("connectRosBtn");
const rosStatus = document.getElementById("rosStatus");
const cameraFeed = document.getElementById("cameraFeed");
const rosBridgeUrlInput = document.getElementById("rosBridgeUrl");
let ros = null;

connectRosBtn.addEventListener("click", () => {
    if (ros && ros.isConnected) {
        ros.close();
        ros = null;
    } else {
        const url = rosBridgeUrlInput.value;
        if (url.trim() === "") {
            alert("Masukkan URL ROS-bridge!");
            return;
        }
        rosStatus.textContent = "Status: Connecting...";
        connectRosBridge(url);
    }
});

function connectRosBridge(url) {
    ros = new ROSLIB.Ros({
        url: url
    });

    ros.on('connection', () => {
        rosStatus.textContent = "Status: Connected ✅";
        connectRosBtn.textContent = "Disconnect Camera";
        subscribeToCameraFeed();
        subscribeToAGVPosition();
    });

    ros.on('error', (error) => {
        rosStatus.textContent = "Status: Error ❌. Cek konsol dan URL.";
        connectRosBtn.textContent = "Connect Camera (ROS)";
        console.error('ROS-bridge Error: ', error);
        ros = null;
    });

    ros.on('close', () => {
        rosStatus.textContent = "Status: Disconnected";
        connectRosBtn.textContent = "Connect Camera (ROS)";
        ros = null;
    });
}

// Function konversi Quaternion ke Yaw (Euler Z)
function quaternionToYaw(q) {
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return Math.atan2(siny_cosp, cosy_cosp) * 180 / Math.PI; // Kembali dalam derajat
}

function subscribeToCameraFeed() {
    // Ganti dengan topic gambar ROS kamu (misal: /usb_cam/image_raw/compressed)
    const imageTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/camera/image_raw/compressed', 
        messageType: 'sensor_msgs/CompressedImage'
    });

    imageTopic.subscribe(function(message) {
        // Format: data:image/[format];base64,[data]
        cameraFeed.src = "data:image/" + message.format.split(';')[0] + ";base64," + message.data;
    });
}

function subscribeToAGVPosition() {
    // Asumsi AGV Pose di-publish ke /agv/pose (geometry_msgs/PoseStamped)
    const agvPosTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/agv/pose', 
        messageType: 'geometry_msgs/PoseStamped' 
    });

    const posXSpan = document.getElementById("posX");
    const posYSpan = document.getElementById("posY");
    const posYawSpan = document.getElementById("posYaw");

    agvPosTopic.subscribe(function(message) {
        const x = message.pose.position.x;
        const y = message.pose.position.y;
        const yaw = quaternionToYaw(message.pose.orientation); 

        // Update Global Pose dan UI
        agvPose.x = x;
        agvPose.y = y;
        agvPose.yaw = yaw;

        posXSpan.textContent = x.toFixed(2) + " m";
        posYSpan.textContent = y.toFixed(2) + " m";
        posYawSpan.textContent = yaw.toFixed(1) + " °";
        
        // Update visualisasi navigasi real-time
        renderOperationMap(); 
    });
}

// Render awal setelah memuat data
loadMapData(); 
renderMap();