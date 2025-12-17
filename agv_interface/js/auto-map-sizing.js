// ===================================================================
// === AUTO MAP SIZING BASED ON MARKER COORDINATES ===
// ===================================================================

/**
 * Calculates optimal map bounds based on marker coordinates
 * Uses a fixed grid system of 100cm intervals
 */
function calculateMapBounds(markers) {
    const GRID_SIZE = 20; // 100 cm grid
    const MIN_GRID_COUNT = 6; // Minimum 6x6 grids (600cm x 600cm)
    
    if (!markers || markers.length === 0) {
        // Default: centered at origin with minimum size
        return {
            minX: -300,
            maxX: 300,
            minY: -300,
            maxY: 300,
            width: 600,
            height: 600,
            centerX: 0,
            centerY: 0,
            gridSize: GRID_SIZE
        };
    }

    // Find min and max coordinates
    let minX = Infinity;
    let maxX = -Infinity;
    let minY = Infinity;
    let maxY = -Infinity;

    markers.forEach(marker => {
        if (marker.x < minX) minX = marker.x;
        if (marker.x > maxX) maxX = marker.x;
        if (marker.y < minY) minY = marker.y;
        if (marker.y > maxY) maxY = marker.y;
    });

    // Add padding (at least 1 grid on each side)
    const paddingX = Math.max(GRID_SIZE * 1.5, (maxX - minX) * 0.15);
    const paddingY = Math.max(GRID_SIZE * 1.5, (maxY - minY) * 0.15);

    // Calculate raw bounds
    let rawMinX = minX - paddingX;
    let rawMaxX = maxX + paddingX;
    let rawMinY = minY - paddingY;
    let rawMaxY = maxY + paddingY;

    // Snap to grid boundaries (round to nearest 100cm)
    minX = Math.floor(rawMinX / GRID_SIZE) * GRID_SIZE;
    maxX = Math.ceil(rawMaxX / GRID_SIZE) * GRID_SIZE;
    minY = Math.floor(rawMinY / GRID_SIZE) * GRID_SIZE;
    maxY = Math.ceil(rawMaxY / GRID_SIZE) * GRID_SIZE;

    // Ensure minimum size
    const width = Math.max(maxX - minX, GRID_SIZE * MIN_GRID_COUNT);
    const height = Math.max(maxY - minY, GRID_SIZE * MIN_GRID_COUNT);

    // Adjust if needed to meet minimum
    if (maxX - minX < width) {
        const diff = width - (maxX - minX);
        minX -= diff / 2;
        maxX += diff / 2;
    }
    if (maxY - minY < height) {
        const diff = height - (maxY - minY);
        minY -= diff / 2;
        maxY += diff / 2;
    }

    return {
        minX,
        maxX,
        minY,
        maxY,
        width,
        height,
        centerX: (minX + maxX) / 2,
        centerY: (minY + maxY) / 2,
        gridSize: GRID_SIZE
    };
}

/**
 * Updates the hidden xLength and yLength fields based on markers
 * This ensures compatibility with existing ROS backend
 */
function updateMapDimensions() {
    const markers = window.markers || [];
    const bounds = calculateMapBounds(markers);
    
    // Update hidden fields for backend compatibility
    const xLengthInput = document.getElementById('xLength');
    const yLengthInput = document.getElementById('yLength');
    
    if (xLengthInput) xLengthInput.value = Math.ceil(bounds.width);
    if (yLengthInput) yLengthInput.value = Math.ceil(bounds.height);
    
    // Store bounds globally for rendering
    window.mapBounds = bounds;
    
    console.log(`Auto-calculated map dimensions: ${bounds.width.toFixed(0)} x ${bounds.height.toFixed(0)} cm`);
    console.log(`Bounds: X[${bounds.minX.toFixed(0)}, ${bounds.maxX.toFixed(0)}], Y[${bounds.minY.toFixed(0)}, ${bounds.maxY.toFixed(0)}]`);
    
    return bounds;
}

/**
 * Modified getMapScale to use auto-calculated bounds
 */
function getMapScaleAuto() {
    const bounds = window.mapBounds || calculateMapBounds(window.markers || []);
    const markerSize = parseFloat(document.getElementById("markerSize")?.value) || 5.0;
    
    const canvas = document.getElementById("mapCanvas");
    if (!canvas) {
        return { 
            width: 600, 
            height: 600, 
            scaleX: 1, 
            scaleY: 1, 
            markerSize,
            bounds 
        };
    }
    
    const scaleX = canvas.width / bounds.width;
    const scaleY = canvas.height / bounds.height;
    
    return { 
        width: bounds.width, 
        height: bounds.height, 
        scaleX, 
        scaleY, 
        markerSize,
        bounds 
    };
}

/**
 * Transforms marker coordinates from world space to canvas space
 */
function worldToCanvas(x, y, bounds, scaleX, scaleY, canvasHeight) {
    // Translate coordinates relative to bounds
    const relX = x - bounds.minX;
    const relY = y - bounds.minY;
    
    // Scale to canvas
    const canvasX = relX * scaleX;
    const canvasY = canvasHeight - (relY * scaleY);
    
    return { x: canvasX, y: canvasY };
}

/**
 * Transforms canvas coordinates to world space
 */
function canvasToWorld(canvasX, canvasY, bounds, scaleX, scaleY, canvasHeight) {
    // Reverse the transformations
    const relY = (canvasHeight - canvasY) / scaleY;
    const relX = canvasX / scaleX;
    
    const worldX = relX + bounds.minX;
    const worldY = relY + bounds.minY;
    
    return { x: worldX, y: worldY };
}

/**
 * Draws grid lines and coordinate labels on canvas borders
 * Grid spacing is 100cm
 */
function drawGridAndLabels(canvas, context, bounds, scaleX, scaleY) {
    if (!context || !canvas) return;
    
    const GRID_SIZE = bounds.gridSize || 20;
    
    // Grid line style
    context.strokeStyle = '#e2e8f0';
    context.lineWidth = 1;
    context.setLineDash([5, 5]);
    
    // Draw vertical grid lines (X-axis)
    for (let x = Math.ceil(bounds.minX / GRID_SIZE) * GRID_SIZE; x <= bounds.maxX; x += GRID_SIZE) {
        const canvasX = (x - bounds.minX) * scaleX;
        context.beginPath();
        context.moveTo(canvasX, 0);
        context.lineTo(canvasX, canvas.height);
        context.stroke();
    }
    
    // Draw horizontal grid lines (Y-axis)
    for (let y = Math.ceil(bounds.minY / GRID_SIZE) * GRID_SIZE; y <= bounds.maxY; y += GRID_SIZE) {
        const canvasY = canvas.height - ((y - bounds.minY) * scaleY);
        context.beginPath();
        context.moveTo(0, canvasY);
        context.lineTo(canvas.width, canvasY);
        context.stroke();
    }
    
    context.setLineDash([]); // Reset line dash
    
    // Label style
    context.font = "11px Arial";
    context.fillStyle = "#4a5568";
    context.textAlign = "center";
    
    // X-axis labels (bottom border)
    context.textAlign = "center";
    context.textBaseline = "top";
    for (let x = Math.ceil(bounds.minX / GRID_SIZE) * GRID_SIZE; x <= bounds.maxX; x += GRID_SIZE) {
        const canvasX = (x - bounds.minX) * scaleX;
        const label = x.toFixed(0);
        
        // Draw background for better readability
        const textWidth = context.measureText(label).width;
        context.fillStyle = "rgba(255, 255, 255, 0.9)";
        context.fillRect(canvasX - textWidth/2 - 2, canvas.height - 18, textWidth + 4, 16);
        
        context.fillStyle = "#4a5568";
        context.fillText(label, canvasX, canvas.height - 15);
    }
    
    // Y-axis labels (left border)
    context.textAlign = "left";
    context.textBaseline = "middle";
    for (let y = Math.ceil(bounds.minY / GRID_SIZE) * GRID_SIZE; y <= bounds.maxY; y += GRID_SIZE) {
        const canvasY = canvas.height - ((y - bounds.minY) * scaleY);
        const label = y.toFixed(0);
        
        // Draw background for better readability
        const textWidth = context.measureText(label).width;
        context.fillStyle = "rgba(255, 255, 255, 0.9)";
        context.fillRect(5, canvasY - 8, textWidth + 4, 16);
        
        context.fillStyle = "#4a5568";
        context.fillText(label, 7, canvasY);
    }
    
    // Draw coordinate system info (top-left corner)
    context.textAlign = "left";
    context.textBaseline = "top";
    context.font = "12px Arial bold";
    
    const infoText = `Grid: ${GRID_SIZE}cm | Range: [${bounds.minX},${bounds.maxX}] × [${bounds.minY},${bounds.maxY}]`;
    const infoWidth = context.measureText(infoText).width;
    
    context.fillStyle = "rgba(255, 255, 255, 0.95)";
    context.fillRect(5, 5, infoWidth + 10, 20);
    
    context.fillStyle = "#2d3748";
    context.fillText(infoText, 10, 10);
}

function drawAxisLabelsWithBounds(canvas, context, bounds, scaleX, scaleY) {
    if (!context || !canvas) return;
    context.font = "10px Arial";
    context.fillStyle = "#777";

    // Y-Axis Labels - use actual bounds
    const yStep = Math.max(Math.round((bounds.maxY - bounds.minY) / 6 / 10) * 10, 10);
    for (let y = Math.ceil(bounds.minY / yStep) * yStep; y <= bounds.maxY; y += yStep) {
        const canvasY = canvas.height - ((y - bounds.minY) * scaleY);
        context.fillText(y.toFixed(0) + "cm", 5, canvasY - 5);
    }
    
    // X-Axis Labels - use actual bounds
    const xStep = Math.max(Math.round((bounds.maxX - bounds.minX) / 6 / 10) * 10, 10);
    for (let x = Math.ceil(bounds.minX / xStep) * xStep; x <= bounds.maxX; x += xStep) {
        const canvasX = (x - bounds.minX) * scaleX;
        context.fillText(x.toFixed(0) + "cm", canvasX + 5, canvas.height - 5);
    }
    
    // Dimension labels
    context.fillStyle = "#333";
    context.font = "12px Arial bold";
    context.fillText(`Range: ${bounds.width.toFixed(0)} cm`, canvas.width - 100, canvas.height - 10);
    context.fillText(`Height: ${bounds.height.toFixed(0)} cm`, 5, 15);
}

// Export functions for use in main.js
if (typeof window !== 'undefined') {
    window.calculateMapBounds = calculateMapBounds;
    window.updateMapDimensions = updateMapDimensions;
    window.getMapScaleAuto = getMapScaleAuto;
    window.worldToCanvas = worldToCanvas;
    window.canvasToWorld = canvasToWorld;
    window.drawGridAndLabels = drawGridAndLabels;
    window.mapBounds = null;
}