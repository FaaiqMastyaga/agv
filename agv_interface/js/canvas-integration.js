// ===================================================================
// === CANVAS INTEGRATION HELPERS ===
// === This file provides helper functions to integrate canvas ===
// === controls with your existing main.js functionality ===
// ===================================================================

/**
 * Disable canvas click for marker placement
 * This prevents the original click handler from placing markers
 */
function disableCanvasClickMarkerPlacement() {
    const mapCanvas = document.getElementById('mapCanvas');
    if (!mapCanvas) return;
    
    // Mark as setup to prevent any click-to-place functionality
    mapCanvas.setAttribute('data-click-disabled', 'true');
    
    // Add a click handler that does nothing (blocks the original)
    mapCanvas.addEventListener("click", function(e) {
        e.stopPropagation();
        e.preventDefault();
    }, true); // Use capture phase to block before other handlers
}

/**
 * Enhanced hover detection that accounts for canvas zoom and pan
 * This properly calculates marker positions with transform applied
 */
function setupCanvasHoverWithTransform(canvas, renderFunction, mapType) {
    if (!canvas) return;
    
    const mapTooltip = document.getElementById('mapTooltip');
    if (!mapTooltip) return;
    
    // Check if already setup
    if (canvas.hasAttribute('data-hover-setup')) return;
    canvas.setAttribute('data-hover-setup', 'true');
    
    // Get the appropriate controller
    const getController = () => (mapType === 'setup') ? window.mapCanvasController : window.opCanvasController;
    
    const setHoveredMarker = (marker, viewportX, viewportY) => { 
        let currentHoveredMarker = (mapType === 'setup') ? window.hoveredMarkerSetup : window.hoveredMarkerOperation;
        
        if (marker !== currentHoveredMarker) {
            if (mapType === 'setup') {
                window.hoveredMarkerSetup = marker;
            } else {
                window.hoveredMarkerOperation = marker;
            }
            if (typeof renderFunction === 'function') renderFunction();
        }

        if (marker) {
            const tooltipHTML = `
                <strong>ID: ${marker.id}</strong><br>
                X: ${marker.x.toFixed(2)} cm<br>
                Y: ${marker.y.toFixed(2)} cm<br>
                Yaw: ${marker.yaw.toFixed(2)} °
            `;
            mapTooltip.innerHTML = tooltipHTML;
            mapTooltip.style.display = 'block';

            const offsetX = 15;
            const offsetY = 15;
            
            let tooltipX = viewportX + offsetX; 
            let tooltipY = viewportY + offsetY;
            
            if (tooltipX + mapTooltip.offsetWidth > window.innerWidth - 10) {
                tooltipX = viewportX - mapTooltip.offsetWidth - offsetX;
            }
            if (tooltipY + mapTooltip.offsetHeight > window.innerHeight - 10) {
                tooltipY = viewportY - mapTooltip.offsetHeight - offsetY; 
            }

            mapTooltip.style.left = `${tooltipX}px`;
            mapTooltip.style.top = `${tooltipY}px`;
            
            canvas.style.cursor = 'pointer';
        } else {
            mapTooltip.style.display = 'none';
            canvas.style.cursor = 'grab';
        }
    };

    canvas.addEventListener('mousemove', (e) => {
        const controller = getController();
        if (!controller) return;
        
        if (typeof getMapScale !== 'function' || !window.markers) return;
        
        // Get the canvas wrapper to calculate proper coordinates
        const wrapper = canvas.parentElement;
        const wrapperRect = wrapper.getBoundingClientRect();
        
        // Calculate mouse position relative to wrapper
        const mouseX = e.clientX - wrapperRect.left;
        const mouseY = e.clientY - wrapperRect.top;
        
        // Apply inverse transform to get actual canvas coordinates
        const canvasX = (mouseX - controller.translateX) / controller.scale;
        const canvasY = (mouseY - controller.translateY) / controller.scale;
        
        const { scaleX, scaleY, height, markerSize } = getMapScale();
        
        // Convert from canvas pixels to map coordinates (cm)
        const x_cm = canvasX / scaleX;
        const y_cm = height - (canvasY / scaleY);
        
        // Detection radius in cm
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
                
                // Use the actual mouse position in viewport for tooltip
                setHoveredMarker(detectedMarker, e.clientX, e.clientY);
                return;
            }
        }
        
        setHoveredMarker(null, e.clientX, e.clientY);
    });
    
    canvas.addEventListener('mouseout', () => {
        setHoveredMarker(null, 0, 0);
    });
}

/**
 * Helper function to calculate distance between two points
 */
function getDistance(p1, p2) {
    return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
}

/**
 * Initialize the enhanced canvas interactions
 * Call this after your canvas controllers are initialized
 */
function initializeEnhancedCanvasInteractions() {
    // Wait a bit for controllers to be ready
    setTimeout(() => {
        // Disable click-to-place markers
        disableCanvasClickMarkerPlacement();
        
        // Setup hover for both canvases with transform support
        const mapCanvas = document.getElementById('mapCanvas');
        const opMapCanvas = document.getElementById('opMapCanvas');
        
        if (mapCanvas && typeof window.renderMap === 'function') {
            setupCanvasHoverWithTransform(mapCanvas, window.renderMap, 'setup');
        }
        
        if (opMapCanvas && typeof window.renderOperationMap === 'function') {
            setupCanvasHoverWithTransform(opMapCanvas, window.renderOperationMap, 'operation');
        }
        
        // Trigger initial render to show markers
        if (typeof window.renderMap === 'function') {
            window.renderMap();
        }
        if (typeof window.renderOperationMap === 'function') {
            window.renderOperationMap();
        }
    }, 100);
}

// Auto-initialize when DOM is ready
if (typeof window !== 'undefined') {
    window.addEventListener('DOMContentLoaded', () => {
        // Wait for canvas controllers to initialize first
        setTimeout(() => {
            initializeEnhancedCanvasInteractions();
        }, 200);
    });
    
    // Export functions for manual initialization if needed
    window.disableCanvasClickMarkerPlacement = disableCanvasClickMarkerPlacement;
    window.setupCanvasHoverWithTransform = setupCanvasHoverWithTransform;
    window.initializeEnhancedCanvasInteractions = initializeEnhancedCanvasInteractions;
}