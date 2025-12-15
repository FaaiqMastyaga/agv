// ===================================================================
// === CANVAS INTERACTION CONTROLS (Pan, Zoom, Reset) ===
// ===================================================================

class CanvasController {
    constructor(canvasId, wrapperId, zoomDisplayId) {
        this.canvas = document.getElementById(canvasId);
        this.wrapper = document.getElementById(wrapperId);
        this.zoomDisplay = document.getElementById(zoomDisplayId);
        
        if (!this.canvas || !this.wrapper) {
            console.warn(`Canvas controller: ${canvasId} or ${wrapperId} not found`);
            return;
        }

        this.scale = 1;
        this.minScale = 0.1;
        this.maxScale = 5;
        this.translateX = 0;
        this.translateY = 0;
        this.isDragging = false;
        this.dragStartX = 0;
        this.dragStartY = 0;
        this.isRightClick = false;

        this.setupEventListeners();
        this.updateTransform();
    }

    setupEventListeners() {
        // Mouse wheel zoom
        this.wrapper.addEventListener('wheel', (e) => {
            e.preventDefault();
            const delta = e.deltaY > 0 ? 0.9 : 1.1;
            this.zoom(delta, e.offsetX, e.offsetY);
        });

        // Pan with mouse drag
        this.wrapper.addEventListener('mousedown', (e) => {
            // For setup canvas, allow panning with left-click but only when not clicking directly on canvas
            // We'll handle marker placement separately
            if (e.button === 0 || e.button === 2) {
                this.isDragging = true;
                this.isRightClick = e.button === 2;
                this.wrapper.classList.add('dragging');
                this.dragStartX = e.clientX - this.translateX;
                this.dragStartY = e.clientY - this.translateY;
                e.preventDefault();
            }
        });

        this.wrapper.addEventListener('mousemove', (e) => {
            if (this.isDragging) {
                this.translateX = e.clientX - this.dragStartX;
                this.translateY = e.clientY - this.dragStartY;
                this.updateTransform();
                e.preventDefault();
            }
        });

        this.wrapper.addEventListener('mouseup', (e) => {
            if (this.isDragging && (e.button === 0 || e.button === 2)) {
                this.isDragging = false;
                this.isRightClick = false;
                this.wrapper.classList.remove('dragging');
            }
        });

        this.wrapper.addEventListener('mouseleave', () => {
            if (this.isDragging) {
                this.isDragging = false;
                this.isRightClick = false;
                this.wrapper.classList.remove('dragging');
            }
        });

        // Prevent context menu on right-click
        this.wrapper.addEventListener('contextmenu', (e) => {
            e.preventDefault();
        });

        // Touch support for mobile
        this.wrapper.addEventListener('touchstart', (e) => {
            if (e.touches.length === 1) {
                this.isDragging = true;
                const touch = e.touches[0];
                const rect = this.wrapper.getBoundingClientRect();
                this.dragStartX = touch.clientX - rect.left - this.translateX;
                this.dragStartY = touch.clientY - rect.top - this.translateY;
            }
        });

        this.wrapper.addEventListener('touchmove', (e) => {
            if (this.isDragging && e.touches.length === 1) {
                e.preventDefault();
                const touch = e.touches[0];
                const rect = this.wrapper.getBoundingClientRect();
                this.translateX = touch.clientX - rect.left - this.dragStartX;
                this.translateY = touch.clientY - rect.top - this.dragStartY;
                this.updateTransform();
            }
        });

        this.wrapper.addEventListener('touchend', () => {
            this.isDragging = false;
        });
    }

    zoom(delta, centerX, centerY) {
        const oldScale = this.scale;
        this.scale = Math.max(this.minScale, Math.min(this.maxScale, this.scale * delta));

        // Zoom towards cursor position
        if (centerX !== undefined && centerY !== undefined) {
            this.translateX = centerX - (centerX - this.translateX) * (this.scale / oldScale);
            this.translateY = centerY - (centerY - this.translateY) * (this.scale / oldScale);
        }

        this.updateTransform();
    }

    zoomIn() {
        const centerX = this.wrapper.offsetWidth / 2;
        const centerY = this.wrapper.offsetHeight / 2;
        this.zoom(1.2, centerX, centerY);
    }

    zoomOut() {
        const centerX = this.wrapper.offsetWidth / 2;
        const centerY = this.wrapper.offsetHeight / 2;
        this.zoom(0.8, centerX, centerY);
    }

    resetView() {
        this.scale = 1;
        this.translateX = 0;
        this.translateY = 0;
        this.updateTransform();
    }

    fitToScreen() {
        const wrapperWidth = this.wrapper.offsetWidth;
        const wrapperHeight = this.wrapper.offsetHeight;
        const canvasWidth = this.canvas.width;
        const canvasHeight = this.canvas.height;

        const scaleX = wrapperWidth / canvasWidth;
        const scaleY = wrapperHeight / canvasHeight;
        this.scale = Math.min(scaleX, scaleY) * 0.95; // 95% to add some padding

        // Center the canvas
        this.translateX = (wrapperWidth - canvasWidth * this.scale) / 2;
        this.translateY = (wrapperHeight - canvasHeight * this.scale) / 2;

        this.updateTransform();
    }

    updateTransform() {
        this.canvas.style.transform = `translate(${this.translateX}px, ${this.translateY}px) scale(${this.scale})`;
        this.canvas.style.transformOrigin = '0 0';
        
        if (this.zoomDisplay) {
            this.zoomDisplay.textContent = `${Math.round(this.scale * 100)}%`;
        }
    }

    // Get the actual coordinates on the canvas considering zoom and pan
    getCanvasCoordinates(clientX, clientY) {
        const rect = this.wrapper.getBoundingClientRect();
        const x = (clientX - rect.left - this.translateX) / this.scale;
        const y = (clientY - rect.top - this.translateY) / this.scale;
        return { x, y };
    }
}

// Initialize controllers when DOM is ready
let mapCanvasController = null;
let opCanvasController = null;

function initializeCanvasControllers() {
    // Setup Map Canvas Controller
    mapCanvasController = new CanvasController('mapCanvas', 'mapCanvasWrapper', 'zoomLevelMap');
    
    // Setup Operation Canvas Controller
    opCanvasController = new CanvasController('opMapCanvas', 'opMapCanvasWrapper', 'zoomLevelOp');

    // Connect zoom buttons for Setup Map
    const zoomInMapBtn = document.getElementById('zoomInMap');
    const zoomOutMapBtn = document.getElementById('zoomOutMap');
    const resetViewMapBtn = document.getElementById('resetViewMap');
    const fitToScreenMapBtn = document.getElementById('fitToScreenMap');

    if (zoomInMapBtn) zoomInMapBtn.addEventListener('click', () => mapCanvasController.zoomIn());
    if (zoomOutMapBtn) zoomOutMapBtn.addEventListener('click', () => mapCanvasController.zoomOut());
    if (resetViewMapBtn) resetViewMapBtn.addEventListener('click', () => mapCanvasController.resetView());
    if (fitToScreenMapBtn) fitToScreenMapBtn.addEventListener('click', () => mapCanvasController.fitToScreen());

    // Connect zoom buttons for Operation Map
    const zoomInOpBtn = document.getElementById('zoomInOp');
    const zoomOutOpBtn = document.getElementById('zoomOutOp');
    const resetViewOpBtn = document.getElementById('resetViewOp');
    const fitToScreenOpBtn = document.getElementById('fitToScreenOp');

    if (zoomInOpBtn) zoomInOpBtn.addEventListener('click', () => opCanvasController.zoomIn());
    if (zoomOutOpBtn) zoomOutOpBtn.addEventListener('click', () => opCanvasController.zoomOut());
    if (resetViewOpBtn) resetViewOpBtn.addEventListener('click', () => opCanvasController.resetView());
    if (fitToScreenOpBtn) fitToScreenOpBtn.addEventListener('click', () => opCanvasController.fitToScreen());
}

// Export for use in main.js
if (typeof window !== 'undefined') {
    window.initializeCanvasControllers = initializeCanvasControllers;
    window.mapCanvasController = null;
    window.opCanvasController = null;
    
    // Make controllers globally accessible after initialization
    window.addEventListener('DOMContentLoaded', () => {
        initializeCanvasControllers();
        window.mapCanvasController = mapCanvasController;
        window.opCanvasController = opCanvasController;
    });
}