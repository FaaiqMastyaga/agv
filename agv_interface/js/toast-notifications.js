// ===================================================================
// === TOAST NOTIFICATION SYSTEM ===
// ===================================================================

/**
 * Creates a toast notification that appears in the top-right corner
 * @param {string} message - The main message to display
 * @param {string} type - Toast type: 'success', 'error', 'warning', 'info'
 * @param {string} title - Optional title (default based on type)
 * @param {number} duration - How long to show in ms (default 5000)
 */
function showToast(message, type = 'info', title = null, duration = 5000) {
    const container = document.getElementById('toastContainer');
    if (!container) {
        console.error('Toast container not found');
        return;
    }

    // Default titles based on type
    const defaultTitles = {
        success: 'Success',
        error: 'Error',
        warning: 'Warning',
        info: 'Info'
    };

    // Icons for each type
    const icons = {
        success: '✓',
        error: '✕',
        warning: '⚠',
        info: 'ℹ'
    };

    const toastTitle = title || defaultTitles[type] || 'Notification';
    const icon = icons[type] || 'ℹ';

    // Create toast element
    const toast = document.createElement('div');
    toast.className = `toast ${type}`;
    toast.innerHTML = `
        <div class="toast-icon">${icon}</div>
        <div class="toast-content">
            <div class="toast-title">${toastTitle}</div>
            <div class="toast-message">${message}</div>
        </div>
        <button class="toast-close" aria-label="Close">×</button>
    `;

    // Add close button functionality
    const closeBtn = toast.querySelector('.toast-close');
    closeBtn.addEventListener('click', () => {
        removeToast(toast);
    });

    // Add to container
    container.appendChild(toast);

    // Auto-remove after duration
    setTimeout(() => {
        removeToast(toast);
    }, duration);
}

/**
 * Removes a toast with animation
 */
function removeToast(toast) {
    toast.style.animation = 'slideInRight 0.3s ease-out reverse';
    setTimeout(() => {
        if (toast.parentNode) {
            toast.parentNode.removeChild(toast);
        }
    }, 300);
}

/**
 * Convenience functions for different toast types
 */
function showSuccessToast(message, title = 'Success', duration = 5000) {
    showToast(message, 'success', title, duration);
}

function showErrorToast(message, title = 'Error', duration = 5000) {
    showToast(message, 'error', title, duration);
}

function showWarningToast(message, title = 'Warning', duration = 5000) {
    showToast(message, 'warning', title, duration);
}

function showInfoToast(message, title = 'Info', duration = 5000) {
    showToast(message, 'info', title, duration);
}

// ===================================================================
// === COMPATIBILITY WRAPPER FOR OLD STATUS ELEMENTS ===
// ===================================================================

/**
 * Creates virtual status element objects that show toasts instead
 * This allows old code to work without modification
 */
function createVirtualStatusElement(elementId) {
    return {
        _text: '',
        get innerText() {
            return this._text;
        },
        set innerText(value) {
            this._text = value;
            
            // Parse the message to determine type
            const lowerValue = value.toLowerCase();
            
            if (lowerValue.includes('success') || lowerValue.includes('saved') || 
                lowerValue.includes('deleted') || lowerValue.includes('found')) {
                showSuccessToast(value);
            } else if (lowerValue.includes('error') || lowerValue.includes('failed') || 
                       lowerValue.includes('cannot') || lowerValue.includes('invalid')) {
                showErrorToast(value);
            } else if (lowerValue.includes('warning') || lowerValue.includes('no ') || 
                       lowerValue.includes('empty')) {
                showWarningToast(value);
            } else if (lowerValue.includes('loading') || lowerValue.includes('fetching') || 
                       lowerValue.includes('requesting') || lowerValue.includes('saving') || 
                       lowerValue.includes('deleting')) {
                showInfoToast(value);
            } else {
                showInfoToast(value);
            }
        },
        get textContent() {
            return this._text;
        },
        set textContent(value) {
            this.innerText = value;
        }
    };
}

// Export functions
if (typeof window !== 'undefined') {
    window.showToast = showToast;
    window.showSuccessToast = showSuccessToast;
    window.showErrorToast = showErrorToast;
    window.showWarningToast = showWarningToast;
    window.showInfoToast = showInfoToast;
    
    // Create virtual status elements for backward compatibility
    // These intercept attempts to set innerText and show toasts instead
    window.createVirtualStatusElement = createVirtualStatusElement;
    
    // Auto-create virtual elements if the real ones don't exist
    document.addEventListener('DOMContentLoaded', () => {
        if (!document.getElementById('load_status')) {
            window.load_status_virtual = createVirtualStatusElement('load_status');
        }
        if (!document.getElementById('save_status')) {
            window.save_status_virtual = createVirtualStatusElement('save_status');
        }
    });
}

/**
 * USAGE EXAMPLES for main.js:
 * 
 * METHOD 1 - Direct toast functions (recommended):
 * showSuccessToast("Map 'room_map' loaded successfully!");
 * showErrorToast("Failed to connect to ROS bridge", "Connection Error");
 * showWarningToast("Map has no markers defined", "Warning");
 * showInfoToast("Fetching map list from database...", "Loading");
 * 
 * METHOD 2 - Compatibility mode (works with old code):
 * const loadStatusElement = document.getElementById('load_status') || window.load_status_virtual;
 * loadStatusElement.innerText = 'Map loaded successfully'; // Will show as toast
 */