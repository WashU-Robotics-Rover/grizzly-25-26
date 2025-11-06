/**
 * State History Manager
 * Tracks and displays state change history
 */

const HistoryManager = {
    history: [],

    /**
     * Add state change to history
     * @param {Object} stateMsg - State message from ROS
     */
    add(stateMsg) {
        const historyList = Utils.getElement('stateHistory');
        if (!historyList) return;

        const stateName = StateDisplay.getStateName(stateMsg.state);
        
        // Remove placeholder if present
        const placeholder = historyList.querySelector('.history-placeholder');
        if (placeholder) {
            placeholder.remove();
        }
        
        // Create history entry
        const entry = {
            state: stateName,
            description: stateMsg.description,
            timestamp: new Date()
        };
        
        this.history.unshift(entry);
        
        // Limit history size
        if (this.history.length > Config.history.maxItems) {
            this.history.pop();
        }
        
        // Update display
        this.render();
        
        Utils.log('info', `Added to history: ${stateName}`);
    },

    /**
     * Render state history list
     */
    render() {
        const historyList = Utils.getElement('stateHistory');
        if (!historyList) return;

        historyList.innerHTML = '';
        
        if (this.history.length === 0) {
            historyList.innerHTML = '<li class="history-placeholder text-xs text-zinc-500">No state changes yet</li>';
            return;
        }
        
        this.history.forEach(entry => {
            const li = document.createElement('li');
            li.className = 'pb-2 mb-2 border-b border-zinc-800 last:border-0';
            
            const timeStr = entry.timestamp.toLocaleTimeString();
            const dateStr = entry.timestamp.toLocaleDateString();
            
            li.innerHTML = `
                <div class="flex justify-between items-start mb-1">
                    <span class="text-xs font-semibold text-zinc-300">${entry.state}</span>
                    <span class="text-[10px] text-zinc-500">${timeStr}</span>
                </div>
                <div class="text-[10px] text-zinc-400">${entry.description || 'No description'}</div>
                <div class="text-[9px] text-zinc-600 mt-1">${dateStr}</div>
            `;
            
            historyList.appendChild(li);
        });
    },

    /**
     * Clear history
     */
    clear() {
        this.history = [];
        this.render();
        Utils.log('info', 'History cleared');
    },

    /**
     * Get history
     * @returns {Array} History entries
     */
    getHistory() {
        return [...this.history];
    },

    /**
     * Export history as JSON
     * @returns {string} JSON string of history
     */
    exportJSON() {
        return JSON.stringify(this.history, null, 2);
    }
};
