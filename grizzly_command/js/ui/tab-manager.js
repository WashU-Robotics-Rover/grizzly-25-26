/**
 * Tab Manager
 * Handles tab switching and navigation
 */

const TabManager = {
    currentTab: 'system-management',
    tabs: ['system-management', 'output'],

    /**
     * Switch to a different tab
     * @param {string} tabName - Name of the tab to switch to
     */
    switchTab(tabName) {
        if (!this.tabs.includes(tabName)) {
            Utils.log('warn', `Invalid tab name: ${tabName}`);
            return;
        }

        // Hide all tab content
        this.tabs.forEach(tab => {
            const content = Utils.getElement(`tab-${tab}`);
            if (content) {
                content.classList.add('hidden');
            }
        });

        // Show selected tab content
        const selectedContent = Utils.getElement(`tab-${tabName}`);
        if (selectedContent) {
            selectedContent.classList.remove('hidden');
        }

        // Update tab button styles
        this._updateTabButtons(tabName);

        this.currentTab = tabName;
        Utils.log('info', `Switched to tab: ${tabName}`);

        // Trigger tab change event
        this._onTabChange(tabName);
    },

    /**
     * Update tab button active states
     * @private
     */
    _updateTabButtons(activeTab) {
        // Remove active class from all buttons
        document.querySelectorAll('.tab-button').forEach(button => {
            button.classList.remove('active');
            button.classList.remove('border-blue-500', 'text-blue-400');
            button.classList.add('border-transparent', 'text-zinc-500');
        });

        // Add active class to selected button
        const activeButton = Utils.getElement(`tab${this._capitalize(activeTab.replace('-', ''))}`);
        if (activeButton) {
            activeButton.classList.add('active');
            activeButton.classList.remove('border-transparent', 'text-zinc-500');
            activeButton.classList.add('border-blue-500', 'text-blue-400');
        }
    },

    /**
     * Called when tab changes
     * @private
     */
    _onTabChange(tabName) {
        // Special handling for terminal tab - start autoscroll
        if (tabName === 'output' && window.TerminalOutput) {
            TerminalOutput.scrollToBottom();
        }
    },

    /**
     * Get current active tab
     * @returns {string} Current tab name
     */
    getCurrentTab() {
        return this.currentTab;
    },

    /**
     * Capitalize helper
     * @private
     */
    _capitalize(str) {
        return str.charAt(0).toUpperCase() + str.slice(1);
    }
};

// Expose to global scope for HTML onclick handlers
window.switchTab = (tabName) => TabManager.switchTab(tabName);
