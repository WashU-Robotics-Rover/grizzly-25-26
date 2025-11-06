# Grizzly Web Interface - Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          GRIZZLY WEB INTERFACE                               │
│                         (index.html + Browser)                               │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                            main.js (GrizzlyApp)                              │
│                      • Application Orchestrator                              │
│                      • Event Handler Setup                                   │
│                      • Module Initialization                                 │
└─────────────────────────────────────────────────────────────────────────────┘
           │                    │                    │                    │
           ▼                    ▼                    ▼                    ▼
    ┌───────────┐        ┌───────────┐       ┌───────────┐       ┌───────────┐
    │    ROS    │        │    UI     │       │ Monitoring│       │   Utils   │
    │ Interface │        │  Modules  │       │           │       │           │
    └───────────┘        └───────────┘       └───────────┘       └───────────┘
           │                    │                    │                    │
           │                    │                    │                    │
    ┌──────┴──────┐      ┌──────┴──────┐    ┌───────▼──────┐    ┌────────▼────┐
    │             │      │             │    │              │    │             │
    ▼             ▼      ▼             ▼    │   Health     │    │   helpers   │
┌────────┐  ┌─────────┐ ┌──────────┐ ┌───────────────┐   │    │     .js     │
│connect │  │subscrib │ │ state    │ │  health      │   │    │             │
│ion.js  │  │ers.js   │ │ -display │ │  -display    │   │    │ • DOM utils │
│        │  │         │ │   .js    │ │    .js       │   │    │ • Formatters│
│• Init  │  │• State  │ │          │ │              │   │    │ • Logging   │
│• Events│  │  topic  │ │• Update  │ │• Update      │   │    │ • Debounce  │
│• Status│  │• Health │ │  badge   │ │  health      │   │    └─────────────┘
│        │  │  topic  │ │• Colors  │ │  message     │   │
└────┬───┘  │• Cleanup│ │• Metadata│ │• Indicator   │   │
     │      └────┬────┘ └────┬─────┘ └──────┬───────┘   │
     │           │           │              │           │
     │      ┌────┴────┐ ┌────┴─────┐  ┌─────▼──────┐   │
     │      │         │ │          │  │            │   │
     │   ┌──▼───────┐ │ │ history  │  │notifications│   │
     │   │ services │ │ │   .js    │  │    .js     │   │
     │   │   .js    │ │ │          │  │            │   │
     │   │          │ │ │• Add     │  │• Success   │   │
     │   │• State   │ │ │  entry   │  │• Error     │   │
     │   │  change  │ │ │• Render  │  │• Warning   │   │
     │   │  service │ │ │• Export  │  │• Service   │   │
     │   │• Promise │ │ │          │  │  response  │   │
     │   │  based   │ │ └──────────┘  └────────────┘   │
     │   └──────────┘ │                                 │
     │                │                                 │
     │                │                          monitor│
     │                │                            .js  │
     │                │                                 │
     │                │                          • Start│
     │                │                          • Stop │
     │                │                          • Reset│
     │                │                          • Timeout│
     │                │                                 │
     └────────────────┴─────────────────────────────────┘
                      │
                      ▼
            ┌──────────────────┐
            │   core/config.js  │
            │                   │
            │  • States         │
            │  • Colors         │
            │  • Topics         │
            │  • Services       │
            │  • Timeouts       │
            │  • UI Config      │
            └──────────────────┘

═══════════════════════════════════════════════════════════════════════════════

                          DATA FLOW DIAGRAM

User Opens Page
      │
      ▼
┌─────────────┐
│  DOM Ready  │
└──────┬──────┘
       │
       ▼
┌─────────────────────────────────┐
│  GrizzlyApp.init()              │
│  • Setup connection handlers    │
│  • Setup subscriber callbacks   │
│  • Expose global functions      │
└────────────┬────────────────────┘
             │
             ▼
┌─────────────────────────────────┐
│  ROSConnection.connect()        │
│  • Create ROSLIB.Ros            │
│  • Setup event handlers         │
└────────────┬────────────────────┘
             │
             ├─────────────────────────────────┐
             │                                 │
             ▼                                 ▼
    ┌─────────────────┐              ┌─────────────────┐
    │  on('connection')│              │  on('error')    │
    │  ✓ Connected!   │              │  ✗ Error        │
    └────────┬────────┘              └────────┬────────┘
             │                                 │
             ▼                                 ▼
    ┌─────────────────┐              ┌─────────────────┐
    │ ROSSubscribers  │              │  Show Error     │
    │    .setup()     │              │  Notification   │
    └────────┬────────┘              └─────────────────┘
             │
             ├──────────────────┐
             │                  │
             ▼                  ▼
    ┌─────────────┐    ┌─────────────┐
    │ Subscribe   │    │ Subscribe   │
    │ /system/    │    │ /system/    │
    │   state     │    │   health    │
    └──────┬──────┘    └──────┬──────┘
           │                  │
           ▼                  ▼
    [State Messages]   [Health Messages]
           │                  │
           ▼                  ▼
    ┌─────────────┐    ┌─────────────┐
    │StateDisplay │    │HealthDisplay│
    │  .update()  │    │  .update()  │
    └──────┬──────┘    └──────┬──────┘
           │                  │
           ▼                  ▼
    ┌─────────────┐    ┌─────────────┐
    │HistoryMgr   │    │HealthMonitor│
    │   .add()    │    │ .recordUpdate()
    └─────────────┘    └─────────────┘

═══════════════════════════════════════════════════════════════════════════════

                        MODULE DEPENDENCIES

    main.js
      ├── Depends on: ALL modules
      └── Creates: Global functions, Debug object

    ros/connection.js
      ├── Depends on: Utils, Notifications, Config
      └── Provides: ROS instance, connection status

    ros/subscribers.js
      ├── Depends on: ROSConnection, Utils, Config, Notifications
      └── Provides: Topic subscriptions, callbacks

    ros/services.js
      ├── Depends on: ROSConnection, Utils, Config, Notifications
      └── Provides: Service calls (Promise-based)

    ui/state-display.js
      ├── Depends on: Utils, Config, Notifications
      └── Provides: State visualization

    ui/health-display.js
      ├── Depends on: Utils, Config, Notifications
      └── Provides: Health visualization

    ui/history.js
      ├── Depends on: Utils, Config, StateDisplay
      └── Provides: History tracking and display

    ui/notifications.js
      ├── Depends on: Utils, Config
      └── Provides: User notifications

    monitoring/health-monitor.js
      ├── Depends on: Utils, Config, StateDisplay, HealthDisplay, Notifications
      └── Provides: Health timeout detection

    utils/helpers.js
      ├── Depends on: None (pure utilities)
      └── Provides: DOM, logging, formatting utilities

    core/config.js
      ├── Depends on: None (configuration only)
      └── Provides: All configuration constants

═══════════════════════════════════════════════════════════════════════════════

                        LOAD ORDER (index.html)

    1. core/config.js         ← Must load first (no dependencies)
    2. utils/helpers.js        ← Used by all other modules
    3. ui/notifications.js     ← Used by many modules
    4. ui/state-display.js     ← State visualization
    5. ui/health-display.js    ← Health visualization
    6. ui/history.js           ← History management
    7. monitoring/health-monitor.js ← Uses UI modules
    8. ros/connection.js       ← ROS connection
    9. ros/subscribers.js      ← Uses connection
    10. ros/services.js        ← Uses connection
    11. main.js                ← Orchestrates everything (loads last)

═══════════════════════════════════════════════════════════════════════════════
