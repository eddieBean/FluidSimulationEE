# Fluid Simulation Logging & Performance Monitoring Feature

## Design Document

---

## 1. Overview

This document describes the implementation of a **real-time logging and performance monitoring system** for a WebGL-based Eulerian fluid simulation. The feature captures frame-level metrics including:
- Frame timing data (delta time, FPS, actual frame generation time)
- Dynamic memory usage (using `measureUserAgentSpecificMemory()`)
- CPU load estimation
- Cumulative elapsed time

Data is accumulated in memory during simulation and can be exported to CSV for analysis.

---

## 2. Architecture & Components

### 2.1 Global State Variables

```javascript
let logBuffer = []              // Array storing CSV rows (strings)
let frameStartTime = 0          // Timestamp marking start of current frame
let currentMemoryMB = 0         // Cache of latest memory measurement
let startTime = 0               // Page load/simulation start timestamp
```

**Purpose:** Store frame metrics and track timing across the simulation lifecycle.

---

### 2.2 Core Functions

#### 2.2.1 `getMemoryStats()` - Async Memory Measurement

**Signature:**
```javascript
async function getMemoryStats()
```

**Behavior:**
1. **Primary Method** - Attempts to use `performance.measureUserAgentSpecificMemory()`
   - Returns object: `{ usedMemoryMB: string, breakdown: array }`
   - Breakdown contains attribution data showing which resources consume memory
   - Most accurate; reflects actual page memory usage across all resources

2. **Fallback Method** - Uses `performance.memory` (Chrome/Edge only)
   - Provides JS heap usage only
   - Less accurate but widely supported
   
3. **Error Handling** - Catches exceptions (permission denied, unsupported browser)
   - Logs warning to console
   - Falls back gracefully

**Return Value:**
```javascript
{
  usedMemoryMB: "45.23",        // Total memory in MB (2 decimal places)
  breakdown: [
    { attribution: "URL or Unknown", bytes: "23.45" },
    // ... additional resources
  ]
}
```

**Why Async?** The `measureUserAgentSpecificMemory()` API is asynchronous because it performs measurement work and may require browser permission.

---

#### 2.2.2 `getCPULoad()` - CPU Utilization Estimation

**Signature:**
```javascript
function getCPULoad()
```

**Behavior:**
1. Retrieves `scene.dt` (delta time in seconds)
2. Converts to milliseconds: `scene.dt * 1000`
3. Compares against ideal 60 FPS frame time (16.67ms)
4. Calculates percentage: `(actual / ideal) * 100`
5. Caps at 100%: `Math.min(100, ...)`
6. Returns as 1-decimal string

**Formula:**
```
CPU Load (%) = min(100, (scene.dt * 1000 / 16.67) * 100)
```

**Interpretation:**
- 100% = frame takes exactly 16.67ms (perfect 60 FPS)
- >100% = frame takes longer than ideal (dropping frames)
- <100% = frame completes faster than next vsync (GPU underutilized)

---

#### 2.2.3 `logToCSV()` - Frame Data Collection

**Signature:**
```javascript
async function logToCSV()
```

**Behavior:**
1. Calculates frame generation time (milliseconds from start to current moment)
2. **Awaits** memory stats (blocking operation to ensure accurate measurement)
3. Updates `currentMemoryMB` cache (used for on-screen display)
4. Calculates CPU load
5. Computes total elapsed time since simulation start
6. Calculates current FPS from scene.dt
7. Constructs CSV row as comma-separated string
8. Pushes row to `logBuffer`

**Timing Sequence:**
```
frameStartTime = now()
  ↓
simulate() + draw()
  ↓
logToCSV() called (async)
  ├─ frameGenTime = now() - frameStartTime (actual execution time)
  ├─ await getMemoryStats() (dynamic measurement)
  ├─ Calculate other metrics
  └─ Append to logBuffer
```

**CSV Row Format:**
```
Frame,DeltaTime(s),FPS,CPULoad(%),FrameGenTime(ms),UsedMemory(MB),TotalElapsed(s)
0,0.0167,60.0,100.0,2.45,45.23,0.02
```

---

#### 2.2.4 `downloadCSV()` - Data Export

**Signature:**
```javascript
function downloadCSV()
```

**Behavior:**
1. Validates `logBuffer` has entries
2. Creates CSV header row with column names
3. Concatenates header + all log entries with newlines
4. Creates Blob from CSV string
5. Generates object URL
6. Creates temporary `<a>` element
7. Sets download attribute with timestamp filename
8. Triggers browser download
9. Cleans up DOM and object URL
10. Logs confirmation to console

**Filename Format:**
```
fluid_sim_log_[timestamp].csv
Example: fluid_sim_log_1716902400000.csv
```

**Error Handling:** Shows alert if no data to export.

---

### 2.3 Integration Points

#### 2.3.1 Animation Loop (`update()`)

**Current Implementation:**
```javascript
async function update() {
    frameStartTime = performance.now();  // Start timing
    simulate();                           // Physics simulation
    draw();                               // Rendering
    await logToCSV();                     // Collect metrics
    requestAnimationFrame(update);        // Schedule next frame
}
```

**Key Points:**
- Frame timing starts BEFORE simulation
- `logToCSV()` is awaited (ensures memory is measured before next frame)
- `requestAnimationFrame()` called AFTER logging
- Makes update() async to support await

---

#### 2.3.2 Keyboard Event Handler

**Key Bindings:**
| Key | Action |
|-----|--------|
| `p` | Toggle pause/resume simulation |
| `m` | Single step (simulate one frame while paused) |
| `e` | Export CSV to file |

**Code Location:** `document.addEventListener('keydown', ...)`

---

#### 2.3.3 On-Screen Display Update

**Location:** Inside `update()` function

**Code:**
```javascript
if (textbox) {
    textbox.innerHTML = `Frame: ${scene.frameNr} | FPS: ${fps.toFixed(1)} | Memory: ${currentMemoryMB}MB | FrameTime: ${frameGenTime.toFixed(2)}ms`;
}
```

**Purpose:** Real-time visualization of key metrics during simulation.

---

## 3. Memory Tracking Methodology

### 3.1 Why `measureUserAgentSpecificMemory()`?

**Traditional `performance.memory` Issues:**
- Only tracks JS heap size
- Not dynamic (reports snapshots)
- Doesn't include DOM, images, or other resources
- Not available in all browsers

**`measureUserAgentSpecificMemory()` Advantages:**
- Measures **all resources** on the page (DOM, canvas, textures, etc.)
- **Dynamic measurement** each frame
- **Attribution breakdown** showing which resources consume memory
- **More accurate** for real-world memory usage
- Designed for performance profiling

### 3.2 Measurement Frequency

- Called **once per frame** via `logToCSV()`
- Occurs after drawing, ensuring all resources are allocated
- Results cached in `currentMemoryMB` for on-screen display
- Added to log entry for export

### 3.3 Browser Compatibility

| Browser | Support |
|---------|---------|
| Chrome/Edge | ✅ Yes (Chromium-based) |
| Firefox | ❌ No |
| Safari | ❌ No |
| Opera | ✅ Yes |

**Graceful Fallback:** Uses `performance.memory` if primary API unavailable.

---

## 4. CSV Export Workflow

### 4.1 Data Structure

**In Memory (logBuffer):**
```javascript
[
  "Frame,DeltaTime(s),FPS,CPULoad(%),FrameGenTime(ms),UsedMemory(MB),TotalElapsed(s)",
  "0,0.0167,60.0,100.0,2.45,45.23,0.02",
  "1,0.0167,60.0,100.0,2.38,45.28,0.03",
  // ... more rows
]
```

### 4.2 Export Process

```
User presses 'E'
    ↓
downloadCSV() called
    ↓
Validate logBuffer not empty
    ↓
Create CSV string (header + data rows)
    ↓
Create Blob from string
    ↓
Generate Object URL
    ↓
Create temporary <a> element
    ↓
Set href and download attributes
    ↓
Trigger click() to start download
    ↓
Clean up: remove element, revoke URL
    ↓
Log success to console
```

### 4.3 File Format

**Format:** UTF-8 CSV (comma-separated values)

**Structure:**
- First row: Column headers
- Subsequent rows: One row per frame
- Values: Comma-delimited, properly formatted

**Example File:**
```csv
Frame,DeltaTime(s),FPS,CPULoad(%),FrameGenTime(ms),UsedMemory(MB),TotalElapsed(s)
0,0.0167,60.0,100.0,2.45,45.23,0.02
1,0.0167,60.0,100.0,2.38,45.28,0.03
2,0.0167,60.0,100.0,2.41,45.31,0.05
```

---

## 5. Metric Definitions

### 5.1 Frame Number
- **Source:** `scene.frameNr`
- **Units:** Integer count
- **Purpose:** Identify frame sequence

### 5.2 DeltaTime
- **Source:** `scene.dt` (time between physics steps)
- **Units:** Seconds
- **Format:** 4 decimal places
- **Example:** 0.0167 (for 60 FPS)

### 5.3 FPS
- **Calculation:** `1.0 / scene.dt` (if dt > 0)
- **Units:** Frames per second
- **Format:** 1 decimal place
- **Note:** Inverse of delta time; drops when delta time increases

### 5.4 CPU Load
- **Calculation:** `(scene.dt * 1000 / 16.67) * 100`
- **Units:** Percentage
- **Format:** 1 decimal place
- **Interpretation:** How much CPU is used relative to 60 FPS target

### 5.5 Frame Generation Time
- **Calculation:** `performance.now() - frameStartTime`
- **Units:** Milliseconds
- **Format:** 2 decimal places
- **Measured:** Actual time from frame start to logging
- **Note:** Excludes requestAnimationFrame scheduling

### 5.6 Used Memory
- **Source:** `performance.measureUserAgentSpecificMemory()` or fallback
- **Units:** Megabytes
- **Format:** 2 decimal places
- **Range:** 0 - varies based on browser limit
- **Fallback:** JS heap size if API unavailable

### 5.7 Total Elapsed
- **Calculation:** `(performance.now() - startTime) / 1000`
- **Units:** Seconds
- **Format:** 2 decimal places
- **Purpose:** Cumulative time since simulation started

---

## 6. Implementation Checklist

### Phase 1: Setup
- [ ] Declare global variables (logBuffer, frameStartTime, currentMemoryMB)
- [ ] Initialize startTime at simulation start

### Phase 2: Measurement Functions
- [ ] Implement `getMemoryStats()` with async/await
- [ ] Add try/catch for API errors
- [ ] Implement fallback to performance.memory
- [ ] Implement `getCPULoad()` calculation
- [ ] Implement `logToCSV()` as async function

### Phase 3: Export Functionality
- [ ] Implement `downloadCSV()` function
- [ ] Test file download and naming
- [ ] Add error handling (empty buffer check)

### Phase 4: Integration
- [ ] Make `update()` function async
- [ ] Add `frameStartTime = performance.now()` at frame start
- [ ] Add `await logToCSV()` before requestAnimationFrame
- [ ] Add keyboard event handler for 'e' key (export)
- [ ] Update on-screen display with real-time metrics

### Phase 5: Testing
- [ ] Test memory measurement across frame sequences
- [ ] Verify CPU load calculation
- [ ] Test CSV export with sample data
- [ ] Verify fallback behavior (no measureUserAgentSpecificMemory)
- [ ] Test on multiple browsers

---

## 7. Performance Considerations

### 7.1 Overhead Analysis

| Component | Overhead | Notes |
|-----------|----------|-------|
| `frameStartTime` capture | <0.1ms | Simple timestamp |
| `logToCSV()` metrics (excluding memory) | ~1-2ms | Calculations only |
| `getMemoryStats()` call | ~5-50ms | API call; varies by browser |
| CSV row construction | <1ms | String joining |
| **Total per frame** | **~6-53ms** | Memory measurement is expensive |

### 7.2 Optimization Strategies

**Option 1: Sample Memory Less Frequently**
```javascript
if (scene.frameNr % 10 === 0) {
    await getMemoryStats();  // Every 10th frame
}
```

**Option 2: Measure Memory Asynchronously Without Blocking**
```javascript
logToCSV().catch(e => console.warn('Logging error:', e));
// Don't await; let it run in background
```

**Option 3: Conditional Memory Measurement**
```javascript
const memStats = scene.showPressure ? await getMemoryStats() : { usedMemoryMB: 'N/A' };
```

### 7.3 Memory Overhead

- `logBuffer` grows ~100 bytes per frame
- For 60 FPS over 1 hour: ~21.6 MB stored
- Recommendation: Export and clear logBuffer periodically for long runs

---

## 8. Error Handling

### 8.1 Scenarios

| Scenario | Handling |
|----------|----------|
| API not available | Use fallback; log warning |
| Permission denied | Use fallback; log warning |
| Empty logBuffer on export | Show alert |
| Large logBuffer | Exports successfully; may take time |
| Async timing issue | Update() waits for logToCSV() completion |

### 8.2 Console Logging

- `console.warn()` for measureUserAgentSpecificMemory errors
- `console.log()` for successful CSV export with entry count

---

## 9. Usage Instructions

### For Users

1. **Start Simulation**
   - Open HTML file in Chrome/Edge for best results
   - Simulation auto-starts and logging begins

2. **Monitor Performance**
   - Watch on-screen display for real-time metrics
   - Metrics update every frame

3. **Export Data**
   - Press 'E' key during or after simulation
   - File downloads automatically with timestamp

4. **Analyze CSV**
   - Open in Excel, Google Sheets, or Python pandas
   - Plot metrics for analysis (FPS trend, memory growth, CPU usage)

### For Developers

1. **Integration**
   ```javascript
   // Add to existing animation loop:
   async function update() {
       frameStartTime = performance.now();
       simulate();
       draw();
       await logToCSV();
       requestAnimationFrame(update);
   }
   ```

2. **Customize Metrics**
   - Modify `logToCSV()` to add/remove columns
   - Update CSV header to match
   - Add calculations as needed

3. **Export Customization**
   - Change filename format in `downloadCSV()`
   - Modify CSV delimiter if needed
   - Add compression (e.g., zip before download)

---

## 10. Future Enhancements

1. **Real-time Plotting** - Display charts alongside simulation
2. **Pause/Resume Logging** - Toggle logging without pausing simulation
3. **Memory Profiling Breakdown** - Detailed breakdown by resource type
4. **Custom Metrics** - User-defined metrics to log
5. **Remote Analytics** - Send data to server instead of CSV download
6. **Performance Budgets** - Warn when metrics exceed thresholds
7. **Comparison Tool** - Compare multiple runs side-by-side

---

## 11. References

- [Performance.measureUserAgentSpecificMemory()](https://developer.mozilla.org/en-US/docs/Web/API/Performance/measureUserAgentSpecificMemory)
- [Performance.memory](https://developer.chrome.com/docs/devtools/memory-problems/memory-101/)
- [requestAnimationFrame](https://developer.mozilla.org/en-US/docs/Web/API/window/requestAnimationFrame)
- [Blob API](https://developer.mozilla.org/en-US/docs/Web/API/Blob)

---

## 12. Code Summary

**Total New Functions:** 4
- `getMemoryStats()` - ~25 lines
- `getCPULoad()` - ~5 lines
- `logToCSV()` - ~20 lines
- `downloadCSV()` - ~20 lines

**Global State:** 4 variables
**Integration Points:** 3 (update loop, keyboard handler, display update)
**External APIs Used:** 2 (measureUserAgentSpecificMemory, performance.memory)
