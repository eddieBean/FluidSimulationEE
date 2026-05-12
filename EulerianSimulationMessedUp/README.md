# Eulerian Fluid Simulation - Refactored

This is a refactored and reorganized version of the Eulerian fluid simulation with improved readability and maintainability.

## Project Structure

```
EulerianSimulation/
├── index.html                 # Main HTML entry point
├── css/
│   └── styles.css            # All styling
├── js/
│   ├── app.js                # Main application file
│   └── modules/
│       ├── Fluid.js          # Eulerian fluid simulator class
│       ├── Scene.js          # Scene setup and configuration
│       ├── Renderer.js       # Drawing and rendering functions
│       └── InputHandler.js   # User input handling
```

## File Descriptions

### `index.html`
Clean HTML markup with semantic structure. Contains:
- Canvas element for rendering
- Control buttons for different scenes
- Checkboxes for visualization options
- References to external CSS and JavaScript modules

### `css/styles.css`
All styling including:
- Layout and spacing
- Button styling with hover effects
- Canvas styling
- Label and checkbox styling

### `js/app.js`
Main application coordinator that:
- Initializes all modules
- Sets up event handlers for UI controls
- Manages the animation loop
- Handles simulation steps

### `js/modules/Fluid.js`
Core physics simulation engine:
- `Fluid` class implementing the Eulerian grid-based solver
- Methods for velocity advection, pressure projection, etc.
- Field sampling and interpolation
- Constants for field types (U_FIELD, V_FIELD, S_FIELD)

### `js/modules/Scene.js`
Scene management and configuration:
- `scene` object with all simulation parameters
- `setupScene()` function for different simulation scenarios
- `setObstacle()` function for interactive obstacle control
- Scene-specific parameters (gravity, resolution, etc.)

### `js/modules/Renderer.js`
Visualization and drawing:
- Canvas initialization
- Coordinate transformation (world to canvas)
- `draw()` function for rendering the simulation
- Overlay drawing (velocities, streamlines, pressure, etc.)
- Helper functions for colors and visualization

### `js/modules/InputHandler.js`
User interaction handling:
- Mouse drag events for obstacle control
- Touch events for mobile compatibility
- Keyboard controls (p for pause, m for single step)
- Event listener setup and management

## Improvements Over Original

1. **Modularity**: Code is split into logical, reusable modules
2. **Readability**: Each file has a clear, single responsibility
3. **Maintainability**: Easier to debug and extend individual components
4. **Documentation**: Functions have JSDoc comments explaining purpose
5. **Cleaner HTML**: Separated concerns - HTML for structure, CSS for styling, JS for logic
6. **Better Naming**: More descriptive variable names and function names
7. **Performance**: Module imports only what's needed

## Usage

1. Open `index.html` in a web browser
2. Choose a scene from the buttons (Wind Tunnel, Hires Tunnel, Tank, Paint)
3. Use checkboxes to toggle visualizations:
   - **Streamlines**: Shows flow direction
   - **Velocities**: Shows velocity vectors
   - **Pressure**: Shows pressure field
   - **Smoke**: Shows density field
   - **Overrelax**: Enables acceleration of solver convergence

## Interaction

- **Mouse/Touch**: Drag to move the obstacle around
- **Keyboard**: Press 'p' to pause/resume, 'm' for single step when paused

## Technical Notes

- Grid-based Eulerian approach for fluid simulation
- Pressure projection for incompressibility
- Semi-Lagrangian advection
- Bilinear interpolation for smooth field sampling
- Over-relaxation option for faster convergence
