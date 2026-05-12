import { scene, setupScene } from './modules/Scene.js';
import { draw, initRenderer, getCanvasScale } from './modules/Renderer.js';
import { initInputHandler } from './modules/InputHandler.js';

/**
 * Initialize the application
 */
function init() {
	const canvas = document.getElementById("myCanvas");
	
	// Initialize renderer first to size canvas
	initRenderer(canvas);
	
	// Now get the proper scale
	const cScale = getCanvasScale();
	
	// Initialize input handler with correct scale
	initInputHandler(canvas, cScale);
	
	// Setup UI button handlers
	setupUIHandlers();
	
	// Start with scene 1
	setupScene(1);
	
	// Start animation loop
	update();
}

/**
 * Setup UI button click handlers
 */
function setupUIHandlers() {
	// Scene buttons - each has data-scene attribute
	document.querySelectorAll('.scene-button').forEach((btn) => {
		btn.addEventListener('click', () => {
			const sceneNum = parseInt(btn.dataset.scene);
			setupScene(sceneNum);
		});
	});
	
	// Toggle buttons
	document.getElementById('streamButton').addEventListener('change', (e) => {
		scene.showStreamlines = e.target.checked;
	});
	
	document.getElementById('velocityButton').addEventListener('change', (e) => {
		scene.showVelocities = e.target.checked;
	});
	
	document.getElementById('pressureButton').addEventListener('change', (e) => {
		scene.showPressure = e.target.checked;
	});
	
	document.getElementById('smokeButton').addEventListener('change', (e) => {
		scene.showSmoke = e.target.checked;
	});
	
	document.getElementById('overrelaxButton').addEventListener('change', (e) => {
		scene.overRelaxation = e.target.checked ? 1.9 : 1.0;
	});
}

/**
 * Simulate one step
 */
function simulate() {
	if (scene.fluid && !scene.paused) {
		scene.fluid.simulate(scene.dt, scene.gravity, scene.numIters, scene.overRelaxation);
	}
	scene.frameNr++;
}

/**
 * Main animation loop
 */
function update() {
	simulate();
	draw();
	requestAnimationFrame(update);
}

// Initialize when DOM is ready
document.addEventListener('DOMContentLoaded', init);
