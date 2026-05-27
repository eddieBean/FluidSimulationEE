import { scene, setObstacle } from './Scene.js';

let mouseDown = false;
let cScale = 1.0;
let canvas;

/**
 * Initialize input handler
 */
export function initInputHandler(canvasElement, scale) {
	canvas = canvasElement;
	cScale = scale;
	
	setupEventListeners();
}

/**
 * Start dragging from mouse/touch position
 */
function startDrag(x, y) {
	const bounds = canvas.getBoundingClientRect();
	const mx = x - bounds.left - canvas.clientLeft;
	const my = y - bounds.top - canvas.clientTop;
	mouseDown = true;

	const worldX = mx / cScale;
	const worldY = (canvas.height - my) / cScale;

	setObstacle(worldX, worldY, true);
}

/**
 * Handle dragging
 */
function drag(x, y) {
	if (mouseDown) {
		const bounds = canvas.getBoundingClientRect();
		const mx = x - bounds.left - canvas.clientLeft;
		const my = y - bounds.top - canvas.clientTop;
		const worldX = mx / cScale;
		const worldY = (canvas.height - my) / cScale;
		setObstacle(worldX, worldY, false);
	}
}

/**
 * End dragging
 */
function endDrag() {
	mouseDown = false;
}

/**
 * Handle keyboard input
 */
function handleKeyDown(event) {
	switch(event.key) {
		case 'p': 
			scene.paused = !scene.paused; 
			break;
		case 'm': 
			scene.paused = false; 
			// Trigger single simulation step (handled by main loop)
			scene.paused = true; 
			break;
	}
}

/**
 * Setup all event listeners
 */
function setupEventListeners() {
	// Mouse events
	canvas.addEventListener('mousedown', event => {
		startDrag(event.x, event.y);
	});

	canvas.addEventListener('mouseup', event => {
		endDrag();
	});

	canvas.addEventListener('mousemove', event => {
		drag(event.x, event.y);
	});

	// Touch events
	canvas.addEventListener('touchstart', event => {
		startDrag(event.touches[0].clientX, event.touches[0].clientY);
	});

	canvas.addEventListener('touchend', event => {
		endDrag();
	});

	canvas.addEventListener('touchmove', event => {
		event.preventDefault();
		event.stopImmediatePropagation();
		drag(event.touches[0].clientX, event.touches[0].clientY);
	}, { passive: false});

	// Keyboard events
	document.addEventListener('keydown', event => {
		handleKeyDown(event);
	});
}
