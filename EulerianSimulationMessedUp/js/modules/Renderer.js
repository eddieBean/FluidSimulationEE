import { U_FIELD, V_FIELD, S_FIELD } from './Fluid.js';
import { scene } from './Scene.js';

let canvas;
let c;
let cScale;
let simWidth;

/**
 * Initialize renderer with canvas
 */
export function initRenderer(canvasElement) {
	canvas = canvasElement;
	c = canvas.getContext("2d");	
	canvas.width = window.innerWidth - 20;
	canvas.height = window.innerHeight - 100;

	const simHeight = 1.1;	
	cScale = canvas.height / simHeight;
	simWidth = canvas.width / cScale;
}

/**
 * Get canvas scale factor
 */
export function getCanvasScale() {
	return cScale;
}

/**
 * Convert world X coordinate to canvas X
 */
function cX(x) {
	return x * cScale;
}

/**
 * Convert world Y coordinate to canvas Y
 */
function cY(y) {
	return canvas.height - y * cScale;
}

/**
 * Set canvas drawing color
 */
function setColor(r, g, b) {
	c.fillStyle = `rgb(
		${Math.floor(255*r)},
		${Math.floor(255*g)},
		${Math.floor(255*b)})`;
	c.strokeStyle = `rgb(
		${Math.floor(255*r)},
		${Math.floor(255*g)},
		${Math.floor(255*b)})`;
}

/**
 * Get scientific color for value in range
 */
function getSciColor(val, minVal, maxVal) {
	val = Math.min(Math.max(val, minVal), maxVal - 0.0001);
	const d = maxVal - minVal;
	val = d == 0.0 ? 0.5 : (val - minVal) / d;
	const m = 0.25;
	const num = Math.floor(val / m);
	const s = (val - num * m) / m;
	let r, g, b;

	switch (num) {
		case 0 : r = 0.0; g = s; b = 1.0; break;
		case 1 : r = 0.0; g = 1.0; b = 1.0-s; break;
		case 2 : r = s; g = 1.0; b = 0.0; break;
		case 3 : r = 1.0; g = 1.0 - s; b = 0.0; break;
	}

	return[255*r, 255*g, 255*b, 255]
}

/**
 * Draw velocity vectors
 */
function drawVelocities() {
	c.strokeStyle = "#000000";	
	const scale = 0.02;	
	const f = scene.fluid;
	const n = f.numY;
	const h = f.h;

	for (let i = 0; i < f.numX; i++) {
		for (let j = 0; j < f.numY; j++) {

			const u = f.u[i*n + j];
			const v = f.v[i*n + j];

			c.beginPath();
			const x0 = cX(i * h);
			const x1 = cX(i * h + u * scale);
			const y = cY((j + 0.5) * h);

			c.moveTo(x0, y);
			c.lineTo(x1, y);
			c.stroke();

			const x = cX((i + 0.5) * h);
			const y0 = cY(j * h);
			const y1 = cY(j * h + v * scale);

			c.beginPath();
			c.moveTo(x, y0);
			c.lineTo(x, y1);
			c.stroke();
		}
	}
}

/**
 * Draw streamlines
 */
function drawStreamlines() {
	const f = scene.fluid;
	const segLen = f.h * 0.2;
	const numSegs = 15;

	c.strokeStyle = "#000000";

	for (let i = 1; i < f.numX - 1; i += 5) {
		for (let j = 1; j < f.numY - 1; j += 5) {

			let x = (i + 0.5) * f.h;
			let y = (j + 0.5) * f.h;

			c.beginPath();
			c.moveTo(cX(x), cY(y));

			for (let n = 0; n < numSegs; n++) {
				const u = f.sampleField(x, y, U_FIELD);
				const v = f.sampleField(x, y, V_FIELD);
				x += u * 0.01;
				y += v * 0.01;
				if (x > f.numX * f.h)
					break;

				c.lineTo(cX(x), cY(y));
			}
			c.stroke();
		}
	}
}

/**
 * Draw obstacle
 */
function drawObstacle() {
	const f = scene.fluid;
	const r = scene.obstacleRadius + f.h;
	
	if (scene.showPressure)
		c.fillStyle = "#000000";
	else
		c.fillStyle = "#DDDDDD";
	
	c.beginPath();	
	c.arc(
		cX(scene.obstacleX), cY(scene.obstacleY), cScale * r, 0.0, 2.0 * Math.PI); 
	c.closePath();
	c.fill();

	c.lineWidth = 3.0;
	c.strokeStyle = "#000000";
	c.beginPath();	
	c.arc(
		cX(scene.obstacleX), cY(scene.obstacleY), cScale * r, 0.0, 2.0 * Math.PI); 
	c.closePath();
	c.stroke();
	c.lineWidth = 1.0;
}

/**
 * Draw pressure information
 */
function drawPressureInfo(minP, maxP) {
	const s = "pressure: " + minP.toFixed(0) + " - " + maxP.toFixed(0) + " N/m";
	c.fillStyle = "#000000";
	c.font = "16px Arial";
	c.fillText(s, 10, 35);
}

/**
 * Main draw function
 */
export function draw() {
	c.clearRect(0, 0, canvas.width, canvas.height);

	// Exit if fluid not initialized
	if (!scene.fluid) {
		return;
	}

	const f = scene.fluid;
	const n = f.numY;
	const cellScale = 1.1;
	const h = f.h;

	// Find pressure range
	let minP = f.p[0];
	let maxP = f.p[0];

	for (let i = 0; i < f.numCells; i++) {
		minP = Math.min(minP, f.p[i]);
		maxP = Math.max(maxP, f.p[i]);
	}

	const id = c.getImageData(0, 0, canvas.width, canvas.height);
	let color = [255, 255, 255, 255];

	// Draw grid cells
	for (let i = 0; i < f.numX; i++) {
		for (let j = 0; j < f.numY; j++) {

			if (scene.showPressure) {
				const p = f.p[i*n + j];
				const s = f.m[i*n + j];
				color = getSciColor(p, minP, maxP);
				if (scene.showSmoke) {
					color[0] = Math.max(0.0, color[0] - 255*s);
					color[1] = Math.max(0.0, color[1] - 255*s);
					color[2] = Math.max(0.0, color[2] - 255*s);
				}
			}
			else if (scene.showSmoke) {
				const s = f.m[i*n + j];
				color[0] = 255*s;
				color[1] = 255*s;
				color[2] = 255*s;
				if (scene.sceneNr == 2)
					color = getSciColor(s, 0.0, 1.0);
			}
			else if (f.s[i*n + j] == 0.0) {
				color[0] = 0;
				color[1] = 0;
				color[2] = 0;
			}

			const x = Math.floor(cX(i * h));
			const y = Math.floor(cY((j+1) * h));
			const cx = Math.floor(cScale * cellScale * h) + 1;
			const cy = Math.floor(cScale * cellScale * h) + 1;

			const r = color[0];
			const g = color[1];
			const b = color[2];

			for (let yi = y; yi < y + cy; yi++) {
				let p = 4 * (yi * canvas.width + x);

				for (let xi = 0; xi < cx; xi++) {
					id.data[p++] = r;
					id.data[p++] = g;
					id.data[p++] = b;
					id.data[p++] = 255;
				}
			}
		}
	}

	c.putImageData(id, 0, 0);

	// Draw visualization overlays
	if (scene.showVelocities) {
		drawVelocities();
	}

	if (scene.showStreamlines) {
		drawStreamlines();
	}

	if (scene.showObstacle) {
		drawObstacle();
	}

	if (scene.showPressure) {
		drawPressureInfo(minP, maxP);
	}
}
