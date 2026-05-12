import { Fluid } from './Fluid.js';

/**
 * Scene configuration and setup
 */
export const scene = {
	gravity: -9.81,
	dt: 1.0 / 120.0,
	numIters: 100,
	frameNr: 0,
	overRelaxation: 1.9,
	obstacleX: 0.0,
	obstacleY: 0.0,
	obstacleRadius: 0.15,
	paused: false,
	sceneNr: 0,
	showObstacle: false,
	showStreamlines: false,
	showVelocities: false,	
	showPressure: false,
	showSmoke: true,
	fluid: null
};

/**
 * Initialize scene based on scenario number
 */
export function setupScene(sceneNr = 0) {
	scene.sceneNr = sceneNr;
	scene.obstacleRadius = 0.15;
	scene.overRelaxation = 1.9;

	scene.dt = 1.0 / 60.0;
	scene.numIters = 40;

	let res = 100;
	
	if (sceneNr == 0)
		res = 50;
	else if (sceneNr == 3)
		res = 200;

	const domainHeight = 1.0;
	const simHeight = 1.1;
	const canvas = document.getElementById("myCanvas");
	const simWidth = canvas.width / (canvas.height / simHeight);
	const domainWidth = domainHeight / simHeight * simWidth;
	const h = domainHeight / res;

	const numX = Math.floor(domainWidth / h);
	const numY = Math.floor(domainHeight / h);

	const density = 1000.0;

	const f = scene.fluid = new Fluid(density, numX, numY, h);

	const n = f.numY;

	if (sceneNr == 0) {   		// tank
		for (let i = 0; i < f.numX; i++) {
			for (let j = 0; j < f.numY; j++) {
				let s = 1.0;	// fluid
				if (i == 0 || i == f.numX-1 || j == 0)
					s = 0.0;	// solid
				f.s[i*n + j] = s
			}
		}
		scene.gravity = -9.81;
		scene.showPressure = true;
		scene.showSmoke = false;
		scene.showStreamlines = false;
		scene.showVelocities = false;
	}
	else if (sceneNr == 1 || sceneNr == 3) { // vortex shedding
		const inVel = 2.0;
		for (let i = 0; i < f.numX; i++) {
			for (let j = 0; j < f.numY; j++) {
				let s = 1.0;	// fluid
				if (i == 0 || j == 0 || j == f.numY-1)
					s = 0.0;	// solid
				f.s[i*n + j] = s

				if (i == 1) {
					f.u[i*n + j] = inVel;
				}
			}
		}

		const pipeH = 0.1 * f.numY;
		const minJ = Math.floor(0.5 * f.numY - 0.5*pipeH);
		const maxJ = Math.floor(0.5 * f.numY + 0.5*pipeH);

		for (let j = minJ; j < maxJ; j++)
			f.m[j] = 0.0;

		setObstacle(0.4, 0.5, true)

		scene.gravity = 0.0;
		scene.showPressure = false;
		scene.showSmoke = true;
		scene.showStreamlines = false;
		scene.showVelocities = false;

		if (sceneNr == 3) {
			scene.dt = 1.0 / 120.0;
			scene.numIters = 100;
			scene.showPressure = true;
		}

	}
	else if (sceneNr == 2) { // paint
		scene.gravity = 0.0;
		scene.overRelaxation = 1.0;
		scene.showPressure = false;
		scene.showSmoke = true;
		scene.showStreamlines = false;
		scene.showVelocities = false;
		scene.obstacleRadius = 0.1;
	}

	// Update UI checkboxes
	document.getElementById("streamButton").checked = scene.showStreamlines;
	document.getElementById("velocityButton").checked = scene.showVelocities;
	document.getElementById("pressureButton").checked = scene.showPressure;
	document.getElementById("smokeButton").checked = scene.showSmoke;
	document.getElementById("overrelaxButton").checked = scene.overRelaxation > 1.0;
}

/**
 * Set obstacle position and properties
 */
export function setObstacle(x, y, reset) {
	let vx = 0.0;
	let vy = 0.0;

	if (!reset) {
		vx = (x - scene.obstacleX) / scene.dt;
		vy = (y - scene.obstacleY) / scene.dt;
	}

	scene.obstacleX = x;
	scene.obstacleY = y;
	const r = scene.obstacleRadius;
	const f = scene.fluid;
	const n = f.numY;
	const cd = Math.sqrt(2) * f.h;

	for (let i = 1; i < f.numX-2; i++) {
		for (let j = 1; j < f.numY-2; j++) {

			f.s[i*n + j] = 1.0;

			const dx = (i + 0.5) * f.h - x;
			const dy = (j + 0.5) * f.h - y;

			if (dx * dx + dy * dy < r * r) {
				f.s[i*n + j] = 0.0;
				if (scene.sceneNr == 2) 
					f.m[i*n + j] = 0.5 + 0.5 * Math.sin(0.1 * scene.frameNr)
				else 
					f.m[i*n + j] = 1.0;
				f.u[i*n + j] = vx;
				f.u[(i+1)*n + j] = vx;
				f.v[i*n + j] = vy;
				f.v[i*n + j+1] = vy;
			}
		}
	}
	
	scene.showObstacle = true;
}
