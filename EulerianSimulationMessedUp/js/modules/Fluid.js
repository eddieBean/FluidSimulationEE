// Field type constants
export const U_FIELD = 0;
export const V_FIELD = 1;
export const S_FIELD = 2;

/**
 * Fluid simulator using Eulerian grid-based approach
 */
export class Fluid {
	constructor(density, numX, numY, h) {
		this.density = density;
		this.numX = numX + 2; 
		this.numY = numY + 2;
		this.numCells = this.numX * this.numY;
		this.h = h;
		this.u = new Float32Array(this.numCells);
		this.v = new Float32Array(this.numCells);
		this.newU = new Float32Array(this.numCells);
		this.newV = new Float32Array(this.numCells);
		this.p = new Float32Array(this.numCells);
		this.s = new Float32Array(this.numCells);
		this.m = new Float32Array(this.numCells);
		this.newM = new Float32Array(this.numCells);
		this.m.fill(1.0)
	}

	/**
	 * Apply gravity to vertical velocity component
	 */
	integrate(dt, gravity) {
		var n = this.numY;
		for (var i = 1; i < this.numX; i++) {
			for (var j = 1; j < this.numY-1; j++) {
				if (this.s[i*n + j] != 0.0 && this.s[i*n + j-1] != 0.0)
					this.v[i*n + j] += gravity * dt;
			}	 
		}
	}

	/**
	 * Solve incompressibility constraint using pressure projection
	 */
	solveIncompressibility(numIters, dt, overRelaxation) {
		var n = this.numY;
		var cp = this.density * this.h / dt;

		for (var iter = 0; iter < numIters; iter++) {
			for (var i = 1; i < this.numX-1; i++) {
				for (var j = 1; j < this.numY-1; j++) {

					if (this.s[i*n + j] == 0.0)
						continue;

					var s = this.s[i*n + j];
					var sx0 = this.s[(i-1)*n + j];
					var sx1 = this.s[(i+1)*n + j];
					var sy0 = this.s[i*n + j-1];
					var sy1 = this.s[i*n + j+1];
					var s = sx0 + sx1 + sy0 + sy1;
					if (s == 0.0)
						continue;

					var div = this.u[(i+1)*n + j] - this.u[i*n + j] + 
						this.v[i*n + j+1] - this.v[i*n + j];

					var p = -div / s;
					p *= overRelaxation;
					this.p[i*n + j] += cp * p;

					this.u[i*n + j] -= sx0 * p;
					this.u[(i+1)*n + j] += sx1 * p;
					this.v[i*n + j] -= sy0 * p;
					this.v[i*n + j+1] += sy1 * p;
				}
			}
		}
	}

	/**
	 * Extrapolate velocities at boundaries
	 */
	extrapolate() {
		var n = this.numY;
		for (var i = 0; i < this.numX; i++) {
			this.u[i*n + 0] = this.u[i*n + 1];
			this.u[i*n + this.numY-1] = this.u[i*n + this.numY-2]; 
		}
		for (var j = 0; j < this.numY; j++) {
			this.v[0*n + j] = this.v[1*n + j];
			this.v[(this.numX-1)*n + j] = this.v[(this.numX-2)*n + j] 
		}
	}

	/**
	 * Sample field value at arbitrary position using bilinear interpolation
	 */
	sampleField(x, y, field) {
		var n = this.numY;
		var h = this.h;
		var h1 = 1.0 / h;
		var h2 = 0.5 * h;

		x = Math.max(Math.min(x, this.numX * h), h);
		y = Math.max(Math.min(y, this.numY * h), h);

		var dx = 0.0;
		var dy = 0.0;
		var f;

		switch (field) {
			case U_FIELD: f = this.u; dy = h2; break;
			case V_FIELD: f = this.v; dx = h2; break;
			case S_FIELD: f = this.m; dx = h2; dy = h2; break
		}

		var x0 = Math.min(Math.floor((x-dx)*h1), this.numX-1);
		var tx = ((x-dx) - x0*h) * h1;
		var x1 = Math.min(x0 + 1, this.numX-1);
		
		var y0 = Math.min(Math.floor((y-dy)*h1), this.numY-1);
		var ty = ((y-dy) - y0*h) * h1;
		var y1 = Math.min(y0 + 1, this.numY-1);

		var sx = 1.0 - tx;
		var sy = 1.0 - ty;

		var val = sx*sy * f[x0*n + y0] +
			tx*sy * f[x1*n + y0] +
			tx*ty * f[x1*n + y1] +
			sx*ty * f[x0*n + y1];
		
		return val;
	}

	/**
	 * Average U velocity at cell
	 */
	avgU(i, j) {
		var n = this.numY;
		var u = (this.u[i*n + j-1] + this.u[i*n + j] +
			this.u[(i+1)*n + j-1] + this.u[(i+1)*n + j]) * 0.25;
		return u;
	}

	/**
	 * Average V velocity at cell
	 */
	avgV(i, j) {
		var n = this.numY;
		var v = (this.v[(i-1)*n + j] + this.v[i*n + j] +
			this.v[(i-1)*n + j+1] + this.v[i*n + j+1]) * 0.25;
		return v;
	}

	/**
	 * Advect velocity field using semi-Lagrangian method
	 */
	advectVel(dt) {
		this.newU.set(this.u);
		this.newV.set(this.v);

		var n = this.numY;
		var h = this.h;
		var h2 = 0.5 * h;

		for (var i = 1; i < this.numX; i++) {
			for (var j = 1; j < this.numY; j++) {

				// u component
				if (this.s[i*n + j] != 0.0 && this.s[(i-1)*n + j] != 0.0 && j < this.numY - 1) {
					var x = i*h;
					var y = j*h + h2;
					var u = this.u[i*n + j];
					var v = this.avgV(i, j);
					x = x - dt*u;
					y = y - dt*v;
					u = this.sampleField(x,y, U_FIELD);
					this.newU[i*n + j] = u;
				}
				// v component
				if (this.s[i*n + j] != 0.0 && this.s[i*n + j-1] != 0.0 && i < this.numX - 1) {
					var x = i*h + h2;
					var y = j*h;
					var u = this.avgU(i, j);
					var v = this.v[i*n + j];
					x = x - dt*u;
					y = y - dt*v;
					v = this.sampleField(x,y, V_FIELD);
					this.newV[i*n + j] = v;
				}
			}	 
		}

		this.u.set(this.newU);
		this.v.set(this.newV);
	}

	/**
	 * Advect smoke/density field
	 */
	advectSmoke(dt) {
		this.newM.set(this.m);

		var n = this.numY;
		var h = this.h;
		var h2 = 0.5 * h;

		for (var i = 1; i < this.numX-1; i++) {
			for (var j = 1; j < this.numY-1; j++) {

				if (this.s[i*n + j] != 0.0) {
					var u = (this.u[i*n + j] + this.u[(i+1)*n + j]) * 0.5;
					var v = (this.v[i*n + j] + this.v[i*n + j+1]) * 0.5;
					var x = i*h + h2 - dt*u;
					var y = j*h + h2 - dt*v;

					this.newM[i*n + j] = this.sampleField(x,y, S_FIELD);
 				}
			}	 
		}
		this.m.set(this.newM);
	}

	/**
	 * Execute one simulation step
	 */
	simulate(dt, gravity, numIters, overRelaxation) {
		this.integrate(dt, gravity);
		this.p.fill(0.0);
		this.solveIncompressibility(numIters, dt, overRelaxation);
		this.extrapolate();
		this.advectVel(dt);
		this.advectSmoke(dt);
	}
}
