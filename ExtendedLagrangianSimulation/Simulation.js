class Simulation{
    constructor(){
        this.particles = [];
        this.particleEmitters = [];
        this.shapes = [];
        //this.springs = new Map();

        this.PARTICLE_SIZE = 3;

        
        this.INFLOW_VELOCITY = 5;

        this.AMOUNT_PARTICLES = 2000;
        this.VELOCITY_DAMPING = 1;
        this.GRAVITY = new Vector2(0,0);
        this.REST_DENSITY = 0;
        this.K_NEAR = 3.0;
        this.K = 0.8;
        this.INTERACTION_RADIUS = 5;

        // viscouse parameters
        this.SIGMA = 0.2;
        this.BETA = 0;

        // plasticity parameters
        this.GAMMA = 0;
        this.PLASTICITY = 0;
        this.SPRING_STIFFNESS = 0;

        // wall-awareness tuning
        this.WALL_PRESSURE_MIN = 0.4; // minimum pressure multiplier at the wall (0.3-0.5 suggested)
        this.WALL_VISCOSITY_MIN = 0.5; // minimum viscosity multiplier at the wall (~0.5 suggested)
        this.WALL_FRICTION = 0.02; // low tangential friction when colliding with walls

        // debug / runtime flags
        this.DEBUG = false; // set true to enable periodic console logging
        this.frameCount = 0;
        this.DEBUG_LOG_INTERVAL = 60; // frames between debug logs

        //sticky parameters
        this.MAXSTICKYDISTANCE = this.INTERACTION_RADIUS*1.4;
        this.K_STICK = 0.1;

        this.fluidHashGrid = new FluidHashGrid(this.INTERACTION_RADIUS);
        this.instantiateParticles(x, y, this.AMOUNT_PARTICLES);
        this.fluidHashGrid.initialize(this.particles);


        this.emitter = this.createParticleEmitter(
            new Vector2(0, canvas.height/2), // position
            new Vector2(1,0), // direction
            100, // size
            0.5,  // spawn interval
            30, // amount
            this.INFLOW_VELOCITY  // speed
        );

        let circle = new Circle(new Vector2(canvas.width/4,canvas.height/2), this.PARTICLE_SIZE*10, "orange");
        this.shapes.push(circle);

    }

    createParticleEmitter(position, direction, size, spawnInterval, amount, velocity){
        let emitter = new ParticleEmitter(position, direction, size, spawnInterval, amount, velocity);
        this.particleEmitters.push(emitter);
        return emitter;
    }

    getShapeAt(pos){
        for(let i=0; i< this.shapes.length; i++){
            if(this.shapes[i].isPointInside(pos)){
                return this.shapes[i];
            }
        }
        return null;
    }

    instantiateParticles(){
        let offsetBetweenParticles = 3;
        let offsetAllParticles = new Vector2(750, 100);

        let xParticles = Math.sqrt(this.AMOUNT_PARTICLES);
        let yParticles = xParticles;


        for(let x=0; x< xParticles; x++){
            for(let y=0; y< yParticles; y++){
                let position = new Vector2(
                    x*offsetBetweenParticles + offsetAllParticles.x,
                    y*offsetBetweenParticles + offsetAllParticles.y);

                let particle = new Particle(position);
                //particle.velocity = Scale(new Vector2(-0.5 + Math.random(),-0.5 + Math.random()), 200);


                this.particles.push(particle);
            }
        }
    }

    neighbourSearch(){
        this.fluidHashGrid.clearGrid();
        this.fluidHashGrid.mapParticleToCell();
    }

    // Compute distance of each particle to the nearest circular wall (if any)
    // and mark particles that are within interaction radius as `nearWall`.
    computeWallDistances(){
        for(let i=0;i<this.particles.length;i++){
            let p = this.particles[i];
            let minDist = Infinity;

            for(let s=0; s<this.shapes.length; s++){
                let shape = this.shapes[s];
                if(shape.position !== undefined && shape.radius !== undefined){
                    let d = Sub(p.position, shape.position).Length() - shape.radius;
                    if(d < minDist) minDist = d;
                }
            }

            p.wallDist = minDist;
            p.nearWall = (minDist < this.INTERACTION_RADIUS);
        }
    }

    update(dt){
        this.neighbourSearch();

        for(let i=0; i< this.particleEmitters.length; i++){
            this.particleEmitters[i].spawn(dt, this.particles);
            if(this.rotate){
                this.emitter.rotate(0.01);
            }
        }


        //this.inflowVelocityEnforcement();

        // compute wall distances based on current positions (used by viscosity)
        this.computeWallDistances();

        this.viscosity(dt);

        this.predictPositions(dt);

        // recompute wall distances on predicted positions (used by density relaxation)
        this.computeWallDistances();


        this.doubleDensityRelaxation(dt);

        this.handleOneWayCoupling();
        this.worldBoundary();

        this.computeNextVelocity(dt);

        // debug logging (periodic)
        if(this.DEBUG){
            this.frameCount++;
            if(this.frameCount % this.DEBUG_LOG_INTERVAL === 0){
                let nearCount = 0;
                let sumDist = 0;
                for(let i=0;i<this.particles.length;i++){
                    let p = this.particles[i];
                    if(p.nearWall){ nearCount++; sumDist += Math.max(0, p.wallDist); }
                }
                let avgDist = nearCount > 0 ? (sumDist / nearCount).toFixed(3) : 'n/a';
                console.log(`Sim Debug: particles=${this.particles.length}, nearWall=${nearCount}, avgWallDist=${avgDist}, K=${this.K.toFixed(3)}, K_NEAR=${this.K_NEAR.toFixed(3)}, SIGMA=${this.SIGMA.toFixed(3)}`);
            }
        }
    }



handleOneWayCoupling(){
    for (let particle of this.particles) {
        for (let shape of this.shapes) {

            let dir = shape.getDirectionOut(particle.position);
            if (dir !== null){
                // --- push particle out ---
                particle.position = Add(particle.position, dir);

                // --- compute normal ---
                let n = dir.Normalize();
                // --- normal velocity ---
                let vn = particle.velocity.Dot(n);

                if (vn < 0) {
                    // remove inward normal motion
                    particle.velocity = Sub(
                        particle.velocity,
                        Scale(n, vn)
                    );

                    // --- partial slip (low tangential friction) ---
                    const friction = this.WALL_FRICTION;
                    particle.velocity = Scale(particle.velocity, 1 - friction);
                }
            }
        }
    }
}

    viscosity(dt){
        for(let i=0; i< this.particles.length; i++){
			let neighbours = this.fluidHashGrid.getNeighbourOfParticleIdx(i);
			let particleA = this.particles[i];

            for(let j = 0; j < neighbours.length;j++){
				let particleB = this.particles[neighbours[j]];
				if(particleA == particleB) continue;

                let rij = Sub(particleB.position,particleA.position);
                let velocityA = particleA.velocity;
                let velocityB = particleB.velocity;
                let q = rij.Length() / this.INTERACTION_RADIUS;
                
                if(q < 1){

                    rij.Normalize();
                    let u = Sub(velocityA, velocityB).Dot(rij);

                    if(u > 0){
                        // attenuate viscosity near walls: compute per-particle scale in [WALL_VISCOSITY_MIN, 1]
                        let wallDistA = (particleA.wallDist !== undefined) ? particleA.wallDist : Infinity;
                        let wallDistB = (particleB.wallDist !== undefined) ? particleB.wallDist : Infinity;

                        let clampA = Math.max(0, Math.min(1, wallDistA / this.INTERACTION_RADIUS));
                        let clampB = Math.max(0, Math.min(1, wallDistB / this.INTERACTION_RADIUS));

                        let scaleA = particleA.nearWall ? (this.WALL_VISCOSITY_MIN + (1 - this.WALL_VISCOSITY_MIN) * clampA) : 1.0;
                        let scaleB = particleB.nearWall ? (this.WALL_VISCOSITY_MIN + (1 - this.WALL_VISCOSITY_MIN) * clampB) : 1.0;

                        // use the smaller scale (more attenuation) when only one particle is near-wall
                        let sigmaEff = this.SIGMA * Math.min(scaleA, scaleB);

                        let ITerm = dt * (1-q) * (sigmaEff * u + this.BETA * u * u);
                        let I = Scale(rij, ITerm);

                        particleA.velocity = Sub(particleA.velocity, Scale(I, 0.5));
                        particleB.velocity = Add(particleB.velocity, Scale(I, 0.5));
                    }
                }
            }
        }
    }

    doubleDensityRelaxation(dt){
        for(let i=0; i< this.particles.length; i++){
			let density = 0;
			let densityNear = 0;
			let neighbours = this.fluidHashGrid.getNeighbourOfParticleIdx(i);
			let particleA = this.particles[i];

			for(let j = 0; j < neighbours.length;j++){
				let particleB = this.particles[neighbours[j]];
				if(particleA == particleB) continue;
				
				
				let rij = Sub(particleB.position,particleA.position);
				let q = rij.Length() / this.INTERACTION_RADIUS;
				
				if(q < 1){
					let oneMinusQ = (1-q);
					density += oneMinusQ*oneMinusQ;
					densityNear += oneMinusQ*oneMinusQ*oneMinusQ;					
				}
			}

            let pressure = this.K * (density - this.REST_DENSITY);
            let pressureNear = this.K_NEAR * densityNear;
            let particleADisplacement = Vector2.Zero();

            // attenuate pressure for particles near walls to compensate truncated kernel
            if(particleA.nearWall){
                let wd = (particleA.wallDist !== undefined) ? particleA.wallDist : 0;
                let t = Math.max(0, Math.min(1, wd / this.INTERACTION_RADIUS));
                let factor = this.WALL_PRESSURE_MIN + (1 - this.WALL_PRESSURE_MIN) * t;
                pressure *= factor;
                pressureNear *= factor;
            }

            
            for(let j=0; j< neighbours.length; j++){
                let particleB = this.particles[neighbours[j]];
                if(particleA == particleB){
                    continue;
                }

                let rij = Sub(particleB.position, particleA.position);
                let q = rij.Length() / this.INTERACTION_RADIUS;

                if(q < 1.0){
                    rij.Normalize();
                    let displacementTerm = Math.pow(dt, 2) * 
                        (pressure * (1-q) + pressureNear * Math.pow(1-q, 2));
                    let D = Scale(rij, displacementTerm);

                    particleB.position = Add(particleB.position, Scale(D,0.5));
                    particleADisplacement = Sub(particleADisplacement, Scale(D,0.5));
                }
            }
            particleA.position = Add(particleA.position, particleADisplacement);
        }
    }

    inflowVelocityEnforcement(){
        for(let i=0; i< this.particles.length; i++) {
            if (this.particles[i].position.x < 0) {
            this.particles[i].velocity = new Vector2(this.INFLOW_VELOCITY, 0);
            }
        }
    }

    predictPositions(dt){
        for(let i=0; i< this.particles.length; i++){
            this.particles[i].prevPosition = this.particles[i].position.Cpy();
            let positionDelta = Scale(this.particles[i].velocity, dt * this.VELOCITY_DAMPING);
            this.particles[i].position = Add(this.particles[i].position, positionDelta);
        }
    }

    computeNextVelocity(dt){
        for(let i=0; i< this.particles.length; i++){
            let velocity = Scale(Sub(this.particles[i].position, this.particles[i].prevPosition), 1.0 / dt);
            this.particles[i].velocity = velocity;
        }
    }

    worldBoundary(){
        for(let i=0; i< this.particles.length; i++){
            let pos = this.particles[i].position;

            if(pos.x < 0){
                this.particles[i].position.x = 0;
                this.particles[i].prevPosition.x = 0;
            }
            if(pos.y < 0){
                this.particles[i].position.y = 0;
                this.particles[i].prevPosition.y = 0;
            }
            if(pos.x > canvas.width){
                // remove particle that moved past the right edge
                this.particles.splice(i, 1);
                i--; // adjust index after removal
                continue;
            }
            if(pos.y > canvas.height){
                this.particles[i].position.y = canvas.height-1;
                this.particles[i].prevPosition.y = canvas.height-1;
            }
        }
    }


    draw(){
        for(let i=0; i< this.shapes.length; i++){
            this.shapes[i].draw();
        }

        for(let i=0; i< this.particles.length; i++){
            let position = this.particles[i].position;
            let color = this.particles[i].color;
            DrawUtils.drawPoint(position, this.PARTICLE_SIZE, color);
        }

        for(let i=0; i< this.particleEmitters.length; i++){
            this.particleEmitters[i].draw();
        }
    }
}