class Simulation{
    constructor(){
        this.particles = [];
        this.particleEmitters = [];
        this.shapes = [];
        this.springs = new Map();

        this.HEIGHT = 1781;
        this.WIDTH = 712;

        this.PARTICLE_SIZE = 3;

        
        this.INFLOW_VELOCITY = 5;

        this.AMOUNT_PARTICLES = 2000;
        this.VELOCITY_DAMPING = 1;
        this.GRAVITY = new Vector2(0,10);
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



        //sticky parameters
        this.MAXSTICKYDISTANCE = this.INTERACTION_RADIUS*1.4;
        this.K_STICK = 0.1;

        this.fluidHashGrid = new FluidHashGrid(this.INTERACTION_RADIUS);
        //this.instantiateParticles();

        this.scene = 1; // 0: inflow, 1: dam break
        if (this.scene === 0){
        this.emitter = this.createParticleEmitter(
            new Vector2(0, canvas.height/2), // position
            new Vector2(1,0), // direction
            canvas.height/10, // size
            0.5,  // spawn interval
            25, // amount
            this.INFLOW_VELOCITY  // speed
        );
        let circle = new Circle(new Vector2(canvas.width/4,canvas.height/2), canvas.height*0.15, "orange");
        this.shapes.push(circle);
        this.GRAVITY = new Vector2(0,0);
        }

        else if(this.scene === 1){

            this.AMOUNT_PARTICLES = 2000;
            this.PARTICLE_SIZE = 8;
            this.INTERACTION_RADIUS = this.PARTICLE_SIZE*1.6;
            this.instantiateParticles();
            this.BETA = 0.0;
            this.SIGMA = 0.2;
            
            // plasticity parameters
            this.GAMMA = 0.1;
            this.PLASTICITY = 0.9;
            this.SPRING_STIFFNESS = 1;
        }
        else if (this.scene === 2){
            this.PARTICLE_SIZE = 10;
            this.INTERACTION_RADIUS = this.PARTICLE_SIZE*1.6;
            let circle = new Circle(new Vector2(canvas.width/2,canvas.height/2), canvas.height*0.15, "orange");
            this.shapes.push(circle);
            this.GRAVITY = new Vector2(0,0);
            this.instantiateParticles();
            console.log("Particles instantiated: " + this.particles.length);
        }
        this.fluidHashGrid.initialize(this.particles);

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
        let offset = this.INTERACTION_RADIUS*0.95;
        let startPos = new Vector2(0, canvas.height/2);

        let xParticles = Math.sqrt(this.AMOUNT_PARTICLES);
        let yParticles = xParticles;
        if (this.scene === 2){
            xParticles = canvas.width / offset;
            yParticles = canvas.height / offset;
        }


        for(let x=0; x< xParticles; x++){
            for(let y=yParticles; y>0; y--){
                let position = new Vector2(
                    x*offset + startPos.x,
                    y*offset + startPos.y
                );
                if (this.scene === 2){
                    for (let shape of this.shapes) {
                        if (!shape.isPointInside(position)){
                            let particle = new Particle(position);
                            this.particles.push(particle);
                        }
                    }
                }
                let particle = new Particle(position);
                this.particles.push(particle);

                //particle.velocity = Scale(new Vector2(-0.5 + Math.random(),-0.5 + Math.random()), 200);


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
        this.applyGravity(dt);


        //this.inflowVelocityEnforcement();

        // compute wall distances based on current positions (used by viscosity)
        this.computeWallDistances();

        this.viscosity(dt);

        this.predictPositions(dt);

        // this.adjustSprings(dt);
        // this.springDisplacement(dt);

        // recompute wall distances on predicted positions (used by density relaxation)
        this.computeWallDistances();


        this.doubleDensityRelaxation(dt);

        this.handleOneWayCoupling();
        this.worldBoundary();
        

        this.computeNextVelocity(dt);        
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

    adjustSprings(dt){
        for(let i=0; i< this.particles.length; i++){
            let neighbours = this.fluidHashGrid.getNeighbourOfParticleIdx(i);
            let particleA = this.particles[i];

            for(let j = 0; j < neighbours.length;j++){
				let particleB = this.particles[neighbours[j]];
				if(particleA == particleB) continue;

                let springId = i + neighbours[j] * this.particles.length;

                if(this.springs.has(springId)){
                    continue;
                }

                let rij = Sub(particleB.position,particleA.position); 
                let q = rij.Length() / this.INTERACTION_RADIUS;

                if(q < 1){
                    let newSpring = new Spring(i, neighbours[j], this.INTERACTION_RADIUS);
                    this.springs.set(springId, newSpring);
                }
            }
        }


        for(let [key, spring] of this.springs){
            let pi = this.particles[spring.particleAIdx];
            let pj = this.particles[spring.particleBIdx];

            let rij = Sub(pi.position, pj.position).Length();
            let Lij = spring.length;
            let d = this.GAMMA * Lij;

            if(rij > Lij + d){
                spring.length += dt * this.PLASTICITY * (rij - Lij - d); // stretching

            }else if(rij < Lij - d){ 
                spring.length -= dt * this.PLASTICITY * (Lij - d - rij); // compression
            }

            if(spring.length > this.INTERACTION_RADIUS){
                this.springs.delete(key);
            }
        }
    }

    springDisplacement(dt){
        let dtSquared = dt * dt;

        for(let [key, spring] of this.springs){
            let pi = this.particles[spring.particleAIdx];
            let pj = this.particles[spring.particleBIdx];

            let rij = Sub(pi.position, pj.position);
            let distance = rij.Length();

            if(distance < 0.0001){
                continue;
            }

            rij.Normalize();
            let displacementTerm = dtSquared * this.SPRING_STIFFNESS * 
                (1 - spring.length / this.INTERACTION_RADIUS) * (spring.length - distance);

            rij = Scale(rij, displacementTerm * 0.5);

            pi.position = Add(pi.position, rij);
            pj.position = Sub(pj.position, rij);
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

   applyGravity(dt){
        for(let i=0; i< this.particles.length; i++){
            this.particles[i].velocity = Add(this.particles[i].velocity, Scale(this.GRAVITY, dt));
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
            if(pos.x > canvas.width && this.scene === 0){
                // remove particle that moved past the right edge
                this.particles.splice(i, 1);
                i--; // adjust index after removal
                continue;
            }
            if(pos.x > canvas.width && this.scene != 0){
                this.particles[i].position.x = canvas.width-1;
                this.particles[i].prevPosition.x = canvas.width-1;
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
    componentToHex(c) {
        var hex = Math.floor(c * 255).toString(16);
        return hex.length == 1 ? "0" + hex : hex;
    }
    getSciColor(val, minVal, maxVal) {
		val = Math.min(Math.max(val, minVal), maxVal- 0.0001);
		var d = maxVal - minVal;
		val = d == 0.0 ? 0.5 : (val - minVal) / d;
		var m = 0.25;
		var num = Math.floor(val / m);
		var s = (val - num * m) / m;
		var r, g, b;

		switch (num) {
			case 0 : r = 0.0; g = s; b = 1.0; break;
			case 1 : r = 0.0; g = 1.0; b = 1.0-s; break;
			case 2 : r = s; g = 1.0; b = 0.0; break;
			case 3 : r = 1.0; g = 1.0 - s; b = 0.0; break;
		}
        value = "#" + componentToHex(255*r) + componentToHex(255*g) + componentToHex(255*b);
		return value.toString();
	}
}