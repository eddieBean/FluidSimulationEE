class Simulation{
    constructor(){
        this.particles = [];
        this.AMOUNT_PARTICLES = 1000;
        this.VELOCITY_DAMPING = 1;

        this.instantiateParticles();
    }

    instantiateParticles(){
        let offset = 10;
        let startPos = new Vector2(750,100);
        let xParticles = Math.sqrt(this.AMOUNT_PARTICLES);
        let yParticles = xParticles;

        for(let x = 0; x< xParticles;x++){
            for(let y = 0; y<yParticles;y++){
                let position = new Vector2(
                    x*offset + startPos.x,
                    y*offset + startPos.y
                );                
                this.particles.push(new Particle(position));
				this.particles[this.particles.length-1].velocity = Scale(new Vector2(-0.5 + Math.random(),-0.5 + Math.random()),200);

            }
        }
    }

    worldBoundary(){
        for(let i=0;i<this.particles.length;i++){
            let pos = this.particles[i].position;
            if (pos.x <0){
                this.particles[i].velocity.x *= -1;
            }
            if (pos.x >canvas.width){
                this.particles[i].velocity.x *= -1;
            }
            if (pos.y <0){
                this.particles[i].velocity.y *= -1;
            }
            if (pos.y > canvas.height){
                this.particles[i].velocity.y *= -1;
            }

        }
    }

    update(dt){
        
        this.predictPositions(dt);

        this.computeNextVelocity(dt);

        this.worldBoundary();
    }
    predictPositions(dt){
		for(let i = 0; i < this.particles.length;i++){
			this.particles[i].prevPosition = this.particles[i].position.Cpy();
			this.particles[i].position = Add(this.particles[i].position, Scale(this.particles[i].velocity,dt * this.VELOCITY_DAMPING));
		}		
	}

	computeNextVelocity(dt){
		for(let i = 0; i < this.particles.length;i++){
			this.particles[i].velocity = Scale(Sub(this.particles[i].position,this.particles[i].prevPosition),1/dt);
		}		
	}

    draw(){
        for(let i = 0; i<this.particles.length;i++){
            let position = this.particles[i].position;
            let color = this.particles[i].color
            DrawUtils.drawPoint(position,5,color);
        }

    }
}
