class Playground{
    constructor(){
        this.simulation = new Simulation();
    }

    update(dt){
        this.simulation.update(dt);
    }

    draw(){
        this.simulation.draw();
    }

    onMouseMove(pos){
        console.log("mouse moved to: " +pos.x+" "+pos.y);
    }

    onMouseDown(button){
        console.log("mouse button pressed: " +button);
    }

    onMouseUp(button){
        console.log("mouse button release: " +button);
    }
}