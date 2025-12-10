class Playground{
    constructor(){
        this.simulation = new Simulation();
        this.mousePos = Vector2.Zero();
        this.lastMousePos = Vector2.Zero();
        this.selectedShape = null;
    }

    update(dt){
        this.simulation.update(0.25);
    }

    draw(){
        this.simulation.draw();
    }

    onMouseMove(position){
        this.lastMousePos = this.mousePos.Cpy();
        this.mousePos = position;

        if(this.selectedShape){
            let delta = Sub(this.mousePos, this.lastMousePos);
            this.selectedShape.moveBy(delta);
        }
    }

    onMouseDown(button){

        if(button === 0){
            this.selectedShape = this.simulation.getShapeAt(this.mousePos);
        }

        if(button === 1){
            this.simulation.rotate = !this.simulation.rotate;
        }

    }

    onMouseUp(button){
        if(button === 0){
            this.selectedShape = null;
        }
    }
}