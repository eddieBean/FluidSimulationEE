class Circle extends Shape{
    constructor(position, radius, color){
        super([]);
        this.position = position;
        this.radius = radius;
        this.color = color;
    }

    isPointInside(pos){
        let distance = Sub(pos, this.position).Length();
        return distance < this.radius;
    }

    getNearestVector(position, maxStickyRange){
        let direction = Sub(position, this.position);
        let length = direction.Length();
        if (length<this.radius+ this.maxStickyRange){
            direction.Normalize();
            direction = Scale(direction,this.radius+ this.maxStickyRange-length);
            return direction;
        }else{
            return null;
        }
    }

    getDirectionOut(pos){
        let direction = Sub(pos, this.position);
        if(direction.Length2() < this.radius * this.radius){
            let penetration = this.radius - direction.Length();
            direction.Normalize();
            direction = Scale(direction, penetration);
            return direction;
        }else{
            return null;
        }
    }


    moveBy(delta){
        this.position = Add(this.position, delta);
    }

    draw(){
        DrawUtils.strokePoint(this.position, this.radius, this.color,3);
    }
}