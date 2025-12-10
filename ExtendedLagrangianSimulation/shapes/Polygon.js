class Polygon extends Shape{
    constructor(vertices, color){
        super(vertices);
        this.color = color;
        this.normals = [];
        this.calculateNormals();
    }

    calculateNormals(){
        
        for(let i=0; i < this.vertices.length-1;i++){
            let direction = Sub(this.vertices[i+1], this.vertices[i]);
            let normal = direction.GetNormal();
            normal.Normalize();
            this.normals.push(normal);
        }

        let direction = Sub(this.vertices[0], this.vertices[this.vertices.length-1]);
        let normal = direction.GetNormal();
        normal.Normalize();
        this.normals.push(normal);
    }
    
    draw(){
        super.draw();
/*
        for(let i=0; i < this.vertices.length-1;i++){
            let direction = Sub(this.vertices[i+1], this.vertices[i]);
            let start = Add(this.vertices[i], Scale(direction, 0.5));
            let end = Add(start, Scale(this.normals[i],15));
            DrawUtils.drawLine(start, end, "orange");
        }
        let direction = Sub(this.vertices[0], this.vertices[this.vertices.length-1]);
        let start = Add(this.vertices[this.vertices.length-1], Scale(direction, 0.5));
        let end = Add(start, Scale(this.normals[this.vertices.length-1],15));
        DrawUtils.drawLine(start, end, "orange");*/

        /*
        for(let i=0; i < this.vertices.length;i++){
           let start = this.vertices[i];
           let end = Add(start, Scale(this.normals[i],15));
           DrawUtils.drawLine(start, end, "orange");
        }
        */
    }
}