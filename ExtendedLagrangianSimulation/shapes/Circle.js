// Circle shape: represents a circle with a center position and radius.
// Provides utility methods for hit-testing, nearest-vector queries,
// collision resolution (direction out), movement, and drawing.
class Circle extends Shape{
    /**
     * Create a Circle
     * @param {Vector2} position - center position of the circle
     * @param {number} radius - circle radius
     * @param {string} color - stroke/fill color used for drawing
     */
    constructor(position, radius, color){
        super([]);
        this.position = position;
        this.radius = radius;
        this.color = color;
    }

    /**
     * Test whether a point is strictly inside the circle.
     * @param {Vector2} pos - point to test
     * @returns {boolean} true if point is inside (distance < radius)
     */
    isPointInside(pos){
        let distance = Sub(pos, this.position).Length();
        return distance < this.radius;
    }

    /**
     * Compute a vector from `position` to the nearest point on (or just outside)
     * the circle, taking into account a `maxStickyRange` padding.
     * If the input `position` is outside the radius + sticky range, returns null.
     *
     * Note: this method expects a global `maxStickyRange` property or similar;
     * the second parameter `maxStickyRange` is present for API clarity but the
     * implementation references `this.maxStickyRange` like the original code.
     *
     * @param {Vector2} position - point to query from
     * @param {number} maxStickyRange - extra range beyond radius to consider (not used directly)
     * @returns {Vector2|null} vector pointing outward from circle surface (or null)
     */
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

    /**
     * If `pos` is inside the circle, returns a vector that pushes the point
     * out of the circle along the radial direction. The returned vector's
     * length is equal to the penetration depth (how far inside the circle).
     * Returns null when the point is outside or exactly on the boundary.
     *
     * @param {Vector2} pos - point to test and resolve
     * @returns {Vector2|null} displacement vector to move `pos` outside
     */
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


    /**
     * Translate the circle by `delta`.
     * @param {Vector2} delta - translation vector
     */
    moveBy(delta){
        this.position = Add(this.position, delta);
    }

    /**
     * Draw the circle using DrawUtils. Uses a stroked point representation
     * with the circle's color and a fixed line width of 3.
     */
    draw(){
        DrawUtils.strokePoint(this.position, this.radius, this.color,3);
    }
}