// Rectangle shape: represents an axis-aligned rectangle.
// Created by extending Polygon with four vertices.
class Rectangle extends Polygon{
    /**
     * Create a Rectangle
     * @param {number} x - left x position
     * @param {number} y - top y position
     * @param {number} width - rectangle width
     * @param {number} height - rectangle height
     * @param {string} color - fill color used for drawing
     */
    constructor(x, y, width, height, color){
        const vertices = [
            new Vector2(x, y),
            new Vector2(x + width, y),
            new Vector2(x + width, y + height),
            new Vector2(x, y + height)
        ];
        super(vertices, color);
        this.x = x;
        this.y = y;
        this.width = width;
        this.height = height;
    }
}
