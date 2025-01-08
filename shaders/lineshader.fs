#version 330 core

in vec3 fragPosition; // Interpolated position from vertex shader
out vec4 fragColor;

uniform vec3 vertices[3]; // Triangle vertices
uniform vec4 lineColor;   // Line color
uniform float edgeThreshold; // Threshold for edge proximity

float distanceToEdge(vec3 p, vec3 a, vec3 b) {
    vec3 ab = b - a;
    vec3 ap = p - a;
    float t = clamp(dot(ap, ab) / dot(ab, ab), 0.0, 1.0);
    vec3 closestPoint = a + t * ab;
    return length(p - closestPoint);
}

void main()
{
    float d1 = distanceToEdge(fragPosition, vertices[0], vertices[1]);
    float d2 = distanceToEdge(fragPosition, vertices[1], vertices[2]);
    float d3 = distanceToEdge(fragPosition, vertices[2], vertices[0]);

    if (min(d1, min(d2, d3)) < edgeThreshold) {
        fragColor = lineColor; // Color for edges
    } else {
        discard; // Skip rendering for non-edge pixels
    }
}
