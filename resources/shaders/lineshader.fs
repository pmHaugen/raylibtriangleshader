#version 330 core

in vec3 fragPosition; // Interpolated position from vertex shader
out vec4 fragColor;

uniform vec3 vertices[12]; // Triangle vertices
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

    float d4 = distanceToEdge(fragPosition, vertices[3], vertices[4]);
    float d5 = distanceToEdge(fragPosition, vertices[4], vertices[5]);
    float d6 = distanceToEdge(fragPosition, vertices[5], vertices[3]);

    float d7 = distanceToEdge(fragPosition, vertices[6], vertices[7]);
    float d8 = distanceToEdge(fragPosition, vertices[7], vertices[8]);
    float d9 = distanceToEdge(fragPosition, vertices[8], vertices[6]);

    float d10 = distanceToEdge(fragPosition, vertices[9], vertices[10]);
    float d11 = distanceToEdge(fragPosition, vertices[10], vertices[11]);
    float d12 = distanceToEdge(fragPosition, vertices[11], vertices[9]);

    if (min(min(min(d1, d2), min(d3, d4)), min(min(d5, d6), min(min(d7, d8), min(d9, min(d10, min(d11, d12)))))) < edgeThreshold) {
        fragColor = lineColor; // Color for edges
    } else {
        discard; // Skip rendering for non-edge pixels
    }
}
