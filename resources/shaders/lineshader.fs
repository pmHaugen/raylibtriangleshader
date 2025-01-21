#version 330
in vec4 fragColor;
in vec3 barycentric; // Barycentric input from vertex shader
out vec4 finalColor;

void main() {
    // Calculate edge width using barycentric derivatives
    vec3 edge = fwidth(barycentric);
    vec3 factor = smoothstep(vec3(0.0), edge * 1.5, barycentric);
    float edgeFactor = 1.0 - min(min(factor.x, factor.y), factor.z);
    
    // Mix between original color and line color (black)
    finalColor = mix(fragColor, vec4(0.0, 0.0, 0.0, 1.0), edgeFactor);
}