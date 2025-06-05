//
//  OccupancyMapShaders.metal
//  MobileRobotics
//
//  Metal shaders for accelerated occupancy map rendering
//

#include <metal_stdlib>
using namespace metal;

struct MapUniforms {
    float4 visibleRegion; // startX, startY, endX, endY
    float2 robotPosition;
    float robotHeading;
    float2 viewportSize;
    float2 frontierTarget;
};

struct VertexOut {
    float4 position [[position]];
    float2 texCoord;
};

// MARK: - Map Rendering Shaders

vertex VertexOut mapVertexShader(uint vertexID [[vertex_id]],
                                 constant float4 *vertices [[buffer(0)]],
                                 constant MapUniforms &uniforms [[buffer(1)]]) {
    VertexOut out;
    out.position = float4(vertices[vertexID].xy, 0.0, 1.0);
    out.texCoord = vertices[vertexID].zw;
    return out;
}

fragment float4 mapFragmentShader(VertexOut in [[stage_in]],
                                 texture2d<uint> mapTexture [[texture(0)]],
                                 constant MapUniforms &uniforms [[buffer(0)]]) {
    // Calculate which part of the map texture to sample
    float2 visibleSize = uniforms.visibleRegion.zw - uniforms.visibleRegion.xy + 1.0;
    float2 mapCoord = uniforms.visibleRegion.xy + in.texCoord * visibleSize;
    
    // Check bounds
    if (mapCoord.x < 0 || mapCoord.x >= 500 || mapCoord.y < 0 || mapCoord.y >= 500) {
        return float4(0.1, 0.1, 0.1, 1.0); // Dark gray for out of bounds
    }
    
    // Sample the texture
    uint2 texCoord = uint2(mapCoord);
    uint mapValue = mapTexture.read(texCoord).r;
    
    // Check if this is the frontier target
    bool isFrontierTarget = (distance(float2(texCoord), uniforms.frontierTarget) < 0.5);
    
    // Color based on occupancy value
    float4 color;
    if (isFrontierTarget && uniforms.frontierTarget.x >= 0) {
        color = float4(1.0, 0.0, 0.0, 1.0); // Red for frontier target
    } else if (mapValue == 0) {
        color = float4(1.0, 1.0, 1.0, 1.0); // White for free space
    } else if (mapValue == 255) {
        color = float4(0.0, 0.0, 0.0, 1.0); // Black for occupied
    } else if (mapValue == 128) {
        color = float4(0.5, 0.5, 0.5, 1.0); // Gray for unknown
    } else {
        color = float4(0.5, 0.0, 0.5, 1.0); // Purple for error
    }
    
    // Add subtle grid lines for better visibility when zoomed in
    float2 gridPos = fract(mapCoord);
    if (gridPos.x < 0.02 || gridPos.y < 0.02) {
        color = mix(color, float4(0.2, 0.2, 0.2, 1.0), 0.2);
    }
    
    return color;
}

// MARK: - Robot Overlay Shaders

vertex VertexOut robotVertexShader(uint vertexID [[vertex_id]],
                                  constant MapUniforms &uniforms [[buffer(0)]]) {
    // Create vertices for full screen quad
    float2 vertices[4] = {
        float2(-1.0, -1.0),
        float2( 1.0, -1.0),
        float2(-1.0,  1.0),
        float2( 1.0,  1.0)
    };
    
    VertexOut out;
    out.position = float4(vertices[vertexID], 0.0, 1.0);
    out.texCoord = (vertices[vertexID] + 1.0) * 0.5;
    return out;
}

fragment float4 robotFragmentShader(VertexOut in [[stage_in]],
                                   constant MapUniforms &uniforms [[buffer(0)]]) {
    // Convert fragment position to map coordinates
    float2 visibleSize = uniforms.visibleRegion.zw - uniforms.visibleRegion.xy + 1.0;
    float2 mapCoord = uniforms.visibleRegion.xy + in.texCoord * visibleSize;
    
    // Calculate distance from robot position
    float2 robotPos = uniforms.robotPosition;
    float distance = length(mapCoord - robotPos);
    
    // Robot radius in map cells
    float robotRadius = 1.5;
    
    // Anti-aliasing factor
    float aa = 0.1;
    
    // Draw robot body with smooth edges
    float robotAlpha = 1.0 - smoothstep(robotRadius - aa, robotRadius + aa, distance);
    if (robotAlpha > 0.01) {
        // Add inner circle for better visibility
        float innerAlpha = 1.0 - smoothstep(robotRadius * 0.7 - aa, robotRadius * 0.7 + aa, distance);
        float4 robotColor = mix(float4(0.0, 0.0, 1.0, robotAlpha),
                               float4(0.3, 0.3, 1.0, robotAlpha),
                               innerAlpha);
        return robotColor;
    }
    
    // Draw direction arrow
    float arrowLength = robotRadius * 2.0;
    float2 arrowDir = float2(sin(uniforms.robotHeading), -cos(uniforms.robotHeading));
    
    // Create arrow shape using SDF
    float2 toPoint = mapCoord - robotPos;
    float projLength = dot(toPoint, arrowDir);
    
    if (projLength > robotRadius * 0.5 && projLength < arrowLength) {
        float2 projPoint = robotPos + arrowDir * projLength;
        float perpDist = length(mapCoord - projPoint);
        
        // Arrow shaft
        float shaftWidth = robotRadius * 0.3;
        float shaftAlpha = 1.0 - smoothstep(shaftWidth - aa, shaftWidth + aa, perpDist);
        
        // Arrow head (triangle)
        if (projLength > arrowLength * 0.7) {
            float headWidth = robotRadius * 0.8 * (1.0 - (projLength - arrowLength * 0.7) / (arrowLength * 0.3));
            float headAlpha = 1.0 - smoothstep(headWidth - aa, headWidth + aa, perpDist);
            shaftAlpha = max(shaftAlpha, headAlpha);
        }
        
        if (shaftAlpha > 0.01) {
            return float4(1.0, 0.0, 0.0, shaftAlpha * 0.9);
        }
    }
    
    return float4(0.0, 0.0, 0.0, 0.0); // Transparent
}
