//
//  OccupancyComputeShaders.metal
//  MobileRobotics
//
//  Compute shaders for GPU-accelerated occupancy map updates
//

#include <metal_stdlib>
#include <metal_atomic>
using namespace metal;

// Obstacle point data
struct ObstaclePoint {
    float2 position; // Map coordinates
};

// Ray casting parameters
struct RayCastParams {
    float2 cameraPosition;  // Camera position in map coordinates
    uint obstacleCount;     // Number of obstacle points to process
    uint mapSize;          // Map dimensions (500)
};

// Atomic wrapper for confidence map updates
struct AtomicFloat {
    atomic<int> value;
};

// Convert float to fixed-point integer for atomic operations
inline int floatToFixed(float f) {
    return int(f * 10000.0f);
}

inline float fixedToFloat(int i) {
    return float(i) / 10000.0f;
}

// Bresenham line algorithm for ray marching
kernel void rayCastUpdate(device ObstaclePoint* obstacles [[buffer(0)]],
                         device AtomicFloat* confidenceMap [[buffer(1)]],
                         constant RayCastParams& params [[buffer(2)]],
                         uint tid [[thread_position_in_grid]]) {
    
    if (tid >= params.obstacleCount) return;
    
    // Get obstacle point
    float2 obstaclePos = obstacles[tid].position;
    float2 cameraPos = params.cameraPosition;
    
    // Bresenham's line algorithm
    int2 start = int2(cameraPos);
    int2 end = int2(obstaclePos);
    
    int dx = abs(end.x - start.x);
    int dy = abs(end.y - start.y);
    int sx = start.x < end.x ? 1 : -1;
    int sy = start.y < end.y ? 1 : -1;
    int err = dx - dy;
    
    int2 current = start;
    int steps = 0;
    int maxSteps = dx + dy; // Maximum possible steps
    
    while (steps < maxSteps) {
        // Check bounds
        if (current.x >= 0 && current.x < int(params.mapSize) &&
            current.y >= 0 && current.y < int(params.mapSize)) {
            
            uint index = current.y * params.mapSize + current.x;
            
            // Update confidence value
            if (all(current == end)) {
                // Obstacle point - increase confidence
                atomic_fetch_add_explicit(&confidenceMap[index].value,
                                        floatToFixed(0.15f),
                                        memory_order_relaxed);
            } else if (all(current != start)) {
                // Free space along ray - decrease confidence
                atomic_fetch_add_explicit(&confidenceMap[index].value,
                                        floatToFixed(-0.05f),
                                        memory_order_relaxed);
            }
        }
        
        // Check if we've reached the end
        if (all(current == end)) break;
        
        // Bresenham step
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            current.x += sx;
        }
        if (e2 < dx) {
            err += dx;
            current.y += sy;
        }
        
        steps++;
    }
}

// Convert confidence map to discrete occupancy values with hysteresis
kernel void confidenceToOccupancy(device const AtomicFloat* confidenceMap [[buffer(0)]],
                                 device uint8_t* occupancyMap [[buffer(1)]],
                                 device const uint8_t* previousMap [[buffer(2)]],
                                 constant uint& mapSize [[buffer(3)]],
                                 uint2 gid [[thread_position_in_grid]]) {
    
    if (gid.x >= mapSize || gid.y >= mapSize) return;
    
    uint index = gid.y * mapSize + gid.x;
    
    // Get confidence value
    float confidence = fixedToFloat(atomic_load_explicit(&confidenceMap[index].value,
                                                        memory_order_relaxed));
    
    // Clamp confidence to reasonable range
    confidence = clamp(confidence, -1.0f, 1.0f);
    
    // Get previous state
    uint8_t previousState = previousMap[index];
    
    // Apply hysteresis thresholds
    uint8_t newState;
    
    switch (previousState) {
        case 0: // Currently free
            if (confidence > 0.4f) {
                newState = 1; // Switch to occupied
            } else if (confidence < -0.2f) {
                newState = 0; // Stay free
            } else {
                newState = 2; // Unknown
            }
            break;
            
        case 1: // Currently occupied
            if (confidence < -0.4f) {
                newState = 0; // Switch to free
            } else if (confidence > 0.2f) {
                newState = 1; // Stay occupied
            } else {
                newState = 2; // Unknown
            }
            break;
            
        default: // Currently unknown
            if (confidence > 0.3f) {
                newState = 1; // Occupied
            } else if (confidence < -0.3f) {
                newState = 0; // Free
            } else {
                newState = 2; // Stay unknown
            }
            break;
    }
    
    occupancyMap[index] = newState;
}

// Clear confidence map for reset
kernel void clearConfidenceMap(device AtomicFloat* confidenceMap [[buffer(0)]],
                              constant uint& mapSize [[buffer(1)]],
                              uint2 gid [[thread_position_in_grid]]) {
    
    if (gid.x >= mapSize || gid.y >= mapSize) return;
    
    uint index = gid.y * mapSize + gid.x;
    atomic_store_explicit(&confidenceMap[index].value, 0, memory_order_relaxed);
}
