//
//  DataStructures.swift
//  571
//
//  Created by Kurt Gu on 6/3/25.
//

import Foundation
import ARKit
import simd

// MARK: - Sensor Data Structures

struct DepthData {
    let depthMap: CVPixelBuffer
    let confidenceMap: CVPixelBuffer?
    let timestamp: TimeInterval
}

struct PointCloudData {
    let points: [SIMD3<Float>]
    let colors: [SIMD4<Float>]?
    let timestamp: TimeInterval
}

struct CameraData {
    let imageBuffer: CVPixelBuffer
    let cameraIntrinsics: simd_float3x3
    let cameraTransform: simd_float4x4
    let timestamp: TimeInterval
}

struct PoseData {
    let position: SIMD3<Float>
    let orientation: simd_quatf
    let timestamp: TimeInterval
}

// MARK: - Occupancy Map Data Structure

struct OccupancyMapData {
    let map: [[Int]] // 500x500 array
    let robotPosition: (x: Int, y: Int)
    let mapResolution: Float // meters per cell
    let mapOrigin: SIMD3<Float> // world coordinates of map center
    let frontierTarget: (x: Int, y: Int)? // Current frontier exploration target
    
    init() {
        // Initialize with unknown map (value 2)
        self.map = Array(repeating: Array(repeating: 2, count: 500), count: 500)
        self.robotPosition = (x: 250, y: 250) // Center of map
        self.mapResolution = 0.05 // 5cm per cell
        self.mapOrigin = SIMD3<Float>(0, 0, 0) // Map center at world origin
        self.frontierTarget = nil // No frontier target initially
    }
    
    init(map: [[Int]], robotPosition: (x: Int, y: Int), mapResolution: Float = 0.05, mapOrigin: SIMD3<Float> = SIMD3<Float>(0, 0, 0), frontierTarget: (x: Int, y: Int)? = nil) {
        self.map = map
        self.robotPosition = robotPosition
        self.mapResolution = mapResolution
        self.mapOrigin = mapOrigin
        self.frontierTarget = frontierTarget
    }
}

// MARK: - Useful Extensions

extension SIMD4 {
    var xyz: SIMD3<Scalar> {
        SIMD3(x, y, z)
    }
}
