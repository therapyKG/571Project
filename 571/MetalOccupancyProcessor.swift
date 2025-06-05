//
//  MetalOccupancyProcessor.swift
//  571
//
//  Created by Kurt Gu on 6/4/25.
//
//  GPU-accelerated occupancy map processing using Metal compute shaders
//

import Foundation
import Metal
import simd

class MetalOccupancyProcessor {
    private let device: MTLDevice
    private let commandQueue: MTLCommandQueue
    
    // Compute pipeline states
    private var rayCastPipeline: MTLComputePipelineState?
    private var confidenceToOccupancyPipeline: MTLComputePipelineState?
    private var clearConfidencePipeline: MTLComputePipelineState?
    
    // Buffers
    private var confidenceMapBuffer: MTLBuffer?
    private var occupancyMapBuffer: MTLBuffer?
    private var previousMapBuffer: MTLBuffer?
    private var obstacleBuffer: MTLBuffer?
    
    // Map parameters
    private let mapSize: Int = 500
    private let maxObstacles: Int = 10000 // Maximum obstacles per frame
    
    // Thread group sizes
    private let threadgroupSize = MTLSize(width: 32, height: 1, depth: 1)
    private let threadgroupSize2D = MTLSize(width: 16, height: 16, depth: 1)
    
    init?() {
        guard let device = MTLCreateSystemDefaultDevice(),
              let commandQueue = device.makeCommandQueue() else {
            return nil
        }
        
        self.device = device
        self.commandQueue = commandQueue
        
        setupPipelines()
        setupBuffers()
    }
    
    private func setupPipelines() {
        guard let library = device.makeDefaultLibrary() else {
            print("Failed to create Metal library")
            return
        }
        
        // Ray cast pipeline
        if let rayCastFunction = library.makeFunction(name: "rayCastUpdate") {
            do {
                rayCastPipeline = try device.makeComputePipelineState(function: rayCastFunction)
            } catch {
                print("Failed to create ray cast pipeline: \(error)")
            }
        }
        
        // Confidence to occupancy pipeline
        if let confidenceFunction = library.makeFunction(name: "confidenceToOccupancy") {
            do {
                confidenceToOccupancyPipeline = try device.makeComputePipelineState(function: confidenceFunction)
            } catch {
                print("Failed to create confidence pipeline: \(error)")
            }
        }
        
        // Clear confidence pipeline
        if let clearFunction = library.makeFunction(name: "clearConfidenceMap") {
            do {
                clearConfidencePipeline = try device.makeComputePipelineState(function: clearFunction)
            } catch {
                print("Failed to create clear pipeline: \(error)")
            }
        }
    }
    
    private func setupBuffers() {
        let mapElements = mapSize * mapSize
        
        // Confidence map buffer (atomic integers)
        let confidenceSize = mapElements * MemoryLayout<Int32>.size
        confidenceMapBuffer = device.makeBuffer(length: confidenceSize, options: .storageModeShared)
        confidenceMapBuffer?.label = "Confidence Map"
        
        // Clear confidence map initially
        if let buffer = confidenceMapBuffer {
            let pointer = buffer.contents().assumingMemoryBound(to: Int32.self)
            for i in 0..<mapElements {
                pointer[i] = 0
            }
        }
        
        // Occupancy map buffers
        let occupancySize = mapElements * MemoryLayout<UInt8>.size
        occupancyMapBuffer = device.makeBuffer(length: occupancySize, options: .storageModeShared)
        occupancyMapBuffer?.label = "Occupancy Map"
        
        previousMapBuffer = device.makeBuffer(length: occupancySize, options: .storageModeShared)
        previousMapBuffer?.label = "Previous Map"
        
        // Initialize with unknown values
        if let occBuffer = occupancyMapBuffer,
           let prevBuffer = previousMapBuffer {
            let occPointer = occBuffer.contents().assumingMemoryBound(to: UInt8.self)
            let prevPointer = prevBuffer.contents().assumingMemoryBound(to: UInt8.self)
            for i in 0..<mapElements {
                occPointer[i] = 2  // Unknown
                prevPointer[i] = 2 // Unknown
            }
        }
        
        // Obstacle buffer
        let obstacleSize = maxObstacles * MemoryLayout<SIMD2<Float>>.size
        obstacleBuffer = device.makeBuffer(length: obstacleSize, options: .storageModeShared)
        obstacleBuffer?.label = "Obstacle Points"
    }
    
    // Process obstacle points and update occupancy map asynchronously
    func updateOccupancyMapAsync(obstaclePoints: [SIMD3<Float>],
                                cameraPosition: SIMD3<Float>,
                                mapOrigin: SIMD3<Float>,
                                mapResolution: Float,
                                currentMap: [[Int]],
                                completion: @escaping ([[Int]]?) -> Void) {
        
        guard let commandBuffer = commandQueue.makeCommandBuffer(),
              let rayCastPipeline = rayCastPipeline,
              let confidencePipeline = confidenceToOccupancyPipeline,
              let obstacleBuffer = obstacleBuffer,
              let confidenceBuffer = confidenceMapBuffer,
              let occupancyBuffer = occupancyMapBuffer,
              let previousBuffer = previousMapBuffer else {
            completion(nil)
            return
        }
        
        // Convert world coordinates to map coordinates
        let cameraMapPos = worldToMap(cameraPosition, mapOrigin: mapOrigin, mapResolution: mapResolution)
        guard cameraMapPos.x >= 0 && cameraMapPos.x < Float(mapSize) &&
              cameraMapPos.y >= 0 && cameraMapPos.y < Float(mapSize) else {
            completion(nil)
            return
        }
        
        // Prepare obstacle points in map coordinates
        let obstacleCount = min(obstaclePoints.count, maxObstacles)
        if obstacleCount == 0 { 
            completion(nil)
            return
        }
        
        let obstaclePointer = obstacleBuffer.contents().assumingMemoryBound(to: SIMD2<Float>.self)
        var validObstacles = 0
        
        for i in 0..<obstacleCount {
            let mapPos = worldToMap(obstaclePoints[i], mapOrigin: mapOrigin, mapResolution: mapResolution)
            if mapPos.x >= 0 && mapPos.x < Float(mapSize) &&
               mapPos.y >= 0 && mapPos.y < Float(mapSize) {
                obstaclePointer[validObstacles] = mapPos
                validObstacles += 1
            }
        }
        
        if validObstacles == 0 { 
            completion(nil)
            return
        }
        
        // Update previous map buffer
        updatePreviousMap(currentMap)
        
        // Ray casting parameters
        var params = RayCastParams(
            cameraPosition: cameraMapPos,
            obstacleCount: UInt32(validObstacles),
            mapSize: UInt32(mapSize)
        )
        
        // Encode ray casting
        if let encoder = commandBuffer.makeComputeCommandEncoder() {
            encoder.setComputePipelineState(rayCastPipeline)
            encoder.setBuffer(obstacleBuffer, offset: 0, index: 0)
            encoder.setBuffer(confidenceBuffer, offset: 0, index: 1)
            encoder.setBytes(&params, length: MemoryLayout<RayCastParams>.size, index: 2)
            
            let threadsPerGrid = MTLSize(width: validObstacles, height: 1, depth: 1)
            encoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadgroupSize)
            encoder.endEncoding()
        }
        
        // Encode confidence to occupancy conversion
        if let encoder = commandBuffer.makeComputeCommandEncoder() {
            encoder.setComputePipelineState(confidencePipeline)
            encoder.setBuffer(confidenceBuffer, offset: 0, index: 0)
            encoder.setBuffer(occupancyBuffer, offset: 0, index: 1)
            encoder.setBuffer(previousBuffer, offset: 0, index: 2)
            
            var size = UInt32(mapSize)
            encoder.setBytes(&size, length: MemoryLayout<UInt32>.size, index: 3)
            
            let threadsPerGrid = MTLSize(width: mapSize, height: mapSize, depth: 1)
            encoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadgroupSize2D)
            encoder.endEncoding()
        }
        
        // Add completion handler for asynchronous execution
        commandBuffer.addCompletedHandler { [weak self] _ in
            // Read back results on completion
            let result = self?.readOccupancyMap()
            completion(result)
        }
        
        // Commit without waiting
        commandBuffer.commit()
    }
    
    // Legacy synchronous method for backwards compatibility (deprecated)
    @available(*, deprecated, message: "Use updateOccupancyMapAsync instead for better performance")
    func updateOccupancyMap(obstaclePoints: [SIMD3<Float>],
                           cameraPosition: SIMD3<Float>,
                           mapOrigin: SIMD3<Float>,
                           mapResolution: Float,
                           currentMap: [[Int]]) -> [[Int]]? {
        var result: [[Int]]? = nil
        let semaphore = DispatchSemaphore(value: 0)
        
        updateOccupancyMapAsync(obstaclePoints: obstaclePoints,
                               cameraPosition: cameraPosition,
                               mapOrigin: mapOrigin,
                               mapResolution: mapResolution,
                               currentMap: currentMap) { map in
            result = map
            semaphore.signal()
        }
        
        semaphore.wait()
        return result
    }
    
    // Clear the confidence map
    func clearMap() {
        guard let commandBuffer = commandQueue.makeCommandBuffer(),
              let clearPipeline = clearConfidencePipeline,
              let confidenceBuffer = confidenceMapBuffer else {
            return
        }
        
        if let encoder = commandBuffer.makeComputeCommandEncoder() {
            encoder.setComputePipelineState(clearPipeline)
            encoder.setBuffer(confidenceBuffer, offset: 0, index: 0)
            
            var size = UInt32(mapSize)
            encoder.setBytes(&size, length: MemoryLayout<UInt32>.size, index: 1)
            
            let threadsPerGrid = MTLSize(width: mapSize, height: mapSize, depth: 1)
            encoder.dispatchThreads(threadsPerGrid, threadsPerThreadgroup: threadgroupSize2D)
            encoder.endEncoding()
        }
        
        commandBuffer.commit()
        commandBuffer.waitUntilCompleted()
        
        // Also clear occupancy maps
        if let occBuffer = occupancyMapBuffer,
           let prevBuffer = previousMapBuffer {
            let occPointer = occBuffer.contents().assumingMemoryBound(to: UInt8.self)
            let prevPointer = prevBuffer.contents().assumingMemoryBound(to: UInt8.self)
            for i in 0..<(mapSize * mapSize) {
                occPointer[i] = 2  // Unknown
                prevPointer[i] = 2 // Unknown
            }
        }
    }
    
    // Helper: Convert world coordinates to map coordinates
    private func worldToMap(_ worldPos: SIMD3<Float>, mapOrigin: SIMD3<Float>, mapResolution: Float) -> SIMD2<Float> {
        let relativeX = worldPos.x - mapOrigin.x
        let relativeZ = worldPos.z - mapOrigin.z
        
        let gridX = (relativeX / mapResolution) + 250.0
        let gridY = (relativeZ / mapResolution) + 250.0
        
        return SIMD2<Float>(gridX, gridY)
    }
    
    // Update previous map buffer
    private func updatePreviousMap(_ currentMap: [[Int]]) {
        guard let buffer = previousMapBuffer else { return }
        
        let pointer = buffer.contents().assumingMemoryBound(to: UInt8.self)
        for y in 0..<mapSize {
            for x in 0..<mapSize {
                pointer[y * mapSize + x] = UInt8(currentMap[y][x])
            }
        }
    }
    
    // Read occupancy map from GPU buffer
    private func readOccupancyMap() -> [[Int]]? {
        guard let buffer = occupancyMapBuffer else { return nil }
        
        let pointer = buffer.contents().assumingMemoryBound(to: UInt8.self)
        var map = Array(repeating: Array(repeating: 2, count: mapSize), count: mapSize)
        
        for y in 0..<mapSize {
            for x in 0..<mapSize {
                map[y][x] = Int(pointer[y * mapSize + x])
            }
        }
        
        return map
    }
}

// Supporting structures for Metal shaders
struct RayCastParams {
    let cameraPosition: SIMD2<Float>
    let obstacleCount: UInt32
    let mapSize: UInt32
}
