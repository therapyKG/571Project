//
//  RoboticsFramework.swift
//  MobileRobotics
//
//  Created by Kurt Gu on 5/5/25.
//

import Foundation
import SwiftUI
import RealityKit
import ARKit
import Combine
import Accelerate

// Main class that provides access to ARKit sensors and occupancy mapping
class RoboticsFramework: NSObject, ARSessionDelegate, ObservableObject {
    // Singleton instance for easy access
    static let shared = RoboticsFramework()
    
    // AR session and configuration
    private let session = ARSession()
    private var configuration: ARWorldTrackingConfiguration
    
    // Publishers for different data streams
    private let depthPublisher = PassthroughSubject<DepthData, Never>()
    private let pointCloudPublisher = PassthroughSubject<PointCloudData, Never>()
    private let cameraPublisher = PassthroughSubject<CameraData, Never>()
    private let posePublisher = PassthroughSubject<PoseData, Never>()
    
    // Public accessors for publishers
    var depthStream: AnyPublisher<DepthData, Never> {
        depthPublisher.eraseToAnyPublisher()
    }
    
    var pointCloudStream: AnyPublisher<PointCloudData, Never> {
        pointCloudPublisher.eraseToAnyPublisher()
    }
    
    var cameraStream: AnyPublisher<CameraData, Never> {
        cameraPublisher.eraseToAnyPublisher()
    }
    
    var poseStream: AnyPublisher<PoseData, Never> {
        posePublisher.eraseToAnyPublisher()
    }
    
    // Current data values
    @Published var currentDepthData: DepthData?
    @Published var currentPointCloud: PointCloudData?
    @Published var currentCameraData: CameraData?
    @Published var currentPose: PoseData?
    @Published var currentHeading: Float = 0.0 // Device heading in radians
    
    // Occupancy map data with persistence
    @Published var occupancyMapData = OccupancyMapData()
    private var persistentOccupancyMap: [[Float]] = Array(repeating: Array(repeating: 0.0, count: 500), count: 500) // Confidence values for each cell
    private let mapUpdateQueue = DispatchQueue(label: "com.robotics.mapupdate", qos: .userInitiated)
    
    // Framework status
    @Published var isRunning = false
    @Published var hasLiDAR = false
    @Published var hasUltrawide = false
    @Published var currentCameraType = "Unknown"
    @Published var statusMessage = "Not initialized"
    @Published var debugMode = 0 // 0: standard, 1: no scene reconstruction, 2: minimal debug
    @Published var pointsProcessed = 0 // For debugging
    @Published var mapUpdates = 0 // For debugging
    @Published var floorDetected = false // Floor plane detection status
    @Published var obstaclePointsFiltered = 0 // Points after height filtering
    
    // Map generation tracking (for race condition prevention during map clearing)
    private var privateMapGenerationID = 0 // Internal counter for background thread synchronization
    @Published var mapGenerationID = 0 // Published version for UI display
    
    // Frame processing throttling
    private var lastProcessedTime: TimeInterval = 0
    private let minFrameInterval: TimeInterval = 0.1 // Process at most 10 FPS
    private var frameProcessingQueue = DispatchQueue(label: "com.robotics.frameprocessing", qos: .userInitiated)
    
    // LiDAR processing parameters
    private let maxDepthRange: Float = 5.0 // Maximum depth to consider (meters)
    private let minDepthRange: Float = 0.1 // Minimum depth to consider (meters)
    private let depthSamplingRate = 4 // Process every Nth pixel for performance
    
    // Robot navigation parameters
    private let robotHeight: Float = 0.3 // Robot height (meters)
    private let minObstacleHeight: Float = 0.1 // Minimum height above floor to consider obstacle
    private let maxObstacleHeight: Float = 1.8 // Maximum height to consider (ignore ceiling)
    private let floorDetectionTolerance: Float = 0.1 // Tolerance for floor plane detection
    
    // Floor plane tracking
    private var detectedFloorPlane: ARPlaneAnchor?
    private var floorY: Float = 0.0 // Y coordinate of detected floor
    
    // Initialize the framework
    private override init() {
        // Create configuration
        configuration = ARWorldTrackingConfiguration()
        
        super.init()
        
        // Check LiDAR availability
        hasLiDAR = ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .smoothedSceneDepth])
        
        // Check ultrawide camera availability
        hasUltrawide = detectUltrawideSupport()
        
        // Set up session
        session.delegate = self
        
        // Initialize persistent occupancy map with unknown values
        initializeEmptyOccupancyMap()
    }
    
    // MARK: - Initialization and Configuration
    
    // Initialize empty occupancy map
    private func initializeEmptyOccupancyMap() {
        // Increment generation ID to invalidate any pending updates
        privateMapGenerationID += 1
        
        // Initialize persistent map with neutral confidence values
        persistentOccupancyMap = Array(repeating: Array(repeating: 0.0, count: 500), count: 500)
        
        // Start with unknown occupancy map
        occupancyMapData = OccupancyMapData()
        
        // Reset counters and floor detection
        pointsProcessed = 0
        mapUpdates = 0
        obstaclePointsFiltered = 0
        floorDetected = false
        detectedFloorPlane = nil
        floorY = 0.0
        mapGenerationID = privateMapGenerationID
        
        print("🗺️ Map cleared - Generation ID: \(privateMapGenerationID)")
    }
    
    // Detect if ultrawide camera is supported
    private func detectUltrawideSupport() -> Bool {
        let supportedFormats = ARWorldTrackingConfiguration.supportedVideoFormats
        return supportedFormats.contains { format in
            // Ultrawide cameras typically have field of view > 100 degrees
            // and specific capture device types
            if #available(iOS 13.0, *) {
                return format.captureDeviceType == .builtInUltraWideCamera
            }
            return false
        }
    }
    
    // Find the best ultrawide video format
    private func selectUltrawideVideoFormat() -> ARConfiguration.VideoFormat? {
        let supportedFormats = ARWorldTrackingConfiguration.supportedVideoFormats
        
        // Look for ultrawide camera formats
        let ultrawideFormats = supportedFormats.filter { format in
            if #available(iOS 13.0, *) {
                return format.captureDeviceType == .builtInUltraWideCamera
            }
            return false
        }
        
        // Prefer higher resolution formats
        return ultrawideFormats.max { format1, format2 in
            let resolution1 = format1.imageResolution.width * format1.imageResolution.height
            let resolution2 = format2.imageResolution.width * format2.imageResolution.height
            return resolution1 < resolution2
        }
    }
    
    // Create and configure a fresh AR configuration
    private func createConfiguration() -> ARWorldTrackingConfiguration {
        let config = ARWorldTrackingConfiguration()
        
        // Set preferred frame rate to reduce processing load
        if #available(iOS 13.0, *) {
            config.videoHDRAllowed = false // Disable HDR to reduce processing
        }
        
        // Try to use ultrawide camera if available
        if hasUltrawide, let ultrawideFormat = selectUltrawideVideoFormat() {
            config.videoFormat = ultrawideFormat
            currentCameraType = "Ultrawide"
        } else {
            currentCameraType = "Standard Wide"
        }
        
        // Configure the session for depth data if available
        if hasLiDAR {
            config.frameSemantics = [.sceneDepth, .smoothedSceneDepth]
            
            // Check if scene reconstruction is supported and conditionally enable based on debug mode
            if debugMode != 1 && ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
                config.sceneReconstruction = .mesh
                print("✅ Scene reconstruction enabled")
            } else {
                print("❌ Scene reconstruction disabled (debug mode \(debugMode))")
            }
        }
        
        // Enable plane detection
        config.planeDetection = [.horizontal, .vertical]
        
        // Optimize for performance
        config.isAutoFocusEnabled = true
        config.environmentTexturing = .none // Disable environment texturing to save resources
        
        // Log configuration details
        print("📱 Device: \(UIDevice.current.model)")
        print("🔧 LiDAR: \(hasLiDAR)")
        print("📹 Camera: \(currentCameraType)")
        print("🎯 Frame semantics: \(config.frameSemantics)")
        print("🔍 Scene reconstruction: \(config.sceneReconstruction)")
        
        return config
    }
    
    // MARK: - Session Control
    
    // Start the AR session with the given options
    func start() {
        // Check actual session state, not just our flag
        guard session.currentFrame == nil || !isRunning else {
            print("⚠️ Session already running, skipping start")
            return
        }
        
        // Create fresh configuration
        configuration = createConfiguration()
        
        // Update status message
        if hasLiDAR {
            statusMessage = "LiDAR mapping enabled, using \(currentCameraType) camera"
        } else {
            statusMessage = "Using \(currentCameraType) camera (no LiDAR - mapping disabled)"
        }
        
        // Run the session with proper options
        session.run(configuration, options: [])
        isRunning = true
        print("✅ AR session started")
    }
    
    // Stop the AR session
    func stop() {
        guard isRunning else {
            print("⚠️ Session not running, skipping stop")
            return
        }
        
        session.pause()
        isRunning = false
        statusMessage = "Session paused"
        print("⏸️ AR session stopped")
        
        // Clear current data
        clearCurrentData()
    }
    
    // Reset the AR session completely
    func reset() {
        print("🔄 Resetting AR session...")
        
        // Clear current data first
        clearCurrentData()
        
        var newGenerationID = 0
        
        // Clear map synchronously on the map update queue
        mapUpdateQueue.sync { [weak self] in
            guard let self = self else { return }
            
            // Increment generation ID to invalidate any pending updates
            self.privateMapGenerationID += 1
            newGenerationID = self.privateMapGenerationID
            
            // Clear persistent map
            self.persistentOccupancyMap = Array(repeating: Array(repeating: 0.0, count: 500), count: 500)
            
            print("🗺️ Map reset - Generation ID: \(self.privateMapGenerationID)")
        }
        
        // Reset UI state on main thread
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            
            self.occupancyMapData = OccupancyMapData()
            self.pointsProcessed = 0
            self.mapUpdates = 0
            self.obstaclePointsFiltered = 0
            self.floorDetected = false
            self.detectedFloorPlane = nil
            self.floorY = 0.0
            self.mapGenerationID = newGenerationID
        }
        
        // Stop the session properly if it's running
        if isRunning {
            session.pause()
            isRunning = false
        }
        
        // Wait a moment for the session to fully stop
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.2) {
            // Create fresh configuration
            self.configuration = self.createConfiguration()
            
            // Update status message
            self.statusMessage = "Resetting session..."
            
            // Run session with reset options
            self.session.run(self.configuration, options: [.resetTracking, .removeExistingAnchors])
            self.isRunning = true
            print("✅ AR session reset complete")
            
            // Update status after reset
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.3) {
                if self.hasLiDAR {
                    self.statusMessage = "LiDAR mapping enabled, using \(self.currentCameraType) camera (reset)"
                } else {
                    self.statusMessage = "Using \(self.currentCameraType) camera (no LiDAR - mapping disabled) (reset)"
                }
            }
        }
    }
    
    // Clear all current sensor data
    private func clearCurrentData() {
        DispatchQueue.main.async {
            self.currentDepthData = nil
            self.currentPointCloud = nil
            self.currentCameraData = nil
            self.currentPose = nil
            self.currentHeading = 0.0
        }
        
        // Reset frame processing time
        lastProcessedTime = 0
    }
    
    // Toggle debug mode to help diagnose rendering differences
    func toggleDebugMode() {
        debugMode = (debugMode + 1) % 3
        
        switch debugMode {
        case 0:
            statusMessage = "Debug: Standard mode"
        case 1:
            statusMessage = "Debug: No scene reconstruction"
        case 2:
            statusMessage = "Debug: Minimal debug options"
        default:
            break
        }
        
        // Reset to apply new debug settings
        reset()
    }
    
    // Clear occupancy map
    func clearOccupancyMap() {
        print("🧹 Clearing occupancy map...")
        
        var newGenerationID = 0
        
        // Run on map update queue to ensure proper synchronization
        mapUpdateQueue.sync { [weak self] in
            guard let self = self else { return }
            
            // Increment generation ID to invalidate any pending updates
            self.privateMapGenerationID += 1
            newGenerationID = self.privateMapGenerationID
            
            // Clear persistent map
            self.persistentOccupancyMap = Array(repeating: Array(repeating: 0.0, count: 500), count: 500)
            
            print("🗺️ Map data cleared - Generation ID: \(self.privateMapGenerationID)")
        }
        
        // Update UI on main thread
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            
            // Reset all data
            self.occupancyMapData = OccupancyMapData()
            self.pointsProcessed = 0
            self.mapUpdates = 0
            self.obstaclePointsFiltered = 0
            self.floorDetected = false
            self.detectedFloorPlane = nil
            self.floorY = 0.0
            self.currentHeading = 0.0
            self.mapGenerationID = newGenerationID // Update published property
            
            self.statusMessage = "Occupancy map cleared - Generation ID: \(newGenerationID)"
            
            print("✅ Map clear complete")
        }
    }
    
    func getSession() -> ARSession {
        return session
    }
    
    // MARK: - Coordinate Transformations
    
    // Convert world coordinates to map grid coordinates
    private func worldToMap(_ worldPos: SIMD3<Float>) -> (x: Int, y: Int)? {
        let mapData = occupancyMapData
        
        // Calculate relative position from map origin
        let relativeX = worldPos.x - mapData.mapOrigin.x
        let relativeZ = worldPos.z - mapData.mapOrigin.z
        
        // Convert to grid coordinates (using X and Z, Y is height)
        let gridX = Int((relativeX / mapData.mapResolution) + 250) // 250 is center
        let gridY = Int((relativeZ / mapData.mapResolution) + 250)
        
        // Check bounds
        if gridX >= 0 && gridX < 500 && gridY >= 0 && gridY < 500 {
            return (x: gridX, y: gridY)
        }
        return nil
    }
    
    // Extract heading direction in XZ plane (top-down view)
    private func extractHeading(from cameraTransform: simd_float4x4) -> Float {
        // Get the camera's forward direction vector (negative Z axis in camera space)
        // In ARKit, the camera looks down the negative Z axis, but we want the actual forward direction
        let forwardVector = SIMD3<Float>(
            cameraTransform[2][0],   // Z column X component (flip sign)
            cameraTransform[2][1],   // Z column Y component (flip sign)
            cameraTransform[2][2]    // Z column Z component (flip sign)
        )
        
        // Project the forward vector onto the XZ plane (ignore Y/height component)
        let forwardXZ = SIMD2<Float>(forwardVector.x, forwardVector.z)
        
        // Calculate heading angle in XZ plane and negate to match rotation direction
        // atan2(x, z) gives angle from positive Z axis (north)
        // Negate to make clockwise phone rotation = clockwise arrow rotation
        let heading = -atan2(forwardXZ.x, forwardXZ.y)
        
        return heading
    }
    
    // MARK: - LiDAR Processing
    
    // Convert depth map to filtered 3D obstacle points (robot navigation height only)
    private func depthMapToObstaclePoints(_ depthData: ARDepthData, cameraTransform: simd_float4x4, intrinsics: simd_float3x3) -> [SIMD3<Float>] {
        let depthMap = depthData.depthMap
        
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let baseAddress = CVPixelBufferGetBaseAddress(depthMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        
        guard let depthBuffer = baseAddress?.assumingMemoryBound(to: Float32.self) else {
            return []
        }
        
        var obstaclePoints: [SIMD3<Float>] = []
        let estimatedCapacity = Int((width * height) / (depthSamplingRate * depthSamplingRate))
        obstaclePoints.reserveCapacity(estimatedCapacity)
        
        // Use detected floor level, or fall back to camera position if no floor detected
        let referenceFloorY = floorDetected ? floorY : (cameraTransform.columns.3.y - robotHeight)
        
        // Sample every depthSamplingRate pixels for performance
        for y in stride(from: 0, to: height, by: depthSamplingRate) {
            for x in stride(from: 0, to: width, by: depthSamplingRate) {
                let pixelIndex = y * (bytesPerRow / MemoryLayout<Float32>.size) + x
                let depth = depthBuffer[pixelIndex]
                
                // Skip invalid or out-of-range depths
                guard depth > minDepthRange && depth < maxDepthRange && depth.isFinite else {
                    continue
                }
                
                // Convert pixel coordinates to normalized camera coordinates
                let fx = intrinsics[0][0]
                let fy = intrinsics[1][1]
                let cx = intrinsics[2][0]
                let cy = intrinsics[2][1]
                
                // Convert to camera space
                let cameraX = (Float(x) - cx) * depth / fx
                let cameraY = (Float(y) - cy) * depth / fy
                let cameraZ = -depth // ARKit uses negative Z for forward
                
                let cameraPoint = SIMD4<Float>(cameraX, cameraY, cameraZ, 1.0)
                
                // Transform to world space
                let worldPoint = cameraTransform * cameraPoint
                let worldPosition = worldPoint.xyz
                
                // Filter by height relative to floor - only keep points that would block robot navigation
                let heightAboveFloor = worldPosition.y - referenceFloorY
                
                if heightAboveFloor >= minObstacleHeight && heightAboveFloor <= maxObstacleHeight {
                    // This point represents a potential obstacle for robot navigation
                    obstaclePoints.append(worldPosition)
                }
            }
        }
        
        return obstaclePoints
    }
    
    // Update occupancy map with filtered obstacle points using ray tracing
    private func updateOccupancyMapWithObstaclePoints(_ obstaclePoints: [SIMD3<Float>], cameraPosition: SIMD3<Float>) {
        // Capture current generation ID to check for stale updates
        let currentGenerationID = privateMapGenerationID
        
        mapUpdateQueue.async { [weak self] in
            guard let self = self else { return }
            
            // Check if this update is still valid (map hasn't been cleared)
            guard currentGenerationID == self.privateMapGenerationID else {
                print("⚠️ Ignoring stale map update (Generation: \(currentGenerationID) vs Current: \(self.privateMapGenerationID))")
                return
            }
            
            var mapCopy = self.persistentOccupancyMap
            let mapData = self.occupancyMapData
            
            // Convert camera position to map coordinates
            guard let cameraMapPos = self.worldToMap(cameraPosition) else { return }
            
            // Keep track of cells we've already processed in this frame to avoid double-updates
            var processedCells: Set<String> = Set()
            
            for obstaclePoint in obstaclePoints {
                // Convert world point to map coordinates
                guard let obstacleMapPos = self.worldToMap(obstaclePoint) else { continue }
                
                // Perform ray tracing from camera to obstacle
                let rayPoints = self.bresenhamLine(
                    from: (x: cameraMapPos.x, y: cameraMapPos.y),
                    to: (x: obstacleMapPos.x, y: obstacleMapPos.y)
                )
                
                // Skip very short rays (likely noise)
                guard rayPoints.count > 2 else { continue }
                
                // Mark cells along the ray as free space (except the last one)
                for i in 0..<(rayPoints.count - 1) {
                    let point = rayPoints[i]
                    let cellKey = "\(point.x),\(point.y)"
                    
                    if point.x >= 0 && point.x < 500 && point.y >= 0 && point.y < 500 && !processedCells.contains(cellKey) {
                        // More conservative free space update to prevent oscillation
                        mapCopy[point.y][point.x] = max(-1.0, mapCopy[point.y][point.x] - 0.05)
                        processedCells.insert(cellKey)
                    }
                }
                
                // Mark the obstacle point as occupied
                if rayPoints.count > 0 {
                    let obstacleMapPoint = rayPoints.last!
                    let cellKey = "\(obstacleMapPoint.x),\(obstacleMapPoint.y)"
                    
                    if obstacleMapPoint.x >= 0 && obstacleMapPoint.x < 500 && obstacleMapPoint.y >= 0 && obstacleMapPoint.y < 500 {
                        // Stronger obstacle confidence to overcome free space rays
                        let currentConfidence = mapCopy[obstacleMapPoint.y][obstacleMapPoint.x]
                        mapCopy[obstacleMapPoint.y][obstacleMapPoint.x] = min(1.0, currentConfidence + 0.15)
                        processedCells.insert(cellKey)
                    }
                }
            }
            
            // Double-check generation ID before applying changes
            guard currentGenerationID == self.privateMapGenerationID else {
                print("⚠️ Map cleared during processing, discarding update")
                return
            }
            
            // Convert confidence map to discrete occupancy values with hysteresis
            var discreteMap = Array(repeating: Array(repeating: 2, count: 500), count: 500)
            for y in 0..<500 {
                for x in 0..<500 {
                    let confidence = mapCopy[y][x]
                    let currentState = mapData.map[y][x]
                    
                    // Use hysteresis to prevent oscillation
                    switch currentState {
                    case 0: // Currently free
                        if confidence > 0.4 {
                            discreteMap[y][x] = 1 // Switch to occupied only with higher threshold
                        } else if confidence < -0.2 {
                            discreteMap[y][x] = 0 // Stay free
                        } else {
                            discreteMap[y][x] = 2 // Unknown
                        }
                    case 1: // Currently occupied
                        if confidence < -0.4 {
                            discreteMap[y][x] = 0 // Switch to free only with lower threshold
                        } else if confidence > 0.2 {
                            discreteMap[y][x] = 1 // Stay occupied
                        } else {
                            discreteMap[y][x] = 2 // Unknown
                        }
                    default: // Currently unknown
                        if confidence > 0.3 {
                            discreteMap[y][x] = 1 // Occupied
                        } else if confidence < -0.3 {
                            discreteMap[y][x] = 0 // Free
                        } else {
                            discreteMap[y][x] = 2 // Stay unknown
                        }
                    }
                }
            }
            
            // Update persistent map and published data
            self.persistentOccupancyMap = mapCopy
            
            DispatchQueue.main.async {
                // Final check before updating UI
                guard currentGenerationID == self.privateMapGenerationID else {
                    print("⚠️ Map cleared before UI update, skipping")
                    return
                }
                
                self.occupancyMapData = OccupancyMapData(
                    map: discreteMap,
                    robotPosition: mapData.robotPosition,
                    mapResolution: mapData.mapResolution,
                    mapOrigin: mapData.mapOrigin
                )
                self.pointsProcessed += obstaclePoints.count
                self.obstaclePointsFiltered = obstaclePoints.count
                self.mapUpdates += 1
            }
        }
    }
    
    // Bresenham's line algorithm for ray tracing
    private func bresenhamLine(from start: (x: Int, y: Int), to end: (x: Int, y: Int)) -> [(x: Int, y: Int)] {
        var points: [(x: Int, y: Int)] = []
        
        let dx = abs(end.x - start.x)
        let dy = abs(end.y - start.y)
        let sx = start.x < end.x ? 1 : -1
        let sy = start.y < end.y ? 1 : -1
        var err = dx - dy
        
        var x = start.x
        var y = start.y
        
        while true {
            points.append((x: x, y: y))
            
            if x == end.x && y == end.y { break }
            
            let e2 = 2 * err
            if e2 > -dy {
                err -= dy
                x += sx
            }
            if e2 < dx {
                err += dx
                y += sy
            }
        }
        
        return points
    }
    
    // MARK: - ARSessionDelegate methods
    
    func session(_ session: ARSession, didAdd anchors: [ARAnchor]) {
        for anchor in anchors {
            if let planeAnchor = anchor as? ARPlaneAnchor,
               planeAnchor.alignment == .horizontal {
                
                // Check if this is likely a floor plane (below camera)
                let planeY = planeAnchor.transform.columns.3.y
                let cameraY = session.currentFrame?.camera.transform.columns.3.y ?? 0
                
                // Floor should be below the camera
                if planeY < cameraY && (!floorDetected || planeY < floorY) {
                    detectedFloorPlane = planeAnchor
                    floorY = planeY
                    
                    DispatchQueue.main.async {
                        self.floorDetected = true
                        print("✅ Floor detected at Y: \(self.floorY)")
                    }
                }
            }
        }
    }
    
    func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        for anchor in anchors {
            if let planeAnchor = anchor as? ARPlaneAnchor,
               planeAnchor.alignment == .horizontal,
               planeAnchor.identifier == detectedFloorPlane?.identifier {
                
                // Update floor plane
                detectedFloorPlane = planeAnchor
                floorY = planeAnchor.transform.columns.3.y
                //print("🔄 Floor updated at Y: \(floorY)")
            }
        }
    }
    
    func session(_ session: ARSession, didRemove anchors: [ARAnchor]) {
        for anchor in anchors {
            if let planeAnchor = anchor as? ARPlaneAnchor,
               planeAnchor.identifier == detectedFloorPlane?.identifier {
                
                DispatchQueue.main.async {
                    self.floorDetected = false
                    self.detectedFloorPlane = nil
                    print("❌ Floor plane lost")
                }
            }
        }
    }
    
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // Throttle frame processing to prevent retention
        let currentTime = frame.timestamp
        guard currentTime - lastProcessedTime >= minFrameInterval else {
            return // Skip this frame
        }
        lastProcessedTime = currentTime
        
        // Process on background queue to avoid blocking
        frameProcessingQueue.async { [weak self] in
            guard let self = self else { return }
            
            // Check tracking quality and provide feedback
            let trackingState = frame.camera.trackingState
            
            switch trackingState {
            case .normal:
                // Only update status if it's currently showing tracking issues
                if self.statusMessage.contains("tracking") {
                    DispatchQueue.main.async {
                        if self.hasLiDAR {
                            self.statusMessage = "LiDAR mapping enabled, using \(self.currentCameraType) camera"
                        } else {
                            self.statusMessage = "Using \(self.currentCameraType) camera (no LiDAR - mapping disabled)"
                        }
                    }
                }
            case .limited(let reason):
                DispatchQueue.main.async {
                    switch reason {
                    case .initializing:
                        self.statusMessage = "Initializing tracking... (move device slowly)"
                    case .insufficientFeatures:
                        self.statusMessage = "Tracking limited - point at textured surfaces"
                    case .excessiveMotion:
                        self.statusMessage = "Tracking limited - move device slower"
                    case .relocalizing:
                        self.statusMessage = "Relocalizing tracking..."
                    @unknown default:
                        self.statusMessage = "Tracking limited"
                    }
                }
            case .notAvailable:
                DispatchQueue.main.async {
                    self.statusMessage = "Tracking not available"
                }
            }
            
            // Process camera image (extract data but don't retain frame)
            let cameraData = CameraData(
                imageBuffer: frame.capturedImage,
                cameraIntrinsics: frame.camera.intrinsics,
                cameraTransform: frame.camera.transform,
                timestamp: frame.timestamp
            )
            
            // Process device pose
            let pose = PoseData(
                position: frame.camera.transform.columns.3.xyz,
                orientation: simd_quatf(frame.camera.transform),
                timestamp: frame.timestamp
            )
            
            // Extract heading from camera transform matrix (direction in XZ plane)
            let heading = self.extractHeading(from: frame.camera.transform)
            
            // Update UI on main thread with batched updates
            DispatchQueue.main.async {
                self.currentCameraData = cameraData
                self.currentPose = pose
                self.currentHeading = heading
                
                // Update robot position on occupancy map based on pose
                if let robotMapPos = self.worldToMap(pose.position) {
                    self.occupancyMapData = OccupancyMapData(
                        map: self.occupancyMapData.map,
                        robotPosition: robotMapPos,
                        mapResolution: self.occupancyMapData.mapResolution,
                        mapOrigin: self.occupancyMapData.mapOrigin
                    )
                }
            }
            
            // Send to publishers (only for subscribers that need real-time data)
            self.cameraPublisher.send(cameraData)
            self.posePublisher.send(pose)
            
            // Process depth data if available and update occupancy map
            if let sceneDepth = frame.sceneDepth ?? frame.smoothedSceneDepth {
                let depthData = DepthData(
                    depthMap: sceneDepth.depthMap,
                    confidenceMap: sceneDepth.confidenceMap,
                    timestamp: frame.timestamp
                )
                
                DispatchQueue.main.async {
                    self.currentDepthData = depthData
                }
                self.depthPublisher.send(depthData)
                
                // Process depth data for occupancy mapping
                if self.hasLiDAR && trackingState == .normal {
                    let obstaclePoints = self.depthMapToObstaclePoints(
                        sceneDepth,
                        cameraTransform: frame.camera.transform,
                        intrinsics: frame.camera.intrinsics
                    )
                    
                    if !obstaclePoints.isEmpty {
                        self.updateOccupancyMapWithObstaclePoints(obstaclePoints, cameraPosition: pose.position)
                    }
                    
                    // Create point cloud for visualization (using filtered obstacle points)
                    let pointCloud = PointCloudData(
                        points: Array(obstaclePoints.prefix(1000)), // Limit for performance
                        colors: obstaclePoints.prefix(1000).map { _ in SIMD4<Float>(1, 0, 0, 1) }, // Red for obstacles
                        timestamp: frame.timestamp
                    )
                    
                    DispatchQueue.main.async {
                        self.currentPointCloud = pointCloud
                    }
                    self.pointCloudPublisher.send(pointCloud)
                }
            }
        }
    }
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        DispatchQueue.main.async {
            self.statusMessage = "Session failed: \(error.localizedDescription)"
            self.isRunning = false
        }
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        DispatchQueue.main.async {
            self.statusMessage = "Session interrupted"
        }
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        DispatchQueue.main.async {
            self.statusMessage = "Session resumed"
            // Don't automatically restart - let user control this
            self.isRunning = false
        }
    }
}
