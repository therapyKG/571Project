//
//  RoboticsFramework.swift (Simplified with Lightweight Localization)
//  MobileRobotics
//
//  Created by Kurt Gu on 5/5/25.
//  Updated with lightweight ARKit-native localization
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
    
    // Lightweight localization system (replaces complex enhanced system)
    @Published var localizationManager = LightweightLocalizationManager()
    
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
    
    // Current data values (simplified)
    @Published var currentDepthData: DepthData?
    @Published var currentPointCloud: PointCloudData?
    @Published var currentCameraData: CameraData?
    @Published var currentPose: PoseData?
    @Published var currentHeading: Float = 0.0 // Device heading in radians
    
    // Occupancy map data with persistence
    @Published var occupancyMapData = OccupancyMapData()
    private var persistentOccupancyMap: [[Float]] = Array(repeating: Array(repeating: 0.0, count: 500), count: 500)
    private let mapUpdateQueue = DispatchQueue(label: "com.robotics.mapupdate", qos: .userInitiated)
    
    // Metal GPU processor for occupancy mapping
    @Published var metalOccupancyProcessor: MetalOccupancyProcessor?
    
    // Framework status
    @Published var isRunning = false
    @Published var hasLiDAR = false
    @Published var hasUltrawide = false
    @Published var currentCameraType = "Unknown"
    @Published var statusMessage = "Not initialized"
    @Published var debugMode = 1
    @Published var pointsProcessed = 0
    @Published var mapUpdates = 0
    @Published var floorDetected = false
    @Published var obstaclePointsFiltered = 0
    
    // Simplified tracking status (from lightweight system)
    @Published var isTrackingReliable = false
    @Published var trackingQuality = "Unknown"
    
    // Map generation tracking
    private var privateMapGenerationID = 0
    @Published var mapGenerationID = 0
    
    // Simplified frame processing (much faster)
    private var lastProcessedTime: TimeInterval = 0
    private let minFrameInterval: TimeInterval = 0.1
    private var frameProcessingQueue = DispatchQueue(label: "com.robotics.frameprocessing", qos: .userInitiated)
    
    // LiDAR processing parameters (simplified)
    private let maxDepthRange: Float = 3.0 // Reduced range for speed
    private let minDepthRange: Float = 0.2
    private let depthSamplingRate = 8 // More aggressive sampling for speed
    
    // Robot navigation parameters
    private let robotHeight: Float = 1
    private let minObstacleHeight: Float = 0.1
    private let maxObstacleHeight: Float = 1.8
    
    // Floor plane tracking (simplified)
    private var detectedFloorPlane: ARPlaneAnchor?
    private var floorY: Float = 0.0
    
    // Combine cancellables for localization system
    private var cancellables = Set<AnyCancellable>()
    
    // Initialize the framework
    private override init() {
        configuration = ARWorldTrackingConfiguration()
        
        super.init()
        
        // Check device capabilities
        hasLiDAR = ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .smoothedSceneDepth])
        hasUltrawide = detectUltrawideSupport()
        
        // Set up session
        session.delegate = self
        
        // Setup lightweight localization system
        setupLightweightLocalization()
        
        // Initialize persistent occupancy map
        initializeEmptyOccupancyMap()
    }
    
    // MARK: - Lightweight Localization Setup
    
    private func setupLightweightLocalization() {
        // Subscribe to lightweight localization manager updates
        localizationManager.$currentRobotPosition
            .sink { [weak self] position in
                self?.updateOccupancyMapRobotPosition(position)
            }
            .store(in: &cancellables)
        
        localizationManager.$currentHeading
            .sink { [weak self] heading in
                DispatchQueue.main.async {
                    self?.currentHeading = heading
                }
            }
            .store(in: &cancellables)
        
        localizationManager.$trackingQuality
            .sink { [weak self] quality in
                DispatchQueue.main.async {
                    self?.trackingQuality = quality
                }
            }
            .store(in: &cancellables)
        
        localizationManager.$isTrackingReliable
            .sink { [weak self] isReliable in
                DispatchQueue.main.async {
                    self?.isTrackingReliable = isReliable
                }
            }
            .store(in: &cancellables)
    }
    
    private func updateOccupancyMapRobotPosition(_ position: (x: Int, y: Int)) {
        // Update frontier target for new robot position
        let newFrontierTarget = updateFrontierTarget(
            robotPos: position,
            map: occupancyMapData.map,
            currentTarget: occupancyMapData.frontierTarget
        )
        
        // Update occupancy map with new robot position and frontier target
        occupancyMapData = OccupancyMapData(
            map: occupancyMapData.map,
            robotPosition: position,
            mapResolution: occupancyMapData.mapResolution,
            mapOrigin: SIMD3<Float>(0, 0, 0), // Simplified origin
            frontierTarget: newFrontierTarget
        )
    }
    
    // MARK: - Initialization (Simplified)
    
    private func initializeEmptyOccupancyMap() {
        privateMapGenerationID += 1
        persistentOccupancyMap = Array(repeating: Array(repeating: 0.0, count: 500), count: 500)
        
        occupancyMapData = OccupancyMapData()
        
        // Reset counters
        pointsProcessed = 0
        mapUpdates = 0
        obstaclePointsFiltered = 0
        floorDetected = false
        detectedFloorPlane = nil
        floorY = 0.0
        mapGenerationID = privateMapGenerationID
        
        print("🗺️ Map cleared - Generation ID: \(privateMapGenerationID)")
    }
    
    private func detectUltrawideSupport() -> Bool {
        let supportedFormats = ARWorldTrackingConfiguration.supportedVideoFormats
        return supportedFormats.contains { format in
            if #available(iOS 13.0, *) {
                return format.captureDeviceType == .builtInUltraWideCamera
            }
            return false
        }
    }
    
    private func createConfiguration() -> ARWorldTrackingConfiguration {
        let config = ARWorldTrackingConfiguration()
        
        if #available(iOS 13.0, *) {
            config.videoHDRAllowed = false
        }
        
        if hasUltrawide {
            // Try to use ultrawide for better tracking
            let supportedFormats = ARWorldTrackingConfiguration.supportedVideoFormats
            let ultrawideFormats = supportedFormats.filter { format in
                if #available(iOS 13.0, *) {
                    return format.captureDeviceType == .builtInUltraWideCamera
                }
                return false
            }
            
            if let bestFormat = ultrawideFormats.max(by: { f1, f2 in
                let res1 = f1.imageResolution.width * f1.imageResolution.height
                let res2 = f2.imageResolution.width * f2.imageResolution.height
                return res1 < res2
            }) {
                config.videoFormat = bestFormat
                currentCameraType = "Ultrawide"
            }
        } else {
            currentCameraType = "Standard Wide"
        }
        
        if hasLiDAR {
            config.frameSemantics = [.sceneDepth, .smoothedSceneDepth]
            
            // Only enable scene reconstruction in debug mode 0
            if debugMode == 0 && ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
                config.sceneReconstruction = .mesh
                print("✅ Scene reconstruction enabled")
            }
        }
        
        config.planeDetection = [.horizontal, .vertical]
        config.isAutoFocusEnabled = true
        config.environmentTexturing = .none
        
        return config
    }
    
    // MARK: - Session Control (Simplified)
    
    func start() {
        guard !isRunning else { return }
        
        configuration = createConfiguration()
        statusMessage = hasLiDAR ? "Starting ARKit with LiDAR..." : "Starting ARKit camera tracking..."
        
        session.run(configuration, options: [])
        isRunning = true
        print("✅ AR session started with lightweight localization")
    }
    
    func stop() {
        guard isRunning else { return }
        
        session.pause()
        isRunning = false
        statusMessage = "Session paused"
        print("⏸️ AR session stopped")
        
        clearCurrentData()
    }
    
    func reset() {
        print("🔄 Resetting AR session...")
        
        clearCurrentData()
        
        var newGenerationID = 0
        
        mapUpdateQueue.sync { [weak self] in
            guard let self = self else { return }
            self.privateMapGenerationID += 1
            newGenerationID = self.privateMapGenerationID
            self.persistentOccupancyMap = Array(repeating: Array(repeating: 0.0, count: 500), count: 500)
        }
        
        // Reset lightweight localization
        localizationManager.reset()
        
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
        
        if isRunning {
            session.pause()
            isRunning = false
        }
        
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.2) {
            self.configuration = self.createConfiguration()
            self.session.run(self.configuration, options: [.resetTracking, .removeExistingAnchors])
            self.isRunning = true
            self.statusMessage = "Reset complete"
        }
    }
    
    private func clearCurrentData() {
        DispatchQueue.main.async {
            self.currentDepthData = nil
            self.currentPointCloud = nil
            self.currentCameraData = nil
            self.currentPose = nil
        }
        
        lastProcessedTime = 0
    }
    
    func toggleDebugMode() {
        debugMode = (debugMode + 1) % 3
        statusMessage = "Debug mode: \(debugMode)"
        reset()
    }
    
    func clearOccupancyMap() {
        print("🧹 Clearing occupancy map...")
        
        var newGenerationID = 0
        
        mapUpdateQueue.sync { [weak self] in
            guard let self = self else { return }
            self.privateMapGenerationID += 1
            newGenerationID = self.privateMapGenerationID
            self.persistentOccupancyMap = Array(repeating: Array(repeating: 0.0, count: 500), count: 500)
        }
        
        DispatchQueue.main.async { [weak self] in
            guard let self = self else { return }
            
            // Preserve robot position from localization system
            let currentRobotPos = self.localizationManager.currentRobotPosition
            
            self.occupancyMapData = OccupancyMapData(
                map: Array(repeating: Array(repeating: 2, count: 500), count: 500),
                robotPosition: currentRobotPos,
                mapResolution: 0.05,
                mapOrigin: SIMD3<Float>(0, 0, 0),
                frontierTarget: nil
            )
            
            self.pointsProcessed = 0
            self.mapUpdates = 0
            self.obstaclePointsFiltered = 0
            self.mapGenerationID = newGenerationID
            
            self.statusMessage = "Map cleared"
        }
    }
    
    func getSession() -> ARSession {
        return session
    }
    
    // MARK: - Simple Obstacle Detection (Fast)
    
    private func simpleObstacleDetection(_ depthData: ARDepthData,
                                       cameraTransform: simd_float4x4,
                                       intrinsics: simd_float3x3) -> [SIMD3<Float>] {
        
        let depthMap = depthData.depthMap
        
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        
        guard let depthBuffer = CVPixelBufferGetBaseAddress(depthMap)?.assumingMemoryBound(to: Float32.self) else {
            return []
        }
        
        var obstacles: [SIMD3<Float>] = []
        obstacles.reserveCapacity(1000) // Pre-allocate for speed
        
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float32>.size
        
        // Aggressive sampling for speed (every 8th pixel)
        for y in stride(from: 0, to: height, by: depthSamplingRate) {
            for x in stride(from: 0, to: width, by: depthSamplingRate) {
                let depth = depthBuffer[y * bytesPerRow + x]
                
                // Simple, fast depth filtering
                guard depth > minDepthRange && depth < maxDepthRange && depth.isFinite else { continue }
                
                // Convert to world space (simplified - trust ARKit's intrinsics)
                let fx = intrinsics[0][0]
                let fy = intrinsics[1][1]
                let cx = intrinsics[2][0]
                let cy = intrinsics[2][1]
                
                let cameraX = (Float(x) - cx) * depth / fx
                let cameraY = (Float(y) - cy) * depth / fy
                let cameraZ = -depth
                
                let cameraPoint = SIMD4<Float>(cameraX, cameraY, cameraZ, 1.0)
                let worldPoint = cameraTransform * cameraPoint
                
                // Simple height filter relative to camera (trust ARKit's coordinate system)
                let heightAboveCamera = worldPoint.y - cameraTransform.columns.3.y
                if heightAboveCamera > -robotHeight && heightAboveCamera < 0.5 {
                    obstacles.append(worldPoint.xyz)
                }
            }
        }
        
        return obstacles
    }
    
    // MARK: - Coordinate Transformations (Simplified)
    
    private func worldToMap(_ worldPos: SIMD3<Float>) -> (x: Int, y: Int)? {
        // Simple coordinate transform - trust ARKit's origin
        let mapData = occupancyMapData
        
        let relativeX = worldPos.x - mapData.mapOrigin.x
        let relativeZ = worldPos.z - mapData.mapOrigin.z
        
        let gridX = Int((relativeX / mapData.mapResolution) + 250)
        let gridY = Int((relativeZ / mapData.mapResolution) + 250)
        
        if gridX >= 0 && gridX < 500 && gridY >= 0 && gridY < 500 {
            return (x: gridX, y: gridY)
        }
        return nil
    }
    
    // MARK: - Frontier Exploration (Unchanged)
    
    private func findNearestUnexploredCell(robotPos: (x: Int, y: Int), map: [[Int]]) -> (x: Int, y: Int)? {
        let maxSearchRadius = 100
        var nearestDistance = Float.infinity
        var nearestCell: (x: Int, y: Int)? = nil
        
        for radius in 1...maxSearchRadius {
            let minX = max(0, robotPos.x - radius)
            let maxX = min(499, robotPos.x + radius)
            let minY = max(0, robotPos.y - radius)
            let maxY = min(499, robotPos.y + radius)
            
            for x in minX...maxX {
                for y in minY...maxY {
                    let isPerimeter = (x == minX || x == maxX || y == minY || y == maxY)
                    if !isPerimeter { continue }
                    
                    if map[y][x] == 2 {
                        let distance = sqrt(Float((x - robotPos.x) * (x - robotPos.x) + (y - robotPos.y) * (y - robotPos.y)))
                        if distance < nearestDistance {
                            nearestDistance = distance
                            nearestCell = (x: x, y: y)
                        }
                    }
                }
            }
            
            if nearestCell != nil {
                return nearestCell
            }
        }
        
        return nil
    }
    
    private func updateFrontierTarget(robotPos: (x: Int, y: Int), map: [[Int]], currentTarget: (x: Int, y: Int)?) -> (x: Int, y: Int)? {
        if let target = currentTarget {
            if target.x >= 0 && target.x < 500 && target.y >= 0 && target.y < 500 {
                if map[target.y][target.x] == 2 {
                    return target
                }
            }
        }
        
        return findNearestUnexploredCell(robotPos: robotPos, map: map)
    }
    
    private func updateOccupancyMapWithObstaclePoints(_ obstaclePoints: [SIMD3<Float>], cameraPosition: SIMD3<Float>) {
        // Only update if tracking is reliable
        guard isTrackingReliable else {
            print("⚠️ Skipping map update - tracking unreliable")
            return
        }
        
        let currentGenerationID = privateMapGenerationID
        
        mapUpdateQueue.async { [weak self] in
            guard let self = self else { return }
            
            guard currentGenerationID == self.privateMapGenerationID else {
                return
            }
            
            var mapCopy = self.persistentOccupancyMap
            let mapData = self.occupancyMapData
            
            guard let cameraMapPos = self.worldToMap(cameraPosition) else { return }
            
            var processedCells: Set<String> = Set()
            
            for obstaclePoint in obstaclePoints {
                guard let obstacleMapPos = self.worldToMap(obstaclePoint) else { continue }
                
                let rayPoints = self.bresenhamLine(
                    from: (x: cameraMapPos.x, y: cameraMapPos.y),
                    to: (x: obstacleMapPos.x, y: obstacleMapPos.y)
                )
                
                guard rayPoints.count > 2 else { continue }
                
                // Mark free space
                for i in 0..<(rayPoints.count - 1) {
                    let point = rayPoints[i]
                    let cellKey = "\(point.x),\(point.y)"
                    
                    if point.x >= 0 && point.x < 500 && point.y >= 0 && point.y < 500 && !processedCells.contains(cellKey) {
                        mapCopy[point.y][point.x] = max(-1.0, mapCopy[point.y][point.x] - 0.05)
                        processedCells.insert(cellKey)
                    }
                }
                
                // Mark obstacle
                if rayPoints.count > 0 {
                    let obstacleMapPoint = rayPoints.last!
                    let cellKey = "\(obstacleMapPoint.x),\(obstacleMapPoint.y)"
                    
                    if obstacleMapPoint.x >= 0 && obstacleMapPoint.x < 500 && obstacleMapPoint.y >= 0 && obstacleMapPoint.y < 500 {
                        let currentConfidence = mapCopy[obstacleMapPoint.y][obstacleMapPoint.x]
                        mapCopy[obstacleMapPoint.y][obstacleMapPoint.x] = min(1.0, currentConfidence + 0.15)
                        processedCells.insert(cellKey)
                    }
                }
            }
            
            guard currentGenerationID == self.privateMapGenerationID else {
                return
            }
            
            // Convert to discrete map
            var discreteMap = Array(repeating: Array(repeating: 2, count: 500), count: 500)
            for y in 0..<500 {
                for x in 0..<500 {
                    let confidence = mapCopy[y][x]
                    let currentState = mapData.map[y][x]
                    
                    switch currentState {
                    case 0:
                        if confidence > 0.4 {
                            discreteMap[y][x] = 1
                        } else if confidence < -0.2 {
                            discreteMap[y][x] = 0
                        } else {
                            discreteMap[y][x] = 2
                        }
                    case 1:
                        if confidence < -0.4 {
                            discreteMap[y][x] = 0
                        } else if confidence > 0.2 {
                            discreteMap[y][x] = 1
                        } else {
                            discreteMap[y][x] = 2
                        }
                    default:
                        if confidence > 0.3 {
                            discreteMap[y][x] = 1
                        } else if confidence < -0.3 {
                            discreteMap[y][x] = 0
                        } else {
                            discreteMap[y][x] = 2
                        }
                    }
                }
            }
            
            self.persistentOccupancyMap = mapCopy
            
            DispatchQueue.main.async {
                guard currentGenerationID == self.privateMapGenerationID else {
                    return
                }
                
                let robotPos = self.localizationManager.currentRobotPosition
                
                let newFrontierTarget = self.updateFrontierTarget(
                    robotPos: robotPos,
                    map: discreteMap,
                    currentTarget: mapData.frontierTarget
                )
                
                self.occupancyMapData = OccupancyMapData(
                    map: discreteMap,
                    robotPosition: robotPos,
                    mapResolution: 0.05,
                    mapOrigin: SIMD3<Float>(0, 0, 0),
                    frontierTarget: newFrontierTarget
                )
                
                self.pointsProcessed += obstaclePoints.count
                self.obstaclePointsFiltered = obstaclePoints.count
                self.mapUpdates += 1
            }
        }
    }
    
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
    
    // MARK: - ARSessionDelegate (Simplified for Speed)
    
    func session(_ session: ARSession, didAdd anchors: [ARAnchor]) {
        for anchor in anchors {
            // Let lightweight localization handle anchors for stability
            localizationManager.addAnchor(anchor)
            
            // Handle floor plane detection
            if let planeAnchor = anchor as? ARPlaneAnchor,
               planeAnchor.alignment == .horizontal {
                
                let planeY = planeAnchor.transform.columns.3.y
                let cameraY = session.currentFrame?.camera.transform.columns.3.y ?? 0
                
                if planeY < cameraY && (!floorDetected || planeY < floorY) {
                    detectedFloorPlane = planeAnchor
                    floorY = planeY
                    
                    DispatchQueue.main.async {
                        self.floorDetected = true
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
                
                detectedFloorPlane = planeAnchor
                floorY = planeAnchor.transform.columns.3.y
            }
        }
    }
    
    func session(_ session: ARSession, didRemove anchors: [ARAnchor]) {
        for anchor in anchors {
            // Notify lightweight localization
            localizationManager.removeAnchor(anchor)
            
            if let planeAnchor = anchor as? ARPlaneAnchor,
               planeAnchor.identifier == detectedFloorPlane?.identifier {
                
                DispatchQueue.main.async {
                    self.floorDetected = false
                    self.detectedFloorPlane = nil
                }
            }
        }
    }
    
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // Simplified, fast frame processing
        let currentTime = frame.timestamp
        guard currentTime - lastProcessedTime >= minFrameInterval else {
            return // Throttle frames
        }
        lastProcessedTime = currentTime
        
        // Use lightweight localization (fast ARKit-native tracking)
        let localizationSuccess = localizationManager.updateWithFrame(frame)
        
        // Update status simply
        DispatchQueue.main.async { [weak self] in
            if localizationSuccess {
                self?.statusMessage = "ARKit tracking: \(self?.trackingQuality ?? "Unknown")"
            } else {
                self?.statusMessage = "ARKit tracking unstable"
            }
        }
        
        // Create pose data for publishers
        let poseData = PoseData(
            position: frame.camera.transform.columns.3.xyz,
            orientation: simd_quatf(frame.camera.transform),
            timestamp: frame.timestamp
        )
        
        DispatchQueue.main.async { [weak self] in
            self?.currentPose = poseData
        }
        
        posePublisher.send(poseData)
        
        // Process depth data only when tracking is reliable
        if localizationSuccess && hasLiDAR,
           let sceneDepth = frame.sceneDepth ?? frame.smoothedSceneDepth {
            
            // Fast depth processing on background queue
            frameProcessingQueue.async { [weak self] in
                guard let self = self else { return }
                
                // Simple, fast obstacle detection
                let obstaclePoints = self.simpleObstacleDetection(
                    sceneDepth,
                    cameraTransform: frame.camera.transform,
                    intrinsics: frame.camera.intrinsics
                )
                
                if !obstaclePoints.isEmpty {
                    let cameraPosition = frame.camera.transform.columns.3.xyz
                    self.updateOccupancyMapWithObstaclePoints(obstaclePoints, cameraPosition: cameraPosition)
                    
                    // Create point cloud for visualization
                    let pointCloud = PointCloudData(
                        points: Array(obstaclePoints.prefix(500)), // Limit for speed
                        colors: obstaclePoints.prefix(500).map { _ in SIMD4<Float>(1, 0, 0, 1) },
                        timestamp: frame.timestamp
                    )
                    
                    DispatchQueue.main.async {
                        self.currentPointCloud = pointCloud
                    }
                    
                    self.pointCloudPublisher.send(pointCloud)
                }
                
                // Send depth data to publisher
                let depthDataObj = DepthData(
                    depthMap: sceneDepth.depthMap,
                    confidenceMap: sceneDepth.confidenceMap,
                    timestamp: frame.timestamp
                )
                
                DispatchQueue.main.async {
                    self.currentDepthData = depthDataObj
                }
                
                self.depthPublisher.send(depthDataObj)
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
            self.isRunning = false
        }
    }
}
