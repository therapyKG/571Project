//
//  EnhancedLocalization.swift
//  MobileRobotics
//
//  Enhanced SLAM-based localization system for stable robot positioning
//

import Foundation
import ARKit
import simd
import Combine

// MARK: - Pose Tracking and Validation

class PoseTracker: ObservableObject {
    // Tracking state
    @Published var isTrackingStable = false
    @Published var lastStableTrackingTime: TimeInterval = 0
    @Published var trackingLostCount = 0
    
    // Pose filtering
    var poseHistory: [PoseData] = [] // Made public for UI access
    private let maxHistorySize = 10
    private let maxVelocity: Float = 2.0 // m/s
    private let maxAngularVelocity: Float = 1.0 // rad/s
    
    // Smoothed pose output
    @Published var smoothedPose: PoseData?
    @Published var smoothedHeading: Float = 0.0
    
    init() {}
    
    func updateWithFrame(_ frame: ARFrame) -> Bool {
        let trackingState = frame.camera.trackingState
        let currentTime = frame.timestamp
        
        // Extract pose immediately without retaining frame
        let poseData = PoseData(
            position: frame.camera.transform.columns.3.xyz,
            orientation: simd_quatf(frame.camera.transform),
            timestamp: currentTime
        )
        
        return updateWithExtractedData(trackingState: trackingState, pose: poseData)
    }
    
    func updateWithExtractedData(trackingState: ARCamera.TrackingState, pose: PoseData) -> Bool {
        // Evaluate tracking quality
        let isCurrentlyStable = evaluateTrackingQuality(trackingState)
        let currentTime = pose.timestamp
        
        // Store current tracking state for immediate use
        let wasTrackingStable = isTrackingStable
        let currentTrackingLostCount = trackingLostCount
        
        // Update tracking state variables for immediate evaluation
        var shouldProcessPose = false
        var newTrackingStable = wasTrackingStable
        var newTrackingLostCount = currentTrackingLostCount
        
        if isCurrentlyStable {
            newTrackingStable = true
            newTrackingLostCount = 0
            shouldProcessPose = true
        } else {
            if wasTrackingStable {
                newTrackingLostCount += 1
                // Allow brief tracking issues but mark as unstable after multiple failures
                if newTrackingLostCount > 5 {
                    newTrackingStable = false
                    shouldProcessPose = false
                } else {
                    shouldProcessPose = true // Still allow processing during brief instability
                }
            } else {
                shouldProcessPose = false
            }
        }
        
        // Update published properties on main thread
        DispatchQueue.main.async { [weak self] in
            self?.isTrackingStable = newTrackingStable
            self?.lastStableTrackingTime = isCurrentlyStable ? currentTime : self?.lastStableTrackingTime ?? 0
            self?.trackingLostCount = newTrackingLostCount
        }
        
        // Only process pose updates when tracking is stable
        guard shouldProcessPose else {
            print("⚠️ Skipping pose update - tracking unstable")
            return false
        }
        
        // Validate pose against motion constraints
        guard validatePoseUpdate(pose) else {
            print("⚠️ Pose update rejected - motion constraints violated")
            return false
        }
        
        // Add to history and smooth
        addToHistory(pose)
        updateSmoothedPose()
        
        return true
    }
    
    private func evaluateTrackingQuality(_ trackingState: ARCamera.TrackingState) -> Bool {
        switch trackingState {
        case .normal:
            return true
        case .limited(let reason):
            switch reason {
            case .initializing, .relocalizing:
                return false // Wait for stable tracking
            case .insufficientFeatures, .excessiveMotion:
                return false // Poor tracking conditions
            @unknown default:
                return false
            }
        case .notAvailable:
            return false
        }
    }
    
    private func validatePoseUpdate(_ newPose: PoseData) -> Bool {
        guard let lastPose = poseHistory.last else {
            return true // First pose is always valid
        }
        
        let deltaTime = Float(newPose.timestamp - lastPose.timestamp)
        guard deltaTime > 0 && deltaTime < 1.0 else {
            return false // Invalid time delta
        }
        
        // Check linear velocity
        let deltaPosition = newPose.position - lastPose.position
        let velocity = length(deltaPosition) / deltaTime
        
        if velocity > maxVelocity {
            print("⚠️ Velocity too high: \(velocity) m/s")
            return false
        }
        
        // Check angular velocity
        let deltaOrientation = newPose.orientation.inverse * lastPose.orientation
        let angle = 2.0 * acos(abs(deltaOrientation.real))
        let angularVelocity = angle / deltaTime
        
        if angularVelocity > maxAngularVelocity {
            print("⚠️ Angular velocity too high: \(angularVelocity) rad/s")
            return false
        }
        
        return true
    }
    
    private func addToHistory(_ pose: PoseData) {
        poseHistory.append(pose)
        if poseHistory.count > maxHistorySize {
            poseHistory.removeFirst()
        }
    }
    
    private func updateSmoothedPose() {
        guard poseHistory.count >= 3 else {
            let latestPose = poseHistory.last
            let latestHeading = latestPose != nil ? extractHeading(from: latestPose!.orientation) : 0.0
            
            // Update on main thread
            DispatchQueue.main.async { [weak self] in
                self?.smoothedPose = latestPose
                self?.smoothedHeading = latestHeading
            }
            return
        }
        
        // Use weighted average with recent poses having higher weight
        var weightedPosition = SIMD3<Float>(0, 0, 0)
        var totalWeight: Float = 0
        
        for (index, pose) in poseHistory.enumerated() {
            let weight = Float(index + 1) // More recent poses have higher weight
            weightedPosition += pose.position * weight
            totalWeight += weight
        }
        
        let smoothedPosition = weightedPosition / totalWeight
        
        // For orientation, use most recent (quaternion averaging is complex)
        let smoothedOrientation = poseHistory.last!.orientation
        
        let newSmoothedPose = PoseData(
            position: smoothedPosition,
            orientation: smoothedOrientation,
            timestamp: poseHistory.last!.timestamp
        )
        
        // Update smoothed heading
        let newSmoothedHeading = extractHeading(from: smoothedOrientation)
        
        // Update on main thread to avoid publishing warnings
        DispatchQueue.main.async { [weak self] in
            self?.smoothedPose = newSmoothedPose
            self?.smoothedHeading = newSmoothedHeading
        }
    }
    
    private func extractHeading(from orientation: simd_quatf) -> Float {
        let rotationMatrix = matrix_float4x4(orientation)
        let forwardVector = SIMD3<Float>(-rotationMatrix[2][0], -rotationMatrix[2][1], -rotationMatrix[2][2])
        let forwardXZ = SIMD2<Float>(forwardVector.x, forwardVector.z)
        return atan2(forwardXZ.x, forwardXZ.y)
    }
    
    func reset() {
        poseHistory.removeAll()
        
        // Update published properties on main thread
        DispatchQueue.main.async { [weak self] in
            self?.smoothedPose = nil
            self?.smoothedHeading = 0.0
            self?.isTrackingStable = false
            self?.trackingLostCount = 0
        }
    }
}

// MARK: - Anchor-Based Map Coordinate System

class MapCoordinateSystem: ObservableObject {
    // Map reference system
    @Published var mapOrigin = SIMD3<Float>(0, 0, 0)
    @Published var mapOriginAnchor: ARAnchor?
    @Published var isMapOriginStable = false
    
    // Coordinate transform tracking
    private var lastWorldTransform = simd_float4x4(diagonal: SIMD4<Float>(1, 1, 1, 1))
    var accumulatedDrift = SIMD3<Float>(0, 0, 0) // Made public for UI access
    
    // Map parameters
    let mapResolution: Float = 0.05 // 5cm per cell
    let mapSize = 500
    
    init() {}
    
    func initializeMapOrigin(with pose: PoseData) {
        mapOrigin = pose.position
        lastWorldTransform = matrix_float4x4(pose.orientation)
        lastWorldTransform.columns.3 = SIMD4<Float>(pose.position, 1.0)
        accumulatedDrift = SIMD3<Float>(0, 0, 0)
        
        // Update published property on main thread
        DispatchQueue.main.async { [weak self] in
            self?.isMapOriginStable = true
        }
        
        print("🗺️ Map origin initialized at: \(mapOrigin)")
    }
    
    func updateWithAnchor(_ anchor: ARAnchor) {
        // Use anchors to detect coordinate system changes
        if mapOriginAnchor == nil {
            mapOriginAnchor = anchor
            print("⚓ Map origin anchor set")
        }
    }
    
    func detectCoordinateSystemChange(_ currentTransform: simd_float4x4) -> Bool {
        // Detect if ARKit has made a significant correction to the coordinate system
        let currentPosition = currentTransform.columns.3.xyz
        let lastPosition = lastWorldTransform.columns.3.xyz
        
        let positionDrift = length(currentPosition - lastPosition)
        
        // If position has jumped significantly, ARKit likely made a correction
        if positionDrift > 0.5 { // 50cm threshold
            print("🔄 Coordinate system change detected: \(positionDrift)m drift")
            accumulatedDrift += (currentPosition - lastPosition)
            return true
        }
        
        lastWorldTransform = currentTransform
        return false
    }
    
    func worldToMap(_ worldPos: SIMD3<Float>) -> (x: Int, y: Int)? {
        // Apply drift correction
        let correctedPos = worldPos - accumulatedDrift
        
        // Calculate relative position from map origin
        let relativeX = correctedPos.x - mapOrigin.x
        let relativeZ = correctedPos.z - mapOrigin.z
        
        // Convert to grid coordinates
        let gridX = Int((relativeX / mapResolution) + Float(mapSize / 2))
        let gridY = Int((relativeZ / mapResolution) + Float(mapSize / 2))
        
        // Check bounds
        if gridX >= 0 && gridX < mapSize && gridY >= 0 && gridY < mapSize {
            return (x: gridX, y: gridY)
        }
        return nil
    }
    
    func adjustMapOrigin(_ newOrigin: SIMD3<Float>) {
        let originShift = newOrigin - mapOrigin
        mapOrigin = newOrigin
        
        print("🗺️ Map origin adjusted by: \(originShift)")
    }
    
    func reset() {
        mapOrigin = SIMD3<Float>(0, 0, 0)
        mapOriginAnchor = nil
        lastWorldTransform = simd_float4x4(diagonal: SIMD4<Float>(1, 1, 1, 1))
        accumulatedDrift = SIMD3<Float>(0, 0, 0)
        
        // Update published property on main thread
        DispatchQueue.main.async { [weak self] in
            self?.isMapOriginStable = false
        }
    }
}

// MARK: - Enhanced Localization Manager

class LocalizationManager: ObservableObject {
    // Components
    let poseTracker = PoseTracker()
    let coordinateSystem = MapCoordinateSystem()
    
    // Current state
    @Published var currentRobotPosition: (x: Int, y: Int) = (250, 250)
    @Published var currentHeading: Float = 0.0
    @Published var localizationQuality: String = "Unknown"
    @Published var isLocalizationStable = false
    
    // Initialization state
    private var isInitialized = false
    private var initializationFrameCount = 0
    private let requiredInitFrames = 30 // Require 30 stable frames before initialization
    
    init() {
        // Observe pose tracker changes
        poseTracker.$smoothedPose
            .compactMap { $0 }
            .sink { [weak self] pose in
                self?.updateRobotPosition(with: pose)
            }
            .store(in: &cancellables)
        
        poseTracker.$isTrackingStable
            .sink { [weak self] isStable in
                self?.updateLocalizationQuality(isStable)
            }
            .store(in: &cancellables)
    }
    
    private var cancellables = Set<AnyCancellable>()
    
    func updateWithFrame(_ frame: ARFrame) -> Bool {
        // Extract data immediately without retaining frame
        let trackingState = frame.camera.trackingState
        let poseData = PoseData(
            position: frame.camera.transform.columns.3.xyz,
            orientation: simd_quatf(frame.camera.transform),
            timestamp: frame.timestamp
        )
        
        return updateWithExtractedData(trackingState: trackingState, pose: poseData)
    }
    
    func updateWithExtractedData(trackingState: ARCamera.TrackingState, pose: PoseData) -> Bool {
        // Update pose tracking with extracted data
        guard poseTracker.updateWithExtractedData(trackingState: trackingState, pose: pose) else {
            return false // Tracking not stable
        }
        
        // Handle initialization
        if !isInitialized {
            return handleInitialization(pose)
        }
        
        // Detect coordinate system changes
        var currentTransform = matrix_float4x4(pose.orientation)
        currentTransform.columns.3 = SIMD4<Float>(pose.position, 1.0)
        if coordinateSystem.detectCoordinateSystemChange(currentTransform) {
            handleCoordinateSystemChange()
        }
        
        return true
    }
    
    private func handleInitialization(_ pose: PoseData) -> Bool {
        initializationFrameCount += 1
        
        if initializationFrameCount >= requiredInitFrames {
            // Initialize map coordinate system
            if let smoothedPose = poseTracker.smoothedPose {
                coordinateSystem.initializeMapOrigin(with: smoothedPose)
                
                // Update published properties on main thread
                DispatchQueue.main.async { [weak self] in
                    self?.isInitialized = true
                    self?.isLocalizationStable = true
                    self?.localizationQuality = "Initialized"
                }
                
                print("✅ Localization system initialized after \(initializationFrameCount) frames")
                return true
            }
        } else {
            // Update on main thread
            DispatchQueue.main.async { [weak self] in
                self?.localizationQuality = "Initializing (\(self?.initializationFrameCount ?? 0)/\(self?.requiredInitFrames ?? 30))"
            }
        }
        
        return false
    }
    
    private func handleCoordinateSystemChange() {
        // When ARKit corrects the coordinate system, we need to adjust our map
        print("🔄 Handling coordinate system change")
        
        // Option 1: Adjust map origin to maintain consistency
        if let currentPose = poseTracker.smoothedPose {
            coordinateSystem.adjustMapOrigin(currentPose.position)
        }
        
        // Update published properties on main thread
        DispatchQueue.main.async { [weak self] in
            // Mark localization as temporarily unstable
            self?.isLocalizationStable = false
            self?.localizationQuality = "Relocalized"
        }
        
        // Re-stabilize after a brief period
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) { [weak self] in
            self?.isLocalizationStable = true
            self?.localizationQuality = "Stable"
        }
    }
    
    private func updateRobotPosition(with pose: PoseData) {
        guard isInitialized else { return }
        
        // Convert world position to map coordinates
        if let mapPos = coordinateSystem.worldToMap(pose.position) {
            let newHeading = poseTracker.smoothedHeading
            
            // Update on main thread
            DispatchQueue.main.async { [weak self] in
                self?.currentRobotPosition = mapPos
                self?.currentHeading = newHeading
                
                // Update quality indicator
                if self?.isLocalizationStable == true {
                    self?.localizationQuality = "Stable"
                }
            }
        } else {
            // Update on main thread
            DispatchQueue.main.async { [weak self] in
                self?.localizationQuality = "Out of bounds"
                self?.isLocalizationStable = false
            }
        }
    }
    
    private func updateLocalizationQuality(_ isTrackingStable: Bool) {
        DispatchQueue.main.async { [weak self] in
            if !isTrackingStable {
                self?.isLocalizationStable = false
                self?.localizationQuality = "Tracking Lost"
            }
        }
    }
    
    func addAnchor(_ anchor: ARAnchor) {
        coordinateSystem.updateWithAnchor(anchor)
    }
    
    func reset() {
        poseTracker.reset()
        coordinateSystem.reset()
        isInitialized = false
        initializationFrameCount = 0
        
        // Update published properties on main thread
        DispatchQueue.main.async { [weak self] in
            self?.currentRobotPosition = (250, 250)
            self?.currentHeading = 0.0
            self?.localizationQuality = "Reset"
            self?.isLocalizationStable = false
        }
    }
}

// MARK: - Supporting Extensions

import Combine

extension simd_float4x4 {
    init(_ quaternion: simd_quatf) {
        let q = quaternion
        let x = q.imag.x
        let y = q.imag.y
        let z = q.imag.z
        let w = q.real
        
        self.init(
            SIMD4<Float>(1 - 2*y*y - 2*z*z, 2*x*y + 2*w*z, 2*x*z - 2*w*y, 0),
            SIMD4<Float>(2*x*y - 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z + 2*w*x, 0),
            SIMD4<Float>(2*x*z + 2*w*y, 2*y*z - 2*w*x, 1 - 2*x*x - 2*y*y, 0),
            SIMD4<Float>(0, 0, 0, 1)
        )
    }
}
