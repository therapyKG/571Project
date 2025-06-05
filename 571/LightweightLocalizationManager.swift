//
//  LightweightLocalizationManager.swift
//  571
//
//  Created by Kurt Gu on 6/4/25.
//


//
//  LightweightLocalization.swift
//  MobileRobotics
//
//  Lightweight localization that leverages ARKit's native capabilities
//

import Foundation
import ARKit
import simd
import Combine

// Simple, fast localization that trusts ARKit's SLAM
class LightweightLocalizationManager: ObservableObject {
    // Basic tracking state
    @Published var isTrackingReliable = false
    @Published var currentRobotPosition: (x: Int, y: Int) = (250, 250)
    @Published var currentHeading: Float = 0.0
    @Published var trackingQuality: String = "Unknown"
    
    // Map coordinate system (simple, stable approach)
    private var mapOrigin: SIMD3<Float>?
    private var isMapOriginSet = false
    private let mapResolution: Float = 0.05
    private let mapSize = 500
    
    // Simple pose smoothing (minimal overhead)
    private var recentPoses: [SIMD3<Float>] = []
    private let maxPoseHistory = 3
    
    // Anchor-based stability (leverage ARKit's anchors)
    private var referenceAnchor: ARAnchor?
    
    init() {}
    
    // Fast update using ARKit's native tracking
    func updateWithFrame(_ frame: ARFrame) -> Bool {
        let trackingState = frame.camera.trackingState
        let cameraTransform = frame.camera.transform
        let currentPosition = cameraTransform.columns.3.xyz
        
        // Simple tracking quality assessment
        let isCurrentlyReliable = evaluateTrackingQuality(trackingState)
        
        DispatchQueue.main.async { [weak self] in
            self?.isTrackingReliable = isCurrentlyReliable
            self?.trackingQuality = self?.trackingQualityString(trackingState) ?? "Unknown"
        }
        
        guard isCurrentlyReliable else { return false }
        
        // Initialize map origin on first reliable tracking
        if !isMapOriginSet {
            mapOrigin = currentPosition
            isMapOriginSet = true
            print("🗺️ Map origin set at: \(currentPosition)")
        }
        
        // Simple pose smoothing (just average last few poses)
        recentPoses.append(currentPosition)
        if recentPoses.count > maxPoseHistory {
            recentPoses.removeFirst()
        }
        
        let smoothedPosition = recentPoses.reduce(SIMD3<Float>(0,0,0), +) / Float(recentPoses.count)
        
        // Convert to map coordinates
        guard let mapPos = worldToMap(smoothedPosition) else { return false }
        
        // Extract heading (simple approach)
        let heading = extractHeading(from: cameraTransform)
        
        DispatchQueue.main.async { [weak self] in
            self?.currentRobotPosition = mapPos
            self?.currentHeading = heading
        }
        
        return true
    }
    
    // Use ARKit anchors for coordinate stability (leverage ARKit's loop closure)
    func addAnchor(_ anchor: ARAnchor) {
        if referenceAnchor == nil {
            referenceAnchor = anchor
            print("⚓ Reference anchor set for coordinate stability")
        }
    }
    
    func removeAnchor(_ anchor: ARAnchor) {
        if anchor.identifier == referenceAnchor?.identifier {
            referenceAnchor = nil
            print("⚓ Reference anchor lost")
        }
    }
    
    private func evaluateTrackingQuality(_ trackingState: ARCamera.TrackingState) -> Bool {
        switch trackingState {
        case .normal:
            return true
        case .limited(let reason):
            // Be more permissive - ARKit handles most edge cases well
            switch reason {
            case .insufficientFeatures, .excessiveMotion:
                return false
            case .initializing, .relocalizing:
                return false
            @unknown default:
                return false
            }
        case .notAvailable:
            return false
        }
    }
    
    private func trackingQualityString(_ trackingState: ARCamera.TrackingState) -> String {
        switch trackingState {
        case .normal:
            return "Normal"
        case .limited(let reason):
            switch reason {
            case .initializing:
                return "Initializing"
            case .insufficientFeatures:
                return "Insufficient Features"
            case .excessiveMotion:
                return "Excessive Motion"
            case .relocalizing:
                return "Relocalizing"
            @unknown default:
                return "Limited"
            }
        case .notAvailable:
            return "Not Available"
        }
    }
    
    private func worldToMap(_ worldPos: SIMD3<Float>) -> (x: Int, y: Int)? {
        guard let origin = mapOrigin else { return nil }
        
        let relativeX = worldPos.x - origin.x
        let relativeZ = worldPos.z - origin.z
        
        let gridX = Int((relativeX / mapResolution) + Float(mapSize / 2))
        let gridY = Int((relativeZ / mapResolution) + Float(mapSize / 2))
        
        guard gridX >= 0 && gridX < mapSize && gridY >= 0 && gridY < mapSize else {
            return nil
        }
        
        return (x: gridX, y: gridY)
    }
    
    private func extractHeading(from transform: simd_float4x4) -> Float {
        let forwardVector = SIMD3<Float>(-transform[2][0], -transform[2][1], -transform[2][2])
        let forwardXZ = SIMD2<Float>(forwardVector.x, forwardVector.z)
        return atan2(forwardXZ.x, forwardXZ.y)
    }
    
    func reset() {
        mapOrigin = nil
        isMapOriginSet = false
        recentPoses.removeAll()
        referenceAnchor = nil
        
        DispatchQueue.main.async { [weak self] in
            self?.currentRobotPosition = (250, 250)
            self?.currentHeading = 0.0
            self?.isTrackingReliable = false
            self?.trackingQuality = "Reset"
        }
    }
}