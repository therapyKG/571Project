//
//  ARViewContainer.swift
//  571
//
//  Created by Kurt Gu on 6/3/25.
//

import SwiftUI
import RealityKit
import ARKit

// SwiftUI view that wraps an ARView for displaying camera and AR content
struct ARViewContainer: UIViewRepresentable {
    @ObservedObject var framework = RoboticsFramework.shared
    
    func makeUIView(context: Context) -> ARView {
        // Create ARView with manual session configuration
        let arView = ARView(frame: .zero, cameraMode: .ar, automaticallyConfigureSession: false)
        
        // Use the session from the framework
        arView.session = framework.getSession()
        
        // Add a coaching overlay to help the user find planes
        let coachingOverlay = ARCoachingOverlayView()
        coachingOverlay.session = framework.getSession()
        coachingOverlay.goal = .horizontalPlane
        coachingOverlay.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        arView.addSubview(coachingOverlay)
        
        // Configure debug visualizations based on debug mode
        updateDebugOptions(arView: arView, debugMode: framework.debugMode)
        
        return arView
    }
    
    func updateUIView(_ uiView: ARView, context: Context) {
        // Update debug options when debug mode changes
        updateDebugOptions(arView: uiView, debugMode: framework.debugMode)
    }
    
    // MARK: - Private Methods
    
    private func updateDebugOptions(arView: ARView, debugMode: Int) {
        #if DEBUG
        switch debugMode {
        case 0:
            arView.debugOptions = [.showFeaturePoints, .showAnchorOrigins, .showAnchorGeometry]
        case 1:
            arView.debugOptions = [.showFeaturePoints, .showAnchorOrigins, .showAnchorGeometry]
        case 2:
            arView.debugOptions = [.showFeaturePoints]
        default:
            arView.debugOptions = []
        }
        #else
        // No debug options in release builds
        arView.debugOptions = []
        #endif
    }
}

// MARK: - Preview

#Preview {
    ARViewContainer()
        .frame(width: 400, height: 300)
}
