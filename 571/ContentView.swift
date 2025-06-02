//
//  ContentView.swift
//  MobileRobotics
//
//  Created by Kurt Gu on 5/5/25.
//

import SwiftUI
import RealityKit
import ARKit
import Combine

// Data structures for sensor outputs
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

// Main class that provides access to ARKit sensors
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
    
    // Framework status
    @Published var isRunning = false
    @Published var hasLiDAR = false
    @Published var statusMessage = "Not initialized"
    
    // Initialize the framework
    private override init() {
        // Create configuration
        configuration = ARWorldTrackingConfiguration()
        
        super.init()
        
        // Check LiDAR availability
        hasLiDAR = ARWorldTrackingConfiguration.supportsFrameSemantics([.sceneDepth, .smoothedSceneDepth])
        
        // Set up session
        session.delegate = self
    }
    
    // Start the AR session with the given options
    func start() {
        guard !isRunning else { return }
        
        // Configure the session for depth data if available
        if hasLiDAR {
            configuration.frameSemantics = [.sceneDepth, .smoothedSceneDepth]
            statusMessage = "LiDAR sensor enabled"
        } else {
            statusMessage = "Device does not have LiDAR, using camera only"
        }
        
        // Enable plane detection
        configuration.planeDetection = [.horizontal, .vertical]
        
        // Run the session
        session.run(configuration)
        isRunning = true
    }
    
    // Stop the AR session
    func stop() {
        guard isRunning else { return }
        
        session.pause()
        isRunning = false
        statusMessage = "Session paused"
    }
    
    // MARK: - ARSessionDelegate methods
    
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // Process camera image
        let cameraData = CameraData(
            imageBuffer: frame.capturedImage,
            cameraIntrinsics: frame.camera.intrinsics,
            cameraTransform: frame.camera.transform,
            timestamp: frame.timestamp
        )
        
        currentCameraData = cameraData
        cameraPublisher.send(cameraData)
        
        // Process device pose
        let pose = PoseData(
            position: frame.camera.transform.columns.3.xyz,
            orientation: simd_quatf(frame.camera.transform),
            timestamp: frame.timestamp
        )
        
        currentPose = pose
        posePublisher.send(pose)
        
        // Process depth data if available
        if let sceneDepth = frame.sceneDepth ?? frame.smoothedSceneDepth {
            let depthData = DepthData(
                depthMap: sceneDepth.depthMap,
                confidenceMap: sceneDepth.confidenceMap,
                timestamp: frame.timestamp
            )
            
            currentDepthData = depthData
            depthPublisher.send(depthData)
            
            // Generate point cloud from depth data
            processPointCloud(depthData: sceneDepth, cameraData: cameraData)
        }
    }
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        statusMessage = "Session failed: \(error.localizedDescription)"
        isRunning = false
    }
    
    func sessionWasInterrupted(_ session: ARSession) {
        statusMessage = "Session interrupted"
    }
    
    func sessionInterruptionEnded(_ session: ARSession) {
        statusMessage = "Session resumed"
        start() // Restart the session
    }
    
    func getSession() -> ARSession {
        return session
    }
    
    // MARK: - Helper methods
    
    private func processPointCloud(depthData: ARDepthData, cameraData: CameraData) {
        // This is a simplified implementation - in a real app, you would
        // generate a proper point cloud from the depth map using camera intrinsics
        
        // For demonstration, create a simple point cloud with a few points
        let points: [SIMD3<Float>] = [
            SIMD3<Float>(0, 0, 1),
            SIMD3<Float>(0.1, 0, 1),
            SIMD3<Float>(-0.1, 0, 1)
        ]
        
        let colors: [SIMD4<Float>] = [
            SIMD4<Float>(1, 0, 0, 1),  // Red
            SIMD4<Float>(0, 1, 0, 1),  // Green
            SIMD4<Float>(0, 0, 1, 1)   // Blue
        ]
        
        let pointCloud = PointCloudData(
            points: points,
            colors: colors,
            timestamp: cameraData.timestamp
        )
        
        currentPointCloud = pointCloud
        pointCloudPublisher.send(pointCloud)
    }
}

// Extension to get xyz from a vector
extension SIMD4 {
    var xyz: SIMD3<Scalar> {
        SIMD3(x, y, z)
    }
}

// SwiftUI view that wraps an ARView for displaying camera and AR content
struct ARViewContainer: UIViewRepresentable {
    @ObservedObject var framework = RoboticsFramework.shared
    
    func makeUIView(context: Context) -> ARView {
        // Create ARView
        let arView = ARView(frame: .zero)
        
        // Use the session from the framework
        arView.session = framework.getSession()
        
        // Add a coaching overlay to help the user find planes
        let coachingOverlay = ARCoachingOverlayView()
        coachingOverlay.session = framework.getSession()
        coachingOverlay.goal = .horizontalPlane
        coachingOverlay.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        arView.addSubview(coachingOverlay)
        
        // Add some debug visualizations
        #if DEBUG
        arView.debugOptions = [.showFeaturePoints, .showAnchorOrigins, .showAnchorGeometry]
        #endif
        
        return arView
    }
    
    func updateUIView(_ uiView: ARView, context: Context) {
        // Update view if needed
    }
}

// Main content view
struct ContentView: View {
    @ObservedObject var framework = RoboticsFramework.shared
    
    var body: some View {
        ZStack {
            // AR View takes the full screen
            ARViewContainer()
                .edgesIgnoringSafeArea(.all)
            
            // Overlay with status and controls
            VStack {
                Spacer()
                
                // Status panel
                VStack(spacing: 5) {
                    Text("Robotics Framework")
                        .font(.headline)
                    
                    Text(framework.statusMessage)
                        .font(.subheadline)
                    
                    if let depthData = framework.currentDepthData {
                        Text("Depth data available: \(String(format: "%.2f", depthData.timestamp))")
                            .font(.caption)
                    }
                    
                    if let pose = framework.currentPose {
                        Text("Position: \(String(format: "%.2f, %.2f, %.2f", pose.position.x, pose.position.y, pose.position.z))")
                            .font(.caption)
                    }
                    
                    // Control buttons
                    HStack(spacing: 20) {
                        Button(framework.isRunning ? "Stop" : "Start") {
                            if framework.isRunning {
                                framework.stop()
                            } else {
                                framework.start()
                            }
                        }
                        .padding()
                        .background(framework.isRunning ? Color.red : Color.green)
                        .foregroundColor(.white)
                        .cornerRadius(8)
                        
                        Button("Reset") {
                            framework.stop()
                            DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
                                framework.start()
                            }
                        }
                        .padding()
                        .background(Color.blue)
                        .foregroundColor(.white)
                        .cornerRadius(8)
                    }
                }
                .padding()
                .background(Color.black.opacity(0.7))
                .cornerRadius(12)
                .padding()
            }
        }
        .onAppear {
            // Start the framework when the view appears
            framework.start()
        }
        .onDisappear {
            // Stop the framework when the view disappears
            framework.stop()
        }
    }
}

// Preview for SwiftUI canvas
#Preview {
    ContentView()
}
