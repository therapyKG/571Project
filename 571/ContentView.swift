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
    @Published var hasUltrawide = false
    @Published var currentCameraType = "Unknown"
    @Published var statusMessage = "Not initialized"
    @Published var debugMode = 0 // 0: standard, 1: no scene reconstruction, 2: minimal debug
    
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
        
        // Log configuration details
        print("📱 Device: \(UIDevice.current.model)")
        print("🔧 LiDAR: \(hasLiDAR)")
        print("📹 Camera: \(currentCameraType)")
        print("🎯 Frame semantics: \(config.frameSemantics)")
        print("🔍 Scene reconstruction: \(config.sceneReconstruction)")
        
        return config
    }
    
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
            statusMessage = "LiDAR enabled, using \(currentCameraType) camera"
        } else {
            statusMessage = "Using \(currentCameraType) camera (no LiDAR)"
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
                    self.statusMessage = "LiDAR enabled, using \(self.currentCameraType) camera (reset)"
                } else {
                    self.statusMessage = "Using \(self.currentCameraType) camera (no LiDAR) (reset)"
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
        }
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
    
    // MARK: - ARSessionDelegate methods
    
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        // Check tracking quality and provide feedback
        let trackingState = frame.camera.trackingState
        
        switch trackingState {
        case .normal:
            // Only update status if it's currently showing tracking issues
            if statusMessage.contains("tracking") {
                DispatchQueue.main.async {
                    if self.hasLiDAR {
                        self.statusMessage = "LiDAR enabled, using \(self.currentCameraType) camera"
                    } else {
                        self.statusMessage = "Using \(self.currentCameraType) camera (no LiDAR)"
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
        
        // Process camera image
        let cameraData = CameraData(
            imageBuffer: frame.capturedImage,
            cameraIntrinsics: frame.camera.intrinsics,
            cameraTransform: frame.camera.transform,
            timestamp: frame.timestamp
        )
        
        DispatchQueue.main.async {
            self.currentCameraData = cameraData
        }
        cameraPublisher.send(cameraData)
        
        // Process device pose
        let pose = PoseData(
            position: frame.camera.transform.columns.3.xyz,
            orientation: simd_quatf(frame.camera.transform),
            timestamp: frame.timestamp
        )
        
        DispatchQueue.main.async {
            self.currentPose = pose
        }
        posePublisher.send(pose)
        
        // Process depth data if available
        if let sceneDepth = frame.sceneDepth ?? frame.smoothedSceneDepth {
            let depthData = DepthData(
                depthMap: sceneDepth.depthMap,
                confidenceMap: sceneDepth.confidenceMap,
                timestamp: frame.timestamp
            )
            
            DispatchQueue.main.async {
                self.currentDepthData = depthData
            }
            depthPublisher.send(depthData)
            
            // Generate point cloud from depth data
            processPointCloud(depthData: sceneDepth, cameraData: cameraData)
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
        
        DispatchQueue.main.async {
            self.currentPointCloud = pointCloud
        }
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
        
        // Add some debug visualizations based on debug mode
        #if DEBUG
        switch framework.debugMode {
        case 0:
            arView.debugOptions = [.showFeaturePoints, .showAnchorOrigins, .showAnchorGeometry]
        case 1:
            arView.debugOptions = [.showFeaturePoints, .showAnchorOrigins, .showAnchorGeometry]
        case 2:
            arView.debugOptions = [.showFeaturePoints]
        default:
            arView.debugOptions = []
        }
        #endif
        
        return arView
    }
    
    func updateUIView(_ uiView: ARView, context: Context) {
        // Update debug options when debug mode changes
        #if DEBUG
        switch framework.debugMode {
        case 0:
            uiView.debugOptions = [.showFeaturePoints, .showAnchorOrigins, .showAnchorGeometry]
        case 1:
            uiView.debugOptions = [.showFeaturePoints, .showAnchorOrigins, .showAnchorGeometry]
        case 2:
            uiView.debugOptions = [.showFeaturePoints]
        default:
            uiView.debugOptions = []
        }
        #endif
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
                    
                    Text("Camera: \(framework.currentCameraType)")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    
                    Text("Debug Mode: \(framework.debugMode)")
                        .font(.caption)
                        .foregroundColor(.secondary)
                    
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
                            framework.reset()
                        }
                        .padding()
                        .background(Color.blue)
                        .foregroundColor(.white)
                        .cornerRadius(8)
                        
                        Button("Debug") {
                            framework.toggleDebugMode()
                        }
                        .padding()
                        .background(Color.orange)
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
            // Only start if not already running
            if !framework.isRunning {
                framework.start()
            }
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
