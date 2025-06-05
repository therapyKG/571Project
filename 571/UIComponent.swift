//
//  UIComponent.swift (Simplified for Lightweight Localization)
//  571
//
//  Created by Kurt Gu on 6/3/25.
//  Updated for lightweight ARKit-native localization
//

import SwiftUI
import Foundation

// MARK: - Status Overlay Component (Simplified)

struct StatusOverlay: View {
    @ObservedObject var framework: RoboticsFramework
    
    var body: some View {
        VStack(alignment: .trailing, spacing: 5) {
            // Main status message
            Text(framework.statusMessage)
                .font(.caption)
                .padding(8)
                .background(Color.black.opacity(0.7))
                .foregroundColor(.white)
                .cornerRadius(8)
            
            // ARKit tracking status (simplified)
            VStack(alignment: .trailing, spacing: 2) {
                // Tracking quality indicator
                HStack(spacing: 4) {
                    Text("ARKit:")
                        .font(.caption2)
                    Text(framework.trackingQuality)
                        .font(.caption2)
                        .bold()
                }
                .padding(4)
                .background(trackingColor)
                .foregroundColor(.white)
                .cornerRadius(4)
                
                // Tracking reliability indicator
                HStack(spacing: 4) {
                    Text("Reliable:")
                        .font(.caption2)
                    Text(framework.isTrackingReliable ? "✅" : "❌")
                        .font(.caption2)
                }
                .padding(4)
                .background(framework.isTrackingReliable ? Color.green.opacity(0.7) : Color.red.opacity(0.7))
                .foregroundColor(.white)
                .cornerRadius(4)
            }
            
            // Position and heading info (only if tracking is reliable)
            if framework.isTrackingReliable, let pose = framework.currentPose {
                VStack(alignment: .trailing, spacing: 2) {
                    Text("Pos: \(String(format: "%.2f, %.2f, %.2f", pose.position.x, pose.position.y, pose.position.z))")
                        .font(.caption2)
                        .padding(4)
                        .background(Color.black.opacity(0.7))
                        .foregroundColor(.white)
                        .cornerRadius(4)
                    
                    Text("Heading: \(String(format: "%.1f°", framework.currentHeading * 180 / Float.pi))")
                        .font(.caption2)
                        .padding(4)
                        .background(Color.black.opacity(0.7))
                        .foregroundColor(.white)
                        .cornerRadius(4)
                    
                    // Robot position in map coordinates
                    let robotPos = framework.localizationManager.currentRobotPosition
                    Text("Map pos: (\(robotPos.x), \(robotPos.y))")
                        .font(.caption2)
                        .padding(4)
                        .background(Color.blue.opacity(0.7))
                        .foregroundColor(.white)
                        .cornerRadius(4)
                }
            }
            
            // LiDAR-specific info (simplified)
            if framework.hasLiDAR {
                VStack(alignment: .trailing, spacing: 2) {
                    // Floor detection status
                    HStack(spacing: 4) {
                        Text("Floor:")
                            .font(.caption2)
                        Text(framework.floorDetected ? "✅" : "❌")
                            .font(.caption2)
                    }
                    .padding(4)
                    .background(framework.floorDetected ? Color.green.opacity(0.7) : Color.red.opacity(0.7))
                    .foregroundColor(.white)
                    .cornerRadius(4)
                    
                    // Performance metrics (only show if tracking is reliable)
                    if framework.isTrackingReliable {
                        Text("Obstacles: \(framework.obstaclePointsFiltered)")
                            .font(.caption2)
                            .padding(4)
                            .background(Color.blue.opacity(0.7))
                            .foregroundColor(.white)
                            .cornerRadius(4)
                        
                        Text("Updates: \(framework.mapUpdates)")
                            .font(.caption2)
                            .padding(4)
                            .background(Color.purple.opacity(0.7))
                            .foregroundColor(.white)
                            .cornerRadius(4)
                    }
                    
                    // Frontier exploration status
                    if framework.isTrackingReliable {
                        if let frontierTarget = framework.occupancyMapData.frontierTarget {
                            Text("Target: (\(frontierTarget.x), \(frontierTarget.y))")
                                .font(.caption2)
                                .padding(4)
                                .background(Color.red.opacity(0.7))
                                .foregroundColor(.white)
                                .cornerRadius(4)
                        } else {
                            Text("Exploration: Complete")
                                .font(.caption2)
                                .padding(4)
                                .background(Color.green.opacity(0.7))
                                .foregroundColor(.white)
                                .cornerRadius(4)
                        }
                    } else {
                        Text("Exploration: Paused")
                            .font(.caption2)
                            .padding(4)
                            .background(Color.gray.opacity(0.7))
                            .foregroundColor(.white)
                            .cornerRadius(4)
                    }
                }
            }
            
            // Debug info (simplified)
            if framework.debugMode == 0 {
                VStack(alignment: .trailing, spacing: 2) {
                    Text("Camera: \(framework.currentCameraType)")
                        .font(.system(size: 10))
                        .padding(2)
                        .background(Color.gray.opacity(0.7))
                        .foregroundColor(.white)
                        .cornerRadius(3)
                    
                    Text("Debug mode: \(framework.debugMode)")
                        .font(.system(size: 10))
                        .padding(2)
                        .background(Color.gray.opacity(0.7))
                        .foregroundColor(.white)
                        .cornerRadius(3)
                }
            }
        }
        .padding()
    }
    
    private var trackingColor: Color {
        switch framework.trackingQuality.lowercased() {
        case "normal":
            return Color.green.opacity(0.7)
        case let quality where quality.contains("initializ"):
            return Color.yellow.opacity(0.7)
        case let quality where quality.contains("insufficient"), let quality where quality.contains("excessive"):
            return Color.orange.opacity(0.7)
        case let quality where quality.contains("relocal"):
            return Color.blue.opacity(0.7)
        case let quality where quality.contains("not available"):
            return Color.red.opacity(0.7)
        default:
            return Color.gray.opacity(0.7)
        }
    }
}

// MARK: - Control Buttons Component (Simplified)

struct ControlButtons: View {
    @ObservedObject var framework: RoboticsFramework
    
    var body: some View {
        VStack(spacing: 10) {
            // Primary controls
            HStack(spacing: 15) {
                // Start/Stop button
                Button(framework.isRunning ? "Stop" : "Start") {
                    if framework.isRunning {
                        framework.stop()
                    } else {
                        framework.start()
                    }
                }
                .padding(.horizontal, 16)
                .padding(.vertical, 8)
                .background(framework.isRunning ? Color.red : Color.green)
                .foregroundColor(.white)
                .cornerRadius(6)
                
                // Reset session button
                Button("Reset") {
                    framework.reset()
                }
                .padding(.horizontal, 16)
                .padding(.vertical, 8)
                .background(Color.blue)
                .foregroundColor(.white)
                .cornerRadius(6)
                
                // Clear map button (enabled when tracking is reliable)
                Button("Clear Map") {
                    framework.clearOccupancyMap()
                }
                .padding(.horizontal, 16)
                .padding(.vertical, 8)
                .background(framework.isTrackingReliable ? Color.purple : Color.gray)
                .foregroundColor(.white)
                .cornerRadius(6)
                .disabled(!framework.isTrackingReliable)
            }
            
            // Secondary controls
            HStack(spacing: 15) {
                // Debug mode toggle button
                Button("Debug (\(framework.debugMode))") {
                    framework.toggleDebugMode()
                }
                .padding(.horizontal, 16)
                .padding(.vertical, 8)
                .background(Color.orange)
                .foregroundColor(.white)
                .cornerRadius(6)
                
                // Performance indicator
                Text("⚡ Lightweight Mode")
                    .font(.caption2)
                    .padding(.horizontal, 16)
                    .padding(.vertical, 8)
                    .background(Color.green.opacity(0.3))
                    .foregroundColor(.green)
                    .cornerRadius(6)
            }
        }
        .padding(.bottom, 16)
    }
}

// MARK: - ARKit Performance Panel (New Component)

struct ARKitPerformancePanel: View {
    @ObservedObject var localizationManager: LightweightLocalizationManager
    
    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("ARKit Performance")
                    .font(.caption.bold())
                    .foregroundColor(.white)
                Spacer()
            }
            
            // Tracking status
            HStack {
                Text("Tracking:")
                    .font(.caption2)
                    .foregroundColor(.white)
                Spacer()
                Text(localizationManager.trackingQuality)
                    .font(.caption2)
                    .foregroundColor(localizationManager.isTrackingReliable ? .green : .red)
            }
            
            // Robot position
            HStack {
                Text("Robot:")
                    .font(.caption2)
                    .foregroundColor(.white)
                Spacer()
                let pos = localizationManager.currentRobotPosition
                Text("(\(pos.x), \(pos.y))")
                    .font(.caption2.monospacedDigit())
                    .foregroundColor(.cyan)
            }
            
            // Heading
            HStack {
                Text("Heading:")
                    .font(.caption2)
                    .foregroundColor(.white)
                Spacer()
                Text("\(String(format: "%.1f°", localizationManager.currentHeading * 180 / Float.pi))")
                    .font(.caption2.monospacedDigit())
                    .foregroundColor(.cyan)
            }
            
            // Performance indicator
            HStack {
                Text("Mode:")
                    .font(.caption2)
                    .foregroundColor(.white)
                Spacer()
                Text("⚡ Native ARKit")
                    .font(.caption2)
                    .foregroundColor(.green)
            }
        }
        .padding(8)
        .background(Color.black.opacity(0.7))
        .cornerRadius(8)
        .frame(width: 200)
    }
}

// MARK: - Controls Visibility Toggle (unchanged)

struct ControlsToggleButton: View {
    @Binding var showControls: Bool
    
    var body: some View {
        VStack {
            HStack {
                Spacer()
                Button(action: {
                    withAnimation {
                        showControls.toggle()
                    }
                }) {
                    Image(systemName: showControls ? "eye.slash" : "eye")
                        .padding(8)
                        .background(Color.black.opacity(0.5))
                        .foregroundColor(.white)
                        .clipShape(Circle())
                }
                .padding()
            }
            Spacer()
        }
    }
}

// MARK: - Performance Comparison View (New)

struct PerformanceComparisonView: View {
    @ObservedObject var framework: RoboticsFramework
    
    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("Performance Stats")
                    .font(.caption.bold())
                    .foregroundColor(.white)
                Spacer()
                Text("⚡ Fast Mode")
                    .font(.caption2)
                    .foregroundColor(.green)
            }
            
            if framework.hasLiDAR && framework.isTrackingReliable {
                VStack(alignment: .leading, spacing: 4) {
                    HStack {
                        Text("Frame Rate:")
                            .font(.caption2)
                            .foregroundColor(.white)
                        Spacer()
                        Text("~60 FPS")
                            .font(.caption2)
                            .foregroundColor(.green)
                    }
                    
                    HStack {
                        Text("Depth Sampling:")
                            .font(.caption2)
                            .foregroundColor(.white)
                        Spacer()
                        Text("Every 8th pixel")
                            .font(.caption2)
                            .foregroundColor(.yellow)
                    }
                    
                    HStack {
                        Text("Processing:")
                            .font(.caption2)
                            .foregroundColor(.white)
                        Spacer()
                        Text("ARKit Native")
                            .font(.caption2)
                            .foregroundColor(.green)
                    }
                }
            } else {
                Text("Enable LiDAR tracking to see performance stats")
                    .font(.caption2)
                    .foregroundColor(.gray)
            }
        }
        .padding(8)
        .background(Color.black.opacity(0.7))
        .cornerRadius(8)
        .frame(width: 220)
    }
}

// MARK: - Previews

#Preview("Simplified Status Overlay") {
    StatusOverlay(framework: RoboticsFramework.shared)
        .background(Color.blue.opacity(0.3))
}

#Preview("Simplified Control Buttons") {
    ControlButtons(framework: RoboticsFramework.shared)
        .background(Color.gray.opacity(0.2))
}

#Preview("ARKit Performance Panel") {
    ARKitPerformancePanel(localizationManager: LightweightLocalizationManager())
        .background(Color.blue.opacity(0.3))
}

#Preview("Performance Comparison") {
    PerformanceComparisonView(framework: RoboticsFramework.shared)
        .background(Color.blue.opacity(0.3))
}

#Preview("Controls Toggle") {
    ControlsToggleButton(showControls: .constant(true))
        .frame(width: 200, height: 100)
        .background(Color.gray.opacity(0.2))
}
