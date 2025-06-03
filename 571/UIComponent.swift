//
//  UIComponent.swift
//  571
//
//  Created by Kurt Gu on 6/3/25.
//

import SwiftUI
import Foundation

// MARK: - Status Overlay Component

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
            
            // Position and heading info
            if let pose = framework.currentPose {
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
                }
            }
            
            // LiDAR-specific debug info
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
                    
                    // Obstacle points count
                    Text("Obstacles: \(framework.obstaclePointsFiltered)")
                        .font(.caption2)
                        .padding(4)
                        .background(Color.blue.opacity(0.7))
                        .foregroundColor(.white)
                        .cornerRadius(4)
                    
                    // Map updates count
                    Text("Updates: \(framework.mapUpdates)")
                        .font(.caption2)
                        .padding(4)
                        .background(Color.purple.opacity(0.7))
                        .foregroundColor(.white)
                        .cornerRadius(4)
                    
                    // Map generation ID for debugging
                    Text("Gen: \(framework.mapGenerationID)")
                        .font(.caption2)
                        .padding(4)
                        .background(Color.orange.opacity(0.7))
                        .foregroundColor(.white)
                        .cornerRadius(4)
                    
                    // Frontier exploration status
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
                }
            }
        }
        .padding()
    }
}

// MARK: - Control Buttons Component

struct ControlButtons: View {
    @ObservedObject var framework: RoboticsFramework
    
    var body: some View {
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
            
            // Clear map button
            Button("Clear Map") {
                framework.clearOccupancyMap()
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 8)
            .background(Color.purple)
            .foregroundColor(.white)
            .cornerRadius(6)
            
            // Debug mode toggle button
            Button("Debug") {
                framework.toggleDebugMode()
            }
            .padding(.horizontal, 16)
            .padding(.vertical, 8)
            .background(Color.orange)
            .foregroundColor(.white)
            .cornerRadius(6)
        }
        .padding(.bottom, 16)
    }
}

// MARK: - Controls Visibility Toggle

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

// MARK: - Previews

#Preview("Status Overlay") {
    StatusOverlay(framework: RoboticsFramework.shared)
        .background(Color.blue.opacity(0.3))
}

#Preview("Control Buttons") {
    ControlButtons(framework: RoboticsFramework.shared)
        .background(Color.gray.opacity(0.2))
}

#Preview("Controls Toggle") {
    ControlsToggleButton(showControls: .constant(true))
        .frame(width: 200, height: 100)
        .background(Color.gray.opacity(0.2))
}
