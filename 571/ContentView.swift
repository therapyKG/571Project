//
//  ContentView.swift
//  MobileRobotics
//
//  Created by Kurt Gu on 5/5/25.
//

import SwiftUI

// Main content view with vertical split layout
struct ContentView: View {
    @ObservedObject var framework = RoboticsFramework.shared
    @State private var showControls = true
    
    var body: some View {
        GeometryReader { geometry in
            VStack(spacing: 0) {
                // Top half - AR Camera Stream
                ARViewContainer()
                    .frame(height: geometry.size.height / 2)
                    .overlay(alignment: .topTrailing) {
                        StatusOverlay(framework: framework)
                    }
                
                // Bottom half - Occupancy Map with Controls
                ZStack {
                    // Background for the entire bottom half
                    Color.black.opacity(0.05)
                    
                    // Occupancy map visualization
                    OccupancyMapView(
                        mapData: framework.occupancyMapData,
                        robotHeading: framework.currentHeading
                    )
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
                    .padding(4)
                    
                    // Control buttons overlay (conditionally visible)
                    if showControls {
                        VStack {
                            Spacer()
                            ControlButtons(framework: framework)
                        }
                    }
                    
                    // Toggle controls visibility button
                    ControlsToggleButton(showControls: $showControls)
                }
                .frame(height: geometry.size.height / 2)
            }
        }
        .edgesIgnoringSafeArea(.all)
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

// MARK: - Preview

#Preview {
    ContentView()
}
