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
    @StateObject private var performanceMonitor = PerformanceMonitor()
    @State private var showControls = true
    @State private var showTip = true
    
    var body: some View {
        GeometryReader { geometry in
            VStack(spacing: 0) {
                // Top half - AR Camera Stream with Performance Overlay
                ZStack {
                    ARViewContainer(performanceMonitor: performanceMonitor)
                        .frame(height: geometry.size.height / 2)
                    
                    // Overlays
                    VStack {
                        HStack {
                            // Performance overlay in top-left
                            PerformanceOverlay(monitor: performanceMonitor)
                            Spacer()
                            // Status overlay in top-right
                            StatusOverlay(framework: framework)
                        }
                        Spacer()
                        
                        // Double-tap instruction overlay (only show briefly)
                        if !performanceMonitor.showMetrics && showTip {
                            HStack {
                                Text("Double-tap AR view for performance metrics")
                                    .font(.caption2)
                                    .foregroundColor(.white)
                                    .padding(6)
                                    .background(Color.black.opacity(0.6))
                                    .cornerRadius(6)
                                Spacer()
                            }
                            .padding(.bottom, 8)
                            .transition(.opacity)
                        }
                    }
                    .padding()
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
            
            // Hide tip after 4 seconds
            DispatchQueue.main.asyncAfter(deadline: .now() + 4) {
                withAnimation(.easeOut(duration: 0.5)) {
                    showTip = false
                }
            }
        }
        .onDisappear {
            // Stop the framework when the view disappears
            framework.stop()
        }
        .onChange(of: performanceMonitor.showMetrics) { _, newValue in
            // Hide tip when performance metrics are shown
            if newValue {
                withAnimation(.easeOut(duration: 0.3)) {
                    showTip = false
                }
            }
        }
    }
}

// MARK: - Preview

#Preview {
    ContentView()
}
