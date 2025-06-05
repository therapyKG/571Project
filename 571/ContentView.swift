//
//  ContentView.swift (Updated for Lightweight Localization)
//  MobileRobotics
//
//  Created by Kurt Gu on 5/5/25.
//  Updated for lightweight ARKit-native localization
//

import SwiftUI

// Main content view with vertical split layout
struct ContentView: View {
    @ObservedObject var framework = RoboticsFramework.shared
    @StateObject private var performanceMonitor = PerformanceMonitor()
    @State private var showControls = true
    @State private var showTip = true
    @State private var showPerformancePanel = false
    
    var body: some View {
        GeometryReader { geometry in
            VStack(spacing: 0) {
                // Top half - AR Camera Stream with Overlays
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
                        
                        // Optional performance comparison panel
                        if showPerformancePanel {
                            HStack {
                                VStack(spacing: 8) {
                                    ARKitPerformancePanel(localizationManager: framework.localizationManager)
                                    PerformanceComparisonView(framework: framework)
                                }
                                Spacer()
                            }
                            .padding(.leading)
                        }
                        
                        Spacer()
                        
                        // Performance tip overlay
                        if !performanceMonitor.showMetrics && showTip {
                            VStack(spacing: 4) {
                                HStack {
                                    Text("⚡ Lightweight ARKit Mode Active")
                                        .font(.caption2.bold())
                                        .foregroundColor(.green)
                                        .padding(6)
                                        .background(Color.black.opacity(0.6))
                                        .cornerRadius(6)
                                    Spacer()
                                }
                                
                                HStack {
                                    Text("Double-tap AR view for device performance metrics")
                                        .font(.caption2)
                                        .foregroundColor(.white)
                                        .padding(6)
                                        .background(Color.black.opacity(0.6))
                                        .cornerRadius(6)
                                    Spacer()
                                }
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
                    
                    // Metal-accelerated occupancy map visualization
                    MetalOccupancyMapView(
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
                    
                    // Performance panel toggle (top-right of map view)
                    VStack {
                        HStack {
                            Spacer()
                            Button(action: {
                                withAnimation(.easeInOut(duration: 0.3)) {
                                    showPerformancePanel.toggle()
                                }
                            }) {
                                Image(systemName: showPerformancePanel ? "chart.bar.fill" : "chart.bar")
                                    .padding(8)
                                    .background(Color.blue.opacity(0.7))
                                    .foregroundColor(.white)
                                    .clipShape(Circle())
                            }
                            .padding(.trailing)
                            .padding(.top, 8)
                        }
                        Spacer()
                    }
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
            
            // Hide tip after 5 seconds (longer since we have more text)
            DispatchQueue.main.asyncAfter(deadline: .now() + 5) {
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
    }}
