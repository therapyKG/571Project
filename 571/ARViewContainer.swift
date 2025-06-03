//
//  ARViewContainer.swift
//  MobileRobotics
//
//  Created by Kurt Gu on 5/5/25.
//

import SwiftUI
import RealityKit
import ARKit
import Foundation

// Performance monitoring class
class PerformanceMonitor: ObservableObject {
    @Published var cpuUsage: Double = 0.0
    @Published var perCoreCpuUsage: [Double] = []
    @Published var coreCount: Int = 0
    @Published var memoryUsage: (used: Double, total: Double) = (0, 0)
    @Published var fps: Double = 0.0
    @Published var showMetrics: Bool = false
    
    private var timer: Timer?
    private var lastFrameTime: CFTimeInterval = 0
    private var frameCount = 0
    
    init() {
        // Get core count
        coreCount = ProcessInfo.processInfo.processorCount
        perCoreCpuUsage = Array(repeating: 0.0, count: coreCount)
        startMonitoring()
    }
    
    deinit {
        stopMonitoring()
    }
    
    func startMonitoring() {
        timer = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { [weak self] _ in
            self?.updateMetrics()
        }
    }
    
    func stopMonitoring() {
        timer?.invalidate()
        timer = nil
    }
    
    func toggleMetrics() {
        showMetrics.toggle()
    }
    
    func updateFrameRate() {
        let currentTime = CACurrentMediaTime()
        if lastFrameTime > 0 {
            let deltaTime = currentTime - lastFrameTime
            if deltaTime > 0 {
                let newFPS = 1.0 / deltaTime
                // Update on next run loop to avoid publishing during view updates
                DispatchQueue.main.async { [weak self] in
                    self?.fps = newFPS
                }
            }
        }
        lastFrameTime = currentTime
    }
    
    private func updateMetrics() {
        DispatchQueue.global(qos: .background).async { [weak self] in
            let (totalCpu, perCoreCpu) = self?.getCPUUsage() ?? (0.0, [])
            let memory = self?.getMemoryUsage() ?? (0, 0)
            
            DispatchQueue.main.async {
                self?.cpuUsage = totalCpu
                self?.perCoreCpuUsage = perCoreCpu
                self?.memoryUsage = memory
            }
        }
    }
    
    private func getCPUUsage() -> (total: Double, perCore: [Double]) {
        var info = mach_task_basic_info()
        var count = mach_msg_type_number_t(MemoryLayout<mach_task_basic_info>.size)/4
        
        let kerr: kern_return_t = withUnsafeMutablePointer(to: &info) {
            $0.withMemoryRebound(to: integer_t.self, capacity: 1) {
                task_info(mach_task_self_,
                         task_flavor_t(MACH_TASK_BASIC_INFO),
                         $0,
                         &count)
            }
        }
        
        if kerr == KERN_SUCCESS {
            var threadsList: thread_act_array_t?
            var threadsCount = mach_msg_type_number_t(0)
            
            let kret = task_threads(mach_task_self_, &threadsList, &threadsCount)
            
            if kret == KERN_SUCCESS, let threads = threadsList {
                var totalCPU: Double = 0
                var coreUsage = Array(repeating: 0.0, count: coreCount)
                
                for i in 0..<threadsCount {
                    var threadInfo = thread_basic_info()
                    var threadInfoCount = mach_msg_type_number_t(THREAD_INFO_MAX)
                    
                    let threadRet = withUnsafeMutablePointer(to: &threadInfo) {
                        $0.withMemoryRebound(to: integer_t.self, capacity: 1) {
                            thread_info(threads[Int(i)], thread_flavor_t(THREAD_BASIC_INFO), $0, &threadInfoCount)
                        }
                    }
                    
                    if threadRet == KERN_SUCCESS {
                        if threadInfo.flags & TH_FLAGS_IDLE == 0 {
                            let cpuUsage = Double(threadInfo.cpu_usage) / Double(TH_USAGE_SCALE) * 100.0
                            totalCPU += cpuUsage
                            
                            // Distribute thread usage across cores (approximation)
                            // Since we can't easily get exact core assignment, we'll distribute evenly
                            let coreIndex = Int(i) % coreCount
                            coreUsage[coreIndex] += cpuUsage
                        }
                    }
                }
                
                let deallocRet = vm_deallocate(mach_task_self_, vm_address_t(bitPattern: threads), vm_size_t(Int(threadsCount) * MemoryLayout<thread_t>.size))
                if deallocRet != KERN_SUCCESS {
                    print("Failed to deallocate threads array")
                }
                
                return (totalCPU, coreUsage) // No clamping - allow >100%
            }
        }
        return (0.0, Array(repeating: 0.0, count: coreCount))
    }
    
    private func getMemoryUsage() -> (used: Double, total: Double) {
        var info = mach_task_basic_info()
        var count = mach_msg_type_number_t(MemoryLayout<mach_task_basic_info>.size)/4
        
        let kerr: kern_return_t = withUnsafeMutablePointer(to: &info) {
            $0.withMemoryRebound(to: integer_t.self, capacity: 1) {
                task_info(mach_task_self_,
                         task_flavor_t(MACH_TASK_BASIC_INFO),
                         $0,
                         &count)
            }
        }
        
        if kerr == KERN_SUCCESS {
            let usedMemory = Double(info.resident_size) / 1024.0 / 1024.0 // Convert to MB
            let totalMemory = Double(ProcessInfo.processInfo.physicalMemory) / 1024.0 / 1024.0 / 1024.0 // Convert to GB
            
            // Ensure reasonable values
            let clampedUsed = max(0, usedMemory)
            let clampedTotal = max(0.1, totalMemory) // Prevent division by zero
            
            return (clampedUsed, clampedTotal)
        }
        
        return (0, 0)
    }
}

// Performance metrics overlay
struct PerformanceOverlay: View {
    @ObservedObject var monitor: PerformanceMonitor
    
    var body: some View {
        if monitor.showMetrics {
            VStack(alignment: .leading, spacing: 4) {
                HStack {
                    Text("Performance")
                        .font(.caption.bold())
                        .foregroundColor(.white)
                    Spacer()
                    Button("×") {
                        monitor.toggleMetrics()
                    }
                    .font(.caption.bold())
                    .foregroundColor(.white)
                }
                
                Text("CPU: \(String(format: "%.1f", monitor.cpuUsage))% (\(monitor.coreCount) cores)")
                    .font(.caption2)
                    .foregroundColor(.white)
                
                // Per-core CPU usage (compact display)
                if monitor.perCoreCpuUsage.count > 0 {
                    HStack(spacing: 2) {
                        ForEach(0..<min(monitor.perCoreCpuUsage.count, 8), id: \.self) { index in
                            VStack(spacing: 1) {
                                Text("\(index)")
                                    .font(.system(size: 8))
                                    .foregroundColor(.white.opacity(0.7))
                                Rectangle()
                                    .fill(coreColor(for: monitor.perCoreCpuUsage[index]))
                                    .frame(width: 12, height: max(2, min(20, CGFloat(monitor.perCoreCpuUsage[index] / 25.0) * 20)))
                            }
                        }
                        if monitor.perCoreCpuUsage.count > 8 {
                            Text("...")
                                .font(.system(size: 8))
                                .foregroundColor(.white.opacity(0.7))
                        }
                    }
                }
                
                Text("RAM: \(String(format: "%.1f", monitor.memoryUsage.used))MB / \(String(format: "%.1f", monitor.memoryUsage.total))GB")
                    .font(.caption2)
                    .foregroundColor(.white)
                
                Text("FPS: \(String(format: "%.1f", monitor.fps))")
                    .font(.caption2)
                    .foregroundColor(.white)
                
                // CPU usage bar (scales with core count - 100% per core)
                let maxCpuPercent = Double(monitor.coreCount) * 100.0
                let cpuProgress = monitor.cpuUsage / maxCpuPercent
                ProgressView(value: min(max(cpuProgress, 0.0), 1.0))
                    .progressViewStyle(LinearProgressViewStyle(tint: cpuColor))
                    .frame(height: 4)
                
                // Memory usage bar
                let memoryPercent = monitor.memoryUsage.total > 0 ?
                    min(max(monitor.memoryUsage.used / (monitor.memoryUsage.total * 1024.0), 0.0), 1.0) : 0.0
                ProgressView(value: memoryPercent)
                    .progressViewStyle(LinearProgressViewStyle(tint: memoryColor))
                    .frame(height: 4)
            }
            .padding(8)
            .background(Color.black.opacity(0.7))
            .cornerRadius(8)
            .frame(width: 200)
        }
    }
    
    private var cpuColor: Color {
        // Color based on percentage of total possible CPU (cores * 100%)
        let maxPossible = Double(monitor.coreCount) * 100.0
        let percentage = (monitor.cpuUsage / maxPossible) * 100.0
        
        switch percentage {
        case 0..<50: return .green
        case 50..<80: return .yellow
        default: return .red
        }
    }
    
    private func coreColor(for usage: Double) -> Color {
        switch usage {
        case 0..<25: return .green
        case 25..<50: return .yellow
        case 50..<75: return .orange
        default: return .red
        }
    }
    
    private var memoryColor: Color {
        guard monitor.memoryUsage.total > 0 else { return .green }
        let memoryPercentage = (monitor.memoryUsage.used / (monitor.memoryUsage.total * 1024.0)) * 100.0
        switch memoryPercentage {
        case 0..<60: return .green
        case 60..<80: return .yellow
        default: return .red
        }
    }
}

// SwiftUI view that wraps an ARView for displaying camera and AR content
struct ARViewContainer: UIViewRepresentable {
    @ObservedObject var framework = RoboticsFramework.shared
    var performanceMonitor: PerformanceMonitor?
    
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
        
        // Add double-tap gesture to toggle performance metrics
        let tapGesture = UITapGestureRecognizer(target: context.coordinator, action: #selector(Coordinator.handleDoubleTap))
        tapGesture.numberOfTapsRequired = 2
        arView.addGestureRecognizer(tapGesture)
        
        // Store reference to monitor in coordinator
        context.coordinator.performanceMonitor = performanceMonitor
        
        return arView
    }
    
    func updateUIView(_ uiView: ARView, context: Context) {
        // Update debug options when debug mode changes
        updateDebugOptions(arView: uiView, debugMode: framework.debugMode)
        
        // Update FPS
        performanceMonitor?.updateFrameRate()
    }
    
    func makeCoordinator() -> Coordinator {
        Coordinator()
    }
    
    // MARK: - Coordinator
    
    class Coordinator: NSObject {
        var performanceMonitor: PerformanceMonitor?
        
        @objc func handleDoubleTap() {
            performanceMonitor?.toggleMetrics()
        }
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
