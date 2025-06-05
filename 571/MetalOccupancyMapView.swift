//
//  MetalOccupancyMapView.swift
//  MobileRobotics
//
//  Metal-accelerated occupancy map renderer
//

import SwiftUI
import MetalKit
import Metal
import simd

// MARK: - Metal View Wrapper
struct MetalOccupancyMapView: UIViewRepresentable {
    let mapData: OccupancyMapData
    let robotHeading: Float
    
    func makeUIView(context: Context) -> MTKView {
        let metalView = MTKView()
        metalView.device = MTLCreateSystemDefaultDevice()
        metalView.clearColor = MTLClearColor(red: 0.1, green: 0.1, blue: 0.1, alpha: 1.0)
        metalView.delegate = context.coordinator
        metalView.enableSetNeedsDisplay = true
        metalView.isPaused = false
        metalView.preferredFramesPerSecond = 30
        
        // Initialize the renderer
        context.coordinator.initialize(metalView: metalView)
        
        return metalView
    }
    
    func updateUIView(_ metalView: MTKView, context: Context) {
        // Update the map data in the coordinator
        context.coordinator.updateMapData(mapData, robotHeading: robotHeading)
        metalView.setNeedsDisplay()
    }
    
    func makeCoordinator() -> Coordinator {
        Coordinator()
    }
    
    // MARK: - Coordinator (Metal Renderer)
    class Coordinator: NSObject, MTKViewDelegate {
        private var device: MTLDevice?
        private var commandQueue: MTLCommandQueue?
        private var pipelineState: MTLRenderPipelineState?
        private var vertexBuffer: MTLBuffer?
        private var mapTexture: MTLTexture?
        private var robotPipelineState: MTLRenderPipelineState?
        
        // Map data
        private var currentMapData: OccupancyMapData?
        private var currentRobotHeading: Float = 0.0
        
        // Viewport parameters
        private let visibleRadius = 50
        
        func initialize(metalView: MTKView) {
            guard let device = metalView.device else { return }
            self.device = device
            self.commandQueue = device.makeCommandQueue()
            
            setupPipeline()
            setupVertexBuffer()
            setupMapTexture()
        }
        
        private func setupPipeline() {
            guard let device = device else { return }
            
            let library = device.makeDefaultLibrary()
            let vertexFunction = library?.makeFunction(name: "mapVertexShader")
            let fragmentFunction = library?.makeFunction(name: "mapFragmentShader")
            let robotVertexFunction = library?.makeFunction(name: "robotVertexShader")
            let robotFragmentFunction = library?.makeFunction(name: "robotFragmentShader")
            
            // Map pipeline
            let pipelineDescriptor = MTLRenderPipelineDescriptor()
            pipelineDescriptor.vertexFunction = vertexFunction
            pipelineDescriptor.fragmentFunction = fragmentFunction
            pipelineDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
            
            do {
                pipelineState = try device.makeRenderPipelineState(descriptor: pipelineDescriptor)
            } catch {
                print("Failed to create map pipeline state: \(error)")
            }
            
            // Robot pipeline (with alpha blending)
            let robotPipelineDescriptor = MTLRenderPipelineDescriptor()
            robotPipelineDescriptor.vertexFunction = robotVertexFunction
            robotPipelineDescriptor.fragmentFunction = robotFragmentFunction
            robotPipelineDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
            robotPipelineDescriptor.colorAttachments[0].isBlendingEnabled = true
            robotPipelineDescriptor.colorAttachments[0].rgbBlendOperation = .add
            robotPipelineDescriptor.colorAttachments[0].alphaBlendOperation = .add
            robotPipelineDescriptor.colorAttachments[0].sourceRGBBlendFactor = .sourceAlpha
            robotPipelineDescriptor.colorAttachments[0].sourceAlphaBlendFactor = .sourceAlpha
            robotPipelineDescriptor.colorAttachments[0].destinationRGBBlendFactor = .oneMinusSourceAlpha
            robotPipelineDescriptor.colorAttachments[0].destinationAlphaBlendFactor = .oneMinusSourceAlpha
            
            do {
                robotPipelineState = try device.makeRenderPipelineState(descriptor: robotPipelineDescriptor)
            } catch {
                print("Failed to create robot pipeline state: \(error)")
            }
        }
        
        private func setupVertexBuffer() {
            guard let device = device else { return }
            
            // Full screen quad vertices
            let vertices: [Float] = [
                -1.0, -1.0, 0.0, 1.0,  // bottom left
                 1.0, -1.0, 1.0, 1.0,  // bottom right
                -1.0,  1.0, 0.0, 0.0,  // top left
                 1.0,  1.0, 1.0, 0.0,  // top right
            ]
            
            vertexBuffer = device.makeBuffer(bytes: vertices,
                                           length: vertices.count * MemoryLayout<Float>.size,
                                           options: [])
        }
        
        private func setupMapTexture() {
            guard let device = device else { return }
            
            let textureDescriptor = MTLTextureDescriptor.texture2DDescriptor(
                pixelFormat: .r8Uint,
                width: 500,
                height: 500,
                mipmapped: false
            )
            textureDescriptor.usage = [.shaderRead, .shaderWrite]
            
            mapTexture = device.makeTexture(descriptor: textureDescriptor)
        }
        
        func updateMapData(_ mapData: OccupancyMapData, robotHeading: Float) {
            self.currentMapData = mapData
            self.currentRobotHeading = robotHeading
            
            // Update texture with map data
            updateMapTexture()
        }
        
        private func updateMapTexture() {
            guard let mapTexture = mapTexture,
                  let mapData = currentMapData else { return }
            
            // Convert map data to texture format
            var textureData = [UInt8](repeating: 0, count: 500 * 500)
            
            for y in 0..<500 {
                for x in 0..<500 {
                    let value = mapData.map[y][x]
                    // Map occupancy values to texture values
                    // 0 (free) -> 0, 1 (occupied) -> 255, 2 (unknown) -> 128
                    let textureValue: UInt8 = {
                        switch value {
                        case 0: return 0    // Free space
                        case 1: return 255  // Occupied
                        case 2: return 128  // Unknown
                        default: return 200 // Error (purple-ish)
                        }
                    }()
                    textureData[y * 500 + x] = textureValue
                }
            }
            
            // Upload to texture
            mapTexture.replace(region: MTLRegionMake2D(0, 0, 500, 500),
                             mipmapLevel: 0,
                             withBytes: textureData,
                             bytesPerRow: 500)
        }
        
        // MARK: - MTKViewDelegate
        
        func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
            // Handle view resize if needed
        }
        
        func draw(in view: MTKView) {
            guard let drawable = view.currentDrawable,
                  let pipelineState = pipelineState,
                  let robotPipelineState = robotPipelineState,
                  let commandBuffer = commandQueue?.makeCommandBuffer(),
                  let renderPassDescriptor = view.currentRenderPassDescriptor,
                  let mapData = currentMapData else { return }
            
            renderPassDescriptor.colorAttachments[0].clearColor = MTLClearColor(red: 0.5, green: 0.5, blue: 0.5, alpha: 1.0)
            
            guard let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) else { return }
            
            // Calculate visible region
            let robotPos = mapData.robotPosition
            let startX = max(0, robotPos.x - visibleRadius)
            let endX = min(499, robotPos.x + visibleRadius)
            let startY = max(0, robotPos.y - visibleRadius)
            let endY = min(499, robotPos.y + visibleRadius)
            
            var mapUniforms = MapUniforms(
                visibleRegion: SIMD4<Float>(Float(startX), Float(startY), Float(endX), Float(endY)),
                robotPosition: SIMD2<Float>(Float(robotPos.x), Float(robotPos.y)),
                robotHeading: currentRobotHeading,
                viewportSize: SIMD2<Float>(Float(view.drawableSize.width), Float(view.drawableSize.height)),
                frontierTarget: SIMD2<Float>(-1, -1) // Default to no target
            )
            
            // Set frontier target if available
            if let frontierTarget = mapData.frontierTarget {
                mapUniforms.frontierTarget = SIMD2<Float>(Float(frontierTarget.x), Float(frontierTarget.y))
            }
            
            // Render map
            renderEncoder.setRenderPipelineState(pipelineState)
            renderEncoder.setVertexBuffer(vertexBuffer, offset: 0, index: 0)
            renderEncoder.setVertexBytes(&mapUniforms, length: MemoryLayout<MapUniforms>.size, index: 1)
            renderEncoder.setFragmentTexture(mapTexture, index: 0)
            renderEncoder.setFragmentBytes(&mapUniforms, length: MemoryLayout<MapUniforms>.size, index: 0)
            renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
            
            // Render robot overlay
            renderEncoder.setRenderPipelineState(robotPipelineState)
            renderEncoder.setVertexBytes(&mapUniforms, length: MemoryLayout<MapUniforms>.size, index: 0)
            renderEncoder.setFragmentBytes(&mapUniforms, length: MemoryLayout<MapUniforms>.size, index: 0)
            renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 4)
            
            renderEncoder.endEncoding()
            
            commandBuffer.present(drawable)
            commandBuffer.commit()
        }
    }
}

// MARK: - Shader Uniforms
struct MapUniforms {
    let visibleRegion: SIMD4<Float> // startX, startY, endX, endY
    let robotPosition: SIMD2<Float>
    let robotHeading: Float
    let viewportSize: SIMD2<Float>
    var frontierTarget: SIMD2<Float>
}



// MARK: - Preview
#Preview {
    let sampleMapData = OccupancyMapData(
        map: Array(repeating: Array(repeating: 2, count: 500), count: 500),
        robotPosition: (x: 250, y: 250),
        mapResolution: 0.05,
        mapOrigin: SIMD3<Float>(0, 0, 0)
    )
    
    return MetalOccupancyMapView(mapData: sampleMapData, robotHeading: 0.5)
        .frame(width: 400, height: 400)
}
