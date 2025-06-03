//
//  OccupancyMapView.swift
//  571
//
//  Created by Kurt Gu on 6/3/25.
//

import SwiftUI
import Foundation

// Enhanced Occupancy Map View with real-time robot visualization
struct OccupancyMapView: View {
    let mapData: OccupancyMapData
    let robotHeading: Float // Heading in radians
    
    var body: some View {
        Canvas { context, size in
            // Calculate visible region around robot (e.g., 100x100 cells centered on robot)
            let visibleRadius = 50
            let startX = max(0, mapData.robotPosition.x - visibleRadius)
            let endX = min(499, mapData.robotPosition.x + visibleRadius)
            let startY = max(0, mapData.robotPosition.y - visibleRadius)
            let endY = min(499, mapData.robotPosition.y + visibleRadius)
            
            // Calculate scale to fill the entire view with the visible region
            let visibleWidth = CGFloat(endX - startX + 1)
            let visibleHeight = CGFloat(endY - startY + 1)
            let scale = min(size.width / visibleWidth, size.height / visibleHeight)
            
            // Draw background
            context.fill(
                Path(CGRect(origin: .zero, size: size)),
                with: .color(.gray.opacity(0.1))
            )
            
            // Center the visible region in the canvas
            let offsetX = (size.width - visibleWidth * scale) / 2
            let offsetY = (size.height - visibleHeight * scale) / 2
            
            // Draw occupancy grid
            for y in startY...endY {
                for x in startX...endX {
                    let value = mapData.map[y][x]
                    let rect = CGRect(
                        x: offsetX + CGFloat(x - startX) * scale,
                        y: offsetY + CGFloat(y - startY) * scale,
                        width: scale,
                        height: scale
                    )
                    
                    // Color based on occupancy value, with special handling for frontier target
                    let color: Color = {
                        // Check if this cell is the frontier target
                        if let frontierTarget = mapData.frontierTarget,
                           frontierTarget.x == x && frontierTarget.y == y {
                            return .red // Frontier target is always red
                        }
                        
                        // Regular occupancy coloring
                        switch value {
                        case 0: return .white // Free space
                        case 1: return .black // Occupied
                        case 2: return .gray.opacity(0.5)  // Unknown
                        default: return .purple.opacity(0.5) // Error state
                        }
                    }()
                    
                    context.fill(Path(rect), with: .color(color))
                }
            }
            
            // Draw grid lines for better visibility (only if cells are large enough)
            if scale > 2 {
                context.stroke(
                    Path { path in
                        // Vertical lines
                        for i in 0...(endX - startX + 1) {
                            path.move(to: CGPoint(x: offsetX + CGFloat(i) * scale, y: offsetY))
                            path.addLine(to: CGPoint(x: offsetX + CGFloat(i) * scale, y: offsetY + CGFloat(endY - startY + 1) * scale))
                        }
                        // Horizontal lines
                        for i in 0...(endY - startY + 1) {
                            path.move(to: CGPoint(x: offsetX, y: offsetY + CGFloat(i) * scale))
                            path.addLine(to: CGPoint(x: offsetX + CGFloat(endX - startX + 1) * scale, y: offsetY + CGFloat(i) * scale))
                        }
                    },
                    with: .color(.gray.opacity(0.2)),
                    lineWidth: max(0.5, scale * 0.01)
                )
            }
            
            // Draw robot position
            let robotScreenX = offsetX + CGFloat(mapData.robotPosition.x - startX) * scale + scale / 2
            let robotScreenY = offsetY + CGFloat(mapData.robotPosition.y - startY) * scale + scale / 2
            let robotRadius = max(scale * 1.5, 12) // Ensure minimum visible size
            
            // Robot body
            context.fill(
                Path(ellipseIn: CGRect(
                    x: robotScreenX - robotRadius,
                    y: robotScreenY - robotRadius,
                    width: robotRadius * 2,
                    height: robotRadius * 2
                )),
                with: .color(.blue)
            )
            
            // Robot direction indicator (pointing in actual device direction)
            // Convert heading to arrow direction (note: map Y axis is flipped from screen Y)
            let arrowLength = robotRadius * 1.5
            let arrowTipX = robotScreenX + sin(CGFloat(robotHeading)) * arrowLength
            let arrowTipY = robotScreenY - cos(CGFloat(robotHeading)) * arrowLength // Negative because screen Y is flipped
            
            // Calculate arrow base points
            let arrowHalfWidth = robotRadius * 0.5
            let baseAngle1 = robotHeading + Float.pi * 0.75 // 135 degrees offset
            let baseAngle2 = robotHeading - Float.pi * 0.75 // -135 degrees offset
            
            let baseX1 = robotScreenX + sin(CGFloat(baseAngle1)) * arrowHalfWidth
            let baseY1 = robotScreenY - cos(CGFloat(baseAngle1)) * arrowHalfWidth
            let baseX2 = robotScreenX + sin(CGFloat(baseAngle2)) * arrowHalfWidth
            let baseY2 = robotScreenY - cos(CGFloat(baseAngle2)) * arrowHalfWidth
            
            context.fill(
                Path { path in
                    path.move(to: CGPoint(x: arrowTipX, y: arrowTipY))
                    path.addLine(to: CGPoint(x: baseX1, y: baseY1))
                    path.addLine(to: CGPoint(x: baseX2, y: baseY2))
                    path.closeSubpath()
                },
                with: .color(.red)
            )
            
            // Draw coordinate info and scale
            let coordText = "Robot: (\(mapData.robotPosition.x), \(mapData.robotPosition.y)) | Resolution: \(String(format: "%.2f", mapData.mapResolution))m/cell"
            context.draw(
                Text(coordText)
                    .font(.system(size: 12, weight: .medium))
                    .foregroundColor(.black),
                at: CGPoint(x: size.width / 2, y: 20)
            )
            
            // Show height filtering info and frontier exploration
            let heightText = "Height filter: 0.1m - 1.8m | Red = Frontier target"
            context.draw(
                Text(heightText)
                    .font(.system(size: 10, weight: .regular))
                    .foregroundColor(.gray),
                at: CGPoint(x: size.width / 2, y: 35)
            )
        }
        .background(Color.gray.opacity(0.1))
        .cornerRadius(8)
        .drawingGroup() // Optimize rendering performance
    }
}

// MARK: - Preview

#Preview {
    // Create sample data for preview
    let sampleMapData = OccupancyMapData(
        map: Array(repeating: Array(repeating: 2, count: 500), count: 500),
        robotPosition: (x: 250, y: 250),
        mapResolution: 0.05,
        mapOrigin: SIMD3<Float>(0, 0, 0)
    )
    
    return OccupancyMapView(mapData: sampleMapData, robotHeading: 0.5)
        .frame(width: 400, height: 400)
}
