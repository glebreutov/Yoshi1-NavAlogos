//
// Created by Gleb Reutov on 18.09.2021.
//

import Foundation
import simd

public func xSign(_ v: simd_double2) -> Double {
    if(v.x != 0.0){
        return v.x / abs(v.x)
    }else{
        return 1
    }

}

public func ySign(_ v: simd_double2) -> Double {
    if(v.y != 0.0){
        let a = v.y / abs(v.y)
        return a
    }else{
        return 1
    }

}

public func angle(v1: simd_double2, v2: simd_double2) -> Double {
    let dotProduct = dot(v1, v2)
    let lengths = length(v1) * length(v2)

    let vectorAngle = acos(dotProduct / lengths)

    return vectorAngle
}

public func axisAngle2(v: simd_double2) -> Double{
    let vn = normalize(v)
    let atan: Double = atan2(vn.x, vn.y) - .pi / 2

    return atan
}

public func axisAngle(v: simd_double2) -> Double {
    angle(v1: v, v2: WorldModel.xAxis)
}

func ordinateAngle(v: simd_double2) -> Double {
    angle(v1: v, v2: WorldModel.yAxis)
}

//rotate from axis X counter cloclwise
public func rotateVec(v: simd_double2, angle: Double) -> simd_double2 {
    let rotationMatrix = simd_double2x2(rows: [
        simd_double2(cos(angle), -sin(angle)),
        simd_double2(sin(angle), cos(angle)),
    ])

    return rotationMatrix * v
}

func intersect(p1: simd_double2, p2: simd_double2, p3: simd_double2, p4: simd_double2) -> simd_double2?{
    let matrix = simd_double2x2(p4 - p3, p1 - p2)
    guard matrix.determinant != 0 else { return nil } // Determinent == 0 => parallel lines
    let multipliers = matrix.inverse * (p1 - p3)
    // If either of the multipliers is outside the range 0 ... 1, then the
    // intersection would be off the end of one of the line segments.
    guard (0.0 ... 1.0).contains(multipliers.x) && (0.0 ... 1.0).contains(multipliers.y)
            else { return nil }
    return p1 + multipliers.y * (p2 - p1)
}


func projectFromVec(vec: simd_double2, origin: simd_double2) -> simd_double2 {
    let unitX = normalize(origin)
    let unitY = rotateVec(v: unitX, angle: .pi / 2)

    return projectToDefaultCS(v: vec, unitX: unitX, unitY: unitY, origin: origin)
}

func projectToDefaultCS(v: simd_double2, unitX: simd_double2, unitY: simd_double2, origin: simd_double2) -> simd_double2 {
    let transform = simd_double2x2(rows: [unitX, unitY])
    //let transform = simd_double2x2(columns: (simd_normalize(unitX), simd_normalize(unitY)))
    let inverseTransform = simd_inverse(transform)
    return inverseTransform * v + origin
}
