//
// Created by Gleb Reutov on 20.09.2021.
//

import Foundation
import simd
import SpriteKit

struct LidarConfig{
    static let imgWidth: Double = 256.0
    static let imgHeight: Double = 192.0

    static let hAngle: Double = 0.59747594594955444
    static let vAngle: Double = 0.46853139996528625

}

public func realDistVecLinear(centeredX: CGFloat, centeredY: CGFloat, depth: Float32) -> simd_double3{
//    let halfWidth = LidarConfig.imgWidth / 2.0
//    let halfHeight = LidarConfig.imgHeight / 2.0
//    let hAngle: Double = abs(centeredX) * (LidarConfig.hAngle / halfWidth)
//    let vAngle: Double = abs(centeredY) * (LidarConfig.vAngle / halfHeight)
//
//    let d1 = abs(Double(depth) / cos(hAngle))
//    let x1 = d1 * cos(hAngle) * (centeredX / abs(centeredX))
//    let y1 = d1 * cos(vAngle) * (centeredY / abs(centeredY))
//    let vec = simd_float3(x: Float(x1), y: Float(y1), z: Float(d1))

    let mult: Double = 213.0
    let vec = simd_double3(Double(centeredX) / mult, Double(centeredY) / mult, Double(depth))
//    let rotY = rotateY(vec: vec, angleRad: Float(hAngle))
//    let finalVec = rotateX(vec: rotY, angleRad: Float(vAngle))
    let oneDegree = 0.0174533

    let adjustY = rotateY(vec: vec, angleRad: 0 * oneDegree)
    let adjustX = rotateX(vec: adjustY, angleRad: 4 * oneDegree)
    let adjustZ = rotateZ(vec: adjustX, angleRad: -2.2 * oneDegree)
    return adjustZ
}

public func realDistVec(centeredX: CGFloat, centeredY: CGFloat, depth: Float32) -> simd_double3{
    let halfWidth = LidarConfig.imgWidth / 2.0
    let halfHeight = LidarConfig.imgHeight / 2.0
    let hAngle: Double = centeredX * (LidarConfig.hAngle / halfWidth)
    let vAngle: Double = -1 * centeredY * (LidarConfig.vAngle / halfHeight)

//    let d1 = abs(Double(depth) /  sin(hAngle))
//    let vec = simd_float3(x: 0, y: 0, z: Float(d1))
//let vec3d = simd_float3(Float(x) / 180.0, Float(y) / 180.0, Float(depth))
    let vec = simd_double3(x: 0, y: 0, z: Double(depth))


    let oneDegree = 0.0174533
    let rotY = rotateY(vec: vec, angleRad: hAngle)
    let finalVec = rotateX(vec: rotY, angleRad: vAngle)
    let adjustY = rotateY(vec: finalVec, angleRad: 0 * oneDegree)
    let adjustX = rotateX(vec: adjustY, angleRad: 4 * oneDegree)
    let adjustZ = rotateZ(vec: adjustX, angleRad: -2.2 * oneDegree)
    return adjustZ
}


func rotateX(vec: simd_double3, angleRad: Double) -> simd_double3 {
    let rotationMatrix = simd_double3x3(rows: [
        simd_double3(1, 0, 0),
        simd_double3(0, cos(angleRad), -sin(angleRad)),
        simd_double3(0, sin(angleRad), cos(angleRad)),
    ])

    return rotationMatrix * vec
}



func rotateY(vec: simd_double3, angleRad: Double) -> simd_double3 {
    let rotationMatrix = simd_double3x3(rows: [
        simd_double3(cos(angleRad), 0, sin(angleRad)),
        simd_double3(0, 1, 0),
        simd_double3(-sin(angleRad), 0, cos(angleRad)),
    ])

    return rotationMatrix * vec
}


func rotateZ(vec: simd_double3, angleRad: Double) -> simd_double3 {
    let rotationMatrix = simd_double3x3(rows: [
        simd_double3(cos(angleRad), -sin(angleRad), 0),
        simd_double3(sin(angleRad), cos(angleRad), 0),
        simd_double3(0, 0, 1),
    ])

    return rotationMatrix * vec
}

func collision(map: [simd_double2], origin: simd_double2, dest: simd_double2, offset: Double) -> [simd_double2] {
    let destAngle = -1 * axisAngle(v: dest) + .pi / 2
    let destLen = length(dest)
    let alignedMap = map.map {a in rotateVec(v: a, angle: destAngle) - origin}
    let interPointsLocal = alignedMap.filter {a in abs(a.x) < offset && a.y > 0 && a.y < destLen}
    let interPointsGlobal = interPointsLocal.map{v in rotateVec(v: v, angle: -1 * destAngle) + origin}

    return interPointsGlobal
}
func intersectionPoints(map: [simd_double2], origin: simd_double2, dest: simd_double2, offset: Double) -> [simd_double2] {
    let localDest = dest - origin
    let destAngle = -1 * axisAngle(v: localDest) + .pi / 2
    let destLen = length(localDest)

    let alignedMap = map.map {a in rotateVec(v: a - origin, angle: destAngle)}
    let interPointsLocal = alignedMap.filter {a in abs(a.x) <= offset && a.y > 0 && a.y < destLen + offset * 2}
    let interPointsGlobal = interPointsLocal.map{v in rotateVec(v: v, angle: -1 * destAngle) + origin}
    for p in interPointsGlobal {
        let maybeSamePoint = map.first {a in distance(p, a) < 0.00001}
        assert(maybeSamePoint != nil)
    }
    return interPointsGlobal
}

public func plotNearestSubDest(map: [simd_double2], origin: simd_double2, dest: simd_double2, offset: Double) -> simd_double2 {
    print("origin \(origin) dest \(dest)")
    if(distance(origin, dest) < 0.01){
        return dest
    }

    let interPointsGlobal = intersectionPoints(map: map, origin: origin, dest: dest, offset: offset)
    let maybeFarCollision = interPointsGlobal
            .min(by: { a, b in simd_length(a) > simd_length(b)})
    if let nearCollision = maybeFarCollision {

        let destAngle = -1 * axisAngle(v: dest - origin) + .pi / 2
        let nearCollisionLocal = rotateVec(v: nearCollision - origin, angle: destAngle)

        let subDestLocal = nearCollisionLocal + xSign(nearCollisionLocal) * simd_double2(x: 3 * offset, y: 0)
        let subDestGlobal = rotateVec(v: subDestLocal, angle: -1 * destAngle) + origin


        //assert(intersectionPoints(map: [subDestGlobal], origin: origin, dest: dest, offset: offset).isEmpty)

//        assert(distance(subDest, nearCollision) >= 2 * offset)
//        assert(
//                intersectionPoints(map: map, origin: origin, dest: subDest, offset: offset)
//                        .first { a in
//                            distance(nearCollision, a) < 0.0001
//                        } == nil
//        )


        return plotNearestSubDest(map: map, origin: origin, dest: subDestGlobal, offset: offset)
    }else {
        return dest
    }

}

public func plotCourse(map: [simd_double2], origin: simd_double2, dest: simd_double2, offset: Double, course: [simd_double2] = []) -> [simd_double2] {
    let p = plotNearestSubDest(map: map, origin: origin, dest: dest, offset: offset)
    let courseUpdate = course + [p]

    if (distance(p, dest) < 0.0001){
        return courseUpdate
    }else {
        return plotCourse(map: map, origin: p, dest: dest, offset: offset, course: courseUpdate)
    }

}
