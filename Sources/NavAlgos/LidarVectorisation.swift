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
func loadCones() -> [[Float32]]{
    let rData: Data = try! Data(contentsOf: URL(fileURLWithPath: "cones.bin"))

    var rArray: [Float32]?

    rData.withUnsafeBytes { (bytes: UnsafePointer<Float32>) in
        rArray = Array(UnsafeBufferPointer(start: bytes, count: rData.count / MemoryLayout<Float32>.size))
    }

    let height = 192
    let width = 256
    var res: [[Float32]] = []

    for row in 0...(height - 1) {
    //for row in stride(from: height - 1, through: 0, by: -1) {
        var rowArr: [Float32] = []
        //for col in stride(from: width - 1, through: 0, by: -1){
        for col in 0...(width - 1){

            let depth = rArray![row * height + col]
            rowArr.append(depth)

        }

        res.append(rowArr)
    }

    return res
}
public func realDistVec2d(centeredX: CGFloat, centeredY: CGFloat, depth: Float32) -> simd_float3{
    let halfWidth = LidarConfig.imgWidth / 2.0
    let halfHeight = LidarConfig.imgHeight / 2.0


    //let hAngleMax = LidarConfig.hAngle
    //let vAngleMax = verticalCameraAngle(intrinsics: intrinsics)


    let hAngle = centeredX * (LidarConfig.hAngle / halfWidth)
    let vAngle = -1 * centeredY * (LidarConfig.vAngle / halfHeight)

    let vec = simd_float3(0, 0, depth)

    let rotY = rotateY(vec: vec, angleRad: Float(hAngle))

    //rotateVec(v: simd_double2(x: 0, y: depth), angle: hAngle)
    return rotateX(vec: rotY, angleRad: Float(vAngle))
}

public func realDistVec(centeredX: CGFloat, centeredY: CGFloat, depth: Float32) -> simd_float3{
    let halfWidth = LidarConfig.imgWidth / 2.0
    let halfHeight = LidarConfig.imgHeight / 2.0



    let hAngle = centeredX * (LidarConfig.hAngle / halfWidth)
    let vAngle = -1 * centeredY * (LidarConfig.vAngle / halfHeight)

    let vec = simd_float3(0, 0, depth)

//    var hAngleNorm = hAngle
//    if(hAngle < 0){
//        hAngleNorm = .pi + hAngle
//    }else {
//        hAngleNorm = .pi / 2 - hAngle
//    }
//
//    var vAngleNorm = vAngle
//    if(vAngle < 0){
//        vAngleNorm = .pi + vAngle
//    }else{
//        vAngleNorm = .pi / 2  - vAngle
//    }

    let rotY = rotateY(vec: vec, angleRad: Float(hAngle))

    return rotateX(vec: rotY, angleRad: Float(vAngle))
}

func rotateX(vec: simd_float3, angleRad: Float) -> simd_float3 {
    let rotationMatrix = simd_float3x3(rows: [
        simd_float3(1, 0, 0),
        simd_float3(0, cos(angleRad), -sin(angleRad)),
        simd_float3(0, sin(angleRad), cos(angleRad)),
    ])

    return rotationMatrix * vec
}

func rotateY(vec: simd_float3, angleRad: Float) -> simd_float3 {
    let rotationMatrix = simd_float3x3(rows: [
        simd_float3(cos(angleRad), 0, sin(angleRad)),
        simd_float3(0, 1, 0),
        simd_float3(-sin(angleRad), 0, cos(angleRad)),
    ])

    return rotationMatrix * vec
}

func rotateZ(vec: simd_float3, angleRad: Float) -> simd_float3 {
    let rotationMatrix = simd_float3x3(rows: [
        simd_float3(cos(angleRad), -sin(angleRad), 0),
        simd_float3(sin(angleRad), cos(angleRad), 0),
        simd_float3(0, 0, 1),
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

func plotNearestSubDest(map: [simd_double2], origin: simd_double2, dest: simd_double2, offset: Double) -> simd_double2 {
    print("origin \(origin) dest \(dest)")
    if(distance(origin, dest) < 0.001){
        return dest
    }

    let interPointsGlobal = intersectionPoints(map: map, origin: origin, dest: dest, offset: offset)
    let maybeFarCollision = interPointsGlobal
            .min(by: { a, b in simd_length(a) > simd_length(b)})
    if let nearCollision = maybeFarCollision {

        let destAngle = -1 * axisAngle(v: dest - origin) + .pi / 2
        let nearCollisionLocal = rotateVec(v: nearCollision - origin, angle: destAngle)

        let subDestLocal = nearCollisionLocal + simd_double2(x: offset * 2, y: 0)
        let subDestGlobal = rotateVec(v: subDestLocal, angle: -1 * destAngle) + origin


        assert(intersectionPoints(map: [subDestGlobal], origin: origin, dest: dest, offset: offset).isEmpty)

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
