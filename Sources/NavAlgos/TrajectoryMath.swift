//
//  TrajectoryMath.swift
//  TrajectoryMath
//
//  Created by Gleb Reutov on 14.09.2021.
//

import Foundation
import simd

struct WorldModel{
    static let xAxis = simd_double2(x: 1, y: 0)
    static let yAxis = simd_double2(x: 0, y: 1)
    static let origin = simd_double2(x: 0, y: 0)
}


public struct VehiclePose{
    let head: simd_double2
    let tail: simd_double2
}
public struct TrajectoryModel{
    let turnRadius: Double
    //position where we starting motion
    let startPose: VehiclePose
    //position where we going to reach
    let destPose: VehiclePose

    //center of the turn radius number one
    let turnPoint0: simd_double2
    let turnPoint0RotationCloclwise: Bool

    //center of the turn radius number two
    let turnPoint1: simd_double2

    //point where we changing direction of steering
    let crossoverPoint: simd_double2

    //
    let endOfSteerPoint: simd_double2
}

public func trajectoryLocal(turnRadius: Double, dest: simd_double2) -> TrajectoryModel? {
    let origin = WorldModel.origin
    let turnPoint0 = turnPoint0(dest: dest, r: turnRadius)
//    let r0to = r0Calc(dest: dest, turnPoint: turnPoint0)
//
//    guard let intersection = intersect(p1: origin, p2: dest, p3: turnPoint0, p4: r0to) else {
//        print("No intersection point")
//        return nil
//    }
//
//    let h0 = distance(r0to, intersection)

    //let r1 = r1Calc(offset: h0, intersection: intersection, tp0: turnPoint0, dest: dest)
    guard let r1 = r1Calc(turnPoint0: turnPoint0, dest: dest) else {
        return nil
    }

    let turnPoint1 = turnPoint1(r1: r1, tp0: turnPoint0, dest: dest, turnRadius: turnRadius)

    let r2 = r2Calc(turnPoint1: turnPoint1, dest: dest, turnRadius: turnRadius)

    let destTail = dest - normalize(dest) * 10
    let startPose = VehiclePose(head: origin, tail: simd_double2(x: 0, y: -1))
    let turnPoint0Rotation = arcRotation(center: turnPoint0, tail: startPose.tail, head: startPose.head)
    return TrajectoryModel(
            turnRadius: turnRadius,
            startPose: startPose,
            destPose: VehiclePose(head: dest, tail: destTail),
            turnPoint0: turnPoint0,
            turnPoint0RotationCloclwise: turnPoint0Rotation,
            turnPoint1: turnPoint1,
            crossoverPoint: r1,
            endOfSteerPoint: r2
    )
}


public func pointReltiveToPose(startPose: VehiclePose, dest: simd_double2) -> simd_double2 {
    let unitY = normalize(startPose.head - startPose.tail)
    let unitX = rotateVec(v: unitY, angle: -.pi / 2)

    let startAngle = axisAngle2(v: unitX)
    
    return rotateVec(v: dest - startPose.head, angle: startAngle)
}

public func pointRelativeToGlobal(startPose: VehiclePose, dest: simd_double2) -> simd_double2 {
    let unitY = normalize(startPose.head - startPose.tail)
    let unitX = rotateVec(v: unitY, angle: -.pi / 2)

    let startAngle = axisAngle2(v: unitX)
    
    return rotateVec(v: dest, angle: -startAngle) + startPose.head
}

public func trajectoryGlobal(startPose: VehiclePose, dest: simd_double2, turnRadius: Double) -> TrajectoryModel? {

    let unitY = normalize(startPose.head - startPose.tail)
    let unitX = rotateVec(v: unitY, angle: -.pi / 2)

    let startAngle = axisAngle2(v: unitX)
//    let startAngle = length(startPose.head) > 0 ? .pi / 2 - axisAngle(v: startPose.head - startPose.tail) : 0
    let start = startPose.head
    let localDest = rotateVec(v: dest - start, angle: startAngle)
    //let localOrigin = rotateVec(v: dest - start, angle: startAngle)
//    assert(abs(distance(start, dest) - length(localDest)) < 0.0001)
    if let traj = trajectoryLocal(turnRadius: turnRadius, dest: localDest) {

        let turnPoint0Global = rotateVec(v: traj.turnPoint0, angle: -startAngle) + start
        let turnPoint1Global = rotateVec(v: traj.turnPoint1, angle: -startAngle) + start
        let r1Global = rotateVec(v: traj.crossoverPoint, angle: -startAngle) + start
        let r2Global = rotateVec(v: traj.endOfSteerPoint, angle: -startAngle) + start

        let destPoseHead = rotateVec(v: traj.destPose.head, angle: -startAngle) + start
        let destPoseTail = rotateVec(v: traj.destPose.tail, angle: -startAngle) + start
        //assert(distance(destPoseHead, dest) < 0.0001)
        return TrajectoryModel(
                turnRadius: turnRadius,
                startPose: startPose,
                destPose: VehiclePose(head: destPoseHead, tail: destPoseTail),
                turnPoint0: turnPoint0Global,
                turnPoint0RotationCloclwise: traj.turnPoint0RotationCloclwise,
                turnPoint1: turnPoint1Global,
                crossoverPoint: r1Global,
                endOfSteerPoint: r2Global
        )
    }else {
        return nil
    }
}

func arcRotation(center: simd_double2, tail: simd_double2, head: simd_double2) -> Bool {
    let normX = normalize(head - tail)

    //we have point in global coordinate sys
    //we need to convert it to local CS defined bby normXY

    let localCenter = rotateVec(v: center - head, angle: axisAngle2(v: normX))
    return localCenter.x > 0
}

func turnPoint0(dest: simd_double2, r: Double) -> simd_double2 {
    let ordDirection = xSign(dest)
    return simd_double2(x: r * ordDirection, y: 0)
}

func r0CalcRwd(dest: simd_double2, turnPoint: simd_double2) -> simd_double2 {
    let r0 = normalize(dest) * length(turnPoint) + turnPoint
    return r0
}

func perpendicular(to: simd_double2, from: simd_double2) -> simd_double2 {
    rotateVec(v: normalize(to), angle: .pi / 2 * xSign(to) * ySign(to)) * length(from) + from
}
func r1Calc(turnPoint0: simd_double2, dest: simd_double2) -> simd_double2?{
    if(ySign(dest) < 0){
        return r1CalcRwd(turnPoint0: turnPoint0, dest: dest)
    }else {
        return r1CalcFwd(turnPoint0: turnPoint0, dest: dest)
    }
}

func r1CalcRwd(turnPoint0: simd_double2, dest: simd_double2) -> simd_double2?{
    let r0 = perpendicular(to: dest, from: turnPoint0)
    if let interPoint = intersect(p1: WorldModel.origin, p2: dest, p3: turnPoint0, p4: r0) {
        let offset = distance(interPoint, r0)
        let parallel = r0CalcRwd(dest: dest, turnPoint: turnPoint0)

        return parallel - (r0 - interPoint) / 2
    }else{
        print("Can't calc r1, no intersect")
        return nil
    }
}

func r1CalcFwd(turnPoint0: simd_double2, dest: simd_double2) -> simd_double2?{
    let r0 = perpendicular(to: dest, from: turnPoint0)
    guard let intersection = intersect(p1: WorldModel.origin, p2: dest, p3: turnPoint0, p4: r0) else {
        print("No intersection point")
        return nil
    }

    let offset = distance(r0, intersection)
    let angle = r1Angle(turnRadius: length(turnPoint0), offset: offset) * ySign(dest)
    let vec = rotateVec(v: abs(turnPoint0), angle: angle)
    
    
    let unitY = normalize(intersection) * ySign(dest)
    let unitX = rotateVec(v: unitY, angle: xSign(dest) * .pi / 2)
    
    return projectToDefaultCS(
        v: vec,
        unitX: unitX,
        unitY: unitY,
        origin: turnPoint0
    )
}

func r2Calc(turnPoint1: simd_double2, dest: simd_double2, turnRadius: Double) -> simd_double2 {
    turnPoint1 + simd_normalize(rotateVec(v: dest, angle: -1 * xSign(dest) * .pi / 2)) * turnRadius


}

func turnPoint1(r1: simd_double2, tp0: simd_double2, dest: simd_double2, turnRadius: Double) -> simd_double2 {

    let unitX = normalize(tp0 - r1)
    let unitY = rotateVec(v: unitX, angle: .pi / 2)
    let tp1Local = simd_double2(x: -1 * turnRadius, y: 0)
    return projectToDefaultCS(
        v: tp1Local,
        unitX: unitX,
        unitY: unitY,
        origin: r1
    )
}



func r1Angle(turnRadius: Double, offset: Double) -> Double{
    let a = turnRadius - offset / 2
    let c = turnRadius
    
    let cos = a / c
    let angle = acos(cos)
    
    return angle
    
}







