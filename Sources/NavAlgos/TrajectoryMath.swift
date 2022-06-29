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
    public let head: simd_double2
    public let tail: simd_double2
    
    public init(head: simd_double2, tail: simd_double2) {
        self.head = head
        self.tail = tail
    }
    
    public static let zeroPose = VehiclePose(head: simd_double2(x: 0, y: 0), tail: simd_double2(x: 0, y: -1))
}

extension VehiclePose {
    public func global(frame: VehiclePose) -> VehiclePose {
        return VehiclePose(
            head: inGlobalReferenceFrame(frame: frame, localPoint: head),
            tail: inGlobalReferenceFrame(frame: frame, localPoint: tail)
        )
    }
    
    public func local(frame: VehiclePose) -> VehiclePose {
        return VehiclePose(
            head: inLocalReferenceFrame(frame: frame, globalPoint: head),
            tail: inLocalReferenceFrame(frame: frame, globalPoint: tail)
        )
    }
    
    public static func -(_ pose1: VehiclePose, _ pose2: VehiclePose) -> VehiclePose {
        VehiclePose(head: pose1.head - pose2.head, tail: pose1.tail - pose2.tail)
    }

    public static func +(_ pose1: VehiclePose, _ pose2: VehiclePose) -> VehiclePose {
        VehiclePose(head: pose1.head + pose2.head, tail: pose1.tail + pose2.tail)
    }
}

extension simd_double2 {
    public func global(pose: VehiclePose) -> simd_double2 {
        return inGlobalReferenceFrame(frame: pose, localPoint: self)
    }
    
    public func local(pose: VehiclePose) -> simd_double2 {
        return inLocalReferenceFrame(frame: pose, globalPoint: self)
    }
    
    public func length() -> Double {
        simd_length(self)
    }
}

public struct TrajectoryModel{
    public let turnRadius: Double
    //position where we starting motion
    public let startPose: VehiclePose
    //position where we going to reach
    public let destPose: VehiclePose

    //center of the turn radius number one
    public let turnPoint0: simd_double2
    public let turnPoint0RotationClockwise: Bool

    //center of the turn radius number two
    public let turnPoint1: simd_double2

    //point where we changing direction of steering
    public let crossoverPoint: simd_double2

    //
    public let endOfSteerPoint: simd_double2

    public static let stopTrajectory = TrajectoryModel(
        turnRadius: 0.6,
        startPose: VehiclePose.zeroPose,
        destPose: VehiclePose.zeroPose,
        turnPoint0: simd_double2(0.6, 0),
        turnPoint0RotationClockwise: true,
        turnPoint1: simd_double2(-0.6, 0),
        crossoverPoint: simd_double2(0.0, 0),
        endOfSteerPoint: simd_double2(0.0, 0)
    )
}

public func trajectoryLocal(turnRadius: Double, dest: simd_double2) -> TrajectoryModel? {
    
    
    let origin = WorldModel.origin
    let turnPoint0 = turnPoint0(dest: dest, r: turnRadius)

    guard let r1 = r1Calc(turnPoint0: turnPoint0, dest: dest) else {
        return nil
    }

    let turnPoint1 = turnPoint1(r1: r1, tp0: turnPoint0, dest: dest, turnRadius: turnRadius)

    let r2 = r2Calc(turnPoint1: turnPoint1, dest: dest, turnRadius: turnRadius)

    let destTail = dest * 0.9
    let startPose = VehiclePose(head: origin, tail: simd_double2(x: 0, y: -1))
    let turnPoint0Rotation = arcRotation(center: turnPoint0, tail: startPose.tail, head: startPose.head)
    
    guard length(dest) - length(r2) > -0.001 else {
        print("r2 beyond dest. dest: ", dest)
        return nil
    }
    
    return TrajectoryModel(
            turnRadius: turnRadius,
            startPose: startPose,
            destPose: VehiclePose(head: dest, tail: destTail),
            turnPoint0: turnPoint0,
            turnPoint0RotationClockwise: turnPoint0Rotation,
            turnPoint1: turnPoint1,
            crossoverPoint: r1,
            endOfSteerPoint: r2
    )
}




//
//public func pointRelativeToPose(startPose: VehiclePose, dest: simd_double2) -> simd_double2 {
////    let unitY = normalize(startPose.head - startPose.tail)
////    let unitX = rotateVec(v: unitY, angle: -.pi / 2)
////
////    let startAngle = axisAngle2(v: unitX)
//
//    //return rotateVec(v: dest  - startPose.head, angle: -startAngle)
//
//    return inLocalReferenceFrame(frame: startPose, globalPoint: dest)
//}



public func trajectoryGlobal(startPose: VehiclePose, dest: simd_double2, turnRadius: Double) -> TrajectoryModel? {
    //let localDest = pointRelativeToPose(startPose: startPose, dest: dest)
    let localDest = inLocalReferenceFrame(frame: startPose, globalPoint: dest)

    if let traj = trajectoryLocal(turnRadius: turnRadius, dest: localDest) {

        let turnPoint0Global = inGlobalReferenceFrame(frame: startPose, localPoint: traj.turnPoint0)
        let turnPoint1Global = inGlobalReferenceFrame(frame: startPose, localPoint: traj.turnPoint1)
        let r1Global = inGlobalReferenceFrame(frame: startPose, localPoint: traj.crossoverPoint)
        let r2Global = inGlobalReferenceFrame(frame: startPose, localPoint: traj.endOfSteerPoint)

        let destPoseHead = inGlobalReferenceFrame(frame: startPose, localPoint: traj.destPose.head)
        let destPoseTail = inGlobalReferenceFrame(frame: startPose, localPoint: traj.destPose.tail)

        return TrajectoryModel(
                turnRadius: turnRadius,
                startPose: startPose,
                destPose: VehiclePose(head: destPoseHead, tail: destPoseTail),
                turnPoint0: turnPoint0Global,
                turnPoint0RotationClockwise: traj.turnPoint0RotationClockwise,
                turnPoint1: turnPoint1Global,
                crossoverPoint: r1Global,
                endOfSteerPoint: r2Global
        )
    }else {
        return nil
    }
}

public func localTrajectories(pose: VehiclePose, points: Array<simd_double2>, turnRadius: Double, acc: [TrajectoryModel] = []) ->[TrajectoryModel] {
    if let nextDest = points.first {
        let localDest = inLocalReferenceFrame(frame: pose, globalPoint: nextDest)
        if let nextManeur = trajectoryLocal(turnRadius: turnRadius, dest: localDest) {
            //assert(length(pose.head + nextManeur.destPose.head - nextDest) < 0.001)
            let globalPose = VehiclePose(
                    head: inGlobalReferenceFrame(frame: pose, localPoint: nextManeur.destPose.head),
                    tail: inGlobalReferenceFrame(frame: pose, localPoint: nextManeur.destPose.tail)
            )

            return localTrajectories(
                    pose: globalPose,
                    points: Array(points.dropFirst()),
                    turnRadius: turnRadius,
                    acc: acc + [nextManeur]
            )
        }else{
            print("Can't calculate next pose")
            return acc
        }

    }else {
        return acc
    }
}

public func localTrajectoriesBestEffort(pose: VehiclePose, points: Array<simd_double2>, turnRadius: Double, acc: [TrajectoryModel] = []) ->[TrajectoryModel] {
    if let nextDest = points.first {
        let localDest = inLocalReferenceFrame(frame: pose, globalPoint: nextDest)
        if let nextManeur = trajectoryLocal(turnRadius: turnRadius, dest: localDest) {
            //assert(length(pose.head + nextManeur.destPose.head - nextDest) < 0.001)
            let globalPose = VehiclePose(
                    head: inGlobalReferenceFrame(frame: pose, localPoint: nextManeur.destPose.head),
                    tail: inGlobalReferenceFrame(frame: pose, localPoint: nextManeur.destPose.tail)
            )

            return localTrajectories(
                    pose: globalPose,
                    points: Array(points.dropFirst()),
                    turnRadius: turnRadius,
                    acc: acc + [nextManeur]
            )
        }else{
            print("Can't calculate traj to \(nextDest), trying next point from \(pose.head)")
            return localTrajectoriesBestEffort(pose: pose, points: Array(points.dropFirst()), turnRadius: turnRadius, acc: acc)
        }

    }else {
        return acc
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
