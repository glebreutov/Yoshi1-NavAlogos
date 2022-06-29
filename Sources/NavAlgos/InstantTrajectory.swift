//
//  File.swift
//  
//
//  Created by Gleb Reutov on 18.11.2021.
//

import Foundation
import simd
import Darwin

private class InstantTrajectory{
    
    private let trajectories: [TrajectoryModel]
    //private var currentPose: VehiclePose = VehiclePose(head: simd_double2(x: 0, y: 0), tail: simd_double2(x: 0, y: -1))
    //private var prevDist = 0.0
    
    private let globalStep = 0.001
    
    public init(_ trajectories: [TrajectoryModel]) {
        self.trajectories = trajectories
    }
    
    
    private func nextStep(dist: Double, step: Double, currentPose: VehiclePose) -> VehiclePose {
        if step > 0, let p = InstantTrajectory.currentPosition(trajectories: trajectories, d: dist, step: step) {
            let head = inGlobalReferenceFrame(frame: currentPose, localPoint: p)
            return VehiclePose(head: head, tail: currentPose.head)
        }else {
            return currentPose
        }
    
    }
    
    private func poseStepByStep(dist: Double, delta: Double, currentPose: VehiclePose)  -> VehiclePose {
        if delta > globalStep {
            let nextPose = nextStep(dist: dist, step: globalStep, currentPose: currentPose)
            return poseStepByStep(dist: dist + globalStep, delta: delta - globalStep, currentPose: nextPose)
        }else {
            return nextStep(dist: dist, step: delta, currentPose: currentPose)
        }
    }
    
    public func poseAt(dist: Double)  -> VehiclePose {
        let startPose: VehiclePose = VehiclePose(head: simd_double2(x: 0, y: 0), tail: simd_double2(x: 0, y: -1))
        

        return poseStepByStep(dist: 0.0, delta: dist, currentPose: startPose)
    }
    


    private static func currentPositionOnLine(step: Double) -> simd_double2 {
        simd_double2(x: 0, y: step)
    }

    private static func currentPositionOnArc(radius: Double, clockwise: Bool, step: Double) -> simd_double2 {
        let unit = simd_double2(x: 0, y: step)
        let angle = step / radius

        let direction = clockwise ? -1.0 : 1.0
        return rotateVec(v: unit, angle: direction * angle)
    }

    private static func arcAngleByChord(d: Double, r: Double) -> Double {
        return 2 * asin(d / (2 * r))
    }

    public static func currentPosition(traj: TrajectoryModel, d: Double, step: Double) -> simd_double2 {
        let turnOneDist = traj.turnRadius * arcAngleByChord(d: distance(traj.startPose.head, traj.crossoverPoint), r:  traj.turnRadius)
        let turnTwoDist = turnOneDist + traj.turnRadius * arcAngleByChord(d: distance(traj.crossoverPoint, traj.endOfSteerPoint), r:  traj.turnRadius)
        let fullDist = turnOneDist + turnTwoDist + distance(traj.endOfSteerPoint, traj.destPose.head)
        if d < turnOneDist {
            return currentPositionOnArc(radius: traj.turnRadius, clockwise: traj.turnPoint0RotationClockwise, step: step)
        }else if d < turnTwoDist {
            return currentPositionOnArc(radius: traj.turnRadius, clockwise: !traj.turnPoint0RotationClockwise, step: step)
        }else if d <= fullDist{
            return currentPositionOnLine(step: step)
        }else {
            return simd_double2(0, 0)
        }
    }

    public static func length(t: TrajectoryModel) -> Double {
        
        let turnOneDist = t.turnRadius * arcAngleByChord(d: distance(t.startPose.head, t.crossoverPoint), r:  t.turnRadius)
        let turnTwoDist = t.turnRadius * arcAngleByChord(d: distance(t.crossoverPoint, t.endOfSteerPoint), r:  t.turnRadius)
        
        return turnOneDist + turnTwoDist + distance(t.endOfSteerPoint, t.destPose.head)
    }

    public func maneursLeft(dist: Double) -> [TrajectoryModel] {
        InstantTrajectory.maneursLeft(trajectories: trajectories, d: dist)
    }
    
    private static func maneursLeft(trajectories: [TrajectoryModel], d: Double, acc: Double = 0.0) -> [TrajectoryModel] {
        if let first = trajectories.first {
            let newAcc = acc + length(t: first)
            if newAcc > d {
                return trajectories
            }else {
                return maneursLeft(trajectories: Array(trajectories.dropFirst()), d: d, acc: newAcc)
            }
        }
        
        return []
    }
    
    public static func currentPosition(trajectories: [TrajectoryModel], d: Double, step: Double, acc: Double = 0.0) -> simd_double2? {
        if let first = trajectories.first {
            let newAcc = acc + length(t: first)
            if newAcc > d {
                return currentPosition(traj: first, d: d - acc, step: step)
            }else {
                return currentPosition(trajectories: Array(trajectories.dropFirst()), d: d, step: step, acc: newAcc)
            }
        }
        
        return nil
    }

}

public class InstantTrajectory2 {
    private static func arcAngleByChord(d: Double, r: Double) -> Double {
        return 2 * asin(d / (2 * r))
    }
    
    public static func length(t: TrajectoryModel) -> Double {
        
        let turnOneDist = t.turnRadius * arcAngleByChord(d: distance(t.startPose.head, t.crossoverPoint), r:  t.turnRadius)
        let turnTwoDist = t.turnRadius * arcAngleByChord(d: distance(t.crossoverPoint, t.endOfSteerPoint), r:  t.turnRadius)
        
        return turnOneDist + turnTwoDist + distance(t.endOfSteerPoint, t.destPose.head)
    }
        
    private static func positionOnArc1(t: TrajectoryModel, d: Double) -> simd_double2 {

        let arcAngle = arcAngleByChord(d: simd_length(t.crossoverPoint), r:  t.turnRadius)
        if arcAngle < 0.01 {
            return simd_double2(0, 0)
        }
        let ratio = d / (t.turnRadius * arcAngle)
        let res = rotateVec(v: simd_double2(x: t.turnRadius, y: 0), angle: arcAngle * ratio)
        return res
    }
    
    private static func positionOnArc2(t: TrajectoryModel, d: Double) -> simd_double2 {

        let arcAngle = arcAngleByChord(d: distance(t.crossoverPoint, t.endOfSteerPoint), r:  t.turnRadius)
        if arcAngle < 0.01 {
            return simd_double2(0, 0)
        }
        let ratio = d / (t.turnRadius * arcAngle)
        let res = rotateVec(v: simd_double2(x: t.turnRadius, y: 0), angle: arcAngle - arcAngle * ratio)
        return res
    }
    
    public static func pointAt(traj: TrajectoryModel, dist: Double) -> simd_double2 {
        let turnOneDist = traj.turnRadius * arcAngleByChord(d: distance(traj.startPose.head, traj.crossoverPoint), r:  traj.turnRadius)
        let turnTwoDist = turnOneDist + traj.turnRadius * arcAngleByChord(d: distance(traj.crossoverPoint, traj.endOfSteerPoint), r:  traj.turnRadius)
        let fullDist = turnTwoDist + distance(traj.endOfSteerPoint, traj.destPose.head)
//        print("new crossover", crossoverPoint(dest: traj.destPose.head, r: traj.turnRadius))
//        print("old crossover", traj.crossoverPoint)
        if dist <= turnOneDist {

            return inGlobalReferenceFrame(
                zero: traj.turnPoint0,
                axisX: simd_double2(x: 0, y: 0),
                localPoint: positionOnArc1(t: traj, d: dist))
                                                                                                                                                                           
        }else if dist <= turnTwoDist {
            return inGlobalReferenceFrame(
                zero: traj.turnPoint1,
                axisX: traj.endOfSteerPoint,
                axisY: traj.turnPoint1 - traj.endOfSteerPoint,
                localPoint: positionOnArc2(t: traj, d: dist - turnOneDist))
            
//            return inGlobalReferenceFrame(
//                zero: traj.turnPoint1,
//                axisX: traj.crossoverPoint,
//                axisY: traj.turnPoint1 + rotateVec(v: traj.crossoverPoint - traj.turnPoint1, angle: (traj.crossoverPoint.x / abs(traj.crossoverPoint.x)) * .pi / 2),
//                localPoint: positionOnArc2(t: traj, d: dist - turnOneDist))
            
        }else if dist <= fullDist {
            return traj.endOfSteerPoint + normalize(traj.destPose.head) * (dist - turnTwoDist)
            //return traj.endOfSteerPoint
        }else {
            return traj.destPose.head
        }
    }
    
    static func poseAt(traj: TrajectoryModel, dist: Double) -> VehiclePose {
        let len = length(t: traj)
        let d = min(len, dist)
        let head = pointAt(traj: traj, dist: d)
        let tail = pointAt(traj: traj, dist: d - 0.01)
        
        return VehiclePose(head: head, tail: tail)
    }
    
    

    public static func simpleTrajectory(dest: simd_double2, r: Double) -> TrajectoryModel? {
        func turnPoint1(dest: simd_double2, p: simd_double2) -> simd_double2{
            //solving system of equations
            // "turn point 1" belongs to line p + dest * t
            // length of tp1 is 2 radiuses
            
            let a = pow(dest.x, 2) + pow(dest.y, 2)
            
            let b = (2 * (dest.x * p.x + dest.y * p.y)) / a
            let c = (pow(p.x, 2) - pow(2 * r, 2) + pow(p.y, 2)) / a
            
            let d = pow(b, 2) - 4 * c
            
            let t = (-b + sqrt(d)) / 2
            return p + dest * t
        }

        if !dest.x.isZero && !dest.y.isZero {
            let signX = dest.x / abs(dest.x)
            let signY = dest.y / abs(dest.y)
            let shift = simd_double2(-1 * signX * r, 0)
            
            let p = shift + normalize(simd_double2(x: -1, y: dest.x / dest.y)) * r * signX * signY
            
            let tp1 = turnPoint1(dest: dest, p: p)

            let tp1g = tp1 - shift

            let eos = normalize(dest) * distance(p, tp1)

            if simd_length(eos) <= simd_length(dest) {

                return TrajectoryModel(
                    turnRadius: r,
                    startPose: VehiclePose.zeroPose,
                    destPose: VehiclePose(
                        head: dest,
                        tail: dest * 0.99
                    ),
                    turnPoint0: shift * -1,
                    turnPoint0RotationClockwise: signX > 0 ? true : false,
                    turnPoint1: tp1g,
                    crossoverPoint: normalize(tp1) * r - shift,
                    endOfSteerPoint: eos
                )
            }else{
                return nil
            }
        }else if dest.x.isZero && dest.y > 0 {
            return TrajectoryModel(
                    turnRadius: r,
                    startPose: VehiclePose.zeroPose,
                    destPose: VehiclePose(
                            head: dest,
                            tail: dest * 0.99
                    ),
                    turnPoint0: simd_double2(r, 0),
                    turnPoint0RotationClockwise: true,
                    turnPoint1: simd_double2(-r, 0),
                    crossoverPoint: simd_double2(0, 0),
                    endOfSteerPoint: simd_double2(0, 0)
            )
        }
        return nil
        
    }
    
}


