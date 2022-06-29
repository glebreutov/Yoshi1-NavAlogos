//
//  NewNavAlgo.swift
//  TrajectoryMath
//
//  Created by Gleb Reutov on 07.12.2021.
//

import Foundation
import simd
import CoreGraphics

public func loadEuler(path: URL) -> (roll: Double, pitch: Double, yaw: Double){
    let rData: Data = try! Data(contentsOf: path)
    let bufferSize1 = rData.count / MemoryLayout<Double>.size
    
    let rArray: [Double] = rData.withUnsafeBytes { (bytes: UnsafePointer<Double>) in
        Array(UnsafeBufferPointer(start: bytes, count: bufferSize1))
    }
    
    return (roll: rArray[0], pitch: rArray[1], yaw: rArray[2])
}

public func loadFrame(path: URL) -> [Float32]{
    let rData: Data = try! Data(contentsOf: path)

    let bufferSize1 = rData.count / MemoryLayout<Float32>.size
    
    let rArray: [Float32] = rData.withUnsafeBytes { (bytes: UnsafePointer<Float32>) in
        Array(UnsafeBufferPointer(start: bytes, count: bufferSize1))
    }

    return rArray
}

public func loadConfidenceMap(path: URL) -> [Int8] {
    
    let rData: Data = try! Data(contentsOf: path)

    let bufferSize1 = rData.count / MemoryLayout<Int8>.size
    
    let rArray: [Int8] = rData.withUnsafeBytes { (bytes: UnsafePointer<Int8>) in
        Array(UnsafeBufferPointer(start: bytes, count: bufferSize1))
    }

    return rArray
        
}

public func confidenceToProbablility(i: Int8) -> Double {
    switch i {
        case 0: return 0.1
        case 1: return 0.5
        case 2: return 1.0
        default: return 8.0
    }
}


public func cloudPoint(rArray: [Float32], confMap: [Double], euler: (roll: Double, pitch: Double, yaw: Double)) -> [simd_double3] {
    let height = 192
    let width = 256
    
    var cloud: [simd_double3] = []


    for row in 0...(height - 1) {
        for col in 0...(width - 1) {
//            let y = ((height - row - 1) - height / 2)
//            let x = (col - width / 2)
            //let pos = CGPoint(x: x, y: y)

            let depth = rArray[row * width + col]
            let conf = confMap[row * width + col]
            
            if conf < 1.0 {
                continue
            }

            let vec3d = pinholeVec(x: Float(col), y: Float(row), depth: depth)
            
            let vecAligned = rotateEuler(vec: vec3d, roll: euler.pitch, pitch: abs(.pi / 2 + euler.roll))
            cloud.append(vecAligned)
            
            //cloud.append(vec3d)
        }
    }
    return cloud
}


public func alignPoint(_ p: simd_double3, euler: (roll: Double, pitch: Double, yaw: Double)) -> simd_double3 {
    let p1 = rotateX(vec: p, angleRad: -euler.yaw)
    let p2 = rotateZ(vec: p1, angleRad: -euler.roll)
    
    //let p3 = rotateY(vec: p2 - simd_double3(0, 0, 2), angleRad: .pi / 2) + simd_double3(0, 0, 2)
    
    return p2
}

public func to2dVector(_ p: simd_double3) -> simd_double2 {
    simd_double2(x: Double(p.x), y: Double(p.z))
}



public func horisontalSlice(rArray: [Float32], minY: Double = -0.06, maxY: Double = -0.059) -> [simd_double2] {
    let height = 192
    let width = 256
    
    var slice: [simd_double2] = []


    for row in 0...(height - 1) {
        for col in 0...(width - 1) {
            let y = ((height - row - 1) - height / 2)
            let x = (col - width / 2)
            //let pos = CGPoint(x: x, y: y)

            let depth = rArray[row * width + col]
            //print(row * height + col)

            //let vec3d = realDistVecLinear(centeredX: CGFloat(x), centeredY: CGFloat(y), depth: depth)
            let vec3d = pinholeVec(x: Float(col), y: Float(row), depth: depth)
            if vec3d.y <= maxY && vec3d.y > minY {

                let v2d = simd_double2(x: Double(vec3d.x), y: Double(vec3d.z))
                slice.append(v2d)

            }
        }

    }
    return slice
}



public func sliceToRibbons(slice: [simd_double2], maxPointDist: Double = 0.2, offset: Double) -> [LidarRibbon] {
    func belongsToCluster(cluster: [simd_double2], v: simd_double2) -> Bool {
        cluster.first{simd_distance_squared($0, v) < maxPointDist * maxPointDist} != nil
    }
    
    guard let prev = slice.first else {
        return []
    }

    var clusters: [[simd_double2]] = [[prev]]
    
    for v in slice.dropFirst() {
        
        if let clusterIndex = clusters.firstIndex(where: {belongsToCluster(cluster: $0, v: v)}) {
            clusters[clusterIndex].append(v)
        }else{
            clusters.append([v])
        }

    }
    
    
    var res: [LidarRibbon] = []
    for (i, c) in clusters.enumerated() {
        let sortedC = c.sorted{$0.x < $1.x}
        if let r = LidarRibbon.fromArray(name: "c\(i)", vectors: sortedC, offset: offset) {
            res.append(r)
        }
    }
    
    return res
}

public class NewNavigationAlgo{
    
    public static func intersectObs(start: simd_double2, finish: simd_double2, obs: [LidarRibbon]) -> [(simd_double2, LidarRibbon)] {
        intersectObs(path: [start, finish], map: obs)
    }

    public static func intersectObs(path: [simd_double2], map: [LidarRibbon]) -> [(simd_double2, LidarRibbon)] {
        var acc: [(simd_double2, LidarRibbon)] = []
        if var from = path.first {
            for to in path.dropFirst() {
                for o in map {
                    let (p1, p2) = o.intersectionLine()
                
                    if let isect = intersect(p1: from, p2: to, p3: p1, p4: p2), (simd_distance_squared(isect, p2) > 0.001 && simd_distance_squared(isect, p1) > 0.001) {
                        acc.append((isect, o))
                    }
                }
                
                from = to
            }
            
            acc.sort{$0.0.y < $1.0.y}
            
        }
        
        return acc
    }

    public static func plotPathNextPoint(
        finish: simd_double2,
        obs: [LidarRibbon],
        turnRadius: Double) -> simd_double2? {
            let start = simd_double2(0, 0)
            
            func bestDetourPoint(p1: simd_double2, p2: simd_double2, finish: simd_double2) -> simd_double2? {
                
                let intersectP1 = !intersectObs(path: trajectoryPrediction(to: p1), map: obs).isEmpty
                
                let intersectP2 = !intersectObs(path: trajectoryPrediction(to: p2), map: obs).isEmpty
                
                if intersectP1 && intersectP2 {
                    return nil
                }else if intersectP1 {
                    return p2
                }else if intersectP2 {
                    return p1
                }else if angle(v1: p1, v2: finish) < angle(v1: p2, v2: finish) {
                    return p1
                }else {
                    return p2
                }
            }
            
            func adjustVectorLength(point: simd_double2) -> simd_double2{
                let minVLen = trajectoryLocal(turnRadius: turnRadius, dest: normalize(point) * 10)!.endOfSteerPoint.length()
                if length(point) < minVLen {
                    return start + simd_normalize(point) * minVLen
                }else {
                    return point
                }
            }
        
        let pred = trajectoryPrediction(to: finish, turnRadius: turnRadius)
        let isects = intersectObs(path: pred, map: obs)
        if pred.isEmpty {
            return nil
        } else if distance(start, finish) < 0.01 {
            return nil
        } else if let (impact, impactedShape) = isects.first {
            let (p1, p2) = impactedShape.intersectionLine()
            let adj1 = adjustVectorLength(point: p1)
            let adj2 = adjustVectorLength(point: p2)
            return bestDetourPoint(p1: adj1, p2: adj2, finish: finish)

        }else {
            return finish
        }
        

    }
    
    public static func trajectoryPrediction(to: simd_double2, turnRadius: Double = 0.6, step: Double = 0.1) -> [simd_double2] {
        guard let trajectoryModel = trajectoryLocal(turnRadius: turnRadius, dest: to) else {
            return []
        }
        
        let len = InstantTrajectory2.length(t: trajectoryModel)

        var predictedTraj: [simd_double2] = []
        for i in stride(from: 0.0, to: len, by: step){
            let localPoi = InstantTrajectory2.pointAt(traj: trajectoryModel, dist: i)
            let poi = localPoi
            
            predictedTraj.append(poi)
        }
        return predictedTraj
    }
    
    public static func trajectoryPrediction(pose: VehiclePose, distanceMade: Double, trajectoryModel: TrajectoryModel) -> [simd_double2]{
        let predictedPose = InstantTrajectory2.poseAt(traj: trajectoryModel, dist: distanceMade)
        let len = InstantTrajectory2.length(t: trajectoryModel)
        
        let drift = predictedPose - pose + VehiclePose.zeroPose
                
        var predictedTraj: [simd_double2] = []
        
        for i in stride(from: distanceMade, to: len, by: 0.15){
            let localPoi = InstantTrajectory2.pointAt(traj: trajectoryModel, dist: i)
            let poi = localPoi.global(pose: drift)
            
            predictedTraj.append(poi)
        }
        
        return predictedTraj
    }

    
    public static func updateWorld(pose: VehiclePose, distanceMade: Double, maybeState: JournyState, localFinish: simd_double2, map: [LidarRibbon]) -> JournyState {

        
        func atFinish() -> Bool{
            if let trajectoryModel = maybeState.trajectoryModel {
                return distanceMade >= InstantTrajectory2.length(t: trajectoryModel)
            }else {
                return true
            }
        }
        
        
        func vehicleDrifted(_ pred: [simd_double2], dest: simd_double2) -> Bool {
            if let last = pred.last {
                return distance(last, dest) > 0.1
            }else{
                return false
            }
            
            
            
        }
        
        func willCrash(_ pred: [simd_double2]) -> Bool {
            !NewNavigationAlgo.intersectObs(path: pred, map: map).isEmpty
        }
        
        func driftedOrWillCrash() -> Bool  {
            if let trajectoryModel = maybeState.trajectoryModel {
                let localPose = pose.local(frame: maybeState.startPose)
                let pred = trajectoryPrediction(pose: localPose, distanceMade: distanceMade, trajectoryModel: trajectoryModel)
                let driftedRes = vehicleDrifted(pred, dest: trajectoryModel.destPose.head)
//                let driftedRes = false
//                let willCrashRes = false
                let willCrashRes = willCrash(pred)
                if driftedRes {print("drifted: ", driftedRes)}
                if willCrashRes {print("will crash: ", willCrashRes)}
//                print("drifted: ", driftedRes)
//                print("will crash: ", willCrashRes)
                return willCrashRes
            }else {
                return false
            }
        }
        
        func shouldChangeTraj() -> Bool {
            let fin = atFinish()
            let drifted = driftedOrWillCrash()
            if fin {print("at finish: ", fin)}
            //if drifted {print("will crash: ", drifted)}
            return fin || drifted
        }
        
        if shouldChangeTraj() {
            let maybeSubDest = plotPathNextPoint(finish: localFinish, obs: map, turnRadius: 0.6)
            //print("next point", maybeSubDest)
            if let newSubDest = maybeSubDest, let traj = trajectoryLocal(turnRadius: 0.6, dest: newSubDest) {

                print("NEW STATE")
                return JournyState(trajectoryModel: traj, distBeforeManeuver: maybeState.distBeforeManur + distanceMade, startPose: pose)
            }else{
                print("NEW STATE STOP")
                print("can't build next point")
                return JournyState(trajectoryModel: nil, distBeforeManeuver: maybeState.distBeforeManur + distanceMade, startPose: pose)
            }
            
        }else{
            return maybeState
        }
    }
}

public class JournyState{
    public let trajectoryModel: TrajectoryModel?
    public let distBeforeManur: Double
    public let startPose: VehiclePose
    
    public init(trajectoryModel: TrajectoryModel?, distBeforeManeuver: Double, startPose: VehiclePose) {
        self.trajectoryModel = trajectoryModel
        self.distBeforeManur = distBeforeManeuver
        self.startPose = startPose
    }
    
    public func localPoseAt(distTraveled: Double) -> VehiclePose {
        if let traj = trajectoryModel {
            return InstantTrajectory2.poseAt(traj: traj, dist: distTraveled - distBeforeManur)
        }else {
            return  VehiclePose.zeroPose
        }

    }
    
    public func globalPoseAt(distTraveled: Double) -> VehiclePose {


        if let traj = trajectoryModel {
            let trajDist = distTraveled - distBeforeManur
            return InstantTrajectory2.poseAt(traj: traj, dist: trajDist).global(frame: startPose)
        }else {
            return  startPose
        }
    }
}



public class LidarRibbon{
    public let name: String
    public let vectors: [simd_double2]
    public let left: simd_double2
    public let right: simd_double2

    let offset: Double
    public let lines: [(simd_double2, simd_double2)]
    
    init(name: String, vectors: [simd_double2], left: simd_double2, right: simd_double2, offset: Double) {
        self.lines = LidarRibbon.intersectionLines(vectors: vectors, offset: offset)
        self.name = name
        self.vectors = vectors
        self.left = left
        self.right = right
        self.offset = offset
    }
    
    public func local(pose: VehiclePose) -> LidarRibbon {
        let localVectors = vectors.map{$0.local(pose: pose)}
        return LidarRibbon.fromArray(name: self.name, vectors: localVectors, offset: offset)!
    }

    public func center() -> simd_double2 {
        vectors.reduce(simd_double2(0, 0)){ $0 + $1 / Double(vectors.count) }
    }
    
    public func intersectionLine() -> (simd_double2, simd_double2) {

        return lines.first ?? (simd_double2(x: 999, y: 0), simd_double2(x: 999, y: 0))
        let minY = vectors.min{$0.y < $1.y}!.y
        //let maxY = vectors.max{$0.y < $1.y}!.y
        assert(!minY.isNaN)
        return (
            simd_double2(x: left.x - offset, y: minY),
            simd_double2(x: right.x + offset, y: minY)
        )
    }

    static func intersectionLines(vectors: [simd_double2], offset: Double) -> [(simd_double2, simd_double2)] {
        let minY = vectors.min{$0.y < $1.y}!.y
        let maxY = vectors.max{$0.y < $1.y}!.y
        let xOffset = 0.1
        var res: [(simd_double2, simd_double2)] = []

        for from in stride(from: minY, to: maxY, by: offset){
            let to = from + offset
            let portion = vectors.filter {$0.y >= from && $0.y < to}

            if let minX = portion.min(by: {$0.x < $1.x})?.x, let maxX = portion.max(by: {$0.x < $1.x})?.x {
                let line = (simd_double2(x: minX - offset, y: from - xOffset),
                        simd_double2(x: maxX + offset, y: from - xOffset))
                res.append(line)
            }
        }

        return res
    }


    
    static func fromArray(name: String, vectors: [simd_double2], offset: Double) -> LidarRibbon? {
        if let left = vectors.first, let right = vectors.last {
            return LidarRibbon(name: name, vectors: vectors, left: left, right: right, offset: offset)
        }else{
            return nil
        }
    }
}
