//
//  File.swift
//  
//
//  Created by Gleb Reutov on 13.10.2021.
//

import Foundation
import simd
import Accelerate

func findCorrespondences(frame1: [simd_double2], frame2: [simd_double2]) -> [(Int, Int, Double)] {
    var res: [(Int, Int, Double)] = []
    for (idx1, p) in frame1.enumerated() {
        var nearestIdx = -1
        var minDist = Double.greatestFiniteMagnitude
        for (idx2, j) in frame2.enumerated() {
            let distSq = simd_distance_squared(p, j)
            if distSq < minDist {
                nearestIdx = idx2
                minDist = distSq
            }
        }
        res.append((idx1, nearestIdx, minDist))
    }

    return res
}

func convergeErrorSum(frame1: [simd_double2], frame2: [simd_double2]) -> Double {
    findCorrespondences(frame1: frame1, frame2: frame2).map{$0.2}.reduce(0.0, +)
    
}

public func convergeErrorAverage(frame1: [simd_double2], frame2: [simd_double2]) -> Double{
    convergeErrorSum(frame1: frame1, frame2: frame2) / Double(frame1.count)
}


func error(p: simd_double2, q: simd_double2, rot: simd_double2x2, shift: simd_double2) -> simd_double2 {
    rot * p + shift - q
}

private func dR(theta: Double) -> simd_double2x2 {
    simd_double2x2(rows: [
        simd_double2(-sin(theta), -cos(theta)),
        simd_double2(cos(theta), -sin(theta)),
    ])
}

private func r(_ theta: Double) -> simd_double2x2 {
    simd_double2x2(rows: [
        simd_double2(cos(theta), -sin(theta)),
        simd_double2(sin(theta), cos(theta)),
    ])
}

private func jacobian(theta: Double, point: simd_double2) -> simd_double3x2 {
    //let theta = pose.z
    let rot = dR(theta: theta) * point
    return simd_double3x2(rows: [
        simd_double3(1, 0, rot.x),
        simd_double3(0, 1, rot.y)
    ])
}



func alignCandidate(candidate: [simd_double2], offset: simd_double2, angle: Double) -> [simd_double2] {
    candidate.map{rotateVec(v: $0, angle: angle) + offset}
}

func thresholdErrorValue(percent: Int, errors: [Double]) -> Double{
    let idx = Int((errors.count / 100) * percent)
    
    return errors[idx]
}

func kernel(threshold: Double,  p1: simd_double2, p2: simd_double2, err: simd_double2) -> Double {
    if simd_length_squared(err) > 0.03 {
        return 0.0
    }else{
        return 1.0
    }
    
//    return 1.0
}


@available(macOS 10.13, *)
public func icp_run(frame1: [simd_double2], frame2: [simd_double2], maxIter: Int = 10) -> (dx: Double, dy: Double, theta: Double){
    var dx = 0.0
    var dy = 0.0
    var theta = 0.0
    let STOP_THRESHOLD = 0.0001
    var iterations = 0
    
    //print("updated 1")
    
    for i in 0 ... maxIter {
        let offset = simd_double2(dx, dy)
        let nv = icp_step(frame1: frame1, frame2: transform(frame: frame2, offset: offset, theta: theta))

        iterations = i
        
        dx = dx + nv[0]
        dy = dy + nv[1]
        theta = atan2(sin(theta + nv[2]), cos(theta + nv[2]))

        
        
        if i > 5 && nv[0] < STOP_THRESHOLD && nv[1] < STOP_THRESHOLD && nv[2] < STOP_THRESHOLD {
            break
        }
    }
    
    print("icp iterations: \(iterations)")
    return (dx: dx, dy: dy, theta: theta)
    
}

@available(macOS 10.13, *)
public func icp_step(frame1: [simd_double2], frame2: [simd_double2]) -> simd_double3 {
    
    let correspondencies = findCorrespondences(frame1: frame1, frame2: frame2)
    let topError = correspondencies.map({$0.2}).sorted(by: >)
    let threshold = thresholdErrorValue(percent: 20, errors: topError)
    let shift = simd_double2(0, 0)
    let angle = 0.0
    
    var H = simd_double3x3()
    var G = simd_double3()
    
    for (i, j, _) in correspondencies {

        let p = frame1[i]
        let q = frame2[j]
        let err = error(p: p, q: q, rot: r(angle), shift: shift)
        let jacobian = jacobian(theta: angle, point: p)
        
        let hessian = jacobian.transpose * jacobian
        let gradient = jacobian.transpose * err

        let weight = kernel(threshold: threshold, p1: p, p2: q, err: err)
        
        //let weight = 1.0 / (1.0 + simd_length_squared(err))
        H = H + weight * hessian
        G = G + weight * gradient
        //let solution = -1 * gradient * hessian.inverse
    }
    

    return (H.transpose * H).inverse * H.transpose * (G * -1)
}


public func transform(frame: [simd_double2], offset: simd_double2, theta: Double) -> [simd_double2] {
    frame.map({
        rotateVec(v: $0, angle: -1 * theta)
        - offset
    })
}

public func transform3d(frame: [simd_double3], offset: simd_double2, theta: Double) -> [simd_double3] {
let offset3d = simd_double3(x: offset.x, y: 0, z: offset.y)
    return frame.map({
        rotateY(vec: $0, angleRad: -1 * theta)  - offset3d
        
    })
}
