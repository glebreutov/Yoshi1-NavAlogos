//
//  File.swift
//  
//
//  Created by Gleb Reutov on 13.10.2021.
//

import Foundation
import simd

func findNearestVectorDistance(q: simd_double2, frame: [simd_double2]) -> Double {
    frame.map{v in simd_distance_squared(v, q)}.min() ?? 0.0
}

func fitness(reference: [simd_double2], candidate: [simd_double2]) -> Double {
    candidate.map{ q in findNearestVectorDistance(q: q, frame: reference)}
    .reduce(0){$0 + $1 / Double(candidate.count)}
}

func alignCandidate(candidate: [simd_double2], offset: simd_double2, angle: Double) -> [simd_double2] {
    candidate.map{rotateVec(v: $0, angle: angle) + offset}
}

func approxPosition(
    source: [simd_double2],
    dest: [simd_double2],
    maxOffset: Double,
    minOffset: Double,
    angleZero: Double,
angleMaxMagnitude: Double,
    minError: Double = 5.5
) -> (angle: Double, offset: simd_double2, error: Double)?{
    

    let ancors = [(-1.0, 1.0), (-1.0, -1.0), (1.0, 1.0), (1.0, -1.0)]
    let halfLength = (maxOffset - minOffset) / 2
    let quaterLength = halfLength / 2
    
    //print("maxOffset \(maxOffset), minOffset: \(minOffset) zeroAngle: \(angleZero) maxMagnitude: \(angleMaxMagnitude)")

    let results = ancors.map{
        (fwd, side) -> ((pos: (fwd: Double, side: Double), error: Double)) in
        let len = halfLength + fwd * halfLength / 2
//        let max = v + quaterLength
//        let min = v - quaterLength
        let angle = angleZero + side * angleMaxMagnitude / 2
//        let newZero = angle - angleMaxMagnitude / 4
//        let newMangnitude = angleMaxMagnitude / 2
        let v = rotateVec(v: simd_double2(x: 0, y: len), angle: angle)
        //print(v)
        let candidate = alignCandidate(candidate: dest, offset: v, angle: angle)
        let error = fitness(reference: source, candidate: candidate)
        return (pos: (fwd, side), error: error)
    }
    
    if let (pos, error) = results.min(by: {$0.error < $1.error}) {
        
        let angle = angleZero + pos.side * angleMaxMagnitude / 2
        let len = halfLength + pos.fwd * halfLength / 2
        
        let bestBreed = rotateVec(v: dest[0], angle: angle) + rotateVec(v: simd_double2(x: 0, y: len), angle: angle)
        print("best from breed, \(bestBreed) angle: \(angle) len: \(len)")
        if error <= minError {
            let v = rotateVec(v: simd_double2(x: 0, y: len), angle: angle)
            return (angle: angle, offset: v, error: error)
        }else {
            let newMaxOffset = len + quaterLength
            let newMinOffset = len - quaterLength
            
            return autoreleasepool{
                approxPosition(
                    source: source,
                    dest: dest,
                    maxOffset: newMaxOffset,
                    minOffset: newMinOffset,
                    angleZero: angle,
                    angleMaxMagnitude: angleMaxMagnitude / 2,
                    minError: minError
                )
            }
        }
    }else{
        return nil
    }

}
