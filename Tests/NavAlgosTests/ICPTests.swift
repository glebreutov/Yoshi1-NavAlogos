//
//  File.swift
//  
//
//  Created by Gleb Reutov on 13.10.2021.
//

import Foundation
import XCTest
@testable import NavAlgos
import simd
final class ICPTests: XCTestCase {
    
    func testDoNothing() throws {
        let src = [simd_double2(x: 0, y: 1)]
        let dest = [simd_double2(x: 0, y: 1)]
        
        let maybeRes = approxPosition(
            source: src,
            dest: dest,
            maxOffset: 0,
            minOffset: 0,
            angleZero: 0,
            angleMaxMagnitude: 0)
        
        
        if let res = maybeRes {
            assert(abs(res.angle) < 0.00001)
            assert(length(res.offset) < 0.00001)
            assert(res.error < 0.00001)
        }else{
            assertionFailure()
        }
    }
    
    func testVectorRotate() throws {
        let a = -1.047197545928283
        //[0]    SIMD2<Double>    (-1.7320508075688772, 1.0000000000000002)
        let v = simd_double2(x: -1.7320508075688772, y: 1.0000000000000002)
        let r = rotateVec(v: v, angle: a)
        
        assert(length(r) > 0)
    }
    
    
    func testFindRealTransformation() throws {
        let src = simd_double2(x: 0, y: 2)
        let dest = rotateVec(v: src / 2, angle: .pi / 3)
        
        let maybeRes = approxPosition(
            source: [src],
            dest: [dest],
            maxOffset: 4,
            minOffset: 0,
            angleZero: 0,
            angleMaxMagnitude: 2 * .pi / 3,
            minError: 0.01
        )
        
        
        if let res = maybeRes {
            let transform = res.offset + rotateVec(v: dest, angle: res.angle)
            let d = distance(src, transform)
            print(d)
            assert(d < 0.001)
        }else{
            assertionFailure()
        }
    }
     
}

