import XCTest
@testable import NavAlgos
import simd
final class LocalTrajectoryTest: XCTestCase {

    func testAxisAngle2() {
        XCTAssertEqual(axisAngle2(v: simd_double2(0, 1)), .pi / 2, accuracy: 0.001)
        XCTAssertEqual(axisAngle2(v: simd_double2(1, 1)), .pi / 4, accuracy: 0.001)
        XCTAssertEqual(axisAngle2(v: simd_double2(1, 0)), 0, accuracy: 0.001)
        XCTAssertEqual(axisAngle2(v: simd_double2(1, -1)), 3 * .pi / 2 + .pi / 4, accuracy: 0.001)
        XCTAssertEqual(axisAngle2(v: simd_double2(0, -1)), .pi + .pi / 2, accuracy: 0.001)
        XCTAssertEqual(axisAngle2(v: simd_double2(-1, -1)), .pi + .pi / 4, accuracy: 0.001)
        XCTAssertEqual(axisAngle2(v: simd_double2(-1, 0)), .pi, accuracy: 0.001)
        XCTAssertEqual(axisAngle2(v: simd_double2(-1, 1)), .pi / 2 + .pi / 4, accuracy: 0.001)



    }



    func testOldVsNew() {
        let d = simd_double2(x: 0.4, y: 1)

        print()

        if let m1 = trajectoryLocal(turnRadius: 0.6, dest: d), let m2 = InstantTrajectory2.simpleTrajectory(dest: d, r: 0.6) {
            XCTAssertEqual(distance(m1.turnPoint0, m2.turnPoint0), 0.0, accuracy: 0.001)
            XCTAssertEqual(distance(m1.turnPoint1, m2.turnPoint1), 0.0, accuracy: 0.001)
            XCTAssertEqual(distance(m1.endOfSteerPoint, m2.endOfSteerPoint), 0.0, accuracy: 0.001)
            XCTAssertEqual(distance(m1.destPose.head, m2.destPose.head), 0.0, accuracy: 0.001)
            XCTAssertEqual(distance(m1.crossoverPoint, m2.crossoverPoint), 0.0, accuracy: 0.001)
//            XCTAssertEqual(distance(m1.destPose.tail, m2.startPose.tail), 0.0, accuracy: 0.001)
        }else {
            assertionFailure("something is nil")
        }

        let d2 = simd_double2(x: -0.4, y: 1)
        if let m1 = trajectoryLocal(turnRadius: 0.6, dest: d2), let m2 = InstantTrajectory2.simpleTrajectory(dest: d2, r: 0.6) {
            XCTAssertEqual(distance(m1.turnPoint0, m2.turnPoint0), 0.0, accuracy: 0.001)

            XCTAssertEqual(distance(m1.turnPoint1, m2.turnPoint1), 0.0, accuracy: 0.001)
            XCTAssertEqual(distance(m1.endOfSteerPoint, m2.endOfSteerPoint), 0.0, accuracy: 0.001)
            XCTAssertEqual(distance(m1.destPose.head, m2.destPose.head), 0.0, accuracy: 0.001)
            XCTAssertEqual(distance(m1.crossoverPoint, m2.crossoverPoint), 0.0, accuracy: 0.001)
        }else {
            assertionFailure("something is nil")
        }

        //print("test output")

    }


    func testSpecificPoint() {
        let v1 = simd_double2(-0.9340828668022203, 0.10834715666646699)
        let t = trajectoryLocal(turnRadius: 0.6, dest: v1)
        print(t?.destPose)
        XCTAssertNotNil(t)
    }


}
