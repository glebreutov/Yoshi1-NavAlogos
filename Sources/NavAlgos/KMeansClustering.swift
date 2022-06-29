//
// Created by Gleb Reutov on 18.10.2021.
//

import Foundation
import simd

public class KMeansClustering {
    static func randomClusters(d: [simd_double3], k: Int) -> [[simd_double3]] {
        var clusters: [[simd_double3]] = Array(repeating: [], count: k)
        for p in d {
            let randK = Int.random(in: 0...(k-1))
            clusters[randK].append(p)
        }
        return clusters
    }

    static func nearsetClusters(d: [simd_double3], centers: [simd_double3]) -> [[simd_double3]] {
        var clusters: [[simd_double3]] = Array(repeating: [], count: centers.count)
        for p in d {
            let min = zip(0...centers.count, centers).min{
                distance(p, $0.1) < distance(p, $1.1)
            }
            if let (k, _) = min {
                clusters[k].append(p)
            }
        }
        return clusters
    }

    public static func clusterCenter(d: [simd_double3]) -> simd_double3 {
        if d.isEmpty {
            return simd_double3(0,0,0)
        }else {
            return d.reduce(simd_double3(0,0,0), +) / Double(d.count)
        }

    }

    static func centersDiff(c1: [simd_double3], c2: [simd_double3]) -> Double {
        zip(c1, c2).map{distance($0, $1)}.reduce(0, +)
    }

    public static func kMean(ds: [simd_double3], k: Int) -> [[simd_double3]] {
        let clusters: [[simd_double3]] = randomClusters(d: ds, k: k)
        return kMeanIter(clusters: clusters)
    }

    static func kMeanIter(clusters: [[simd_double3]]) -> [[simd_double3]] {

        let centers = clusters.map(clusterCenter)
        let newClusters = nearsetClusters(d: Array(clusters.joined()), centers: centers)
        let newCenters = newClusters.map(clusterCenter)
        let diff = centersDiff(c1: centers, c2: newCenters)
        print(diff)
        if diff < 0.0000001 {
            return newClusters
        }else{
            return kMeanIter(clusters: newClusters)
        }
    }
}
