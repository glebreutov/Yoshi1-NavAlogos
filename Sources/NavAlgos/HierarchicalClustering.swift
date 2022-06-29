//
// Created by Gleb Reutov on 18.10.2021.
//

import Foundation
import simd

public class HierarchicalClustering {

    static func distToCluster(d: simd_double2, clust: [simd_double2]) -> Double? {
        clust.map{distance_squared($0, d)}.min().map(sqrt)
    }

    static func nearestClusterIdx(d: simd_double2, clust: [[simd_double2]], dist: Double) -> Int? {
        let clusterDistances = clust.map{distToCluster(d: d, clust: $0) ?? dist * 1000}

        let nearestIdx = zip(0...clusterDistances.count, clusterDistances)
                .filter{$0.1 <= dist}
                .min{$0.1 < $1.1}
                .map{$0.0}

        return nearestIdx
    }
    static func mergeClusters(clust: [[simd_double2]], idx1: Int, idx2: Int) -> [[simd_double2]] {
        let merged = clust[idx1] + clust[idx2]

        var newClusters = [merged]

        for (idx, c) in zip(0...clust.count, clust) {
            if idx != idx1 && idx != idx2 {
                newClusters.append(c)
            }
        }
        return  newClusters
    }
    static func joinClusters(clust: [[simd_double2]], dist: Double) -> [[simd_double2]] {
        let idxClust = zip(0...clust.count, clust)
        for (idx1, c1) in idxClust {
            for (idx2, c2) in idxClust {
                if idx1 != idx2 {
                    for p1 in c1 {
                        for p2 in c2 {
                            if distance(p1, p2) <= dist {
                                let newClusters = mergeClusters(clust: clust, idx1: idx1, idx2: idx2)
                                return joinClusters(clust: newClusters, dist: dist)
                            }
                        }
                    }
                }
            }
        }

        return clust
    }

    public static func reclaculateClusters(clust: [[simd_double2]], dist: Double) -> [[simd_double2]] {
        var newClust: [[simd_double2]] = Array(repeating: [], count: clust.count)

        //let flatClusters: FlattenSequence<Array<[(Int, simd_double2)]>> = zip(0...clust.count, clust).map{idx, clust in clust.map{(idx, $0)}}.joined()

        let flatClusters = clust.joined()
        for p in flatClusters {
            if let idx = nearestClusterIdx(d: p, clust: clust, dist: dist){
                newClust[idx].append(p)
            }

        }

        return newClust
    }

    public static func initClusters(slice: [simd_double2], dist: Double) ->[[simd_double2]] {
        var clusters: [[simd_double2]] = []
        for d in slice {
            if let idx = nearestClusterIdx(d: d, clust: clusters, dist: dist) {
                clusters[idx].append(d)
            }else {
                clusters.append([d])
            }
        }
        return clusters
    }

    public static func findClusters(slice: [simd_double2], dist: Double, filterOutClustersLess: Int = 2)->[[simd_double2]] {
        joinClusters(clust: initClusters(slice: slice, dist: dist), dist: dist)
                .filter{$0.count > filterOutClustersLess}
    }
}