// Standalone regression: link with include/ikd-Tree/ikd_Tree.cpp and pcl_common.
#include <ikd-Tree/ikd_Tree.h>
#include <cassert>
#include <memory>
#include <iostream>

int main() {
    auto tree = std::unique_ptr<KD_TREE>(new KD_TREE());
    PointVector points;
    for (int x = 0; x < 100; ++x) {
        PointType p{};
        p.x = x;
        points.push_back(p);
    }
    tree->Build(points);
    BoxPointType box{{-1, -1, -1}, {70, 1, 1}};
    std::vector<BoxPointType> boxes{box};
    tree->Delete_Point_Boxes(boxes);
    PointVector before, after;
    tree->flatten(tree->Root_Node, before, NOT_RECORD);
    assert(before.size() == 30);
    // Explicitly populate export history, including lazily deleted nodes.
    PointVector scratch;
    tree->flatten(tree->Root_Node, scratch, DELETE_POINTS_REC);
    PointVector removed;
    tree->acquire_removed_points(removed);
    assert(!removed.empty());
    tree->Build(points);
    tree->Delete_Point_Boxes(boxes);
    scratch.clear();
    tree->flatten(tree->Root_Node, scratch, DELETE_POINTS_REC);
    for (int i = 0; i < 1000; ++i) {
        tree->discard_removed_points();
    }
    tree->flatten(tree->Root_Node, after, NOT_RECORD);
    assert(after.size() == before.size());
    PointVector nearest;
    std::vector<float> distances;
    tree->Nearest_Search(points[90], 1, nearest, distances);
    assert(nearest.size() == 1 && nearest[0].x == 90);
    removed.clear();
    tree->acquire_removed_points(removed);
    assert(removed.empty());
    std::cout << "PASS: discard preserves active map and nearest search\n";
}
