#include <ikd-Tree/ikd_Tree.h>
#include <cassert>
#include <iostream>
#include <memory>
#include <thread>

int main() {
    MANUAL_Q queue;
    Operation_Logger_Type operation{};
    // More than the former ring capacity, both FIFO order and correct back().
    for (int i = 0; i < 1000100; ++i) {
        operation.point.x = i;
        queue.push(operation);
        assert(queue.back().point.x == i);
        if (i >= 4) { assert(queue.front().point.x == i - 4); queue.pop(); }
    }
    queue.clear();
    assert(queue.empty() && queue.size() == 0);
    auto tree = std::unique_ptr<KD_TREE>(new KD_TREE());
    tree->disable_removed_point_history();
    tree->set_downsample_param(1.0);
    PointVector seed;
    for (int i = 0; i < 100; ++i) { PointType p{}; p.x = i; seed.push_back(p); }
    // Repeated Build used to leak the sentinel. ASan/LSan checks ownership.
    for (int i = 0; i < 10; ++i) tree->Build(seed);
    assert(tree->live_nodes.load() == 100);
    for (int frame = 0; frame < 100; ++frame) {
        PointVector points = seed;
        tree->Add_Points(points, true);
        tree->discard_removed_points();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    assert(tree->validnum() == 100);
    PointVector points;
    tree->acquire_removed_points(points);
    assert(points.empty());
    BoxPointType box{{-1,-1,-1},{60,1,1}};
    std::vector<BoxPointType> boxes{box};
    tree->Delete_Point_Boxes(boxes);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    assert(tree->validnum() == 40);
    tree->acquire_removed_points(points);
    assert(points.empty());
    PointVector nearest;
    std::vector<float> distances;
    tree->Nearest_Search(seed[90], 1, nearest, distances);
    assert(nearest.size() == 1 && nearest[0].x == 90);
    // Exercise asynchronous rebuilds and repeatedly move a bounded map.
    PointVector grid;
    for (int x = 0; x < 40; ++x)
        for (int y = 0; y < 40; ++y)
            for (int z = 0; z < 4; ++z) {
                PointType p{}; p.x=x; p.y=y; p.z=z; grid.push_back(p);
            }
    tree->Build(grid);
    for (int step = 1; step <= 40; ++step) {
        BoxPointType removed{{float(step-1),-1,-1},{float(step),41,5}};
        std::vector<BoxPointType> slabs{removed};
        tree->Delete_Point_Boxes(slabs);
        PointVector added;
        for (int y = 0; y < 40; ++y)
            for (int z = 0; z < 4; ++z) {
                PointType p{}; p.x=step+39; p.y=y; p.z=z; added.push_back(p);
            }
        tree->Add_Points(added, true);
        tree->discard_removed_points();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    assert(tree->validnum() == 6400);
    assert(tree->live_nodes.load() < 12800);
    assert(tree->rebuild_count.load() > 0);
    assert(tree->rebuild_capacity.load() == 0);
    std::cout << "PASS: FIFO, rebuild ownership, bounded repeated voxel insertion, deletion, nearest search\n";
}
