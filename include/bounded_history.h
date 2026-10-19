#pragma once
#include <cstddef>
#include <queue>

template<class Container>
void trimFront(Container& container, int limit) {
    if (limit > 0 && container.size() > static_cast<std::size_t>(limit))
        container.erase(container.begin(), container.begin() + (container.size()-limit));
}

template<class T>
void trimQueue(std::queue<T>& queue, int limit) {
    if (limit > 0)
        while (queue.size() > static_cast<std::size_t>(limit)) queue.pop();
}

template<class Map>
void trimPoseMapByCount(Map& poses, int limit) {
    if (limit > 0)
        while (poses.size() > static_cast<std::size_t>(limit)) poses.erase(poses.begin());
}

template<class Map>
void trimPoseMapBefore(Map& poses, double oldest) {
    while (!poses.empty() && poses.begin()->first < oldest) poses.erase(poses.begin());
}
