#include "mappoint.hpp"
#include "feature.hpp"

void MapPoint::RemoveObservation(std::shared_ptr<Feature> feature) {
  std::unique_lock<std::mutex> lock(mutex_);
  for (auto iter = observations_.begin(); iter != observations_.end(); iter++) {
    if (iter->lock() == feature) {
      observations_.erase(iter);
      feature->map_point_.reset();
      observed_times_--;
      break;
    }
  }
}

MapPoint::Ptr MapPoint::CreateNewMapPoint() {
  static unsigned long factory_id = 0;
  MapPoint::Ptr new_map_point(new MapPoint);
  new_map_point->id_ = factory_id++;
  return new_map_point;
}