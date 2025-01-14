#ifndef MARINE_RADAR_TRACKER_TARGET_H
#define MARINE_RADAR_TRACKER_TARGET_H

#include <grid_map_ros/grid_map_ros.hpp>

namespace marine_radar_tracker
{

struct BoundingBox
{
  grid_map::Position min;
  grid_map::Position max;

  BoundingBox(const grid_map::Position &p):min(p),max(p)
  {
  }

  BoundingBox(const BoundingBox &other):min(other.min),max(other.max)
  {

  }

  void expand(const grid_map::Position &p)
  {
    min.x() = std::min(min.x(), p.x());
    min.y() = std::min(min.y(), p.y());
    max.x() = std::max(max.x(), p.x());
    max.y() = std::max(max.y(), p.y());
  }

  void expand(const BoundingBox &bb)
  {
    expand(bb.min);
    expand(bb.max);
  }


};

// collection of points that touch each other
class Blob
{
public:
  Blob(ros::Time stamp, double resolution):stamp_(stamp), resolution_(resolution)
  {

  }

  void add(grid_map::Position position, double intensity)
  {
    if(intensity > 0.0)
    {
      points_.push_back(position);
      position_sum_ += position*intensity;
      total_intensity_ += intensity;
    }
    else
      perimeter_points_.push_back(position);
  }

  bool empty() const
  {
    return points_.empty();
  }

  std::size_t size() const
  {
    return points_.size();
  }

  BoundingBox bounds() const
  {
    BoundingBox ret(points_.front());
    for(auto p: points_)
      ret.expand(p);
    return ret;
  }

  grid_map::Position centroid() const
  {
    if(total_intensity_ > 0.0)
      return position_sum_/total_intensity_;
    return position_sum_;
  }

  // in units of resolution
  double effectiveRadius() const
  {
    if(!points_.empty())
      return sqrt(double(points_.size())/M_PI)*resolution_;
    //if(total_intensity_ > 0.0)
      //return sqrt(total_intensity_/M_PI)*resolution_;
    return 0.0;
  }

  double averageIntensity() const
  {
    if(!points_.empty())
      return total_intensity_/double(points_.size());
    return 0.0;
  }

  double circularity() const
  {
    auto c = centroid();
    double r = effectiveRadius();
    double error2_sum = 0.0;
    for(auto p: perimeter_points_)
    {
      auto d = (c-p).norm();
      error2_sum += pow(d-r,2.0);
    }
    if (error2_sum > 0.0)
    {
      double stdev = sqrt(error2_sum/perimeter_points_.size());
      return 1.0/(1.0+stdev/r);
    }
    return 0.0;
  }

  ros::Time timestamp() const
  {
    return stamp_;
  }

private:
  std::vector<grid_map::Position> points_;
  std::vector<grid_map::Position> perimeter_points_;
  ros::Time stamp_;
  double resolution_;
  double total_intensity_ = 0.0;
  grid_map::Position position_sum_ = grid_map::Position(0.0, 0.0);

};

class Target
{
public:
  Target(std::shared_ptr<Blob> blob, uint16_t id):id_(id)
  {
    addBlob(blob);
  }

  void addBlob(std::shared_ptr<Blob> blob)
  {
    blobs_.push_back(blob);
  }

  // clears older blobs and returns true if blobs remaining
  bool update(ros::Time current_time)
  {
    // detect restart of sim or data replay
    if(!blobs_.empty() && blobs_.back()->timestamp() > current_time)
      blobs_.clear();

    auto expired = current_time - history_;
    while(!blobs_.empty() && blobs_.front()->timestamp() < expired)
      blobs_.erase(blobs_.begin());
      
    return !blobs_.empty();
  }

  BoundingBox bounds() const
  {
    BoundingBox ret = blobs_.back()->bounds();
    for(auto b: blobs_)
      ret.expand(b->bounds());
    return ret;
  }

  grid_map::Position centroid() const
  {
    grid_map::Position ret(0.0, 0.0);
    double total_weight = 0.0;
    for(auto b: blobs_)
    {
      auto weight = 1.0 - (blobs_.back()->timestamp()-b->timestamp()).toSec()/history_.toSec();
      if(weight > 0.0)
      {
        ret += b->centroid()*weight;
        total_weight += weight;
      }
    }
    if(total_weight > 0.0)
      return ret/total_weight;
    return ret;
  }

  // in units of resolution
  double effectiveRadius() const
  {
    double ret = 0.0;
    double total_weight = 0.0;
    for(auto b: blobs_)
    {
      auto weight = 1.0 - (blobs_.back()->timestamp()-b->timestamp()).toSec()/history_.toSec();
      if(weight > 0.0)
      {
        ret += b->effectiveRadius()*weight;
        total_weight += weight;
      }
    }
    if(total_weight > 0.0)
      return ret/total_weight;
    return ret;
  }

  double distanceScore(std::shared_ptr<Blob> blob) const
  {
    double distance = (blob->centroid() - centroid()).norm();
    return std::max(effectiveRadius(), blob->effectiveRadius())/distance;
  }

  std::size_t blobCount() const
  {
    return blobs_.size();
  }

  ros::Time latestUpdate() const
  {
    if(!blobs_.empty())
      return blobs_.back()->timestamp();
    return ros::Time();
  }

  uint16_t id() const
  {
    return id_;
  }

private:
  std::list<std::shared_ptr<Blob> > blobs_;
  uint16_t id_;
  ros::Duration history_ = ros::Duration(10.0);
};

} // namespace marine_radar_tracker

#endif
