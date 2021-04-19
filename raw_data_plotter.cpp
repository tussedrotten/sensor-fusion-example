#include "raw_data_plotter.h"
#include "image_landmark_track_source.h"

#include "nav/navlab_utils.h"
#include "plot/plot.h"

RawDataPlotter::RawDataPlotter(const nav::NavlabGPS& init_gps)
: local_(init_gps.pos_geo)
{ }

using plot::Plot;

void RawDataPlotter::addGpsData(const nav::NavlabGPS& gps)
{
  if (gps.isValid())
  {
    const auto pos_local = local_.toLocalPosition(gps.pos_geo);

    x.push_back(pos_local.x());
    y.push_back(pos_local.y());
    h.push_back(-pos_local.z());
    gps_time.push_back(gps.time);
  }
}

void RawDataPlotter::addImuData(const nav::NavlabIMU& imu)
{
  imu_time.push_back(imu.time);
  drot_x.push_back(imu.drot.x());
  drot_y.push_back(imu.drot.y());
  drot_z.push_back(imu.drot.z());
  dvel_x.push_back(imu.dvel.x());
  dvel_y.push_back(imu.dvel.y());
  dvel_z.push_back(imu.dvel.z());
}

void RawDataPlotter::addTrackData(const TrackingFrame& track_frame)
{
  track_time.push_back(track_frame.time);
  num_tracks.push_back(track_frame.tracks.size());
}

void RawDataPlotter::plot()
{
  Plot::subplot(2, 2, 1);
  Plot::title("GPS track in local coordinates");
  Plot::plot(y, x, "g");
  Plot::axis("equal");

  Plot::subplot(2, 2, 2);
  Plot::title("Local height and number of tracks");
  Plot::plotLabeled(gps_time, h, "g", "Height");
  Plot::plotLabeled(track_time, num_tracks, "b", "Number of tracks");
  Plot::legend();

  Plot::subplot(2, 2, 3);
  Plot::title("IMU rotation");
  Plot::plotLabeled(imu_time, drot_x, "r", "drot x");
  Plot::plotLabeled(imu_time, drot_y, "g", "drot y");
  Plot::plotLabeled(imu_time, drot_z, "b", "drot z");
  Plot::legend();

  Plot::subplot(2, 2, 4);
  Plot::title("IMU velocity");
  Plot::plotLabeled(imu_time, dvel_x, "c", "dv x");
  Plot::plotLabeled(imu_time, dvel_y, "m", "dv y");
  Plot::plotLabeled(imu_time, dvel_z, "y", "dv z");
  Plot::legend();

  Plot::show();
}
