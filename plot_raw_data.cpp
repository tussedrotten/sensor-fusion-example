#include "image_landmark_track_source.h"
#include "imu_source.h"
#include "gps_source.h"
#include "raw_data_plotter.h"

int main()
{
  // Create data sources.
  GpsSource gps_src("../data/GPS_Data_0000.bin");
  ImuSource imu_src("../data/IMU_Data_0000.bin");
  ImageLandmarkTrackSource track_src("../data/image_data.txt");

  // Initialize local coordinate system in plotter with first GPS measurement.
  RawDataPlotter plotter(gps_src.next());

  // Add data to plotter (chronologically to show how this can be done).
  while (gps_src.hasNext() || imu_src.hasNext() || track_src.hasNext())
  {
    if (imu_src.nextTime() < track_src.nextTime() && imu_src.nextTime() < gps_src.nextTime())
    {
      plotter.addImuData(imu_src.next());
      imu_src.pop();
    }
    else if (track_src.nextTime() < gps_src.nextTime())
    {
      plotter.addTrackData(track_src.next());
      track_src.pop();
    }
    else
    {
      plotter.addGpsData(gps_src.next());
      gps_src.pop();
    }
  }

  // Plot data.
  plotter.plot();

  return EXIT_SUCCESS;
}
