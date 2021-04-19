#include "gps_imu_fixed_lag_fuser.h"
#include "gps_source.h"
#include "imu_source.h"
#include "nav/attitude.h"
#include "trigonometry.h"

int main()
{
  // Create data sources.
  GpsSource gps_src("../data/GPS_Data_0000.bin");
  ImuSource imu_src("../data/IMU_Data_0000.bin");

  // Skip GPS data before init_time.
  const double init_time = 1485176340.0;
  while (gps_src.nextTime() < init_time)
  {
    gps_src.pop();
  }

  // Skip IMU data before first remaining GPS measurement.
  while (imu_src.nextTime() <= gps_src.nextTime())
  {
    imu_src.pop();
  }

  // Initialize fuser with first GPS measurement and (bad) guess on initial attitude.
  GpsImuFixedLagFuser fuser(
      gps_src.next(),
      nav::Attitude(0.0, 0.0, trig::deg2rad(45.0)),
      nav::Attitude(trig::deg2rad(10.0), trig::deg2rad(10.0), trig::deg2rad(180.0))
  );
  gps_src.pop();

  // Run fuser.
  while (gps_src.hasNext() || imu_src.hasNext())
  {
    if (imu_src.nextTime() < gps_src.nextTime())
    {
      fuser.addImuData(imu_src.next());
      imu_src.pop();
    }
    else
    {
      fuser.addGpsData(gps_src.next());
      gps_src.pop();
    }
  }

  return EXIT_SUCCESS;
}
