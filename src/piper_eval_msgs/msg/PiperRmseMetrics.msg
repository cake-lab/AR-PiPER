std_msgs/Header header

#Sample counts
int32 total_samples
int32 window_samples

#Position metrics (mm)
float64 cumulative_pos_rmse_mm
float64 window_pos_rmse_mm
float64 instantaneous_pos_error_mm
float64 pos_error_x_mm
float64 pos_error_y_mm
float64 pos_error_z_mm
float64 pos_max_error_mm
float64 pos_p95_error_mm

#Orientation metrics (degrees)
float64 cumulative_orient_rmse_deg
float64 window_orient_rmse_deg
float64 instantaneous_orient_error_deg
float64 orient_max_error_deg

#Performance metrics
float64 data_rate_hz
float64 avg_processing_time_ms
bool data_quality_good
