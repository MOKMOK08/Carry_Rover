const double goal_location[2] = {35.9185333, 139.9081672};
const double current_location[2] = {35.9185000, 139.9081000};

void setup() {
  Serial.begin(115200);

  Serial.println(Distance(current_location[0], current_location[1], goal_location[0], goal_location[1]), 6);
  Serial.println(Azimuth(current_location[0], current_location[1], goal_location[0], goal_location[1]), 6);
}

void loop() {

}

// ２点間の距離計算
double Distance(double lat1, double long1, double lat2, double long2){
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);

  return delta * 6372795;
}

// 方位角
double Azimuth(double lat1, double lng1, double lat2, double lng2) {
  double x1 = lng1 * M_PI / 180.0;
  double y1 = lat1 * M_PI / 180.0;
  double x2 = lng2 * M_PI / 180.0;
  double y2 = lat2 * M_PI / 180.0;
  double x_dif = x2 - x1;
  double azimuth = atan2(sin(x_dif), (cos(y1) * tan(y2) - sin(y1) * cos(x_dif))) * 180 / M_PI;

  return azimuth;
}