\\Azimuth(方位角)と距離算出

double AzimuthDistance(){
  imu::Quaternion quat = bno.getQuat();
    double w = quat.w();
    double x = quat.x();
    double y = quat.y();
    double z = quat.z();
  //方位角取得
  double azimuth= atan2(y,x)*180.0/M_PI;
    Serial.print(azimuth);
    Serial.println("degree");
  double turnpower;
    turnpower = currentlocation[2] - eulerdata[2];
    
    azidata[0] = turnpower;
    azidata[1] = distanceBetween(goalGPSdata2[0],goalGPSdata2[1],currentlocation[0],currentlocation[1]);
    Serial.print("\tDistance: ");
    Serial.println(azidata[1]);
  
}
