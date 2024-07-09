void P_GPS_Moter(){ 
    Serial.println("P_GPS_Moter");
    while(true){
    AzimuthDistance();
    if(azidata[1] < 5){
        break;
        }
    else{
        int PID_left = 0.65 * azidata[0] + 126;
        int PID_right = - 0.65 * azidata[0] + 126;
        MoterControl(PID_left, PID_right);
        delay(250);
        }
    } 
}
