



void pid_ctrl(int cur_vel, int ref_vel, int kp, int ki, int kd, int last_error, int error_sum){
  Serial.println("PID Controller Included!");

  ///////////
  //Velocty calculations - we do it here fornow :)

  // Her skal vi hapse glenns msgQ fra ISR
  int fictivecount = 0; // dette skal være ISR Q, SÅ DEN ER FAKE!
  
  //time difference findes, bruges også i pid
  unsigned long time_diff = k_millis() - time_pre;
  time_pre = k_millis(); /// skal måske tages fra den ovenfor tror forskellen er miniskul
    


  int cur_rel = 

  if (DEBUG == 1){
  Serial.print(F("Current Velocety = ")); 
  Serial.println(cur_rel)


  //////////
  //actual PID :)
  //calculate error,  error sum, and error change
  int error = 0;
  error = ref_vel - cur_vel;


  error_sum += error * time_diff;


  int error_chng = (error - last_error)/time_diff;
  last_error = error;




  
  
  //calculate P, I, and D
  int pid = kp * error + ki * error_sum + kd * error_chng;

  

  if (DEBUG == 1){
  Serial.print(F("error = "));
  Serial.println(error);
  Serial.print(F("time difference = "));
  Serial.println(time_diff);
  Serial.print(F("error_sum = "));
  Serial.println(error_sum);
  Serial.print(F("error_chng = "));
  Serial.println(error_chng);
  Serial.print(F("PID = "));
  Serial.println(pid);  
  }

  

  return pid;
}