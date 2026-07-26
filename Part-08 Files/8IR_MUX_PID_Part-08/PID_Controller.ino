void PID_Controller(int base_speed, int p, int d) {

  while (1) {
a:
    read_black_line();

    if (sumOnSensor <= 2) line_position = sensorWight / sumOnSensor;
    error = center_position - line_position;

    switch (bitSensor) {
      //left side detection
      case 0b11111000: direction = "left"; break;
      case 0b11110000: direction = "left"; break;
      case 0b11111100: direction = "left"; break;
      //right side detection
      case 0b00011111: direction = "right"; break;
      case 0b00001111: direction = "right"; break;
      case 0b00111111: direction = "right"; break;
    }

    //turn execution when all the sensor on the white surface.
    if (bitSensor == 0) {
      error = 0;
      if (direction != "straight") {  //if the direction is either left or right
        //digitalWrite(led, HIGH);      //turning on led as indicator when left or right turn is detected.
        delay(delay_before_turn);

        if (direction == "right") {
          //right turn logic
          turnRight(turnSpeed, turnSpeed);
        } else {
          //left turn logic
          turnLeft(turnSpeed, turnSpeed);
        }
        //digitalWrite(led, LOW);  //after execution of turns the led will off
        //hard_stop(); //after executing turns the robot will stop. this is only for turn test
      }
    } else if (bitSensor == 255) {  //Stop Point, T, Cross Intersections
      digitalWrite(led, HIGH);      //led on when all black
      distance(5);                  //move 10cm forward.
      read_black_line();            //then read sensor value
      while (bitSensor == 255) {    //if sensor value still on black line. that means its stop point
        stop();                     //then robot stop
        read_black_line();          //update sensor if it's changing or not.
      }
      if (bitSensor == 0) {
        direction = "right";  //turn for T intersection.
      } else {
        if (bitSensor > 0 && bitSensor < 255) {
          direction = "straight";  //for Cross intersection
        }
      }
    } else if (bitSensor == 207 || bitSensor == 231 || bitSensor == 239 || bitSensor == 231 || bitSensor == 247 || bitSensor == 243) {
      inverseON = !inverseON;
      digitalWrite(led, inverseON);
      Bit_Sensor_Show();
      goto a;
    }

    //show sensor pattern
    //Bit_Sensor_Show();
    //digitalWrite(led, LOW);

    derivative = error - previous_error;
    int right_motor_correction = base_speed + (error * p + derivative * d);
    int left_motor_correction = base_speed - (error * p + derivative * d);
    previous_error = error;
    //Drive Motor According to PID Value
    motor(left_motor_correction, right_motor_correction);
  }
}
