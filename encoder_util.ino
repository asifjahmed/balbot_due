void updateEncoderSpeeds(float dt) {
    leftEncSpeed = (float)(leftEnc.read() - leftEncLastPosition) / dt;
    rightEncSpeed = (float)(rightEnc.read() - rightEncLastPosition) / dt;
  
    leftEncLastPosition = leftEnc.read();
    rightEncLastPosition = rightEnc.read();
}

