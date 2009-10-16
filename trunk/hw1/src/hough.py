
  def hough(self, reading):
    curAngle = reading.angle_max  # I believe laser index 0 is the maximum angle, so we start there
    x = y = 0
    for dist in reading.ranges:  # cycle through the readings
      if dist >= self.maxRadius:  # if distance is too large, laser doesn't see anything
        continue
      x = dist*cos(curAngle)  # convert from polar to cartesian coords - this needs work, I don't think it's correct
      y = dist*sin(curAngle)
      rospy.loginfo("Reading found. Polar=(%0.2f,%0.2f degrees) --> Cartesian=(%0.2f,%0.2f)", dist, curAngle*r2d, x, y)
      for h in range(self.T):  # cycle through all bins of theta
        p = abs(x*cos(self.thetaValues[h]) + y*sin(self.thetaValues[h]))  # calculate the distance for the given theta
        k = int(round(p/self.radiusInc))  # find the bin for that distance
        if k == self.R:  # correct it if the bin was too far
          k -= 1
        if k == 0:  # this is the error case that's happening, k == 0 is getting a lot of hits
          rospy.loginfo("k = 0:  x=%0.2f, y=%0.2f, h=%0.2f, p=%0.2f", x, y, h, p)
        self.A[k][h] += 1  # increment the accumulator for the calculated distance, radius bin
      curAngle -= reading.angle_increment  # decrement the laser angle for the next reading
    peaks = []  # the list of local maxima in the parameter space
    numPeaks = 6  # arbitrarily chosen
    for distIndex in range(len(self.A)):  # cycle through the distances
      for angIndex in range(len(self.A[distIndex])):  # cycle through the angles
        if len(peaks) < numPeaks:  # if we have less than 6 peaks, just grab this one
          peaks.append([self.A[distIndex][angIndex], distIndex, angIndex])
          peaks.sort()
        elif self.A[distIndex][angIndex] > peaks[0]:  # otherwise, if this is greater than the current minimum, replace it
          peaks[0] = [self.A[distIndex, angIndex], distIndex, angIndex]  # [0] is minimum because we sort() every time
          peaks.sort()
    for peak in peaks:
      rospy.loginfo( "Count = %d  dist = %0.2f  angle = %0.2f", peak[0], self.radiusValues[peak[1]], self.thetaValues[peak[2]]*r2d)
