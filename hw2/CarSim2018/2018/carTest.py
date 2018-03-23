# modifications by Doug from Spring 2016 using single loop
import csv
import math
import os.path
import sys
import time
import numpy as np
import vrep
import vrepInterface


class tripwire(object):
  """Abstraction object for the tripwire, providing intuitive functions for the timer flag
  """
  def __init__(self, vrep_interface, name='Proximity_sensor'):
    self.vr = vrep_interface
    self.handle = self.vr.simxGetObjectHandle(name, 
                                                  vrep.simx_opmode_oneshot_wait)
    
    self.vr.simxReadProximitySensor(self.handle, vrep.simx_opmode_streaming)


  def get_distance(self):
    """Returns the distance that the sensor sees
    """
    

    # Specify -1 to retrieve the absolute position.
    return self.vr.simxReadProximitySensor(self.handle, vrep.simx_opmode_buffer)

class Car(object):
  """Abstraction object for the car, providing intuitive functions for getting
  car state and setting outputs.
  """
  def __init__(self, vrep_interface, name='AckermannSteeringCar'):
    self.vr = vrep_interface
    self.car_handle = self.vr.simxGetObjectHandle(name, 
                                                  vrep.simx_opmode_oneshot_wait)
    self.boom_handle = self.vr.simxGetObjectHandle('BoomSensor', 
                                                   vrep.simx_opmode_oneshot_wait)
    self.camera_handle = []
    self.camera_handle.append(self.vr.simxGetObjectHandle('LineCamera0', 
                                                          vrep.simx_opmode_oneshot_wait))
    self.camera_handle.append(self.vr.simxGetObjectHandle('LineCamera1', 
                                                          vrep.simx_opmode_oneshot_wait))

    # Open these variables in streaming mode for high efficiency access.
    self.vr.simxGetObjectPosition(self.car_handle, -1, vrep.simx_opmode_streaming)
    self.vr.simxGetObjectVelocity(self.car_handle, vrep.simx_opmode_streaming)
    
    self.vr.simxGetFloatSignal('yDist', vrep.simx_opmode_streaming)
    self.vr.simxGetFloatSignal('steerAngle', vrep.simx_opmode_streaming)
    
    for handle in self.camera_handle:
      self.vr.simxGetVisionSensorImage(handle, 1, vrep.simx_opmode_streaming)
    
    # Get the wheel diameter so we can set velocity in physical units.
    # Let's assume all the wheels are the same.
    # TODO: check this assumption?
    wheel_dia = self.vr.get_bounding_size('RearLeftWheel_respondable')[0]
    # multiply linear velocity by this to get wheel radians/s
    self.speed_factor = 2 / wheel_dia 

    # Default parameters
    self.steering_limit = 30  #i believe that it's all 45 degrees...
    self.steering_slew_rate = 60/0.16 # depends on servo 600 degrees in 160 ms
    #self.steering_slew_rate = 60/0.16 # depends on servo 60 degrees in 160 ms
    self.steering_time_ms = self.get_time()
    
    # state variables
    self.steering_state = 0
    self.old_lat_err = 0
    self.int_err = 0.0  # integral error
  # 
  # Some helper functions to get/set important car data/state
  #
  def get_position(self):
    """Returns the car's absolute position as a 3-tuple of (x, y, z), in meters.
    """
    # Specify -1 to retrieve the absolute position.
    return self.vr.simxGetObjectPosition(self.car_handle, -1, 
                                         vrep.simx_opmode_buffer)
  
  def get_velocity(self):
    """Returns the car's linear velocity as a 3-tuple of (x, y, z), in m/s.
    """
    return self.vr.simxGetObjectVelocity(self.car_handle, 
                                         vrep.simx_opmode_buffer)[0]

  def get_steering_angle(self):
    """Returns the car's steering angle in degrees.
    """
    return math.degrees(self.vr.simxGetFloatSignal('steerAngle', 
                                                   vrep.simx_opmode_buffer))

  def get_lateral_error(self):
    """Returns the lateral error (distance from sensor to line) in meters.
    """
    return self.vr.simxGetFloatSignal('yDist', vrep.simx_opmode_buffer)
  
  def get_line_camera_image(self, camera_index):
    """Returns the line sensor image as an array of pixels, where each pixel is
    between [0, 255], with 255 being brightest.
    Likely non-physical (graphical) units of linear perceived pixel intensity.
    """
    # Magical '1' argument specifies to return the data as greyscale.
    handle = self.camera_handle[camera_index]
    _, image = self.vr.simxGetVisionSensorImage(handle, 1,
                                                vrep.simx_opmode_buffer)
    for i, intensity in enumerate(image):
      if intensity < 0:	# undo two's complement
        image[i] = 256 + intensity
    return image

  def set_speed(self, speed, blocking=False):
    """Sets the car's target speed in m/s. Subject to acceleration limiting in 
    the simulator.
    """
    if blocking:
      op_mode = vrep.simx_opmode_oneshot_wait
    else:
      op_mode = vrep.simx_opmode_oneshot
    self.vr.simxSetFloatSignal('xSpeed', speed*self.speed_factor, op_mode)
  def get_time(self): #gets time in Ms since the epoch.
    return int(round(time.time() * 1000)) #time in milli seconds.

  def set_steering(self, angle_cmd, dt):
    """Sets the car's steering angle in degrees.
    Steering angle rate limiting happens here.
    Returns the actual commanded angle.
    """
    angle = self.steering_state # get present angle
    angle_err = angle_cmd - angle
    if (abs(angle_err) < dt*self.steering_slew_rate):
        angle = min(angle_cmd, self.steering_limit)
        angle = max(angle_cmd, -self.steering_limit)
        return(angle) # can reach angle in single time step
    
    if (angle_cmd > self.steering_state +1):    # add some dead band
        angle = self.steering_state + dt * self.steering_slew_rate
    if (angle_cmd < self.steering_state -1):
        angle = self.steering_state - dt * self.steering_slew_rate
    angle = min(angle, self.steering_limit)
    angle = max(angle, -self.steering_limit)
    self.steering_state = angle # update state
    self.vr.simxSetFloatSignal('steerAngle', angle*(math.pi/180.0), 
                               vrep.simx_opmode_oneshot)
#==============================================================================
#     print('steering angle:',angle)
#     if (angle > 30):
#         print 'servo beyond range'
#==============================================================================
    return angle

  def set_boom_sensor_offset(self, boom_length):
    """Sets the car's boom sensor's offset (approximate distance from front of
    car, in meters).
    This is provided so you don't have to learn how to mess with the V-REP
    scene to tune your boom sensor parameters.
    NOTE: this doesn't update the graphical boom stick.
    """
    self.vr.simxSetObjectPosition(self.boom_handle, vrep.sim_handle_parent, 
                                  (0, 0, -(boom_length-0.35)),
                                  vrep.simx_opmode_oneshot_wait)

  def set_line_camera_parameters(self, camera_index, height=0.2,
                                 orientation=30, fov=90):
    """Sets the car's line camera parameters.
    This is provided so you don't have to learn how to mess with the V-REP
    scene to tune your camera parameters.
    Args:
        height -- height of the camera, in meters.
        orientation -- downward angle of the camera, in degrees from horizontal.
        fov -- field of vision of the camera, in degrees.
    """
    handle = self.camera_handle[camera_index]
    self.vr.simxSetObjectPosition(handle, vrep.sim_handle_parent, 
                                  (height, 0, 0), 
                                  vrep.simx_opmode_oneshot_wait)
    self.vr.simxSetObjectOrientation(handle, vrep.sim_handle_parent, 
                                     (0, 
                                      -math.radians(orientation), 
                                      math.radians(90)), 
                                     vrep.simx_opmode_oneshot_wait)
    self.vr.simxSetObjectFloatParameter(handle, 1004, math.radians(fov), 
                                        vrep.simx_opmode_oneshot_wait)

  def set_steering_limit(self, steering_limit):
    """Sets the car's steering limit in degrees from center.
    Attempts to steer past this angle (on either side) get clipped.
    """
    self.steering_limit = steering_limit

class SimulationAssignment():
  """Everything you need to implement for the assignment is here.
  You may """
  def __init__(self, vr, car, wire):
    # You may initialize additional state variables here
    self.last_sim_time = vr.simxGetFloatSignal('simTime', 
                                               vrep.simx_opmode_oneshot_wait)
  
  def get_line_camera_error(self, image):
    """Returns the distance from the line, as seen by the line camera, in 
    pixels. The actual physical distance (in meters) can be derived with some
    trig given the camera parameters.
    """
    #
    # ASSIGNMENT: You should implement your line tracking algorithm here.
    # The default code implements a very simple, very not-robust line detector.
    #
    # NOTE: unlike the actual camera, get_line_camera_image() returns pixel
    # intensity data in the range of [0, 255] and aren't physically based (i.e.
    # intensity in display intensity units, no integration time).
    #
    INTENSITY_THRESHOLD = 192

    weighted_sum = 0
    element_sum = 0
    for i, intensity in enumerate(image):
      if intensity > INTENSITY_THRESHOLD:
        weighted_sum += i
        element_sum += 1
    if element_sum == 0:
      return None
    return weighted_sum / element_sum - 63
  
  def setup_car(self, vr, car):
    """Sets up the car's physical parameters.
    """
    #
    # ASSIGNMENT: You may want to tune these paraneters.
    #
    car.set_boom_sensor_offset(0.1)
    # In the scene, we provide two cameras.
    # Camera 0 is used by default in the control loop and is the "near" one.
    # Some teams have also used a "far" camera in the past (Camera 1).
    # You can use additional cameras, but will have to add then in the V-REP scene
    # and bind the handle in Car.__init__. The V-REP remote API doesn't provide a
    # way to instantiate additional vision sensors.
    car.set_line_camera_parameters(0, height=0.3, orientation=30, fov=60)
    car.set_line_camera_parameters(1, height=0.4, orientation=15, fov=60)
    # You should measure the steering sevo limit and set it here.
    # A more accurate approach would be to implement servo slew limiting.
    car.set_steering_limit(30)
  
  def control_loop(self, vr, car, wire, csvfile=None):
    """Control iteration. This is called on a regular basis.
    Args:
        vr -- VRepInterface object, which is an abstraction on top of the VREP
              Python API. See the class docstring near the top of this file.
        car -- Car object, providing abstractions for the car (like steering and
               velocity control).
        csvfile -- Optional csv.DictWriter for logging data. None to disable.
    """
    # Waiting mode is used here to 
    time = vr.simxGetFloatSignal('simTime', vrep.simx_opmode_oneshot_wait)
    dt = time - self.last_sim_time
    self.last_sim_time = time
    crossed = wire.get_distance()

    #
    # ASSIGNMENT: Tune / implement a better controller loop here.
    #
    
    # Here are two different sensors you can play with.
    # One provides an ideal shortest-distance-to-path (in meters), but isn't
    # path-following and may jump at crossings.
    # The other uses a more realistic line sensor (in pixels), which you can
    # plug your track detection algorithm into.
    lat_err = car.get_lateral_error()
    if (dt > 0.0):
        lat_vel = (lat_err - car.old_lat_err)/dt
    else:
            lat_vel = 0.0
    car.old_lat_err = lat_err
    
    #calculate integral error    
    car.int_err = car.int_err + dt*lat_err
    
    line0_err = self.get_line_camera_error(car.get_line_camera_image(0))
    line1_err = self.get_line_camera_error(car.get_line_camera_image(1))
    
    # Proportional gain in steering control (degrees) / lateral error (meters)
    kp = 500.0
    kd = 100.0 # deg per m/s
    ki = 0.0 # deg per m-s
    steer_angle = -kp * lat_err - kd*lat_vel - ki* car.int_err
    steer_angle = car.set_steering(steer_angle,dt)
 
    # Constant speed for now. You can tune this and/or implement advanced
    # controllers.
    #car.set_speed(ve)
    if (np.absolute(lat_err) > 0.15):
      car.set_speed(2.5) #slow down
    else:
      car.set_speed(3.0) #setpoint
    
    # Print out debugging info
    pos = car.get_position()
    vel = car.get_velocity()
    vel = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
    print('t=%6.3f (x=%5.2f, y=%5.2f, sp=%5.2f): lat_err=%5.2f, int_err=%5.2f, line0_err=%3i, steer_angle=%3.1f'
          % (time, pos[0], pos[1], vel, 
             lat_err, car.int_err, (line0_err or 0), steer_angle))
  
    if csvfile is not None:
      csvfile.writerow({'t': time, 'x': pos[0], 'y': pos[1], 'speed': vel,
                        'lat_err': lat_err, 
                        'line0_err': (line0_err or ""), 
                        'line1_err': (line1_err or ""), 
                        'steer_angle': steer_angle
                        })
    
    return crossed[0] #tells me if i've crossed the line or not!  Do NOT delete this line.
  
if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(description='ee192 Python V-REP Car controller.')
  parser.add_argument('--iterations', metavar='iteration', type=int, default=10000,
                      help='number of control iterations to run')
  parser.add_argument('--synchronous', metavar='s', type=bool, default=True,
                      help="""enable synchronous mode, forcing the simulator to 
                      operate in lockstep with the simulator - potentially 
                      increases accuracy / repeatability at the cost of 
                      performance""")
  parser.add_argument('--restart', metavar='r', type=bool, default=False,
                      help="""whether to restart the simulation if a simulation
                      is currently running""")
  parser.add_argument('--csvfile', metavar='c', default='car_data.csv',
                      help='csv filename to log to')
  parser.add_argument('--csvfile_overwrite', metavar='csvfile_overwrite', type=bool, default=True,
                      help='overwrite the specified csvfile without warning')
  parser.add_argument('--oneLoop', metavar='l',default = False,
                      help="""Run the car through only one round of the track, based on the tripwire.  Defaults to False.""")
  parser.add_argument('--constants', metavar = 'k', nargs = "+", default = None, type = float,
                       help="""A list (of airbitrary length) of contants.  Useful for PID tuning, for example""")
  parser.add_argument('--velocity', metavar = 'v', type = float, default = 1.5,
                     help="""Set the Velocity, in m/s.  Default, 3.0""")
  args = parser.parse_args()
  

  ve = args.velocity


  # Check that we won't overwrite an existing csvfile before mucking with the
  # simulator.
  if (args.csvfile is not None and os.path.exists(args.csvfile) 
      and not args.csvfile_overwrite):
    print("csvfile '%s' already exists: aborting." % args.csvfile)
    sys.exit()
  
  # Terminate any existing sessions, just in case.
  vrep.simxFinish(-1)
  
  # Stop the existing simulation if requested. Not using a separate
  # VRepInterface (and hence VREP API client id) seems to cause crashes.
  if args.restart:
    with vrepInterface.VRepInterface.open() as vr:
      vr.simxStopSimulation(vrep.simx_opmode_oneshot_wait)
  
  # Open a V-REP API connection and get the car.
  with vrepInterface.VRepInterface.open() as vr:
    if args.synchronous:
      vr.simxSynchronous(1)
      
    vr.simxStartSimulation(vrep.simx_opmode_oneshot_wait)
    
    car = Car(vr)
    wire = tripwire(vr)
    assignment = SimulationAssignment(vr, car, wire) #give it the tripwire, too
    assignment.setup_car(vr, car)
    
    csvfile = None
    if args.csvfile:
      # Dirty hack to get this (potentially) working in Python 2 and 3
      if sys.version_info.major < 3:
        outfile = open(args.csvfile, 'wb')
      else:
        outfile = open(args.csvfile, 'w', newline='')
        
      fieldnames = ['t', 'x', 'y', 'speed', 'lat_err', 'line0_err', 'line1_err', 
                    'steer_angle']
      fielddict = {}
      for fieldname in fieldnames:
        fielddict[fieldname] = fieldname
      csvfile = csv.DictWriter(outfile, fieldnames=fieldnames)  # change so don't over write orig file object!
      csvfile.writerow(fielddict)
    
    if not args.oneLoop:
      try:
        for i in range(0, args.iterations):
          assignment.control_loop(vr, car, wire, csvfile)
          
          # Advance to the next frame
          if args.synchronous:
            vr.simxSynchronousTrigger()
            
        print("Finished %i control iterations: pausing simulation." 
              % args.iterations)
        vr.simxPauseSimulation(vrep.simx_opmode_oneshot_wait)
        # need to have clean file close
        outfile.close()   # close file and writer

      except KeyboardInterrupt:
      # Allow a keyboard interrupt to break out of the loop while still shutting
      # down gracefully. 
        print("KeyboardInterrupt: pausing simulation.")
        vr.simxPauseSimulation(vrep.simx_opmode_oneshot_wait)
    else:
      try:
        keepGoing = True
        count = 0
        oldCrossed = 0;
        while keepGoing:
          crossed = assignment.control_loop(vr, car, wire, csvfile)
          if (oldCrossed == 0) and (crossed == 1): #transition!
            count = count + 1
          oldCrossed = crossed
          # Advance to the next frame
          if args.synchronous:
             vr.simxSynchronousTrigger()
          if count == 2: #two lowToHigh Transitions.  First one is crossing the tripwire the first time (get a run-up on the speed).  Second is to stop.
             keepGoing = False
            
        print("Finished one loop: ending simulation." )
        vr.simxStopSimulation(vrep.simx_opmode_oneshot_wait)

      except KeyboardInterrupt:
        # Allow a keyboard interrupt to break out of the loop while still shutting
        # down gracefully. 
        print("KeyboardInterrupt: pausing simulation.")
        vr.simxPauseSimulation(vrep.simx_opmode_oneshot_wait)


    